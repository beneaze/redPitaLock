#include "tcp_server.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>

/* ------------------------------------------------------------- helpers */

static void send_line(int fd, const char *line) {
    size_t len = strlen(line);
    send(fd, line, len, MSG_NOSIGNAL);
}

static void send_params(int fd, int ch, const channel_state_t *s) {
    char buf[640];
    snprintf(buf, sizeof(buf),
             "P %d enabled=%d setpoint=%.4f kp=%.4f ki=%.4f kd=%.4f "
             "error_sign=%.1f loop_rate=%d use_lut=%d "
             "in_scale=%.4f in_offset=%.4f out_scale=%.4f out_offset=%.4f "
             "integral_max=%.4f out_min=%.4f out_max=%.4f "
             "out_mode=%d manual_v=%.4f wave_freq=%.4f wave_amp=%.4f wave_offset=%.4f\n",
             ch, s->enabled, s->setpoint_v,
             s->pid.kp, s->pid.ki, s->pid.kd,
             s->error_sign, s->loop_period_us, s->use_lut,
             s->cal.input_scale, s->cal.input_offset,
             s->cal.output_scale, s->cal.output_offset,
             s->pid.integral_max, s->pid.out_min, s->pid.out_max,
             s->out_mode, s->manual_v, s->wave_freq_hz,
             s->wave_amplitude, s->wave_offset);
    send_line(fd, buf);
}

/* Parse one command line. Returns 0 on success, -1 on parse error. */
static int handle_command(const char *line, int fd, channel_state_t *channels) {
    char cmd[16];
    int ch;
    char key[32];
    float val;

    if (sscanf(line, "%15s", cmd) != 1)
        return -1;

    if (strcmp(cmd, "SET") == 0) {
        char val_str[32];
        if (sscanf(line, "SET %d %31s %31s", &ch, key, val_str) != 3)
            return -1;
        if (ch < 0 || ch >= NUM_CHANNELS)
            return -1;
        val = (float)atof(val_str);
        channel_state_t *s = &channels[ch];

        if      (strcmp(key, "enabled")    == 0) { s->enabled = (int)val; if (!s->enabled) pid_reset(&s->pid); }
        else if (strcmp(key, "setpoint")   == 0)   s->setpoint_v = val;
        else if (strcmp(key, "kp")         == 0)   s->pid.kp = val;
        else if (strcmp(key, "ki")         == 0)   s->pid.ki = val;
        else if (strcmp(key, "kd")         == 0)   s->pid.kd = val;
        else if (strcmp(key, "error_sign") == 0)   s->error_sign = val;
        else if (strcmp(key, "loop_rate")  == 0)   s->loop_period_us = (int)val;
        else if (strcmp(key, "use_lut")    == 0)   s->use_lut = (int)val;
        else if (strcmp(key, "in_scale")   == 0)   s->cal.input_scale = val;
        else if (strcmp(key, "in_offset")  == 0)   s->cal.input_offset = val;
        else if (strcmp(key, "out_scale")  == 0)   s->cal.output_scale = val;
        else if (strcmp(key, "out_offset") == 0)   s->cal.output_offset = val;
        else if (strcmp(key, "integral_max") == 0)  s->pid.integral_max = val;
        else if (strcmp(key, "out_min")    == 0)   s->pid.out_min = val;
        else if (strcmp(key, "out_max")    == 0)   s->pid.out_max = val;
        else if (strcmp(key, "reset")      == 0)   pid_reset(&s->pid);
        else if (strcmp(key, "out_mode")   == 0)   s->out_mode = (int)val;
        else if (strcmp(key, "manual_v")   == 0)   s->manual_v = val;
        else if (strcmp(key, "wave_freq")  == 0)   s->wave_freq_hz = val;
        else if (strcmp(key, "wave_amp")   == 0)   s->wave_amplitude = val;
        else if (strcmp(key, "wave_offset") == 0)  s->wave_offset = val;
        else if (strcmp(key, "autotune")  == 0) {
            if ((int)val == 1 && s->enabled && s->autotune.state != AUTOTUNE_RUNNING) {
                float center = (s->pid.out_min + s->pid.out_max) * 0.5f;
                float half   = (s->pid.out_max - s->pid.out_min) * 0.5f;
                float amp    = (s->autotune_relay_amp < half)
                             ?  s->autotune_relay_amp : half;
                autotune_init(&s->autotune, amp, center,
                              s->autotune_hysteresis, s->error_sign,
                              AUTOTUNE_MIN_CYCLES, AUTOTUNE_SETTLE_CYCLES,
                              AUTOTUNE_TIMEOUT_S);
            } else if ((int)val == 0) {
                s->autotune.state = AUTOTUNE_IDLE;
            }
        }
        else if (strcmp(key, "autotune_amp") == 0)  s->autotune_relay_amp = val;
        else if (strcmp(key, "autotune_hyst") == 0) s->autotune_hysteresis = val;
        else return -1;

        send_line(fd, "OK\n");
        return 0;
    }

    if (strcmp(cmd, "GET") == 0) {
        if (sscanf(line, "GET %d %31s", &ch, key) != 2)
            return -1;
        if (ch < 0 || ch >= NUM_CHANNELS)
            return -1;
        if (strcmp(key, "params") == 0) {
            send_params(fd, ch, &channels[ch]);
            return 0;
        }
        return -1;
    }

    return -1;
}

/* ------------------------------------------------- per-client handler */

typedef struct {
    int              fd;
    channel_state_t *channels;
} client_ctx_t;

static void *client_thread(void *arg) {
    client_ctx_t *ctx = (client_ctx_t *)arg;
    int fd = ctx->fd;
    channel_state_t *channels = ctx->channels;
    free(ctx);

    int opt = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    FILE *fp = fdopen(fd, "r");
    if (!fp) { close(fd); return NULL; }

    /* Telemetry counter: we send telemetry every ~10 ms in the background
     * while also checking for incoming commands (non-blocking readline). */
    char line_buf[256];
    int telem_counter = 0;

    while (1) {
        /* Non-blocking check for incoming commands. */
        fd_set rfds;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 }; /* 10 ms */
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel > 0) {
            if (fgets(line_buf, sizeof(line_buf), fp) == NULL)
                break; /* client disconnected */

            /* Strip trailing newline */
            size_t len = strlen(line_buf);
            while (len > 0 && (line_buf[len-1] == '\n' || line_buf[len-1] == '\r'))
                line_buf[--len] = '\0';

            if (len > 0) {
                if (handle_command(line_buf, fd, channels) < 0)
                    send_line(fd, "ERR unknown command\n");
            }
        } else if (sel < 0) {
            break; /* error */
        }

        /* Send telemetry for all channels */
        telem_counter++;
        if (telem_counter >= 1) { /* every select timeout (~10 ms = ~100 Hz) */
            telem_counter = 0;
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                channel_state_t *s = &channels[ch];
                char tbuf[160];
                snprintf(tbuf, sizeof(tbuf),
                         "D %d %.6f %.5f %.5f %.5f %.4f %d\n",
                         ch, s->telem_time_s,
                         s->telem_input_v, s->telem_output_v,
                         s->telem_actual_output_v,
                         s->setpoint_v, s->enabled);
                if (send(fd, tbuf, strlen(tbuf), MSG_NOSIGNAL) < 0)
                    goto done;

                if (s->autotune.state == AUTOTUNE_RUNNING) {
                    char atbuf[96];
                    snprintf(atbuf, sizeof(atbuf),
                             "AT %d %d %d %.1f\n",
                             ch, s->autotune.half_cycle_count,
                             s->autotune.n_measured,
                             s->autotune.elapsed);
                    if (send(fd, atbuf, strlen(atbuf), MSG_NOSIGNAL) < 0)
                        goto done;
                } else if (s->autotune.state == AUTOTUNE_DONE) {
                    char abuf[128];
                    snprintf(abuf, sizeof(abuf),
                             "A %d %.6f %.6f %.6f %.6f\n",
                             ch, s->autotune.Ku, s->autotune.Tu,
                             s->autotune.Kp, s->autotune.Ki);
                    if (send(fd, abuf, strlen(abuf), MSG_NOSIGNAL) < 0)
                        goto done;
                    s->autotune.state = AUTOTUNE_IDLE;
                } else if (s->autotune.state == AUTOTUNE_FAILED) {
                    char abuf[64];
                    snprintf(abuf, sizeof(abuf), "AF %d\n", ch);
                    if (send(fd, abuf, strlen(abuf), MSG_NOSIGNAL) < 0)
                        goto done;
                    s->autotune.state = AUTOTUNE_IDLE;
                }

                /* Send PSD when a new frame is available */
                if (s->psd_ready) {
                    char hdr[64];
                    snprintf(hdr, sizeof(hdr), "PSD %d %d %.1f\n",
                             ch, PSD_BINS, s->psd_fs);
                    if (send(fd, hdr, strlen(hdr), MSG_NOSIGNAL) < 0)
                        goto done;
                    /* Send bins as space-separated floats on one line.
                     * Max line ~12 chars/bin * 1025 bins ≈ 12 KB. */
                    char *pbuf = malloc(PSD_BINS * 14 + 2);
                    if (pbuf) {
                        int off = 0;
                        for (int k = 0; k < PSD_BINS; k++) {
                            off += snprintf(pbuf + off, 14, "%.6g ",
                                            (double)s->psd_bins[k]);
                        }
                        pbuf[off++] = '\n';
                        pbuf[off]   = '\0';
                        if (send(fd, pbuf, (size_t)off, MSG_NOSIGNAL) < 0) {
                            free(pbuf);
                            goto done;
                        }
                        free(pbuf);
                    }
                    s->psd_ready = 0;
                }
            }
        }
    }

done:
    fclose(fp); /* also closes fd */
    return NULL;
}

/* -------------------------------------------------------- server loop */

void tcp_server_run(channel_state_t *channels) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return; }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TCP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(srv);
        return;
    }

    if (listen(srv, TCP_MAX_CLIENTS) < 0) {
        perror("listen");
        close(srv);
        return;
    }

    printf("TCP server listening on port %d\n", TCP_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t clen = sizeof(client_addr);
        int cfd = accept(srv, (struct sockaddr *)&client_addr, &clen);
        if (cfd < 0) { perror("accept"); continue; }

        printf("Client connected (fd %d)\n", cfd);

        client_ctx_t *ctx = malloc(sizeof(*ctx));
        ctx->fd       = cfd;
        ctx->channels = channels;

        pthread_t tid;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        pthread_create(&tid, &attr, client_thread, ctx);
        pthread_attr_destroy(&attr);
    }
}
