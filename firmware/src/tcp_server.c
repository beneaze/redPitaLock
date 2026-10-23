#include "tcp_server.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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

/* Send a buffer completely (send() may write only part of a large block). */
static int send_all(int fd, const char *buf, size_t len) {
    while (len > 0) {
        ssize_t n = send(fd, buf, len, MSG_NOSIGNAL);
        if (n <= 0)
            return -1;
        buf += n;
        len -= (size_t)n;
    }
    return 0;
}

/* Stream a finished step capture to the client that requested it. */
static int send_step_capture(int fd, int ch, channel_state_t *s) {
    int n = s->step_count;
    char hdr[160];
    snprintf(hdr, sizeof(hdr), "STEP %d %d %d %d %.5f %.5f %s\n",
             ch, n, s->step_pre_n, s->step_period_us,
             s->step_sp_before, s->step_delta_v,
             s->step_state == STEP_DONE ? "ok" : "aborted");
    if (send_all(fd, hdr, strlen(hdr)) < 0)
        return -1;
    static char sbuf[16384];
    size_t used = 0;
    for (int i = 0; i < n; i++) {
        char line[64];
        int w = snprintf(line, sizeof(line), "S %.1f %.5f %.5f\n",
                         s->step_t_us[i], s->step_in[i], s->step_out[i]);
        if (used + (size_t)w >= sizeof(sbuf)) {
            if (send_all(fd, sbuf, used) < 0)
                return -1;
            used = 0;
        }
        memcpy(sbuf + used, line, (size_t)w);
        used += (size_t)w;
    }
    if (used > 0 && send_all(fd, sbuf, used) < 0)
        return -1;
    s->step_state = STEP_IDLE;
    return 0;
}

static void send_params(int fd, int ch, const channel_state_t *s) {
    char buf[1536];
    snprintf(buf, sizeof(buf),
             "P %d enabled=%d setpoint=%.4f kp=%.4f ki=%.4f kd=%.4f "
             "error_sign=%.1f loop_rate=%d use_lut=%d "
             "in_scale=%.4f in_offset=%.4f out_scale=%.4f out_offset=%.4f "
             "integral_max=%.4f out_min=%.4f out_max=%.4f "
             "out_mode=%d manual_v=%.4f wave_freq=%.4f wave_amp=%.4f wave_offset=%.4f "
             "psd_avg=%d psd_interval=%d "
             "hold_mode=%d hold=%d hold_pin=%d hold_pol=%d hold_thresh=%.4f "
             "hold_hyst=%.4f hold_debounce=%d resume_delay=%d hold_park=%d "
             "hold_park_v=%.4f hold_count=%d hold_lookback=%d hold_avg=%d "
             "loop_actual=%.1f "
             "sp_source=%d sp_user=%.4f sp_ext_pct=%.1f sp_ext_min=%.4f sp_ext_max=%.4f "
             "sp_ext_ref=%.4f sp_ext_alpha=%.3f spcal_state=%d spcal_ramp_ms=%d "
             "spcal_avg_ms=%d spcal_zero=%.4f spcal_full=%.4f spcal_out_min=%.4f "
             "spcal_out=%.4f sp_ext_v=%.4f\n",
             ch, s->enabled, s->setpoint_v,
             s->pid.kp, s->pid.ki, s->pid.kd,
             s->error_sign, s->loop_period_us, s->use_lut,
             s->cal.input_scale, s->cal.input_offset,
             s->cal.output_scale, s->cal.output_offset,
             s->pid.integral_max, s->pid.out_min, s->pid.out_max,
             s->out_mode, s->manual_v, s->wave_freq_hz,
             s->wave_amplitude, s->wave_offset,
             s->psd_avg, s->psd_interval_ms,
             s->hold_mode, s->hold_manual, s->hold_pin, s->hold_polarity,
             s->hold_thresh_v, s->hold_hyst_v, s->hold_debounce,
             s->resume_delay_us, s->hold_park, s->hold_park_v,
             s->telem_hold_count, s->hold_lookback_ms, s->hold_avg_ms,
             s->telem_period_us,
             s->sp_source, s->sp_user_v, s->sp_ext_pct, s->sp_ext_min_v, s->sp_ext_max_v,
             s->sp_ext_ref_v, s->sp_ext_alpha, s->spcal_state, s->spcal_ramp_ms,
             s->spcal_avg_ms, s->spcal_zero_v, s->spcal_full_v, s->spcal_out_min_v,
             s->spcal_out_v, s->telem_sp_ext_v);
    send_line(fd, buf);
}

static int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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
        else if (strcmp(key, "setpoint")   == 0) {
            s->sp_user_v = val;
            if (s->sp_source != SP_SOURCE_EXTERNAL) s->setpoint_v = val;
        }
        else if (strcmp(key, "kp")         == 0)   s->pid.kp = val;
        else if (strcmp(key, "ki")         == 0) {
            /* Bumpless gain change: keep ki*integral (the I contribution to
             * the output) constant, so retuning does not kick the output.  */
            if (val > 0.0f && s->pid.ki > 0.0f)
                s->pid.integral *= s->pid.ki / val;
            s->pid.ki = val;
        }
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
                float center = s->telem_actual_output_v;
                float headroom = fminf(center - s->pid.out_min,
                                       s->pid.out_max - center);
                float amp = (s->autotune_relay_amp < headroom)
                          ?  s->autotune_relay_amp : headroom;
                if (amp < 0.01f) amp = 0.01f;
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
        else if (strcmp(key, "psd_avg")      == 0)  s->psd_avg = (int)val;
        else if (strcmp(key, "psd_interval") == 0)  s->psd_interval_ms = (int)val;
        /* --- Sample and hold --- */
        else if (strcmp(key, "hold")         == 0)  s->hold_manual = ((int)val != 0);
        else if (strcmp(key, "hold_mode")    == 0)  s->hold_mode = clampi((int)val, HOLD_MODE_OFF, HOLD_MODE_ANALOG);
        else if (strcmp(key, "hold_pin")     == 0)  s->hold_pin = clampi((int)val, 0, DIGITAL_IN_COUNT - 1);
        else if (strcmp(key, "hold_pol")     == 0)  s->hold_polarity = ((int)val != 0);
        else if (strcmp(key, "hold_thresh")  == 0)  s->hold_thresh_v = val;
        else if (strcmp(key, "hold_hyst")    == 0)  s->hold_hyst_v = (val < 0.0f) ? 0.0f : val;
        else if (strcmp(key, "hold_debounce") == 0) s->hold_debounce = clampi((int)val, 1, 100000);
        else if (strcmp(key, "resume_delay") == 0)  s->resume_delay_us = clampi((int)val, 0, 10000000);
        else if (strcmp(key, "hold_lookback") == 0) s->hold_lookback_ms = clampi((int)val, 0, 60000);
        else if (strcmp(key, "hold_avg")     == 0)  s->hold_avg_ms = clampi((int)val, 0, 60000);
        else if (strcmp(key, "hold_park")    == 0)  s->hold_park = ((int)val != 0);
        else if (strcmp(key, "hold_park_v")  == 0)  s->hold_park_v = val;
        else if (strcmp(key, "hold_count_reset") == 0) s->telem_hold_count = 0;
        /* --- External setpoint --- */
        else if (strcmp(key, "sp_source")    == 0) {
            int src = ((int)val != 0) ? SP_SOURCE_EXTERNAL : SP_SOURCE_INTERNAL;
            if (src != s->sp_source) {
                s->sp_source = src;
                if (src == SP_SOURCE_INTERNAL) s->setpoint_v = s->sp_user_v;
            }
        }
        else if (strcmp(key, "sp_ext_pct")   == 0) {
            if (val < 0.0f) val = 0.0f;
            if (val > 100.0f) val = 100.0f;
            s->sp_ext_pct = val;
            if (s->spcal_full_v > s->spcal_zero_v)
                s->sp_ext_max_v = s->spcal_zero_v
                                + (s->spcal_full_v - s->spcal_zero_v) * val / 100.0f;
        }
        else if (strcmp(key, "sp_ext_min")   == 0)  s->sp_ext_min_v = val;
        else if (strcmp(key, "sp_ext_max")   == 0)  s->sp_ext_max_v = val;
        else if (strcmp(key, "sp_ext_ref")   == 0)  s->sp_ext_ref_v = (val < 0.001f) ? 0.001f : val;
        else if (strcmp(key, "sp_ext_alpha") == 0)  s->sp_ext_alpha = (val <= 0.0f || val > 1.0f) ? 1.0f : val;
        else if (strcmp(key, "spcal_ramp_ms") == 0) s->spcal_ramp_ms = clampi((int)val, 1, 10000);
        else if (strcmp(key, "spcal_avg_ms") == 0)  s->spcal_avg_ms = clampi((int)val, 1, 10000);
        else if (strcmp(key, "spcal")        == 0) {
            int st = s->spcal_state;
            int busy = (st >= SPCAL_ARMED && st <= SPCAL_RAMP_BACK) || st == SPCAL_ABORT;
            if ((int)val == 1) {
                if (busy) {
                    send_line(fd, "ERR spcal busy\n");
                    return 0;
                }
                s->spcal_state = SPCAL_ARMED;   /* the PID thread takes it from here */
            } else if (busy && st != SPCAL_RAMP_BACK) {
                s->spcal_state = SPCAL_ABORT;
            }
        }
        /* --- Step-response capture --- */
        else if (strcmp(key, "step_ms")      == 0)  s->step_ms = clampi((int)val, 1, 60000);
        else if (strcmp(key, "step_pre_ms")  == 0)  s->step_pre_ms = clampi((int)val, 0, 1000);
        else if (strcmp(key, "step")         == 0) {
            if (s->step_state == STEP_ARMED || s->step_state == STEP_RUNNING) {
                send_line(fd, "ERR step busy\n");
                return 0;
            }
            if (!s->enabled) {
                send_line(fd, "ERR step needs the PID on\n");
                return 0;
            }
            if (s->spcal_state >= SPCAL_ARMED && s->spcal_state <= SPCAL_RAMP_BACK) {
                send_line(fd, "ERR step: setpoint calibration running\n");
                return 0;
            }
            s->step_delta_v = val;
            s->step_fd = fd;
            s->step_state = STEP_ARMED;   /* the PID thread takes it from here */
        }
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

    /* Incoming commands are accumulated in `rx` and split on newlines, so
     * several commands arriving in one TCP segment are all handled in the
     * same cycle. (A stdio fgets() here would buffer the extra lines and
     * select() would not see them until the client sent more data.)      */
    char rx[1024];
    size_t rx_len = 0;
    int telem_counter = 0;

    while (1) {
        /* Non-blocking check for incoming commands. */
        fd_set rfds;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 }; /* 10 ms */
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel > 0) {
            ssize_t n = recv(fd, rx + rx_len, sizeof(rx) - 1 - rx_len, 0);
            if (n <= 0)
                break; /* client disconnected or error */
            rx_len += (size_t)n;
            rx[rx_len] = '\0';

            /* Handle every complete line in the buffer. */
            char *start = rx;
            char *nl;
            while ((nl = memchr(start, '\n', rx_len - (size_t)(start - rx))) != NULL) {
                *nl = '\0';
                size_t len = (size_t)(nl - start);
                while (len > 0 && (start[len-1] == '\r' || start[len-1] == ' '))
                    start[--len] = '\0';
                if (len > 0) {
                    if (handle_command(start, fd, channels) < 0)
                        send_line(fd, "ERR unknown command\n");
                }
                start = nl + 1;
            }
            /* Keep any partial trailing line; drop it if it cannot fit. */
            size_t rest = rx_len - (size_t)(start - rx);
            if (rest >= sizeof(rx) - 1)
                rest = 0;
            memmove(rx, start, rest);
            rx_len = rest;
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
                /* Fields: ch time input target actual setpoint enabled
                 *         holding gate   (last two added for sample-and-hold) */
                snprintf(tbuf, sizeof(tbuf),
                         "D %d %.6f %.5f %.5f %.5f %.4f %d %d %d %d %.5f\n",
                         ch, s->telem_time_s,
                         s->telem_input_v, s->telem_output_v,
                         s->telem_actual_output_v,
                         s->setpoint_v, s->enabled,
                         s->telem_holding, s->telem_gate, s->telem_level,
                         s->telem_input_raw);
                if (send(fd, tbuf, strlen(tbuf), MSG_NOSIGNAL) < 0)
                    goto done;

                if ((s->step_state == STEP_DONE || s->step_state == STEP_ABORTED)
                    && s->step_fd == fd) {
                    if (send_step_capture(fd, ch, s) < 0)
                        goto done;
                }

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
                    char abuf[160];
                    snprintf(abuf, sizeof(abuf),
                             "A %d %.6f %.6f %.6f %.6f %.6f\n",
                             ch, s->autotune.Ku, s->autotune.Tu,
                             s->autotune.Kp, s->autotune.Ki,
                             s->autotune.Kd);
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
                    /* Send bins as space-separated floats on one line. */
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
    /* A capture requested by this client must not block future requests. */
    for (int ch = 0; ch < NUM_CHANNELS; ch++)
        if (channels[ch].step_fd == fd &&
            (channels[ch].step_state == STEP_DONE || channels[ch].step_state == STEP_ABORTED))
            channels[ch].step_state = STEP_IDLE;
    close(fd);
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
