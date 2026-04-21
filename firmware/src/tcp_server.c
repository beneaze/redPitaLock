#include "tcp_server.h"
#include "config.h"
#include "fpga_pi.h"

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

/* ------------------------------------------------------- shutdown plumbing
 *
 * tcp_server_stop() has to be safe to call from a signal handler, which
 * means it can only do async-signal-safe operations: write to a volatile
 * sig_atomic_t flag and call shutdown(2).  Everything else (locking the
 * client table, joining client threads) is deferred to tcp_server_run()
 * itself, which drains and joins all live clients before returning.        */

#include <signal.h>

static volatile sig_atomic_t g_stopping    = 0;
static volatile int          g_listen_fd   = -1;

static pthread_mutex_t g_clients_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       g_client_tids[TCP_MAX_CLIENTS];
static int             g_client_fds [TCP_MAX_CLIENTS];
static int             g_n_clients   = 0;

/* Returns 0 on success, -1 if the table is full or the server is shutting
 * down (in which case the caller must close the fd).                       */
static int register_client(pthread_t tid, int fd) {
    pthread_mutex_lock(&g_clients_mu);
    if (g_stopping || g_n_clients >= TCP_MAX_CLIENTS) {
        pthread_mutex_unlock(&g_clients_mu);
        return -1;
    }
    g_client_tids[g_n_clients] = tid;
    g_client_fds [g_n_clients] = fd;
    g_n_clients++;
    pthread_mutex_unlock(&g_clients_mu);
    return 0;
}

static void unregister_client_self(void) {
    pthread_t self = pthread_self();
    pthread_mutex_lock(&g_clients_mu);
    for (int i = 0; i < g_n_clients; i++) {
        if (pthread_equal(g_client_tids[i], self)) {
            g_client_tids[i] = g_client_tids[g_n_clients - 1];
            g_client_fds [i] = g_client_fds [g_n_clients - 1];
            g_n_clients--;
            break;
        }
    }
    pthread_mutex_unlock(&g_clients_mu);
}

/* ------------------------------------------------------------- helpers */

static void send_line(int fd, const char *line) {
    size_t len = strlen(line);
    send(fd, line, len, MSG_NOSIGNAL);
}

static void send_params(int fd, int ch, const channel_state_t *s) {
    char buf[512];
    snprintf(buf, sizeof(buf),
             "P %d enabled=%d setpoint=%.4f kp=%.4f ki=%.4f "
             "error_sign=%.1f "
             "out_mode=%d manual_v=%.4f wave_freq=%.4f wave_amp=%.4f wave_offset=%.4f "
             "autotune_amp=%.4f autotune_hyst=%.4f "
             "psd_avg=%d psd_interval=%d hw_loop_hz=%d\n",
             ch, s->enabled, s->setpoint_v,
             s->kp, s->ki,
             s->error_sign,
             s->out_mode, s->manual_v, s->wave_freq_hz,
             s->wave_amplitude, s->wave_offset,
             s->autotune_relay_amp, s->autotune_hysteresis,
             s->psd_avg, s->psd_interval_ms,
             HARDWARE_LOOP_HZ);
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

        if      (strcmp(key, "enabled")    == 0) {
            s->enabled = (int)val;
            if (!s->enabled) {
                /* Supervisor will also disable the FPGA on its next tick,
                 * but pulse the integrator now so a quick re-enable starts
                 * from a known-clean state.                                  */
                fpga_pi_reset_integrator(ch);
            }
        }
        else if (strcmp(key, "setpoint")   == 0)   s->setpoint_v = val;
        else if (strcmp(key, "kp")         == 0)   s->kp = val;
        else if (strcmp(key, "ki")         == 0)   s->ki = val;
        else if (strcmp(key, "error_sign") == 0)   s->error_sign = val;
        else if (strcmp(key, "reset")      == 0)   fpga_pi_reset_integrator(ch);
        else if (strcmp(key, "out_mode")   == 0)   s->out_mode = (int)val;
        else if (strcmp(key, "manual_v")   == 0)   s->manual_v = val;
        else if (strcmp(key, "wave_freq")  == 0)   s->wave_freq_hz = val;
        else if (strcmp(key, "wave_amp")   == 0)   s->wave_amplitude = val;
        else if (strcmp(key, "wave_offset") == 0)  s->wave_offset = val;
        else if (strcmp(key, "autotune")  == 0) {
            if ((int)val == 1
                && s->autotune.state != AUTOTUNE_RUNNING
                && s->autotune.state != AUTOTUNE_BIASING) {
                /* Refuse rather than silently no-op so the operator can see
                 * (in /tmp/stabilizer_rp.log and via the ERR reply) why the
                 * relay never started swinging.                              */
                if (!s->enabled) {
                    fprintf(stderr,
                            "[autotune] refusing ch%d: PI is OFF -- "
                            "enable PI first so the supervisor can drive "
                            "the relay\n", ch);
                    send_line(fd, "ERR autotune requires PI enabled\n");
                    return 0;
                }
                /* Hand off to the supervisor: BIASING bisects the DAC drive
                 * across the full +/- 1 V envelope to find a center where
                 * input ~= setpoint, then transitions to RUNNING via
                 * autotune_init().                                           */
                autotune_request_bias(&s->autotune,
                                      s->autotune_relay_amp,
                                      s->autotune_hysteresis,
                                      s->error_sign,
                                      AUTOTUNE_MIN_CYCLES,
                                      AUTOTUNE_SETTLE_CYCLES,
                                      AUTOTUNE_TIMEOUT_S);
            } else if ((int)val == 0) {
                s->autotune.state = AUTOTUNE_IDLE;
            }
        }
        else if (strcmp(key, "autotune_amp") == 0)  s->autotune_relay_amp = val;
        else if (strcmp(key, "autotune_hyst") == 0) s->autotune_hysteresis = val;
        else if (strcmp(key, "psd_avg")      == 0)  s->psd_avg = (int)val;
        else if (strcmp(key, "psd_interval") == 0)  s->psd_interval_ms = (int)val;
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
    if (!fp) {
        close(fd);
        unregister_client_self();
        return NULL;
    }

    /* Per-client cursor on the PSD frame counter so each connected client
     * gets every new spectrum.                                              */
    int last_psd_seq[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) last_psd_seq[i] = 0;

    char line_buf[256];

    while (!g_stopping) {
        /* Wait up to 10 ms for incoming command bytes.  Telemetry is sent
         * once per loop iteration regardless of socket activity.            */
        fd_set rfds;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 }; /* 10 ms */
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel > 0) {
            if (fgets(line_buf, sizeof(line_buf), fp) == NULL)
                break; /* client disconnected */

            size_t len = strlen(line_buf);
            while (len > 0 && (line_buf[len-1] == '\n' || line_buf[len-1] == '\r'))
                line_buf[--len] = '\0';

            if (len > 0) {
                if (handle_command(line_buf, fd, channels) < 0)
                    send_line(fd, "ERR unknown command\n");
            }
        } else if (sel < 0) {
            break; /* error or shutdown */
        }

        /* Send telemetry for all channels */
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            channel_state_t *s = &channels[ch];
            char tbuf[160];
            /* %g serialises NaN as "nan", which the GUI parses as float
             * NaN and treats as "no valid output sample".                   */
            snprintf(tbuf, sizeof(tbuf),
                     "D %d %.6f %.5f %g %.4f %d\n",
                     ch, s->telem_time_s,
                     s->telem_input_v,
                     (double)s->telem_output_v,
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
                char abuf[160];
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

            /* Send PSD once per new frame, per client, so multiple GUIs
             * connected at once each see every spectrum.                    */
            int seq = s->psd_seq;
            if (seq != last_psd_seq[ch]) {
                last_psd_seq[ch] = seq;

                char hdr[64];
                snprintf(hdr, sizeof(hdr), "PSD %d %d %.1f\n",
                         ch, PSD_BINS, s->psd_fs);
                if (send(fd, hdr, strlen(hdr), MSG_NOSIGNAL) < 0)
                    goto done;

                /* Pre-size the buffer once per send rather than malloc/snprintf
                 * 8193 times.  ~14 chars per bin + newline + slack.          */
                char *pbuf = malloc((size_t)PSD_BINS * 14 + 4);
                if (pbuf) {
                    int off = 0;
                    int cap = PSD_BINS * 14 + 4;
                    for (int k = 0; k < PSD_BINS; k++) {
                        off += snprintf(pbuf + off, cap - off,
                                        "%.6g ", (double)s->psd_bins[k]);
                    }
                    pbuf[off++] = '\n';
                    pbuf[off]   = '\0';
                    if (send(fd, pbuf, (size_t)off, MSG_NOSIGNAL) < 0) {
                        free(pbuf);
                        goto done;
                    }
                    free(pbuf);
                }
            }
        }
    }

done:
    fclose(fp); /* also closes fd */
    unregister_client_self();
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

    g_listen_fd = srv;
    g_stopping  = 0;
    printf("TCP server listening on port %d\n", TCP_PORT);

    while (!g_stopping) {
        struct sockaddr_in client_addr;
        socklen_t clen = sizeof(client_addr);
        int cfd = accept(srv, (struct sockaddr *)&client_addr, &clen);
        if (cfd < 0) {
            if (g_stopping) break;
            if (errno == EINTR) continue;
            perror("accept");
            continue;
        }

        /* Re-check after the (possibly long) accept() blocked: if a signal
         * arrived during the call we don't want to spin up a brand-new
         * client just to immediately tear it down.                         */
        if (g_stopping) {
            close(cfd);
            break;
        }

        client_ctx_t *ctx = malloc(sizeof(*ctx));
        if (!ctx) { close(cfd); continue; }
        ctx->fd       = cfd;
        ctx->channels = channels;

        pthread_t tid;
        if (pthread_create(&tid, NULL, client_thread, ctx) != 0) {
            close(cfd);
            free(ctx);
            continue;
        }

        if (register_client(tid, cfd) < 0) {
            /* Either the table is full or shutdown started in the gap
             * between our pre-check and now.  Kick the socket and JOIN
             * synchronously rather than detach: a detached thread could
             * outlive tcp_server_run's drain and race the FPGA / librp
             * teardown that the caller does after we return.  The client
             * thread will see the shutdown fd and exit within a few ms.   */
            send_line(cfd, "ERR server busy\n");
            shutdown(cfd, SHUT_RDWR);
            pthread_join(tid, NULL);
        } else {
            printf("Client connected (fd %d)\n", cfd);
        }
    }

    /* Drain: kick every still-registered client and join their threads
     * before we return, so the caller can safely tear down shared
     * resources (FPGA mmap, librp) afterwards without racing them.        */
    pthread_mutex_lock(&g_clients_mu);
    int n = g_n_clients;
    pthread_t tids[TCP_MAX_CLIENTS];
    for (int i = 0; i < n; i++) {
        tids[i] = g_client_tids[i];
        if (g_client_fds[i] >= 0)
            shutdown(g_client_fds[i], SHUT_RDWR);
    }
    pthread_mutex_unlock(&g_clients_mu);

    for (int i = 0; i < n; i++)
        pthread_join(tids[i], NULL);

    g_listen_fd = -1;
    close(srv);
}

/* Async-signal-safe: only writes to sig_atomic_t and calls shutdown(2),
 * both of which POSIX explicitly lists as safe inside a signal handler.
 * The actual client-table manipulation and pthread_join() happen inside
 * tcp_server_run(), which the main thread joins after this returns.       */
void tcp_server_stop(void) {
    g_stopping = 1;
    int fd = g_listen_fd;
    if (fd >= 0)
        shutdown(fd, SHUT_RDWR);
}
