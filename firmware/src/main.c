#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

#include "config.h"
#include "fpga_pi.h"
#include "analog_io.h"
#include "autotune.h"
#include "psd.h"
#include "tcp_server.h"

static volatile sig_atomic_t running = 1;

static void sig_handler(int sig) {
    (void)sig;
    running = 0;
    /* Wake the TCP server out of accept().  tcp_server_stop is documented
     * as async-signal-safe: it only writes a sig_atomic_t flag and calls
     * shutdown(2), both of which POSIX permits inside a signal handler.   */
    tcp_server_stop();
}

/* -------------------------------------------------- thread arguments */

typedef struct {
    int              ch_idx;
    channel_state_t *state;
} thread_arg_t;

/* -------------------------------------------------- supervisor thread
 *
 * Runs at SUPERVISOR_PERIOD_US cadence (default 1 kHz).  Responsibilities:
 *
 *   1. Push setpoint / Kp / Ki / error_sign into the FPGA PI registers
 *      every tick when enabled.  The MMIO cost is negligible on Zynq and
 *      removes a whole "did anything change?" cache layer.
 *   2. Manage PI enable / disable transitions: zero the FPGA gains when
 *      disabling, pulse the integrator-reset bit when (re-)engaging.
 *   3. Sample the input at supervisor rate purely for telemetry / GUI
 *      plotting.  The actual control loop runs at 125 MHz in hardware.
 *   4. Drive the relay output during autotune, and write the resulting
 *      gains back into the channel state when it completes.
 *   5. Reconfigure the FPGA signal generator (manual DC / triangle / sine)
 *      when the PI is OFF.  We cache the last configuration so we don't
 *      re-arm the ASG every tick (each librp call is ~10 us of MMIO).
 */

static void *supervisor_thread(void *arg) {
    thread_arg_t *a  = (thread_arg_t *)arg;
    int               ch = a->ch_idx;
    channel_state_t  *s  = a->state;
    free(a);

    /* Edge-detect for engage / disengage and autotune transitions. */
    int   was_enabled    = -1;       /* -1 forces first-pass init             */
    int   was_autotune   =  0;

    /* ASG configuration cache (only used while PI is OFF).  Sentinel = NaN
     * / -1 means "force push next time".  rp_GenWaveform / rp_GenAmp /
     * rp_GenOffset are ~10 us each, so re-pushing every supervisor tick
     * (1000x/s) would add up; pushing only on change is essentially free. */
    int   last_out_mode  = -1;
    float last_manual_v  = NAN;
    float last_wave_freq = NAN;
    float last_wave_amp  = NAN;
    float last_wave_off  = NAN;

    /* Autotune drive-voltage cache.  The relay flips a few times per
     * second; without this cache we'd burn ~3000 librp ops/sec writing
     * the same DC setpoint over and over.                                  */
    float last_drive_v   = NAN;

    struct timespec next, t0;
    clock_gettime(CLOCK_MONOTONIC, &next);
    t0 = next;

    while (running) {
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        s->telem_time_s = (double)(now_ts.tv_sec - t0.tv_sec)
                        + (double)(now_ts.tv_nsec - t0.tv_nsec) * 1e-9;

        float input_v = analog_read_raw(ch);
        s->telem_input_v = input_v;

        if (s->enabled) {
            /* ---------------- AUTOTUNE branch ---------------- *
             * Covers both the BIASING (operating-point bisection) and
             * RUNNING (relay) phases.  Both need the FPGA loop disabled so
             * the supervisor owns the DAC.                                  */
            if (s->autotune.state == AUTOTUNE_BIASING ||
                s->autotune.state == AUTOTUNE_RUNNING) {
                if (!was_autotune) {
                    fpga_pi_disable(ch);
                    last_drive_v = NAN;
                    was_autotune = 1;
                }

                float drive_v;
                if (s->autotune.state == AUTOTUNE_BIASING) {
                    int settle_ticks =
                        (AUTOTUNE_BIAS_SETTLE_MS * 1000) / SUPERVISOR_PERIOD_US;
                    drive_v = autotune_bias_step(&s->autotune, s->setpoint_v,
                                                 input_v, settle_ticks);
                } else {
                    const float dt = (float)SUPERVISOR_PERIOD_US / 1e6f;
                    drive_v = autotune_step(&s->autotune, s->setpoint_v,
                                            input_v, dt);
                }

                if (drive_v != last_drive_v) {
                    s->telem_output_v = analog_set_dc(ch, drive_v);
                    last_drive_v = drive_v;
                } else {
                    s->telem_output_v = drive_v;
                }
                /* Force an ASG re-push when we leave autotune. */
                last_out_mode = -1;

                if (s->autotune.state == AUTOTUNE_DONE) {
                    s->kp = s->autotune.Kp;
                    s->ki = s->autotune.Ki;
                    printf("[autotune] applied ch%d: kp=%.4f ki=%.4f "
                           "(will land in FPGA next tick)\n",
                           ch, (double)s->autotune.Kp, (double)s->autotune.Ki);
                    /* Park the manual output before we re-engage the FPGA. */
                    analog_set_dc(ch, 0.0f);
                    last_drive_v = 0.0f;
                    fpga_pi_reset_integrator(ch);
                    was_autotune = 0;
                } else if (s->autotune.state == AUTOTUNE_FAILED) {
                    analog_set_dc(ch, 0.0f);
                    last_drive_v = 0.0f;
                    was_autotune = 0;
                }
            }
            /* ---------------- Normal closed-loop branch ---------------- */
            else {
                if (was_enabled != 1 || was_autotune) {
                    /* Engage: park the ASG offset at zero (DC mode) so the
                     * user's last manual / waveform output isn't summed
                     * onto the PI output (the v0.94 bitstream sums ASG +
                     * PID-block at the DAC), then clear stale integral state. */
                    analog_set_dc(ch, 0.0f);
                    last_out_mode = -1;
                    fpga_pi_reset_integrator(ch);
                    was_autotune = 0;
                }

                /* Push every tick.  At ~4 register writes per channel per
                 * ms, the MMIO cost is negligible on Zynq and we no longer
                 * need a "did anything change?" cache layer.                */
                fpga_pi_set(ch, s->setpoint_v, s->kp, s->ki, s->error_sign);

                /* Telemetry: while the FPGA owns the DAC at 125 MHz, the
                 * instantaneous output isn't cheaply readable from
                 * software, and reporting anything else (e.g. the
                 * setpoint) just confuses the GUI's "output" trace.  Send
                 * NaN; the GUI plots NaN as a gap thanks to
                 * connect="finite".  Use the PSD pane or an external scope
                 * to observe the high-rate output.                           */
                s->telem_output_v = NAN;
            }

            was_enabled = 1;
        } else {
            /* ---------------- PI OFF branch ---------------- */
            if (was_enabled != 0) {
                fpga_pi_disable(ch);
                last_out_mode = -1;
                was_enabled   = 0;
                was_autotune  = 0;
            }

            if (s->autotune.state == AUTOTUNE_RUNNING ||
                s->autotune.state == AUTOTUNE_BIASING)
                s->autotune.state = AUTOTUNE_IDLE;

            int   mode  = s->out_mode;
            float manv  = s->manual_v;
            float wfreq = s->wave_freq_hz;
            float wamp  = s->wave_amplitude;
            float woff  = s->wave_offset;

            if (mode != last_out_mode ||
                manv  != last_manual_v ||
                wfreq != last_wave_freq ||
                wamp  != last_wave_amp ||
                woff  != last_wave_off) {

                if (mode == OUT_MODE_DC) {
                    analog_set_dc(ch, manv);
                } else {
                    analog_set_waveform(ch, mode, wfreq, wamp, woff);
                }

                last_out_mode  = mode;
                last_manual_v  = manv;
                last_wave_freq = wfreq;
                last_wave_amp  = wamp;
                last_wave_off  = woff;
            }

            /* Telemetry "output" only makes sense in DC mode.  For sine /
             * triangle the waveform varies far faster than 1 kHz, so we
             * report the DC offset (the only thing about the output that
             * a 1 kHz observer can see).  analog_set_dc / analog_set_waveform
             * already hardware-clamp to +/- 1 V.                            */
            s->telem_output_v = (mode == OUT_MODE_DC) ? manv : woff;
        }

        /* Advance to the next period */
        const long period_ns = (long)SUPERVISOR_PERIOD_US * 1000L;
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    /* On shutdown, leave the FPGA quiet. */
    fpga_pi_disable(ch);
    analog_set_dc(ch, 0.0f);
    return NULL;
}

/* ------------------------------------------------------- PSD thread */

static void *psd_thread(void *arg) {
    thread_arg_t *a  = (thread_arg_t *)arg;
    int               ch = a->ch_idx;
    channel_state_t  *s  = a->state;
    free(a);

    psd_state_t psd;
    if (psd_init(&psd) != 0) {
        fprintf(stderr, "PSD ch%d: init failed\n", ch);
        return NULL;
    }

    float *buf = malloc(PSD_N * sizeof(float));
    if (!buf) {
        fprintf(stderr, "PSD ch%d: malloc failed\n", ch);
        psd_free(&psd);
        return NULL;
    }

    while (running) {
        int interval_ms = s->psd_interval_ms;
        if (interval_ms < 10) interval_ms = 10;
        usleep((useconds_t)interval_ms * 1000u);

        int avg_target = s->psd_avg;
        if (avg_target < 1) avg_target = 1;

        for (int seg = 0; seg < avg_target && running; seg++) {
            analog_read_bulk(ch, buf, PSD_N);
            psd_process_buffer(&psd, buf, PSD_FS, avg_target);

            if (seg < avg_target - 1)
                usleep(200);
        }

        if (psd.ready) {
            memcpy((void *)s->psd_bins, psd.out, sizeof(psd.out));
            s->psd_fs    = psd.fs;
            /* Atomic-ish bump; clients sample this and remember the last
             * value they sent, so concurrent readers each get every frame. */
            s->psd_seq   = s->psd_seq + 1;
            psd.ready    = 0;
        }
    }

    free(buf);
    psd_free(&psd);
    return NULL;
}

/* ------------------------------------------------------- TCP thread */

static void *tcp_thread(void *arg) {
    channel_state_t *channels = (channel_state_t *)arg;
    tcp_server_run(channels);
    return NULL;
}

/* ------------------------------------------------------------ main */

static void init_channel(channel_state_t *s) {
    memset(s, 0, sizeof(*s));
    s->enabled        = 0;
    s->setpoint_v     = DEFAULT_SETPOINT_V;
    s->error_sign     = DEFAULT_ERROR_SIGN;

    s->kp             = DEFAULT_KP;
    s->ki             = DEFAULT_KI;

    s->out_mode        = DEFAULT_OUT_MODE;
    s->manual_v        = DEFAULT_MANUAL_V;
    s->wave_freq_hz    = DEFAULT_WAVE_FREQ_HZ;
    s->wave_amplitude  = DEFAULT_WAVE_AMPLITUDE;
    s->wave_offset     = DEFAULT_WAVE_OFFSET;

    s->autotune.state       = AUTOTUNE_IDLE;
    s->autotune_relay_amp   = AUTOTUNE_RELAY_AMP;
    s->autotune_hysteresis  = AUTOTUNE_HYSTERESIS;

    s->psd_avg         = PSD_DEFAULT_AVG;
    s->psd_interval_ms = PSD_DEFAULT_INTERVAL_MS;
    s->psd_seq         = 0;

    s->telem_output_v  = NAN;
}

int main(void) {
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    setvbuf(stdout, NULL, _IOLBF, 0);
    printf("redPitaLock v2.0 -- two-channel FPGA PI stabilizer (125 MHz HW loop)\n");

    /* Initialise analog I/O (librp) -- still needed for telemetry reads,
     * PSD bulk reads, and the manual / waveform / autotune output paths.   */
    if (analog_io_init() != 0) {
        fprintf(stderr, "Failed to initialise Red Pitaya analog I/O\n");
        return 1;
    }

    /* Open the FPGA PI register block (the stock RP red_pitaya_pid_block). */
    if (fpga_pi_open() != 0) {
        fprintf(stderr, "Failed to mmap FPGA PI block at 0x%08lx.\n"
                        "Check that the v0.94 bitstream is loaded and the\n"
                        "daemon is running as root.\n",
                        FPGA_PI_BASE);
        analog_io_cleanup();
        return 1;
    }
    printf("FPGA PI mmaped at 0x%08lx (hardware loop %d Hz)\n",
           FPGA_PI_BASE, HARDWARE_LOOP_HZ);

    channel_state_t channels[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++)
        init_channel(&channels[i]);

    printf("Channels initialised (both disabled). Waiting for TCP commands on port %d.\n",
           TCP_PORT);

    /* Launch supervisor threads.  No SCHED_FIFO: the actual control loop
     * runs in the FPGA, so the supervisor's wake-up jitter does not affect
     * loop performance.                                                     */
    pthread_t sup_tids[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        thread_arg_t *a = malloc(sizeof(*a));
        if (!a) {
            fprintf(stderr, "FATAL: out of memory creating supervisor ch%d\n", i);
            return 1;
        }
        a->ch_idx = i;
        a->state  = &channels[i];

        int rc = pthread_create(&sup_tids[i], NULL, supervisor_thread, a);
        if (rc != 0) {
            fprintf(stderr, "FATAL: pthread_create supervisor ch%d failed: %s\n",
                    i, strerror(rc));
            free(a);
            return 1;
        }
    }

    /* Launch PSD threads (normal priority, one per channel) */
    pthread_t psd_tids[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        thread_arg_t *a = malloc(sizeof(*a));
        if (!a) {
            fprintf(stderr, "FATAL: out of memory creating PSD ch%d\n", i);
            return 1;
        }
        a->ch_idx = i;
        a->state  = &channels[i];
        int rc = pthread_create(&psd_tids[i], NULL, psd_thread, a);
        if (rc != 0) {
            fprintf(stderr, "FATAL: pthread_create PSD ch%d failed: %s\n",
                    i, strerror(rc));
            free(a);
            return 1;
        }
    }

    /* Launch TCP server in its own thread */
    pthread_t tcp_tid;
    int rc = pthread_create(&tcp_tid, NULL, tcp_thread, channels);
    if (rc != 0) {
        fprintf(stderr, "FATAL: pthread_create TCP server failed: %s\n",
                strerror(rc));
        return 1;
    }

    /* Wait for SIGINT/SIGTERM. */
    for (int i = 0; i < NUM_CHANNELS; i++)
        pthread_join(sup_tids[i], NULL);

    for (int i = 0; i < NUM_CHANNELS; i++)
        pthread_join(psd_tids[i], NULL);

    /* sig_handler may have already called this; calling twice is safe. */
    tcp_server_stop();
    pthread_join(tcp_tid, NULL);

    printf("Shutting down...\n");
    fpga_pi_close();
    analog_io_cleanup();

    return 0;
}
