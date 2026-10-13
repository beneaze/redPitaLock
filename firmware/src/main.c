#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>

#include "config.h"
#include "pid.h"
#include "aom_lut.h"
#include "analog_io.h"
#include "autotune.h"
#include "psd.h"
#include "tcp_server.h"

static volatile int running = 1;

static void sig_handler(int sig) {
    (void)sig;
    running = 0;
}

/* -------------------------------------------------- thread arguments */

typedef struct {
    int              ch_idx;
    channel_state_t *state;
} thread_arg_t;

/* -------------------------------------------------------- PID thread */

static float triangle_wave(float phase) {
    float t = fmodf(phase, 1.0f);
    if (t < 0.0f) t += 1.0f;
    return (t < 0.5f) ? (4.0f * t - 1.0f) : (3.0f - 4.0f * t);
}

static void *pid_thread(void *arg) {
    thread_arg_t *a  = (thread_arg_t *)arg;
    int               ch = a->ch_idx;
    channel_state_t  *s  = a->state;
    free(a);

    static float in_lpf[NUM_CHANNELS];
    static int   in_lpf_init[NUM_CHANNELS];
    float wave_phase = 0.0f;

    /* --- Sample-and-hold state (private to this thread) --- */
    enum { SH_RUN = 0, SH_HOLD = 1, SH_RESUME = 2 };
    int   sh_state        = SH_RUN;
    int   gate_count      = 0;      /* consecutive gated samples (debounce)   */
    int   thresh_below    = 0;      /* threshold-mode latch (hysteresis)      */
    long  resume_elapsed  = 0;      /* us since the gate released             */
    float held_pid_out    = 0.0f;   /* last PID output (pre-LUT) before hold  */
    float held_drive_v    = 0.0f;   /* last DAC drive (post-LUT) before hold  */
    int   was_enabled     = 0;

    struct timespec next, t0;
    clock_gettime(CLOCK_MONOTONIC, &next);
    t0 = next;

    while (running) {
        /* Timestamp relative to thread start */
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        s->telem_time_s = (double)(now_ts.tv_sec - t0.tv_sec)
                        + (double)(now_ts.tv_nsec - t0.tv_nsec) * 1e-9;

        float raw = analog_read(ch, &s->cal);
        float dt  = (float)s->loop_period_us / 1e6f;

        /* ---------------- Sample-and-hold gate evaluation ---------------- */
        /* gate = 1 means "input is invalid, freeze". Evaluated every loop,
         * also while the PID is off, so telem_gate lets the user verify the
         * TTL wiring and polarity before enabling the loop.                 */
        int gate = s->hold_manual ? 1 : 0;
        int hold_mode = s->hold_mode;
        if (hold_mode == HOLD_MODE_DIO) {
            int level = digital_in_read(s->hold_pin);
            if (level >= 0 && level == (s->hold_polarity ? 1 : 0))
                gate = 1;
            thresh_below = 0;
        } else if (hold_mode == HOLD_MODE_THRESHOLD) {
            float th = s->hold_thresh_v;
            float hy = s->hold_hyst_v;
            if (thresh_below) {
                if (raw > th + hy) thresh_below = 0;
            } else {
                if (raw < th) thresh_below = 1;
            }
            if (thresh_below) gate = 1;
        } else {
            thresh_below = 0;
        }
        s->telem_gate = gate;

        /* Debounce on entry (manual hold is immediate). */
        if (gate) {
            if (gate_count < 1000000) gate_count++;
        } else {
            gate_count = 0;
        }
        int debounce = s->hold_debounce;
        if (debounce < 1) debounce = 1;
        int gate_active = s->hold_manual || (gate && gate_count >= debounce);

        /* ---------------- Hold state machine (only while PID on) --------- */
        int enabled = s->enabled;
        if (enabled && !was_enabled) {
            /* Fresh enable: the "last output" is whatever is on the DAC now. */
            held_pid_out = s->telem_actual_output_v;
            held_drive_v = s->telem_actual_output_v;
            sh_state = SH_RUN;
        }
        was_enabled = enabled;

        if (enabled) {
            if (sh_state == SH_RUN) {
                if (gate_active) {
                    sh_state = SH_HOLD;
                    s->telem_hold_count++;
                    if (s->autotune.state == AUTOTUNE_RUNNING) {
                        /* A gap corrupts the relay experiment: abort it. */
                        s->autotune.state = AUTOTUNE_IDLE;
                        pid_reset(&s->pid);
                    }
                }
            } else if (sh_state == SH_HOLD) {
                if (!gate_active) {
                    sh_state = SH_RESUME;
                    resume_elapsed = 0;
                }
            } else { /* SH_RESUME: gate released, wait for the plant to settle */
                if (gate_active) {
                    sh_state = SH_HOLD;
                } else {
                    resume_elapsed += s->loop_period_us;
                    if (resume_elapsed >= (long)s->resume_delay_us) {
                        /* Bumpless resume: restart the filter on fresh data
                         * and seed the integrator so the first PID output
                         * equals the held output.                          */
                        in_lpf[ch] = raw;
                        in_lpf_init[ch] = 1;
                        pid_seed_output(&s->pid, s->setpoint_v, raw,
                                        s->error_sign, held_pid_out);
                        sh_state = SH_RUN;
                    }
                }
            }
        } else {
            sh_state = SH_RUN;
        }
        int holding = (sh_state != SH_RUN);
        s->telem_holding = holding;

        /* ---------------- Input low-pass filter (frozen while holding) --- */
        if (!in_lpf_init[ch]) {
            in_lpf[ch] = raw;
            in_lpf_init[ch] = 1;
        } else if (!holding) {
            in_lpf[ch] = INPUT_LPF_ALPHA * raw
                       + (1.0f - INPUT_LPF_ALPHA) * in_lpf[ch];
        }
        float input_v = in_lpf[ch];
        /* While holding, show the live (unfiltered) input so the gap is
         * visible on the plot; the controller itself does not see it.      */
        s->telem_input_v = holding ? raw : input_v;

        if (enabled) {
            float drive_v;

            if (holding) {
                drive_v = s->hold_park ? s->hold_park_v : held_drive_v;
            } else if (s->autotune.state == AUTOTUNE_RUNNING) {
                drive_v = autotune_step(&s->autotune, s->setpoint_v,
                                        input_v, dt);
                if (s->autotune.state == AUTOTUNE_DONE) {
                    s->pid.kp = s->autotune.Kp;
                    s->pid.ki = s->autotune.Ki;
                    s->pid.kd = 0.0f;
                    pid_reset(&s->pid);
                } else if (s->autotune.state == AUTOTUNE_FAILED) {
                    pid_reset(&s->pid);
                }
                held_pid_out = drive_v;
                held_drive_v = drive_v;
            } else {
                float pid_out = pid_update(&s->pid, s->setpoint_v, input_v,
                                           dt, s->error_sign);
                drive_v = s->use_lut ? aom_linearize(pid_out) : pid_out;
                held_pid_out = pid_out;
                held_drive_v = drive_v;
            }

            wave_phase = 0.0f;
            float actual_v = analog_write(ch, drive_v, &s->cal);
            s->telem_output_v = drive_v;
            s->telem_actual_output_v = actual_v;
        } else {
            if (s->autotune.state == AUTOTUNE_RUNNING)
                s->autotune.state = AUTOTUNE_IDLE;

            float drive_v;
            int mode = s->out_mode;

            if (mode == OUT_MODE_TRIANGLE) {
                drive_v = s->wave_offset
                        + s->wave_amplitude * triangle_wave(wave_phase);
                wave_phase += s->wave_freq_hz * dt;
                if (wave_phase >= 1.0f) wave_phase -= floorf(wave_phase);
            } else if (mode == OUT_MODE_SINE) {
                drive_v = s->wave_offset
                        + s->wave_amplitude * sinf(2.0f * (float)M_PI * wave_phase);
                wave_phase += s->wave_freq_hz * dt;
                if (wave_phase >= 1.0f) wave_phase -= floorf(wave_phase);
            } else {
                drive_v = s->manual_v;
                wave_phase = 0.0f;
            }

            float actual_v = analog_write_raw(ch, drive_v);
            s->telem_output_v = drive_v;
            s->telem_actual_output_v = actual_v;
        }

        /* Advance to the next period */
        long period_ns = (long)s->loop_period_us * 1000L;
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

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
            s->psd_ready = 1;
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

static void init_channel(channel_state_t *s, int ch) {
    memset(s, 0, sizeof(*s));
    s->enabled        = 0;
    s->setpoint_v     = DEFAULT_SETPOINT_V;
    s->error_sign     = DEFAULT_ERROR_SIGN;
    s->loop_period_us = DEFAULT_LOOP_PERIOD_US;
    s->use_lut        = DEFAULT_USE_LUT;

    s->cal.input_scale   = DEFAULT_INPUT_SCALE;
    s->cal.input_offset  = DEFAULT_INPUT_OFFSET;
    s->cal.output_scale  = DEFAULT_OUTPUT_SCALE;
    s->cal.output_offset = DEFAULT_OUTPUT_OFFSET;

    s->out_mode        = DEFAULT_OUT_MODE;
    s->manual_v        = DEFAULT_MANUAL_V;
    s->wave_freq_hz    = DEFAULT_WAVE_FREQ_HZ;
    s->wave_amplitude  = DEFAULT_WAVE_AMPLITUDE;
    s->wave_offset     = DEFAULT_WAVE_OFFSET;

    pid_init(&s->pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEGRAL_MAX);

    s->autotune.state       = AUTOTUNE_IDLE;
    s->autotune_relay_amp   = AUTOTUNE_RELAY_AMP;
    s->autotune_hysteresis  = AUTOTUNE_HYSTERESIS;

    s->hold_mode        = DEFAULT_HOLD_MODE;
    s->hold_manual      = 0;
    s->hold_pin         = DEFAULT_HOLD_PIN(ch);
    s->hold_polarity    = DEFAULT_HOLD_POLARITY;
    s->hold_thresh_v    = DEFAULT_HOLD_THRESH_V;
    s->hold_hyst_v      = DEFAULT_HOLD_HYST_V;
    s->hold_debounce    = DEFAULT_HOLD_DEBOUNCE;
    s->resume_delay_us  = DEFAULT_RESUME_DELAY_US;
    s->hold_park        = DEFAULT_HOLD_PARK;
    s->hold_park_v      = DEFAULT_HOLD_PARK_V;

    s->psd_avg         = PSD_DEFAULT_AVG;
    s->psd_interval_ms = PSD_DEFAULT_INTERVAL_MS;
}

int main(void) {
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    setvbuf(stdout, NULL, _IOLBF, 0);
    printf("redPitaLock v1.0 -- two-channel PID stabilizer\n");

    /* Initialise analog I/O (librp) */
    if (analog_io_init() != 0) {
        fprintf(stderr, "Failed to initialise Red Pitaya analog I/O\n");
        return 1;
    }

    /* Digital inputs for the sample-and-hold gate (non-fatal if it fails:
     * the DIO hold mode then simply never asserts). */
    if (digital_in_init() != 0)
        fprintf(stderr, "Warning: DIO init failed, TTL hold gate unavailable\n");

    /* Initialise channel state */
    channel_state_t channels[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++)
        init_channel(&channels[i], i);

    printf("Channels initialised (both disabled). Waiting for TCP commands on port %d.\n",
           TCP_PORT);

    /* Launch PID threads with real-time priority */
    pthread_t pid_tids[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        thread_arg_t *a = malloc(sizeof(*a));
        a->ch_idx = i;
        a->state  = &channels[i];

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        struct sched_param sp = { .sched_priority = 80 };
        pthread_attr_setschedparam(&attr, &sp);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

        if (pthread_create(&pid_tids[i], &attr, pid_thread, a) != 0) {
            /* Fall back to default scheduler if RT fails (non-root) */
            fprintf(stderr, "Warning: RT scheduling failed for ch%d, using default\n", i);
            pthread_attr_destroy(&attr);
            pthread_create(&pid_tids[i], NULL, pid_thread, a);
        } else {
            pthread_attr_destroy(&attr);
        }
    }

    /* Launch PSD threads (normal priority, one per channel) */
    pthread_t psd_tids[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        thread_arg_t *a = malloc(sizeof(*a));
        a->ch_idx = i;
        a->state  = &channels[i];
        pthread_create(&psd_tids[i], NULL, psd_thread, a);
    }

    /* Launch TCP server in its own thread */
    pthread_t tcp_tid;
    pthread_create(&tcp_tid, NULL, tcp_thread, channels);

    /* Wait for PID threads (they exit when `running` becomes 0) */
    for (int i = 0; i < NUM_CHANNELS; i++)
        pthread_join(pid_tids[i], NULL);

    for (int i = 0; i < NUM_CHANNELS; i++)
        pthread_join(psd_tids[i], NULL);

    printf("Shutting down...\n");
    analog_io_cleanup();

    return 0;
}
