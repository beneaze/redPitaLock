#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <math.h>
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

/* -------------------------------------------------------- PID thread */

typedef struct {
    int              ch_idx;
    channel_state_t *state;
} pid_thread_arg_t;

static float triangle_wave(float phase) {
    float t = fmodf(phase, 1.0f);
    if (t < 0.0f) t += 1.0f;
    return (t < 0.5f) ? (4.0f * t - 1.0f) : (3.0f - 4.0f * t);
}

static void *pid_thread(void *arg) {
    pid_thread_arg_t *a  = (pid_thread_arg_t *)arg;
    int               ch = a->ch_idx;
    channel_state_t  *s  = a->state;
    free(a);

    static float in_lpf[NUM_CHANNELS];
    static int   in_lpf_init[NUM_CHANNELS];
    float wave_phase = 0.0f;

    psd_state_t psd;
    psd_init(&psd);

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
        if (!in_lpf_init[ch]) {
            in_lpf[ch] = raw;
            in_lpf_init[ch] = 1;
        } else {
            in_lpf[ch] = INPUT_LPF_ALPHA * raw
                       + (1.0f - INPUT_LPF_ALPHA) * in_lpf[ch];
        }
        float input_v = in_lpf[ch];
        s->telem_input_v = input_v;

        float dt = (float)s->loop_period_us / 1e6f;

        if (s->enabled) {
            float drive_v;

            if (s->autotune.state == AUTOTUNE_RUNNING) {
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
            } else {
                float pid_out = pid_update(&s->pid, s->setpoint_v, input_v,
                                           dt, s->error_sign);
                drive_v = s->use_lut ? aom_linearize(pid_out) : pid_out;
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

        /* Feed PSD and copy result to shared state when ready */
        float fs = (dt > 0.0f) ? (1.0f / dt) : 1000.0f;
        psd_push_sample(&psd, input_v, fs);
        if (psd.ready) {
            memcpy((void *)s->psd_bins, psd.out, sizeof(psd.out));
            s->psd_fs    = psd.fs;
            s->psd_ready = 1;
            psd.ready    = 0;
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

    /* Initialise channel state */
    channel_state_t channels[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++)
        init_channel(&channels[i]);

    printf("Channels initialised (both disabled). Waiting for TCP commands on port %d.\n",
           TCP_PORT);

    /* Launch PID threads with real-time priority */
    pthread_t pid_tids[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        pid_thread_arg_t *a = malloc(sizeof(*a));
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

    /* Launch TCP server in its own thread */
    pthread_t tcp_tid;
    pthread_create(&tcp_tid, NULL, tcp_thread, channels);

    /* Wait for PID threads (they exit when `running` becomes 0) */
    for (int i = 0; i < NUM_CHANNELS; i++)
        pthread_join(pid_tids[i], NULL);

    printf("Shutting down...\n");
    analog_io_cleanup();

    return 0;
}
