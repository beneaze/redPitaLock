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

/* Ring buffer of recent controller outputs, per channel, so a hold can take
 * the output from before the gate arrived (hold_lookback_ms / hold_avg_ms). */
static float out_hist_pid[NUM_CHANNELS][HOLD_HIST_LEN];
static float out_hist_drive[NUM_CHANNELS][HOLD_HIST_LEN];

static void hist_push(int ch, int *head, int *count, float pid_out, float drive_v) {
    out_hist_pid[ch][*head]   = pid_out;
    out_hist_drive[ch][*head] = drive_v;
    *head = (*head + 1) % HOLD_HIST_LEN;
    if (*count < HOLD_HIST_LEN) (*count)++;
}

/* Average of the history entries whose age (0 = newest) lies in
 * [n_look, n_look + n_avg), clamped to what is available. */
static void hist_pick(int ch, int head, int count, int n_look, int n_avg,
                      float *pid_out, float *drive_v) {
    if (count <= 0)
        return;                         /* nothing recorded: keep caller's values */
    if (n_look > count - 1) n_look = count - 1;
    if (n_avg < 1) n_avg = 1;
    if (n_avg > count - n_look) n_avg = count - n_look;
    double sp = 0.0, sd = 0.0;
    for (int age = n_look; age < n_look + n_avg; age++) {
        int idx = (head - 1 - age + 2 * HOLD_HIST_LEN) % HOLD_HIST_LEN;
        sp += out_hist_pid[ch][idx];
        sd += out_hist_drive[ch][idx];
    }
    *pid_out = (float)(sp / n_avg);
    *drive_v = (float)(sd / n_avg);
}

static void *pid_thread(void *arg) {
    thread_arg_t *a  = (thread_arg_t *)arg;
    int               ch = a->ch_idx;
    channel_state_t  *s  = a->state;
    free(a);
    int hist_head = 0, hist_count = 0;

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
    /* --- Step-response capture state (private to this thread) --- */
    int    step_total   = 0;      /* samples to record in total (pre + post) */
    int    step_applied = 0;      /* setpoint step has been applied          */
    double step_t0      = 0.0;    /* time of the step (s)                    */
    /* --- External setpoint / calibration state (private to this thread) --- */
    float  ext_lpf  = 0.0f;       /* filtered control voltage                */
    int    ext_init = 0;
    int    cal_k = 0, cal_n_ramp = 1, cal_n_wait = 1, cal_n_avg = 1, cal_cnt = 0;
    double cal_sum = 0.0;
    float  cal_prev_drive = 0.0f; /* DAC drive before the calibration        */
    float  cal_prev_pid   = 0.0f; /* PID output (pre-LUT) before it          */
    float  cal_lo = 0.0f, cal_hi = 0.0f;   /* outputs measured at (out_min/out_max) */
    float  cal_from = 0.0f, cal_to = 0.0f; /* current ramp                    */
    float  cal_cur        = 0.0f; /* drive of the last calibration iteration */
    int    cal_failed     = 0;

    struct timespec next, t0;
    clock_gettime(CLOCK_MONOTONIC, &next);
    t0 = next;

    while (running) {
        /* Timestamp relative to thread start */
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        {
            double t_now = (double)(now_ts.tv_sec - t0.tv_sec)
                         + (double)(now_ts.tv_nsec - t0.tv_nsec) * 1e-9;
            /* Measured loop period: the requested period is a lower bound
             * that the CPU may not reach (librp reads dominate). */
            float meas_us = (float)((t_now - s->telem_time_s) * 1e6);
            if (s->telem_time_s > 0.0 && meas_us > 0.0f && meas_us < 1e6f)
                s->telem_period_us = (s->telem_period_us <= 0.0f)
                                   ? meas_us
                                   : 0.98f * s->telem_period_us + 0.02f * meas_us;
            s->telem_time_s = t_now;
        }

        float raw = analog_read(ch, &s->cal);
        s->telem_input_raw = raw;
        float dt  = (float)s->loop_period_us / 1e6f;

        /* ---------------- Sample-and-hold gate evaluation ---------------- */
        /* gate = 1 means "input is invalid, freeze". Evaluated every loop,
         * also while the PID is off, so telem_gate lets the user verify the
         * TTL wiring and polarity before enabling the loop.                 */
        int gate = s->hold_manual ? 1 : 0;
        int level = -1;                 /* raw level of the gate source       */
        int hold_mode = s->hold_mode;
        if (hold_mode == HOLD_MODE_DIO) {
            level = digital_in_read(s->hold_pin);
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
            level = thresh_below ? 0 : 1;
            if (thresh_below) gate = 1;
        } else if (hold_mode == HOLD_MODE_ANALOG) {
            /* TTL on the other fast input: level with hysteresis, then the
             * same polarity rule as the DIO mode (hold while LOW or HIGH). */
            float g  = analog_read_raw((ch + 1) % NUM_CHANNELS);
            float th = s->hold_thresh_v;
            float hy = s->hold_hyst_v;
            if (thresh_below) {
                if (g > th + hy) thresh_below = 0;
            } else {
                if (g < th) thresh_below = 1;
            }
            level = thresh_below ? 0 : 1;
            if (level == (s->hold_polarity ? 1 : 0))
                gate = 1;
        } else {
            thresh_below = 0;
        }
        s->telem_gate = gate;
        s->telem_level = level;

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
            hist_count = 0;
        }
        was_enabled = enabled;

        if (enabled) {
            if (sh_state == SH_RUN) {
                if (gate_active) {
                    sh_state = SH_HOLD;
                    s->telem_hold_count++;
                    /* Pick the value to hold: by default the previous
                     * iteration's output, optionally from further back
                     * and/or averaged (see config.h).                    */
                    {
                        int per = s->loop_period_us > 0 ? s->loop_period_us : 1;
                        int n_look = (int)((long)s->hold_lookback_ms * 1000L / per);
                        int n_avg  = (int)((long)s->hold_avg_ms * 1000L / per);
                        if (n_look > 0 || n_avg > 1)
                            hist_pick(ch, hist_head, hist_count, n_look, n_avg,
                                      &held_pid_out, &held_drive_v);
                    }
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

        /* ---------------- External-setpoint calibration ------------------ */
        /* Two-point measurement of the plant: ramp the output to out_min and
         * average the input ("zero"), ramp to out_max and average it again
         * ("full"), ramp back and resume the PID bumplessly. The external
         * setpoint then scales between sp_ext_min = zero and
         * sp_ext_max = zero + pct/100 * (full - zero). Runs with the PID on
         * or off and ignores the hold.                                       */
        int cal_override = 0;
        if (s->spcal_state == SPCAL_ARMED) {
            int per = (s->telem_period_us >= 1.0f) ? (int)(s->telem_period_us + 0.5f)
                    : (s->loop_period_us > 0 ? s->loop_period_us : 1);
            cal_n_ramp = (int)((long)s->spcal_ramp_ms * 1000L / per);
            cal_n_avg  = (int)((long)s->spcal_avg_ms * 1000L / per);
            if (cal_n_ramp < 1) cal_n_ramp = 1;
            if (cal_n_avg < 1) cal_n_avg = 1;
            cal_n_wait = cal_n_avg;
            cal_prev_drive = s->telem_actual_output_v;
            cal_prev_pid   = enabled ? held_pid_out : s->telem_actual_output_v;
            cal_lo = s->pid.out_min;
            cal_hi = s->pid.out_max;
            if (cal_lo < RP_OUTPUT_MIN) cal_lo = RP_OUTPUT_MIN;
            if (cal_lo > RP_OUTPUT_MAX) cal_lo = RP_OUTPUT_MAX;
            if (cal_hi < RP_OUTPUT_MIN) cal_hi = RP_OUTPUT_MIN;
            if (cal_hi > RP_OUTPUT_MAX) cal_hi = RP_OUTPUT_MAX;
            s->spcal_out_min_v = cal_lo;
            s->spcal_out_v = cal_hi;
            cal_from = cal_prev_drive;
            cal_to   = cal_lo;
            cal_k = 0; cal_cnt = 0; cal_sum = 0.0; cal_failed = 0;
            cal_cur = cal_prev_drive;
            s->spcal_state = SPCAL_RAMP_MIN;
        } else if (s->spcal_state == SPCAL_ABORT) {
            /* Abort: ramp back from wherever the output is now. */
            cal_from = cal_cur;
            cal_to   = cal_prev_drive;
            cal_k = 0;
            cal_failed = 1;
            s->spcal_state = SPCAL_RAMP_BACK;
        }
        if (s->spcal_state >= SPCAL_RAMP_MIN && s->spcal_state <= SPCAL_RAMP_BACK) {
            float drive_v = cal_to;
            int st = s->spcal_state;
            if (st == SPCAL_RAMP_MIN || st == SPCAL_RAMP_MAX || st == SPCAL_RAMP_BACK) {
                drive_v = cal_from + (cal_to - cal_from) * (float)cal_k / (float)cal_n_ramp;
                if (++cal_k >= cal_n_ramp) {
                    drive_v = cal_to;
                    cal_k = 0;
                    if (st == SPCAL_RAMP_BACK) {
                        if (enabled) {
                            /* Bumpless hand-back to the PID at the old output. */
                            in_lpf[ch] = raw;
                            in_lpf_init[ch] = 1;
                            pid_seed_output(&s->pid, s->setpoint_v, raw,
                                            s->error_sign, cal_prev_pid);
                        }
                        s->spcal_state = cal_failed ? SPCAL_FAILED : SPCAL_DONE;
                    } else {
                        s->spcal_state = st + 1;      /* -> SETTLE_MIN / SETTLE_MAX */
                    }
                }
            } else if (st == SPCAL_SETTLE_MIN || st == SPCAL_SETTLE_MAX) {
                if (++cal_k >= cal_n_wait) {
                    cal_k = 0; cal_cnt = 0; cal_sum = 0.0;
                    s->spcal_state = st + 1;          /* -> MEASURE_MIN / MEASURE_MAX */
                }
            } else { /* SPCAL_MEASURE_MIN / SPCAL_MEASURE_MAX */
                /* Average only while the hold gate is idle (beam on); the
                 * window extends until enough samples were seen, up to
                 * SPCAL_GATE_TIMEOUT x the averaging time. */
                if (!gate_active) {
                    cal_sum += raw;
                    cal_cnt++;
                }
                cal_k++;
                int timed_out = cal_k >= SPCAL_GATE_TIMEOUT * cal_n_avg;
                if (cal_cnt >= cal_n_avg || timed_out) {
                    if (cal_cnt < (cal_n_avg + 3) / 4) cal_failed = 1;  /* beam never on */
                    float mean = cal_cnt > 0 ? (float)(cal_sum / cal_cnt) : 0.0f;
                    cal_k = 0;
                    if (st == SPCAL_MEASURE_MIN) {
                        s->spcal_zero_v = mean;
                        cal_from = cal_lo;
                        cal_to   = cal_hi;
                        s->spcal_state = SPCAL_RAMP_MAX;
                    } else {
                        s->spcal_full_v = mean;
                        float zero = s->spcal_zero_v;
                        if (!cal_failed && mean - zero > 1e-3f) {
                            s->sp_ext_min_v = zero;
                            s->sp_ext_max_v = zero + (mean - zero) * s->sp_ext_pct / 100.0f;
                        } else {
                            cal_failed = 1;     /* no light / no response       */
                        }
                        cal_from = cal_hi;
                        cal_to   = cal_prev_drive;
                        s->spcal_state = SPCAL_RAMP_BACK;
                    }
                }
            }
            cal_cur = drive_v;
            float actual_v = analog_write(ch, drive_v, &s->cal);
            s->telem_output_v = drive_v;
            s->telem_actual_output_v = actual_v;
            cal_override = 1;
        }

        /* ---------------- Step-response capture: arm / apply / abort ---- */
        if (s->step_state == STEP_ARMED) {
            /* Size the capture with the measured period, not the requested one. */
            int per = (s->telem_period_us >= 1.0f) ? (int)(s->telem_period_us + 0.5f)
                    : (s->loop_period_us > 0 ? s->loop_period_us : 1);
            int pre = (int)((long)s->step_pre_ms * 1000L / per);
            int post = (int)((long)s->step_ms * 1000L / per);
            if (pre > STEP_MAX / 4) pre = STEP_MAX / 4;
            if (pre + post > STEP_MAX) post = STEP_MAX - pre;
            if (post < 2) post = 2;
            s->step_period_us = per;
            s->step_pre_n = pre;
            s->step_count = 0;
            s->step_sp_before = s->setpoint_v;
            step_total = pre + post;
            step_applied = 0;
            step_t0 = s->telem_time_s;
            s->step_state = STEP_RUNNING;
        }
        if (s->step_state == STEP_RUNNING) {
            if (!enabled || holding || cal_override) {
                if (step_applied) s->setpoint_v = s->step_sp_before;
                s->step_state = STEP_ABORTED;
            } else if (!step_applied && s->step_count >= s->step_pre_n) {
                /* Apply the step now, before this iteration's PID update, so
                 * sample index step_pre_n is the first one with the new
                 * setpoint. */
                s->setpoint_v = s->step_sp_before + s->step_delta_v;
                step_applied = 1;
                /* Re-reference the pre-trigger samples to the step instant
                 * so their times come out negative. */
                float shift = (float)((s->telem_time_s - step_t0) * 1e6);
                for (int k = 0; k < s->step_count && k < STEP_MAX; k++)
                    s->step_t_us[k] -= shift;
                step_t0 = s->telem_time_s;
            }
        }

        /* ---------------- External setpoint ------------------------------ */
        /* setpoint = min + (max - min) * clamp(ctrl / sp_ext_ref, 0, 1), with the
         * control voltage read from the other fast input and low-pass
         * filtered. A running step capture adds its delta on top.          */
        if (s->sp_source == SP_SOURCE_EXTERNAL) {
            float c = analog_read_raw((ch + 1) % NUM_CHANNELS);
            float a = s->sp_ext_alpha;
            if (!(a > 0.0f && a <= 1.0f)) a = 1.0f;
            if (!ext_init) { ext_lpf = c; ext_init = 1; }
            else           ext_lpf += a * (c - ext_lpf);
            s->telem_sp_ext_v = ext_lpf;
            float ref = s->sp_ext_ref_v;
            if (ref < 1e-3f) ref = 1e-3f;
            float frac = ext_lpf / ref;
            if (frac < 0.0f) frac = 0.0f;
            if (frac > 1.0f) frac = 1.0f;
            float sp = s->sp_ext_min_v + (s->sp_ext_max_v - s->sp_ext_min_v) * frac;
            if (s->step_state == STEP_RUNNING && step_applied)
                sp += s->step_delta_v;
            s->setpoint_v = sp;
        } else {
            ext_init = 0;
        }

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

        if (cal_override) {
            /* Output already written by the calibration block above. */
        } else if (enabled) {
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
                hist_push(ch, &hist_head, &hist_count, drive_v, drive_v);
            } else {
                float pid_out = pid_update(&s->pid, s->setpoint_v, input_v,
                                           dt, s->error_sign);
                drive_v = s->use_lut ? aom_linearize(pid_out) : pid_out;
                held_pid_out = pid_out;
                held_drive_v = drive_v;
                hist_push(ch, &hist_head, &hist_count, pid_out, drive_v);
            }

            wave_phase = 0.0f;
            float actual_v = analog_write(ch, drive_v, &s->cal);
            s->telem_output_v = drive_v;
            s->telem_actual_output_v = actual_v;

            /* ---------------- Step-response capture: record ------------- */
            if (s->step_state == STEP_RUNNING) {
                int i = s->step_count;
                if (i < STEP_MAX) {
                    s->step_t_us[i] = (float)((s->telem_time_s - step_t0) * 1e6);
                    s->step_in[i]   = raw;
                    s->step_out[i]  = drive_v;
                    s->step_count   = i + 1;
                }
                if (s->step_count >= step_total || i >= STEP_MAX - 1) {
                    s->setpoint_v = s->step_sp_before;
                    s->step_state = STEP_DONE;
                }
            }
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
    s->telem_level      = -1;
    s->hold_manual      = 0;
    s->hold_pin         = DEFAULT_HOLD_PIN(ch);
    s->hold_polarity    = DEFAULT_HOLD_POLARITY;
    s->hold_thresh_v    = DEFAULT_HOLD_THRESH_V;
    s->hold_hyst_v      = DEFAULT_HOLD_HYST_V;
    s->hold_debounce    = DEFAULT_HOLD_DEBOUNCE;
    s->resume_delay_us  = DEFAULT_RESUME_DELAY_US;
    s->hold_lookback_ms = DEFAULT_HOLD_LOOKBACK_MS;
    s->step_state       = STEP_IDLE;
    s->step_ms          = DEFAULT_STEP_MS;
    s->step_pre_ms      = DEFAULT_STEP_PRE_MS;
    s->step_fd          = -1;
    s->hold_avg_ms      = DEFAULT_HOLD_AVG_MS;
    s->hold_park        = DEFAULT_HOLD_PARK;
    s->hold_park_v      = DEFAULT_HOLD_PARK_V;

    s->sp_source        = SP_SOURCE_INTERNAL;
    s->sp_user_v        = DEFAULT_SETPOINT_V;
    s->sp_ext_pct       = DEFAULT_SP_EXT_PCT;
    s->sp_ext_min_v     = 0.0f;
    s->sp_ext_max_v     = 0.0f;
    s->sp_ext_ref_v     = DEFAULT_SP_EXT_REF_V;
    s->sp_ext_alpha     = DEFAULT_SP_EXT_ALPHA;
    s->spcal_state      = SPCAL_IDLE;
    s->spcal_ramp_ms    = DEFAULT_SPCAL_RAMP_MS;
    s->spcal_avg_ms     = DEFAULT_SPCAL_AVG_MS;

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
