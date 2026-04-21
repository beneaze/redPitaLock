#include "autotune.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void autotune_request_bias(autotune_state_t *at,
                           float relay_amp,
                           float hysteresis, float error_sign,
                           int target_cycles, int settle_cycles,
                           float timeout_s) {
    /* Stash everything the relay phase will need; the supervisor will call
     * autotune_init() with the discovered center once the bisection ends.
     * The bisection always sweeps the full +/- 1 V DAC envelope.            */
    at->relay_amp     = relay_amp;
    at->hysteresis    = hysteresis;
    at->error_sign    = error_sign;
    at->target_cycles = target_cycles;
    at->settle_cycles = settle_cycles;
    at->timeout_s     = timeout_s;

    at->bias_lo         = AUTOTUNE_BIAS_LO;
    at->bias_hi         = AUTOTUNE_BIAS_HI;
    at->bias_step       = 0;
    at->bias_drive      = 0.5f * (AUTOTUNE_BIAS_LO + AUTOTUNE_BIAS_HI);
    at->bias_wait_ticks = 0;

    at->elapsed         = 0.0f;

    __sync_synchronize();
    at->state = AUTOTUNE_BIASING;

    printf("[autotune] biasing: bisecting in [%.3f, %.3f] V (esign=%.0f)\n",
           (double)AUTOTUNE_BIAS_LO, (double)AUTOTUNE_BIAS_HI,
           (double)error_sign);
}

float autotune_bias_step(autotune_state_t *at, float setpoint, float input,
                         int settle_ticks_per_step) {
    if (at->state != AUTOTUNE_BIASING)
        return at->bias_drive;

    /* Wait for the plant to settle at the current drive before sampling.   */
    if (at->bias_wait_ticks > 0) {
        at->bias_wait_ticks--;
        return at->bias_drive;
    }

    /* The plant has settled -- evaluate this step.                         */
    float err = at->error_sign * (input - setpoint);
    printf("[autotune] bias step k=%d drive=%.4f input=%.4f err=%+.4f\n",
           at->bias_step, at->bias_drive, input, err);

    if (fabsf(input - setpoint) < AUTOTUNE_BIAS_TOL_V
        || at->bias_step >= AUTOTUNE_BIAS_STEPS - 1) {
        printf("[autotune] bias DONE: center=%.4f after %d step(s)\n",
               at->bias_drive, at->bias_step + 1);
        /* Hand off to the relay: autotune_init flips state to RUNNING.     */
        autotune_init(at, at->relay_amp, at->bias_drive,
                      at->hysteresis, at->error_sign,
                      at->target_cycles, at->settle_cycles,
                      at->timeout_s);
        return at->bias_drive;
    }

    /* Tighten the bracket: positive polarity-corrected error means the
     * drive over-shot, so contract the upper bound; negative means we need
     * more drive.                                                          */
    if (err > 0.0f) at->bias_hi = at->bias_drive;
    else            at->bias_lo = at->bias_drive;

    at->bias_step++;
    at->bias_drive      = 0.5f * (at->bias_lo + at->bias_hi);
    at->bias_wait_ticks = settle_ticks_per_step;
    return at->bias_drive;
}

void autotune_init(autotune_state_t *at, float relay_amp, float relay_center,
                   float hysteresis, float error_sign,
                   int target_cycles, int settle_cycles,
                   float timeout_s) {
    at->relay_amp      = relay_amp;
    at->relay_center   = relay_center;
    at->hysteresis     = hysteresis;
    at->error_sign     = error_sign;
    at->target_cycles  = target_cycles;
    at->settle_cycles  = settle_cycles;
    at->timeout_s      = timeout_s;

    at->relay_high       = 1;
    at->half_cycle_count = 0;
    at->peak_hi          = -1e9f;
    at->peak_lo          =  1e9f;
    at->time_in_cycle    = 0.0f;
    at->elapsed          = 0.0f;
    at->n_measured       = 0;

    at->Ku = 0.0f;
    at->Tu = 0.0f;
    at->Kp = 0.0f;
    at->Ki = 0.0f;

    /* Memory barrier: ensure all fields above are visible before the
     * supervisor thread sees state == RUNNING.                               */
    __sync_synchronize();
    at->state = AUTOTUNE_RUNNING;

    printf("[autotune] started: center=%.3f amp=%.3f hyst=%.4f esign=%.0f timeout=%.1fs\n",
           relay_center, relay_amp, hysteresis, error_sign, timeout_s);
}

float autotune_step(autotune_state_t *at, float setpoint, float measurement,
                    float dt) {
    if (at->state != AUTOTUNE_RUNNING)
        return at->relay_center;

    at->elapsed += dt;

    if (at->elapsed > at->timeout_s) {
        printf("[autotune] TIMEOUT after %.1fs — %d crossings, %d measured cycles\n",
               at->elapsed, at->half_cycle_count, at->n_measured);
        at->state = AUTOTUNE_FAILED;
        return at->relay_center;
    }

    /* error_sign-aware error: positive means "measurement is above setpoint
     * in the direction the relay should react to."  For error_sign = -1
     * (inverted plant), we flip so that the crossing logic still works
     * the same way.                                                          */
    float error = at->error_sign * (measurement - setpoint);
    at->time_in_cycle += dt;

    /* Track global max / min over the current cycle.  Earlier the update was
     * gated by relay_high -- but for any plant with phase lag the actual
     * measurement peak occurs DURING the opposite relay half (overshoot
     * after the relay flip).  Gating threw those samples away and collapsed
     * the measured amplitude toward 2*hysteresis, blowing up Ku.            */
    if (measurement > at->peak_hi)
        at->peak_hi = measurement;
    if (measurement < at->peak_lo)
        at->peak_lo = measurement;

    /* Detect zero crossing with hysteresis */
    int crossed = 0;
    if (at->relay_high && error > at->hysteresis) {
        at->relay_high = 0;
        crossed = 1;
    } else if (!at->relay_high && error < -at->hysteresis) {
        at->relay_high = 1;
        crossed = 1;
    }

    if (crossed) {
        at->half_cycle_count++;

        /* Every two half-cycles = one full cycle */
        if ((at->half_cycle_count % 2) == 0 && at->half_cycle_count >= 2) {
            int full_cycle = at->half_cycle_count / 2;
            int past_settle = full_cycle - at->settle_cycles;

            if (past_settle > 0 && at->n_measured < AUTOTUNE_MAX_CYCLES
                && at->n_measured < at->target_cycles) {
                float amplitude = at->peak_hi - at->peak_lo;
                at->periods[at->n_measured]    = at->time_in_cycle;
                at->amplitudes[at->n_measured] = amplitude;
                printf("[autotune] cycle %d: T=%.4fs a=%.5fV\n",
                       at->n_measured, at->time_in_cycle, amplitude);
                at->n_measured++;
            }

            /* Reset for next full cycle */
            at->time_in_cycle = 0.0f;
            at->peak_hi = -1e9f;
            at->peak_lo =  1e9f;
        }

        /* Check if we have enough cycles */
        if (at->n_measured >= at->target_cycles) {
            float sum_T = 0.0f, sum_a = 0.0f;
            for (int i = 0; i < at->n_measured; i++) {
                sum_T += at->periods[i];
                sum_a += at->amplitudes[i];
            }
            at->Tu = sum_T / (float)at->n_measured;
            /* Stored amplitudes are peak-to-peak; Astrom-Hagglund's `a` is
             * the single-sided sinusoidal amplitude, hence the /2.          */
            float a_half = 0.5f * (sum_a / (float)at->n_measured);

            /* Ku = 4d / (pi * a)  where d = relay half-amplitude            */
            if (a_half > 1e-9f)
                at->Ku = (4.0f * at->relay_amp) / ((float)M_PI * a_half);
            else
                at->Ku = 0.0f;

            /* Tyreus-Luyben PI rules (conservative, good for fast digital
             * loops).  Kd is intentionally not produced -- the FPGA Kd
             * register can't span anything useful for this plant.          */
            at->Kp = at->Ku / 3.2f;
            at->Ki = (at->Tu > 1e-9f) ? (at->Ku / (7.04f * at->Tu)) : 0.0f;

            printf("[autotune] DONE: Ku=%.4f Tu=%.4fs -> Kp=%.4f Ki=%.4f\n",
                   at->Ku, at->Tu, at->Kp, at->Ki);
            at->state = AUTOTUNE_DONE;
        }
    }

    /* Relay output centered around relay_center.
     * relay_high means "drive measurement up". For normal plant (error_sign=+1)
     * that means higher output. For inverted plant (error_sign=-1) it means
     * lower output.                                                          */
    float out_hi = at->relay_center + at->relay_amp;
    float out_lo = at->relay_center - at->relay_amp;

    if (at->error_sign < 0.0f) {
        float tmp = out_hi;
        out_hi = out_lo;
        out_lo = tmp;
    }

    return at->relay_high ? out_hi : out_lo;
}
