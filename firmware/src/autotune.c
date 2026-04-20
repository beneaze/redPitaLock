#include "autotune.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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
    at->Kd = 0.0f;

    /* Memory barrier: ensure all fields above are visible before the PID
     * thread sees state == RUNNING.                                          */
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

    /* Track peaks in each half-cycle */
    if (at->relay_high) {
        if (measurement > at->peak_hi)
            at->peak_hi = measurement;
    } else {
        if (measurement < at->peak_lo)
            at->peak_lo = measurement;
    }

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
            float a  = sum_a / (float)at->n_measured;

            /* Ku = 4d / (pi * a)  where d = relay_amp */
            if (a > 1e-9f)
                at->Ku = (4.0f * at->relay_amp) / ((float)M_PI * a);
            else
                at->Ku = 0.0f;

            /* Tyreus-Luyben PI rules (conservative, good for fast digital loops).
             * Kd is computed for reference but not auto-applied.              */
            at->Kp = at->Ku / 3.2f;
            at->Ki = (at->Tu > 1e-9f) ? (at->Ku / (7.04f * at->Tu)) : 0.0f;
            at->Kd = at->Ku * at->Tu / 13.86f;

            printf("[autotune] DONE: Ku=%.4f Tu=%.4fs -> Kp=%.4f Ki=%.4f Kd=%.6f\n",
                   at->Ku, at->Tu, at->Kp, at->Ki, at->Kd);
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
