#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#define AUTOTUNE_IDLE    0
#define AUTOTUNE_RUNNING 1
#define AUTOTUNE_DONE    2
#define AUTOTUNE_FAILED  3
#define AUTOTUNE_BIASING 4   /* one-shot bisection to find operating point   */

#define AUTOTUNE_MAX_CYCLES 32

/* One-shot bisection that finds a DAC drive where input ~= setpoint, used as
 * the relay center.  Tuned so the whole pre-relay phase finishes well under
 * a second at 50 ms / step.                                                  */
#define AUTOTUNE_BIAS_STEPS       8
#define AUTOTUNE_BIAS_SETTLE_MS   50
#define AUTOTUNE_BIAS_TOL_V       0.005f

/* Bisection bracket for the auto-bias phase.  We search the whole DAC
 * envelope rather than a user-configurable window -- if the loop polarity
 * is right and the plant reaches the setpoint anywhere in [-1, +1] V, the
 * bisection will find it.                                                    */
#define AUTOTUNE_BIAS_LO         (-1.0f)
#define AUTOTUNE_BIAS_HI         ( 1.0f)

typedef struct {
    /* Configuration */
    float relay_amp;        /* relay half-amplitude (volts)                    */
    float relay_center;     /* output center point (volts)                     */
    float hysteresis;       /* noise rejection band around setpoint (volts)    */
    float error_sign;       /* +1 or -1: plant polarity                       */
    int   target_cycles;    /* full cycles to measure before computing gains   */
    int   settle_cycles;    /* initial cycles to discard as transient          */
    float timeout_s;        /* abort if no result after this many seconds      */

    /* --- BIASING-only fields (operating-point bisection) ---------------- */
    float bias_lo;          /* current bisection lower bound (V)              */
    float bias_hi;          /* current bisection upper bound (V)              */
    int   bias_step;        /* number of bisection steps taken                */
    int   bias_wait_ticks;  /* supervisor ticks left to wait at current drive */
    float bias_drive;       /* DAC voltage being driven this step             */

    /* Runtime state -- `state` is volatile: written by TCP thread, read by supervisor */
    volatile int state;     /* AUTOTUNE_IDLE / RUNNING / DONE / FAILED        */
    int   relay_high;       /* 1 = output is center+amp, 0 = center-amp       */
    int   half_cycle_count; /* number of half-cycles (zero crossings) seen     */
    float peak_hi;          /* global max input in the current cycle           */
    float peak_lo;          /* global min input in the current cycle           */
    float time_in_cycle;    /* elapsed time in current full cycle (seconds)    */
    float elapsed;          /* total elapsed time since start (seconds)        */

    /* Collected measurements (after settle period) */
    float periods[AUTOTUNE_MAX_CYCLES];
    float amplitudes[AUTOTUNE_MAX_CYCLES];
    int   n_measured;       /* number of completed full cycles stored          */

    /* Results */
    float Ku;
    float Tu;
    float Kp;
    float Ki;
} autotune_state_t;

void  autotune_init(autotune_state_t *at, float relay_amp, float relay_center,
                    float hysteresis, float error_sign,
                    int target_cycles, int settle_cycles,
                    float timeout_s);

/* Request the one-shot bisection that finds a DAC drive where the input
 * sits near the setpoint.  The bisection sweeps the full +/- 1 V DAC
 * envelope (AUTOTUNE_BIAS_LO / AUTOTUNE_BIAS_HI) and the supervisor
 * eventually calls autotune_init() with the discovered center.              */
void  autotune_request_bias(autotune_state_t *at,
                            float relay_amp,
                            float hysteresis, float error_sign,
                            int target_cycles, int settle_cycles,
                            float timeout_s);

/* One bisection tick.  Caller wakes us at SUPERVISOR_PERIOD_US; we count
 * down `bias_wait_ticks` between drives.  When enough steps have run (or
 * the input is within tolerance) we kick off the relay phase by calling
 * autotune_init() internally.  Returns the DAC voltage to drive this tick. */
float autotune_bias_step(autotune_state_t *at, float setpoint, float input,
                         int settle_ticks_per_step);

/* Call once per loop iteration while autotune is RUNNING.
 * Returns the relay output voltage to write to the DAC.
 * When enough cycles have been measured, sets state to AUTOTUNE_DONE and
 * populates Ku, Tu, Kp, Ki.  On timeout sets state to AUTOTUNE_FAILED.     */
float autotune_step(autotune_state_t *at, float setpoint, float measurement,
                    float dt);

#endif /* AUTOTUNE_H */
