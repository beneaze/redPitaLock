#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#define AUTOTUNE_IDLE    0
#define AUTOTUNE_RUNNING 1
#define AUTOTUNE_DONE    2
#define AUTOTUNE_FAILED  3

#define AUTOTUNE_MAX_CYCLES 32

typedef struct {
    /* Configuration */
    float relay_amp;        /* relay half-amplitude (volts)                    */
    float relay_center;     /* output center point (volts)                     */
    float hysteresis;       /* noise rejection band around setpoint (volts)    */
    float error_sign;       /* +1 or -1: plant polarity                       */
    int   target_cycles;    /* full cycles to measure before computing gains   */
    int   settle_cycles;    /* initial cycles to discard as transient          */
    float timeout_s;        /* abort if no result after this many seconds      */

    /* Runtime state -- `state` is volatile: written by TCP thread, read by PID thread */
    volatile int state;     /* AUTOTUNE_IDLE / RUNNING / DONE / FAILED        */
    int   relay_high;       /* 1 = output is center+amp, 0 = center-amp       */
    int   half_cycle_count; /* number of half-cycles (zero crossings) seen     */
    float peak_hi;          /* max input in current high half-cycle            */
    float peak_lo;          /* min input in current low half-cycle             */
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
    float Kd;
} autotune_state_t;

void  autotune_init(autotune_state_t *at, float relay_amp, float relay_center,
                    float hysteresis, float error_sign,
                    int target_cycles, int settle_cycles,
                    float timeout_s);

/* Call once per loop iteration while autotune is RUNNING.
 * Returns the relay output voltage to write to the DAC.
 * When enough cycles have been measured, sets state to AUTOTUNE_DONE and
 * populates Ku, Tu, Kp, Ki, Kd.  On timeout sets state to AUTOTUNE_FAILED.  */
float autotune_step(autotune_state_t *at, float setpoint, float measurement,
                    float dt);

#endif /* AUTOTUNE_H */
