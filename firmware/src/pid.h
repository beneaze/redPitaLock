#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;       /* unused for D; kept for possible future use       */
    float prev_measurement; /* for derivative-on-measurement (noise robust)   */
    int   deriv_init;       /* 0 = skip first D sample to avoid spike          */
    float out_min;
    float out_max;
    float integral_max;
} pid_state_t;

void  pid_init(pid_state_t *pid, float kp, float ki, float kd,
               float out_min, float out_max, float integral_max);

void  pid_reset(pid_state_t *pid);

float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt,
                 float error_sign);

/* Bumpless transfer: re-seed the integrator so that the next pid_update()
 * call (with the same setpoint/measurement) returns `target_output`.
 * Also restarts the derivative so no D kick occurs on the first sample.
 * If ki is ~0 the integrator is left untouched.                               */
void  pid_seed_output(pid_state_t *pid, float setpoint, float measurement,
                      float error_sign, float target_output);

#endif /* PID_H */
