#include "pid.h"

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(pid_state_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float integral_max) {
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->out_min      = out_min;
    pid->out_max      = out_max;
    pid->integral_max = integral_max;
}

void pid_reset(pid_state_t *pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt,
                 float error_sign) {
    float error = error_sign * (setpoint - measurement);

    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    float p_term = pid->kp * error;
    float d_term = pid->kd * derivative;

    /* Only integrate when doing so doesn't push us further into saturation */
    float new_integral = pid->integral + error * dt;
    new_integral = clampf(new_integral, -pid->integral_max, pid->integral_max);
    float output_new = p_term + pid->ki * new_integral + d_term;

    if (output_new > pid->out_max && error > 0.0f) {
        /* Would saturate high -- freeze integral */
    } else if (output_new < pid->out_min && error < 0.0f) {
        /* Would saturate low -- freeze integral */
    } else {
        pid->integral = new_integral;
    }

    float output = p_term + pid->ki * pid->integral + d_term;
    return clampf(output, pid->out_min, pid->out_max);
}
