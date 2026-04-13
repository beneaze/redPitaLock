#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
    float integral_max;
} pid_state_t;

void  pid_init(pid_state_t *pid, float kp, float ki, float kd,
               float out_min, float out_max, float integral_max);

void  pid_reset(pid_state_t *pid);

float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt,
                 float error_sign);

#endif /* PID_H */
