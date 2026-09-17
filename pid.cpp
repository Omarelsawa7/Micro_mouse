#include "pid.h"

void pid_init(PidController *pid, float kp, float ki, float kd,
              float max_integral, float max_output)
{
    if (pid == 0) {
        return;
    }
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->max_integral = max_integral;
    pid->max_output   = max_output;
    pid->first_compute = true;
}

void pid_reset(PidController *pid)
{
    if (pid == 0) {
        return;
    }
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_compute = true;
}

float pid_compute(PidController *pid, float error, float dt_sec)
{
    float derivative;
    float output;

    if (pid == 0) {
        return 0.0f;
    }
    if (dt_sec <= 0.0f) {
        return 0.0f;
    }

    /* Integral with anti-windup clamp */
    pid->integral += error * dt_sec;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    /* FIX: suppress derivative kick on first compute after reset.
     * Old code had prev_error=0, so first error of e.g. 200 with dt=0.005
     * gave D = 0.02*40000 = 800 -> saturated every cell entry. */
    if (pid->first_compute) {
        pid->first_compute = false;
        pid->prev_error = error;
        derivative = 0.0f;
    } else {
        derivative = (error - pid->prev_error) / dt_sec;
    }
    pid->prev_error = error;

    output = (pid->kp * error)
           + (pid->ki * pid->integral)
           + (pid->kd * derivative);

    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }
    return output;
}
