#ifndef PID_H
#define PID_H

/* Standard C PID controller, no dynamic allocation. */

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float max_integral;
    float max_output;
    bool  first_compute; /* true after init/reset: next compute has D=0 */
} PidController;

void  pid_init(PidController *pid, float kp, float ki, float kd,
               float max_integral, float max_output);
void  pid_reset(PidController *pid);
float pid_compute(PidController *pid, float error, float dt_sec);

#endif /* PID_H */
