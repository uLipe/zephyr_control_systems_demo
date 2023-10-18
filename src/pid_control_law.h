#ifndef __PID_CONTROL_LAW_H
#define __PID_CONTROL_LAW_H

#include "control_law.h"

struct pid_control_law {
    struct siso_control_law interface;
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float max_integrator_value;
};

static inline struct siso_control_law * get_pid_control_law_interface(struct pid_control_law  *pl)
{
    if(pl) {
        return &pl->interface;
    }

    return NULL;
}

int pid_control_law_tune(struct pid_control_law *pl, float kp, float ki, float kd);

#endif


