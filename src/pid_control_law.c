#include "pid_control_law.h"

static int reset (struct siso_control_law * self)
{
    struct pid_control_law *pl =
        CL_CONTAINER_OF(self, struct pid_control_law, interface);

    pl->integrator = 0.0f;
    pl->prev_error = 0.0f;
    pl->interface.reference = 0.0f;
    pl->interface.measurement = 0.0f;

    return 0;
}

static int set (struct siso_control_law * self, float reference, float measurement)
{
    struct pid_control_law *pl =
        CL_CONTAINER_OF(self, struct pid_control_law, interface);

    pl->interface.reference = reference;
    pl->interface.measurement = measurement;
    return 0;
}

static int update (struct siso_control_law * self, float dt, float *command)
{
    (void)dt;

    struct pid_control_law *pl =
        CL_CONTAINER_OF(self, struct pid_control_law, interface);

    if(!command)
        return -EINVAL;

    float ref = pl->interface.reference;
    float mes = pl->interface.measurement;
    float kp = pl->kp;
    float ki = pl->ki;
    float kd = pl->kd;
    float integrator = pl->integrator;
    float prev_error = pl->prev_error;
    float max_integrator = pl->max_integrator;

    float error = ref - mes;
    float error_derivative = error - prev_error;

    integrator += error;
    if(integrator > max_integrator) {
        integrator = max_integrator;
    } else if ( integrator < -max_integrator) {
        integrator = -max_integrator;
    }

    float u = (kp * error) + (ki * integrator) + (kd * error_derivative);

    prev_error = error;

    pl->integrator = integrator;
    pl->prev_error = prev_error;
    pl->max_integrator = max_integrator;
    *command = u;

    return 0;
}

int pid_control_law_tune(struct pid_control_law *pl, float kp, float ki, float kd, float max_integrator)
{
    if(!pl)
        return -EINVAL;

    pl->interface.reset = reset;
    pl->interface.set = set;
    pl->interface.update = update;

    pl->kp = kp;
    pl->ki = ki;
    pl->kd = kd;
    pl->max_integrator = max_integrator;

    return (control_law_reset(&pl->interface));
}