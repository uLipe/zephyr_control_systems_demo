#include "adrc_control_law.h"

static int reset (struct siso_control_law * self)
{
    struct adrc_control_law *al =
        CL_CONTAINER_OF(self, struct adrc_control_law, interface);

    al->interface.reference = 0.0f;
    al->interface.measurement = 0.0f;
    al->zhat[0] = 0.0f;
    al->zhat[1] = 0.0f;
    al->zhat[2] = 0.0f;
    al->u_prev = 0.0f;

    return 0;
}

static int set (struct siso_control_law * self, float reference, float measurement)
{
    struct adrc_control_law *al =
        CL_CONTAINER_OF(self, struct adrc_control_law, interface);

    al->interface.reference = reference;
    al->interface.measurement = measurement;

    return 0;
}

static int update (struct siso_control_law * self, float dt, float *command)
{
    struct adrc_control_law *al =
        CL_CONTAINER_OF(self, struct adrc_control_law, interface);

    if(!command)
        return -EINVAL;

    float ref = al->interface.reference;
    float mes = al->interface.measurement;
    float zhat_1 = al->zhat[0];
    float zhat_2 = al->zhat[1];
    float zhat_3 = al->zhat[2];
    float l1 = al->l[0];
    float l2 = al->l[1];
    float l3 = al->l[2];
    float kp = al->kp;
    float kd = al->kd;
    float b0 = al->b0;
    float u_prev = al->u_prev;

    //First do the observer dynamics:
    float zhat_dot_1 = -l1 * zhat_1 + zhat_2 + l1 * mes;
    float zhat_dot_2 = -l2 * zhat_1 + zhat_3 + l2 * mes + b0 * u_prev;
    float zhat_dot_3 = -l3 * zhat_1 + l3 * mes;

    //Now update the estimated states:
    zhat_1 += zhat_dot_1 * dt;
    zhat_2 += zhat_dot_2 * dt;
    zhat_3 += zhat_dot_3 * dt;

    //and finally compute the control law:
    float u = (kp * (ref - zhat_1) - kd * zhat_2 - zhat_3) / b0;

    al->u_prev = u;
    al->zhat[0] = zhat_1;
    al->zhat[1] = zhat_2;
    al->zhat[2] = zhat_3;

    *command = u;

    return 0;
}

int adrc_control_law_tune(struct adrc_control_law *al, float dt, float wo, float b0, float kp, float kd)
{
    if(!al)
        return -EINVAL;

    al->interface.reset = reset;
    al->interface.set = set;
    al->interface.update = update;
    al->b0 = b0;
    al->kp = kp;
    al->kd = kd;

    //Compute discrete time observer gains:
    float z_eso = exp(-wo * dt);
    float one_minus_zeso = (1 - z_eso);

    al->l[2] = 1 - (z_eso * z_eso * z_eso);
    al->l[1] = (3.0f /(2 * dt)) * (one_minus_zeso * one_minus_zeso) * (1 + z_eso);
    al->l[0] = (1 / ( dt * dt)) * (one_minus_zeso * one_minus_zeso * one_minus_zeso * one_minus_zeso);

    return (control_law_reset(&al->interface));
}