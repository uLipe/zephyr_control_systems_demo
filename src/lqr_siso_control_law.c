#include "lqr_siso_control_law.h"

static int reset (struct siso_control_law * self)
{
    struct lqr_control_law *ll =
        CL_CONTAINER_OF(self, struct lqr_control_law, interface);

    ll->interface.reference = 0.0f;
    ll->interface.measurement = 0.0f;
    ll->zhat[0] = 0.0f;
    ll->zhat[1] = 0.0f;
    ll->zhat[2] = 0.0f;
    ll->u_prev = 0.0f;
    ll->interface.error = 0.0f;

    return 0;
}

static int set (struct siso_control_law * self, float reference, float measurement)
{
    struct lqr_control_law *ll =
        CL_CONTAINER_OF(self, struct lqr_control_law, interface);

    ll->interface.reference = reference;
    ll->interface.measurement = measurement;

    return 0;
}

static int update (struct siso_control_law * self, float dt, float *command)
{
    struct lqr_control_law *ll =
        CL_CONTAINER_OF(self, struct lqr_control_law, interface);

    if(!command)
        return -EINVAL;

    float ref = ll->interface.reference;
    float mes = ll->interface.measurement;
    float zhat_1 = ll->zhat[0];
    float zhat_2 = ll->zhat[1];
    float zhat_3 = ll->zhat[2];
    float l1 = ll->l[0];
    float l2 = ll->l[1];
    float l3 = ll->l[2];
    float u_prev = ll->u_prev;
    float kx[3] = {ll->kx[0], ll->kx[1], ll->kx[2]};
    float kr = ll->kr;

    //We don't have a full picture of the double integrator
    //state, so observe the dynamics to get them:
    float zhat_dot_1 = -l1 * zhat_1 + zhat_2 + l1 * mes;
    float zhat_dot_2 = -l2 * zhat_1 + zhat_3 + l2 * mes +  u_prev;
    float zhat_dot_3 = -l3 * zhat_1 + l3 * mes;

    zhat_1 += zhat_dot_1 * dt;
    zhat_2 += zhat_dot_2 * dt;
    zhat_3 += zhat_dot_3 * dt;

    //Now compute LQR control law as regular full state
    //feedback control:
    float u = (ref * kr) - (kx[0] * zhat_1 +
                            kx[1] * zhat_2 +
                            kx[2] * zhat_3 );

    ll->u_prev = u;
    ll->zhat[0] = zhat_1;
    ll->zhat[1] = zhat_2;
    ll->zhat[2] = zhat_3;

    *command = u;

    return 0;
}

int lqr_control_law_tune(struct lqr_control_law *ll, float dt, float wo, float kr, float kx[3])
{
    ll->interface.reset = reset;
    ll->interface.set = set;
    ll->interface.update = update;

    ll->kr = kr;
    ll->kx[0] = kx[0];
    ll->kx[1] = kx[1];
    ll->kx[2] = kx[2];

    //Compute discrete time observer gains:
    float z_eso = exp(-wo * dt);
    float one_minus_zeso = (1 - z_eso);

    ll->l[2] = 1 - (z_eso * z_eso * z_eso);
    ll->l[1] = (3.0f /(2 * dt)) * (one_minus_zeso * one_minus_zeso) * (1 + z_eso);
    ll->l[0] = (1 / ( dt * dt)) * (one_minus_zeso * one_minus_zeso * one_minus_zeso * one_minus_zeso);

    return (control_law_reset(&ll->interface));
}
