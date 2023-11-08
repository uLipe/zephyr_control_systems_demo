#ifndef __LQR_SISO_CONTROL_LAW_H
#define __LQR_SISO_CONTROL_LAW_H

#include "control_law.h"

struct lqr_control_law {
    struct siso_control_law interface;
    float kr;
    float kx[3];
    float l[3];
    float zhat[3];
    float u_prev;
};

static inline struct siso_control_law * get_lqr_control_law_interface(struct lqr_control_law *ll)
{
    if(ll) {
        return &ll->interface;
    }

    return NULL;
}

int lqr_control_law_tune(struct lqr_control_law *ll, float dt, float wo, float kr, float kx[3]);

#endif


