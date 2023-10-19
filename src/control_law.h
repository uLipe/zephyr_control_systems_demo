#ifndef __CONTROL_LAW_H
#define __CONTROL_LAW_H

#include <stddef.h>
#include <errno.h>
#include <math.h>

//Abstract control Law class and methods:
struct siso_control_law {
    int (*reset)(struct siso_control_law * self);
    int (*set)(struct siso_control_law * self, float reference, float measurement);
    int (*update)(struct siso_control_law * self, float dt, float *command);

    float error;
    float measurement;
    float reference;
};

static inline int control_law_reset(struct siso_control_law *cl)
{
    if(!cl)
        return -EINVAL;

    if(cl->reset) {
        return cl->reset(cl);
    }

    return -ENOTSUP;
}

static inline int control_law_set(struct siso_control_law *cl, float reference, float measurement)
{
    if(!cl)
        return -EINVAL;

    if(cl->set) {
        return cl->set(cl, reference, measurement);
    }

    return -ENOTSUP;
}

static inline float control_law_update(struct siso_control_law *cl, float dt, float *command)
{
    if(!cl)
        return -EINVAL;

    if(cl->update) {
        return cl->update(cl, dt, command);
    }

    return -ENOTSUP;
}


//extract derived control law class from this base class:
#define CL_CONTAINER_OF(ptr, type, field)  ((type *)(((char *)(ptr)) - offsetof(type, field)))

#endif