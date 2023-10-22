#ifndef __MOTOR_HARDWARE_H
#define __MOTOR_HARDWARE_H

#include <stddef.h>
#include <errno.h>
#include <math.h>

struct motor_hardware_if {
    int (*reset) (struct motor_hardware_if *self);
    int (*set_current) (struct motor_hardware_if *self, float current);
    int (*get_angle)(struct motor_hardware_if *self, float *degrees);
};

static inline int motor_hardware_reset(struct motor_hardware_if *mh)
{
    if(!mh)
        return -EINVAL;

    if(mh->reset) {
        return mh->reset(mh);
    }

    return -ENOTSUP;
}

static inline int motor_hardware_set_current(struct motor_hardware_if *mh, float current)
{
    if(!mh)
        return -EINVAL;

    if(mh->set_current) {
        return mh->set_current(mh, current);
    }

    return -ENOTSUP;
}

static inline float motor_hardware_get_angle(struct motor_hardware_if *mh, float *angle)
{
    if(!mh)
        return -EINVAL;

    if(mh->get_angle) {
        return mh->get_angle(mh, angle);
    }

    return -ENOTSUP;
}

//extract derived motor hw class from this base class:
#define MH_CONTAINER_OF(ptr, type, field)  ((type *)(((char *)(ptr)) - offsetof(type, field)))

#endif