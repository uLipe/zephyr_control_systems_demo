#ifndef __MOTOR_HARDWARE_MF4005_H
#define __MOTOR_HARDWARE_MF4005_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include "motor_hardware.h"

struct motor_hardware_mf4005 {
    struct motor_hardware_if interface;
    const struct device *can_port;
    float current_angle_deg;
};

static inline
struct motor_hardware_if * motor_hardware_mf4005_get_if(struct motor_hardware_mf4005 *mh)
{
    if(mh) {
        return &mh->interface;
    }

    return NULL;
}

int motor_hardware_mf4005_init(struct motor_hardware_mf4005 *mh, const struct device *can_port);

#endif
