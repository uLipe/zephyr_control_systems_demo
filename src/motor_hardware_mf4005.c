#include <zephyr/sys/printk.h>
#include "motor_hardware_mf4005.h"

#define MF4005_CAN_DEVID        0x141
#define MF4005_SET_CURRENT_CMD  0xA1
#define MF4005_SET_SPEED_CMD    0xA2
#define MF4005_TURN_OFF_CMD     0x80
#define MF4005_TURN_ON_CMD      0x88

CAN_MSGQ_DEFINE(mh_can_msgq, 2);

const float mh_max_current_amperes = 2.8f;
const float mh_max_speed_dps = 14000.0f;
const float mh_encoder_to_degrees = (360.0f / 65536.0f); //MF4005 encoder is 16bit
const float mh_current_to_raw = (32.0f / 0.240f); //MF4005 max current is 240 mA
const float mh_speed_to_raw = 100.0f;

static const struct can_filter motor_filter = {
    .flags = CAN_FILTER_DATA,
    .id = MF4005_CAN_DEVID,
    .mask = CAN_STD_ID_MASK
};

static int reset (struct motor_hardware_if *self)
{
    struct motor_hardware_mf4005 *mh =
        MH_CONTAINER_OF(self, struct motor_hardware_mf4005, interface);

    struct can_frame tx_frame = {
        .flags = 0,
        .id = MF4005_CAN_DEVID,
        .dlc = 8,
        .data[0] = MF4005_TURN_OFF_CMD
	};

    struct can_frame rx_frame = {
        .flags = 0,
        .id = MF4005_CAN_DEVID,
        .dlc = 8
    };

    can_start(mh->can_port);
    can_add_rx_filter_msgq(mh->can_port, &mh_can_msgq, &motor_filter);

    int err = can_send(mh->can_port, &tx_frame, K_MSEC(100), NULL, NULL);
    if(err)
        return err;

    err = k_msgq_get(&mh_can_msgq, &rx_frame, K_MSEC(100));
    if(err)
        return err;

    tx_frame.data[0] = MF4005_TURN_ON_CMD;

    can_send(mh->can_port, &tx_frame, K_MSEC(100), NULL, NULL);
    if(err)
        return err;

    err = k_msgq_get(&mh_can_msgq, &rx_frame, K_MSEC(100));
        return err;

    return motor_hardware_set_speed(self, 0.0f);
}

static int set_speed (struct motor_hardware_if *self, float speed)
{
    struct motor_hardware_mf4005 *mh =
        MH_CONTAINER_OF(self, struct motor_hardware_mf4005, interface);

    struct can_frame tx_frame = {
        .flags = 0,
        .id = MF4005_CAN_DEVID,
        .dlc = 8
    };

    if(speed > mh_max_speed_dps) {
        speed = mh_max_speed_dps;
    } else if (speed < -mh_max_speed_dps) {
        speed = -mh_max_speed_dps;
    }

    int32_t command = (int32_t)(speed * mh_speed_to_raw);

    tx_frame.data[0] = MF4005_SET_SPEED_CMD;
    tx_frame.data[4] = (uint8_t)(command & 0xFF);
    tx_frame.data[5] = (uint8_t)((command >> 8)  & 0xFF);
    tx_frame.data[6] = (uint8_t)((command >> 16) & 0xFF);
    tx_frame.data[7] = (uint8_t)((command >> 24) & 0xFF);

    int err = can_send(mh->can_port, &tx_frame, K_NO_WAIT, NULL, NULL);
    if(err)
        return err;

    return 0;
}

static int get_angle(struct motor_hardware_if *self, float *degrees)
{
    struct motor_hardware_mf4005 *mh =
        MH_CONTAINER_OF(self, struct motor_hardware_mf4005, interface);

    struct can_frame rx_frame = {
        .flags = 0,
        .id = MF4005_CAN_DEVID,
        .dlc = 8
    };

    if(!degrees) {
        return -EINVAL;
    }

    int err = k_msgq_get(&mh_can_msgq, &rx_frame, K_NO_WAIT);
    if(!err) {
        uint16_t raw_angle = rx_frame.data[7];
        raw_angle <<= 8;
        raw_angle |= rx_frame.data[6];

        mh->current_angle_deg = (float )(raw_angle) * mh_encoder_to_degrees;
    }

    *degrees = mh->current_angle_deg;
    return err;
}

int motor_hardware_mf4005_init(struct motor_hardware_mf4005 *mh, const struct device *can_port)
{
    if(!can_port)
        return -EINVAL;

    mh->can_port = can_port;
    mh->interface.reset = reset;
    mh->interface.set_speed = set_speed;
    mh->interface.get_angle = get_angle;

    return motor_hardware_reset(&mh->interface);
}
