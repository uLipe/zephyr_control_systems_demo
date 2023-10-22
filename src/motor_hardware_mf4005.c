#include "motor_hardware_mf4005.h"

#define MF4005_CAN_DEVID        0x141
#define MF4005_SET_CURRENT_CMD  0xA1
#define MF4005_TURN_OFF_CMD     0x80
#define MF4005_TURN_ON_CMD      0x88

CAN_MSGQ_DEFINE(mh_can_msgq, 2);

const float mh_max_current_amperes = 2.8f;
const float mh_encoder_to_degrees = (360.0f / (float)(1 << 16)); //MF4005 encoder is 18bit
const float mh_current_to_raw = (2048.0f / 3.0f); //MF4005 max current is 3.0A

static const struct can_filter motor_filter = {
    .flags = CAN_FILTER_DATA | CAN_FILTER_IDE,
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

    can_add_rx_filter_msgq(mh->can_port, &mh_can_msgq, &motor_filter);

    int err = can_send(mh->can_port, &tx_frame, K_MSEC(100), NULL, NULL);
    if(err)
        return err;

    err = k_msgq_get(&mh_can_msgq, &rx_frame, K_MSEC(100));
    if(err)
        return err;

    k_sleep(K_SECONDS(1));
    tx_frame.data[0] = MF4005_TURN_ON_CMD;

    can_send(mh->can_port, &tx_frame, K_MSEC(100), NULL, NULL);
    if(err)
        return err;

    err = k_msgq_get(&mh_can_msgq, &rx_frame, K_MSEC(100));
        return err;

    return motor_hardware_set_current(self, 0.0f);
}

static int set_current (struct motor_hardware_if *self, float current)
{
    struct motor_hardware_mf4005 *mh =
        MH_CONTAINER_OF(self, struct motor_hardware_mf4005, interface);

    struct can_frame tx_frame = {
		.flags = 0,
		.id = MF4005_CAN_DEVID,
		.dlc = 8
	};

    struct can_frame rx_frame = {
		.flags = 0,
		.id = MF4005_CAN_DEVID,
		.dlc = 8
	};

    if(current > mh_max_current_amperes) {
        current = mh_max_current_amperes;
    } else if (current < -mh_max_current_amperes) {
        current = -mh_max_current_amperes;
    }

    int16_t command = (int16_t)(current * mh_current_to_raw);

    tx_frame.data[0] = MF4005_SET_CURRENT_CMD;
    tx_frame.data[1] = (uint8_t)(command & 0xFF);
    tx_frame.data[2] = (uint8_t)((command >> 8) & 0xFF);

    int err = can_send(mh->can_port, &tx_frame, K_MSEC(100), NULL, NULL);
    if(err)
        return err;

    err = k_msgq_get(&mh_can_msgq, &rx_frame, K_MSEC(100));
    if(err)
        return err;

    int16_t raw_angle = rx_frame.data[7];
    raw_angle <<= 8;
    raw_angle |= rx_frame.data[6];

    mh->current_angle_deg = (float )(raw_angle) * mh_encoder_to_degrees;

    return 0;
}

static int get_angle(struct motor_hardware_if *self, float *degrees)
{
    struct motor_hardware_mf4005 *mh =
        MH_CONTAINER_OF(self, struct motor_hardware_mf4005, interface);

    return mh->current_angle_deg;
}

int motor_hardware_mf4005_init(struct motor_hardware_mf4005 *mh, const struct device *can_port)
{
    if(!can_port)
        return -EINVAL;

    mh->can_port = can_port;
    mh->interface.reset = reset;
    mh->interface.set_current = set_current;
    mh->interface.get_angle = get_angle;

    return motor_hardware_reset(&mh->interface);
}
