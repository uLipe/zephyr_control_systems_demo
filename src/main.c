/*
 * Copyright (c) 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include "motor_hardware.h"
#include "motor_hardware_mf4005.h"

static const struct device * motor_can_port = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
static struct motor_hardware_mf4005 motor_1;

int main(void)
{
	return 0;
}

static int motor_set_ref(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		return -EINVAL;
	}
	return 0;
}

static int motor_gen_ramp(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		return -EINVAL;
	}

	return 0;
}


static int motor_change_control(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		return -EINVAL;
	}

	return 0;
}

static int motor_set_gains_pid(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		return -EINVAL;
	}

	return 0;
}

static int motor_set_gains_adrc(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		return -EINVAL;
	}

	return 0;
}

static int motor_set_power(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		return -EINVAL;
	}
	struct motor_hardware_if *m = motor_hardware_mf4005_get_if(&motor_1);
	return motor_hardware_set_current(m, (strtof(argv[1], NULL)));
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_control,
	SHELL_CMD(generate_test_ramp, NULL, "Generate an up and down ramp of 5 seconds", motor_gen_ramp),
	SHELL_CMD(set_reference, NULL, "Set velocity reference in RPM", motor_set_ref),
	SHELL_CMD(change_control_algo, NULL, "0 - PID, 1 - ADRC", motor_change_control),
	SHELL_CMD(set_gains_pid, NULL, "Set velocity PID KP and KI", motor_set_gains_pid),
	SHELL_CMD(set_gains_adrc, NULL, "Set velocity ADRC Gains", motor_set_gains_adrc),
	SHELL_CMD(motor_set_torque, NULL, "Set motor power in openloop", motor_set_power),
	SHELL_SUBCMD_SET_END
	);

SHELL_CMD_REGISTER(zephyr_motor_control, &motor_control, "Zephyr RTOS Motor control commands", NULL);
