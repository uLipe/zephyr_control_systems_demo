/*
 * Copyright (c) 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include "control_law.h"
#include "adrc_control_law.h"
#include "pid_control_law.h"

#include "motor_hardware.h"
#include "motor_hardware_mf4005.h"
#include "motor_control_pipeline.h"

static const struct device * motor_can_port = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
static struct motor_hardware_mf4005 motor_1;
static struct motor_control_pipeline control_1;
static struct adrc_control_law motor_adrc_1;
static struct pid_control_law motor_pid_1;

int main(void)
{
	pid_control_law_tune(&motor_pid_1, 1.0f, 0.0f, 0.0f, 36000);
	adrc_control_law_tune(&motor_adrc_1, 0.0005f, 300.0f, 1.0f, 10.0f, 0.001f);
	motor_hardware_mf4005_init(&motor_1, motor_can_port);
	motor_control_pipeline_add_hw(&control_1, motor_hardware_mf4005_get_if(&motor_1));
	motor_control_pipeline_add_control(&control_1, get_adrc_control_law_interface(&motor_adrc_1));
	motor_control_pipeline_register(&control_1, 1);
	motor_control_pipeline_set_position(&control_1, 90);

	return 0;
}

static int motor_set_ref(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		return -EINVAL;
	}

	return motor_control_pipeline_set_position(&control_1, (strtof(argv[1], NULL)));
}

static int motor_gen_ramp(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		return -EINVAL;
	}

	float start = strtof(argv[1], NULL);
	float end = strtof(argv[2], NULL);
	float current = start;
	float steps = (end - start) / 16.0f;

	for(; current < end; current += steps) {
		motor_control_pipeline_set_position(&control_1, current);
		k_sleep(K_MSEC(250));
	}

	return 0;

}


static int motor_change_control(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		return -EINVAL;
	}

	long option = strtol(argv[1], NULL, 10);

	if(!option) {
		motor_control_pipeline_add_control(&control_1, get_pid_control_law_interface(&motor_pid_1));
	} else {
		motor_control_pipeline_add_control(&control_1, get_adrc_control_law_interface(&motor_adrc_1));
	}

	return 0;
}

static int motor_set_gains_pid(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 5) {
		return -EINVAL;
	}

	pid_control_law_tune(&motor_pid_1,
						(strtof(argv[1], NULL)),
						(strtof(argv[2], NULL)),
						(strtof(argv[3], NULL)),
						(strtof(argv[4], NULL)));

	return 0;

}

static int motor_set_gains_adrc(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 6) {
		return -EINVAL;
	}

	adrc_control_law_tune(&motor_adrc_1,
						(strtof(argv[1], NULL)),
						(strtof(argv[2], NULL)),
						(strtof(argv[3], NULL)),
						(strtof(argv[4], NULL)),
						(strtof(argv[5], NULL)));

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
