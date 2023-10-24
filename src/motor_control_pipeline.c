#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/counter.h>
#include "motor_control_pipeline.h"

#define CONTROL_PIPELINE_US_PERIOD (500)

static struct motor_control_pipeline* pipeline;
static float now_seconds = 0.0f;
static const struct device *timer_dev = DEVICE_DT_GET( DT_INST(0,st_stm32_counter));
static struct counter_alarm_cfg alarm_cfg;

static void motor_control_work_handler(struct k_work *work)
{
    uint32_t now_ticks;
    counter_get_value(timer_dev, &now_ticks);
    now_seconds = (float)(counter_ticks_to_us(timer_dev, now_ticks)) * 1e-6;

    if(pipeline) {
        float last_time = pipeline->last_time;
        float target_position = pipeline->target_position;
        float current_position;
        float control_effort;
        int ticks = pipeline->ticks;
        int sample_ratio = pipeline->sample_ratio;
        int err, werr;

        ticks++;
        if(ticks == sample_ratio) {
            ticks = 0;
            float dt = fabs(now_seconds - last_time);

            err = motor_hardware_get_angle(pipeline->hw, &current_position);

            if(pipeline->position_cl) {
                control_law_set(pipeline->position_cl, target_position, current_position);
                control_law_update(pipeline->position_cl, dt, &control_effort);
            } else {
                control_effort = target_position;
            }

            werr = motor_hardware_set_speed(pipeline->hw, control_effort);

            pipeline->control_effort = control_effort;
            pipeline->current_position = current_position;
            pipeline->last_time = now_seconds;
            pipeline->dt = dt;
            pipeline->reading_error = err;
            pipeline->writing_error = werr;
        }
        pipeline->ticks = ticks;
    }
}

K_WORK_DEFINE(motor_work, motor_control_work_handler);

static void timer_interrupt_fn(const struct device *counter_dev,
				      uint8_t chan_id, uint32_t ticks,
				      void *user_data)
{
    k_work_submit(&motor_work);
    counter_set_channel_alarm(timer_dev, 0, &alarm_cfg);
}

static int motor_control_pipeline_init_backend(void)
{
    counter_start(timer_dev);
	alarm_cfg.flags = 0;
	alarm_cfg.ticks = counter_us_to_ticks(timer_dev, CONTROL_PIPELINE_US_PERIOD);
	alarm_cfg.callback = timer_interrupt_fn;
	alarm_cfg.user_data = &alarm_cfg;
    counter_set_channel_alarm(timer_dev, 0, &alarm_cfg);

    return 0;
}

SYS_INIT(motor_control_pipeline_init_backend, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

int motor_control_pipeline_add_hw(struct motor_control_pipeline* cp, struct motor_hardware_if *hw)
{
    if(!cp || !hw)
        return -EINVAL;

    cp->hw = hw;

    return 0;
}

int motor_control_pipeline_add_control(struct motor_control_pipeline* cp, struct siso_control_law *cl)
{
    if(!cp)
        return -EINVAL;

    cp->position_cl = cl;

    return 0;
}

int motor_control_pipeline_remove_control(struct motor_control_pipeline* cp)
{
    if(!cp)
        return -EINVAL;

    cp = NULL;

    return 0;
}

int motor_control_pipeline_register(struct motor_control_pipeline* cp, int sample_ratio)
{
    if(!cp)
        return -EINVAL;

    if(!cp->hw)
        return -ENODEV;

    if(!sample_ratio)
        return -EINVAL;

    cp->sample_ratio = sample_ratio;
    cp->ticks = 0;
    cp->last_time = 0.0f;

    pipeline = cp;

    return 0;
}

int motor_control_pipeline_set_position(struct motor_control_pipeline* cp, float target_position)
{
    if(!cp)
        return -EINVAL;

    cp->target_position = target_position;

    return 0;
}
