#include "motor_driver.h"

static TIM_HandleTypeDef      *s_htim = NULL;
static motor_channel_config_t  s_channels[NUM_MOTORS];
static int16_t                 s_current_speeds[NUM_MOTORS] = {0, 0, 0, 0};

void motor_driver_init(TIM_HandleTypeDef *htim, const motor_channel_config_t *configs)
{
    s_htim = htim;
    for (int i = 0; i < NUM_MOTORS; i++) {
        s_channels[i] = configs[i];
        if (s_htim != NULL) {
            HAL_TIM_PWM_Start(s_htim, s_channels[i].tim_channel);
        }
        s_current_speeds[i] = 0;
    }
    motor_driver_stop_all();
}

void motor_driver_set(uint8_t motor_id, int16_t speed)
{
    if (motor_id >= NUM_MOTORS || s_htim == NULL) {
        return;
    }

    /* Clamp speed strictly to [-1000, 1000] */
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    s_current_speeds[motor_id] = speed;

    /* Set Direction Pin: SET for forward, RESET for reverse */
    if (s_channels[motor_id].dir_port != NULL) {
        HAL_GPIO_WritePin(s_channels[motor_id].dir_port,
                          s_channels[motor_id].dir_pin,
                          speed >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /* Compute duty cycle based on active Timer Auto-Reload Register (ARR) */
    uint32_t mag = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)speed;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(s_htim);
    uint32_t duty = (mag * arr) / 1000;

    __HAL_TIM_SET_COMPARE(s_htim, s_channels[motor_id].tim_channel, duty);
}

void motor_driver_set_all(const int16_t speeds[NUM_MOTORS])
{
    if (!speeds) return;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_driver_set(i, speeds[i]);
    }
}

void motor_driver_stop_all(void)
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_driver_set(i, 0);
    }
}

int16_t motor_driver_get_speed(uint8_t motor_id)
{
    if (motor_id < NUM_MOTORS) {
        return s_current_speeds[motor_id];
    }
    return 0;
}
