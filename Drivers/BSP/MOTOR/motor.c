#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

/* Direction GPIOs; PWM channels are selected in Motor_SetSpeed(). */
#define LF_IN1_PORT GPIOC
#define LF_IN1_PIN  GPIO_PIN_13
#define LF_IN2_PORT GPIOG
#define LF_IN2_PIN  GPIO_PIN_5

#define RF_IN1_PORT GPIOG
#define RF_IN1_PIN  GPIO_PIN_3
#define RF_IN2_PORT GPIOD
#define RF_IN2_PIN  GPIO_PIN_15

#define RR_IN1_PORT GPIOB
#define RR_IN1_PIN  GPIO_PIN_14
#define RR_IN2_PORT GPIOQ
#define RR_IN2_PIN  GPIO_PIN_0

#define LR_IN1_PORT GPIOH
#define LR_IN1_PIN  GPIO_PIN_6
#define LR_IN2_PORT GPIOE
#define LR_IN2_PIN  GPIO_PIN_15

/**
 * @brief  单独设置某个电机的速度和方向
 * @param  motor_id: 1(左前), 2(右前), 3(左后), 4(右后)
 * @param  speed: -999 ~ 999
 */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed)
{
    uint8_t is_forward = (speed >= 0) ? 1 : 0;
    uint16_t abs_speed = (speed >= 0) ? speed : -speed;
    if (abs_speed > 999) abs_speed = 999;

    switch (motor_id)
    {
        case 1: /* Left front, TIM1_CH1. */
            if (is_forward) {
                HAL_GPIO_WritePin(LF_IN1_PORT, LF_IN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LF_IN2_PORT, LF_IN2_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(LF_IN1_PORT, LF_IN1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LF_IN2_PORT, LF_IN2_PIN, GPIO_PIN_RESET);
            }
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs_speed);
            break;

        case 2: /* Right front, TIM1_CH4. */
            if (is_forward) {
                HAL_GPIO_WritePin(RF_IN1_PORT, RF_IN1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RF_IN2_PORT, RF_IN2_PIN, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(RF_IN1_PORT, RF_IN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(RF_IN2_PORT, RF_IN2_PIN, GPIO_PIN_SET);
            }
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs_speed);
            break;

        case 3: /* Right rear, TIM8_CH1. */
            if (is_forward) {
                HAL_GPIO_WritePin(RR_IN1_PORT, RR_IN1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RR_IN2_PORT, RR_IN2_PIN, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(RR_IN1_PORT, RR_IN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(RR_IN2_PORT, RR_IN2_PIN, GPIO_PIN_SET);
            }
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, abs_speed);
            break;

        case 4: /* Left rear, TIM8_CH2. */
            if (is_forward) {
                HAL_GPIO_WritePin(LR_IN1_PORT, LR_IN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LR_IN2_PORT, LR_IN2_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(LR_IN1_PORT, LR_IN1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LR_IN2_PORT, LR_IN2_PIN, GPIO_PIN_RESET);
            }
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, abs_speed);
            break;
    }
}
