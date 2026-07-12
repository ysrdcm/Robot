#include "tracker.h"
#define LASER_PORT GPIOH
#define LASER_PIN  GPIO_PIN_2

extern TIM_HandleTypeDef htim4;
#define SERVO_PAN_MIN   500
#define SERVO_PAN_MAX   2500
#define SERVO_TILT_MIN  1000
#define SERVO_TILT_MAX  2000

static float current_pwm_pan = 1500.0f;
static float current_pwm_tilt = 1500.0f;

// 1. 负责云台跟随（不再控制激光）
void Gimbal_Targeting_Update(float target_x, float target_y, uint8_t is_detected)
{
    static float current_pwm_pan = 1500.0f;
    static float current_pwm_tilt = 1500.0f;

    // 引入平滑目标点
    static float filtered_x = 0.5f;
    static float filtered_y = 0.5f;

    if (is_detected) {
        // 1. 横轴死区与低通滤波
        if (fabs(target_x - filtered_x) > 0.05f) {
            filtered_x = filtered_x * 0.90f + target_x * 0.10f;
        }
        // 2. 纵轴死区与低通滤波
        if (fabs(target_y - filtered_y) > 0.05f) {
            filtered_y = filtered_y * 0.90f + target_y * 0.10f;
        }

        // 3. 计算目标脉宽
        float target_pwm_pan = SERVO_PAN_MAX - filtered_x * (SERVO_PAN_MAX - SERVO_PAN_MIN);
        float target_pwm_tilt = SERVO_TILT_MAX - filtered_y * (SERVO_TILT_MAX - SERVO_TILT_MIN);

        // 4. 降低逼近速度系数（从 0.15f 降到 0.06f），杜绝物理惯性带来的超调震荡
        current_pwm_pan += 0.06f * (target_pwm_pan - current_pwm_pan);
        current_pwm_tilt += 0.06f * (target_pwm_tilt - current_pwm_tilt);
    } else {
        // 慢回中
        current_pwm_pan += 0.05f * (1500.0f - current_pwm_pan);
        current_pwm_tilt += 0.05f * (1500.0f - current_pwm_tilt);
        filtered_x = 0.5f;
        filtered_y = 0.5f;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)current_pwm_pan);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint32_t)current_pwm_tilt);
}

// 2. 单独控制激光开火
void Laser_Fire(uint8_t state)
{
    if(state) HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
    else      HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
}
