#include "tracker.h"

#define LASER_PORT GPIOH
#define LASER_PIN  GPIO_PIN_2

extern TIM_HandleTypeDef htim4;

#define SERVO_PAN_MIN                    500.0f
#define SERVO_PAN_MAX                   2500.0f
#define SERVO_TILT_MIN                  1000.0f
#define SERVO_TILT_MAX                  2000.0f
/* 限制单次变化量，避免检测抖动造成云台突跳。 */
#define SERVO_TRACK_MAX_STEP               3.0f
#define GIMBAL_UPDATE_PERIOD_MS            20U
#define GIMBAL_DETECTION_HOLD_MS          300U

/* 摄像头和云台均用 PWM 微秒值标定，向车体右侧转动时 PWM 减小。 */
#define CAMERA_PAN_CENTER_PWM           1500.0f
#define CAMERA_PAN_HALF_RANGE_PWM        800.0f
#define GIMBAL_PAN_CENTER_PWM           1500.0f
/* 该行程只补偿摄像头偏航，不改变摄像头居中时的图像跟踪比例。 */
#define GIMBAL_PAN_HALF_RANGE_PWM        670.0f
#define CAMERA_TO_GIMBAL_PAN_SIGN          1.0f

static float current_pwm_pan = GIMBAL_PAN_CENTER_PWM;
static float current_pwm_tilt = 1500.0f;

static float Servo_Smooth_Step(float current, float target, float gain)
{
    float step = gain * (target - current);

    if (step > SERVO_TRACK_MAX_STEP)
    {
        step = SERVO_TRACK_MAX_STEP;
    }
    else if (step < -SERVO_TRACK_MAX_STEP)
    {
        step = -SERVO_TRACK_MAX_STEP;
    }
    return current + step;
}

static float Servo_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void Gimbal_Targeting_Update(float target_x, float target_y,
                             uint8_t is_detected, float camera_pan_pwm)
{
    static float filtered_x = 0.5f;
    static float filtered_y = 0.5f;
    static uint32_t last_detect_tick;
    static uint32_t last_update_tick;
    uint32_t now = HAL_GetTick();

    if (is_detected != 0U)
    {
        last_detect_tick = now;
    }

    /* 按真实时间限制云台速度，避免速度随控制线程频率变化。 */
    if ((now - last_update_tick) < GIMBAL_UPDATE_PERIOD_MS)
    {
        return;
    }
    last_update_tick = now;

    if (is_detected != 0U)
    {
        float camera_pan_offset;
        float image_pan_offset;
        float target_pwm_pan;
        float target_pwm_tilt;

        if (fabsf(target_x - filtered_x) > 0.05f)
        {
            filtered_x = filtered_x * 0.90f + target_x * 0.10f;
        }
        if (fabsf(target_y - filtered_y) > 0.05f)
        {
            filtered_y = filtered_y * 0.90f + target_y * 0.10f;
        }

        /* 先计算图像误差，再叠加摄像头相对车体的偏航补偿。 */
        camera_pan_offset =
            (camera_pan_pwm - CAMERA_PAN_CENTER_PWM) /
            CAMERA_PAN_HALF_RANGE_PWM;
        camera_pan_offset = Servo_Clamp(camera_pan_offset, -1.0f, 1.0f);
        image_pan_offset =
            (0.5f - filtered_x) * (SERVO_PAN_MAX - SERVO_PAN_MIN);

        target_pwm_pan =
            GIMBAL_PAN_CENTER_PWM +
            image_pan_offset +
            CAMERA_TO_GIMBAL_PAN_SIGN * camera_pan_offset *
            GIMBAL_PAN_HALF_RANGE_PWM;
        target_pwm_pan = Servo_Clamp(target_pwm_pan,
                                     SERVO_PAN_MIN, SERVO_PAN_MAX);

        target_pwm_tilt =
            SERVO_TILT_MAX -
            filtered_y * (SERVO_TILT_MAX - SERVO_TILT_MIN);
        target_pwm_tilt = Servo_Clamp(target_pwm_tilt,
                                      SERVO_TILT_MIN, SERVO_TILT_MAX);

        current_pwm_pan =
            Servo_Smooth_Step(current_pwm_pan, target_pwm_pan, 0.06f);
        current_pwm_tilt =
            Servo_Smooth_Step(current_pwm_tilt, target_pwm_tilt, 0.06f);
    }
    else if ((last_detect_tick == 0U) ||
             ((now - last_detect_tick) >
              GIMBAL_DETECTION_HOLD_MS))
    {
        /* 短暂丢检时保持位置，超时后再回中。 */
        current_pwm_pan =
            Servo_Smooth_Step(current_pwm_pan, GIMBAL_PAN_CENTER_PWM, 0.05f);
        current_pwm_tilt =
            Servo_Smooth_Step(current_pwm_tilt, 1500.0f, 0.05f);
        filtered_x = 0.5f;
        filtered_y = 0.5f;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,
                          (uint32_t)current_pwm_pan);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,
                          (uint32_t)current_pwm_tilt);
}

void Laser_Fire(uint8_t state)
{
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN,
                      (state != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
