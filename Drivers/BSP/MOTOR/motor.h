#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h" // 引入 STM32N6 的 HAL 库和 CubeMX 生成的引脚宏

/**
 * @brief  单独设置某个电机的速度和方向
 * @param  motor_id: 1(左前), 2(右前), 3(左后), 4(右后)
 * @param  speed: -999 ~ 999 (负数代表反转，正数代表正转，0为停止)
 */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed);

#endif
