#include "car.h"
#include "motor.h"

// 假设基础速度为 500 (占空比 50%)
#define BASE_SPEED 680

void Car_Forward(void)
{
    Motor_SetSpeed(1, BASE_SPEED);
    Motor_SetSpeed(2, BASE_SPEED);
    Motor_SetSpeed(3, BASE_SPEED);
    Motor_SetSpeed(4, BASE_SPEED);
}

void Car_Backward(void)
{
    Motor_SetSpeed(1, -BASE_SPEED);
    Motor_SetSpeed(2, -BASE_SPEED);
    Motor_SetSpeed(3, -BASE_SPEED);
    Motor_SetSpeed(4, -BASE_SPEED);
}

// 差速左转：左边两个轮子(1,4)反转，右边两个轮子(2,3)正转
void Car_TurnLeft(void)
{
    Motor_SetSpeed(1, -BASE_SPEED); // 左前：倒转
    Motor_SetSpeed(4, -BASE_SPEED); // 左后：倒转
    Motor_SetSpeed(2, BASE_SPEED);  // 右前：正转
    Motor_SetSpeed(3, BASE_SPEED);  // 右后：正转
}

// 差速右转：左边两个轮子(1,4)正转，右边两个轮子(2,3)反转
void Car_TurnRight(void)
{
    Motor_SetSpeed(1, BASE_SPEED);  // 左前：正转
    Motor_SetSpeed(4, BASE_SPEED);  // 左后：正转
    Motor_SetSpeed(2, -BASE_SPEED); // 右前：倒转
    Motor_SetSpeed(3, -BASE_SPEED); // 右后：倒转
}

void Car_Stop(void)
{
    Motor_SetSpeed(1, 0);
    Motor_SetSpeed(2, 0);
    Motor_SetSpeed(3, 0);
    Motor_SetSpeed(4, 0);
}

// 微调左转：左边轮子停转（或低速），右边轮子正常转
void Car_SlightTurnLeft(void)
{
    Motor_SetSpeed(1, 0);             // 左前停
    Motor_SetSpeed(2, BASE_SPEED);    // 右前正常
    Motor_SetSpeed(4, 0);             // 左后停
    Motor_SetSpeed(3, BASE_SPEED);    // 右后正常
}

// 微调右转：左边轮子正常转，右边轮子停转（或低速）
void Car_SlightTurnRight(void)
{
    Motor_SetSpeed(1, BASE_SPEED);    // 左前正常
    Motor_SetSpeed(2, 0);             // 右前停
    Motor_SetSpeed(4, BASE_SPEED);    // 左后正常
    Motor_SetSpeed(3, 0);             // 右后停
}
