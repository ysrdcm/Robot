#include "car.h"
#include "motor.h"

#define BASE_SPEED 749

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

void Car_TurnLeft(void)
{
    Motor_SetSpeed(1, -BASE_SPEED);
    Motor_SetSpeed(4, -BASE_SPEED);
    Motor_SetSpeed(2, BASE_SPEED);
    Motor_SetSpeed(3, BASE_SPEED);
}

void Car_TurnRight(void)
{
    Motor_SetSpeed(1, BASE_SPEED);
    Motor_SetSpeed(4, BASE_SPEED);
    Motor_SetSpeed(2, -BASE_SPEED);
    Motor_SetSpeed(3, -BASE_SPEED);
}

void Car_Stop(void)
{
    Motor_SetSpeed(1, 0);
    Motor_SetSpeed(2, 0);
    Motor_SetSpeed(3, 0);
    Motor_SetSpeed(4, 0);
}

void Car_SlightTurnLeft(void)
{
    Motor_SetSpeed(1, 0);
    Motor_SetSpeed(2, BASE_SPEED);
    Motor_SetSpeed(4, 0);
    Motor_SetSpeed(3, BASE_SPEED);
}

void Car_SlightTurnRight(void)
{
    Motor_SetSpeed(1, BASE_SPEED);
    Motor_SetSpeed(2, 0);
    Motor_SetSpeed(4, BASE_SPEED);
    Motor_SetSpeed(3, 0);
}
