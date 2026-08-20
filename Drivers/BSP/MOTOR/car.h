#ifndef __CAR_H
#define __CAR_H

#include "main.h"

// 声明小车的运动控制函数，供 main.c 或 AI 业务逻辑调用
void Car_Forward(void);
void Car_Backward(void);
void Car_TurnLeft(void);
void Car_TurnRight(void);
void Car_Stop(void);

void Car_SlightTurnLeft(void);
void Car_SlightTurnRight(void);

#endif
