#ifndef __WAVE_H
#define __WAVE_H

#include "main.h"

void Wave_Init(void);
float Wave_Get_Distance(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif
