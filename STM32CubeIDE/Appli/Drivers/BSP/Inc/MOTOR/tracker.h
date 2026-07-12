#ifndef __tracker_H
#define __tracker_H

#include "main.h"

//void Laser_Targeting_Update(float target_x, float target_y, uint8_t is_detected);
void Gimbal_Targeting_Update(float target_x, float target_y, uint8_t is_detected);
void Laser_Fire(uint8_t state);

#endif
