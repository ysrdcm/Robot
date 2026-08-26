#ifndef __tracker_H
#define __tracker_H

#include "main.h"

void Gimbal_Targeting_Update(float target_x, float target_y,
                             uint8_t is_detected, float camera_pan_pwm);
void Laser_Fire(uint8_t state);

#endif
