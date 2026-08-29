/**
 * @file beep.h
 * @brief 蜂鸣器 GPIO 接口。
 */

#ifndef __BEEP_H
#define __BEEP_H

#include "main.h"

#define BEEP_GPIO_PORT  GPIOD
#define BEEP_GPIO_PIN   GPIO_PIN_3

#define BEEP(x)         do { (x) ?                                                              \
                            HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_SET):     \
                            HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET);   \
                        } while (0)

void beep_init(void);

#endif
