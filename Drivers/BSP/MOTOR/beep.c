/**
 * @file beep.c
 * @brief 蜂鸣器 GPIO 接口。
 */

#include "beep.h"

void beep_init(void)
{
    BEEP(0);
}
