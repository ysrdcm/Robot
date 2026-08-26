/**
 * @file beep.c
 * @brief Buzzer GPIO interface.
 * @license Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 */

#include "beep.h"

void beep_init(void)
{
    BEEP(0);
}
