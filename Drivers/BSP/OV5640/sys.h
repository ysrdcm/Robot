#ifndef __SYS_H
#define __SYS_H

#include "main.h"

void sys_clock_config_debug(void);  /* 配置系统时钟 */
void sys_delay_us(uint32_t us);     /* 微秒级延时 */

#endif
