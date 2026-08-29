#ifndef __OV5640_DCMIPP_H
#define __OV5640_DCMIPP_H

#include "main.h"

/* 变量导出 */
extern uint8_t ov5640_dcmipp_buf[2 * 1024 * 1024] __attribute__((aligned(16))) __attribute__((section(".EXTRAM")));

void ov5640_dcmipp_init(void);  /* 初始化OV5640 DCMIPP */
void ov5640_dcmipp_start(void); /* 启动OV5640 DCMIPP传输 */
void ov5640_dcmipp_stop(void);  /* 停止OV5640 DCMIPP传输 */

#endif
