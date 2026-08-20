/**
 ****************************************************************************************************
 * @file        ov5640_dcmipp.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       OV5640 DCMIPP驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 N647开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#ifndef __OV5640_DCMIPP_H
#define __OV5640_DCMIPP_H

#include "main.h"

/* 变量导出 */
extern uint8_t ov5640_dcmipp_buf[2 * 1024 * 1024] __attribute__((aligned(16))) __attribute__((section(".EXTRAM")));

/* 函数声明 */
void ov5640_dcmipp_init(void);  /* 初始化OV5640 DCMIPP */
void ov5640_dcmipp_start(void); /* 启动OV5640 DCMIPP传输 */
void ov5640_dcmipp_stop(void);  /* 停止OV5640 DCMIPP传输 */

#endif
