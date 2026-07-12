/**
 ****************************************************************************************************
 * @file        app.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app.h文件
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

#ifndef __APP_H
#define __APP_H

#include "stm32n6xx_hal.h"

void app_run(void);
void app_dcmipp_pipe_vsync_cb(void);
void app_dcmipp_pipe_frame_cb(void);

#endif
