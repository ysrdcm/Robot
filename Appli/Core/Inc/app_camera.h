/**
 ****************************************************************************************************
 * @file        app_camera.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_camera.h文件
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

#ifndef __APP_CAMERA_H
#define __APP_CAMERA_H

#include "stm32n6xx_hal.h"

void app_camera_init(void (*display_pipe_vsync_cb)(void), void (*display_pipe_frame_cb)(void), void (*nn_pipe_vsync_cb)(void), void (*nn_pipe_frame_cb)(void));
void app_camera_display_pipe_start(uint8_t *display_pipe_destination, uint32_t capture_mode);
void app_camera_display_pipe_set_address(uint8_t *display_pipe_destination);
void app_camera_nn_pipe_start(uint8_t *nn_pipe_destination, uint32_t capture_mode);
void app_camera_nn_pipe_set_address(uint8_t *nn_pipe_destination);
void app_camera_isp_update(void);

#endif
