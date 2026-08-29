#ifndef __APP_CAMERA_H
#define __APP_CAMERA_H

#include "stm32n6xx_hal.h"

HAL_StatusTypeDef app_camera_init(void (*display_pipe_vsync_cb)(void), void (*display_pipe_frame_cb)(void), void (*nn_pipe_vsync_cb)(void), void (*nn_pipe_frame_cb)(void));
void app_camera_display_pipe_start(uint8_t *display_pipe_destination, uint32_t capture_mode);
void app_camera_display_pipe_set_address(uint8_t *display_pipe_destination);
void app_camera_nn_pipe_start(uint8_t *nn_pipe_destination, uint32_t capture_mode);
void app_camera_nn_pipe_set_address(uint8_t *nn_pipe_destination);
void app_camera_isp_update(void);
/* 在线程上下文中恢复停止响应的显示与推理管线。 */
uint8_t app_camera_recovery_requested(void);
HAL_StatusTypeDef app_camera_recover(void);

#endif
