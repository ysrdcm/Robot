/**
 ****************************************************************************************************
 * @file        ov5640.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       OV5640 驱动代码
 * @license     Copyright (c) 2023-2035, 广州市星翼电子科技有限公司
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

#ifndef __OV5640_H
#define __OV5640_H

#include "main.h"
#include "ov5640_dcmipp.h"

/* 引脚定义 */
#define OV_RESET_GPIO_PORT  GPIOQ
#define OV_RESET_GPIO_PIN   GPIO_PIN_2
#define OV_PWDN_GPIO_PORT   GPIOG
#define OV_PWDN_GPIO_PIN    GPIO_PIN_14

/* IO操作 */
#define OV5640_PWDN(x)      do { (x) ?                                                                      \
                                HAL_GPIO_WritePin(OV_PWDN_GPIO_PORT, OV_PWDN_GPIO_PIN, GPIO_PIN_SET):       \
                                HAL_GPIO_WritePin(OV_PWDN_GPIO_PORT, OV_PWDN_GPIO_PIN, GPIO_PIN_RESET);     \
                            } while (0)
#define OV5640_RST(x)       do { (x) ?                                                                      \
                                HAL_GPIO_WritePin(OV_RESET_GPIO_PORT, OV_RESET_GPIO_PIN, GPIO_PIN_SET):     \
                                HAL_GPIO_WritePin(OV_RESET_GPIO_PORT, OV_RESET_GPIO_PIN, GPIO_PIN_RESET);   \
                            } while (0)

/* OV5640 ID和SCCB访问地址定义 */
#define OV5640_ID                   0x5640
#define OV5640_ADDR                 0x3C

/* 函数声明 */
uint8_t ov5640_read_reg(uint16_t reg);                                                          /* 读OV5640寄存器 */
void ov5640_write_reg(uint16_t reg, uint8_t data);                                              /* 写OV5640寄存器 */
uint8_t ov5640_init(void);                                                                      /* 初始化OV5640 */
void ov5640_flash_ctrl(uint8_t sw);                                                             /* 控制OV5640闪光灯 */
void ov5640_rgb565_mode(void);                                                                  /* 配置OV5640 RGB565输出 */
void ov5640_jpeg_mode(void);                                                                    /* 配置OV5640 JPEG输出 */
void ov5640_light_mode(uint8_t mode);                                                           /* 设置OV5640灯光模式 */
void ov5640_exposure(uint8_t level);                                                            /* 设置OV5640曝光等级 */
void ov5640_brightness(uint8_t level);                                                          /* 设置OV5640亮度 */
void ov5640_color_saturation(uint8_t level);                                                    /* 设置OV5640色彩饱和度 */
void ov5640_contrast(uint8_t level);                                                            /* 设置OV5640对比度 */
void ov5640_sharpness(uint8_t level);                                                           /* 设置OV5640锐度 */
void ov5640_special_effects(uint8_t effects);                                                   /* 设置OV5640特效 */
void ov5640_test_pattern(uint8_t mode);                                                         /* OV5640测试 */
uint8_t ov5640_outsize_set(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height);      /* 设置OV5640输出图像大小 */
uint8_t ov5640_image_window_set(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height); /* 设置OV5640图像开窗大小 */
uint8_t ov5640_focus_init(void);                                                                /* 初始化OV5640自动对焦 */
uint8_t ov5640_focus_single(void);                                                              /* OV5640单次自动对焦 */
uint8_t ov5640_focus_constant(void);                                                            /* OV5640连续自动对焦 */

#endif
