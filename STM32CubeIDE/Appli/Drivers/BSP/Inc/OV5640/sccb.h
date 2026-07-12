/**
 ****************************************************************************************************
 * @file        sccb.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       SCCB 驱动代码
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

#ifndef __SCCB_H
#define __SCCB_H

#include "main.h"

/* 引脚定义 */
#define SCCB_SCL_GPIO_PORT  GPIOE
#define SCCB_SCL_GPIO_PIN   GPIO_PIN_13
#define SCCB_SDA_GPIO_PORT  GPIOE
#define SCCB_SDA_GPIO_PIN   GPIO_PIN_14

/* IO操作 */
#define SCCB_SCL(x)         do { (x) ?                                                                      \
                                HAL_GPIO_WritePin(SCCB_SCL_GPIO_PORT, SCCB_SCL_GPIO_PIN, GPIO_PIN_SET):     \
                                HAL_GPIO_WritePin(SCCB_SCL_GPIO_PORT, SCCB_SCL_GPIO_PIN, GPIO_PIN_RESET);   \
                            } while (0)
#define SCCB_SDA(x)         do { (x) ?                                                                      \
                                HAL_GPIO_WritePin(SCCB_SDA_GPIO_PORT, SCCB_SDA_GPIO_PIN, GPIO_PIN_SET):     \
                                HAL_GPIO_WritePin(SCCB_SDA_GPIO_PORT, SCCB_SDA_GPIO_PIN, GPIO_PIN_RESET);   \
                            } while (0)
#define SCCB_SDA_READ       ((HAL_GPIO_ReadPin(SCCB_SDA_GPIO_PORT, SCCB_SDA_GPIO_PIN) == GPIO_PIN_RESET) ? 0 : 1)

/* 函数声明 */
void sccb_init(void);                   /* 初始化SCCB */
void sccb_start(void);                  /* 产生SCCB起始信号 */
void sccb_stop(void);                   /* 产生SCCB停止信号 */
void sccb_nack(void);                   /* 产生SCCB NACK信号 */
uint8_t sccb_send_byte(uint8_t data);   /* SCCB发送一个字节 */
uint8_t sccb_read_byte(void);           /* SCCB读取一个字节 */

#endif
