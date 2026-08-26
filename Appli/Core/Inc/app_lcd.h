/**
 ****************************************************************************************************
 * @file        app_lcd.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_lcd.h文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */

#ifndef __APP_LCD_H
#define __APP_LCD_H

#include "stm32n6xx_hal.h"
#include "stm32_lcd.h"
#include "stm32_lcd_ex.h"

void app_lcd_init(void);
uint8_t *app_lcd_get_bg_buffer(void);
void app_lcd_switch_bg_buffer(void);
void app_lcd_draw_area_update(void);
void app_lcd_draw_area_commit(void);

#endif
