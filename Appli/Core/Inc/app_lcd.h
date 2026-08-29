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
