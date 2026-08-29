#ifndef __RC100_H
#define __RC100_H

#include "main.h"

/* ROBOTIS RC-100B 按键位图。 */
#define RC100_BUTTON_UP       0x0001U
#define RC100_BUTTON_DOWN     0x0002U
#define RC100_BUTTON_LEFT     0x0004U
#define RC100_BUTTON_RIGHT    0x0008U
#define RC100_BUTTON_1        0x0010U
#define RC100_BUTTON_2        0x0020U
#define RC100_BUTTON_3        0x0040U
#define RC100_BUTTON_4        0x0080U
#define RC100_BUTTON_5        0x0100U
#define RC100_BUTTON_6        0x0200U

typedef struct
{
    uint16_t buttons;
    uint32_t last_packet_tick;
    uint32_t packet_count;
    uint32_t byte_count;
    uint32_t invalid_frame_count;
    uint32_t uart_error_count;
    uint32_t receive_start_failure_count;
    uint8_t has_packet;
} rc100_state_t;

void RC100_Init(void);
void RC100_UART_IRQHandler(void);
void RC100_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void RC100_UART_ErrorCallback(UART_HandleTypeDef *huart);
void RC100_GetState(rc100_state_t *state);

#endif
