#ifndef __UART_H
#define __UART_H

#include "main.h"
#include <stdio.h>

#define UART_UX         USART1

#ifndef BSP_UART_RX_DISABLE
#define UART_EN_RX      1                   /* 使能串口接收功能 */
#else
#define UART_EN_RX      0                   /* 禁用串口接收功能 */
#endif
#define RXBUFFERSIZE    1                   /* 串口中断接收缓冲区大小 */
#define UART_REC_LEN    200                 /* 串口接收缓冲区大小 */

#if UART_EN_RX
extern uint8_t g_rx_buffer[RXBUFFERSIZE];   /* 串口中断接收缓冲区 */
extern uint8_t g_uart_rx_buf[UART_REC_LEN]; /* 串口接收缓冲区 */
extern uint16_t g_uart_rx_sta;              /* 串口接收状态标记 */
#endif

void uart_stdio_init(void);

#endif
