#include "uart.h"
#include "esp8266_log.h"
#include "rc100.h"

extern UART_HandleTypeDef huart1;

#if UART_EN_RX
/* 串口中断接收缓冲区 */
uint8_t g_rx_buffer[RXBUFFERSIZE];
/* 串口接收缓冲区 */
uint8_t g_uart_rx_buf[UART_REC_LEN];
/* 串口接收状态标记 */
uint16_t g_uart_rx_sta = 0;
#endif

void uart_stdio_init(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

#if UART_EN_RX
    /* 可选控制台模式使用 HAL 单字节中断接收。 */
    HAL_UART_Receive_IT(&huart1, g_rx_buffer, sizeof(g_rx_buffer));
#endif
}

#if UART_EN_RX
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        ESP8266_Log_UART_RxCpltCallback(huart);
        return;
    }

    if (huart->Instance == UART_UX)
    {
        if((g_uart_rx_sta & 0x8000) == 0)
        {
            if(g_uart_rx_sta & 0x4000)
            {
                if(g_rx_buffer[0] != 0x0A)
                {
                    g_uart_rx_sta = 0;
                }
                else
                {
                    g_uart_rx_sta |= 0x8000;
                }
            }
            else
            {
                if(g_rx_buffer[0] == 0x0D)
                {
                    g_uart_rx_sta |= 0x4000;
                }
                else
                {
                    g_uart_rx_buf[g_uart_rx_sta & 0x3FFF] = g_rx_buffer[0];
                    g_uart_rx_sta++;
                    if(g_uart_rx_sta > (UART_REC_LEN - 1))
                    {
                        g_uart_rx_sta = 0;
                    }
                }
            }
        }

        HAL_UART_Receive_IT(&huart1, g_rx_buffer, sizeof(g_rx_buffer));
    }
}
#else
/* 禁用 USART1 控制台接收时，UART4 仍需分发 ESP-01S 接收回调。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RC100_UART_RxCpltCallback(huart);
    }
    else if (huart->Instance == UART4)
    {
        ESP8266_Log_UART_RxCpltCallback(huart);
    }
}
#endif

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
#if UART_EN_RX
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_Receive_IT(&huart1, g_rx_buffer,
                                  sizeof(g_rx_buffer));
#else
        RC100_UART_ErrorCallback(huart);
#endif
    }
    else if (huart->Instance == UART4)
    {
        ESP8266_Log_UART_ErrorCallback(huart);
    }
}

/* USART1 用于 BT-410 时禁止 printf 占用遥控链路。 */
int __io_putchar(int ch)
{
#if UART_EN_RX
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
#endif

    return ch;
}
