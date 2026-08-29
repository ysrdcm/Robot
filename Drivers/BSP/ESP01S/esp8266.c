#include "esp8266.h"
#include "string.h"
#include <stdio.h>
#include "tx_api.h"

uint8_t ESP8266_Send_Cmd(const char *cmd, const char *ack, uint32_t timeout)
{
    uint8_t rx_buffer[128] = {0};
    uint32_t start_time;
    uint16_t rx_idx = 0U;

    if ((cmd == NULL) || (ack == NULL))
    {
        return 1U;
    }
    if (HAL_UART_Transmit(&huart4, (uint8_t *)cmd,
                          (uint16_t)strlen(cmd), 100U) != HAL_OK)
    {
        return 1U;
    }

    start_time = HAL_GetTick();
    
    while ((HAL_GetTick() - start_time) < timeout)
    {
        uint8_t ch;
        if (HAL_UART_Receive(&huart4, &ch, 1, 10) == HAL_OK)
        {
            if (rx_idx >= (sizeof(rx_buffer) - 1U))
            {
                memmove(rx_buffer, &rx_buffer[sizeof(rx_buffer) / 2U],
                        sizeof(rx_buffer) / 2U);
                rx_idx = sizeof(rx_buffer) / 2U;
            }
            rx_buffer[rx_idx++] = ch;
            rx_buffer[rx_idx] = '\0';

            if (strstr((char *)rx_buffer, ack) != NULL)
            {
                return 0;
            }
        }
        tx_thread_sleep(1);
    }
    return 1;
}

uint8_t ESP8266_AP_Server_Init(void)
{
    ESP8266_Send_Cmd("AT+RST\r\n", "ready", 2000); 
    
    if (ESP8266_Send_Cmd("AT+CWMODE=2\r\n", "OK", 1000) != 0) return 1;
    
    if (ESP8266_Send_Cmd("AT+CWSAP=\"Robot_Cam\",\"12345678\",1,3\r\n", "OK", 3000) != 0) return 2;
    
    if (ESP8266_Send_Cmd("AT+CIPMUX=1\r\n", "OK", 1000) != 0) return 3;
    
    if (ESP8266_Send_Cmd("AT+CIPSERVER=1,80\r\n", "OK", 1000) != 0) return 4;
    
    return 0;
}

void ESP8266_Send_HTTP_Image(uint8_t link_id, const uint8_t *img_buf, uint32_t img_size)
{
    char header[] = "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nConnection: close\r\n\r\n";
    uint32_t header_len = strlen(header);
    char cmd[32];

    (void)snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%lu\r\n",
                   link_id, (unsigned long)header_len);
    if (ESP8266_Send_Cmd(cmd, ">", 1000) == 0) {
        HAL_UART_Transmit(&huart4, (uint8_t*)header, header_len, 1000);
    }

    /* 每个分块不超过 AT 固件的单次发送上限。 */
    uint32_t sent_len = 0;
    uint32_t chunk_size = 2048;
    while (sent_len < img_size) 
    {
        uint32_t remain = img_size - sent_len;
        uint32_t send_now = (remain > chunk_size) ? chunk_size : remain;

        (void)snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%lu\r\n",
                       link_id, (unsigned long)send_now);
        if (ESP8266_Send_Cmd(cmd, ">", 1000) == 0) {
            HAL_UART_Transmit(&huart4, &img_buf[sent_len], send_now, 1000);
            sent_len += send_now;
        } else {
            break;
        }
    }

    (void)snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%u\r\n", link_id);
    ESP8266_Send_Cmd(cmd, "OK", 1000);
}
