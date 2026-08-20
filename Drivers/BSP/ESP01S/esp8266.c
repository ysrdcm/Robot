// esp8266.c 示例
#include "esp8266.h"
#include "string.h"
#include <stdio.h> /* CODEX 2026-07-20: Declare sprintf used by the legacy helper. */
#include "tx_api.h" // 需要用到 ThreadX 的延时

// 发送AT指令并等待特定应答的简单封装
uint8_t ESP8266_Send_Cmd(char *cmd, char *ack, uint32_t timeout)
{
    uint8_t rx_buffer[128] = {0};
    
    // 1. 发送指令
    HAL_UART_Transmit(&huart4, (uint8_t *)cmd, strlen(cmd), 100);
    
    // 2. 简易轮询接收等待应答 (实际工程中强烈建议改用 DMA + 串口空闲中断)
    uint32_t start_time = HAL_GetTick();
    uint16_t rx_idx = 0;
    
    while ((HAL_GetTick() - start_time) < timeout)
    {
        uint8_t ch;
        if (HAL_UART_Receive(&huart4, &ch, 1, 10) == HAL_OK)
        {
            rx_buffer[rx_idx++] = ch;
            if (rx_idx >= sizeof(rx_buffer)) rx_idx = 0;
            
            // 检查是否包含期望的应答字符串
            if (strstr((char *)rx_buffer, ack) != NULL)
            {
                return 0; // 成功
            }
        }
        tx_thread_sleep(1); // 防止死占 CPU
    }
    return 1; // 超时失败
}

uint8_t ESP8266_AP_Server_Init(void)
{
    // 重置模块以保证状态干净
    ESP8266_Send_Cmd("AT+RST\r\n", "ready", 2000); 
    
    // 设置为 AP 模式
    if (ESP8266_Send_Cmd("AT+CWMODE=2\r\n", "OK", 1000) != 0) return 1;
    
    // 配置热点 SSID 为 "Robot_Cam"，密码 "12345678"，通道 1，加密方式 3 (WPA2)
    if (ESP8266_Send_Cmd("AT+CWSAP=\"Robot_Cam\",\"12345678\",1,3\r\n", "OK", 3000) != 0) return 2;
    
    // 允许多链接 (建立服务器必须开启)
    if (ESP8266_Send_Cmd("AT+CIPMUX=1\r\n", "OK", 1000) != 0) return 3;
    
    // 开启 TCP 服务器，端口设置为 80 (HTTP 默认端口)
    if (ESP8266_Send_Cmd("AT+CIPSERVER=1,80\r\n", "OK", 1000) != 0) return 4;
    
    return 0; // 初始化成功
}

void ESP8266_Send_HTTP_Image(uint8_t link_id, uint8_t *img_buf, uint32_t img_size)
{
    // 1. 准备 HTTP 响应头
    char header[] = "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nConnection: close\r\n\r\n";
    uint32_t header_len = strlen(header);
    char cmd[32];

    // 2. 发送 HTTP 头部
    sprintf(cmd, "AT+CIPSEND=%d,%lu\r\n", link_id, header_len);
    if (ESP8266_Send_Cmd(cmd, ">", 1000) == 0) {
        HAL_UART_Transmit(&huart4, (uint8_t*)header, header_len, 1000);
    }

    // 3. 分包发送图片实体数据 (ESP8266 单包上限通常为 2048 字节)
    uint32_t sent_len = 0;
    uint32_t chunk_size = 2048;
    while (sent_len < img_size) 
    {
        uint32_t remain = img_size - sent_len;
        uint32_t send_now = (remain > chunk_size) ? chunk_size : remain;

        sprintf(cmd, "AT+CIPSEND=%d,%lu\r\n", link_id, send_now);
        if (ESP8266_Send_Cmd(cmd, ">", 1000) == 0) {
            HAL_UART_Transmit(&huart4, &img_buf[sent_len], send_now, 1000);
            sent_len += send_now;
        } else {
            break; // 发送出错，退出循环
        }
    }

    // 4. 断开此 TCP 连接，通知浏览器数据传输完毕
    sprintf(cmd, "AT+CIPCLOSE=%d\r\n", link_id);
    ESP8266_Send_Cmd(cmd, "OK", 1000);
}
