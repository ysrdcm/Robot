#ifndef __ESP8266_H
#define __ESP8266_H

#include "main.h"

// 声明外部的 UART4 句柄
extern UART_HandleTypeDef huart4; 

uint8_t ESP8266_AP_Server_Init(void);
uint8_t ESP8266_Send_Cmd(char *cmd, char *ack, uint32_t timeout);
void ESP8266_Send_HTTP_Image(uint8_t link_id, uint8_t *img_buf, uint32_t img_size);

#endif
