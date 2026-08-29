#ifndef __ESP8266_H
#define __ESP8266_H

#include "main.h"

extern UART_HandleTypeDef huart4; 

uint8_t ESP8266_AP_Server_Init(void);
uint8_t ESP8266_Send_Cmd(const char *cmd, const char *ack, uint32_t timeout);
void ESP8266_Send_HTTP_Image(uint8_t link_id, const uint8_t *img_buf, uint32_t img_size);

#endif
