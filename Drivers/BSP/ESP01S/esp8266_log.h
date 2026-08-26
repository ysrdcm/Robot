#ifndef __ESP8266_LOG_H
#define __ESP8266_LOG_H

#include "main.h"

extern UART_HandleTypeDef huart4;

/* ESP-01S AP server interface used by the event log. */
uint8_t ESP8266_Log_Server_Init(void);
uint8_t ESP8266_Log_Send_Response(uint8_t link_id,
                                  const char *content_type,
                                  const uint8_t *body,
                                  uint32_t body_size);

/* Interrupt-driven UART4 receive interface. */
void ESP8266_Log_UART_Start(void);
void ESP8266_Log_UART_Flush(void);
uint8_t ESP8266_Log_UART_ReadByte(uint8_t *byte, uint32_t timeout_ms);
void ESP8266_Log_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void ESP8266_Log_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
