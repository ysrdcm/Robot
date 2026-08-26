#include "esp8266_log.h"

#include <stdio.h>
#include <string.h>

#include "tx_api.h"

#define ESP8266_RX_BUFFER_SIZE 256U
#define ESP8266_TX_CHUNK_SIZE  2048U
#define ESP8266_UART_RING_SIZE 2048U
#define ESP8266_UART_RING_MASK (ESP8266_UART_RING_SIZE - 1U)
#define ESP8266_PENDING_RING_SIZE 8192U
#define ESP8266_PENDING_RING_MASK (ESP8266_PENDING_RING_SIZE - 1U)
#define ESP8266_IPD_HEADER_MAX 96U
#define ESP8266_SEND_RETRY_COUNT  3U
#define ESP8266_SEND_RETRY_TICKS  5U
#define ESP8266_SEND_SETTLE_TICKS 2U

/* Retained for timeout and field diagnostics. */
static char esp_last_response[ESP8266_RX_BUFFER_SIZE];

/* UART ISR ring decouples incoming bytes from the Wi-Fi worker thread. */
static uint8_t esp_uart_ring[ESP8266_UART_RING_SIZE];
static uint8_t esp_uart_it_byte;
static volatile uint16_t esp_uart_head;
static volatile uint16_t esp_uart_tail;
static volatile uint32_t esp_uart_overflow_count;

/* Complete unsolicited +IPD frames are separated from synchronous AT replies. */
static uint8_t esp_pending_ring[ESP8266_PENDING_RING_SIZE];
static volatile uint16_t esp_pending_head;
static volatile uint16_t esp_pending_tail;
static uint8_t esp_ipd_prefix_index;
static uint8_t esp_ipd_capture_active;
static uint8_t esp_ipd_length_field;
static uint8_t esp_ipd_header_length;
static uint32_t esp_ipd_payload_length;
static uint32_t esp_ipd_payload_remaining;

static void esp_ipd_capture_reset(void)
{
    esp_ipd_prefix_index = 0U;
    esp_ipd_capture_active = 0U;
    esp_ipd_length_field = 0U;
    esp_ipd_header_length = 0U;
    esp_ipd_payload_length = 0U;
    esp_ipd_payload_remaining = 0U;
}

static void esp_pending_push(uint8_t data)
{
    uint16_t next = (uint16_t)((esp_pending_head + 1U) & ESP8266_PENDING_RING_MASK);

    if (next == esp_pending_tail)
    {
        esp_pending_tail = (uint16_t)((esp_pending_tail + 1U) & ESP8266_PENDING_RING_MASK);
    }
    esp_pending_ring[esp_pending_head] = data;
    esp_pending_head = next;
}

static uint8_t esp_pending_read(uint8_t *data)
{
    uint16_t tail = esp_pending_tail;

    if (tail == esp_pending_head)
    {
        return 1U;
    }
    *data = esp_pending_ring[tail];
    esp_pending_tail = (uint16_t)((tail + 1U) & ESP8266_PENDING_RING_MASK);
    return 0U;
}

static uint8_t esp_uart_read_raw(uint8_t *byte, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        uint16_t tail = esp_uart_tail;

        if (tail != esp_uart_head)
        {
            *byte = esp_uart_ring[tail];
            esp_uart_tail = (uint16_t)((tail + 1U) & ESP8266_UART_RING_MASK);
            return 0U;
        }
        tx_thread_sleep(1);
    }
    return 1U;
}

static uint8_t esp_capture_ipd_byte(uint8_t data)
{
    static const uint8_t prefix[] = "+IPD,";

    if (esp_ipd_capture_active != 0U)
    {
        esp_pending_push(data);

        if (esp_ipd_payload_remaining != 0U)
        {
            esp_ipd_payload_remaining--;
            if (esp_ipd_payload_remaining == 0U)
            {
                esp_ipd_capture_reset();
            }
            return 1U;
        }

        esp_ipd_header_length++;
        if (esp_ipd_header_length > ESP8266_IPD_HEADER_MAX)
        {
            esp_ipd_capture_reset();
            return 1U;
        }

        if (esp_ipd_length_field == 1U)
        {
            if ((data >= (uint8_t)'0') && (data <= (uint8_t)'9'))
            {
                if (esp_ipd_payload_length < 1000000U)
                {
                    esp_ipd_payload_length =
                        (esp_ipd_payload_length * 10U) + (uint32_t)(data - (uint8_t)'0');
                }
            }
            else if (data == (uint8_t)':')
            {
                esp_ipd_payload_remaining = esp_ipd_payload_length;
                if (esp_ipd_payload_remaining == 0U)
                {
                    esp_ipd_capture_reset();
                }
            }
            else if (data == (uint8_t)',')
            {
                /* Skip optional CIPDINFO address fields until the payload colon. */
                esp_ipd_length_field = 2U;
            }
        }
        else if ((esp_ipd_length_field == 0U) && (data == (uint8_t)','))
        {
            esp_ipd_length_field = 1U;
        }
        else if ((esp_ipd_length_field == 2U) && (data == (uint8_t)':'))
        {
            esp_ipd_payload_remaining = esp_ipd_payload_length;
            if (esp_ipd_payload_remaining == 0U)
            {
                esp_ipd_capture_reset();
            }
        }
        return 1U;
    }

    if (data == prefix[esp_ipd_prefix_index])
    {
        esp_ipd_prefix_index++;
        if (esp_ipd_prefix_index == (sizeof(prefix) - 1U))
        {
            uint32_t index;

            for (index = 0U; index < (sizeof(prefix) - 1U); index++)
            {
                esp_pending_push(prefix[index]);
            }
            esp_ipd_capture_active = 1U;
            esp_ipd_prefix_index = 0U;
            esp_ipd_length_field = 0U;
            esp_ipd_header_length = 0U;
            esp_ipd_payload_length = 0U;
            esp_ipd_payload_remaining = 0U;
            return 1U;
        }
    }
    else
    {
        esp_ipd_prefix_index = (data == prefix[0]) ? 1U : 0U;
    }
    return 0U;
}

void ESP8266_Log_UART_Start(void)
{
    esp_uart_head = 0U;
    esp_uart_tail = 0U;
    esp_uart_overflow_count = 0U;
    esp_pending_head = 0U;
    esp_pending_tail = 0U;
    esp_ipd_capture_reset();
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    (void)HAL_UART_Receive_IT(&huart4, &esp_uart_it_byte, 1U);
}

void ESP8266_Log_UART_Flush(void)
{
    esp_uart_tail = esp_uart_head;
    esp_pending_tail = esp_pending_head;
    esp_ipd_capture_reset();
}

uint8_t ESP8266_Log_UART_ReadByte(uint8_t *byte, uint32_t timeout_ms)
{
    if ((byte == NULL) || (timeout_ms == 0U))
    {
        return 1U;
    }
    if (esp_pending_read(byte) == 0U)
    {
        return 0U;
    }
    return esp_uart_read_raw(byte, timeout_ms);
}

void ESP8266_Log_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t head;
    uint16_t next;

    if (huart->Instance != UART4)
    {
        return;
    }

    head = esp_uart_head;
    next = (uint16_t)((head + 1U) & ESP8266_UART_RING_MASK);
    if (next != esp_uart_tail)
    {
        esp_uart_ring[head] = esp_uart_it_byte;
        esp_uart_head = next;
    }
    else
    {
        esp_uart_overflow_count++;
    }

    (void)HAL_UART_Receive_IT(&huart4, &esp_uart_it_byte, 1U);
}

void ESP8266_Log_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART4)
    {
        return;
    }

    /* Clear line errors before restarting one-byte interrupt reception. */
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(&huart4, &esp_uart_it_byte, 1U);
}

static void esp_store_response(const char *response)
{
    strncpy(esp_last_response, response, sizeof(esp_last_response) - 1U);
    esp_last_response[sizeof(esp_last_response) - 1U] = '\0';
}

static uint8_t esp_wait_response(const char *ack, uint32_t timeout_ms)
{
    char rx_buffer[ESP8266_RX_BUFFER_SIZE] = {0};
    uint32_t start_time = HAL_GetTick();
    uint16_t rx_idx = 0;

    while ((HAL_GetTick() - start_time) < timeout_ms)
    {
        uint8_t ch;

        if (esp_uart_read_raw(&ch, 10U) == 0U)
        {
            /* Divert +IPD data so it cannot be mistaken for an AT response. */
            if (esp_capture_ipd_byte(ch) != 0U)
            {
                continue;
            }

            if (rx_idx >= (sizeof(rx_buffer) - 1U))
            {
                memmove(rx_buffer, &rx_buffer[sizeof(rx_buffer) / 2U], sizeof(rx_buffer) / 2U);
                rx_idx = sizeof(rx_buffer) / 2U;
            }

            rx_buffer[rx_idx++] = (char)ch;
            rx_buffer[rx_idx] = '\0';
            if (strstr(rx_buffer, ack) != NULL)
            {
                esp_store_response(rx_buffer);
                return 0;
            }
            if ((strcmp(ack, "CLOSE_ACK") == 0) &&
                ((strstr(rx_buffer, "OK") != NULL) ||
                 (strstr(rx_buffer, "ERROR") != NULL) ||
                 (strstr(rx_buffer, "CLOSED") != NULL) ||
                 (strstr(rx_buffer, "link is not valid") != NULL)))
            {
                esp_store_response(rx_buffer);
                return 0;
            }
            if ((strcmp(ack, ">") == 0) && (strstr(rx_buffer, "busy") != NULL))
            {
                /* Return a distinct status so CIPSEND can retry after busy. */
                esp_store_response(rx_buffer);
                return 2U;
            }
            if ((strcmp(ack, "OK") == 0) && (strstr(rx_buffer, "no change") != NULL))
            {
                /* Older firmware reports an already-applied setting as success. */
                esp_store_response(rx_buffer);
                return 0;
            }
            if ((strstr(rx_buffer, "ERROR") != NULL) || (strstr(rx_buffer, "FAIL") != NULL))
            {
                esp_store_response(rx_buffer);
                return 1;
            }

            continue;
        }

        tx_thread_sleep(1);
    }

    esp_store_response(rx_buffer);
    return 1;
}

static uint8_t esp_send_cmd(const char *cmd, const char *ack, uint32_t timeout_ms)
{
    uint8_t status;

    if (HAL_UART_Transmit(&huart4, (uint8_t *)cmd, strlen(cmd), 1000) != HAL_OK)
    {
        printf("ESP UART transmit failed: %.*s\r\n", (int)strcspn(cmd, "\r\n"), cmd);
        return 1;
    }

    status = esp_wait_response(ack, timeout_ms);
    if (status != 0U)
    {
        printf("ESP AT failed: %.*s | response: %s\r\n",
               (int)strcspn(cmd, "\r\n"), cmd,
               (esp_last_response[0] != '\0') ? esp_last_response : "<timeout>");
    }
    return status;
}

uint8_t ESP8266_Log_Server_Init(void)
{
    uint8_t attempt;

    /* Synchronize with AT firmware after the external hardware reset. */
    for (attempt = 0; attempt < 5U; attempt++)
    {
        if (esp_send_cmd("AT\r\n", "OK", 1000) == 0U)
        {
            break;
        }
        tx_thread_sleep(200);
    }
    if (attempt == 5U) return 1;
    if (esp_send_cmd("ATE0\r\n", "OK", 1000) != 0) return 1;
    /* Support AT firmware variants that only expose the _CUR commands. */
    if ((esp_send_cmd("AT+CWMODE=2\r\n", "OK", 1000) != 0) &&
        (esp_send_cmd("AT+CWMODE_CUR=2\r\n", "OK", 1000) != 0)) return 2;
    if ((esp_send_cmd("AT+CWSAP=\"Robot_Cam\",\"12345678\",1,3\r\n", "OK", 3000) != 0) &&
        (esp_send_cmd("AT+CWSAP_CUR=\"Robot_Cam\",\"12345678\",1,3\r\n", "OK", 3000) != 0)) return 3;
    if (esp_send_cmd("AT+CIPMUX=1\r\n", "OK", 1000) != 0) return 4;
    if (esp_send_cmd("AT+CIPSERVER=1,80\r\n", "OK", 1000) != 0) return 5;

    /* CIPSTO is optional and unsupported by some AT firmware versions. */
    (void)esp_send_cmd("AT+CIPSTO=30\r\n", "OK", 1000);

    return 0;
}

static uint8_t esp_send_block(uint8_t link_id, const uint8_t *data, uint32_t size)
{
    char cmd[40];
    uint8_t attempt;
    uint8_t status = 1U;
    int cmd_len = snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%lu\r\n",
                           link_id, (unsigned long)size);

    if ((cmd_len <= 0) || ((size_t)cmd_len >= sizeof(cmd)))
    {
        return 1;
    }

    /* Consecutive blocks can hit a short ESP8266 busy window. */
    for (attempt = 0U; attempt < ESP8266_SEND_RETRY_COUNT; attempt++)
    {
        if (HAL_UART_Transmit(&huart4, (uint8_t *)cmd, (uint16_t)cmd_len, 1000U) != HAL_OK)
        {
            status = 1U;
            break;
        }

        status = esp_wait_response(">", 2000U);
        if (status == 0U)
        {
            break;
        }
        if (status != 2U)
        {
            break;
        }
        tx_thread_sleep(ESP8266_SEND_RETRY_TICKS);
    }
    if (status != 0U)
    {
        printf("ESP AT failed after retries: %.*s | response: %s\r\n",
               (int)strcspn(cmd, "\r\n"), cmd,
               (esp_last_response[0] != '\0') ? esp_last_response : "<timeout>");
        return 1U;
    }

    if (HAL_UART_Transmit(&huart4, (uint8_t *)data, size, 3000) != HAL_OK)
    {
        return 1;
    }

    status = esp_wait_response("SEND OK", 5000U);
    if (status == 0U)
    {
        /* Allow AT firmware to finish the send state before the next command. */
        tx_thread_sleep(ESP8266_SEND_SETTLE_TICKS);
    }
    return status;
}

uint8_t ESP8266_Log_Send_Response(uint8_t link_id,
                                  const char *content_type,
                                  const uint8_t *body,
                                  uint32_t body_size)
{
    char header[192];
    char close_cmd[32];
    uint32_t sent = 0;
    uint8_t status = 0;
    int header_len;

    header_len = snprintf(header, sizeof(header),
                          "HTTP/1.1 200 OK\r\n"
                          "Content-Type: %s\r\n"
                          "Content-Length: %lu\r\n"
                          "Cache-Control: no-store\r\n"
                          "Connection: close\r\n\r\n",
                          content_type, (unsigned long)body_size);
    if ((header_len <= 0) || ((size_t)header_len >= sizeof(header)))
    {
        return 1;
    }

    if (esp_send_block(link_id, (const uint8_t *)header, (uint32_t)header_len) != 0)
    {
        status = 1;
    }

    while ((status == 0) && (sent < body_size))
    {
        uint32_t remaining = body_size - sent;
        uint32_t chunk = (remaining > ESP8266_TX_CHUNK_SIZE) ? ESP8266_TX_CHUNK_SIZE : remaining;

        if (esp_send_block(link_id, &body[sent], chunk) != 0)
        {
            status = 1;
            break;
        }
        sent += chunk;
    }

    (void)snprintf(close_cmd, sizeof(close_cmd), "AT+CIPCLOSE=%u\r\n", link_id);
    (void)esp_send_cmd(close_cmd, "CLOSE_ACK", 1000);

    if (status != 0U)
    {
        printf("ESP HTTP response failed on link %u\r\n", link_id);
    }
    return status;
}
