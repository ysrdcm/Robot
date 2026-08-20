#include "rc100.h"

extern UART_HandleTypeDef huart1;

/* CODEX 2026-07-24: USART1 receives the six-byte RC-100B packet at 57600 8N1. */
static uint8_t rc100_frame[6];
static uint8_t rc100_frame_index;
static volatile uint16_t rc100_buttons;
static volatile uint32_t rc100_last_packet_tick;
static volatile uint32_t rc100_packet_count;
static volatile uint32_t rc100_byte_count;
static volatile uint32_t rc100_invalid_frame_count;
static volatile uint32_t rc100_uart_error_count;
static volatile uint32_t rc100_receive_start_failure_count;
static volatile uint8_t rc100_has_packet;

static void rc100_parse_byte(uint8_t byte)
{
    if (rc100_frame_index == 0U)
    {
        if (byte == 0xFFU)
        {
            rc100_frame[0] = byte;
            rc100_frame_index = 1U;
        }
        return;
    }

    if (rc100_frame_index == 1U)
    {
        if (byte == 0x55U)
        {
            rc100_frame[1] = byte;
            rc100_frame_index = 2U;
        }
        else
        {
            rc100_frame_index = (byte == 0xFFU) ? 1U : 0U;
        }
        return;
    }

    rc100_frame[rc100_frame_index++] = byte;
    if (rc100_frame_index < sizeof(rc100_frame))
    {
        return;
    }

    rc100_frame_index = 0U;
    if ((((uint8_t)(rc100_frame[2] ^ rc100_frame[3])) == 0xFFU) &&
        (((uint8_t)(rc100_frame[4] ^ rc100_frame[5])) == 0xFFU))
    {
        rc100_buttons = (uint16_t)rc100_frame[2] |
                        ((uint16_t)rc100_frame[4] << 8);
        rc100_last_packet_tick = HAL_GetTick();
        rc100_packet_count++;
        rc100_has_packet = 1U;
    }
    else
    {
        rc100_invalid_frame_count++;
    }
}

void RC100_Init(void)
{
    rc100_frame_index = 0U;
    rc100_buttons = 0U;
    rc100_last_packet_tick = 0U;
    rc100_packet_count = 0U;
    rc100_byte_count = 0U;
    rc100_invalid_frame_count = 0U;
    rc100_uart_error_count = 0U;
    rc100_receive_start_failure_count = 0U;
    rc100_has_packet = 0U;

    /*
     * CODEX 2026-07-24: Receive directly from RXFNE instead of depending on
     * HAL's one-byte asynchronous state machine and callback dispatch.
     */
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXFNE);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXFNE);

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_REACK) == RESET)
    {
        rc100_receive_start_failure_count++;
    }
}

void RC100_UART_IRQHandler(void)
{
    uint32_t isr = READ_REG(huart1.Instance->ISR);
    uint32_t error_flags = isr & (USART_ISR_PE | USART_ISR_FE |
                                  USART_ISR_NE | USART_ISR_ORE);

    if (error_flags != 0U)
    {
        /* CODEX 2026-07-24: Clear line errors and restart packet framing. */
        rc100_uart_error_count++;
        WRITE_REG(huart1.Instance->ICR,
                  USART_ICR_PECF | USART_ICR_FECF |
                  USART_ICR_NECF | USART_ICR_ORECF);
        rc100_frame_index = 0U;
    }

    while ((READ_REG(huart1.Instance->ISR) & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t byte = (uint8_t)READ_REG(huart1.Instance->RDR);

        rc100_byte_count++;
        rc100_parse_byte(byte);
    }
}

void RC100_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    rc100_byte_count++;
    RC100_UART_IRQHandler();
}

void RC100_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    /* CODEX 2026-07-24: Resynchronize after noise or electrical contention. */
    rc100_uart_error_count++;
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    rc100_frame_index = 0U;
}

void RC100_GetState(rc100_state_t *state)
{
    uint32_t interrupt_state;

    if (state == NULL)
    {
        return;
    }

    /* CODEX 2026-07-24: Copy the ISR-owned state as one coherent snapshot. */
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    state->buttons = rc100_buttons;
    state->last_packet_tick = rc100_last_packet_tick;
    state->packet_count = rc100_packet_count;
    state->byte_count = rc100_byte_count;
    state->invalid_frame_count = rc100_invalid_frame_count;
    state->uart_error_count = rc100_uart_error_count;
    state->receive_start_failure_count = rc100_receive_start_failure_count;
    state->has_packet = rc100_has_packet;
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
}
