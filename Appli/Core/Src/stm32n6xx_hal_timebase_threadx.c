/**
 * @file stm32n6xx_hal_timebase_threadx.c
 * @brief HAL millisecond time base backed by the ThreadX system timer.
 */

#include "stm32n6xx_hal.h"
#include "tx_api.h"

#if TX_TIMER_TICKS_PER_SECOND != 1000U
#error "HAL time base requires a 1000 Hz ThreadX system timer"
#endif

static void HAL_BusyDelayMs(uint32_t delay)
{
    uint32_t cycles_per_ms = SystemCoreClock / 1000U;

    while (delay-- != 0U)
    {
        uint32_t start = DWT->CYCCNT;

        while ((uint32_t)(DWT->CYCCNT - start) < cycles_per_ms)
        {
        }
    }
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    UNUSED(TickPriority);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
    return (uint32_t)tx_time_get();
}

void HAL_Delay(uint32_t Delay)
{
    if (Delay == 0U)
    {
        return;
    }

    if ((__get_IPSR() == 0U) && (tx_thread_identify() != TX_NULL))
    {
        (void)tx_thread_sleep((ULONG)Delay);
    }
    else
    {
        /* Sleeping is illegal before scheduling starts and from an ISR. */
        HAL_BusyDelayMs(Delay);
    }
}
