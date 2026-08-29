/**
 * @file stm32n6xx_hal_timebase_threadx.c
 * @brief 使用 ThreadX 系统定时器提供 HAL 毫秒时基。
 */

#include "stm32n6xx_hal.h"
#include "tx_api.h"

#if TX_TIMER_TICKS_PER_SECOND != 1000U
#error "HAL time base requires TX_TIMER_TICKS_PER_SECOND == 1000"
#endif

static uint32_t hal_dwt_start_cycles;
static volatile uint32_t hal_threadx_tick_offset;
static volatile uint8_t hal_threadx_time_active;

static uint32_t HAL_DwtGetTick(void)
{
    uint32_t cycles_per_ms = SystemCoreClock / 1000U;

    if (cycles_per_ms == 0U)
    {
        return 0U;
    }
    return (DWT->CYCCNT - hal_dwt_start_cycles) / cycles_per_ms;
}

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
    uint32_t preserved_tick = 0U;
    uint8_t kernel_running;

    UNUSED(TickPriority);

    kernel_running = ((__get_IPSR() == 0U) &&
                      (tx_thread_identify() != TX_NULL)) ? 1U : 0U;
    if (kernel_running != 0U)
    {
        preserved_tick = (hal_threadx_time_active != 0U) ?
                         hal_threadx_tick_offset + (uint32_t)tx_time_get() :
                         (uint32_t)tx_time_get();
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    hal_dwt_start_cycles = DWT->CYCCNT;
    if (kernel_running != 0U)
    {
        hal_threadx_tick_offset = preserved_tick - (uint32_t)tx_time_get();
        hal_threadx_time_active = 1U;
    }
    else
    {
        hal_threadx_tick_offset = 0U;
        hal_threadx_time_active = 0U;
    }

    return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
    uint32_t kernel_tick;

    if (hal_threadx_time_active != 0U)
    {
        return hal_threadx_tick_offset + (uint32_t)tx_time_get();
    }

    /* 内核启动前 ThreadX tick 不递增，HAL 超时必须临时使用 DWT。 */
    if ((__get_IPSR() == 0U) && (tx_thread_identify() != TX_NULL))
    {
        uint32_t interrupt_state = __get_PRIMASK();

        __disable_irq();
        if (hal_threadx_time_active == 0U)
        {
            kernel_tick = (uint32_t)tx_time_get();
            hal_threadx_tick_offset = HAL_DwtGetTick() - kernel_tick;
            __DMB();
            hal_threadx_time_active = 1U;
        }
        if (interrupt_state == 0U)
        {
            __enable_irq();
        }
        return hal_threadx_tick_offset + (uint32_t)tx_time_get();
    }

    return HAL_DwtGetTick();
}

void HAL_Delay(uint32_t Delay)
{
    if (Delay == 0U)
    {
        return;
    }

    if ((__get_IPSR() == 0U) && (tx_thread_identify() != TX_NULL))
    {
        ULONG sleep_ticks = (ULONG)Delay;

        if (sleep_ticks != TX_WAIT_FOREVER)
        {
            sleep_ticks++;
        }
        (void)tx_thread_sleep(sleep_ticks);
    }
    else
    {
        /* 调度器启动前或中断中不能休眠，改用 DWT 忙等待。 */
        HAL_BusyDelayMs(Delay);
    }
}
