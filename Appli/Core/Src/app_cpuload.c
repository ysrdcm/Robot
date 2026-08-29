#include "app_cpuload.h"
#include "tx_api.h"
#include <string.h>

static float app_cpuload_calculate(uint64_t current_total, uint64_t previous_total,
                                   uint64_t current_thread, uint64_t previous_thread)
{
    uint64_t total_delta;
    uint64_t thread_delta;

    if ((current_total <= previous_total) || (current_thread < previous_thread))
    {
        return 0.0f;
    }

    total_delta = current_total - previous_total;
    thread_delta = current_thread - previous_thread;
    if (thread_delta > total_delta)
    {
        return 0.0f;
    }

    return 100.0f * (float)thread_delta / (float)total_delta;
}

void app_cpuload_init(app_cpuload_t *cpuload)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    memset(cpuload, 0, sizeof(app_cpuload_t));
}

void app_cpuload_update(app_cpuload_t *cpuload)
{
    EXECUTION_TIME thread_total;
    EXECUTION_TIME isr;
    EXECUTION_TIME idle;
    uint8_t i;

    cpuload->history[1] = cpuload->history[0];

    _tx_execution_thread_total_time_get(&thread_total);
    _tx_execution_isr_time_get(&isr);
    _tx_execution_idle_time_get(&idle);

    cpuload->history[0].total = thread_total + isr + idle;
    cpuload->history[0].thread = thread_total;
    cpuload->history[0].tick = HAL_GetTick();

    if ((cpuload->history[1].tick - cpuload->history[2].tick) < 1000)
    {
        return;
    }

    for (i = 0; i < CPU_LOAD_HISTORY_DEPTH - 2; i++)
    {
        cpuload->history[CPU_LOAD_HISTORY_DEPTH - 1 - i] = cpuload->history[CPU_LOAD_HISTORY_DEPTH - 1 - i - 1];
    }
}

void app_cpuload_get_info(app_cpuload_t *cpuload, float *cpuload_last, float *cpuload_last_second, float *cpuload_last_five_seconds)
{
    if (cpuload_last != NULL)
    {
        *cpuload_last = app_cpuload_calculate(cpuload->history[0].total, cpuload->history[1].total,
                                              cpuload->history[0].thread, cpuload->history[1].thread);
    }

    if (cpuload_last_second != NULL)
    {
        *cpuload_last_second = app_cpuload_calculate(cpuload->history[2].total, cpuload->history[3].total,
                                                     cpuload->history[2].thread, cpuload->history[3].thread);
    }

    if (cpuload_last_five_seconds != NULL)
    {
        *cpuload_last_five_seconds = app_cpuload_calculate(cpuload->history[2].total, cpuload->history[7].total,
                                                          cpuload->history[2].thread, cpuload->history[7].thread);
    }
}
