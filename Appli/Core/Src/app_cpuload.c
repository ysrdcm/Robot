/**
 ****************************************************************************************************
 * @file        app_cpuload.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_cpuload.c文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 N647开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#include "app_cpuload.h"
#include "tx_api.h"
#include <string.h>

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
        *cpuload_last = 100.0 * (cpuload->history[0].thread - cpuload->history[1].thread) / (cpuload->history[0].total - cpuload->history[1].total);
    }

    if (cpuload_last_second != NULL)
    {
        *cpuload_last_second = 100.0 * (cpuload->history[2].thread - cpuload->history[3].thread) / (cpuload->history[2].total - cpuload->history[3].total);
    }

    if (cpuload_last_five_seconds != NULL)
    {
        *cpuload_last_five_seconds = 100.0 * (cpuload->history[2].thread - cpuload->history[7].thread) / (cpuload->history[2].total - cpuload->history[7].total);
    }
}
