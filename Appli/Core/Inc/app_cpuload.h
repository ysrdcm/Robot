/**
 ****************************************************************************************************
 * @file        app_cpuload.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_cpuload.h文件
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

#ifndef __APP_CPULOAD_H
#define __APP_CPULOAD_H

#include "app_config.h"

typedef struct {
    uint64_t current_total;
    uint64_t current_thread_total;
    uint64_t prev_total;
    uint64_t prev_thread_total;
    struct {
        uint64_t total;
        uint64_t thread;
        uint32_t tick;
    } history[CPU_LOAD_HISTORY_DEPTH];
} app_cpuload_t;

void app_cpuload_init(app_cpuload_t *cpuload);
void app_cpuload_update(app_cpuload_t *cpuload);
void app_cpuload_get_info(app_cpuload_t *cpuload, float *cpuload_last, float *cpuload_last_second, float *cpuload_last_five_seconds);

#endif
