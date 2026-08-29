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
