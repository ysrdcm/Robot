/**
 ****************************************************************************************************
 * @file        app_bqueue.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_bqueue.h文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */

#ifndef __APP_BQUEUE_H
#define __APP_BQUEUE_H

#include "tx_api.h"
#include "app_config.h"

typedef struct {
    TX_SEMAPHORE free;
    TX_SEMAPHORE ready;
    uint8_t buffer_nb;
    uint8_t *buffers[BQUEUE_MAX_BUFFERS];
    uint8_t free_idx;
    uint8_t ready_idx;
} app_bqueue_t;

void app_bqueue_init(app_bqueue_t *bqueue, uint8_t buffer_nb, uint8_t **buffers);
uint8_t *app_bqueue_get_free(app_bqueue_t *bqueue, uint8_t is_blocking);
void app_bqueue_put_free(app_bqueue_t *bqueue);
uint8_t *app_bqueue_get_ready(app_bqueue_t *bqueue);
void app_bqueue_put_ready(app_bqueue_t *bqueue);

#endif
