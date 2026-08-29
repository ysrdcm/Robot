#include "app_bqueue.h"

UINT app_bqueue_init(app_bqueue_t *bqueue, uint8_t buffer_nb, uint8_t **buffers)
{
    uint8_t i;
    UINT status;

    if ((bqueue == NULL) || (buffers == NULL))
    {
        return TX_PTR_ERROR;
    }
    if ((buffer_nb == 0U) || (buffer_nb > BQUEUE_MAX_BUFFERS))
    {
        return TX_SIZE_ERROR;
    }

    status = tx_semaphore_create(&bqueue->free, NULL, buffer_nb);
    if (status != TX_SUCCESS)
    {
        return status;
    }
    status = tx_semaphore_create(&bqueue->ready, NULL, 0U);
    if (status != TX_SUCCESS)
    {
        (void)tx_semaphore_delete(&bqueue->free);
        return status;
    }

    bqueue->buffer_nb = buffer_nb;
    for (i = 0; i < buffer_nb; i++)
    {
        bqueue->buffers[i] = buffers[i];
    }
    bqueue->free_idx = 0;
    bqueue->ready_idx = 0;

    return TX_SUCCESS;
}

uint8_t *app_bqueue_get_free(app_bqueue_t *bqueue, uint8_t is_blocking)
{
    uint8_t *free;

    if (tx_semaphore_get(&bqueue->free,
                         is_blocking ? TX_WAIT_FOREVER : TX_NO_WAIT) != TX_SUCCESS)
    {
        return NULL;
    }

    free = bqueue->buffers[bqueue->free_idx];
    bqueue->free_idx = (bqueue->free_idx + 1) % bqueue->buffer_nb;

    return free;
}

void app_bqueue_put_free(app_bqueue_t *bqueue)
{
    tx_semaphore_put(&bqueue->free);
}

uint8_t *app_bqueue_get_ready(app_bqueue_t *bqueue)
{
    uint8_t *ready;

    if (tx_semaphore_get(&bqueue->ready, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
        return NULL;
    }

    ready = bqueue->buffers[bqueue->ready_idx];
    bqueue->ready_idx = (bqueue->ready_idx + 1) % bqueue->buffer_nb;

    return ready;
}

void app_bqueue_put_ready(app_bqueue_t *bqueue)
{
    tx_semaphore_put(&bqueue->ready);
}
