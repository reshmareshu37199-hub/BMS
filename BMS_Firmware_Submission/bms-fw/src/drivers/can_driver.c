#include "drivers/can_driver.h"

#include "hal/hal.h"

#include <string.h>

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[CANDRV_MAX_DLC];
} CanFrame_t;

typedef struct
{
    uint32_t id;
    CanDrvRxHandler_t handler;
    bool used;
} CanHandlerEntry_t;

static CanFrame_t s_tx_queue[CANDRV_TX_QUEUE_DEPTH];
static uint8_t s_tx_head;
static uint8_t s_tx_tail;
static uint8_t s_tx_count;

static CanHandlerEntry_t s_handlers[CANDRV_RX_HANDLER_COUNT];
static CanDrvStats_t s_stats;

void CanDrv_Init(void)
{
    (void)memset(s_tx_queue, 0, sizeof(s_tx_queue));
    (void)memset(s_handlers, 0, sizeof(s_handlers));
    (void)memset(&s_stats, 0, sizeof(s_stats));
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_count = 0U;
}

bool CanDrv_Queue(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if ((data == NULL) || (dlc > CANDRV_MAX_DLC))
    {
        s_stats.tx_dropped++;
        return false;
    }
    if (s_tx_count >= CANDRV_TX_QUEUE_DEPTH)
    {
        s_stats.tx_dropped++;
        return false;
    }

    CanFrame_t *frame = &s_tx_queue[s_tx_tail];
    frame->id = id;
    frame->dlc = dlc;
    (void)memset(frame->data, 0, sizeof(frame->data));
    (void)memcpy(frame->data, data, dlc);

    s_tx_tail = (uint8_t)((s_tx_tail + 1U) % CANDRV_TX_QUEUE_DEPTH);
    s_tx_count++;
    s_stats.tx_queued++;
    return true;
}

bool CanDrv_RegisterHandler(uint32_t id, CanDrvRxHandler_t handler)
{
    if (handler == NULL)
    {
        return false;
    }

    for (uint8_t i = 0U; i < CANDRV_RX_HANDLER_COUNT; ++i)
    {
        if (s_handlers[i].used && (s_handlers[i].id == id))
        {
            s_handlers[i].handler = handler;
            return true;
        }
    }

    for (uint8_t i = 0U; i < CANDRV_RX_HANDLER_COUNT; ++i)
    {
        if (!s_handlers[i].used)
        {
            s_handlers[i].used = true;
            s_handlers[i].id = id;
            s_handlers[i].handler = handler;
            return true;
        }
    }

    return false;
}

static void DrainTx(void)
{
    uint8_t sent = 0U;

    while ((s_tx_count > 0U) && (sent < CANDRV_MAX_TX_PER_POLL))
    {
        const CanFrame_t *frame = &s_tx_queue[s_tx_head];
        if (!Hal_CanSend(frame->id, frame->data, frame->dlc))
        {
            /* Mailbox busy: leave the frame queued and try again next poll. */
            break;
        }

        s_tx_head = (uint8_t)((s_tx_head + 1U) % CANDRV_TX_QUEUE_DEPTH);
        s_tx_count--;
        s_stats.tx_sent++;
        sent++;
    }
}

static void DispatchRx(void)
{
    uint8_t received = 0U;

    while (received < CANDRV_MAX_RX_PER_POLL)
    {
        uint32_t id = 0U;
        uint8_t data[CANDRV_MAX_DLC] = {0};
        uint8_t dlc = 0U;

        if (!Hal_CanRecv(&id, data, &dlc))
        {
            break;
        }
        received++;
        s_stats.rx_frames++;

        if (dlc > CANDRV_MAX_DLC)
        {
            s_stats.rx_ignored++;
            continue;
        }

        bool dispatched = false;
        for (uint8_t i = 0U; i < CANDRV_RX_HANDLER_COUNT; ++i)
        {
            if (s_handlers[i].used && (s_handlers[i].id == id))
            {
                s_handlers[i].handler(id, data, dlc);
                s_stats.rx_dispatched++;
                dispatched = true;
                break;
            }
        }
        if (!dispatched)
        {
            s_stats.rx_ignored++;
        }
    }
}

void CanDrv_Poll(void)
{
    DrainTx();
    DispatchRx();
}

CanDrvStats_t CanDrv_GetStats(void)
{
    return s_stats;
}

uint8_t CanDrv_TxQueueDepth(void)
{
    return s_tx_count;
}
