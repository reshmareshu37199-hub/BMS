#ifndef BMS_DRIVERS_CAN_DRIVER_H
#define BMS_DRIVERS_CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#define CANDRV_TX_QUEUE_DEPTH 8U
#define CANDRV_RX_HANDLER_COUNT 8U
#define CANDRV_MAX_TX_PER_POLL 4U
#define CANDRV_MAX_RX_PER_POLL 8U
#define CANDRV_MAX_DLC 8U

typedef void (*CanDrvRxHandler_t)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct
{
    uint32_t tx_queued;
    uint32_t tx_sent;
    uint32_t tx_dropped;
    uint32_t rx_frames;
    uint32_t rx_dispatched;
    uint32_t rx_ignored;
} CanDrvStats_t;

/**
 * Initialise the software CAN driver.
 *
 * The driver owns a bounded Tx queue so that the 100 ms task never blocks on
 * a full hardware mailbox.  If the queue is full the newest frame is dropped
 * and counted; periodic frames are retransmitted on the next cycle anyway.
 */
void CanDrv_Init(void);

/** Enqueue one classic CAN frame.  Returns false if the queue is full. */
bool CanDrv_Queue(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * Drain a bounded number of Tx frames and dispatch a bounded number of Rx
 * frames.  Worst case is 4 SPI-free HAL calls plus 8 handler calls, so the
 * function has a fixed upper bound independent of bus load.
 */
void CanDrv_Poll(void);

bool CanDrv_RegisterHandler(uint32_t id, CanDrvRxHandler_t handler);
CanDrvStats_t CanDrv_GetStats(void);
uint8_t CanDrv_TxQueueDepth(void);

#endif /* BMS_DRIVERS_CAN_DRIVER_H */
