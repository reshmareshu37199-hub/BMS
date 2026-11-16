#ifndef BMS_SERVICES_ISOTP_H
#define BMS_SERVICES_ISOTP_H

#include <stdbool.h>
#include <stdint.h>

#define ISOTP_MAX_PAYLOAD 64U
#define ISOTP_FC_TIMEOUT_MS 1000U
#define ISOTP_CF_TIMEOUT_MS 1000U

typedef bool (*IsoTpSendFn)(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * Minimal ISO 15765-2 transport for classic CAN.
 *
 * Supports single frames, first frame / flow control / consecutive frames in
 * both directions.  Only one reception and one transmission may be active at
 * a time, which is sufficient for a request/response diagnostic server.
 */
typedef struct
{
    uint32_t rx_id;
    uint32_t tx_id;
    IsoTpSendFn send;

    uint8_t rx_buf[ISOTP_MAX_PAYLOAD];
    uint16_t rx_len;
    uint16_t rx_expected;
    uint8_t rx_next_seq;
    bool rx_active;
    bool rx_ready;
    uint32_t rx_deadline_ms;

    uint8_t tx_buf[ISOTP_MAX_PAYLOAD];
    uint16_t tx_len;
    uint16_t tx_offset;
    uint8_t tx_seq;
    bool tx_active;
    bool tx_wait_fc;
    uint8_t tx_block_size;
    uint8_t tx_block_sent;
    uint32_t tx_next_ms;
    uint32_t tx_deadline_ms;
    uint32_t tx_stmin_ms;

    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t rx_dropped;
    uint32_t tx_aborted;
} IsoTp_t;

void IsoTp_Init(IsoTp_t *tp, uint32_t rx_id, uint32_t tx_id, IsoTpSendFn send);
void IsoTp_Reset(IsoTp_t *tp);

/**
 * Feed one received CAN frame.  Returns true when a complete diagnostic
 * message is available through IsoTp_Read().
 */
bool IsoTp_OnFrame(IsoTp_t *tp, const uint8_t *data, uint8_t dlc, uint32_t now_ms);

/** Copy the completed message out and clear the pending flag. */
uint16_t IsoTp_Read(IsoTp_t *tp, uint8_t *out, uint16_t out_len);

/**
 * Start a response.  Single frames are sent immediately.  Multi-frame
 * responses send the First Frame and then wait for a Flow Control frame.
 */
bool IsoTp_Write(IsoTp_t *tp, const uint8_t *payload, uint16_t len, uint32_t now_ms);

/** Drive consecutive-frame transmission and timeouts.  Call every 10 ms. */
void IsoTp_Poll(IsoTp_t *tp, uint32_t now_ms);

bool IsoTp_IsBusy(const IsoTp_t *tp);

#endif /* BMS_SERVICES_ISOTP_H */
