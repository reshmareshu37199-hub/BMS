#include "services/isotp.h"

#include <string.h>

#define ISOTP_PCI_SF 0x0U
#define ISOTP_PCI_FF 0x1U
#define ISOTP_PCI_CF 0x2U
#define ISOTP_PCI_FC 0x3U

#define ISOTP_FC_CTS 0x0U
#define ISOTP_FC_WAIT 0x1U
#define ISOTP_FC_OVERFLOW 0x2U

static void SendFrame(IsoTp_t *tp, const uint8_t frame[8])
{
    if ((tp->send != NULL) && tp->send(tp->tx_id, frame, 8U))
    {
        tp->tx_frames++;
    }
}

static void SendFlowControl(IsoTp_t *tp, uint8_t status)
{
    uint8_t frame[8] = {0};
    frame[0] = (uint8_t)((ISOTP_PCI_FC << 4U) | (status & 0x0FU));
    frame[1] = 0x00U; /* block size: send everything */
    frame[2] = 0x00U; /* STmin: immediate */
    SendFrame(tp, frame);
}

static uint32_t StMinToMs(uint8_t stmin)
{
    if (stmin <= 0x7FU)
    {
        return (uint32_t)stmin;
    }
    if ((stmin >= 0xF1U) && (stmin <= 0xF9U))
    {
        return 1U; /* 100..900 us: round up to one millisecond */
    }
    return 0U;
}

void IsoTp_Init(IsoTp_t *tp, uint32_t rx_id, uint32_t tx_id, IsoTpSendFn send)
{
    if (tp == NULL)
    {
        return;
    }
    (void)memset(tp, 0, sizeof(*tp));
    tp->rx_id = rx_id;
    tp->tx_id = tx_id;
    tp->send = send;
}

void IsoTp_Reset(IsoTp_t *tp)
{
    if (tp == NULL)
    {
        return;
    }
    uint32_t rx_frames = tp->rx_frames;
    uint32_t tx_frames = tp->tx_frames;
    uint32_t rx_dropped = tp->rx_dropped;
    uint32_t tx_aborted = tp->tx_aborted;
    uint32_t rx_id = tp->rx_id;
    uint32_t tx_id = tp->tx_id;
    IsoTpSendFn send = tp->send;

    IsoTp_Init(tp, rx_id, tx_id, send);
    tp->rx_frames = rx_frames;
    tp->tx_frames = tx_frames;
    tp->rx_dropped = rx_dropped;
    tp->tx_aborted = tx_aborted;
}

static bool HandleFlowControl(IsoTp_t *tp, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    if (!tp->tx_wait_fc || (dlc < 3U))
    {
        return false;
    }

    uint8_t status = (uint8_t)(data[0] & 0x0FU);
    if (status == ISOTP_FC_CTS)
    {
        tp->tx_block_size = data[1];
        tp->tx_block_sent = 0U;
        tp->tx_stmin_ms = StMinToMs(data[2]);
        tp->tx_wait_fc = false;
        tp->tx_next_ms = now_ms;
        tp->tx_deadline_ms = now_ms + ISOTP_FC_TIMEOUT_MS;
    }
    else if (status == ISOTP_FC_WAIT)
    {
        tp->tx_deadline_ms = now_ms + ISOTP_FC_TIMEOUT_MS;
    }
    else
    {
        tp->tx_active = false;
        tp->tx_wait_fc = false;
        tp->tx_aborted++;
    }
    return true;
}

static bool HandleConsecutive(IsoTp_t *tp, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    if (!tp->rx_active)
    {
        return false;
    }

    uint8_t sequence = (uint8_t)(data[0] & 0x0FU);
    if (sequence != tp->rx_next_seq)
    {
        tp->rx_active = false;
        tp->rx_dropped++;
        return false;
    }

    uint16_t remaining = (uint16_t)(tp->rx_expected - tp->rx_len);
    uint16_t available = (uint16_t)(dlc - 1U);
    uint16_t copy = (remaining < available) ? remaining : available;
    if (copy > 0U)
    {
        (void)memcpy(&tp->rx_buf[tp->rx_len], &data[1], copy);
        tp->rx_len = (uint16_t)(tp->rx_len + copy);
    }

    tp->rx_next_seq = (uint8_t)((tp->rx_next_seq + 1U) & 0x0FU);
    tp->rx_deadline_ms = now_ms + ISOTP_CF_TIMEOUT_MS;

    if (tp->rx_len >= tp->rx_expected)
    {
        tp->rx_active = false;
        tp->rx_ready = true;
        return true;
    }
    return false;
}

static bool HandleFirstFrame(IsoTp_t *tp, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    uint16_t total = (uint16_t)((((uint16_t)data[0] & 0x0FU) << 8U) | (uint16_t)data[1]);
    if ((total <= 7U) || (total > ISOTP_MAX_PAYLOAD) || (dlc < 8U))
    {
        tp->rx_dropped++;
        SendFlowControl(tp, ISOTP_FC_OVERFLOW);
        return false;
    }

    tp->rx_expected = total;
    tp->rx_len = 6U;
    (void)memcpy(tp->rx_buf, &data[2], 6U);
    tp->rx_next_seq = 1U;
    tp->rx_active = true;
    tp->rx_ready = false;
    tp->rx_deadline_ms = now_ms + ISOTP_CF_TIMEOUT_MS;
    SendFlowControl(tp, ISOTP_FC_CTS);
    return false;
}

static bool HandleSingleFrame(IsoTp_t *tp, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    (void)now_ms;
    uint8_t length = (uint8_t)(data[0] & 0x0FU);
    if ((length == 0U) || (length > 7U) || ((uint8_t)(length + 1U) > dlc))
    {
        tp->rx_dropped++;
        return false;
    }

    (void)memcpy(tp->rx_buf, &data[1], length);
    tp->rx_len = length;
    tp->rx_expected = length;
    tp->rx_active = false;
    tp->rx_ready = true;
    return true;
}

bool IsoTp_OnFrame(IsoTp_t *tp, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    if ((tp == NULL) || (data == NULL) || (dlc == 0U))
    {
        return false;
    }

    tp->rx_frames++;
    uint8_t pci = (uint8_t)((data[0] >> 4U) & 0x0FU);

    switch (pci)
    {
    case ISOTP_PCI_SF:
        return HandleSingleFrame(tp, data, dlc, now_ms);
    case ISOTP_PCI_FF:
        return HandleFirstFrame(tp, data, dlc, now_ms);
    case ISOTP_PCI_CF:
        return HandleConsecutive(tp, data, dlc, now_ms);
    case ISOTP_PCI_FC:
        (void)HandleFlowControl(tp, data, dlc, now_ms);
        return false;
    default:
        tp->rx_dropped++;
        return false;
    }
}

uint16_t IsoTp_Read(IsoTp_t *tp, uint8_t *out, uint16_t out_len)
{
    if ((tp == NULL) || (out == NULL) || !tp->rx_ready)
    {
        return 0U;
    }

    uint16_t length = tp->rx_len;
    if (length > out_len)
    {
        length = out_len;
    }
    (void)memcpy(out, tp->rx_buf, length);
    tp->rx_ready = false;
    tp->rx_len = 0U;
    tp->rx_expected = 0U;
    return length;
}

bool IsoTp_Write(IsoTp_t *tp, const uint8_t *payload, uint16_t len, uint32_t now_ms)
{
    if ((tp == NULL) || (payload == NULL) || (len == 0U) || (len > ISOTP_MAX_PAYLOAD) || tp->tx_active)
    {
        return false;
    }

    (void)memcpy(tp->tx_buf, payload, len);
    tp->tx_len = len;
    tp->tx_offset = 0U;
    tp->tx_seq = 1U;
    tp->tx_block_size = 0U;
    tp->tx_block_sent = 0U;
    tp->tx_stmin_ms = 0U;
    tp->tx_wait_fc = false;
    tp->tx_next_ms = now_ms;

    uint8_t frame[8] = {0};
    if (len <= 7U)
    {
        frame[0] = (uint8_t)((ISOTP_PCI_SF << 4U) | (uint8_t)len);
        (void)memcpy(&frame[1], payload, len);
        SendFrame(tp, frame);
        tp->tx_active = false;
        return true;
    }

    frame[0] = (uint8_t)((ISOTP_PCI_FF << 4U) | (uint8_t)((len >> 8U) & 0x0FU));
    frame[1] = (uint8_t)(len & 0xFFU);
    (void)memcpy(&frame[2], payload, 6U);
    SendFrame(tp, frame);

    tp->tx_offset = 6U;
    tp->tx_active = true;
    tp->tx_wait_fc = true;
    tp->tx_deadline_ms = now_ms + ISOTP_FC_TIMEOUT_MS;
    return true;
}

void IsoTp_Poll(IsoTp_t *tp, uint32_t now_ms)
{
    if (tp == NULL)
    {
        return;
    }

    if (tp->rx_active && (now_ms > tp->rx_deadline_ms))
    {
        tp->rx_active = false;
        tp->rx_dropped++;
    }

    if (!tp->tx_active)
    {
        return;
    }

    if (tp->tx_wait_fc)
    {
        if (now_ms > tp->tx_deadline_ms)
        {
            tp->tx_active = false;
            tp->tx_wait_fc = false;
            tp->tx_aborted++;
        }
        return;
    }

    if (now_ms < tp->tx_next_ms)
    {
        return;
    }

    /* Bounded loop: at most ISOTP_MAX_PAYLOAD/6 consecutive frames. */
    while ((tp->tx_offset < tp->tx_len) && tp->tx_active)
    {
        if ((tp->tx_block_size != 0U) && (tp->tx_block_sent >= tp->tx_block_size))
        {
            tp->tx_wait_fc = true;
            tp->tx_deadline_ms = now_ms + ISOTP_FC_TIMEOUT_MS;
            break;
        }

        uint16_t remaining = (uint16_t)(tp->tx_len - tp->tx_offset);
        uint16_t chunk = (remaining > 6U) ? 6U : remaining;
        uint8_t frame[8] = {0};
        frame[0] = (uint8_t)((ISOTP_PCI_CF << 4U) | (tp->tx_seq & 0x0FU));
        (void)memcpy(&frame[1], &tp->tx_buf[tp->tx_offset], chunk);
        SendFrame(tp, frame);

        tp->tx_offset = (uint16_t)(tp->tx_offset + chunk);
        tp->tx_seq = (uint8_t)((tp->tx_seq + 1U) & 0x0FU);
        tp->tx_block_sent++;

        if (tp->tx_offset >= tp->tx_len)
        {
            tp->tx_active = false;
            break;
        }

        /* Block boundary: wait for the next Flow Control as soon as the
         * negotiated block is exhausted, even if a separation time is set. */
        if ((tp->tx_block_size != 0U) && (tp->tx_block_sent >= tp->tx_block_size))
        {
            tp->tx_wait_fc = true;
            tp->tx_deadline_ms = now_ms + ISOTP_FC_TIMEOUT_MS;
            break;
        }

        tp->tx_next_ms = now_ms + tp->tx_stmin_ms;
        if (tp->tx_stmin_ms > 0U)
        {
            break; /* wait for the next Poll at the requested separation time */
        }
    }
}

bool IsoTp_IsBusy(const IsoTp_t *tp)
{
    return (tp == NULL) ? false : (tp->tx_active || tp->rx_active || tp->rx_ready);
}
