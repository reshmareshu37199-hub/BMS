#include "drivers/afe.h"

#include "common/crc.h"
#include "drivers/ntc_table.h"
#include "hal/hal.h"

#include <string.h>

#define AFE_REG_DEVICE_ID 0x00U
#define AFE_REG_STATUS 0x01U
#define AFE_REG_ADC_CTRL 0x02U
#define AFE_REG_CELL_BASE 0x10U
#define AFE_REG_TEMP_BASE 0x30U
#define AFE_REG_BAL_CTRL 0x40U

#define AFE_STATUS_CONV_DONE 0x01U
#define AFE_STATUS_INTERNAL_FAULT 0x02U
#define AFE_STATUS_OPEN_WIRE 0x04U

typedef enum
{
    AFE_ST_READ_ID = 0,
    AFE_ST_IDLE,
    AFE_ST_START_CONV,
    AFE_ST_WAIT_DONE,
    AFE_ST_READ_CELLS,
    AFE_ST_READ_TEMPS,
    AFE_ST_PUBLISH
} AfeState_t;

static AfeState_t s_state;
static uint8_t s_id_attempts;
static uint8_t s_retries;
static uint8_t s_failed_cycles;
static bool s_comm_fault;
static bool s_id_failed;
static bool s_cycle_requested;
static uint8_t s_step;
static uint32_t s_deadline_ms;
static uint16_t s_cell_raw[AFE_CELL_COUNT];
static uint16_t s_temp_raw[AFE_TEMP_COUNT];
static bool s_internal_fault;
static bool s_open_wire;
static uint16_t s_bal_mask;

static AfeSnapshot_t s_buffers[2];
static volatile uint8_t s_active;
static uint32_t s_sequence;

static bool Afe_ReadRegisters(uint16_t address, uint8_t length, uint8_t *out)
{
    uint8_t tx[3];
    uint8_t rx[3];

    if ((out == NULL) || (length == 0U) || (length > 2U))
    {
        return false;
    }

    tx[0] = (uint8_t)(address & 0x7FU);
    tx[1] = length;
    tx[2] = Crc8_Poly07(tx, 2U);

    if (!Hal_SpiTransfer(tx, 3U, rx, (uint16_t)(length + 1U)))
    {
        return false;
    }
    if (Crc8_Poly07(rx, (size_t)length) != rx[length])
    {
        return false;
    }

    for (uint8_t i = 0U; i < length; ++i)
    {
        out[i] = rx[i];
    }
    return true;
}

static bool Afe_WriteRegisters(uint16_t address, const uint8_t *data, uint8_t length)
{
    uint8_t tx[5];

    if ((data == NULL) || (length == 0U) || (length > 2U))
    {
        return false;
    }

    tx[0] = (uint8_t)((address & 0x7FU) | 0x80U);
    tx[1] = length;
    for (uint8_t i = 0U; i < length; ++i)
    {
        tx[2U + i] = data[i];
    }
    tx[2U + length] = Crc8_Poly07(tx, (size_t)(2U + length));

    return Hal_SpiTransfer(tx, (uint16_t)(3U + length), NULL, 0U);
}

static void Afe_PublishInvalid(void)
{
    uint8_t next = (uint8_t)(1U - s_active);
    (void)memset(&s_buffers[next], 0, sizeof(s_buffers[next]));
    s_buffers[next].valid = false;
    s_buffers[next].sequence = ++s_sequence;

    Hal_CriticalEnter();
    s_active = next;
    Hal_CriticalExit();
}

static void Afe_HandleTransactionError(void)
{
    s_retries++;
    if (s_retries <= AFE_MAX_RETRIES)
    {
        s_state = AFE_ST_START_CONV;
        return;
    }

    s_retries = 0U;
    s_cycle_requested = false;
    s_failed_cycles++;
    s_state = AFE_ST_IDLE;

    if (s_failed_cycles >= AFE_MAX_FAILED_CYCLES)
    {
        s_comm_fault = true;
        Afe_PublishInvalid();
    }
}

static void Afe_PublishSnapshot(void)
{
    uint8_t next = (uint8_t)(1U - s_active);
    AfeSnapshot_t *snap = &s_buffers[next];

    (void)memset(snap, 0, sizeof(*snap));
    uint32_t sum_mv = 0U;
    for (uint8_t i = 0U; i < AFE_CELL_COUNT; ++i)
    {
        uint16_t value = s_cell_raw[i];
        snap->cell_mv[i] = (uint16_t)(value / 10U);
        sum_mv += (uint32_t)snap->cell_mv[i];
    }
    for (uint8_t i = 0U; i < AFE_TEMP_COUNT; ++i)
    {
        uint16_t raw = (uint16_t)(s_temp_raw[i] & 0x0FFFU);
        snap->temp_dc[i] = Ntc_RawToDeciC(raw);
    }

    uint16_t pack_mv = Hal_PackVoltageMv();
    uint32_t diff = (sum_mv > (uint32_t)pack_mv) ? (sum_mv - (uint32_t)pack_mv)
                                                 : ((uint32_t)pack_mv - sum_mv);
    snap->sum_mismatch = (diff > AFE_CELL_SUM_TOLERANCE_MV);
    snap->internal_fault = s_internal_fault;
    snap->open_wire = s_open_wire;
    snap->valid = !s_internal_fault;
    snap->sequence = ++s_sequence;

    Hal_CriticalEnter();
    s_active = next;
    Hal_CriticalExit();

    s_failed_cycles = 0U;
    s_retries = 0U;
    s_cycle_requested = false;
    s_state = AFE_ST_IDLE;
}

void Afe_Init(void)
{
    s_state = AFE_ST_READ_ID;
    s_id_attempts = 0U;
    s_retries = 0U;
    s_failed_cycles = 0U;
    s_comm_fault = false;
    s_id_failed = false;
    s_cycle_requested = false;
    s_step = 0U;
    s_deadline_ms = 0U;
    s_internal_fault = false;
    s_open_wire = false;
    s_bal_mask = 0U;
    (void)memset(s_cell_raw, 0, sizeof(s_cell_raw));
    (void)memset(s_temp_raw, 0, sizeof(s_temp_raw));
    (void)memset(s_buffers, 0, sizeof(s_buffers));
    s_active = 0U;
    s_sequence = 0U;
}

void Afe_StartCycle(uint32_t now_ms)
{
    (void)now_ms;
    if (!Afe_IsBusy() && !s_id_failed)
    {
        s_cycle_requested = true;
    }
}

bool Afe_IsBusy(void)
{
    return (s_state != AFE_ST_IDLE) && (s_state != AFE_ST_READ_ID);
}

bool Afe_Poll(uint32_t now_ms)
{
    uint8_t buffer[2] = {0};

    switch (s_state)
    {
    case AFE_ST_READ_ID:
        if (Afe_ReadRegisters(AFE_REG_DEVICE_ID, 1U, buffer) && (buffer[0] == AFE_DEVICE_ID_VALUE))
        {
            s_id_attempts = 0U;
            s_state = AFE_ST_IDLE;
        }
        else
        {
            s_id_attempts++;
            if (s_id_attempts >= 3U)
            {
                s_comm_fault = true;
                s_id_failed = true;
                s_state = AFE_ST_IDLE;
            }
        }
        break;

    case AFE_ST_IDLE:
        if (s_cycle_requested && !s_id_failed)
        {
            s_state = AFE_ST_START_CONV;
        }
        break;

    case AFE_ST_START_CONV:
    {
        uint8_t ctrl = 0x01U;
        if (Afe_WriteRegisters(AFE_REG_ADC_CTRL, &ctrl, 1U))
        {
            s_deadline_ms = now_ms + AFE_CONVERSION_TIMEOUT_MS;
            s_state = AFE_ST_WAIT_DONE;
        }
        else
        {
            Afe_HandleTransactionError();
        }
        break;
    }

    case AFE_ST_WAIT_DONE:
        if (!Afe_ReadRegisters(AFE_REG_STATUS, 1U, buffer))
        {
            Afe_HandleTransactionError();
        }
        else
        {
            s_internal_fault = ((buffer[0] & AFE_STATUS_INTERNAL_FAULT) != 0U);
            s_open_wire = ((buffer[0] & AFE_STATUS_OPEN_WIRE) != 0U);
            if ((buffer[0] & AFE_STATUS_CONV_DONE) != 0U)
            {
                s_step = 0U;
                s_state = AFE_ST_READ_CELLS;
            }
            else if (now_ms >= s_deadline_ms)
            {
                Afe_HandleTransactionError();
            }
        }
        break;

    case AFE_ST_READ_CELLS:
        if (!Afe_ReadRegisters((uint16_t)(AFE_REG_CELL_BASE + ((uint16_t)s_step * 2U)), 2U, buffer))
        {
            Afe_HandleTransactionError();
        }
        else
        {
            s_cell_raw[s_step] = (uint16_t)(((uint16_t)buffer[0] << 8U) | (uint16_t)buffer[1]);
            s_step++;
            if (s_step >= AFE_CELL_COUNT)
            {
                s_step = 0U;
                s_state = AFE_ST_READ_TEMPS;
            }
        }
        break;

    case AFE_ST_READ_TEMPS:
        if (!Afe_ReadRegisters((uint16_t)(AFE_REG_TEMP_BASE + ((uint16_t)s_step * 2U)), 2U, buffer))
        {
            Afe_HandleTransactionError();
        }
        else
        {
            s_temp_raw[s_step] = (uint16_t)(((uint16_t)buffer[0] << 8U) | (uint16_t)buffer[1]);
            s_step++;
            if (s_step >= AFE_TEMP_COUNT)
            {
                s_step = 0U;
                s_state = AFE_ST_PUBLISH;
            }
        }
        break;

    case AFE_ST_PUBLISH:
        Afe_PublishSnapshot();
        return true;

    default:
        s_state = AFE_ST_IDLE;
        break;
    }

    return false;
}

bool Afe_GetSnapshot(AfeSnapshot_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    Hal_CriticalEnter();
    *out = s_buffers[s_active];
    Hal_CriticalExit();
    return true;
}

bool Afe_IsValid(void)
{
    bool valid;

    Hal_CriticalEnter();
    valid = s_buffers[s_active].valid;
    Hal_CriticalExit();
    return valid;
}

uint8_t Afe_GetFailedCycles(void)
{
    return s_failed_cycles;
}

bool Afe_HasCommFault(void)
{
    return s_comm_fault;
}

void Afe_ClearLatchedFault(void)
{
    s_comm_fault = false;
    s_id_failed = false;
    s_failed_cycles = 0U;
    s_retries = 0U;
    s_id_attempts = 0U;
    s_state = AFE_ST_READ_ID;
}

uint16_t Afe_GetBalMask(void)
{
    return s_bal_mask;
}

bool Afe_SetBalMask(uint16_t mask)
{
    uint8_t data[2];

    if (Afe_IsBusy())
    {
        return false;
    }

    data[0] = (uint8_t)((mask >> 8U) & 0xFFU);
    data[1] = (uint8_t)(mask & 0xFFU);
    if (!Afe_WriteRegisters(AFE_REG_BAL_CTRL, data, 2U))
    {
        return false;
    }
    s_bal_mask = mask;
    return true;
}
