#include "hal/afe_sim.h"

#include "common/crc.h"
#include "common/saturate.h"
#include "drivers/ntc_table.h"

#define AFE_REG_DEVICE_ID 0x00U
#define AFE_REG_STATUS 0x01U
#define AFE_REG_ADC_CTRL 0x02U
#define AFE_REG_CELL_BASE 0x10U
#define AFE_REG_TEMP_BASE 0x30U
#define AFE_REG_BAL_CTRL 0x40U

#define AFE_STATUS_CONV_DONE 0x01U
#define AFE_STATUS_INTERNAL_FAULT 0x02U
#define AFE_STATUS_OPEN_WIRE 0x04U

#define AFE_CONVERSION_MS 2U
#define AFE_MAX_PAYLOAD 64U

typedef struct
{
    uint16_t cell_raw[AFE_SIM_CELL_COUNT];
    uint16_t temp_raw[AFE_SIM_TEMP_COUNT];
    uint16_t bal_mask;
    bool converting;
    uint32_t conversion_done_ms;
    bool wrong_id;
    bool force_crc_error;
    bool force_timeout;
    bool internal_fault;
    bool open_wire;
} AfeSimState_t;

static AfeSimState_t s_sim;

void AfeSim_Init(void)
{
    s_sim = (AfeSimState_t){0};
    for (uint8_t i = 0U; i < AFE_SIM_CELL_COUNT; ++i)
    {
        s_sim.cell_raw[i] = (uint16_t)(3300U * 10U);
    }
    for (uint8_t i = 0U; i < AFE_SIM_TEMP_COUNT; ++i)
    {
        s_sim.temp_raw[i] = Ntc_DeciCToRaw(250);
    }
}

void AfeSim_SetCellMv(uint8_t index, uint16_t millivolts)
{
    if (index < AFE_SIM_CELL_COUNT)
    {
        uint32_t raw = (uint32_t)millivolts * 10U;
        s_sim.cell_raw[index] = (uint16_t)Sat_U32(raw, 0xFFFFU);
    }
}

void AfeSim_SetTempDeciC(uint8_t index, int16_t deci_celsius)
{
    if (index < AFE_SIM_TEMP_COUNT)
    {
        s_sim.temp_raw[index] = Ntc_DeciCToRaw(deci_celsius);
    }
}

void AfeSim_SetWrongId(bool wrong)
{
    s_sim.wrong_id = wrong;
}

void AfeSim_SetForceCrcError(bool force)
{
    s_sim.force_crc_error = force;
}

void AfeSim_SetForceTimeout(bool force)
{
    s_sim.force_timeout = force;
}

void AfeSim_SetInternalFault(bool fault)
{
    s_sim.internal_fault = fault;
}

void AfeSim_SetOpenWire(bool open_wire)
{
    s_sim.open_wire = open_wire;
}

uint16_t AfeSim_GetBalMask(void)
{
    return s_sim.bal_mask;
}

bool AfeSim_IsConverting(void)
{
    return s_sim.converting;
}

uint16_t AfeSim_GetCellRaw(uint8_t index)
{
    return (index < AFE_SIM_CELL_COUNT) ? s_sim.cell_raw[index] : 0U;
}

static uint16_t GetRegister(uint16_t address)
{
    if (address == AFE_REG_DEVICE_ID)
    {
        return s_sim.wrong_id ? 0x00U : 0x5AU;
    }
    if (address == AFE_REG_STATUS)
    {
        uint16_t status = 0U;
        if (s_sim.converting && (s_sim.conversion_done_ms == 0U))
        {
            status |= AFE_STATUS_CONV_DONE;
        }
        if (s_sim.internal_fault)
        {
            status |= AFE_STATUS_INTERNAL_FAULT;
        }
        if (s_sim.open_wire)
        {
            status |= AFE_STATUS_OPEN_WIRE;
        }
        return status;
    }
    if ((address >= AFE_REG_CELL_BASE) && (address < (AFE_REG_CELL_BASE + (AFE_SIM_CELL_COUNT * 2U))))
    {
        uint16_t index = (uint16_t)((address - AFE_REG_CELL_BASE) / 2U);
        return s_sim.cell_raw[index];
    }
    if ((address >= AFE_REG_TEMP_BASE) && (address < (AFE_REG_TEMP_BASE + (AFE_SIM_TEMP_COUNT * 2U))))
    {
        uint16_t index = (uint16_t)((address - AFE_REG_TEMP_BASE) / 2U);
        return s_sim.temp_raw[index];
    }
    if (address == AFE_REG_BAL_CTRL)
    {
        return s_sim.bal_mask;
    }
    return 0U;
}

static bool CheckRequestCrc(const uint8_t *tx, uint16_t tx_len)
{
    if (tx_len < 3U)
    {
        return false;
    }

    bool is_write = ((tx[0] & 0x80U) != 0U);
    uint16_t expected_len = is_write ? (uint16_t)(tx[1] + 3U) : 3U;
    if (tx_len != expected_len)
    {
        return false;
    }

    uint16_t crc_index = is_write ? (uint16_t)(tx_len - 1U) : 2U;
    uint8_t expected = Crc8_Poly07(tx, (size_t)crc_index);
    return expected == tx[crc_index];
}

bool AfeSim_Transfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len, uint32_t now_ms)
{
    if ((tx == NULL) || (tx_len < 3U))
    {
        return false;
    }
    if (s_sim.force_timeout)
    {
        return false;
    }
    if (!CheckRequestCrc(tx, tx_len))
    {
        return false;
    }

    uint8_t command = tx[0];
    uint16_t address = (uint16_t)(command & 0x7FU);
    uint16_t length = tx[1];

    if ((command & 0x80U) != 0U)
    {
        /* Write transaction. */
        if ((uint32_t)(length + 3U) != (uint32_t)tx_len)
        {
            return false;
        }
        if (address == AFE_REG_ADC_CTRL)
        {
            if ((length == 1U) && (tx[2] == 0x01U))
            {
                s_sim.converting = true;
                s_sim.conversion_done_ms = now_ms + AFE_CONVERSION_MS;
            }
        }
        else if (address == AFE_REG_BAL_CTRL)
        {
            if (length == 2U)
            {
                s_sim.bal_mask = (uint16_t)(((uint16_t)tx[2] << 8U) | (uint16_t)tx[3]);
            }
        }
        else
        {
            s_sim.internal_fault = true;
        }
        return true;
    }

    /* Read transaction. */
    if ((rx == NULL) || (rx_len < (uint16_t)(length + 1U)) || (length > AFE_MAX_PAYLOAD))
    {
        return false;
    }

    if (s_sim.converting && (now_ms >= s_sim.conversion_done_ms))
    {
        s_sim.conversion_done_ms = 0U; /* done; status bit stays set until next start */
    }

    uint8_t data[AFE_MAX_PAYLOAD] = {0};
    if (length == 1U)
    {
        data[0] = (uint8_t)(GetRegister(address) & 0xFFU);
    }
    else
    {
        /* Multi-byte registers are big-endian, two bytes per register. */
        for (uint16_t i = 0U; (uint16_t)(i + 1U) < length; i = (uint16_t)(i + 2U))
        {
            uint16_t reg_value = GetRegister((uint16_t)(address + i));
            data[i] = (uint8_t)((reg_value >> 8U) & 0xFFU);
            data[i + 1U] = (uint8_t)(reg_value & 0xFFU);
        }
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        rx[i] = data[i];
    }
    uint8_t crc = Crc8_Poly07(data, (size_t)length);
    if (s_sim.force_crc_error)
    {
        crc = (uint8_t)(crc ^ 0xFFU);
    }
    rx[length] = crc;
    return true;
}
