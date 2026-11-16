#include "services/can_matrix.h"

#include "common/crc.h"
#include "services/can_signal.h"

#include <string.h>

#define SIG(start, len, sig_type, num, den, off, lo, hi) \
    { (uint8_t)(start), (uint8_t)(len), sig_type, num, den, off, lo, hi }

/* ------------------------------------------------------------------------- */
/* Signal definitions (single source of truth, mirrored in dbc/bms.dbc)      */
/* ------------------------------------------------------------------------- */
static const CanSignalDef_t S_PACK_VOLTAGE = SIG(0U, 16U, CAN_SIG_UNSIGNED, 10, 1, 0, 0, 655350);
static const CanSignalDef_t S_PACK_CURRENT = SIG(16U, 16U, CAN_SIG_SIGNED, 100, 1, 0, -3276800, 3276700);
static const CanSignalDef_t S_SOC = SIG(32U, 8U, CAN_SIG_UNSIGNED, 5, 1, 0, 0, 1000);
static const CanSignalDef_t S_BMS_STATE = SIG(40U, 4U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 15);
static const CanSignalDef_t S_ALIVE = SIG(44U, 4U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 15);

static const CanSignalDef_t S_CCL = SIG(0U, 16U, CAN_SIG_UNSIGNED, 100, 1, 0, 0, 6553500);
static const CanSignalDef_t S_DCL = SIG(16U, 16U, CAN_SIG_UNSIGNED, 100, 1, 0, 0, 6553500);
static const CanSignalDef_t S_CHARGE_VOLTAGE = SIG(32U, 16U, CAN_SIG_UNSIGNED, 10, 1, 0, 0, 655350);
static const CanSignalDef_t S_CHARGE_ENABLE = SIG(48U, 1U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 1);
static const CanSignalDef_t S_LIMITS_ALIVE = SIG(52U, 4U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 15);

static const CanSignalDef_t S_VMAX = SIG(0U, 16U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 65535);
static const CanSignalDef_t S_VMIN = SIG(16U, 16U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 65535);
static const CanSignalDef_t S_VMAX_IDX = SIG(32U, 8U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 255);
static const CanSignalDef_t S_VMIN_IDX = SIG(40U, 8U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 255);
static const CanSignalDef_t S_TMAX = SIG(48U, 8U, CAN_SIG_SIGNED, 1, 1, -40, -40, 215);
static const CanSignalDef_t S_TMIN = SIG(56U, 8U, CAN_SIG_SIGNED, 1, 1, -40, -40, 215);

static const CanSignalDef_t S_MUX = SIG(0U, 8U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 5);
static const CanSignalDef_t S_CELL_A = SIG(8U, 16U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 65535);
static const CanSignalDef_t S_CELL_B = SIG(24U, 16U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 65535);
static const CanSignalDef_t S_CELL_C = SIG(40U, 16U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 65535);

static const CanSignalDef_t S_CONTACTOR_REQ = SIG(0U, 2U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 3);
static const CanSignalDef_t S_BAL_ENABLE = SIG(2U, 1U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 1);
static const CanSignalDef_t S_SLEEP_REQ = SIG(3U, 1U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 1);
static const CanSignalDef_t S_FAULT_CLEAR_REQ = SIG(4U, 1U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 1);
static const CanSignalDef_t S_VCU_ALIVE = SIG(52U, 4U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 15);

static void ClearFrame(uint8_t data[CAN_DLC])
{
    (void)memset(data, 0, CAN_DLC);
}

bool CanMatrix_CheckCrc(const uint8_t data[CAN_DLC])
{
    if (data == NULL)
    {
        return false;
    }
    return data[7] == Crc8_SaeJ1850(data, 7U);
}

void CanMatrix_PackPackStatus(uint8_t data[CAN_DLC], const CanPackStatus_t *status, uint8_t alive)
{
    if ((data == NULL) || (status == NULL))
    {
        return;
    }
    ClearFrame(data);

    (void)CanSig_Pack(data, CAN_DLC, &S_PACK_VOLTAGE, (int32_t)status->pack_voltage_mv);
    (void)CanSig_Pack(data, CAN_DLC, &S_PACK_CURRENT, status->pack_current_ma);
    (void)CanSig_Pack(data, CAN_DLC, &S_SOC, (int32_t)status->soc_permille);
    (void)CanSig_Pack(data, CAN_DLC, &S_BMS_STATE, (int32_t)status->bms_state);
    (void)CanSig_Pack(data, CAN_DLC, &S_ALIVE, (int32_t)(alive & 0x0FU));

    data[7] = Crc8_SaeJ1850(data, 7U);
}

void CanMatrix_PackLimits(uint8_t data[CAN_DLC], const CanLimits_t *limits, uint8_t alive)
{
    if ((data == NULL) || (limits == NULL))
    {
        return;
    }
    ClearFrame(data);

    uint32_t ccl = (limits->ccl_ma > 6553500U) ? 6553500U : limits->ccl_ma;
    uint32_t dcl = (limits->dcl_ma > 6553500U) ? 6553500U : limits->dcl_ma;

    (void)CanSig_Pack(data, CAN_DLC, &S_CCL, (int32_t)ccl);
    (void)CanSig_Pack(data, CAN_DLC, &S_DCL, (int32_t)dcl);
    (void)CanSig_Pack(data, CAN_DLC, &S_CHARGE_VOLTAGE, (int32_t)limits->charge_voltage_mv);
    (void)CanSig_Pack(data, CAN_DLC, &S_CHARGE_ENABLE, limits->charge_enable ? 1 : 0);
    (void)CanSig_Pack(data, CAN_DLC, &S_LIMITS_ALIVE, (int32_t)(alive & 0x0FU));

    data[7] = Crc8_SaeJ1850(data, 7U);
}

void CanMatrix_PackCellStats(uint8_t data[CAN_DLC], const CanCellStats_t *stats)
{
    if ((data == NULL) || (stats == NULL))
    {
        return;
    }
    ClearFrame(data);

    (void)CanSig_Pack(data, CAN_DLC, &S_VMAX, (int32_t)stats->vmax_mv);
    (void)CanSig_Pack(data, CAN_DLC, &S_VMIN, (int32_t)stats->vmin_mv);
    (void)CanSig_Pack(data, CAN_DLC, &S_VMAX_IDX, (int32_t)stats->vmax_idx);
    (void)CanSig_Pack(data, CAN_DLC, &S_VMIN_IDX, (int32_t)stats->vmin_idx);
    (void)CanSig_Pack(data, CAN_DLC, &S_TMAX, (int32_t)stats->tmax_c);
    (void)CanSig_Pack(data, CAN_DLC, &S_TMIN, (int32_t)stats->tmin_c);

    /* 0x352 deliberately has no CRC and no alive counter: it is a low-rate
     * informative frame and an independent watchdog covers its loss. */
}

void CanMatrix_PackCellVoltages(uint8_t data[CAN_DLC], const CanCellVoltages_t *cells)
{
    if ((data == NULL) || (cells == NULL))
    {
        return;
    }
    ClearFrame(data);

    (void)CanSig_Pack(data, CAN_DLC, &S_MUX, (int32_t)(cells->mux % CAN_CELL_MUX_COUNT));
    (void)CanSig_Pack(data, CAN_DLC, &S_CELL_A, (int32_t)cells->cell_mv[0]);
    (void)CanSig_Pack(data, CAN_DLC, &S_CELL_B, (int32_t)cells->cell_mv[1]);
    (void)CanSig_Pack(data, CAN_DLC, &S_CELL_C, (int32_t)cells->cell_mv[2]);
    data[7] = (uint8_t)(cells->balance_bits & 0x07U);
}

void CanMatrix_PackFaults(uint8_t data[CAN_DLC], const CanFaults_t *faults)
{
    if ((data == NULL) || (faults == NULL))
    {
        return;
    }
    ClearFrame(data);

    /* These are bit fields, not scaled measurements, so they are written raw. */
    CanSig_WriteRaw(data, CAN_DLC, 0U, 32U, (uint64_t)faults->fault_bits);
    CanSig_WriteRaw(data, CAN_DLC, 32U, 16U, (uint64_t)faults->warning_bits);
    CanSig_WriteRaw(data, CAN_DLC, 48U, 8U, (uint64_t)faults->active_dtc_count);
}

void CanMatrix_PackVcuCommand(uint8_t data[CAN_DLC], const CanVcuCommand_t *command)
{
    if ((data == NULL) || (command == NULL))
    {
        return;
    }
    ClearFrame(data);

    (void)CanSig_Pack(data, CAN_DLC, &S_CONTACTOR_REQ, (int32_t)(command->contactor_req & 0x03U));
    (void)CanSig_Pack(data, CAN_DLC, &S_BAL_ENABLE, command->balancing_enable ? 1 : 0);
    (void)CanSig_Pack(data, CAN_DLC, &S_SLEEP_REQ, command->sleep_req ? 1 : 0);
    (void)CanSig_Pack(data, CAN_DLC, &S_FAULT_CLEAR_REQ, command->fault_clear_req ? 1 : 0);
    (void)CanSig_Pack(data, CAN_DLC, &S_VCU_ALIVE, (int32_t)(command->alive_counter & 0x0FU));

    data[7] = Crc8_SaeJ1850(data, 7U);
}

bool CanMatrix_UnpackPackStatus(const uint8_t data[CAN_DLC], CanPackStatus_t *status)
{
    if ((data == NULL) || (status == NULL))
    {
        return false;
    }

    int32_t value = 0;
    (void)CanSig_Unpack(data, CAN_DLC, &S_PACK_VOLTAGE, &value);
    status->pack_voltage_mv = (uint16_t)((value < 0) ? 0 : value);
    (void)CanSig_Unpack(data, CAN_DLC, &S_PACK_CURRENT, &status->pack_current_ma);
    (void)CanSig_Unpack(data, CAN_DLC, &S_SOC, &value);
    status->soc_permille = (uint16_t)((value < 0) ? 0 : value);
    (void)CanSig_Unpack(data, CAN_DLC, &S_BMS_STATE, &value);
    status->bms_state = (uint8_t)((value < 0) ? 0 : value);
    return true;
}

bool CanMatrix_UnpackLimits(const uint8_t data[CAN_DLC], CanLimits_t *limits)
{
    if ((data == NULL) || (limits == NULL))
    {
        return false;
    }

    int32_t value = 0;
    (void)CanSig_Unpack(data, CAN_DLC, &S_CCL, &value);
    limits->ccl_ma = (value < 0) ? 0U : (uint32_t)value;
    (void)CanSig_Unpack(data, CAN_DLC, &S_DCL, &value);
    limits->dcl_ma = (value < 0) ? 0U : (uint32_t)value;
    (void)CanSig_Unpack(data, CAN_DLC, &S_CHARGE_VOLTAGE, &value);
    limits->charge_voltage_mv = (uint16_t)((value < 0) ? 0 : value);
    (void)CanSig_Unpack(data, CAN_DLC, &S_CHARGE_ENABLE, &value);
    limits->charge_enable = (value != 0);
    return true;
}

bool CanMatrix_UnpackCellVoltages(const uint8_t data[CAN_DLC], CanCellVoltages_t *cells)
{
    if ((data == NULL) || (cells == NULL))
    {
        return false;
    }

    int32_t value = 0;
    (void)CanSig_Unpack(data, CAN_DLC, &S_MUX, &value);
    cells->mux = (uint8_t)((value < 0) ? 0 : value);
    (void)CanSig_Unpack(data, CAN_DLC, &S_CELL_A, &value);
    cells->cell_mv[0] = (uint16_t)((value < 0) ? 0 : value);
    (void)CanSig_Unpack(data, CAN_DLC, &S_CELL_B, &value);
    cells->cell_mv[1] = (uint16_t)((value < 0) ? 0 : value);
    (void)CanSig_Unpack(data, CAN_DLC, &S_CELL_C, &value);
    cells->cell_mv[2] = (uint16_t)((value < 0) ? 0 : value);
    cells->balance_bits = (uint8_t)(data[7] & 0x07U);
    return true;
}

CanVcuFrameResult_t CanMatrix_UnpackVcuCommand(const uint8_t *data, uint8_t dlc, CanVcuCommand_t *command)
{
    if ((data == NULL) || (command == NULL))
    {
        return VCU_FRAME_BAD_DLC;
    }
    if (dlc != CAN_DLC)
    {
        return VCU_FRAME_BAD_DLC;
    }
    if (!CanMatrix_CheckCrc(data))
    {
        return VCU_FRAME_BAD_CRC;
    }

    int32_t value = 0;
    (void)CanSig_Unpack(data, CAN_DLC, &S_CONTACTOR_REQ, &value);
    command->contactor_req = (uint8_t)((value < 0) ? 0 : value);
    (void)CanSig_Unpack(data, CAN_DLC, &S_BAL_ENABLE, &value);
    command->balancing_enable = (value != 0);
    (void)CanSig_Unpack(data, CAN_DLC, &S_SLEEP_REQ, &value);
    command->sleep_req = (value != 0);
    (void)CanSig_Unpack(data, CAN_DLC, &S_FAULT_CLEAR_REQ, &value);
    command->fault_clear_req = (value != 0);
    (void)CanSig_Unpack(data, CAN_DLC, &S_VCU_ALIVE, &value);
    command->alive_counter = (uint8_t)((value < 0) ? 0 : value);
    return VCU_FRAME_OK;
}

/* ------------------------------------------------------------------------- */
/* VCU command supervision                                                   */
/* ------------------------------------------------------------------------- */
void VcuMonitor_Init(VcuMonitor_t *mon, uint32_t now_ms)
{
    if (mon == NULL)
    {
        return;
    }
    (void)memset(mon, 0, sizeof(*mon));
    mon->last_valid_ms = now_ms;
    mon->fault = true; /* no VCU traffic yet */
}

static void NoteInvalid(VcuMonitor_t *mon)
{
    mon->consecutive_invalid++;
    if (mon->consecutive_invalid >= CAN_VCU_MAX_INVALID_STREAK)
    {
        mon->fault = true;
    }
}

bool VcuMonitor_Update(VcuMonitor_t *mon, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    if (mon == NULL)
    {
        return false;
    }

    if (dlc != CAN_DLC)
    {
        mon->dlc_errors++;
        NoteInvalid(mon);
        return false;
    }

    CanVcuCommand_t command;
    CanVcuFrameResult_t result = CanMatrix_UnpackVcuCommand(data, dlc, &command);
    if (result == VCU_FRAME_BAD_CRC)
    {
        mon->crc_errors++;
        NoteInvalid(mon);
        return false;
    }
    if (result != VCU_FRAME_OK)
    {
        mon->dlc_errors++;
        NoteInvalid(mon);
        return false;
    }

    if (mon->have_last)
    {
        uint8_t diff = (uint8_t)((command.alive_counter - mon->last_alive) & 0x0FU);
        if (diff == 0U)
        {
            mon->alive_errors++;
            NoteInvalid(mon);
            return false;
        }
        if (diff > CAN_VCU_MAX_ALIVE_JUMP)
        {
            mon->jump_errors++;
            NoteInvalid(mon);
            return false;
        }
    }

    mon->last_alive = command.alive_counter;
    mon->last_command = command;
    mon->have_last = true;
    mon->valid_seen = true;
    mon->consecutive_invalid = 0U;
    mon->last_valid_ms = now_ms;
    mon->fault = false;
    return true;
}

void VcuMonitor_Poll(VcuMonitor_t *mon, uint32_t now_ms)
{
    if (mon == NULL)
    {
        return;
    }

    if (!mon->valid_seen)
    {
        mon->fault = true;
        return;
    }

    if ((now_ms - mon->last_valid_ms) > CAN_VCU_TIMEOUT_MS)
    {
        if (!mon->fault)
        {
            mon->timeout_events++;
        }
        mon->fault = true;
    }
}

bool VcuMonitor_IsFault(const VcuMonitor_t *mon)
{
    return (mon == NULL) ? true : mon->fault;
}
