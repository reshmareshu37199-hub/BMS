#include "app/app.h"

#include "app/scheduler.h"
#include "common/saturate.h"
#include "drivers/can_driver.h"
#include "hal/hal.h"
#include "services/isotp.h"
#include "services/nvm.h"
#include "services/soc.h"
#include "services/uds.h"

#include <string.h>

#define APP_DTC_CLEAR_ALL 0x00FFFFFFUL
#define APP_CHARGE_CURRENT_THRESHOLD_MA 500
#define APP_DISCHARGE_CURRENT_THRESHOLD_MA (-500)

static SocState_t s_soc;
static UdsServer_t s_uds;
static IsoTp_t s_isotp;
static VcuMonitor_t s_vcu;
static NvmParams_t s_params;
static AppSnapshot_t s_snapshot;

static uint8_t s_alive_350;
static uint8_t s_alive_351;
static uint8_t s_mux_353;

static bool s_params_dirty;
static bool s_clear_request;
static bool s_prev_fault_clear;
static bool s_precharge_pending;
static bool s_faults_initialised;
static bool s_faults_dirty;

static uint32_t s_last_params_save_ms;
static uint32_t s_last_fault_bits;
static uint16_t s_last_warning_bits;
static uint8_t s_last_dtc_count;

/* ------------------------------------------------------------------------- */
/* UDS bindings                                                              */
/* ------------------------------------------------------------------------- */
static const char s_sw_version[] = "BMS-FW-1.0.0";

static uint16_t Uds_ReadPackVoltageMv(void)
{
    return Hal_PackVoltageMv();
}

static int32_t Uds_ReadPackCurrentMa(void)
{
    return Hal_PackCurrentMa();
}

static bool Uds_ReadCellMv(uint16_t *cells, uint8_t count)
{
    if ((cells == NULL) || (count == 0U))
    {
        return false;
    }

    AfeSnapshot_t snap;
    if (!Afe_GetSnapshot(&snap) || (snap.sequence == 0U) || !snap.valid)
    {
        return false;
    }

    uint8_t copy = (count < AFE_CELL_COUNT) ? count : (uint8_t)AFE_CELL_COUNT;
    for (uint8_t i = 0U; i < copy; ++i)
    {
        cells[i] = snap.cell_mv[i];
    }
    for (uint8_t i = copy; i < count; ++i)
    {
        cells[i] = 0U;
    }
    return true;
}

static bool Uds_WriteBalStart(uint16_t mv)
{
    const BalConfig_t *config = Bal_GetConfig();
    if ((config == NULL) || !Bal_SetStartStop(mv, config->stop_mv))
    {
        return false;
    }
    s_params.bal_start_mv = mv;
    s_params_dirty = true;
    return true;
}

static bool Uds_WriteBalStop(uint16_t mv)
{
    const BalConfig_t *config = Bal_GetConfig();
    if ((config == NULL) || !Bal_SetStartStop(config->start_mv, mv))
    {
        return false;
    }
    s_params.bal_stop_mv = mv;
    s_params_dirty = true;
    return true;
}

static uint8_t Uds_DtcCount(void)
{
    return Prot_GetDtcCount();
}

static bool Uds_DtcByIndex(uint8_t index, uint32_t *dtc, uint8_t *status)
{
    return Prot_GetDtcByIndex(index, dtc, status);
}

static void Uds_ClearDtcs(void)
{
    /* Deferred to Task_Fast so that the AFE latch is released before the
     * protection layer re-evaluates the release conditions. */
    s_clear_request = true;
}

static uint8_t Uds_DtcStatusMask(void)
{
    return 0xFFU;
}

static const UdsHooks_t s_uds_hooks = {
    .sw_version = s_sw_version,
    .read_pack_voltage_mv = Uds_ReadPackVoltageMv,
    .read_pack_current_ma = Uds_ReadPackCurrentMa,
    .read_cell_mv = Uds_ReadCellMv,
    .write_bal_start = Uds_WriteBalStart,
    .write_bal_stop = Uds_WriteBalStop,
    .dtc_count = Uds_DtcCount,
    .dtc_by_index = Uds_DtcByIndex,
    .clear_dtcs = Uds_ClearDtcs,
    .dtc_status_mask = Uds_DtcStatusMask,
};

/* ------------------------------------------------------------------------- */
/* CAN plumbing                                                              */
/* ------------------------------------------------------------------------- */
static bool IsoTp_Send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    return CanDrv_Queue(id, data, dlc);
}

static void OnVcuFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    (void)id;
    (void)VcuMonitor_Update(&s_vcu, data, dlc, Hal_TimeMs());
}

static void OnUdsFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    (void)id;
    (void)IsoTp_OnFrame(&s_isotp, data, dlc, Hal_TimeMs());
}

static void SendPackStatus(void)
{
    CanPackStatus_t status;
    (void)memset(&status, 0, sizeof(status));

    status.pack_voltage_mv = Hal_PackVoltageMv();
    status.pack_current_ma = Hal_PackCurrentMa();

    int32_t soc = Soc_GetPermille(&s_soc);
    if (soc < 0)
    {
        soc = 0;
    }
    status.soc_permille = (uint16_t)Sat_U32((uint64_t)soc, 1000U);
    status.bms_state = (uint8_t)BmsState_GetOutput().state;

    uint8_t data[CAN_DLC];
    CanMatrix_PackPackStatus(data, &status, s_alive_350);
    if (CanDrv_Queue(CAN_ID_BMS_PACK_STATUS, data, CAN_DLC))
    {
        s_alive_350 = (uint8_t)((s_alive_350 + 1U) & 0x0FU);
    }
}

static void SendLimits(void)
{
    ProtOutput_t prot = Prot_GetOutput();
    CanLimits_t limits;
    (void)memset(&limits, 0, sizeof(limits));

    limits.ccl_ma = (prot.ccl_ma > 0) ? (uint32_t)prot.ccl_ma : 0U;
    limits.dcl_ma = (prot.dcl_ma > 0) ? (uint32_t)prot.dcl_ma : 0U;
    limits.charge_voltage_mv = APP_CHARGE_VOLTAGE_REQ_MV;
    limits.charge_enable = prot.charge_enable;

    uint8_t data[CAN_DLC];
    CanMatrix_PackLimits(data, &limits, s_alive_351);
    if (CanDrv_Queue(CAN_ID_BMS_LIMITS, data, CAN_DLC))
    {
        s_alive_351 = (uint8_t)((s_alive_351 + 1U) & 0x0FU);
    }
}

static void SendCellStats(void)
{
    AfeSnapshot_t snap;
    (void)Afe_GetSnapshot(&snap);

    CanCellStats_t stats;
    (void)memset(&stats, 0, sizeof(stats));

    if ((snap.sequence > 0U) && snap.valid)
    {
        stats.vmax_mv = snap.cell_mv[0];
        stats.vmin_mv = snap.cell_mv[0];
        stats.vmax_idx = 0U;
        stats.vmin_idx = 0U;
        for (uint8_t i = 1U; i < AFE_CELL_COUNT; ++i)
        {
            if (snap.cell_mv[i] > stats.vmax_mv)
            {
                stats.vmax_mv = snap.cell_mv[i];
                stats.vmax_idx = i;
            }
            if (snap.cell_mv[i] < stats.vmin_mv)
            {
                stats.vmin_mv = snap.cell_mv[i];
                stats.vmin_idx = i;
            }
        }

        int16_t tmax = snap.temp_dc[0];
        int16_t tmin = snap.temp_dc[0];
        for (uint8_t i = 1U; i < APP_CELL_TEMP_COUNT; ++i)
        {
            if (snap.temp_dc[i] > tmax)
            {
                tmax = snap.temp_dc[i];
            }
            if (snap.temp_dc[i] < tmin)
            {
                tmin = snap.temp_dc[i];
            }
        }
        stats.tmax_c = tmax;
        stats.tmin_c = tmin;
    }

    uint8_t data[CAN_DLC];
    CanMatrix_PackCellStats(data, &stats);
    (void)CanDrv_Queue(CAN_ID_BMS_CELL_STATS, data, CAN_DLC);
}

static void SendCellVoltages(void)
{
    AfeSnapshot_t snap;
    (void)Afe_GetSnapshot(&snap);
    bool valid = (snap.sequence > 0U) && snap.valid;

    CanCellVoltages_t cells;
    (void)memset(&cells, 0, sizeof(cells));

    cells.mux = s_mux_353;
    uint8_t base = (uint8_t)(s_mux_353 * 3U);
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        uint8_t index = (uint8_t)(base + i);
        cells.cell_mv[i] = (valid && (index < AFE_CELL_COUNT)) ? snap.cell_mv[index] : 0U;
    }

    BalStatus_t bal = Bal_GetStatus();
    cells.balance_bits = (uint8_t)((bal.mask >> base) & 0x07U);

    uint8_t data[CAN_DLC];
    CanMatrix_PackCellVoltages(data, &cells);
    if (CanDrv_Queue(CAN_ID_BMS_CELL_VOLTAGES, data, CAN_DLC))
    {
        s_mux_353 = (uint8_t)((s_mux_353 + 1U) % CAN_CELL_MUX_COUNT);
    }
}

static void SendFaults(void)
{
    ProtOutput_t prot = Prot_GetOutput();
    CanFaults_t faults;
    faults.fault_bits = prot.fault_bits;
    faults.warning_bits = prot.warning_bits;
    faults.active_dtc_count = prot.active_dtc_count;

    uint8_t data[CAN_DLC];
    CanMatrix_PackFaults(data, &faults);
    (void)CanDrv_Queue(CAN_ID_BMS_FAULTS, data, CAN_DLC);
    s_faults_dirty = false;
}

/* ------------------------------------------------------------------------- */
/* Housekeeping                                                              */
/* ------------------------------------------------------------------------- */
static void PersistParameters(uint32_t now_ms, bool force)
{
    const BalConfig_t *config = Bal_GetConfig();
    if (config == NULL)
    {
        return;
    }

    if (!force && !s_params_dirty && ((now_ms - s_last_params_save_ms) < APP_SOC_SAVE_PERIOD_MS))
    {
        return;
    }

    s_params.bal_start_mv = config->start_mv;
    s_params.bal_stop_mv = config->stop_mv;
    s_params.soc_permille = (int16_t)Sat_I32((int64_t)Soc_GetPermille(&s_soc), 0, 1000);
    s_params.flags = 0U;

    if (Nvm_Save(&s_params))
    {
        s_params_dirty = false;
        s_last_params_save_ms = now_ms;
    }
}

static void UpdateBalancing(const AfeSnapshot_t *snap)
{
    BalInput_t input;
    (void)memset(&input, 0, sizeof(input));

    bool meas_valid = (snap->sequence > 0U) && snap->valid;
    input.meas_valid = meas_valid;
    for (uint8_t i = 0U; i < BAL_CELL_COUNT; ++i)
    {
        input.cell_mv[i] = snap->cell_mv[i];
    }

    int16_t max_temp = 0;
    if (meas_valid)
    {
        max_temp = snap->temp_dc[0];
        for (uint8_t i = 1U; i < APP_CELL_TEMP_COUNT; ++i)
        {
            if (snap->temp_dc[i] > max_temp)
            {
                max_temp = snap->temp_dc[i];
            }
        }
    }
    input.max_cell_temp_dc = max_temp;
    input.board_temp_dc = Hal_BoardTempDc();
    input.pack_current_mA = Hal_PackCurrentMa();
    input.vcu_balancing_enable = (!VcuMonitor_IsFault(&s_vcu)) && s_vcu.last_command.balancing_enable;

    ProtOutput_t prot = Prot_GetOutput();
    input.any_fault_active = prot.warning_active || prot.contactor_open_request;

    Bal_SetMeasurementWindow(false);
    uint16_t mask = Bal_Run(&input);

    if (!Afe_IsBusy())
    {
        (void)Afe_SetBalMask(mask);
    }
}

static void UpdateSnapshot(const AfeSnapshot_t *snap,
                           bool meas_valid,
                           bool vcu_valid,
                           const ProtOutput_t *prot,
                           const BmsStateOutput_t *state_out)
{
    s_snapshot.state = state_out->state;
    s_snapshot.state_name = BmsState_Name(state_out->state);
    s_snapshot.contactor_neg = state_out->close_neg;
    s_snapshot.contactor_pre = state_out->close_pre;
    s_snapshot.contactor_pos = state_out->close_pos;
    s_snapshot.pack_voltage_mv = Hal_PackVoltageMv();
    s_snapshot.bus_voltage_mv = Hal_BusVoltageMv();
    s_snapshot.pack_current_ma = Hal_PackCurrentMa();
    s_snapshot.soc_permille = Soc_GetPermille(&s_soc);
    s_snapshot.meas_valid = meas_valid;

    for (uint8_t i = 0U; i < AFE_CELL_COUNT; ++i)
    {
        s_snapshot.cell_mv[i] = snap->cell_mv[i];
    }
    for (uint8_t i = 0U; i < APP_CELL_TEMP_COUNT; ++i)
    {
        s_snapshot.cell_temp_dc[i] = snap->temp_dc[i];
    }
    s_snapshot.board_temp_dc = Hal_BoardTempDc();

    BalStatus_t bal = Bal_GetStatus();
    s_snapshot.bal_mask = bal.mask;
    s_snapshot.bal_active = bal.active;
    s_snapshot.bal_active_count = bal.active_count;
    s_snapshot.bal_inhibit = (uint8_t)bal.inhibit;
    s_snapshot.bal_active_time_ms = bal.active_time_ms;

    s_snapshot.vcu_valid = vcu_valid;
    s_snapshot.vcu_contactor_req = s_vcu.last_command.contactor_req;
    s_snapshot.vcu_bal_enable = s_vcu.last_command.balancing_enable;
    s_snapshot.vcu_sleep_req = s_vcu.last_command.sleep_req;
    s_snapshot.vcu_alive = s_vcu.last_command.alive_counter;

    s_snapshot.ccl_ma = prot->ccl_ma;
    s_snapshot.dcl_ma = prot->dcl_ma;
    s_snapshot.fault_bits = prot->fault_bits;
    s_snapshot.warning_bits = prot->warning_bits;
    s_snapshot.active_dtc_count = prot->active_dtc_count;
    s_snapshot.state_transitions = state_out->transitions;

    CanDrvStats_t stats = CanDrv_GetStats();
    s_snapshot.can_tx_queued = stats.tx_queued;
    s_snapshot.can_tx_dropped = stats.tx_dropped;
    s_snapshot.sched_executions = Sched_GetExecutions();
    s_snapshot.sched_overruns = Sched_GetOverruns();
}

/* ------------------------------------------------------------------------- */
/* Periodic tasks                                                            */
/* ------------------------------------------------------------------------- */
static void Task_Tick(uint32_t now_ms)
{
    if (Afe_Poll(now_ms))
    {
        AfeSnapshot_t snap;
        if (Afe_GetSnapshot(&snap))
        {
            UpdateBalancing(&snap);
        }
    }

    CanDrv_Poll();
    Hal_WatchdogKick();
}

static void Task_Fast(uint32_t now_ms)
{
    AfeSnapshot_t snap;
    (void)Afe_GetSnapshot(&snap);
    bool have_snapshot = (snap.sequence > 0U);
    bool meas_valid = have_snapshot && snap.valid;

    VcuMonitor_Poll(&s_vcu, now_ms);
    bool vcu_valid = !VcuMonitor_IsFault(&s_vcu);

    /* The VCU FaultClearReq bit is level-based on the bus, so act only on the
     * rising edge; otherwise a held request would keep clearing latched DTCs. */
    bool clear_req = vcu_valid && s_vcu.last_command.fault_clear_req;
    if (clear_req && !s_prev_fault_clear)
    {
        s_clear_request = true;
    }
    s_prev_fault_clear = clear_req;

    if (s_clear_request)
    {
        Afe_ClearLatchedFault();
    }

    int32_t current_ma = Hal_PackCurrentMa();
    bool afe_comm = Afe_HasCommFault() || (have_snapshot && (!snap.valid || snap.open_wire));

    ProtInput_t prot_in;
    (void)memset(&prot_in, 0, sizeof(prot_in));
    prot_in.cells_valid = meas_valid;
    for (uint8_t i = 0U; i < PROT_CELL_COUNT; ++i)
    {
        prot_in.cell_mv[i] = snap.cell_mv[i];
    }
    for (uint8_t i = 0U; i < APP_CELL_TEMP_COUNT; ++i)
    {
        prot_in.temp_dc[i] = snap.temp_dc[i];
    }
    /* The fifth AFE channel is reserved; duplicating cell 1 keeps the
     * protection min/max scan limited to the four cell sensors. */
    prot_in.temp_dc[APP_CELL_TEMP_COUNT] = (meas_valid ? snap.temp_dc[0] : 0);
    prot_in.board_temp_dc = Hal_BoardTempDc();
    prot_in.current_ma = current_ma;
    prot_in.charging = (current_ma > APP_CHARGE_CURRENT_THRESHOLD_MA);
    prot_in.discharging = (current_ma < APP_DISCHARGE_CURRENT_THRESHOLD_MA);
    prot_in.afe_comm_fault = afe_comm;
    prot_in.vcu_timeout = !vcu_valid;
    prot_in.cell_sum_mismatch = have_snapshot && snap.sum_mismatch;
    prot_in.precharge_timeout = s_precharge_pending;

    Prot_Update(&prot_in, APP_CONTROL_PERIOD_MS);
    s_precharge_pending = false;

    if (s_clear_request)
    {
        Prot_ClearDtc(APP_DTC_CLEAR_ALL);
        s_clear_request = false;
    }

    ProtOutput_t prot = Prot_GetOutput();

    bool fault_active = false;
    for (uint8_t id = 0U; id < (uint8_t)FAULT_COUNT; ++id)
    {
        if (id == (uint8_t)FAULT_VCU_TIMEOUT)
        {
            continue;
        }
        if (Prot_IsFaultActive((FaultId)id))
        {
            fault_active = true;
            break;
        }
    }

    BmsStateInput_t state_in;
    (void)memset(&state_in, 0, sizeof(state_in));
    state_in.afe_ok = have_snapshot && snap.valid;
    state_in.afe_fault = afe_comm;
    state_in.meas_valid = meas_valid;
    state_in.fault_active = fault_active;
    state_in.fault_latched = prot.latched_fault_active;
    state_in.open_request = prot.contactor_open_request;
    state_in.open_immediate = prot.contactor_open_immediate;
    state_in.vcu_valid = vcu_valid;
    state_in.vcu_close_request = vcu_valid && (s_vcu.last_command.contactor_req == 1U);
    state_in.charger_present = Hal_ChargerPresent();
    state_in.sleep_request = vcu_valid && s_vcu.last_command.sleep_req;
    state_in.pack_voltage_mv = Hal_PackVoltageMv();
    state_in.bus_voltage_mv = Hal_BusVoltageMv();

    BmsState_Update(&state_in, now_ms);
    BmsStateOutput_t state_out = BmsState_GetOutput();

    if (state_out.precharge_failed)
    {
        s_precharge_pending = true;
    }

    Hal_ContactorSet(HAL_CONTACTOR_NEG, state_out.close_neg);
    Hal_ContactorSet(HAL_CONTACTOR_PRE, state_out.close_pre);
    Hal_ContactorSet(HAL_CONTACTOR_POS, state_out.close_pos);

    uint16_t ocv_mv = 0U;
    if (meas_valid)
    {
        ocv_mv = snap.cell_mv[0];
        for (uint8_t i = 1U; i < AFE_CELL_COUNT; ++i)
        {
            if (snap.cell_mv[i] < ocv_mv)
            {
                ocv_mv = snap.cell_mv[i];
            }
        }
    }
    Soc_Update(&s_soc, current_ma, ocv_mv, APP_CONTROL_PERIOD_MS);

    IsoTp_Poll(&s_isotp, now_ms);
    Uds_Poll(&s_uds, now_ms);
    if (!s_isotp.tx_active)
    {
        uint8_t request[ISOTP_MAX_PAYLOAD];
        uint16_t request_len = IsoTp_Read(&s_isotp, request, (uint16_t)sizeof(request));
        if (request_len > 0U)
        {
            uint8_t response[UDS_MAX_RESPONSE];
            uint16_t response_len = Uds_Process(&s_uds, request, request_len,
                                                response, (uint16_t)sizeof(response), now_ms);
            if (response_len > 0U)
            {
                (void)IsoTp_Write(&s_isotp, response, response_len, now_ms);
            }
        }
    }

    if (!s_faults_initialised || (prot.fault_bits != s_last_fault_bits) ||
        (prot.warning_bits != s_last_warning_bits) || (prot.active_dtc_count != s_last_dtc_count))
    {
        s_faults_dirty = true;
        s_last_fault_bits = prot.fault_bits;
        s_last_warning_bits = prot.warning_bits;
        s_last_dtc_count = prot.active_dtc_count;
        s_faults_initialised = true;
    }

    UpdateSnapshot(&snap, meas_valid, vcu_valid, &prot, &state_out);
}

static void Task_Main(uint32_t now_ms)
{
    /* All bleed switches must be off during a conversion (BAL_MEAS_PAUSE).
     * The mask computed from the previous snapshot is restored by
     * UpdateBalancing() as soon as the new snapshot is published. */
    Bal_SetMeasurementWindow(true);
    (void)Afe_SetBalMask(0U);
    Afe_StartCycle(now_ms);

    SendPackStatus();
    SendLimits();

    if (s_faults_dirty)
    {
        SendFaults();
    }

    if (s_params_dirty)
    {
        PersistParameters(now_ms, true);
    }
    else if ((now_ms - s_last_params_save_ms) >= APP_SOC_SAVE_PERIOD_MS)
    {
        PersistParameters(now_ms, false);
    }
}

static void Task_CellStats(uint32_t now_ms)
{
    (void)now_ms;
    SendCellStats();
}

static void Task_Slow(uint32_t now_ms)
{
    (void)now_ms;
    SendCellVoltages();
    SendFaults();
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */
void App_Init(void)
{
    uint32_t now = Hal_TimeMs();

    CanDrv_Init();
    Afe_Init();
    Prot_Init();
    Bal_Init(NULL);
    VcuMonitor_Init(&s_vcu, now);

    Nvm_Init();
    (void)Nvm_Load(&s_params);
    if (!Bal_SetStartStop(s_params.bal_start_mv, s_params.bal_stop_mv))
    {
        s_params.bal_start_mv = BAL_DEFAULT_START_MV;
        s_params.bal_stop_mv = BAL_DEFAULT_STOP_MV;
    }

    SocConfig_t soc_config;
    soc_config.capacity_mah = SOC_DEFAULT_CAPACITY_MAH;
    soc_config.efficiency_permille = SOC_DEFAULT_EFFICIENCY_PERMILLE;
    Soc_Init(&s_soc, &soc_config, (int32_t)s_params.soc_permille);

    BmsState_Init(now);

    IsoTp_Init(&s_isotp, UDS_REQUEST_ID, UDS_RESPONSE_ID, IsoTp_Send);
    Uds_Init(&s_uds, &s_uds_hooks, now);

    (void)CanDrv_RegisterHandler(CAN_ID_VCU_COMMAND, OnVcuFrame);
    (void)CanDrv_RegisterHandler(UDS_REQUEST_ID, OnUdsFrame);

    (void)memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.state_name = BmsState_Name(BMS_STATE_INIT);

    s_alive_350 = 0U;
    s_alive_351 = 0U;
    s_mux_353 = 0U;
    s_params_dirty = false;
    s_clear_request = false;
    s_prev_fault_clear = false;
    s_precharge_pending = false;
    s_faults_initialised = false;
    s_faults_dirty = false;
    s_last_params_save_ms = now;
    s_last_fault_bits = 0U;
    s_last_warning_bits = 0U;
    s_last_dtc_count = 0U;

    Sched_Init(now);
    (void)Sched_Register("tick", 1U, Task_Tick);
    (void)Sched_Register("fast", APP_CONTROL_PERIOD_MS, Task_Fast);
    (void)Sched_Register("main", APP_BALANCE_PERIOD_MS, Task_Main);
    (void)Sched_Register("cellstats", APP_CAN_STATS_PERIOD_MS, Task_CellStats);
    (void)Sched_Register("slow", APP_CAN_SLOW_PERIOD_MS, Task_Slow);
}

void App_Step(uint32_t now_ms)
{
    Sched_Run(now_ms);
}

void App_Run(void)
{
    for (;;)
    {
        App_Step(Hal_TimeMs());
    }
}

void App_GetSnapshot(AppSnapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }
    Hal_CriticalEnter();
    *out = s_snapshot;
    Hal_CriticalExit();
}

uint32_t App_GetCanAlive(void)
{
    return (uint32_t)s_alive_350;
}

int32_t App_GetSocPermille(void)
{
    return Soc_GetPermille(&s_soc);
}
