#include "services/protection.h"

#include "common/saturate.h"

#include <string.h>

#define FAULT_ACT_LIMIT_CHARGE 0x01U
#define FAULT_ACT_LIMIT_DISCHARGE 0x02U
#define FAULT_ACT_OPEN 0x04U
#define FAULT_ACT_OPEN_IMMEDIATE 0x08U

typedef bool (*FaultCondFn)(const ProtInput_t *in, int32_t threshold);

typedef struct
{
    FaultId id;
    uint32_t dtc;
    int32_t set_threshold;
    int32_t release_threshold;
    uint32_t debounce_ms;
    bool latched;
    uint8_t action;
    FaultCondFn set_fn;
    FaultCondFn release_fn;
    bool has_hard_threshold;
    int32_t hard_threshold;
    FaultCondFn hard_fn;
} FaultDef_t;

typedef struct
{
    bool active;
    bool pending;
    bool confirmed;
    bool release_ok;
    uint32_t timer_ms;
    uint8_t status;
} FaultState_t;

static FaultState_t s_faults[FAULT_COUNT];
static ProtOutput_t s_output;
static bool s_initialised;

/* ------------------------------------------------------------------------- */
/* Condition helpers                                                          */
/* ------------------------------------------------------------------------- */
static int16_t TempMax(const ProtInput_t *in)
{
    int16_t value = in->temp_dc[0];
    for (uint8_t i = 1U; i < PROT_TEMP_COUNT; ++i)
    {
        if (in->temp_dc[i] > value)
        {
            value = in->temp_dc[i];
        }
    }
    return value;
}

static int16_t TempMin(const ProtInput_t *in)
{
    int16_t value = in->temp_dc[0];
    for (uint8_t i = 1U; i < PROT_TEMP_COUNT; ++i)
    {
        if (in->temp_dc[i] < value)
        {
            value = in->temp_dc[i];
        }
    }
    return value;
}

static uint16_t CellMax(const ProtInput_t *in)
{
    uint16_t value = in->cell_mv[0];
    for (uint8_t i = 1U; i < PROT_CELL_COUNT; ++i)
    {
        if (in->cell_mv[i] > value)
        {
            value = in->cell_mv[i];
        }
    }
    return value;
}

static uint16_t CellMin(const ProtInput_t *in)
{
    uint16_t value = in->cell_mv[0];
    for (uint8_t i = 1U; i < PROT_CELL_COUNT; ++i)
    {
        if (in->cell_mv[i] < value)
        {
            value = in->cell_mv[i];
        }
    }
    return value;
}

static bool CondCellMaxGe(const ProtInput_t *in, int32_t threshold)
{
    return (int32_t)CellMax(in) >= threshold;
}

static bool CondCellMaxGt(const ProtInput_t *in, int32_t threshold)
{
    return (int32_t)CellMax(in) > threshold;
}

static bool CondAllCellsLe(const ProtInput_t *in, int32_t threshold)
{
    for (uint8_t i = 0U; i < PROT_CELL_COUNT; ++i)
    {
        if ((int32_t)in->cell_mv[i] > threshold)
        {
            return false;
        }
    }
    return true;
}

static bool CondCellMinLe(const ProtInput_t *in, int32_t threshold)
{
    return (int32_t)CellMin(in) <= threshold;
}

static bool CondAllCellsGe(const ProtInput_t *in, int32_t threshold)
{
    for (uint8_t i = 0U; i < PROT_CELL_COUNT; ++i)
    {
        if ((int32_t)in->cell_mv[i] < threshold)
        {
            return false;
        }
    }
    return true;
}

static bool CondTempMaxGe(const ProtInput_t *in, int32_t threshold)
{
    return (int32_t)TempMax(in) >= threshold;
}

static bool CondTempMaxLe(const ProtInput_t *in, int32_t threshold)
{
    return (int32_t)TempMax(in) <= threshold;
}

static bool CondTempMinGe(const ProtInput_t *in, int32_t threshold)
{
    return (int32_t)TempMin(in) >= threshold;
}

static bool CondChargingTempMaxGe(const ProtInput_t *in, int32_t threshold)
{
    return in->charging && ((int32_t)TempMax(in) >= threshold);
}

static bool CondChargingTempMinLe(const ProtInput_t *in, int32_t threshold)
{
    return in->charging && ((int32_t)TempMin(in) <= threshold);
}

static bool CondCurrentLe(const ProtInput_t *in, int32_t threshold)
{
    return in->current_ma <= threshold;
}

static bool CondCurrentGe(const ProtInput_t *in, int32_t threshold)
{
    return in->current_ma >= threshold;
}

static bool CondAfeComm(const ProtInput_t *in, int32_t threshold)
{
    (void)threshold;
    return in->afe_comm_fault;
}

static bool CondNotAfeComm(const ProtInput_t *in, int32_t threshold)
{
    (void)threshold;
    return !in->afe_comm_fault;
}

static bool CondVcuTimeout(const ProtInput_t *in, int32_t threshold)
{
    (void)threshold;
    return in->vcu_timeout;
}

static bool CondNotVcuTimeout(const ProtInput_t *in, int32_t threshold)
{
    (void)threshold;
    return !in->vcu_timeout;
}

static bool CondCellSum(const ProtInput_t *in, int32_t threshold)
{
    (void)threshold;
    return in->cell_sum_mismatch;
}

static bool CondNotCellSum(const ProtInput_t *in, int32_t threshold)
{
    (void)threshold;
    return !in->cell_sum_mismatch;
}

static bool CondPrecharge(const ProtInput_t *in, int32_t threshold)
{
    (void)threshold;
    return in->precharge_timeout;
}

/* ------------------------------------------------------------------------- */
/* Fault table                                                                */
/* ------------------------------------------------------------------------- */
static const FaultDef_t s_defs[FAULT_COUNT] = {
    {FAULT_CELL_OV, 0x0A01U, 3650, 3400, 1000U, true,
     FAULT_ACT_LIMIT_CHARGE | FAULT_ACT_OPEN, CondCellMaxGe, CondAllCellsLe,
     true, 3750, CondCellMaxGt},
    {FAULT_CELL_UV, 0x0A02U, 2500, 2900, 1000U, true,
     FAULT_ACT_LIMIT_DISCHARGE | FAULT_ACT_OPEN, CondCellMinLe, CondAllCellsGe,
     true, 2300, CondCellMinLe},
    {FAULT_CHARGE_OT, 0x0A03U, 550, 500, 2000U, false,
     FAULT_ACT_LIMIT_CHARGE, CondChargingTempMaxGe, CondTempMaxLe,
     false, 0, NULL},
    {FAULT_DISCHARGE_OT, 0x0A04U, 600, 550, 2000U, true,
     FAULT_ACT_OPEN, CondTempMaxGe, CondTempMaxLe,
     false, 0, NULL},
    {FAULT_CHARGE_UT, 0x0A05U, 0, 30, 2000U, false,
     FAULT_ACT_LIMIT_CHARGE, CondChargingTempMinLe, CondTempMinGe,
     false, 0, NULL},
    {FAULT_DISCHARGE_OC, 0x0A06U, -200000, 0, 500U, true,
     FAULT_ACT_OPEN | FAULT_ACT_OPEN_IMMEDIATE, CondCurrentLe, NULL,
     false, 0, NULL},
    {FAULT_CHARGE_OC, 0x0A07U, 100000, 0, 1000U, true,
     FAULT_ACT_OPEN | FAULT_ACT_OPEN_IMMEDIATE, CondCurrentGe, NULL,
     false, 0, NULL},
    {FAULT_AFE_COMM, 0x0A08U, 0, 0, 0U, true,
     FAULT_ACT_OPEN, CondAfeComm, CondNotAfeComm,
     false, 0, NULL},
    {FAULT_VCU_TIMEOUT, 0x0A09U, 0, 0, 0U, false,
     FAULT_ACT_OPEN, CondVcuTimeout, CondNotVcuTimeout,
     false, 0, NULL},
    {FAULT_CELL_SUM_MISMATCH, 0x0A0AU, 0, 0, 0U, false,
     FAULT_ACT_LIMIT_CHARGE | FAULT_ACT_LIMIT_DISCHARGE, CondCellSum, CondNotCellSum,
     false, 0, NULL},
    {FAULT_PRECHARGE, 0x0A0BU, 0, 0, 0U, true,
     FAULT_ACT_OPEN | FAULT_ACT_OPEN_IMMEDIATE, CondPrecharge, NULL,
     false, 0, NULL},
};

/* ------------------------------------------------------------------------- */
/* Derating and ramping                                                       */
/* ------------------------------------------------------------------------- */
static int32_t Linear(int32_t x, int32_t x0, int32_t x1, int32_t y0, int32_t y1)
{
    if (x <= x0)
    {
        return y0;
    }
    if (x >= x1)
    {
        return y1;
    }
    int64_t num = (int64_t)(x - x0) * (int64_t)(y1 - y0);
    int64_t den = (int64_t)(x1 - x0);
    return Sat_I32((int64_t)y0 + (num / den), INT32_MIN, INT32_MAX);
}

static int32_t ChargeTempLimit(int16_t t_dc)
{
    if (t_dc <= 0)
    {
        return 0;
    }
    if (t_dc < 50)
    {
        return Linear(t_dc, 0, 50, 0, 50000);
    }
    if (t_dc <= 450)
    {
        return 100000;
    }
    if (t_dc < 550)
    {
        return Linear(t_dc, 450, 550, 100000, 0);
    }
    return 0;
}

static int32_t DischargeTempLimit(int16_t t_dc)
{
    if (t_dc <= -200)
    {
        return 0;
    }
    if (t_dc < 0)
    {
        return Linear(t_dc, -200, 0, 0, 100000);
    }
    if (t_dc <= 450)
    {
        return 200000;
    }
    if (t_dc < 600)
    {
        return Linear(t_dc, 450, 600, 200000, 0);
    }
    return 0;
}

static int32_t ChargeVoltageLimit(uint16_t vmax_mv)
{
    if (vmax_mv >= 3650U)
    {
        return 0;
    }
    if (vmax_mv <= 3550U)
    {
        return 100000;
    }
    return Linear((int32_t)vmax_mv, 3550, 3650, 100000, 0);
}

static int32_t DischargeVoltageLimit(uint16_t vmin_mv)
{
    if (vmin_mv <= 2500U)
    {
        return 0;
    }
    if (vmin_mv >= 3000U)
    {
        return 200000;
    }
    return Linear((int32_t)vmin_mv, 2500, 3000, 0, 200000);
}

static int32_t Ramp(int32_t current, int32_t target, uint32_t dt_ms)
{
    /* 10 A per 100 ms = 100 mA per ms. */
    int32_t max_delta = Sat_I32((int64_t)100 * (int64_t)dt_ms, 0, 200000);
    if (target > current)
    {
        int32_t next = current + max_delta;
        return (next > target) ? target : next;
    }
    if (target < current)
    {
        int32_t next = current - max_delta;
        return (next < target) ? target : next;
    }
    return current;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */
void Prot_Init(void)
{
    (void)memset(s_faults, 0, sizeof(s_faults));
    (void)memset(&s_output, 0, sizeof(s_output));
    s_initialised = true;
}

void Prot_Update(const ProtInput_t *in, uint32_t dt_ms)
{
    if (!s_initialised)
    {
        Prot_Init();
    }
    if (in == NULL)
    {
        return;
    }

    uint32_t fault_bits = 0U;
    uint16_t warning_bits = 0U;
    uint8_t active_dtc_count = 0U;
    bool limit_charge = false;
    bool limit_discharge = false;
    bool open_request = false;
    bool open_immediate = false;
    bool latched = false;
    bool warning = false;

    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; ++i)
    {
        const FaultDef_t *def = &s_defs[i];
        FaultState_t *state = &s_faults[i];

        bool set_now = def->set_fn(in, def->set_threshold);
        bool release_now = (def->release_fn != NULL) ? def->release_fn(in, def->release_threshold) : true;
        state->release_ok = release_now;

        if (!state->active)
        {
            if (set_now)
            {
                state->pending = true;
                state->timer_ms += dt_ms;
                if (state->timer_ms >= def->debounce_ms)
                {
                    state->active = true;
                    state->confirmed = true;
                    state->timer_ms = 0U;
                }
            }
            else
            {
                state->pending = false;
                state->timer_ms = 0U;
            }
        }
        else
        {
            if (!def->latched && release_now)
            {
                state->active = false;
                state->pending = false;
                state->timer_ms = 0U;
            }
        }

        uint8_t status = 0U;
        if (state->active)
        {
            status |= DTC_STATUS_TEST_FAILED;
            status |= DTC_STATUS_TEST_FAILED_THIS_CYCLE;
            status |= DTC_STATUS_WARNING;
        }
        if (state->pending)
        {
            status |= DTC_STATUS_PENDING;
        }
        if (state->confirmed)
        {
            status |= DTC_STATUS_CONFIRMED;
            status |= DTC_STATUS_TEST_FAILED_SINCE_CLEAR;
        }
        state->status = status;

        if (state->active || state->pending || state->confirmed)
        {
            if (state->active)
            {
                fault_bits |= (uint32_t)(1UL << (uint32_t)def->id);
                warning_bits |= (uint16_t)(1U << (uint16_t)def->id);
                if (def->latched)
                {
                    latched = true;
                }
                warning = true;
            }
            if (state->active || state->confirmed)
            {
                active_dtc_count++;
            }
        }

        if (state->active)
        {
            if ((def->action & FAULT_ACT_LIMIT_CHARGE) != 0U)
            {
                limit_charge = true;
            }
            if ((def->action & FAULT_ACT_LIMIT_DISCHARGE) != 0U)
            {
                limit_discharge = true;
            }
            if ((def->action & FAULT_ACT_OPEN) != 0U)
            {
                open_request = true;
            }
            if ((def->action & FAULT_ACT_OPEN_IMMEDIATE) != 0U)
            {
                open_immediate = true;
            }
        }

        /* Emergency thresholds (e.g. 3750 mV cell voltage) must trip without
         * waiting for the debounce interval.  The DTC is still reported through
         * the normal pending/confirmed path, but the contactors are opened at
         * once to protect the cells. */
        if (def->has_hard_threshold && (def->hard_fn != NULL) &&
            def->hard_fn(in, def->hard_threshold))
        {
            open_request = true;
            open_immediate = true;
        }
    }

    int32_t ccl_target = 0;
    int32_t dcl_target = 0;
    if (in->cells_valid)
    {
        ccl_target = ChargeTempLimit(TempMax(in));
        int32_t v_limit = ChargeVoltageLimit(CellMax(in));
        if (v_limit < ccl_target)
        {
            ccl_target = v_limit;
        }

        dcl_target = DischargeTempLimit(TempMax(in));
        int32_t dv_limit = DischargeVoltageLimit(CellMin(in));
        if (dv_limit < dcl_target)
        {
            dcl_target = dv_limit;
        }
    }

    if (limit_charge || open_request)
    {
        ccl_target = 0;
    }
    if (limit_discharge || open_request)
    {
        dcl_target = 0;
    }

    s_output.ccl_ma = Ramp(s_output.ccl_ma, ccl_target, dt_ms);
    s_output.dcl_ma = Ramp(s_output.dcl_ma, dcl_target, dt_ms);
    s_output.charge_enable = (s_output.ccl_ma > 0) && !open_request && !limit_charge;
    s_output.contactor_open_request = open_request;
    s_output.contactor_open_immediate = open_immediate;
    s_output.latched_fault_active = latched;
    s_output.warning_active = warning;
    s_output.fault_bits = fault_bits;
    s_output.warning_bits = warning_bits;
    s_output.active_dtc_count = active_dtc_count;
}

ProtOutput_t Prot_GetOutput(void)
{
    return s_output;
}

bool Prot_IsFaultActive(FaultId id)
{
    if ((uint32_t)id >= (uint32_t)FAULT_COUNT)
    {
        return false;
    }
    return s_faults[id].active;
}

uint8_t Prot_GetDtcStatus(FaultId id)
{
    if ((uint32_t)id >= (uint32_t)FAULT_COUNT)
    {
        return 0U;
    }
    return s_faults[id].status;
}

uint8_t Prot_GetDtcCount(void)
{
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; ++i)
    {
        if ((s_faults[i].confirmed) || (s_faults[i].active))
        {
            count++;
        }
    }
    return count;
}

bool Prot_GetDtcByIndex(uint8_t index, uint32_t *dtc, uint8_t *status)
{
    if ((dtc == NULL) || (status == NULL))
    {
        return false;
    }

    uint8_t seen = 0U;
    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; ++i)
    {
        if ((s_faults[i].confirmed) || (s_faults[i].active))
        {
            if (seen == index)
            {
                *dtc = s_defs[i].dtc;
                *status = s_faults[i].status;
                return true;
            }
            seen++;
        }
    }
    return false;
}

void Prot_ClearDtc(uint32_t group)
{
    if (group != 0x00FFFFFFUL)
    {
        return;
    }

    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; ++i)
    {
        FaultState_t *state = &s_faults[i];
        if ((!state->active) && (!state->confirmed))
        {
            continue;
        }
        if (state->active && (!state->release_ok))
        {
            continue;
        }
        state->active = false;
        state->pending = false;
        state->confirmed = false;
        state->timer_ms = 0U;
        state->status = 0U;
    }

    s_output.fault_bits = 0U;
    s_output.warning_bits = 0U;
    s_output.active_dtc_count = Prot_GetDtcCount();
}
