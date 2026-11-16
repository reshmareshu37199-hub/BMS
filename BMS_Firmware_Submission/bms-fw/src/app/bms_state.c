#include "app/bms_state.h"

#include <string.h>

static BmsState_t s_state;
static uint32_t s_entry_ms;
static uint32_t s_phase_ms;
static uint32_t s_precharge_start_ms;
static uint8_t s_precharge_phase;
static uint32_t s_transitions;
static bool s_fault_open;
static bool s_hold_neg;
static bool s_hold_pos;
static bool s_precharge_failed_pulse;
static uint32_t s_vcu_lost_ms;
static BmsStateOutput_t s_out;

static void EnterState(BmsState_t next, uint32_t now_ms)
{
    s_state = next;
    s_entry_ms = now_ms;
    s_phase_ms = now_ms;
    s_precharge_phase = 0U;
    s_vcu_lost_ms = 0U;
    s_transitions++;

    if (next == BMS_STATE_FAULT)
    {
        s_hold_neg = s_out.close_neg;
        s_hold_pos = s_out.close_pos;
        s_fault_open = false;
    }
    if (next == BMS_STATE_STANDBY)
    {
        s_precharge_failed_pulse = false;
    }

    s_out.state = s_state;
    s_out.transitions = s_transitions;
}

static void UpdateContactors(void)
{
    switch (s_state)
    {
    case BMS_STATE_INIT:
    case BMS_STATE_STANDBY:
    case BMS_STATE_SLEEP:
        s_out.close_neg = false;
        s_out.close_pre = false;
        s_out.close_pos = false;
        break;

    case BMS_STATE_PRECHARGE:
        s_out.close_neg = true;
        s_out.close_pre = (s_precharge_phase >= 1U) && (s_precharge_phase <= 3U);
        s_out.close_pos = (s_precharge_phase >= 3U);
        break;

    case BMS_STATE_DRIVE:
    case BMS_STATE_CHARGE:
        s_out.close_neg = true;
        s_out.close_pre = false;
        s_out.close_pos = true;
        break;

    case BMS_STATE_FAULT:
        if (s_fault_open)
        {
            s_out.close_neg = false;
            s_out.close_pre = false;
            s_out.close_pos = false;
        }
        else
        {
            s_out.close_neg = s_hold_neg;
            s_out.close_pre = false;
            s_out.close_pos = s_hold_pos;
        }
        break;

    default:
        s_out.close_neg = false;
        s_out.close_pre = false;
        s_out.close_pos = false;
        break;
    }
}

void BmsState_Init(uint32_t now_ms)
{
    (void)memset(&s_out, 0, sizeof(s_out));
    s_state = BMS_STATE_INIT;
    s_entry_ms = now_ms;
    s_phase_ms = now_ms;
    s_precharge_start_ms = now_ms;
    s_precharge_phase = 0U;
    s_transitions = 0U;
    s_fault_open = false;
    s_hold_neg = false;
    s_hold_pos = false;
    s_precharge_failed_pulse = false;
    s_vcu_lost_ms = 0U;
    s_out.state = s_state;
}

void BmsState_Update(const BmsStateInput_t *in, uint32_t now_ms)
{
    if (in == NULL)
    {
        return;
    }

    s_out.precharge_failed = false;

    /* Global fault entry.  INIT is exempt for one cycle so that a failed
     * AFE self-test can be reported through the normal path. */
    if ((s_state != BMS_STATE_FAULT) && (s_state != BMS_STATE_INIT) &&
        (in->fault_active || in->afe_fault))
    {
        EnterState(BMS_STATE_FAULT, now_ms);
        /* Emergency conditions (over-current, hard OV/UV limits, precharge
         * failure) must open on the same cycle as the fault is detected. */
        if (in->open_immediate)
        {
            s_fault_open = true;
        }
    }
    else
    {
        BmsState_t next = s_state;

        switch (s_state)
        {
        case BMS_STATE_INIT:
            if (in->afe_fault)
            {
                next = BMS_STATE_FAULT;
            }
            else if (in->afe_ok && in->meas_valid &&
                     ((now_ms - s_entry_ms) >= BMS_INIT_HOLD_MS))
            {
                next = BMS_STATE_STANDBY;
            }
            break;

        case BMS_STATE_STANDBY:
            if (in->sleep_request)
            {
                next = BMS_STATE_SLEEP;
            }
            else if (in->vcu_valid && in->vcu_close_request && in->meas_valid)
            {
                next = BMS_STATE_PRECHARGE;
            }
            break;

        case BMS_STATE_PRECHARGE:
        {
            if (!in->vcu_valid || !in->vcu_close_request)
            {
                next = BMS_STATE_STANDBY;
                break;
            }

            switch (s_precharge_phase)
            {
            case 0U:
                if ((now_ms - s_phase_ms) >= BMS_PRECHARGE_SETTLE_MS)
                {
                    s_precharge_phase = 1U;
                    s_phase_ms = now_ms;
                }
                break;

            case 1U:
                /* Negative contactor closed; now close the precharge relay. */
                s_precharge_phase = 2U;
                s_phase_ms = now_ms;
                s_precharge_start_ms = now_ms;
                break;

            case 2U:
            {
                uint32_t threshold = ((uint32_t)in->pack_voltage_mv * 95U) / 100U;
                if ((in->pack_voltage_mv > 0U) && ((uint32_t)in->bus_voltage_mv >= threshold))
                {
                    s_precharge_phase = 3U;
                    s_phase_ms = now_ms;
                }
                else if ((now_ms - s_precharge_start_ms) > BMS_PRECHARGE_TIMEOUT_MS)
                {
                    s_precharge_failed_pulse = true;
                    next = BMS_STATE_FAULT;
                }
                break;
            }

            case 3U:
                if ((now_ms - s_phase_ms) >= BMS_PRECHARGE_SETTLE_MS)
                {
                    next = in->charger_present ? BMS_STATE_CHARGE : BMS_STATE_DRIVE;
                }
                break;

            default:
                s_precharge_phase = 0U;
                break;
            }
            break;
        }

        case BMS_STATE_DRIVE:
            if (!in->vcu_valid)
            {
                if (s_vcu_lost_ms == 0U)
                {
                    s_vcu_lost_ms = now_ms;
                }
                if ((now_ms - s_vcu_lost_ms) >= BMS_VCU_LOSS_OPEN_DELAY_MS)
                {
                    next = BMS_STATE_STANDBY;
                }
            }
            else
            {
                s_vcu_lost_ms = 0U;
                if (!in->vcu_close_request)
                {
                    next = BMS_STATE_STANDBY;
                }
            }
            break;

        case BMS_STATE_CHARGE:
            if (!in->vcu_valid)
            {
                if (s_vcu_lost_ms == 0U)
                {
                    s_vcu_lost_ms = now_ms;
                }
                if ((now_ms - s_vcu_lost_ms) >= BMS_VCU_LOSS_OPEN_DELAY_MS)
                {
                    next = BMS_STATE_STANDBY;
                }
            }
            else
            {
                s_vcu_lost_ms = 0U;
                if (!in->vcu_close_request)
                {
                    next = BMS_STATE_STANDBY;
                }
                else if (!in->charger_present)
                {
                    next = BMS_STATE_DRIVE;
                }
            }
            break;

        case BMS_STATE_FAULT:
            if (in->open_immediate || ((now_ms - s_entry_ms) >= BMS_FAULT_OPEN_DELAY_MS))
            {
                s_fault_open = true;
            }
            if ((now_ms - s_entry_ms) >= BMS_FAULT_OPEN_DELAY_MS)
            {
                if (!in->fault_active && !in->afe_fault)
                {
                    next = BMS_STATE_STANDBY;
                }
            }
            break;

        case BMS_STATE_SLEEP:
            if ((!in->sleep_request) && in->vcu_valid)
            {
                next = BMS_STATE_STANDBY;
            }
            break;

        default:
            next = BMS_STATE_STANDBY;
            break;
        }

        if (next != s_state)
        {
            EnterState(next, now_ms);
            if (next == BMS_STATE_FAULT)
            {
                /* Over-current/precharge failures must open immediately; other
                 * faults may wait for the VCU to reduce current first. */
                s_fault_open = in->open_immediate || s_precharge_failed_pulse;
            }
        }
    }

    UpdateContactors();
    s_out.state = s_state;
    s_out.precharge_failed = s_precharge_failed_pulse;
    s_precharge_failed_pulse = false;
    s_out.state_time_ms = now_ms - s_entry_ms;
    s_out.transitions = s_transitions;
}

BmsStateOutput_t BmsState_GetOutput(void)
{
    return s_out;
}

const char *BmsState_Name(BmsState_t state)
{
    switch (state)
    {
    case BMS_STATE_INIT:
        return "INIT";
    case BMS_STATE_STANDBY:
        return "STANDBY";
    case BMS_STATE_PRECHARGE:
        return "PRECHARGE";
    case BMS_STATE_DRIVE:
        return "DRIVE";
    case BMS_STATE_CHARGE:
        return "CHARGE";
    case BMS_STATE_FAULT:
        return "FAULT";
    case BMS_STATE_SLEEP:
        return "SLEEP";
    default:
        return "UNKNOWN";
    }
}

bool BmsState_ContactorsClosed(void)
{
    return s_out.close_neg && s_out.close_pos;
}
