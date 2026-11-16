#include "services/balancing.h"

#include <string.h>

static BalConfig_t s_cfg;
static BalStatus_t s_status;
static bool s_over_temp_latched;
static bool s_measurement_window;
static uint8_t s_priority_cell; /* 0 = none, otherwise cell index + 1 */

static const BalConfig_t s_default_cfg = {
    .vmin_mv = BAL_DEFAULT_VMIN_MV,
    .start_mv = BAL_DEFAULT_START_MV,
    .stop_mv = BAL_DEFAULT_STOP_MV,
    .max_cells = BAL_DEFAULT_MAX_CELLS,
    .cell_temp_max_dc = BAL_DEFAULT_CELL_TEMP_MAX_DC,
    .board_temp_max_dc = BAL_DEFAULT_BOARD_TEMP_MAX_DC,
    .temp_hyst_dc = BAL_DEFAULT_TEMP_HYST_DC,
    .discharge_inhibit_ma = BAL_DEFAULT_DISCHARGE_INHIBIT_MA,
    .timeout_ms = BAL_DEFAULT_TIMEOUT_MS,
    .cycle_ms = BAL_DEFAULT_CYCLE_MS,
    .fairness_band_mv = BAL_DEFAULT_FAIRNESS_BAND_MV,
};

static uint8_t PopCount16(uint16_t value)
{
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < 16U; ++i)
    {
        if ((value & (uint16_t)(1U << i)) != 0U)
        {
            count++;
        }
    }
    return count;
}

static bool Adjacent(uint16_t mask)
{
    return (mask & (uint16_t)(mask << 1U)) != 0U;
}

static void ResetState(void)
{
    (void)memset(&s_status, 0, sizeof(s_status));
    s_status.inhibit = BAL_INHIBIT_NONE;
    s_over_temp_latched = false;
    s_measurement_window = false;
    s_priority_cell = 0U;
}

void Bal_Init(const BalConfig_t *cfg)
{
    if (cfg != NULL)
    {
        s_cfg = *cfg;
    }
    else
    {
        s_cfg = s_default_cfg;
    }

    /* Defensive validation of calibratable data. */
    if (s_cfg.stop_mv >= s_cfg.start_mv)
    {
        s_cfg.stop_mv = (uint16_t)(s_cfg.start_mv - 1U);
    }
    if (s_cfg.max_cells == 0U)
    {
        s_cfg.max_cells = 1U;
    }
    if (s_cfg.max_cells > BAL_CELL_COUNT)
    {
        s_cfg.max_cells = BAL_CELL_COUNT;
    }
    if (s_cfg.cycle_ms == 0U)
    {
        s_cfg.cycle_ms = BAL_DEFAULT_CYCLE_MS;
    }
    if (s_cfg.timeout_ms == 0U)
    {
        s_cfg.timeout_ms = BAL_DEFAULT_TIMEOUT_MS;
    }
    ResetState();
}

const BalConfig_t *Bal_GetConfig(void)
{
    return &s_cfg;
}

bool Bal_SetStartStop(uint16_t start_mv, uint16_t stop_mv)
{
    if ((start_mv < 5U) || (start_mv > 50U) || (stop_mv < 1U) || (stop_mv >= start_mv))
    {
        return false;
    }
    s_cfg.start_mv = start_mv;
    s_cfg.stop_mv = stop_mv;
    return true;
}

void Bal_SetMeasurementWindow(bool active)
{
    s_measurement_window = active;
}

void Bal_NotifyChargeCycle(void)
{
    s_status.active_time_ms = 0U;
    s_status.timeout_warning = false;
    s_priority_cell = 0U;
}

static BalInhibit_t EvaluateInhibit(const BalInput_t *in)
{
    if (s_measurement_window)
    {
        return BAL_INHIBIT_MEAS_PAUSE;
    }
    if (!in->meas_valid)
    {
        return BAL_INHIBIT_MEAS_INVALID;
    }
    if (in->any_fault_active)
    {
        return BAL_INHIBIT_FAULT;
    }
    if (!in->vcu_balancing_enable)
    {
        return BAL_INHIBIT_VCU_DISABLED;
    }
    if (s_status.timeout_warning)
    {
        return BAL_INHIBIT_TIMEOUT;
    }

    bool cell_hot = (in->max_cell_temp_dc >= s_cfg.cell_temp_max_dc);
    bool board_hot = (in->board_temp_dc >= s_cfg.board_temp_max_dc);
    if (cell_hot || board_hot)
    {
        s_over_temp_latched = true;
    }
    else if (s_over_temp_latched)
    {
        bool cell_ok = (in->max_cell_temp_dc <= (s_cfg.cell_temp_max_dc - s_cfg.temp_hyst_dc));
        bool board_ok = (in->board_temp_dc <= (s_cfg.board_temp_max_dc - s_cfg.temp_hyst_dc));
        if (cell_ok && board_ok)
        {
            s_over_temp_latched = false;
        }
    }
    if (s_over_temp_latched)
    {
        return BAL_INHIBIT_OVER_TEMP;
    }

    if (in->pack_current_mA < s_cfg.discharge_inhibit_ma)
    {
        return BAL_INHIBIT_DISCHARGE;
    }

    return BAL_INHIBIT_NONE;
}

/* Insertion sort is appropriate for 16 cells: deterministic, no allocation,
 * no recursion, and O(n^2) worst case with n fixed at 16. */
static void SortCandidates(uint8_t *candidates, uint8_t count, const BalInput_t *in, uint16_t vmax)
{
    for (uint8_t i = 1U; i < count; ++i)
    {
        uint8_t key = candidates[i];
        uint8_t j = i;

        while (j > 0U)
        {
            uint8_t prev = candidates[j - 1U];
            bool key_priority = ((uint8_t)(key + 1U) == s_priority_cell) &&
                                ((uint16_t)(vmax - in->cell_mv[key]) <= s_cfg.fairness_band_mv);
            bool prev_priority = ((uint8_t)(prev + 1U) == s_priority_cell) &&
                                 ((uint16_t)(vmax - in->cell_mv[prev]) <= s_cfg.fairness_band_mv);

            bool key_before;
            if (key_priority != prev_priority)
            {
                key_before = key_priority;
            }
            else if (in->cell_mv[key] != in->cell_mv[prev])
            {
                key_before = (in->cell_mv[key] > in->cell_mv[prev]);
            }
            else
            {
                key_before = (key < prev);
            }

            if (!key_before)
            {
                break;
            }
            candidates[j] = prev;
            --j;
        }
        candidates[j] = key;
    }
}

uint16_t Bal_Run(const BalInput_t *in)
{
    if (in == NULL)
    {
        s_status.mask = 0U;
        s_status.active = false;
        s_status.active_count = 0U;
        s_status.inhibit = BAL_INHIBIT_MEAS_INVALID;
        return 0U;
    }

    s_status.cycles++;

    /* A charge cycle (net charge current) clears the 8 h timeout warning. */
    if (in->pack_current_mA > 500)
    {
        Bal_NotifyChargeCycle();
    }

    BalInhibit_t inhibit = EvaluateInhibit(in);
    if (inhibit != BAL_INHIBIT_NONE)
    {
        if (inhibit != BAL_INHIBIT_MEAS_PAUSE)
        {
            /* Continuous balancing was interrupted, so the 8 h window ends. */
            s_status.active_time_ms = 0U;
        }
        s_status.inhibit = inhibit;
        s_status.mask = 0U;
        s_status.active = false;
        s_status.active_count = 0U;
        return 0U;
    }

    uint16_t vmin = 0xFFFFU;
    uint16_t vmax = 0U;
    for (uint8_t i = 0U; i < BAL_CELL_COUNT; ++i)
    {
        if (in->cell_mv[i] < vmin)
        {
            vmin = in->cell_mv[i];
        }
        if (in->cell_mv[i] > vmax)
        {
            vmax = in->cell_mv[i];
        }
    }

    uint8_t candidates[BAL_CELL_COUNT];
    uint8_t candidate_count = 0U;

    if (vmax >= s_cfg.vmin_mv)
    {
        for (uint8_t i = 0U; i < BAL_CELL_COUNT; ++i)
        {
            uint16_t delta = (uint16_t)(in->cell_mv[i] - vmin);
            bool was_on = ((s_status.mask & (uint16_t)(1U << i)) != 0U);
            bool qualifies;

            if (was_on)
            {
                qualifies = (delta >= s_cfg.stop_mv);
            }
            else
            {
                qualifies = (delta > s_cfg.start_mv);
            }

            if (qualifies)
            {
                candidates[candidate_count] = i;
                candidate_count++;
            }
        }
    }

    SortCandidates(candidates, candidate_count, in, vmax);

    uint16_t new_mask = 0U;
    uint8_t selected = 0U;
    uint8_t first_skipped = 0U;

    for (uint8_t i = 0U; i < candidate_count; ++i)
    {
        uint8_t cell = candidates[i];
        uint16_t bit = (uint16_t)(1U << cell);

        if (selected >= s_cfg.max_cells)
        {
            if (first_skipped == 0U)
            {
                first_skipped = (uint8_t)(cell + 1U);
            }
            continue;
        }

        uint16_t trial = (uint16_t)(new_mask | bit);
        if (Adjacent(trial))
        {
            if (first_skipped == 0U)
            {
                first_skipped = (uint8_t)(cell + 1U);
            }
            continue;
        }

        new_mask = trial;
        selected++;
    }

    s_priority_cell = first_skipped;

    s_status.mask = new_mask;
    s_status.active = (new_mask != 0U);
    s_status.active_count = PopCount16(new_mask);
    s_status.inhibit = BAL_INHIBIT_NONE;

    if (new_mask != 0U)
    {
        uint32_t elapsed = s_status.active_time_ms + s_cfg.cycle_ms;
        if (elapsed >= s_cfg.timeout_ms)
        {
            s_status.active_time_ms = s_cfg.timeout_ms;
            s_status.timeout_warning = true;
            s_status.mask = 0U;
            s_status.active = false;
            s_status.active_count = 0U;
            s_status.inhibit = BAL_INHIBIT_TIMEOUT;
            return 0U;
        }
        s_status.active_time_ms = elapsed;
    }
    else
    {
        /* A gap ends the continuous-balancing interval. */
        s_status.active_time_ms = 0U;
    }

    return new_mask;
}

BalStatus_t Bal_GetStatus(void)
{
    return s_status;
}
