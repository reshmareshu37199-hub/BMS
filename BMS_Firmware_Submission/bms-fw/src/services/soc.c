#include "services/soc.h"

#include "common/saturate.h"

#include <string.h>

typedef struct
{
    uint16_t ocv_mv;
    int32_t soc_permille;
} OcvPoint_t;

/* LFP open-circuit voltage table from the assignment, extended with 0/100 %
 * endpoints.  Interpolated linearly. */
static const OcvPoint_t s_ocv_table[] = {
    {2800U, 0},
    {3200U, 100},
    {3250U, 200},
    {3280U, 300},
    {3300U, 500},
    {3320U, 700},
    {3340U, 900},
    {3380U, 950},
    {3450U, 1000},
};

#define OCV_TABLE_SIZE (sizeof(s_ocv_table) / sizeof(s_ocv_table[0]))

static int32_t AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

void Soc_Init(SocState_t *state, const SocConfig_t *config, int32_t initial_permille)
{
    if (state == NULL)
    {
        return;
    }

    (void)memset(state, 0, sizeof(*state));
    if (config != NULL)
    {
        state->config = *config;
    }
    else
    {
        state->config.capacity_mah = SOC_DEFAULT_CAPACITY_MAH;
        state->config.efficiency_permille = SOC_DEFAULT_EFFICIENCY_PERMILLE;
    }

    if (state->config.capacity_mah <= 0)
    {
        state->config.capacity_mah = SOC_DEFAULT_CAPACITY_MAH;
    }
    if ((state->config.efficiency_permille <= 0) || (state->config.efficiency_permille > 1000))
    {
        state->config.efficiency_permille = SOC_DEFAULT_EFFICIENCY_PERMILLE;
    }

    state->soc_permille = Sat_I32((int64_t)initial_permille, 0, SOC_PERMILLE_FULL);
    state->initialised = true;
}

int32_t Soc_OcvToPermille(uint16_t ocv_mv)
{
    if (ocv_mv <= s_ocv_table[0].ocv_mv)
    {
        return 0;
    }
    if (ocv_mv >= s_ocv_table[OCV_TABLE_SIZE - 1U].ocv_mv)
    {
        return SOC_PERMILLE_FULL;
    }

    for (uint32_t i = 1U; i < OCV_TABLE_SIZE; ++i)
    {
        if (ocv_mv <= s_ocv_table[i].ocv_mv)
        {
            int32_t v0 = (int32_t)s_ocv_table[i - 1U].ocv_mv;
            int32_t v1 = (int32_t)s_ocv_table[i].ocv_mv;
            int32_t s0 = s_ocv_table[i - 1U].soc_permille;
            int32_t s1 = s_ocv_table[i].soc_permille;
            int64_t num = (int64_t)((int32_t)ocv_mv - v0) * (int64_t)(s1 - s0);
            return s0 + (int32_t)(num / (int64_t)(v1 - v0));
        }
    }
    return SOC_PERMILLE_FULL;
}

static void ApplyOcvCorrection(SocState_t *state, uint16_t ocv_mv)
{
    int32_t target = Soc_OcvToPermille(ocv_mv);
    int32_t diff = target - state->soc_permille;

    if (AbsI32(diff) < 5)
    {
        state->last_ocv_permille = target;
        return;
    }

    bool flat = (ocv_mv >= SOC_OCV_FLAT_LOW_MV) && (ocv_mv <= SOC_OCV_FLAT_HIGH_MV);
    int32_t weight_num = flat ? SOC_OCV_WEIGHT_FLAT_NUM : SOC_OCV_WEIGHT_EDGE_NUM;
    int32_t weight_den = flat ? SOC_OCV_WEIGHT_FLAT_DEN : SOC_OCV_WEIGHT_EDGE_DEN;
    int32_t correction = (diff * weight_num) / weight_den;

    state->soc_permille = Sat_I32((int64_t)state->soc_permille + correction, 0, SOC_PERMILLE_FULL);
    state->last_ocv_permille = target;
    state->ocv_corrections++;
}

void Soc_Update(SocState_t *state, int32_t current_ma, uint16_t cell_ocv_mv, uint32_t dt_ms)
{
    if ((state == NULL) || !state->initialised || (dt_ms == 0U))
    {
        return;
    }

    int32_t corrected_ma = current_ma - state->offset_ma;

    /* Coulomb counting.  Charge is stored in micro-amp-hours so that the
     * 10 ms step does not lose precision through repeated integer division. */
    int32_t effective_ma = corrected_ma;
    if (effective_ma > 0)
    {
        effective_ma = (int32_t)(((int64_t)effective_ma * state->config.efficiency_permille) / 1000);
    }

    int64_t mah_ms = ((int64_t)effective_ma * (int64_t)dt_ms) + state->charge_remainder;
    int64_t delta_uah = mah_ms / 3600; /* 1 uAh = 3600 mA*ms */
    state->charge_remainder = mah_ms % 3600;

    int64_t capacity_uah = (int64_t)state->config.capacity_mah * 1000;
    int64_t soc_num = (delta_uah * 1000) + state->soc_remainder;
    int32_t delta_permille = (int32_t)(soc_num / capacity_uah);
    state->soc_remainder = soc_num % capacity_uah;
    state->soc_permille = Sat_I32((int64_t)state->soc_permille + delta_permille, 0, SOC_PERMILLE_FULL);

    /* Rest detection and current-offset learning. */
    if (AbsI32(current_ma) < SOC_REST_CURRENT_MA)
    {
        if (state->rest_ms <= (UINT32_MAX - dt_ms))
        {
            state->rest_ms += dt_ms;
        }
        if (state->offset_ms <= (UINT32_MAX - dt_ms))
        {
            state->offset_ms += dt_ms;
        }
        state->offset_sum_ma_ms += (int64_t)current_ma * (int64_t)dt_ms;

        if (state->offset_ms >= SOC_OFFSET_WINDOW_MS)
        {
            int32_t average = (int32_t)(state->offset_sum_ma_ms / (int64_t)state->offset_ms);
            state->offset_ma = Sat_I32((int64_t)average, -2000, 2000);
            state->offset_ms = 0U;
            state->offset_sum_ma_ms = 0;
        }

        if ((!state->ocv_done) && (state->rest_ms >= SOC_OCV_REST_MS) && (cell_ocv_mv > 0U))
        {
            ApplyOcvCorrection(state, cell_ocv_mv);
            state->ocv_done = true;
        }
    }
    else
    {
        state->rest_ms = 0U;
        state->offset_ms = 0U;
        state->offset_sum_ma_ms = 0;
        state->ocv_done = false;
    }
}

int32_t Soc_GetPermille(const SocState_t *state)
{
    return (state == NULL) ? 0 : state->soc_permille;
}

int32_t Soc_GetPercent(const SocState_t *state)
{
    return (state == NULL) ? 0 : (state->soc_permille / 10);
}

int32_t Soc_GetOffsetMa(const SocState_t *state)
{
    return (state == NULL) ? 0 : state->offset_ma;
}

uint32_t Soc_GetRestMs(const SocState_t *state)
{
    return (state == NULL) ? 0U : state->rest_ms;
}

uint32_t Soc_GetOcvCorrections(const SocState_t *state)
{
    return (state == NULL) ? 0U : state->ocv_corrections;
}

void Soc_SetPermille(SocState_t *state, int32_t permille)
{
    if (state == NULL)
    {
        return;
    }
    state->soc_permille = Sat_I32((int64_t)permille, 0, SOC_PERMILLE_FULL);
}
