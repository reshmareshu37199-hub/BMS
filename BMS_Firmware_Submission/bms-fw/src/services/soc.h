#ifndef BMS_SERVICES_SOC_H
#define BMS_SERVICES_SOC_H

#include <stdbool.h>
#include <stdint.h>

#define SOC_PERMILLE_FULL 1000
#define SOC_DEFAULT_CAPACITY_MAH 100000
#define SOC_DEFAULT_EFFICIENCY_PERMILLE 990
#define SOC_REST_CURRENT_MA 500
#define SOC_OFFSET_WINDOW_MS 30000U
#define SOC_OCV_REST_MS 3600000U
#define SOC_OCV_FLAT_LOW_MV 3280U
#define SOC_OCV_FLAT_HIGH_MV 3340U
#define SOC_OCV_WEIGHT_FLAT_NUM 1
#define SOC_OCV_WEIGHT_FLAT_DEN 20   /* 5 % in the flat LFP region */
#define SOC_OCV_WEIGHT_EDGE_NUM 1
#define SOC_OCV_WEIGHT_EDGE_DEN 4    /* 25 % where the curve is steep */

typedef struct
{
    int32_t capacity_mah;
    int32_t efficiency_permille;
} SocConfig_t;

typedef struct
{
    SocConfig_t config;
    int32_t soc_permille;
    int32_t offset_ma;
    int64_t charge_remainder;
    int64_t soc_remainder;
    uint32_t rest_ms;
    uint32_t offset_ms;
    int64_t offset_sum_ma_ms;
    bool ocv_done;
    bool initialised;
    uint32_t ocv_corrections;
    int32_t last_ocv_permille;
} SocState_t;

void Soc_Init(SocState_t *state, const SocConfig_t *config, int32_t initial_permille);

/**
 * Coulomb counting step.  `current_ma` is positive for charge.  `cell_ocv_mv`
 * is the rested cell voltage, or 0 when no valid OCV is available.
 *
 * A current offset is learned whenever |I| stays below 0.5 A for 30 s.
 * After one hour at rest the SOC is pulled towards the OCV estimate, with a
 * much smaller weight in the flat 30–90 % region of the LFP curve.
 */
void Soc_Update(SocState_t *state, int32_t current_ma, uint16_t cell_ocv_mv, uint32_t dt_ms);

int32_t Soc_GetPermille(const SocState_t *state);
int32_t Soc_GetPercent(const SocState_t *state);
int32_t Soc_GetOffsetMa(const SocState_t *state);
uint32_t Soc_GetRestMs(const SocState_t *state);
uint32_t Soc_GetOcvCorrections(const SocState_t *state);
int32_t Soc_OcvToPermille(uint16_t ocv_mv);
void Soc_SetPermille(SocState_t *state, int32_t permille);

#endif /* BMS_SERVICES_SOC_H */
