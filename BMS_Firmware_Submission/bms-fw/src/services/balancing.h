#ifndef BMS_SERVICES_BALANCING_H
#define BMS_SERVICES_BALANCING_H

#include <stdbool.h>
#include <stdint.h>

#define BAL_CELL_COUNT 16U
#define BAL_DEFAULT_VMIN_MV 3400U
#define BAL_DEFAULT_START_MV 15U
#define BAL_DEFAULT_STOP_MV 5U
#define BAL_DEFAULT_MAX_CELLS 8U
#define BAL_DEFAULT_CELL_TEMP_MAX_DC 500
#define BAL_DEFAULT_BOARD_TEMP_MAX_DC 700
#define BAL_DEFAULT_TEMP_HYST_DC 50
#define BAL_DEFAULT_DISCHARGE_INHIBIT_MA (-2000)
#define BAL_DEFAULT_TIMEOUT_MS (8UL * 60UL * 60UL * 1000UL)
#define BAL_DEFAULT_CYCLE_MS 100U
#define BAL_DEFAULT_FAIRNESS_BAND_MV 20U

typedef enum
{
    BAL_INHIBIT_NONE = 0,
    BAL_INHIBIT_MEAS_PAUSE,
    BAL_INHIBIT_MEAS_INVALID,
    BAL_INHIBIT_FAULT,
    BAL_INHIBIT_VCU_DISABLED,
    BAL_INHIBIT_OVER_TEMP,
    BAL_INHIBIT_DISCHARGE,
    BAL_INHIBIT_TIMEOUT
} BalInhibit_t;

typedef struct
{
    uint16_t vmin_mv;
    uint16_t start_mv;
    uint16_t stop_mv;
    uint8_t max_cells;
    int16_t cell_temp_max_dc;
    int16_t board_temp_max_dc;
    int16_t temp_hyst_dc;
    int32_t discharge_inhibit_ma;
    uint32_t timeout_ms;
    uint32_t cycle_ms;
    uint16_t fairness_band_mv;
} BalConfig_t;

typedef struct
{
    uint16_t cell_mv[BAL_CELL_COUNT];
    bool meas_valid;
    int16_t max_cell_temp_dc;
    int16_t board_temp_dc;
    int32_t pack_current_mA;
    bool vcu_balancing_enable;
    bool any_fault_active;
} BalInput_t;

typedef struct
{
    bool active;
    uint16_t mask;
    uint8_t active_count;
    uint32_t active_time_ms;
    BalInhibit_t inhibit;
    bool timeout_warning;
    uint32_t cycles;
    uint8_t priority_cell;
} BalStatus_t;

void Bal_Init(const BalConfig_t *cfg);
uint16_t Bal_Run(const BalInput_t *in);
BalStatus_t Bal_GetStatus(void);
const BalConfig_t *Bal_GetConfig(void);
bool Bal_SetStartStop(uint16_t start_mv, uint16_t stop_mv);
void Bal_SetMeasurementWindow(bool active);
void Bal_NotifyChargeCycle(void);

#endif /* BMS_SERVICES_BALANCING_H */
