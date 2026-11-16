#ifndef BMS_APP_APP_H
#define BMS_APP_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "app/bms_state.h"
#include "drivers/afe.h"
#include "services/balancing.h"
#include "services/can_matrix.h"
#include "services/protection.h"

#define APP_CELL_TEMP_COUNT 4U
#define APP_CHARGE_VOLTAGE_REQ_MV 56800U /* 16 x 3550 mV, see ASSUMPTIONS.md */
#define APP_SOC_SAVE_PERIOD_MS 60000U
#define APP_BALANCE_PERIOD_MS 100U
#define APP_CAN_FAST_PERIOD_MS 100U
#define APP_CAN_STATS_PERIOD_MS 500U
#define APP_CAN_SLOW_PERIOD_MS 1000U
#define APP_CONTROL_PERIOD_MS 10U

typedef struct
{
    BmsState_t state;
    const char *state_name;
    bool contactor_neg;
    bool contactor_pre;
    bool contactor_pos;
    uint16_t pack_voltage_mv;
    uint16_t bus_voltage_mv;
    int32_t pack_current_ma;
    int32_t soc_permille;
    bool meas_valid;
    uint16_t cell_mv[AFE_CELL_COUNT];
    int16_t cell_temp_dc[APP_CELL_TEMP_COUNT];
    int16_t board_temp_dc;
    uint16_t bal_mask;
    bool bal_active;
    uint8_t bal_active_count;
    uint8_t bal_inhibit;
    uint32_t bal_active_time_ms;
    bool vcu_valid;
    uint8_t vcu_contactor_req;
    bool vcu_bal_enable;
    bool vcu_sleep_req;
    uint8_t vcu_alive;
    int32_t ccl_ma;
    int32_t dcl_ma;
    uint32_t fault_bits;
    uint16_t warning_bits;
    uint8_t active_dtc_count;
    uint32_t state_transitions;
    uint32_t can_tx_queued;
    uint32_t can_tx_dropped;
    uint32_t sched_executions;
    uint32_t sched_overruns;
} AppSnapshot_t;

/** Initialise every service and register the periodic tasks. */
void App_Init(void);

/** Execute one scheduler tick.  Call once per millisecond. */
void App_Step(uint32_t now_ms);

/** Target super-loop.  Not used by the host simulator. */
void App_Run(void);

/** Consistent copy of the application state for logging and tests. */
void App_GetSnapshot(AppSnapshot_t *out);

/** Exposed for unit tests and diagnostics. */
uint32_t App_GetCanAlive(void);
int32_t App_GetSocPermille(void);

#endif /* BMS_APP_APP_H */
