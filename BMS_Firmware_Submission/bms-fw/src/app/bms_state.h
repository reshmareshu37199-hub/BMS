#ifndef BMS_APP_BMS_STATE_H
#define BMS_APP_BMS_STATE_H

#include <stdbool.h>
#include <stdint.h>

#define BMS_PRECHARGE_TIMEOUT_MS 2000U
#define BMS_PRECHARGE_SETTLE_MS 20U
#define BMS_FAULT_OPEN_DELAY_MS 1000U
#define BMS_VCU_LOSS_OPEN_DELAY_MS 1000U
#define BMS_INIT_HOLD_MS 200U

typedef enum
{
    BMS_STATE_INIT = 0,
    BMS_STATE_STANDBY,
    BMS_STATE_PRECHARGE,
    BMS_STATE_DRIVE,
    BMS_STATE_CHARGE,
    BMS_STATE_FAULT,
    BMS_STATE_SLEEP,
    BMS_STATE_COUNT
} BmsState_t;

typedef struct
{
    bool afe_ok;          /* first valid AFE snapshot has been published */
    bool afe_fault;       /* latched AFE communication / device fault */
    bool meas_valid;
    bool fault_active;    /* any protection fault except VCU timeout */
    bool fault_latched;
    bool open_request;
    bool open_immediate;
    bool vcu_valid;
    bool vcu_close_request;
    bool charger_present;
    bool sleep_request;
    uint16_t pack_voltage_mv;
    uint16_t bus_voltage_mv;
} BmsStateInput_t;

typedef struct
{
    BmsState_t state;
    bool close_neg;
    bool close_pre;
    bool close_pos;
    bool precharge_failed; /* one-cycle pulse consumed by the protection layer */
    uint32_t state_time_ms;
    uint32_t transitions;
} BmsStateOutput_t;

void BmsState_Init(uint32_t now_ms);
void BmsState_Update(const BmsStateInput_t *in, uint32_t now_ms);
BmsStateOutput_t BmsState_GetOutput(void);
const char *BmsState_Name(BmsState_t state);
bool BmsState_ContactorsClosed(void);

#endif /* BMS_APP_BMS_STATE_H */
