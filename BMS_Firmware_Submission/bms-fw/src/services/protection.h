#ifndef BMS_SERVICES_PROTECTION_H
#define BMS_SERVICES_PROTECTION_H

#include <stdbool.h>
#include <stdint.h>

#define PROT_CELL_COUNT 16U
#define PROT_TEMP_COUNT 5U

/* ISO 14229-1 DTC status bits. */
#define DTC_STATUS_TEST_FAILED 0x01U
#define DTC_STATUS_TEST_FAILED_THIS_CYCLE 0x02U
#define DTC_STATUS_PENDING 0x04U
#define DTC_STATUS_CONFIRMED 0x08U
#define DTC_STATUS_TEST_NOT_COMPLETED_SINCE_CLEAR 0x10U
#define DTC_STATUS_TEST_FAILED_SINCE_CLEAR 0x20U
#define DTC_STATUS_TEST_NOT_COMPLETED_THIS_CYCLE 0x40U
#define DTC_STATUS_WARNING 0x80U

typedef enum
{
    FAULT_CELL_OV = 0,
    FAULT_CELL_UV,
    FAULT_CHARGE_OT,
    FAULT_DISCHARGE_OT,
    FAULT_CHARGE_UT,
    FAULT_DISCHARGE_OC,
    FAULT_CHARGE_OC,
    FAULT_AFE_COMM,
    FAULT_VCU_TIMEOUT,
    FAULT_CELL_SUM_MISMATCH,
    FAULT_PRECHARGE,
    FAULT_COUNT
} FaultId;

typedef struct
{
    bool cells_valid;
    uint16_t cell_mv[PROT_CELL_COUNT];
    int16_t temp_dc[PROT_TEMP_COUNT];
    int16_t board_temp_dc;
    int32_t current_ma;
    bool charging;
    bool discharging;
    bool afe_comm_fault;
    bool vcu_timeout;
    bool cell_sum_mismatch;
    bool precharge_timeout;
} ProtInput_t;

typedef struct
{
    int32_t ccl_ma;
    int32_t dcl_ma;
    bool charge_enable;
    bool contactor_open_request;
    bool contactor_open_immediate;
    bool latched_fault_active;
    bool warning_active;
    uint32_t fault_bits;
    uint16_t warning_bits;
    uint8_t active_dtc_count;
} ProtOutput_t;

void Prot_Init(void);
void Prot_Update(const ProtInput_t *in, uint32_t dt_ms);
ProtOutput_t Prot_GetOutput(void);

bool Prot_IsFaultActive(FaultId id);
uint8_t Prot_GetDtcStatus(FaultId id);
uint8_t Prot_GetDtcCount(void);
bool Prot_GetDtcByIndex(uint8_t index, uint32_t *dtc, uint8_t *status);
void Prot_ClearDtc(uint32_t group);

#endif /* BMS_SERVICES_PROTECTION_H */
