#ifndef BMS_SERVICES_CAN_MATRIX_H
#define BMS_SERVICES_CAN_MATRIX_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_ID_BMS_PACK_STATUS 0x350U
#define CAN_ID_BMS_LIMITS 0x351U
#define CAN_ID_BMS_CELL_STATS 0x352U
#define CAN_ID_BMS_CELL_VOLTAGES 0x353U
#define CAN_ID_BMS_FAULTS 0x354U
#define CAN_ID_VCU_COMMAND 0x200U

#define CAN_DLC 8U
#define CAN_VCU_TIMEOUT_MS 300U
#define CAN_VCU_MAX_ALIVE_JUMP 2U
#define CAN_VCU_MAX_INVALID_STREAK 3U
#define CAN_CELL_MUX_COUNT 6U
#define CAN_BALANCE_START_MV 15U
#define CAN_BALANCE_STOP_MV 5U

/* BMS_STATE values on 0x350.  Kept as plain numbers so the CAN matrix does
 * not depend on the application state-machine header. */
#define CAN_STATE_INIT 0U
#define CAN_STATE_STANDBY 1U
#define CAN_STATE_PRECHARGE 2U
#define CAN_STATE_DRIVE 3U
#define CAN_STATE_CHARGE 4U
#define CAN_STATE_FAULT 5U
#define CAN_STATE_SLEEP 6U

typedef struct
{
    uint16_t pack_voltage_mv;
    int32_t pack_current_ma;
    uint16_t soc_permille; /* 0..1000 */
    uint8_t bms_state;
} CanPackStatus_t;

typedef struct
{
    uint32_t ccl_ma;
    uint32_t dcl_ma;
    uint16_t charge_voltage_mv;
    bool charge_enable;
} CanLimits_t;

typedef struct
{
    uint16_t vmax_mv;
    uint16_t vmin_mv;
    uint8_t vmax_idx; /* 0-based */
    uint8_t vmin_idx;
    int16_t tmax_c;
    int16_t tmin_c;
} CanCellStats_t;

typedef struct
{
    uint8_t mux;
    uint16_t cell_mv[3];
    uint8_t balance_bits;
} CanCellVoltages_t;

typedef struct
{
    uint32_t fault_bits;
    uint16_t warning_bits;
    uint8_t active_dtc_count;
} CanFaults_t;

typedef struct
{
    uint8_t contactor_req; /* 0 open, 1 close, 2/3 invalid */
    bool balancing_enable;
    bool sleep_req;
    bool fault_clear_req;
    uint8_t alive_counter;
} CanVcuCommand_t;

typedef enum
{
    VCU_FRAME_OK = 0,
    VCU_FRAME_BAD_DLC,
    VCU_FRAME_BAD_CRC
} CanVcuFrameResult_t;

typedef struct
{
    uint8_t last_alive;
    bool have_last;
    bool valid_seen;
    bool fault;
    uint8_t consecutive_invalid;
    uint32_t last_valid_ms;
    uint32_t crc_errors;
    uint32_t alive_errors;
    uint32_t jump_errors;
    uint32_t dlc_errors;
    uint32_t timeout_events;
    CanVcuCommand_t last_command;
} VcuMonitor_t;

/* Pack functions: always produce 8 bytes.  `alive` is 0..15. */
void CanMatrix_PackPackStatus(uint8_t data[CAN_DLC], const CanPackStatus_t *status, uint8_t alive);
void CanMatrix_PackLimits(uint8_t data[CAN_DLC], const CanLimits_t *limits, uint8_t alive);
void CanMatrix_PackCellStats(uint8_t data[CAN_DLC], const CanCellStats_t *stats);
void CanMatrix_PackCellVoltages(uint8_t data[CAN_DLC], const CanCellVoltages_t *cells);
void CanMatrix_PackFaults(uint8_t data[CAN_DLC], const CanFaults_t *faults);
void CanMatrix_PackVcuCommand(uint8_t data[CAN_DLC], const CanVcuCommand_t *command);

bool CanMatrix_UnpackPackStatus(const uint8_t data[CAN_DLC], CanPackStatus_t *status);
bool CanMatrix_UnpackLimits(const uint8_t data[CAN_DLC], CanLimits_t *limits);
bool CanMatrix_UnpackCellVoltages(const uint8_t data[CAN_DLC], CanCellVoltages_t *cells);
CanVcuFrameResult_t CanMatrix_UnpackVcuCommand(const uint8_t *data, uint8_t dlc, CanVcuCommand_t *command);

/** True if byte 7 is the correct CRC-8 SAE J1850 over bytes 0..6. */
bool CanMatrix_CheckCrc(const uint8_t data[CAN_DLC]);

void VcuMonitor_Init(VcuMonitor_t *mon, uint32_t now_ms);
bool VcuMonitor_Update(VcuMonitor_t *mon, const uint8_t *data, uint8_t dlc, uint32_t now_ms);
void VcuMonitor_Poll(VcuMonitor_t *mon, uint32_t now_ms);
bool VcuMonitor_IsFault(const VcuMonitor_t *mon);

#endif /* BMS_SERVICES_CAN_MATRIX_H */
