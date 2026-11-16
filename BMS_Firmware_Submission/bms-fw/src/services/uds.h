#ifndef BMS_SERVICES_UDS_H
#define BMS_SERVICES_UDS_H

#include <stdbool.h>
#include <stdint.h>

#define UDS_REQUEST_ID 0x7E0U
#define UDS_RESPONSE_ID 0x7E8U
#define UDS_S3_TIMEOUT_MS 5000U
#define UDS_MAX_RESPONSE 64U
#define UDS_CELL_COUNT 16U

/* Negative response codes used by this server. */
#define UDS_NRC_SERVICE_NOT_SUPPORTED 0x11U
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED 0x12U
#define UDS_NRC_INCORRECT_LENGTH 0x13U
#define UDS_NRC_CONDITIONS_NOT_CORRECT 0x22U
#define UDS_NRC_REQUEST_OUT_OF_RANGE 0x31U
#define UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION 0x7FU

typedef struct
{
    const char *sw_version;

    uint16_t (*read_pack_voltage_mv)(void);
    int32_t (*read_pack_current_ma)(void);
    bool (*read_cell_mv)(uint16_t *cells, uint8_t count);

    bool (*write_bal_start)(uint16_t mv);
    bool (*write_bal_stop)(uint16_t mv);

    uint8_t (*dtc_count)(void);
    bool (*dtc_by_index)(uint8_t index, uint32_t *dtc, uint8_t *status);
    void (*clear_dtcs)(void);
    uint8_t (*dtc_status_mask)(void);
} UdsHooks_t;

typedef struct
{
    const UdsHooks_t *hooks;
    bool extended;
    bool session_change;
    uint32_t last_request_ms;
    uint32_t request_count;
    uint32_t nrc_count;
    uint8_t last_nrc;
} UdsServer_t;

void Uds_Init(UdsServer_t *uds, const UdsHooks_t *hooks, uint32_t now_ms);

/**
 * Process one complete diagnostic request and build the response.
 * Returns the response length, or 0 when the positive response is suppressed.
 */
uint16_t Uds_Process(UdsServer_t *uds,
                     const uint8_t *req,
                     uint16_t req_len,
                     uint8_t *resp,
                     uint16_t resp_cap,
                     uint32_t now_ms);

/** Enforce the 5 s S3 timer for the extended session. */
void Uds_Poll(UdsServer_t *uds, uint32_t now_ms);

bool Uds_IsExtended(const UdsServer_t *uds);

#endif /* BMS_SERVICES_UDS_H */
