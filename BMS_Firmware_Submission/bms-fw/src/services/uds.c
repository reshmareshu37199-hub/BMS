#include "services/uds.h"

#include <string.h>

#define UDS_SID_SESSION_CONTROL 0x10U
#define UDS_SID_ECU_RESET 0x11U
#define UDS_SID_CLEAR_DTC 0x14U
#define UDS_SID_READ_DTC 0x19U
#define UDS_SID_READ_DID 0x22U
#define UDS_SID_WRITE_DID 0x2EU
#define UDS_SID_TESTER_PRESENT 0x3EU

#define UDS_DID_SW_VERSION 0xF189U
#define UDS_DID_PACK_V_I 0x0101U
#define UDS_DID_CELL_VOLTAGES 0x0100U
#define UDS_DID_BAL_START 0x0200U
#define UDS_DID_BAL_STOP 0x0201U

#define UDS_SESSION_DEFAULT 0x01U
#define UDS_SESSION_EXTENDED 0x03U

static const UdsHooks_t s_empty_hooks = {0};

static uint16_t Negative(uint8_t sid, uint8_t nrc, uint8_t *resp, uint16_t cap)
{
    if (cap < 3U)
    {
        return 0U;
    }
    resp[0] = 0x7FU;
    resp[1] = sid;
    resp[2] = nrc;
    return 3U;
}

static uint16_t Suppressed(uint8_t sub_function)
{
    return ((sub_function & 0x80U) != 0U) ? 1U : 0U;
}
static uint16_t HandleSessionControl(UdsServer_t *uds,
                                     const uint8_t *req,
                                     uint16_t req_len,
                                     uint8_t *resp,
                                     uint16_t cap,
                                     uint32_t now_ms)
{
    (void)now_ms;
    if (req_len != 2U)
    {
        return Negative(req[0], UDS_NRC_INCORRECT_LENGTH, resp, cap);
    }

    uint8_t sub = req[1];
    uint8_t session = (uint8_t)(sub & 0x7FU);

    if ((session != UDS_SESSION_DEFAULT) && (session != UDS_SESSION_EXTENDED))
    {
        return Negative(req[0], UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, resp, cap);
    }
    if (cap < 6U)
    {
        return 0U;
    }

    uds->extended = (session == UDS_SESSION_EXTENDED);
    uds->session_change = true;

    resp[0] = 0x50U;
    resp[1] = session;
    resp[2] = 0x00U; /* P2 server max: 50 ms */
    resp[3] = 0x32U;
    resp[4] = 0x01U; /* P2* server max: 5000 ms = 0x01F4 */
    resp[5] = 0xF4U;

    if (Suppressed(sub) != 0U)
    {
        return 0U;
    }
    return 6U;
}

static uint16_t HandleTesterPresent(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t cap)
{
    if (req_len != 2U)
    {
        return Negative(req[0], UDS_NRC_INCORRECT_LENGTH, resp, cap);
    }

    uint8_t sub = req[1];
    if ((sub & 0x7FU) != 0x00U)
    {
        return Negative(req[0], UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, resp, cap);
    }
    if (Suppressed(sub) != 0U)
    {
        return 0U;
    }
    if (cap < 2U)
    {
        return 0U;
    }
    resp[0] = 0x7EU;
    resp[1] = 0x00U;
    return 2U;
}

static uint16_t HandleReadDid(const UdsServer_t *uds,
                              const uint8_t *req,
                              uint16_t req_len,
                              uint8_t *resp,
                              uint16_t cap)
{
    if (req_len != 3U)
    {
        return Negative(req[0], UDS_NRC_INCORRECT_LENGTH, resp, cap);
    }

    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8U) | (uint16_t)req[2]);

    switch (did)
    {
    case UDS_DID_SW_VERSION:
    {
        const char *version = (uds->hooks->sw_version != NULL) ? uds->hooks->sw_version : "";
        size_t length = strlen(version);
        if ((uint32_t)cap < (uint32_t)(3U + length))
        {
            return 0U;
        }
        resp[0] = 0x62U;
        resp[1] = req[1];
        resp[2] = req[2];
        (void)memcpy(&resp[3], version, length);
        return (uint16_t)(3U + length);
    }

    case UDS_DID_PACK_V_I:
    {
        if ((uds->hooks->read_pack_voltage_mv == NULL) || (uds->hooks->read_pack_current_ma == NULL))
        {
            return Negative(req[0], UDS_NRC_CONDITIONS_NOT_CORRECT, resp, cap);
        }
        if (cap < 9U)
        {
            return 0U;
        }
        uint16_t mv = uds->hooks->read_pack_voltage_mv();
        int32_t ma = uds->hooks->read_pack_current_ma();
        uint32_t current_raw = (uint32_t)ma;

        resp[0] = 0x62U;
        resp[1] = req[1];
        resp[2] = req[2];
        resp[3] = (uint8_t)(mv >> 8U);
        resp[4] = (uint8_t)(mv & 0xFFU);
        resp[5] = (uint8_t)((current_raw >> 24U) & 0xFFU);
        resp[6] = (uint8_t)((current_raw >> 16U) & 0xFFU);
        resp[7] = (uint8_t)((current_raw >> 8U) & 0xFFU);
        resp[8] = (uint8_t)(current_raw & 0xFFU);
        return 9U;
    }

    case UDS_DID_CELL_VOLTAGES:
    {
        if (uds->hooks->read_cell_mv == NULL)
        {
            return Negative(req[0], UDS_NRC_CONDITIONS_NOT_CORRECT, resp, cap);
        }
        uint16_t cells[UDS_CELL_COUNT] = {0};
        if (!uds->hooks->read_cell_mv(cells, UDS_CELL_COUNT))
        {
            return Negative(req[0], UDS_NRC_CONDITIONS_NOT_CORRECT, resp, cap);
        }
        if (cap < (uint16_t)(3U + (UDS_CELL_COUNT * 2U)))
        {
            return 0U;
        }
        resp[0] = 0x62U;
        resp[1] = req[1];
        resp[2] = req[2];
        for (uint8_t i = 0U; i < UDS_CELL_COUNT; ++i)
        {
            resp[3U + (i * 2U)] = (uint8_t)(cells[i] >> 8U);
            resp[4U + (i * 2U)] = (uint8_t)(cells[i] & 0xFFU);
        }
        return (uint16_t)(3U + (UDS_CELL_COUNT * 2U));
    }

    default:
        return Negative(req[0], UDS_NRC_REQUEST_OUT_OF_RANGE, resp, cap);
    }
}

static uint16_t HandleWriteDid(UdsServer_t *uds,
                               const uint8_t *req,
                               uint16_t req_len,
                               uint8_t *resp,
                               uint16_t cap)
{
    if (!uds->extended)
    {
        return Negative(req[0], UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION, resp, cap);
    }
    if (req_len != 5U)
    {
        return Negative(req[0], UDS_NRC_INCORRECT_LENGTH, resp, cap);
    }

    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8U) | (uint16_t)req[2]);
    uint16_t value = (uint16_t)(((uint16_t)req[3] << 8U) | (uint16_t)req[4]);

    if ((value < 5U) || (value > 50U))
    {
        return Negative(req[0], UDS_NRC_REQUEST_OUT_OF_RANGE, resp, cap);
    }

    bool accepted = false;
    if (did == UDS_DID_BAL_START)
    {
        accepted = (uds->hooks->write_bal_start != NULL) && uds->hooks->write_bal_start(value);
    }
    else if (did == UDS_DID_BAL_STOP)
    {
        accepted = (uds->hooks->write_bal_stop != NULL) && uds->hooks->write_bal_stop(value);
    }
    else
    {
        return Negative(req[0], UDS_NRC_REQUEST_OUT_OF_RANGE, resp, cap);
    }

    if (!accepted)
    {
        /* Most likely STOP >= START; the balancing module owns that rule. */
        return Negative(req[0], UDS_NRC_REQUEST_OUT_OF_RANGE, resp, cap);
    }

    if (cap < 3U)
    {
        return 0U;
    }
    resp[0] = 0x6EU;
    resp[1] = req[1];
    resp[2] = req[2];
    return 3U;
}

static uint16_t HandleReadDtc(const UdsServer_t *uds,
                              const uint8_t *req,
                              uint16_t req_len,
                              uint8_t *resp,
                              uint16_t cap)
{
    if (req_len != 3U)
    {
        return Negative(req[0], UDS_NRC_INCORRECT_LENGTH, resp, cap);
    }
    if (req[1] != 0x02U)
    {
        return Negative(req[0], UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, resp, cap);
    }
    if (cap < 3U)
    {
        return 0U;
    }

    uint8_t status_mask = req[2];
    uint8_t availability = 0xFFU;
    if (uds->hooks->dtc_status_mask != NULL)
    {
        availability = uds->hooks->dtc_status_mask();
    }

    resp[0] = 0x59U;
    resp[1] = 0x02U;
    resp[2] = availability;
    uint16_t length = 3U;

    uint8_t count = (uds->hooks->dtc_count != NULL) ? uds->hooks->dtc_count() : 0U;
    if (count > 32U)
    {
        count = 32U;
    }

    for (uint8_t i = 0U; (i < count) && (uds->hooks->dtc_by_index != NULL); ++i)
    {
        uint32_t dtc = 0U;
        uint8_t status = 0U;
        if (!uds->hooks->dtc_by_index(i, &dtc, &status))
        {
            break;
        }
        if ((status & status_mask) == 0U)
        {
            continue;
        }
        if ((uint32_t)cap < (uint32_t)(length + 4U))
        {
            break;
        }
        resp[length + 0U] = (uint8_t)((dtc >> 16U) & 0xFFU);
        resp[length + 1U] = (uint8_t)((dtc >> 8U) & 0xFFU);
        resp[length + 2U] = (uint8_t)(dtc & 0xFFU);
        resp[length + 3U] = status;
        length = (uint16_t)(length + 4U);
    }

    return length;
}

static uint16_t HandleClearDtc(const UdsServer_t *uds,
                               const uint8_t *req,
                               uint16_t req_len,
                               uint8_t *resp,
                               uint16_t cap)
{
    if (req_len != 4U)
    {
        return Negative(req[0], UDS_NRC_INCORRECT_LENGTH, resp, cap);
    }

    uint32_t group = ((uint32_t)req[1] << 16U) | ((uint32_t)req[2] << 8U) | (uint32_t)req[3];
    if (group != 0x00FFFFFFUL)
    {
        return Negative(req[0], UDS_NRC_REQUEST_OUT_OF_RANGE, resp, cap);
    }
    if (uds->hooks->clear_dtcs == NULL)
    {
        return Negative(req[0], UDS_NRC_CONDITIONS_NOT_CORRECT, resp, cap);
    }

    uds->hooks->clear_dtcs();

    if (cap < 1U)
    {
        return 0U;
    }
    resp[0] = 0x54U;
    return 1U;
}

void Uds_Init(UdsServer_t *uds, const UdsHooks_t *hooks, uint32_t now_ms)
{
    if (uds == NULL)
    {
        return;
    }
    (void)memset(uds, 0, sizeof(*uds));
    uds->hooks = hooks;
    uds->last_request_ms = now_ms;
}

uint16_t Uds_Process(UdsServer_t *uds,
                     const uint8_t *req,
                     uint16_t req_len,
                     uint8_t *resp,
                     uint16_t resp_cap,
                     uint32_t now_ms)
{
    if ((uds == NULL) || (req == NULL) || (resp == NULL) || (req_len == 0U) || (resp_cap < 3U))
    {
        return 0U;
    }
    if (uds->hooks == NULL)
    {
        uds->hooks = &s_empty_hooks;
    }

    uds->last_request_ms = now_ms;
    uds->request_count++;

    uint16_t length = 0U;
    switch (req[0])
    {
    case UDS_SID_SESSION_CONTROL:
        length = HandleSessionControl(uds, req, req_len, resp, resp_cap, now_ms);
        break;
    case UDS_SID_TESTER_PRESENT:
        length = HandleTesterPresent(req, req_len, resp, resp_cap);
        break;
    case UDS_SID_READ_DID:
        length = HandleReadDid(uds, req, req_len, resp, resp_cap);
        break;
    case UDS_SID_WRITE_DID:
        length = HandleWriteDid(uds, req, req_len, resp, resp_cap);
        break;
    case UDS_SID_READ_DTC:
        length = HandleReadDtc(uds, req, req_len, resp, resp_cap);
        break;
    case UDS_SID_CLEAR_DTC:
        length = HandleClearDtc(uds, req, req_len, resp, resp_cap);
        break;
    case UDS_SID_ECU_RESET:
    default:
        return Negative(req[0], UDS_NRC_SERVICE_NOT_SUPPORTED, resp, resp_cap);
    }

    if ((length == 3U) && (resp[0] == 0x7FU))
    {
        uds->nrc_count++;
        uds->last_nrc = resp[2];
    }
    return length;
}

void Uds_Poll(UdsServer_t *uds, uint32_t now_ms)
{
    if (uds == NULL)
    {
        return;
    }
    if (uds->extended && ((now_ms - uds->last_request_ms) > UDS_S3_TIMEOUT_MS))
    {
        uds->extended = false;
        uds->session_change = true;
    }
}

bool Uds_IsExtended(const UdsServer_t *uds)
{
    return (uds == NULL) ? false : uds->extended;
}
