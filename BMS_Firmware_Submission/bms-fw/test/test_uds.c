#include "unity.h"

#include "test_groups.h"

#include "services/isotp.h"
#include "services/uds.h"

#include <string.h>

static uint32_t s_tx_frames;
static uint32_t s_tx_id;
static uint8_t s_tx_data[8];
static uint8_t s_tx_dlc;

static bool SendStub(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    s_tx_frames++;
    s_tx_id = id;
    s_tx_dlc = dlc;
    (void)memset(s_tx_data, 0, sizeof(s_tx_data));
    (void)memcpy(s_tx_data, data, (dlc < 8U) ? dlc : 8U);
    return true;
}

static uint16_t s_pack_mv;
static int32_t s_current_ma;
static uint16_t s_cells[UDS_CELL_COUNT];
static bool s_cells_valid;
static uint16_t s_bal_start;
static uint16_t s_bal_stop;
static uint8_t s_dtc_count;
static uint32_t s_dtcs[8];
static uint8_t s_dtc_status[8];
static uint8_t s_status_mask;
static bool s_cleared;

static uint16_t HookPackMv(void)
{
    return s_pack_mv;
}

static int32_t HookCurrentMa(void)
{
    return s_current_ma;
}

static bool HookReadCells(uint16_t *cells, uint8_t count)
{
    if (!s_cells_valid || (cells == NULL))
    {
        return false;
    }
    for (uint8_t i = 0U; i < count; ++i)
    {
        cells[i] = (i < UDS_CELL_COUNT) ? s_cells[i] : 0U;
    }
    return true;
}

static bool HookWriteBalStart(uint16_t mv)
{
    if (mv <= s_bal_stop)
    {
        return false;
    }
    s_bal_start = mv;
    return true;
}

static bool HookWriteBalStop(uint16_t mv)
{
    if (mv >= s_bal_start)
    {
        return false;
    }
    s_bal_stop = mv;
    return true;
}

static uint8_t HookDtcCount(void)
{
    return s_dtc_count;
}

static bool HookDtcByIndex(uint8_t index, uint32_t *dtc, uint8_t *status)
{
    if ((index >= s_dtc_count) || (dtc == NULL) || (status == NULL))
    {
        return false;
    }
    *dtc = s_dtcs[index];
    *status = s_dtc_status[index];
    return true;
}

static void HookClearDtcs(void)
{
    s_cleared = true;
}

static uint8_t HookStatusMask(void)
{
    return s_status_mask;
}

static const UdsHooks_t s_hooks = {
    .sw_version = "TEST-1.0",
    .read_pack_voltage_mv = HookPackMv,
    .read_pack_current_ma = HookCurrentMa,
    .read_cell_mv = HookReadCells,
    .write_bal_start = HookWriteBalStart,
    .write_bal_stop = HookWriteBalStop,
    .dtc_count = HookDtcCount,
    .dtc_by_index = HookDtcByIndex,
    .clear_dtcs = HookClearDtcs,
    .dtc_status_mask = HookStatusMask,
};

static void ResetHooks(void)
{
    s_pack_mv = 51200U;
    s_current_ma = -1234;
    for (uint8_t i = 0U; i < UDS_CELL_COUNT; ++i)
    {
        s_cells[i] = (uint16_t)(3300U + i);
    }
    s_cells_valid = true;
    s_bal_start = 15U;
    s_bal_stop = 5U;
    s_dtc_count = 0U;
    (void)memset(s_dtcs, 0, sizeof(s_dtcs));
    (void)memset(s_dtc_status, 0, sizeof(s_dtc_status));
    s_status_mask = 0xFFU;
    s_cleared = false;
}

static uint16_t Process(UdsServer_t *uds, const uint8_t *req, uint16_t len, uint8_t *resp, uint32_t now)
{
    return Uds_Process(uds, req, len, resp, UDS_MAX_RESPONSE, now);
}

static void ExpectNrc(UdsServer_t *uds, const uint8_t *req, uint16_t len, uint8_t sid, uint8_t nrc)
{
    uint8_t resp[UDS_MAX_RESPONSE];
    uint16_t out = Process(uds, req, len, resp, 0U);
    TEST_ASSERT_EQUAL_UINT16(3U, out);
    TEST_ASSERT_EQUAL_HEX8(0x7FU, resp[0]);
    TEST_ASSERT_EQUAL_HEX8(sid, resp[1]);
    TEST_ASSERT_EQUAL_HEX8(nrc, resp[2]);
}

static void test_session_control(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t extended[] = {0x10U, 0x03U};
    TEST_ASSERT_EQUAL_UINT16(6U, Process(&uds, extended, 2U, resp, 0U));
    TEST_ASSERT_EQUAL_HEX8(0x50U, resp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03U, resp[1]);
    TEST_ASSERT_TRUE(Uds_IsExtended(&uds));

    const uint8_t default_session[] = {0x10U, 0x01U};
    TEST_ASSERT_EQUAL_UINT16(6U, Process(&uds, default_session, 2U, resp, 1U));
    TEST_ASSERT_FALSE(Uds_IsExtended(&uds));

    const uint8_t bad_session[] = {0x10U, 0x02U};
    ExpectNrc(&uds, bad_session, 2U, 0x10U, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);

    const uint8_t bad_length[] = {0x10U, 0x03U, 0x00U};
    ExpectNrc(&uds, bad_length, 3U, 0x10U, UDS_NRC_INCORRECT_LENGTH);

    /* Suppress-positive-response bit. */
    const uint8_t suppressed[] = {0x10U, 0x83U};
    TEST_ASSERT_EQUAL_UINT16(0U, Process(&uds, suppressed, 2U, resp, 2U));
    TEST_ASSERT_TRUE(Uds_IsExtended(&uds));
}

static void test_tester_present(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t present[] = {0x3EU, 0x00U};
    TEST_ASSERT_EQUAL_UINT16(2U, Process(&uds, present, 2U, resp, 0U));
    TEST_ASSERT_EQUAL_HEX8(0x7EU, resp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, resp[1]);

    const uint8_t suppressed[] = {0x3EU, 0x80U};
    TEST_ASSERT_EQUAL_UINT16(0U, Process(&uds, suppressed, 2U, resp, 10U));

    const uint8_t bad_sub[] = {0x3EU, 0x01U};
    ExpectNrc(&uds, bad_sub, 2U, 0x3EU, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);

    const uint8_t bad_length[] = {0x3EU};
    ExpectNrc(&uds, bad_length, 1U, 0x3EU, UDS_NRC_INCORRECT_LENGTH);
}

static void test_read_data_by_identifier(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t sw_version[] = {0x22U, 0xF1U, 0x89U};
    uint16_t out = Process(&uds, sw_version, 3U, resp, 0U);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)(3U + strlen("TEST-1.0")), out);
    TEST_ASSERT_EQUAL_HEX8(0x62U, resp[0]);
    TEST_ASSERT_EQUAL_HEX8(0xF1U, resp[1]);
    TEST_ASSERT_EQUAL_HEX8(0x89U, resp[2]);
    TEST_ASSERT_EQUAL_MEMORY("TEST-1.0", &resp[3], strlen("TEST-1.0"));

    const uint8_t pack_vi[] = {0x22U, 0x01U, 0x01U};
    out = Process(&uds, pack_vi, 3U, resp, 0U);
    TEST_ASSERT_EQUAL_UINT16(9U, out);
    TEST_ASSERT_EQUAL_HEX8(0xC8U, resp[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, resp[4]); /* 51200 mV */
    TEST_ASSERT_EQUAL_HEX8(0xFFU, resp[5]); /* -1234 = 0xFFFFFB2E */
    TEST_ASSERT_EQUAL_HEX8(0xFBU, resp[7]);
    TEST_ASSERT_EQUAL_HEX8(0x2EU, resp[8]);

    const uint8_t cell_voltages[] = {0x22U, 0x01U, 0x00U};
    out = Process(&uds, cell_voltages, 3U, resp, 0U);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)(3U + (UDS_CELL_COUNT * 2U)), out);
    TEST_ASSERT_EQUAL_HEX8(0x0CU, resp[3]); /* 3300 = 0x0CE4 */
    TEST_ASSERT_EQUAL_HEX8(0xE4U, resp[4]);
    TEST_ASSERT_EQUAL_HEX8(0x0CU, resp[5]); /* 3301 = 0x0CE5 */
    TEST_ASSERT_EQUAL_HEX8(0xE5U, resp[6]);

    s_cells_valid = false;
    ExpectNrc(&uds, cell_voltages, 3U, 0x22U, UDS_NRC_CONDITIONS_NOT_CORRECT);

    const uint8_t unknown[] = {0x22U, 0x12U, 0x34U};
    ExpectNrc(&uds, unknown, 3U, 0x22U, UDS_NRC_REQUEST_OUT_OF_RANGE);

    const uint8_t bad_length[] = {0x22U, 0x01U};
    ExpectNrc(&uds, bad_length, 2U, 0x22U, UDS_NRC_INCORRECT_LENGTH);
}

static void test_write_data_by_identifier(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t write_start[] = {0x2EU, 0x02U, 0x00U, 0x00U, 0x14U}; /* 20 mV */

    /* Service is not allowed in the default session. */
    ExpectNrc(&uds, write_start, 5U, 0x2EU, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);

    const uint8_t extended[] = {0x10U, 0x03U};
    (void)Process(&uds, extended, 2U, resp, 10U);

    TEST_ASSERT_EQUAL_UINT16(3U, Process(&uds, write_start, 5U, resp, 20U));
    TEST_ASSERT_EQUAL_HEX8(0x6EU, resp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02U, resp[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, resp[2]);
    TEST_ASSERT_EQUAL_UINT16(20U, s_bal_start);

    const uint8_t write_stop[] = {0x2EU, 0x02U, 0x01U, 0x00U, 0x0AU}; /* 10 mV */
    TEST_ASSERT_EQUAL_UINT16(3U, Process(&uds, write_stop, 5U, resp, 30U));
    TEST_ASSERT_EQUAL_UINT16(10U, s_bal_stop);

    /* STOP >= START is rejected by the balancing rule, reported as NRC 31. */
    s_bal_stop = 25U;
    ExpectNrc(&uds, write_start, 5U, 0x2EU, UDS_NRC_REQUEST_OUT_OF_RANGE);

    const uint8_t out_of_range_low[] = {0x2EU, 0x02U, 0x00U, 0x00U, 0x04U};
    ExpectNrc(&uds, out_of_range_low, 5U, 0x2EU, UDS_NRC_REQUEST_OUT_OF_RANGE);
    const uint8_t out_of_range_high[] = {0x2EU, 0x02U, 0x00U, 0x00U, 0x33U};
    ExpectNrc(&uds, out_of_range_high, 5U, 0x2EU, UDS_NRC_REQUEST_OUT_OF_RANGE);

    const uint8_t unknown_did[] = {0x2EU, 0x12U, 0x34U, 0x00U, 0x10U};
    ExpectNrc(&uds, unknown_did, 5U, 0x2EU, UDS_NRC_REQUEST_OUT_OF_RANGE);

    const uint8_t bad_length[] = {0x2EU, 0x02U, 0x00U, 0x00U};
    ExpectNrc(&uds, bad_length, 4U, 0x2EU, UDS_NRC_INCORRECT_LENGTH);
}

static void test_read_dtc(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t no_dtc[] = {0x19U, 0x02U, 0xFFU};
    TEST_ASSERT_EQUAL_UINT16(3U, Process(&uds, no_dtc, 3U, resp, 0U));
    TEST_ASSERT_EQUAL_HEX8(0x59U, resp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02U, resp[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, resp[2]);

    s_dtc_count = 2U;
    s_dtcs[0] = 0x0A01U;
    s_dtc_status[0] = 0x09U;
    s_dtcs[1] = 0x0A06U;
    s_dtc_status[1] = 0x01U;

    uint16_t out = Process(&uds, no_dtc, 3U, resp, 0U);
    TEST_ASSERT_EQUAL_UINT16(11U, out);
    TEST_ASSERT_EQUAL_HEX8(0x00U, resp[3]);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, resp[4]);
    TEST_ASSERT_EQUAL_HEX8(0x01U, resp[5]);
    TEST_ASSERT_EQUAL_HEX8(0x09U, resp[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, resp[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, resp[8]);
    TEST_ASSERT_EQUAL_HEX8(0x06U, resp[9]);

    /* Status mask filters the reported records. */
    s_status_mask = 0x08U;
    const uint8_t mask_req[] = {0x19U, 0x02U, 0x08U};
    out = Process(&uds, mask_req, 3U, resp, 0U);
    TEST_ASSERT_EQUAL_UINT16(7U, out);
    TEST_ASSERT_EQUAL_HEX8(0x08U, resp[2]);

    const uint8_t wrong_sub[] = {0x19U, 0x01U, 0xFFU};
    ExpectNrc(&uds, wrong_sub, 3U, 0x19U, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);

    const uint8_t bad_length[] = {0x19U, 0x02U};
    ExpectNrc(&uds, bad_length, 2U, 0x19U, UDS_NRC_INCORRECT_LENGTH);
}

static void test_clear_dtc(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t clear[] = {0x14U, 0xFFU, 0xFFU, 0xFFU};
    TEST_ASSERT_EQUAL_UINT16(1U, Process(&uds, clear, 4U, resp, 0U));
    TEST_ASSERT_EQUAL_HEX8(0x54U, resp[0]);
    TEST_ASSERT_TRUE(s_cleared);

    const uint8_t bad_group[] = {0x14U, 0x00U, 0x00U, 0x01U};
    ExpectNrc(&uds, bad_group, 4U, 0x14U, UDS_NRC_REQUEST_OUT_OF_RANGE);

    const uint8_t bad_length[] = {0x14U, 0xFFU};
    ExpectNrc(&uds, bad_length, 2U, 0x14U, UDS_NRC_INCORRECT_LENGTH);
}

static void test_unsupported_and_guards(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t reset[] = {0x11U, 0x01U};
    ExpectNrc(&uds, reset, 2U, 0x11U, UDS_NRC_SERVICE_NOT_SUPPORTED);

    const uint8_t unknown[] = {0x31U, 0x01U, 0x02U};
    ExpectNrc(&uds, unknown, 3U, 0x31U, UDS_NRC_SERVICE_NOT_SUPPORTED);

    TEST_ASSERT_EQUAL_UINT16(0U, Uds_Process(NULL, unknown, 3U, resp, sizeof(resp), 0U));
    TEST_ASSERT_EQUAL_UINT16(0U, Uds_Process(&uds, NULL, 3U, resp, sizeof(resp), 0U));
    TEST_ASSERT_EQUAL_UINT16(0U, Uds_Process(&uds, unknown, 3U, NULL, sizeof(resp), 0U));
    TEST_ASSERT_EQUAL_UINT16(0U, Uds_Process(&uds, unknown, 0U, resp, sizeof(resp), 0U));
    TEST_ASSERT_EQUAL_UINT16(0U, Uds_Process(&uds, unknown, 3U, resp, 2U, 0U));

    /* A server with no hooks answers with NRC 0x22 rather than crashing. */
    UdsServer_t bare;
    Uds_Init(&bare, NULL, 0U);
    const uint8_t pack_vi[] = {0x22U, 0x01U, 0x01U};
    ExpectNrc(&bare, pack_vi, 3U, 0x22U, UDS_NRC_CONDITIONS_NOT_CORRECT);

    const uint8_t clear[] = {0x14U, 0xFFU, 0xFFU, 0xFFU};
    ExpectNrc(&bare, clear, 4U, 0x14U, UDS_NRC_CONDITIONS_NOT_CORRECT);

    TEST_ASSERT_FALSE(Uds_IsExtended(NULL));
}

static void test_s3_session_timeout(void)
{
    ResetHooks();
    UdsServer_t uds;
    Uds_Init(&uds, &s_hooks, 0U);
    uint8_t resp[UDS_MAX_RESPONSE];

    const uint8_t extended[] = {0x10U, 0x03U};
    (void)Process(&uds, extended, 2U, resp, 0U);
    TEST_ASSERT_TRUE(Uds_IsExtended(&uds));

    Uds_Poll(&uds, 5000U);
    TEST_ASSERT_TRUE(Uds_IsExtended(&uds));
    Uds_Poll(&uds, 5001U);
    TEST_ASSERT_FALSE(Uds_IsExtended(&uds));

    Uds_Poll(NULL, 100U);
}

static void test_isotp_single_frame_rx(void)
{
    IsoTp_t tp;
    IsoTp_Init(&tp, UDS_REQUEST_ID, UDS_RESPONSE_ID, SendStub);

    const uint8_t frame[8] = {0x02U, 0x10U, 0x03U, 0U, 0U, 0U, 0U, 0U};
    TEST_ASSERT_TRUE(IsoTp_OnFrame(&tp, frame, 3U, 0U));
    TEST_ASSERT_TRUE(IsoTp_IsBusy(&tp));

    uint8_t out[8] = {0};
    TEST_ASSERT_EQUAL_UINT16(2U, IsoTp_Read(&tp, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8(0x10U, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03U, out[1]);
    TEST_ASSERT_EQUAL_UINT16(0U, IsoTp_Read(&tp, out, sizeof(out)));

    /* Invalid length and PCI types are dropped. */
    const uint8_t empty[8] = {0x00U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, empty, 8U, 0U));
    const uint8_t bad_pci[8] = {0x70U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, bad_pci, 8U, 0U));
    TEST_ASSERT_TRUE(tp.rx_dropped >= 2U);

    TEST_ASSERT_FALSE(IsoTp_OnFrame(NULL, frame, 3U, 0U));
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, NULL, 3U, 0U));
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, frame, 0U, 0U));
    TEST_ASSERT_EQUAL_UINT16(0U, IsoTp_Read(NULL, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT16(0U, IsoTp_Read(&tp, NULL, sizeof(out)));
}

static void test_isotp_multi_frame_rx(void)
{
    IsoTp_t tp;
    IsoTp_Init(&tp, UDS_REQUEST_ID, UDS_RESPONSE_ID, SendStub);
    s_tx_frames = 0U;

    /* First frame announces 10 bytes and carries the first 6. */
    const uint8_t ff[8] = {0x10U, 0x0AU, 1U, 2U, 3U, 4U, 5U, 6U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, ff, 8U, 0U));
    TEST_ASSERT_TRUE(tp.rx_active);
    TEST_ASSERT_EQUAL_UINT32(1U, s_tx_frames);
    TEST_ASSERT_EQUAL_HEX8(0x30U, s_tx_data[0]); /* flow control CTS */
    TEST_ASSERT_EQUAL_HEX32(UDS_RESPONSE_ID, s_tx_id);

    /* A wrong sequence number aborts the transfer. */
    const uint8_t wrong_cf[8] = {0x22U, 7U, 8U, 9U, 10U, 0U, 0U, 0U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, wrong_cf, 5U, 10U));
    TEST_ASSERT_FALSE(tp.rx_active);

    /* Restart and complete the transfer. */
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, ff, 8U, 20U));
    const uint8_t cf[8] = {0x21U, 7U, 8U, 9U, 10U, 0U, 0U, 0U};
    TEST_ASSERT_TRUE(IsoTp_OnFrame(&tp, cf, 5U, 30U));

    uint8_t out[ISOTP_MAX_PAYLOAD] = {0};
    TEST_ASSERT_EQUAL_UINT16(10U, IsoTp_Read(&tp, out, sizeof(out)));
    for (uint8_t i = 0U; i < 10U; ++i)
    {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(i + 1U), out[i]);
    }

    /* First frame larger than the buffer is rejected with FC overflow. */
    const uint8_t too_big[8] = {0x10U, 0xFFU, 0U, 0U, 0U, 0U, 0U, 0U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, too_big, 8U, 40U));
    TEST_ASSERT_EQUAL_HEX8(0x32U, s_tx_data[0]);

    /* Reception timeout discards a partial message. */
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, ff, 8U, 50U));
    IsoTp_Poll(&tp, 50U + ISOTP_CF_TIMEOUT_MS + 1U);
    TEST_ASSERT_FALSE(tp.rx_active);
}

static void test_isotp_multi_frame_tx(void)
{
    IsoTp_t tp;
    IsoTp_Init(&tp, UDS_REQUEST_ID, UDS_RESPONSE_ID, SendStub);
    s_tx_frames = 0U;

    const uint8_t payload[10] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U};
    TEST_ASSERT_TRUE(IsoTp_Write(&tp, payload, 10U, 0U));
    TEST_ASSERT_EQUAL_UINT32(1U, s_tx_frames);
    TEST_ASSERT_EQUAL_HEX8(0x10U, s_tx_data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, s_tx_data[1]);
    TEST_ASSERT_TRUE(tp.tx_wait_fc);

    /* Without a Flow Control the transmission is aborted after 1 s. */
    IsoTp_Poll(&tp, ISOTP_FC_TIMEOUT_MS + 1U);
    TEST_ASSERT_FALSE(tp.tx_active);
    TEST_ASSERT_EQUAL_UINT32(1U, tp.tx_aborted);

    /* Retry: accept the flow control and send the consecutive frames. */
    s_tx_frames = 0U;
    TEST_ASSERT_TRUE(IsoTp_Write(&tp, payload, 10U, 100U));
    const uint8_t fc[8] = {0x30U, 0x00U, 0x00U, 0U, 0U, 0U, 0U, 0U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, fc, 3U, 110U));
    TEST_ASSERT_FALSE(tp.tx_wait_fc);

    IsoTp_Poll(&tp, 110U);
    TEST_ASSERT_FALSE(tp.tx_active);
    TEST_ASSERT_EQUAL_UINT32(2U, s_tx_frames); /* FF + one CF */
    TEST_ASSERT_EQUAL_HEX8(0x21U, s_tx_data[0]);

    /* Single-frame response is sent immediately. */
    s_tx_frames = 0U;
    const uint8_t short_payload[3] = {0x50U, 0x03U, 0x00U};
    TEST_ASSERT_TRUE(IsoTp_Write(&tp, short_payload, 3U, 200U));
    TEST_ASSERT_EQUAL_UINT32(1U, s_tx_frames);
    TEST_ASSERT_EQUAL_HEX8(0x03U, s_tx_data[0]);
    TEST_ASSERT_FALSE(tp.tx_active);

    /* Invalid writes are rejected. */
    TEST_ASSERT_FALSE(IsoTp_Write(&tp, NULL, 3U, 300U));
    TEST_ASSERT_FALSE(IsoTp_Write(&tp, short_payload, 0U, 300U));
    TEST_ASSERT_FALSE(IsoTp_Write(&tp, short_payload, ISOTP_MAX_PAYLOAD + 1U, 300U));
}

static void test_isotp_stmin_and_block_size(void)
{
    IsoTp_t tp;
    IsoTp_Init(&tp, UDS_REQUEST_ID, UDS_RESPONSE_ID, SendStub);
    s_tx_frames = 0U;

    const uint8_t payload[20] = {0};
    TEST_ASSERT_TRUE(IsoTp_Write(&tp, payload, 20U, 0U));

    /* Flow control with block size 2 and STmin 5 ms. */
    const uint8_t fc[8] = {0x30U, 0x02U, 0x05U, 0U, 0U, 0U, 0U, 0U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, fc, 3U, 1U));
    TEST_ASSERT_EQUAL_UINT32(5U, tp.tx_stmin_ms);

    IsoTp_Poll(&tp, 1U);
    TEST_ASSERT_EQUAL_UINT32(2U, s_tx_frames); /* FF + first CF */

    /* Separation time not elapsed yet. */
    IsoTp_Poll(&tp, 4U);
    TEST_ASSERT_EQUAL_UINT32(2U, s_tx_frames);

    IsoTp_Poll(&tp, 6U);
    TEST_ASSERT_EQUAL_UINT32(3U, s_tx_frames);

    /* Block size reached: wait for another flow control. */
    TEST_ASSERT_TRUE(tp.tx_wait_fc);
    const uint8_t fc2[8] = {0x30U, 0x00U, 0x00U, 0U, 0U, 0U, 0U, 0U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, fc2, 3U, 10U));
    IsoTp_Poll(&tp, 10U);
    TEST_ASSERT_FALSE(tp.tx_active);
    TEST_ASSERT_EQUAL_UINT32(4U, s_tx_frames); /* FF + 3 CF */

    /* Flow control overflow aborts. */
    s_tx_frames = 0U;
    TEST_ASSERT_TRUE(IsoTp_Write(&tp, payload, 20U, 100U));
    const uint8_t fc_overflow[8] = {0x32U, 0x00U, 0x00U, 0U, 0U, 0U, 0U, 0U};
    TEST_ASSERT_FALSE(IsoTp_OnFrame(&tp, fc_overflow, 3U, 110U));
    TEST_ASSERT_FALSE(tp.tx_active);
    TEST_ASSERT_EQUAL_UINT32(1U, tp.tx_aborted);
}

void run_uds_tests(void)
{
    RUN_TEST(test_session_control);
    RUN_TEST(test_tester_present);
    RUN_TEST(test_read_data_by_identifier);
    RUN_TEST(test_write_data_by_identifier);
    RUN_TEST(test_read_dtc);
    RUN_TEST(test_clear_dtc);
    RUN_TEST(test_unsupported_and_guards);
    RUN_TEST(test_s3_session_timeout);
    RUN_TEST(test_isotp_single_frame_rx);
    RUN_TEST(test_isotp_multi_frame_rx);
    RUN_TEST(test_isotp_multi_frame_tx);
    RUN_TEST(test_isotp_stmin_and_block_size);
}
