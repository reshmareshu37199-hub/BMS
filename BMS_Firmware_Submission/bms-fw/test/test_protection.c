#include "unity.h"

#include "test_groups.h"

#include "services/protection.h"

#include <string.h>

static ProtInput_t MakeInput(void)
{
    ProtInput_t in;
    (void)memset(&in, 0, sizeof(in));

    for (uint8_t i = 0U; i < PROT_CELL_COUNT; ++i)
    {
        in.cell_mv[i] = 3300U;
    }
    for (uint8_t i = 0U; i < PROT_TEMP_COUNT; ++i)
    {
        in.temp_dc[i] = 250;
    }
    in.board_temp_dc = 250;
    in.current_ma = 0;
    in.charging = false;
    in.discharging = false;
    in.cells_valid = true;
    in.afe_comm_fault = false;
    in.vcu_timeout = false;
    in.cell_sum_mismatch = false;
    in.precharge_timeout = false;
    return in;
}

/* Push a fully valid input for `ms` in 10 ms steps so the derating ramp has a
 * realistic amount of time to settle. */
static void Settle(const ProtInput_t *in, uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += 10U)
    {
        Prot_Update(in, 10U);
    }
}

static void test_ov_debounce_and_latch(void)
{
    Prot_Init();

    ProtInput_t in = MakeInput();
    in.cell_mv[3] = 3700U;

    Prot_Update(&in, 500U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CELL_OV));
    TEST_ASSERT_TRUE((Prot_GetDtcStatus(FAULT_CELL_OV) & DTC_STATUS_PENDING) != 0U);

    Prot_Update(&in, 400U); /* 900 ms, still debouncing */
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CELL_OV));

    Prot_Update(&in, 100U); /* 1000 ms -> confirmed */
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CELL_OV));

    ProtOutput_t out = Prot_GetOutput();
    TEST_ASSERT_EQUAL_HEX32(0x00000001UL, out.fault_bits & 0x00000001UL);
    TEST_ASSERT_TRUE(out.contactor_open_request);
    TEST_ASSERT_TRUE(out.latched_fault_active);

    uint8_t status = Prot_GetDtcStatus(FAULT_CELL_OV);
    TEST_ASSERT_TRUE((status & DTC_STATUS_TEST_FAILED) != 0U);
    TEST_ASSERT_TRUE((status & DTC_STATUS_CONFIRMED) != 0U);
    TEST_ASSERT_TRUE((status & DTC_STATUS_TEST_FAILED_SINCE_CLEAR) != 0U);
    TEST_ASSERT_TRUE((status & DTC_STATUS_WARNING) != 0U);

    /* The debounce timer must reset when the condition disappears early. */
    Prot_Init();
    in = MakeInput();
    in.cell_mv[0] = 3700U;
    Prot_Update(&in, 900U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CELL_OV));
    in.cell_mv[0] = 3300U;
    Prot_Update(&in, 10U);
    in.cell_mv[0] = 3700U;
    Prot_Update(&in, 900U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CELL_OV));
}

static void test_latched_fault_needs_release_and_clear(void)
{
    Prot_Init();

    ProtInput_t in = MakeInput();
    in.cell_mv[5] = 3700U;
    Prot_Update(&in, 1000U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CELL_OV));

    /* Release condition met, but a latched fault stays active. */
    in.cell_mv[5] = 3300U;
    Prot_Update(&in, 10U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CELL_OV));

    /* Clearing while the physical condition is still present must fail. */
    in.cell_mv[5] = 3700U;
    Prot_Update(&in, 10U);
    Prot_ClearDtc(0x00FFFFFFUL);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CELL_OV));

    /* Release condition met -> the latched DTC can now be cleared. */
    in.cell_mv[5] = 3300U;
    Prot_Update(&in, 10U);
    Prot_ClearDtc(0x00FFFFFFUL);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CELL_OV));
    TEST_ASSERT_EQUAL_UINT8(0U, Prot_GetDtcCount());
    TEST_ASSERT_EQUAL_UINT8(0U, Prot_GetDtcStatus(FAULT_CELL_OV));
}

static void test_hard_threshold_opens_immediately(void)
{
    Prot_Init();

    /* Over-voltage emergency limit: 3750 mV, before the 1 s debounce. */
    ProtInput_t in = MakeInput();
    in.cell_mv[2] = 3800U;
    Prot_Update(&in, 10U);

    ProtOutput_t out = Prot_GetOutput();
    TEST_ASSERT_TRUE(out.contactor_open_request);
    TEST_ASSERT_TRUE(out.contactor_open_immediate);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CELL_OV));

    /* Under-voltage emergency limit: 2300 mV. */
    Prot_Init();
    in = MakeInput();
    in.cell_mv[2] = 2200U;
    Prot_Update(&in, 10U);
    out = Prot_GetOutput();
    TEST_ASSERT_TRUE(out.contactor_open_request);
    TEST_ASSERT_TRUE(out.contactor_open_immediate);

    /* A normal cell does not trip the emergency path. */
    Prot_Init();
    in = MakeInput();
    Prot_Update(&in, 10U);
    TEST_ASSERT_FALSE(Prot_GetOutput().contactor_open_immediate);
}

static void test_uv_debounce(void)
{
    Prot_Init();

    ProtInput_t in = MakeInput();
    in.cell_mv[0] = 2400U;
    Prot_Update(&in, 999U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CELL_UV));
    Prot_Update(&in, 1U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CELL_UV));
    TEST_ASSERT_TRUE((Prot_GetOutput().fault_bits & 0x00000002UL) != 0U);
    TEST_ASSERT_TRUE(Prot_GetOutput().contactor_open_request);
}

static void test_temperature_faults(void)
{
    /* Charge over-temperature: Tmax >= 55.0 C while charging. */
    Prot_Init();
    ProtInput_t in = MakeInput();
    in.charging = true;
    in.temp_dc[1] = 560;
    Prot_Update(&in, 1999U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CHARGE_OT));
    Prot_Update(&in, 1U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CHARGE_OT));
    TEST_ASSERT_FALSE(Prot_GetOutput().contactor_open_request);

    in.temp_dc[1] = 490;
    Prot_Update(&in, 10U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CHARGE_OT));

    /* Discharge over-temperature: Tmax >= 60.0 C, latched and opens. */
    Prot_Init();
    in = MakeInput();
    in.temp_dc[2] = 610;
    Prot_Update(&in, 2000U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_DISCHARGE_OT));
    TEST_ASSERT_TRUE(Prot_GetOutput().contactor_open_request);
    TEST_ASSERT_TRUE(Prot_GetOutput().latched_fault_active);

    /* Charge under-temperature: Tmin <= 0.0 C while charging. */
    Prot_Init();
    in = MakeInput();
    in.charging = true;
    in.temp_dc[3] = -10;
    Prot_Update(&in, 2000U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CHARGE_UT));
    in.temp_dc[3] = 40;
    Prot_Update(&in, 10U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_CHARGE_UT));
}

static void test_over_current(void)
{
    Prot_Init();
    ProtInput_t in = MakeInput();
    in.current_ma = -200000;
    in.discharging = true;
    Prot_Update(&in, 499U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_DISCHARGE_OC));
    Prot_Update(&in, 1U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_DISCHARGE_OC));
    ProtOutput_t out = Prot_GetOutput();
    TEST_ASSERT_TRUE(out.contactor_open_request);
    TEST_ASSERT_TRUE(out.contactor_open_immediate);
    TEST_ASSERT_TRUE(out.latched_fault_active);

    Prot_Init();
    in = MakeInput();
    in.current_ma = 100000;
    in.charging = true;
    Prot_Update(&in, 1000U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CHARGE_OC));
    TEST_ASSERT_TRUE(Prot_GetOutput().contactor_open_immediate);
}

static void test_binary_faults(void)
{
    Prot_Init();
    ProtInput_t in = MakeInput();
    in.afe_comm_fault = true;
    Prot_Update(&in, 10U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_AFE_COMM));
    TEST_ASSERT_TRUE(Prot_GetOutput().contactor_open_request);
    in.afe_comm_fault = false;
    Prot_Update(&in, 10U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_AFE_COMM)); /* latched */

    Prot_Init();
    in = MakeInput();
    in.vcu_timeout = true;
    Prot_Update(&in, 10U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_VCU_TIMEOUT));
    in.vcu_timeout = false;
    Prot_Update(&in, 10U);
    TEST_ASSERT_FALSE(Prot_IsFaultActive(FAULT_VCU_TIMEOUT));

    Prot_Init();
    in = MakeInput();
    in.cell_sum_mismatch = true;
    Prot_Update(&in, 10U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CELL_SUM_MISMATCH));
    TEST_ASSERT_FALSE(Prot_GetOutput().contactor_open_request);

    Prot_Init();
    in = MakeInput();
    in.precharge_timeout = true;
    Prot_Update(&in, 10U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_PRECHARGE));
    TEST_ASSERT_TRUE(Prot_GetOutput().contactor_open_immediate);
}

static void test_derating_and_ramp(void)
{
    Prot_Init();
    ProtInput_t in = MakeInput();

    /* No steps larger than 100 mA per millisecond. */
    Prot_Update(&in, 10U);
    TEST_ASSERT_EQUAL_INT32(1000, Prot_GetOutput().ccl_ma);
    Prot_Update(&in, 10U);
    TEST_ASSERT_EQUAL_INT32(2000, Prot_GetOutput().ccl_ma);

    /* At 25 C and 3300 mV the charge target is 100 A. */
    Settle(&in, 2000U);
    TEST_ASSERT_EQUAL_INT32(100000, Prot_GetOutput().ccl_ma);
    TEST_ASSERT_TRUE(Prot_GetOutput().charge_enable);

    /* 50 C is the top of the charge derating knee: 50 A. */
    in.temp_dc[0] = 500;
    in.temp_dc[1] = 500;
    in.temp_dc[2] = 500;
    in.temp_dc[3] = 500;
    in.temp_dc[4] = 500;
    Prot_Update(&in, 2000U);
    TEST_ASSERT_EQUAL_INT32(50000, Prot_GetOutput().ccl_ma);

    /* 3600 mV reduces the charge voltage limit to 50 A as well. */
    Prot_Init();
    in = MakeInput();
    in.cell_mv[0] = 3600U;
    Settle(&in, 2000U);
    TEST_ASSERT_EQUAL_INT32(50000, Prot_GetOutput().ccl_ma);

    /* 2750 mV minimum cell -> discharge derating to 100 A. */
    Prot_Init();
    in = MakeInput();
    in.cell_mv[0] = 2750U;
    Settle(&in, 3000U);
    TEST_ASSERT_EQUAL_INT32(100000, Prot_GetOutput().dcl_ma);

    /* A charge-limiting fault ramps CCL down to zero. */
    Prot_Init();
    in = MakeInput();
    Settle(&in, 2000U);
    TEST_ASSERT_EQUAL_INT32(100000, Prot_GetOutput().ccl_ma);
    in.temp_dc[0] = 560; /* 56 C while charging */
    in.charging = true;
    Prot_Update(&in, 2000U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_CHARGE_OT));
    TEST_ASSERT_EQUAL_INT32(0, Prot_GetOutput().ccl_ma);
    TEST_ASSERT_FALSE(Prot_GetOutput().charge_enable);
}

static void test_dtc_indexing(void)
{
    Prot_Init();
    TEST_ASSERT_EQUAL_UINT8(0U, Prot_GetDtcCount());

    uint32_t dtc = 0U;
    uint8_t status = 0U;
    TEST_ASSERT_FALSE(Prot_GetDtcByIndex(0U, &dtc, &status));

    ProtInput_t in = MakeInput();
    in.cell_mv[0] = 3700U;
    Prot_Update(&in, 1000U);

    TEST_ASSERT_EQUAL_UINT8(1U, Prot_GetDtcCount());
    TEST_ASSERT_TRUE(Prot_GetDtcByIndex(0U, &dtc, &status));
    TEST_ASSERT_EQUAL_HEX32(0x0A01UL, dtc);
    TEST_ASSERT_TRUE((status & DTC_STATUS_CONFIRMED) != 0U);
    TEST_ASSERT_FALSE(Prot_GetDtcByIndex(1U, &dtc, &status));

    in.cell_mv[0] = 3300U;
    in.current_ma = -210000;
    Prot_Update(&in, 500U);
    TEST_ASSERT_TRUE(Prot_IsFaultActive(FAULT_DISCHARGE_OC));
    TEST_ASSERT_EQUAL_UINT8(2U, Prot_GetDtcCount());
    TEST_ASSERT_TRUE(Prot_GetDtcByIndex(1U, &dtc, &status));
    TEST_ASSERT_EQUAL_HEX32(0x0A06UL, dtc);

    /* Unknown DTC group is ignored. */
    Prot_ClearDtc(0x000001UL);
    TEST_ASSERT_EQUAL_UINT8(2U, Prot_GetDtcCount());

    /* Invalid index and NULL pointers are handled. */
    TEST_ASSERT_FALSE(Prot_GetDtcByIndex(99U, &dtc, &status));
    TEST_ASSERT_FALSE(Prot_GetDtcByIndex(0U, NULL, &status));
    TEST_ASSERT_FALSE(Prot_GetDtcByIndex(0U, &dtc, NULL));
    TEST_ASSERT_EQUAL_UINT8(0U, Prot_GetDtcStatus((FaultId)FAULT_COUNT));
    TEST_ASSERT_FALSE(Prot_IsFaultActive((FaultId)FAULT_COUNT));
}

static void test_output_defaults(void)
{
    Prot_Init();

    /* NULL input is ignored and leaves the previous output untouched. */
    Prot_Update(NULL, 10U);
    TEST_ASSERT_EQUAL_INT32(0, Prot_GetOutput().ccl_ma);
    TEST_ASSERT_FALSE(Prot_GetOutput().charge_enable);
    TEST_ASSERT_EQUAL_HEX32(0U, Prot_GetOutput().fault_bits);
}

void run_protection_tests(void)
{
    RUN_TEST(test_ov_debounce_and_latch);
    RUN_TEST(test_latched_fault_needs_release_and_clear);
    RUN_TEST(test_hard_threshold_opens_immediately);
    RUN_TEST(test_uv_debounce);
    RUN_TEST(test_temperature_faults);
    RUN_TEST(test_over_current);
    RUN_TEST(test_binary_faults);
    RUN_TEST(test_derating_and_ramp);
    RUN_TEST(test_dtc_indexing);
    RUN_TEST(test_output_defaults);
}
