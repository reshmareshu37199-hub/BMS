#include "unity.h"

#include "test_groups.h"

#include "services/balancing.h"

#include <string.h>

static BalInput_t MakeInput(uint16_t base_mv)
{
    BalInput_t in;
    (void)memset(&in, 0, sizeof(in));
    for (uint8_t i = 0U; i < BAL_CELL_COUNT; ++i)
    {
        in.cell_mv[i] = base_mv;
    }
    in.meas_valid = true;
    in.max_cell_temp_dc = 250;
    in.board_temp_dc = 300;
    in.pack_current_mA = 0;
    in.vcu_balancing_enable = true;
    in.any_fault_active = false;
    return in;
}

static uint8_t PopCount(uint16_t value)
{
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < 16U; ++i)
    {
        if ((value & (uint16_t)(1U << i)) != 0U)
        {
            count++;
        }
    }
    return count;
}

static bool HasAdjacent(uint16_t mask)
{
    return (mask & (uint16_t)(mask << 1U)) != 0U;
}

static void test_s1_below_vmin(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3300U);
    for (uint8_t i = 0U; i < BAL_CELL_COUNT; ++i)
    {
        in.cell_mv[i] = (uint16_t)(3300 + (int16_t)(i % 7U) - 3);
    }

    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_NONE, Bal_GetStatus().inhibit);
    TEST_ASSERT_EQUAL_UINT8(0U, Bal_GetStatus().active_count);
}

static void test_s2_highest_cell_only(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;

    uint16_t mask = Bal_Run(&in);
    TEST_ASSERT_EQUAL_HEX16(0x0010U, mask);
    TEST_ASSERT_EQUAL_UINT8(1U, Bal_GetStatus().active_count);
}

static void test_s3_adjacent_cells_alternate(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[2] = 3480U;
    in.cell_mv[3] = 3480U;
    in.cell_mv[4] = 3480U;

    uint16_t first = Bal_Run(&in);
    TEST_ASSERT_FALSE(HasAdjacent(first));
    TEST_ASSERT_TRUE((first & 0x0014U) == 0x0014U); /* cells 3 and 5 */
    TEST_ASSERT_EQUAL_HEX16(0U, (uint16_t)(first & 0x0008U));

    uint16_t second = Bal_Run(&in);
    TEST_ASSERT_FALSE(HasAdjacent(second));
    TEST_ASSERT_NOT_EQUAL(0U, (second & 0x0008U)); /* cell 4 gets its turn */
}

static void test_s4_max_cells_and_adjacency(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3480U);
    in.cell_mv[15] = 3400U; /* lowest cell, does not qualify */

    uint16_t mask = Bal_Run(&in);
    TEST_ASSERT_EQUAL_UINT8(8U, PopCount(mask));
    TEST_ASSERT_FALSE(HasAdjacent(mask));
}

static void test_s5_s6_hysteresis(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;
    TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));

    in.cell_mv[4] = 3460U; /* delta 10 mV, inside the hysteresis band */
    TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));

    in.cell_mv[4] = 3454U; /* delta 4 mV, below BAL_STOP_MV */
    TEST_ASSERT_EQUAL_HEX16(0x0000U, Bal_Run(&in));
}

static void test_s7_over_temperature_latch(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;
    in.max_cell_temp_dc = 505;

    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_OVER_TEMP, Bal_GetStatus().inhibit);

    in.max_cell_temp_dc = 470; /* above the 45.0 C resume point */
    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_OVER_TEMP, Bal_GetStatus().inhibit);

    in.max_cell_temp_dc = 450;
    TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_NONE, Bal_GetStatus().inhibit);
}

static void test_s8_discharge_inhibit(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;
    in.pack_current_mA = -10000;

    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_DISCHARGE, Bal_GetStatus().inhibit);
}

static void test_s9_invalid_and_fault_inhibit_same_cycle(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;
    TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));

    in.meas_valid = false;
    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_MEAS_INVALID, Bal_GetStatus().inhibit);

    in.meas_valid = true;
    in.any_fault_active = true;
    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_FAULT, Bal_GetStatus().inhibit);
}

static void test_s10_timeout_and_charge_reset(void)
{
    BalConfig_t config = {
        .vmin_mv = 3400U,
        .start_mv = 15U,
        .stop_mv = 5U,
        .max_cells = 8U,
        .cell_temp_max_dc = 500,
        .board_temp_max_dc = 700,
        .temp_hyst_dc = 50,
        .discharge_inhibit_ma = -2000,
        .timeout_ms = 8UL * 60UL * 60UL * 1000UL,
        .cycle_ms = 60UL * 60UL * 1000UL,
        .fairness_band_mv = 20U,
    };
    Bal_Init(&config);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;

    for (uint8_t hour = 0U; hour < 7U; ++hour)
    {
        TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));
    }

    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_TIMEOUT, Bal_GetStatus().inhibit);
    TEST_ASSERT_TRUE(Bal_GetStatus().timeout_warning);

    /* A subsequent cycle stays inhibited until a charge cycle is seen. */
    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_TIMEOUT, Bal_GetStatus().inhibit);

    in.pack_current_mA = 1000; /* new charge cycle clears the warning */
    TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_NONE, Bal_GetStatus().inhibit);
}

static BalConfig_t MakeConfig(uint8_t max_cells, uint32_t timeout_ms, uint32_t cycle_ms)
{
    BalConfig_t cfg = {
        .vmin_mv = 3400U,
        .start_mv = 15U,
        .stop_mv = 5U,
        .max_cells = max_cells,
        .cell_temp_max_dc = 500,
        .board_temp_max_dc = 700,
        .temp_hyst_dc = 50,
        .discharge_inhibit_ma = -2000,
        .timeout_ms = timeout_ms,
        .cycle_ms = cycle_ms,
        .fairness_band_mv = 20U,
    };
    return cfg;
}

static void test_null_input_clears_status(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;
    TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));

    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(NULL));
    TEST_ASSERT_EQUAL_UINT16(0U, Bal_GetStatus().mask);
    TEST_ASSERT_FALSE(Bal_GetStatus().active);
    TEST_ASSERT_EQUAL_UINT8(0U, Bal_GetStatus().active_count);
    TEST_ASSERT_EQUAL(BAL_INHIBIT_MEAS_INVALID, Bal_GetStatus().inhibit);
}

static void test_measurement_window_inhibit(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;

    Bal_SetMeasurementWindow(true);
    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_MEAS_PAUSE, Bal_GetStatus().inhibit);

    Bal_SetMeasurementWindow(false);
    TEST_ASSERT_EQUAL_HEX16(0x0010U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_NONE, Bal_GetStatus().inhibit);
}

static void test_vcu_disable_inhibit(void)
{
    Bal_Init(NULL);

    BalInput_t in = MakeInput(3450U);
    in.cell_mv[4] = 3480U;
    in.vcu_balancing_enable = false;

    TEST_ASSERT_EQUAL_UINT16(0U, Bal_Run(&in));
    TEST_ASSERT_EQUAL(BAL_INHIBIT_VCU_DISABLED, Bal_GetStatus().inhibit);
}

static void test_max_cells_exhausted_marks_priority(void)
{
    BalConfig_t config = MakeConfig(3U, BAL_DEFAULT_TIMEOUT_MS, BAL_DEFAULT_CYCLE_MS);
    Bal_Init(&config);

    BalInput_t in = MakeInput(3400U);
    for (uint8_t cell = 0U; cell < BAL_CELL_COUNT; cell = (uint8_t)(cell + 2U))
    {
        in.cell_mv[cell] = 3480U;
    }

    /* Eight non-adjacent candidates, only three may be selected. */
    uint16_t mask = Bal_Run(&in);
    TEST_ASSERT_EQUAL_UINT8(3U, PopCount(mask));
    TEST_ASSERT_EQUAL_HEX16(0x0015U, mask); /* cells 1, 3 and 5 */
}

static void test_adjacency_skip_after_limit_sets_priority(void)
{
    BalConfig_t config = MakeConfig(3U, BAL_DEFAULT_TIMEOUT_MS, BAL_DEFAULT_CYCLE_MS);
    Bal_Init(&config);

    BalInput_t in = MakeInput(3400U);
    for (uint8_t cell = 0U; cell < 6U; ++cell)
    {
        in.cell_mv[cell] = 3480U;
    }

    uint16_t mask = Bal_Run(&in);
    TEST_ASSERT_EQUAL_UINT8(3U, PopCount(mask));
    TEST_ASSERT_EQUAL_HEX16(0x0015U, mask); /* cells 1, 3 and 5 */
}

static void test_priority_cell_is_selected_first(void)
{
    BalConfig_t config = MakeConfig(1U, BAL_DEFAULT_TIMEOUT_MS, BAL_DEFAULT_CYCLE_MS);
    Bal_Init(&config);

    BalInput_t in = MakeInput(3400U);
    in.cell_mv[0] = 3480U;
    in.cell_mv[2] = 3480U;

    /* Cell 3 (index 2) is dropped by the one-cell limit and becomes priority. */
    TEST_ASSERT_EQUAL_HEX16(0x0001U, Bal_Run(&in));

    in.cell_mv[0] = 3400U;
    in.cell_mv[2] = 3470U; /* within the fairness band of the maximum */
    in.cell_mv[5] = 3480U;

    /* The starved cell wins even though cell 6 (index 5) is higher. */
    TEST_ASSERT_EQUAL_HEX16(0x0004U, Bal_Run(&in));
}

static void test_sort_compares_voltages_and_bands(void)
{
    BalConfig_t config = MakeConfig(1U, BAL_DEFAULT_TIMEOUT_MS, BAL_DEFAULT_CYCLE_MS);
    Bal_Init(&config);

    BalInput_t in = MakeInput(3400U);
    in.cell_mv[0] = 3480U;
    in.cell_mv[2] = 3480U;
    (void)Bal_Run(&in); /* priority cell becomes index 2 */

    /* Priority cell is now far below the maximum, so voltage decides. */
    in.cell_mv[0] = 3400U;
    in.cell_mv[2] = 3450U;
    in.cell_mv[5] = 3480U;
    TEST_ASSERT_EQUAL_HEX16(0x0020U, Bal_Run(&in));

    /* Neither candidate is priority and the voltages differ: highest wins. */
    in.cell_mv[2] = 3400U;
    in.cell_mv[4] = 3400U;
    in.cell_mv[5] = 3400U;
    in.cell_mv[0] = 3480U;
    in.cell_mv[1] = 3470U;
    TEST_ASSERT_EQUAL_HEX16(0x0001U, Bal_Run(&in));
}

static void test_configuration_validation(void)
{
    Bal_Init(NULL);

    TEST_ASSERT_TRUE(Bal_SetStartStop(20U, 10U));
    TEST_ASSERT_EQUAL_UINT16(20U, Bal_GetConfig()->start_mv);
    TEST_ASSERT_EQUAL_UINT16(10U, Bal_GetConfig()->stop_mv);

    TEST_ASSERT_FALSE(Bal_SetStartStop(10U, 10U));
    TEST_ASSERT_FALSE(Bal_SetStartStop(4U, 2U));
    TEST_ASSERT_FALSE(Bal_SetStartStop(20U, 0U));

    BalConfig_t bad = {
        .vmin_mv = 3400U,
        .start_mv = 10U,
        .stop_mv = 12U,
        .max_cells = 0U,
        .cell_temp_max_dc = 500,
        .board_temp_max_dc = 700,
        .temp_hyst_dc = 50,
        .discharge_inhibit_ma = -2000,
        .timeout_ms = 1000U,
        .cycle_ms = 0U,
        .fairness_band_mv = 20U,
    };
    Bal_Init(&bad);
    TEST_ASSERT_EQUAL_UINT16(9U, Bal_GetConfig()->stop_mv);
    TEST_ASSERT_EQUAL_UINT8(1U, Bal_GetConfig()->max_cells);
    TEST_ASSERT_EQUAL_UINT32(BAL_DEFAULT_CYCLE_MS, Bal_GetConfig()->cycle_ms);

    /* Oversized cell count and zero timeout are clamped to their defaults. */
    BalConfig_t oversized = bad;
    oversized.max_cells = 20U;
    oversized.timeout_ms = 0U;
    oversized.cycle_ms = BAL_DEFAULT_CYCLE_MS;
    Bal_Init(&oversized);
    TEST_ASSERT_EQUAL_UINT8(BAL_CELL_COUNT, Bal_GetConfig()->max_cells);
    TEST_ASSERT_EQUAL_UINT32(BAL_DEFAULT_TIMEOUT_MS, Bal_GetConfig()->timeout_ms);
}

void run_balancing_tests(void)
{
    RUN_TEST(test_s1_below_vmin);
    RUN_TEST(test_s2_highest_cell_only);
    RUN_TEST(test_s3_adjacent_cells_alternate);
    RUN_TEST(test_s4_max_cells_and_adjacency);
    RUN_TEST(test_s5_s6_hysteresis);
    RUN_TEST(test_s7_over_temperature_latch);
    RUN_TEST(test_s8_discharge_inhibit);
    RUN_TEST(test_s9_invalid_and_fault_inhibit_same_cycle);
    RUN_TEST(test_s10_timeout_and_charge_reset);
    RUN_TEST(test_null_input_clears_status);
    RUN_TEST(test_measurement_window_inhibit);
    RUN_TEST(test_vcu_disable_inhibit);
    RUN_TEST(test_max_cells_exhausted_marks_priority);
    RUN_TEST(test_adjacency_skip_after_limit_sets_priority);
    RUN_TEST(test_priority_cell_is_selected_first);
    RUN_TEST(test_sort_compares_voltages_and_bands);
    RUN_TEST(test_configuration_validation);
}
