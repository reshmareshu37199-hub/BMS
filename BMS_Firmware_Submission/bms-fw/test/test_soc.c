#include "unity.h"

#include "test_groups.h"

#include "services/soc.h"

static SocConfig_t MakeConfig(int32_t capacity_mah, int32_t efficiency_permille)
{
    SocConfig_t config = {
        .capacity_mah = capacity_mah,
        .efficiency_permille = efficiency_permille,
    };
    return config;
}

static void test_init_and_ocv_table(void)
{
    SocState_t state;
    SocConfig_t config = MakeConfig(100000, 1000);

    Soc_Init(&state, &config, 500);
    TEST_ASSERT_EQUAL_INT32(500, Soc_GetPermille(&state));
    TEST_ASSERT_EQUAL_INT32(50, Soc_GetPercent(&state));

    Soc_Init(&state, &config, 1500);
    TEST_ASSERT_EQUAL_INT32(1000, Soc_GetPermille(&state));
    Soc_Init(&state, &config, -5);
    TEST_ASSERT_EQUAL_INT32(0, Soc_GetPermille(&state));

    /* NULL config falls back to the documented defaults. */
    Soc_Init(&state, NULL, 300);
    TEST_ASSERT_EQUAL_INT32(SOC_DEFAULT_CAPACITY_MAH, state.config.capacity_mah);
    TEST_ASSERT_EQUAL_INT32(SOC_DEFAULT_EFFICIENCY_PERMILLE, state.config.efficiency_permille);

    TEST_ASSERT_EQUAL_INT32(0, Soc_OcvToPermille(2700U));
    TEST_ASSERT_EQUAL_INT32(0, Soc_OcvToPermille(2800U));
    TEST_ASSERT_EQUAL_INT32(500, Soc_OcvToPermille(3300U));
    TEST_ASSERT_EQUAL_INT32(700, Soc_OcvToPermille(3320U));
    TEST_ASSERT_EQUAL_INT32(900, Soc_OcvToPermille(3340U));
    TEST_ASSERT_EQUAL_INT32(1000, Soc_OcvToPermille(3450U));
    TEST_ASSERT_EQUAL_INT32(1000, Soc_OcvToPermille(3600U));
}

static void test_coulomb_counting_discharge(void)
{
    SocState_t state;
    SocConfig_t config = MakeConfig(100000, 1000);
    Soc_Init(&state, &config, 500);

    /* 10 A for one hour on a 100 Ah pack = 100 permille. */
    for (uint32_t i = 0U; i < 3600U; ++i)
    {
        Soc_Update(&state, -10000, 0U, 1000U);
    }

    TEST_ASSERT_EQUAL_INT32(400, Soc_GetPermille(&state));

    /* The estimate saturates at zero instead of wrapping. */
    Soc_SetPermille(&state, 10);
    for (uint32_t i = 0U; i < 3600U; ++i)
    {
        Soc_Update(&state, -10000, 0U, 1000U);
    }
    TEST_ASSERT_EQUAL_INT32(0, Soc_GetPermille(&state));
}

static void test_coulomb_counting_charge_efficiency(void)
{
    SocState_t state;
    SocConfig_t config = MakeConfig(100000, 1000);
    Soc_Init(&state, &config, 500);

    for (uint32_t i = 0U; i < 3600U; ++i)
    {
        Soc_Update(&state, 10000, 0U, 1000U);
    }
    TEST_ASSERT_EQUAL_INT32(600, Soc_GetPermille(&state));

    /* 99 % coulombic efficiency shaves one permille off the same charge. */
    SocConfig_t low_eff = MakeConfig(100000, 990);
    Soc_Init(&state, &low_eff, 500);
    for (uint32_t i = 0U; i < 3600U; ++i)
    {
        Soc_Update(&state, 10000, 0U, 1000U);
    }
    TEST_ASSERT_EQUAL_INT32(599, Soc_GetPermille(&state));

    /* Saturation at 100 %. */
    Soc_SetPermille(&state, 990);
    for (uint32_t i = 0U; i < 3600U; ++i)
    {
        Soc_Update(&state, 10000, 0U, 1000U);
    }
    TEST_ASSERT_EQUAL_INT32(1000, Soc_GetPermille(&state));
}

static void test_current_offset_learning(void)
{
    SocState_t state;
    SocConfig_t config = MakeConfig(100000, 1000);
    Soc_Init(&state, &config, 500);

    /* 100 mA of sensor bias for 30 s is learned as an offset. */
    for (uint32_t i = 0U; i < 30U; ++i)
    {
        Soc_Update(&state, 100, 0U, 1000U);
    }
    TEST_ASSERT_EQUAL_INT32(100, Soc_GetOffsetMa(&state));
    TEST_ASSERT_EQUAL_UINT32(0U, state.offset_ms);

    /* A real current resets the rest window. */
    Soc_Update(&state, 600, 0U, 1000U);
    TEST_ASSERT_EQUAL_UINT32(0U, Soc_GetRestMs(&state));

    /* Negative bias is learned too. */
    Soc_Init(&state, &config, 500);
    for (uint32_t i = 0U; i < 30U; ++i)
    {
        Soc_Update(&state, -200, 0U, 1000U);
    }
    TEST_ASSERT_EQUAL_INT32(-200, Soc_GetOffsetMa(&state));
}

static void test_ocv_correction(void)
{
    SocState_t state;
    SocConfig_t config = MakeConfig(100000, 1000);
    Soc_Init(&state, &config, 100);

    /* After one hour at rest the OCV pulls a 10 % estimate towards 90 %.
     * 3340 mV is inside the flat LFP region, so only 5 % of the error is
     * applied per correction. */
    for (uint32_t i = 0U; i < 3600U; ++i)
    {
        Soc_Update(&state, 0, 3340U, 1000U);
    }
    TEST_ASSERT_EQUAL_UINT32(1U, Soc_GetOcvCorrections(&state));
    TEST_ASSERT_EQUAL_INT32(140, Soc_GetPermille(&state));

    /* The correction happens only once per rest period. */
    for (uint32_t i = 0U; i < 3600U; ++i)
    {
        Soc_Update(&state, 0, 3340U, 1000U);
    }
    TEST_ASSERT_EQUAL_UINT32(1U, Soc_GetOcvCorrections(&state));
    TEST_ASSERT_EQUAL_INT32(140, Soc_GetPermille(&state));

    /* A charge/discharge cycle re-arms the correction. */
    Soc_Update(&state, 5000, 0U, 1000U);
    TEST_ASSERT_EQUAL_INT32(0, (int32_t)state.ocv_done);
    for (uint32_t i = 0U; i < 3601U; ++i)
    {
        Soc_Update(&state, 0, 3340U, 1000U);
    }
    TEST_ASSERT_EQUAL_UINT32(2U, Soc_GetOcvCorrections(&state));
}

static void test_helpers_and_guards(void)
{
    SocState_t state;
    Soc_Init(&state, NULL, 500);

    Soc_SetPermille(&state, 1200);
    TEST_ASSERT_EQUAL_INT32(1000, Soc_GetPermille(&state));
    Soc_SetPermille(&state, -100);
    TEST_ASSERT_EQUAL_INT32(0, Soc_GetPermille(&state));

    TEST_ASSERT_EQUAL_INT32(0, Soc_GetPermille(NULL));
    TEST_ASSERT_EQUAL_INT32(0, Soc_GetPercent(NULL));
    TEST_ASSERT_EQUAL_INT32(0, Soc_GetOffsetMa(NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, Soc_GetRestMs(NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, Soc_GetOcvCorrections(NULL));

    /* NULL state and zero dt are no-ops. */
    Soc_Update(NULL, 1000, 3300U, 1000U);
    int32_t before = Soc_GetPermille(&state);
    Soc_Update(&state, 1000, 3300U, 0U);
    TEST_ASSERT_EQUAL_INT32(before, Soc_GetPermille(&state));
    Soc_Init(NULL, NULL, 500);
    Soc_SetPermille(NULL, 500);
}

void run_soc_tests(void)
{
    RUN_TEST(test_init_and_ocv_table);
    RUN_TEST(test_coulomb_counting_discharge);
    RUN_TEST(test_coulomb_counting_charge_efficiency);
    RUN_TEST(test_current_offset_learning);
    RUN_TEST(test_ocv_correction);
    RUN_TEST(test_helpers_and_guards);
}
