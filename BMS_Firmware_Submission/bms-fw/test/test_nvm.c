#include "unity.h"

#include "test_groups.h"

#include "hal/hal_sim.h"
#include "services/nvm.h"

#include <string.h>

static NvmParams_t MakeParams(uint16_t start, uint16_t stop, int16_t soc)
{
    NvmParams_t params = {
        .bal_start_mv = start,
        .bal_stop_mv = stop,
        .soc_permille = soc,
        .flags = 0U,
    };
    return params;
}

static void CorruptSlot(uint8_t slot)
{
    uint8_t zeros[NVM_PAYLOAD_MAX] = {0};
    TEST_ASSERT_TRUE(HalSim_NvmWriteRaw(slot, zeros, (uint16_t)sizeof(zeros)));
}

static void test_defaults_and_blank_store(void)
{
    HalSim_Init();
    HalSim_NvmClear();
    Nvm_Init();

    NvmParams_t params;
    Nvm_GetDefaults(&params);
    TEST_ASSERT_EQUAL_UINT16(NVM_DEFAULT_BAL_START_MV, params.bal_start_mv);
    TEST_ASSERT_EQUAL_UINT16(NVM_DEFAULT_BAL_STOP_MV, params.bal_stop_mv);
    TEST_ASSERT_EQUAL_INT16(NVM_DEFAULT_SOC_PERMILLE, params.soc_permille);

    (void)memset(&params, 0, sizeof(params));
    TEST_ASSERT_FALSE(Nvm_Load(&params));
    TEST_ASSERT_EQUAL_UINT16(NVM_DEFAULT_BAL_START_MV, params.bal_start_mv);
    TEST_ASSERT_EQUAL_UINT16(NVM_DEFAULT_BAL_STOP_MV, params.bal_stop_mv);
    TEST_ASSERT_EQUAL_INT16(NVM_DEFAULT_SOC_PERMILLE, params.soc_permille);
    TEST_ASSERT_EQUAL_UINT8(0xFFU, Nvm_GetValidSlot());
    TEST_ASSERT_EQUAL_UINT32(0U, Nvm_GetSequence());
}

static void test_save_and_load_round_trip(void)
{
    HalSim_Init();
    HalSim_NvmClear();
    Nvm_Init();

    NvmParams_t saved = MakeParams(20U, 10U, 640);
    TEST_ASSERT_TRUE(Nvm_Save(&saved));
    TEST_ASSERT_EQUAL_UINT32(1U, Nvm_GetSequence());
    TEST_ASSERT_EQUAL_UINT8(0U, Nvm_GetValidSlot());
    TEST_ASSERT_EQUAL_UINT32(1U, Nvm_GetWriteCount());

    NvmParams_t loaded;
    TEST_ASSERT_TRUE(Nvm_Load(&loaded));
    TEST_ASSERT_EQUAL_UINT16(20U, loaded.bal_start_mv);
    TEST_ASSERT_EQUAL_UINT16(10U, loaded.bal_stop_mv);
    TEST_ASSERT_EQUAL_INT16(640, loaded.soc_permille);

    /* The next write alternates to slot B with sequence + 1. */
    NvmParams_t second = MakeParams(25U, 8U, 500);
    TEST_ASSERT_TRUE(Nvm_Save(&second));
    TEST_ASSERT_EQUAL_UINT32(2U, Nvm_GetSequence());
    TEST_ASSERT_EQUAL_UINT8(1U, Nvm_GetValidSlot());

    TEST_ASSERT_TRUE(Nvm_Load(&loaded));
    TEST_ASSERT_EQUAL_UINT16(25U, loaded.bal_start_mv);
    TEST_ASSERT_EQUAL_UINT16(8U, loaded.bal_stop_mv);
}

static void test_corrupt_slot_falls_back_to_previous(void)
{
    HalSim_Init();
    HalSim_NvmClear();
    Nvm_Init();

    NvmParams_t first = MakeParams(18U, 6U, 700);
    NvmParams_t second = MakeParams(30U, 12U, 800);
    TEST_ASSERT_TRUE(Nvm_Save(&first));
    TEST_ASSERT_TRUE(Nvm_Save(&second));

    /* Slot B holds the newest record; destroy it. */
    CorruptSlot(1U);

    NvmParams_t loaded;
    TEST_ASSERT_TRUE(Nvm_Load(&loaded));
    TEST_ASSERT_EQUAL_UINT16(18U, loaded.bal_start_mv);
    TEST_ASSERT_EQUAL_UINT16(6U, loaded.bal_stop_mv);
    TEST_ASSERT_EQUAL_INT16(700, loaded.soc_permille);
    TEST_ASSERT_EQUAL_UINT8(0U, Nvm_GetValidSlot());

    /* Both slots corrupt -> defaults, and the caller is told so. */
    CorruptSlot(0U);
    (void)memset(&loaded, 0, sizeof(loaded));
    TEST_ASSERT_FALSE(Nvm_Load(&loaded));
    TEST_ASSERT_EQUAL_UINT16(NVM_DEFAULT_BAL_START_MV, loaded.bal_start_mv);
    TEST_ASSERT_EQUAL_UINT8(0xFFU, Nvm_GetValidSlot());
}

static void test_sequence_selects_newest_record(void)
{
    HalSim_Init();
    HalSim_NvmClear();
    Nvm_Init();

    NvmParams_t a = MakeParams(16U, 6U, 100);
    NvmParams_t b = MakeParams(17U, 7U, 200);
    NvmParams_t c = MakeParams(18U, 8U, 300);
    TEST_ASSERT_TRUE(Nvm_Save(&a));
    TEST_ASSERT_TRUE(Nvm_Save(&b));
    TEST_ASSERT_TRUE(Nvm_Save(&c)); /* slot A again, sequence 3 */

    NvmParams_t loaded;
    TEST_ASSERT_TRUE(Nvm_Load(&loaded));
    TEST_ASSERT_EQUAL_UINT16(18U, loaded.bal_start_mv);
    TEST_ASSERT_EQUAL_UINT32(3U, Nvm_GetSequence());
    TEST_ASSERT_EQUAL_UINT8(0U, Nvm_GetValidSlot());
}

static void test_guards_and_error_paths(void)
{
    HalSim_Init();
    HalSim_NvmClear();
    Nvm_Init();

    TEST_ASSERT_FALSE(Nvm_Save(NULL));
    TEST_ASSERT_FALSE(Nvm_Load(NULL));
    Nvm_GetDefaults(NULL);

    /* Reading an out-of-range slot fails cleanly. */
    NvmParams_t params;
    TEST_ASSERT_FALSE(HalSim_NvmReadRaw(9U, &params, (uint16_t)sizeof(params)));
}

void run_nvm_tests(void)
{
    RUN_TEST(test_defaults_and_blank_store);
    RUN_TEST(test_save_and_load_round_trip);
    RUN_TEST(test_corrupt_slot_falls_back_to_previous);
    RUN_TEST(test_sequence_selects_newest_record);
    RUN_TEST(test_guards_and_error_paths);
}
