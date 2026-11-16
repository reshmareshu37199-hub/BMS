#include "unity.h"

#include "test_groups.h"

#include "drivers/afe.h"
#include "hal/afe_sim.h"
#include "hal/hal.h"
#include "hal/hal_sim.h"

#include <string.h>

static void InitAfe(void)
{
    HalSim_Init();
    Afe_Init();

    /* First poll performs the device-ID self test and leaves the driver idle. */
    TEST_ASSERT_FALSE(Afe_Poll(Hal_TimeMs()));
    TEST_ASSERT_FALSE(Afe_IsBusy());
    TEST_ASSERT_FALSE(Afe_HasCommFault());
}

static void SetUniformCells(uint16_t millivolts)
{
    for (uint8_t i = 0U; i < AFE_CELL_COUNT; ++i)
    {
        HalSim_SetCellMv(i, millivolts);
    }
    HalSim_SetPackVoltageMv((uint16_t)(millivolts * AFE_CELL_COUNT));
}

static bool RunUntilSnapshot(uint32_t max_steps)
{
    for (uint32_t i = 0U; i < max_steps; ++i)
    {
        HalSim_AdvanceMs(1U);
        if (Afe_Poll(Hal_TimeMs()))
        {
            return true;
        }
    }
    return false;
}

static void RunUntilCommFault(uint32_t max_steps)
{
    for (uint32_t i = 0U; i < max_steps; ++i)
    {
        if (Afe_HasCommFault())
        {
            return;
        }
        if (!Afe_IsBusy())
        {
            Afe_StartCycle(Hal_TimeMs());
        }
        HalSim_AdvanceMs(1U);
        (void)Afe_Poll(Hal_TimeMs());
    }
}

static void test_device_id_self_test(void)
{
    HalSim_Init();
    Afe_Init();
    TEST_ASSERT_FALSE(Afe_Poll(Hal_TimeMs()));
    TEST_ASSERT_FALSE(Afe_IsBusy());

    AfeSnapshot_t snap;
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&snap));
    TEST_ASSERT_EQUAL_UINT32(0U, snap.sequence);
    TEST_ASSERT_FALSE(snap.valid);
}

static void test_wrong_device_id_is_a_comm_fault(void)
{
    HalSim_Init();
    AfeSim_SetWrongId(true);
    Afe_Init();

    TEST_ASSERT_FALSE(Afe_Poll(Hal_TimeMs()));
    TEST_ASSERT_FALSE(Afe_HasCommFault());
    TEST_ASSERT_FALSE(Afe_Poll(Hal_TimeMs()));
    TEST_ASSERT_FALSE(Afe_HasCommFault());
    TEST_ASSERT_FALSE(Afe_Poll(Hal_TimeMs()));
    TEST_ASSERT_TRUE(Afe_HasCommFault());
    TEST_ASSERT_FALSE(Afe_IsBusy());

    /* No conversion can start once the identity check has failed. */
    Afe_StartCycle(Hal_TimeMs());
    HalSim_AdvanceMs(1U);
    (void)Afe_Poll(Hal_TimeMs());
    TEST_ASSERT_FALSE(Afe_IsBusy());

    /* Clearing the latch re-runs the self test. */
    Afe_ClearLatchedFault();
    TEST_ASSERT_FALSE(Afe_HasCommFault());
    TEST_ASSERT_FALSE(Afe_Poll(Hal_TimeMs()));
}

static void test_successful_acquisition(void)
{
    InitAfe();
    SetUniformCells(3330U);
    HalSim_SetCellTempDeciC(0U, 250);
    HalSim_SetCellTempDeciC(1U, -100);

    Afe_StartCycle(Hal_TimeMs());
    TEST_ASSERT_TRUE(RunUntilSnapshot(200U));

    AfeSnapshot_t snap;
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&snap));
    TEST_ASSERT_TRUE(snap.valid);
    TEST_ASSERT_TRUE(snap.sequence >= 1U);
    TEST_ASSERT_FALSE(snap.internal_fault);
    TEST_ASSERT_FALSE(snap.open_wire);
    TEST_ASSERT_FALSE(snap.sum_mismatch);
    TEST_ASSERT_EQUAL_UINT16(3330U, snap.cell_mv[0]);
    TEST_ASSERT_EQUAL_UINT16(3330U, snap.cell_mv[15]);
    TEST_ASSERT_INT_WITHIN(20, 250, snap.temp_dc[0]);
    TEST_ASSERT_INT_WITHIN(20, -100, snap.temp_dc[1]);
    TEST_ASSERT_FALSE(Afe_IsBusy());

    /* A second cycle publishes a new buffer with a higher sequence number. */
    Afe_StartCycle(Hal_TimeMs());
    TEST_ASSERT_TRUE(RunUntilSnapshot(200U));
    AfeSnapshot_t second;
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&second));
    TEST_ASSERT_TRUE(second.sequence > snap.sequence);
}

static void test_balancing_mask_write(void)
{
    InitAfe();

    TEST_ASSERT_TRUE(Afe_SetBalMask(0x0005U));
    TEST_ASSERT_EQUAL_HEX16(0x0005U, Afe_GetBalMask());
    TEST_ASSERT_EQUAL_HEX16(0x0005U, AfeSim_GetBalMask());

    Afe_StartCycle(Hal_TimeMs());
    (void)Afe_Poll(Hal_TimeMs()); /* IDLE -> START_CONV */
    (void)Afe_Poll(Hal_TimeMs()); /* issues the conversion command */
    TEST_ASSERT_TRUE(Afe_IsBusy());
    TEST_ASSERT_FALSE(Afe_SetBalMask(0x000AU));
    TEST_ASSERT_EQUAL_HEX16(0x0005U, Afe_GetBalMask());

    TEST_ASSERT_TRUE(RunUntilSnapshot(200U));
    TEST_ASSERT_TRUE(Afe_SetBalMask(0x000AU));
    TEST_ASSERT_EQUAL_HEX16(0x000AU, AfeSim_GetBalMask());
}

static void test_sum_mismatch_detection(void)
{
    InitAfe();
    SetUniformCells(3300U);
    HalSim_SetPackVoltageMv(50000U); /* 1600 mV below the cell sum */

    Afe_StartCycle(Hal_TimeMs());
    TEST_ASSERT_TRUE(RunUntilSnapshot(200U));

    AfeSnapshot_t snap;
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&snap));
    TEST_ASSERT_TRUE(snap.valid);
    TEST_ASSERT_TRUE(snap.sum_mismatch);
}

static void test_internal_fault_and_open_wire(void)
{
    InitAfe();
    SetUniformCells(3300U);
    AfeSim_SetInternalFault(true);

    Afe_StartCycle(Hal_TimeMs());
    TEST_ASSERT_TRUE(RunUntilSnapshot(200U));

    AfeSnapshot_t snap;
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&snap));
    TEST_ASSERT_TRUE(snap.internal_fault);
    TEST_ASSERT_FALSE(snap.valid);

    InitAfe();
    SetUniformCells(3300U);
    AfeSim_SetOpenWire(true);

    Afe_StartCycle(Hal_TimeMs());
    TEST_ASSERT_TRUE(RunUntilSnapshot(200U));
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&snap));
    TEST_ASSERT_TRUE(snap.open_wire);
    TEST_ASSERT_TRUE(snap.valid);
}

static void test_crc_errors_retry_then_latch(void)
{
    InitAfe();
    SetUniformCells(3300U);
    AfeSim_SetForceCrcError(true);

    RunUntilCommFault(5000U);

    TEST_ASSERT_TRUE(Afe_HasCommFault());
    TEST_ASSERT_TRUE(Afe_GetFailedCycles() >= AFE_MAX_FAILED_CYCLES);

    AfeSnapshot_t snap;
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&snap));
    TEST_ASSERT_FALSE(snap.valid);

    /* Recovery after the bus is healthy again. */
    AfeSim_SetForceCrcError(false);
    Afe_ClearLatchedFault();
    TEST_ASSERT_FALSE(Afe_HasCommFault());

    Afe_StartCycle(Hal_TimeMs());
    TEST_ASSERT_TRUE(RunUntilSnapshot(200U));
    TEST_ASSERT_TRUE(Afe_GetSnapshot(&snap));
    TEST_ASSERT_TRUE(snap.valid);
}

static void test_spi_timeout_latches(void)
{
    InitAfe();
    SetUniformCells(3300U);
    AfeSim_SetForceTimeout(true);

    RunUntilCommFault(5000U);

    TEST_ASSERT_TRUE(Afe_HasCommFault());
    TEST_ASSERT_FALSE(Afe_IsBusy());
}

static void test_api_guards(void)
{
    TEST_ASSERT_FALSE(Afe_GetSnapshot(NULL));
    TEST_ASSERT_FALSE(Afe_GetSnapshot(NULL));
}

void run_afe_tests(void)
{
    RUN_TEST(test_device_id_self_test);
    RUN_TEST(test_wrong_device_id_is_a_comm_fault);
    RUN_TEST(test_successful_acquisition);
    RUN_TEST(test_balancing_mask_write);
    RUN_TEST(test_sum_mismatch_detection);
    RUN_TEST(test_internal_fault_and_open_wire);
    RUN_TEST(test_crc_errors_retry_then_latch);
    RUN_TEST(test_spi_timeout_latches);
    RUN_TEST(test_api_guards);
}
