#include "unity.h"

#include "test_groups.h"

#include "drivers/can_driver.h"
#include "hal/hal_sim.h"
#include "services/can_matrix.h"

#include <string.h>

static uint32_t s_rx_count;
static uint32_t s_rx_id;
static uint8_t s_rx_dlc;
static uint8_t s_rx_data[CAN_DLC];

static void TestHandler(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    s_rx_count++;
    s_rx_id = id;
    s_rx_dlc = dlc;
    (void)memset(s_rx_data, 0, sizeof(s_rx_data));
    (void)memcpy(s_rx_data, data, dlc);
}

static void ResetHandlerCounters(void)
{
    s_rx_count = 0U;
    s_rx_id = 0U;
    s_rx_dlc = 0U;
    (void)memset(s_rx_data, 0, sizeof(s_rx_data));
}

static void MakeVcuFrame(uint8_t data[CAN_DLC], uint8_t alive)
{
    CanVcuCommand_t command = {
        .contactor_req = 1U,
        .balancing_enable = true,
        .sleep_req = false,
        .fault_clear_req = false,
        .alive_counter = alive,
    };
    CanMatrix_PackVcuCommand(data, &command);
}

static void test_queue_and_drain(void)
{
    HalSim_Init();
    CanDrv_Init();

    const uint8_t first[CAN_DLC] = {1, 2, 3, 4, 5, 6, 7, 8};
    const uint8_t second[CAN_DLC] = {9, 10, 11, 12, 13, 14, 15, 16};

    TEST_ASSERT_TRUE(CanDrv_Queue(0x350U, first, CAN_DLC));
    TEST_ASSERT_TRUE(CanDrv_Queue(0x351U, second, CAN_DLC));
    TEST_ASSERT_EQUAL_UINT8(2U, CanDrv_TxQueueDepth());

    CanDrvStats_t stats = CanDrv_GetStats();
    TEST_ASSERT_EQUAL_UINT32(2U, stats.tx_queued);

    CanDrv_Poll();

    TEST_ASSERT_EQUAL_UINT8(0U, CanDrv_TxQueueDepth());
    TEST_ASSERT_EQUAL_UINT32(2U, HalSim_CanTxCount());

    uint32_t id = 0U;
    uint8_t data[CAN_DLC] = {0};
    uint8_t dlc = 0U;
    TEST_ASSERT_TRUE(HalSim_CanTxGet(0U, &id, data, &dlc));
    TEST_ASSERT_EQUAL_HEX32(0x350U, id);
    TEST_ASSERT_EQUAL_UINT8(CAN_DLC, dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(first, data, CAN_DLC);

    TEST_ASSERT_TRUE(HalSim_CanTxGet(1U, &id, data, &dlc));
    TEST_ASSERT_EQUAL_HEX32(0x351U, id);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(second, data, CAN_DLC);

    stats = CanDrv_GetStats();
    TEST_ASSERT_EQUAL_UINT32(2U, stats.tx_sent);
}

static void test_queue_full_is_bounded(void)
{
    HalSim_Init();
    CanDrv_Init();

    const uint8_t data[CAN_DLC] = {0};

    for (uint8_t i = 0U; i < CANDRV_TX_QUEUE_DEPTH; ++i)
    {
        TEST_ASSERT_TRUE(CanDrv_Queue(0x350U, data, CAN_DLC));
    }
    TEST_ASSERT_FALSE(CanDrv_Queue(0x350U, data, CAN_DLC));
    TEST_ASSERT_EQUAL_UINT32(1U, CanDrv_GetStats().tx_dropped);

    /* The poll is bounded to CANDRV_MAX_TX_PER_POLL frames. */
    CanDrv_Poll();
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(CANDRV_TX_QUEUE_DEPTH - CANDRV_MAX_TX_PER_POLL),
                            CanDrv_TxQueueDepth());
    CanDrv_Poll();
    TEST_ASSERT_EQUAL_UINT8(0U, CanDrv_TxQueueDepth());

    TEST_ASSERT_FALSE(CanDrv_Queue(0x350U, NULL, CAN_DLC));
    TEST_ASSERT_FALSE(CanDrv_Queue(0x350U, data, CANDRV_MAX_DLC + 1U));
}

static void test_rx_dispatch(void)
{
    HalSim_Init();
    CanDrv_Init();
    ResetHandlerCounters();

    TEST_ASSERT_TRUE(CanDrv_RegisterHandler(CAN_ID_VCU_COMMAND, TestHandler));

    const uint8_t data[CAN_DLC] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x18};
    HalSim_CanInject(CAN_ID_VCU_COMMAND, data, CAN_DLC);

    uint8_t ignored[CAN_DLC] = {0};
    HalSim_CanInject(0x123U, ignored, CAN_DLC);

    CanDrv_Poll();

    TEST_ASSERT_EQUAL_UINT32(1U, s_rx_count);
    TEST_ASSERT_EQUAL_HEX32(CAN_ID_VCU_COMMAND, s_rx_id);
    TEST_ASSERT_EQUAL_UINT8(CAN_DLC, s_rx_dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, s_rx_data, CAN_DLC);

    CanDrvStats_t stats = CanDrv_GetStats();
    TEST_ASSERT_EQUAL_UINT32(2U, stats.rx_frames);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.rx_dispatched);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.rx_ignored);
}

static void test_handler_table(void)
{
    CanDrv_Init();

    for (uint8_t i = 0U; i < CANDRV_RX_HANDLER_COUNT; ++i)
    {
        TEST_ASSERT_TRUE(CanDrv_RegisterHandler(0x100U + i, TestHandler));
    }
    TEST_ASSERT_FALSE(CanDrv_RegisterHandler(0x200U, TestHandler));

    /* Re-registering an existing ID updates the handler. */
    TEST_ASSERT_TRUE(CanDrv_RegisterHandler(0x100U, TestHandler));
    TEST_ASSERT_FALSE(CanDrv_RegisterHandler(0x100U, NULL));
}

static void test_vcu_monitor_accepts_valid_sequence(void)
{
    VcuMonitor_t mon;
    VcuMonitor_Init(&mon, 0U);
    TEST_ASSERT_TRUE(VcuMonitor_IsFault(&mon));

    uint8_t data[CAN_DLC];
    MakeVcuFrame(data, 7U);
    TEST_ASSERT_TRUE(VcuMonitor_Update(&mon, data, CAN_DLC, 10U));
    TEST_ASSERT_FALSE(VcuMonitor_IsFault(&mon));
    TEST_ASSERT_EQUAL_UINT8(7U, mon.last_command.alive_counter);
    TEST_ASSERT_TRUE(mon.last_command.balancing_enable);
    TEST_ASSERT_EQUAL_UINT8(1U, mon.last_command.contactor_req);

    MakeVcuFrame(data, 8U);
    TEST_ASSERT_TRUE(VcuMonitor_Update(&mon, data, CAN_DLC, 20U));
    TEST_ASSERT_EQUAL_UINT32(0U, mon.alive_errors);
}

static void test_vcu_monitor_rejects_bad_frames(void)
{
    VcuMonitor_t mon;
    VcuMonitor_Init(&mon, 0U);

    uint8_t data[CAN_DLC];
    MakeVcuFrame(data, 5U);
    TEST_ASSERT_TRUE(VcuMonitor_Update(&mon, data, CAN_DLC, 10U));

    /* Alive counter did not change. */
    TEST_ASSERT_FALSE(VcuMonitor_Update(&mon, data, CAN_DLC, 20U));
    TEST_ASSERT_EQUAL_UINT32(1U, mon.alive_errors);
    TEST_ASSERT_EQUAL_UINT8(1U, mon.consecutive_invalid);

    /* Alive counter jumped by more than 2 (5 -> 12 = 7). */
    MakeVcuFrame(data, 12U);
    TEST_ASSERT_FALSE(VcuMonitor_Update(&mon, data, CAN_DLC, 30U));
    TEST_ASSERT_EQUAL_UINT32(1U, mon.jump_errors);
    TEST_ASSERT_EQUAL_UINT8(2U, mon.consecutive_invalid);

    /* Bad CRC. */
    MakeVcuFrame(data, 6U);
    data[7] ^= 0xFFU;
    TEST_ASSERT_FALSE(VcuMonitor_Update(&mon, data, CAN_DLC, 40U));
    TEST_ASSERT_EQUAL_UINT32(1U, mon.crc_errors);

    /* Wrong DLC. */
    MakeVcuFrame(data, 6U);
    TEST_ASSERT_FALSE(VcuMonitor_Update(&mon, data, CAN_DLC - 1U, 50U));
    TEST_ASSERT_EQUAL_UINT32(1U, mon.dlc_errors);

    /* Three consecutive invalid frames declare the VCU lost. */
    TEST_ASSERT_TRUE(VcuMonitor_IsFault(&mon));

    /* A good frame restores communication. */
    MakeVcuFrame(data, 6U);
    TEST_ASSERT_TRUE(VcuMonitor_Update(&mon, data, CAN_DLC, 60U));
    TEST_ASSERT_FALSE(VcuMonitor_IsFault(&mon));
    TEST_ASSERT_EQUAL_UINT8(0U, mon.consecutive_invalid);
}

static void test_vcu_timeout_and_wrap(void)
{
    VcuMonitor_t mon;
    VcuMonitor_Init(&mon, 0U);

    uint8_t data[CAN_DLC];
    MakeVcuFrame(data, 1U);
    TEST_ASSERT_TRUE(VcuMonitor_Update(&mon, data, CAN_DLC, 100U));
    TEST_ASSERT_FALSE(VcuMonitor_IsFault(&mon));

    VcuMonitor_Poll(&mon, 400U); /* exactly 300 ms: still tolerated */
    TEST_ASSERT_FALSE(VcuMonitor_IsFault(&mon));

    VcuMonitor_Poll(&mon, 401U);
    TEST_ASSERT_TRUE(VcuMonitor_IsFault(&mon));
    TEST_ASSERT_EQUAL_UINT32(1U, mon.timeout_events);

    VcuMonitor_Poll(&mon, 500U); /* no double count */
    TEST_ASSERT_EQUAL_UINT32(1U, mon.timeout_events);

    MakeVcuFrame(data, 2U);
    TEST_ASSERT_TRUE(VcuMonitor_Update(&mon, data, CAN_DLC, 600U));
    TEST_ASSERT_FALSE(VcuMonitor_IsFault(&mon));

    /* No valid frame ever seen: Poll keeps the fault asserted. */
    VcuMonitor_Init(&mon, 0U);
    VcuMonitor_Poll(&mon, 1000U);
    TEST_ASSERT_TRUE(VcuMonitor_IsFault(&mon));
}

static void test_vcu_monitor_guards(void)
{
    uint8_t data[CAN_DLC] = {0};

    TEST_ASSERT_FALSE(VcuMonitor_Update(NULL, data, CAN_DLC, 0U));
    TEST_ASSERT_TRUE(VcuMonitor_IsFault(NULL));
    VcuMonitor_Poll(NULL, 0U);
    VcuMonitor_Init(NULL, 0U);
}

void run_can_tests(void)
{
    RUN_TEST(test_queue_and_drain);
    RUN_TEST(test_queue_full_is_bounded);
    RUN_TEST(test_rx_dispatch);
    RUN_TEST(test_handler_table);
    RUN_TEST(test_vcu_monitor_accepts_valid_sequence);
    RUN_TEST(test_vcu_monitor_rejects_bad_frames);
    RUN_TEST(test_vcu_timeout_and_wrap);
    RUN_TEST(test_vcu_monitor_guards);
}
