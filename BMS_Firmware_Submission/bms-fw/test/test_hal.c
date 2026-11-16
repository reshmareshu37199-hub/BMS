#include "unity.h"

#include "test_groups.h"

#include "common/crc.h"
#include "drivers/ntc_table.h"
#include "hal/hal.h"
#include "hal/hal_sim.h"
#include "services/can_matrix.h"
#include "services/can_signal.h"

static void test_crc_known_vectors(void)
{
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

    TEST_ASSERT_EQUAL_HEX8(0xF4U, Crc8_Poly07(check, sizeof(check)));
    TEST_ASSERT_EQUAL_HEX8(0x4BU, Crc8_SaeJ1850(check, sizeof(check)));
    TEST_ASSERT_EQUAL_HEX16(0x29B1U, Crc16_Ccitt(check, sizeof(check)));
    TEST_ASSERT_EQUAL_HEX8(0x00U, Crc8_Poly07(NULL, 0U));
}

static void test_ntc_round_trip(void)
{
    const int16_t points[] = {-400, -100, 0, 250, 450, 850, 1250};

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(points) / sizeof(points[0])); ++i)
    {
        uint16_t raw = Ntc_DeciCToRaw(points[i]);
        int16_t back = Ntc_RawToDeciC(raw);
        TEST_ASSERT_INT_WITHIN(10, points[i], back);
    }
}

static void test_can_signal_round_trip(void)
{
    CanSignalDef_t signed_sig = {
        .start_bit = 4U,
        .length = 12U,
        .type = CAN_SIG_SIGNED,
        .scale_num = 1,
        .scale_den = 1,
        .offset = 0,
        .phys_min = -2048,
        .phys_max = 2047,
    };

    uint8_t data[8] = {0};
    TEST_ASSERT_TRUE(CanSig_Pack(data, 8U, &signed_sig, -100));

    int32_t value = 0;
    TEST_ASSERT_TRUE(CanSig_Unpack(data, 8U, &signed_sig, &value));
    TEST_ASSERT_EQUAL_INT32(-100, value);

    CanSignalDef_t offset_sig = {
        .start_bit = 8U,
        .length = 8U,
        .type = CAN_SIG_UNSIGNED,
        .scale_num = 1,
        .scale_den = 1,
        .offset = -40,
        .phys_min = -40,
        .phys_max = 215,
    };

    (void)CanSig_Pack(data, 8U, &offset_sig, 25);
    TEST_ASSERT_EQUAL_UINT8(65U, data[1]);
    TEST_ASSERT_TRUE(CanSig_Unpack(data, 8U, &offset_sig, &value));
    TEST_ASSERT_EQUAL_INT32(25, value);
}

static void test_can_signal_saturates(void)
{
    CanSignalDef_t unsigned_sig = {
        .start_bit = 0U,
        .length = 8U,
        .type = CAN_SIG_UNSIGNED,
        .scale_num = 1,
        .scale_den = 1,
        .offset = 0,
        .phys_min = 0,
        .phys_max = 1000,
    };
    CanSignalDef_t signed_sig = {
        .start_bit = 0U,
        .length = 8U,
        .type = CAN_SIG_SIGNED,
        .scale_num = 1,
        .scale_den = 1,
        .offset = 0,
        .phys_min = -1000,
        .phys_max = 1000,
    };

    TEST_ASSERT_EQUAL_HEX8(0xFFU, CanSig_PhysToRaw(&unsigned_sig, 5000));
    TEST_ASSERT_EQUAL_HEX8(0x80U, CanSig_PhysToRaw(&signed_sig, -500));
    TEST_ASSERT_EQUAL_HEX8(0x7FU, CanSig_PhysToRaw(&signed_sig, 500));
}

static void test_can_matrix_pack_status(void)
{
    CanPackStatus_t status = {
        .pack_voltage_mv = 51200U,
        .pack_current_ma = -12300,
        .soc_permille = 500U,
        .bms_state = CAN_STATE_DRIVE,
    };
    uint8_t data[CAN_DLC];

    CanMatrix_PackPackStatus(data, &status, 5U);

    TEST_ASSERT_EQUAL_HEX8(0x00U, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x14U, data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x85U, data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, data[3]);
    TEST_ASSERT_EQUAL_HEX8(0x64U, data[4]);
    TEST_ASSERT_EQUAL_HEX8(0x53U, data[5]);
    TEST_ASSERT_TRUE(CanMatrix_CheckCrc(data));

    CanPackStatus_t back;
    TEST_ASSERT_TRUE(CanMatrix_UnpackPackStatus(data, &back));
    TEST_ASSERT_EQUAL_UINT16(51200U, back.pack_voltage_mv);
    TEST_ASSERT_EQUAL_INT32(-12300, back.pack_current_ma);
    TEST_ASSERT_EQUAL_UINT16(500U, back.soc_permille);
    TEST_ASSERT_EQUAL_UINT8(CAN_STATE_DRIVE, back.bms_state);

    data[7] ^= 0x01U;
    TEST_ASSERT_FALSE(CanMatrix_CheckCrc(data));
}

static void test_can_matrix_pack_limits(void)
{
    CanLimits_t limits = {
        .ccl_ma = 100000U,
        .dcl_ma = 200000U,
        .charge_voltage_mv = 56800U,
        .charge_enable = true,
    };
    uint8_t data[CAN_DLC];

    CanMatrix_PackLimits(data, &limits, 7U);

    TEST_ASSERT_EQUAL_HEX8(0xE8U, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03U, data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xD0U, data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x07U, data[3]);
    TEST_ASSERT_EQUAL_HEX8(0x30U, data[4]);
    TEST_ASSERT_EQUAL_HEX8(0x16U, data[5]);
    TEST_ASSERT_EQUAL_HEX8(0x71U, data[6]);
    TEST_ASSERT_TRUE(CanMatrix_CheckCrc(data));

    CanLimits_t back;
    TEST_ASSERT_TRUE(CanMatrix_UnpackLimits(data, &back));
    TEST_ASSERT_EQUAL_UINT32(100000U, back.ccl_ma);
    TEST_ASSERT_EQUAL_UINT32(200000U, back.dcl_ma);
    TEST_ASSERT_EQUAL_UINT16(56800U, back.charge_voltage_mv);
    TEST_ASSERT_TRUE(back.charge_enable);
}

static void test_can_matrix_cell_frames(void)
{
    CanCellStats_t stats = {
        .vmax_mv = 3480U,
        .vmin_mv = 3450U,
        .vmax_idx = 4U,
        .vmin_idx = 3U,
        .tmax_c = 25,
        .tmin_c = 20,
    };
    uint8_t data[CAN_DLC];

    CanMatrix_PackCellStats(data, &stats);
    TEST_ASSERT_EQUAL_HEX8(0x98U, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0DU, data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x7AU, data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x0DU, data[3]);
    TEST_ASSERT_EQUAL_UINT8(4U, data[4]);
    TEST_ASSERT_EQUAL_UINT8(3U, data[5]);
    TEST_ASSERT_EQUAL_UINT8(65U, data[6]);
    TEST_ASSERT_EQUAL_UINT8(60U, data[7]);

    CanCellVoltages_t cells = {
        .mux = 2U,
        .cell_mv = {3300U, 3301U, 3302U},
        .balance_bits = 0x05U,
    };
    CanMatrix_PackCellVoltages(data, &cells);
    TEST_ASSERT_EQUAL_UINT8(2U, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xE4U, data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x0CU, data[2]);

    CanCellVoltages_t back;
    TEST_ASSERT_TRUE(CanMatrix_UnpackCellVoltages(data, &back));
    TEST_ASSERT_EQUAL_UINT8(2U, back.mux);
    TEST_ASSERT_EQUAL_UINT16(3300U, back.cell_mv[0]);
    TEST_ASSERT_EQUAL_UINT16(3301U, back.cell_mv[1]);
    TEST_ASSERT_EQUAL_UINT16(3302U, back.cell_mv[2]);
    TEST_ASSERT_EQUAL_UINT8(0x05U, back.balance_bits);
}

static void test_can_matrix_faults(void)
{
    CanFaults_t faults = {
        .fault_bits = 0x00000A01UL,
        .warning_bits = 0x0003U,
        .active_dtc_count = 2U,
    };
    uint8_t data[CAN_DLC];

    CanMatrix_PackFaults(data, &faults);
    TEST_ASSERT_EQUAL_HEX8(0x01U, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, data[3]);
    TEST_ASSERT_EQUAL_HEX8(0x03U, data[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, data[5]);
    TEST_ASSERT_EQUAL_HEX8(0x02U, data[6]);
}

static void test_can_matrix_vcu_round_trip(void)
{
    CanVcuCommand_t command = {
        .contactor_req = 1U,
        .balancing_enable = true,
        .sleep_req = false,
        .fault_clear_req = true,
        .alive_counter = 7U,
    };
    uint8_t data[CAN_DLC];

    CanMatrix_PackVcuCommand(data, &command);
    TEST_ASSERT_TRUE(CanMatrix_CheckCrc(data));

    CanVcuCommand_t back;
    TEST_ASSERT_EQUAL(VCU_FRAME_OK, CanMatrix_UnpackVcuCommand(data, CAN_DLC, &back));
    TEST_ASSERT_EQUAL_UINT8(1U, back.contactor_req);
    TEST_ASSERT_TRUE(back.balancing_enable);
    TEST_ASSERT_FALSE(back.sleep_req);
    TEST_ASSERT_TRUE(back.fault_clear_req);
    TEST_ASSERT_EQUAL_UINT8(7U, back.alive_counter);

    TEST_ASSERT_EQUAL(VCU_FRAME_BAD_DLC, CanMatrix_UnpackVcuCommand(data, 7U, &back));
    data[7] ^= 0x01U;
    TEST_ASSERT_EQUAL(VCU_FRAME_BAD_CRC, CanMatrix_UnpackVcuCommand(data, CAN_DLC, &back));
}

static void test_hal_null_ops_are_safe(void)
{
    Hal_Init(NULL);

    TEST_ASSERT_EQUAL_UINT32(0U, Hal_TimeMs());
    TEST_ASSERT_FALSE(Hal_SpiTransfer(NULL, 0U, NULL, 0U));
    TEST_ASSERT_FALSE(Hal_ContactorGet(HAL_CONTACTOR_NEG));
    TEST_ASSERT_EQUAL_UINT16(0U, Hal_PackVoltageMv());
    TEST_ASSERT_EQUAL_INT32(0, Hal_PackCurrentMa());
    TEST_ASSERT_FALSE(Hal_CanSend(0x123U, NULL, 0U));
    TEST_ASSERT_FALSE(Hal_NvmRead(0U, NULL, 0U));

    HalSim_Init();
    TEST_ASSERT_EQUAL_UINT32(0U, Hal_TimeMs());
}

void run_hal_tests(void)
{
    RUN_TEST(test_crc_known_vectors);
    RUN_TEST(test_ntc_round_trip);
    RUN_TEST(test_can_signal_round_trip);
    RUN_TEST(test_can_signal_saturates);
    RUN_TEST(test_can_matrix_pack_status);
    RUN_TEST(test_can_matrix_pack_limits);
    RUN_TEST(test_can_matrix_cell_frames);
    RUN_TEST(test_can_matrix_faults);
    RUN_TEST(test_can_matrix_vcu_round_trip);
    RUN_TEST(test_hal_null_ops_are_safe);
}
