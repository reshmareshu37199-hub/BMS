#include "unity.h"

#include "test_groups.h"

#include "app/bms_state.h"

#include <string.h>

static BmsStateInput_t s_in;

static BmsStateInput_t MakeInput(void)
{
    BmsStateInput_t in;
    (void)memset(&in, 0, sizeof(in));
    in.afe_ok = true;
    in.afe_fault = false;
    in.meas_valid = true;
    in.fault_active = false;
    in.fault_latched = false;
    in.open_request = false;
    in.open_immediate = false;
    in.vcu_valid = true;
    in.vcu_close_request = false;
    in.charger_present = false;
    in.sleep_request = false;
    in.pack_voltage_mv = 51200U;
    in.bus_voltage_mv = 0U;
    return in;
}

/* Walk INIT -> STANDBY -> PRECHARGE -> DRIVE/CHARGE starting at `start`. */
static uint32_t EnterDrive(uint32_t start, bool charger)
{
    BmsState_Init(start);
    s_in = MakeInput();
    s_in.vcu_close_request = true;
    s_in.charger_present = charger;
    s_in.bus_voltage_mv = 50000U;

    BmsState_Update(&s_in, start + 200U); /* INIT -> STANDBY */
    BmsState_Update(&s_in, start + 210U); /* STANDBY -> PRECHARGE */
    BmsState_Update(&s_in, start + 230U); /* phase 0 -> 1 */
    BmsState_Update(&s_in, start + 240U); /* phase 1 -> 2 */
    BmsState_Update(&s_in, start + 250U); /* bus high -> phase 3 */
    BmsState_Update(&s_in, start + 270U); /* settled -> DRIVE/CHARGE */

    return start + 270U;
}

static void test_init_to_standby(void)
{
    BmsState_Init(0U);
    BmsStateInput_t in = MakeInput();

    BmsState_Update(&in, 0U);
    TEST_ASSERT_EQUAL(BMS_STATE_INIT, BmsState_GetOutput().state);

    BmsState_Update(&in, 199U);
    TEST_ASSERT_EQUAL(BMS_STATE_INIT, BmsState_GetOutput().state);

    BmsState_Update(&in, 200U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_neg);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_pre);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_pos);
    TEST_ASSERT_FALSE(BmsState_ContactorsClosed());

    /* Missing AFE data keeps the state machine in INIT. */
    BmsState_Init(0U);
    in = MakeInput();
    in.afe_ok = false;
    BmsState_Update(&in, 500U);
    TEST_ASSERT_EQUAL(BMS_STATE_INIT, BmsState_GetOutput().state);
}

static void test_afe_fault_during_init_and_recovery(void)
{
    BmsState_Init(0U);
    BmsStateInput_t in = MakeInput();
    in.afe_fault = true;

    BmsState_Update(&in, 0U);
    TEST_ASSERT_EQUAL(BMS_STATE_FAULT, BmsState_GetOutput().state);

    in.afe_fault = false;
    BmsState_Update(&in, 999U);
    TEST_ASSERT_EQUAL(BMS_STATE_FAULT, BmsState_GetOutput().state);
    BmsState_Update(&in, 1000U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);
}

static void test_precharge_sequence(void)
{
    BmsState_Init(0U);
    BmsStateInput_t in = MakeInput();
    in.vcu_close_request = true;
    in.pack_voltage_mv = 51200U;

    BmsState_Update(&in, 200U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);

    BmsState_Update(&in, 210U);
    TEST_ASSERT_EQUAL(BMS_STATE_PRECHARGE, BmsState_GetOutput().state);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_neg);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_pre);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_pos);

    BmsState_Update(&in, 230U);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_pre);

    BmsState_Update(&in, 240U);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_neg);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_pre);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_pos);

    /* Bus must reach 95 % of the pack voltage before the main contactor
     * closes.  50000 mV is above 48640 mV. */
    in.bus_voltage_mv = 50000U;
    BmsState_Update(&in, 250U);
    TEST_ASSERT_EQUAL(BMS_STATE_PRECHARGE, BmsState_GetOutput().state);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_pos);

    BmsState_Update(&in, 270U);
    TEST_ASSERT_EQUAL(BMS_STATE_DRIVE, BmsState_GetOutput().state);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_neg);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_pre);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_pos);
    TEST_ASSERT_TRUE(BmsState_ContactorsClosed());
}

static void test_precharge_abort_by_vcu(void)
{
    BmsState_Init(0U);
    BmsStateInput_t in = MakeInput();
    in.vcu_close_request = true;

    BmsState_Update(&in, 200U);
    BmsState_Update(&in, 210U);
    TEST_ASSERT_EQUAL(BMS_STATE_PRECHARGE, BmsState_GetOutput().state);

    in.vcu_close_request = false;
    BmsState_Update(&in, 220U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);
    TEST_ASSERT_FALSE(BmsState_ContactorsClosed());
}

static void test_precharge_timeout(void)
{
    BmsState_Init(0U);
    BmsStateInput_t in = MakeInput();
    in.vcu_close_request = true;
    in.pack_voltage_mv = 51200U;
    in.bus_voltage_mv = 0U; /* never rises */

    BmsState_Update(&in, 200U);
    BmsState_Update(&in, 210U);
    BmsState_Update(&in, 230U);
    BmsState_Update(&in, 240U); /* phase 2, precharge timer starts */

    BmsState_Update(&in, 240U + BMS_PRECHARGE_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(BMS_STATE_PRECHARGE, BmsState_GetOutput().state);

    BmsState_Update(&in, 240U + BMS_PRECHARGE_TIMEOUT_MS + 1U);
    BmsStateOutput_t out = BmsState_GetOutput();
    TEST_ASSERT_EQUAL(BMS_STATE_FAULT, out.state);
    TEST_ASSERT_TRUE(out.precharge_failed);
    TEST_ASSERT_FALSE(out.close_neg);
    TEST_ASSERT_FALSE(out.close_pre);
    TEST_ASSERT_FALSE(out.close_pos);

    /* The failure pulse lasts exactly one cycle. */
    BmsState_Update(&in, 240U + BMS_PRECHARGE_TIMEOUT_MS + 2U);
    TEST_ASSERT_FALSE(BmsState_GetOutput().precharge_failed);
}

static void test_drive_vcu_loss_and_release(void)
{
    uint32_t now = EnterDrive(0U, false);
    TEST_ASSERT_EQUAL(BMS_STATE_DRIVE, BmsState_GetOutput().state);

    /* A lost VCU opens the contactors only after 1 s. */
    s_in.vcu_valid = false;
    BmsState_Update(&s_in, now + 1U);
    TEST_ASSERT_EQUAL(BMS_STATE_DRIVE, BmsState_GetOutput().state);
    BmsState_Update(&s_in, now + 1000U);
    TEST_ASSERT_EQUAL(BMS_STATE_DRIVE, BmsState_GetOutput().state);
    BmsState_Update(&s_in, now + 1001U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);

    /* An explicit open request acts immediately. */
    now = EnterDrive(10000U, false);
    s_in.vcu_close_request = false;
    BmsState_Update(&s_in, now + 1U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);

    /* Losing measurement validity does not, on its own, leave DRIVE. */
    now = EnterDrive(20000U, false);
    s_in.meas_valid = false;
    BmsState_Update(&s_in, now + 1U);
    TEST_ASSERT_EQUAL(BMS_STATE_DRIVE, BmsState_GetOutput().state);
}

static void test_charge_state(void)
{
    uint32_t now = EnterDrive(0U, true);
    TEST_ASSERT_EQUAL(BMS_STATE_CHARGE, BmsState_GetOutput().state);

    s_in.charger_present = false;
    BmsState_Update(&s_in, now + 10U);
    TEST_ASSERT_EQUAL(BMS_STATE_DRIVE, BmsState_GetOutput().state);

    /* VCU open request returns to STANDBY from CHARGE. */
    now = EnterDrive(10000U, true);
    s_in.vcu_close_request = false;
    BmsState_Update(&s_in, now + 10U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);
}

static void test_sleep_state(void)
{
    BmsState_Init(0U);
    BmsStateInput_t in = MakeInput();
    BmsState_Update(&in, 200U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);

    in.sleep_request = true;
    BmsState_Update(&in, 210U);
    TEST_ASSERT_EQUAL(BMS_STATE_SLEEP, BmsState_GetOutput().state);
    TEST_ASSERT_FALSE(BmsState_ContactorsClosed());

    in.sleep_request = false;
    BmsState_Update(&in, 220U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);
}

static void test_fault_hold_then_open(void)
{
    uint32_t now = EnterDrive(0U, false);

    s_in.fault_active = true;
    s_in.open_immediate = false;
    BmsState_Update(&s_in, now + 10U);
    TEST_ASSERT_EQUAL(BMS_STATE_FAULT, BmsState_GetOutput().state);

    /* Contactors are held closed for up to 1 s so the VCU can shed current. */
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_neg);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_pos);

    BmsState_Update(&s_in, now + 1009U);
    TEST_ASSERT_TRUE(BmsState_GetOutput().close_neg);

    BmsState_Update(&s_in, now + 1010U);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_neg);
    TEST_ASSERT_FALSE(BmsState_GetOutput().close_pos);

    /* The fault must clear before the state machine returns to STANDBY. */
    BmsState_Update(&s_in, now + 1020U);
    TEST_ASSERT_EQUAL(BMS_STATE_FAULT, BmsState_GetOutput().state);

    s_in.fault_active = false;
    BmsState_Update(&s_in, now + 1030U);
    TEST_ASSERT_EQUAL(BMS_STATE_STANDBY, BmsState_GetOutput().state);
}

static void test_immediate_open_on_fault(void)
{
    uint32_t now = EnterDrive(0U, false);

    s_in.fault_active = true;
    s_in.open_immediate = true;
    BmsState_Update(&s_in, now + 10U);

    BmsStateOutput_t out = BmsState_GetOutput();
    TEST_ASSERT_EQUAL(BMS_STATE_FAULT, out.state);
    TEST_ASSERT_FALSE(out.close_neg);
    TEST_ASSERT_FALSE(out.close_pre);
    TEST_ASSERT_FALSE(out.close_pos);
}

static void test_names_and_guards(void)
{
    TEST_ASSERT_EQUAL_STRING("INIT", BmsState_Name(BMS_STATE_INIT));
    TEST_ASSERT_EQUAL_STRING("STANDBY", BmsState_Name(BMS_STATE_STANDBY));
    TEST_ASSERT_EQUAL_STRING("PRECHARGE", BmsState_Name(BMS_STATE_PRECHARGE));
    TEST_ASSERT_EQUAL_STRING("DRIVE", BmsState_Name(BMS_STATE_DRIVE));
    TEST_ASSERT_EQUAL_STRING("CHARGE", BmsState_Name(BMS_STATE_CHARGE));
    TEST_ASSERT_EQUAL_STRING("FAULT", BmsState_Name(BMS_STATE_FAULT));
    TEST_ASSERT_EQUAL_STRING("SLEEP", BmsState_Name(BMS_STATE_SLEEP));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", BmsState_Name(BMS_STATE_COUNT));

    BmsState_Init(0U);
    BmsState_t before = BmsState_GetOutput().state;
    BmsState_Update(NULL, 100U);
    TEST_ASSERT_EQUAL(before, BmsState_GetOutput().state);
}

void run_state_tests(void)
{
    RUN_TEST(test_init_to_standby);
    RUN_TEST(test_afe_fault_during_init_and_recovery);
    RUN_TEST(test_precharge_sequence);
    RUN_TEST(test_precharge_abort_by_vcu);
    RUN_TEST(test_precharge_timeout);
    RUN_TEST(test_drive_vcu_loss_and_release);
    RUN_TEST(test_charge_state);
    RUN_TEST(test_sleep_state);
    RUN_TEST(test_fault_hold_then_open);
    RUN_TEST(test_immediate_open_on_fault);
    RUN_TEST(test_names_and_guards);
}
