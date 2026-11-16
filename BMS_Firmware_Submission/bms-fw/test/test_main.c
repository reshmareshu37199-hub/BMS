#include "unity.h"

#include "test_groups.h"

void setUp(void)
{
}

void tearDown(void)
{
}

int main(void)
{
    UNITY_BEGIN();

    run_hal_tests();
    run_balancing_tests();
    run_protection_tests();
    run_afe_tests();
    run_can_tests();
    run_uds_tests();
    run_soc_tests();
    run_state_tests();
    run_nvm_tests();

    return UNITY_END();
}
