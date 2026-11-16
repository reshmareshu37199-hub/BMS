cmake_minimum_required(VERSION 3.16)

project(bms_fw
    VERSION 1.0.0
    DESCRIPTION "16S LFP Battery Management System reference firmware"
    LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

option(BMS_COVERAGE "Build tests with gcov coverage instrumentation" OFF)

set(BMS_WARNINGS
    -Wall
    -Wextra
    -Werror
    -Wpedantic
    -Wshadow
    -Wstrict-prototypes
    -Wmissing-prototypes
    -Wdouble-promotion
    -Wformat=2
    -Wundef
    -Wcast-qual
    -Wpointer-arith)

if(MSVC)
    set(BMS_WARNINGS /W4 /WX)
endif()

# ---------------------------------------------------------------------------
# Core firmware library.  No dynamic allocation, no printf, no OS calls.
# ---------------------------------------------------------------------------
set(BMS_CORE_SOURCES
    src/common/crc.c
    src/hal/hal.c
    src/hal/hal_sim.c
    src/hal/afe_sim.c
    src/drivers/afe.c
    src/drivers/ntc_table.c
    src/drivers/can_driver.c
    src/services/balancing.c
    src/services/protection.c
    src/services/can_signal.c
    src/services/can_matrix.c
    src/services/isotp.c
    src/services/uds.c
    src/services/soc.c
    src/services/nvm.c
    src/app/bms_state.c
    src/app/app.c
    src/app/scheduler.c)

add_library(bms_core STATIC ${BMS_CORE_SOURCES})
target_include_directories(bms_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(bms_core PRIVATE ${BMS_WARNINGS})

if(BMS_COVERAGE)
    target_compile_options(bms_core PRIVATE --coverage -O0 -g)
    target_link_options(bms_core PUBLIC --coverage)
endif()

# ---------------------------------------------------------------------------
# Unity test framework (MIT).  Compiled with warnings suppressed because it is
# third-party code that we do not modify.
# ---------------------------------------------------------------------------
add_library(unity STATIC third_party/unity/unity.c)
target_include_directories(unity PUBLIC third_party/unity)
if(NOT MSVC)
    target_compile_options(unity PRIVATE -w)
endif()

# ---------------------------------------------------------------------------
# Host simulation executable
# ---------------------------------------------------------------------------
add_executable(bms_sim sim/sim_main.c)
target_link_libraries(bms_sim PRIVATE bms_core)
target_compile_options(bms_sim PRIVATE ${BMS_WARNINGS})

# ---------------------------------------------------------------------------
# Dependency-free CSV -> SVG plotter used by the `simulate` target.
# ---------------------------------------------------------------------------
add_executable(bms_plot tools/plot_svg.c)
target_compile_options(bms_plot PRIVATE ${BMS_WARNINGS})

set(BMS_TRACE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sim/out)

# Regenerate the scenario traces and the SVG plots that are checked in as
# documentation artifacts:  cmake --build <build> --target simulate
add_custom_target(simulate
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BMS_TRACE_DIR}
    COMMAND bms_sim ${CMAKE_CURRENT_SOURCE_DIR}/sim/scenarios/charge_balance.csv ${BMS_TRACE_DIR}/charge_balance.csv
    COMMAND bms_sim ${CMAKE_CURRENT_SOURCE_DIR}/sim/scenarios/fault_recovery.csv ${BMS_TRACE_DIR}/fault_recovery.csv
    COMMAND bms_plot ${BMS_TRACE_DIR}/charge_balance.csv ${BMS_TRACE_DIR}/charge_balance_voltage.svg cell0_mv cell4_mv
    COMMAND bms_plot ${BMS_TRACE_DIR}/charge_balance.csv ${BMS_TRACE_DIR}/charge_balance_balance.svg bal_mask
    COMMAND bms_plot ${BMS_TRACE_DIR}/charge_balance.csv ${BMS_TRACE_DIR}/charge_balance_soc.svg soc_permille
    COMMAND bms_plot ${BMS_TRACE_DIR}/fault_recovery.csv ${BMS_TRACE_DIR}/fault_recovery_voltage.svg cell2_mv
    COMMAND bms_plot ${BMS_TRACE_DIR}/fault_recovery.csv ${BMS_TRACE_DIR}/fault_recovery_contacts.svg neg pre pos fault_bits
    DEPENDS bms_sim bms_plot
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    USES_TERMINAL)

# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------
set(BMS_TEST_SOURCES
    test/test_main.c
    test/test_hal.c
    test/test_balancing.c
    test/test_protection.c
    test/test_afe.c
    test/test_can.c
    test/test_uds.c
    test/test_soc.c
    test/test_state.c
    test/test_nvm.c)

add_executable(bms_tests ${BMS_TEST_SOURCES})
target_link_libraries(bms_tests PRIVATE bms_core unity)
target_include_directories(bms_tests PRIVATE test)
target_compile_options(bms_tests PRIVATE ${BMS_WARNINGS})
if(BMS_COVERAGE)
    target_compile_options(bms_tests PRIVATE --coverage -O0 -g)
    target_link_options(bms_tests PRIVATE --coverage)
endif()

enable_testing()
add_test(NAME bms_unit_tests COMMAND bms_tests)

# Convenience target: run tests from the build directory.
add_custom_target(check
    COMMAND ${CMAKE_COMMAND} -E echo "Running BMS unit tests..."
    COMMAND bms_tests
    DEPENDS bms_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    USES_TERMINAL)
