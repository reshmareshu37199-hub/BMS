#include "hal/hal_sim.h"

#include "hal/afe_sim.h"
#include "hal/hal.h"

#include <string.h>

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} SimCanFrame_t;

typedef struct
{
    uint32_t time_ms;
    uint16_t pack_voltage_mv;
    uint16_t bus_voltage_mv;
    int32_t pack_current_ma;
    int16_t board_temp_dc;
    bool charger_present;
    bool contactors[HAL_CONTACTOR_COUNT];
    uint8_t nvm[HAL_NVM_SLOT_COUNT][HAL_NVM_SLOT_SIZE];
    bool nvm_valid[HAL_NVM_SLOT_COUNT];
    SimCanFrame_t tx_log[HALSIM_CAN_TX_LOG_SIZE];
    uint32_t tx_count;
    SimCanFrame_t rx_queue[64];
    uint8_t rx_head;
    uint8_t rx_tail;
} HalSimState_t;

static HalSimState_t s_sim;

static uint32_t Sim_TimeMs(void)
{
    return s_sim.time_ms;
}

static bool Sim_SpiTransfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    return AfeSim_Transfer(tx, tx_len, rx, rx_len, s_sim.time_ms);
}

static void Sim_ContactorSet(HalContactor_t contactor, bool closed)
{
    if ((uint32_t)contactor < (uint32_t)HAL_CONTACTOR_COUNT)
    {
        s_sim.contactors[contactor] = closed;
    }
}

static bool Sim_ContactorGet(HalContactor_t contactor)
{
    return ((uint32_t)contactor < (uint32_t)HAL_CONTACTOR_COUNT) ? s_sim.contactors[contactor] : false;
}

static uint16_t Sim_PackVoltageMv(void)
{
    return s_sim.pack_voltage_mv;
}

static uint16_t Sim_BusVoltageMv(void)
{
    return s_sim.bus_voltage_mv;
}

static int32_t Sim_PackCurrentMa(void)
{
    return s_sim.pack_current_ma;
}

static int16_t Sim_BoardTempDc(void)
{
    return s_sim.board_temp_dc;
}

static bool Sim_ChargerPresent(void)
{
    return s_sim.charger_present;
}

static bool Sim_CanSend(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if ((data == NULL) || (dlc > 8U) || (s_sim.tx_count >= HALSIM_CAN_TX_LOG_SIZE))
    {
        return false;
    }
    SimCanFrame_t *frame = &s_sim.tx_log[s_sim.tx_count];
    frame->id = id;
    frame->dlc = dlc;
    (void)memset(frame->data, 0, sizeof(frame->data));
    (void)memcpy(frame->data, data, dlc);
    s_sim.tx_count++;
    return true;
}

static bool Sim_CanRecv(uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    if ((id == NULL) || (data == NULL) || (dlc == NULL) || (s_sim.rx_head == s_sim.rx_tail))
    {
        return false;
    }
    const SimCanFrame_t *frame = &s_sim.rx_queue[s_sim.rx_head];
    *id = frame->id;
    *dlc = frame->dlc;
    (void)memcpy(data, frame->data, frame->dlc);
    s_sim.rx_head = (uint8_t)((s_sim.rx_head + 1U) % 64U);
    return true;
}

static void Sim_CriticalEnter(void)
{
    /* Single-threaded host simulation: nothing to do. */
}

static void Sim_CriticalExit(void)
{
    /* Single-threaded host simulation: nothing to do. */
}

static bool Sim_NvmRead(uint16_t slot, void *buf, uint16_t len)
{
    if ((buf == NULL) || (slot >= HAL_NVM_SLOT_COUNT) || (len > HAL_NVM_SLOT_SIZE) || !s_sim.nvm_valid[slot])
    {
        return false;
    }
    (void)memcpy(buf, s_sim.nvm[slot], len);
    return true;
}

static bool Sim_NvmWrite(uint16_t slot, const void *buf, uint16_t len)
{
    if ((buf == NULL) || (slot >= HAL_NVM_SLOT_COUNT) || (len > HAL_NVM_SLOT_SIZE))
    {
        return false;
    }
    (void)memset(s_sim.nvm[slot], 0xFF, HAL_NVM_SLOT_SIZE);
    (void)memcpy(s_sim.nvm[slot], buf, len);
    s_sim.nvm_valid[slot] = true;
    return true;
}

static void Sim_WatchdogKick(void)
{
    /* No-op in the simulation. */
}

static const HalOps_t s_sim_ops = {
    .time_ms = Sim_TimeMs,
    .spi_transfer = Sim_SpiTransfer,
    .contactor_set = Sim_ContactorSet,
    .contactor_get = Sim_ContactorGet,
    .pack_voltage_mv = Sim_PackVoltageMv,
    .bus_voltage_mv = Sim_BusVoltageMv,
    .pack_current_ma = Sim_PackCurrentMa,
    .board_temp_dc = Sim_BoardTempDc,
    .charger_present = Sim_ChargerPresent,
    .can_send = Sim_CanSend,
    .can_recv = Sim_CanRecv,
    .critical_enter = Sim_CriticalEnter,
    .critical_exit = Sim_CriticalExit,
    .nvm_read = Sim_NvmRead,
    .nvm_write = Sim_NvmWrite,
    .watchdog_kick = Sim_WatchdogKick,
};

void HalSim_Init(void)
{
    (void)memset(&s_sim, 0, sizeof(s_sim));
    s_sim.pack_voltage_mv = 51200U;
    s_sim.bus_voltage_mv = 0U;
    s_sim.board_temp_dc = 250;
    AfeSim_Init();
    Hal_Init(&s_sim_ops);
}

void HalSim_Reset(void)
{
    HalSim_Init();
}

void HalSim_AdvanceMs(uint32_t milliseconds)
{
    s_sim.time_ms += milliseconds;
}

uint32_t HalSim_TimeMs(void)
{
    return s_sim.time_ms;
}

void HalSim_SetCellMv(uint8_t index, uint16_t millivolts)
{
    AfeSim_SetCellMv(index, millivolts);
}

void HalSim_SetCellTempDeciC(uint8_t index, int16_t deci_celsius)
{
    AfeSim_SetTempDeciC(index, deci_celsius);
}

void HalSim_SetPackVoltageMv(uint16_t millivolts)
{
    s_sim.pack_voltage_mv = millivolts;
}

void HalSim_SetBusVoltageMv(uint16_t millivolts)
{
    s_sim.bus_voltage_mv = millivolts;
}

void HalSim_SetPackCurrentMa(int32_t milliamps)
{
    s_sim.pack_current_ma = milliamps;
}

void HalSim_SetBoardTempDc(int16_t deci_celsius)
{
    s_sim.board_temp_dc = deci_celsius;
}

void HalSim_SetChargerPresent(bool present)
{
    s_sim.charger_present = present;
}

void HalSim_CanInject(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if ((data == NULL) || (dlc > 8U))
    {
        return;
    }
    uint8_t next = (uint8_t)((s_sim.rx_tail + 1U) % 64U);
    if (next == s_sim.rx_head)
    {
        return;
    }
    SimCanFrame_t *frame = &s_sim.rx_queue[s_sim.rx_tail];
    frame->id = id;
    frame->dlc = dlc;
    (void)memset(frame->data, 0, sizeof(frame->data));
    (void)memcpy(frame->data, data, dlc);
    s_sim.rx_tail = next;
}

uint32_t HalSim_CanTxCount(void)
{
    return s_sim.tx_count;
}

bool HalSim_CanTxGet(uint32_t index, uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    if ((id == NULL) || (data == NULL) || (dlc == NULL) || (index >= s_sim.tx_count))
    {
        return false;
    }
    *id = s_sim.tx_log[index].id;
    *dlc = s_sim.tx_log[index].dlc;
    (void)memcpy(data, s_sim.tx_log[index].data, s_sim.tx_log[index].dlc);
    return true;
}

void HalSim_CanTxClear(void)
{
    s_sim.tx_count = 0U;
}

void HalSim_NvmClear(void)
{
    (void)memset(s_sim.nvm, 0xFF, sizeof(s_sim.nvm));
    (void)memset(s_sim.nvm_valid, 0, sizeof(s_sim.nvm_valid));
}

bool HalSim_NvmReadRaw(uint16_t slot, void *buf, uint16_t len)
{
    return Sim_NvmRead(slot, buf, len);
}

bool HalSim_NvmWriteRaw(uint16_t slot, const void *buf, uint16_t len)
{
    return Sim_NvmWrite(slot, buf, len);
}

bool HalSim_ContactorClosed(uint8_t contactor)
{
    return (contactor < (uint8_t)HAL_CONTACTOR_COUNT) ? s_sim.contactors[contactor] : false;
}
