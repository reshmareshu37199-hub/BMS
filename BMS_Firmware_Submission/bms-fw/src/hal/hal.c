#include "hal/hal.h"

#include <stddef.h>

static const HalOps_t s_null_ops = {0};
static const HalOps_t *s_ops = &s_null_ops;

void Hal_Init(const HalOps_t *ops)
{
    s_ops = (ops != NULL) ? ops : &s_null_ops;
}

const HalOps_t *Hal_GetOps(void)
{
    return s_ops;
}

uint32_t Hal_TimeMs(void)
{
    return (s_ops->time_ms != NULL) ? s_ops->time_ms() : 0U;
}

bool Hal_SpiTransfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    if (s_ops->spi_transfer == NULL)
    {
        return false;
    }
    return s_ops->spi_transfer(tx, tx_len, rx, rx_len);
}

void Hal_ContactorSet(HalContactor_t contactor, bool closed)
{
    if (s_ops->contactor_set != NULL)
    {
        s_ops->contactor_set(contactor, closed);
    }
}

bool Hal_ContactorGet(HalContactor_t contactor)
{
    return (s_ops->contactor_get != NULL) ? s_ops->contactor_get(contactor) : false;
}

uint16_t Hal_PackVoltageMv(void)
{
    return (s_ops->pack_voltage_mv != NULL) ? s_ops->pack_voltage_mv() : 0U;
}

uint16_t Hal_BusVoltageMv(void)
{
    return (s_ops->bus_voltage_mv != NULL) ? s_ops->bus_voltage_mv() : 0U;
}

int32_t Hal_PackCurrentMa(void)
{
    return (s_ops->pack_current_ma != NULL) ? s_ops->pack_current_ma() : 0;
}

int16_t Hal_BoardTempDc(void)
{
    return (s_ops->board_temp_dc != NULL) ? s_ops->board_temp_dc() : 0;
}

bool Hal_ChargerPresent(void)
{
    return (s_ops->charger_present != NULL) ? s_ops->charger_present() : false;
}

bool Hal_CanSend(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (s_ops->can_send == NULL)
    {
        return false;
    }
    return s_ops->can_send(id, data, dlc);
}

bool Hal_CanRecv(uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    if (s_ops->can_recv == NULL)
    {
        return false;
    }
    return s_ops->can_recv(id, data, dlc);
}

void Hal_CriticalEnter(void)
{
    if (s_ops->critical_enter != NULL)
    {
        s_ops->critical_enter();
    }
}

void Hal_CriticalExit(void)
{
    if (s_ops->critical_exit != NULL)
    {
        s_ops->critical_exit();
    }
}

bool Hal_NvmRead(uint16_t slot, void *buf, uint16_t len)
{
    if (s_ops->nvm_read == NULL)
    {
        return false;
    }
    return s_ops->nvm_read(slot, buf, len);
}

bool Hal_NvmWrite(uint16_t slot, const void *buf, uint16_t len)
{
    if (s_ops->nvm_write == NULL)
    {
        return false;
    }
    return s_ops->nvm_write(slot, buf, len);
}

void Hal_WatchdogKick(void)
{
    if (s_ops->watchdog_kick != NULL)
    {
        s_ops->watchdog_kick();
    }
}
