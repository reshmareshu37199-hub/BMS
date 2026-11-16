#ifndef BMS_HAL_HAL_SIM_H
#define BMS_HAL_HAL_SIM_H

#include <stdbool.h>
#include <stdint.h>

#define HALSIM_CAN_TX_LOG_SIZE 512U

void HalSim_Init(void);
void HalSim_Reset(void);
void HalSim_AdvanceMs(uint32_t milliseconds);
uint32_t HalSim_TimeMs(void);

void HalSim_SetCellMv(uint8_t index, uint16_t millivolts);
void HalSim_SetCellTempDeciC(uint8_t index, int16_t deci_celsius);
void HalSim_SetPackVoltageMv(uint16_t millivolts);
void HalSim_SetBusVoltageMv(uint16_t millivolts);
void HalSim_SetPackCurrentMa(int32_t milliamps);
void HalSim_SetBoardTempDc(int16_t deci_celsius);
void HalSim_SetChargerPresent(bool present);

void HalSim_CanInject(uint32_t id, const uint8_t *data, uint8_t dlc);
uint32_t HalSim_CanTxCount(void);
bool HalSim_CanTxGet(uint32_t index, uint32_t *id, uint8_t *data, uint8_t *dlc);
void HalSim_CanTxClear(void);

void HalSim_NvmClear(void);
bool HalSim_NvmReadRaw(uint16_t slot, void *buf, uint16_t len);
bool HalSim_NvmWriteRaw(uint16_t slot, const void *buf, uint16_t len);

bool HalSim_ContactorClosed(uint8_t contactor);

#endif /* BMS_HAL_HAL_SIM_H */
