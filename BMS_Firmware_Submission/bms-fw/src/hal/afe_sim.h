#ifndef BMS_HAL_AFE_SIM_H
#define BMS_HAL_AFE_SIM_H

#include <stdbool.h>
#include <stdint.h>

#define AFE_SIM_CELL_COUNT 16U
#define AFE_SIM_TEMP_COUNT 5U

void AfeSim_Init(void);

/** SPI slave model.  Called by the PC HAL. */
bool AfeSim_Transfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len, uint32_t now_ms);

void AfeSim_SetCellMv(uint8_t index, uint16_t millivolts);
void AfeSim_SetTempDeciC(uint8_t index, int16_t deci_celsius);
void AfeSim_SetWrongId(bool wrong);
void AfeSim_SetForceCrcError(bool force);
void AfeSim_SetForceTimeout(bool force);
void AfeSim_SetInternalFault(bool fault);
void AfeSim_SetOpenWire(bool open_wire);

uint16_t AfeSim_GetBalMask(void);
bool AfeSim_IsConverting(void);
uint16_t AfeSim_GetCellRaw(uint8_t index);

#endif /* BMS_HAL_AFE_SIM_H */
