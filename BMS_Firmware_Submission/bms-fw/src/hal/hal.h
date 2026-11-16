#ifndef BMS_HAL_HAL_H
#define BMS_HAL_HAL_H

#include <stdbool.h>
#include <stdint.h>

#define HAL_NVM_SLOT_A 0U
#define HAL_NVM_SLOT_B 1U
#define HAL_NVM_SLOT_COUNT 2U
#define HAL_NVM_SLOT_SIZE 512U

typedef enum
{
    HAL_CONTACTOR_NEG = 0,
    HAL_CONTACTOR_PRE = 1,
    HAL_CONTACTOR_POS = 2,
    HAL_CONTACTOR_COUNT = 3
} HalContactor_t;

/**
 * Hardware abstraction layer.
 *
 * Every function pointer must remain valid for the lifetime of the program.
 * A NULL pointer means "not implemented"; the wrapper functions return a safe
 * default (false / 0 / no-op) so that a missing optional peripheral cannot
 * cause undefined behaviour.
 */
typedef struct
{
    uint32_t (*time_ms)(void);

    /* Full-duplex SPI transfer.  tx_len bytes are clocked out; rx_len bytes
     * are captured.  rx_len may differ from tx_len for register reads. */
    bool (*spi_transfer)(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len);

    void (*contactor_set)(HalContactor_t contactor, bool closed);
    bool (*contactor_get)(HalContactor_t contactor);

    uint16_t (*pack_voltage_mv)(void);
    uint16_t (*bus_voltage_mv)(void);
    int32_t (*pack_current_ma)(void);
    int16_t (*board_temp_dc)(void);
    bool (*charger_present)(void);

    /* Classic CAN, 11-bit identifiers, 8 data bytes maximum. */
    bool (*can_send)(uint32_t id, const uint8_t *data, uint8_t dlc);
    bool (*can_recv)(uint32_t *id, uint8_t *data, uint8_t *dlc);

    void (*critical_enter)(void);
    void (*critical_exit)(void);

    bool (*nvm_read)(uint16_t slot, void *buf, uint16_t len);
    bool (*nvm_write)(uint16_t slot, const void *buf, uint16_t len);

    void (*watchdog_kick)(void);
} HalOps_t;

void Hal_Init(const HalOps_t *ops);
const HalOps_t *Hal_GetOps(void);

uint32_t Hal_TimeMs(void);
bool Hal_SpiTransfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len);
void Hal_ContactorSet(HalContactor_t contactor, bool closed);
bool Hal_ContactorGet(HalContactor_t contactor);
uint16_t Hal_PackVoltageMv(void);
uint16_t Hal_BusVoltageMv(void);
int32_t Hal_PackCurrentMa(void);
int16_t Hal_BoardTempDc(void);
bool Hal_ChargerPresent(void);
bool Hal_CanSend(uint32_t id, const uint8_t *data, uint8_t dlc);
bool Hal_CanRecv(uint32_t *id, uint8_t *data, uint8_t *dlc);
void Hal_CriticalEnter(void);
void Hal_CriticalExit(void);
bool Hal_NvmRead(uint16_t slot, void *buf, uint16_t len);
bool Hal_NvmWrite(uint16_t slot, const void *buf, uint16_t len);
void Hal_WatchdogKick(void);

#endif /* BMS_HAL_HAL_H */
