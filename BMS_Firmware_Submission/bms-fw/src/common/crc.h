#ifndef BMS_COMMON_CRC_H
#define BMS_COMMON_CRC_H

#include <stddef.h>
#include <stdint.h>

/**
 * CRC-8 used by the AFE SPI transport.
 *
 * Polynomial 0x07, initial value 0x00, no reflection, no final XOR.
 * This is the "CRC-8/SMBUS" parameter set.
 */
uint8_t Crc8_Poly07(const uint8_t *data, size_t len);

/**
 * CRC-8 SAE J1850 used by the CAN messages.
 *
 * Polynomial 0x1D, initial value 0xFF, no reflection, final XOR 0xFF.
 */
uint8_t Crc8_SaeJ1850(const uint8_t *data, size_t len);

/**
 * CRC-16/CCITT-FALSE used by the non-volatile parameter store.
 *
 * Polynomial 0x1021, initial value 0xFFFF, no reflection, no final XOR.
 */
uint16_t Crc16_Ccitt(const uint8_t *data, size_t len);

#endif /* BMS_COMMON_CRC_H */
