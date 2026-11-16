#ifndef BMS_SERVICES_CAN_SIGNAL_H
#define BMS_SERVICES_CAN_SIGNAL_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_SIGNAL_MAX_BITS 32U
#define CAN_SIGNAL_MAX_BYTES 8U

typedef enum
{
    CAN_SIG_UNSIGNED = 0,
    CAN_SIG_SIGNED = 1
} CanSigType_t;

/**
 * Generic description of one CAN signal.
 *
 * Bit numbering follows the DBC/Intel convention: bit 0 is the least
 * significant bit of byte 0, and a signal grows towards higher bit numbers,
 * wrapping into the next byte.  This is the little-endian ("Intel") layout
 * used by the whole matrix.
 *
 * Physical value = raw * scale_num / scale_den + offset.
 * The physical value is always an integer in the unit shown in the DBC
 * (mV, mA, 0.1 degC, ...), which keeps the whole signal layer free of
 * floating point.
 */
typedef struct
{
    uint8_t start_bit;
    uint8_t length;
    CanSigType_t type;
    int32_t scale_num;
    int32_t scale_den;
    int32_t offset;
    int32_t phys_min;
    int32_t phys_max;
} CanSignalDef_t;

/** Write a raw bit pattern into an Intel-ordered CAN payload. */
void CanSig_WriteRaw(uint8_t *data, uint8_t data_len, uint8_t start_bit, uint8_t length, uint64_t raw);

/** Read a raw bit pattern from an Intel-ordered CAN payload. */
uint64_t CanSig_ReadRaw(const uint8_t *data, uint8_t data_len, uint8_t start_bit, uint8_t length);

/** Convert a raw bit pattern to a physical integer (with sign extension). */
int32_t CanSig_RawToPhys(const CanSignalDef_t *sig, uint64_t raw);

/**
 * Convert a physical value to a raw bit pattern.
 *
 * The physical value is saturated to [phys_min, phys_max] before scaling and
 * the result is saturated to the representable raw range.  There is therefore
 * no silent overflow, regardless of the input.
 */
uint64_t CanSig_PhysToRaw(const CanSignalDef_t *sig, int32_t physical);

bool CanSig_Pack(uint8_t *data, uint8_t data_len, const CanSignalDef_t *sig, int32_t physical);
bool CanSig_Unpack(const uint8_t *data, uint8_t data_len, const CanSignalDef_t *sig, int32_t *physical);

/** Largest raw value that fits in `length` bits for the given signedness. */
uint64_t CanSig_RawMax(uint8_t length, CanSigType_t type);

/** Smallest raw value that fits in `length` bits for the given signedness. */
int64_t CanSig_RawMin(uint8_t length, CanSigType_t type);

#endif /* BMS_SERVICES_CAN_SIGNAL_H */
