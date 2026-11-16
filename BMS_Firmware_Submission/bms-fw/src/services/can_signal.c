#include "services/can_signal.h"

#include "common/saturate.h"

static uint64_t BitMask(uint8_t length)
{
    if (length == 0U)
    {
        return 0U;
    }
    if (length >= 64U)
    {
        return UINT64_MAX;
    }
    return ((uint64_t)1U << length) - 1U;
}

void CanSig_WriteRaw(uint8_t *data, uint8_t data_len, uint8_t start_bit, uint8_t length, uint64_t raw)
{
    if ((data == NULL) || (length == 0U) || (length > CAN_SIGNAL_MAX_BITS))
    {
        return;
    }
    if ((uint16_t)start_bit + (uint16_t)length > ((uint16_t)data_len * 8U))
    {
        return;
    }

    for (uint8_t i = 0U; i < length; ++i)
    {
        uint16_t bit = (uint16_t)start_bit + (uint16_t)i;
        uint8_t byte_index = (uint8_t)(bit / 8U);
        uint8_t bit_index = (uint8_t)(bit % 8U);

        if (((raw >> i) & 1U) != 0U)
        {
            data[byte_index] = (uint8_t)(data[byte_index] | (uint8_t)(1U << bit_index));
        }
        else
        {
            data[byte_index] = (uint8_t)(data[byte_index] & (uint8_t)~(uint8_t)(1U << bit_index));
        }
    }
}

uint64_t CanSig_ReadRaw(const uint8_t *data, uint8_t data_len, uint8_t start_bit, uint8_t length)
{
    uint64_t raw = 0U;

    if ((data == NULL) || (length == 0U) || (length > CAN_SIGNAL_MAX_BITS))
    {
        return 0U;
    }
    if ((uint16_t)start_bit + (uint16_t)length > ((uint16_t)data_len * 8U))
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < length; ++i)
    {
        uint16_t bit = (uint16_t)start_bit + (uint16_t)i;
        uint8_t byte_index = (uint8_t)(bit / 8U);
        uint8_t bit_index = (uint8_t)(bit % 8U);

        if (((data[byte_index] >> bit_index) & 1U) != 0U)
        {
            raw |= ((uint64_t)1U << i);
        }
    }

    return raw;
}

uint64_t CanSig_RawMax(uint8_t length, CanSigType_t type)
{
    uint64_t mask = BitMask(length);

    if (type == CAN_SIG_SIGNED)
    {
        return (uint64_t)(mask >> 1U);
    }
    return mask;
}

int64_t CanSig_RawMin(uint8_t length, CanSigType_t type)
{
    if (length == 0U)
    {
        return 0;
    }
    if (type == CAN_SIG_SIGNED)
    {
        return -(int64_t)((uint64_t)1U << (length - 1U));
    }
    return 0;
}

int32_t CanSig_RawToPhys(const CanSignalDef_t *sig, uint64_t raw)
{
    int64_t signed_raw = (int64_t)raw;

    if ((sig == NULL) || (sig->scale_den == 0))
    {
        return 0;
    }

    if (sig->type == CAN_SIG_SIGNED)
    {
        uint64_t mask = BitMask(sig->length);
        if ((raw & ((uint64_t)1U << (sig->length - 1U))) != 0U)
        {
            signed_raw = (int64_t)(raw | ~mask);
        }
    }

    int64_t scaled = Sat_DivRound(signed_raw * (int64_t)sig->scale_num, (int64_t)sig->scale_den);
    int64_t physical = scaled + (int64_t)sig->offset;
    return Sat_I32(physical, INT32_MIN, INT32_MAX);
}

uint64_t CanSig_PhysToRaw(const CanSignalDef_t *sig, int32_t physical)
{
    if ((sig == NULL) || (sig->scale_num == 0) || (sig->scale_den == 0) || (sig->length == 0U))
    {
        return 0U;
    }

    int32_t clamped = Sat_I32((int64_t)physical, sig->phys_min, sig->phys_max);
    int64_t numerator = ((int64_t)clamped - (int64_t)sig->offset) * (int64_t)sig->scale_den;
    int64_t raw64 = Sat_DivRound(numerator, (int64_t)sig->scale_num);

    int64_t min_raw = CanSig_RawMin(sig->length, sig->type);
    int64_t max_raw = (int64_t)CanSig_RawMax(sig->length, sig->type);

    if (raw64 < min_raw)
    {
        raw64 = min_raw;
    }
    if (raw64 > max_raw)
    {
        raw64 = max_raw;
    }

    return (uint64_t)raw64 & BitMask(sig->length);
}

bool CanSig_Pack(uint8_t *data, uint8_t data_len, const CanSignalDef_t *sig, int32_t physical)
{
    if ((data == NULL) || (sig == NULL))
    {
        return false;
    }
    if ((uint16_t)sig->start_bit + (uint16_t)sig->length > ((uint16_t)data_len * 8U))
    {
        return false;
    }
    CanSig_WriteRaw(data, data_len, sig->start_bit, sig->length, CanSig_PhysToRaw(sig, physical));
    return true;
}

bool CanSig_Unpack(const uint8_t *data, uint8_t data_len, const CanSignalDef_t *sig, int32_t *physical)
{
    if ((data == NULL) || (sig == NULL) || (physical == NULL))
    {
        return false;
    }
    if ((uint16_t)sig->start_bit + (uint16_t)sig->length > ((uint16_t)data_len * 8U))
    {
        return false;
    }
    *physical = CanSig_RawToPhys(sig, CanSig_ReadRaw(data, data_len, sig->start_bit, sig->length));
    return true;
}
