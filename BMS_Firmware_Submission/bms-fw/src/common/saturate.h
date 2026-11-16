#ifndef BMS_COMMON_SATURATE_H
#define BMS_COMMON_SATURATE_H

#include <stdint.h>

/** Clamp a signed 64-bit value into the int32_t range. */
static inline int32_t Sat_ToI32(int64_t value)
{
    if (value > (int64_t)INT32_MAX)
    {
        return INT32_MAX;
    }
    if (value < (int64_t)INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)value;
}

/** Clamp a signed 64-bit value into [lo, hi]. */
static inline int32_t Sat_I32(int64_t value, int32_t lo, int32_t hi)
{
    if (value > (int64_t)hi)
    {
        return hi;
    }
    if (value < (int64_t)lo)
    {
        return lo;
    }
    return (int32_t)value;
}

/** Clamp an unsigned 64-bit value into [0, hi]. */
static inline uint32_t Sat_U32(uint64_t value, uint32_t hi)
{
    if (value > (uint64_t)hi)
    {
        return hi;
    }
    return (uint32_t)value;
}

/** Round a rational value num/den to the nearest integer (den > 0). */
static inline int64_t Sat_DivRound(int64_t num, int64_t den)
{
    if (den <= 0)
    {
        return 0;
    }
    if (num >= 0)
    {
        return (num + (den / 2)) / den;
    }
    return -((-num + (den / 2)) / den);
}

#endif /* BMS_COMMON_SATURATE_H */
