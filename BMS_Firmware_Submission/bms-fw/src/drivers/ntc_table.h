#ifndef BMS_DRIVERS_NTC_TABLE_H
#define BMS_DRIVERS_NTC_TABLE_H

#include <stdint.h>

#define NTC_TABLE_POINTS 166
#define NTC_TEMP_MIN_DC (-400)
#define NTC_TEMP_MAX_DC (1250)

/**
 * Convert a 12-bit NTC divider reading to temperature in 0.1 degC.
 *
 * The NTC is a 10 kOhm, B25/85 = 3435 K part connected as the low side of a
 * divider with a 10 kOhm pull-up to 3.3 V, so raw increases as temperature
 * decreases.  The table contains one point per 1 degC; adjacent points are
 * linearly interpolated.  At the ends the result is clamped.
 */
int16_t Ntc_RawToDeciC(uint16_t raw);

/** Inverse of Ntc_RawToDeciC, used by the hardware simulator and tests. */
uint16_t Ntc_DeciCToRaw(int16_t deci_c);

#endif /* BMS_DRIVERS_NTC_TABLE_H */
