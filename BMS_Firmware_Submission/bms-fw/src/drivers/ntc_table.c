#include "drivers/ntc_table.h"

/*
 * Generated 10 kOhm NTC table, Beta 25/85 = 3435 K, 10 kOhm pull-up to 3.3 V,
 * 12-bit ADC (0..4095).  Index i corresponds to -40.0 C + 1.0 C * i.
 * The table stores the ideal ADC code at each 1 C point.  The conversion
 * function interpolates linearly between adjacent points.
 */
static const uint16_t s_ntc_raw[NTC_TABLE_POINTS] = {
    3936, 3927, 3916, 3905, 3894, 3882, 3869, 3856,
    3843, 3828, 3813, 3798, 3782, 3765, 3747, 3729,
    3710, 3690, 3670, 3649, 3627, 3605, 3581, 3557,
    3532, 3507, 3480, 3453, 3425, 3397, 3368, 3337,
    3307, 3275, 3243, 3210, 3177, 3143, 3108, 3073,
    3037, 3001, 2964, 2926, 2888, 2850, 2812, 2773,
    2733, 2694, 2654, 2614, 2573, 2533, 2492, 2451,
    2411, 2370, 2329, 2289, 2248, 2208, 2167, 2127,
    2087, 2048, 2008, 1969, 1930, 1892, 1854, 1816,
    1778, 1742, 1705, 1669, 1634, 1599, 1564, 1530,
    1496, 1463, 1431, 1399, 1368, 1337, 1307, 1277,
    1248, 1219, 1191, 1163, 1137, 1110, 1084, 1059,
    1034, 1010, 986, 963, 940, 918, 897, 875,
    855, 834, 815, 795, 777, 758, 740, 723,
    706, 689, 673, 657, 642, 627, 612, 598,
    584, 570, 557, 544, 531, 519, 507, 495,
    484, 473, 462, 452, 441, 431, 421, 412,
    403, 394, 385, 376, 368, 360, 352, 344,
    336, 329, 322, 315, 308, 301, 295, 289,
    282, 276, 271, 265, 259, 254, 249, 243,
    238, 233, 229, 224, 219, 215
};

int16_t Ntc_RawToDeciC(uint16_t raw)
{
    /* Table is monotonically decreasing with temperature. */
    if (raw >= s_ntc_raw[0])
    {
        return NTC_TEMP_MIN_DC;
    }
    if (raw <= s_ntc_raw[NTC_TABLE_POINTS - 1U])
    {
        return NTC_TEMP_MAX_DC;
    }

    uint16_t lo = 0U;
    uint16_t hi = (uint16_t)(NTC_TABLE_POINTS - 1U);
    while ((uint16_t)(hi - lo) > 1U)
    {
        uint16_t mid = (uint16_t)((uint16_t)(lo + hi) / 2U);
        if (raw <= s_ntc_raw[mid])
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    int32_t span = (int32_t)s_ntc_raw[lo] - (int32_t)s_ntc_raw[hi];
    int32_t delta = (int32_t)s_ntc_raw[lo] - (int32_t)raw;
    int32_t t_dc = NTC_TEMP_MIN_DC + (int32_t)lo * 10;
    if (span > 0)
    {
        t_dc += (delta * 10) / span;
    }
    return (int16_t)t_dc;
}

uint16_t Ntc_DeciCToRaw(int16_t deci_c)
{
    if (deci_c <= NTC_TEMP_MIN_DC)
    {
        return s_ntc_raw[0];
    }
    if (deci_c >= NTC_TEMP_MAX_DC)
    {
        return s_ntc_raw[NTC_TABLE_POINTS - 1U];
    }

    int32_t idx = (int32_t)(deci_c - NTC_TEMP_MIN_DC) / 10;
    int32_t rem = (int32_t)(deci_c - NTC_TEMP_MIN_DC) % 10;
    int32_t a = (int32_t)s_ntc_raw[idx];
    int32_t b = (int32_t)s_ntc_raw[idx + 1];
    int32_t raw = a + ((b - a) * rem) / 10;
    return (uint16_t)raw;
}
