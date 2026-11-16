#include "common/crc.h"

uint8_t Crc8_Poly07(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00U;

    if (data == NULL)
    {
        return crc;
    }

    for (size_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

uint8_t Crc8_SaeJ1850(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFFU;

    if (data == NULL)
    {
        return (uint8_t)(crc ^ 0xFFU);
    }

    for (size_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x1DU);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return (uint8_t)(crc ^ 0xFFU);
}

uint16_t Crc16_Ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL)
    {
        return crc;
    }

    for (size_t i = 0U; i < len; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((uint16_t)(crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}
