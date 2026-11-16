#include "services/nvm.h"

#include "common/crc.h"
#include "hal/hal.h"

#include <stddef.h>
#include <string.h>

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_len;
    uint32_t sequence;
    uint8_t payload[NVM_PAYLOAD_MAX];
    uint16_t crc;
} NvmRecord_t;

static NvmRecord_t s_record;
static uint32_t s_sequence;
static uint8_t s_valid_slot;
static uint32_t s_write_count;
static bool s_initialised;

static void ResetState(void)
{
    s_sequence = 0U;
    s_valid_slot = 0xFFU;
    s_write_count = 0U;
}

void Nvm_Init(void)
{
    (void)memset(&s_record, 0, sizeof(s_record));
    ResetState();
    s_initialised = true;
}

void Nvm_GetDefaults(NvmParams_t *params)
{
    if (params == NULL)
    {
        return;
    }
    params->bal_start_mv = NVM_DEFAULT_BAL_START_MV;
    params->bal_stop_mv = NVM_DEFAULT_BAL_STOP_MV;
    params->soc_permille = NVM_DEFAULT_SOC_PERMILLE;
    params->flags = 0U;
}

static bool RecordValid(const NvmRecord_t *record)
{
    if (record->magic != NVM_MAGIC)
    {
        return false;
    }
    if (record->version != NVM_VERSION)
    {
        return false;
    }
    if (record->payload_len != (uint16_t)sizeof(NvmParams_t))
    {
        return false;
    }
    uint16_t crc = Crc16_Ccitt((const uint8_t *)record, offsetof(NvmRecord_t, crc));
    return crc == record->crc;
}

bool Nvm_Load(NvmParams_t *params)
{
    if (!s_initialised)
    {
        Nvm_Init();
    }
    if (params == NULL)
    {
        return false;
    }

    ResetState();

    bool found = false;
    for (uint8_t slot = 0U; slot < NVM_SLOT_COUNT; ++slot)
    {
        NvmRecord_t candidate;
        (void)memset(&candidate, 0, sizeof(candidate));

        if (!Hal_NvmRead((uint16_t)slot, &candidate, (uint16_t)sizeof(candidate)))
        {
            continue;
        }
        if (!RecordValid(&candidate))
        {
            continue;
        }

        if (!found || (candidate.sequence > s_sequence))
        {
            s_record = candidate;
            s_sequence = candidate.sequence;
            s_valid_slot = slot;
            found = true;
        }
    }

    if (!found)
    {
        Nvm_GetDefaults(params);
        return false;
    }

    (void)memcpy(params, s_record.payload, sizeof(NvmParams_t));
    return true;
}

bool Nvm_Save(const NvmParams_t *params)
{
    if (!s_initialised)
    {
        Nvm_Init();
    }
    if (params == NULL)
    {
        return false;
    }

    uint8_t target;
    if (s_valid_slot == 0U)
    {
        target = 1U;
    }
    else if (s_valid_slot == 1U)
    {
        target = 0U;
    }
    else
    {
        target = 0U;
    }

    (void)memset(&s_record, 0, sizeof(s_record));
    s_record.magic = NVM_MAGIC;
    s_record.version = NVM_VERSION;
    s_record.payload_len = (uint16_t)sizeof(NvmParams_t);
    s_record.sequence = s_sequence + 1U;
    (void)memcpy(s_record.payload, params, sizeof(NvmParams_t));
    s_record.crc = Crc16_Ccitt((const uint8_t *)&s_record, offsetof(NvmRecord_t, crc));

    if (!Hal_NvmWrite((uint16_t)target, &s_record, (uint16_t)sizeof(s_record)))
    {
        return false;
    }

    s_sequence = s_record.sequence;
    s_valid_slot = target;
    s_write_count++;
    return true;
}

uint32_t Nvm_GetSequence(void)
{
    return s_sequence;
}

uint8_t Nvm_GetValidSlot(void)
{
    return s_valid_slot;
}

uint32_t Nvm_GetWriteCount(void)
{
    return s_write_count;
}
