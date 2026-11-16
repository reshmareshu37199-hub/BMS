#ifndef BMS_SERVICES_NVM_H
#define BMS_SERVICES_NVM_H

#include <stdbool.h>
#include <stdint.h>

#define NVM_MAGIC 0x424D534EU /* 'BMSN' */
#define NVM_VERSION 1U
#define NVM_PAYLOAD_MAX 64U
#define NVM_SLOT_COUNT 2U
#define NVM_DEFAULT_BAL_START_MV 15U
#define NVM_DEFAULT_BAL_STOP_MV 5U
#define NVM_DEFAULT_SOC_PERMILLE 500

typedef struct
{
    uint16_t bal_start_mv;
    uint16_t bal_stop_mv;
    int16_t soc_permille;
    uint16_t flags;
} NvmParams_t;

/** Reset the in-RAM view.  Does not erase flash. */
void Nvm_Init(void);

void Nvm_GetDefaults(NvmParams_t *params);

/**
 * Load the newest valid record from the A/B slots.
 *
 * Each record carries a magic, format version, sequence number and CRC-16.
 * If both slots are corrupt or blank the defaults are returned and the
 * function returns false.  This is the "default fallback" of the B2 design.
 */
bool Nvm_Load(NvmParams_t *params);

/**
 * Write the parameters to the older slot with sequence + 1.
 *
 * Alternating between the two slots (A/B) doubles the effective endurance and
 * guarantees that a power loss during the write cannot destroy the previous
 * valid record.  A real data-flash driver would add more slots and a
 * round-robin erase counter; the algorithm is the same.
 */
bool Nvm_Save(const NvmParams_t *params);

uint32_t Nvm_GetSequence(void);
uint8_t Nvm_GetValidSlot(void); /* 0, 1, or 0xFF when no valid record exists */
uint32_t Nvm_GetWriteCount(void);

#endif /* BMS_SERVICES_NVM_H */
