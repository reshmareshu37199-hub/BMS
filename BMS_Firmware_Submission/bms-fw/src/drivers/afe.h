#ifndef BMS_DRIVERS_AFE_H
#define BMS_DRIVERS_AFE_H

#include <stdbool.h>
#include <stdint.h>

#define AFE_CELL_COUNT 16U
#define AFE_TEMP_COUNT 5U
#define AFE_DEVICE_ID_VALUE 0x5AU

#define AFE_CONVERSION_TIMEOUT_MS 10U
#define AFE_MAX_RETRIES 2U
#define AFE_MAX_FAILED_CYCLES 3U
#define AFE_CELL_SUM_TOLERANCE_MV 1000U

typedef struct
{
    uint16_t cell_mv[AFE_CELL_COUNT];
    int16_t temp_dc[AFE_TEMP_COUNT];
    uint32_t sequence;
    bool valid;
    bool sum_mismatch;
    bool internal_fault;
    bool open_wire;
} AfeSnapshot_t;

/**
 * Initialise the driver and start the device-ID self test.  The first
 * Afe_Poll() calls perform the ID read.
 */
void Afe_Init(void);

/** Request a new conversion/read cycle.  Ignored if a cycle is already busy. */
void Afe_StartCycle(uint32_t now_ms);

/**
 * Advance the non-blocking acquisition state machine by one bounded step.
 * Returns true when a new snapshot has been published.
 */
bool Afe_Poll(uint32_t now_ms);

/** Copy the most recent snapshot atomically. */
bool Afe_GetSnapshot(AfeSnapshot_t *out);

bool Afe_IsValid(void);
uint8_t Afe_GetFailedCycles(void);
bool Afe_HasCommFault(void);
void Afe_ClearLatchedFault(void);
bool Afe_IsBusy(void);

uint16_t Afe_GetBalMask(void);
bool Afe_SetBalMask(uint16_t mask);

#endif /* BMS_DRIVERS_AFE_H */
