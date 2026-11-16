# Code Review — Task 7

**Reviewer:** candidate
**Subject:** three listings supplied in the assignment brief — `Balancing_Run`,
`CAN_TxPackStatus`, `AFE_SPI_IRQHandler`
**Verdict:** **changes required — do not merge as-is.** Nineteen findings are safety-relevant
(critical), seventeen are correctness or requirement violations (major), and the remaining
eleven are maintainability. Corrected implementations are in `src/services/balancing.c`,
`src/services/can_matrix.c`, `src/drivers/afe.c` and `src/services/can_signal.c`; the review
below shows the corrected shape of each function.

---

## 0. Method and severity scale

I reviewed each listing against five questions, in this order:

1. **Can it hurt the pack?** Undefined behaviour, out-of-bounds access, contactor action,
   loss of a safety gate.
2. **Can it hang or miss a deadline?** Unbounded loops, blocking calls, work in an ISR.
3. **Is the data coherent?** Multi-byte values shared with an ISR without a snapshot or
   critical section.
4. **Does it meet the wire/functional contract?** Scaling, endianness, CRC, alive counter,
   DLC, DBC agreement.
5. **Can it be tested and maintained?** Layering, magic numbers, error propagation, naming.

Severity:

| Severity | Meaning | Merge rule |
|---|---|---|
| **Critical** | Can cause an unsafe pack state, undefined behaviour, a hang, or silent loss of a protection function | Must fix before any test on hardware |
| **Major** | Wrong data or a violated requirement; the system runs but is not correct | Must fix before merge |
| **Minor** | Portability, maintainability, duplicated knowledge, missing error path | Fix in the same change set or raise a ticket |
| **Nit** | Naming, comment, formatting | Optional |

The listings are reproduced with the identifiers used in the brief. Findings are keyed to
the code structure (loop, field, call) rather than line numbers so they stay valid if the
listing is reformatted.

---

## 1. Listing 1 — `Balancing_Run`

```c
/* balancing.c — as received */
#include "afe.h"

float cellVoltage[16];
uint8_t balancingMask;
uint32_t balanceStartTime;

void Balancing_Run(void)
{
    uint8_t i, j;
    float vmin = 5.0f;
    float vmax = 0.0f;

    /* Wait for the AFE conversion to finish. */
    while (!AFE_ConversionDone())
    {
    }

    for (i = 0; i <= 16; i++)
    {
        cellVoltage[i] = AFE_ReadCell(i) * 0.0001f;
        if (cellVoltage[i] < vmin) vmin = cellVoltage[i];
        if (cellVoltage[i] > vmax) vmax = cellVoltage[i];
    }

    if (vmax < 3.4f)
    {
        balancingMask = 0;
        return;
    }

    balancingMask = 0;
    for (i = 0; i < 16; i++)
    {
        if ((cellVoltage[i] - vmin) > 0.010f)
        {
            balancingMask |= (1 << i);
        }
    }

    /* Never balance two adjacent cells. */
    for (j = 0; j < 16; j++)
    {
        if ((balancingMask & (1 << j)) &&
            (balancingMask & (1 << (j + 1))))
        {
            balancingMask &= ~(1 << (j + 1));
        }
    }

    AFE_WriteBalancing(balancingMask);
    balanceStartTime = HAL_GetTick();
}
```

### 1.1 Findings

| ID | Where | Severity | Finding | Why it matters |
|---|---|---|---|---|
| B-01 | `for (i = 0; i <= 16; i++)` | **Critical** | Off-by-one: writes `cellVoltage[16]` and calls `AFE_ReadCell(16)` on a 16-cell pack. | Out-of-bounds write corrupts the adjacent object; the AFE read is an invalid register access. Undefined behaviour (MISRA C:2012 Rule 18.1). |
| B-02 | `cellVoltage[i] = AFE_ReadCell(i) * ...` | **Critical** | The return value of `AFE_ReadCell` is never checked. A failed/CRC-rejected read becomes `0`, which becomes `vmin`, so every other cell looks 3.3 V high and is bled. | A single SPI error can bleed 15 cells continuously. This is a pack-damage path, not a cosmetic bug. |
| B-03 | `while (!AFE_ConversionDone()) { }` | **Critical** | Busy-wait with no timeout. | If the AFE is absent, wedged, or its interrupt is lost, the task never returns. The 100 ms task and everything below it stops; the watchdog resets the MCU in a loop. The assignment explicitly forbids blocking on conversion. |
| B-04 | Whole function | **Critical** | No inhibit gates: no measurement-window pause, no temperature check, no active-fault check, no VCU enable, no discharge inhibit. | Balancing during the AFE measurement window corrupts the very voltage it uses; balancing a hot board or a faulted pack is a thermal/abuse path. |
| B-05 | Adjacency loop, `1 << (j + 1)` | **Critical** | When `j == 15`, the shift is by 16. On a 16-bit `int` this is undefined; on 32-bit it produces a value outside the 16-bit mask, and `~(1 << 16)` is then truncated. `1 << 15` in the selection loop has the same portability problem. | Undefined/implementation-defined behaviour; the adjacency rule can be silently bypassed. Use `(uint16_t)1U << i` on an unsigned type. |
| B-06 | Selection loop | **Major** | No maximum-cell limit. | Up to 16 bleed resistors can be on at once. The board thermal limit is 8; this violates the requirement and can overheat the PCB. |
| B-07 | `if ((cellVoltage[i] - vmin) > 0.010f)` | **Major** | No hysteresis. A cell sitting at 10 mV ± noise toggles every cycle. | Contactor-like chatter on the bleed switch; unnecessary thermal cycling and measurement disturbance. The requirement is 15 mV start / 5 mV stop. |
| B-08 | Whole function | **Major** | No continuous-balancing timeout. | The 8 h limit and warning are mandatory. A stuck high cell would bleed indefinitely. |
| B-09 | Adjacency loop | **Major** | Deterministic lowest-index removal starves the same cell every cycle. | A cell that is always the second of an adjacent pair never balances, so the pack divergence persists. The requirement asks how starvation is prevented. |
| B-10 | `float` arithmetic and `0.0001f` conversion | **Major** | Float in a periodic embedded task, with binary floating-point equality/ordering. | Pulls in the soft-float library on a Cortex-M4 without FPU, makes WCET data-dependent, and loses the exact integer millivolt domain the AFE and DBC use. |
| B-11 | `AFE_WriteBalancing(balancingMask)` | **Major** | Return value ignored; mask is written on every call, even when unchanged, and while the AFE may be converting. | A failed write is invisible; repeated writes waste SPI bandwidth and can corrupt an acquisition. |
| B-12 | Globals `cellVoltage`, `balancingMask`, `balanceStartTime` | **Minor** | External linkage, writable by every module; no initialisation/validation. | Any other translation unit can change the balancing state. `balanceStartTime` is written and never read — dead state. |
| B-13 | Magic numbers `3.4f`, `0.010f`, `16`, `5.0f` | **Minor** | No named constants, no configuration validation. | The thresholds are calibratable and must be range-checked. A typo in one literal is invisible in review. |
| B-14 | `void Balancing_Run(void)` | **Minor** | No status output: mask, active count, inhibit reason and timeout are not observable. | The application, CAN and diagnostics cannot report why balancing is off. Testability is poor: the function can only be checked through its side effects. |
| B-15 | No `static` state / no re-entrancy note | **Minor** | The function reads and writes globals that an ISR could also touch. | If the mask is applied from an interrupt (see Listing 3), the update is not atomic. |

### 1.2 What the author did well

* The intent is readable: find `Vmin`, bleed cells above it, avoid adjacent pairs.
* The top-of-charge gate (`3.4 V`) shows awareness that LFP mid-range voltage is flat.
* The adjacency rule is present at all, which is more than a naive implementation.

The problem is not the idea; it is that every safety-relevant constraint is missing and the
implementation has two undefined-behaviour paths.

### 1.3 Corrected implementation

The production version separates **inhibit policy**, **candidate selection** and **mask
application**. The key points, in the order they must happen:

1. Refuse to run on invalid measurements, an active fault, a VCU disable, over-temperature,
   discharge, or the AFE measurement window.
2. Validate calibratable data once, not on every cycle.
3. Use integer millivolts and a `uint16_t` mask.
4. Apply 15 mV start / 5 mV stop hysteresis.
5. Sort by voltage, cap at 8 cells, reject adjacent pairs.
6. Remember the first cell that was dropped and give it priority next cycle inside a 20 mV
   fairness band.
7. Accumulate active time and stop at 8 h; clear the warning on a charge cycle.
8. Publish a status record so the mask, inhibit reason and timeout are observable.

Abridged from `src/services/balancing.c` (full file is 352 lines and covered by 17 tests):

```c
/* Called every 100 ms. Returns the 16-bit bleed mask, bit 0 = cell 1. */
uint16_t Bal_Run(const BalInput_t *in)
{
    if (in == NULL)
    {
        s_status.mask = 0U;
        s_status.active = false;
        s_status.active_count = 0U;
        s_status.inhibit = BAL_INHIBIT_MEAS_INVALID;
        return 0U;
    }

    s_status.cycles++;

    /* A real charge current ends the continuous-balancing window. */
    if (in->pack_current_mA > 500)
    {
        Bal_NotifyChargeCycle();
    }

    BalInhibit_t inhibit = EvaluateInhibit(in);
    if (inhibit != BAL_INHIBIT_NONE)
    {
        if (inhibit != BAL_INHIBIT_MEAS_PAUSE)
        {
            s_status.active_time_ms = 0U;
        }
        s_status.inhibit = inhibit;
        s_status.mask = 0U;
        s_status.active = false;
        s_status.active_count = 0U;
        return 0U;
    }

    uint16_t vmin = 0xFFFFU;
    uint16_t vmax = 0U;
    for (uint8_t i = 0U; i < BAL_CELL_COUNT; ++i)
    {
        if (in->cell_mv[i] < vmin) { vmin = in->cell_mv[i]; }
        if (in->cell_mv[i] > vmax) { vmax = in->cell_mv[i]; }
    }

    uint8_t candidates[BAL_CELL_COUNT];
    uint8_t candidate_count = 0U;

    if (vmax >= s_cfg.vmin_mv)                  /* 3400 mV top-of-charge gate */
    {
        for (uint8_t i = 0U; i < BAL_CELL_COUNT; ++i)
        {
            uint16_t delta = (uint16_t)(in->cell_mv[i] - vmin);
            bool was_on = ((s_status.mask & (uint16_t)(1U << i)) != 0U);
            bool qualifies = was_on ? (delta >= s_cfg.stop_mv)   /*  5 mV */
                                    : (delta >  s_cfg.start_mv); /* 15 mV */
            if (qualifies)
            {
                candidates[candidate_count++] = i;
            }
        }
    }

    SortCandidates(candidates, candidate_count, in, vmax);

    uint16_t new_mask = 0U;
    uint8_t selected = 0U;
    uint8_t first_skipped = 0U;

    for (uint8_t i = 0U; i < candidate_count; ++i)
    {
        uint8_t cell = candidates[i];
        uint16_t bit = (uint16_t)(1U << cell);

        if (selected >= s_cfg.max_cells)
        {
            if (first_skipped == 0U) { first_skipped = (uint8_t)(cell + 1U); }
            continue;
        }

        uint16_t trial = (uint16_t)(new_mask | bit);
        if (Adjacent(trial))
        {
            if (first_skipped == 0U) { first_skipped = (uint8_t)(cell + 1U); }
            continue;
        }

        new_mask = trial;
        selected++;
    }

    s_priority_cell = first_skipped;   /* starvation fix for the next cycle */

    s_status.mask = new_mask;
    s_status.active = (new_mask != 0U);
    s_status.active_count = PopCount16(new_mask);
    s_status.inhibit = BAL_INHIBIT_NONE;

    if (new_mask != 0U)
    {
        uint32_t elapsed = s_status.active_time_ms + s_cfg.cycle_ms;
        if (elapsed >= s_cfg.timeout_ms)
        {
            s_status.active_time_ms = s_cfg.timeout_ms;
            s_status.timeout_warning = true;
            s_status.mask = 0U;
            s_status.active = false;
            s_status.active_count = 0U;
            s_status.inhibit = BAL_INHIBIT_TIMEOUT;
            return 0U;
        }
        s_status.active_time_ms = elapsed;
    }
    else
    {
        s_status.active_time_ms = 0U;
    }

    return new_mask;
}
```

`EvaluateInhibit()` encodes the gate order explicitly, and `Bal_Init()` repairs an invalid
configuration instead of trusting it. The measurement-window pause is driven from the
application (`Bal_SetMeasurementWindow`), because only the acquisition code knows when the
bleed switches must be off.

**Verification:** `test/test_balancing.c` covers S1–S10 plus validation, pause, VCU disable,
max-cell exhaustion, adjacency-after-limit, priority selection and sort ties. Coverage is
100.00 % lines / 96.08 % branches taken (`docs/coverage.txt`).

---

## 2. Listing 2 — `CAN_TxPackStatus`

```c
/* can_tx.c — as received */
typedef struct
{
    uint16_t packVoltage;   /* mV */
    int16_t  packCurrent;   /* A  */
    uint8_t  soc;           /* %  */
    uint8_t  state;
} PackStatus_t;

volatile uint8_t aliveCounter;

void CAN_TxPackStatus(void)
{
    uint8_t data[8];
    PackStatus_t status;

    status = *(PackStatus_t *)CAN_GetStatusBuffer();

    data[0] = status.packVoltage & 0xFF;
    data[1] = status.packVoltage >> 8;
    data[2] = status.packCurrent & 0xFF;
    data[3] = status.packCurrent >> 8;
    data[4] = status.soc;
    data[5] = status.state;
    data[6] = aliveCounter++;
    data[7] = 0;

    while (CAN_Transmit(CAN_ID_BMS_STATUS, data, 8) != CAN_OK)
    {
    }
}
```

### 2.1 Findings

| ID | Where | Severity | Finding | Why it matters |
|---|---|---|---|---|
| C-01 | `data[0..1]` and `data[2..3]` | **Critical** | Scaling disagrees with the DBC. The DBC defines `PackVoltage` as 0.01 V/bit (raw = mV / 10) and `PackCurrent` as 0.1 A/bit (raw = mA / 100). The code writes millivolts and amps raw, a 10× error on both. | The VCU reads 480.0 V on a 48 V pack and a 100 A limit as 10 A. Every downstream control decision is wrong. |
| C-02 | `data[7] = 0` | **Critical** | No CRC, although the DBC and the receiver require CRC-8 SAE J1850 over bytes 0–6 in byte 7. | Every frame fails the receiver's CRC check, or a receiver without a check consumes corrupt data. Safety-relevant data is unprotected. |
| C-03 | `data[6] = aliveCounter++` | **Critical** | Layout mismatch: the DBC puts the 4-bit alive counter at bits 44–47 (byte 5, high nibble), and byte 7 is CRC. The code puts an 8-bit counter in byte 6 and leaves byte 7 as zero. | A DBC decoder sees a stale/zero counter and a bogus CRC. The liveness supervision that protects against a stuck transmitter is lost. |
| C-04 | `while (CAN_Transmit(...) != CAN_OK) { }` | **Critical** | Blocking transmit with no timeout. | A full mailbox or a bus-off condition hangs the task. The 10 ms protection task stops updating; the watchdog resets the ECU. This is exactly the failure the "bounded execution" requirement forbids. |
| C-05 | `*(PackStatus_t *)CAN_GetStatusBuffer()` | **Critical** | Type-punning a byte buffer to a struct assumes layout, packing, alignment and endianness. | Strict-aliasing undefined behaviour; an unaligned access can hard-fault on a Cortex-M0; a compiler is free to reorder the reads. |
| C-06 | Same copy | **Critical** | No coherent snapshot. The source is a live buffer written by the control/ISR context. | Voltage, current, SOC and state can come from different cycles (torn read), so the VCU can see a combination that never existed. |
| C-07 | `aliveCounter++` | **Major** | `volatile` does not make read-modify-write atomic. If the counter is also observed or advanced elsewhere, an update is lost. It wraps at 255, not 15. | The receiver sees repeated or skipped counters and treats good frames as invalid. |
| C-08 | `status.packCurrent & 0xFF`, `>> 8` | **Major** | Right-shifting a negative signed value is implementation-defined. | Portability defect; on a compiler that shifts logically the sign extension is lost. Cast to `uint16_t` before shifting. |
| C-09 | `status.soc` (uint8 percent) | **Major** | SOC is sent as whole percent while the DBC defines 0.5 %/bit. | Loses half the resolution and disagrees with the receiver's scale; 100 % becomes 50 % after decoding. |
| C-10 | All fields | **Major** | No saturation before packing. | A value outside the signal range wraps (e.g. a negative current becomes a large positive current). The requirement is saturation, not wrap. |
| C-11 | `CAN_Transmit` return value | **Major** | Ignored; no queue, no drop counter, no retry policy. | A transient full mailbox silently loses the frame with no telemetry. There is no way to prove the 100 ms period was met. |
| C-12 | `CAN_GetStatusBuffer()` | **Minor** | Returns a raw pointer across a module boundary. | Breaks the layering rule and makes the producer's representation part of the CAN module's contract. |
| C-13 | No DLC/pointer checks | **Minor** | No guard on the buffer or length. | The function is not safe to call from a test or a second caller. |
| C-14 | `aliveCounter` global | **Minor** | External linkage and no single owner. | Any module can reset it; the receiver's jump check can be defeated. |
| C-15 | Magic ID, length, offsets | **Nit** | No named signal descriptors. | Every message re-implements bit placement, which is how sign/scale bugs get copied. |

### 2.2 What the author did well

* The byte order chosen for the multi-byte fields is little-endian, which is correct for
  Intel signals — the mistake is the scale and the missing CRC, not the byte order itself.
* The function is short and the field order matches the DBC's byte order, which makes the
  defect easy to spot.
* Using a named CAN ID constant rather than a literal is good practice.

### 2.3 Corrected implementation

The corrected path has three layers:

1. **A signal descriptor** owns start bit, length, signedness, scale, offset and physical
   range, so the wire format is defined once.
2. **A packer** saturates, converts with a 64-bit intermediate, and writes bits across byte
   boundaries without type-punning.
3. **A transmitter** takes a coherent snapshot, packs, queues without blocking and increments
   the 4-bit alive counter only when the frame is accepted.

Signal descriptors (`src/services/can_matrix.c`):

```c
#define SIG(start, len, sig_type, num, den, off, lo, hi) \
    { (uint8_t)(start), (uint8_t)(len), sig_type, num, den, off, lo, hi }

/* 0x350: voltage 0.01 V/bit, current 0.1 A/bit, SOC 0.5 %/bit. */
static const CanSignalDef_t S_PACK_VOLTAGE = SIG(0U, 16U, CAN_SIG_UNSIGNED, 10, 1, 0, 0, 655350);
static const CanSignalDef_t S_PACK_CURRENT = SIG(16U, 16U, CAN_SIG_SIGNED, 100, 1, 0, -3276800, 3276700);
static const CanSignalDef_t S_SOC          = SIG(32U,  8U, CAN_SIG_UNSIGNED, 5, 1, 0, 0, 1000);
static const CanSignalDef_t S_BMS_STATE    = SIG(40U,  4U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 15);
static const CanSignalDef_t S_ALIVE        = SIG(44U,  4U, CAN_SIG_UNSIGNED, 1, 1, 0, 0, 15);
```

Scaling core (`src/services/can_signal.c`). Note the saturation **before** scaling, the
64-bit intermediate, and the explicit two's-complement conversion:

```c
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

    if (raw64 < min_raw) { raw64 = min_raw; }
    if (raw64 > max_raw) { raw64 = max_raw; }

    return (uint64_t)raw64 & BitMask(sig->length);
}
```

The message packer (`src/services/can_matrix.c`) clears the frame, packs every field through
the descriptor, and computes the CRC last:

```c
void CanMatrix_PackPackStatus(uint8_t data[CAN_DLC],
                              const CanPackStatus_t *status,
                              uint8_t alive)
{
    if ((data == NULL) || (status == NULL))
    {
        return;
    }
    ClearFrame(data);

    (void)CanSig_Pack(data, CAN_DLC, &S_PACK_VOLTAGE, (int32_t)status->pack_voltage_mv);
    (void)CanSig_Pack(data, CAN_DLC, &S_PACK_CURRENT, status->pack_current_ma);
    (void)CanSig_Pack(data, CAN_DLC, &S_SOC,          (int32_t)status->soc_permille);
    (void)CanSig_Pack(data, CAN_DLC, &S_BMS_STATE,    (int32_t)status->bms_state);
    (void)CanSig_Pack(data, CAN_DLC, &S_ALIVE,        (int32_t)(alive & 0x0FU));

    data[7] = Crc8_SaeJ1850(data, 7U);
}
```

The caller (`src/app/app.c`) takes the coherent application snapshot, queues without
blocking, and advances the counter only on success:

```c
static void SendPackStatus(void)
{
    AppSnapshot_t snap;
    App_GetSnapshot(&snap);                 /* critical-section copy */

    CanPackStatus_t status;
    (void)memset(&status, 0, sizeof(status));
    status.pack_voltage_mv = snap.pack_voltage_mv;
    status.pack_current_ma = snap.pack_current_ma;
    status.soc_permille    = snap.soc_permille;
    status.bms_state       = (uint8_t)snap.state;

    uint8_t data[CAN_DLC];
    CanMatrix_PackPackStatus(data, &status, s_alive_350);

    if (CanDrv_Queue(CAN_ID_BMS_PACK_STATUS, data, CAN_DLC))
    {
        s_alive_350 = (uint8_t)((s_alive_350 + 1U) & 0x0FU);
    }
}
```

`CanDrv_Queue()` copies the frame into a fixed 8-entry software queue and returns `false`
when full; `CanDrv_Poll()` drains at most four frames per 1 ms tick and counts drops. There
is no blocking call anywhere on the transmit path.

**Verification:** `test/test_can.c` covers pack/unpack round trips, saturation, CRC rejection,
alive-counter repeat/jump rejection, timeout and queue overflow. `dbc/bms.dbc` is the single
external contract for scale and layout.

---

## 3. Listing 3 — `AFE_SPI_IRQHandler`

```c
/* afe_irq.c — as received */
volatile uint16_t cellRaw[16];
volatile uint16_t tempRaw[5];
volatile uint8_t  spiBuffer[64];
volatile uint8_t  spiPos;
volatile bool     afeReady;
uint16_t          balanceMask;

void AFE_SPI_IRQHandler(void)
{
    uint8_t i;
    float temperature;

    if (SPI1->SR & SPI_SR_RXNE)
    {
        spiBuffer[spiPos++] = SPI1->DR;

        if (spiPos == 64)
        {
            for (i = 0; i < 16; i++)
            {
                cellRaw[i] = (spiBuffer[2 * i] << 8) | spiBuffer[2 * i + 1];
                if (cellRaw[i] > 3650)
                {
                    OpenContactors();
                    SetDtc(DTC_CELL_OV);
                }
            }

            for (i = 0; i < 5; i++)
            {
                tempRaw[i] = (spiBuffer[32 + 2 * i] << 8) |
                              spiBuffer[32 + 2 * i + 1];
                temperature = 1.0f / (1.0f / 298.15f +
                    log((float)tempRaw[i] / 4096.0f) / 3435.0f) - 273.15f;
                if (temperature > 60.0f)
                {
                    OpenContactors();
                    SetDtc(DTC_DISCHARGE_OT);
                }
            }

            AFE_WriteBalancing(balanceMask);
            CAN_SendStatus();
            afeReady = true;
            spiPos = 0;
        }
    }

    SPI1->SR = 0;
    NVIC_ClearPendingIRQ(SPI1_IRQn);
}
```

### 3.1 Findings

| ID | Where | Severity | Finding | Why it matters |
|---|---|---|---|---|
| A-01 | Whole ISR | **Critical** | The ISR parses 64 bytes, converts 16 cells and 5 temperatures, calls `log()`, opens contactors, sets DTCs, writes the balancing register and sends CAN. | ISR latency is unbounded and data-dependent. At 1 kHz SPI with a 64-byte frame, this can consume milliseconds, delaying the tick, CAN and every lower-priority interrupt. Violates "short ISR" and the 1 ms budget. |
| A-02 | `AFE_WriteBalancing(...)` inside the SPI ISR | **Critical** | A blocking SPI write is issued from inside the SPI interrupt. | If the driver waits for TXE/RXNE or a completion flag, it deadlocks against its own interrupt. If it is DMA-based, it still re-enters a bus that the ISR is servicing, with no ownership protocol. |
| A-03 | `log(...)` and `float` in the ISR | **Critical** | `log()` is not reentrant, may use a shared errno/FP context, and has unbounded execution time. The Beta formula itself is wrong (the resistance ratio and the divider are missing), so the result can be NaN or meaningless. | Floating point in an ISR forces context save on an FPU-equipped core and a library call otherwise; a non-reentrant libm call can corrupt the main-context result. The computed temperature is not even stored. |
| A-04 | `SPI1->SR = 0;` | **Critical** | Writing 0 to a status register does not clear the RXNE flag. RXNE is cleared by reading `DR`; error flags are usually write-1-to-clear. | The interrupt condition remains asserted, so the handler re-enters forever (interrupt storm) or the real error flag is never cleared. The system appears hung. |
| A-05 | `NVIC_ClearPendingIRQ(SPI1_IRQn)` | **Critical** | Clears the NVIC pending bit, not the peripheral source. | The peripheral immediately re-asserts the request. This masks the symptom for one instruction and then loops; it also hides a real overrun. |
| A-06 | `spiBuffer[spiPos++]` | **Critical** | No bound check before the increment. A 65th byte writes past the array. | Memory corruption in interrupt context; can overwrite the stack or the adjacent globals. MISRA C:2012 Rule 18.1. |
| A-07 | Fixed offsets `2*i`, `32 + 2*i`, length 64 | **Critical** | No header, length or CRC validation. The frame layout is assumed, not checked. | A short frame, a shifted frame or a corrupt frame is accepted as valid. A single bit error becomes a false cell voltage or a false over-temperature. The AFE contract requires a CRC check. |
| A-08 | Contactors and DTCs in the ISR | **Critical** | Opens contactors and latches DTCs directly from an interrupt, with no debounce, no 1 s current-reduction policy, no state-machine coordination. | Bypasses the protection table and the state machine; a single noisy sample opens the contactors under load (arc, welded contactor). Two writers to the contactor state race with the 10 ms task. |
| A-09 | `cellRaw` / `tempRaw` / `afeReady` | **Major** | `volatile` gives no atomic multi-word snapshot. The main loop can read cells from cycle N and temperatures from cycle N−1; the compiler can still reorder the flag update relative to the data. | Torn measurement set: protection may compare a new cell voltage with an old temperature. A `volatile` flag is not a memory barrier. |
| A-10 | `balanceMask` | **Major** | Written by the main task and read by the ISR with no critical section. | The applied mask can be half old, half new. With adjacent channels this can enable an adjacent pair, violating the thermal rule. |
| A-11 | No error handling | **Major** | OVR, MODF, CRC error and conversion timeout are not checked or counted. | A single glitch leaves the driver wedged; there is no retry, no failed-cycle counter, and no path to a latched communication fault. |
| A-12 | No completion timeout | **Major** | If fewer than 64 bytes arrive, `afeReady` is never set and nothing recovers. | The application waits forever for a snapshot. There is no deadline and no diagnostic. |
| A-13 | Direct `SPI1->SR` / `SPI1->DR` access | **Major** | The driver bypasses the HAL. | The code cannot be built or tested on the PC, cannot be reused on a different MCU, and hard-codes register addresses in application logic. |
| A-14 | `CAN_SendStatus()` in the ISR | **Major** | The CAN stack may block, take a lock or allocate a mailbox. | ISR-to-task priority inversion and unbounded latency; the CAN send should be queued and drained from the 1 ms task. |
| A-15 | `tempRaw[i]` computed then discarded | **Minor** | Dead computation; no temperature output. | Wasted cycles and a misleading review surface. |
| A-16 | No priority configuration shown | **Minor** | The SPI IRQ priority relative to CAN/tick is undefined. | A high-rate SPI interrupt can starve CAN; a low-priority one can miss bytes. The priority must be explicit and documented. |
| A-17 | `afeReady` never cleared | **Minor** | The consumer has no acknowledge path. | Repeated processing of the same frame, or a missed update if the flag is cleared at the wrong time. |

### 3.2 What the author did well

* Reading `DR` in response to RXNE is the correct way to clear RXNE — the mistake is the
  subsequent writes to `SR` and the NVIC.
* The 16-bit big-endian reconstruction `(hi << 8) | lo` matches the AFE's register format.
* `volatile` on the ISR-shared objects shows the right instinct; the review point is that
  `volatile` is necessary but not sufficient for a coherent multi-field snapshot.

### 3.3 Corrected implementation

The corrected design has one rule: **the ISR moves bytes, the 1 ms task does everything
else.** On the PC build the HAL is polled, so the same state machine is driven by
`Afe_Poll()` with one bounded SPI transaction per tick. On target the ISR only captures the
byte and sets a flag; the state machine still runs in the 1 ms task, where it can be timed,
tested and preempted safely.

ISR glue (target HAL; not compiled in the PC build):

```c
static volatile uint8_t s_rx_byte;
static volatile bool    s_rx_ready;
static volatile bool    s_spi_error;

void AFE_SPI_IRQHandler(void)
{
    uint32_t sr = SPI1->SR;

    if ((sr & SPI_SR_RXNE) != 0U)
    {
        s_rx_byte = (uint8_t)SPI1->DR;   /* reading DR clears RXNE */
        s_rx_ready = true;
    }
    if ((sr & (SPI_SR_OVR | SPI_SR_MODF)) != 0U)
    {
        (void)SPI1->DR;                  /* clear OVR */
        s_spi_error = true;
    }
}
```

No parsing, no `log`, no contactor, no CAN, no DTC in the ISR. The handler is a few
instructions and has a constant upper bound.

Main-context acquisition state machine (`src/drivers/afe.c`). One SPI transaction per 1 ms
tick, with a conversion deadline and retry counter:

```c
bool Afe_Poll(uint32_t now_ms)
{
    uint8_t buffer[2] = {0};

    switch (s_state)
    {
    case AFE_ST_READ_ID:
        if (Afe_ReadRegisters(AFE_REG_DEVICE_ID, 1U, buffer) &&
            (buffer[0] == AFE_DEVICE_ID_VALUE))
        {
            s_id_attempts = 0U;
            s_state = AFE_ST_IDLE;
        }
        else
        {
            s_id_attempts++;
            if (s_id_attempts >= 3U)
            {
                s_comm_fault = true;
                s_id_failed = true;
                s_state = AFE_ST_IDLE;
            }
        }
        break;

    case AFE_ST_IDLE:
        if (s_cycle_requested && !s_id_failed)
        {
            s_state = AFE_ST_START_CONV;
        }
        break;

    case AFE_ST_START_CONV:
    {
        uint8_t ctrl = 0x01U;
        if (Afe_WriteRegisters(AFE_REG_ADC_CTRL, &ctrl, 1U))
        {
            s_deadline_ms = now_ms + AFE_CONVERSION_TIMEOUT_MS;
            s_state = AFE_ST_WAIT_DONE;
        }
        else
        {
            Afe_HandleTransactionError();
        }
        break;
    }

    case AFE_ST_WAIT_DONE:
        if (!Afe_ReadRegisters(AFE_REG_STATUS, 1U, buffer))
        {
            Afe_HandleTransactionError();
        }
        else
        {
            s_internal_fault = ((buffer[0] & AFE_STATUS_INTERNAL_FAULT) != 0U);
            s_open_wire     = ((buffer[0] & AFE_STATUS_OPEN_WIRE) != 0U);

            if ((buffer[0] & AFE_STATUS_CONV_DONE) != 0U)
            {
                s_step = 0U;
                s_state = AFE_ST_READ_CELLS;
            }
            else if (now_ms >= s_deadline_ms)
            {
                Afe_HandleTransactionError();
            }
        }
        break;

    case AFE_ST_READ_CELLS:
        if (!Afe_ReadRegisters(
                (uint16_t)(AFE_REG_CELL_BASE + ((uint16_t)s_step * 2U)), 2U, buffer))
        {
            Afe_HandleTransactionError();
        }
        else
        {
            s_cell_raw[s_step] =
                (uint16_t)(((uint16_t)buffer[0] << 8U) | (uint16_t)buffer[1]);
            s_step++;
            if (s_step >= AFE_CELL_COUNT)
            {
                s_step = 0U;
                s_state = AFE_ST_READ_TEMPS;
            }
        }
        break;

    /* AFE_ST_READ_TEMPS is the same pattern; AFE_ST_PUBLISH publishes a snapshot. */
    default:
        s_state = AFE_ST_IDLE;
        break;
    }

    return false;
}
```

Publication uses two buffers and a short critical section around the index swap. The producer
never runs inside the critical section; the consumer copies the active buffer under the same
short section:

```c
static void Afe_PublishSnapshot(void)
{
    uint8_t next = (uint8_t)(1U - s_active);
    AfeSnapshot_t *snap = &s_buffers[next];

    (void)memset(snap, 0, sizeof(*snap));
    uint32_t sum_mv = 0U;
    for (uint8_t i = 0U; i < AFE_CELL_COUNT; ++i)
    {
        snap->cell_mv[i] = (uint16_t)(s_cell_raw[i] / 10U);   /* 100 uV/LSB */
        sum_mv += (uint32_t)snap->cell_mv[i];
    }
    for (uint8_t i = 0U; i < AFE_TEMP_COUNT; ++i)
    {
        snap->temp_dc[i] = Ntc_RawToDeciC((uint16_t)(s_temp_raw[i] & 0x0FFFU));
    }

    uint16_t pack_mv = Hal_PackVoltageMv();
    uint32_t diff = (sum_mv > (uint32_t)pack_mv) ? (sum_mv - (uint32_t)pack_mv)
                                                 : ((uint32_t)pack_mv - sum_mv);
    snap->sum_mismatch  = (diff > AFE_CELL_SUM_TOLERANCE_MV);
    snap->internal_fault = s_internal_fault;
    snap->open_wire      = s_open_wire;
    snap->valid          = !s_internal_fault;
    snap->sequence       = ++s_sequence;

    Hal_CriticalEnter();
    s_active = next;
    Hal_CriticalExit();

    s_failed_cycles = 0U;
    s_retries = 0U;
    s_cycle_requested = false;
    s_state = AFE_ST_IDLE;
}

bool Afe_GetSnapshot(AfeSnapshot_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    Hal_CriticalEnter();
    *out = s_buffers[s_active];
    Hal_CriticalExit();
    return true;
}
```

Failed transactions go through a bounded retry counter; three consecutive failed cycles
latch the AFE communication fault and publish an invalid snapshot, which the protection layer
turns into `FAULT_AFE_COMM` and an open request.

**Verification:** `test/test_afe.c` covers device-ID self-test, wrong ID, successful
acquisition, balancing-mask write, cell-sum mismatch, internal fault/open wire, CRC retry and
latch, SPI timeout and API guards.

---

## 4. Cross-cutting review checklist

The same checklist applies to every file in the repository, not just the three listings:

| Area | Rule applied | Where it is demonstrated |
|---|---|---|
| Bounds | Every array index is bounded by a compile-time count; every public entry validates pointers and lengths | `Afe_ReadRegisters`, `CanSig_WriteRaw`, `Nvm_Load`, UDS/ISO-TP |
| Integer | Fixed-width types; no signed shift of negative values; saturation before narrowing | `saturate.h`, `CanSig_PhysToRaw` |
| Timing | No unbounded loop; one bounded transaction per tick; overrun counter | `afe.c`, `can_driver.c`, `scheduler.c` |
| Concurrency | ISR only captures/flags; double buffer + short critical section; no lock held across SPI | `afe.c`, `app.c` |
| Error handling | Every HAL call has a failure path; failures are counted and latched | `afe.c`, `nvm.c`, `can_driver.c` |
| Safety | Threat table-driven; hard thresholds independent of debounce; safe state is the default | `protection.c`, `bms_state.c` |
| Contract | Scaling/CRC/alive match the DBC and are tested both ways | `can_matrix.c`, `test_can.c`, `dbc/bms.dbc` |
| Testability | Logic is independent of registers; HAL is an injected ops table | `hal.c`, all service tests |
| Portability | No compiler-specific extensions; no dynamic allocation; no recursion | whole tree, `-Wpedantic` build |

---

## 5. Review summary

| Listing | Critical | Major | Minor | Nit | Total |
|---|---:|---:|---:|---:|---:|
| `Balancing_Run` | 5 | 6 | 4 | 0 | 15 |
| `CAN_TxPackStatus` | 6 | 5 | 3 | 1 | 15 |
| `AFE_SPI_IRQHandler` | 8 | 6 | 3 | 0 | 17 |
| **Total** | **19** | **17** | **10** | **1** | **47** |

The critical findings cluster into five root causes:

1. **No ownership boundary between ISR and task** — long ISR, shared arrays, no snapshot.
2. **No error path** — unchecked reads, ignored return values, no timeout.
3. **No safety policy in the algorithm** — balancing without gates, contactors opened on one
   sample.
4. **Wire contract treated as an implementation detail** — scaling, CRC and alive counter
   not derived from one source of truth.
5. **Undefined behaviour** — out-of-bounds index, 16-bit shift, type-punning, signed shift.

The corrected code fixes the root causes rather than the symptoms: a HAL boundary, a
non-blocking state machine, a double-buffered snapshot, a table-driven signal layer and a
protection table. That is why the same defects cannot reappear in another message or another
sensor without failing a test.

### Re-review acceptance criteria

* [ ] No ISR performs parsing, floating point, CRC, CAN, NVM or contactor action.
* [ ] Every shared multi-field object is published through a double buffer or a critical
      section; no `volatile` array is treated as atomic.
* [ ] Every loop has a static or input-derived upper bound; every blocking HAL call has a
      timeout.
* [ ] Every packed signal is generated from a descriptor whose scale is checked against the
      DBC; a decode test exists for each message.
* [ ] Every fault has set, debounce-reset, release and clear tests.
* [ ] `-Wall -Wextra -Werror -Wpedantic` passes, cppcheck reports no findings, and the
      balancing/protection branch coverage target is met.

All six criteria are met by the delivered tree; the evidence is in `docs/coverage.txt`,
`docs/static_analysis.txt`, `test/` and the `simulate` target.
