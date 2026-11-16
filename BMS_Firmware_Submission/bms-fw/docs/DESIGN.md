# BMS-FW-ASG-01 — Firmware Design Document

**Project:** 16S1P LFP battery management system, 48 V, 100 Ah
**Deliverable:** Task 1 (architecture, scheduling, state machine, data sharing, functional safety)
**Status:** implementation-complete; this document describes the code in `src/`.

---

## 1. Scope and platform assumptions

The firmware is a fixed-function embedded application with five periodic activities and no
dynamic workload. It is written in C11 with fixed-width integer types, static allocation only,
and no calls to `malloc`/`free` in application code. The PC build links the same application,
driver and service code against a simulated HAL (`src/hal/hal_sim.c`, `src/hal/afe_sim.c`), so
the code under test is the code that would run on the target.

Target assumptions used for the timing budget (recorded in `ASSUMPTIONS.md`):

| Item | Assumption |
|---|---|
| MCU | 32-bit Arm Cortex-M4 class, 64 MHz, no FPU use in hot paths |
| SPI | 1 MHz, one 3–5 byte transaction costs ≈ 30–50 µs including driver overhead |
| CAN | 500 kbit/s, classic 11-bit identifiers, 8 data bytes |
| Tick | 1 ms SysTick; the ISR only increments a volatile millisecond counter |
| NVM | 2 × 512-byte parameter slots; the PC HAL implements them as memory |

The host timings in this document are **estimates** derived from the operation counts in the
code, not measurements on silicon. They are deliberately conservative. The host build has a
larger pointer size than the target, so its measured stack usage is an upper bound.

---

## 2. Layered architecture

```
┌───────────────────────────────────────────────────────────────────────────┐
│ APPLICATION                                                               │
│   scheduler.c   bms_state.c   app.c                                      │
│   periodic execution, state machine, module orchestration, CAN/UDS glue   │
├───────────────────────────────────────────────────────────────────────────┤
│ SERVICES (pure logic unless noted)                                        │
│   balancing.c  protection.c  soc.c  can_signal.c  can_matrix.c           │
│   isotp.c      uds.c         nvm.c (HAL NVM)                             │
├───────────────────────────────────────────────────────────────────────────┤
│ DRIVERS                                                                   │
│   afe.c (SPI state machine)  can_driver.c (Tx queue / Rx dispatch)        │
│   ntc_table.c (raw ADC → 0.1 °C)                                          │
├───────────────────────────────────────────────────────────────────────────┤
│ HARDWARE ABSTRACTION LAYER                                                │
│   hal.h: time, SPI, contactors, pack/bus voltage, current, temperatures,  │
│           charger, CAN, critical sections, NVM, watchdog                  │
├──────────────────────────────┬────────────────────────────────────────────┤
│ TARGET IMPLEMENTATION        │ PC SIMULATION                              │
│ vendor HAL, real peripherals │ hal_sim.c + afe_sim.c (register model)      │
└──────────────────────────────┴────────────────────────────────────────────┘
```

**Dependency rule.** Arrows point downwards only. Services never include driver or application
headers; the state machine, balancing and protection are pure functions of their inputs.
`afe.c` and `can_driver.c` are the only drivers that call the HAL. This is what makes the
majority of the firmware testable on the host with no hardware and no mocking framework.

**Module interfaces (header-level).** Every module has a narrow API; no module reaches into
another module's state.

| Module | Header | Public interface (summary) |
|---|---|---|
| HAL | `hal/hal.h` | `Hal_Init`, `Hal_TimeMs`, `Hal_SpiTransfer`, contactor get/set, pack/bus/current/temp reads, CAN send/recv, critical enter/exit, NVM read/write, watchdog |
| AFE driver | `drivers/afe.h` | `Afe_Init`, `Afe_StartCycle`, `Afe_Poll`, `Afe_GetSnapshot`, `Afe_IsValid`, `Afe_GetFailedCycles`, `Afe_HasCommFault`, `Afe_ClearLatchedFault`, `Afe_Get/SetBalMask` |
| CAN driver | `drivers/can_driver.h` | `CanDrv_Init`, `CanDrv_Queue`, `CanDrv_Poll`, `CanDrv_RegisterHandler`, `CanDrv_GetStats` |
| NTC table | `drivers/ntc_table.h` | `Ntc_RawToDeciC` |
| Balancing | `services/balancing.h` | `Bal_Init`, `Bal_Run`, `Bal_GetStatus`, `Bal_GetConfig`, `Bal_SetStartStop`, `Bal_SetMeasurementWindow`, `Bal_NotifyChargeCycle` |
| Protection | `services/protection.h` | `Prot_Init`, `Prot_Update`, `Prot_GetOutput`, `Prot_IsFaultActive`, `Prot_GetDtcStatus`, `Prot_GetDtcCount`, `Prot_GetDtcByIndex`, `Prot_ClearDtc` |
| SOC | `services/soc.h` | `Soc_Init`, `Soc_Update`, `Soc_GetPermille`, `Soc_OcvToPermille`, `Soc_GetOffsetMa`, `Soc_SetPermille` |
| CAN signal layer | `services/can_signal.h` | generic pack/unpack with scaling, offset, saturation, CRC-8 SAE J1850 |
| CAN matrix | `services/can_matrix.h` | `CanMatrix_PackPackStatus`, `PackLimits`, `PackCellStats`, `PackCellVoltages`, `PackFaults`, `UnpackVcuCommand` |
| ISO-TP | `services/isotp.h` | `IsoTp_Init`, `IsoTp_Write`, `IsoTp_Read`, `IsoTp_Poll` |
| UDS | `services/uds.h` | `Uds_Init`, `Uds_Process`, `Uds_Poll`, session/S3 handling, hooks for DID and DTC access |
| NVM | `services/nvm.h` | `Nvm_Init`, `Nvm_Load`, `Nvm_Save`, `Nvm_GetDefaults`, status accessors |
| Scheduler | `app/scheduler.h` | `Sched_Init`, `Sched_Register`, `Sched_Run`, statistics |
| State machine | `app/bms_state.h` | `BmsState_Init`, `BmsState_Update`, `BmsState_GetOutput`, `BmsState_Name`, `BmsState_ContactorsClosed` |
| Application | `app/app.h` | `App_Init`, `App_Step`, `App_Run`, `App_GetSnapshot` |

---

## 3. Scheduling concept

### 3.1 Options considered

| Option | Strengths | Weaknesses for this system | Decision |
|---|---|---|---|
| Bare super-loop with software delays | Smallest code, no infrastructure | Blocking delays destroy jitter and make WCET unbounded; no evidence of timing | Rejected |
| Event-driven loop with flags | Flexible, low overhead | Hard to prove periodic deadlines; priority between events implicit | Rejected |
| **Timer-driven cooperative scheduler** | Deterministic order, bounded work per tick, trivial to test, no RTOS footprint, overrun observable | Tasks must be short and non-blocking; no preemption | **Chosen** |
| Preemptive RTOS (FreeRTOS, ThreadX) | Priorities, blocking primitives, mature | RAM/flash cost; priority inversion and shared-state analysis; five simple periodic jobs do not justify it; harder to reproduce on a PC host | Rejected |
| Hardware-timer ISR doing the work | Lowest latency for one event | Long ISRs block everything; not composable; unsafe for SPI/CAN stacks | Rejected |
| Time-triggered architecture with a static schedule table | Provable schedule | More machinery than needed; the same fixed order is already achieved by the cooperative table | Not needed |

The selected design is a **fixed cyclic executive**: a 1 ms tick drives the AFE state machine
and the CAN drain. The control cycles required by the brief are the **10 ms fast task**
(current, over-current, protection, state machine) and the **100 ms main task** (AFE cycle,
balancing, CAN Tx). Task execution order is registration order, which is the priority order.
The scheduler does not catch up after an overrun: it re-bases the deadline and increments
`Sched_GetOverruns()`. This bounds the work performed in any tick and turns a missed deadline
into an observable counter instead of a burst of late work.

### 3.2 Task table

| Order | Task | Period | Work performed | Estimated WCET (target) |
|---|---|---|---|---|
| 1 | `tick` | 1 ms | Advance AFE acquisition state machine by one SPI transaction; apply a new balancing mask when a snapshot is published; drain ≤4 CAN Tx frames and dispatch ≤8 Rx frames; kick watchdog | 80 µs |
| 2 | `fast` | 10 ms | Sample current and run over-current; copy AFE snapshot; poll VCU command monitor; update protection and DTCs; update state machine and contactors; SOC coulomb count; poll ISO-TP/UDS; publish application snapshot | 40 µs |
| 3 | `main` | 100 ms | Force balancing off, start AFE conversion, send 0x350/0x351, send 0x354 on change, persist parameters when due | 60 µs (see NVM note) |
| 4 | `cellstats` | 500 ms | Send 0x352 | 15 µs |
| 5 | `slow` | 1000 ms | Send 0x353 (6 mux frames) and 0x354 | 60 µs |

The AFE conversion itself is not a blocking wait. `Afe_Poll()` performs **one** SPI
transaction per 1 ms scheduler tick, so a complete 16-cell + 5-temperature acquisition spans
about 22 ms. The `main` task starts a conversion every 100 ms; the snapshot used by `fast` is
therefore always from the previous completed cycle and at most ~120 ms old. This satisfies the
"do not block the CPU while waiting for conversion" requirement.

### 3.3 CPU load and timing budget

Estimated steady-state load:

| Task | WCET / period | Load |
|---|---|---|
| tick | 80 µs / 1 ms | 8.0 % (upper bound) |
| fast | 40 µs / 10 ms | 0.40 % |
| main | 60 µs / 100 ms | 0.06 % |
| cellstats | 15 µs / 500 ms | 0.003 % |
| slow | 60 µs / 1000 ms | 0.006 % |
| **Total** | | **≈ 8.5 % upper bound; ≈ 1–2 % average** |

The 8.0 % charges 80 µs to every 1 ms tick. In reality the AFE is active for only about 22 ms
of each 100 ms cycle, so the average is much lower; the bound is used so the timing budget
cannot be argued downward.

Worst-case tick: all five entries become due in the same 1 ms tick. Sum ≈ 255 µs, still below
1 ms. The one long operation is an NVM save: the PC HAL is a memory copy, but a real data-flash
page erase + program can take tens of milliseconds. A synchronous call from the 100 ms task
would delay the 10 ms fast task and is therefore **not acceptable** for the over-current path.
The application isolates the operation behind `Hal_NvmWrite`; the target implementation must
be page-buffered/non-blocking, or the save must be deferred to STANDBY. This is recorded as a
known limitation in §13.

**Worst-case execution order (Big-O).**

* Scheduler: O(`SCHED_MAX_TASKS`) = O(8) per tick.
* Protection: O(`FAULT_COUNT` × 16) for the min/max scans = O(1) with fixed tables.
* Balancing: O(16) scan + O(n²) insertion sort with n ≤ 16 + O(16) selection = bounded,
  at most 120 comparisons. No recursion, no allocation.
* CAN packing: O(signal count) per message, ≤ 6 signals.
* UDS: bounded by `UDS_MAX_RESPONSE` (64 bytes); ISO-TP is a byte-wise state machine with a
  64-byte payload buffer.

**Stack.** `-fstack-usage` on the host build reports a maximum single frame of 400 bytes
(`Task_Fast`, including `ProtInput_t`, `BmsStateInput_t` and the snapshot). The deepest
synchronous chain (`Task_Fast → Uds_Process → HandleReadDid`) is below 1.2 kB on x86-64,
which is an upper bound for a 32-bit target. A 2 kB stack reserve leaves headroom. `Bal_Run`
uses 112 bytes total including its 16-byte candidate array; no variable-length arrays are
used anywhere.

---

## 4. BMS state machine

### 4.1 States and transitions

| State | Entry action | Exit action | Transitions |
|---|---|---|---|
| `INIT` | Contactors open | — | → `STANDBY` when AFE snapshot valid and ≥200 ms elapsed; → `FAULT` on AFE fault |
| `STANDBY` | Contactors open | — | → `PRECHARGE` on valid VCU close request and valid measurements; → `SLEEP` on sleep request; → `FAULT` on global fault |
| `PRECHARGE` | Negative contactor closes; precharge relay closes after 20 ms | Precharge relay opens on entry to `DRIVE`/`CHARGE` | → `STANDBY` if the VCU request disappears or the VCU times out; → `FAULT` on 2 s precharge timeout; → `DRIVE`/`CHARGE` after bus ≥95 % of pack + 20 ms settle |
| `DRIVE` | Negative and positive contactors closed | — | → `STANDBY` when close request is removed or VCU is lost for 1 s; → `FAULT` on global fault |
| `CHARGE` | Negative and positive contactors closed | — | → `STANDBY` on request removal/VCU loss; → `DRIVE` when the charger disappears; → `FAULT` on global fault |
| `FAULT` | Contactors held for up to 1 s, then opened | — | → `STANDBY` after 1 s when no fault is active |
| `SLEEP` | Contactors open | — | → `STANDBY` when the sleep request is removed and the VCU is valid |

### 4.2 Precharge sequence

The requirement is: close negative → close precharge relay → wait for bus ≥95 % of pack within
2 s → close positive → open precharge. The implementation uses four phases, evaluated every
10 ms:

| Phase | Duration / condition | Contactors |
|---|---|---|
| 0 | 20 ms settle after entering `PRECHARGE` | NEG on, PRE off, POS off |
| 1 | Transitional (one cycle) | NEG on, PRE on, POS off |
| 2 | Wait for `bus_mv ≥ 0.95 × pack_mv`; timeout at 2 s | NEG on, PRE on, POS off |
| 3 | 20 ms overlap before `DRIVE`/`CHARGE` | NEG on, PRE on, POS on |
| → `DRIVE`/`CHARGE` | `EnterState` resets the phase | NEG on, PRE off, POS on |

A precharge timeout produces a one-cycle pulse that is fed into the protection layer as
`FAULT_PRECHARGE`; that fault is latched and can be cleared through the normal DTC path once
the release condition is met.

### 4.3 Fault policy and contactor handling

Two independent signals leave the protection layer:

* `contactor_open_request` — the fault requires the contactors to open, but the BMS first
  gives the VCU a chance to reduce current. The state machine holds the contactors for 1 s
  after entering `FAULT`, then opens.
* `contactor_open_immediate` — set for over-current, hard over-voltage (>3750 mV), hard
  under-voltage (≤2300 mV) and precharge failure. These open on the same cycle.

This implements the requirement "never open under high current if it can be avoided" while
keeping the fast protection paths independent of the 1 s policy.

The VCU timeout is deliberately **excluded** from `fault_active`, so a lost VCU moves the
system to `STANDBY` (after the 1 s delay) rather than latching a hardware fault. The timeout
is still reported as a DTC.

---

## 5. Data sharing and concurrency

The application is a single-context cooperative system. All task bodies run in the main loop.
The only interrupt-context code on target is the 1 ms tick (increments a `volatile` counter)
and the CAN/SPI peripheral glue. The design rules are:

1. **AFE double buffer.** The AFE driver owns two `AfeSnapshot_t` buffers and a `volatile`
   active index. The producer fills the inactive buffer (`s_buffers[1 - s_active]`), then
   swaps `s_active` with interrupts disabled (`Hal_CriticalEnter/Exit`). Consumers copy the
   active buffer under the same critical section. There is no window in which a consumer can
   observe a half-written snapshot, and no lock is held while the SPI transfer runs.
2. **Critical sections are short.** `Hal_CriticalEnter/Exit` surrounds only the index swap and
   the snapshot copy (a few dozen bytes). No SPI, no CRC, no loops over 16 cells inside a
   critical section.
3. **`volatile` only where required.** `s_active` and the HAL millisecond counter are
   `volatile`. Arrays that are only touched in main context are not marked `volatile`, which
   avoids defeating compiler optimisation.
4. **CAN Rx is polled, not handled in the ISR.** `CanDrv_Poll()` drains the HAL receive queue
   in the 1 ms tick task and invokes handlers in main context. Handlers only update small state
   records (VCU command, ISO-TP byte stream); they do not call the scheduler or block.
5. **Tx queue.** The CAN driver has a fixed 8-entry software queue and sends at most four
   frames per poll. A full queue drops the newest frame and increments `tx_dropped`; periodic
   frames are retransmitted next cycle. The 100 ms task therefore never blocks on a mailbox.
6. **Application snapshot.** The logging/CAN snapshot is published under a critical section
   and read through `App_GetSnapshot()`, so the simulator and diagnostic code see a coherent
   set of state, voltages, limits and fault bits.

---

## 6. Functional safety mindset

### 6.1 Detected faults

| Fault | DTC | Set condition | Debounce | Release | Latch | Reaction |
|---|---|---|---|---|---|---|
| Cell over-voltage | 0x000A01 | any cell ≥3650 mV | 1 s | all cells ≤3400 mV | yes | CCL = 0; open at >3750 mV |
| Cell under-voltage | 0x000A02 | any cell ≤2500 mV | 1 s | all cells ≥2900 mV | yes | DCL = 0; open at ≤2300 mV |
| Charge over-temperature | 0x000A03 | Tmax ≥55 °C while charging | 2 s | Tmax ≤50 °C | no | CCL = 0 |
| Discharge over-temperature | 0x000A04 | Tmax ≥60 °C | 2 s | Tmax ≤55 °C | yes | open |
| Charge under-temperature | 0x000A05 | Tmin ≤0 °C while charging | 2 s | Tmin ≥3 °C | no | CCL = 0 |
| Discharge over-current | 0x000A06 | I ≤ −200 A | 500 ms | — | yes | open immediately |
| Charge over-current | 0x000A07 | I ≥ +100 A | 1 s | — | yes | open immediately |
| AFE communication | 0x000A08 | 3 consecutive failed cycles | — | good cycle | yes | open |
| VCU timeout | 0x000A09 | no valid 0x200 for 300 ms | — | valid frame | no | STANDBY; open after 1 s |
| Cell-sum mismatch | 0x000A0A | \|Σcells − pack\| > 1000 mV | — | in tolerance | no | CCL/DCL = 0 |
| Precharge failure | 0x000A0B | bus < 95 % after 2 s | — | — | yes | open immediately |

Each fault is stored with ISO 14229 status bits (`testFailed`, `testFailedThisCycle`,
`pending`, `confirmed`, `testFailedSinceClear`, `warning`). The DTC record is three bytes
plus the status byte, as reported by UDS 0x19 0x02.

### 6.2 Safe state

The safe state is: both contactors open, precharge relay open, CCL = DCL = 0, balancing mask
= 0, and a latched DTC recorded. `Prot_Update()` computes the safe state from a table and the
state machine applies the contactor outputs; no application code can bypass the table.

### 6.3 Freedom from interference

The usual ISO 26262 / IEC 62619 concerns are addressed structurally rather than with a full
safety case:

* **No dynamic memory** anywhere in application code; all tables and buffers are static.
* **Bounded loops** in every task; no `while` loop depends on external data or a timer.
* **Fixed-width types** and explicit saturation (`saturate.h`) for all scaled values.
* **CRC on every boundary**: CRC-8 on SPI frames, CRC-8 SAE J1850 on CAN frames that carry
  safety-relevant data, CRC-16 on NVM records.
* **Independent hard thresholds**: over-voltage, under-voltage and over-current bypass the
  debounce logic and request an immediate open, even if the normal fault object is still
  pending. A single bug in the debounce cannot delay a hard limit.
* **Watchdog** kicked only from the 1 ms tick task; a stuck scheduler stops the kick.
* **Overrun counter** turns missed deadlines into observable telemetry.
* **Defensive validation** of calibratable data (`Bal_Init`, `Bal_SetStartStop`, UDS range
  checks, NVM CRC/default fallback) so corrupt parameters cannot cause unsafe behaviour.
* **Latch semantics**: a latched fault clears only when both the clear request and the release
  condition are true; a clear request alone does not reset a present fault.

### 6.4 What is explicitly out of scope

A production safety case (FMEDA, diagnostic coverage metrics, secure boot chain, dual-channel
plausibility, lockstep core) is out of scope for a three-day assignment. The design document
states the mechanisms that would be extended, not a claim of ISO 26262 compliance.

---

## 7. Design decision log — alternatives considered and rejected

This is the section to use when explaining *why* the code looks the way it does.

| Area | Options | Chosen | Why the others were rejected |
|---|---|---|---|
| Scheduling | bare super-loop, event loop, cooperative timer table, preemptive RTOS, timer-ISR work | Cooperative timer-driven table: 1 ms quantum, 10 ms fast, 100 ms main | RTOS adds footprint, priority-inversion analysis and host-reproducibility problems for five fixed periodic jobs; blocking delays and ISR work destroy the timing evidence; the cooperative table gives deterministic order and an overrun counter |
| AFE acquisition | blocking wait for conversion done, interrupt per register, one state-machine step per tick | Non-blocking state machine, one SPI transaction per 1 ms tick | Blocking violates the "do not block the CPU" requirement; per-register interrupts create excessive ISR load and shared-state complexity; the state machine gives a bounded step and a natural retry point |
| NTC conversion | Beta equation every sample, Steinhart–Hart, lookup table with interpolation | Lookup table with linear interpolation (`ntc_table.c`) | Beta/Steinhart need `log`/`exp` (float or expensive fixed-point) per channel per cycle; the table is deterministic, needs no FPU and gives adequate accuracy over the automotive range. Cost: a small const table in flash |
| Snapshot consistency | global array with `volatile`, mutex, seqlock, double buffer | Double buffer + short critical-section index swap | A plain `volatile` array can be torn between reads; a mutex is overkill and can block; a seqlock is elegant but harder to review; double buffering is simple, allocation-free and gives a consistent 16-cell set from one conversion |
| Balancing criterion | voltage-based, SOC-based, OCV-based, capacity-based | Voltage-based with top-of-charge gate | The hardware is a passive bleed resistor and the pack is LFP; voltage is the only per-cell measurement available, and SOC-based balancing needs per-cell SOC/capacity estimates that the BOM cannot provide. Voltage-based is the standard LFP top-balance approach |
| Starvation prevention | fixed odd/even alternation, round-robin, random, priority for cells skipped last cycle | Priority cell = first candidate dropped by max-cell or adjacency, honoured within a fairness band | Fixed alternation can starve a cell that is not adjacent and ignores temperature/voltage ranking; random is not deterministic; the priority rule is deterministic, bounded and directly targets the cell that missed out |
| Candidate ordering | bubble/insertion sort, selection sort, `qsort`, bucket by voltage | Insertion sort, n ≤ 16 | n is fixed at 16; insertion sort is 120 comparisons worst case, no allocation, no recursion, stable and easy to reason about. `qsort` would add a function pointer and an implementation-defined library dependency for no benefit |
| Protection structure | one `if` block per fault, function per fault, table-driven `FaultDef_t` | Table-driven definitions + generic debounce/DTC object | Copying the same debounce code 11 times invites divergence and makes the DTC status logic untestable; the table makes thresholds, actions and release conditions data, and one state machine implements them all |
| Debounce representation | `now - start >= debounce` timestamp, incrementing counter, decrementing counter | Accumulated timer with `dt_ms` | A timestamp is sensitive to clock wrap and to missed calls; the accumulator is explicit about elapsed time, works with a variable step and is easy to test with synthetic time |
| CCL/DCL derivation | fixed limits, single temperature curve, piecewise-linear table | Piecewise-linear temperature and voltage derating + 100 mA/ms ramp | Fixed limits ignore cell temperature and voltage; a single curve cannot represent both charge and discharge; the tables make the policy reviewable and the ramp prevents steps |
| CAN signal packing | hand-written shifts per message, macro per signal, table-driven descriptor | Table-driven signal descriptors with `num/den` scaling, offset, saturation and Intel packing | Hand-written shifts duplicate signed/offset/cross-byte logic and are a classic source of sign-extension bugs; macros are hard to debug; the table is testable once and reused for every message |
| CAN Tx blocking | wait for mailbox, blocking send, queue + bounded drain | Fixed 8-entry queue, ≤4 frames per poll, drop-newest on overflow | Blocking violates the 100 ms task budget; unbounded queue would need allocation; drop-newest plus a counter is deterministic and periodic frames recover next cycle |
| ISO-TP | full ISO 15765-2 stack, single-frame only, minimal FF/FC/CF state machine | Minimal multi-frame state machine with FC handling | Full stack is unnecessary for two DIDs; single-frame only would violate the 0x0100 requirement; the minimal implementation is auditable and testable, at the cost of no extended addressing or CAN-FD |
| UDS scope | full ISO 14229 server, 0x10/0x3E/0x22/0x2E/0x19/0x14 subset | Required subset with a hook table for DID/DTC access | A full server is a large surface with little assessment value; the subset covers every requested service and keeps the NRC logic explicit |
| State machine implementation | free-form `if` chain, table of function pointers, `switch` with entry actions | `switch` + `EnterState()` helper | The transition graph is small and mostly conditional; a table would need parameters and callbacks that obscure the precharge timing; the switch keeps each transition readable and tests can drive it directly |
| HAL binding | compile-time `#ifdef`, weak symbols, function-pointer ops table | `HalOps_t` function-pointer table installed at start-up | Preprocessor HALs make it hard to build both target and simulator in one tree; weak symbols are toolchain-specific; the ops table gives one explicit injection point and NULL-safe wrappers |
| SOC estimation | coulomb counting only, OCV only, EKF, coulomb + OCV correction | Coulomb counting + rest offset learning + weighted OCV correction | Counting-only drifts; OCV-only is unusable while loaded and flat in the LFP mid-range; an EKF needs a cell model, tuning and floating point. The chosen method meets B1 with fixed point and explicit flat-region weighting |
| NVM storage | single record, A/B ping-pong, log-structured wear levelling, external EEPROM | A/B ping-pong with magic/version/sequence/CRC | A single record loses the old value on power loss during write; log-structured wear levelling is the right production answer but overkill for two parameters in the assignment; A/B doubles endurance and always leaves one valid record. The API is compatible with a future wear-levelling backend |
| Test framework | Unity, CppUTest, GoogleTest, custom asserts | Unity (MIT) | Unity is tiny, C99, allocation-free in the test runner, and vendored so the build is offline. GoogleTest is C++ and heavy; CppUTest adds more infrastructure than needed |
| Coverage tooling | gcov+lcov, gcovr, clang source-based coverage | gcov directly | lcov/gcovr were not installed in the environment; `gcov -b -c` reports the same line/branch data and the `.gcov` files are committed as evidence |
| Plotting | gnuplot, Python/matplotlib, Excel, custom SVG writer | `tools/plot_svg.c` (fixed buffers, no dependencies) | gnuplot/Python were not available; a 300-line C tool keeps the toolchain to the same compiler, uses no allocation and produces artifacts that can be committed and viewed in any browser |
| Build system | Make, CMake, Meson, hand-written scripts | CMake + Ninja + GCC, one command | CMake is portable across Windows/Linux/macOS, gives `-Wall -Wextra -Werror` per target, and supports the `simulate`, `check` and coverage options without shell-specific scripts |
| Firmware update (B2) | single-bank bootloader, A/B banks, external flash staging | Design-only A/B image with signature and power-loss recovery | The assignment marks B2 as design-only; implementing a bootloader without a target would add untestable code. The design is documented, and parameter storage (the other half of B2) is implemented |
| Parameter storage | RAM only, single flash record, A/B with CRC, wear-levelled log | Implemented A/B slots with magic/version/sequence/CRC | Meets versioning, CRC, default fallback and power-loss safety; wear levelling is documented as the production extension |

---

## 8. Balancing algorithm

`Bal_Run()` is called every 100 ms and returns the 16-bit bleed mask (bit 0 = cell 1).

**Gate order** (first match wins, and the reason is returned in `Bal_GetStatus().inhibit`):

1. `BAL_INHIBIT_MEAS_PAUSE` — the AFE conversion window; the application forces the mask to 0
   before starting a conversion and restores the new mask when the snapshot is published.
2. `BAL_INHIBIT_MEAS_INVALID` — `meas_valid == false`.
3. `BAL_INHIBIT_FAULT` — `any_fault_active`.
4. `BAL_INHIBIT_VCU_DISABLED` — `vcu_balancing_enable == false`.
5. `BAL_INHIBIT_TIMEOUT` — the 8 h continuous-balancing warning is set.
6. `BAL_INHIBIT_OVER_TEMP` — cell ≥50.0 °C or board ≥70.0 °C; latched until both are 5 °C
   below the limit.
7. `BAL_INHIBIT_DISCHARGE` — pack current below −2 A.
8. Otherwise `BAL_INHIBIT_NONE`.

**Selection.**

* Balancing is allowed only when `Vmax ≥ BAL_VMIN_MV` (3400 mV), the LFP top-of-charge gate.
* A cell already bleeding continues while `Vcell − Vmin ≥ BAL_STOP_MV` (5 mV).
* A cell not bleeding starts when `Vcell − Vmin > BAL_START_MV` (15 mV).
* Candidates are sorted by descending voltage, except that the **priority cell** (the first
  cell dropped by the max-cell or adjacency rule on the previous cycle) is moved first if it
  is within `BAL_FAIRNESS_BAND_MV` (20 mV) of `Vmax`. This prevents permanent starvation
  without ignoring a much higher cell.
* Cells are added in order while `selected < BAL_MAX_CELLS` and the new mask has no adjacent
  pair. The first rejected candidate becomes the next priority.
* `BAL_TIMEOUT` stops balancing after 8 h of accumulated active time and sets
  `timeout_warning`; a charge cycle (current > 500 mA) clears it.

**Complexity and stack.** One 16-cell min/max scan, one 16-cell candidate scan, an insertion
sort with at most 120 comparisons and a 16-cell selection pass. O(n²) with n fixed at 16,
no recursion, no allocation. Measured stack frame: 112 bytes including `candidates[16]`.

**Mandatory scenarios.** S1–S10 are implemented in `test/test_balancing.c`, plus seven
additional cases for null input, measurement window, VCU disable, max-cell exhaustion,
adjacency after the limit, priority selection and sort tie-breaking.

**Voltage-based vs SOC-based balancing (required comparison).** Voltage-based balancing is
correct for LFP at the top of charge: the OCV curve is flat between roughly 30 % and 90 % SOC,
so mid-range voltage differences say little about SOC, but above ~3.4 V the curve steepens and
a voltage delta is a usable proxy for the least-charged cell. It needs only the AFE already in
the BOM, is robust to capacity mismatch and is the standard choice for passive bleed
hardware. SOC-based balancing compares estimated per-cell SOC and is preferable when each
cell has a current sensor or a high-fidelity model, when the pack must be balanced in the flat
region, or when minimising degradation from cycling at different depths. It needs per-cell
SOC estimation and is sensitive to model/offset errors. For this 16S1P LFP pack with one pack
current sensor and passive bleed, voltage-based top-balancing is the appropriate choice;
SOC-based balancing is listed as a future extension if per-cell SOC becomes available.

---

## 9. Protection and current limits

`Prot_Update()` runs every 10 ms with `dt_ms = 10`. Each fault object holds `active`,
`pending`, `confirmed`, `release_ok`, a debounce timer and the ISO 14229 status byte. A
release condition is evaluated even while the fault is active; a latched fault clears only
when the release condition is true and a clear request arrives.

**Derating tables** (piecewise linear, `Linear()` with integer interpolation):

| Limit | Condition | Value |
|---|---|---|
| CCL | T ≤ 0.0 °C | 0 |
| | 0.0–5.0 °C | 0 → 50 A |
| | 5.0–45.0 °C | 100 A |
| | 45.0–55.0 °C | 100 → 0 A |
| | ≥55.0 °C | 0 |
| DCL | T ≤ −20.0 °C | 0 |
| | −20.0–0.0 °C | 0 → 100 A |
| | 0.0–45.0 °C | 200 A |
| | 45.0–60.0 °C | 200 → 0 A |
| | ≥60.0 °C | 0 |
| Charge voltage limit | Vmax ≤3550 mV | 100 A |
| | 3550–3650 mV | 100 → 0 A |
| | ≥3650 mV | 0 |
| Discharge voltage limit | Vmin ≤2500 mV | 0 |
| | 2500–3000 mV | 0 → 200 A |
| | ≥3000 mV | 200 A |

The final CCL/DCL is the minimum of the temperature and voltage limits, ramped at
100 mA/ms (10 A per 100 ms). `charge_enable` is false when CCL is zero, when an open request
is active, or when a charge-limiting fault is active.

---

## 10. CAN and UDS

* **CAN matrix**: 0x350 pack status, 0x351 limits, 0x352 cell statistics, 0x353 multiplexed
  cell voltages, 0x354 faults, 0x200 VCU command. Signals are Intel byte order. Messages with
  a CRC use CRC-8 SAE J1850 (poly 0x1D, init 0xFF, xorout 0xFF) over bytes 0–6. The alive
  counter is 4 bits and wraps 15 → 0.
* **Signal layer**: a descriptor table stores start bit, length, signedness, scale numerator
  and denominator, offset and physical range. Packing saturates the physical value before
  scaling, converts to a 64-bit intermediate, applies the offset and writes bits
  little-endian across byte boundaries. Unpacking uses the same descriptors.
* **VCU command validation**: wrong CRC, an unchanged alive counter and a jump of more than 2
  are rejected and counted separately. Three consecutive invalid messages are treated as a
  timeout (300 ms).
* **UDS**: 0x10 (default/extended, S3 = 5 s), 0x3E (0x00 and 0x80 suppress response),
  0x22 (0xF189 SW version, 0x0101 pack voltage/current, 0x0100 all 16 cell voltages as a
  32-byte multi-frame response), 0x2E (0x0200/0x0201 in extended session only, 5–50 mV and
  STOP < START, otherwise NRC 0x31), 0x19 0x02, 0x14 group 0xFFFFFF. NRCs implemented:
  0x11, 0x12, 0x13, 0x22, 0x31, 0x7F.
* **ISO-TP**: single-frame Rx/Tx, First Frame / Flow Control / Consecutive Frame for the
  multi-frame response. The transmitter waits for a Flow Control after the negotiated block
  size and honours STmin; a missing FC aborts after the configured timeout. Tests and the
  simulator inject the FC frame so the multi-frame path is exercised.
* **DBC**: `dbc/bms.dbc` contains the full matrix, value tables for states and the VCU
  command, cycle-time attributes and comments.

---

## 11. Verification summary

| Evidence | Location | Result |
|---|---|---|
| Unit tests | `test/`, Unity | 88 tests, 0 failures |
| Coverage (balancing) | `docs/coverage.txt` | 100.00 % lines, 96.08 % branches taken |
| Coverage (protection) | `docs/coverage.txt` | 97.30 % lines, 91.38 % branches taken |
| Static analysis | `docs/static_analysis.txt` | cppcheck 2.22.0, no findings |
| Simulation | `sim/scenarios/`, `sim/out/` | charge+balancing and fault+recovery traces and SVG plots |
| Build | `CMakeLists.txt` | one command; `-Wall -Wextra -Werror -Wpedantic` plus extra warnings |

The balancing and protection coverage requirement is ≥85 % branch coverage; both modules
exceed it using the "taken at least once" metric. Tests cover the mandatory S1–S10 scenarios
and at least two cases (set, debounce reset, release) per protection function.

---

## 12. Bonus items

**B1 — SOC estimation (implemented).** `Soc_Update()` runs every 10 ms. It accumulates charge
with a 64-bit remainder, learns the current-sensor offset whenever |I| < 0.5 A for 30 s, and
after one hour at rest pulls SOC towards the OCV estimate. The OCV lookup uses the supplied
table with linear interpolation. Because the LFP curve is flat between 3.28 V and 3.34 V, the
correction weight is limited to 5 % there and 25 % on the steep edges. SOC is stored in the
A/B NVM record every 60 s and at parameter saves, so a reset resumes near the last value.

**B2 — firmware update and parameter storage (design only, parameter storage implemented).**
Parameter storage is implemented with two slots, magic `BMSN`, version, sequence and CRC-16;
the newest valid record wins and a corrupt slot falls back to the other slot, then to
defaults. Wear levelling is documented as the production extension: a log-structured region
with erase counters, because two slots only double endurance.

The bootloader design (not implemented, as permitted) would use UDS 0x10 02, 0x27, 0x34, 0x36,
0x37 and 0x31 over CAN, an A/B image layout with a signed manifest (image hash + ECDSA/RSA
signature), a boot flag in a dedicated NVM record, and a recovery rule: the bootloader always
boots the last confirmed image; a new image is confirmed only after the application passes
its self-test and writes the confirm flag. A power loss during download leaves the old image
valid; a power loss during confirmation falls back to the old image. The application and
bootloader share the CAN/ISO-TP/UDS transport but not the safety logic.

---

## 13. Known limitations and next steps

1. **NVM write latency on target** (§3.3): make the flash write non-blocking or move it out
   of the 100 ms task. The HAL boundary isolates the change.
2. **No target hardware**: all timing is estimated; the next step is to measure WCET with a
   GPIO toggled around each task on the real MCU.
3. **No MISRA C:2012 add-on** in the environment. The code follows MISRA-friendly rules
   (fixed-width types, no dynamic memory, single exit where practical, explicit casts), but a
   formal deviation list is not produced.
4. **Cell open-wire** is reported as part of the AFE fault reaction; the driver does not
   localise which cell is open. A production AFE would read the open-wire diagnostic register
   and map it to a cell index.
5. **ISO-TP** has no CAN-FD or extended addressing; it covers the assignment's 7-byte classic
   CAN payloads.
6. **No CI file**: the environment had no Git or hosted runner. The `check` CMake target is
   the local equivalent; a GitHub Actions workflow would run the same three commands.
7. **Bootloader** is design-only per the assignment.
