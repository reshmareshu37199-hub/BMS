# BMS-FW-ASG-01 — Submission

**16S1P LFP battery management system — firmware design, implementation, verification and
code review**

| | |
|---|---|
| **Candidate** | ______________________________ |
| **Date** | 16 November 2026 |
| **Deliverable** | `bms-fw/` — source, tests, simulation, DBC and documentation |
| **Companion documents** | `README.md`, `ASSUMPTIONS.md`, `CODE_REVIEW.md`, `docs/DESIGN.md` |

> **Summary.** This is the complete answer to the assignment: all eight mandatory tasks and
> both bonuses. It covers the architecture and timing argument, the AFE driver, balancing,
> protection and DTCs, CAN/DBC/UDS diagnostics, the test and simulation evidence, the code
> review, and the parameter-storage and firmware-update design. Section 14 records every
> alternative that was considered and why it was rejected; section 15 maps every requirement
> to a file, test or trace.

### Assignment parameters used

| Parameter | Value |
|---|---|
| Pack | 16S1P LFP, 100 Ah, nominal 51.2 V |
| Cell voltage range | 2.50 V – 3.65 V |
| Pack voltage range | 40.0 V – 58.4 V |
| Temperature sensors | 4 × NTC 10 kΩ (B25/85 = 3435) on cells, 1 × NTC on the BMS board |
| Current sensor | ±300 A, already scaled to mA by the HAL; positive = charge |
| Balancing | passive, 39 Ω bleed per cell (≈ 85 mA at 3.3 V) |
| Main control cycle | 100 ms — measurement, balancing, protection, CAN Tx |
| Fast cycle | 10 ms — current measurement, over-current protection |
| CAN | 500 kbit/s, 11-bit identifiers, Intel (little-endian) byte order |
| States | INIT → STANDBY → PRECHARGE → DRIVE / CHARGE → STANDBY → SLEEP; FAULT from any state |

**How to read this document.** Sections 1–3 are the summary; sections 4–11 are the eight
tasks in order; sections 12–13 are the bonuses; sections 14–20 are the alternatives,
traceability, speaking notes, likely questions, the three-day plan, limitations and appendix.
The code is the authority: if a detail here disagrees with `src/`, the code and its tests win.

---

## Contents

1. [The one-page answer](#1-the-one-page-answer)
2. [The problem, decoded](#2-the-problem-decoded)
3. [Solution at a glance](#3-solution-at-a-glance)
4. [Task 1 — architecture, scheduling, state machine, safety](#4-task-1--architecture-scheduling-state-machine-safety)
5. [Task 2 — AFE driver](#5-task-2--afe-driver)
6. [Task 3 — passive balancing](#6-task-3--passive-balancing)
7. [Task 4 — protection and DTCs](#7-task-4--protection-and-dtcs)
8. [Task 5 — CAN, DBC and UDS](#8-task-5--can-dbc-and-uds)
9. [Task 6 — tests, simulation, static analysis, coverage](#9-task-6--tests-simulation-static-analysis-coverage)
10. [Task 7 — code review](#10-task-7--code-review)
11. [Task 8 — documentation and code quality](#11-task-8--documentation-and-code-quality)
12. [Bonus B1 — SOC estimation](#12-bonus-b1--soc-estimation)
13. [Bonus B2 — parameter storage and firmware update](#13-bonus-b2--parameter-storage-and-firmware-update)
14. [Every alternative, and why it was not chosen](#14-every-alternative-and-why-it-was-not-chosen)
15. [Requirement traceability](#15-requirement-traceability)
16. [Review-session speaking notes](#16-review-session-speaking-notes)
17. [Likely questions and model answers](#17-likely-questions-and-model-answers)
18. [The three-day plan](#18-the-three-day-plan)
19. [Limitations and production roadmap](#19-limitations-and-production-roadmap)
20. [Appendix — file index, DTC map, CAN map, glossary](#20-appendix)

---

## 1. The one-page answer

> I built a complete, host-runnable BMS firmware in C11: a 1 ms tick with a 10 ms fast control
> cycle and a 100 ms main control cycle, a non-blocking AFE acquisition state machine with CRC
> and retries, table-driven passive balancing with hysteresis, adjacency and starvation
> control, a table-driven protection layer with 11 DTCs and derating, a table-driven CAN
> signal layer with CRC and alive counters, a UDS/ISO-TP diagnostic server, SOC estimation
> with OCV correction, and A/B NVM parameter storage. It has 88 unit tests, 100 % line
> coverage on balancing and 96 % branch coverage, 97 % line/91 % branch on protection, zero
> cppcheck findings, and two simulation scenarios with committed plots. The architecture is
> layered so that every safety-relevant decision is a pure function tested on the PC, and
> every hardware access is behind a HAL.

The four decisions that carry the design:

1. **Fixed cyclic executive, not an RTOS.** Five periodic entries with hard, known periods. The
   scheduler gives deterministic order, bounded work per tick, an overrun counter, and a
   host build that behaves like the target. An RTOS would add footprint, priority-inversion
   analysis and non-reproducible timing for no functional gain.
2. **Every periodic activity is non-blocking and bounded.** The AFE does one SPI transaction
   per 1 ms tick; CAN drains at most four frames per tick; NVM is the only potentially slow
   call and is isolated behind the HAL and documented.
3. **One source of truth per contract.** `FaultDef_t` defines every protection behaviour;
   `CanSignalDef_t` defines every signal's bit position and scale; `bms.dbc` is the external
   contract; `BalConfig_t` defines every balancing threshold. Adding a fault, a signal or a
   threshold is a table edit, not a new branch.
4. **Safety is structural, not a claim.** Safe state is the default, hard OV/UV/OC bypass the
   debounce, contactors are held for 1 s when opening is not urgent, all shared multi-field
   data is double-buffered, and every loop has a static bound. No `malloc`, no recursion, no
   variable-length arrays.

The honest limitations: no target hardware, so the timing numbers are conservative
estimates; no MISRA add-on or clang-tidy in the environment; the bootloader half of B2 is
design-only; NVM wear levelling would be log-structured in production.

---

## 2. The problem, decoded

### 2.1 What the assignment asks for

| # | Task | What a strong answer must show |
|---|---|---|
| 1 | Architecture, scheduling, state machine, data sharing, functional safety | A defensible decomposition, a timing argument, a transition table, a concurrency argument, a hazard mindset |
| 2 | AFE driver: CRC, retries, plausibility, snapshot | Non-blocking acquisition, real error handling, a coherent data hand-off, testability |
| 3 | Balancing: S1–S10, fairness, inhibits, timeout | Correct hysteresis, adjacency, max-cell and starvation logic, every inhibit, a bounded algorithm |
| 4 | Protection: table, debounce, DTCs, derating | Data-driven faults, correct latch/clear, hard thresholds, documented CCL/DCL |
| 5 | CAN matrix, signal layer, DBC, UDS/ISO-TP | Correct scaling/endianness/CRC, a complete DBC, a diagnostic subset with real NRCs |
| 6 | Tests, simulation, static analysis, ≥85 % branch coverage | Evidence, not claims: a runnable test suite, scenarios, coverage, cppcheck |
| 7 | Code review of three listings | Find the unsafe and undefined-behaviour defects, explain why, give corrected code |
| 8 | Documentation and code quality | A README that builds in one command, assumptions, clean headers, no warnings |
| B1 | SOC estimation, flat OCV region | Coulomb counting + a principled correction that does not jump in the flat region |
| B2 | Firmware update + parameter storage (design-only) | A/B, CRC, versioning, power-loss cases, signature, rollback |

### 2.2 Constraints that shape the solution

| Constraint | Consequence in the code |
|---|---|
| C11, fixed-width types | `stdint.h` everywhere; `bool` from `stdbool.h`; no compiler extensions |
| No `malloc`/`free` in application code | All buffers are static or caller-provided; fixed capacities (`CANDRV_QUEUE_LEN`, `ISOTP_MAX_PAYLOAD`, `NVM_PAYLOAD_MAX`) |
| Bounded execution | Every loop has a compile-time or input-bounded limit; one SPI transaction per tick; ≤4 CAN frames per poll; no `while (!flag)` |
| PC-host build with simulated HAL | `hal_sim.c` implements time, SPI, contactors, voltages, current, temperatures, CAN, NVM; `afe_sim.c` models the register map |
| `-Wall -Wextra -Werror` | Plus `-Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wdouble-promotion -Wformat=2 -Wundef -Wcast-qual -Wpointer-arith` |
| 3 calendar days | Scope chosen so every mandatory item is complete; bonuses are real but bounded |
| AI allowed but disclosed | README §7 lists where AI helped and how every line was verified |

### 2.3 What "complete" means here

The repository does not stop at code. For each requirement there is a runnable artefact:

* a unit test that fails if the behaviour regresses;
* a simulation trace that shows the behaviour over time;
* a coverage or static-analysis report;
* a paragraph in the design document explaining the trade-off.

That is the difference between "I wrote a BMS" and "I can defend a BMS in a review".

---

## 3. Solution at a glance

### 3.1 Repository map

```text
bms-fw/
├── README.md                     build, design summary, AI disclosure, traceability
├── ASSUMPTIONS.md                A1–J4, each with why and impact
├── CODE_REVIEW.md                Task 7: 47 findings + corrected code
├── CMakeLists.txt                build, tests, simulate, check, coverage
├── dbc/bms.dbc                   complete CAN database
├── docs/
│   ├── DESIGN.md                 Task 1 + alternatives log
│   ├── BMS-FW-ASG-01_Submission.md    this document
│   ├── BMS-FW-ASG-01_Submission.pdf   print-ready A4
│   ├── BMS-FW-ASG-01_Submission.docx  editable Word version
│   ├── COMPLETE_SOLUTION.md      the same master answer (internal copy)
│   ├── COMPLETE_SOLUTION.html    self-contained HTML
│   ├── COMPLETE_SOLUTION.pdf     print-ready A4
│   ├── style.css                 print style for the HTML/PDF
│   ├── coverage.txt              gcov evidence
│   ├── static_analysis.txt       cppcheck evidence
│   └── plots/                    checked-in SVG plots
├── src/
│   ├── common/                   CRC-8/16, saturation
│   ├── hal/                      HAL interface, PC HAL, AFE register model
│   ├── drivers/                  AFE state machine, CAN queue, NTC table
│   ├── services/                 balancing, protection, SOC, CAN signal/matrix,
│   │                             ISO-TP, UDS, NVM
│   └── app/                      scheduler, state machine, application
├── test/                         88 Unity tests
├── sim/                          CSV scenario runner, scenarios, traces, plots
├── tools/plot_svg.c              dependency-free CSV → SVG
└── third_party/unity/            vendored MIT test framework
```

### 3.2 Numbers

| Metric | Result | Where |
|---|---|---|
| Unit tests | 88, 0 failures | `test/`, `ctest` |
| Balancing coverage | 100.00 % lines, 96.08 % branches taken | `docs/coverage.txt` |
| Protection coverage | 97.30 % lines, 91.38 % branches taken | `docs/coverage.txt` |
| Static analysis | cppcheck 2.22.0, no findings, 18 files | `docs/static_analysis.txt` |
| Max stack frame | 400 B (`Task_Fast`), 112 B (`Bal_Run`) | `-fstack-usage` |
| Estimated CPU load | ≈ 8.5 % upper bound (1 ms tick charged at 80 µs); ≈ 1–2 % average | `docs/DESIGN.md` §3.3 |
| CAN messages | 6 (5 Tx, 1 Rx) | `dbc/bms.dbc` |
| DTCs | 11 | `protection.c` |
| Simulation | 2 scenarios, 5 SVG plots | `sim/out/` |

### 3.3 Build and evidence in four commands

```bash
cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build
ctest --test-dir build --output-on-failure
cmake --build build --target simulate
```

Expected:

```text
88 Tests 0 Failures 0 Ignored
OK
```

`simulate` writes `sim/out/charge_balance*.{csv,svg}` and `sim/out/fault_recovery*.{csv,svg}`.

---

## 4. Task 1 — architecture, scheduling, state machine, safety

### 4.1 Layering and the dependency rule

```text
APPLICATION     scheduler, state machine, app orchestration, CAN/UDS glue
     │
SERVICES        balancing, protection, SOC, CAN signal/matrix, ISO-TP, UDS, NVM
     │
DRIVERS         AFE state machine, CAN queue/dispatch, NTC table
     │
HAL             time, SPI, contactors, voltages, current, temperature, CAN, NVM,
     │          critical section, watchdog
     ├── target: vendor HAL + real peripherals
     └── PC:     hal_sim.c + afe_sim.c
```

The rule is: **arrows point down only.** A service never includes a driver or application
header. `afe.c` and `can_driver.c` are the only modules that call the HAL. That is what makes
balancing, protection, SOC, the signal layer and UDS pure logic that runs on a PC with no
mocking framework.

The HAL is an injected ops table (`HalOps_t`), not `#ifdef` and not weak symbols:

```c
/* one explicit injection point; wrappers are NULL-safe */
typedef struct {
    uint32_t (*time_ms)(void);
    bool     (*spi_transfer)(const uint8_t *tx, uint16_t tx_len,
                             uint8_t *rx, uint16_t rx_len);
    void     (*set_contactor)(HalContactor_t which, bool closed);
    uint16_t (*pack_voltage_mv)(void);
    uint16_t (*bus_voltage_mv)(void);
    int32_t  (*pack_current_ma)(void);
    /* ... CAN, NVM, critical section, watchdog ... */
} HalOps_t;
```

Why not `#ifdef`? It makes it hard to build the target and the simulator from one tree and
it hides which calls are hardware access. Why not weak symbols? Toolchain-specific and easy
to link the wrong implementation silently. Why not a full RTOS HAL? The assignment does not
need threads.

### 4.2 Scheduling

**Chosen:** a fixed cyclic executive. A 1 ms tick runs a table of five entries in registration
order. The 1 ms entry is the **scheduler quantum**: it advances the AFE state machine by one
bounded SPI transaction, drains CAN and kicks the watchdog. The control cycles required by the
brief are the **10 ms fast task** (current, over-current, protection, state machine) and the
**100 ms main task** (AFE cycle, balancing, CAN Tx). The order is the priority order.

| Order | Task | Period | Work | Est. WCET | Load |
|---|---|---|---:|---:|---:|
| 1 | `tick` | 1 ms | advance the AFE state machine by one SPI transaction; apply a new balancing mask when a snapshot is published; drain ≤4 CAN Tx; dispatch ≤8 CAN Rx; kick watchdog | 80 µs | 8.0 % |
| 2 | `fast` | 10 ms | sample current; over-current; VCU monitor; protection/DTC; state machine/contactors; SOC; ISO-TP/UDS; publish snapshot | 40 µs | 0.40 % |
| 3 | `main` | 100 ms | force balancing off, start the AFE conversion, send 0x350/0x351, persist when due | 60 µs | 0.06 % |
| 4 | `cellstats` | 500 ms | send 0x352 | 15 µs | 0.003 % |
| 5 | `slow` | 1000 ms | send 0x353 (6 mux frames) + 0x354 | 60 µs | 0.006 % |
| | **Total** | | | | **≈ 8.5 %** |

The 8.0 % for the tick is a deliberate **upper bound**: it charges 80 µs to every millisecond.
The AFE is only active for about 22 ms of each 100 ms cycle, so the steady-state average is
closer to 1–2 %. The bound is used so the timing budget cannot be argued downward.

Worst-case 1 ms tick: all five entries due at once ≈ 255 µs, still below the 1 ms quantum.
The 10 ms fast task therefore has at least 9.7 ms of slack in every period.

**Overrun policy.** If a task's deadline has passed when it is next considered, the scheduler
runs it once, re-bases the deadline to `now + period`, and increments an overrun counter. It
does **not** catch up. Catching up would turn one late task into a burst of late work and
could cascade into a missed watchdog. A missed deadline becomes observable telemetry instead
of a hidden latency spike.

**Why not an RTOS?** Five fixed periodic entries do not justify the footprint, the
priority-inversion analysis, the per-task stacks or the loss of host reproducibility. Why not
a bare super-loop with delays? Blocking delays destroy jitter and make WCET unbounded, which
contradicts the requirement. Why not timer-ISR work? Long ISRs block everything and are not
composable with SPI/CAN. The full comparison is in §14.

**The NVM caveat.** On the PC, `Hal_NvmWrite` is a memory copy. On target, a data-flash page
erase + program can take tens of milliseconds. A synchronous call from the 100 ms main task
would delay the 10 ms fast task by the same amount, which is **not acceptable** for the
over-current path. The operation is isolated behind `Hal_NvmWrite`; the production
implementation must be page-buffered/non-blocking, or the save must be deferred to STANDBY.
The delivered PC HAL keeps the build and tests deterministic. This is the top item in §19.

### 4.3 State machine

```text
                 ┌─────────┐
                 │  INIT   │
                 └────┬────┘
        valid AFE + 200 ms
                      ▼
                 ┌─────────┐  sleep request   ┌─────────┐
                 │ STANDBY │ ───────────────► │  SLEEP  │
                 └────┬────┘ ◄─────────────── └─────────┘
      close request  │        wake + VCU valid
                      ▼
                 ┌───────────┐   bus ≥ 95 % + 20 ms   ┌─────────┐
                 │ PRECHARGE │ ─────────────────────► │  DRIVE  │
                 └─────┬─────┘                        └────┬────┘
        charger present│                                  │
                      ▼                                   │
                 ┌─────────┐  charger removed             │
                 │ CHARGE  │ ─────────────────────────────┘
                 └────┬────┘
                      │ fault (any state)
                      ▼
                 ┌─────────┐  no fault + 1 s
                 │  FAULT  │ ───────────────► STANDBY
                 └─────────┘
```

| State | Contactors | Entry/exit behaviour |
|---|---|---|
| `INIT` | all open | wait for a valid AFE snapshot and ≥200 ms; AFE fault → `FAULT` |
| `STANDBY` | all open | close request + valid measurements → `PRECHARGE`; sleep → `SLEEP`; fault → `FAULT` |
| `PRECHARGE` | NEG on, PRE on after 20 ms | wait for `bus ≥ 0.95 × pack` (2 s timeout); then 20 ms overlap, POS on, PRE off → `DRIVE`/`CHARGE` |
| `DRIVE` | NEG + POS on | request removed or VCU lost 1 s → `STANDBY`; fault → `FAULT` |
| `CHARGE` | NEG + POS on | as DRIVE; charger disappears → `DRIVE` |
| `FAULT` | held up to 1 s, then open | return to `STANDBY` after 1 s with no active fault |
| `SLEEP` | all open | wake when sleep request removed and VCU valid |

**Precharge timing.** The four phases are evaluated every 10 ms:

1. 20 ms settle: NEG on, PRE off, POS off.
2. One cycle: PRE on.
3. Wait for bus ≥ 95 % of pack, timeout 2 s.
4. 20 ms overlap: POS on, then PRE off on entering DRIVE/CHARGE.

A 20 ms settle is an engineering choice: it debounces the voltage comparison and guarantees
overlap when the positive contactor closes. A precharge timeout emits a one-cycle pulse that
the application feeds to protection as `FAULT_PRECHARGE`; the fault latches and clears through
the normal DTC path.

**Fault opening policy.** Two independent outputs:

* `contactor_open_request` — open, but first give the VCU 1 s to reduce current.
* `contactor_open_immediate` — open on the same cycle for over-current, hard OV (>3750 mV),
  hard UV (≤2300 mV) and precharge failure.

This satisfies "never open under high current if it can be avoided" without slowing the fast
protection paths.

**VCU timeout.** A missing/invalid 0x200 for 300 ms moves the system to `STANDBY` and opens
after 1 s. It is reported as DTC 0x000A09 but deliberately **excluded** from `fault_active`,
so a lost VCU is a communication loss, not a latched hardware fault.

### 4.4 Data sharing and concurrency

The application is single-context cooperative. On target the only ISR work is the 1 ms tick
(increments a volatile counter) and short peripheral glue. Rules:

1. **AFE double buffer.** Two `AfeSnapshot_t` buffers plus a volatile active index. The
   producer fills the inactive buffer, then swaps the index inside a critical section.
   Consumers copy the active buffer under the same section. No SPI runs inside it.
2. **Short critical sections.** Only the index swap and the snapshot copy — no CRC, no loops
   over 16 cells, no blocking.
3. **`volatile` only where required.** The active index and the tick counter. Arrays touched
   only in main context are not volatile, so the optimizer is not defeated.
4. **CAN Rx is polled.** `CanDrv_Poll()` drains the HAL queue in the 1 ms tick task and calls
   handlers in main context. Handlers update small records; they do not block.
5. **Tx queue.** 8 entries, ≤4 sends per poll, drop-newest with a counter. A full mailbox
   never blocks the 100 ms task.
6. **Application snapshot.** `App_GetSnapshot()` copies under a critical section so the
   simulator, CAN and diagnostics see a coherent set of state, voltages, limits and faults.

**Why double buffer, not a mutex or seqlock?** A mutex can block and needs an RTOS; a seqlock
is elegant but harder to review and needs an atomic sequence; a plain volatile array tears
between fields. Double buffering is allocation-free, gives a consistent 16-cell set from one
conversion, and the critical section is a few dozen bytes. It is the simplest mechanism that
is obviously correct.

### 4.5 Functional safety mindset

The safe state is: **both contactors open, precharge open, CCL = DCL = 0, balancing mask = 0,
latched DTC recorded.** `Prot_Update()` computes it from a table; application code cannot
bypass the table.

Structural mechanisms:

| Mechanism | What it prevents |
|---|---|
| Table-driven fault definitions | Divergent debounce/latch logic copied 11 times |
| Independent hard thresholds | A debounce bug delaying an OV/UV/OC open |
| Bounded loops, no allocation, no recursion | Unbounded latency, fragmentation, stack overflow |
| CRC-8 on SPI and safety CAN frames; CRC-16 on NVM | Silent corruption crossing a boundary |
| Watchdog kicked only from the 1 ms tick task | A wedged scheduler going unnoticed |
| Overrun counter | Missed deadlines hidden as "normal" |
| Defensive calibration validation | Corrupt NVM/balancing config causing unsafe behaviour |
| Latch + release + explicit clear | A transient clear request masking a present fault |
| VCU command CRC/alive/jump checks | Stuck, replayed or spoofed commands |

Out of scope, and stated as such: a full ISO 26262 safety case (FMEDA, diagnostic coverage
targets, lockstep/dual-channel plausibility, secure boot chain). The assignment asks for a
mindset, not a certification package.

---

## 5. Task 2 — AFE driver

### 5.1 Contract

SPI frame: `[ADDR|R/W] [LEN] [DATA…] [CRC8]`, bit 7 of the address is write, CRC-8 poly
0x07, init 0x00, no reflection, no final XOR. Read response: `[DATA…][CRC8]`, CRC over the
data bytes. Device ID register 0x00 returns 0x5A. Cell registers are 16-bit big-endian,
100 µV/LSB. A 2 ms conversion is started by writing 0x01 to `ADC_CTRL`.

### 5.2 Non-blocking state machine

```text
READ_ID ──ok──► IDLE ──request──► START_CONV ──ok──► WAIT_DONE
   │                                 │ fail             │ done
   │ 3 fails                         ▼                  ▼
   └────────► comm fault        retry ≤ N          READ_CELLS (16 × 1 transaction)
                                                     │
                                                     ▼
                                                 READ_TEMPS (5 × 1 transaction)
                                                     │
                                                     ▼
                                                  PUBLISH
```

* One SPI transaction per 1 ms scheduler tick.
* A complete 16-cell + 5-temperature cycle takes ≈ 22 ms.
* The 100 ms main task starts a cycle; the 10 ms fast task uses the previous completed
  snapshot, so data is at most ≈ 120 ms old.
* `Afe_PublishSnapshot()` converts, checks the cell-sum plausibility, copies flags, swaps the
  buffer and clears the retry counters.

### 5.3 Error handling

| Error | Reaction |
|---|---|
| Device ID wrong or unreadable, 3 attempts | latched AFE communication fault, `id_failed` blocks acquisitions |
| CRC mismatch | transaction failure → retry |
| SPI transfer failure | transaction failure → retry |
| Conversion not done before deadline | transaction failure → retry |
| Retries exhausted | failed-cycle counter++; after 3 cycles → latched comm fault, publish invalid |
| Internal AFE fault / open wire | `valid = false`, flags in snapshot; protection opens and latches |
| Cell sum vs pack > 1000 mV | `sum_mismatch` flag; protection limits CCL/DCL |

A single good cycle clears the failure counters but **not** the latched fault; the latch is
cleared only through `Afe_ClearLatchedFault()` on the explicit DTC-clear path.

### 5.4 Snapshot and testability

Two buffers, critical-section index swap, `Afe_GetSnapshot()` copies. Tests inject register
values through `afe_sim.c` and drive `Afe_Poll()` with synthetic time; no hardware, no sleeps.

**Tests:** device-ID self-test, wrong ID, successful acquisition, balancing-mask write,
sum mismatch, internal fault/open wire, CRC retry then latch, SPI timeout, API guards.

---

## 6. Task 3 — passive balancing

### 6.1 Requirements, restated as gates

Balancing may bleed only when **all** of these hold. First match wins, and the reason is
reported in `Bal_GetStatus().inhibit`:

| Order | Inhibit | Condition |
|---:|---|---|
| 1 | `BAL_INHIBIT_MEAS_PAUSE` | AFE conversion window (application forces mask 0) |
| 2 | `BAL_INHIBIT_MEAS_INVALID` | snapshot invalid / stale |
| 3 | `BAL_INHIBIT_FAULT` | any protection fault active |
| 4 | `BAL_INHIBIT_VCU_DISABLED` | VCU balancing-enable bit is 0 |
| 5 | `BAL_INHIBIT_TIMEOUT` | 8 h continuous-balancing warning set |
| 6 | `BAL_INHIBIT_OVER_TEMP` | cell ≥ 50.0 °C or board ≥ 70.0 °C; latched until 5 °C below both |
| 7 | `BAL_INHIBIT_DISCHARGE` | pack current < −2 A |
| — | `BAL_INHIBIT_NONE` | run selection |

### 6.2 Selection algorithm

```text
every 100 ms:
  if gate active: mask = 0, active_time = 0 (except measurement pause), report reason
  else:
    find vmin, vmax
    if vmax < 3400 mV: mask = 0                  # LFP top-of-charge gate
    else:
      candidates = cells where
          (was bleeding  AND vcell - vmin >= 5 mV)   # stop threshold
       or (not bleeding AND vcell - vmin > 15 mV)   # start threshold
      sort candidates: priority cell first if within 20 mV of vmax,
                       then by descending voltage, ties by index
      for each candidate:
          stop if 8 selected
          skip if adding it creates an adjacent pair
          remember the first skipped cell as next cycle's priority
      mask = selected
      accumulate active time; at 8 h: mask = 0, warning, inhibit
```

**Why 15/5 mV hysteresis?** A single threshold chatters when a cell sits near the boundary
and the bleed current perturbs the measurement. 15 mV start / 5 mV stop gives a 10 mV dead
band, so a cell that starts bleeding keeps bleeding until it is genuinely closer to the pack
minimum.

**Why max 8 cells?** The board thermal limit and the adjacency rule both cap the theoretical
maximum at 8 (16 cells, no two neighbours). The configuration is validated to 1…16.

**How starvation is prevented.** A naive loop always drops the same lowest-priority cell of
an adjacent pair. The algorithm records the first cell rejected by the max-cell or adjacency
rule and promotes it to the front of the next cycle's candidate list — but only if it is
within 20 mV of `Vmax`. A genuinely much-higher cell still wins. This is deterministic,
bounded and directly targets the cell that missed out; random or fixed odd/even schemes are
neither.

**Why an 8 h timeout?** Continuous bleeding for days is a symptom (a cell that never
converges, a measurement offset, a stuck switch). The timeout stops it, raises a warning and
requires a charge cycle to clear. It is a warning, not a DTC, because it is not a pack
hazard by itself.

**Measurement-window pause.** The application forces the mask to 0 before starting a
conversion and restores the newly computed mask when a snapshot is published. This prevents
the bleed current's IR drop from corrupting the cell voltage being measured. The cost is
≈ 22 ms of every 100 ms cycle, so the maximum duty is ≈ 78 %.

### 6.3 Voltage-based vs SOC-based balancing (required comparison)

| | Voltage-based (chosen) | SOC-based |
|---|---|---|
| Input | per-cell voltage from the AFE | per-cell SOC estimate |
| Works in the flat LFP region | No — gated to `Vmax ≥ 3.4 V` | Yes |
| Hardware needed | the AFE already in the BOM | per-cell current sensing or a validated cell model |
| Robust to capacity mismatch | Yes | Less so; depends on the model |
| Sensitivity | measurement offset and IR drop | model error, offset error, integration drift |
| Best for | passive bleed, top-of-charge balancing |active balancing, flat-region balancing, degradation management |

For a 16S1P LFP pack with one pack current sensor and 39 Ω passive bleed resistors,
voltage-based top-balancing is the right engineering choice. SOC-based balancing is listed
as a future extension if per-cell SOC becomes available.

### 6.4 Complexity, stack and verification

* One 16-cell min/max scan, one candidate scan, insertion sort with ≤120 comparisons,
  one selection pass = O(n²), n fixed at 16, no recursion, no allocation.
* `Bal_Run` stack frame: 112 B including `candidates[16]`.
* Tests: S1–S10 plus 7 cases (validation, measurement pause, VCU disable, max-cell
  exhaustion, adjacency-after-limit, priority selection, sort ties).
* Coverage: 100 % lines, 96.08 % branches taken.

### 6.5 The mandatory scenarios

| ID | Scenario | Expected |
|---|---|---|
| S1 | all cells below 3.4 V | mask 0 |
| S2 | one cell clearly highest | only that cell |
| S3 | two adjacent high cells alternate | never both; priority alternates |
| S4 | more than 8 cells qualify | ≤8, no adjacent pair |
| S5/S6 | hysteresis | start >15 mV, continue ≥5 mV |
| S7 | over-temperature | inhibit, latch, resume only 5 °C below |
| S8 | discharge < −2 A | inhibit immediately |
| S9 | invalid measurement / active fault | mask 0 on the same cycle |
| S10 | 8 h timeout and charge reset | mask 0, warning, cleared by charge |

---

## 7. Task 4 — protection and DTCs

### 7.1 Table-driven design

`FaultDef_t` holds: DTC, set threshold, release threshold, debounce, latch, action mask,
set/release condition functions, and an optional hard threshold. One generic object
(`active`, `pending`, `confirmed`, `release_ok`, timer, ISO 14229 status) implements all 11.

Why? Eleven copied `if` blocks diverge over time, and the DTC status logic becomes
untestable. The table makes thresholds, actions and release conditions data; one state
machine executes them. Adding a fault is a table row plus a condition pair.

### 7.2 Fault table

| Fault | DTC | Set | Debounce | Release | Latch | Reaction |
|---|---|---|---:|---|---|---|
| Cell OV | 0x000A01 | any ≥3650 mV | 1 s | all ≤3400 mV | yes | CCL 0; open >3750 mV immediately |
| Cell UV | 0x000A02 | any ≤2500 mV | 1 s | all ≥2900 mV | yes | DCL 0; open ≤2300 mV immediately |
| Charge OT | 0x000A03 | Tmax ≥55 °C while charging | 2 s | Tmax ≤50 °C | no | CCL 0 |
| Discharge OT | 0x000A04 | Tmax ≥60 °C | 2 s | Tmax ≤55 °C | yes | open |
| Charge UT | 0x000A05 | Tmin ≤0 °C while charging | 2 s | Tmin ≥3 °C | no | CCL 0 |
| Discharge OC | 0x000A06 | I ≤ −200 A | 500 ms | — | yes | open immediately |
| Charge OC | 0x000A07 | I ≥ +100 A | 1 s | — | yes | open immediately |
| AFE comm | 0x000A08 | 3 failed cycles | — | good cycle | yes | open |
| VCU timeout | 0x000A09 | no valid 0x200 for 300 ms | — | valid frame | no | STANDBY; open after 1 s |
| Cell-sum mismatch | 0x000A0A | \|Σcells − pack\| >1000 mV | — | in tolerance | no | CCL/DCL 0 |
| Precharge | 0x000A0B | bus <95 % after 2 s | — | — | yes | open immediately |

"Charging" is `I > +500 mA`; "discharging" is `I < −500 mA`. The dead band prevents
temperature faults chattering around zero.

The current sensor is specified as ±300 A. The over-current thresholds are deliberately set
below full scale — 200 A discharge and 100 A charge — so the comparator trips while the
signal is still in its linear range and the contactor opens before the sensor saturates. The
thresholds are configuration, not hard-coded to the sensor range.

### 7.3 Debounce semantics

```text
set condition true  ──► timer += dt
                        timer >= debounce?
                            yes: active = true, latch if configured
set condition false ──► timer = 0        # a single bad sample cannot accumulate
active + release true ──► release_ok = true
release false         ──► release_ok = false
clear request + release_ok + latched ──► clear active/latch, record "since clear"
clear request while condition present ──► ignored
```

An accumulated timer is used instead of a timestamp because it is explicit about elapsed
time, works with a variable step, is immune to clock wrap, and is trivial to test with
synthetic time.

### 7.4 ISO 14229 status bits

| Bit | Name | Use |
|---|---|---|
| 0x01 | testFailed | condition currently true |
| 0x02 | testFailedThisCycle | failed since the last 0x14/cycle reset |
| 0x04 | pending | debounce running, not confirmed |
| 0x08 | confirmed | debounce elapsed and latched |
| 0x10 | testNotCompletedSinceClear | cleared, not re-tested |
| 0x20 | testFailedSinceClear | failed at least once since clear |
| 0x40 | testNotCompletedThisCycle | not tested this cycle |
| 0x80 | warning | warning-class fault |

UDS 0x19 0x02 returns three DTC bytes plus the status byte for records matching the request
mask. The availability mask is 0xFF. Manufacturer range `0x000A01…0x000A0B` is used because
the assignment does not assign numbers; a production project would take the OEM allocation.

### 7.5 CCL/DCL derating

| Limit | Temperature / voltage | Value |
|---|---|---|
| CCL | ≤0.0 °C | 0 |
| | 0.0–5.0 °C | 0 → 50 A |
| | 5.0–45.0 °C | 100 A |
| | 45.0–55.0 °C | 100 → 0 A |
| | ≥55.0 °C | 0 |
| DCL | ≤−20.0 °C | 0 |
| | −20.0–0.0 °C | 0 → 100 A |
| | 0.0–45.0 °C | 200 A |
| | 45.0–60.0 °C | 200 → 0 A |
| | ≥60.0 °C | 0 |
| Charge voltage | ≤3550 mV | 100 A |
| | 3550–3650 mV | 100 → 0 A |
| | ≥3650 mV | 0 |
| Discharge voltage | ≤2500 mV | 0 |
| | 2500–3000 mV | 0 → 200 A |
| | ≥3000 mV | 200 A |

Final CCL/DCL = minimum of the temperature and voltage limits, ramped at 100 mA/ms
(10 A per 100 ms). Both start at 0 at boot so the first CAN frame cannot command a step.
`charge_enable` is false when CCL is 0, an open request is active, or a charge-limiting fault
is active.

**Tests:** each fault has set, debounce-reset and release cases; hard-threshold and
immediate-open paths; derating and ramp; DTC indexing; output defaults. Coverage: 97.30 %
lines, 91.38 % branches taken.

---

## 8. Task 5 — CAN, DBC and UDS

### 8.1 Message matrix

| ID | Message | DLC | Period | Content | CRC / alive |
|---|---|---:|---:|---|---|
| 0x350 | `BMS_PackStatus` | 8 | 100 ms | pack voltage, current, SOC, state, alive | CRC byte 7, alive bits 44–47 |
| 0x351 | `BMS_Limits` | 8 | 100 ms | CCL, DCL, charge voltage request, enable, alive | CRC byte 7, alive bits 52–55 |
| 0x352 | `BMS_CellStats` | 8 | 500 ms | Vmax/Vmin + indices, Tmax/Tmin | none |
| 0x353 | `BMS_CellVoltages` | 8 | 1000 ms | mux 0–5, 3 cells, 3 balance bits | none |
| 0x354 | `BMS_Faults` | 8 | 1000 ms + on change | fault bits, warning bits, active DTC count | none |
| 0x200 | `VCU_Command` | 8 | 100 ms (Rx) | contactor request, balancing enable, sleep, fault clear, alive | CRC byte 7, alive bits 52–55 |

All signals are Intel (little-endian). The DBC uses decimal IDs 848–852 and 512.

### 8.2 Signal layer

A descriptor is the single source of truth:

```c
typedef struct {
    uint8_t  start_bit;
    uint8_t  length;
    CanSigType_t type;      /* unsigned / signed */
    int32_t  scale_num;
    int32_t  scale_den;
    int32_t  offset;
    int32_t  phys_min;
    int32_t  phys_max;
} CanSignalDef_t;
```

Packing:

1. saturate the physical value to `[phys_min, phys_max]`;
2. subtract the offset, multiply by `den`, divide by `num` with round-to-nearest;
3. clamp to the raw range for the bit length and signedness;
4. write bits little-endian across byte boundaries into a cleared frame;
5. compute the CRC last, over bytes 0–6.

Why a table? Hand-written shifts duplicate sign extension, offset and cross-byte logic for
every signal, and those are the classic CAN bugs. The table is tested once and reused for
every message. Why not a struct overlay? Alignment, packing, endianness and strict aliasing
make it undefined behaviour; see the review in §10.

**Vertical signals**: `PackVoltage` 0.01 V/bit, `PackCurrent` 0.1 A/bit signed (positive =
charge), `SOC` 0.5 %/bit, `CCL`/`DCL` 0.1 A/bit, temperatures 1 °C/bit with −40 offset.

### 8.3 VCU command supervision

The receiver rejects, separately counted:

* wrong DLC;
* wrong CRC;
* an unchanged 4-bit alive counter;
* an alive jump > 2 (mod 16).

Three consecutive invalid frames are treated as a timeout. A single lost frame does not trip
it; a stuck or spoofed counter does. A timeout moves the state machine to STANDBY and opens
after 1 s, and raises DTC 0x000A09.

### 8.4 UDS subset

| Service | Sub / DID | Response | Notes |
|---|---|---|---|
| 0x10 | 01 default, 02 extended | 0x50 | extended S3 = 5 s |
| 0x3E | 00, 80 | 0x7E | 0x80 suppresses the positive response |
| 0x22 | 0xF189 | software version ASCII | `BMS-FW-1.0.0` |
| 0x22 | 0x0101 | pack mV (2 B) + current mA (4 B signed) | |
| 0x22 | 0x0100 | 16 × 2 B cell mV = 32 B | requires ISO-TP multi-frame |
| 0x2E | 0x0200 / 0x0201 | 0x6E | balancing start/stop; extended only; 5–50 mV; stop < start |
| 0x19 | 02 | 0x59 + availability + DTC/status records | mask-filtered |
| 0x14 | 0xFFFFFF | 0x54 | other groups → NRC 0x31 |

NRCs: 0x11 service not supported, 0x12 subfunction not supported, 0x13 incorrect length,
0x22 conditions not correct, 0x31 request out of range, 0x7F service not supported in
session.

### 8.5 ISO-TP multi-frame sequence

The BMS is the responder. For DID 0x0100 (32 bytes):

```text
tester → BMS   0x7E0  SF  [02 22 01 00 ...]        request
BMS → tester   0x7E8  FF  [10 24 62 01 00 ...]     32-byte length, first 6 bytes
tester → BMS   0x7E8  FC  [30 00 00 ...]           continue, block size, STmin
BMS → tester   0x7E8  CF  [21 ...]                 7 bytes
BMS → tester   0x7E8  CF  [22 ...]                 ...
```

Block size and STmin are honoured on transmit; the sender waits for FC between blocks and
aborts after the FC timeout. Only one reception and one transmission are active at a time,
which is sufficient for a request/response server. No CAN-FD, no extended addressing.

### 8.6 DBC

`dbc/bms.dbc` contains every message and signal, value tables for BMS state and VCU commands,
cycle-time attributes, comments documenting CRC/alive and scaling, and multiplexed cell
signals for all six mux values. The six mux copies exist because classic DBC assigns one mux
value per signal; duplicating the three-signal layout is the portable way to let generic
tools decode every frame.

---

## 9. Task 6 — tests, simulation, static analysis, coverage

### 9.1 Test strategy

| Layer | How it is tested | Examples |
|---|---|---|
| Common | pure functions | CRC vectors, saturation boundaries |
| HAL | simulator contract | time, SPI, contactors, NVM |
| AFE | register model + synthetic time | ID, CRC retry, timeout, snapshot |
| Balancing | table-driven inputs | S1–S10 + 7 cases |
| Protection | synthetic time steps | set, reset, release, clear, hard open |
| CAN | round-trip pack/unpack | scale, saturation, CRC, alive, queue |
| UDS/ISO-TP | byte-level requests | sessions, DIDs, NRCs, FC/CF, block boundary |
| SOC | synthetic current/rest | counting, offset, flat/edge correction |
| State machine | direct input records | every transition, precharge, fault hold |
| NVM | A/B slot corruption | valid, corrupt, blank, sequence, CRC |

88 tests, 0 failures. `ctest` runs the same binary.

### 9.2 Coverage

Built with `-DBMS_COVERAGE=ON` (`--coverage -O0`), run the full suite, then `gcov -b -c`:

| Module | Lines | Branches executed | Taken at least once |
|---|---:|---:|---:|
| `balancing.c` | 100.00 % (168) | 100.00 % (102) | **96.08 %** (102) |
| `protection.c` | 97.30 % (259) | 100.00 % (174) | **91.38 %** (174) |

The reported branch metric is the strict one: the fraction of branch **outcomes** executed at
least once. "Branches executed" only counts branches reached, so it is weaker; both numbers
are reported to avoid ambiguity. Both modules exceed the 85 % target. The unexecuted
protection lines are defensive paths that need corrupted internal state and are unreachable
through the public API.

### 9.3 Simulation

Two CSV scenarios run the real application modules against the simulated HAL.

**`charge_balance.csv` (260 s)** — charge current, one weak cell, balancing enabled:
observed cell 5 balancing mask from ≈20.1 s to 140 s; cell 5 converges toward the pack
minimum; SOC rises; CCL derates near the top.

**`fault_recovery.csv` (100 s)** — one cell crosses the OV threshold at ≈25 s:
fault trips at **31.1 s** (1 s debounce + 10 ms task), contactors open after the 1 s hold,
DTC latches, clear request at 70 s releases it and the system returns to STANDBY.

The trace columns are `time_ms,state,soc_permille,current_ma,cell0_mv..cell4_mv,temp0_dc,
bal_mask,bal_inhibit,ccl_ma,dcl_ma,neg,pre,pos,fault_bits`. Five SVG plots are generated:

* `charge_balance_voltage.svg` — cell 1 vs cell 5;
* `charge_balance_balance.svg` — mask over time;
* `charge_balance_soc.svg` — SOC over time;
* `fault_recovery_voltage.svg` — faulted cell;
* `fault_recovery_contacts.svg` — contactors and fault bits.

Why a custom SVG plotter? gnuplot and Python were not available; a 300-line C tool with fixed
buffers keeps the toolchain to the same compiler, uses no allocation, and produces artefacts
that can be committed and opened in any browser.

### 9.4 Static analysis

```text
cppcheck --enable=warning,style,performance,portability --std=c11 \
         --suppress=missingIncludeSystem --suppress=unusedFunction \
         -I src src
```

Cppcheck 2.22.0, 18 translation units, **no findings**. The two suppressions are justified:
the target SDK headers are not installed, and public service APIs are consumed by tests and
the simulator rather than from within their own file. clang-tidy and the MISRA add-on were
not available; that is stated rather than papered over.

### 9.5 What the evidence proves

* The mandatory scenarios are encoded as tests, not prose.
* Branch coverage above 85 % is measured with the stricter "taken" metric.
* No static-analysis findings in the shipped firmware sources.
* The strict warning set passes with `-Werror`.
* The simulation demonstrates the two end-to-end behaviours a reviewer cares about:
  balancing actually converges, and a fault actually trips, latches, clears and recovers.

---

## 10. Task 7 — code review

The full review is in [`CODE_REVIEW.md`](../CODE_REVIEW.md). It covers the three listings in
the brief and reports **47 findings: 19 critical, 17 major, 10 minor, 1 nit.**

The five root causes:

1. **No ISR/task ownership boundary.** The supplied handler parses a 64-byte frame, calls
   `log()`, opens contactors, sets DTCs, writes SPI and sends CAN inside the SPI interrupt.
   The corrected design moves bytes only in the ISR and runs a state machine in the 1 ms
   task.
2. **No error path.** Unchecked reads, ignored return values, no conversion timeout, no retry
   limit. The corrected AFE has a deadline, a retry counter and a latched comm fault.
3. **No safety policy in the algorithm.** The supplied balancing function has no
   measurement-window, temperature, fault, VCU or discharge gate, and no max-cell or timeout
   limit. The corrected `Bal_Run` makes every gate explicit.
4. **The wire contract is duplicated by hand.** The supplied CAN function writes millivolts
   and amps into signals scaled as 0.01 V and 0.1 A, omits the CRC, and puts the alive
   counter in the wrong byte. The corrected signal table derives layout and scale from one
   definition, and the DBC is the external check.
5. **Undefined behaviour.** `i <= 16`, `1 << 16`, a signed right shift, and a struct overlay
   on a byte buffer. The corrected code uses bounded loops, unsigned masks, explicit
   two's-complement conversion and bit-level packing.

The review verdict is **changes required**, with six acceptance criteria: no parsing in the
ISR; no volatile array treated as atomic; every loop bounded and every blocking call timed;
every signal decoded by a DBC-derived test; every fault tested for set/reset/release/clear;
strict build + cppcheck + coverage. All six are met by the delivered tree.

---

## 11. Task 8 — documentation and code quality

### 11.1 Documentation set

| File | Purpose |
|---|---|
| `README.md` | one-command build, layout, design summary, SOC/voltage comparison, traceability, AI disclosure, licences |
| `docs/DESIGN.md` | Task 1 detail, task table, state machine, fault table, alternatives log, limitations |
| `ASSUMPTIONS.md` | A1–J4: platform, AFE, state machine, balancing, protection, CAN/UDS, SOC, NVM, tooling, safety |
| `CODE_REVIEW.md` | Task 7: findings, severity, fixes, corrected code, re-review criteria |
| `docs/coverage.txt` | exact commands and gcov results |
| `docs/static_analysis.txt` | exact cppcheck command, result, suppressions, manual review notes |
| `dbc/bms.dbc` | external CAN contract |
| this document | the single end-to-end answer |

### 11.2 Code-quality rules applied

* C11, `stdint.h`/`stdbool.h` only; no compiler extensions.
* No dynamic allocation, no recursion, no variable-length arrays.
* Static file-scope state; public functions have prototypes; internal helpers are `static`.
* Narrow interfaces; no module reaches into another's state.
* Explicit casts and saturation at every boundary; no implicit narrowing.
* `const` on inputs; pointer and length validation at public entry points.
* Comments explain *why* (timing, safety, trade-off), not *what*.
* Build flags: `-Wall -Wextra -Werror -Wpedantic -Wshadow -Wstrict-prototypes
  -Wmissing-prototypes -Wdouble-promotion -Wformat=2 -Wundef -Wcast-qual -Wpointer-arith`.

### 11.3 AI-tool disclosure

AI assistance was used, as permitted. The layered skeleton, portions of the drivers and
services, Unity scaffolding, DBC, simulation runner, SVG plotter and first drafts of the
Markdown were drafted with an AI assistant. Every file was compiled under the strict warning
set, run through the tests and simulator, and reviewed line by line; defects found during
that review (ISO-TP block boundary, immediate-open on fault entry, DTC/ISO-TP test
expectations) were fixed in code and tests. No AI runs at firmware runtime, and no generated
code is linked without review. The complete rationale for every non-obvious choice is in the
alternatives log so each decision can be defended or changed in the review.

---

## 12. Bonus B1 — SOC estimation

### 12.1 Algorithm

Runs every 10 ms:

```text
I_corr = I_measured - I_offset
if I_corr > 0: I_eff = I_corr × η        # η = 0.990, charge only
else:         I_eff = I_corr

ΔQ_uAh = I_eff × Δt_ms / 3600           # 1 uAh = 3600 mA·ms
SOC    = clamp(SOC + ΔQ / (C × 1000) × 1000, 0, 1000)   # permille
```

Charge is accumulated in micro-amp-hours with a 64-bit remainder so a 10 ms step does not
lose precision through repeated integer division. A second remainder keeps the SOC update
exact.

**Offset learning:** while `|I| < 0.5 A`, average the current for 30 s and store the average
as the offset (clamped to ±2 A). This removes a persistent sensor bias at rest.

**OCV correction:** after 1 h at rest (`|I| < 0.5 A`), convert the cell OCV through the
supplied table (0/10/20/30/50/70/90/95/100 % → 2800/3200/3250/3280/3300/3320/3340/3380/3450
mV) with linear interpolation, and move SOC toward it. The weight is **5 %** between 3.28 V
and 3.34 V (the flat LFP region) and **25 %** outside it. The correction is applied once per
rest period; a 5 ‰ dead band prevents pointless updates.

### 12.2 Why this design

| Option | Why not |
|---|---|
| Coulomb counting only | Drifts with offset and efficiency error; no absolute anchor |
| OCV only | Unusable under load; LFP OCV is flat across most of the range |
| EKF | Needs a cell model, tuning, matrix maths and floating point; not justified for one pack current sensor |
| Coulomb + offset + weighted OCV (chosen) | Fixed point, bounded, uses the supplied table, and explicitly handles the flat region |

**Persistence:** SOC is written to NVM every 60 s and on parameter changes, not on every
sample, to bound flash wear. A power loss can lose up to 60 s of drift (≈0.017 % of 100 Ah at
100 A).

**Limitation:** the OCV correction assumes the pack is at rest and uses the minimum cell
voltage supplied by the application; it does not model hysteresis or temperature dependence.
Those are documented as the next refinements.

---

## 13. Bonus B2 — parameter storage and firmware update

### 13.1 Parameter storage (implemented)

Record layout per slot:

```text
offset  size  field
0       4     magic 'BMSN' (0x424D534E)
4       2     format version (1)
6       4     sequence number
10      2     payload length
12      N     payload (NvmParams_t: bal_start, bal_stop, soc_permille, flags)
12+N    2     CRC-16 over the header + payload
```

Algorithm:

1. Load both slots; validate magic, version, length ≤ 64 and CRC.
2. Choose the valid slot with the highest sequence.
3. If both are invalid/blank, return defaults and report "no valid slot".
4. Save: write the **older** slot with `sequence + 1`, then verify by reading back.
5. A power loss during a write leaves the other slot valid, so the previous parameters are
   never lost.

Why A/B and not a single record? A single record is destroyed by a power loss mid-write. Why
not wear-levelled log? It is the right production answer for frequent writes, but two
parameters do not justify it; the API is compatible with a future log backend. Endurance is
2× the slot rating.

### 13.2 Firmware update (design only, as permitted)

Bootloader flow over CAN using UDS:

```text
0x10 02   enter programming session
0x27      security access (seed/key)
0x34      request download (address, size)
0x36      transfer data (blocks)
0x37      request transfer exit
0x31 01   check integrity (hash/signature)
0x31 01   activate new image
```

A/B image layout:

```text
┌──────────────┬──────────────┬──────────────────────┐
│ Bootloader   │ Image A      │ Image B              │
│ (immutable)  │ + manifest   │ + manifest           │
└──────────────┴──────────────┴──────────────────────┘
manifest: version, length, SHA-256, ECDSA/RSA signature, confirmed flag
```

Recovery rules:

* The bootloader always boots the **last confirmed** image.
* A new image is confirmed only after the application passes its self-test and writes the
  confirm flag.
* Power loss during download: the old image remains valid.
* Power loss during confirmation: the bootloader falls back to the old image.
* Signature failure: the image is never booted; the old image stays.

The application and bootloader share the CAN/ISO-TP/UDS transport, not the safety logic.
This is a design, not shipped code — the assignment marks B2 design-only, and untested boot
code would be worse than a clear design.

---

## 14. Every alternative, and why it was not chosen

This is the table to use when the reviewer asks "why didn't you just…".

| Area | Options considered | Chosen | Why the others were rejected |
|---|---|---|---|
| Scheduling | bare super-loop with delays; event loop with flags; cooperative timer table; preemptive RTOS; work in a timer ISR; static time-triggered table | **Cooperative timer-driven table: 1 ms quantum, 10 ms fast, 100 ms main** | Delays and ISR work destroy jitter and WCET evidence; an event loop hides priority and deadline proof; an RTOS adds footprint, priority-inversion analysis and host-reproducibility loss for five fixed jobs; a static schedule table is the same order with more machinery |
| AFE acquisition | blocking wait for conversion; one interrupt per register; one state-machine step per tick | **Non-blocking state machine, one SPI transaction per tick** | Blocking violates the requirement; per-register interrupts create ISR load and shared-state complexity; the state machine has a bounded step, a natural retry point and a deadline |
| NTC conversion | Beta equation every sample; Steinhart–Hart; lookup table with interpolation | **Lookup table + linear interpolation** | `log`/`exp` cost float or expensive fixed point per channel per cycle; the table is deterministic, FPU-free and accurate enough over the automotive range; cost is a small const table |
| Snapshot consistency | plain volatile array; mutex; seqlock; double buffer | **Double buffer + short critical-section index swap** | A volatile array tears between fields; a mutex can block and needs an RTOS; a seqlock is correct but harder to review; double buffering is allocation-free and gives one coherent 16-cell set |
| Balancing criterion | voltage-based; SOC-based; OCV-based; capacity-based | **Voltage-based with top-of-charge gate** | Only per-cell voltage is measured; SOC-based needs per-cell SOC/capacity that the BOM cannot provide; LFP voltage is a usable proxy above 3.4 V |
| Starvation prevention | fixed odd/even; round-robin; random; priority for the cell skipped last cycle | **Priority cell within a 20 mV fairness band** | Fixed alternation can starve a non-adjacent cell and ignores ranking; random is not deterministic; the priority rule is deterministic, bounded and targets the cell that missed out |
| Candidate ordering | bubble/insertion sort; selection sort; `qsort`; bucket by voltage | **Insertion sort, n ≤ 16** | 120 comparisons worst case, no allocation, no recursion, stable, easy to reason about; `qsort` adds a function pointer and a library dependency for no benefit |
| Protection structure | one `if` per fault; one function per fault; table-driven `FaultDef_t` | **Table + generic debounce/DTC object** | Copying debounce 11 times invites divergence; the table makes thresholds/actions/release data and one state machine executes them |
| Debounce representation | timestamp `now - start`; incrementing counter; decrementing counter; accumulated timer | **Accumulated timer with `dt_ms`** | A timestamp is sensitive to wrap and missed calls; the accumulator is explicit, works with variable steps and is easy to test |
| CCL/DCL derivation | fixed limits; one temperature curve; piecewise-linear temperature + voltage derating | **Piecewise-linear tables + 100 mA/ms ramp** | Fixed limits ignore temperature and voltage; one curve cannot represent charge and discharge; tables are reviewable and the ramp prevents steps |
| CAN packing | hand-written shifts; macro per signal; table-driven descriptor | **Table-driven descriptor with num/den scaling, offset, saturation, Intel packing** | Hand-written shifts duplicate sign/offset/cross-byte logic; macros are hard to debug; the table is tested once and reused |
| CAN Tx blocking | wait for mailbox; blocking send; queue with bounded drain | **8-entry queue, ≤4 per poll, drop-newest** | Blocking violates the 100 ms budget; an unbounded queue needs allocation; drop-newest plus a counter is deterministic and periodic frames recover |
| ISO-TP | full ISO 15765-2; single-frame only; minimal FF/FC/CF | **Minimal multi-frame with FC** | A full stack is unnecessary for two DIDs; single-frame only violates the 0x0100 requirement; the minimal stack is auditable and testable; no CAN-FD/extended addressing |
| UDS scope | full ISO 14229; required subset with hooks | **Required subset + DID/DTC hooks** | A full server is a large surface with little assessment value; the subset covers every requested service and keeps NRCs explicit |
| State machine | free-form `if` chain; function-pointer table; `switch` + entry helper | **`switch` + `EnterState()`** | The graph is small and mostly conditional; a table would need parameters/callbacks that obscure precharge timing; the switch is readable and directly testable |
| HAL binding | compile-time `#ifdef`; weak symbols; function-pointer ops table | **`HalOps_t` installed at start-up** | `#ifdef` makes dual builds hard; weak symbols are toolchain-specific; the ops table gives one explicit injection point and NULL-safe wrappers |
| SOC | coulomb only; OCV only; EKF; coulomb + offset + weighted OCV | **Coulomb + offset learning + weighted OCV** | Counting drifts; OCV is unusable under load and flat; an EKF needs a model/tuning/floating point; the chosen method meets B1 in fixed point and handles the flat region |
| NVM | single record; A/B ping-pong; log-structured wear levelling; external EEPROM | **A/B with magic/version/sequence/CRC** | A single record loses the old value on power loss; log-structured is the production answer but overkill for two parameters; A/B doubles endurance and always leaves one valid record |
| Test framework | Unity; CppUTest; GoogleTest; custom asserts | **Unity (MIT), vendored** | Tiny, C99, allocation-free runner, offline; GoogleTest is C++ and heavy; CppUTest adds infrastructure |
| Coverage tooling | gcov+lcov; gcovr; clang source-based | **gcov directly** | lcov/gcovr not installed; `gcov -b -c` reports the same line/branch data and the `.gcov` files are the evidence |
| Plotting | gnuplot; Python/matplotlib; Excel; custom SVG | **`tools/plot_svg.c`** | gnuplot/Python unavailable; a small C tool keeps one compiler, no allocation, and produces committable browser-viewable artefacts |
| Build system | Make; CMake; Meson; scripts | **CMake + Ninja + GCC** | Portable across Windows/Linux/macOS, per-target warning flags, `simulate`/`check`/coverage options without shell-specific scripts |
| Firmware update | single-bank bootloader; A/B banks; external flash staging | **Design-only A/B with signature and power-loss recovery** | B2 is design-only; implementing a bootloader without a target adds untestable code; the design covers signature, confirmation and rollback |
| Parameter storage | RAM only; single flash record; A/B with CRC; wear-levelled log | **A/B with magic/version/sequence/CRC** | Meets versioning, CRC, default fallback and power-loss safety; wear levelling is the documented production extension |

---

## 15. Requirement traceability

| Requirement | Implementation | Evidence |
|---|---|---|
| Task 1 architecture, scheduling, state machine, data sharing, safety | `src/app/`, `src/services/`, `docs/DESIGN.md` | design doc, 88 tests, simulation |
| Task 2 AFE driver, CRC, retries, plausibility, snapshot | `src/drivers/afe.c`, `src/hal/afe_sim.c` | `test/test_afe.c` (9 tests) |
| Task 3 balancing, S1–S10, fairness, inhibits, timeout | `src/services/balancing.c` | `test/test_balancing.c` (17 tests), 96.08 % branches |
| Task 4 protection table, debounce, DTCs, CCL/DCL ramp | `src/services/protection.c` | `test/test_protection.c` (10 tests), 91.38 % branches |
| Task 5 CAN matrix, signal layer, DBC, UDS, ISO-TP | `src/services/can_*.c`, `uds.c`, `isotp.c`, `dbc/bms.dbc` | `test/test_can.c`, `test/test_uds.c` |
| Task 6 unit tests, simulation, static analysis, coverage | `test/`, `sim/`, `docs/` | 88 tests, 2 scenarios, 5 SVGs, cppcheck clean |
| Task 7 code review | `CODE_REVIEW.md` | 47 findings, corrected code |
| Task 8 documentation and quality | all `.md`, headers, strict build | one-command build, zero warnings |
| B1 SOC | `src/services/soc.c` | `test/test_soc.c`, flat/edge weights |
| B2 parameter storage | `src/services/nvm.c` | `test/test_nvm.c`, A/B/CRC/fallback |
| B2 firmware update | `docs/DESIGN.md` §12, this document §13.2 | bootloader design with A/B, signature, recovery |

---

## 16. Review-session speaking notes

### 16.1 The two-minute answer

> "It's a 16-cell LFP BMS. The firmware uses a 1 ms tick with a 10 ms fast task and a 100 ms
> main task: the AFE state machine and CAN drain step at 1 ms, current, over-current and
> protection at 10 ms, balancing and CAN Tx at 100 ms, statistics at 500 ms, and slow frames
> at 1 s. The AFE is a non-blocking state machine that
> does one SPI transaction per tick, so a full 21-channel scan takes about 22 ms without
> blocking. Everything safety-relevant is table-driven: eleven faults with debounce, latch,
> release and DTC status; CAN signals with scale, offset and CRC; balancing thresholds and
> inhibits. The state machine handles precharge, drive, charge, fault and sleep, holds
> contactors for a second before opening unless it's an over-current or hard OV/UV, and
> treats a lost VCU as STANDBY rather than a latched fault. Shared data is double-buffered
> with a short critical section. There are 88 tests, 96 % branch coverage on balancing and
> 91 % on protection, cppcheck is clean, and two simulation scenarios with plots show
> balancing converging and a fault tripping, latching, clearing and recovering."

Then stop. Let them pick a thread.

### 16.2 The ten-minute whiteboard flow

1. **Draw four layers** (app → services → drivers → HAL) and say the dependency rule.
2. **Draw the scheduler table** (five rows, period, WCET) and the 8.5 % total. Mention the
   overrun policy in one sentence.
3. **Draw the state machine** and the precharge timeline (NEG → 20 ms → PRE → 95 %/2 s →
   POS → 20 ms → PRE off).
4. **Draw the fault path**: table → debounce → latch → open request → 1 s hold / immediate.
5. **Draw the AFE state machine** and the double-buffer swap.
6. **Draw the CAN stack**: signals → frame → queue → mailbox, plus CRC/alive.
7. **Draw the UDS path**: tester → ISO-TP SF/FF/FC/CF → UDS → DID → response.
8. **Close with evidence**: 88 tests, coverage, cppcheck, two traces.

### 16.3 What to emphasise

* Every claim is backed by a file, test or trace.
* The safe state is the default; unsafe action requires a positive command.
* The design is deliberately boring: fixed tables, fixed buffers, no dynamic behaviour.
* The alternatives were considered, not ignored; §14 has the list.
* The limitations are stated before the reviewer finds them.

### 16.4 Traps to avoid

* Do not claim ISO 26262 compliance. Claim a safety mindset and name what is out of scope.
* Do not claim target timing measurements. Say they are conservative estimates and describe
  the GPIO/WCET measurement you would do next.
* Do not hide the NVM write latency. Name it, quantify it, and give the HAL-level fix.
* Do not say "the RTOS would be better". Say why five fixed jobs do not need one, and what
  would change your mind (blocking drivers, many priorities, dynamic workloads).
* Do not defend a bug in the supplied listings. The review is supposed to find them.

---

## 17. Likely questions and model answers

**Q1. Why no RTOS?**
Five fixed periodic jobs with known WCET. A cooperative table gives deterministic order,
bounded work per tick, an overrun counter and a host build that behaves like the target. An
RTOS adds RAM/flash, per-task stacks, priority-inversion analysis and non-reproducible host
timing without adding functionality. I would move to an RTOS if a driver needed to block, if
there were many asynchronous priorities, or if a task had to preempt another.

**Q2. What happens if a task overruns?**
It runs once, the deadline is re-based to `now + period`, and the overrun counter increments.
No catch-up. That bounds the work in the next tick and turns a missed deadline into
telemetry instead of a latency burst.

**Q3. How do you know the 1 ms tick and the 10 ms fast task fit?**
The 1 ms tick is one bounded SPI step plus CAN drain: a 3–5 byte transfer at 1 MHz plus driver
overhead is 30–50 µs, so 80 µs is a conservative bound (8 % if charged to every tick). The AFE
is active only ~22 ms per 100 ms, so the average is closer to 1–2 %. The 10 ms fast task is
~40 µs (0.4 %). On target I would toggle a GPIO at task entry/exit and measure with a scope or
DWT cycle counter; that is explicitly the next step because there is no hardware here.

**Q4. Why one SPI transaction per tick instead of reading all cells at once?**
A single blocking burst would violate the "do not block while waiting for conversion"
requirement and make the tick's WCET depend on the AFE. One transaction per tick gives
a bounded step, a natural retry point, and a 22 ms scan that is still far inside the 100 ms
control cycle.

**Q5. Why double buffering instead of a mutex?**
No RTOS and no blocking. A mutex would block the producer or consumer and needs priority
inheritance. Double buffering gives a coherent set with a critical section of a few dozen
bytes, and no SPI runs inside it.

**Q6. Why not a seqlock?**
It is correct, but it needs atomic sequence handling and careful retry logic. Double
buffering is simpler to review and has a fixed, tiny critical section. For 16 cells the copy
cost is irrelevant.

**Q7. How do you prevent torn 16-bit reads?**
I do not rely on the width. The whole snapshot is copied under a critical section, so the
consumer sees one complete conversion. Individual 16-bit reads being atomic on Cortex-M is
not the argument.

**Q8. Explain balancing in 30 seconds.**
Gate on valid measurement, no fault, VCU enable, temperature, not discharging. Require
`Vmax ≥ 3.4 V`. A cell starts bleeding above 15 mV over the minimum and continues until 5 mV.
Sort by voltage, cap at 8 cells, reject adjacent pairs, and give the first skipped cell
priority next cycle if it is within 20 mV of the maximum. Stop after 8 h and require a charge
cycle to clear.

**Q9. Why voltage-based, not SOC-based?**
LFP OCV is flat from about 30 % to 90 % SOC, so mid-range voltage differences say little. At
the top of charge the curve steepens and voltage is a usable proxy. The BOM has one pack
current sensor, not per-cell current, so per-cell SOC would be a model guess. Voltage-based
top-balancing is the standard choice for passive bleed.

**Q10. How do you prevent two adjacent cells from bleeding?**
`Adjacent(mask) = (mask & (mask << 1)) != 0`. Candidates are added one at a time; a candidate
that would create an adjacent pair is skipped and becomes the starvation priority.

**Q11. What if nine cells qualify?**
The first eight non-adjacent cells are selected; the first rejected cell is remembered. With
16 cells and no adjacency the maximum is eight, so the limit and the adjacency rule agree.

**Q12. How do you prevent starvation?**
Remember the first cell dropped by the max-cell or adjacency rule. Next cycle, if it is
within 20 mV of `Vmax`, sort it first. A much higher cell still wins, so fairness never
overrides a real divergence.

**Q13. Why 15/5 mV?**
Enough hysteresis to stop chatter from bleed-current IR drop and measurement noise, small
enough to converge in a reasonable time. It is calibratable in `BalConfig_t` and validated
(5–50 mV start, stop < start).

**Q14. Why an 8 h timeout?**
Continuous balancing for days indicates a cell that will not converge, a measurement offset,
or a stuck switch. Stopping and warning is safer than continuing. A charge cycle is the
natural proof that the pack has been cycled and the warning can clear.

**Q15. How does the measurement window work?**
The application forces the mask to zero before starting an AFE conversion and applies the new
mask only after a valid snapshot is published. That removes the bleed current's IR drop from
the measurement. Cost: ≈22 ms per 100 ms, so ≈78 % maximum duty.

**Q16. Walk me through the protection debounce.**
Each fault has a set condition, a release condition, a debounce, a latch flag and an action
mask. While set, an accumulator grows by `dt`; when it reaches the debounce the fault becomes
active and latches if configured. A single false sample resets the accumulator. Release is
evaluated even while active; a latched fault clears only when release is true and a clear
request arrives.

**Q17. Why table-driven protection?**
Eleven copies of the same debounce/status logic diverge. The table makes thresholds, actions,
release conditions and DTCs data; one state machine executes them. Adding a fault is a row
plus a condition pair, and the status logic is tested once.

**Q18. How do you avoid opening contactors under load?**
Non-urgent faults request an open but the state machine holds for 1 s so the VCU can reduce
current. Over-current, hard OV (>3750 mV), hard UV (≤2300 mV) and precharge failure request
an immediate open, independent of the debounce. That is the explicit exception in the
requirement.

**Q19. Why is VCU timeout not a FAULT?**
The assignment says "go to STANDBY, open after 1 s". A lost VCU is a communication loss, not
a pack hazard. It is still a DTC and still opens the contactors after 1 s, but it does not
latch the hardware fault state. That lets the system recover when the VCU returns.

**Q20. Explain the DTC clear semantics.**
A clear request alone does nothing if the release condition is false. When release is true,
the active/latch state clears and the status records "test not completed since clear" /
"failed since clear" as appropriate. The DTC remains visible to 0x19 until it is retested, so
the tester can see the history rather than the record vanishing.

**Q21. How are CCL and DCL derived?**
Take the minimum of the temperature derating and the voltage derating, then ramp at
100 mA/ms. Charge: 0 below 0 °C, 50 A at 5 °C, 100 A to 45 °C, 0 at 55 °C. Discharge: 0 below
−20 °C, 100 A at 0 °C, 200 A to 45 °C, 0 at 60 °C. Voltage tapers: 100→0 A from 3550→3650 mV
charge, 0→200 A from 2500→3000 mV discharge. Both start at 0 at boot.

**Q22. Why CRC-8 SAE J1850 and a 4-bit alive counter?**
CRC-8 catches bus/EMC corruption on safety-relevant frames; the alive counter catches a
stuck or replayed transmitter. Four bits fit the byte layout and wrap 15→0; the receiver
rejects repeats and jumps >2. 0x352–0x354 are low-rate informative frames and deliberately
carry no CRC, as specified.

**Q23. How does UDS 0x0100 fit in 8 bytes?**
It does not, so ISO-TP multi-frame is used: First Frame with the 32-byte length, tester Flow
Control, then Consecutive Frames. Block size and STmin are honoured; a missing FC aborts
after the timeout. The BMS is the responder; the tester sends the FC.

**Q24. Why does SOC correct slowly in the flat region?**
Because a few millivolts there correspond to tens of percent SOC. A 25 % correction on a
3.30 V reading could jump SOC by 20 points for a tiny measurement error. The 5 % weight
between 3.28 V and 3.34 V makes the correction slow and stable; on the steep edges 25 % is
safe because the voltage-to-SOC slope is high.

**Q25. How is NVM power-loss safe?**
Two slots, each with magic, version, sequence, length and CRC-16. Save writes the older slot
with sequence+1. A power loss mid-write leaves the other slot valid. On boot, the newest
valid record wins; if both are corrupt, defaults are used and reported.

**Q26. What does the bootloader do on a bad signature or a power loss?**
It never boots an unconfirmed or invalid image. The last confirmed image stays valid. A new
image is confirmed only after the application self-test writes the confirm flag. Power loss
during download or confirmation leaves the old image bootable.

**Q27. What is the biggest risk in this design?**
The NVM write latency on target, because it is the one potentially unbounded operation inside
a periodic task. It is isolated behind the HAL, quantified, and has a clear fix (non-blocking
page buffer or move to the 1 s task). The second is the lack of target timing measurements;
the 8.5 % load is an estimate, and the next step is GPIO/WCET measurement.

**Q28. What would you do with another week?**
Measure WCET on target, make NVM non-blocking, add cell open-wire localisation, add a
GitHub Actions CI file, run a MISRA check with a deviation list, add temperature-dependent
OCV/hysteresis to SOC, and implement the bootloader if hardware were available.

**Q29. How was AI used, and how do you know the code is correct?**
It drafted skeleton and implementation material, which I reviewed line by line, compiled
under `-Werror`, exercised with 88 tests, two simulations and cppcheck. Defects found during
that review were fixed in code and tests. The design rationale is written down so I can
defend or change every decision. No AI runs at runtime and no generated code is linked
unreviewed.

**Q30. If I change the cell count to 12, what breaks?**
`AFE_CELL_COUNT`, `BAL_CELL_COUNT`, `PROT_CELL_COUNT`, the CAN mux count and the DBC need to
change together; the code is table/count driven, so the algorithms are unchanged. The tests
use the count constant. I would add a compile-time assertion that the three counts agree and
that the mux count covers `ceil(N/3)`. That is the honest answer: the design is count-driven
but the constants are not yet centralised in one header.

---

## 18. The three-day plan

### Day 1 — architecture and acquisition

* Decide scheduling, layering, HAL injection, state machine and fault policy.
* Implement common (CRC, saturation), HAL simulator, AFE register model and driver.
* Bring up the scheduler and the state machine with synthetic inputs.
* Write the AFE and HAL tests. First full build with `-Werror`.

### Day 2 — safety and communication

* Balancing: gates, hysteresis, adjacency, max cells, priority, timeout, measurement window.
* Protection: `FaultDef_t`, debounce/DTC object, derating, ramp, hard thresholds.
* CAN signal layer, CAN matrix, DBC, VCU monitor, Tx queue.
* ISO-TP and UDS subset; unit tests for every service.
* Start coverage and close the gaps.

### Day 3 — evidence and documentation

* Simulation scenarios, trace schema, SVG plotter, run and inspect the traces.
* Coverage to target, cppcheck, strict build, stack measurement.
* README, DESIGN, ASSUMPTIONS, CODE_REVIEW, coverage/static-analysis reports.
* Final pass: re-read every file, check requirement traceability, verify one-command build.

The plan deliberately front-loads the architecture because a bad layering decision on day 3
cannot be fixed in the time left. Bonuses were implemented after the mandatory paths were
green, not before.

---

## 19. Limitations and production roadmap

| # | Limitation | Why it exists | Production fix |
|---|---|---|---|
| 1 | No target timing measurements | No hardware in the environment | Toggle GPIO at task entry/exit; measure WCET and stack high-water on the MCU |
| 2 | NVM write can block up to tens of ms on target | PC HAL is a memory copy; target flash timing not measured | Page-buffered/non-blocking HAL, or defer saves to STANDBY |
| 3 | No MISRA add-on or clang-tidy | Not installed | Run both; produce a deviation list; keep the strict warning set |
| 4 | Cell open-wire not localised | AFE reports a single flag | Read the AFE open-wire diagnostic register and map to a cell |
| 5 | ISO-TP has no CAN-FD/extended addressing | 7-byte classic CAN is the requirement | Add CAN-FD and addressing modes if the harness requires them |
| 6 | Bootloader is design-only | B2 marks it design-only | Implement A/B with signature and confirmation on hardware |
| 7 | No CI file | No Git/hosted runner in the environment | GitHub Actions running configure, build, ctest, coverage, cppcheck |
| 8 | SOC has no temperature/hysteresis model | Scope and one pack current sensor | Add temperature-dependent OCV and charge/discharge hysteresis |
| 9 | Parameter storage has no wear levelling | Two parameters, A/B is sufficient | Log-structured region with erase counters behind the same API |
| 10 | Packs count constants are duplicated | Time; three modules own their counts | One `bms_config.h` with compile-time assertions tying counts to the DBC |

---

## 20. Appendix

### 20.1 File index (what to open first)

| Question | File |
|---|---|
| How do I build and run it? | `README.md` §1 |
| What is the architecture and why? | `docs/DESIGN.md` §2–§7 |
| What did you assume? | `ASSUMPTIONS.md` |
| How does balancing work? | `src/services/balancing.c`, `test/test_balancing.c` |
| How does protection work? | `src/services/protection.c`, `test/test_protection.c` |
| How does the state machine work? | `src/app/bms_state.c`, `test/test_state.c` |
| How is the AFE non-blocking? | `src/drivers/afe.c`, `test/test_afe.c` |
| How is CAN packed? | `src/services/can_signal.c`, `can_matrix.c`, `dbc/bms.dbc` |
| How does UDS work? | `src/services/uds.c`, `isotp.c`, `test/test_uds.c` |
| What does the review find? | `CODE_REVIEW.md` |
| What is the evidence? | `docs/coverage.txt`, `docs/static_analysis.txt`, `sim/out/` |
| What are the alternatives? | §14 of this document, `docs/DESIGN.md` §7 |

### 20.2 DTC map

| DTC | Fault | Primary reaction |
|---|---|---|
| 0x000A01 | Cell over-voltage | CCL 0; open >3750 mV immediately |
| 0x000A02 | Cell under-voltage | DCL 0; open ≤2300 mV immediately |
| 0x000A03 | Charge over-temperature | CCL 0 |
| 0x000A04 | Discharge over-temperature | open |
| 0x000A05 | Charge under-temperature | CCL 0 |
| 0x000A06 | Discharge over-current | open immediately |
| 0x000A07 | Charge over-current | open immediately |
| 0x000A08 | AFE communication | open, latched |
| 0x000A09 | VCU timeout | STANDBY, open after 1 s |
| 0x000A0A | Cell-sum mismatch | CCL/DCL 0 |
| 0x000A0B | Precharge failure | open immediately, latched |

### 20.3 CAN map

| ID | Name | Direction | Period | Safety content |
|---|---|---|---|---|
| 0x350 | Pack status | BMS → VCU | 100 ms | V, I, SOC, state; CRC + alive |
| 0x351 | Limits | BMS → VCU | 100 ms | CCL, DCL, Vreq, enable; CRC + alive |
| 0x352 | Cell statistics | BMS → VCU | 500 ms | Vmax/Vmin/indices, Tmax/Tmin |
| 0x353 | Cell voltages | BMS → VCU | 1000 ms | 6 mux frames, 3 cells each, balance bits |
| 0x354 | Faults | BMS → VCU | 1000 ms + change | fault/warning bits, DTC count |
| 0x200 | VCU command | VCU → BMS | 100 ms | contactor, balance, sleep, clear; CRC + alive |

### 20.4 Glossary

| Term | Meaning |
|---|---|
| AFE | Analog front end — the cell-voltage / temperature measurement IC |
| CCL / DCL | Charge / discharge current limit |
| DTC | Diagnostic trouble code (ISO 14229) |
| FC / CF / FF / SF | ISO-TP flow control / consecutive frame / first frame / single frame |
| LFP | Lithium iron phosphate |
| NRC | Negative response code (UDS) |
| OCV | Open-circuit voltage |
| S3 | UDS session timeout (5 s here) |
| SOC | State of charge |
| WCET | Worst-case execution time |

---

**End of document.** The code is the authority; this document is the map. If a detail here
ever disagrees with `src/`, the code and its tests win, and the document should be corrected.
