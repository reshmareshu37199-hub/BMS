# BMS-FW-ASG-01 — 16S LFP Battery Management System Firmware

Reference solution for the Senior Firmware Engineer take-home assignment.
The firmware is a complete, host-runnable BMS application: AFE acquisition, passive
balancing, protection and DTC management, CAN, UDS/ISO-TP diagnostics, SOC estimation,
parameter storage, a scenario simulator, plots, unit tests, coverage and static analysis.

> **If you read one document, read [`docs/BMS-FW-ASG-01_Submission.pdf`](docs/BMS-FW-ASG-01_Submission.pdf)**
> (editable: [`docx`](docs/BMS-FW-ASG-01_Submission.docx); source: [`md`](docs/BMS-FW-ASG-01_Submission.md)).
> It is the end-to-end answer: every task, the design rationale, the alternatives that were
> rejected, the verification evidence and the review speaking notes. The same content is also
> kept as [`docs/COMPLETE_SOLUTION.md`](docs/COMPLETE_SOLUTION.md).

* **Pack:** 16S1P LFP, 100 Ah, 48 V nominal (40.0–58.4 V)
* **Language:** C11, fixed-width types, no dynamic allocation in application code
* **Target:** any 32-bit MCU; the PC build uses a simulated HAL and AFE register model
* **Build:** CMake + Ninja + GCC (also builds with Makefiles or MSVC; warnings are stricter on GCC/Clang)

---

## 1. Quick start (one command per step)

### Windows (PowerShell) / Linux / macOS

```bash
cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build
ctest --test-dir build --output-on-failure      # or: cmake --build build --target check
```

Expected result:

```
88 Tests 0 Failures 0 Ignored
OK
```

Run the two scenarios and regenerate the plots:

```bash
cmake --build build --target simulate
```

Artifacts are written to `sim/out/`:

| File | Content |
|---|---|
| `charge_balance.csv` | full charge with passive balancing, 260 s |
| `fault_recovery.csv` | over-voltage trip, FAULT hold, DTC clear, recovery, 100 s |
| `charge_balance_voltage.svg` | cell 1 and cell 5 voltages over time |
| `charge_balance_balance.svg` | balancing mask over time |
| `charge_balance_soc.svg` | SOC over time |
| `fault_recovery_voltage.svg` | faulted cell voltage |
| `fault_recovery_contacts.svg` | contactors and fault bits |

Open the SVGs in any browser; no plotting tool is required.

The master document is available in three forms: [`docs/COMPLETE_SOLUTION.md`](docs/COMPLETE_SOLUTION.md)
(source), `docs/COMPLETE_SOLUTION.html` (self-contained) and `docs/COMPLETE_SOLUTION.pdf`
(print-ready). They are pre-rendered; no extra tooling is needed to read them.

If Ninja is not installed, omit `-G Ninja`. The build has no third-party dependencies
except the vendored Unity test framework.

### Coverage

```bash
cmake -S . -B build-cov -G Ninja -DBMS_COVERAGE=ON
cmake --build build-cov
cd build-cov
./bms_tests.exe                 # Linux/macOS: ./bms_tests
mkdir -p coverage && cd coverage
gcov -b -c ../CMakeFiles/bms_core.dir/src/services/balancing.c.gcno
gcov -b -c ../CMakeFiles/bms_core.dir/src/services/protection.c.gcno
```

Results and the exact command are recorded in [`docs/coverage.txt`](docs/coverage.txt).

### Static analysis

```bash
cppcheck --enable=warning,style,performance,portability --std=c11 \
         --suppress=missingIncludeSystem --suppress=unusedFunction \
         -I src src
```

Result: **no findings** with Cppcheck 2.22.0. See [`docs/static_analysis.txt`](docs/static_analysis.txt).

---

## 2. Repository layout

```
bms-fw/
  README.md                 this file: build, design summary, AI usage, licences
  ASSUMPTIONS.md            every assumption, with rationale and impact
  CODE_REVIEW.md            Task 7: review of the junior engineer's code + corrected version
  docs/
    DESIGN.md               Task 1: architecture, scheduling, state machine, safety, alternatives
    BMS-FW-ASG-01_Submission.md     the submission document (master source)
    BMS-FW-ASG-01_Submission.pdf    print-ready A4 — send this one
    BMS-FW-ASG-01_Submission.docx   editable Word version
    COMPLETE_SOLUTION.md    the same master answer (internal copy)
    COMPLETE_SOLUTION.html  self-contained HTML of the same document
    COMPLETE_SOLUTION.pdf   print-ready A4 (same content)
    style.css               print style for the HTML/PDF rendering
    coverage.txt            Task 6: coverage report
    static_analysis.txt     Task 6: cppcheck report
    plots/                  checked-in SVG plots
  dbc/bms.dbc               Task 5: CAN database
  src/
    common/                 CRC-8/CRC-16, saturation helpers
    hal/                    HAL interface, PC simulator, AFE register model
    drivers/                AFE driver (non-blocking), CAN driver (Tx queue/Rx dispatch), NTC table
    services/               balancing, protection/DTC, SOC, CAN signal layer, CAN matrix, ISO-TP, UDS, NVM
    app/                    scheduler, BMS state machine, application orchestration
  test/                     Unity unit tests (88 tests)
  sim/
    sim_main.c              CSV scenario runner
    scenarios/              charge_balance.csv, fault_recovery.csv
    out/                    generated traces and SVG plots
  tools/plot_svg.c          dependency-free CSV -> SVG plotter
  third_party/unity/        Unity test framework (MIT)
  CMakeLists.txt            one-command build, test, simulate, coverage
```

---

## 3. Design summary

The firmware is layered **application → services → drivers → HAL**, with dependencies
pointing downwards only. Services such as balancing, protection, SOC, CAN semantics and UDS
are pure logic and are tested without hardware. The AFE and CAN drivers are the only modules
that touch the HAL, and the HAL is an injected function-pointer table, so the same
application code runs against the PC simulator or a target board.

**Scheduling.** A 1 ms tick drives the AFE state machine and CAN drain. The fast control task
runs at 10 ms (current, over-current, protection, state machine) and the main control task at
100 ms (AFE cycle, balancing, CAN Tx); statistics run at 500 ms and slow frames at 1 s. Task
order is the priority order. A task that overruns is re-based, not run repeatedly, so work per
tick is bounded and misses are visible in `Sched_GetOverruns()`. Estimated CPU load is ≈ 8.5 %
as a conservative upper bound (80 µs charged to every 1 ms tick); the AFE is active for only
≈ 22 ms of each 100 ms cycle, so the average is closer to 1–2 %. A preemptive RTOS was
considered and rejected: five fixed periodic entries do not justify the footprint, priority
inversion analysis and reduced host reproducibility. See
[`docs/DESIGN.md` §3 and §7](docs/DESIGN.md).

**AFE acquisition.** The AFE driver is a non-blocking state machine that performs one SPI
transaction per 1 ms tick: read device ID (3 attempts), start conversion, poll status, read
16 cells and 5 temperatures, validate CRC, publish a double-buffered snapshot. A full cycle
takes ≈ 22 ms without ever blocking the CPU. Three consecutive failed cycles raise the
latched AFE communication fault. A single good cycle clears the error counter but not the
latched fault.

**Data consistency.** AFE snapshots use two buffers and a critical-section index swap; the
consumer copies the active buffer under the same short critical section. No SPI transfer
runs inside a critical section. CAN reception is polled in the 1 ms tick task, and handlers only
update small state records.

**State machine.** `INIT → STANDBY → PRECHARGE → DRIVE/CHARGE → STANDBY → SLEEP`, with `FAULT`
reachable from every state. Precharge closes the negative contactor, waits 20 ms, closes the
precharge relay, waits for bus ≥ 95 % of pack (2 s timeout), closes the positive contactor,
overlaps for 20 ms, then opens the precharge relay. Faults that do not require an immediate
open hold the contactors for 1 s so the VCU can reduce current; over-current, hard OV/UV and
precharge failure open immediately.

**Protection.** A table (`FaultDef_t`) defines DTC, set/release thresholds, debounce, latch
and action for 11 faults. One generic debounce/DTC object implements them all. CCL/DCL come
from piecewise-linear temperature and voltage derating tables and ramp at 100 mA/ms. DTCs
carry ISO 14229 status bits and are reported through UDS 0x19.

**Communication.** The CAN signal layer is table-driven and handles signed values, offsets,
cross-byte Intel packing and saturation. Messages with safety-relevant data carry CRC-8 SAE
J1850 and a 4-bit alive counter. The VCU command rejects wrong CRC, unchanged alive counters
and jumps > 2; three consecutive invalid frames equal a timeout. UDS implements 0x10, 0x3E,
0x22, 0x2E, 0x19 and 0x14 with the required NRCs; ISO-TP implements single-frame and the
multi-frame response for DID 0x0100.

**SOC (bonus B1).** 64-bit coulomb counting at 10 ms, current-offset learning after 30 s at
rest, and OCV correction after 1 h at rest with a reduced weight in the flat LFP region
(5 % between 3.28 V and 3.34 V, 25 % on the steep edges). SOC is persisted in NVM.

**Parameter storage (bonus B2, implemented part).** A/B slots with magic, version, sequence
and CRC-16; the newest valid record wins, a corrupt slot falls back to the other slot, then
to defaults. The firmware-update half of B2 is design-only as permitted; see
[`docs/DESIGN.md` §12](docs/DESIGN.md).

---

## 4. Voltage-based vs SOC-based balancing (required comparison)

Voltage-based balancing compares each cell with the pack minimum and bleeds the highest
cells. For LFP it is the right default at the **top of charge**: the OCV curve is flat from
about 30 % to 90 % SOC, so mid-range voltage differences say little about SOC, but above
≈ 3.4 V the curve steepens and a voltage delta is a usable proxy for the least-charged cell.
It needs only the AFE that is already in the BOM and is robust to cell capacity mismatch,
which is why this design gates balancing on `Vmax ≥ 3400 mV` and uses 15/5 mV hysteresis.
Its weakness is that it cannot balance in the flat region and is sensitive to measurement
offset and IR drop; the measurement-window pause addresses the bleed-current error.

SOC-based balancing compares per-cell state of charge and is preferable when each cell has
its own current sensor or a validated cell model, when the pack must be balanced in the flat
mid-range, or when minimising degradation from cycling at different depths of discharge. It
requires per-cell SOC estimation and is sensitive to model and offset errors. For this
16S1P pack with one pack current sensor and passive 39 Ω bleed resistors, voltage-based
top-balancing is the appropriate engineering choice; SOC-based balancing is a documented
future extension if per-cell SOC becomes available.

The algorithm prevents starvation by remembering the first cell dropped by the max-cell or
adjacency rule and giving it priority next cycle when it is within 20 mV of the maximum.
This is deterministic and bounded, unlike random or fixed odd/even schemes.

---

## 5. Requirement traceability

| Requirement | Where | Evidence |
|---|---|---|
| Task 1 architecture, scheduling, state machine, data sharing, safety | `src/app/`, `src/services/`, `docs/DESIGN.md` | design document; 88 tests; simulation |
| Task 2 AFE driver, CRC, retries, plausibility, snapshot | `src/drivers/afe.c`, `src/hal/afe_sim.c` | `test/test_afe.c` |
| Task 3 balancing, S1–S10, fairness, inhibits, timeout | `src/services/balancing.c` | `test/test_balancing.c`, 96.08 % branch taken |
| Task 4 protection table, debounce, DTCs, CCL/DCL ramp | `src/services/protection.c` | `test/test_protection.c`, 91.38 % branch taken |
| Task 5 CAN matrix, signal layer, DBC, UDS, ISO-TP | `src/services/can_*.c`, `src/services/uds.c`, `src/services/isotp.c`, `dbc/bms.dbc` | `test/test_can.c`, `test/test_uds.c` |
| Task 6 unit tests, simulation, static analysis, coverage | `test/`, `sim/`, `docs/` | this README, `docs/coverage.txt`, `docs/static_analysis.txt`, SVG plots |
| Task 7 code review | `CODE_REVIEW.md` | issues + corrected code |
| Task 8 documentation and code quality | all `.md` files, headers, comments | strict build, no warnings |
| B1 SOC | `src/services/soc.c` | `test/test_soc.c` |
| B2 parameter storage (implemented) | `src/services/nvm.c` | `test/test_nvm.c` |
| B2 firmware update (design only) | `docs/DESIGN.md` §12 | bootloader design |

---

## 6. Test and verification summary

* **Unit tests:** 88 tests across HAL, AFE, balancing, protection, CAN, UDS/ISO-TP, SOC,
  state machine and NVM. `ctest` runs the same binary.
* **Mandatory balancing scenarios S1–S10** are implemented explicitly, plus seven extra
  cases for validation and the starvation logic.
* **Protection:** each fault has set, debounce-reset and release tests; hard-threshold and
  immediate-open paths are covered.
* **Simulation:** two CSV-driven scenarios run the real application modules in a loop and
  log state, SOC, cell voltages, balancing mask, current limits, contactors and fault bits.
* **Coverage:** balancing 100.00 % lines / 96.08 % branches taken; protection 97.30 % lines /
  91.38 % branches taken. Both exceed the 85 % branch target.
* **Static analysis:** Cppcheck 2.22.0 (warning, style, performance, portability) reports no
  findings.
* **Build:** `-Wall -Wextra -Werror -Wpedantic -Wshadow -Wstrict-prototypes
  -Wmissing-prototypes -Wdouble-promotion -Wformat=2 -Wundef -Wcast-qual -Wpointer-arith`.

---

## 7. AI-tool usage disclosure

AI assistance was used, as permitted by the assignment. In the interest of transparency:

* **Where:** the layered module skeleton, portions of the driver/service implementations,
  the Unity test scaffolding, the DBC, the simulation runner, the SVG plotter and the first
  drafts of the Markdown documents were drafted with an AI coding assistant (OpenCode,
  model “Big Pickle”).
* **How:** the assistant proposed interfaces and implementations from the assignment
  requirements; every file was then compiled, run under the strict warning set, exercised by
  unit tests and the simulator, and reviewed line by line. Failures found during review were
  fixed in the code and tests (for example the ISO-TP block-boundary condition, the
  immediate-open path on fault entry, and several DTC/ISO-TP test expectations).
* **What it did not do:** no AI runs at firmware runtime, no generated code is linked without
  being reviewed, and no external library was added beyond the vendored Unity test framework.
* **Review readiness:** the design rationale for every non-obvious choice is written down in
  `docs/DESIGN.md` §7 (alternatives and why they were rejected), so each decision can be
  defended or changed during the review session.

If a stricter disclosure format is required, this section can be expanded into a per-file
table (file, AI involvement, reviewer, verification evidence).

---

## 8. Licences

* **Unity** test framework — MIT licence, vendored at `third_party/unity/` (see
  `third_party/unity/LICENSE.txt`).
* All other code in this repository is original work created for this assignment.
* No other third-party runtime libraries, no OS calls, no dynamic allocation in `src/`.

---

## 9. Assumptions and limitations

All assumptions are listed in [`ASSUMPTIONS.md`](ASSUMPTIONS.md). The most important are:
the target MCU class and SPI clock used for the timing budget, the 5th AFE temperature
channel being reserved, the DTC numbering scheme, the VCU timeout being treated as a STANDBY
condition rather than a latched hardware fault, and the NVM HAL being a memory copy on the
PC (the target flash-write budget is discussed in `docs/DESIGN.md` §3.3 and §13).

Known limitations: no target hardware measurements, no MISRA add-on, no Git/CI runner in the
build environment, cell open-wire not localised per cell, and the bootloader is design-only.
None of these affects the mandatory functionality or the test evidence.
