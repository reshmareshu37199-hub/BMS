# BMS Firmware Assignment Submission

Assignment: BMS-FW-ASG-01 — Senior Firmware Engineer, BMS.

This folder contains the submission document and the accompanying `bms-fw` source project. The project includes the C implementation, tests, DBC, assumptions, code review, simulation scenarios, reports and plots.

## Build and test

Run these commands from the `bms-fw` directory:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
cmake --build build --target simulate
```

See `bms-fw/README.md` for coverage, static-analysis commands and details. AI-tool use is disclosed there and in the submission document.
