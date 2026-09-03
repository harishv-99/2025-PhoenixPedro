---
tags:
  - Reference
---

# FTC adapters, testing, and tuning quick reference

## Ordinary entry points

| Need | API |
|---|---|
| obtain low-level FTC command channels | [`FtcHardware`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcHardware.html>) |
| build a bounded mechanism Plant | [`FtcActuators`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcActuators.html>) |
| build a mecanum drive owner | [`FtcDrives`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcDrives.html>) |
| choose standard hardware bring-up tools | [`StandardTesters`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/StandardTesters.html>) |
| host an exclusive selectable tester | [`FtcTeleOpTesterOpMode`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcTeleOpTesterOpMode.html>) |
| build a guided physical calibration sequence | [`CalibrationWalkthroughBuilder`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/calibration/CalibrationWalkthroughBuilder.html>) |

## Remember

Defaults establish coherent software, not reviewed physical safety. Software tests prove software
decisions; probes do not simulate physics. Physical direction, ranges, wiring, switch placement,
controller tuning, and safe motion require isolated robot evidence.

Start with [How to test a Sushi component](<../testing-calibration/How to test a Sushi component.md>),
then [Actuator Bring-up](<../testing-calibration/Actuator Bring-up.md>) or
[Control Tuning](<../testing-calibration/Control Tuning Workflow.md>).
