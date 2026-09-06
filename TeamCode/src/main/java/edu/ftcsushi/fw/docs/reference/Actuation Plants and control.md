---
tags:
  - Reference
---

# Actuation, Plants, and control quick reference

## Ordinary entry points

| Need | API |
|---|---|
| build an FTC-owned motor or servo Plant | [`FtcActuators`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcActuators.html>) |
| own one scalar request, update, snapshot, and stop | [`Plant`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/Plant.html>) |
| inspect requested/applied/measured/arrival facts | [`PlantSnapshot`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/PlantSnapshot.html>) |
| retain position range and reference facts | [`PositionPlant`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/PositionPlant.html>) |
| publish named and numeric intent together | [`SemanticScalarCommand<S>`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarCommand.html>) |
| choose exact, periodic-equivalent, overlay, or planned targets | [`PlantTargets`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/PlantTargets.html>) |

**Advanced assembly only:** [`Plants`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/Plants.html>)
builds a hardware-neutral Plant from completed output adapters for tests, portable hosts, or custom
adapters. Ordinary FTC mechanisms use `FtcActuators`, not two interchangeable construction paths.

## Remember

A mechanism privately owns its final Plant graph and update order. Command, requested target,
applied target, actuator output, measurement, and arrival are different facts. Standard-servo
native `[0, 1]` is not evidence that the installed mechanism is safe across that interval.

Read [FTC Actuators and Plants](<../ftc-boundary/FTC Actuators & Plants.md>) and
[Mechanism Target Planning](<../drive-vision/Mechanism Target Planning.md>).
