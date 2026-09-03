---
tags:
  - Reference
---

# Program and lifecycle quick reference

## Ordinary entry points

| Need | API |
|---|---|
| declare an FTC TeleOp or Auto | [`FtcRobotOpMode`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcRobotOpMode.html>) |
| register services, bindings, Tasks, outputs, and presenters | [`RobotProgram`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html>) |
| read shared cycle identity and monotonic loop time | [`LoopClock`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/time/LoopClock.html>) |
| retain a clock-created observation time | [`LoopTimestamp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/time/LoopTimestamp.html>) |
| aggregate best-effort cleanup without choosing policy | [`CleanupActions`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/lifecycle/CleanupActions.html>) |

## Remember

The active order is Clock → Services → Bindings → Tasks → Outputs/Drive → Presenters → one
telemetry commit. The managed host advances the one clock exactly once per cycle and owns total
cleanup. A component must never sleep or run a competing robot loop.

Read [Loop Structure](<../core-concepts/Loop Structure.md>) for exact phase and failure behavior or
[Sushi in one picture](<../getting-started/Framework Overview.md>) for the short mental model.
