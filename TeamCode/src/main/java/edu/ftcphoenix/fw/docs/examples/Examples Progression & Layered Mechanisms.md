# Manual-lifecycle concept labs

**Audience:** students and mentors who already understand the managed Phoenix robot structure and
want to inspect one loop phase or mechanism pattern in isolation.

The numbered `fw.tools.examples` OpModes are disabled, one-file concept labs. They extend the FTC
SDK `OpMode` directly so their clock, bindings, Tasks, outputs, telemetry, and cleanup remain
visible. They are useful for focused teaching and custom-host work, but they are not a beginner
course or an ordinary robot template.

For a first robot, start at the canonical [`Phoenix docs hub`](<../README.md>) and use
[`Modern Starter Robot`](<Modern Starter Robot.md>) as the compiling TeleOp/Auto ownership
reference. Ordinary robot code uses `FtcRobotOpMode` and `RobotProgram` instead of copying the
manual lifecycle ceremony in these labs.

## Choose a lab by concept

| Lab | Focus |
|---|---|
| [`TeleOp_01_MecanumBasic`](<../../tools/examples/TeleOp_01_MecanumBasic.java>) | One `LoopClock`, a manual `DriveSource`, and an immediate direct-drive write. |
| [`TeleOp_02_ShooterBasic`](<../../tools/examples/TeleOp_02_ShooterBasic.java>) | FTC actuator construction, simple scoring modes, and one owner for Plant updates and stop. |
| [`TeleOp_03_ShooterMacro`](<../../tools/examples/TeleOp_03_ShooterMacro.java>) | Fresh, non-blocking Task graphs for a timed shooter sequence. |
| [`TeleOp_04_ShooterInterpolated`](<../../tools/examples/TeleOp_04_ShooterInterpolated.java>) | A calibration table that maps range to a shooter request. |
| [`TeleOp_05_ShooterTagAimVision`](<../../tools/examples/TeleOp_05_ShooterTagAimVision.java>) | Shared AprilTag selection, omega-only guidance, and shooter speed from range. |
| [`TeleOp_06_ShooterTagAimMacroVision`](<../../tools/examples/TeleOp_06_ShooterTagAimMacroVision.java>) | The same targeting graph combined with a non-blocking shooting macro. |
| [`TeleOp_07_SupervisorPoseMechanism`](<../../tools/examples/TeleOp_07_SupervisorPoseMechanism.java>) | Remembered semantic poses plus a temporary output-queue override. |
| [`TeleOp_08_LiftExternalSensorControl`](<../../tools/examples/TeleOp_08_LiftExternalSensorControl.java>) | A framework-regulated position Plant using external analog feedback. |
| [`TeleOp_09_LayeredShooterMechanism`](<../../tools/examples/TeleOp_09_LayeredShooterMechanism.java>) | Explicit Requests, Behavior, and Realization roles inside one mechanism lesson. |

Examples 05 and 06 are explained in
[`Shooter Case Study & Examples Walkthrough`](<Shooter Case Study & Examples Walkthrough.md>).
Example 09 is explained in
[`Layered Shooter Example`](<Layered Shooter Example.md>).

## Contracts shared by every lab

The visible lifecycle is part of each lesson. When reading or adapting a lab, preserve these
contracts:

1. Advance one `LoopClock` exactly once at the start of each loop.
2. Sample controls and advance Tasks before the downstream owner applies Plant or drive output.
3. Give each Plant one final resolver and one update owner.
4. Keep drive intent upstream of one final drive writer.
5. Use fresh Task instances for repeated behavior and make active cancellation safe.
6. Stop every owned output and resource from the FTC `stop()` boundary, even though ordinary
   managed robot code delegates that work to `RobotProgram`.
7. Keep telemetry observational; it must not advance a source, controller, Task, or hardware owner.

The labs are `@Disabled` because their hardware names, directions, bounds, tuning, and physical
motion must be reviewed for the robot that runs them.

## Mechanism-layering labs

The last three labs answer different architecture questions. They are alternatives selected by
the concept being studied, not steps required in every mechanism.

### Example 07: remembered intent plus a temporary override

`TeleOp_07_SupervisorPoseMechanism` keeps the selected semantic pose as persistent intent. An
`OutputTaskRunner` proposes a short-lived override through `PlantTargets.overlay(...)`; when that
proposal ends, the resolver naturally returns to the remembered pose. The subsystem remains the
only Plant update and stop owner.

Use this pattern when a temporary behavior should win without erasing the base request.

### Example 08: external feedback realization

`TeleOp_08_LiftExternalSensorControl` maps a held height selection into a regulated position Plant
whose feedback comes from an analog sensor. The middle behavior decision is intentionally small;
the useful lesson is that the Plant realization owns the feedback loop, units, bounds, update, and
stop.

Use this lab to inspect an external-feedback control path. In ordinary robot code, construct and
retain that Plant inside the mechanism rather than in the OpMode.

### Example 09: requests, behavior, and realization

`TeleOp_09_LayeredShooterMechanism` places three kinds of intent in one lesson:

- **held:** flywheel enable and selected velocity;
- **frame:** manual feed power refreshed each loop;
- **pending:** bounded shot requests retained until behavior consumes them.

`Behavior` owns readiness, timing, priority, and feed-pulse decisions. `Realization` alone writes
the graph-owned Plant commands, advances the Plants, and publishes readback for the next loop.
This separation is useful when a mechanism has enough policy to justify named internal roles; a
simple mechanism does not need empty layers merely for symmetry.

## Related reading

- [`Modern Starter Robot`](<Modern Starter Robot.md>) — ordinary managed robot structure
- [`Tasks and Macros`](<../design/Tasks & Macros Quickstart.md>) — Task construction and cancellation
- [`Output Tasks & Queues`](<../design/Output Tasks & Queues.md>) — source proposals and temporary overrides
- [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) — mechanism and composition-root ownership
