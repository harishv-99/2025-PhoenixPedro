---
tags:
  - Learn
---

# Plants and hardware

**Learning mode:** Architecture reference

This page traces the realization path. Use the
hardware-free mechanism module for a complete compiling test of the maintained mechanism.

**Question:** How does a semantic request become one final hardware command, and what can robot
code truthfully report?

A **Plant** is the mechanism-owned object that resolves one requested actuator target and performs
the final hardware write. The mechanism owns the complete path: capability intent, its private
Plant, managed update, cached Plant facts, capability status, and terminal stop. This is a
source-only lesson; no hardware motion is required.

## Follow the Starter intake

### Critical code

```text
setMode(COLLECT) -> remember Mode.COLLECT -> map it to configured power
                 -> persistent Plant command target -> Plant.update(shared clock)
                 -> resolve/cache applied target -> one actuator write path
```

There is no second writer for a button, Task, or emergency. Temporary and guarded intent must join
the final target resolver; the Plant remains the only actuator writer.

[`StarterIntakeMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.html>)
receives `HardwareMap` plus data-only configuration, copies and validates that configuration, then
constructs its private Plant:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
plant = FtcActuators.plant(
                Objects.requireNonNull(hardwareMap, "hardwareMap is required")
        )
        .motor(motorName, direction)
        .power()
        .targetExactlyFrom(modeCommand)
        .build();
```

**What to notice**

- The mechanism constructs and privately owns the complete Plant graph.
- The builder names one motor and one semantic target source whose initial request is safe zero.

**Key APIs:** `FtcActuators.plant(hardwareMap)` begins ordinary FTC construction;
`targetExactlyFrom(...)` binds the mechanism-owned semantic command into the final resolver. The
name is literal: a periodic mechanism that wants an interchangeable representative uses
`PlantTargets.equivalentPositionsOf(...)` explicitly instead.

The builder answers which FTC motor is owned, its logical direction, the public target kind, and
where the final target comes from. A direct-power Plant owns normalized range `[-1, +1]` and begins
with a zero command. Construction does not command motion; the managed output phase later calls
`plant.update(clock)`.

`setMode(...)` keeps the named request and maps it forward to the configured power. It does not
write hardware. Internally, the mechanism composes that retained request with cached Plant facts:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
return new Status(modeCommand.snapshot(plant.snapshot()));
```

**What to notice:** semantic request and applied numeric target remain separate facts.

**Key APIs:** `SemanticScalarCommand.snapshot(...)` pairs the named request with one immutable
snapshot of cached Plant facts without polling hardware. `StarterIntake.status()` projects
`mode()`, `requestedPower()`, `appliedPower()`, and the advanced `plantSnapshot()` escape hatch.

The mechanism never infers a mode from power, so collect and eject values need not be unique. Both
must still be finite, nonzero, inside `[-1, +1]`, and physically reviewed. `appliedPower()` is the
cached resolved target, not motor readback or proof of movement.

## Keep the evidence names honest

| Evidence | Question answered | Example |
| --- | --- | --- |
| Semantic request | What behavior did a client request? | `Mode.COLLECT` |
| Requested target | What number did the command source request? | requested power or lift inches |
| Applied target | What final target survived resolution and guards? | `getAppliedTarget()` |
| Measurement | What feedback did the Plant cache? | lift position or flywheel velocity |
| Ready / `atTarget` | Does cached controller evidence meet tolerance? | both flywheels within tolerance |
| Physical result | Did the game piece move or score? | operator observation or separate sensor evidence |

Clients read capability status after update. It uses cached Plant facts without polling hardware;
multi-source capabilities may publish them together. A requested or applied target is not a
measurement, and controller readiness
is not proof of a successful game action.

## Active idle is not terminal stop

During a match, an abort or idle command changes persistent targets—such as requesting zero
flywheel velocity—so the same Plants remain reusable on later cycles. At total cleanup, the
mechanism calls `Plant.stop()`. Stop is terminal for that Plant instance; a later OpMode constructs
a fresh graph. `RobotProgram` performs declared output cleanup, not controls or presenters.

At FTC STOP, do not assume a newly requested servo position will be realized: cancellation may
change a request, but terminal cleanup follows and no later normal update is promised.

## How the pattern scales

The
[`ReferencePeriodicTurretMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretMechanism.html>)
adds a bounded radian coordinate, encoder conversion, tolerance, and cached measurement. Its
`equivalentPositionsOf(...)` resolver turns one logical periodic request into the nearest legal
physical representative. Status keeps requested, selected, applied, measured, and arrived facts
separate; selection alone is not physical arrival.

The
[`ReferenceFlywheelMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelMechanism.html>)
owns one grouped paired-velocity Plant. Each successful update publishes a
[`ReferenceFlywheels.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.Status.html>)
containing the grouped snapshot and two independent wheel measurements. Readiness needs a positive
value selected and applied without fallback plus both wheels in tolerance; it cannot prove release
or scoring. `ReferenceLauncherMechanism` delegates to that owner and composes its publication with
object-sensor and transfer state instead of duplicating the Plant or flywheel status fields.

## Check your understanding

**`status().appliedPower()` reports `0.20`. Did the motor turn?** That cannot be concluded. Sushi
knows the cached applied command, not physical motion. `status().plantSnapshot()` is available when
an advanced diagnostic needs the full generic Plant capture.

**Collect and eject use the same configured power. Can status still distinguish them?** Yes.
`status().mode()` reports the retained semantic request, not a mode inferred from that number.

**Where should maximum lift power live?** In lift configuration and Plant construction, not a
button callback.

**Can a temporary feed pulse write a transfer actuator beside its Plant?** No. Compose it into the
one final resolver and keep the Plant as sole writer.

## Go deeper when needed

- [FTC Actuators and Plants](<../../ftc-boundary/FTC Actuators & Plants.md>) — complete construction and lifecycle grammar
- [Mechanism target planning](<../../drive-vision/Mechanism Target Planning.md>) — overlays, guards, and plans
- [Tasks and autonomous](<Tasks and Autonomous.md>) — time-based behavior changes requests
- [Learn Sushi topic guide](<../Beginner's Guide.md>) — choose another topic
