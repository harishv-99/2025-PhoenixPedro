# Plants and hardware

**Question:** How does a semantic request become one final hardware command, and what can robot
code truthfully report?

A **Plant** is the mechanism-owned object that resolves one requested actuator target and performs
the final hardware write. The mechanism owns the complete path: capability intent, its private
Plant, managed update, cached status, and terminal stop. This is a source-only lesson; no hardware
motion is required.

## Follow the Starter intake

### Critical code

```text
setMode(COLLECT) -> remember Mode.COLLECT -> map it to configured power
                 -> persistent Plant command target -> Plant.update(shared clock)
                 -> resolve/cache applied target -> one actuator write path
```

There is no second writer for a button, Task, or emergency. Temporary and guarded intent must join
the final target resolver; the Plant remains the only actuator writer.

[`StarterIntakeMechanism`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java>)
receives `HardwareMap` plus data-only configuration, copies and validates that configuration, then
constructs its private Plant:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
plant = FtcActuators.plant(
                Objects.requireNonNull(hardwareMap, "hardwareMap is required")
        )
        .motor(motorName, direction)
        .power()
        .targetFromNewCommand(STOPPED_POWER)
        .build();
```

**What to notice**

- The mechanism constructs and privately owns the complete Plant graph.
- The builder names one motor, one target source, and a safe initial zero request.

**Key APIs:** `FtcActuators.plant(hardwareMap)` begins ordinary FTC construction;
`targetFromNewCommand(...)` creates the Plant-owned persistent command source.

The builder answers which FTC motor is owned, its logical direction, the public target kind, and
where the final target comes from. A direct-power Plant owns normalized range `[-1, +1]` and begins
with a zero command. Construction does not command motion; the managed output phase later calls
`plant.update(clock)`.

`setMode(...)` keeps the named request and maps it forward to the configured power. It does not
write hardware. Status reports that retained request directly:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
return new Status(requestedMode, plant.getAppliedTarget());
```

**What to notice:** semantic request and applied numeric target remain separate facts.

**Key APIs:** `Plant.getAppliedTarget()` reports the cached resolved command; it is not motor
feedback.

The mechanism never guesses a mode by reading a power back. Collect and eject powers therefore do
not need to be unique, though both Starter action powers must still be finite, nonzero, and inside
`[-1, +1]`. That is a software-identity rule, not a recommendation to configure a real one-motor
intake with equal powers: the team must still verify that its chosen values produce the intended
different physical actions. `appliedTargetPower` is the cached final target after resolution and
guards. It is not motor readback and does not prove physical movement.

## Keep the evidence names honest

| Evidence | Question answered | Example |
| --- | --- | --- |
| Semantic request | What behavior did a client request? | `Mode.COLLECT` |
| Requested target | What number did the command source request? | requested power or lift inches |
| Applied target | What final target survived resolution and guards? | `getAppliedTarget()` |
| Measurement | What feedback did the Plant cache? | lift position or flywheel velocity |
| Ready / `atTarget` | Does cached controller evidence meet tolerance? | both flywheels within tolerance |
| Physical result | Did the game piece move or score? | operator observation or separate sensor evidence |

Presenters and clients read the mechanism's cached snapshot after its update. They do not resample
hardware or advance the Plant. A requested or applied target is not a measurement; controller
readiness is not proof of a successful game action.

## Active idle is not terminal stop

During a match, an abort or idle command changes persistent targets—such as requesting zero
flywheel velocity—so the same Plants remain reusable on later cycles. At total cleanup, the
mechanism calls `Plant.stop()`. Stop is terminal for that Plant instance; a later OpMode constructs
a fresh graph. `RobotProgram` performs declared output cleanup, not controls or presenters.

At FTC STOP, do not assume a newly requested servo position will be realized: cancellation may
change a request, but terminal cleanup follows and no later normal update is promised.

## How the pattern scales

The
[`ReferenceLiftMechanism`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/lift/ReferenceLiftMechanism.java>)
adds bounded public inches, encoder conversion, a required homing reference, tolerance, and cached
measurement. Homing is a non-blocking Task that establishes the reference; the Plant still owns
coordinates and final output.

The
[`ReferenceLauncherMechanism`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.java>)
cohesively owns several Plants, a temporary transfer overlay, and per-wheel velocity evidence. Its
one `update(clock)` defines their order. `ready` requires a positive request and both wheels within
tolerance; it cannot prove that a game piece scored.

## Check your understanding

**`appliedTargetPower()` reports `0.20`. Did the motor turn?** That cannot be concluded. Sushi
knows the cached applied command, not physical motion.

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
