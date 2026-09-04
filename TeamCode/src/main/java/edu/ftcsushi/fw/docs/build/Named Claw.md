---
tags:
  - Build
---

# Move a claw through named positions

**Outcome:** use `CLOSED`, `HALF`, and `OPEN` everywhere while one mechanism maps the normalized
coordinates `0.0`, `0.5`, and `1.0` into configured native standard-servo endpoint candidates.

**Prerequisites:** complete [the continuous-intake lesson](<Continuous Intake.md>) through its
software checkpoint. No servo or linkage is required before this page's isolated hardware gate.

**Builds on:** the intake's capability/configuration/mechanism split, semantic command, private
Plant, managed output heartbeat, controls binding, cached status, and terminal stop.

**New here:** a bounded logical position is mapped into a smaller native servo interval, and an
applied position command is kept distinct from physical arrival because a standard servo supplies
no position feedback.

## Critical production idea

The owner/heartbeat path does not change:

```text
named request -> semantic command -> private Plant -> managed update -> one servo write
```

Only the Plant questions specific to a positional standard servo are new.

### Keep two native endpoint candidates in configuration

The complete software baseline carries an initial semantic request and two native FTC positions:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java -->
```java
config.servoName = "clawServo";
config.direction = Direction.FORWARD;
config.closedNativePosition = 0.25;
config.openNativePosition = 0.70;
config.initialState = State.CLOSED;
return config;
```

These numbers are software-valid candidates in native interval `[0, 1]`; they are not reviewed
safe linkage endpoints. Unlike the intake's initial zero-power request, the first successful output
heartbeat may move the servo toward `CLOSED`. Keep the motion gate false until that initial motion
has a safe test setup.

The mechanism still maps semantic names forward once:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java -->
```java
stateCommand = SemanticScalarCommand.forEnum(c.initialState)
        .map(State.CLOSED, CLOSED_TARGET)
        .map(State.HALF, HALF_TARGET)
        .map(State.OPEN, OPEN_TARGET)
        .build();
```

`CLOSED_TARGET`, `HALF_TARGET`, and `OPEN_TARGET` are mechanism coordinates `0.0`, `0.5`, and `1.0`.
They let callers and status keep stable names even if the configured native endpoints change.

### Add logical bounds and one native range map

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java -->
```java
claw = FtcActuators.plant(map)
        .servo(c.servoName, c.direction)
        .position()
        .nonPeriodic()
        .bounded(CLOSED_TARGET, OPEN_TARGET)
        .rangeMapsToNative(c.closedNativePosition, c.openNativePosition)
        .targetExactlyFrom(stateCommand)
        .build();
```

Read only the new stages:

| Stage | Decision added by this lesson |
|---|---|
| `servo(...)` | Own one standard positional servo. |
| `position().nonPeriodic()` | Use one linear mechanism coordinate, not a wrapping angle. |
| `bounded(0.0, 1.0)` | Reject or clamp outside the named logical interval at the Plant boundary. |
| `rangeMapsToNative(closed, open)` | Map the logical endpoints into the configured native interval. |

`HALF` therefore derives halfway between the two configured native endpoints; with candidates
`0.25` and `0.70`, the submitted native command is `0.475`. It does not claim the linkage is
physically halfway open. The existing `targetExactlyFrom(...)`, `build()`, mechanism
`update(clock)`, `stop()`, `program.output(...)`, and callback-binding roles mean exactly what they
did in the intake lesson.

### Name status by the evidence available

[`BasicClaw.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.Status.html>)
reports the requested state, its requested normalized coordinate, and the Plant's cached applied
coordinate. The standard-servo adapter has no shaft or linkage measurement, so this capability
does not expose `atTarget()` or call an applied coordinate “arrived.”

Notice:

- The public capability still uses names; native servo numbers remain mechanism configuration.
- Logical bounds and native endpoint mapping are separate decisions in one private Plant.
- A recorded servo command is not evidence of linkage position, clearance, or arrival.

### Give the driver names, not servo numbers

The controls call the same semantic setter used everywhere else. In this teaching OpMode, A asks
for `CLOSED`, Y asks for `HALF`, and B asks for `OPEN`:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java -->
```java
requiredCallbacks.onRise(
        driver.a(),
        () -> requiredClaw.setState(BasicClaw.State.CLOSED));
requiredCallbacks.onRise(
        driver.y(),
        () -> requiredClaw.setState(BasicClaw.State.HALF));
requiredCallbacks.onRise(
        driver.b(),
        () -> requiredClaw.setState(BasicClaw.State.OPEN));
```

These are persistent requests: a button changes the requested state, and the normal output
heartbeat continues to apply it. The buttons do not wait for physical arrival because this servo
has no position feedback.

## Files in this checkpoint

**Main added here:**

- [`BasicClaw`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.html>) — capability and status.
  [Complete source: `BasicClaw.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.java>)
- [`BasicClawMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.html>) — mapping and Plant owner.
  [Complete source: `BasicClawMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java>)
- [`BasicClawProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.html>) — candidate endpoints and motion gate.
  [Complete source: `BasicClawProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.java>)
- `BasicClawControls` — rising-edge A/Y/B meanings.
  [Complete source: `BasicClawControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java>)
- [`BasicClawTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.html>) — mechanism-only managed host.
  [Complete source: `BasicClawTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.java>)

**Test:**

- [Complete source: `BasicClawSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawSoftwareScenarioTest.java>)

## Software checkpoint: normalized half derives from two endpoints

- **Question:** Does `HALF` remain a named request while its normalized midpoint maps through the
  same configured native range?
- **Keep real:** the production semantic command, servo Plant, bounds, and range mapping.
- **Replace:** only the FTC servo with a recording software device.
- **Observe:** requested state/coordinate, applied coordinate, and recorded native command.
- **Cannot conclude:** endpoint clearance, linkage geometry, interpolation, or physical arrival.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawSoftwareScenarioTest.java -->
```java
// HALF: semantic intent changes now; the normal heartbeat derives the native midpoint.
claw.setState(BasicClaw.State.HALF);
assertEquals(BasicClaw.State.HALF, claw.status().requestedState());
assertEquals(0.5, claw.status().requestedCoordinate(), 0.0);
assertEquals(1, servo.positionWrites());
claw.update(time.nextCycle(0.02));
assertEquals(0.5, claw.status().appliedCoordinate(), 0.0);
assertEquals(0.475, servo.position(), 1e-12);
assertEquals(2, servo.positionWrites());
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicClawSoftwareScenarioTest
```

**Read the causal chain:** `HALF` selects normalized `0.5`; the managed Plant heartbeat applies the
bounded target; the range map derives the native midpoint; the software servo records that command.

**Proves:** the named request and submitted command stay coherent through the configured mapping.

**Does not prove:** the claw is halfway open, reached the command, or avoided a mechanical stop.

## Isolated hardware gate

Keep `BasicClawTeleOp` disabled and `allowClawMotion` false while reviewing configuration. Confirm
that the horn is removable or the linkage can be tested without entering a pinch or hard-stop
region. Use [Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) with the horn or
linkage disconnected when practical to establish backed-off endpoints incrementally, then copy
those observations into `BasicClawProfile.current()` and reinstall the linkage. Only then enable
the mechanism OpMode and motion gate for a supervised A (`CLOSED`) → Y (`HALF`) → B (`OPEN`) run
while watching clearance and pinch zones.

**Next gate:** after recording the isolated endpoint run, continue to
[establishing a lift reference](<Referenced Lift.md>). It keeps the same ownership path and adds
encoder units, an explicit reference requirement, and switch evidence.
