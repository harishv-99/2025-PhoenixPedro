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

### Keep the active endpoint candidates and permission together

This lesson moves into the independent `basicmechanisms` example package. Its active edit point is
`BasicClawProfile.current()`, where the complete software baseline, robot-specific candidate
overrides, and fail-closed permission remain together:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.java -->
```java
public static BasicClawProfile current() {
    BasicClawProfile profile = new BasicClawProfile();
    profile.claw = BasicClawMechanism.Config.defaults();
    profile.claw.servoName = "clawServo";
    profile.claw.direction = Direction.FORWARD;
    profile.claw.closedNativePosition = 0.25;
    profile.claw.openNativePosition = 0.70;
    profile.claw.initialState = BasicClaw.State.CLOSED;
    profile.allowClawMotion = false;
    return profile;
}
```

`Config.defaults()` supplies a complete compiling baseline; the following assignments deliberately
show every active answer a team should review. The two numbers are software-valid candidates in
native interval `[0, 1]`, not reviewed safe linkage endpoints. Unlike the intake's initial
zero-power request, the first successful output heartbeat may move the servo toward `CLOSED`.
Keep `allowClawMotion` false until that initial motion has a safe test setup.

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
`update(clock)`, `program.output(...)`, and callback-binding roles mean exactly what they did in
the intake lesson. `stop()` is still terminal lifecycle cleanup, but its hardware effect is
different: a standard servo has no zero-power command. After a position has been submitted, the
FTC adapter reasserts that last successful position during stop. STOP does not select normalized
zero, `CLOSED`, `OPEN`, or a mechanical release.

### Name status by the evidence available

The mechanism creates status from one coherent semantic/Plant snapshot:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java -->
```java
@Override
public Status status() {
    return new Status(stateCommand.snapshot(claw.snapshot()));
}
```

The capability-shaped wrapper retains exactly that one delegate and reads the semantic request
from it:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.java -->
```java
final class Status {
    private final SemanticScalarSnapshot<State, PlantSnapshot> delegate;

    /** Wraps one coherent semantic request and scalar Plant snapshot. */
    public Status(SemanticScalarSnapshot<State, PlantSnapshot> delegate) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
    }

    /** Returns the most recent semantic claw request. */
    public State requestedState() {
        return delegate.request().semantic();
    }
```

Its numeric accessors delegate to the paired command and Plant facts rather than copying them:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.java -->
```java
/** Returns the normalized coordinate paired with the semantic request. */
public double requestedCoordinate() {
    return delegate.request().commandTarget();
}

/**
 * Returns the Plant's cached final normalized target after bounds and guards.
 * This is a software command fact, not standard-servo position feedback.
 */
public double appliedCoordinate() {
    return delegate.plant().appliedTarget();
}
```

[`BasicClaw.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.Status.html>)
is a thin immutable view over that one snapshot. Its `requestedState()`, `requestedCoordinate()`,
and `appliedCoordinate()` methods only rename already-captured facts for claw callers; they do not
poll the servo or maintain another cache. The standard-servo adapter has no shaft or linkage
measurement, so this capability does not expose `atTarget()` or call an applied coordinate
“arrived.”

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

### Put this focused fixture into your robot

The focused OpMode reads the active profile, checks permission before hardware construction,
declares the mechanism as the managed output, and binds the controls to the capability:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.java -->
```java
BasicClawProfile clawProfile = BasicClawProfile.current();
BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Claw TeleOp");

BasicClawMechanism claw = program.output(
        new BasicClawMechanism(hardwareMap, clawProfile.claw));
GamepadDevice driver = new GamepadDevice(gamepad1);
BasicClawControls clawControls = new BasicClawControls(driver);
clawControls.bind(program.callbackBindings(), claw);
```

The same composition root declares the cached evidence students should watch:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.java -->
```java
program.presenter((clock, telemetry) -> {
    BasicClaw.Status status = claw.status();
    telemetry.addData("claw.request", status.requestedState());
    telemetry.addData("claw.appliedCoordinate", "%.2f", status.appliedCoordinate());
    telemetry.addLine("claw coordinate is a normalized target, not servo feedback");
});
```

`program.output(...)` gives the same managed `update` and terminal `stop` ownership introduced by
the intake lesson. For this standard-servo Plant, terminal stop reasserts the last submitted
position rather than zeroing or relaxing it. `program.presenter(...)` runs after outputs and reads
the immutable `claw.status()` view; its label deliberately calls the applied coordinate a
normalized target, not feedback. There is no student-written loop or telemetry commit.

Notice:

- The public capability still uses names; native servo numbers remain mechanism configuration.
- Logical bounds and native endpoint mapping are separate decisions in one private Plant.
- A recorded servo command is not evidence of linkage position, clearance, or arrival.

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

- **Question:** Do the named positions map through one configured native range, and does terminal
  standard-servo stop retain the last submitted command?
- **Keep real:** the production semantic command, servo Plant, bounds, and range mapping.
- **Replace:** only the FTC servo with a recording software device.
- **Observe:** requested state/coordinate, applied coordinate, recorded native command, and the
  command reasserted at stop.
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

The test then explicitly selects `OPEN` before making the different standard-servo stop effect
visible:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawSoftwareScenarioTest.java -->
```java
// OPEN: the same mapping sends normalized 1.0 to the configured native upper endpoint.
claw.setState(BasicClaw.State.OPEN);
claw.update(time.nextCycle(0.02));
assertEquals(1.0, claw.status().appliedCoordinate(), 0.0);
assertEquals(0.70, servo.position(), 1e-12);
assertEquals(3, servo.positionWrites());

// STOP: a standard servo has no zero-power command, so lifecycle stop reasserts OPEN.
int writesBeforeStop = servo.positionWrites();
claw.stop();
assertEquals(writesBeforeStop + 1, servo.positionWrites());
assertEquals(0.70, servo.position(), 1e-12);
```

**Proves:** the named request and submitted command stay coherent through the configured mapping,
and terminal standard-servo stop reasserts the last submitted position rather than inventing zero
power or another semantic state.

**Does not prove:** the claw is halfway open, reached the command, or avoided a mechanical stop.

## Isolated hardware gate

Keep `BasicClawTeleOp` disabled and `allowClawMotion` false while reviewing configuration. Confirm
that the horn is removable or the linkage can be tested without entering a pinch or hard-stop
region. Use [Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) with the horn or
linkage disconnected when practical to establish backed-off endpoints incrementally, then copy
those observations into `BasicClawProfile.current()` and reinstall the linkage. Only then enable
the mechanism OpMode and motion gate for a supervised A (`CLOSED`) → Y (`HALF`) → B (`OPEN`) run
while watching clearance and pinch zones. Pressing OpMode STOP terminally ends managed updates but
reasserts the last standard-servo position command; it is not an open, zero-power, or release
action. De-energize the robot before touching the horn or linkage.

**Next gate:** after recording the isolated endpoint run, continue to
[establishing a lift reference](<Referenced Lift.md>). It keeps the same ownership path and adds
encoder units, an explicit reference requirement, and switch evidence.
