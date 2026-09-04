---
tags:
  - Build
---

# Run a continuous intake by name

**Outcome:** press one button to request `COLLECT`, another to request `EJECT`, and a third to
request `STOPPED`, while one managed mechanism owns every motor write.

**Prerequisites:** the project software checks pass. No prior Plant, capability, binding, or Task
knowledge is assumed. Keep the intake motor disconnected until the isolated hardware gate.

## First pass: run a function once per press

Start with the familiar event, before the motor internals. This exact production registration says,
“when A changes from released to pressed, call `setMode(COLLECT)` once”:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java -->
```java
requiredCallbacks.onRise(
        driver.a(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
```

The second argument is a Java **lambda**. `()` means it needs no arguments, and `->` separates that
empty argument list from the method call to run later. Passing the lambda to `onRise(...)` saves the
call; registering it during INIT does **not** execute `setMode(...)`.

After the first button sample establishes whether A started pressed or released, the important part
is equivalent to this familiar loop code. This is a timing comparison, not a second robot-code
architecture or production implementation:

```text
boolean aIsPressedNow = gamepad1.a;
if (aIsPressedNow && !aWasPressedLastLoop) {
    intake.setMode(StarterIntake.Mode.COLLECT);
}
aWasPressedLastLoop = aIsPressedNow;
```

During a later active FTC loop, an accepted released-to-pressed change invokes the saved lambda
synchronously: the call finishes right there in the button-checking part of that same loop. It does
not start a thread. The intake's output update comes afterward in the loop, so the new request can
reach the motor command in that cycle.

| Button history | What happens |
| --- | --- |
| First sample released | Establish the starting value; do not call the lambda. |
| Press A | Call `setMode(COLLECT)` once. |
| Hold A | Do not call it again; the `COLLECT` request remains selected. |
| Release A | Do not call it; the `COLLECT` request still remains selected. |
| Press A again | Accept a new rise and call it once again. |

In the simple A/X trace, releasing A does not stop the intake: `COLLECT` persists until X selects
`STOPPED`. The production B button can replace it with `EJECT`. The full build below shows how those
named requests become one managed motor command and how STOP cleanup remains safe.

## Full build: reconstruct the production path

**Start here:** this is the complete first actuator lesson. It follows one request from the public
robot meaning through configuration, Plant construction, managed update, status, controls, the
OpMode host, a software test, and finally supervised hardware evidence.

## Critical production idea

A **capability** is the small vocabulary TeleOp and Auto share. For this mechanism, callers ask for
a named `Mode`; they never repeat motor-power numbers:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java -->
```java
enum Mode {
    STOPPED,
    COLLECT,
    EJECT
}
```

`StarterIntake` also declares `setMode(...)`, `collectForSeconds(...)`, and `status()`. The
interface says what the robot can do. It does not expose an FTC motor or a Plant.

The **mechanism** is the private hardware realization behind that interface. It copies the robot's
configuration, maps the named request, owns the Plant, and exposes only the capability to clients.

### 1. Keep physical answers in data-only configuration

The mechanism configuration owns the FTC name, logical direction, and normalized action powers.
In the maintained host, edit the active candidate values together in `StarterProfile.current()`:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.java -->
```java
profile.intake = StarterIntakeMechanism.Config.defaults();
profile.intake.motorName = "intakeMotor";
profile.intake.direction = Direction.FORWARD;
profile.intake.collectPower = 0.20;
profile.intake.ejectPower = -0.20;
profile.allowIntakeMotion = false;
```

`Config.defaults()` first supplies a complete software baseline; the profile is the one active edit
point and keeps its fail-closed motion permission beside those candidates. None of these lines
proves the name, direction, power, or mechanism is safe on your robot. The mechanism constructor
copies and validates every retained value before hardware lookup.

### 2. Map each name forward once

[`SemanticScalarCommand`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarCommand.html>)
keeps the selected name and its numeric command together. `STOPPED` is the initial request and maps
to software target zero:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
modeCommand = SemanticScalarCommand.forEnum(Mode.STOPPED)
        .map(Mode.STOPPED, 0.0)
        .map(Mode.COLLECT, copiedCollectPower)
        .map(Mode.EJECT, copiedEjectPower)
        .build();
```

The mechanism maps from meaning to power. It never tries to infer a meaning from a number, so two
modes could even have the same configured power without corrupting status.

“Persistent” means the selected request stays in effect across loop cycles until a setter or Task
replaces it; student code does not resend it in a private loop.

### 3. Build the one final hardware writer

A **Plant** is the mechanism-owned object that resolves one requested target, caches the resulting
facts, and performs the final actuator write during its managed update:

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

Read the staged builder as five concrete questions:

| Stage | Question answered |
|---|---|
| `plant(hardwareMap)` | Which FTC registry will this mechanism use? |
| `motor(motorName, direction)` | Which one motor does it own, and which sign is logically forward? |
| `power()` | Is the public target normalized power in `[-1, +1]`? |
| `targetExactlyFrom(modeCommand)` | Which persistent request is the sole source of the target? |
| `build()` | Are all required answers present so the object graph can be completed? |

`build()` performs construction and configuration; it does **not** command motion. The initial
command is zero because `Mode.STOPPED` mapped to zero. A later output heartbeat applies the held
request.

### 4. Separate requests, cached status, update, and stop

Calling the capability setter changes the persistent request immediately but does not write the
motor:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
@Override
public void setMode(Mode mode) {
    modeCommand.set(Objects.requireNonNull(mode, "mode"));
}
```

Status combines that named request with one immutable snapshot of already-cached Plant facts. It
does not poll hardware:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
@Override
public Status status() {
    return new Status(modeCommand.snapshot(plant.snapshot()));
}
```

The managed output phase calls `update(clock)` once per active cycle. Total cleanup calls terminal
`stop()`; ordinary button callbacks call neither method:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
@Override
public void update(LoopClock clock) {
    plant.update(clock);
}

@Override
public void stop() {
    plant.stop();
}
```

### 5. Declare the output owner once

The composition root constructs the mechanism with `HardwareMap` plus its data-only config and
declares the mechanism—not its private Plant—as the managed output:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
private StarterIntakeMechanism declareIntake(RobotProgram program,
                                              StarterProfile profile) {
    StarterIntakeMechanism intake = program.output(
            new StarterIntakeMechanism(hardwareMap, profile.intake));
    program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
    return intake;
}
```

[`RobotProgram.output(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html>)
registers the owner's update and cleanup. The presenter reads cached capability status after the
output phase; it does not decide behavior or commit a separate telemetry frame.

### 6. Give buttons semantic meaning

The composition root creates one controls owner around the FTC gamepad adapter, then connects it to
the managed callback graph and the capability returned by the mechanism declaration:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
StarterIntakeMechanism intake = declareIntake(program, activeProfile);
StarterTeleOpControls controls = new StarterTeleOpControls(
        new GamepadDevice(requiredGamepad));
controls.bind(program.callbackBindings(), intake);
```

Inside that owner, controls give each rising edge a semantic meaning. They call the capability and
never reach into hardware:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java -->
```java
requiredCallbacks.onRise(
        driver.a(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
requiredCallbacks.onRise(
        driver.b(),
        () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
requiredCallbacks.onRise(
        driver.x(),
        () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
```

`program.callbackBindings()` observes those sources in the managed bindings phase. A press changes
the request; the downstream output phase realizes it during the same managed cycle.

### 7. Put the complete slice under the managed host

The focused OpMode chooses the profile and asks the composition root to declare this slice:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterIntakeTeleOp.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareIntakeTeleOp(program, profile, gamepad1);
}
```

After `configure(...)` returns, [`FtcRobotOpMode`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcRobotOpMode.html>)
and `RobotProgram` own the one clock and recurring order. Student code does not add a loop:

```text
button source -> callback binding -> semantic request -> Plant update -> motor write -> presenter
```

At FTC STOP or a runtime failure, the managed host best-effort stops declared outputs. During an
active match, request `STOPPED`; do not terminally stop a Plant that must be reused.

Notice:

- The capability owns robot meaning, the controls own button meaning, and the mechanism owns hardware.
- One persistent request reaches one Plant and one final motor writer on the shared heartbeat.
- Requested or applied power is a software command fact, not proof that the motor turned.

## Files in this checkpoint

**Main:**

- [`StarterIntake`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.html>) — capability API.
  [Complete source: `StarterIntake.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>)
- [`StarterIntakeMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.html>) — semantic mapping and private Plant owner.
  [Complete source: `StarterIntakeMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java>)
- [`StarterProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.html>) — configuration and motion gate.
  [Complete source: `StarterProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.java>)
- `StarterTeleOpControls` — A/B/X button meanings.
  [Complete source: `StarterTeleOpControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java>)
- [`StarterRobot`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.html>) — composition and managed declarations.
  [Complete source: `StarterRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java>)
- [`StarterIntakeTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterIntakeTeleOp.html>) — mechanism-only host.
  [Complete source: `StarterIntakeTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterIntakeTeleOp.java>)

**Test:**

- [Complete source: `StarterMechanismLessonTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterMechanismLessonTest.java>)

## Software checkpoint: request first, apply on heartbeat

- **Question:** Does one semantic request reach the configured motor power through the production
  Plant update?
- **Keep real:** the production capability, mechanism, command mapping, and Plant.
- **Replace:** only the FTC motor with a recording software device.
- **Observe:** semantic status, cached applied power, and the recorded device write.
- **Cannot conclude:** physical direction, safe ingestion, current draw, jamming, or stopping time.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterMechanismLessonTest.java -->
```java
// REQUEST: semantic intent changes immediately; hardware has not been updated yet.
intake.setMode(StarterIntake.Mode.COLLECT);
assertEquals(StarterIntake.Mode.COLLECT, intake.status().mode());
assertEquals(0.0, intake.status().appliedPower(), 0.0);
assertEquals(0, motor.powerWrites());

// HEARTBEAT: the same production update path maps the request and writes the motor.
intake.update(time.clock());
assertEquals(config.collectPower, intake.status().appliedPower(), 0.0);
assertEquals(config.collectPower, motor.power(), 0.0);
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterMechanismLessonTest
```

**Read the causal chain:** a button or test call changes one named request; the next normal output
heartbeat resolves it; the software motor records the boundary write.

**Proves:** semantic mapping, update order, cached status, and the submitted software command.

**Does not prove:** the mechanism collects, ejects, or stops safely on the physical robot.

## Isolated hardware gate

Keep `StarterIntakeTeleOp` disabled and `allowIntakeMotion` false while reviewing configuration.
Restrain loose material, disconnect other motion owners, verify the motor name and direction, and
appoint an immediate STOP operator. Use [Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>)
for a low-power dead-man direction check before copying the observed direction into
`StarterProfile.current()`. Only then enable the focused OpMode and set the gate true for a
supervised low-power run that begins from `STOPPED`.

**Next gate:** after isolated direction and stop behavior are established, continue to
[named claw positions](<Named Claw.md>). It reuses this complete owner/heartbeat path and teaches
only bounded standard-servo position mapping.
