---
tags:
  - Build
---

# Establish a lift reference

**Outcome:** use a bottom switch to establish where encoder position zero means, without blocking
the loop or pretending that a software-valid coordinate is already physically known.

**Prerequisites:** complete [the named-claw lesson](<Named Claw.md>) through its software
checkpoint. No lift, switch, or encoder is required before this page's isolated hardware gate, and
passing software tests does not authorize motion.

**Builds on:** one mechanism-owned Plant, data-only configuration, a managed output heartbeat,
capability status, controls, and the separation between a command and physical evidence.

**New here:** an encoder coordinate remains invalid until a real reference cue establishes it; a
non-blocking calibration Task temporarily searches with low power, and exact success selects one
semantic `STOWED` hold while the same mechanism remains the sole Plant heartbeat owner.

## Critical production idea

A motor encoder reports changes in ticks, not an absolute lift height after startup. The team must
author the relationship between hardware and mechanism units, then establish one trustworthy
coordinate reference before ordinary position requests can be realized.

### Put every active coordinate answer behind one motion lock

This lesson uses the independent `basicmechanisms` example package. Its one active edit point is
`BasicLiftProfile.current()`. The first half keeps wiring, coordinate scale, tolerance, and output
limit together:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.java -->
```java
public static BasicLiftProfile current() {
    BasicLiftProfile profile = new BasicLiftProfile();
    profile.lift = BasicLiftMechanism.Config.defaults();
    profile.lift.motorName = "liftMotor";
    profile.lift.direction = Direction.FORWARD;
    profile.lift.bottomSwitchName = "liftBottom";
    profile.lift.maximumHeightIn = 18.0;
    profile.lift.ticksPerIn = 100.0;
    profile.lift.toleranceIn = 0.20;
    profile.lift.maximumPower = 0.30;
```

The same method keeps named positions, homing/move timing, and fail-closed permission beside them:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.java -->
```java
    profile.lift.stowedHeightIn = 0.0;
    profile.lift.lowHeightIn = 4.0;
    profile.lift.highHeightIn = 14.0;
    profile.lift.homingPower = -0.15;
    profile.lift.homingTimeoutSec = 3.0;
    profile.lift.moveTimeoutSec = 2.0;
    profile.allowLiftMotion = false;
    return profile;
}
```

`Config.defaults()` supplies a complete compiling baseline; these explicit assignments are the
active candidates a team reviews. None proves the motor or switch identity, direction, scale,
range, power, timeouts, or heights on a robot. Leave `allowLiftMotion` false through the software
checkpoint and initial physical setup.

### Declare the coordinate and its reference requirement

`BasicLiftMechanism.Config` keeps the motor and active-low switch names, direction, maximum height
in inches, ticks per inch, power limit, search power, timeouts, and named-height values together.
The mechanism owns the fixed debounce behavior. Its defaults are compiling software candidates,
not measured robot facts.

The private Plant answers the position-specific construction questions:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
lift = FtcActuators.plant(map)
        .motor(c.motorName, c.direction)
        .position()
        .deviceManaged()
        .nonPeriodic()
        .bounded(0.0, c.maximumHeightIn)
        .scaleToNative(c.ticksPerIn)
        .needsReference("basic lift has not been homed")
        .positionTolerance(c.toleranceIn)
        .outputPowerLimitedTo(c.maximumPower)
        .targetExactlyFrom(heightCommand)
        .build();
```

Read the new stages in order:

| Stage | Reference-related decision |
|---|---|
| `position().deviceManaged()` | The motor controller owns the position loop and exposes encoder feedback. |
| `nonPeriodic().bounded(0.0, maximumHeightIn)` | Lift inches do not wrap and legal targets stay inside one travel interval. |
| `scaleToNative(ticksPerIn)` | Convert between mechanism inches and native encoder ticks. |
| `needsReference(...)` | Keep the target range invalid until calibration establishes the coordinate. |
| `outputPowerLimitedTo(maximumPower)` | Bound the controller's normalized power; this is not a claim that the value is physically safe. |

`build()` still does not start motion. Before reference, the Plant reports why its target range is
unavailable instead of treating the current encoder count as known height.

### Turn an electrical observation into stable evidence

The switch source is explicitly active-low: electrical LOW means pressed. The mechanism decorates
that source with 0.02 seconds of on/off debounce, so one brief sample does not establish the
reference. Switch polarity and placement are authored configuration facts that only a physical
check can validate.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
bottomSwitch = FtcSensors.digitalLow(
        map, c.bottomSwitchName)
        .debouncedOnOff(0.02, 0.02);
```

`digitalLow(...)` performs the electrical LOW-to-pressed interpretation. `debouncedOnOff(...)`
requires the interpreted value to remain stable in clock time before publishing either edge; it
does not inspect where the switch is mounted.

### Search cooperatively, then publish policy only on success

The mechanism establishes the semantic height vocabulary before it constructs the Plant. This
page uses only `STOWED` as the success-only post-home hold; the next lesson teaches direct and
feedback-waiting requests for all three names:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
heightCommand = SemanticScalarCommand.forEnum(Height.STOWED)
        .map(Height.STOWED, c.stowedHeightIn)
        .map(Height.LOW, c.lowHeightIn)
        .map(Height.HIGH, c.highHeightIn)
        .build();
```

[`PositionCalibrationTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/PositionCalibrationTasks.html>)
owns the temporary search request, cue, reference value, timeout, and release. It never calls
`lift.update(clock)`; the mechanism remains the one Plant heartbeat and final hardware writer:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
Task search = PositionCalibrationTasks.search(lift)
        .withPower(homingPower)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .failAfterSec(homingTimeoutSec)
        .build();

return Tasks.sequence(
        search,
        SemanticScalarTasks.set(heightCommand, Height.STOWED).build());
```

The managed order is `Tasks -> Outputs`. A cycle first updates the homing Task, then the output
phase lets the same Plant apply either temporary search power or its ordinary target. Exact search
success establishes zero and admits the `STOWED` request before that downstream output phase.
Timeout or active cancellation releases search power, does not establish a reference, and does not
run the success-only `STOWED` step.

[`BasicLift.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.Status.html>)
exposes `referenced()` from the Plant snapshot. The debounced switch stays private to the mechanism;
its public effect is that an active home Task may establish reference. Reading status performs no
extra sensor poll or duplicate status publication.
The reference checkpoint has a deliberately narrow controls owner. It maps only X to a fresh home
Task, so a student cannot accidentally invoke the later named-move controls while establishing the
first reference:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftHomeControls.java -->
```java
void bind(TaskBindings tasks, BasicLift lift) {
    TaskBindings requiredTasks = Objects.requireNonNull(tasks, "tasks");
    BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
    claimBind();

    requiredTasks.onRise(driver.x(), requiredLift::home);
}
```

`TaskBindings.onRise(...)` accepts the method reference as a factory: every X rise calls `home()`
again and queues a fresh single-use Task.

### Put this focused fixture into your robot

The disabled `BasicLiftHomeTeleOp` reads the active profile, checks permission before constructing
hardware, declares the mechanism as the output owner, and binds only the X control:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftHomeTeleOp.java -->
```java
BasicLiftProfile profile = BasicLiftProfile.current();
BasicLiftProfile.requireMotionAllowed(profile, "Basic Lift Home TeleOp");

BasicLiftMechanism lift = program.output(
        new BasicLiftMechanism(hardwareMap, profile.lift));
BasicLiftHomeControls controls = new BasicLiftHomeControls(
        new GamepadDevice(gamepad1));
controls.bind(program.taskBindings(), lift);
```

The focused presenter exposes only the evidence needed to reason about this home attempt:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftHomeTeleOp.java -->
```java
program.presenter((clock, telemetry) -> {
    BasicLift.Status status = lift.status();
    telemetry.addData("lift.request", status.requestedHeight());
    telemetry.addData("lift.positionIn", "%.2f / %.2f",
            status.measuredPositionIn(), status.requestedPositionIn());
    telemetry.addData("lift.referenced", status.referenced());
    telemetry.addLine("X: home");
});
```

`program.presenter(...)` reports the held semantic request, cached measured/requested inches, and
reference state after the output heartbeat. `FtcRobotOpMode` supplies the one managed clock,
`Tasks -> Outputs -> Presenters` order, and telemetry commit; student code adds no loop. This page
uses only switch/reference behavior and the success-only `STOWED` hold. Requesting another height
and waiting for feedback is the next lesson.

Notice:

- Encoder ticks become mechanism inches only through an authored conversion and established reference.
- The calibration Task temporarily proposes output, while the mechanism's Plant remains the sole writer.
- Only exact switch-backed search success establishes reference and selects the post-home request.

## Files in this checkpoint

**Main added here:**

- [`BasicLift`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.html>) — semantic capability and evidence.
  [Complete source: `BasicLift.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.java>)
- [`BasicLiftMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.html>) — switch, reference Task, and Plant owner.
  [Complete source: `BasicLiftMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java>)
- [`BasicLiftProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.html>) — coordinate candidates and motion gate.
  [Complete source: `BasicLiftProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.java>)
- `BasicLiftHomeControls` — the reference lesson's X-only fresh home binding.
  [Complete source: `BasicLiftHomeControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftHomeControls.java>)
- [`BasicLiftHomeTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftHomeTeleOp.html>) — disabled home-only managed host.
  [Complete source: `BasicLiftHomeTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftHomeTeleOp.java>)

**Test:**

- [Complete source: `BasicLiftSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java>)

## Software checkpoint: authored switch evidence controls homing

- **Question:** Does the authored active-low switch observation establish reference only after its
  debounce interval and apply the configured `STOWED` hold, while missing evidence times out safely?
- **Keep real:** the production lift, homing Task, switch interpretation, Plant, and loop order.
- **Replace:** only the motor and digital channel with recording software devices.
- **Observe:** search power, exact outcome, reference status, and the held semantic request.
- **Cannot conclude:** switch placement, wiring, motor polarity, conversion accuracy, limits, or
  safe motion.

Each scenario first selects `LOW` only as a software sentinel. The unreferenced Plant never submits
that position to the motor; the distinct name lets the test prove whether the success-only
continuation actually replaced the earlier request:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
scenario.motor.setCurrentPositionTicks(0);
scenario.bottomSwitch.setHigh(true);
scenario.lift.setHeight(BasicLift.Height.LOW);
assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
assertEquals(0, scenario.motor.targetPositionWrites());
```

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
// INJECT EVIDENCE: LOW must remain observed long enough to pass the configured debouncer.
scenario.bottomSwitch.setHigh(false);
scenario.advance(0.01);
assertFalse(scenario.task.isComplete());
int targetWritesBeforeSuccess = scenario.motor.targetPositionWrites();
scenario.advance(0.01);
assertEquals(TaskOutcome.SUCCESS, scenario.task.getOutcome());
assertTrue(scenario.lift.status().referenced());
assertEquals(BasicLift.Height.STOWED, scenario.lift.status().requestedHeight());
```

The same success-only step pairs the name with its configured inches and the downstream output
heartbeat submits that hold through the normal Plant path:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
assertEquals(scenario.config.stowedHeightIn,
        scenario.lift.status().requestedPositionIn(), 0.0);
assertEquals((int) Math.round(
                scenario.config.stowedHeightIn * scenario.config.ticksPerIn),
        scenario.motor.targetPositionTicks());
assertEquals(targetWritesBeforeSuccess + 1, scenario.motor.targetPositionWrites());
```

The second scenario supplies no pressed observation. The Task reports timeout, leaves the Plant
unreferenced, and releases temporary search power:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
scenario.advance(scenario.config.homingTimeoutSec);

assertEquals(TaskOutcome.TIMEOUT, scenario.task.getOutcome());
assertFalse(scenario.lift.status().referenced());
assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
assertEquals(0.0, scenario.motor.power(), 0.0);
assertEquals(0, scenario.motor.targetPositionWrites());
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftSoftwareScenarioTest
```

**Read the causal chain:** the test explicitly supplies switch evidence across real clock cycles;
the first observation is shorter than the debounce interval; the second observation lets the
production homing Task establish zero, select `STOWED`, and let the downstream output hold it. The
separate missing-evidence scenario reaches the configured timeout without establishing reference
and releases the temporary search output while preserving the `LOW` sentinel. That contrast proves
the `STOWED` continuation belongs to exact search success, rather than merely observing the
mechanism's construction-time default.

**Proves:** software polarity interpretation, debounce timing, exact success/timeout outcomes,
reference transition, search-power release, and success-only semantic `STOWED` policy for the
authored observations.

**Does not prove:** the real switch changes safely, the lift travels toward it, or the encoder scale
and power limit are correct.

## Isolated hardware gate

Keep `BasicLiftHomeTeleOp` disabled and `allowLiftMotion` false while reviewing configuration.
Write the switch-polarity, motor-direction, travel-envelope, and emergency-stop check plan first.
Mechanically support the lift and start away from hard stops. Use
[Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) only if a mechanism-specific
support plan makes its generic jog appropriate; otherwise build a lift-specific fixture using the
same low-power, dead-man evidence discipline. Establish direction and a backed-off travel span
before setting `allowLiftMotion = true`, removing `@Disabled`, and starting the home-only OpMode.
That host exposes no D-pad named-move bindings. For the first polarity check, mechanically
disengage the motor from the lift or use an equivalent fixture that prevents linkage motion while
allowing the motor to run briefly. Restart the OpMode with the switch released and confirm
`lift.referenced = false`. Press X:
reference must remain false until you manually hold the switch pressed through its debounce, then
become true. If it becomes true before the manual press, stop and correct the polarity or wiring.
After the isolated polarity proof, press STOP, de-energize the robot, and only then reconnect the
mechanism. The next OpMode run correctly starts unreferenced. With the lift supported and its path
clear, press X for real low-power homing while an immediate STOP operator watches the motion.
Record repeatable activation and reference status; do not try a named-height move yet, and never
touch or reconnect the linkage while the OpMode or controller is active.

**Next gate:** after repeatable low-power homing is established, continue to
[moving the referenced lift](<Move a Referenced Lift.md>). That page adds direct named moves,
fresh position feedback, and the explicit success/timeout/cancellation contract.
