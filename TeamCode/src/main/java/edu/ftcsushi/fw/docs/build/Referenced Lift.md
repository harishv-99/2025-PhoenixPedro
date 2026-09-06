---
tags:
  - Build
---

# Establish a lift reference

**Outcome:** use a bottom switch to establish where encoder position zero means, without blocking
the loop or pretending that a software-valid coordinate is already physically known.

**Optional feedback lesson. Knowledge before this page:** understand the owner/Plant path in
[Continuous Intake](<Continuous Intake.md>), polarity and debounce in [Read a Switch](<Read a Switch.md>),
and cooperative work in [Run One Timed Auto](<Run One Timed Auto.md>). Reading those explanations is
enough. No installation, test run, lift, switch, or encoder is required to learn this page.

**Builds on:** one mechanism-owned Plant, data-only configuration, a managed output heartbeat,
capability status, controls, and the separation between a command and physical evidence.

**New here:** an encoder coordinate remains invalid until a real reference cue establishes it; a
non-blocking calibration Task temporarily searches with low power, and exact success selects one
semantic `STOWED` hold while the same mechanism remains the sole Plant heartbeat owner.

## Critical production idea

An **encoder** counts shaft movement in increments called **ticks**; the count alone does not tell
you the lift's height after startup. A **coordinate reference** ties one count to a known height.
**Homing** means moving carefully until a known cue, here the bottom switch, establishes that tie.
The example calls that height zero; it does not secretly reset the motor's encoder.

**Feedback** is a returned measurement. A position **controller** repeatedly compares the requested
height with measured height and adjusts motor effort to reduce the difference, or **error**. Here
the FTC controller does that work. Sushi chooses the request, limits, and completion rule.

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

The example's `ticksPerIn = 100.0` means 100 encoder counts per inch; after reference, a four-inch
request corresponds to 400 counts from zero. **Tolerance** is how close a measurement must be to
the target for software completion; `0.20 in` allows an error of at most two tenths of an inch.
Those are example choices to measure on a real lift, not universal conversions.

The mechanism validates the complete shared lift coordinate before any hardware lookup, even
though this page exercises only homing:

| Active candidates | Relationship that must remain true |
|---|---|
| `maximumHeightIn = 18.0`, `ticksPerIn = 100.0` | The maximum and scale are finite and greater than zero. |
| `STOWED = 0.0`, `LOW = 4.0`, `HIGH = 14.0` inches | `0 <= STOWED < LOW < HIGH <= maximumHeightIn`. |
| `toleranceIn = 0.20` | It is positive and strictly less than half the closest named-height gap: `0.20 < 2.0` here. The next lesson uses this non-overlapping arrival rule. |
| `maximumPower = 0.30`, `homingPower = -0.15` | Maximum power is in `(0, 1]`; search power is in `[-1, 0)` by this mechanism's convention. |
| home `3.0 s`, move `2.0 s` | Both timeout budgets are finite and greater than zero. |

Blank hardware names, a missing direction, non-finite values, or a broken relationship reject
construction with an actionable error. This proves coherent software configuration, not that the
numbers describe the physical lift.

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

The switch source is explicitly active-low: electrical LOW means pressed. The mechanism wraps
that source with 0.02 seconds of on/off debounce, reusing the sampled filtering taught in
[Read a Switch](<Read a Switch.md#first-pass-observations-every-loop>). Switch polarity and placement
are authored configuration facts that only a physical check can validate.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
bottomSwitch = FtcSensors.digitalLow(
        map, c.bottomSwitchName)
        .debouncedOnOff(0.02, 0.02);
```

`digitalLow(...)` performs the electrical LOW-to-pressed interpretation. `debouncedOnOff(...)`
accumulates each differing sample's preceding loop interval toward the `0.02 s` delay; observing
the already-accepted value clears that pending time. This applies to both press and release.
It filters flicker visible in the samples, but one sample after a long loop can meet the delay.
It cannot prove continuous physical stability, where the switch is mounted, or that the lift is
at bottom. Unlike the switch-only service, this private source is sampled by the active home Task
when it needs the reference cue.

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

The search recipe means “apply this search power until the switch is accepted, establish zero,
and fail if the time budget expires.” `Tasks.sequence(...)` saves two Tasks to run in order: only
the first one's `SUCCESS` starts the second. Constructing the sequence does not start either Task.

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

`requiredLift::home` is a Java **method reference**, shorthand for `() -> requiredLift.home()`.
It saves a factory call, not a Task object. Unlike a short callback setter, a **Task binding** builds
and queues work: each accepted X press calls the factory, and the managed Task phase runs the new
Task. A queue holds later work until the current Task finishes. `claimBind()` is this controls
owner's helper for rejecting a second registration attempt.

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

**Expected observations:** HIGH leaves the active-low switch released and the lift unreferenced.
After search starts, LOW samples accumulating the configured `0.02` seconds of loop intervals
establish zero, end the home Task with `SUCCESS`, and select `STOWED`. If LOW never arrives, the
configured timeout ends search
with `TIMEOUT` and zero search power while the prior named request remains unchanged. Read the
causal example below; running it is optional.

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
Scenario scenario = new Scenario();

// ARRANGE: HIGH is the explicitly authored "not pressed" state of this active-low switch.
scenario.motor.setCurrentPositionTicks(0);
scenario.bottomSwitch.setHigh(true);
scenario.lift.setHeight(BasicLift.Height.LOW);
assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
assertEquals(0, scenario.motor.targetPositionWrites());
```

The request then starts and first-updates a fresh home Task before the normal output heartbeat
applies search power. The assertions observe that no switch hit was invented:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
// REQUEST: start a cooperative home Task; it may command search power but cannot invent a hit.
scenario.task = scenario.lift.home();
scenario.task.start(scenario.time.clock());
scenario.task.update(scenario.time.clock());
scenario.lift.update(scenario.time.clock());
assertFalse(scenario.task.isComplete());
assertEquals(scenario.config.homingPower, scenario.motor.power(), 0.0);
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

The second scenario arranges a released switch, starts the same request, and performs the same
Task-then-output heartbeat without ever injecting a pressed observation:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
Scenario scenario = new Scenario();
scenario.bottomSwitch.setHigh(true); // The authored switch observation never becomes pressed.
scenario.lift.setHeight(BasicLift.Height.LOW);
assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
scenario.task = scenario.lift.home();
scenario.task.start(scenario.time.clock());
scenario.task.update(scenario.time.clock());
scenario.lift.update(scenario.time.clock());
```

Advancing the configured time budget then lets the Task heartbeat observe timeout, leaves the
Plant unreferenced, and releases temporary search power:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
scenario.advance(scenario.config.homingTimeoutSec);

assertEquals(TaskOutcome.TIMEOUT, scenario.task.getOutcome());
assertFalse(scenario.lift.status().referenced());
assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
assertEquals(0.0, scenario.motor.power(), 0.0);
assertEquals(0, scenario.motor.targetPositionWrites());
```

Optionally run the maintained scenario after [software setup](<../getting-started/Build and Run.md>):

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftSoftwareScenarioTest
    ```

**Read the causal chain:** the test explicitly supplies switch evidence across real clock cycles;
arrangement leaves it released, the request starts search, and the output heartbeat applies search
power. The first pressed observation is shorter than the debounce interval; the second lets the
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

**Reading checkpoint:** explain why `referenced` is initially false, why a brief LOW does not
finish homing, and why timeout cannot establish zero. This optional fixture adds a motor and a
reference Task to familiar switch evidence; it does not require constructing the earlier claw.
Use the [robot-package guidance](<README.md#author-in-your-robot>) if you choose to author it.

## Isolated hardware gate

This separate procedure applies only if you choose to operate a real lift.

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

**Next gate:** read [moving the referenced lift](<Move a Referenced Lift.md>) to add direct named
moves and fresh arrival evidence. A physical move requires repeatable low-power homing first;
reading the next explanation does not.
