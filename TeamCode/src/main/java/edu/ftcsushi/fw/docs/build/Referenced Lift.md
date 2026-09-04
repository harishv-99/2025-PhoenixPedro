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
non-blocking calibration Task temporarily searches with low power while the same mechanism remains
the sole Plant heartbeat owner.

## Critical production idea

A motor encoder reports changes in ticks, not an absolute lift height after startup. The team must
author the relationship between hardware and mechanism units, then establish one trustworthy
coordinate reference before ordinary position requests can be realized.

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
bottomSwitch = FtcSensors.digitalLow(map, c.bottomSwitchName)
        .debouncedOnOff(0.02, 0.02);
```

`digitalLow(...)` performs the electrical LOW-to-pressed interpretation. `debouncedOnOff(...)`
requires the interpreted value to remain stable in clock time before publishing either edge; it
does not inspect where the switch is mounted.

### Search cooperatively, then publish policy only on success

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
`BasicLiftControls` maps X to a fresh home Task:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java -->
```java
// A Supplier creates a fresh single-use homing Task on every X-button rise.
requiredTasks.onRise(driver.x(), requiredLift::home);
```

This page uses only switch/reference facts; requesting and waiting for a height is the next lesson.

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
- `BasicLiftControls` — fresh home Task binding.
  [Complete source: `BasicLiftControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java>)
- [`BasicLiftTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.html>) — mechanism-only managed host.
  [Complete source: `BasicLiftTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.java>)

**Test:**

- [Complete source: `BasicLiftSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java>)

## Software checkpoint: authored switch evidence controls homing

- **Question:** Does the authored active-low switch observation establish reference only after its
  debounce interval?
- **Keep real:** the production lift, homing Task, switch interpretation, Plant, and loop order.
- **Replace:** only the motor and digital channel with recording software devices.
- **Observe:** the homing Task outcome and the mechanism's reference status.
- **Cannot conclude:** switch placement, wiring, motor polarity, conversion accuracy, limits, or
  safe motion.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
// INJECT EVIDENCE: LOW must remain observed long enough to pass the configured debouncer.
scenario.bottomSwitch.setHigh(false);
scenario.advance(0.01);
assertFalse(scenario.task.isComplete());
scenario.advance(0.01);
assertEquals(TaskOutcome.SUCCESS, scenario.task.getOutcome());
assertTrue(scenario.lift.status().referenced());
assertEquals(BasicLift.Height.STOWED, scenario.lift.status().requestedHeight());
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftSoftwareScenarioTest
```

**Read the causal chain:** the test explicitly supplies switch evidence across real clock cycles;
the first observation is shorter than the debounce interval; the second observation lets the
production homing Task finish and publish referenced status.

**Proves:** software polarity interpretation, debounce timing, reference transition, and
success-only post-home policy for the authored observations.

**Does not prove:** the real switch changes safely, the lift travels toward it, or the encoder scale
and power limit are correct.

## Isolated hardware gate

Keep `BasicLiftTeleOp` disabled and `allowLiftMotion` false while reviewing configuration. Write the
switch-polarity, motor-direction, travel-envelope, and emergency-stop check plan first. Mechanically
support the lift and start away from hard stops. Use
[Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) only if a mechanism-specific
support plan makes its generic jog appropriate; otherwise build a lift-specific fixture using the
same low-power, dead-man evidence discipline. Establish direction and a backed-off travel span
before setting `allowLiftMotion = true`, removing `@Disabled`, and starting the lift OpMode. Do not
press a D-pad direction yet. For the first polarity check, mechanically disengage the motor from
the lift or use an equivalent fixture that prevents linkage motion while allowing the motor to run
briefly. Restart the OpMode with the switch released and confirm `lift.referenced = false`. Press X:
reference must remain false until you manually hold the switch pressed through its debounce, then
become true. If it becomes true before the manual press, stop and correct the polarity or wiring.
After the isolated polarity proof, press STOP, de-energize the robot, and only then reconnect the
mechanism. The next OpMode run correctly starts unreferenced. With the lift supported and its path
clear, press X for real low-power homing while an immediate STOP operator watches the motion.
Record repeatable activation and reference status; do not try a named-height move yet, and never
touch or reconnect the linkage while the OpMode or controller is active.

**Next gate:** after repeatable low-power homing is established, continue to
[moving the referenced lift](<Move a Referenced Lift.md>). That page adds semantic height mapping,
fresh position feedback, and the explicit success/timeout/cancellation contract.
