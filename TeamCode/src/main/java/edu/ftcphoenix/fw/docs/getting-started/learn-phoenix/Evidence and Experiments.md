# Evidence and experiments

**Question:** What does a boolean, status row, or experiment result actually prove?

**Reading time:** about 20 minutes

Phoenix keeps raw observation, robot meaning, and physical conclusions separate. This prevents a
convenient `true` from claiming more than the source can know.

This chapter does not require a sensor, mechanism, or robot run. It teaches how to read the checked-
in example and how to design evidence before a team performs supervised hardware validation.

## Source map

- [`ReferenceLiftMechanism.java`](<../../../../robots/examples/reference/capability/lift/ReferenceLiftMechanism.java>)
  converts an active-low electrical input into a debounced semantic source.
- [`ReferenceLauncherMechanism.java`](<../../../../robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.java>)
  publishes controller and object-sensor evidence in one cached status.
- [`ReferenceRobot.java`](<../../../../robots/examples/reference/robot/ReferenceRobot.java>) presents
  those snapshots without making behavior decisions.
- [`ReferenceFlywheelSpinUpCriteria.java`](<../../../../robots/examples/reference/tester/ReferenceFlywheelSpinUpCriteria.java>)
  keeps the worked wheel experiment locked until the team reviews its physical card.
- [`ReferenceRobotTesters.java`](<../../../../robots/examples/reference/tester/ReferenceRobotTesters.java>)
  registers a fresh experiment without exposing tester lifecycle in the robot OpMode.
- [`Subsystem Experiments`](<../../examples/Subsystem Experiments.md>) explains how a team authors
  and validates a real experiment card.

## Semantic input is not electrical level

A gamepad button source already has a human meaning: `gamepad.y()` is `true` while Y is pressed. A
gamepad trigger is not a digital boolean; it is a `ScalarSource` whose logical range runs from
`0.0` released to `1.0` fully pressed.

A `DigitalChannel` reports an electrical level instead. Phoenix makes the selected polarity
explicit:

- `FtcSensors.digitalHigh(...)` is `true` while the pin is electrically HIGH.
- `FtcSensors.digitalLow(...)` is `true` while the pin is electrically LOW.

The Reference lift selects active-low meaning at construction:

```java
public ReferenceLiftMechanism(HardwareMap hardwareMap, Config config) {
    HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
    Config c = copyAndValidate(config);
    bottomSwitch = FtcSensors.digitalLow(map, c.bottomSwitchName)
            .debouncedOnOff(0.02, 0.02);
```

For the wiring assumed by this example, the intended truth table is:

| Physical state | Pin level | `digitalLow` value | `bottomSwitch` after debounce |
|---|---|---:|---:|
| Released | HIGH | `false` | `false` |
| Depressed | LOW | `true` | `true` |

`digitalLow` does not discover the mechanical state or wiring. It only converts “the pin is LOW”
into `true`. The robot author chooses that adapter because the reviewed circuit makes LOW mean
“bottom limit depressed.” Other wiring may require `digitalHigh` or a different semantic name.

Debounce is a separate decision. `.debouncedOnOff(0.02, 0.02)` requires the selected boolean value
to remain stable for 20 ms before turning on or off; it does not invert the value or decide what the
switch means.

```text
electrical pin level
    -> explicit digitalLow polarity
    -> 20 ms on/off debounce
    -> semantic bottomSwitch source
    -> homing Task condition
```

## Status is cached evidence

Mechanism outputs update their Plants and sensors, then replace one immutable status snapshot. The
launcher publishes these facts:

```java
lastStatus = new Status(
        targetVelocityTicksPerSec,
        leftVelocityTicksPerSec,
        rightVelocityTicksPerSec,
        leftAtTarget,
        rightAtTarget,
        objectPresent.getAsBoolean(clock),
        transferOverrides.activeSource().getAsBoolean(clock));
```

This separates what velocity was requested, what each wheel measured, whether each wheel is within
tolerance, whether the conditioned object input is active, and whether a temporary transfer pulse
is active. `Status` derives aggregate `ready` only when the target is positive and both per-wheel
facts are true, so an idle zero target cannot masquerade as launch readiness. Every velocity name
states encoder ticks per second. A presenter may format the snapshot, but it does not resample
hardware or decide robot behavior.

The public snapshot names those facts `targetVelocityTicksPerSec`,
`leftMeasuredVelocityTicksPerSec`, `rightMeasuredVelocityTicksPerSec`, `leftAtTarget`,
`rightAtTarget`, `ready`, `objectPresent`, and `transferPulseActive`.

The object sensor is **status-only** in this Reference mechanism. `launchOne()` never reads
`objectPresent`; its feed decision depends only on flywheel readiness and timeout. Seeing
`object=true` in telemetry therefore does not gate or authorize a launch. A team that needs an
object-present gate must add that policy deliberately in its own capability or supervisor.

Likewise, `ready=true` means only that the positive current target and both finite per-wheel
measurements meet the configured ticks-per-second tolerance. Idle at zero is not ready. Readiness
is not evidence that an object left the mechanism, followed the desired trajectory, or scored.

## Experiments print only computed evidence

The Reference flywheel trial changes only the flywheel-velocity request; it never requests a
release or transfer pulse and does not launch an object. It still updates the complete production
mechanism, whose normal active idle realizes zero transfer and the configured retracted release
position. The servo can therefore move to that reviewed retracted position, and the lab card must
include its clearance. The experiment prints values the operator cannot reliably derive by
watching:

```java
ctx.telemetry.addData("trialNumber", trialNumber);
ctx.telemetry.addData("trialState", trialState);
ctx.telemetry.addData("targetVelocityTicksPerSec", targetVelocityTicksPerSec);
ctx.telemetry.addData("leftMeasuredVelocityTicksPerSec", leftMeasuredVelocityTicksPerSec);
ctx.telemetry.addData("rightMeasuredVelocityTicksPerSec", rightMeasuredVelocityTicksPerSec);
ctx.telemetry.addData("elapsedSec", elapsedSec);
```

The experiment retains `IDLE`, `RUNNING`, `TARGET_REACHED`, `TIME_LIMIT_REACHED`, and `ABORTED`.
When a trial ends, it freezes elapsed time and both wheel measurements before requesting zero, so
later deceleration cannot relabel the result. The monotonic trial number lets an operator match
telemetry to an external row. The operator records direction, vibration, sound, damage, clearance,
and STOP response outside the Robot Controller.

The checked-in experiment is locked:

```java
ReferenceFlywheelSpinUpCriteria criteria =
        ReferenceFlywheelSpinUpCriteria.current();
```

`current()` keeps `reviewedForMotion` false in the checked-in source. Its target and maximum powered
run are software-valid placeholders, not physical success criteria. The maximum powered run is a
cooperative safety boundary and terminal condition, checked once per active loop rather than by a
hard real-time interrupt. The tester applies zero on the first loop observed at or after the
boundary and retains that loop's elapsed time, so the lab card and STOP plan must allow for the
worst-case loop delay. It is not automatically an acceptable spin-up-time threshold. Likewise,
`TARGET_REACHED` is a computed trial outcome, not an overall `PASS`. The team must define the
question, safe range, success threshold, procedure, stop conditions, and external observations
before unlocking motion. Reading this course never requires doing so.

## Copy, adapt, and leave alone

- **Copy the separation:** electrical adapter, conditioning, semantic name, cached status, and
  presenter are distinct steps.
- **Adapt the evidence:** publish only the requested, applied, measured, readiness, or timing values
  the real subsystem can truthfully compute.
- **Keep physical conclusions external when appropriate:** a human can record make/miss, jams,
  damage, sound, and other visible results.
- **Do not copy the Reference criteria as permission:** physical validation belongs to the adopting
  team's supervised tester and calibration process.

## Trace it

1. **When is the Reference lift's `bottomSwitch` true?**

   After its selected input has remained electrically LOW for the configured on-debounce time. The
   example assumes reviewed wiring in which that means the switch is depressed.

2. **Does `objectPresent=false` prevent `launchOne()` from feeding?**

   No. That source is included in status only.

3. **Who turns a status into telemetry text?**

   The presenter in `ReferenceRobot`; it observes the cached status and owns no policy.

## Predict it

The experiment reports `TARGET_REACHED`, but the operator observes unacceptable vibration. What can
the experiment claim?

It may report its frozen per-wheel measurements and computed spin-up result. The operator records
the vibration separately, and the team may reject the configuration. Neither result should be
silently converted into the other.

**Previous:** [Tasks and autonomous](<Tasks and Autonomous.md>)

**Next:** [From requirement to robot](<From Requirement to Robot.md>)
