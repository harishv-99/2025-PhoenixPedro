# FTC Actuators & Plants

This page covers the FTC boundary for Phoenix mechanism wiring:

* `FtcActuators.plant(...)` — the staged beginner builder
* `FtcHardware` — low-level command channels
* `FtcSensors` — low-level measurement sources
* device-managed vs regulated control
* position plant geometry, plant/native unit mapping, and reference policy
* how tolerances and FTC motor tuning interact

The FTC boundary is where FTC SDK hardware names become Phoenix `Plant` objects. Robot services,
Tasks, and scalar planners should use the built Plants instead of touching raw SDK devices directly.

---

## 1. The staged entrypoint: `FtcActuators.plant(...)`

Use `edu.ftcphoenix.fw.ftc.FtcActuators` when you want to create a `Plant` from FTC hardware.

```java
Plant flywheel = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetedByDefaultWritable(0.0)
        .build();

PositionPlant pusher = FtcActuators.plant(hardwareMap)
        .servo("pusher", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .nativeUnits()
        .targetedByDefaultWritable(0.0)
        .build();
```

The builder is staged on purpose. Each required conceptual question is answered explicitly, while
optional tuning appears only after the user enters a tuning branch. This keeps autocomplete focused
on the next meaningful choice.

For example, motor position wiring asks:

1. Which hardware? `motor(...)`
2. Which target domain? `position()`
3. Who manages the position loop? `deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated()` followed by one direct feedback answer and `regulator(...)`
4. What topology? `linear()` or `periodic(period)`
5. What bounds? `bounded(min, max)` or `unbounded()`
6. How do plant units map to native units? `nativeUnits()`, `scaleToNative(...)`, or bounded-only `rangeMapsToNative(...)`
7. How is the reference/offset known? `alreadyReferenced()`, `plantPositionMapsToNative(...)`, `assumeCurrentPositionIs(...)`, or `needsReference(...)`
8. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
9. Target binding: `targetedBy(ScalarTarget)`, `targetedBy(readOnlySource)`, or `targetedByDefaultWritable(initialTarget)`, then `build()`

Motor velocity wiring asks a parallel but smaller set of questions:

1. Which hardware? `motor(...)`
2. Which target domain? `velocity()`
3. Who manages the velocity loop? `deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated()` followed by one direct feedback answer and `regulator(...)`
4. What target bounds are legal? `bounded(min, max)` or `unbounded()`
5. How do plant velocity units map to native velocity units? `nativeUnits()` or `scaleToNative(...)`
6. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
7. Target binding: `targetedBy(ScalarTarget)`, `targetedBy(readOnlySource)`, or `targetedByDefaultWritable(initialTarget)`, then `build()`

Velocity and power Plants stay simpler than position Plants because they do not have position
geometry, periodicity, or homing/reference questions. Power is simpler still: every direct power
Plant has the fixed normalized range `[-1.0, +1.0]`, so there is no redundant bounds step in the
builder. A finite request outside that range clamps before the output. The Plant reports
`CLAMPED_TO_RANGE` when that clamp remains the active final transform; a later rate limit,
interlock, or fallback may report its more specific status instead.

### FTC motor run-mode ownership

The core `PowerOutput` interface is hardware-neutral; it does not promise that every implementation
owns an FTC motor mode. The concrete `FtcHardware.motorPower(...)` adapter does have a narrower
contract: it means raw/open-loop motor power.

Construction resolves the motor and sets its direction but does not acquire a run mode. Every
explicit `setPower(...)` command, including `setPower(0.0)`, conditionally establishes
`RUN_WITHOUT_ENCODER`. If another mode is selected, the adapter writes zero in that mode, selects
and verifies `RUN_WITHOUT_ENCODER`, and only then writes the requested power. Lifecycle `stop()` is
deliberately different: it writes zero without acquiring or restoring a mode. The adapter never
uses `STOP_AND_RESET_ENCODER`.

Device-managed `FtcHardware.motorPosition(...)` and `motorVelocity(...)` outputs own their required
FTC modes when commanded. `FtcActuators` selects these output semantics from the target domain and
control strategy already chosen in the staged builder, so student robot code should not surround
standard Plants with manual `setMode(...)` calls. Each command path asserting its own mode supports
an orderly handoff; it does not make simultaneous writers safe.

---

## 2. Behavior sources vs Plant target guards

Every robot-facing Plant is source-driven:

```text
behavior PlantTargetSource
    ↓
requested target + PlantTargetPlan
    ↓
Plant static range / reference policy
    ↓
Plant targetGuards() hardware protection
    ↓
final finite / declared-range defense
    ↓
applied target + PlantTargetStatus
    ↓
hardware/control
```

`getTargetPlan()` explains how behavior selected the requested target. `getTargetStatus()` explains
how the Plant protected and applied that request.

Use behavior sources for robot policy:

```java
PlantTargetSource finalFeeder = PlantTargets.overlay(baseFeeder)
        .add("feedPulse", feedQueue.activeSource(), feedQueue)
        .add("eject", ejectRequested, -1.0)
        .build();
```

Use `targetGuards()` only for Plant-level protection that should apply no matter which behavior
requested the target:

```java
PositionPlant lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetGuards()
            .maxTargetRate(1200.0)
            .holdLastTargetUnless("wristClear", wristClear)
            .doneTargetGuards()
        .targetedByDefaultWritable(0.0)
        .build();
```

`bounded(...)` is also a hardware limit, but it is kept outside `targetGuards()` because it defines
the Plant's legal coordinate system. A direct power Plant declares its normalized `[-1.0, +1.0]`
range internally. Dynamic guards such as rate limits, hold-last interlocks, and fallback targets
live in `targetGuards()`. When a Plant has a fixed range, each static fallback must lie inside that
range or `build()` reports which guard and value to fix. After dynamic guards run, the Plant verifies
the result is still finite and inside the range before updating `getAppliedTarget()` or commanding
hardware. If that final defense changes a result, status explains the correction and any rate
limiter is reconciled to the command that was actually applied.

A max-rate guard uses actual elapsed loop time; it does not predict the future loop period. The first
sample initializes directly to the first guarded candidate, and `stop()`/`reset()` clear dynamic guard
state. If loop time is temporarily non-finite, it holds the last output until a finite time baseline
has been restored. If a mechanism needs startup limiting from a known physical position, initialize
the writable target from that measurement or use an appropriate reference policy before requesting
a far-away target.

---

## 3. Plant units vs native units

For position and velocity Plants, Phoenix distinguishes two coordinate systems:

* **Plant units** are the public units used by robot code, `ScalarTarget` requests, Plant target
  sources, scalar planner requests, target ranges, position periods, reference values, and plant-level tolerances.
* **Native units** are the units used by the selected hardware/control path: servo raw fraction,
  motor encoder ticks, motor ticks/sec, external encoder units, or a caller-supplied feedback source.

Public position and velocity APIs use **plant units** unless the method name explicitly says
**Native**.

As a rule of thumb:

* `bounded(...)`, `unbounded()`, `periodic(...)`, `targetedBy(...)`, `getRequestedTarget()`,
  `getAppliedTarget()`, `getMeasurement()`, `positionTolerance(...)`, and `velocityTolerance(...)` all speak in
  **plant units**.
* Methods that cross the plant/native boundary say so explicitly in the name or docs, for example
  `nativeUnits()`, `scaleToNative(...)`, `rangeMapsToNative(...)`,
  `plantPositionMapsToNative(...)`, and `devicePositionToleranceTicks(...)`.

Examples:

```java
.linear().bounded(0.0, 18.0)          // inches if the mechanism is declared in inches
.periodic(360.0)                      // degrees if the mechanism is declared in degrees
.positionTolerance(0.10)              // plant units
.assumeCurrentPositionIs(0.0)         // plant units
.establishReferenceAt(0.0)            // plant units, through PositionCalibrationTasks

.scaleToNative(TICKS_PER_INCH)        // native ticks per plant inch
.rangeMapsToNative(0.30, 0.80)        // native servo fractions at plant-range endpoints
.plantPositionMapsToNative(0.0, ARM_ZERO_TICKS) // plant position 0 maps to native encoder tick offset
.devicePositionToleranceTicks(12)     // explicitly native/controller ticks
```

This convention keeps common robot code readable while making boundary-crossing methods obvious.

---

## 4. Motor position control: device-managed or regulated

After `motor(...).position()`, Phoenix asks who manages the position loop.

### Device-managed with defaults

Use this when FTC `RUN_TO_POSITION` is good enough and you do not need controller-specific tuning:

```java
PositionPlant lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetedByDefaultWritable(0.0)
        .build();
```

The public lift coordinate is in plant units. In this example, plant units are also native encoder
ticks because the builder chose `nativeUnits()`.

### Device-managed with FTC tuning

Enter `deviceManaged()` only when you want optional FTC motor-controller tuning. While in this
branch, autocomplete shows only device-managed tuning knobs. `doneDeviceManaged()` returns to the
main position questions.

```java
PositionPlant lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .deviceManaged()
            .maxPower(0.8)
            .outerPositionP(5.0)
            .devicePositionToleranceTicks(12)
            .doneDeviceManaged()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetedByDefaultWritable(0.0)
        .build();
```

Device-managed tuning options:

* `maxPower(...)` — power Phoenix reapplies after each FTC `RUN_TO_POSITION` target.
* `outerPositionP(...)` — FTC outer position-loop proportional gain.
* `innerVelocityPidf(...)` — FTC inner velocity-loop PIDF used under position mode.
* `devicePositionToleranceTicks(...)` — FTC motor-controller target tolerance in native ticks.

Notice that plant-level `positionTolerance(...)` lives after the coordinate mapping stage because it
is in plant units. Device-level methods that use native/controller units say so in their names.

### Framework-regulated motor position

Use `regulated()` when Phoenix should drive raw motor power from an explicit feedback source and a
regulator:

```java
PositionPlant arm = FtcActuators.plant(hardwareMap)
        .motor("armMotor", Direction.FORWARD)
        .position()
        .regulated()
            .externalEncoder("armEncoder")
            .regulator(ScalarRegulators.pid(Pid.withGains(0.006, 0.0, 0.0002)))
        .linear()
            .bounded(-300.0, 1200.0)
            .nativeUnits()
            .alreadyReferenced()
        .positionTolerance(20.0)
        .targetedByDefaultWritable(0.0)
        .build();
```

The regulator sees plant-unit error. If the plant uses `nativeUnits()`, plant units and native units
are the same. If the plant uses `scaleToNative(...)`, the feedback is converted into plant units
before the regulator runs.

### Controller limits, complete-regulator limits, and output safety

These three protections answer different questions:

| Layer | Question it answers |
|---|---|
| `Pid.setOutputLimits(...)` | How large may the PID controller's own contribution be? |
| `ScalarRegulators.outputLimited(...)` | How large may this complete inner regulator composition be? |
| Plant/output command safety | What finite command may the actuator channel ultimately accept? |

PID output limits run before feedforward or another outer decorator. For example, voltage
compensation can legitimately scale a previously limited PIDF command above the PID controller's
limit. When the robot intentionally wants a narrower command range around everything, make the
limiter the outermost output-changing decorator. The complete regulated-velocity example in
[Velocity bounds, mapping, and tuning](#13-velocity-bounds-mapping-and-tuning) shows that composition
around PIDF and battery-voltage compensation.

`outputLimited(...)` constrains exactly the regulator passed to it. Another output-changing
decorator placed outside it is not covered. Its bounds are generic regulator-command units; they
are not Plant target units and are not a replacement for `bounded(...)` or `targetGuards()`.
Likewise, optional narrower policy does not replace the Plant/output path's universal responsibility
to keep normalized actuator commands finite and inside their semantic range.

### Regulated command truth and fail-stop behavior

For a framework-regulated position or velocity Plant, the regulator result passes through one final
normalized-power boundary immediately before the configured `PowerOutput`:

| Regulator/output event | Plant behavior |
|---|---|
| Finite result inside `[-1.0, +1.0]` | Submit it unchanged, including exact boundaries and signed zero. |
| Finite result outside `[-1.0, +1.0]` | Saturate it to the nearest boundary and submit the normalized value. |
| `NaN` or either infinity | Do not submit it; best-effort stop the output, reset the regulator, and throw an actionable failure. |
| Regulator or output write throws | Best-effort stop and reset, then rethrow the original failure with cleanup failures suppressed. |

Finite saturation at this universal boundary is normal actuator-domain behavior. It neither resets
the regulator nor supplies generic anti-windup. Use an outermost `outputLimited(...)` when the robot
intentionally needs a narrower or asymmetric policy such as `[0.0, maximumFlywheelPower]`.

`reset()` clears regulator/completion state but does not send a hardware command. The last normally
submitted normalized command therefore remains the truthful command fact until another output
operation returns normally. `stop()` attempts to submit zero before resetting the regulator and
attempts both operations even when one fails. A normally returning top-level stop supports a
seam-level "zero submitted" fact even if regulator reset then fails; a throwing output stop leaves
the command unknown. In either case, `atTarget()` and `atTarget(value)` stay false until a complete
later regulated actuation returns normally.

The debug fields deliberately keep different kinds of truth separate:

* `.regulatorOutput` is the raw regulator result. Existing `.output` on lower-level regulated
  Plants and `.lastRegulatorOutput` on mapped position Plants remain raw aliases.
* `.normalizedPowerCommand` is the last known value submitted by a normally returning top-level
  `PowerOutput` operation performed by the regulated-command boundary. It becomes unknown when a
  throwing output operation prevents truthful command bookkeeping. An open-loop position-
  calibration search bypasses this boundary and may command the same configured output later; use
  its separate search state when interpreting diagnostics. The normalized command is not a motor
  measurement or hardware acknowledgement.
* `.regulatedPowerStatus` explains whether the last operation submitted, saturated and submitted,
  reset without writing, stopped, or failed, including fail-stop and reset outcomes.
* `.regulator` remains the nested regulator-specific diagnostic prefix.

For a regulated Plant, `getAppliedTarget()` remains the final mechanism target in plant units; it is
not either of the power-command fields above. A custom or grouped `PowerOutput` may transform one
top-level command into several child commands, so these diagnostics do not claim per-child or
physical actuator truth. Standard FTC adapter saturation remains defense in depth. Open-loop
position-calibration search power is a separate configuration path and is not a regulator result.

The decorator reports its unconstrained and applied results through standard regulator debug data,
delegates `reset()` to its inner regulator, and rejects a non-finite inner result instead of hiding
broken control math behind a bound. It does not provide generic saturation-aware anti-windup;
controller-specific integral limits remain explicit. Robot behavior or realization still decides
whether disabled means coast or hold and when an enable/disable transition should reset controller
history.

---

## 5. Position topology and bounds

Every declared position Plant answers two separate questions:

```java
.linear()              // non-wrapping coordinate
.periodic(period)      // equivalent positions separated by period, in plant units

.bounded(min, max)     // legal target range in plant units
.unbounded()           // no software target range
```

Common choices:

```java
// Lift or slide.
.linear()
    .bounded(0.0, 4200.0)

// Raw tuning motor where software travel limits are intentionally not declared yet.
.linear()
    .unbounded()

// Cable-limited turret: facing repeats every rotation, but legal travel is finite.
.periodic(TICKS_PER_TURN)
    .bounded(-900.0, 1100.0)

// Tray/indexer that can rotate continuously.
.periodic(TICKS_PER_REV)
    .unbounded()
```

`PositionPlant.period()` is in plant units. A target request such as
`PlantTargetRequest.equivalentPosition("slot-2", 240.0)` uses the consuming plant's declared period when
it is resolved by `PlantTargets.plan()` during `plant.update(clock)`.

---

## 6. Mapping plant units to native units

After topology and bounds, the builder asks how the public plant coordinate maps to the selected
native coordinate.

### `nativeUnits()`

Use this when plant units are native units:

```java
.linear()
    .bounded(0.0, 4200.0)
    .nativeUnits()
    .needsReference("lift not homed")
```

Here the lift target is in encoder ticks.

### `scaleToNative(...)`

Use this when the scale is known but the reference/offset is a separate question:

```java
.linear()
    .bounded(0.0, 18.0)
    .scaleToNative(TICKS_PER_INCH)
    .needsReference("lift not homed")
```

Here the lift target is in inches. The motor still receives native encoder ticks after homing
establishes the offset.

### `rangeMapsToNative(...)`

Use this only after `bounded(...)`. It maps the declared plant-range endpoints to native endpoints
and therefore establishes both scale and offset statically.

The two arguments are **native values**, not plant values:

* `nativeAtPlantMin` = the native coordinate at the plant minimum from `bounded(min, max)`
* `nativeAtPlantMax` = the native coordinate at the plant maximum from `bounded(min, max)`

With declared plant bounds `plantMin` and `plantMax`, Phoenix builds the affine map:

```text
native = nativeAtPlantMin
       + ((plant - plantMin) / (plantMax - plantMin))
         * (nativeAtPlantMax - nativeAtPlantMin)
```

So for a servo declared as `bounded(-45.0, 90.0)`, calling `rangeMapsToNative(0.22, 0.76)` means
“plant `-45.0` maps to raw servo `0.22`, and plant `90.0` maps to raw servo `0.76`.”

Example:

```java
ScalarTarget clawTarget = ScalarTarget.held(0.0);

PositionPlant claw = FtcActuators.plant(hardwareMap)
        .servo("clawServo", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .rangeMapsToNative(0.30, 0.80)
        .targetedBy(clawTarget)
        .build();
```

Robot code now writes the logical claw target source:

```java
clawTarget.set(0.0); // raw servo 0.30
clawTarget.set(1.0); // raw servo 0.80
claw.update(clock);
```

`rangeMapsToNative(...)` is intentionally not available after `unbounded()` because there is no
finite plant range to map from.

---

## 7. Reference policy and runtime calibration

After `nativeUnits()` or `scaleToNative(...)`, the builder asks how the native/plant reference is
known.

### `alreadyReferenced()`

Use this when the selected native coordinate is already aligned with the plant coordinate after the
chosen scaling.

Good examples:

* a standard servo using native raw `0..1` units,
* an absolute/source feedback value already expressed in arm degrees,
* a simulator source already expressed in plant units,
* a motor encoder that robot code intentionally reset before building/using the Plant.

```java
.linear()
    .bounded(-35.0, 125.0)
    .nativeUnits()
    .alreadyReferenced()
```

### `plantPositionMapsToNative(plantPosition, nativePosition)`

Use this when both scale and one offset point are known in code:

```java
.linear()
    .bounded(-35.0, 125.0)
    .scaleToNative(TICKS_PER_DEGREE)
    .plantPositionMapsToNative(0.0, ARM_ZERO_TICKS)
```

This means plant position `0.0` maps to native encoder tick `ARM_ZERO_TICKS`.

### `assumeCurrentPositionIs(...)`

Use this when the robot is physically placed at a known pose before init:

```java
.linear()
    .bounded(0.0, 4200.0)
    .nativeUnits()
    .assumeCurrentPositionIs(0.0)
```

On the first update, Phoenix samples the native reading and treats that reading as plant position
`0.0`. This is convenient, but it is only correct if the mechanism really starts there.

### `needsReference(...)`

Use this when the mechanism must be homed/indexed before position targets are meaningful:

```java
.linear()
    .bounded(0.0, 4200.0)
    .nativeUnits()
    .needsReference("lift not homed")
```

Before the reference is established, `PositionPlant.targetRange(...)` returns an invalid range with
that reason. `PlantTargets.plan()` sources see that invalid range during `plant.update(clock)` and use
their explicit `whenUnavailable()` policy instead of producing unsafe requested targets.

Use `PositionCalibrationTasks` to establish the reference:

```java
Task homeLift = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .holdAfterReference(0.0)
        .failAfterSec(3.0)
        .build();
```

The search power temporarily asserts raw/open-loop mode through the motor-power adapter. Ending the
search writes zero without restoring the previous mode; the next device-managed position command
reasserts `RUN_TO_POSITION`. Calibration establishes the Plant reference and never resets the
encoder as a hidden side effect.

The timeout policy is explicit: use `failAfterSec(...)` for a bounded search or `neverTimeout()` only when another safety path is guaranteed to cancel the task.

For periodic mechanisms, `establishReferenceAt(...)` preserves the nearest equivalent unwrapped
position. If a tray has period `360.0` degrees and the current estimate is `1081.5`, an index mark
for reference `0.0` corrects the estimate to `1080.0`, not all the way back to zero.

---

## 8. Standard servo position Plants

Standard servos are command-only position outputs. They do not expose motor control strategy,
open-loop calibration search, or unbounded travel.

Raw servo units:

```java
PositionPlant wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .nativeUnits()
        .targetedByDefaultWritable(0.0)
        .build();
```

Logical units mapped to raw endpoints:

```java
PositionPlant wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .linear()
            .bounded(-45.0, 90.0)
            .rangeMapsToNative(0.22, 0.76)
        .targetedByDefaultWritable(0.0)
        .build();
```

Robot code can now command degrees, while the servo receives raw fractions.

---

## 9. Regulated CR-servo position Plants

CR servos do not have a device-managed position mode. Position control requires native feedback and
a regulator:

```java
PositionPlant turret = FtcActuators.plant(hardwareMap)
        .crServo("turretServo", Direction.FORWARD)
        .position()
        .regulated()
            .externalEncoder("turretEncoder")
            .regulator(ScalarRegulators.pid(Pid.withGains(0.01, 0.0, 0.0005)))
        .periodic(TURRET_TICKS_PER_TURN)
            .bounded(-900.0, 1100.0)
            .nativeUnits()
            .needsReference("turret not homed")
        .positionTolerance(8.0)
        .targetedByDefaultWritable(0.0)
        .build();
```

A CR-servo position Plant can participate in `PositionCalibrationTasks.search(...)` because it can
be driven with temporary open-loop power while looking for a reference.

---

## 10. Feedback answers on regulated stages

After `.regulated()`, answer the feedback question directly on that already-domain-specific builder
stage. There is no separate feedback-selector object to construct and immediately pass back.

The feedback answer selects only a measurement source; it does not select the powered motor's run
mode. A regulated motor remains on the raw/open-loop command path whether feedback comes from its
internal encoder, an external encoder on the same configured channel, or a separate encoder-only
channel. A separate external channel is read-only: `.externalEncoder(...)` does not set its power,
change its mode, or reset its encoder. If another owner deliberately leaves that channel in
`STOP_AND_RESET_ENCODER`, that owner must explicitly leave reset mode before expecting useful
measurements.

### Motor position

* `.internalEncoder()`
* `.internalEncoder("leftLift")`
* `.averageInternalEncoders()`
* `.externalEncoder("liftEncoder")`
* `.externalEncoder("liftEncoder", direction)`
* `.nativeFeedback(customNativePositionSource)`

The encoder answers report native position in FTC ticks. `nativeFeedback(...)` is the advanced seam
for an analog, vendor, simulated, fused, or already-composed source that directly reports the native
position required by this stage. If that source already uses the public coordinate you want, choose
`nativeUnits().alreadyReferenced()` after the geometry step.

### Motor velocity

* `.internalEncoder()`
* `.internalEncoder("flywheel")`
* `.averageInternalEncoders()`
* `.externalEncoder("flywheelEncoder")`
* `.externalEncoder("flywheelEncoder", direction)`
* `.nativeFeedback(customNativeVelocitySource)`

Internal motor-encoder answers use the FTC SDK's direct velocity reading in ticks/second. An external
incremental encoder answer instead reads the SDK-observed signed 32-bit position continuously and
derives interval-average ticks/second from position change over actual elapsed accepted sample time.
The builder hides that acquisition difference; robot code still chooses only which physical feedback
source it wired.

The first valid external-position sample establishes the baseline and reports bootstrap velocity
`0.0`. Plant/source reset clears that history, so reset feedback while the mechanism is stopped or
otherwise account for the new baseline in robot policy. Repeated samples in one
`LoopClock.cycle()` reuse one result, skipped cycles use their complete elapsed interval, and the
estimator does not hide smoothing, counts-per-revolution conversion, or outlier policy.

Position derivation avoids the FTC direct-velocity field's narrower numeric representation, but it
can only describe positions that the SDK reports. It cannot prove that a hub captured every physical
encoder edge. For a high-count-rate REV Through Bore encoder, use a hardware-counted Control/
Expansion Hub encoder port 0 or 3 and validate the exact wiring, firmware, loop conditions, maximum
speed, and regulator tuning on the robot before claiming match readiness. Optional filtering remains
separate source composition rather than a hidden encoder behavior.

### CR-servo position

CR servos have no internal encoder choice. Their regulated position stage exposes only:

* `.externalEncoder("turretEncoder"[, direction])`
* `.nativeFeedback(customNativePositionSource)`

---

## 11. Measurement readback

Phoenix separates **commands** from **measurements**.

### Command channels

`FtcHardware` creates command-only HAL outputs:

* `PowerOutput`
* `PositionOutput`
* `VelocityOutput`

These answer “what command should we send?” — not “what did the mechanism measure?” The interfaces
remain hardware-neutral; FTC run-mode ownership belongs to the concrete motor adapters described
above.

### Measurement sources

Use `FtcSensors` for raw measurement sources:

```java
ScalarSource ticks = FtcSensors.motorPositionTicks(hardwareMap, "armMotor");
ScalarSource vel = FtcSensors.motorVelocityTicksPerSec(hardwareMap, "flywheel");
```

### Plant status

Feedback-capable plants expose their authoritative measurement through the plant itself:

```java
plant.update(clock);

telemetry.addData("target", plant.getRequestedTarget());
telemetry.addData("measurement", plant.getMeasurement());
telemetry.addData("error", plant.getTargetError());
telemetry.addData("atTarget", plant.atTarget());
```

For `PositionPlant`, `getRequestedTarget()`, `getAppliedTarget()`, `getMeasurement()`, and `getTargetError()` are all in plant units.
`PositionPlant.positionSource()` is also in plant units. Smart target sources such as
`PlantTargets.plan()` receive the Plant measurement and range automatically through the Plant target
context during `update(clock)`.

---

## 12. Position tolerances and FTC motor tuning

For device-managed motor position control there are two different tolerance concepts.

### `positionTolerance(...)`

This is the **plant-level** completion band used by `Plant.atTarget()`.

* units: plant position units
* use this first when you want to define “close enough for robot logic”
* examples: ticks for `nativeUnits()`, inches for `scaleToNative(TICKS_PER_INCH)`, degrees for a degree-based arm

### `devicePositionToleranceTicks(...)`

This is an **optional FTC motor-controller override** for the motor's own target-position tolerance.

* default in Phoenix: unchanged unless you call it
* units: native encoder ticks
* use this only when you intentionally want to change the FTC motor controller's own completion threshold via `DcMotorEx.setTargetPositionTolerance(int)`
* this does not change what `Plant.atTarget()` means unless your plant-level `positionTolerance(...)` happens to match it

## 13. Velocity bounds, mapping, and tuning

Motor velocity uses the same guided-builder rule as position: required conceptual questions are
explicit, and optional controller tuning appears only after entering a tuning branch.

```java
Plant shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetedByDefaultWritable(0.0)
        .build();
```

If you need FTC motor velocity PIDF coefficients, deliberately enter the device-managed tuning
branch:

```java
Plant shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManaged()
            .velocityPidf(kP, kI, kD, kF)
            .doneDeviceManaged()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetedByDefaultWritable(0.0)
        .build();
```

The velocity tuning branch intentionally exposes only `velocityPidf(...)`. The separate FTC
position-loop gain (`outerPositionP(...)`) belongs to device-managed motor **position** mode, not
pure velocity mode.

Use `regulated()` when Phoenix should own the velocity loop and command raw motor power. This is the
right path for custom power-based flywheel control, including optional battery-voltage compensation:

```java
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.control.Pid;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulators;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcSensors;

static final double TICKS_PER_FLYWHEEL_REV = 28.0;
static final double TICKS_PER_RPM = TICKS_PER_FLYWHEEL_REV / 60.0;

ScalarSource batteryVoltage = FtcSensors.batteryVoltage(hardwareMap);

ScalarRegulator nominalFlywheel = ScalarRegulators.pidf(
        Pid.withGains(kP, kI, kD).setIntegralLimits(-0.15, 0.15),
        rpm -> kV * rpm
);

ScalarRegulator flywheelRegulator = ScalarRegulators.outputLimited(
        ScalarRegulators.voltageCompensated(
                nominalFlywheel,
                batteryVoltage,
                13.0,  // reference voltage used while tuning
                9.0,   // denominator floor for low/noisy readings
                1.4),  // maximum multiplier
        0.0,
        maximumFlywheelPower
);

Plant shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .regulated()
            .internalEncoder()
            .regulator(flywheelRegulator)
        .bounded(0.0, 5000.0)          // plant units: RPM
        .scaleToNative(TICKS_PER_RPM)  // native units: FTC ticks/sec
        .velocityTolerance(75.0)       // plant units: RPM
        .targetedByDefaultWritable(0.0)
        .build();
```

The old-style formula
`(referenceVoltage / measuredVoltage) * controllerOutput` belongs in the regulator decorator, not in
the OpMode loop. The Plant still owns target sampling, target bounds, unit conversion, feedback
measurement, and final hardware output. The regulator receives setpoint and measurement in plant
units, so in the example above both values are RPM even though the native encoder reports ticks/sec.

`ScalarRegulators.voltageCompensated(...)` is intentionally a generic core decorator rather than a
flywheel-only builder method. It can wrap PID, PIDF, or a custom `ScalarRegulator`, and regulated
Plants automatically include its debug fields under `plantPrefix.regulator`. In the example,
`outputLimited(...)` is deliberately outside voltage compensation so the requested
`[0.0, maximumFlywheelPower]` policy covers the fully compensated command. A PID output limit inside
`nominalFlywheel` would cover only the PID contribution before feedforward and compensation.

If robot code wants nicer plant velocity units, keep the controller native units explicit:

```java
Plant shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 5000.0)          // plant units: RPM
        .scaleToNative(TICKS_PER_RPM)  // native units: FTC ticks/sec
        .velocityTolerance(75.0)       // plant units: RPM
        .targetedByDefaultWritable(0.0)
        .build();
```

Velocity mapping is deliberately zero-preserving. Phoenix exposes `nativeUnits()` and
`scaleToNative(...)`, but not `rangeMapsToNative(...)`, because a velocity target of `0.0` should
always mean stop. Semantic mappings like “driver command 0..1 maps to useful shooter speeds” belong
above the Plant in a robot service, table, or request source.

That also means `bounded(min, max)` still uses **plant velocity units**, even when the next step is
`scaleToNative(...)`. The scale changes how Phoenix converts plant velocity to native velocity, but
it does not create a velocity offset or separate reference question. In every supported velocity
mapping, plant `0.0` means stationary.

Velocity bounds should represent hardware-safe target bounds, not scoring semantics. For example,
if a shooter has a minimum useful scoring speed of 700 ticks/sec, the Plant range should usually
still start at `0.0` so setting the flywheel target source to `0.0` stops the flywheel. Keep the 700 value in shooter selection
logic, not in the Plant's legal target range.

---

## 14. Multi-motor groups

For grouped device-managed Plants, Phoenix supports per-child `scale(...)` / `bias(...)` and computes
one aggregate measurement in group units.

For grouped **regulated** Plants, the staged builder intentionally requires default per-child scaling
and bias. If you need a more advanced grouped regulated mechanism, build one deliberate custom
group-output adapter and compose it with `Plants.positionFromPower(...)` or
`Plants.velocityFromPower(...)`. That advanced adapter owns its complete group lifecycle and failure
contract. Independently combining public `FtcHardware.motorPower(...)` outputs does not receive the
standard builder's all-child mode preflight, so do not treat sequential child writes as an equivalent
safe construction path.

That restriction keeps the common builder path simple and makes ambiguous regulated group semantics
fail fast with an actionable error.
