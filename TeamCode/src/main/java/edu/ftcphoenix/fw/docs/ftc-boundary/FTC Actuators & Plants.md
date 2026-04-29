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
3. Who manages the position loop? `deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated().nativeFeedback(...).regulator(...)`
4. What topology? `linear()` or `periodic(period)`
5. What bounds? `bounded(min, max)` or `unbounded()`
6. How do plant units map to native units? `nativeUnits()`, `scaleToNative(...)`, or bounded-only `rangeMapsToNative(...)`
7. How is the reference/offset known? `alreadyReferenced()`, `plantPositionMapsToNative(...)`, `assumeCurrentPositionIs(...)`, or `needsReference(...)`
8. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
9. Target binding: `targetedBy(ScalarTarget)`, `targetedBy(readOnlySource)`, or `targetedByDefaultWritable(initialTarget)`, then `build()`

Motor velocity wiring asks a parallel but smaller set of questions:

1. Which hardware? `motor(...)`
2. Which target domain? `velocity()`
3. Who manages the velocity loop? `deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated().nativeFeedback(...).regulator(...)`
4. What target bounds are legal? `bounded(min, max)` or `unbounded()`
5. How do plant velocity units map to native velocity units? `nativeUnits()` or `scaleToNative(...)`
6. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
7. Target binding: `targetedBy(ScalarTarget)`, `targetedBy(readOnlySource)`, or `targetedByDefaultWritable(initialTarget)`, then `build()`

Velocity and power Plants stay simpler than position Plants because they do not have position
geometry, periodicity, or homing/reference questions.

---

## 2. Plant units vs native units

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

## 3. Motor position control: device-managed or regulated

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
            .nativeFeedback(FtcActuators.PositionFeedback.externalEncoder("armEncoder"))
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

---

## 4. Position topology and bounds

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

`PositionPlant.period()` is in plant units. A scalar setpoint request such as
`ScalarSetpointRequest.equivalentPosition("slot-2", 240.0)` uses the plant's declared period when the
planner is built with `ScalarSetpoints.plan().request(...).forPositionPlant(tray)`.

---

## 5. Mapping plant units to native units

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

## 6. Reference policy and runtime calibration

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
that reason. Scalar setpoint planners built with `forPositionPlant(...)` block rather than producing
unsafe setpoints.

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

The timeout policy is explicit: use `failAfterSec(...)` for a bounded search or `neverTimeout()` only when another safety path is guaranteed to cancel the task.

For periodic mechanisms, `establishReferenceAt(...)` preserves the nearest equivalent unwrapped
position. If a tray has period `360.0` degrees and the current estimate is `1081.5`, an index mark
for reference `0.0` corrects the estimate to `1080.0`, not all the way back to zero.

---

## 7. Standard servo position Plants

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

## 8. Regulated CR-servo position Plants

CR servos do not have a device-managed position mode. Position control requires native feedback and
a regulator:

```java
PositionPlant turret = FtcActuators.plant(hardwareMap)
        .crServo("turretServo", Direction.FORWARD)
        .position()
        .regulated()
            .nativeFeedback(FtcActuators.PositionFeedback.externalEncoder("turretEncoder"))
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

## 9. Feedback selectors

Builder feedback selectors are typed so errors are caught early.

### Native position feedback

* `PositionFeedback.internalEncoder()`
* `PositionFeedback.internalEncoder("leftLift")`
* `PositionFeedback.averageInternalEncoders()`
* `PositionFeedback.externalEncoder("liftEncoder")`
* `PositionFeedback.externalEncoder("liftEncoder", direction)`
* `PositionFeedback.fromSource(customNativePositionSource)`

These are native position sources. If the source already reports the plant units you want robot code
to use, choose `nativeUnits().alreadyReferenced()` after the geometry step.

### Velocity feedback

* `VelocityFeedback.internalEncoder()`
* `VelocityFeedback.internalEncoder("flywheel")`
* `VelocityFeedback.averageInternalEncoders()`
* `VelocityFeedback.externalEncoder("flywheelEncoder")`
* `VelocityFeedback.externalEncoder("flywheelEncoder", direction)`
* `VelocityFeedback.fromSource(customVelocitySource)`

The built-in encoder helpers use native FTC units:

* position: **ticks**
* velocity: **ticks/sec**

---

## 10. Measurement readback

Phoenix separates **commands** from **measurements**.

### Command channels

`FtcHardware` creates command-only HAL outputs:

* `PowerOutput`
* `PositionOutput`
* `VelocityOutput`

These answer “what command should we send?” — not “what did the mechanism measure?”

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
`PositionPlant.positionSource()` is also in plant units and is the preferred measurement source for
`ScalarSetpoints.plan().request(...).forPositionPlant(plant)`.

---

## 11. Position tolerances and FTC motor tuning

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

## 12. Velocity bounds, mapping, and tuning

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

## 13. Multi-motor groups

For grouped device-managed Plants, Phoenix supports per-child `scale(...)` / `bias(...)` and computes
one aggregate measurement in group units.

For grouped **regulated** Plants, the staged builder intentionally requires default per-child scaling
and bias. If you need a more advanced grouped regulated mechanism, build the raw outputs manually and
compose with `Plants.positionFromPower(...)` or `Plants.velocityFromPower(...)`.

That restriction keeps the common builder path simple and makes ambiguous regulated group semantics
fail fast with an actionable error.
