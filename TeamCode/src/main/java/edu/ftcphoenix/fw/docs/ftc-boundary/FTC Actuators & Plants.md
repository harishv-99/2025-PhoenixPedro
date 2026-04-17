# FTC Actuators & Plants

This page covers the FTC boundary for Phoenix mechanism wiring:

* `FtcActuators.plant(...)` — the staged beginner builder
* `FtcHardware` — low-level command channels
* `FtcSensors` — low-level measurement sources
* device-managed vs regulated control
* how tolerances and FTC motor tuning interact

---

## 1. The staged entrypoint: `FtcActuators.plant(...)`

Use `edu.ftcphoenix.fw.ftc.FtcActuators` when you want to create a `Plant` from FTC hardware.

```java
Plant flywheel = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .build();

Plant pusher = FtcActuators.plant(hardwareMap)
        .servo("pusher", Direction.FORWARD)
        .position()
        .build();
```

The public builder stays domain-first:

* `power()`
* `position()`
* `velocity()`

That keeps the common path simple while still allowing advanced overrides when needed.

---

## 2. Device-managed vs regulated control

For motors, the no-arg `position()` and `velocity()` methods default to the **device-managed FTC motor path**.

### Device-managed motor position

```java
Plant arm = FtcActuators.plant(hardwareMap)
        .motor("armMotor", Direction.FORWARD)
        .position()
        .build();
```

This uses FTC's motor-side position control and samples the motor's internal encoder for plant status.

### Regulated motor position

```java
Plant arm = FtcActuators.plant(hardwareMap)
        .motor("armMotor", Direction.FORWARD)
        .position(
                FtcActuators.MotorPositionControl.regulated(
                        FtcActuators.PositionFeedback.externalEncoder("armEncoder"),
                        ScalarRegulators.pid(Pid.withGains(0.006, 0.0, 0.0002))
                ).positionTolerance(20.0)
        )
        .build();
```

This uses:

* raw motor power as the actuation lever
* an explicit feedback source
* a framework-owned regulator

The same regulated path works with an internal encoder too:

```java
.position(
        FtcActuators.MotorPositionControl.regulated(
                FtcActuators.PositionFeedback.internalEncoder(),
                ScalarRegulators.pid(Pid.withGains(0.006, 0.0, 0.0002))
        )
)
```

### Regulated CR-servo position

CR servos do not have a device-managed position mode, so regulated control is the only supported position path:

```java
Plant turret = FtcActuators.plant(hardwareMap)
        .crServo("turretServo", Direction.FORWARD)
        .position(
                FtcActuators.CrServoPositionControl.regulated(
                        FtcActuators.PositionFeedback.externalEncoder("turretEncoder"),
                        ScalarRegulators.pid(Pid.withGains(0.01, 0.0, 0.0005))
                ).positionTolerance(8.0)
        )
        .build();
```

---

## 3. Feedback selectors

Builder feedback selectors are typed so errors are caught early:

### Position feedback

* `PositionFeedback.internalEncoder()`
* `PositionFeedback.internalEncoder("leftLift")`
* `PositionFeedback.averageInternalEncoders()`
* `PositionFeedback.externalEncoder("liftEncoder")`
* `PositionFeedback.externalEncoder("liftEncoder", direction)`
* `PositionFeedback.fromSource(customSource)`

### Velocity feedback

* `VelocityFeedback.internalEncoder()`
* `VelocityFeedback.internalEncoder("flywheel")`
* `VelocityFeedback.averageInternalEncoders()`
* `VelocityFeedback.externalEncoder("flywheelEncoder")`
* `VelocityFeedback.externalEncoder("flywheelEncoder", direction)`
* `VelocityFeedback.fromSource(customSource)`

The built-in encoder helpers use native FTC units:

* position: **ticks**
* velocity: **ticks/sec**

For custom units (inches, radians, RPM, mechanism travel, etc.), provide a derived `ScalarSource` via `fromSource(...)`.

---

## 4. Measurement readback

Phoenix now separates **commands** from **measurements**.

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

telemetry.addData("target", plant.getTarget());
telemetry.addData("measurement", plant.getMeasurement());
telemetry.addData("error", plant.getError());
telemetry.addData("atSetpoint", plant.atSetpoint());
```

`getMeasurement()` is always the **last value sampled during `plant.update(clock)`**. Phoenix does not silently resample hardware when you call `getMeasurement()`.

---

## 5. Position tolerances and FTC motor tuning

For device-managed motor position control there are two different tolerance concepts.

### `positionTolerance(...)`

This is the **plant-level** completion band used by `Plant.atSetpoint()`.

* default: **10 ticks** for the built-in motor-position helpers
* units: the plant's position units (ticks for the built-in encoder helpers)
* use this first when you want to define “close enough for robot logic”

### `devicePositionToleranceTicks(...)`

This is an **optional FTC motor-controller override** for the motor's own target-position tolerance.

* default in Phoenix: **unchanged unless you call it**
* units: **encoder ticks**
* use this only when you intentionally want to change the FTC motor controller's own completion threshold via `DcMotorEx.setTargetPositionTolerance(int)`
* this does **not** change what `Plant.atSetpoint()` means unless your plant-level `positionTolerance(...)` happens to match it

### Other device-managed motor-position knobs

* `maxPower(...)`
    * default: **1.0**
    * power used when Phoenix commands a new FTC `RUN_TO_POSITION` target

* `outerPositionP(...)`
    * default: **unchanged unless set**
    * FTC outer position-loop proportional gain

* `innerVelocityPidf(...)`
    * default: **unchanged unless set**
    * FTC inner velocity-loop PIDF used underneath position mode

### Device-managed motor-velocity knobs

* `velocityTolerance(...)`
    * default: **100 ticks/sec**
    * plant-level completion band used by `Plant.atSetpoint()`

* `velocityPidf(...)`
    * default: **unchanged unless set**
    * FTC device-managed velocity PIDF override

### Best practice

Start with the plant-level tolerances:

* `positionTolerance(...)`
* `velocityTolerance(...)`

That is what your robot logic, tasks, and `Plant.atSetpoint()` will observe.

Only add device-specific FTC tuning (`outerPositionP`, `innerVelocityPidf`, `devicePositionToleranceTicks`, `velocityPidf`) when you have a concrete reason to tune the motor controller itself. A good rule of thumb is:

* set `positionTolerance(...)` when you want to define **robot-level close enough**
* set `devicePositionToleranceTicks(...)` only when you want to change **motor-controller-level completion behavior**

---

## 6. Multi-motor groups

For grouped device-managed plants, Phoenix supports per-child `scale(...)` / `bias(...)` and computes one aggregate measurement in group units.

For grouped **regulated** plants, the staged builder intentionally requires default per-child scaling and bias. If you need a more advanced grouped regulated mechanism, build the raw outputs manually and compose with `Plants.positionFromPower(...)` or `Plants.velocityFromPower(...)`.

That restriction keeps the common builder path simple and makes ambiguous regulated group semantics fail fast with an actionable error.
