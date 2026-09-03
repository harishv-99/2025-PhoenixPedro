---
tags:
  - Advanced
---

# FTC Actuators & Plants

This page covers the FTC boundary for Sushi mechanism wiring:

* `FtcActuators.plant(...)` and `Plants.fromOutputs()` — two gateways into one staged grammar
* `FtcHardware` — low-level command channels
* `FtcSensors` — low-level measurement sources
* device-managed vs regulated control
* target, coordinate-reference, setpoint, PID, typed feedforward, and output-power vocabulary
* position plant geometry, plant/native unit mapping, and reference policy
* how tolerances and FTC motor tuning interact

The FTC boundary is where FTC SDK hardware names become Sushi `Plant` objects. In ordinary robot
code, the composition root passes `HardwareMap` and the active data-only profile slice to a
mechanism constructor. That mechanism defensively snapshots and validates its complete slice before
its own hardware lookup, uses the builders on this page, keeps the resulting Plants private, and
owns their update and stop lifecycle. A valid default proves only software shape, not the identity,
direction, or safe response of installed hardware. Robot services, Tasks, and scalar planners use
the mechanism's semantic API instead of touching either raw SDK devices or raw Plants directly.

The compact builder snippets below are therefore **mechanism-constructor excerpts** unless a
section explicitly labels a custom hardware adapter or other advanced boundary. Sections that
compare low-level builder alternatives may declare a short local solely to keep the API difference
visible; production mechanism code assigns the selected build to a private field, as the complete
examples do. The compiling
[`StarterIntakeMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.html>) shows an
ordinary direct-power private-Plant owner. The managed
[`ReferenceFlywheelMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelMechanism.html>)
adds grouped velocity plus independent member evidence. The separate launcher delegates that Plant
ownership while adding servo, sensor, overlay, and outcome-aware Task policy.

---

## 1. Two boundary gateways, one Plant grammar

Use `edu.ftcsushi.fw.ftc.FtcActuators` when a mechanism starts from FTC `HardwareMap`, configured
device names, and FTC controller choices:

```java
// Inside the mechanism constructor; flywheel and pusher are private fields.
this.flywheel = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManaged()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetFromNewCommand(0.0)
        .build();

this.pusher = FtcActuators.plant(hardwareMap)
        .servo("pusher", Direction.FORWARD)
        .position()
        .nonPeriodic()
        .bounded(0.0, 1.0) // normalized RETRACTED-to-EXTENDED mechanism coordinate
        .rangeMapsToNative(backedOffNativeRetracted, backedOffNativeExtended)
        .targetFromNewCommand(0.0)
        .build();
```

The `backedOffNative...` values above are copied, validated robot configuration established by
hardware review. Although both domains use numbers between zero and one here, the Plant target is a
normalized mechanism coordinate and the mapped values are FTC SDK servo commands.

Use `Plants.fromOutputs()` only when a custom adapter, hardware-neutral test, or portable host
already owns Sushi output ports and feedback sources:

```java
Plant roller = Plants.fromOutputs()
        .power(powerOut)
        .targetFromNewCommand(0.0)
        .build();

PositionPlant wrist = Plants.fromOutputs()
        .commandedPosition(positionOut)
        .nonPeriodic()
        .bounded(-45.0, 90.0)          // public Plant units
        .rangeMapsToNative(0.22, 0.76) // native output units
        .targetFromNewCommand(0.0)
        .build();
```

These are two boundary gateways into one staged grammar and hidden runtime engine, not two builder
implementations. `FtcActuators` alone owns SDK lookup, direction, grouping, run modes, and FTC
controller configuration. `Plants` remains hardware-neutral. Ordinary FTC code should not unwrap
devices merely to call the advanced gateway, and neutral code should not depend on an FTC-named
facade.

The builder is staged on purpose. Each required conceptual question is answered explicitly, while
optional tuning appears only after the user enters a tuning branch. This keeps autocomplete focused
on the next meaningful choice.

For example, motor position wiring asks:

1. Which hardware? `motor(...)`
2. Which target domain? `position()`
3. Who manages the position loop? Ordinary FTC `deviceManaged()`, advanced FTC
   `deviceManagedWithOverrides()...doneOverrides()`, or `regulated()` followed by one feedback
   answer
4. Is the public coordinate `nonPeriodic()` or `periodic(period)`?
5. What bounds in public Plant units are legal? `bounded(min, max)` with two finite endpoints, or
   `unbounded()`
6. How do Plant units map to native units? `nativeUnits()`, `scaleToNative(...)`, or bounded-only `rangeMapsToNative(...)`
7. How is the reference/offset known? `alreadyReferenced()`, `plantPositionMapsToNative(...)`, `assumeCurrentPositionIs(...)`, or `needsReference(...)`
8. What public position error counts as complete? The required `positionTolerance(...)` answer
9. For regulated control only: where does the per-cycle setpoint come from, what PID and optional
   typed feedforward produce normalized power, and what final output-power policy applies?
10. Optional normal-control output limit, when the selected output truthfully supports it:
    `outputPowerLimitedTo(...)`
11. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`,
    `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
12. Target binding: `targetFromNewCommand(initialValue)` for a numeric command,
    `targetExactlyFrom(command)` for an exact semantic command, or
    `targetFromResolver(finalResolver)` for a caller-supplied final resolver, then `build()`

Motor velocity wiring asks a parallel but smaller set of questions:

1. Which hardware? `motor(...)`
2. Which target domain? `velocity()`
3. Who manages the velocity loop? Ordinary FTC `deviceManaged()`, the one-answer advanced
   `deviceManagedWithOverrides().velocityPidf(...)` branch, or `regulated()` followed by one
   feedback answer
4. What target bounds in Plant units are legal? `bounded(min, max)` with two finite endpoints, or
   `unbounded()`
5. How do Plant velocity units map to native velocity units? `nativeUnits()` or `scaleToNative(...)`
6. What public velocity error counts as complete? The required `velocityTolerance(...)` answer
7. For regulated control only: choose direct or acceleration-limited setpoint behavior, PID,
   optional typed feedforward, and optional output-power policy
8. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`,
   `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
9. Target binding: `targetFromNewCommand(initialValue)`, `targetExactlyFrom(command)`, or
   `targetFromResolver(finalResolver)`, then `build()`

The neutral gateway exposes the same control choices without FTC acquisition: `power(...)`,
`commandedPosition(...)`, `deviceManagedPosition(...)`, `deviceManagedVelocity(...)`,
`regulatedPosition(...)`, and `regulatedVelocity(...)`. Each branch asks only the later facts its
inputs cannot prove. The tolerance question appears exactly once and only after the public
coordinate is known. Command-only position skips feedback-only reference, calibration-search, and
tolerance questions.

The control vocabulary is deliberately precise:

```text
coordinate reference -> aligns Plant position with native position
target               -> final mechanism goal selected by the resolver and guards
setpoint             -> one cycle's position / velocity / acceleration control state
output               -> normalized actuator effort after feedback, feedforward, and policy
```

`alreadyReferenced()`, `plantPositionMapsToNative(...)`, `assumeCurrentPositionIs(...)`, and
`needsReference(...)` answer only the coordinate question. They do not create a target or a motion
setpoint. Likewise, `setpointFromAppliedTarget()` does not establish zero or alter the coordinate
map: it says that the current guarded target is used directly as the controller setpoint.

For an ordinary exact Plant, keep one mechanism variable. `targetFromNewCommand(initialValue)` is the concise
spelling of creating one fresh `ScalarTarget` and binding `PlantTargets.exact(...)`; the Plant's
stable, side-effect-free `commandTarget()` returns that same target identity. It rejects a
non-finite or out-of-Plant-range initial value before build instead of treating configuration as a
runtime request to clamp. This configures the source graph but does not command hardware during
construction. A named `ScalarTarget` remains useful when it stands alone, is shared with
target-only policy, or is needed while assembling an overlay, equivalent-position, or advanced
resolver graph; bind it explicitly with `targetFromResolver(PlantTargets.exact(target))`.
`targetFromResolver(...)` binds the supplied resolver without promising that the built Plant has a
command target. A recognized exact `ScalarTarget` or command-bearing framework graph carries one; a
read-only, planned, or arbitrary custom resolver carries none. A read-only `ScalarSource` crosses
the same boundary through `PlantTargets.exact(source)`. For
`PlantTargets.overlay(...)`, only a `ScalarTarget` carried by the base graph becomes the command
target; conditional layers never do.

When the robot API names a semantic request such as `Height`, `Mode`, or `Pose`, use a
[`SemanticScalarCommand<S>`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarCommand.html>)
instead of exposing the numeric command. Its mapper validates and publishes one immutable
semantic/numeric request, and every successful set receives a fresh request identity even when the
name and number repeat. Use `forEnum(initial).map(...).build()` when a fixed enum table should fail
fast on incomplete coverage; keep `create(initial, mapper)` for computed mappings and non-enum
types. Bind an exact command through `targetExactlyFrom(command)`. If periodic positions are
interchangeable, use `targetFromResolver(PlantTargets.equivalentPositionsOf(command))` explicitly;
equivalent-position and overlay resolvers retain the same private provenance. The command
intentionally provides no `ScalarTarget`, so direct numeric
`ScalarTasks` cannot bypass the owner. Build immediate, timed, and feedback-aware named Tasks with
[`SemanticScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarTasks.html>)
through that same command.

Velocity and power Plants stay simpler than position Plants because they do not have position
periodicity or homing/reference questions. Power is simpler still: every direct power Plant has the
fixed normalized range `[-1.0, +1.0]`, so there is no redundant bounds step. A finite request outside
that range clamps during normal targeting.

### One final Plant lifecycle action

`Plant.stop()` permanently ends that Plant instance. It claims the terminal state before invoking
external cleanup and immediately attempts the selected realization's natural stop. Every later
`update(clock)` is inert before feedback, resolver or plan, guards, controllers, and hardware. The
stop leaves the resolver graph, target guards, and optional command target unchanged, so a
planned/read-only Plant needs no special disable path and a shared command is not silently changed
for another consumer.
Repeated stop is harmless; even when cleanup throws, that Plant remains terminal. Construct a new
Plant for another lifecycle.

When stop returns normally, target status is `STOPPED`; power and velocity applied targets are zero,
while a position applied target remains at its last applied value. If an output stop throws, the
lifecycle is still terminal, but Sushi retains the prior applied/status/resolution facts instead
of inventing zero, hold, or successful-stop proof.

The contract is shared by `Plant` and `PositionPlant`, but the natural hardware attempt remains
truthful to the realization:

| Realization | Natural final-stop attempt |
| --- | --- |
| Power | Submit zero power |
| Velocity | Submit zero velocity, or zero regulated power |
| Standard-servo position | Reassert the last successfully submitted position; do nothing if this adapter has never submitted one |
| Device-managed motor position | Remove power and return to `RUN_USING_ENCODER` |
| Framework-regulated position | Stop the regulated power channel and reset controller state |

This is different from active-match idle. Use a zero target when a power or velocity mechanism
should remain available, and use an ordinary position target when an active mechanism should hold.
Do not add a second public pause/resume or output-stop path around the Plant source graph.

### Grouped hardware-name identity

The student-facing calls stay unchanged, but grouped construction validates the configured names
before it touches new group hardware. The FTC SDK trims surrounding whitespace before a
case-sensitive lookup, so Sushi uses the same effective identity:

* `"left"` and `" left "` select the same configured name and cannot both belong to one group.
* `"left"` and `"Left"` remain distinct configured names.
* The caller's original string is retained for FTC lookup and diagnostics.

Each motor, standard-servo, or CR-servo command group requires nonblank, distinct names under that
rule. A name is checked before it is accepted into the staged group, and the complete group is
preflighted before fresh hardware resolution or configuration. If code retains an earlier builder
stage and later tries to add a bad member through that alias, rejection does not mutate the group or
cause additional hardware effects.

This check is intentionally scoped to one homogeneous command group. It does not prove that two
different Robot Configuration entries identify different physical devices, and it is not a global
ownership registry. The same name may be used by separately constructed owners when robot lifecycle
policy makes that reuse intentional. Feedback is also a separate role: a regulated Plant may
deliberately read an internal or external encoder through the same configured name as one of its
commanded actuators.

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

### FTC numeric-domain ownership

The FTC boundary also owns the last representation check that a hardware-neutral Plant cannot
know:

* a standard-Servo command must be finite and its raw domain is `[0.0, 1.0]`;
* a motor-velocity command must be finite, but Sushi does not invent a controller-independent
  maximum ticks/second;
* a motor-position command must be finite, round once to a `long`, fit inside the SDK's signed
  integer target-position domain, and only then narrow to `int`; and
* framework-derived motor/CR-servo power children must be finite and inside normalized
  `[-1.0, +1.0]` before group fan-out.

These checks happen before the cache, mode, target, or power effects owned by the corresponding raw
adapter. The standard Servo adapter still clamps a finite direct expert command into `[0.0, 1.0]`,
and the raw power adapters retain their finite saturation defense. Those clamps are last-resort
boundary protection, not a way for an accepted Plant recipe to hide an invalid map. Raw
`PowerOutput` write/cache/cleanup behavior follows the contract below.

### Power-output command and failure truth

Every conforming `PowerOutput`, including a custom adapter supplied through
`Plants.fromOutputs()`, reports only top-level logical command truth. Its command cache starts as
`NaN`, becomes unknown before every command or stop attempt, and publishes a value only after that
complete operation returns normally. A throwing validation, preflight, child, SDK, or cleanup call
leaves it unknown. A later independently invoked successful command or stop may establish a known
logical value again.

A non-finite raw request is never forwarded as requested power. The output creates an actionable
`IllegalArgumentException` and best-effort invokes its natural zero behavior. After normal cleanup
or a cleanup `RuntimeException`, it throws that primary with any distinct cleanup
`RuntimeException` suppressed. A cleanup `Error` remains uncaught. That nested zero is cleanup, not
a separate successful top-level command, and the top-level cache remains unknown. FTC motor cleanup
writes zero without acquiring or restoring a run mode. Finite FTC motor and CR-servo requests
retain their last-resort saturation to `[-1.0, +1.0]`, and publish the saturated value only after
all adapter work returns normally.

The exact and mapped FtcActuators power groups calculate and validate every child command before
the first requested write. A finite mapping rejection therefore performs no requested child write
or cleanup, while leaving the attempted group's cache unknown. A non-finite top-level request
instead begins an ordered best-effort stop traversal and throws without requested fan-out. Once
requested fan-out has begun, a child write `RuntimeException` abandons later requested writes and
begins the same traversal, including the failed and not-yet-written children. Cleanup continues
across `RuntimeException`s; the exact write failure remains primary and distinct cleanup
`RuntimeException`s are suppressed in order. An `Error` remains uncaught and can interrupt later
cleanup.

Grouped `stop()` attempts every child in declaration order, continuing across
`RuntimeException`s. It publishes group zero only if every child stop returns normally; otherwise
the first `RuntimeException` remains primary, later distinct failures are suppressed, and the group
cache is `NaN`. An `Error` remains uncaught and can interrupt later stops. A child may publish zero
after its own successful cleanup while the enclosing failed group remains unknown. These ordered
SDK calls are not atomic and do not roll back. Neither a finite cache value nor `NaN` proves SDK
acceptance, controller state, electrical output, physical zero, actuator response, or safety.

Raw output operations remain repeatable so an independently owned cleanup attempt is never
suppressed and an isolated deterministic fixture can test logical recovery. That is not a robot-host
recovery policy: a `RuntimeException` escaping any managed or approved advanced host lifecycle
phase is terminal and triggers the host's owned best-effort cleanup.

---

## 2. Behavior sources vs Plant target guards

Every robot-facing Plant is source-driven:

```text
behavior PlantTargetResolver
    ↓
requested target + PlantTargetResolution
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

`getTargetResolution()` explains how behavior selected the requested target. `getTargetStatus()` explains
how the Plant protected and applied that request.

Use behavior sources for robot policy:

```java
PlantTargetResolver finalFeeder = PlantTargets.overlay(baseFeeder)
        .add("feedPulse", feedQueue.activeSource(), feedQueue)
        .add("eject", ejectRequested, -1.0)
        .build();
```

Use `targetGuards()` only for Plant-level protection that should apply no matter which behavior
requested the target:

```java
this.lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManaged()
        .nonPeriodic()
        .bounded(0.0, 4200.0)
        .nativeUnits()
        .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetGuards()
            .maxTargetRate(1200.0)
            .holdLastTargetUnless("wristClear", wristClear)
            .doneTargetGuards()
        .targetFromNewCommand(0.0)
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
sample initializes directly to the first guarded candidate. If loop time is temporarily non-finite,
it holds the last output until a finite time baseline has been restored. If a mechanism needs
startup limiting from a known physical position, initialize the command target from that measurement
or use an appropriate reference policy before requesting a far-away target.

---

## 3. Plant units vs native units

For position and velocity Plants, Sushi distinguishes two coordinate systems:

* **Plant units** are the public units used by robot code, `ScalarTarget` requests, resolved Plant
  targets, scalar planner requests, target ranges, position periods, reference values, and plant-level tolerances.
* **Native units** are the units used by the selected hardware/control path: servo raw fraction,
  motor encoder ticks, motor ticks/sec, external encoder units, or a caller-supplied feedback source.

Public position and velocity APIs use **plant units** unless the method name explicitly says
**Native**.

As a rule of thumb:

* `bounded(...)`, `unbounded()`, `periodic(...)`, `targetFromNewCommand(...)`,
  `targetExactlyFrom(...)`, `targetFromResolver(...)`, `getRequestedTarget()`,
  `getAppliedTarget()`, `getMeasurement()`, `positionTolerance(...)`, and `velocityTolerance(...)` all speak in
  **plant units**.
* Methods that cross the plant/native boundary say so explicitly in the name or docs, for example
  `nativeUnits()`, `scaleToNative(...)`, `rangeMapsToNative(...)`,
  `plantPositionMapsToNative(...)`, and `devicePositionToleranceTicks(...)`.

Examples:

```java
.nonPeriodic().bounded(0.0, 18.0)     // inches if the mechanism is declared in inches
.periodic(360.0)                      // degrees if the mechanism is declared in degrees
.positionTolerance(0.10)              // plant units
.assumeCurrentPositionIs(0.0)         // plant units
.establishReferenceAt(0.0)            // plant units, through PositionCalibrationTasks

.scaleToNative(TICKS_PER_INCH)        // native ticks per plant inch
.rangeMapsToNative(0.30, 0.80)        // native servo fractions at plant-range endpoints
.plantPositionMapsToNative(0.0, ARM_ZERO_TICKS) // plant position 0 maps to native encoder tick offset
.devicePositionToleranceTicks(12)     // explicitly native/controller ticks
```

The complete normal target-command pipeline is:

```text
Plant target
    -> shared Plant-to-native map
    -> childNative = childScale * sharedNative + childBias
    -> FTC adapter
```

The hardware-neutral mapping engine owns finite arithmetic and reference state. `FtcActuators`
adds the selected FTC family, control-path, and raw-domain facts. This is why
`Plants.fromOutputs()` can validate overflow but cannot assume that an arbitrary `PositionOutput`
uses the standard-Servo `[0.0, 1.0]` or FTC integer-tick domain.

A temporary grouped device-managed motor-position calibration search is deliberately outside this
position-coordinate pipeline. Its one normalized power command fans out identically through each
motor's configured `Direction`; position scale and bias do not transform raw search power.

Every explicitly supplied position-reference component must be finite. These values define a
coordinate anchor rather than a target request, so the plant-unit reference need not lie inside the
Plant's declared target range. Reference answers are rejected rather than clamped. A post-reference
hold is different: it is a finite plant-unit logical command that still passes through the complete
resolver, range, overlays, and target guards.

This convention keeps common robot code readable while making boundary-crossing methods obvious.

---

## 4. Motor position control: device-managed or regulated

After `motor(...).position()`, Sushi asks who manages the position loop. The two answers share
the same coordinate, reference, tolerance, target, lifecycle, and completion vocabulary. They differ
only where the controller owner provides different evidence.

### Device-managed position: the ordinary FTC path

Use `deviceManaged()` when FTC `RUN_TO_POSITION` owns the control loop:

```java
this.lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .deviceManaged()
        .nonPeriodic()
        .bounded(0.0, 18.0)                 // Plant position: inches
        .scaleToNative(TICKS_PER_INCH)
        .needsReference("lift not homed")
        .positionTolerance(0.10)            // Plant position: inches
        .outputPowerLimitedTo(0.70)         // optional normal RUN_TO_POSITION magnitude
        .targetFromNewCommand(0.0)
        .build();
```

Omitting `outputPowerLimitedTo(...)` uses the full normalized magnitude `1.0`. That is a valid
software default, not a claim that full power is physically safe. The answer lives after
`positionTolerance(...)`, beside the same normal-output policy on regulated control; it is not an
FTC controller-coefficient override.

This capability is truthful because the FTC motor-position adapter implements
`PowerLimitedPositionOutput`: every command carries a native target and its maximum normalized
output-power magnitude together through one paired logical call. This is not a hardware
transaction: the FTC adapter prevalidates both values, then performs sequential target, mode, and
power SDK effects. If a later effect fails, an earlier effect may remain installed and the cached
submitted pair becomes unavailable. A plain hardware-neutral `PositionOutput` and a standard Servo
do not promise effort control and therefore do not expose this answer.

At the advanced hardware-neutral gateway, the compile-time capability determines the staged tail:

```java
PowerLimitedPositionOutput motorOut =
        FtcHardware.motorPosition(hardwareMap, "liftMotor", Direction.FORWARD);

PositionPlant lift = Plants.fromOutputs()
        .deviceManagedPosition(motorOut, nativePositionSource)
        .nonPeriodic()
        .bounded(0.0, 4200.0)
        .nativeUnits()
        .alreadyReferenced()
        .positionTolerance(20.0)
        .outputPowerLimitedTo(0.70)
        .targetFromNewCommand(0.0)
        .build();
```

Passing a plain `PositionOutput` to the parallel overload reaches target selection immediately
after `positionTolerance(...)`; no runtime type probe or unsupported power method appears.

### Device-managed position with FTC overrides

Use `deviceManagedWithOverrides()` only when the recipe deliberately changes FTC controller
configuration. The branch must accept at least one override before `doneOverrides()`; use ordinary
`deviceManaged()` when there is nothing to override.

```java
this.lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .deviceManagedWithOverrides()
            .outerPositionP(5.0)
            .innerVelocityPidf(innerP, innerI, innerD, innerF)
            .devicePositionToleranceTicks(12)
            .doneOverrides()
        .nonPeriodic()
        .bounded(0.0, 4200.0)
        .nativeUnits()
        .needsReference("lift not homed")
        .positionTolerance(20.0)
        .outputPowerLimitedTo(0.70)
        .targetFromNewCommand(0.0)
        .build();
```

The override branch contains only FTC-controller facts:

* `outerPositionP(...)` — FTC outer position-loop proportional gain;
* `innerVelocityPidf(...)` — FTC inner velocity-loop PIDF used under position mode; and
* `devicePositionToleranceTicks(...)` — FTC controller target tolerance in native ticks.

`outputPowerLimitedTo(...)` remains after Plant-unit tolerance for both ordinary and overridden
device-managed recipes. Keeping it out of the override branch makes the normal-output policy
parallel with regulated Plants and leaves calibration-search power independent.

### Framework-regulated position: the standard control model

Use `regulated()` when Sushi should command normalized motor power from position feedback. After
feedback, coordinate mapping, reference, and tolerance, choose one setpoint model and PID directly
in the Plant recipe. A simple lift that applies its target immediately looks like this:

```java
this.lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .regulated()
            .externalEncoder("liftEncoder")
        .nonPeriodic()
        .bounded(0.0, 18.0)
        .scaleToNative(TICKS_PER_INCH)
        .needsReference("lift not homed")
        .positionTolerance(0.10)
        .setpointFromAppliedTarget()
        .feedbackFromPid(kP, kI, kD)
        .feedforwardFromLift(kG)
        .outputPowerLimitedTo(maximumPower)
        .targetFromNewCommand(0.0)
        .build();
```

`feedbackFromPid(kP)` is the shorter P-only answer and means exactly `kI = 0` and `kD = 0`.
Feedforward is optional; omitting it means exact zero feedforward. There is no hidden PID gain,
gravity term, profile limit, tolerance, or safe-power assumption.

For controlled acceleration and complete motion evidence, use a trapezoidal position setpoint:

```java
this.lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .regulated()
            .internalEncoder()
        .nonPeriodic()
        .bounded(0.0, 18.0)
        .scaleToNative(TICKS_PER_INCH)
        .needsReference("lift not homed")
        .positionTolerance(0.10)
        .setpointFromTrapezoidalProfile(maxVelocityInPerSec,
                                       maxAccelerationInPerSec2)
        .feedbackFromPid(kP, kI, kD)
        .feedforwardFromLift(kG, kS, kV, kA)
        .outputPowerLimitedTo(maximumPower)
        .targetFromNewCommand(0.0)
        .build();
```

An arm uses the same grammar but a position-dependent gravity model:

```java
.setpointFromTrapezoidalProfile(maxDegreesPerSec, maxDegreesPerSec2)
.feedbackFromPid(kP, kI, kD)
.feedforwardFromArm(kG,
                    degreesAtMaximumGravity,
                    Math.PI / 180.0,
                    kS, kV, kA)
.outputPowerLimitedTo(maximumPower)
```

The standard controller evaluates one immutable setpoint snapshot per Plant cycle:

```text
error    = controlledSetpoint - measurement
feedback = kP*error + integral(kI*error*dt) + kD*delta(error)/dt
motionFF = kS*sign(vSetpoint) + kV*vSetpoint + kA*aSetpoint
liftFF   = motionFF + kG
armFF    = motionFF
           + kG*cos((pSetpoint - positionAtMaximumGravity)*radiansPerPlantUnit)
output   = outputPolicy(voltageScale * (feedback + feedforward))
```

`sign(0)` is exactly zero. Every gain and geometry value must be finite. `kS` and `kG` are normalized
power; `kV` is normalized power per setpoint-velocity unit; `kA` is normalized power per
setpoint-acceleration unit. `positionAtMaximumGravity` uses Plant position units, and
`radiansPerPlantUnit` converts those units to radians. If the controlled coordinate unit is `U`,
PID gain units are normalized power / `U` for `kP`, normalized power / (`U * second`) for `kI`, and
normalized power / (`U / second`) for `kD`.

The staged types expose only models supported by the chosen evidence:

| Setpoint choice | Available feedforward evidence |
| --- | --- |
| Direct position | no motion term; optional lift or arm gravity |
| Trapezoidal position | position, velocity, acceleration; motion, lift, or arm |
| Direct velocity | velocity but no acceleration; motion or lift without `kA` |
| Acceleration-limited velocity | velocity and acceleration; motion or lift with optional `kA` |

Profile limits must be positive and finite. Position maximum velocity uses Plant position units per
second and maximum acceleration uses Plant position units per second squared. The velocity-profile
maximum acceleration uses Plant velocity units per second. A standard profile and
`targetGuards().maxTargetRate(...)` are two competing motion shapers, so one recipe cannot select
both; use the profile for ordinary regulated motion. Hold/fallback interlocks remain compatible
because they answer whether a target may be applied, not how the setpoint approaches it.

On the first update, after reset, and after calibration changes the coordinate reference, standard
control seeds position from the finite measured position with zero velocity/acceleration, or seeds
velocity from the finite measured velocity. It never charges the `dtSec()` interval from before
that boundary. A changed target replans from the retained setpoint state rather than jumping a
profile back to the old goal. One successful update advances the profile/PID and writes at most once
per `LoopClock.cycle()`; a same-cycle failure is retained and rethrown.

For standard profiled control, `Plant.atTarget()` becomes true only when all ordinary Plant evidence
agrees **and** the setpoint has settled at the current applied target. Measurement entering the
tolerance band while the profile is still moving is not completion. Direct setpoints are settled
immediately by definition. The advanced custom-regulator seam supplies no separate profile-settled
claim, so completion follows its existing Plant measurement/status evidence.

Use `feedbackIntegralLimitedTo(minimum, maximum)` to limit the retained integral contribution and
`feedbackOutputLimitedTo(minimum, maximum)` to limit the complete P+I+D contribution before
feedforward. The final `outputPowerLimitedTo(...)` policy covers feedback, feedforward, and any
voltage compensation. All limits saturate finite excursions only; non-finite math remains a
fail-closed error. The standard controller also prevents integral growth farther into final
saturation while still allowing integral unwind.

### Output-power policy and voltage compensation

The one-argument policy is a symmetric normalized magnitude:

```java
.outputPowerLimitedTo(0.65)       // [-0.65, +0.65]
```

Software-regulated power paths additionally support an asymmetric interval that contains zero:

```java
.outputPowerLimitedTo(0.0, 0.65)  // flywheel may drive forward or stop
```

Both overloads reject invalid or non-finite configuration rather than clamping it. Omission means
the full normal-control domain `[-1.0, +1.0]`. This policy constrains normalized actuator output,
not Plant targets, and never replaces `bounded(...)` or `targetGuards()`. It applies only to normal
control. Position calibration search uses its separately validated explicit normalized search power
and bypasses the normal setpoint, PID, feedforward, voltage, and output-policy chain.

When voltage compensation is needed, answer it before the final limit:

```java
.voltageCompensationFrom(batteryVoltage,
                         13.0,  // tuning/reference voltage
                         9.0,   // denominator floor
                         1.4)   // maximum multiplier
.outputPowerLimitedTo(0.0, maximumFlywheelPower)
```

The final limit therefore covers the compensated result. An unavailable/non-positive voltage uses
scale `1.0`; finite low voltage is bounded by the denominator floor and maximum multiplier.

### Advanced custom-regulator seam

Use `controlFromCustomRegulator(regulator)` only for a genuinely custom complete control law. It
appears after units and tolerance, receives the guarded applied target and Plant-unit measurement,
and returns to the same output-power policy:

```java
.positionTolerance(0.10)
.controlFromCustomRegulator(customRegulator)
.outputPowerLimitedTo(maximumPower)
.targetFromNewCommand(0.0)
```

This seam does not create a typed trajectory snapshot or claim velocity/acceleration evidence;
custom code owns its complete law, semantics, and reset behavior. New ordinary Plants use the
inline `setpointFrom... -> feedbackFromPid(...) -> feedforwardFrom...` grammar. A genuinely custom
law implements `ScalarRegulator` directly; the remaining generic voltage/output decorators are
advanced composition tools for that custom regulator. `Pid` and `PidController` remain the
separate error-centric controller family for guidance and other non-Plant calculations; Sushi no
longer exposes the obsolete peer software-PIDF construction layer.

### Regulated command truth and fail-stop behavior

For a framework-regulated position or velocity Plant, the complete control result passes through one final
normalized-power boundary immediately before the configured `PowerOutput`:

For regulated **position**, that same `PowerOutput` channel also realizes calibration search. This
does not reuse the normal controller's numeric result: normal control submits each complete result, while
search mode bypasses the regulator and submits the independent power selected by
`PositionCalibrationTasks.search(...).withPower(...)`. The modes are mutually exclusive. A
device-managed position Plant needs an additional raw-power adapter only because its normal
`PositionOutput` cannot express open-loop power; `FtcActuators` derives both adapters from the same
named motor group for ordinary robot code.

| Control/output event | Plant behavior |
|---|---|
| Finite result inside `[-1.0, +1.0]` | Submit it unchanged, including exact boundaries and signed zero. |
| Finite result outside `[-1.0, +1.0]` | Saturate it to the nearest boundary and submit the normalized value. |
| `NaN` or either infinity | Do not submit it; best-effort stop the output, reset control state, and throw an actionable failure. |
| Control evaluation or output write throws a `RuntimeException` | Best-effort stop and reset, then rethrow the original failure with cleanup `RuntimeException`s suppressed; an `Error` remains uncaught. |

Finite saturation at this universal boundary is normal actuator-domain behavior. It does not reset
control state. Standard control uses post-control `outputPowerLimitedTo(...)` for an intentional
narrower/asymmetric policy and prevents integral growth farther into that final saturation. A custom
regulator owns its own anti-windup; the generic `outputLimited(...)` decorator remains an advanced
way to bound a composition before it is passed through `controlFromCustomRegulator(...)`.

The public Plant has no reset or restart method. Standard profile/PID/feedforward state is private
to the Plant and resets at its owned lifecycle boundaries. An advanced custom-regulator owner may
reset its retained regulator while it separately owns a safe actuator policy, but that reset does
not send a hardware command. Terminal `Plant.stop()` attempts to submit zero before resetting control and
attempts both operations even when one fails. If the output-zero submission succeeds but the later
control reset fails, the stop throws while retaining the truthful seam-level "zero submitted"
fact. If the output stop itself throws, cleanup cannot invent that fact. Either way the Plant remains
terminal, and `atTarget()` and `atTarget(value)` cannot become true from a later update.

A runtime mapping, regulator, or output failure uses an internal nonterminal fail-stop, resets the
control state it owns, and propagates the original failure. This is not public `Plant.stop()`: an
isolated deterministic fixture can exercise the primitive's mechanically available
different-cycle attempt. It is not permission for a robot host to recover. The managed runtime and
every approved advanced host treat a propagated lifecycle failure as terminal and perform owned
best-effort cleanup. A successful internal output stop may publish target status `STOPPED` as a
diagnostic fact; `PlantTargetStatus` is not a terminal-lifecycle query.

The debug fields deliberately keep different kinds of truth separate:

* `.regulatorOutput` is the raw regulator result. Existing `.output` on lower-level regulated
  Plants and `.lastRegulatorOutput` on regulated position Plants remain raw aliases.
* `.normalizedPowerCommand` is the last known value submitted by a normally returning top-level
  `PowerOutput` operation performed by the regulated-command boundary. It becomes unknown when a
  throwing output operation prevents truthful command bookkeeping. An open-loop position-
  calibration search bypasses this boundary and may command the same configured output later; use
  its separate search state when interpreting diagnostics. The normalized command is not a motor
  measurement or hardware acknowledgement.
* `.regulatedPowerStatus` explains whether the last operation submitted, saturated and submitted,
  reset without writing, stopped, or failed, including fail-stop and reset outcomes.
* `.regulator` remains the nested regulator-specific diagnostic prefix.
  Standard control reports its retained setpoint position/velocity/acceleration and availability,
  profile-settled state, feedback error/integral/output and limiting, typed feedforward output,
  voltage sample/scale, and final output/limit facts beneath that prefix. Diagnostics read cached
  state only; they do not advance control or resample voltage.

For a regulated Plant, `getAppliedTarget()` remains the final mechanism target in plant units; it is
not either of the power-command fields above. A custom or grouped `PowerOutput` may transform one
top-level command into several child commands, so these diagnostics do not claim per-child or
physical actuator truth. Standard FTC adapter saturation remains defense in depth. Open-loop
position-calibration search power is a separate configuration path and is not a regulator result.

An advanced output-limit decorator reports its unconstrained and applied results through regulator
debug data, delegates `reset()` to its inner regulator, and rejects a non-finite inner result rather
than hiding broken custom math behind a bound. Robot behavior or realization still decides whether
disabled means coast or hold; terminal Plant lifecycle remains owned by the Plant.

---

## 5. Position periodicity and bounds

Every declared position Plant answers periodicity and bounds. The general position grammar offers:

```java
.nonPeriodic()         // no declared fixed equivalence period
.periodic(period)      // equivalent positions separated by period, in Plant units

.bounded(min, max)     // legal target range in public Plant units
.unbounded()           // no software target range
```

These are the complete ordinary builder choices. `bounded(min, max)` requires two finite endpoints
with `min <= max`; do not use `NaN` or infinity to imply a missing bound. `unbounded()` is the
explicit no-software-bound answer, but its targets must still be finite. A standard-servo position
path is bounded-only because the FTC device proves a finite native `[0.0, 1.0]` command domain. The
advanced `ScalarRange` value used by custom Plant/planner protocols can represent one-sided ranges,
but those shapes are deliberately not additional `Plants.fromOutputs()` or `FtcActuators` stage
answers. See
[`Mechanism Target Planning`](<../drive-vision/Mechanism Target Planning.md#advanced-target-range-protocol>).

Common choices:

```java
// Lift or slide.
.nonPeriodic()
    .bounded(0.0, 4200.0)

// Raw tuning motor where software travel limits are intentionally not declared yet.
.nonPeriodic()
    .unbounded()

// Cable-limited turret: facing repeats every rotation, but legal travel is finite.
.periodic(TICKS_PER_TURN)
    .bounded(-900.0, 1100.0)

// Tray/indexer that can rotate continuously.
.periodic(TICKS_PER_REV)
    .unbounded()
```

`PositionPlant.periodicity()` returns `NON_PERIODIC` or `PERIODIC`; `period()` is separate data in
Plant units and is meaningful only for a periodic coordinate. A mechanism is periodic only when
positions separated by its period are actually interchangeable. A limited servo arm can rotate and
still be non-periodic, while a rotating plate can be periodic. Physical rotation, affine command
mapping, and periodic equivalence are separate facts.

For one normal logical command, wrap its final resolver
with `PlantTargets.equivalentPositionsOf(commandTarget)` and choose a preference plus explicit
unavailable answer. The wrapper uses the consuming Plant's declared period during
`plant.update(clock)`; robot code does not repeat it. `PlantTargets.plan(request)` remains the
advanced path for multiple alternatives, relative/explicit-period requests, observation metadata,
or clamp policy.
Periodic equivalence alone never wraps an exact target automatically.

---

## 6. Mapping plant units to native units

After periodicity and bounds, the builder asks how the public Plant coordinate maps to the selected
native coordinate.

Every explicit mapping coefficient or endpoint must be finite. An invalid numeric answer throws
`IllegalArgumentException` before it changes the retained recipe, so a retained stage can retry
with a valid value. A mapping that is numerically finite by itself but incompatible with the later
actuator/control branch throws `IllegalStateException` when the staged recipe first has enough
information to decide. Target selection and `build()` repeat the complete preflight before hardware
resolution so an older retained builder alias cannot bypass it.

### `nativeUnits()`

Use this when plant units are native units:

```java
.nonPeriodic()
    .bounded(0.0, 4200.0)
    .nativeUnits()
    .needsReference("lift not homed")
```

Here the lift target is in encoder ticks.

### `scaleToNative(...)`

Use this when the scale is known but the reference/offset is a separate question:

```java
.nonPeriodic()
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

With declared plant bounds `plantMin` and `plantMax`, Sushi first computes the scale, then uses
the same multiply-before-add order as the runtime map:

```text
nativePerPlantUnit = (nativeAtPlantMax - nativeAtPlantMin)
                   / (plantMax - plantMin)

native = nativeAtPlantMin
       + nativePerPlantUnit * (plant - plantMin)
```

For every fully known static bounded affine map, Sushi evaluates the same runtime arithmetic at
both inclusive Plant endpoints before build. Affinity makes those endpoint images a complete
numeric proof, including a negative scale or reversed native endpoints. The check is exact: a
one-ULP raw-domain overshoot is rejected rather than rounded or clamped into place. A mapping whose
native offset awaits `assumeCurrentPositionIs(...)` or `needsReference(...)` instead validates the
candidate core map at reference commit and checks the FTC raw domain before each realized write.

So for a servo declared as `bounded(-45.0, 90.0)`, calling `rangeMapsToNative(0.22, 0.76)` means
“plant `-45.0` maps to raw servo `0.22`, and plant `90.0` maps to raw servo `0.76`.”

Those native values are FTC SDK logical commands. The configured FTC servo type/controller maps
them to PWM, while servo programming and mechanics determine physical travel.
`rangeMapsToNative(...)` does not change any of those downstream mappings, measure shaft angle, or
prove that a linkage is physically linear. The ordinary supervised way to establish candidate
endpoints is [`HW: Actuator Bring-up`](<../testing-calibration/Actuator Bring-up.md>).

Example:

```java
private final PositionPlant claw;

// In the claw mechanism constructor; production values come from its copied config.
this.claw = FtcActuators.plant(hardwareMap)
        .servo("clawServo", Direction.FORWARD)
        .position()
        .nonPeriodic()
        .bounded(0.0, 1.0)
        .rangeMapsToNative(0.30, 0.80)
        .targetFromNewCommand(0.0)
        .build();
```

The mechanism exposes logical intent and owns the Plant update:

```java
public void close() {
    claw.commandTarget().set(0.0); // raw servo 0.30
}

public void open() {
    claw.commandTarget().set(1.0); // raw servo 0.80
}

void update(LoopClock clock) {
    claw.update(clock);
}
```

`rangeMapsToNative(...)` is intentionally not available after `unbounded()` because there is no
finite plant range to map from.

`unbounded()` still supports `nativeUnits()` and `scaleToNative(...)`; no finite scale can prove a
safe result for every possible finite `double`. The hardware-neutral Plant therefore computes each
actual Plant-to-native conversion into a temporary and requires a finite result before changing the
applied target or calling the output. A dynamic position reference similarly validates its complete
candidate core map before committing the reference or public-measurement state. The FTC output then
precomputes every final child command and native-domain check before writing the first child; this
later layer does not claim a transactional Plant-state rollback. Inverse child, aggregate, and final
Plant-unit feedback arithmetic must finish finite; otherwise measurement is unavailable and
`atTarget()` is false.

A runtime forward-mapping overflow throws an actionable `IllegalStateException` before submitting
the invalid command, then best-effort invokes the output's natural nonterminal fail-stop. It does
not call terminal `Plant.stop()`, so an isolated deterministic fixture can exercise a later-cycle
primitive attempt. A failure escaping any managed or approved advanced host remains terminal for
that host. If cleanup itself fails, that failure is suppressed under the original mapping failure
and prior applied/status/resolution state is retained. This deterministic mapping policy does not
claim that arbitrary SDK writes are transactional.

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
.nonPeriodic()
    .bounded(-35.0, 125.0)
    .nativeUnits()
    .alreadyReferenced()
```

### `plantPositionMapsToNative(plantPosition, nativePosition)`

Use this when both scale and one offset point are known in code:

```java
.nonPeriodic()
    .bounded(-35.0, 125.0)
    .scaleToNative(TICKS_PER_DEGREE)
    .plantPositionMapsToNative(0.0, ARM_ZERO_TICKS)
```

This means plant position `0.0` maps to native encoder tick `ARM_ZERO_TICKS`. Both the plant and
native reference values must be finite. The builder rejects `NaN` or infinity before retaining this
answer, performs no SDK call or hardware effect for that rejection, and does not clamp either
coordinate. It therefore prevents a later build from realizing command hardware with the invalid
pair. Feedback-selection stages retain only validated recipe data (or a caller-supplied neutral
source); named FTC sensor lookup and actuator lookup/configuration wait until the complete recipe
passes validation at `build()`. The plant reference is an affine-map anchor and need not be a legal
target inside the declared Plant range.

### `assumeCurrentPositionIs(...)`

Use this when the robot is physically placed at a known pose before init:

```java
.nonPeriodic()
    .bounded(0.0, 4200.0)
    .nativeUnits()
    .assumeCurrentPositionIs(0.0)
```

The supplied plant position must be finite and is rejected immediately rather than clamped. While
the reference is pending, Sushi samples native feedback and uses the first finite reading as plant
position `0.0`; a non-finite reading leaves the Plant unreferenced. This is convenient, but it is
only correct if the mechanism really starts there.

### `needsReference(...)`

Use this when the mechanism must be homed/indexed before position targets are meaningful:

```java
.nonPeriodic()
    .bounded(0.0, 4200.0)
    .nativeUnits()
    .needsReference("lift not homed")
```

Before the reference is established, `PositionPlant.targetRange()` returns an invalid range with
that reason. `PlantTargets.equivalentPositionsOf(...)` and `PlantTargets.plan(request)` resolvers see that
invalid range during `plant.update(clock)` and use their explicit `whenUnavailable()` policy instead
of producing unsafe requested targets.

Use `PositionCalibrationTasks` to establish the reference:

```java
Task findBottomReference = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .failAfterSec(3.0)
        .build();
```

`.withPower(...)` accepts only a finite normalized command in the inclusive `[-1.0, +1.0]` range.
It rejects `NaN`, infinities, and finite overshoot at the builder step. The direct
`PositionPlant.beginCalibrationSearch(...)` seam enforces the same contract before changing
search state or stopping the normal position output. Neither path clamps an invalid search command;
low-level FTC adapter clamping remains boundary defense only.

For a grouped device-managed motor-position Plant, the search output sends that shared normalized
command unchanged to every child's logical raw-power port. Each configured `Direction` still owns
physical opposition, while the group's normal position scale and native-tick bias continue to apply
only to position targets and inverse feedback.

The reference supplied to `establishReferenceAt(...)` must also be finite in plant units. The Task
recipe rejects `NaN` or infinity at that answer before starting or changing its search lifecycle;
the direct `PositionPlant` seam repeats the check before sampling feedback or changing reference state.
This coordinate anchor is not required to lie inside `targetRange()` and is never clamped into it.

This numeric contract is structural, not robot-specific safety approval. The mechanism owner must
still select a safe magnitude and direction, verify the cue and timeout/cancellation paths, provide
any required external interlock or physical safeguard, and confirm that the mechanism is free to
move. An active raw-power search bypasses the normal target resolver and inline `targetGuards()`
chain; those normal-target protections do not make the temporary search safe.

Starting this Task acquires a temporary search after requesting an immediate stop of the prior
normal position output, but it only stages the configured search power. The Task never calls
`plant.update(clock)` or writes that power itself. In the documented Tasks-before-Plants loop, the
owning mechanism's one downstream Plant update submits the staged raw/open-loop command while the
search remains active. If the cue is already true, the Task establishes the reference and releases
the search before that Plant phase, so no search-power command is submitted.

Success, timeout, and active cancellation all preserve the existing persistent command and final
resolver. Every release clears search ownership and requests an immediate output stop, so a later
update cannot refresh search power even if that stop throws. For a normally returning FTC
motor-power stop, zero remains in raw/open-loop mode; the next device-managed position command
reasserts `RUN_TO_POSITION`. Calibration establishes the Plant reference and never resets the
encoder as a hidden side effect.

Post-reference policy belongs to the mechanism that owns the request. For example, a named-height
lift can put the search first in `Tasks.sequence(...)`, then call its ordinary
`setHeight(STOWED)` setter in a `Tasks.runOnce(...)` continuation. Exact success starts that setter
before the same downstream Plant phase; timeout and cancellation skip it and retain the prior or
latest coherent semantic/numeric request. Do not preselect STOWED merely to enter search: temporary
search ownership already suspends normal target realization.

The timeout policy is explicit: use `failAfterSec(...)` for a bounded search or `neverTimeout()` only when another safety path is guaranteed to cancel the task.

For periodic mechanisms, the Task's clocked `establishReferenceAt(...)` samples native feedback on
the cue cycle and preserves the nearest equivalent unwrapped position from that current sample, not
from the prior Plant update's cached measurement. If a tray has period `360.0` degrees and the
current estimate is `1081.5`, an index mark for reference `0.0` corrects the estimate to `1080.0`,
not all the way back to zero. When the current plant estimate is finite, a non-finite final
nearest-equivalent result fails without committing the new reference. The same atomic rule covers
the complete candidate affine map: every bounded endpoint image and derived public measurement
must remain finite before the new reference becomes visible. A later unbounded core
Plant-to-native conversion is checked individually before applied-target commit or output.

These checks establish a software numeric contract only. They do not prove the mechanism is at the
declared physical pose, that plant/native scale and sign are correct, that the cue is repeatable, or
that the selected reference and hold are safe under load. Those remain adopting-robot checks.

---

## 8. Standard servo position Plants

An FTC standard `Servo` proves only a command-only native position channel with raw range
`[0.0, 1.0]`. It does not prove whether the mechanism coordinate is periodic: a limited wrist is
normally non-periodic, while a plate or indexer may be periodic when positions one full turn apart
are interchangeable. The mechanism therefore answers periodicity explicitly just like every other
position branch. Standard Servo construction does not expose motor control strategy, feedback
tolerance, reference policy, open-loop calibration search, or unbounded travel.

The `[0.0, 1.0]` native domain is the FTC SDK logical command range; it is not a promise of 180
degrees or of any specific physical span. The configured FTC servo type/controller maps it to PWM,
and servo programming and mechanics determine the resulting travel. A team may configure and
program a nominal 270-degree response and then map only the mechanically safe 180-degree subrange
into Plant degrees. Changing the FTC servo type/PWM configuration, `Direction`, servo programming,
horn geometry, or linkage invalidates previously measured physical endpoint evidence.

Normalized two-state mechanism coordinate mapped to reviewed native endpoints:

```java
this.wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .nonPeriodic()
        .bounded(0.0, 1.0)
        .rangeMapsToNative(backedOffNativeClosed, backedOffNativeOpen)
        .targetFromNewCommand(0.0)
        .build();
```

Physical/logical units mapped to reviewed native endpoints:

```java
this.wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .nonPeriodic()
        .bounded(-45.0, 90.0)
        .rangeMapsToNative(backedOffNativeMinus45Deg, backedOffNativePlus90Deg)
        .targetFromNewCommand(0.0)
        .build();
```

In the first example, robot code commands `0.0` for closed and `1.0` for open. Those are normalized
mechanism coordinates, not aliases for the servo's full native domain. `rangeMapsToNative(...)`
maps them to the mechanism's separately reviewed, backed-off FTC commands; reversed endpoints are
valid. A target of `0.5` is halfway through that configured command interval, not evidence that the
shaft or linkage moved halfway and not servo feedback.

In the second example, robot code commands degrees while the servo still receives raw fractions.
In both cases, `bounded(...)` names the public Plant coordinate and
`rangeMapsToNative(...)` maps that range's endpoints to raw servo fractions. Neither mapping
promises a linear physical linkage. A mechanism that deliberately exposes raw native units must
instead state its reviewed safe subrange explicitly with
`.position().nonPeriodic().bounded(safeNativeMin, safeNativeMax).nativeUnits()`; the SDK's full
`[0.0, 1.0]` envelope is not itself that safety review.

Before hardware lookup, the completed Servo recipe maps both Plant bounds through the shared map
and every configured child transform. Every final raw endpoint must be finite and inside inclusive
`[0.0, 1.0]`. Thus `.bounded(-1.0, 1.0).nativeUnits()` and
`.rangeMapsToNative(-0.01, 1.01)` are configuration errors, while reversed endpoints and a valid
mirror such as child scale `-1.0`, bias `1.0` remain legal. Teams must review every endpoint after
choosing the servo type, direction, horn, linkage, and intended load. This software-domain proof
says nothing about horn geometry, linkage linearity, collision, feedback, or safe physical travel.

---

## 9. Regulated CR-servo position Plants

CR servos do not have a device-managed position mode. Position control requires native feedback and
a standard or explicitly custom control model:

```java
this.turret = FtcActuators.plant(hardwareMap)
        .crServo("turretServo", Direction.FORWARD)
        .position()
        .regulated()
            .externalEncoder("turretEncoder")
        .periodic(TURRET_TICKS_PER_TURN)
            .bounded(-900.0, 1100.0)
            .nativeUnits()
            .needsReference("turret not homed")
        .positionTolerance(8.0)
        .setpointFromTrapezoidalProfile(MAX_TICKS_PER_SEC, MAX_TICKS_PER_SEC2)
        .feedbackFromPid(0.01, 0.0, 0.0005)
        .outputPowerLimitedTo(0.45)
        .targetFromNewCommand(0.0)
        .build();
```

A CR-servo position Plant can participate in `PositionCalibrationTasks.search(...)` because its one
mechanism-owned Plant update can submit temporary open-loop power while looking for a reference.
The Task manages the search lifecycle without becoming another Plant writer.

The hardware-neutral gateway expresses the same rotating-plate design when a custom adapter already
owns the CR-servo power channel and continuous position feedback:

```java
PositionPlant plate = Plants.fromOutputs()
        .regulatedPosition(crServoPowerOut, unwrappedAngle)
        .periodic(2.0 * Math.PI)
        .bounded(-4.0 * Math.PI, 4.0 * Math.PI)
        .nativeUnits()
        .needsReference("plate not indexed")
        .positionTolerance(Math.toRadians(2.0))
        .setpointFromAppliedTarget()
        .feedbackFromPid(kP, kI, kD)
        .outputPowerLimitedTo(0.45)
        .targetFromResolver(plateFinalTarget)
        .build();
```

Here periodicity belongs to the mechanism coordinate, not to the `CRServo` hardware. The supplied
measurement is explicitly continuous/unwrapped, and `plateFinalTarget` opts into full-turn
equivalents only if it composes `PlantTargets.equivalentPositionsOf(...)`; an exact target still
means one literal unwrapped position.

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
`0.0`. A newly constructed Plant/source lifetime starts with a fresh baseline. An advanced owner of
a directly supplied feedback source may reset that source while its active mechanism is safely idle,
but a terminally stopped Plant is never restarted. Repeated samples in one
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

Sushi separates **commands** from **measurements**.

### Command channels

`FtcHardware` creates command-only HAL outputs:

* `PowerOutput`
* `PositionOutput`
* `PowerLimitedPositionOutput`
* `VelocityOutput`

These answer “what command should we send?” — not “what did the mechanism measure?” The interfaces
remain hardware-neutral; FTC run-mode ownership belongs to the concrete motor adapters described
above. `PowerLimitedPositionOutput` is the narrow evidence-bearing subtype for one paired logical
native-position plus normalized maximum-output-power command. Its inherited one-argument
`setPosition(...)` means magnitude `1.0`; `getCommandedMaximumOutputPowerMagnitude()` reports the
cached submitted limit, not applied power or feedback. The pair is prevalidated before effects, but
the interface does not claim transactional rollback across sequential device writes.
Implementations must provide an explicit natural `stop()`.
`FtcHardware.motorPosition(...)` returns this capability; `FtcHardware.servoPosition(...)` remains
the smaller `PositionOutput`.

### Measurement sources

Use `FtcSensors` for raw measurement sources:

```java
ScalarSource ticks = FtcSensors.motorPositionTicks(hardwareMap, "armMotor");
ScalarSource vel = FtcSensors.motorVelocityTicksPerSec(hardwareMap, "flywheel");
ScalarSource currentAmps = FtcSensors.motorCurrentAmps(hardwareMap, "intakeMotor");
```

Motor current remains a raw, shareable measurement source rather than Plant/actuator readback or
current-response policy. See [`FTC Sensors`](<FTC Sensors.md#motor-current>) for its sampling,
returned-value, reset, and adopting-hardware boundaries.

### Plant snapshots

Every Plant can freeze its common cached public facts in one immutable `PlantSnapshot`:

```java
plant.update(clock);
PlantSnapshot status = plant.snapshot();

telemetry.addData("target", status.requestedTarget());
telemetry.addData("appliedTarget", status.appliedTarget());
telemetry.addData("measurement", status.measurement());
telemetry.addData("requestedError", status.requestedTargetError());
telemetry.addData("atTarget", status.atTarget());
```

The snapshot also contains target resolution/status, feedback and measurement availability, applied
error, the captured live numeric command when one exists, and provenance-aware
`atCommandTarget()`. It performs no Plant update or hardware poll: it captures the current live
command value and the cached facts from the most recent heartbeat. An older snapshot never changes,
and the API does not claim atomic publication, safe capture from inside an update callback, or
cross-thread synchronization.

`PositionPlant.snapshot()` returns the covariant `PositionPlantSnapshot`, adding periodicity,
period, legal range, reference state/status, and calibration-search support. Requested, applied,
measured, and error values remain in Plant units. `PositionPlant.positionSource()` is also in Plant
units. Context-aware target resolvers such as
`PlantTargets.equivalentPositionsOf(...)` and `PlantTargets.plan(request)` receive the Plant measurement,
range, and periodicity automatically through the Plant target context during `update(clock)`.

For named requests, the mechanism composes rather than copies those facts internally:

```java
SemanticScalarSnapshot<Height, PositionPlantSnapshot> status =
        heightCommand.snapshot(lift.snapshot());

telemetry.addData("height", status.request().semantic());
telemetry.addData("heightIn", status.request().commandTarget());
telemetry.addData("measuredIn", status.plant().measurement());
telemetry.addData("atCurrentHeight", status.currentRequestAtTarget());
```

This distinguishes a newly issued request from same-valued evidence cached for an older request.
That nested value is framework/mechanism backing and an advanced diagnostic, not the ordinary
robot-facing lift API. A named capability projects it into its own vocabulary:

```java
BasicLift.Status status = lift.status();
telemetry.addData("height", status.requestedHeight());
telemetry.addData("heightIn", status.requestedPositionIn());
telemetry.addData("appliedIn", status.appliedPositionIn());
telemetry.addData("measuredIn", status.measuredPositionIn());
telemetry.addData("atCurrentHeight", status.atTarget());
```

The enum is the semantic name. `setHeight(height)` replaces the persistent request and returns
without waiting; `moveTo(height)` builds a fresh single-use feedback Task that requests the same
height when it starts and waits for truthful arrival. Numeric coordinates stay in configuration and
the mechanism's semantic-to-numeric mapping. Internally, `SemanticScalarTasks` tracks exact request
identity and Plant evidence without exposing either through capability status. Do not add a second
`NamedPosition` type or parallel `toHigh` aliases for the same decision.

The same pattern applies to named launcher speeds: a semantic speed enum maps forward to a velocity
command, while a fresh move/spin-up Task waits on feedback. Power and velocity Plants use the same
`PlantSnapshot`; velocity adds no different generic status facts and therefore no separate snapshot
subtype. For a grouped shooter, generic `PlantSnapshot.atTarget()` remains the grouped Plant's
aggregate contract. Per-wheel balance and readiness remain explicit capability-status facts.

A capability that combines those facts may retain one immutable publication instead of rebuilding
it on read.
[`ReferenceFlywheels.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.Status.html>)
owns one grouped Plant snapshot plus captured per-wheel measurements and readiness, and publishes
the complete value only after a successful mechanism update. Its requested/selected/applied
velocity methods keep ordinary robot code in capability vocabulary; `plantSnapshot()` is reserved
for advanced diagnostics. `ReferenceLauncher.Status` composes that immutable value as
`flywheels()` beside object and transfer facts rather than mirroring the velocity fields.

---

## 12. Position tolerances and FTC motor tuning

For device-managed motor position control there are two different tolerance concepts. They belong
to different owners and neither can be derived safely from the other.

### `positionTolerance(...)`

This is the **plant-level** completion band used by `Plant.atTarget()` and literal physical
`Plant.atTarget(value)`. A feedback-aware
`ScalarTasks.set(command, value).untilReachedBy(plant)` may correlate a logical periodic command
with a different selected physical equivalent, but the Plant query itself never compares modulo a
period. A `SemanticScalarTasks.set(command, request).untilReachedBy(plant)` branch uses the same
physical completion evidence while also requiring its exact named request to remain selected.

* required exactly once after position mapping and reference policy
* units: plant position units
* examples: ticks for `nativeUnits()`, inches for `scaleToNative(TICKS_PER_INCH)`, degrees for a degree-based arm
* defines “close enough for robot logic” for the complete mechanism; Sushi supplies no hidden or
  native-unit default

### `devicePositionToleranceTicks(...)`

This is an **optional FTC motor-controller override** for the motor's own target-position tolerance.

* default in Sushi: unchanged unless you call it
* units: native encoder ticks
* use this only when you intentionally need the advanced
  `DcMotorEx.setTargetPositionTolerance(int)` override
* it does not define or replace the required plant-unit `positionTolerance(...)`

Sushi evaluates Plant completion from its requested target, applied target, converted
measurement, target status, and required Plant tolerance. It does not use `DcMotor.isBusy()` for
that decision. The public FTC SDK contract also does not establish that the configured device
tolerance is the controller's complete correction deadband, so do not infer that the motor stops
correcting at that boundary. Ordinary robot code should normally use
`deviceManaged()` and choose only the required Plant tolerance; enter
`deviceManagedWithOverrides()...doneOverrides()` when an FTC controller override is deliberately
needed.

### Controller-configuration domains

FTC device-managed P, I, D, and F coefficients—including `outerPositionP(...)`, every component of
`innerVelocityPidf(...)`, and every component of velocity `velocityPidf(...)`—must be finite and
inside this inclusive symmetric domain:

```text
[-Integer.MAX_VALUE / 65536.0, +Integer.MAX_VALUE / 65536.0]
```

That is approximately `[-32767.99998474121, +32767.99998474121]`. It is the no-saturation domain of
the pinned FTC SDK 11.1 REV **public coefficient conversion**, not the complete raw wire-field range
and not a recommendation that every representable gain is useful or safe. Sushi does not infer a
nonnegative-gain rule: negative values and both signed zeros remain valid. The controller still
applies its ordinary 1/65536 coefficient quantization.

The other explicit position settings have their own domains:

* `devicePositionToleranceTicks(...)` must be inside the pinned FTC/REV native unsigned 16-bit
  target-command range `[0, 65535]`; and
* post-tolerance `outputPowerLimitedTo(maximumMagnitude)` must be finite and inside `[0.0, 1.0]`.
  Both signed zeros are accepted. Negative values, overshoot, `NaN`, and infinities are rejected
  rather than saturated.

Every tuning answer is validated before it changes the recipe. A complete PIDF tuple is accepted
all-or-nothing, a rejected first answer can be corrected through the same retained stage, and a
second answer to the same knob is rejected without replacing the first. Position
`doneOverrides()` requires at least one accepted setting, closes the multi-setting section, and
prevents repeated close or later setters. Sushi revalidates the complete branch before hardware
lookup or configuration so a retained-stage cast cannot bypass the staged grammar.

The ordinary path remains deliberately smaller. `deviceManaged()` calls no optional controller P,
PIDF, or target-tolerance setter. Both ordinary and overridden position paths use maximum magnitude
`1.0` unless the post-tolerance recipe answers `outputPowerLimitedTo(...)`. FTC device-managed
velocity and a prebuilt plain `PositionOutput` expose no corresponding effort answer because those
boundaries provide no truthful normalized-power capability.

## 13. Velocity bounds, mapping, and tuning

Motor velocity uses the same guided-builder rule as position: required conceptual questions are
explicit, including one `velocityTolerance(...)` answer after public-unit mapping. Optional
controller tuning appears only after entering a tuning branch.

```java
this.shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManaged()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetFromNewCommand(0.0)
        .build();
```

If you need FTC motor velocity PIDF coefficients, deliberately enter the device-managed tuning
branch:

```java
this.shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithOverrides()
            .velocityPidf(kP, kI, kD, kF)
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetFromNewCommand(0.0)
        .build();
```

The velocity override branch intentionally exposes only the complete `velocityPidf(...)` answer,
so that answer advances directly to bounds; there is no administrative `doneOverrides()` step and
no empty override spelling of the ordinary path. The separate FTC position-loop gain
(`outerPositionP(...)`) belongs to device-managed motor **position** mode, not pure velocity mode.
This method writes FTC device-controller coefficients under the validation contract above; it is
distinct from Sushi's standard software-control grammar.

### Framework-owned live FTC control workflow

The Plant builder remains the ordinary production configuration path. The ready-made
`FtcPanelsTuners.velocityControl(...)` and `positionControl(...)` workflows own a fresh Plant
supplied by robot code and derive a configuration-only handle from that exact completed Plant:

```java
FtcMotorVelocityControl velocity =
        FtcMotorControllers.velocityControl(flywheelPlant);

FtcMotorPositionControl position =
        FtcMotorControllers.positionControl(armPlant);
```

These handles are advanced framework-integration seams, not another Plant builder or the ordinary
robot tuning API. They expose no target, power, direction, or run-mode command. A permanent claim
binds one handle to the completed Plant, so a workflow cannot command one Plant while changing a
second named motor. Robot code should normally call `FtcPanelsTuners`; that workflow already owns
coherent draft capture, segment history, response metrics, failure handling, and cleanup.

Velocity accepts one immutable complete PIDF candidate. A single-motor handle may apply it while
running. A grouped handle uses the exact ordered motor group already owned by the Plant and applies
one logical candidate to every member. Because FTC SDK writes are sequential, a grouped gain change
requires same-cycle cold-zero evidence for every member. Each member retains its own initial and
accepted configuration, native target, native measurement, native error, and mapped tolerance;
Sushi never substitutes an average as proof that every child agreed.

Position is limited to one FTC motor. Its complete candidate contains the outer
`RUN_TO_POSITION` P and the inner `RUN_USING_ENCODER` P/I/D/F tuple. Output-power limit and target
tolerance remain immutable Plant policy. The position handle also forwards the exact same Plant's
coordinate-reference and pre-output hold-preparation capability so a tuning workflow never homes
one object and controls another.

Both handles capture exact initial configuration and algorithms. A setter/readback failure may
leave hardware partially changed; it latches terminal uncertainty. The workflow stops the Plant
and best-effort restores every captured field/member without claiming transactional rollback.
Follow the [`control tuning workflow`](<../testing-calibration/Control Tuning Workflow.md>) for target
range semantics, group cold boundaries, position reference/hold behavior, evidence, and adoption.

Use `regulated()` when Sushi should own the velocity loop and command raw motor power. A flywheel
can use an acceleration-limited setpoint, typed velocity/acceleration feedforward, voltage
compensation, and a forward-only final power range without assembling peer controller objects:

```java
static final double TICKS_PER_FLYWHEEL_REV = 28.0;
static final double TICKS_PER_RPM = TICKS_PER_FLYWHEEL_REV / 60.0;

ScalarSource batteryVoltage = FtcSensors.batteryVoltage(hardwareMap);

this.shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .regulated()
            .internalEncoder()
        .bounded(0.0, 5000.0)          // Plant units: RPM
        .scaleToNative(TICKS_PER_RPM)  // native units: FTC ticks/sec
        .velocityTolerance(75.0)       // Plant units: RPM
        .setpointFromAccelerationLimitedProfile(maxRpmPerSec)
        .feedbackFromPid(kP, kI, kD)
        .feedbackIntegralLimitedTo(-0.15, 0.15)
        .feedforwardFromMotion(kS, kV, kA)
        .voltageCompensationFrom(batteryVoltage, 13.0, 9.0, 1.4)
        .outputPowerLimitedTo(0.0, maximumFlywheelPower)
        .targetFromNewCommand(0.0)
        .build();
```

Here the PID error, setpoint velocity, and setpoint acceleration are in Plant units: RPM and
RPM/second. The builder converts native feedback into RPM before control. The final power policy is
outside feedforward and voltage compensation, so it covers the complete normalized command. Use
`setpointFromAppliedTarget()` instead when no acceleration limiting is desired; that direct branch
offers `kS`/`kV` motion feedforward but cannot expose `kA` because it has no acceleration evidence.

For nonlinear or table-driven control, pass one complete custom regulator through
`controlFromCustomRegulator(...)`. That is the explicitly advanced compatibility seam, not a second
ordinary PID/feedforward recipe.

If robot code wants nicer plant velocity units, keep the controller native units explicit:

```java
this.shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManaged()
        .bounded(0.0, 5000.0)          // plant units: RPM
        .scaleToNative(TICKS_PER_RPM)  // native units: FTC ticks/sec
        .velocityTolerance(75.0)       // plant units: RPM
        .targetFromNewCommand(0.0)
        .build();
```

Velocity mapping is deliberately zero-preserving. Sushi exposes `nativeUnits()` and
`scaleToNative(...)`, but not `rangeMapsToNative(...)`, because a velocity target of `0.0` should
always mean stop. Semantic mappings like “driver command 0..1 maps to useful shooter speeds” belong
above the Plant in a robot service, table, or request source.

That also means `bounded(min, max)` still uses **plant velocity units**, even when the next step is
`scaleToNative(...)`. The scale changes how Sushi converts plant velocity to native velocity, but
it does not create a velocity offset or separate reference question. In every supported velocity
mapping, plant `0.0` means stationary.

Velocity bounds should represent hardware-safe target bounds, not scoring semantics. For example,
if a shooter has a minimum useful scoring speed of 700 ticks/sec, the Plant range should usually
still start at `0.0` so setting the flywheel command target to `0.0` stops the flywheel. Keep the 700 value in shooter selection
logic, not in the Plant's legal target range.

---

## 14. Grouped actuator child mappings

The staged hardware step has one child-transform grammar. The first actuator establishes the shared
group coordinate. After each `andMotor(...)` or `andServo(...)`, `scale(...)` and a supported
`bias(...)` apply to that most recently added child; `andCrServo(...)` exposes scale only. After the
Plant's shared mapping, normal target realization computes:

```text
childNative = childScale * sharedNative + childBias
```

Scale is a dimensionless static ratio. Bias is an additive native-unit position alignment; it is
not a general “make this motor run a little faster” knob. Every supplied scale or retained bias must
be finite. The later target/control branch applies the narrower rule it owns:

| Group branch | Supported child transform |
| --- | --- |
| Motor or CR-servo direct power | finite scale, zero bias, complete `[-1.0, +1.0]` image |
| Device-managed motor velocity | finite nonzero scale, zero bias, finite command; no invented speed ceiling |
| Device-managed motor position | finite nonzero scale and finite bias; every realized rounded tick must fit `int` |
| Standard-Servo position | finite scale and bias with complete raw image inside `[0.0, 1.0]` |
| Grouped framework-regulated motor/CR-servo path | exact identity scale and zero bias |

Device-managed motor-position calibration search is not another position target. While search is
active, Sushi bypasses the position scale/bias transform and submits one validated normalized
power command identically to each motor through its configured `Direction`. This preserves both
position alignment during normal targeting and exact zero-as-stop during search.

For every fully known bounded recipe, the shared map and every child endpoint are preflighted before
hardware resolution. A runtime-dependent reference gets core candidate-map validation when it is
established and a final FTC-domain check before each realized write. During operation, Sushi
computes and validates every child command into temporaries before writing the first child. That
prevents a predictable later-child mapping error from moving an earlier child, but it does not make
sequential SDK writes atomic if an actual device call fails. This preflight guarantee covers the
requested command itself. If rejection propagates through a Plant update, the Plant then invokes
the group's natural stop as separate nonterminal fail-safe cleanup, so zero-power/zero-velocity or
position-stop writes may follow the rejection.

### Opposed flywheels with a fixed speed ratio

Use `Direction` for opposed physical mounting and a positive child scale for a proven proportional
speed ratio:

```java
this.flywheels = FtcActuators.plant(hardwareMap)
        .motor("leftFlywheel", Direction.FORWARD)
        .andMotor("rightFlywheel", Direction.REVERSE) // opposed mounting
        .scale(0.96)                                   // positive speed ratio
        .velocity()
        .deviceManaged()
        .bounded(0.0, MAX_VELOCITY_NATIVE)
        .nativeUnits()
        .velocityTolerance(VELOCITY_TOLERANCE_NATIVE)
        .targetFromNewCommand(0.0)
        .build();
```

A shared target of `2500.0` commands the reference child to `2500.0` and the scaled child to
`2400.0`. An active target of exact zero passes through the scale and submits zero to both children.
Terminal lifecycle `stop()` invokes each child's natural stop directly, leaves the command unchanged,
and prevents the Plant from realizing that command or any later target. The active-zero and
lifecycle paths agree on zero actuation, but they need not have the same FTC mode acquisition or
lifecycle sequence; for example, an active command may assert its device-managed mode while
lifecycle stop need not reacquire it.

Grouped device-managed feedback inverse-maps each child's sample into shared group units and reports
their overflow-safe arithmetic mean. Consequently, grouped `atTarget()` compares that aggregate
with the one shared target; it is not proof that every motor is independently inside tolerance.
Opposing child errors can cancel in the aggregate. `plant.snapshot()` preserves exactly that same
aggregate measurement and arrival contract; it does not manufacture per-member evidence.

Requiring readiness to prove each wheel independently does not, by itself, create another command
degree of freedom. When every wheel still follows one shared group-unit target through fixed child
scales, keep the grouped Plant. Compose its `PlantSnapshot` with separately sampled per-wheel
measurements and per-wheel readiness facts in one capability-owned Status, published all-or-nothing
after a successful update. The
[`ReferenceFlywheelMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelMechanism.html>)
demonstrates that policy around one grouped flywheel command. Active readiness keeps the later
per-wheel sampling timing and requires a positive captured command value, both finite per-wheel
tolerance facts, and an active Plant intent that requested and applied that same value without
fallback or modification. It does not replace those per-wheel facts with the group's earlier
aggregate arrival sample. A successful
terminal stop publishes a post-stop flywheel Status with readiness false; older Status values
remain immutable historical captures. The launcher separately publishes transfer state.

If robot testing instead proves that the wheels need independently commanded targets—for example,
a live additive or nonlinear trajectory trim—then the mechanism has two commanded degrees of
freedom rather than one scalar group target. Keep two private one-motor Plants inside the shooter
subsystem and expose one semantic paired command:

```java
void setFlywheelVelocity(double baseVelocityNative,
                         double trajectoryTrimVelocityNative) {
    double left = baseVelocityNative == 0.0
            ? 0.0 : baseVelocityNative + trajectoryTrimVelocityNative;
    double right = baseVelocityNative == 0.0
            ? 0.0 : baseVelocityNative - trajectoryTrimVelocityNative;

    requireValidFlywheelPair(left, right); // validate both before changing either command
    leftFlywheel.commandTarget().set(left);
    rightFlywheel.commandTarget().set(right);
}

boolean flywheelsAtTarget() {
    return leftFlywheel.atTarget() && rightFlywheel.atTarget();
}
```

The subsystem constructs both Plants through the same `FtcActuators` grammar, owns their update and
stop order, and does not expose them as peer dependencies to controls. Do not hide a live trim
source inside a custom scalar output: that would give one scalar Plant a second unreported command.

For a genuinely custom grouped **regulated** actuator with one truthful scalar target, an advanced
adapter may still enter the neutral grammar through
`Plants.fromOutputs().regulatedPosition(groupOutput, feedback)` or
`Plants.fromOutputs().regulatedVelocity(groupOutput, feedback)`, then answer the ordinary typed
control stages or the advanced `controlFromCustomRegulator(...)` stage after tolerance. That adapter
owns its complete group lifecycle and failure contract. Its snapshot reports the one scalar
`feedback` source supplied to that regulated Plant; unlike the standard device-managed group, the
framework does not synthesize an inverse-mapped per-member mean for this custom boundary.
Independently combining public
`FtcHardware.motorPower(...)` outputs does not receive the standard builder's all-child preflight,
so sequential child writes are not an equivalent safe construction path.

### Mapping failures and retry

An invalid numeric scale/bias answer is rejected before mutation, so a retained builder stage may
retry. An incompatible branch, such as a nonzero motor bias followed by `velocity()`, fails before
fresh hardware effects once the complete recipe is known. Diagnostics identify the operation,
actuator family and child, relevant endpoint or runtime value, transform, computed result, and
required domain. Valid negative position scales, reversed endpoint maps, signed zero, exact domain
boundaries, and later valid retry remain supported; configuration is never silently clamped.
