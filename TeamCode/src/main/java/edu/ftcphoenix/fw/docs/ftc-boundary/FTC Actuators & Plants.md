# FTC Actuators & Plants

This page covers the FTC boundary for Phoenix mechanism wiring:

* `FtcActuators.plant(...)` and `Plants.fromOutputs()` — two gateways into one staged grammar
* `FtcHardware` — low-level command channels
* `FtcSensors` — low-level measurement sources
* device-managed vs regulated control
* position plant geometry, plant/native unit mapping, and reference policy
* how tolerances and FTC motor tuning interact

The FTC boundary is where FTC SDK hardware names become Phoenix `Plant` objects. In ordinary robot
code, the composition root passes `HardwareMap` and a validated, data-only profile slice to a
mechanism constructor. That mechanism uses the builders on this page, keeps the resulting Plants
private, and owns their update and stop lifecycle. Robot services, Tasks, and scalar planners use
the mechanism's semantic API instead of touching either raw SDK devices or raw Plants directly.

The compact builder snippets below are therefore **mechanism-constructor excerpts** unless a
section explicitly labels a custom hardware adapter or other advanced boundary. Sections that
compare low-level builder alternatives may declare a short local solely to keep the API difference
visible; production mechanism code assigns the selected build to a private field, as the complete
examples do. The compiling
[`StarterIntakeMechanism`](<../../../robots/examples/starter/StarterIntakeMechanism.java>) shows the
ordinary private-Plant owner; the flat
[`TeleOp_03_ShooterMacro`](<../../tools/examples/TeleOp_03_ShooterMacro.java>) shows the same
ownership inside a complete teaching OpMode.

---

## 1. Two boundary gateways, one Plant grammar

Use `edu.ftcphoenix.fw.ftc.FtcActuators` when a mechanism starts from FTC `HardwareMap`, configured
device names, and FTC controller choices:

```java
// Inside the mechanism constructor; flywheel and pusher are private fields.
this.flywheel = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetFromNewCommand(0.0)
        .build();

this.pusher = FtcActuators.plant(hardwareMap)
        .servo("pusher", Direction.FORWARD)
        .position()
        .nonPeriodic()
        .bounded(0.0, 1.0)
        .nativeUnits()
        .targetFromNewCommand(0.0)
        .build();
```

Use `Plants.fromOutputs()` only when a custom adapter, hardware-neutral test, or portable host
already owns Phoenix output ports, feedback sources, and any regulator:

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
3. Who manages the position loop? `deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated()` followed by one direct feedback answer and `regulator(...)`
4. Is the public coordinate `nonPeriodic()` or `periodic(period)`?
5. What bounds in public Plant units are legal? `bounded(min, max)` with two finite endpoints, or
   `unbounded()`
6. How do Plant units map to native units? `nativeUnits()`, `scaleToNative(...)`, or bounded-only `rangeMapsToNative(...)`
7. How is the reference/offset known? `alreadyReferenced()`, `plantPositionMapsToNative(...)`, `assumeCurrentPositionIs(...)`, or `needsReference(...)`
8. What public position error counts as complete? The required `positionTolerance(...)` answer
9. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
10. Target binding: `targetFromNewCommand(initialValue)` for an ordinary exact command or
    `targetFromResolver(finalResolver)` for a caller-supplied final resolver, then `build()`

Motor velocity wiring asks a parallel but smaller set of questions:

1. Which hardware? `motor(...)`
2. Which target domain? `velocity()`
3. Who manages the velocity loop? `deviceManagedWithDefaults()`, the one-answer
   `deviceManaged().velocityPidf(...)` tuning branch, or `regulated()` followed by one direct
   feedback answer and `regulator(...)`
4. What target bounds in Plant units are legal? `bounded(min, max)` with two finite endpoints, or
   `unbounded()`
5. How do Plant velocity units map to native velocity units? `nativeUnits()` or `scaleToNative(...)`
6. What public velocity error counts as complete? The required `velocityTolerance(...)` answer
7. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
8. Target binding: `targetFromNewCommand(initialValue)` or `targetFromResolver(finalResolver)`, then `build()`

The neutral gateway exposes the same control choices without FTC acquisition: `power(...)`,
`commandedPosition(...)`, `deviceManagedPosition(...)`, `deviceManagedVelocity(...)`,
`regulatedPosition(...)`, and `regulatedVelocity(...)`. Each branch asks only the later facts its
inputs cannot prove. The tolerance question appears exactly once and only after the public
coordinate is known. Command-only position skips feedback-only reference, calibration-search, and
tolerance questions.

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

Velocity and power Plants stay simpler than position Plants because they do not have position
periodicity or homing/reference questions. Power is simpler still: every direct power Plant has the
fixed normalized range `[-1.0, +1.0]`, so there is no redundant bounds step. A finite request outside
that range clamps during normal targeting.

### Grouped hardware-name identity

The student-facing calls stay unchanged, but grouped construction validates the configured names
before it touches new group hardware. The FTC SDK trims surrounding whitespace before a
case-sensitive lookup, so Phoenix uses the same effective identity:

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
* a motor-velocity command must be finite, but Phoenix does not invent a controller-independent
  maximum ticks/second;
* a motor-position command must be finite, round once to a `long`, fit inside the SDK's signed
  integer target-position domain, and only then narrow to `int`; and
* framework-derived motor/CR-servo power children must be finite and inside normalized
  `[-1.0, +1.0]` before group fan-out.

These checks happen before the cache, mode, target, or power effects owned by the corresponding raw
adapter. The standard Servo adapter still clamps a finite direct expert command into `[0.0, 1.0]`,
and the raw power adapters retain their finite saturation defense. Those clamps are last-resort
boundary protection, not a way for an accepted Plant recipe to hide an invalid map. Raw
`PowerOutput` write/cache/cleanup behavior after an adapter failure is a separate contract.

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
        .deviceManagedWithDefaults()
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
sample initializes directly to the first guarded candidate, and `stop()`/`reset()` clear dynamic guard
state. If loop time is temporarily non-finite, it holds the last output until a finite time baseline
has been restored. If a mechanism needs startup limiting from a known physical position, initialize
the command target from that measurement or use an appropriate reference policy before requesting
a far-away target.

---

## 3. Plant units vs native units

For position and velocity Plants, Phoenix distinguishes two coordinate systems:

* **Plant units** are the public units used by robot code, `ScalarTarget` requests, resolved Plant
  targets, scalar planner requests, target ranges, position periods, reference values, and plant-level tolerances.
* **Native units** are the units used by the selected hardware/control path: servo raw fraction,
  motor encoder ticks, motor ticks/sec, external encoder units, or a caller-supplied feedback source.

Public position and velocity APIs use **plant units** unless the method name explicitly says
**Native**.

As a rule of thumb:

* `bounded(...)`, `unbounded()`, `periodic(...)`, `targetFromNewCommand(...)`, `targetFromResolver(...)`, `getRequestedTarget()`,
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

After `motor(...).position()`, Phoenix asks who manages the position loop.

### Device-managed with defaults

Use this when FTC `RUN_TO_POSITION` is good enough and you do not need controller-specific tuning:

```java
this.lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .nonPeriodic()
        .bounded(0.0, 4200.0)
        .nativeUnits()
        .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetFromNewCommand(0.0)
        .build();
```

The public lift coordinate is in plant units. In this example, plant units are also native encoder
ticks because the builder chose `nativeUnits()`.

### Device-managed with FTC tuning

Enter `deviceManaged()` only when you want optional FTC motor-controller tuning. While in this
branch, autocomplete shows only device-managed tuning knobs. `doneDeviceManaged()` returns to the
main position questions. The branch must contain at least one accepted override, each knob may be
answered only once, and closing it prevents later changes even through a retained tuning-stage
reference. Use `deviceManagedWithDefaults()` instead of opening an empty tuning section.

```java
this.lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .deviceManaged()
            .maxPower(0.8)
            .outerPositionP(5.0)
            .devicePositionToleranceTicks(12)
            .doneDeviceManaged()
        .nonPeriodic()
        .bounded(0.0, 4200.0)
        .nativeUnits()
        .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetFromNewCommand(0.0)
        .build();
```

Device-managed tuning options:

* `maxPower(...)` — power Phoenix reapplies after each FTC `RUN_TO_POSITION` target.
* `outerPositionP(...)` — FTC outer position-loop proportional gain.
* `innerVelocityPidf(...)` — FTC inner velocity-loop PIDF used under position mode.
* `devicePositionToleranceTicks(...)` — FTC motor-controller target tolerance in native ticks.

These are configuration answers, so Phoenix rejects invalid values instead of clamping them and
checks them again before hardware lookup or controller effects. See
[Controller-configuration domains](#controller-configuration-domains) for the exact contracts.

Notice that plant-level `positionTolerance(...)` lives after coordinate mapping and reference policy
because it is in plant units. It is a required one-time answer for this feedback Plant. Device-level
methods that use native/controller units say so in their names.

### Framework-regulated motor position

Use `regulated()` when Phoenix should drive raw motor power from an explicit feedback source and a
regulator:

```java
this.arm = FtcActuators.plant(hardwareMap)
        .motor("armMotor", Direction.FORWARD)
        .position()
        .regulated()
            .externalEncoder("armEncoder")
            .regulator(ScalarRegulators.pid(Pid.withGains(0.006, 0.0, 0.0002)))
        .nonPeriodic()
        .bounded(-300.0, 1200.0)
        .nativeUnits()
        .alreadyReferenced()
        .positionTolerance(20.0)
        .targetFromNewCommand(0.0)
        .build();
```

The regulator sees plant-unit error. If the plant uses `nativeUnits()`, plant units and native units
are the same. If the plant uses `scaleToNative(...)`, the feedback is converted into plant units
before the regulator runs.

### Controller limits, complete-regulator limits, and output safety

These protections answer different questions:

| Layer | Question it answers |
|---|---|
| `Pid.setOutputLimits(...)` | How large may a plain PID controller's complete P+I+D output be? |
| `PidfRegulator.setPidOutputLimits(...)` | How large may a standard PIDF regulator's P+I+D contribution be before `kF * setpoint` is added? |
| `ScalarRegulators.outputLimited(...)` | How large may this complete inner regulator composition be? |
| Plant/output command safety | What finite command may the actuator channel ultimately accept? |

`PidfRegulator.setIntegralLimits(...)` limits its integral contribution, and
`setPidOutputLimits(...)` limits its combined P+I+D contribution. Neither limits the later
`kF * setpoint` term. These inner limits also run before another outer decorator. For example,
voltage compensation can legitimately scale a previously limited PIDF command above the inner
PID-output limit. When the robot intentionally wants a narrower command range around everything,
make the limiter the outermost output-changing decorator. The complete regulated-velocity example in
[Velocity bounds, mapping, and tuning](#13-velocity-bounds-mapping-and-tuning) shows that composition
around PIDF and battery-voltage compensation.

Every controller or regulator limit above saturates finite excursions only. It never turns
`NaN`, infinity, or arithmetic overflow into a plausible boundary command; invalid math remains
visible so `outputLimited(...)` or the regulated Plant can reject it and fail closed.

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

With declared plant bounds `plantMin` and `plantMax`, Phoenix first computes the scale, then uses
the same multiply-before-add order as the runtime map:

```text
nativePerPlantUnit = (nativeAtPlantMax - nativeAtPlantMin)
                   / (plantMax - plantMin)

native = nativeAtPlantMin
       + nativePerPlantUnit * (plant - plantMin)
```

For every fully known static bounded affine map, Phoenix evaluates the same runtime arithmetic at
both inclusive Plant endpoints before build. Affinity makes those endpoint images a complete
numeric proof, including a negative scale or reversed native endpoints. The check is exact: a
one-ULP raw-domain overshoot is rejected rather than rounded or clamped into place. A mapping whose
native offset awaits `assumeCurrentPositionIs(...)` or `needsReference(...)` instead validates the
candidate core map at reference commit and checks the FTC raw domain before each realized write.

So for a servo declared as `bounded(-45.0, 90.0)`, calling `rangeMapsToNative(0.22, 0.76)` means
“plant `-45.0` maps to raw servo `0.22`, and plant `90.0` maps to raw servo `0.76`.”

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
the invalid command, then best-effort invokes the output's natural stop. If that cleanup returns
normally, the Plant exposes its ordinary stopped state. If cleanup itself fails, that failure is
suppressed under the original mapping failure and prior applied/status/resolution state is retained.
This deterministic mapping policy does not claim that arbitrary SDK writes are transactional.

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
the reference is pending, Phoenix samples native feedback and uses the first finite reading as plant
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
Task homeLift = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .holdAfterReference(0.0)
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

`holdAfterReference(value)` separately requires a finite plant-unit logical command. It rejects a
non-finite answer immediately without overwriting an earlier accepted answer or command. A finite
hold remains a normal command request: the complete resolver can overlay or transform it, and the
Plant range and target guards can clamp or otherwise protect it. Select a deliberately safe,
in-range hold value when exact predictable holding is intended.

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

On success, `holdAfterReference(value)` changes the graph-owned command before releasing the search;
the same downstream Plant phase then evaluates the complete normal resolver. Use
`resumeTargeting()` to preserve the existing persistent command and resume that unchanged resolver.
Timeout and active cancellation make the same preserve-command handoff. Every release clears search
ownership and requests an immediate output stop, so a later update cannot refresh search power even
if that stop throws. For a normally returning FTC motor-power stop, zero remains in raw/open-loop
mode; the next device-managed position command reasserts `RUN_TO_POSITION`. Calibration establishes
the Plant reference and never resets the encoder as a hidden side effect.

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

Raw servo units:

```java
this.wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .nonPeriodic()
        .bounded(0.0, 1.0)
        .nativeUnits()
        .targetFromNewCommand(0.0)
        .build();
```

Logical units mapped to raw endpoints:

```java
this.wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .nonPeriodic()
        .bounded(-45.0, 90.0)
        .rangeMapsToNative(0.22, 0.76)
        .targetFromNewCommand(0.0)
        .build();
```

Robot code can now command degrees, while the servo receives raw fractions. In both examples,
`bounded(...)` is stated first and always names the public Plant coordinate. `nativeUnits()` makes
those public units raw servo fractions; `rangeMapsToNative(...)` instead maps the public range's
two endpoints to raw servo fractions. Neither mapping choice decides periodicity or promises a
linear physical linkage. For example,
`.nonPeriodic().bounded(0.20, 0.80).nativeUnits()` expresses software-limited travel directly in raw
servo fractions; no Servo-only shortcut changes what any of those three answers means.

Before hardware lookup, the completed Servo recipe maps both Plant bounds through the shared map
and every configured child transform. Every final raw endpoint must be finite and inside inclusive
`[0.0, 1.0]`. Thus `.bounded(-1.0, 1.0).nativeUnits()` and
`.rangeMapsToNative(-0.01, 1.01)` are configuration errors, while reversed endpoints and a valid
mirror such as child scale `-1.0`, bias `1.0` remain legal. This software-domain proof says nothing
about horn geometry, linkage linearity, collision, or safe physical travel; those remain mechanism
calibration facts.

---

## 9. Regulated CR-servo position Plants

CR servos do not have a device-managed position mode. Position control requires native feedback and
a regulator:

```java
this.turret = FtcActuators.plant(hardwareMap)
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
        .targetFromNewCommand(0.0)
        .build();
```

A CR-servo position Plant can participate in `PositionCalibrationTasks.search(...)` because its one
mechanism-owned Plant update can submit temporary open-loop power while looking for a reference.
The Task manages the search lifecycle without becoming another Plant writer.

The hardware-neutral gateway expresses the same rotating-plate design when a custom adapter already
owns the CR-servo power channel, continuous position feedback, and regulator:

```java
PositionPlant plate = Plants.fromOutputs()
        .regulatedPosition(crServoPowerOut, unwrappedAngle, regulator)
        .periodic(2.0 * Math.PI)
        .bounded(-4.0 * Math.PI, 4.0 * Math.PI)
        .nativeUnits()
        .needsReference("plate not indexed")
        .positionTolerance(Math.toRadians(2.0))
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
telemetry.addData("requestedError", plant.getRequestedTargetError());
telemetry.addData("atTarget", plant.atTarget());
```

For `PositionPlant`, `getRequestedTarget()`, `getAppliedTarget()`, `getMeasurement()`, and `getRequestedTargetError()` are all in plant units.
`PositionPlant.positionSource()` is also in plant units. Context-aware target resolvers such as
`PlantTargets.equivalentPositionsOf(...)` and `PlantTargets.plan(request)` receive the Plant measurement,
range, and periodicity automatically through the Plant target context during `update(clock)`.

---

## 12. Position tolerances and FTC motor tuning

For device-managed motor position control there are two different tolerance concepts. They belong
to different owners and neither can be derived safely from the other.

### `positionTolerance(...)`

This is the **plant-level** completion band used by `Plant.atTarget()` and literal physical
`Plant.atTarget(value)`. A feedback-aware
`ScalarTasks.set(command, value).untilReachedBy(plant)` may correlate a logical periodic command
with a different selected physical equivalent, but the Plant query itself never compares modulo a
period.

* required exactly once after position mapping and reference policy
* units: plant position units
* examples: ticks for `nativeUnits()`, inches for `scaleToNative(TICKS_PER_INCH)`, degrees for a degree-based arm
* defines “close enough for robot logic” for the complete mechanism; Phoenix supplies no hidden or
  native-unit default

### `devicePositionToleranceTicks(...)`

This is an **optional FTC motor-controller override** for the motor's own target-position tolerance.

* default in Phoenix: unchanged unless you call it
* units: native encoder ticks
* use this only when you intentionally need the advanced
  `DcMotorEx.setTargetPositionTolerance(int)` override
* it does not define or replace the required plant-unit `positionTolerance(...)`

Phoenix evaluates Plant completion from its requested target, applied target, converted
measurement, target status, and required Plant tolerance. It does not use `DcMotor.isBusy()` for
that decision. The public FTC SDK contract also does not establish that the configured device
tolerance is the controller's complete correction deadband, so do not infer that the motor stops
correcting at that boundary. Ordinary robot code should normally use
`deviceManagedWithDefaults()` and choose only the required Plant tolerance; enter
`deviceManaged()...doneDeviceManaged()` when an FTC controller override is deliberately needed.

### Controller-configuration domains

FTC device-managed P, I, D, and F coefficients—including `outerPositionP(...)`, every component of
`innerVelocityPidf(...)`, and every component of velocity `velocityPidf(...)`—must be finite and
inside this inclusive symmetric domain:

```text
[-Integer.MAX_VALUE / 65536.0, +Integer.MAX_VALUE / 65536.0]
```

That is approximately `[-32767.99998474121, +32767.99998474121]`. It is the no-saturation domain of
the pinned FTC SDK 11.1 REV **public coefficient conversion**, not the complete raw wire-field range
and not a recommendation that every representable gain is useful or safe. Phoenix does not infer a
nonnegative-gain rule: negative values and both signed zeros remain valid. The controller still
applies its ordinary 1/65536 coefficient quantization.

The other explicit position-controller settings have their own domains:

* `maxPower(...)` is a magnitude and must be finite and inside `[0.0, 1.0]`. Both signed zeros are
  accepted. Negative values, overshoot, `NaN`, and infinities are rejected rather than saturated.
* `devicePositionToleranceTicks(...)` must be inside the pinned FTC/REV native unsigned 16-bit
  target-command range `[0, 65535]`.

Every tuning answer is validated before it changes the recipe. A complete PIDF tuple is accepted
all-or-nothing, a rejected first answer can be corrected through the same retained stage, and a
second answer to the same knob is rejected without replacing the first. Position
`doneDeviceManaged()` requires at least one accepted setting, closes the multi-setting section, and
prevents repeated close or later setters. Phoenix revalidates the complete branch before hardware
lookup or configuration so a retained-stage cast cannot bypass the staged grammar.

The common defaults path remains deliberately smaller. `deviceManagedWithDefaults()` calls no
optional controller P, PIDF, or target-tolerance setter and leaves position maximum power at `1.0`;
it cannot later be turned into the explicit tuning branch. An explicit position branch also retains
maximum power `1.0` unless it answers `maxPower(...)`. Those optional controller settings therefore
remain unchanged unless the recipe deliberately supplies an override.

## 13. Velocity bounds, mapping, and tuning

Motor velocity uses the same guided-builder rule as position: required conceptual questions are
explicit, including one `velocityTolerance(...)` answer after public-unit mapping. Optional
controller tuning appears only after entering a tuning branch.

```java
this.shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithDefaults()
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
        .deviceManaged()
            .velocityPidf(kP, kI, kD, kF)
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetFromNewCommand(0.0)
        .build();
```

The velocity tuning branch intentionally exposes only the complete `velocityPidf(...)` answer, so
that answer advances directly to bounds; there is no administrative `doneDeviceManaged()` step and
no empty tuned spelling of the defaults path. The separate FTC position-loop gain
(`outerPositionP(...)`) belongs to device-managed motor **position** mode, not pure velocity mode.
This method writes FTC device-controller coefficients under the validation contract above; it is
distinct from Phoenix's software `PidfRegulator` and does not construct one.

Use `regulated()` when Phoenix should own the velocity loop and command raw motor power. This is the
right path for custom power-based flywheel control, including optional battery-voltage compensation:

```java
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.control.PidfRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulators;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcSensors;

static final double TICKS_PER_FLYWHEEL_REV = 28.0;
static final double TICKS_PER_RPM = TICKS_PER_FLYWHEEL_REV / 60.0;

ScalarSource batteryVoltage = FtcSensors.batteryVoltage(hardwareMap);

PidfRegulator nominalFlywheel = ScalarRegulators.pidf(kP, kI, kD, kF)
        .setIntegralLimits(-0.15, 0.15)
        .setPidOutputLimits(-1.0, 1.0);

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

this.shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .regulated()
            .internalEncoder()
            .regulator(flywheelRegulator)
        .bounded(0.0, 5000.0)          // plant units: RPM
        .scaleToNative(TICKS_PER_RPM)  // native units: FTC ticks/sec
        .velocityTolerance(75.0)       // plant units: RPM
        .targetFromNewCommand(0.0)
        .build();
```

`ScalarRegulators.pidf(kP, kI, kD, kF)` is the one public construction path for the standard
software law:

```text
PID(setpoint - measurement, dt) + kF * setpoint
```

The `kF` units are regulator-command units per plant-setpoint unit. It is not a static-friction,
acceleration, gravity, or FTC device PIDF term, and plain `Pid` does not own a `kF` gain.

The old-style formula
`(referenceVoltage / measuredVoltage) * controllerOutput` belongs in the regulator decorator, not in
the OpMode loop. The Plant still owns target sampling, target bounds, unit conversion, feedback
measurement, and final hardware output. The regulator receives setpoint and measurement in plant
units, so in the example above both values are RPM even though the native encoder reports ticks/sec.

`ScalarRegulators.voltageCompensated(...)` is intentionally a generic core decorator rather than a
flywheel-only builder method. It can wrap PID, PIDF, or a custom `ScalarRegulator`, and regulated
Plants automatically include its debug fields under `plantPrefix.regulator`. In the example,
`outputLimited(...)` is deliberately outside voltage compensation so the requested
`[0.0, maximumFlywheelPower]` policy covers the fully compensated command.
`nominalFlywheel.setPidOutputLimits(...)` covers only P+I+D before feedforward and compensation.

Retain both handles when a tuning tool may change gains while the robot is running. Apply the four
standard gains together, then reset the **outermost** composition so every nested stateful
controller or decorator clears its history:

```java
nominalFlywheel.setGains(newKP, newKI, newKD, newKF);
flywheelRegulator.reset();
```

The gain update keeps configured limits and either accepts all four finite values or rejects the
change. Reset does not itself command or stop the actuator; the robot mechanism owner still decides
when a live tuning change is safe and whether a zero target means coast, brake, or active hold.

Keep that update in a dedicated test mode rather than reading mutable tuning fields from production
TeleOp or Auto. Follow the
[`software PIDF tuning workflow`](<../testing-calibration/Software PIDF Tuning Workflow.md>) to
apply one candidate, report the accepted getters, copy them into the checked-in profile, and
restart production.

When feedforward is nonlinear, table-driven, or otherwise custom, keep that difference explicit:

```java
ScalarRegulator customNominal = ScalarRegulators.setpointFeedforward(
        ScalarRegulators.pid(customController),
        targetRpm -> shotModel.feedforwardForRpm(targetRpm)
);
```

That advanced composition remains available without turning the standard four-gain PIDF factory
into a second custom-feedforward API.

If robot code wants nicer plant velocity units, keep the controller native units explicit:

```java
this.shooter = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 5000.0)          // plant units: RPM
        .scaleToNative(TICKS_PER_RPM)  // native units: FTC ticks/sec
        .velocityTolerance(75.0)       // plant units: RPM
        .targetFromNewCommand(0.0)
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
active, Phoenix bypasses the position scale/bias transform and submits one validated normalized
power command identically to each motor through its configured `Direction`. This preserves both
position alignment during normal targeting and exact zero-as-stop during search.

For every fully known bounded recipe, the shared map and every child endpoint are preflighted before
hardware resolution. A runtime-dependent reference gets core candidate-map validation when it is
established and a final FTC-domain check before each realized write. During operation, Phoenix
computes and validates every child command into temporaries before writing the first child. That
prevents a predictable later-child mapping error from moving an earlier child, but it does not make
sequential SDK writes atomic if an actual device call fails.

### Opposed flywheels with a fixed speed ratio

Use `Direction` for opposed physical mounting and a positive child scale for a proven proportional
speed ratio:

```java
this.flywheels = FtcActuators.plant(hardwareMap)
        .motor("leftFlywheel", Direction.FORWARD)
        .andMotor("rightFlywheel", Direction.REVERSE) // opposed mounting
        .scale(0.96)                                   // positive speed ratio
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, MAX_VELOCITY_NATIVE)
        .nativeUnits()
        .velocityTolerance(VELOCITY_TOLERANCE_NATIVE)
        .targetFromNewCommand(0.0)
        .build();
```

A shared target of `2500.0` commands the reference child to `2500.0` and the scaled child to
`2400.0`. An active target of exact zero passes through the scale and submits zero to both children.
Lifecycle `stop()` invokes each child's natural stop directly. Those paths agree on zero actuation,
but they need not have the same FTC mode acquisition or lifecycle sequence; for example, an active
command may assert its device-managed mode while a lifecycle stop need not reacquire it.

Grouped device-managed feedback inverse-maps each child's sample into shared group units and reports
their overflow-safe arithmetic mean. Consequently, grouped `atTarget()` compares that aggregate
with the one shared target; it is not proof that every motor is independently inside tolerance.
Opposing child errors can cancel in the aggregate.

If robot testing proves a constant additive or nonlinear trajectory trim, or readiness must prove
each wheel independently, that mechanism has two commanded degrees of freedom rather than one
scalar group target. Keep two private one-motor Plants inside the shooter subsystem and expose one
semantic paired command:

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
`Plants.fromOutputs().regulatedPosition(groupOutput, feedback, regulator)` or
`Plants.fromOutputs().regulatedVelocity(groupOutput, feedback, regulator)`. That adapter owns its
complete group lifecycle and failure contract. Independently combining public
`FtcHardware.motorPower(...)` outputs does not receive the standard builder's all-child preflight,
so sequential child writes are not an equivalent safe construction path.

### Mapping failures and retry

An invalid numeric scale/bias answer is rejected before mutation, so a retained builder stage may
retry. An incompatible branch, such as a nonzero motor bias followed by `velocity()`, fails before
fresh hardware effects once the complete recipe is known. Diagnostics identify the operation,
actuator family and child, relevant endpoint or runtime value, transform, computed result, and
required domain. Valid negative position scales, reversed endpoint maps, signed zero, exact domain
boundaries, and later valid retry remain supported; configuration is never silently clamped.
