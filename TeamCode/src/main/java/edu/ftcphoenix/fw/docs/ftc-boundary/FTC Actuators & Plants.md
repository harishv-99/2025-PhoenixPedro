# FTC Actuators & Plants

This page covers the FTC boundary for Phoenix mechanism wiring:

* `FtcActuators.plant(...)` — the staged beginner builder
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
examples do.

---

## 1. The staged entrypoint: `FtcActuators.plant(...)`

Use `edu.ftcphoenix.fw.ftc.FtcActuators` when you want to create a `Plant` from FTC hardware.

```java
// Inside the mechanism constructor; flywheel and pusher are private fields.
this.flywheel = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetedBy(ScalarTarget.create(0.0))
        .build();

this.pusher = FtcActuators.plant(hardwareMap)
        .servo("pusher", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .nativeUnits()
        .targetedBy(ScalarTarget.create(0.0))
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
8. What public position error counts as complete? The required `positionTolerance(...)` answer
9. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
10. Target binding: `targetedBy(ScalarTarget.create(initialValue))` for the ordinary command or
    `targetedBy(PlantTargetResolver)` for an advanced graph, then `build()`

Motor velocity wiring asks a parallel but smaller set of questions:

1. Which hardware? `motor(...)`
2. Which target domain? `velocity()`
3. Who manages the velocity loop? `deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated()` followed by one direct feedback answer and `regulator(...)`
4. What target bounds are legal? `bounded(min, max)` or `unbounded()`
5. How do plant velocity units map to native velocity units? `nativeUnits()` or `scaleToNative(...)`
6. What public velocity error counts as complete? The required `velocityTolerance(...)` answer
7. Optional dynamic hardware guards: `targetGuards().maxTargetRate(...)`, `holdLastTargetUnless(...)`, `fallbackTargetUnless(...)`
8. Target binding: `targetedBy(ScalarTarget.create(initialValue))` for the ordinary command or
   `targetedBy(PlantTargetResolver)` for an advanced graph, then `build()`

The tolerance question appears exactly once and only after the public coordinate is known. There is
no inferred or native-unit default. Standard-servo position Plants are command-only, so their
mapping stage proceeds directly to target binding without asking a feedback-only question.

A custom hardware adapter can use the hardware-neutral `MappedPositionPlant` and
`MappedVelocityPlant` entrypoints instead of `FtcActuators`. Those entrypoints keep only the stages
that adapter construction needs: configure mapping/range/reference details, answer the required
feedback tolerance, optionally add target guards, bind the target, and build. Command-only mapped
position goes from configuration directly to the target-and-guards stage. Both public layers
therefore prevent an omitted feedback tolerance or target at compile time without making ordinary
FTC robot code learn a second hardware API.

For an ordinary exact Plant, keep one mechanism variable: create the command inline with
`targetedBy(ScalarTarget.create(initialValue))`, then write it through the Plant's stable,
side-effect-free `commandTarget()` accessor. A named `ScalarTarget` is still useful when the target
stands alone, is shared with target-only policy, or is needed while assembling an overlay,
equivalent-position, or advanced resolver graph. A read-only `ScalarSource` is adapted visibly
with `PlantTargets.exact(source)`. For a composed
`PlantTargets.overlay(...)`, only a `ScalarTarget` carried by the base graph becomes the command
target; conditional layers never do. The final graph supplies that identity automatically, so the
builder cannot accept a second target that disagrees with the source the Plant actually follows.

Velocity and power Plants stay simpler than position Plants because they do not have position
geometry, periodicity, or homing/reference questions. Power is simpler still: every direct power
Plant has the fixed normalized range `[-1.0, +1.0]`, so there is no redundant bounds step in the
builder. A finite request outside that range clamps before the output. The Plant reports
`CLAMPED_TO_RANGE` when that clamp remains the active final transform; a later rate limit,
interlock, or fallback may report its more specific status instead.

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
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetGuards()
            .maxTargetRate(1200.0)
            .holdLastTargetUnless("wristClear", wristClear)
            .doneTargetGuards()
        .targetedBy(ScalarTarget.create(0.0))
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
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetedBy(ScalarTarget.create(0.0))
        .build();
```

The public lift coordinate is in plant units. In this example, plant units are also native encoder
ticks because the builder chose `nativeUnits()`.

### Device-managed with FTC tuning

Enter `deviceManaged()` only when you want optional FTC motor-controller tuning. While in this
branch, autocomplete shows only device-managed tuning knobs. `doneDeviceManaged()` returns to the
main position questions.

```java
this.lift = FtcActuators.plant(hardwareMap)
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
        .targetedBy(ScalarTarget.create(0.0))
        .build();
```

Device-managed tuning options:

* `maxPower(...)` — power Phoenix reapplies after each FTC `RUN_TO_POSITION` target.
* `outerPositionP(...)` — FTC outer position-loop proportional gain.
* `innerVelocityPidf(...)` — FTC inner velocity-loop PIDF used under position mode.
* `devicePositionToleranceTicks(...)` — FTC motor-controller target tolerance in native ticks.

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
        .linear()
            .bounded(-300.0, 1200.0)
            .nativeUnits()
            .alreadyReferenced()
        .positionTolerance(20.0)
        .targetedBy(ScalarTarget.create(0.0))
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

`PositionPlant.period()` is in plant units. For one normal logical command, wrap its final resolver
with `PlantTargets.equivalentPositionsOf(commandTarget)` and choose a preference plus explicit
unavailable answer. The wrapper uses the consuming Plant's declared period during
`plant.update(clock)`; robot code does not repeat it. `PlantTargets.plan(request)` remains the
advanced path for multiple alternatives, relative/explicit-period requests, observation metadata,
or clamp policy.
Periodic topology alone never wraps an exact target automatically.

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
private final PositionPlant claw;

// In the claw mechanism constructor; production values come from its copied config.
this.claw = FtcActuators.plant(hardwareMap)
        .servo("clawServo", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .rangeMapsToNative(0.30, 0.80)
        .targetedBy(ScalarTarget.create(0.0))
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

This means plant position `0.0` maps to native encoder tick `ARM_ZERO_TICKS`. Both the plant and
native reference values must be finite. The builder rejects `NaN` or infinity before retaining this
answer, performs no SDK call or hardware effect for that rejection, and does not clamp either
coordinate. It therefore prevents a later build from realizing command hardware with the invalid
pair. An earlier valid feedback-selection stage may already have resolved its own sensor. The plant
reference is an affine-map anchor and need not be a legal target inside the declared Plant range.

### `assumeCurrentPositionIs(...)`

Use this when the robot is physically placed at a known pose before init:

```java
.linear()
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
.linear()
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
`MappedPositionPlant.beginCalibrationSearch(...)` seam enforces the same contract before changing
search state or stopping the normal position output. Neither path clamps an invalid search command;
low-level FTC adapter clamping remains boundary defense only.

The reference supplied to `establishReferenceAt(...)` must also be finite in plant units. The Task
recipe rejects `NaN` or infinity at that answer before starting or changing its search lifecycle;
the direct mapped-Plant seam repeats the check before sampling feedback or changing reference state.
This coordinate anchor is not required to lie inside `targetRange()` and is never clamped into it.

`holdAfterReference(value)` separately requires a finite plant-unit logical command. It rejects a
non-finite answer immediately without overwriting an earlier accepted answer or command. A finite
hold remains a normal command request: the complete resolver can overlay or transform it, and the
Plant range and target guards can clamp or otherwise protect it. Select a deliberately safe,
in-range hold value when exact predictable holding is intended.

This numeric contract is structural, not robot-specific safety approval. The mechanism owner must
still select a safe magnitude and direction, verify the cue and timeout/cancellation paths, provide
any required external interlock or physical safeguard, and confirm that the mechanism is free to
move. An active raw-power search bypasses the normal target resolver and `PlantTargetGuards`; those
normal-target protections do not make the temporary search safe.

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
nearest-equivalent result fails without committing the new reference. General plant/native affine
overflow remains a separate mapping-domain concern.

These checks establish a software numeric contract only. They do not prove the mechanism is at the
declared physical pose, that plant/native scale and sign are correct, that the cue is repeatable, or
that the selected reference and hold are safe under load. Those remain adopting-robot checks.

---

## 8. Standard servo position Plants

Standard servos are command-only position outputs. They do not expose motor control strategy,
open-loop calibration search, or unbounded travel.

Raw servo units:

```java
this.wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .nativeUnits()
        .targetedBy(ScalarTarget.create(0.0))
        .build();
```

Logical units mapped to raw endpoints:

```java
this.wrist = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .linear()
            .bounded(-45.0, 90.0)
            .rangeMapsToNative(0.22, 0.76)
        .targetedBy(ScalarTarget.create(0.0))
        .build();
```

Robot code can now command degrees, while the servo receives raw fractions.

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
        .targetedBy(ScalarTarget.create(0.0))
        .build();
```

A CR-servo position Plant can participate in `PositionCalibrationTasks.search(...)` because its one
mechanism-owned Plant update can submit temporary open-loop power while looking for a reference.
The Task manages the search lifecycle without becoming another Plant writer.

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
range, and topology automatically through the Plant target context during `update(clock)`.

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
        .targetedBy(ScalarTarget.create(0.0))
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
            .doneDeviceManaged()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(50.0)
        .targetedBy(ScalarTarget.create(0.0))
        .build();
```

The velocity tuning branch intentionally exposes only `velocityPidf(...)`. The separate FTC
position-loop gain (`outerPositionP(...)`) belongs to device-managed motor **position** mode, not
pure velocity mode. This method writes FTC device-controller coefficients; it is distinct from
Phoenix's software `PidfRegulator` and does not construct one.

Use `regulated()` when Phoenix should own the velocity loop and command raw motor power. This is the
right path for custom power-based flywheel control, including optional battery-voltage compensation:

```java
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.control.PidfRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulators;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
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
        .targetedBy(ScalarTarget.create(0.0))
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
        .targetedBy(ScalarTarget.create(0.0))
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
