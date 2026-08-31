# Sources & Signals

Sushi uses a *single loop heartbeat* (see [`Loop Structure`](<Loop Structure.md>)). A lot of robot logic is really
just "read some signals, transform them, and drive plants".

This document introduces the framework's generic, composable building blocks for those signals:

* `Source<T>`
* `ScalarSource` (a `double` source)
* `BooleanSource` (a `boolean` source)
* State helpers: `DebounceBoolean` + `HysteresisBoolean` and their use from sources

---

## One source model

Sensor readings, operator intent, and generated values use one rule:

> A source produces values on the shared `LoopClock`; a stateful source publishes no more than one
> successful observation for a loop cycle.

Stateful filters such as debounce, hysteresis, and rate limiting therefore share the same
one-heartbeat behavior.

`DriveSource` is the drive-specific form of `Source<DriveSignal>`. It follows the same source model
and adds composition methods for drive intent.

---

## The core interfaces

### `Source<T>`

A `Source<T>` is the minimal interface:

* `T get(LoopClock clock)` — sample the value for the current loop
* `reset()` — optional lifecycle hook for stateful sources (clear local memory and, for structural
  source decorators, reset the source/gate children that form that graph; it does not imply
  ownership of every collaborator an object happens to reference)
* `debugDump(...)` — optional debug support

### `ScalarSource`

A `ScalarSource` produces a `double` each loop and adds numeric transforms.

Common uses:

* gamepad sticks / triggers
* sensor readings (distance, velocity, angle error)
* generated targets (e.g. auto-aim heading target)

Common transforms:

* `deadband(...)`
* `scaled(...)`
* `shaped(...)`
* `clamped(...)`

### `BooleanSource`

A `BooleanSource` produces a `boolean` each loop.

Common uses:

* sensor gates (beam break, distance threshold)
* mode enables
* "ready" signals (e.g. shooter at speed)

Common transforms:

* `not()`, `and(...)`, `or(...)`

`and(...)` and `or(...)` describe logical observations, not Java short-circuit control flow. A
composite sample observes the left operand first. If that succeeds, Sushi observes the right
operand once regardless of the left Boolean value, then applies the ordinary truth table. This
keeps a right-hand debouncer, edge detector, toggle, or other stateful source current even while the
left value determines the combined result. The combinator does not add a cache: each stateful
operand owns its own same-cycle protection, and no operand is sampled when the composite itself is
not sampled. Use `choose(...)` when only one selected producer should be sampled.

### `TimeAwareSource<T>`

A `TimeAwareSource<T>` answers at an explicit `LoopTimestamp` through `getAt(clock, timestamp)`.
Use it when a delayed observation must be interpreted with state from that observation's time, such
as a camera frame captured while the robot or an articulated mount was moving. The interface carries
the timestamp but does not impose one validation policy on every implementation: fixed and
current-only adapters may intentionally ignore the request as documented. A validating domain owner
such as `PlanarPoseHistory` instead treats another clock as a wiring error and reports a prior epoch
or materially future request as typed unavailable evidence.

This interface does not promise that every implementation stores history. Fixed and explicitly
current-only adapters remain useful. When historical policy is domain-specific, prefer a named
owner that supplies a stable `TimeAwareSource` projection. For example, `PlanarPoseHistory` owns
bounded planar-pose samples and exposes `lookupSource()` with exact/interpolated/typed-unavailable
results. The localization owner calls `recordCurrent(clock)` after its authoritative estimator
update; consumers only query the projection.

The projection is borrowed. Its `reset()` is deliberately a no-op because a downstream consumer
must not erase shared localization history. The composition owner retains the concrete
`PlanarPoseHistory` and calls its `reset()` at START/STOP. This is the same ownership rule used by
other read projections: a common interface does not transfer lifecycle authority.

## Creating ordinary sources: one grammar

For a clock-aware object value in robot code, use `Source.of(sample)` and then compose the existing
decorators:

```java
Source<TargetingStatus> targetingStatus =
        Source.of(this::calculateTargeting).memoized();

BooleanSource targetSelected = targetingStatus
        .mapToBoolean(status -> status.selection.hasSelection);
```

For a clockless primitive leaf, use the shorter primitive sibling:

```java
ScalarSource requestedSpeed = ScalarSource.of(() -> requestedSpeedRpm);
BooleanSource overrideHeld = BooleanSource.of(() -> overrideButtonHeld);
```

`Source.constant(value)` remains the explicit fixed-object-value case. Ordinary code under
`edu.ftcsushi.robots` should not implement `Source` anonymously or hand-write a cycle cache.
Direct implementation remains an advanced framework/integration seam for a named domain source
whose type adds a real contract, such as `DriveSource`, `TimeAwareSource`, `AprilTagSensor`,
`SpatialQuery`, or `DriveGuidanceQuery`. That extension seam is not a parallel construction recipe
for an ordinary robot value.

---

## Plants as sources

Plants are "sinks" you command, but it is often useful to treat a plant's state as a signal:

- **Is this mechanism at its target?** (`Plant.atTarget()`) → a `BooleanSource`
- **What target is currently commanded?** (`Plant.getRequestedTarget()`) → a `ScalarSource`

Sushi provides a tiny, obvious adapter class: `PlantSources`.

```java
import edu.ftcsushi.fw.actuation.PlantSources;

BooleanSource shooterAtTarget = PlantSources.atTarget(shooterPlant);
BooleanSource shooterReadyStable = shooterAtTarget.debouncedOn(0.15);

ScalarSource requestedTarget = PlantSources.requestedTarget(shooterPlant);
ScalarSource appliedTarget = PlantSources.appliedTarget(shooterPlant);
ScalarSource requestedError = PlantSources.requestedTargetError(shooterPlant);
ScalarSource appliedError = PlantSources.appliedTargetError(shooterPlant);
BooleanSource feedbackCapable = PlantSources.hasFeedback(shooterPlant);
```

Notes:

- `PlantSources.atTarget(...)` reads the plant's cached status from the most recent `plant.update(clock)`.
- `PlantSources.requestedTarget(...)` shows what behavior asked for; `appliedTarget(...)` shows the final mechanism target selected after bounds and target guards. The parallel `requestedTargetError(...)` and `appliedTargetError(...)` methods keep those two reference points explicit. For framework-regulated Plants, the applied target is distinct from the later normalized actuator command.

---

## Memoization

Sushi assumes a single loop heartbeat. In real code, it is easy to accidentally read the same
sensor or derived value multiple times in one loop (especially when it is used in multiple subsystems).

`Source.memoized()` (and the specialized `ScalarSource.memoized()` / `BooleanSource.memoized()`) solves
that by caching a source's value **per loop cycle**:

```java
import edu.ftcsushi.fw.ftc.FtcSensors;

// For FTC hardware sensors, prefer the boundary adapters in fw.ftc.
ScalarSource gateDistanceCm = FtcSensors.distanceCm(distanceSensor);
BooleanSource ballAtGate = gateDistanceCm.hysteresisBelow(6.0, 7.0).debouncedOnOff(0.05, 0.05);
```

After the boundary publishes one successful distance observation, repeated reads in that loop use
the exact same sample. If acquisition throws before publication, a later nonrecursive same-cycle
read retries the sensor instead of receiving a plausible cached value.

Rule of thumb:

- Memoize **raw hardware reads** (distance sensor, encoders, vision measurements).
- Memoize **shared derived signals** that are consumed in multiple places (e.g. `aimLocked`, `shooterReadyStable`).

Do not treat `memoized()` as a repair step that students must add around framework behavior owners.
A stateful owner that advances rate, activation, overlay, or guidance state protects itself by
`clock.cycle()`. Repeated same-cycle reads therefore observe the same successful behavior result.
Pure transforms such as drive scaling can remain stateless; they rely on each stateful dependency to
honor its own cycle contract instead of adding another layer of hidden memory.

When such an owner caches a result, it rejects recursive sampling, stages fallible work, publishes
the complete state/result, and commits the cycle identity last. A failed sample is not recorded as
a null, zero, false, or stale success; its prior committed state remains intact and a later
nonrecursive call may retry in that cycle. Once an attempt succeeds, every repeated same-cycle read
returns that exact published observation. Predicates supplied to a hold/filter are value decisions:
keep them side-effect-free so a failed evaluation can be retried safely.

`ScalarControllers.pid(...)` and `proportional(...)` have one deliberate boundary within this
model. Target and measurement failures happen before controller invocation and remain retryable.
Once an arbitrary stateful controller has been invoked, Sushi cannot roll back its private
integral/filter state. If that invocation throws, the returned source rethrows the same exception
without invoking the controller again for the rest of that cycle; the next cycle may attempt again.

### REV bulk caching is a separate FTC boundary

Sushi source memoization and REV/Lynx bulk caching operate at different layers. A memoized source
publishes one successful result for one `LoopClock` cycle and has Sushi's documented retry
behavior before publication. The advanced `FtcBulkCaching.manual(hardwareMap)` service instead owns
the lifetime of an SDK module-wide packet. It attempts to invalidate that packet before later
services on a claimed cycle; it does not validate, timestamp, or republish the hardware values
returned from it.

Under the reviewed FTC SDK 11.1 behavior, the packet can back digital-input/touch state,
analog-input voltage, motor position, motor velocity, motor busy/at-target state, and motor
over-current status. Motor electrical current and battery voltage use separate ADC commands and are
not cached by this mechanism. Do not infer packet eligibility for I2C, distance, color, vision, or
vendor reads. Writes neither use nor invalidate the packet, so a post-write read may still observe
pre-write cached state.

SDK-managed bulk failures can install a fake zero/false packet that high-level getters cannot
identify: position, velocity, and analog values then read zero; digital and over-current state read
false; and cached motor busy can read true. `ForceStopException` instead escapes after the SDK
clears without installing a packet. Cached `isBusy()` also uses bulk `!atTarget` before the `OFF`
path's `RUN_TO_POSITION` guard, so its result can differ even without a bus failure.

Memoizing any of those getters can consistently repeat the SDK observation; it cannot make a fake
or stale observation fresh, valid, or physically coherent. Neither layer promises device success,
transaction count, timing, or performance. The complete opt-in ownership, invalidation, restoration,
and exclusivity contract is in
[`FTC manual bulk caching`](<../ftc-boundary/FTC Manual Bulk Caching.md>).

## Reset follows ownership

Calling `LoopClock.reset(...)` advances the cycle identity so old cycle caches cannot be reused. It
does **not** call `reset()` on sources or erase controller, filter, selection, or vendor state.
Those lifecycle changes remain explicit responsibilities of their actual owners.

A structural source decorator normally resets its local memory and propagates reset through the
source and gate children it wraps. If one stateful child is deliberately shared by multiple source
graphs, their common owner must reset it at their common lifecycle boundary.

Reset only while the graph is inactive. A structural decorator resets its children first and clears
its own cache/history after all child resets succeed. If a child reset throws, the exception
propagates; an earlier child may already have reset, but the decorator does not pretend that its own
state was cleared or that generic rollback was possible.

That propagation rule does not turn reusable specifications into ownership containers. A
`SpatialQuery` built from a reusable `SpatialQuerySpec` borrows its frame providers, solve lanes,
sensors, estimators, and selection policies. `SpatialQuery.reset()` clears only query-local cached
state; the composition roots that supplied those collaborators retain their reset ownership.
Likewise, drive overlay composition owns activation transitions and cache bookkeeping but does not
reset the overlay's robot-owned dependencies; `DriveOverlay` intentionally has no `reset()` hook.

---

## Deriving rate from linear position

Use `ratePerSecond()` when a source reports a linear, already-unwrapped position and you need its
interval-average rate in the same units per second:

```java
// Simulated or model-owned carriage position in inches.
ScalarSource carriagePositionIn = ScalarSource.of(() -> simulatedCarriagePositionIn);
ScalarSource carriageSpeedInPerSec = carriagePositionIn.ratePerSecond();

// Analog arm position mapped to degrees before differentiation.
ScalarSource armPositionDeg = FtcSensors.analogVoltage(hardwareMap, "armPot")
        .mapToDouble(volts -> volts * DEGREES_PER_VOLT);
ScalarSource armSpeedDegPerSec = armPositionDeg.ratePerSecond();
```

The transform owns its sample history. It uses elapsed `clock.nowSec()` between samples it actually
accepted rather than assuming it ran during the preceding `dtSec()` interval. It publishes at most
one successful result per `clock.cycle()`, returns that exact result for repeated same-cycle reads,
and uses the full interval across skipped cycles. An upstream failure leaves the baseline intact and
the current cycle eligible for a later nonrecursive retry.

The first finite sample establishes the baseline and returns `0.0`. A zero-time sample retains the
previous rate without consuming movement; a regressing clock re-establishes the baseline. A
non-finite sample or calculated rate returns `NaN` for that cycle without poisoning the last valid
baseline. Calling `reset()` propagates upstream and clears the baseline, so restart the estimator at
an intentional source lifecycle boundary. An advanced owner may do this while its active mechanism
is safely idle; terminal `Plant.stop()` is not a reset point and that Plant is never restarted.

`ratePerSecond()` deliberately does not guess a counter width, angular period, discontinuity, sensor
direction, counts per revolution, or filter. Unwrap a periodic or wrapping position in the layer that
knows its period before differentiating, and compose optional signal conditioning separately. For an
FTC external incremental encoder used as regulated velocity feedback, prefer the staged
`FtcActuators...velocity().regulated().externalEncoder(...)` answer: the FTC boundary handles its
signed 32-bit position continuity and the core transform handles elapsed-time differentiation.

---

## Boolean edges and toggles

Many robot behaviors are event-driven: a button press, a ball passing a sensor, a target becoming ready.
`BooleanSource` provides generic helpers so this logic is expressed the same way for gamepads and sensors:

- `risingEdge()` — true for **one loop** when the input transitions `false → true`
- `fallingEdge()` — true for **one loop** when the input transitions `true → false`
- `toggled()` — a stateful boolean that flips on each rising edge (useful for mode toggles)

Example (shoot only when ready and aim locked):

```java
// Example: shoot only while (a) trigger held, (b) aim locked, (c) shooter at speed.

BooleanSource shootHeld = gamepads.p2().rightTrigger().above(0.2);

// Your own error source (absolute heading error in degrees).
ScalarSource headingErrorAbsDeg = ScalarSource.of(() -> Math.abs(rawHeadingErrorDeg));
BooleanSource aimLocked = headingErrorAbsDeg.hysteresisBelow(2.0, 3.0).debouncedOn(0.10);

// Shooter-ready can be any boolean you compute; debouncedOn makes it stable.
// For plants, prefer the tiny adapter helpers from PlantSources.
BooleanSource shooterReadyStable = PlantSources.atTarget(shooterPlant).debouncedOn(0.15);

BooleanSource fireAllowed = shootHeld.and(aimLocked).and(shooterReadyStable);
BooleanSource startFiring = fireAllowed.risingEdge();
```

The same edge/toggle tools apply to sensors (ball entering/leaving a gate sensor) without special cases.

---

## Neutral values and contextual controls

`ScalarSource` and `BooleanSource` are general value streams, not control-only types. A distance,
encoder position, requested-target error, or battery voltage has no universal neutral value. Sushi therefore
does not attach a generic neutral range to every source.

When a source is used as a control, normalize its intended inactive state before registering it. A
Boolean binding treats final `false` as neutral. A contextual scalar copy using
`Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL` treats finite exact zero as neutral, so apply the
appropriate deadband or conditioning first:

```java
ScalarSource winchPower = gamepads.p2().leftY()
        .deadbandNormalized(0.08, -1.0, 1.0);

Bindings.ControlContext endgameControls = bindings.contextWhen(
        endgameMode,
        Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
);
endgameControls.copyEachCycle(winchPower, endgame::commandWinchPower);
```

Each contextual registration rearms independently. A held button does not prevent an unrelated
neutral stick from arming. `ACCEPT_CURRENT` is the explicit alternative when the current held or
finite scalar value should become eligible after the effect-free activation frame.

The FTC adapter `edu.ftcsushi.fw.ftc.input.GamepadDevice` intentionally recenters sticks and
triggers when it is constructed and whenever `calibrate()` is called, then applies its configured
deadband. Robot code constructs it directly with `new GamepadDevice(gamepad1)`. Its boundary
location is what keeps the raw SDK `Gamepad` out of the reusable source and binding packages; the
values it exposes are ordinary Sushi `ScalarSource` and `BooleanSource` objects. The operator
must leave every stick and trigger at its intended physical neutral during construction or
recalibration; software cannot distinguish controller drift from an intentionally displaced
control at that instant.

### Gamepad drive shaping

`GamepadDriveSource` turns three normalized axis sources into robot-centric drive intent. Its
mutable `Config` is setup data, not a live-tuning handle: construction makes an independent copy,
validates that snapshot before sampling an axis, and retains only the snapshot.

| Setting | Required software domain | Boundary meaning |
| --- | --- | --- |
| `deadband` | finite in `[0, 1]` | zero removes the dead zone; one suppresses the complete normalized axis |
| `translateExpo` | finite and `>= 1` | one is linear after deadband normalization; larger values soften center translation |
| `rotateExpo` | finite and `>= 1` | one is linear after deadband normalization; larger values soften center rotation |
| `translateScale` | finite in `[0, 1]` | zero disables translation; one retains full normalized translation scale |
| `rotateScale` | finite in `[0, 1]` | zero disables rotation; one retains full normalized rotation scale |

The framework rejects an invalid authored value at `GamepadDriveSource` construction instead of
silently taking its absolute value, reversing a command, or carrying a non-finite signal toward a
drive sink. `Config.copy()` remains an independent data copy so a profile may copy an unused draft;
the source that understands and retains the shaping answers owns their semantic validation.
Construction validates software coherence only. `GamepadDevice` neutral calibration, safe response,
and drivetrain direction still require the operator and robot checks described in
[`drivetrain direction and integration`](<../testing-calibration/Robot Calibration Tutorials.md#drivetrain-direction-and-integration>).

### Field-relative gamepad drive

`GamepadDriveSource.fieldRelativeTo(...)` interprets its shaped manual translation in a driver
control frame whose up direction is explicitly authored in field coordinates. It consumes cached
`HeadingEstimate` evidence from a separately updated `HeadingEstimator`, rotates translation, and
returns the same robot-centric `DriveSignal` used by overlays and drive sinks.

The configured up heading is independent from the robot's initial heading and from alliance names.
Named station configuration may assign any finite headings, including orthogonal Red/Blue layouts.
When evidence is unusable, translation becomes zero and omega remains available; the source never
silently changes the driver's frame. See the managed
[`Field-relative Drive`](<../examples/Field-relative Drive.md>) lesson for the IMU and full-pose
construction paths.

For complete context ownership, scalar-output limits, and mode examples, see
[`Framework Lanes & Robot Controls`](<../design/Framework Lanes & Robot Controls.md>).

---

## Selection, accumulation, and hold-last patterns

Three patterns show up constantly in real robots:

1. **Manual vs auto selection** ("use the driver's value unless auto-aim is enabled")
2. **Window-local memory** ("remember what I saw during this slot / observation window until a reset event")
3. **Noisy classification** ("sometimes my sensor says UNKNOWN; keep the last good value briefly")

Sushi makes all three patterns explicit and composable.

### Selection: `choose(...)`

`BooleanSource.choose(...)` selects between two other sources based on the boolean's value.
Each observation samples the condition first and then only the selected branch. The unselected
branch is intentionally not sampled.

Example: use an auto-aim computed shooter speed only when aim is locked; otherwise use a manual
driver-set speed.

```java
BooleanSource useAuto = aimLocked.debouncedOn(0.10);

ScalarSource manualRps = ScalarSource.of(() -> 32.0);
ScalarSource autoRps = distanceIn.mapToDouble(d -> lookupShooterRps(d));

ScalarSource shooterTargetRps = useAuto.choose(autoRps, manualRps);
```

### Accumulate within a reset-defined window: `accumulate(...)` / `accumulateUntil(...)`

Sometimes the right memory model is **not** time-based. Instead, you want to keep folding samples
into a remembered state until some explicit boundary signal says "start a new window."

Examples:

* remember a game-piece color while one slot passes under a sensor
* keep the strongest classification seen while an object crosses an observation zone
* fuse repeated noisy samples until an encoder or separator pulse marks the next object

Sushi provides two related helpers:

* `accumulate(step, initial)` — keep state until robot code explicitly calls `reset()`
* `accumulateUntil(reset, step, initial)` — same idea, but reset automatically when a boolean signal is true

Both helpers are transactional value reducers. Treat `initial` and the previous accumulated state
as read-only, perform no external effects in `step`, and return a non-null immutable—or otherwise
independently stable—next value. Returning the same immutable instance for an unchanged state is
fine. Mutating a retained state object in place is not: neither Sushi nor robot code could undo
that mutation if a later read or reducer operation failed.

Example (slot-local color memory):

```java
enum BallColor { GREEN, PURPLE, UNKNOWN }
enum SlotColor { EMPTY, GREEN, PURPLE, UNKNOWN }

Source<BallColor> sampleNow = ...;           // your per-loop fused classification
BooleanSource newSlotPulse = ...;            // one-loop pulse from encoder logic

Source<SlotColor> slotColor = sampleNow.accumulateUntil(
        newSlotPulse,
        (held, cur) -> updateSlotColor(held, cur),
        SlotColor.EMPTY
);
```

If your own observer object already defines the window lifecycle, use `accumulate(...)` and call
`reset()` at the start of each window:

```java
Source<SlotColor> slotColor = sampleNow.accumulate(
        (held, cur) -> updateSlotColor(held, cur),
        SlotColor.EMPTY
);

// startSlot():
slotColor.reset();
```

Use this when the reset condition is a **mechanical / logical boundary**, not a timeout.

### Choosing the reset model

Sushi intentionally keeps **two different reset styles**:

* Use `accumulateUntil(resetSignal, ...)` when the boundary is part of the loop graph
  (encoder pulse, beam-break edge, "new slot" signal, etc.).
* Use `reset()` when some owner outside the graph wants an immediate clear
  (mode change, tester clear, task restart, supervisor restart, OpMode re-init).

`accumulateUntil(...)` does **not** replace `reset()`. It only applies when the source is sampled,
and when the reset signal is true it clears the state and then folds the current sample into the
fresh state in the same loop.

### Hold last valid: `holdLastValid(...)`

For value-object sources (like a color classification), it is common to get short bursts of
invalid output. `holdLastValid(...)` keeps the last valid value for a time window.

Example (ball color classification):

```java
Source<BallColor> rawColor = ...;  // produced by your classifier

// Hold the last non-UNKNOWN classification for up to 0.25 seconds.
Source<BallColor> colorStable = rawColor.holdLastValid(c -> c != BallColor.UNKNOWN, 0.25, BallColor.UNKNOWN);
```

There is also a scalar specialization:

```java
// Hold the last finite measurement for 0.2s; otherwise return NaN.
ScalarSource visionDistanceIn = ...;
ScalarSource distanceStable = visionDistanceIn.holdLastFinite(0.2, Double.NaN);
```

---

## Debounce and hysteresis

Two extremely common forms of signal conditioning are provided as reusable, generic components:

### `DebounceBoolean`

`DebounceBoolean` is a small state machine that takes a raw boolean and returns a debounced boolean.

Use it when:

* you want a signal to become true only after it has stayed true for *N* seconds
* you want to avoid chatter around a threshold

From `BooleanSource`, you can write:

```java
BooleanSource readyStable = readyRaw.debouncedOn(0.15);
```

This means: **turn ON after 0.15s continuously true; turn OFF immediately when false**.

If you want symmetric delays:

```java
BooleanSource stable = raw.debouncedOnOff(0.10, 0.05);
```

### `HysteresisBoolean`

`HysteresisBoolean` is a state machine that turns a noisy scalar measurement into a stable boolean.

From a `ScalarSource`, you can write:

```java
BooleanSource idle = stickMag.hysteresisBelow(0.08, 0.12);
```

That means:

* **ON** when `mag <= 0.08`
* **OFF** when `mag >= 0.12`

This avoids the ON/OFF chatter that happens with a single threshold.

---

## Usage patterns

### Pattern 1: Define sources once, sample them each loop

Define your sources in init:

```java
ScalarSource axial = gamepads.p1().leftY().deadband(0.05).shaped(0.05, 1.7, -1, +1);
BooleanSource shootEnable = BooleanSource.of(() -> gamepads.p2().rightBumperRaw());
```

Sample them in your loop:

```java
DriveSignal cmd = new DriveSignal(
    axial.getAsDouble(clock),
    lateral.getAsDouble(clock),
    omega.getAsDouble(clock)
);
```

### Pattern 2: Prefer composing signals over scattering special-case state

Instead of maintaining multiple boolean flags, compose a clear "ready" signal:

```java
BooleanSource shooterReadyStable = shooterAtSpeed.and(ballPresent).debouncedOn(0.12);
```

That yields a single signal you can use everywhere.

---

## Design rule of thumb

*Sources are for values.*

* Tasks / Plants are for *doing things* based on those values.

If you find yourself writing an ever-growing "controller" class with a bunch of ad-hoc flags, it’s
usually a hint that some of that should become a source.
