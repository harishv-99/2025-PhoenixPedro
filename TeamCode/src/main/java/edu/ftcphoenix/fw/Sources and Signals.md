# Sources & Signals

Phoenix uses a *single loop heartbeat* (see `Loop Structure.md`). A lot of robot logic is really
just "read some signals, transform them, and drive plants".

This document introduces the framework's generic, composable building blocks for those signals:

* `Source<T>`
* `ScalarSource` (a `double` source)
* `BooleanSource` (a `boolean` source)
* State helpers: `DebounceBoolean` + `HysteresisBoolean` and their use from sources

---

## Why this exists

Historically we had:

* "continuous" inputs as `Axis`
* "discrete" inputs as `Button`
* special-purpose latches for stability / hysteresis

That worked, but it created two recurring problems:

1. **Logic scattered across layers**: shaping in `Axis`, edge detection in `Button`, stability in latches,
   and then "actual behavior" elsewhere.
2. **No single mental model** for sensor readings vs operator intent vs generated targets.

`Source` unifies these into one simple rule:

> A source produces a value once per loop, sampled with a `LoopClock`.

That makes stateful filters (debounce, hysteresis, rate limiting, etc.) straightforward and keeps
logic consistent with the one-heartbeat loop.

---

## The core interfaces

### `Source<T>`

A `Source<T>` is the minimal interface:

* `T get(LoopClock clock)` — sample the value for the current loop
* `reset()` — optional (for stateful sources)
* `debugDump(...)` — optional debug support

### `ScalarSource`

A `ScalarSource` produces a `double` each loop. It is the generalized successor to `Axis`.

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

---

## Plants as sources

Plants are "sinks" you command, but it is often useful to treat a plant's state as a signal:

- **Is this mechanism at its target?** (`Plant.atSetpoint()`) → a `BooleanSource`
- **What target is currently commanded?** (`Plant.getTarget()`) → a `ScalarSource`

Phoenix provides a tiny, obvious adapter class: `PlantSources`.

```java
import edu.ftcphoenix.fw.actuation.PlantSources;

BooleanSource shooterAtSetpoint = PlantSources.atSetpoint(shooterPlant);
BooleanSource shooterReadyStable = shooterAtSetpoint.debouncedOn(0.15);

ScalarSource shooterTarget = PlantSources.target(shooterPlant);
BooleanSource feedbackCapable = PlantSources.hasFeedback(shooterPlant);
```

Notes:

- `PlantSources.atSetpoint(...)` is **memoized per loop cycle** so the plant is sampled at most once
  per `LoopClock.cycle()`.
- `PlantSources.target(...)` is **not memoized** by default so you can still observe changes within
  a loop (useful while migrating toward single-writer ownership).

---

## Memoization

Phoenix assumes a single loop heartbeat. In real code, it is easy to accidentally read the same
sensor or derived value multiple times in one loop (especially when it is used in multiple subsystems).

`Source.memoized()` (and the specialized `ScalarSource.memoized()` / `BooleanSource.memoized()`) solves
that by caching a source's value **per loop cycle**:

```java
import edu.ftcphoenix.fw.ftc.FtcSensors;

// For FTC hardware sensors, prefer the boundary adapters in fw.ftc.
ScalarSource gateDistanceCm = FtcSensors.distanceCm(distanceSensor);
BooleanSource ballAtGate = gateDistanceCm.hysteresisBelow(6.0, 7.0).debouncedOnOff(0.05, 0.05);
```

When you call `ballAtGate.getAsBoolean(clock)` multiple times in the same loop, it will not re-sample
the underlying distance sensor more than once (as long as you memoize at the boundary).

Rule of thumb:

- Memoize **raw hardware reads** (distance sensor, encoders, vision measurements).
- Memoize **shared derived signals** that are consumed in multiple places (e.g. `aimLocked`, `shooterReadyStable`).

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
BooleanSource shooterReadyStable = PlantSources.atSetpoint(shooterPlant).debouncedOn(0.15);

BooleanSource fireAllowed = shootHeld.and(aimLocked).and(shooterReadyStable);
BooleanSource startFiring = fireAllowed.risingEdge();
```

The same edge/toggle tools apply to sensors (ball entering/leaving a gate sensor) without special cases.

---

## Selection and hold-last patterns

Two patterns show up constantly in real robots:

1. **Manual vs auto selection** ("use the driver's value unless auto-aim is enabled")
2. **Noisy classification** ("sometimes my sensor says UNKNOWN; keep the last good value briefly")

Phoenix makes both patterns explicit and composable.

### Selection: `choose(...)`

`BooleanSource.choose(...)` selects between two other sources based on the boolean's value.

Example: use an auto-aim computed shooter speed only when aim is locked; otherwise use a manual
driver-set speed.

```java
BooleanSource useAuto = aimLocked.debouncedOn(0.10);

ScalarSource manualRps = ScalarSource.of(() -> 32.0);
ScalarSource autoRps = distanceIn.mapToDouble(d -> lookupShooterRps(d));

ScalarSource shooterTargetRps = useAuto.choose(autoRps, manualRps);
```

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
