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
