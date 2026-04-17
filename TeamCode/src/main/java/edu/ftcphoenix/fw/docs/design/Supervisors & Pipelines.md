# Supervisors & Pipelines

Phoenix is designed so student code can stay simple **and** scale to advanced
automation without turning into "spaghetti".

This document explains the implementation-side architecture for:

- **Subsystems** (hardware + one place that writes plant targets)
- **Supervisors** (policy + orchestration, usually built from signals and tasks)
- **Pipelines** (how to combine base targets and temporary overrides without violating the single-writer rule)

If you're brand new, read these first:

- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Sources and Signals`](<../core-concepts/Sources and Signals.md>)
- [`Recommended Robot Design`](<Recommended Robot Design.md>)
- [`Output Tasks & Queues`](<Output Tasks & Queues.md>)

A useful companion is [`Recommended Robot Design`](<Recommended Robot Design.md>). Before adding structure, decide which lane the behavior belongs to:

- local setpoint (`Plant`)
- scalar regulation (`ScalarSource` + controller + `Plant`)
- event/classification supervision (`BooleanSource` / `Source<T>` + supervisor/task)
- spatial guidance (`DriveGuidance` today)
- external route integration (Road Runner / Pedro wrapped behind Phoenix seams)

That decision keeps subsystems smaller and stops drive-specific abstractions from leaking into every mechanism.

---

## The core rule: one writer per plant

A **Plant** is a mechanism sink: it accepts a scalar target (power / position / velocity)
and drives hardware toward it.

> **Single-writer rule:** Each plant's final target should be computed in one place.
> That place is typically the subsystem.

Why this matters:

- avoids “two things fighting” (e.g., driver power vs. automation power)
- makes debugging simpler (there is one source of truth)
- makes it safe to share code between TeleOp and Auto

---

## The three code roles

### 1) Robot container (the wiring hub)

A robot container is a class that:

- wires hardware once (plants, sensors)
- creates subsystems and supervisors
- defines gamepad bindings
- calls `update(...)` each loop in a clear order

Many Phoenix robots start with a single `Robot` class used by TeleOp and Auto OpModes.

### 2) Subsystem (single writer)

A subsystem is responsible for:

- owning hardware objects for a mechanism (plants + sensor Sources)
- exposing **signals** (`BooleanSource`, `ScalarSource`, `PlantSources`) to the rest of the robot
- owning **output queues** (`OutputTaskRunner`) when needed
- computing the **final plant targets** (base + overrides) and applying them each loop

A subsystem should not contain “match strategy” or cross-subsystem policy.

### 3) Supervisor (policy + orchestration)

A supervisor is responsible for:

- owning small state: cooldowns, request counters, state machines
- translating driver intent and sensor signals into actions
- enqueueing `OutputTask`s to subsystem queues when appropriate
- optionally exposing one small **status snapshot** for telemetry/debug, so callers do not need to understand several internal booleans and queue details

A supervisor usually **does not write plant targets directly**.
It decides *what should happen*, and the subsystem decides *what the final target is*.

---

## Start simple: a beginner robot (one mechanism)

Students should start with as few concepts as possible.

### Level 0: "One file" robot

This is the simplest possible pattern:

- one `Plant`
- one state variable (an enum or boolean)
- bindings set that state
- the loop converts state → plant target

Example: a servo with a few valid positions.

```java
public enum WristPose { STOW, INTAKE, SCORE }

private WristPose pose = WristPose.STOW;

// Bindings
bindings.onRise(gamepads.p1().a(), () -> pose = WristPose.INTAKE);
bindings.onRise(gamepads.p1().b(), () -> pose = WristPose.SCORE);
bindings.onRise(gamepads.p1().y(), () -> pose = WristPose.STOW);

// Loop
double target;
switch (pose) {
  case STOW:   target = 0.10; break;
  case INTAKE: target = 0.45; break;
  case SCORE:  target = 0.80; break;
  default:     target = 0.10; break;
}

wristPlant.setTarget(target);
wristPlant.update(clock);
```

This teaches the essentials:

- loop structure
- state
- bindings
- plants

### Level 1: add one subsystem

When the mechanism grows (more sensors, more signals, more modes), extract a subsystem.

The subsystem:

- owns the plant
- owns the state (`desiredPose`)
- applies the target each loop

The OpMode/RobotContainer becomes cleaner: it just binds inputs and calls `subsystem.update(clock)`.

---

## Add structure: supervisors + pipelines

As soon as you have either of these:

- "do X only when Y is true" (sensor gates)
- repeated actions (feed one ball repeatedly)
- time-based actions (pulse for 0.2s)
- behavior shared between TeleOp and Auto

…it’s time to introduce a supervisor.

### Pattern A (recommended): bindings call supervisor methods

Instead of writing logic inside bindings, bindings call tiny intent methods:

```java
bindings.onRise(pads.p1().a(), supervisor::requestIntakePose);
bindings.onRise(pads.p1().b(), supervisor::requestScorePose);
bindings.onRise(pads.p1().x(), supervisor::pulseOpen);
```

Benefits:

- intent stays readable
- automation logic is reusable (Auto can call the same methods)
- supervisors remain the place where policy lives

---

## What a subsystem does going forward

Subsystems are still the single writer.

A typical subsystem update looks like:

1. update output queues
2. compute base target (from state)
3. apply pipeline/priority rules (base vs overrides)
4. write the final target to the plant

### Base + override pipeline (no extra framework types)

Phoenix already has a clean selector: `BooleanSource.choose(...)`.

```java
// In the subsystem
overrideQueue.update(clock);

ScalarSource base = ScalarSource.constant(baseTarget);
ScalarSource finalTarget = overrideQueue.activeSource().choose(overrideQueue, base);

plant.setTarget(finalTarget.getAsDouble(clock));
```

### Base + multiple overrides

If you have more than one override, use `ScalarOverlayStack`:

```java
ScalarSource finalTarget = ScalarOverlayStack.on(base)
    .add("queue", overrideQueue.activeSource(), overrideQueue)
    .add("eject", ejectRequested, ScalarSource.constant(-1.0))
    .build();
```

Semantics: base always runs; enabled layers override; **last enabled wins**.

---

## When you do and do not need an output queue

### You usually do NOT need a queue for discrete poses

A “pose” (STOW / INTAKE / SCORE) is a long-lived setpoint.
The expected driver behavior is usually:

- **last request wins immediately**

That is a state variable (`desiredPose`), not a queue.

### You DO want a queue for temporary overrides

Use an `OutputTaskRunner` when you want:

- time-shaped overrides (“open for 0.2s”)
- sequential actions (“do this 3 times”)
- sensor-gated actions (“run until beam break clears”)

In that case:

- supervisor enqueues `OutputTask`s
- subsystem overlays queue output while active

---

## Interaction recipes and best practices

This section is the practical “what should we do” guide.

### 1) Direct continuous control (sticks/triggers)

**Use:** `ScalarSource` directly.

Example: motor power from a trigger.

```java
ScalarSource intakePower = pads.p1().rightTrigger().scaled(1.0);
plant.setTarget(intakePower.getAsDouble(clock));
```

Best practices:

- apply shaping/clamping in the source graph (`deadband`, `scaled`, `clamped`)
- keep the subsystem as the single writer

### 2) Hold-to-run

**Use:** `BooleanSource.choose(...)` for simple on/off, or an output queue if you need a shaped action.

Simple on/off:

```java
BooleanSource request = pads.p1().rightBumper();
ScalarSource out = request.choose(ScalarSource.constant(1.0), ScalarSource.constant(0.0));
```

If you need a repeated action, use `OutputTaskRunner.whileTrue(...)` (see recipe #5).

### 3) Toggle modes (latched behavior)

**Use:** `toggled()` if the signal should be derived from a button.

```java
BooleanSource shooterEnabled = pads.p1().leftBumper().toggled(false);
```

Or use a supervisor-owned boolean if toggling depends on other policy.

Best practices:

- keep toggles sampled each loop (toggle sources are stateful)
- if multiple places need the same toggled state, create it once and share it

### 4) Discrete pose selection (servo/motor positions)

**Use:** a `desiredPose` state variable (enum).

Best practices:

- define poses in one place
- map pose → target in a helper method
- last request wins

Optional: add an override queue for “pulse” behaviors.

### 5) Tap-to-queue ("shoot one" pressed multiple times)

You have two design choices:

1) **Allow multiple queued actions** (bounded)
2) **Treat it as a simple request** (no queueing)

If you allow queueing, best practices:

- cap it (e.g. max 3) so spam cannot queue 50 actions
- provide a cancel/clear button
- clear on mode changes (e.g., shooter disabled)

Phoenix provides a tiny helper for the "cap" part: `RequestCounter`.
Supervisors typically:

- `request()` on `onRise`
- `consume()` when the real-world event happens (e.g., ball leaves)

### 6) Hold-to-repeat pulses (repeat while held)

**Use:** `OutputTaskRunner.whileTrue(clock, request, backlog, factory)`

Typical pattern:

- the request signal is driver intent (held trigger)
- the task itself contains readiness gates (`gatedOutputUntil`)
- backlog is usually 1 (keep one action buffered)

When request becomes false, the queue is cleared automatically.

### 6b) Cooldowns (don't do a thing too often)

If a behavior should not run more than once every N seconds (protect hardware, avoid double-feeds),
use a cooldown gate.

Phoenix provides a small helper: `Cooldown`.

- `Cooldown` implements `BooleanSource` where **true means ready**.
- Call `cooldown.trigger(clock)` when you perform the action.

Example:

```java
Cooldown shotCooldown = new Cooldown(0.25);

BooleanSource canShootNow = shooterReady.and(ballPresent).and(shotCooldown);

if (ballLeft.getAsBoolean(clock)) {
  shotCooldown.trigger(clock);
}
```

### 7) Cancel / clear behavior

Clearing a queue means “cancel pending automation”.

Good times to clear:

- the request is released (hold-to-repeat)
- user hits a cancel button
- mode changes (shooter off, climb mode, etc.)
- safety trips (jam detection)

Avoid clearing queues randomly: it makes intent hard to reason about.

### 8) Autonomous vs TeleOp

**Best practice:** Auto should call the same supervisor methods as TeleOp.

- TeleOp expresses intent via bindings.
- Auto expresses intent via tasks (sequence of requests + waits).

A `TaskRunner` is great for sequencing *requests and waits*.
An `OutputTaskRunner` is for producing a scalar output over time.

### 9) Small state machines

Many automation behaviors can be expressed as tasks (sequence/wait) and don't need an explicit
state machine.

When you do need a state machine (multi-step mode with clear phases), keep it tiny.
Phoenix provides `EnumStateMachine<S extends Enum<S>>`:

- stores the current enum state
- records entry time (so you can write time-based transitions)

Best practice: keep transition logic in a supervisor and keep the subsystem as a single writer.

---

## Task vs OutputTask: which tool to use?

- **TaskRunner + Task**: orchestration and sequencing ("do A, then B")
- **OutputTaskRunner + OutputTask**: one scalar output channel over time

A good mental model:

- Tasks decide <i>what happens</i>
- OutputTasks decide <i>what to output</i>

Avoid running `OutputTask` in a plain `TaskRunner` unless you intentionally do not care about its output.

---

## Scaling up: multiple subsystems and supervisors

As the robot grows, you can introduce more structure without changing the basic rules.

Common patterns:

- one subsystem per mechanism
- one supervisor per mechanism
- one higher-level supervisor that coordinates multiple mechanisms for scoring

The architecture remains:

- **robot container** wires and binds
- **supervisors** translate intent + signals into actions
- **subsystems** apply final plant targets (single-writer)

