# Supervisors & Pipelines

Phoenix is designed so student code can stay simple **and** scale to advanced
automation without turning into "spaghetti".

This document explains the implementation-side architecture for:

- **Subsystems** (hardware + one place that writes plant targets)
- **Supervisors** (policy + orchestration, usually built from signals and tasks)
- **Pipelines** (how to combine base targets and temporary overrides without violating the target-resolver ownership rule)

If you're brand new, read these first:

- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Sources and Signals`](<../core-concepts/Sources and Signals.md>)
- [`Recommended Robot Design`](<Recommended Robot Design.md>)
- [`Output Tasks & Queues`](<Output Tasks & Queues.md>)

A useful companion is [`Recommended Robot Design`](<Recommended Robot Design.md>). Before adding
structure, decide which behavior pattern fits the problem:

- local target (`Plant`)
- scalar regulation (`ScalarSource` + controller + `Plant`)
- event/classification supervision (`BooleanSource` / `Source<T>` + supervisor/task)
- spatial guidance (`DriveGuidance` today)
- external route integration (Road Runner / Pedro wrapped behind Phoenix seams)

That decision keeps subsystems smaller and stops drive-specific abstractions from leaking into every mechanism.

---

## The core rule: one writer per plant

A **Plant** is a mechanism sink: it accepts a scalar target (power / position / velocity)
and drives hardware toward it.

> **Target-resolver ownership rule:** Each Plant's final target resolver should be owned in one place.
> That place is typically the subsystem.

Why this matters:

- avoids “two things fighting” (e.g., driver power vs. automation power)
- makes debugging simpler (there is one source of truth)
- makes it safe to share code between TeleOp and Auto

---

## The three code roles

### 1) Robot container (the wiring hub)

A robot container is a class that:

- retains FTC resources and a defensive robot-profile snapshot
- constructs subsystems with `HardwareMap` plus their data-only config, and creates supervisors
- defines gamepad bindings
- calls `update(...)` each loop in a clear order

Many Phoenix robots start with a single `Robot` class used by TeleOp and Auto OpModes.

### 2) Subsystem (target-resolver owner)

A subsystem is responsible for:

- receiving `HardwareMap` plus its data-only config, defensively copying the config, and constructing
  and owning the mechanism hardware (Plants + sensor Sources)
- exposing **signals** (`BooleanSource`, `ScalarSource`, `PlantSources`) to the rest of the robot
- owning **output queues** (`OutputTaskRunner`) when needed
- computing the **final Plant target resolvers** (base + overrides) and updating the Plants each loop

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

- one command-backed `Plant`
- one state variable (an enum or boolean)
- bindings set that state
- the loop converts state → plant target

This flat OpMode is an intentional first-lesson exception. It teaches one Plant before a student has
a robot owner or mechanism class; do not keep the composition root constructing raw Plants after the
mechanism is extracted. The ordinary structured pattern is Level 1 below and the
[`Modern Starter Robot`](<../examples/Modern Starter Robot.md>).

Example: a servo with a few valid positions in that one-file lesson.

```java
public enum WristPose { STOW, INTAKE, SCORE }

private Plant wristPlant;
private WristPose pose = WristPose.STOW;

// init()
wristPlant = FtcActuators.plant(hardwareMap)
        .servo("wrist", Direction.FORWARD)
        .position()
        .nonPeriodic()
            .bounded(0.0, 1.0)
            .nativeUnits()
        .targetFromNewCommand(0.10)
        .build();

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

wristPlant.commandTarget().set(target);
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

- receives `HardwareMap` and a robot-owned data-only config, copies the config, and constructs its
  private Plant
- owns the state (`desiredPose`)
- exposes semantic commands and applies the target each loop

The OpMode/RobotContainer becomes cleaner: it just binds inputs and calls `subsystem.update(clock)`.
It constructs the owner, not the raw Plant:

```java
// Composition root
wrist = new WristSubsystem(hardwareMap, profile.wrist);
```

The corresponding mechanism constructor owns the low-level builder (the illustrative `WristConfig`
is robot code, not a framework type):

```java
final class WristSubsystem {
    private final Plant wrist;
    private final double stowedPosition;
    private final double intakePosition;
    private final double scorePosition;

    WristSubsystem(HardwareMap hardwareMap, WristConfig config) {
        WristConfig snapshot = Objects.requireNonNull(config, "config").copy();
        this.stowedPosition = snapshot.stowedPosition;
        this.intakePosition = snapshot.intakePosition;
        this.scorePosition = snapshot.scorePosition;
        this.wrist = FtcActuators.plant(
                        Objects.requireNonNull(hardwareMap, "hardwareMap"))
                .servo(snapshot.servoName, snapshot.direction)
                .position()
                .nonPeriodic()
                    .bounded(0.0, 1.0)
                    .nativeUnits()
                .targetFromNewCommand(snapshot.stowedPosition)
                .build();
    }

    void selectPose(WristPose pose) {
        wrist.commandTarget().set(targetFor(pose));
    }

    void update(LoopClock clock) {
        wrist.update(clock);
    }

    void stop() {
        wrist.stop();
    }

    private double targetFor(WristPose pose) {
        switch (Objects.requireNonNull(pose, "pose")) {
            case STOW:
                return stowedPosition;
            case INTAKE:
                return intakePosition;
            case SCORE:
                return scorePosition;
            default:
                throw new AssertionError("Unhandled wrist pose: " + pose);
        }
    }
}
```

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

### Pattern B: pick one small input-memory shape before adding logic

When a mechanism grows past one boolean, it helps to make the caller-owned memory explicit.
Phoenix now has three small helpers for the most common cases:

- **`HeldValue<T>`** — the value stays until another call changes it.
- **`FrameValue<T>`** — the value is fresh only for the current loop, then falls back automatically.
- **`RequestCounter`** — pending work tokens that supervisors consume later.

Typical pairings:

```java
HeldValue<Boolean> flywheelRequested = new HeldValue<>(false);
HeldValue<Double> selectedVelocity = new HeldValue<>(1800.0);
FrameValue<Double> manualLiftPower = new FrameValue<>(clock::cycle, 0.0);
RequestCounter shotRequests = new RequestCounter(3);
```

A good default vocabulary is:

- `set...(...)` / `select...(...)` for held values
- `command...(...)` for frame values
- `request...(...)` for pending work in a `RequestCounter`

This keeps layer-1 input memory small and obvious before the supervisor starts deciding what to do with it.

---

## What a subsystem does going forward

Subsystems still own their target resolvers and Plant update order.

A typical subsystem update looks like:

1. update output queues
2. compute base target (from state)
3. apply pipeline/priority rules (base vs overrides)
4. update the Plant after its resolver inputs are ready; the Plant invokes the resolver during that update

### Base + behavior overrides

For Plant targets, use `PlantTargets.overlay(...)`. It keeps simple scalar baselines, queued pulse outputs, and smarter Plant-aware target resolvers in the same target-generation lane.

```java
// Long-lived subsystem fields
private final ScalarTarget baseTarget = ScalarTarget.create(0.0);
private final OutputTaskRunner overrideQueue = Tasks.outputQueue(0.0);
private final Plant plant;

// In the subsystem constructor: copy config, then build the graph and Plant once.
MechanismConfig snapshot = Objects.requireNonNull(config, "config").copy();
PlantTargetResolver finalTarget = PlantTargets.overlay(baseTarget)
        .add("queue", overrideQueue.activeSource(), overrideQueue)
        .add("eject", ejectRequested, -1.0)
        .build();

this.plant = FtcActuators.plant(hardwareMap)
        .motor(snapshot.motorName, snapshot.direction)
        .power()
        .targetFromResolver(finalTarget)
        .build();

// In update(clock): advance changing inputs, then apply the already-built graph.
overrideQueue.update(clock);
plant.update(clock);
```

`MechanismConfig` is illustrative robot-owned data, not a framework type. `ejectRequested` is a
long-lived mechanism/supervisor input; the constructor does not sample it or rebuild the overlay.

Semantics: every layer's activation gate is sampled exactly once each loop, then enabled target
producers are resolved lazily from the last-added, highest-priority layer downward. The first
available enabled layer wins. A normal `add(...)` layer that is enabled but unavailable makes the
overlay unavailable and blocks lower-priority targets. Use `addIfAvailable(...)` only when that
condition is intentionally allowed to fall through to the next enabled layer. The base resolves
only after every layer is disabled or explicitly falls through; disabled and shadowed target
producers do not run.

These are overlay semantics, not extra subsystem bookkeeping. The declaration above stays the same,
and robot code does not reset or notify sources as priority changes. In particular, a
`holdMeasuredTargetOnEntry(...)` source captures again when selected after a complete resolution
cycle in which it was not resolved, instead of retaining a value captured while it was shadowed.

---

## When you do and do not need an output queue

### You usually do NOT need a queue for discrete poses

A “pose” (STOW / INTAKE / SCORE) is a long-lived target.
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

**Use:** shape one `ScalarSource` in controls, then copy it each cycle through a semantic mechanism
command. The mechanism retains the command-backed Plant and owns its update.

Example: motor power from a trigger.

```java
ScalarSource intakePower = pads.p1().rightTrigger().scaled(1.0);
bindings.copyEachCycle(intakePower, intake::commandPower);
```

```java
// Inside the intake mechanism; its constructor built this private Plant from HardwareMap + config.
void commandPower(double power) {
    plant.commandTarget().set(power);
}

void update(LoopClock clock) {
    plant.update(clock);
}
```

Best practices:

- apply shaping/clamping in the source graph (`deadband`, `scaled`, `clamped`)
- let controls call semantic capability methods instead of exposing the Plant
- keep the subsystem as the owner of target resolvers and Plant updates

### 2) Hold-to-run

**Use:** `BooleanSource.choose(...)` for simple on/off, or an output queue if you need a shaped action.

Simple on/off:

```java
BooleanSource request = pads.p1().rightBumper();
ScalarSource out = request.choose(ScalarSource.constant(1.0), ScalarSource.constant(0.0));
```

If this source belongs to controls, copy it through a semantic mechanism command as in recipe 1.
If it belongs to mechanism behavior, compose it into that mechanism's target graph during
construction. In either case, raw Plants remain private.

If you need a repeated action, use `OutputTaskRunner.whileHigh(...)` (see recipe #5).

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
- provide a total-abort button that calls `cancelAndClear()`
- call `cancelAndClear()` on mode changes (e.g., shooter disabled)

Phoenix provides a tiny helper for the "cap" part: `RequestCounter`.
Supervisors typically:

- `request()` or `request(n)` on `onRise`
- `consume()` or `consumeAll()` when the real-world event happens (e.g., ball leaves)

### 6) Hold-to-repeat pulses (repeat while held)

**Use:** `OutputTaskRunner.whileHigh(clock, request, backlog, factory)`

Typical pattern:

- the request signal is driver intent (held trigger)
- the staged `Tasks.outputPulse(...)` recipe contains its readiness gate
- backlog is usually 1 (keep one action buffered)

When request becomes false, the active Task is cancelled and the queue is cleared automatically.

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

### 7) Total-abort behavior

`cancelAndClear()` means “cancel active automation and discard all pending automation.” Queued Tasks
never start and receive no cancellation callback.

Good times to abort:

- the request is released (hold-to-repeat)
- user hits a cancel button
- mode changes (shooter off, climb mode, etc.)
- safety trips (jam detection)

Avoid aborting queues randomly: it makes intent hard to reason about.

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

Best practice: keep transition logic in a supervisor and keep the subsystem as the owner of target resolvers and Plant updates.

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
- **subsystems** own final Plant target resolvers and update order
