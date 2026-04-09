# Robot Capabilities and Mode Clients

This document explains a robot-owned concept that will likely exist every year even though it does
**not** belong in the common framework as a lane: **capability families**.

Read this when you want to answer questions like:

- What API should both TeleOp and Auto talk to?
- How should a robot split that API into a few cohesive interfaces?
- What is parallel between the TeleOp side and the Auto side?
- What should go in `initTeleOp()` versus `initAuto()`?
- How can a robot stay SOLID without exploding into dozens of tiny interfaces?

Useful companions:

- [`Framework Lanes & Robot Controls.md`](<Framework Lanes & Robot Controls.md>)
- [`Recommended Robot Design.md`](<Recommended Robot Design.md>)
- [`Supervisors & Pipelines.md`](<Supervisors & Pipelines.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)

---

## The one-sentence model

A **capability family** is a robot-owned, mode-neutral façade that sits between the robot's
internal owners and the mode clients that consume them.

In practice:

- `TeleOpControls` maps human input into capability calls and reusable sources
- `AutoPlan` / `AutoRoutine` composes tasks over the same capability calls and status snapshots
- the robot container owns the internal graph and exposes the shared capability families

That is the normal yearly pattern.

---

## Why this is not a framework lane

A framework lane exists when the framework can own a stable reusable multi-object graph that repeats
with nearly the same responsibilities year after year.

Capability families do **not** meet that bar:

- the public nouns change with the game
- the cohesive splits change with the robot
- the right façade often wraps several robot-specific internals
- the same family name is not guaranteed to make sense every season

So capabilities should usually remain **robot-owned**.

That does **not** make them optional. It just means the framework should document the pattern rather
than freezing one universal interface too early.

---

## Where capability families sit

```text
OpMode / mode client layer
  ├─ MyTeleOpControls
  └─ MyAutoPlan / MyAutoRoutine

Shared mode-neutral façade
  └─ MyCapabilities
       ├─ gamePiece()
       ├─ targeting()
       ├─ endgame()
       └─ awareness()

Robot internals
  ├─ services
  ├─ supervisors
  ├─ subsystems
  ├─ presenters
  └─ framework lanes / primitives
```

Important detail:

> A capability family is an outside-looking-in API, not a promise about how the robot is implemented internally.

A family may delegate to one subsystem, one supervisor, one service, or several of them.

---

## How to choose the splits

Use this philosophy.

### 1. Split by cohesive public vocabulary

Group methods that tell one coherent public story.

Good split:

- `targeting()` for selection, aim status, and aim execution
- `scoring()` for intake/flywheel/shoot/eject requests and scoring status

Bad split:

- one interface per internal class even when callers always need them together
- one interface per button or one interface per single method

### 2. Split by common clients

If the same callers almost always need the same cluster of methods, that is evidence they belong
together.

If many callers need only one part, that is evidence for a separate family.

### 3. Keep command families separate from read-only awareness when that makes the API cleaner

Examples that are often useful as distinct families:

- command-heavy mechanism family such as `gamePiece()` or `endgame()`
- read-heavy family such as `awareness()` or `targeting()`

Do not force a separation when the public story is still one cohesive thing. The point is clarity,
not purity.

### 4. Do not mirror TeleOp buttons

A capability family is not the same thing as a control scheme.

Prefer:

- `setFlywheelEnabled(boolean)`
- `requestSingleShot()`
- `cancelTransientActions()`

Avoid making the shared API look like:

- `toggleFlywheel()`
- `shootWhileHeld()`
- `rightTriggerMacro()`

### 5. Do not mirror internal class boundaries mechanically

A subsystem/supervisor/service split is about implementation ownership.

A capability-family split is about the public robot vocabulary.

Those boundaries often correlate, but they are not the same problem.

### 6. Make the common path easy

The framework principles still apply here.

Do **not** split APIs into tiny interfaces just to satisfy SOLID mechanically. A good capability
family should be:

- small enough that unrelated callers are not forced to depend on junk they do not need
- large enough that the normal caller does not have to stitch five internals together by hand

---

## Common family shapes that tend to recur

These are examples, not requirements.

### Often recurring

- `mobility()` / `drive()`
- `gamePiece()` / `scoring()` / `manipulator()`
- `targeting()` / `aiming()`
- `endgame()` / `climb()`
- `awareness()` / `localization()` / `fieldAwareness()`
- `signals()` / `lighting()`

### Why the names vary

The same robot pattern can show up under different names depending on the game:

- one year the main mechanism family is naturally `scoring()`
- another year it is really `liftAndWrist()`
- another year it is `specimen()` vs `sample()`

Choose names that match the robot's cohesive public story this year.

---

## Parallel objects between TeleOp and Auto

The parallelism is about **role**, not about forcing identical class names.

### Shared objects

- `MyRobot`
- `MyCapabilities`
- subsystems / supervisors / services / presenters
- framework lanes and primitives

### TeleOp-side mode client

- `MyTeleOpControls`

Responsibilities:

- choose stick meanings
- choose button semantics
- expose reusable control-layer sources
- bind buttons to capability calls

### Auto-side mode client

- `MyAutoPlan`, `MyAutoRoutine`, or an auto-specific class such as `MyPedroPreloadAuto`

Responsibilities:

- choose the routine or route sequence
- compose tasks over capabilities
- wait on status snapshots and signals
- own route-library-specific strategy

The Auto side does **not** need a universal `AutoControls` class just to mirror TeleOp.

---

## What should go in each init/start/update hook

The exact naming can vary, but this is the recommended ownership split.

### `constructor`

Own references and immutable config snapshots only.

Good:

- `hardwareMap`
- `telemetry`
- copied profile/config

Avoid constructing half the robot here.

### `initAny()`

Only truly mode-agnostic setup.

This can be empty if there is nothing useful to share.

### `initTeleOp()`

Usually owns:

- create the TeleOp controls owner
- create the shared runtime internals needed in TeleOp
- create the shared capability families
- bind TeleOp controls to capabilities
- create TeleOp-only drive policy/runtime objects
- emit INIT help text

### `initAuto()`

Usually owns:

- create the shared runtime internals needed in Auto
- create the shared capability families
- create the shared auto runner, if the robot owns one

Usually does **not** own:

- the specific path library follower configuration for one auto
- the actual autonomous routine selection
- route/task composition for a specific strategy

Those normally live in the Auto mode client or the Auto OpMode.

### `startAny()`

Shared runtime reset such as clock reset.

### `startTeleOp()` / `startAuto()`

Only mode-specific arming/reset work that is not already shared.

These can stay empty when no extra work is needed.

### `updateAny()`

Shared timebase update and other truly common loop state.

### `updateTeleOp()`

Recommended shape:

1. update shared state producers
2. update controls
3. update supervisors/services that consume those signals
4. apply drive policy
5. update subsystems
6. emit telemetry/presentation

### `updateAuto()`

Recommended shape:

1. update shared state producers
2. update the auto task runner / routine
3. update supervisors/services that consume those results
4. update subsystems
5. emit telemetry/presentation

### `stopTeleOp()` / `stopAuto()`

Only mode-owned cleanup.

Examples:

- clear button binding state
- cancel the auto task runner
- stop/reset route-library adapters owned by the Auto mode client

### `stopAny()`

Final hardware-safe stop for physical sinks.

---

## Example shape

```java
public interface MyCapabilities {
    GamePiece gamePiece();
    Targeting targeting();

    interface GamePiece {
        void setIntakeEnabled(boolean enabled);
        void setFlywheelEnabled(boolean enabled);
        void requestSingleShot();
        void cancelTransientActions();
        GamePieceStatus status();
    }

    interface Targeting {
        TargetingStatus status(LoopClock clock);
        Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg);
    }
}
```

A matching robot shape usually looks like this:

```text
MyRobot
  ├─ framework lanes
  ├─ subsystems
  ├─ supervisors
  ├─ services
  ├─ presenter
  ├─ MyCapabilities
  ├─ MyTeleOpControls
  └─ TaskRunner autoRunner
```

And the mode clients look like this:

```text
TeleOp:
  MyTeleOpControls -> MyCapabilities -> robot internals

Auto:
  MyAutoPlan / MyAutoRoutine -> MyCapabilities -> robot internals
```

---

## A good checklist for each year's robot

When you start a new season, expect to name and place these kinds of objects explicitly:

- framework lanes for stable FTC-side graphs
- shared field facts
- subsystems as single writers
- supervisors for policy/orchestration
- services for shared game-specific computation
- presenters for human-facing output
- one shared robot-owned `Capabilities` aggregate
- one TeleOp controls owner
- one or more Auto plan/routine classes
- the robot container / composition root

That checklist is more durable than any one exact interface name.

---

## Decision rules

When you are unsure, ask these in order:

1. Is this a stable reusable framework-owned graph?
   - If yes, it might be a lane.
2. Is this the robot's shared public vocabulary for multiple modes?
   - If yes, it probably belongs in a capability family.
3. Is this mapping human input into intent?
   - If yes, it belongs in the TeleOp controls owner.
4. Is this composing a specific autonomous routine?
   - If yes, it belongs in an Auto plan/routine or the Auto OpMode.
5. Is this final actuator ownership?
   - If yes, it belongs in a subsystem.
6. Is this policy/orchestration above a subsystem?
   - If yes, it belongs in a supervisor.
7. Is this shared robot-specific reasoning?
   - If yes, it belongs in a service.

That sequence usually gets you to a clean answer fast.
