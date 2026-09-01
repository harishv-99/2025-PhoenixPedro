# Robot Capabilities and Mode Clients

This document explains a robot-owned concept that will likely exist every year even though it does
**not** belong in the common framework as a lane: **capability families**.

Read this when you want to answer questions like:

- What API should both TeleOp and Auto talk to?
- How should a robot split that API into a few cohesive interfaces?
- What is parallel between the TeleOp side and the Auto side?
- How should TeleOp and Auto declare different graphs through the same managed lifecycle?
- How can a robot stay SOLID without exploding into dozens of tiny interfaces?

Useful companions:

- [`Basic Mechanisms Robot.md`](<../getting-started/Basic Mechanisms Robot.md#complete-source-and-owner-map>)
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
- decide what a non-completed route means for this routine

The Auto side does **not** need a universal `AutoControls` class just to mirror TeleOp.

---

## What should go in the managed program

Ordinary FTC robot code has one lifecycle spelling: extend `FtcRobotOpMode` and override
`configure(RobotProgram)`. TeleOp and Auto differ in the graph they declare, not in which lifecycle
methods a student remembers to forward.

### `constructor`

Keep the composition-root constructor resource-only. It may retain the stable FTC resource needed
to construct the selected graph; pass configuration and mode-active inputs synchronously to the
mode declaration instead.

The Reference example keeps that constructor literal:

```java
ReferenceRobot robot = new ReferenceRobot(hardwareMap);
```

`declareTeleOp(...)` receives one local profile and its active Gamepad. `declareAuto(...)` receives
the local profile and returns only its mode-neutral capabilities. The root retains no aggregate
profile, Telemetry, or dormant Gamepads. Avoid constructing half the robot or retaining mode-inactive
dependencies in the resource-only constructor.

### `configure(RobotProgram)`

Construct each truthful lifecycle owner and register it immediately:

- `program.service(...)` for upstream sensing, localization, policy computation, or a required
  vendor heartbeat;
- `program.output(...)` for each mechanism/subsystem that privately owns its final Plants;
- `program.callbackBindings()` or `program.taskBindings()` for TeleOp meanings;
- `program.drive(source, sink)` for the one final source-driven drive writer;
- `program.rootTask(...)` for one fresh Auto routine; and
- `program.presenter(...)` for additive INIT/active telemetry rows.

The graph freezes when `configure(...)` returns. Construct a new OpMode/program for another mode or
runtime rather than attempting a TeleOp/Auto cross-initialization.

TeleOp normally declares controls plus a drive source. Auto normally declares one fresh root Task
and any route service. Both call the same mode-neutral capability methods and status snapshots. The
specific follower configuration, route selection, and strategy remain in the Auto client, not in
the shared capability family or generic framework.

### Auto-to-TeleOp state is a separate process boundary

FTC creates a new OpMode and robot container for TeleOp. When that new runtime needs a small set of
facts from the completed Auto, use one robot-owned immutable snapshot carried by
`FtcAutoToTeleOpHandoff<T>`. The framework owns the typed, fresh, consume-once process slot; the
robot owns which facts cross, which Auto is eligible to publish, how TeleOp applies them, and what
missing/stale data means.

The safe lifecycle has separate operations:

1. clear pending state at Auto INIT;
2. after a successfully started match Auto, capture cached facts before shutdown clears their
   owners;
3. complete shutdown;
4. publish only after successful capture and shutdown;
5. initialize the new TeleOp robot, then consume/apply once before START.

Do not compress publication into the robot's hardware shutdown operation. A failed stop must not
make uncertain state available to TeleOp. Test/example modes should invalidate a pending
robot-specific handoff, and non-match Auto purposes should never publish.

Missing or stale data needs an explicit robot fallback. Keeping the normally initialized TeleOp
localizer is often safer than fabricating a zero field pose. The carrier uses process-monotonic age,
not either OpMode's `LoopClock`; it intentionally disappears on a Robot Controller process restart.
It is not a capability family, persistent configuration store, global state map, event bus, or
hardware-accuracy check.

See
[`FTC Auto-to-TeleOp Handoff`](<../ftc-boundary/FTC Auto-to-TeleOp Handoff.md>)
for the exact carrier contract and a neutral robot-owned snapshot example. A managed match Auto
registers `RobotProgram.stopHandoff(...)`; a later TeleOp consumes after its normal owners exist but
before START. Delivered values seed only the robot facts named by that wrapper, while every
missing, stale, or repeated-consumption result keeps an explicit fallback.

### Managed INIT, START, loop, and STOP

The final host owns these phases:

```text
INIT       reset clock -> configure/freeze graph -> prestart -> presenters -> one telemetry commit
INIT loop  clock -> prestart -> presenters -> one telemetry commit
START      freeze prestart -> reset clock -> service starts -> root start/first update -> outputs once
loop       clock -> services -> bindings -> Tasks -> outputs/drive -> presenters -> one commit
STOP       cancel Tasks -> clear bindings -> outputs in order -> services in reverse order
```

Capability methods only change robot intent or create fresh Tasks. Services own upstream updates;
mechanisms/subsystems own final Plant realization as outputs. An Auto root is declared once with
`program.rootTask(...)`, starts at the exact FTC boundary, and remains retained for cancellation and
terminal status. The program's private runner is not another object in the normal robot graph.
When prestart returns `BLOCKED`, START runs no services, Tasks, bindings, or outputs; later loops
advance only the clock and presenters.

On configuration or runtime `RuntimeException`, the same terminal cleanup runs and preserves the
original failure. An owner registered before a later configuration failure is still stopped, so
construct and transfer each truthful service/output as soon as its constructor returns.

If generic route or guidance Tasks also invoke the same stateful drive sink's update hook, that sink
must make repeated calls in one `LoopClock.cycle()` idempotent. The composition root remains the
recurring owner; Tasks select behavior and may perform a same-cycle no-op update for compatibility
with integrations whose work is genuinely Task-local.

The integration owner should also preserve each route's terminal meaning in the backend-neutral
`RouteExecution` returned by `RouteFollower.follow(...)`. `RouteTask.getRouteStatus()` lets the Auto
client distinguish `COMPLETED`, `FOLLOWER_TIMEOUT_OR_STALL`, `INTERRUPTED`, `REPLACED`,
`TASK_TIMEOUT`, `CANCELLED`, `FAILED`, and `UNKNOWN_TERMINAL` without exposing vendor types. The
adapter reports integration facts; the Task boundary may add its own timeout, cancellation, or
fail-closed `FAILED` status, so `FAILED` can originate at either boundary. The Auto client owns any
continue, fallback, or abort policy.

That policy must gate position-dependent capability work rather than relying on a generic sequence
to short-circuit. A conservative routine can choose a live-state fallback after timeout and abort on
cancellation-like results. Direct cancellation cancels the active phase and must not start recovery.
Any cleanup goes through the same robot capability family and clears only transient or held requests
owned by the failed phase, leaving unrelated mechanism intent to its actual owner.

### Maintained examples keep each approach separate

The Starter example shows the smallest parallel TeleOp and Auto clients. `StarterTeleOp` binds one
controls owner and final drive source; `StarterAuto` builds one fresh root Task from the same intake
capability. Both construct `StarterRobot` through `configure(program)`, and neither owns another
clock, runner, telemetry commit, or manual cleanup path.

The Reference example expands that same shape with lift and launcher capability families.
`ReferenceRobot.declareTeleOp(...)` binds driver meanings while
`ReferenceRobot.declareAuto(...)` returns `ReferenceCapabilities` for the selected routine. The
Basic Pedro example separately demonstrates a persistent vendor heartbeat, truthful route status,
and one Auto-only root. It does not make Pedro configuration part of the Reference or Starter
robots.

Each example is an independent application of the managed grammar. Copy the example closest to the
problem being solved rather than combining their profiles, roots, or lifecycle owners.

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
        TargetingStatus status();
        Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg);
    }
}
```

Here the targeting service publishes one status during the upstream service phase. Capability
clients and presenters read that immutable snapshot without advancing the service or supplying a
clock.

A matching robot shape usually looks like this:

```text
MyRobot
  ├─ framework lanes
  ├─ subsystems declared as RobotProgram outputs
  ├─ supervisors/services declared in the upstream service phase
  ├─ additive presenters
  ├─ MyCapabilities
  └─ MyTeleOpControls (TeleOp only)
```

And the mode clients look like this:

```text
TeleOp:
  MyTeleOpControls -> MyCapabilities -> robot internals

Auto:
  MyAutoPlan / MyAutoRoutine -> MyCapabilities -> robot internals
```

The maintained [`StarterIntake`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>)
shows the deliberately smaller one-family case. It is itself the shared capability used by
`StarterTeleOpControls` and `StarterAuto`; there is no one-member forwarding aggregate. Add an
aggregate only when a second cohesive capability family makes that grouping useful.

---

## A good checklist for each year's robot

When you start a new season, expect to name and place these kinds of objects explicitly:

- framework lanes for stable FTC-side graphs
- shared field facts
- subsystems as target-resolver owners
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
