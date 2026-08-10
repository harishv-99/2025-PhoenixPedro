# Phoenix Glossary

These definitions describe the current framework. Terms are grouped by what a student encounters,
not by Java package.

## Program and loop

### `FtcRobotOpMode`

The ordinary FTC host. Student code overrides one `configure(RobotProgram)` method; the host owns
the final FTC callbacks and fail-stop behavior.

### `RobotProgram`

The declaration surface supplied during configuration. It retains services, bindings, one optional
root Task, outputs, one optional drive, presenters, and optional INIT/STOP policy.

### `LoopClock`

The program's one time and cycle identity. The host advances it once per OpMode cycle; other owners
read it.

### Prestart

One optional data-only INIT owner for selection and readiness. It freezes to `READY` or `BLOCKED`
at FTC START and never owns hardware.

### Service

An upstream lifecycle owner, such as sensing, localization, or a required vendor heartbeat. It runs
before controls and Tasks.

### Output

A downstream realization owner. A mechanism commonly implements `RobotProgram.Output` and updates
its private Plants after controls and Tasks have changed intent.

### Presenter

A read-only telemetry contributor. It adds rows without advancing state, clearing telemetry, or
calling `telemetry.update()`.

## Values and controls

### `Source<T>`

A value sampled with the shared `LoopClock`. `ScalarSource` and `BooleanSource` are primitive
specializations.

### Binding

A declared mapping from a source event, change, or level to an action. `program.bindings()` exposes
the registration-only surface; the managed program owns its update.

### `TaskBindings`

The program adapter that constructs and enqueues a fresh Task when an input event is accepted. The
shared FIFO `TaskRunner` starts it when it reaches the queue head.

### `ScalarTarget`

A persistent numeric command request. It stores intent; setting it does not itself write hardware.

### Frame-valued request

A command that is valid only when refreshed for the current loop, such as manual stick power.

### Held request

A persistent selection or mode that remains until another request changes it.

### Pending request

Work remembered until behavior can consume it, such as queued shots.

## Mechanisms

### Plant

The framework mechanism runtime. One final `PlantTargetResolver` selects a numeric target; the Plant
applies bounds, references, guards, control, and hardware output when its owner updates it.
`Plant.stop()` applies the realization's natural final stop and makes that Plant instance terminal;
another lifecycle uses a newly constructed Plant.

### `PlantTargetResolver`

The one final source-driven decision for a Plant target. `PlantTargets.exact(...)`,
`equivalentPositionsOf(...)`, `overlay(...)`, and advanced `plan(...)` construct common resolvers.

### Command target

The stable `ScalarTarget` owned by a command-backed Plant. Ordinary exact mechanisms can retrieve it
with `plant.commandTarget()` instead of retaining a duplicate peer target.

### Plant units

The units spoken by robot intent, Plant bounds, references, targets, and tolerances—for example,
inches or degrees.

### Native units

The device/controller representation, such as encoder ticks, ticks per second, or servo fraction.
Only APIs whose names say `Native` or a controller-native unit use them.

### Feedback-capable Plant

A Plant with meaningful measurement and `atTarget` behavior. Feedback-aware ScalarTasks require
both the explicit Plant and the exact command target it follows.

### Open-loop Plant

A Plant without sensor-based completion, such as direct power or set-and-hold servo position. Use a
time or another explicit condition instead of pretending it can report physical arrival.

## Behavior

### Task

A cooperative, non-blocking behavior advanced by the loop. Each Task instance is single-use and has
an explicit completion outcome and active-cancellation policy.

### Task factory

A method or `Supplier<Task>` that creates a fresh Task each time behavior should run.

### Root Task

The one optional top-level Task graph declared for an Auto or another start-owned program behavior.

### `OutputTask` / `OutputTaskRunner`

An advanced temporary-output pattern. The runner queues short output proposals; the mechanism
overlays its source into one final Plant target resolver.

## Drive and spatial data

### `DriveSignal`

A robot-centric translation and rotation command.

### `DriveSource`

A source of `DriveSignal` values. Controls and guidance compose drive intent upstream.

### `DriveCommandSink`

The final owner that submits drive commands and stops the drivetrain.

### Frame

The coordinate system in which a pose or vector is expressed. Phoenix robot frame is right-handed:
`+X` forward, `+Y` left, `+Z` up; positive yaw is counter-clockwise.

### `LoopTimestamp`

A captured time that retains its clock and reset identity. Sensor owners create it; consumers use
its freshness and comparison methods rather than carrying raw epoch values.

## Robot architecture

### Primitive

One narrow reusable framework capability, such as a gamepad drive source or pose estimator.

### Framework lane

A reusable owner of a stable multi-object resource/lifecycle graph, such as a vision or
localization lane. A lane is not a generic name for every helper.

### Field fact

Shared environment data, such as a fixed AprilTag layout, kept separate from sensor ownership and
game strategy.

### Capability family

A robot-owned, mode-neutral intent and status API used by both TeleOp and Auto.

### Controls owner

The robot-owned class that maps sticks and buttons to capability meanings.

### Mechanism / subsystem

The robot owner of one hardware capability, its final target resolvers, private Plants, update
order, status, and stop behavior.

### Supervisor

A robot-owned policy coordinator. It may own requests, queues, gates, or state machines, but it does
not become a competing final Plant writer.

### Composition root

The class that snapshots configuration, constructs owners, connects dependencies, and declares the
program order without becoming the control script.

### Profile

Data-only robot hardware and tuning configuration. Long-lived owners defensively copy the slices
they retain.

[Cheat sheet](<Phoenix Cheat Sheet.md>) · [Reference index](<README.md>) · [Docs home](<../README.md>)
