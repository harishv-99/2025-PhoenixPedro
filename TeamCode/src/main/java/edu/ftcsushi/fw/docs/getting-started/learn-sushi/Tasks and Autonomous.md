---
tags:
  - Learn
---

# Tasks and autonomous

**Learning mode:** Architecture reference

These excerpts teach Task lifecycle and composition;
the Starter and Pedro buildable modules supply complete executable slices.

**Question:** How can behavior continue across loop cycles without blocking the robot?

**Reading time:** about 8 minutes

**Prerequisite:** the timed step in the [First software tour](<../First Software Tour.md>) introduces
the loop-by-loop idea without composition details.

Think of a bookmark in a multi-step robot action: each FTC loop does a little work, records where
to continue, and returns so the rest of the robot can update. Sushi calls that bookmark a `Task`.
It does not create a new thread and never needs `sleep()` or a long-running `while` loop. Reading
this page does not require editing code, enabling an example, or moving hardware.

## Start with one timed intake

### Critical code

Starter Auto declares the same intake capability used by TeleOp, then chooses its routine:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...
StarterIntake intake = new StarterRobot(hardwareMap)
        .declareAuto(program, profile);
program.rootTask(intake.collectForSeconds(0.75));
// ...
```

**What to notice**

- Auto reuses the same capability vocabulary as TeleOp.
- The OpMode selects one fresh root Task; the mechanism remains the hardware writer.

**Key APIs:** `program.rootTask(...)` declares the one managed Auto root; `Task` is cooperative,
single-use work.

[`StarterAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.html>) owns that routine
choice. `StarterRobot` only declares the intake output and presenter. At START, the fresh root Task
publishes `Mode.COLLECT` through the same semantic command owner used by TeleOp's `setMode(...)`.
The mechanism maps that named request to configured power, and managed output updates realize it
through the private Plant. After 0.75 seconds, the Task requests `Mode.STOPPED`. Active cancellation
also restores that stopped mode, and FTC STOP cancels active work before stopping declared outputs.

The mechanism uses the semantic Task builder:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
return SemanticScalarTasks.set(modeCommand, Mode.COLLECT)
        .forSeconds(durationSec)
        .then(Mode.STOPPED)
        .build();
```

[`SemanticScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarTasks.html>)
changes the command owner's request; the mechanism still maps it, updates the Plant, and stops the
Plant. Use `ScalarTasks` when the number itself is the capability request.

## Every run needs a fresh Task

### Critical code

Each `Task` instance may start only once. Calling `collectForSeconds(...)`, `home()`, `moveTo(...)`,
or `launchOne()` is calling a factory method that returns new work. Call it again for another run;
never save and restart the old object.

That is why a repeatable TeleOp macro binds a supplier:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
tasks.onRise(gamepad.x(), lift::home);
tasks.onRise(gamepad.y(), launcher::launchOne);
callbacks.onRise(gamepad.b(), launcher::abortLaunches);
// ...additional control meanings omitted...
```

**What to notice:** method references such as `lift::home` manufacture a new Task on every accepted
edge; they do not reuse a stored instance.

**Key APIs:** `TaskBindings.onRise(...)` accepts fresh-Task suppliers;
`CallbackBindings.onRise(...)` accepts synchronous callbacks.

Each X or Y rising edge constructs a fresh Task. B is a synchronous callback, so it calls the abort
method directly.

## Persistent requests and feedback Tasks answer different questions

`lift.setHeight(LOW)` replaces a persistent request and returns immediately. It is appropriate when
the caller does not need to wait. A fresh `lift.moveTo(LOW)` Task makes that request when it starts
and waits for cached Plant feedback, so Auto can decide whether to begin its next action.

The Reference Auto routine owns that strategy: home, move to LOW only after successful homing, then
launch only after a successful move. The capability and composition root do not choose the match
sequence.

Outcome-aware Tasks retain what happened:

- `NOT_DONE`: the Task is still active.
- `SUCCESS`: the documented completion condition was met.
- `TIMEOUT`: the budget elapsed first.
- `CANCELLED`: active work was cancelled or deliberately failed closed.
- `UNKNOWN`: no more specific result is available.

A timeout reports evidence; it does not silently choose recovery. Ordinary
`Tasks.sequence(...)` starts its next child only after exact `SUCCESS`, so it is the safe default
when each step is a prerequisite. The fixed child Tasks are constructed eagerly, but later children
do not start early.

The Reference routine has a different requirement: after a home, move, or launch timeout, it must
run an explicit launcher-abort repair and still retain `TIMEOUT`. It therefore uses
`Tasks.sequenceOnCompletion(...)`, examines the preceding outcome in a start-time-built policy
Task, and selects either the next motion or the repair. Direct cancellation and lifecycle failure
still start no later Task. Use `Tasks.branchOnOutcome(...)` for the simpler two-way case: exact
success selects its success branch, exact timeout selects its timeout branch, and `CANCELLED` or
`UNKNOWN` selects neither. The checked-in timeout values are software configuration, not proof that
the real mechanism is fast or safe. The launcher feeds only after its readiness wait succeeds;
timeout clears temporary requests without feeding.

Cancellation before start has no effect. Active cancellation is terminal and idempotent; repeated
or terminal cancellation does nothing, and cancelled work still cannot be reused. For the Reference
lift, timeout or active cancellation leaves its selected persistent height request in place. Its
`moveTo(...)` implementation makes that policy explicit with `leaveRequestOnCancel()`. It does not
bypass the source graph with a direct hardware write, and direct cancellation never starts a later
fallback branch.

## Check your understanding

**Auto must wait until the lift physically reaches HIGH before launching. Is `setHeight(HIGH)`
enough?**

No. It only replaces the persistent request. Build a fresh feedback-aware `moveTo(HIGH)` Task and
continue to launch only from its successful outcome.

## Go deeper when needed

- Complete Starter API: [`StarterAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.html>)
- Numeric feedback Task: [`ReferenceFlywheels.setVelocityTask(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.html#setVelocityTask(double,double)>)
- Outcome-aware mechanism Task: [`ReferenceLauncher.launchOne()`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncher.html#launchOne()>)
- Task factories, composition, timing, and cancellation: [Tasks and Macros](<../../design/Tasks & Macros Quickstart.md>)
- [Choose another Sushi topic](<../Beginner's Guide.md>)
