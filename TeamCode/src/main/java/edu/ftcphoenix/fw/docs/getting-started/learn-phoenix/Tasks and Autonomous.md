# Tasks and autonomous

**Question:** How can behavior continue across loop cycles without blocking the robot?

**Reading time:** about 8 minutes

A Phoenix `Task` is cooperative work advanced by the one managed loop. It never needs `sleep()` or
a long-running `while` loop. Reading this page does not require editing code, enabling an example,
or moving hardware.

## Start with one timed intake

Starter Auto declares the same intake capability used by TeleOp, then chooses its routine:

```java
StarterIntake intake = new StarterRobot(hardwareMap)
        .declareAuto(program, profile);
program.rootTask(intake.collectForSeconds(0.75));
```

[`StarterAuto.java`](<../../../../robots/examples/starter/opmode/StarterAuto.java>) owns that routine
choice. `StarterRobot` only declares the intake output and presenter. At START, the fresh root Task
requests `Mode.COLLECT` through the same `setMode(...)` method used by TeleOp. The mechanism maps
that named request to configured power, and managed output updates realize it through the private
Plant. After 0.75 seconds, the Task requests `Mode.STOPPED`. Active cancellation also restores that
stopped mode, and FTC STOP cancels active work before stopping declared outputs.

This is temporal intent, not a second hardware writer: the Task changes the mechanism's semantic
request; the mechanism remains the one object that maps it, updates the Plant, and stops the Plant.
Use `ScalarTasks` directly when the capability request itself is a number, such as flywheel
velocity. The Starter uses `setMode(...)` because its public request and status are named modes.

## Every run needs a fresh Task

Each `Task` instance may start only once. Calling `collectForSeconds(...)`, `home()`, `moveTo(...)`,
or `launchOne()` is calling a factory method that returns new work. Call it again for another run;
never save and restart the old object.

That is why a repeatable TeleOp macro binds a supplier:

```java
tasks.onRise(gamepad.x(), lift::home);
tasks.onRise(gamepad.y(), launcher::launchOne);
callbacks.onRise(gamepad.b(), launcher::abortLaunches);
```

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

A timeout reports evidence; it does not silently choose recovery. The Reference routine explicitly
aborts launcher work after a home, move, or launch timeout and retains `TIMEOUT`. A bare
`Tasks.sequence(...)` retains a child's timeout but can still advance, so use an outcome branch when
the next step requires success. The checked-in timeout values are software configuration, not proof
that the real mechanism is fast or safe. The launcher feeds only after its readiness wait succeeds;
timeout clears temporary requests without feeding.

Cancellation before start has no effect. Active cancellation is terminal and idempotent; repeated
or terminal cancellation does nothing, and cancelled work still cannot be reused. For the Reference
lift, timeout or active cancellation leaves its selected persistent height request in place. It
does not bypass the source graph with a direct hardware write, and direct cancellation never starts
a later fallback branch.

## Check your understanding

**Auto must wait until the lift physically reaches HIGH before launching. Is `setHeight(HIGH)`
enough?**

No. It only replaces the persistent request. Build a fresh feedback-aware `moveTo(HIGH)` Task and
continue to launch only from its successful outcome.

## Go deeper when needed

- Complete Starter source: [`StarterAuto.java`](<../../../../robots/examples/starter/opmode/StarterAuto.java>)
- Scaling example: [`ReferenceAutoRoutines.java`](<../../../../robots/examples/reference/autonomous/ReferenceAutoRoutines.java>)
- Task factories, composition, timing, and cancellation: [Tasks and Macros](<../../design/Tasks & Macros Quickstart.md>)
- [Choose another Phoenix topic](<../Beginner's Guide.md>)
