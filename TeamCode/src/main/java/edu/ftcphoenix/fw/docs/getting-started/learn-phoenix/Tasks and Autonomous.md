# Tasks and autonomous

**Question:** How can robot behavior continue across many loop cycles without blocking the rest of
the robot?

**Reading time:** about 20 minutes

A Phoenix `Task` is a small, cooperative behavior. The framework starts and updates it from the one
managed loop, so a Task never needs `sleep()` or a long-running `while` loop. TeleOp macros and Auto
routines use the same Task vocabulary.

This chapter is a source-reading exercise. You do not need to enable or run the Reference robot.

## Source map

- [`ReferenceRobot.java`](<../../../../robots/examples/reference/robot/ReferenceRobot.java>) declares
  the Auto hardware owners and returns their mode-neutral capabilities.
- [`ReferenceAutoRoutines.java`](<../../../../robots/examples/reference/autonomous/ReferenceAutoRoutines.java>)
  owns the Auto sequence and its outcome policy.
- [`ReferenceLauncher.java`](<../../../../robots/examples/reference/capability/launcher/ReferenceLauncher.java>)
  exposes the small fresh-Task and abort contract while keeping realization details private.
- [`ReferenceTeleOpControls.java`](<../../../../robots/examples/reference/robot/ReferenceTeleOpControls.java>)
  gives `TaskBindings` method references that create fresh Tasks on button rises.
- [`Tasks and Macros`](<../../design/Tasks & Macros Quickstart.md>) is the detailed API guide after
  this walkthrough.

## Trace the Reference Auto

The thin Auto selects a profile, asks the composition root to declare the active owners, and then
selects one routine:

```java
ReferenceProfile profile = ReferenceProfile.current();
ReferenceCapabilities capabilities =
        new ReferenceRobot(hardwareMap).declareAuto(program, profile);
program.rootTask(ReferenceAutoRoutines.homeMoveLowThenLaunch(capabilities));
```

The root constructs and registers resources. `ReferenceAutoRoutines` owns the strategy:

```text
RobotProgram starts the root Task
    -> lift.home() searches non-blockingly for the reference switch
       -> SUCCESS: lift.moveTo(LOW) requests LOW and waits for feedback
          -> SUCCESS: begin launcher.launchOne()
          -> TIMEOUT: abort launcher work; do not launch
       -> TIMEOUT: abort launcher work; do not move or launch
       -> CANCELLED: finish cancelled; do not start later work
    -> declared mechanism outputs realize the resulting requests each loop
```

The lift exposes both meanings deliberately:

```java
lift.setHeight(ReferenceLift.Height.LOW);  // replace the persistent request now
Task move = lift.moveTo(ReferenceLift.Height.LOW); // fresh Task that waits for feedback
```

TeleOp callbacks normally use `setHeight(...)`. Auto uses `moveTo(...)` when the next action must
wait for the cached Plant evidence. Calling `moveTo(...)` constructs the Task; the height request
changes only when the managed runner starts that Task. A move timeout is retained after cleanup
instead of being relabeled as a successful routine. Timeout or active cancellation leaves the
selected persistent height request in place; it does not bypass the mechanism's source graph with
a direct hardware write. The budget comes from the mechanism's copied
`ReferenceLiftMechanism.Config.moveTimeoutSec`; its checked-in value is not physical proof.

Outcome selection is policy, not exception handling hidden by the framework. The routine performs
its explicit abort cleanup after a home, move, or launch timeout while retaining the original
`TIMEOUT`. Direct cancellation does **not** start later or fallback work. At FTC STOP,
`RobotProgram` cancels active Task work and then stops the declared outputs.

## Follow the launcher capability

Students using the capability need its small public contract, not its private cleanup machinery:

- `SUCCESS` from the velocity wait selects `feed`.
- `TIMEOUT` clears its temporary requests without feeding and remains `TIMEOUT`.
- Active cancellation clears its temporary requests and ends `CANCELLED`; it does not select
  `feed`.
- `abortLaunches()` invalidates launch Tasks created before that abort and restores reusable
  active-match requests. A later `launchOne()` call still returns valid fresh work.

The wait requires a positive target and independent left/right ticks-per-second evidence for the
current launch target. The timeout proves only whether that controller condition was met within its
budget. It does not prove that a game piece launched or scored.

## A Task object is single-use

Every Task instance may enter `start(clock)` only once. A method such as `launchOne()` or `home()`
is a **factory method**: call it again to obtain another Task. Never save one Task and try to restart
it.

That is why TeleOp binds suppliers:

```java
        tasks.onRise(gamepad.x(), lift::home);

        tasks.onRise(gamepad.y(), launcher::launchOne);
        callbacks.onRise(gamepad.b(), launcher::abortLaunches);
```

Each rising edge asks the capability for a fresh Task. The callback binding is different:
`abortLaunches()` is synchronous, so it does not need a Task. Declaring Y before B makes a
same-cycle abort the last request and therefore the winner.

## Outcomes and cancellation

Use these meanings when reading a Task graph:

| Outcome | Meaning at the Task boundary |
|---|---|
| `NOT_DONE` | The outcome-aware Task is still active. |
| `SUCCESS` | It completed its documented condition normally. |
| `TIMEOUT` | Its time budget elapsed before that condition. |
| `CANCELLED` | Active work ended through cancellation or a documented fail-closed abort. |
| `UNKNOWN` | The Task does not expose a more specific result. |

Cancellation is active-only and idempotent: cancelling before start has no effect, cancelling an
active Task makes it terminal, and terminal or repeated cancellation does nothing. A cancelled Task
still cannot be reused. When a Task owns temporary output, its cancellation policy must make that
temporary behavior safe; terminal program cleanup remains responsible for stopping each output
owner.

Also remember that `Tasks.sequence(...)` orders children but does not invent robot policy. A child
timeout is retained in the sequence's aggregate outcome, yet a bare sequence still advances to its
next child. Use an explicit outcome branch when the next action depends on success, as the launcher
does.

## Copy, adapt, and leave alone

- **Copy the shape:** capability methods return fresh Tasks; the Auto client selects a routine from
  declared capabilities; controls bind Task suppliers.
- **Adapt the policy:** choose the real success condition, timeout response, cancellation target,
  and order for the team robot.
- **Leave lifecycle ownership alone:** let `RobotProgram` start, update, cancel, and clean up its
  runner. Do not add another OpMode loop or clock.

## Trace it

1. **Does Reference Auto wait for the lift to arrive at LOW before launching?**

   Yes. It sequences the fresh feedback-aware `moveTo(LOW)` Task before `launchOne()`.

2. **If flywheel spin-up times out, does the feed sequence run?**

   No. The launcher cleans its temporary requests and retains `TIMEOUT`.

3. **If FTC STOP cancels the launch while it is waiting, does the timeout branch run?**

   No. Direct cancellation is terminal and does not start a fallback branch. Managed cleanup then
   stops the declared mechanism outputs.

## Predict it

The driver presses Y, releases it, and later presses Y again. Does the second press restart the
first `Task` object?

No. `tasks.onRise(gamepad.y(), launcher::launchOne)` calls the method again, producing a fresh,
single-use Task for the new request.

**Previous:** [Plants and hardware](<Plants and Hardware.md>)

**Next:** [Evidence and experiments](<Evidence and Experiments.md>)
