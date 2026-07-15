# Tasks & Macros Quickstart

This guide explains Phoenix tasks in isolation. For how tasks fit into shared TeleOp + Auto robot design, also read [`Recommended Robot Design`](<Recommended Robot Design.md>) and [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>).

This guide explains how to use **Tasks** in the Phoenix framework to build non‑blocking behaviors:

* TeleOp **macros** (e.g., shooting sequences).
* **Autonomous routines** built out of reusable pieces.

We assume you already have a `PhoenixRobot` wired as in the Beginner’s Guide:

* A `LoopClock` for timing.
* `Gamepads` and `Bindings` for input.
* Drive and mechanisms modeled as `DriveSource` / `MecanumDrivebase` and `Plant`s.

Everything here is **non‑blocking** – there is no `sleep()` and no `while` loops that stall TeleOp.

---

## Related: spatial predicates (zones, safety gating)

Not every decision needs to be a task.

If you want a simple safety gate like “enable shooting only when the robot is in the shooting zone”, use the **spatial predicate** layer:

* `ConvexRegion2d` / `ConvexRegions2d` to describe the zone geometry (convex polygons, circles, AABBs)
* `RobotGeometry2d` + `RobotZones2d` to define what “robot in zone” means (point-in-zone, footprint overlap, fully inside)
* `ZoneLatch` to add hysteresis so the gate doesn’t chatter on a boundary
* `RobotHeadings2d` + `HeadingLatch` if you also want a direction/aim gate (“facing target?”)

These live in `edu.ftcphoenix.fw.spatial` and can be used in TeleOp, tasks, testers, or anywhere else.

---

## 1. The big picture: Tasks coordinate behavior

Phoenix code is built around three ideas:

1. **Tasks** – small, reusable behaviors that run over time.
2. **Plants** – things that accept a numeric target (motors, servos, etc.).
3. **TaskRunner** – drives a queue of tasks every loop.

You describe *what* should happen as a graph of tasks, and the framework figures out *when* each piece runs.

Typical flow in an OpMode:

```java
public class MyTeleOp extends OpMode {
    private final LoopClock clock = new LoopClock();
    private final TaskRunner runner = new TaskRunner();

    @Override
    public void start() {
        clock.reset(getRuntime());
    }

    @Override
    public void loop() {
        // 0) Advance the loop clock once per cycle.
        clock.update(getRuntime());

        // 1) Update bindings.
        // Gamepad axes/buttons are Sources; they are sampled when you call get(...).
        bindings.update(clock);

        // 2) Advance tasks.
        runner.update(clock);

        // 3) Apply the latest drive intent through one final writer.
        DriveSignal driveSignal = driveSource.get(clock).clamped();
        drivebase.update(clock);
        drivebase.drive(driveSignal);

        // 4) Apply the latest mechanism targets.
        double dtSec = clock.dtSec();
        shooter.update(clock);
        // ... etc.
    }
}
```

In ordinary TeleOp, Tasks do decision work, update sources, or request Plant targets. They run before
the one final `DriveSource` writer shown above; they do not also write imperatively to the same drive
sink.

The rest of this guide is about how to create those `Task` objects in a friendly way.

---

## 2. Core interfaces (Task, TaskRunner, TaskOutcome)

### 2.1 `Task`

At its core, a task looks like this:

```java
public interface Task {
    void start(LoopClock clock);
    void update(LoopClock clock);
    default void cancel() { }
    boolean isComplete();

    default TaskOutcome getOutcome() { ... }
}
```

Rules:

* `start(...)` and `update(...)` must **return quickly** (no blocking).
* A `Task` object is **single-use**: it may enter `start(...)` once. To repeat behavior, call the
  macro/builder again or use a `Supplier<Task>` so each run receives a fresh object. Framework
  Tasks throw a clear error on a second start; custom Task implementations must honor the same
  contract.
* Use fields inside the task to remember your own state.
* `isComplete()` becomes `true` when the task is done.
* `cancel()` is the cooperative early-stop hook. Before `start(...)` it is a side-effect-free
  no-op; while active it makes the Task terminal; after completion and on repeated calls it is a
  no-op.
* Calling `update(...)` directly before `start(...)` is a lifecycle error. Framework Tasks report
  it with an actionable exception instead of silently starting or mutating state. `Tasks.noop()`
  is the intentional exception because it is already successfully complete when created.
* `getOutcome()` lets tasks report *why* they finished (success, timeout, cancelled, etc.).
* If a custom task owns a duration or timeout, capture `clock.nowSec()` when that interval starts.
  On a first `update(...)` that shares the start cycle, the current `dtSec()` began before
  `start(...)` and must not be counted as task runtime. The advanced `RunForSecondsTask.onUpdate`
  callback receives that unchanged clock too, so callback-owned timers follow the same rule.

### 2.2 `TaskRunner`

`TaskRunner` is a simple scheduler:

* It keeps an internal queue of `Task`s.
* You call `runner.enqueue(task)` to add new work.
* It rejects the same object if that identity is already current or queued.
* Each loop you call `runner.update(clock)`, and it:

    * Starts the next task when the previous one completes.
    * Calls `start(...)` once and then `update(...)` every loop.

The first `update(...)` may happen immediately after `start(...)` in the same runner call. Phoenix's
timed factories account for that: a long loop immediately before scheduling does not consume the
new task's duration. Positive-duration output and Plant requests remain available to their later
realization phase for at least one loop. The exclusive timed-drive helper instead publishes directly
when its Task starts and on each active cycle, so a positive interval is observable without a
competing downstream drive writer. A zero-duration interval itself is immediate, though an
explicitly configured follow-up or cooldown still runs.

You almost never subclass `TaskRunner`. You just feed it tasks.

Use `cancelAndClear()` for driver override, mode switches, route interruption, and other total
aborts. It asks the active Task to stop, if there is one, and always discards every queued Task.
Queued Tasks have not acquired resources and do not receive a pre-start cancellation callback.
There is no abrupt queue-forgetting operation that can silently abandon the active Task.

The runners also fail closed on lifecycle `RuntimeException`s. If starting, updating, checking
completion, or cancelling active work throws one, the runner detaches and best-effort cancels that
work, clears all pending work, resets its state, and rethrows the original exception. A cleanup
exception is attached as suppressed rather than replacing the original failure.

Aborting a runner or cancelling a Task does not make a started Task reusable. If the behavior is
requested later, enqueue a new Task built by the same macro method or factory.

Example:

```java
bindings.onRise(gamepads.p1().b(), runner::cancelAndClear);
```

That calls the active Task's cooperative cancellation path first, then forgets any queued follow-up
work. The runner is empty afterward even if the cancellation hook fails.

---

## 3. Start with the factories: `Tasks` helpers

For most code, you should use the **factory methods** in `edu.ftcphoenix.fw.task.Tasks` instead of directly `new`‑ing task classes.

This keeps robot code readable:

```java
Task auto = Tasks.sequence(
    // Wait until shooter is spun up.
    Tasks.waitUntil(() -> shooterReady()),

    // This simple Auto is the only drive-command writer while the interval runs.
    DriveTasks.driveExclusivelyForSeconds(drivebase, forwardSignal, 0.8),

    // Small pause before the next step.
    Tasks.waitForSeconds(0.5)
);
```

Common factories (high‑level view):

* **Instant behavior**

    * `Tasks.runOnce(Runnable action)` – run once in `start(...)`, then complete.
    * `Tasks.noop()` – do nothing and complete immediately.

* **Time‑based waits**

    * `Tasks.waitForSeconds(double seconds)` – wait a fixed duration.

* **Condition‑based waits**

    * `Tasks.waitUntil(BooleanSupplier condition)` – wait until a boolean becomes true.
    * `Tasks.waitUntil(BooleanSupplier condition, double timeoutSec)` – wait until true or time out.

* **Composition**

    * `Tasks.sequence(Task... steps)` – run tasks one after another.
    * `Tasks.parallelAll(Task... steps)` – run tasks in parallel and finish when all are done.
    * `Tasks.parallelDeadline(Task deadline, Task... companions)` – run bounded companions only
      while one named Task is active; that deadline determines completion and outcome.
    * `Tasks.withTimeout(Task task, double timeoutSec)` – impose one hard outer time budget on a
      complete Task or Task graph.

> `Tasks.*` is the one public construction layer for generic composition, including `sequence(...)`,
> `parallelAll(...)`, `parallelDeadline(...)`, `withTimeout(...)`, and
> `branchOnOutcome(...)`. Public lower-level leaf
> Tasks such as `InstantTask`, `RunForSecondsTask`, and `WaitUntilTask` remain available. Prefer the
> corresponding `Tasks` helper when it covers the call; use a concrete leaf only for distinct
> callback, configuration, or status access.

### 3.1 Choose the parallel owner explicitly

Use `parallelAll(...)` when every child is required before the group can finish. Use
`parallelDeadline(...)` when one Task owns the lifetime of bounded companion work:

```java
Task collectAlongRoute = Tasks.parallelDeadline(
        followRoute,
        collectWhileMoving
);
```

The first argument is called the deadline because it owns completion, not because it must be a
timer. When `followRoute` ends, its outcome becomes the group's outcome and the composite asks the
start-attempted companions to cancel before they receive another update. A companion that finishes
early does not end the route.

Each companion must already make active cancellation safe. Do not use
`Tasks.sequence(enable, wait, disable)` as a companion: cancelling that sequence skips `disable`.
Build a bounded robot macro whose own `cancel()` restores its caller-selected state, or keep a
long-lived flywheel/intake/aim request as ordinary capability or service state.

Route failure policy is also deliberately outside generic composition. `Tasks.sequence(...)` does
not stop merely because a child route timed out or reported a cancellation-like terminal outcome,
and broad `TaskOutcome` values do not replace a route's precise status. Keep the returned
`RouteTask`, gate position-dependent work in a robot-owned routine, and explicitly choose whether a
non-normal status continues, starts a fallback, or aborts. Direct cancellation never starts the
fallback. Cleanup belongs to the phase that created the request and goes through robot capabilities,
not direct Plant or hardware writes.

### 3.2 Put a continuation outside its time budget

Use `withTimeout(...)` when a parent owns a hard budget around one complete Task or composite. When
the continuation is valid after every non-throwing terminal outcome of that Task, an autonomous
routine can keep it after the timed region:

```java
Task auto = Tasks.sequence(
        Tasks.withTimeout(preParkWork, 25.0),
        parkFromCurrentPose
);
```

If `preParkWork` finishes at 18 seconds, the park starts then. If it is still active at 25 seconds,
the wrapper calls its ordinary active `cancel()` and starts the park only after that cleanup returns
and the child is terminal. Once the park starts, no timer remains to interrupt or restart it.
Cancelling the outer sequence (including FTC STOP) does not start the park. This is a bounded
continuation, not Java `finally`; mandatory physical cleanup belongs in the active Task's
`cancel()` implementation.

If route status or another domain result can suppress the continuation, keep the same timed A/B
shape inside a robot-owned policy coordinator instead of relying on bare `sequence(...)` to decide.
Phoenix does this so interruption, replacement, cancellation, failure, and unknown route endings do
not start its park.

An outer timeout is intentionally different from operation-owned timeouts. A route-local timeout
can retain `RouteStatus.TASK_TIMEOUT`; a feedback move can apply its timeout target; a gated output
can time only its RUN phase. `withTimeout(...)` instead uses the child's normal cancellation path,
so the wrapper may report `TIMEOUT` while the retained child reports `CANCELLED`. Keep both when
they protect different scopes, but do not configure two copies of the same policy.

---

## 4. Mechanisms: `PlantTasks` and `ScalarTasks` for common patterns

For mechanism plants (motors, servos, shooters, arms, etc.), start with `edu.ftcphoenix.fw.actuation.PlantTasks`.

The source-driven plant rule is:

```text
Task writes the plant's registered ScalarTarget
    ↓
Plant samples its final PlantTargetSource each loop
    ↓
Plant target guards protect hardware
    ↓
Plant reports atTarget(requestedValue) when feedback confirms arrival
```

`PlantTasks` retrieves the writable target from the `Plant` itself. That is safer than passing a separate `ScalarTarget` into every task: the task cannot accidentally write a different target variable than the one the plant actually follows.

### 4.1 Guided writes for time-based commands

Use `PlantTasks.write(plant)` for open-loop pulses and simple target writes. These work with feedback and non-feedback plants.

```java
Task intakePulse = PlantTasks.write(intake)
        .to(+1.0)
        .forSeconds(0.7)
        .then(0.0)
        .build();
```

Behavior:

* At start: write `+1.0` to the plant's registered target.
* For `0.7` seconds: keep writing `+1.0`.
* When time elapses: write `0.0` once and complete.

The `.then(0.0)` also runs if this timed write is cancelled while active. That makes the registered
request return to zero; the Plant still applies that request on its next update. If the Task is
still queued and never started, discarding it has no target side effects.

The `0.7` seconds begin when this task starts. Even if the preceding loop was unusually long, the
new `+1.0` target is still available for the following `plant.update(clock)` call before a positive
duration may finish.

To hold and leave the target there:

```java
Task ensureSpinUp = PlantTasks.write(shooter)
        .to(SHOOTER_VELOCITY_NATIVE)
        .forSeconds(0.5)
        .leaveThere()
        .build();
```

Calling `build()` directly after `forSeconds(...)` has the same leave-there behavior. In either
form, normal completion and active cancellation keep the held request.

To set once and finish immediately:

```java
Task stopShooter = PlantTasks.write(shooter)
        .to(0.0)
        .build();
```

The compact helpers still exist for short code:

```java
PlantTasks.holdTargetForThen(intake, +1.0, 0.7, 0.0);
PlantTasks.setTarget(shooter, 0.0);
```

### 4.2 Guided feedback moves

Use `PlantTasks.move(plant)` when the plant has feedback and the task should wait for physical arrival.

```java
Task spinUp = PlantTasks.move(shooter)
        .to(SHOOTER_VELOCITY_NATIVE)
        .cancelTo(0.0)
        .timeout(1.5)
        .build();
```

Behavior:

* On start: write the requested target to the plant's registered target.
* Each loop: check `plant.atTarget(SHOOTER_VELOCITY_NATIVE)`.
* If a behavior overlay, clamp, fallback, or target guard keeps the plant from truly following that value, the task does **not** complete early.
* If the timeout elapses first, the task completes with `TaskOutcome.TIMEOUT`.
* If actively cancelled, the registered shooter request changes to `0.0` once.

For readiness that must remain stable for a short period:

```java
Task spinUpStable = PlantTasks.move(shooter)
        .to(SHOOTER_VELOCITY_NATIVE)
        .leaveTargetOnCancel()
        .stableFor(0.15)
        .timeout(1.5)
        .build();
```

Every feedback move must choose one cancellation behavior immediately after `.to(...)`:

* `.cancelTo(value)` writes that finite value, in the Plant's units, to its registered target when
  an active move is cancelled.
* `.leaveTargetOnCancel()` deliberately leaves the move request unchanged, so motion may continue.

Neither choice is a direct hardware stop. The Plant's next update still resolves overlays and
applies bounds, references, and guards. The robot owner must still cancel related queues, disable
overlays, and reset every mechanism request needed for coordinated shutdown.

To request a final target after success or timeout:

```java
Task moveAndStow = PlantTasks.move(arm)
        .to(ARM_SCORE_POS)
        .cancelTo(ARM_STOW_POS)
        .timeout(1.0)
        .thenTarget(ARM_STOW_POS)
        .build();
```

Here `.thenTarget(...)` handles success or timeout, while `.cancelTo(...)` independently handles
active cancellation.

> If you accidentally call a feedback move on an open-loop plant, `PlantTasks` throws an exception at runtime. Use `PlantTasks.write(...)` for timed open-loop behavior.

### 4.3 Standalone scalar targets

Use `ScalarTasks` when the writable value is not attached to a plant, such as a behavior variable or a command target that feeds a larger source graph.

```java
Task setShotVelocity = ScalarTasks.write(selectedVelocity)
        .to(3200.0)
        .build();
```

---


## 5. TeleOp macros: putting it together

A typical TeleOp macro flows like this:

1. Driver presses a button.
2. A macro `Task` is created (often a `Tasks.sequence(...)`).
3. That task is enqueued into the `TaskRunner`.
4. Each loop, `runner.update(clock)` advances the macro.

### 5.1 Wiring a simple shooter macro

Imagine you have these `Plant`s:

* `shooter` – a velocity plant for the flywheel (has feedback).
* `transfer` – a power plant that feeds discs.

A simple “shoot one disc” macro could look like:

```java
private Task createShootOneDiscMacro() {
    Task spinUp = PlantTasks.move(shooter)
            .to(SHOOTER_VELOCITY_NATIVE)
            .cancelTo(0.0)
            .timeout(SHOOTER_SPINUP_TIMEOUT_SEC)
            .build();

    Task feed = PlantTasks.write(transfer)
            .to(TRANSFER_POWER_SHOOT)
            .forSeconds(TRANSFER_PULSE_SEC)
            .then(0.0)
            .build();

    Task holdBeforeSpinDown = PlantTasks.write(shooter)
            .to(SHOOTER_VELOCITY_NATIVE)
            .forSeconds(SHOOTER_SPINDOWN_HOLD_SEC)
            .build();

    Task spinDown = PlantTasks.write(shooter)
            .to(0.0)
            .build();

    return Tasks.sequence(
        spinUp,
        feed,
        holdBeforeSpinDown,
        spinDown
    );
}
```

Bind a button to enqueue this macro:

```java
bindings.onRise(shootButton, () -> {
    runner.enqueue(createShootOneDiscMacro());
});
```

The rest of your TeleOp loop just calls `bindings.update(clock)` and `runner.update(clock)`.

---

## 6. Autonomous routines with Tasks

The same patterns work in Autonomous – you just enqueue one big task (usually a sequence) in `init()` or `start()` and let it run.

For a simple open-loop Auto or drive tester, the exclusive timed-drive helper is appropriate only
when no route Task, guidance Task, or separate final `DriveSource` loop is also writing behavior
commands to the same sink:

```java
Task auto = Tasks.sequence(
    // 1. Drive off the starting line.
    DriveTasks.driveExclusivelyForSeconds(drivebase, forwardSignal, 1.0),

    // 2. Wait until shooter is ready.
    Tasks.waitUntil(() -> shooterReady()),

    // 3. Shoot three discs with a macro.
    createShootThreeDiscMacro()
);

runner.enqueue(auto);
```

`driveExclusivelyForSeconds(...)` refreshes the sink and writes the requested signal on every active
cycle, then stops the sink when the interval completes or is actively cancelled. If an adapter's
supported lifecycle requires updates beyond active Tasks, its composition root continues calling
`update(clock)` with the shared `LoopClock`; same-cycle deduplication makes the Task's call safe.
Timed open-loop drive is not normal Pedro route movement—use `RouteTasks` or guidance Tasks for
those supported lifecycles.

Because everything is non‑blocking, your loop can also update telemetry, vision, etc., while the auto sequence runs.

---

## 7. When to use a lower-level or custom Task

For **most** robots, you only need:

* `Tasks.*` factory methods.
* `PlantTasks.*` helpers.
* `DriveTasks.driveExclusivelyForSeconds(...)` only for simple Auto/test movement with exclusive
  drive-sink ownership.

Some public leaf Task classes remain available when you need extra control. Prefer the matching
`Tasks` helper when it already expresses the behavior; for example, use `Tasks.runOnce(...)` rather
than constructing `InstantTask` merely for a different spelling. Concrete leaf classes are useful
when they expose a distinct capability:

* `RunForSecondsTask` – when you want full control over what happens during the time window (custom callbacks each loop).
* `WaitUntilTask` – when you need its concrete timeout/status inspection beyond the facade.

Even inside a team-specific helper factory, compose child Tasks through `Tasks.*`, such as
`sequence(...)`, `parallelAll(...)`, `parallelDeadline(...)`, `withTimeout(...)`, or
`branchOnOutcome(...)`. Implement
`Task` directly only when the behavior genuinely needs a new state machine rather than another
spelling of existing composition.

A good rule of thumb:

> Try the factories (`Tasks`, `PlantTasks`, and the narrowly scoped `DriveTasks` helper) first. If you find yourself rewriting the same pattern many times using raw `Task` classes, wrap it in a new helper method so the next student can just call the helper.

---

## 8. Output tasks and queues

Sometimes you want **continuous logic** (e.g. staging balls, holding a gate, keeping a mechanism ready)
*and* short **pulses** (feed one ball, spit out wrong color, etc.).

If both pieces of code try to own the same target variable, they will fight.

Phoenix provides a clean target-source ownership pattern:

- **`OutputTask`** — a `Task` that produces a scalar output (`getOutput()`).
- **`OutputTaskRunner`** — runs `OutputTask`s sequentially and exposes the active output as a `ScalarSource`. Use `cancelAndClear()` when you need to abort the active output task cleanly. This is the right default for feed queues, pulse queues, and “repeat while held” helpers because it lets the current task stop cooperatively, always clears the queue, and returns the source to its configured idle output even when task lifecycle cleanup fails.

That lets your subsystem loop decide the final plant target in one place:

```java
OutputTaskRunner feederQueue = new OutputTaskRunner(0.0);

// In init: define sensor gates as BooleanSource / ScalarSource.
BooleanSource fireAllowed = shooterReady.and(aimLocked).and(ballAtGate);
BooleanSource requestShoot = gamepads.p2().rightTrigger().above(0.50);
BooleanSource ballLeftGate = ballAtGate.fallingEdge();

// In your loop:
// Keep one "feedOne" buffered while the driver requests shooting.
// The task waits in WAIT until fireAllowed is true.
feederQueue.whileHigh(
        clock,
        requestShoot,
        1,
        () -> Tasks.gatedOutputUntil(
                "feedOne",
                fireAllowed,
                ballLeftGate,
                0.90,
                0.05,
                0.30
        )
);
ScalarSource base = ScalarSource.constant(0.0);
PlantTargetSource finalTarget = PlantTargets.overlay(base)
        .add("feedPulse", feederQueue.activeSource(), feederQueue)
        .build();

// Build the Plant with finalTarget during init, then the loop only updates it.
transferShooterPlant.update(clock);
```

For the full design rationale and more examples, see [`Output Tasks & Queues`](<Output Tasks & Queues.md>) and [`Recommended Robot Design`](<Recommended Robot Design.md>).

---

## 9. Common gotchas

* **Do not block** inside `Task.start(...)` or `Task.update(...)`.

    * Never call `Thread.sleep(...)` or spin in `while(!condition)` loops.
    * Use `Tasks.waitForSeconds(...)` or `Tasks.waitUntil(...)` instead.

* **Do not save and restart a Task object.**

    * Task instances are single-use, including sequences and parallel groups.
    * Save a macro method, `Supplier<Task>`, or `OutputTaskFactory` and ask it for a fresh task each
      time instead.

* **Always call `runner.update(clock)` once per loop.**

    * If you forget this, tasks will never progress.

* **Use feedback‑based helpers only on feedback plants.**

    * `PlantTasks.move(...)` requires `plant.hasFeedback() == true`.
    * For servos and other open-loop outputs, use `PlantTasks.write(plant)` or compact helpers such as `holdTargetFor(...)`, `holdTargetForThen(...)`, and `setTarget(...)`.

* **Be intentional about completion and cancellation targets.**

    * `PlantTasks.write(plant).to(...).forSeconds(...).build()` keeps the same target after the timer.
    * `.then(...)` or compact `holdTargetForThen(...)` sets a different final target after time
      elapses and on active cancellation.
    * Every feedback move requires `.cancelTo(...)` or `.leaveTargetOnCancel()` immediately after
      `.to(...)`; `.thenTarget(...)` remains the success/timeout choice.

---

## 10. Summary

* **`Task`** is the basic unit of non‑blocking behavior over time.
* **`TaskRunner`** manages a queue of tasks and advances them with `update(clock)`.
  `cancelAndClear()` is the total-abort operation and lifecycle failures clear owned work before
  they are rethrown.
* **`Tasks` factories** (`runOnce`, `waitForSeconds`, `waitUntil`, `sequence`, `parallelAll`,
  `parallelDeadline`, `withTimeout`, `noop`, ...) are the main building blocks you should reach for
  first.
* **`PlantTasks`** provide common mechanism patterns: `write(plant)` for time-based writes and `move(plant)` for feedback-based moves.
* **`DriveTasks.driveExclusivelyForSeconds(...)`** provides simple timed open-loop Auto/test movement
  when its Task is the sole behavior-command writer for the drive sink.
* Public lower-level leaf Tasks remain available, but prefer their `Tasks` helper unless the
  concrete type exposes distinct callback, configuration, or status control; generic composition
  still goes through `Tasks.*`.
* TeleOp macros and Autonomous routines both use the same task patterns; only the triggers change (buttons vs. init/start).

Once you are comfortable with these patterns, you can layer in more advanced pieces like vision, DriveGuidance (auto-aim / go-to), and interpolated shooter speeds – they all compose naturally on top of the same Task/Plant/Drive structure.
