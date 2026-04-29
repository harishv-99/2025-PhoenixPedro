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

## 1. The big picture: Tasks drive everything

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

        // 3) Apply the latest targets.
        drivebase.update(clock);
        double dtSec = clock.dtSec();
        shooter.update(clock);
        // ... etc.
    }
}
```

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
* Use fields inside the task to remember your own state.
* `isComplete()` becomes `true` when the task is done.
* `cancel()` is the cooperative early-stop hook. Use it when automation should stop cleanly.
* `getOutcome()` lets tasks report *why* they finished (success, timeout, cancelled, etc.).

### 2.2 `TaskRunner`

`TaskRunner` is a simple scheduler:

* It keeps an internal queue of `Task`s.
* You call `runner.enqueue(task)` to add new work.
* Each loop you call `runner.update(clock)`, and it:

    * Starts the next task when the previous one completes.
    * Calls `start(...)` once and then `update(...)` every loop.

You almost never subclass `TaskRunner`. You just feed it tasks.

Two queue-management methods matter in practice:

* `clear()` — forget the current task and queued tasks immediately without calling task cancellation hooks.
* `cancelAndClear()` — ask the active task to stop cleanly, then clear the queue.

For driver override, mode switches, and route interruption, prefer `cancelAndClear()`.

Example:

```java
bindings.onRise(gamepads.p1().b(), runner::cancelAndClear);
```

That calls the task's cooperative cancellation path first, then forgets any queued follow-up work.

---

## 3. Start with the factories: `Tasks` helpers

For most code, you should use the **factory methods** in `edu.ftcphoenix.fw.task.Tasks` instead of directly `new`‑ing task classes.

This keeps robot code readable:

```java
Task auto = Tasks.sequence(
    // Wait until shooter is spun up.
    Tasks.waitUntil(() -> shooterReady()),

    // Drive forward for 0.8 seconds.
    DriveTasks.driveForSeconds(drivebase, forwardSignal, 0.8),

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

> Under the hood these factories use the core classes: `InstantTask`, `RunForSecondsTask`, `WaitUntilTask`, `SequenceTask`, and `ParallelAllTask`. You *can* use those directly, but the factory methods keep TeleOp and Auto code cleaner.

---

## 4. Mechanisms: `PlantTasks` and `ScalarTasks` for common patterns

For mechanism plants (motors, servos, shooters, arms, etc.), start with `edu.ftcphoenix.fw.actuation.PlantTasks`.

The source-driven plant rule is:

```text
Task writes the plant's registered ScalarTarget
    ↓
Plant samples its final ScalarSource each loop
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

To hold and leave the target there:

```java
Task ensureSpinUp = PlantTasks.write(shooter)
        .to(SHOOTER_VELOCITY_NATIVE)
        .forSeconds(0.5)
        .build();
```

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
        .timeout(1.5)
        .build();
```

Behavior:

* On start: write the requested target to the plant's registered target.
* Each loop: check `plant.atTarget(SHOOTER_VELOCITY_NATIVE)`.
* If a behavior overlay, clamp, fallback, or target guard keeps the plant from truly following that value, the task does **not** complete early.
* If the timeout elapses first, the task completes with `TaskOutcome.TIMEOUT`.

For readiness that must remain stable for a short period:

```java
Task spinUpStable = PlantTasks.move(shooter)
        .to(SHOOTER_VELOCITY_NATIVE)
        .stableFor(0.15)
        .timeout(1.5)
        .build();
```

To request a final target after success or timeout:

```java
Task moveAndStow = PlantTasks.move(arm)
        .to(ARM_SCORE_POS)
        .timeout(1.0)
        .thenTarget(ARM_STOW_POS)
        .build();
```

The compact helpers remain available:

```java
PlantTasks.moveTo(shooter, SHOOTER_VELOCITY_NATIVE, 1.5);
PlantTasks.moveToThen(arm, ARM_SCORE_POS, 1.0, ARM_STOW_POS);
```

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

Example sketch:

```java
Task auto = Tasks.sequence(
    // 1. Drive off the starting line.
    DriveTasks.driveForSeconds(drivebase, forwardSignal, 1.0),

    // 2. Wait until shooter is ready.
    Tasks.waitUntil(() -> shooterReady()),

    // 3. Shoot three discs with a macro.
    createShootThreeDiscMacro(),

    // 4. Finally, stop the drivebase.
    DriveTasks.stop(drivebase)
);

runner.enqueue(auto);
```

Because everything is non‑blocking, your loop can also update telemetry, vision, etc., while the auto sequence runs.

---

## 7. When to drop down to raw task classes

For **most** robots, you only need:

* `Tasks.*` factory methods.
* `PlantTasks.*` helpers.
* `DriveTasks.*` helpers.

However, the core task classes are still available when you need extra control:

* `InstantTask` – when you want to inject custom one‑shot behavior.
* `RunForSecondsTask` – when you want full control over what happens during the time window (custom callbacks each loop).
* `WaitUntilTask` – when you need a “wait until X” primitive not covered by your factories.
* `SequenceTask` / `ParallelAllTask` – when you want to build your own higher‑level factories.

A good rule of thumb:

> Try the factories (`Tasks`, `PlantTasks`, `DriveTasks`) first. If you find yourself rewriting the same pattern many times using raw `Task` classes, wrap it in a new helper method so the next student can just call the helper.

---

## 8. Output tasks and queues

Sometimes you want **continuous logic** (e.g. staging balls, holding a gate, keeping a mechanism ready)
*and* short **pulses** (feed one ball, spit out wrong color, etc.).

If both pieces of code try to own the same target variable, they will fight.

Phoenix provides a clean, single-writer pattern:

- **`OutputTask`** — a `Task` that produces a scalar output (`getOutput()`).
- **`OutputTaskRunner`** — runs `OutputTask`s sequentially and exposes the active output as a `ScalarSource`. Use `cancelAndClear()` when you need to abort the active output task cleanly. This is the right default for feed queues, pulse queues, and “repeat while held” helpers because it lets the current task stop cooperatively before the queue is cleared.

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
ScalarSource finalTarget = feederQueue.activeSource().choose(feederQueue, base);

// Build the Plant with finalTarget during init, then the loop only updates it.
transferShooterPlant.update(clock);
```

For the full design rationale and more examples, see [`Output Tasks & Queues`](<Output Tasks & Queues.md>) and [`Recommended Robot Design`](<Recommended Robot Design.md>).

---

## 9. Common gotchas

* **Do not block** inside `Task.start(...)` or `Task.update(...)`.

    * Never call `Thread.sleep(...)` or spin in `while(!condition)` loops.
    * Use `Tasks.waitForSeconds(...)` or `Tasks.waitUntil(...)` instead.

* **Always call `runner.update(clock)` once per loop.**

    * If you forget this, tasks will never progress.

* **Use feedback‑based helpers only on feedback plants.**

    * `PlantTasks.move(...)` and the compact `moveTo*` helpers require `plant.hasFeedback() == true`.
    * For servos and other open-loop outputs, use `PlantTasks.write(plant)` or compact helpers such as `holdTargetFor(...)`, `holdTargetForThen(...)`, and `setTarget(...)`.

* **Be intentional about final targets.**

    * `PlantTasks.write(plant).to(...).forSeconds(...).build()` keeps the same target after the timer.
    * `.then(...)` or compact `holdTargetForThen(...)` lets you explicitly set a different final target (often zero).

---

## 10. Summary

* **`Task`** is the basic unit of non‑blocking behavior over time.
* **`TaskRunner`** manages a queue of tasks and advances them with `update(clock)`. Use `cancelAndClear()` when you want cooperative interruption instead of abrupt forgetting.
* **`Tasks` factories** (`instant`, `waitForSeconds`, `waitUntil`, `sequence`, `parallelAll`, `noop`, ...) are the main building blocks you should reach for first.
* **`PlantTasks`** provide common mechanism patterns: `write(plant)` for time-based writes and `move(plant)` for feedback-based moves.
* **`DriveTasks`** provide simple drive‑related building blocks.
* The core task classes – `InstantTask`, `RunForSecondsTask`, `WaitUntilTask`, `SequenceTask`, `ParallelAllTask` – are there when you need lower‑level control.
* TeleOp macros and Autonomous routines both use the same task patterns; only the triggers change (buttons vs. init/start).

Once you are comfortable with these patterns, you can layer in more advanced pieces like vision, DriveGuidance (auto-aim / go-to), and interpolated shooter speeds – they all compose naturally on top of the same Task/Plant/Drive structure.
