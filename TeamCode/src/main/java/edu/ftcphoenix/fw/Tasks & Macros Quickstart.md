# Tasks & Macros Quickstart

This guide explains how to use **Tasks** in the Phoenix framework to build non‑blocking behaviors:

* TeleOp **macros** (e.g., shooting sequences).
* **Autonomous routines** built out of reusable pieces.

We assume you already have a `PhoenixRobot` wired as in the Beginner’s Guide:

* A `LoopClock` for timing.
* `Gamepads` and `Bindings` for input.
* Drive and mechanisms modeled as `DriveSource` / `MecanumDrivebase` and `Plant`s.

Everything here is **non‑blocking** – there is no `sleep()` and no `while` loops that stall TeleOp.

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
    private final TaskRunner runner = new TaskRunner();

    @Override
    public void loop() {
        LoopClock clock = ...; // however you track dtSec

        // 1) Update inputs / bindings.
        bindings.update();

        // 2) Advance tasks.
        runner.update(clock);

        // 3) Let drive + plants use whatever targets the tasks set.
        drivebase.update(clock);
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
    boolean isComplete();

    default TaskOutcome getOutcome() { ... }
}
```

Rules:

* `start(...)` and `update(...)` must **return quickly** (no blocking).
* Use fields inside the task to remember your own state.
* `isComplete()` becomes `true` when the task is done.
* `getOutcome()` lets tasks report *why* they finished (success, timeout, etc.).

### 2.2 `TaskRunner`

`TaskRunner` is a simple scheduler:

* It keeps an internal queue of `Task`s.
* You call `runner.enqueue(task)` to add new work.
* Each loop you call `runner.update(clock)`, and it:

    * Starts the next task when the previous one completes.
    * Calls `start(...)` once and then `update(...)` every loop.

You almost never subclass `TaskRunner`. You just feed it tasks.

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

    * `Tasks.instant(Runnable action)` – run once in `start(...)`, then complete.
    * `Tasks.noop()` – do nothing and complete immediately.

* **Time‑based waits**

    * `Tasks.waitForSeconds(double seconds)` – wait a fixed duration.

* **Condition‑based waits**

    * `Tasks.waitUntil(BooleanSupplier condition)` – wait until a boolean becomes true.

* **Composition**

    * `Tasks.sequence(Task... steps)` – run tasks one after another.
    * `Tasks.parallelAll(Task... steps)` – run tasks in parallel and finish when all are done.

> Under the hood these factories use the core classes: `InstantTask`, `RunForSecondsTask`, `WaitUntilTask`, `SequenceTask`, and `ParallelAllTask`. You *can* use those directly, but the factory methods keep TeleOp and Auto code cleaner.

---

## 4. Mechanisms: `PlantTasks` for common patterns

For mechanism plants (motors, servos, shooters, arms, etc.), start with `edu.ftcphoenix.fw.actuation.PlantTasks`.

`PlantTasks` focuses on **two big categories**:

1. **Time‑based commands** – “run this plant at value X for N seconds”.
2. **Feedback‑based moves** – “move this plant to a setpoint and wait until it arrives”.

### 4.1 Time‑based helpers (work with any plant)

These work with *both* feedback and non‑feedback plants (for example, power plants, servo position plants, etc.).

* **Hold then change target**

  ```java
  Task t = PlantTasks.holdForThen(plant,
                                  +1.0,  // target during pulse
                                  0.7,   // seconds
                                  0.0);  // final target
  ```

  Behavior:

    * At start: set target to `+1.0`.
    * For `0.7` seconds: keep holding `+1.0`.
    * When time elapses: set target once to `0.0` and complete.

* **Hold and keep target**

  ```java
  Task t = PlantTasks.holdFor(plant, +1.0, 0.7);
  ```

  Behavior:

    * Same as above, **but** the final target stays at `+1.0`.
    * Think: “run for at least this long, but keep that command afterward.”

### 4.2 Feedback‑based move helpers (require feedback

These helpers rely on `plant.hasFeedback() == true` and `plant.atSetpoint()` being meaningful. They are designed for encoder‑backed motor plants (position or velocity) created via `Actuators.plant(...).motor(...).position(...)` or `.velocity(...)`.

* **Move to setpoint and hold**

  ```java
  Task spinUp = PlantTasks.moveTo(shooterPlant, SHOOTER_VELOCITY_NATIVE);
  ```

  Behavior:

    * On start: set shooter target once.
    * Each loop: task checks `plant.atSetpoint()`.
    * When at setpoint: task completes and leaves the target as‑is.

* **Move to setpoint with timeout**

  ```java
  Task spinUpWithTimeout = PlantTasks.moveTo(
      shooterPlant,
      SHOOTER_VELOCITY_NATIVE,
      1.5  // timeoutSec
  );
  ```

  Behavior:

    * Same as `moveTo(...)`, but if `atSetpoint()` never becomes true, the task
      finishes after `1.5` seconds with `TaskOutcome.TIMEOUT`.

* **Move then change target**

  ```java
  Task moveAndStow = PlantTasks.moveToThen(
      armPlant,
      ARM_SCORE_POS,
      1.0,          // timeout
      ARM_STOW_POS  // finalTarget
  );
  ```

  Behavior:

    * Move to `ARM_SCORE_POS` (or until timeout), then set target once to `ARM_STOW_POS`.

> If you accidentally call `moveTo...` on an open‑loop plant (for example, a simple servo position plant), `PlantTasks` will throw an exception at runtime to make the bug obvious.

### 4.3 Instant target helper

Sometimes you just want to set a plant target once and be done:

```java
Task stopShooter = PlantTasks.setInstant(shooterPlant, 0.0);
```

This task:

* Sets the target in `start(...)`.
* Completes immediately.
* Leaves the plant holding `0.0`.

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
    Task spinUp = PlantTasks.moveTo(
        shooter,
        SHOOTER_VELOCITY_NATIVE,
        SHOOTER_SPINUP_TIMEOUT_SEC
    );

    Task feed = PlantTasks.holdFor(
        transfer,
        TRANSFER_POWER_SHOOT,
        TRANSFER_PULSE_SEC
    );

    Task holdBeforeSpinDown = PlantTasks.holdFor(
        shooter,
        SHOOTER_VELOCITY_NATIVE,
        SHOOTER_SPINDOWN_HOLD_SEC
    );

    Task spinDown = PlantTasks.setInstant(shooter, 0.0);

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
bindings.onPress(shootButton, () -> {
    runner.enqueue(createShootOneDiscMacro());
});
```

The rest of your TeleOp loop just calls `bindings.update()` and `runner.update(clock)`.

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

## 8. Common gotchas

* **Do not block** inside `Task.start(...)` or `Task.update(...)`.

    * Never call `Thread.sleep(...)` or spin in `while(!condition)` loops.
    * Use `Tasks.waitForSeconds(...)` or `Tasks.waitUntil(...)` instead.

* **Always call `runner.update(clock)` once per loop.**

    * If you forget this, tasks will never progress.

* **Use feedback‑based helpers only on feedback plants.**

    * `PlantTasks.moveTo*` require `plant.hasFeedback() == true`.
    * For servos and other open‑loop outputs, use `holdFor`, `holdForThen`, or `setInstant`.

* **Be intentional about final targets.**

    * `holdFor(...)` keeps the same target after the timer.
    * `holdForThen(...)` lets you explicitly set a different final target (often zero).

---

## 9. Summary

* **`Task`** is the basic unit of non‑blocking behavior over time.
* **`TaskRunner`** manages a queue of tasks and advances them with `update(clock)`.
* **`Tasks` factories** (`instant`, `waitForSeconds`, `waitUntil`, `sequence`, `parallelAll`, `noop`, ...) are the main building blocks you should reach for first.
* **`PlantTasks`** provide common mechanism patterns: time‑based holds and feedback‑based moves.
* **`DriveTasks`** provide simple drive‑related building blocks.
* The core task classes – `InstantTask`, `RunForSecondsTask`, `WaitUntilTask`, `SequenceTask`, `ParallelAllTask` – are there when you need lower‑level control.
* TeleOp macros and Autonomous routines both use the same task patterns; only the triggers change (buttons vs. init/start).

Once you are comfortable with these patterns, you can layer in more advanced pieces like vision, TagAim, and interpolated shooter speeds – they all compose naturally on top of the same Task/Plant/Drive structure.
