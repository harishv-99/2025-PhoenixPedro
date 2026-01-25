# Phoenix Framework Overview

This document gives a high‑level tour of the Phoenix framework and how the
major pieces fit together. It focuses on the **conceptual layers** and on the
APIs that students should reach for first.

The main ideas:

* Separate **hardware wiring** from **robot behavior**.
* Keep everything **non‑blocking** and **task‑driven**.
* Favor **factory helpers** over low‑level classes in student code.

---

## 1. Layered architecture

Phoenix is organized into a small set of layers:

1. **FTC SDK hardware** – `HardwareMap`, `DcMotor`, `Servo`, `CRServo`, etc.
2. **Phoenix HAL** – small interfaces like `PowerOutput`, `PositionOutput`,
   `VelocityOutput`.
3. **Actuators / Plants** – turn HAL outputs into `Plant` objects.
4. **Drive** – `DriveSource` + `Drivebase` (e.g., `MecanumDrivebase`).
5. **Tasks** – non‑blocking behaviors that run over time.
6. **Bindings** – mapping inputs (gamepads, triggers) to behaviors.

### 1.1 Why this structure?

The goals are:

* Make robot code **read like a story** ("spin up shooter, feed, then spin down").
* Make it easy to **swap hardware** or add new mechanisms.
* Keep timing and scheduling consistent across TeleOp and Autonomous.

Your OpMode lives at the top of this stack and mostly talks to:

* `Drivebase` / `DriveSource` for motion.
* `Plant`s for mechanisms.
* `Tasks` / `TaskRunner` for behavior.

---

## 2. From hardware to HAL

The Phoenix HAL defines small, focused interfaces:

* `PowerOutput` – something that accepts a normalized power (e.g. ‑1..+1).
* `PositionOutput` – something that accepts a position in native units
  (servo position or encoder ticks).
* `VelocityOutput` – something that accepts a velocity in native units
  (ticks/sec, etc.).

Adapters in `edu.ftcphoenix.fw.adapters.ftc` (e.g. `FtcHardware`) take FTC
SDK objects and expose them as HAL interfaces. This layer hides:

* Which controller you’re using.
* How encoders are configured.
* The details of SDK calls like `setPower()`, `setMode()`, etc.

Most student code does not need to touch `FtcHardware` directly.

---

## 3. Actuators and Plants

### 3.1 Plants: behavior at the mechanism level

A `Plant` represents a mechanism you can command with a single numeric target:

```java
public interface Plant {
    void setTarget(double target);
    double getTarget();

    void update(double dtSec);
    void stop();

    boolean atSetpoint();
    boolean hasFeedback();
    void reset();
}
```

Examples:

* Shooter flywheel – target is velocity in native units.
* Elevator – target is position (ticks) with encoder feedback.
* Pusher servo – target is servo position 0.0..1.0.
* Intake – target is power (‑1..+1).

Plants don’t know about gamepads or tasks. They simply move toward whatever
`setTarget(...)` you ask for.

### 3.2 Actuators: recommended way to create Plants

In student code, the preferred entry point is:

```java
Plant shooter = Actuators.plant(hardwareMap)
        .motorPair("shooterLeftMotor",  false,
                   "shooterRightMotor", true)
        .velocity()               // or .velocity(tolerance)
        .build();

Plant transfer = Actuators.plant(hardwareMap)
        .crServoPair("transferLeft", false,
                     "transferRight", true)
        .power()
        .build();

Plant pusher = Actuators.plant(hardwareMap)
        .servo("pusherServo", false)
        .position()               // open‑loop servo position
        .build();
```

The builder has three steps:

1. **Pick hardware**: `.motor(...)`, `.motorPair(...)`, `.servo(...)`,
   `.servoPair(...)`, `.crServo(...)`, `.crServoPair(...)`.
2. **Pick control type**:

    * `.power()` – open‑loop power.
    * `.velocity()` / `.velocity(tolerance)` – closed‑loop velocity.
    * `.position()` / `.position(tolerance)` – positional control.
3. **Optional modifiers** – `.rateLimit(...)` then `.build()`.

Internally, the builder calls the lower‑level `Plants` factories and the FTC
HAL adapters. Student code should normally use the **builder** and avoid
calling `Plants.*` directly.

### 3.3 Position semantics

The `.position(...)` mode is intentionally flexible:

* **Motors** – feedback position plants:

    * Created via `.motor(...)`/`.motorPair(...).position(tolerance)`.
    * Use encoders via `FtcHardware.motorPosition(...)`.
    * `hasFeedback() == true`.
    * `atSetpoint()` checks encoder error vs tolerance.
    * `reset()` re‑zeros the plant’s coordinate frame.
* **Servos** – open‑loop position plants:

    * Created via `.servo(...).position()` or `.servoPair().position()`.
    * Use servo position 0.0..1.0.
    * `hasFeedback() == false`.
    * `atSetpoint()` always returns `true`.

Because of this, feedback‑based helpers (like `PlantTasks.moveTo(...)`) are
only valid for plants where `hasFeedback() == true`.

---

## 4. Drive: DriveSource and Drivebase

Phoenix separates **what you want the robot to do** from **how the drivebase
makes it happen**.

* `DriveSource` – produces a `DriveSignal` based on inputs and state.

    * Examples: `GamepadDriveSource`, `TagAimDriveSource`.
* `DriveSignal` – a high‑level command (e.g., strafe, forward, rotate).
* `Drivebase` – turns a `DriveSignal` into motor outputs.

    * Example: `MecanumDrivebase` from `Drives.mecanum(...)`.

Typical pattern in an OpMode:

```java
DriveSignal cmd = driveSource.get(clock).clamped();
drivebase.drive(cmd);
drivebase.update(clock);
```

Drive sources can be swapped without changing the rest of the robot:

* Basic TeleOp: `GamepadDriveSource`.
* Vision‑assisted alignment: `TagAimDriveSource`.

---

## 5. Tasks: non‑blocking behaviors

### 5.1 Task and TaskRunner

Tasks are small objects that run over time:

```java
public interface Task {
    void start(LoopClock clock);
    void update(LoopClock clock);
    boolean isComplete();
    TaskOutcome getOutcome();
}
```

`TaskRunner` manages a queue of tasks and calls `start` / `update` for you.

The golden rule:

> **Never block** inside `start(...)` or `update(...)`.
>
> Use time and conditions to decide when a task is done, not sleeps or loops.

### 5.2 Factory helpers: Tasks, PlantTasks, DriveTasks

In student code, you should almost always build tasks using **factory helpers**:

* `Tasks.*` – general utilities:

    * `Tasks.instant(...)` – one‑shot actions.
    * `Tasks.waitForSeconds(...)` – time‑based waits.
    * `Tasks.waitUntil(...)` – condition‑based waits.
    * `Tasks.sequence(...)` – run tasks one after another.
    * `Tasks.parallelAll(...)` – run tasks in parallel.
* `PlantTasks.*` – mechanism patterns:

    * `holdFor(...)`, `holdForThen(...)` – time‑based plant commands.
    * `moveTo(...)`, `moveTo(..., timeout)` – feedback‑based moves.
    * `moveToThen(...)` – move then change target once.
    * `setInstant(...)` – set target once and complete.
* `DriveTasks.*` – drive‑specific behaviors (drive for time/distance, stop,
  etc.).

Under the hood, these factories use classes like `InstantTask`,
`RunForSecondsTask`, `WaitUntilTask`, `SequenceTask`, and `ParallelAllTask`,
so you can still drop down a level when needed.

Recommended usage:

* Use **`Tasks`/`PlantTasks`/`DriveTasks`** in your OpModes and robot code.
* Use the raw `Task` classes when:

    * You are building new helper factories for your team.
    * You need a behavior that the existing helpers don’t cover yet.

---

## 6. Bindings: connecting inputs to behavior

Bindings map input events (button presses, stick moves) to actions:

* "When P1.Y is pressed, enqueue shooter macro."
* "While P1.RB is held, enable slow mode."

Example:

```java
bindings.onPress(gamepads.p1().y(), () -> {
    macroRunner.enqueue(buildShootOneDiscMacro());
});

bindings.whileHeld(gamepads.p1().rightBumper(), () -> {
    driveSource.setSlowMode(true);
});
```

The main loop calls:

```java
gamepads.update(clock.dtSec());
bindings.update(clock.dtSec());
```

Bindings stay at the top of the stack and never touch motors directly.
They only call helpers (Tasks, PlantTasks, DriveTasks, or your own methods).

---

## 7. The standard loop shape

Almost every Phoenix OpMode follows this shape:

```java
@Override
public void loop() {
    // 1) Clock
    clock.update(getRuntime());

    // 2) Inputs + bindings
    gamepads.update(clock.dtSec());
    bindings.update(clock.dtSec());

    // 3) Macros / tasks
    macroRunner.update(clock);

    // 4) Drive
    DriveSignal driveCmd = driveSource.get(clock).clamped();
    drivebase.drive(driveCmd);
    drivebase.update(clock);

    // 5) Mechanisms
    shooter.update(clock.dtSec());
    transfer.update(clock.dtSec());
    pusher.update(clock.dtSec());

    // 6) Telemetry
    telemetry.update();
}
```

This works for both TeleOp and Autonomous. In Autonomous you typically:

* Enqueue one big `Tasks.sequence(...)` macro in `start()`.
* Let it run while the loop keeps everything updated.

---

## 8. Putting it together: suggested workflow

When building a new robot or mechanism:

1. **Define hardware mapping** in the FTC Robot Configuration.
2. **Create Plants** using `Actuators.plant(hardwareMap)`:

    * Use motor `.velocity(...)` / `.position(tolerance)` for feedback.
    * Use servo `.position()` or `.power()` for simpler mechanisms.
3. **Write small behaviors** with `PlantTasks` and `Tasks`:

    * Time‑based holds, feedback moves, instant changes.
4. **Compose behaviors into macros**:

    * `Tasks.sequence(...)` and `Tasks.parallelAll(...)`.
5. **Hook macros to inputs** using `Bindings`.
6. Keep the **main loop shape** simple and consistent.

If you find yourself repeating the same wiring of plants and tasks, pull it
into a helper method or a new factory so the next student can call a single
function. The provided examples (especially the shooter case study) are
intended as templates you can adapt to your own robot while keeping the
same structure.
