# Phoenix Framework Principles

This document explains the **design principles** behind the Phoenix framework
and how the main layers fit together. It is not a step‑by‑step tutorial; instead
it explains *why* the APIs look the way they do and how we expect teams to use
them.

If you’re looking to get started quickly, read these in order:

1. **Beginner’s Guide** – basic loop shape and how to wire Plants.
2. **Tasks & Macros Quickstart** – how to use Tasks and PlantTasks.
3. **Shooter Case Study** – a complete, concrete example.

This document fills in the reasoning behind those examples.

---

## 1. High‑level goals

Phoenix has a few core goals:

1. **Non‑blocking by design**
   No `sleep(...)`, no long `while(...)` loops inside TeleOp or Auto. Everything
   is expressed as Tasks that advance one step each loop.

2. **Clear separation of layers**
   Robot code should not have to think about the details of the FTC SDK or
   vendor APIs. Instead, code is written in terms of:

    * `DriveSource` + `Drivebase` for motion.
    * `Plant` for mechanisms.
    * `Task` / `TaskRunner` for behavior over time.

3. **Beginner‑friendly, mentor‑powerful**
   Students use high‑level factories (`Actuators.plant(...)`, `PlantTasks`,
   `Tasks`, `DriveTasks`). Mentors and advanced users can drop down into
   the HAL and raw Task classes when needed.

4. **Predictable, opinionated defaults**
   The framework chooses reasonable defaults (e.g., motor position and
   velocity tolerances) so students can get things working with minimal
   configuration, while still allowing overrides.

5. **Composable building blocks**
   Drive, plants, and tasks are all independent pieces that fit together:

    * You should be able to swap in TagAim instead of a gamepad drive source.
    * You should be able to replace a manual shooter distance with a vision
      distance without changing the macro code.

---

## 2. Layering: from hardware to behavior

Phoenix is organized into layers. Robot code is encouraged to stay near the
**top layers**, while the lower layers hide platform specifics.

### 2.1 Hardware abstraction (HAL)

At the bottom is the **Phoenix HAL**:

* `PowerOutput` – normalized power (usually ‑1..+1).
* `PositionOutput` – position in native units (ticks, servo position 0..1, etc.).
* `VelocityOutput` – velocity in native units (ticks/sec, etc.).

`edu.ftcphoenix.fw.adapters.ftc.FtcHardware` adapts the FTC SDK motors/servos
into these interfaces.

Robot code normally does **not** use the HAL directly; it uses Plants.

### 2.2 Plants

A **Plant** is a small object with a numeric target:

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

Design choices:

* All Plants support `stop()` and `reset()` so the framework can stop
  mechanisms and re‑zero them.
* `hasFeedback()` and `atSetpoint()` let Tasks distinguish between
  open‑loop and feedback‑based behavior.

Plants are created by **`Actuators.plant(...)`** (public API) and, internally,
by the `Plants` factory methods.

### 2.3 Actuators builder

`edu.ftcphoenix.fw.actuation.Actuators` is the recommended way to create
Plants from FTC hardware:

```java
Plant shooter = Actuators.plant(hardwareMap)
        .motorPair("shooterLeftMotor",  false,
                   "shooterRightMotor", true)
        .velocity()   // uses default velocity tolerance
        .build();

Plant transfer = Actuators.plant(hardwareMap)
        .crServoPair("transferLeft", false,
                     "transferRight", true)
        .power()
        .build();

Plant pusher = Actuators.plant(hardwareMap)
        .servo("pusherServo", false)
        .position()   // servo position, open‑loop set‑and‑hold
        .build();
```

The builder is deliberately staged:

1. **Pick hardware:** `motor`, `motorPair`, `servo`, `servoPair`, `crServo`,
   `crServoPair`.
2. **Pick control type:** `power()`, `velocity()`, `velocity(tol)`,
   `position()`, `position(tol)`.
3. **Optional modifiers:** `rateLimit(maxDeltaPerSec)`, then `build()`.

This keeps common wiring compact and discoverable, while still allowing
advanced teams to work directly with the HAL when necessary.

### 2.4 Drive sources and drivebases

Drive is split into two parts:

* `DriveSource` – any object that converts inputs (gamepads, TagAim,
  path followers, etc.) into a `DriveSignal`.
* `Drivebase` – an object that applies a `DriveSignal` to real motors and
  handles kinematics.

Examples:

* `GamepadDriveSource` – reads sticks and triggers.
* `TagAimDriveSource` – uses AprilTags to aim at a goal.
* `Drives.mecanum(...)` – builds a `MecanumDrivebase`.

Robot code usually calls:

```java
DriveSignal driveCmd = driveSource.get(clock).clamped();

drivebase.drive(driveCmd);
drivebase.update(clock);
```

The **drive source** can change freely without touching drivebase wiring.

### 2.5 Tasks and TaskRunner

Tasks represent behavior that runs over time:

```java
public interface Task {
    void start(LoopClock clock);
    void update(LoopClock clock);
    boolean isComplete();
    TaskOutcome getOutcome();
}
```

`TaskRunner` manages a queue of Tasks. In each loop you call
`runner.update(clock)` and it:

* Starts the next task when the previous one finishes.
* Calls `start(...)` once and `update(...)` every loop.

Robot code should rarely implement Tasks from scratch. Instead, you use:

* `Tasks.*` factories for generic behaviors.
* `PlantTasks.*` for mechanism behaviors.
* `DriveTasks.*` for drive behaviors.

The lower‑level classes (`InstantTask`, `RunForSecondsTask`, `WaitUntilTask`,
`SequenceTask`, `ParallelAllTask`, ...) exist so you can build your own
helpers and more advanced patterns.

---

## 3. Opinionated semantics

Some APIs are intentionally opinionated to encourage safe, clear code.

### 3.1 Feedback vs open‑loop plants

Not all Plants are the same:

* **Feedback plants** (e.g., motor position / velocity plants) use encoders
  or sensors and implement `atSetpoint()` with a tolerance.
* **Open‑loop plants** (e.g., servo position, power plants) have no measured
  value; `atSetpoint()` is always true and `hasFeedback()` is false.

This distinction is used heavily in `PlantTasks`:

* `moveTo(...)`, `moveTo(..., timeout)`, `moveToThen(...)` **require** a
  feedback‑capable plant (`hasFeedback() == true`). They will throw at
  runtime if used with an open‑loop plant.
* `holdFor(...)`, `holdForThen(...)`, `setInstant(...)` do **not** require
  feedback and work with any plant.

This makes it difficult to accidentally write a “wait until” task on a
plant that has no concept of measurement.

### 3.2 Position semantics for motors and servos

The `.position(...)` control type in `Actuators` behaves differently by
hardware kind:

* **Motors + `.position(tolerance)`**

    * Feedback‑based position control using encoders.
    * `atSetpoint()` uses the given tolerance in native units.
    * `reset()` re‑zeros the plant’s coordinate frame at the current position.

* **Servos + `.position()`**

    * Open‑loop servo position (0.0..1.0).
    * Always “at setpoint” from the task’s perspective.
    * Intended to be used with time‑based `PlantTasks` like `holdFor(...)`.

We keep this split because it matches how the hardware works and prevents
students from accidentally relying on feedback that doesn’t exist.

### 3.3 Default tolerances

`Actuators` provides default tolerances when you call:

* `.velocity()` – uses a default velocity tolerance.
* `.position()` for motors – uses a default position tolerance.

Advanced users can call `.velocity(tolerance)` or `.position(tolerance)` to
control the error bands directly. The defaults are meant to be “good enough”
for most mechanisms without forcing students to tune every value up front.

---

## 4. Recommended usage patterns

The framework is designed with a **top‑down** recommended usage:

1. **Write your TeleOp / Auto loop in terms of:**

    * Clock → Gamepads → Bindings → TaskRunner(s) → Drivebase → Plants.

2. **Model hardware as Plants via `Actuators.plant(...)`.**

    * Prefer the builder; avoid using `Plants` directly in student code.

3. **Use factory helpers wherever possible:**

    * `PlantTasks` for mechanism behaviors (time‑based holds, move‑to‑setpoint,
      instant changes).
    * `Tasks` for waiting, sequencing, and parallelism.
    * `DriveTasks` for drive behaviors like “drive for seconds”, “follow
      a fixed heading”, etc.

4. **Drop down to lower levels only when needed:**

    * Use raw `Task` classes (`RunForSecondsTask`, `SequenceTask`, etc.) when
      building new factories or advanced patterns.
    * Use HAL (`PowerOutput`, `PositionOutput`, `VelocityOutput`) when building
      new Plant types or adapters.

This keeps most robot code compact and readable while still allowing
deep customization for mentors.

---

## 5. Non‑blocking mindset

A central principle is that **nothing in the main loop blocks**.

Instead of:

```java
// DO NOT DO THIS
while (!shooterReady()) {
    // spin here
}
```

we write:

```java
Task waitForReady = Tasks.waitUntil(() -> shooterReady());
runner.enqueue(waitForReady);
```

Instead of:

```java
// DO NOT DO THIS
shooter.setTarget(SHOOTER_VELOCITY);
Thread.sleep(1500);
transfer.setTarget(TRANSFER_POWER);
```

we write:

```java
Task macro = Tasks.sequence(
        PlantTasks.moveTo(shooter, SHOOTER_VELOCITY, 1.5),
        PlantTasks.holdForThen(transfer, TRANSFER_POWER, 0.3, 0.0)
);

runner.enqueue(macro);
```

The non‑blocking mindset makes it easier to:

* Keep TeleOp responsive.
* Integrate vision processing and telemetry updates.
* Reason about tasks and macros as reusable building blocks.

---

## 6. Extending the framework

The framework is intentionally open to extension. Common extension points:

* **New DriveSources** – e.g., field‑centric drive, auto‑aim drive,
  path‑following drive.
* **New Plant types** – wrapping sensors or actuators in custom control logic.
* **New Task factories** – domain‑specific helpers like
  `MyRobotTasks.autoScoreFromWing()` built from existing Tasks and Plants.

When you extend, try to follow the existing patterns:

* Keep low‑level concerns (SDK calls, vendor APIs) in adapter classes.
* Expose simple, narrow interfaces (`Plant`, `DriveSource`, `Task`).
* Provide **factory helpers** at the top of your module so robot code
  can stay declarative and readable.

---

## 7. Summary

* Phoenix emphasizes **non‑blocking**, **layered**, and **composable** robot
  code.
* Hardware is abstracted through the HAL (`PowerOutput`, `PositionOutput`,
  `VelocityOutput`) and wrapped as **Plants**.
* `Actuators.plant(...)` is the main way students should create Plants.
* `PlantTasks`, `Tasks`, and `DriveTasks` are the recommended entry points
  for building behaviors.
* Feedback vs open‑loop semantics are explicit: feedback‑based helpers
  require `plant.hasFeedback() == true`.
* Default tolerances and builder patterns are chosen to make simple things
  easy, while still supporting advanced usage.

If your code follows these principles, it will tend to be easier to read,
reason about, and share across seasons and between teams.
