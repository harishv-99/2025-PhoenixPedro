# Phoenix Beginner’s Guide

This guide is a gentle introduction to the Phoenix framework. It focuses on:

1. The **big ideas** (loop timing, inputs, tasks, plants, drive).
2. A minimal **TeleOp skeleton** that you can copy.
3. How to wire **hardware into Plants** using `Actuators.plant(...)`.
4. How to use **factory helpers** (`Tasks`, `PlantTasks`, `DriveTasks`) for common patterns.

If you’re new, you don’t need to know how everything works inside.
The goal is: **get a clean, non‑blocking TeleOp running quickly**.

---

## Where to look in the framework

If you’re writing robot code, you’ll spend almost all your time in a small set of packages:

* `edu.ftcphoenix.fw.actuation` — mechanisms as `Plant`s (`Actuators`, `PlantTasks`).
* `edu.ftcphoenix.fw.drive` — driving (`DriveSource`, `DriveSignal`, `MecanumDrivebase`).
* `edu.ftcphoenix.fw.input` — gamepads/buttons + `Bindings`.
* `edu.ftcphoenix.fw.task` — macros (`Task`, `TaskRunner`, `Tasks`).

As you add sensors and pose estimation, you’ll also use:

* `edu.ftcphoenix.fw.sensing` — sensors (vision, odometry, etc.).
* `edu.ftcphoenix.fw.localization` — pose estimation (AprilTags, odometry, fusion).
* `edu.ftcphoenix.fw.field` — field metadata (tag layouts, constants).

Everything else exists to keep those packages clean:

* `edu.ftcphoenix.fw.core.*` is framework plumbing (math/geometry/control/time/HAL).
* `edu.ftcphoenix.fw.ftc.*` is the FTC SDK boundary (hardware + vision adapters).
* `edu.ftcphoenix.fw.tools.*` contains testers and copyable examples.
* `edu.ftcphoenix.fw.legacy.*` contains intentionally-retained old base classes.

If you’re curious *why* the packages are arranged this way, see **Framework Overview → Package structure**.

---

## 1. The big ideas

Phoenix code is built around a few simple concepts:

* **LoopClock** – keeps track of loop timing (`dtSec`) and a per-loop identity (`cycle`).
* **Gamepads** – a consistent wrapper for FTC `gamepad1/gamepad2`.
* **Buttons + Bindings** – edge detection and “do X when pressed / held” behavior.
* **DriveSource → Drivebase** – turn inputs (or automation) into robot motion.
* **Plants** – things you command with a numeric target (motors, servos, etc.).
* **Tasks** – small, non-blocking behaviors that run over time (macros).

Everything is **non‑blocking**:

* No `sleep(...)` in your loop.
* No `while (!condition)` loops inside TeleOp.
* You just update clocks, inputs, bindings, tasks, drive, and mechanisms once per loop.

---

## 2. A minimal Phoenix TeleOp skeleton

Here’s a simplified TeleOp that:

* Sets up mecanum drive.
* Wires a shooter flywheel, transfer, and pusher as Plants.
* Uses a `TaskRunner` for macros.

> Notes:
>
> * This is a *skeleton* — you’ll fill in your hardware names and constants.
> * `FtcDrives.mecanum(hardwareMap)` assumes the standard motor names:
    >   `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`.

```java
import edu.ftcphoenix.fw.core.hal.Direction;

@TeleOp(name = "PhoenixTeleOp", group = "Examples")
public class PhoenixTeleOp extends OpMode {

    // 1) Timekeeping
    private final LoopClock clock = new LoopClock();

    // 2) Inputs + bindings
    private Gamepads gamepads;
    private Bindings bindings;

    // 3) Drive
    private MecanumDrivebase drivebase;
    private DriveSource driveSource;

    // 4) Mechanisms (Plants)
    private Plant shooter;
    private Plant transfer;
    private Plant pusher;

    // 5) Macros
    private final TaskRunner macroRunner = new TaskRunner();

    @Override
    public void init() {
        clock.reset(getRuntime());

        // Gamepads + bindings
        gamepads = Gamepads.create(gamepad1, gamepad2);
        bindings = new Bindings();

        // Drive
        drivebase = FtcDrives.mecanum(hardwareMap);
        driveSource = GamepadDriveSource.teleOpMecanumStandard(gamepads);

        // Mechanism plants
        initShooterPlants();

        // Bind buttons to macros or modes
        initBindings();
    }

    @Override
    public void start() {
        clock.reset(getRuntime());
    }

    @Override
    public void loop() {
        // 1) Clock
        clock.update(getRuntime());

        // 2) Inputs + bindings (edge detection lives here)
        gamepads.update(clock);
        bindings.update(clock);

        // 3) Macros (non-blocking tasks)
        macroRunner.update(clock);

        // 4) Drive (update first so rate limiting has the current dt)
        drivebase.update(clock);
        DriveSignal cmd = driveSource.get(clock).clamped();
        drivebase.drive(cmd);

        // 5) Mechanisms (Plants)
        double dt = clock.dtSec();
        shooter.update(dt);
        transfer.update(dt);
        pusher.update(dt);

        // 6) Telemetry (optional)
        telemetry.addData("dtSec", dt);
        telemetry.update();
    }

    private void initShooterPlants() {
        // ... see section 3
    }

    private void initBindings() {
        // ... see section 6
    }
}
```

Keep this loop shape in mind:

> **Clock → Inputs → Bindings → Tasks → Drive → Plants → Telemetry**

---

## 3. Wiring hardware as Plants with `Actuators.plant(...)`

To control hardware in Phoenix, you wrap it as a **Plant**.

The recommended way is to use the staged builder in
`edu.ftcphoenix.fw.actuation.Actuators`:

```java
private void initShooterPlants() {
    // Shooter: dual DC motors, velocity control with feedback.
    shooter = Actuators.plant(hardwareMap)
            .motor("shooterLeftMotor", Direction.FORWARD)
            .andMotor("shooterRightMotor", Direction.REVERSE)
            .velocity()     // default tolerance (native units)
            .build();

    // Transfer: dual CR servos, power control.
    transfer = Actuators.plant(hardwareMap)
            .crServo("transferLeftServo", Direction.FORWARD)
            .andCrServo("transferRightServo", Direction.REVERSE)
            .power()
            .build();

    // Pusher: positional servo, set-and-hold position.
    pusher = Actuators.plant(hardwareMap)
            .servo("pusherServo", Direction.FORWARD)
            .position()     // open-loop servo position (no feedback)
            .build();
}
```

### 3.1 What the builder is doing

The builder has three steps:

1. **Pick hardware**:

    * `.motor(name, direction)` then (optional) `.andMotor(name, direction)`
        * After you add a second motor, you may optionally calibrate the <i>last added</i>
          motor with `.scale(...)` / `.bias(...)` if one side needs a small adjustment.
    * `.servo(name, direction)` then (optional) `.andServo(name, direction)`
    * `.crServo(name, direction)` then (optional) `.andCrServo(name, direction)`

2. **Pick control type**:

    * `.power()` – open‑loop power (e.g., CR servos, motors as % power).
    * `.velocity()` or `.velocity(tolerance)` – closed‑loop velocity (native units).
    * `.position()` or `.position(tolerance)` – position control.

3. **Optional modifiers**:

    * `.rateLimit(maxDeltaPerSec)` – limit how quickly the target can change.
    * `.build()` – return the final `Plant`.

### 3.2 Position semantics: motors vs servos

The `.position(...)` control mode behaves slightly differently depending on
which hardware you chose:

* **DC motors** (via `.motor(...)` and optional `.andMotor(...)`):

    * `.position(tolerance)` creates a **feedback-based motor position plant**.
    * `plant.hasFeedback() == true`.
    * `plant.atSetpoint()` becomes true when the encoder error is within tolerance.
    * `plant.reset()` re-zeros the plant’s coordinate frame at the current measured position.

* **Servos** (via `.servo(...)` and optional `.andServo(...)`):

    * `.position()` creates a **servo position plant** in the range `0.0..1.0`.
    * This is an open‑loop “set‑and‑hold” behavior.
    * `plant.hasFeedback() == false`.
    * `plant.atSetpoint()` is always true (there is no measured position).

Why this matters:

* For **feedback-based moves** (e.g., “move arm to this angle and wait”),
  you must use a feedback plant (usually motor position or motor velocity).
* For **simple servo motions** (e.g., pusher, claw), it’s fine to use open‑loop
  servo position plants and time-based waits.

---

## 4. Using `PlantTasks` for mechanism behavior

Once you have Plants, the easiest way to create behaviors is
`edu.ftcphoenix.fw.actuation.PlantTasks`.

These are **factory helpers** that build `Task`s for you.

### 4.1 Time‑based helpers (work with any Plant)

These helpers do **not** depend on `plant.atSetpoint()`, so they work with
feedback plants and open-loop plants.

* **Run for N seconds, then change target**

  ```java
  // Intake: run at +1.0 for 0.7 seconds, then stop.
  Task intakePulse = PlantTasks.holdForThen(intake, +1.0, 0.7, 0.0);
  ```

* **Run for N seconds and keep that target afterward**

  ```java
  // Run shooter at target for at least 0.5 seconds, then keep holding it.
  Task ensureSpinUp = PlantTasks.holdFor(shooter, SHOOTER_VELOCITY_NATIVE, 0.5);
  ```

### 4.2 Feedback‑based move helpers (require feedback)

These helpers require `plant.hasFeedback() == true`.

```java
// Move shooter to a target velocity and wait until it's at setpoint (or timeout).
Task spinUp = PlantTasks.moveTo(shooter, SHOOTER_VELOCITY_NATIVE, 1.2);

// Move arm to a setpoint, then command a final target right after.
Task moveAndStow = PlantTasks.moveToThen(arm, ARM_SCORE_POS, 1.0, ARM_STOW_POS);
```

If you accidentally call a feedback-based helper on an open-loop plant (like a
simple servo position plant), `PlantTasks` throws an exception at runtime so the
mistake is obvious.

### 4.3 Instant target helper

For one‑shot changes:

```java
Task stopShooter = PlantTasks.setInstant(shooter, 0.0);
```

This sets the target once in `start(...)`, finishes immediately, and leaves
that target in place.

---

## 5. Using `Tasks` factories for general behavior

For behaviors that aren’t specific to a single Plant, use
`edu.ftcphoenix.fw.task.Tasks`.

Examples:

```java
Task pause = Tasks.waitForSeconds(0.5);
Task waitForReady = Tasks.waitUntil(() -> shooterReady());

Task macro = Tasks.sequence(spinUp, feed, spinDown);
Task parallel = Tasks.parallelAll(moveArm, runIntake);
```

In student code, prefer the factories (`Tasks.*`, `PlantTasks.*`, `DriveTasks.*`)
over constructing task classes directly.

---

## 6. Example: a simple shooter macro

Assume you already created `shooter`, `transfer`, and a `TaskRunner` named
`macroRunner`.

```java
private Task buildShootOneDiscMacro() {
    Task spinUp = PlantTasks.moveTo(
            shooter,
            SHOOTER_VELOCITY_NATIVE,
            SHOOTER_SPINUP_TIMEOUT_SEC
    );

    Task feedTransfer = PlantTasks.holdFor(
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

    return Tasks.sequence(spinUp, feedTransfer, holdBeforeSpinDown, spinDown);
}

private void initBindings() {
    bindings.onPress(gamepads.p1().y(), () -> {
        // Build a fresh task each time (tasks are single-use).
        macroRunner.enqueue(buildShootOneDiscMacro());
    });
}
```

Because you call `macroRunner.update(clock)` every loop, the macro runs over time
without blocking TeleOp.

---

## 7. When to use raw Task classes

For most student code, you only need:

* `Actuators.plant(...)` to build Plants.
* `PlantTasks.*` for mechanism behavior.
* `Tasks.*` for generic timing and composition.
* `DriveTasks.*` for drive‑specific helpers.

The raw task classes (`InstantTask`, `RunForSecondsTask`, `WaitUntilTask`,
`SequenceTask`, `ParallelAllTask`, …) are useful when:

* You’re building **your own helper factories** to share across a team.
* You need a special behavior the existing factories don’t cover yet.

---

## 8. Summary

* **Plants** represent mechanisms you can command with a numeric target.

* **Actuators.plant(...)** is the preferred way to create Plants from FTC hardware.

* **Position semantics differ**:

    * Motors + `.position(tolerance)` → feedback + encoders.
    * Servos + `.position()` → open‑loop set‑and‑hold.

* **PlantTasks** and **Tasks** provide factory helpers that build `Task`s for you.

* The **main loop shape** stays consistent:

  > Clock → Inputs → Bindings → Tasks → Drive → Plants → Telemetry

Next steps:

* Read **Tasks & Macros Quickstart** for deeper task patterns.
* Read **Shooter Case Study & Examples Walkthrough** for an end-to-end example.
* Explore other examples (drive, tag aim, vision) — they all follow the same loop shape.
