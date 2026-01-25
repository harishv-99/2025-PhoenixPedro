# Phoenix Beginner’s Guide

This guide is a gentle introduction to the Phoenix framework. It focuses on:

1. The **big ideas** (loops, tasks, plants, drive).
2. A minimal **TeleOp skeleton** that you can copy.
3. How to wire your **hardware to Plants** using `Actuators.plant(...)`.
4. How to use **factory helpers** (`Tasks`, `PlantTasks`, `DriveTasks`) for common patterns.

If you’re new, you don’t need to know how everything works inside.
The goal is: **get a clean, non‑blocking TeleOp running quickly**.

---

## 1. The big ideas

Phoenix code is built around a few simple concepts:

* **LoopClock** – keeps track of time and `dtSec` (delta time per loop).
* **Gamepads** – read controller input in a consistent, debounced way.
* **DriveSource** → **Drivebase** – turn inputs into robot motion.
* **Plants** – things you send numeric targets to (motors, servos, etc.).
* **Tasks** – small behaviors that run over time (macros, waits, etc.).

Everything is **non‑blocking**:

* No `sleep(...)` in your loop.
* No `while(!condition)` loops inside TeleOp.
* You just update clocks, inputs, tasks, and mechanisms once per loop.

---

## 2. A minimal Phoenix TeleOp skeleton

Here’s a simplified TeleOp that:

* Sets up a mecanum drive.
* Wires a shooter flywheel, transfer, and pusher as Plants.
* Uses a `TaskRunner` for macros.

You can use this as a starting point.

```java
@TeleOp(name = "PhoenixTeleOp", group = "Examples")
public class PhoenixTeleOp extends OpMode {

    // 1) Timekeeping
    private final LoopClock clock = new LoopClock();

    // 2) Gamepads and bindings
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
        // Timekeeping
        clock.reset(getRuntime());

        // Gamepads
        gamepads = new Gamepads(gamepad1, gamepad2);
        bindings = new Bindings();

        // Drive
        drivebase = Drives.mecanum(hardwareMap,
                "frontLeft", "frontRight",
                "backLeft",  "backRight");
        driveSource = new GamepadDriveSource(gamepads.p1());

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

        // 2) Inputs + bindings
        gamepads.update(clock.dtSec());
        bindings.update(clock.dtSec());

        // 3) Macros
        macroRunner.update(clock);

        // 4) Drive
        DriveSignal driveCmd = driveSource.get(clock).clamped();
        drivebase.drive(driveCmd);
        drivebase.update(clock);

        // 5) Mechanism plants
        shooter.update(clock.dtSec());
        transfer.update(clock.dtSec());
        pusher.update(clock.dtSec());

        // 6) Telemetry (optional)
        telemetry.addData("dtSec", clock.dtSec());
        telemetry.update();
    }
}
```

Keep this shape in mind:

> **Clock → Inputs → Tasks → Drive → Plants → Telemetry**

All examples in the framework follow this same pattern.

---

## 3. Wiring hardware as Plants with `Actuators.plant(...)`

To control hardware in Phoenix, you first wrap it as a **Plant**.

The recommended way is to use the **builder factories** in
`edu.ftcphoenix.fw.actuation.Actuators`:

```java
private void initShooterPlants() {
    // Shooter: dual DC motors, velocity control with feedback.
    shooter = Actuators.plant(hardwareMap)
            .motorPair("shooterLeftMotor",  false,
                       "shooterRightMotor", true)
            .velocity()   // uses a reasonable default tolerance
            .build();

    // Transfer: dual CR servos, power control.
    transfer = Actuators.plant(hardwareMap)
            .crServoPair("transferLeftServo",  false,
                         "transferRightServo", true)
            .power()
            .build();

    // Pusher: positional servo, set‑and‑hold position.
    pusher = Actuators.plant(hardwareMap)
            .servo("pusherServo", false)
            .position()   // open‑loop servo position, no feedback
            .build();
}
```

### 3.1 What the builder is doing

The builder has three steps:

1. **Pick hardware**:

    * `.motor(name, inverted)`
    * `.motorPair(nameA, invA, nameB, invB)`
    * `.servo(name, inverted)` / `.servoPair(...)`
    * `.crServo(name, inverted)` / `.crServoPair(...)`
2. **Pick control type**:

    * `.power()` – open‑loop power (e.g., CR servos, motors as % power).
    * `.velocity()` or `.velocity(tolerance)` – closed‑loop velocity.
    * `.position()` or `.position(tolerance)` – positional control.
3. **Optional modifiers**:

    * `.rateLimit(maxDeltaPerSec)` – limit how quickly the target can change.
    * `.build()` – return the final `Plant`.

Internally, the builder uses the `Plants` factories and FTC `HardwareMap`
for you. You should almost never call `Plants.*` directly in student code;
that’s what the builder is for.

### 3.2 Position semantics: motors vs servos

The `.position(...)` control mode behaves slightly differently depending
on which hardware you chose:

* **DC motors** (via `.motor(...)` / `.motorPair(...)`):

    * `.position(tolerance)` creates a **feedback‑based motor position plant**.
    * Uses encoders via `FtcHardware.motorPosition(...)`.
    * `plant.hasFeedback() == true`.
    * `plant.atSetpoint()` is true when the error is within the tolerance.
    * `plant.reset()` re‑zeros the plant’s coordinate frame at the current
      position (without resetting the hardware encoder itself).

* **Servos** (via `.servo(...)` / `.servoPair(...)`):

    * `.position()` creates a **servo position plant** in the range `0.0..1.0`.
    * This is an open‑loop "set‑and‑hold" behavior.
    * `plant.hasFeedback() == false`.
    * `plant.atSetpoint()` is always true (there is no measured position).

Why this matters:

* For **feedback‑based moves** (e.g., "move arm to this angle and wait"),
  you must use a feedback plant (usually a motor position or velocity plant).
* For **simple servo motions** (e.g., pusher, claw), it’s fine to use the
  open‑loop servo position plant and time‑based waits.

---

## 4. Using `PlantTasks` for mechanism behavior

Once you have Plants, the easiest way to create behaviors is to use
`edu.ftcphoenix.fw.actuation.PlantTasks`.

These are **factory helpers** that build `Task`s for you.

### 4.1 Time‑based helpers (work with any Plant)

Use these for both feedback and non‑feedback plants:

* **Run for N seconds, then change target**

  ```java
  // Intake: run at +1.0 for 0.7 seconds, then stop.
  Task intakePulse = PlantTasks.holdForThen(
          intake,
          +1.0,
          0.7,
          0.0
  );
  ```

* **Run for N seconds and keep that target afterward**

  ```java
  // Shooter: make sure we’ve spun up for at least 0.5 seconds,
  // but keep holding this target after the task.
  Task ensureSpinUp = PlantTasks.holdFor(
          shooter,
          SHOOTER_VELOCITY_NATIVE,
          0.5
  );
  ```

These helpers are purely time‑based – they do **not** check
`plant.atSetpoint()`. That’s why they work fine for servo position plants
and power plants.

### 4.2 Feedback‑based move helpers (require feedback)

These helpers expect a feedback plant (`plant.hasFeedback() == true`).
Use them for encoder‑backed motor position or velocity plants:

```java
// Move shooter to a target velocity and wait until it’s there.
Task spinUp = PlantTasks.moveTo(
        shooter,
        SHOOTER_VELOCITY_NATIVE,
        SHOOTER_SPINUP_TIMEOUT_SEC
);

// Move arm to a setpoint, then change to a final position after the move.
Task moveAndStow = PlantTasks.moveToThen(
        arm,
        ARM_SCORE_POS,
        1.0,           // timeout
        ARM_STOW_POS   // finalTarget
);
```

If you accidentally call `moveTo(...)` on an open‑loop plant (like a
simple servo), `PlantTasks` will throw an exception at runtime to make
that mistake obvious.

### 4.3 Instant target helper

For one‑shot changes:

```java
Task stopShooter = PlantTasks.setInstant(shooter, 0.0);
```

This sets the target once in `start(...)`, finishes immediately, and leaves
that target in place.

---

## 5. Using `Tasks` factories for general behavior

For behaviors that aren’t specific to a single Plant, use the factories
in `edu.ftcphoenix.fw.task.Tasks`.

Examples:

```java
// Wait 0.5 seconds.
Task pause = Tasks.waitForSeconds(0.5);

// Wait until a condition is true.
Task waitForReady = Tasks.waitUntil(() -> shooterReady());

// Run tasks in sequence.
Task macro = Tasks.sequence(
        spinUp,
        feed,
        spinDown
);

// Run tasks in parallel and finish when both are done.
Task parallel = Tasks.parallelAll(
        moveArm,
        runIntake
);
```

These factories are built on core task classes like `InstantTask`,
`RunForSecondsTask`, `WaitUntilTask`, `SequenceTask`, and `ParallelAllTask`.
You should usually **use the factories** instead of constructing those
classes directly.

---

## 6. Example: a simple shooter macro

Putting it together, here’s a small shooter macro similar to the example
OpModes.

Assume you already created `shooter`, `transfer`, and a `TaskRunner` named
`macroRunner`.

```java
private Task buildShootOneDiscMacro() {
    // 1) Spin up shooter and wait for it to reach velocity (or timeout).
    Task spinUp = PlantTasks.moveTo(
            shooter,
            SHOOTER_VELOCITY_NATIVE,
            SHOOTER_SPINUP_TIMEOUT_SEC
    );

    // 2) Feed one disc.
    Task feedTransfer = PlantTasks.holdFor(
            transfer,
            TRANSFER_POWER_SHOOT,
            TRANSFER_PULSE_SEC
    );

    // 3) Hold shooter up to speed briefly before spinning down.
    Task holdBeforeSpinDown = PlantTasks.holdFor(
            shooter,
            SHOOTER_VELOCITY_NATIVE,
            SHOOTER_SPINDOWN_HOLD_SEC
    );

    // 4) Spin down.
    Task spinDown = PlantTasks.setInstant(shooter, 0.0);

    return Tasks.sequence(
            spinUp,
            feedTransfer,
            holdBeforeSpinDown,
            spinDown
    );
}
```

Then bind it to a button:

```java
bindings.onPress(gamepads.p1().y(), () -> {
    macroRunner.enqueue(buildShootOneDiscMacro());
});
```

In your main loop, you already call `macroRunner.update(clock);`, so the
macro will run over time without blocking.

---

## 7. When to use raw Task classes

For **most** student code, you only need:

* `Actuators.plant(...)` to build Plants.
* `PlantTasks.*` for mechanism behavior.
* `Tasks.*` for generic timing and composition.
* `DriveTasks.*` for drive‑specific helpers.

The raw task classes (`InstantTask`, `RunForSecondsTask`, `WaitUntilTask`,
`SequenceTask`, `ParallelAllTask`, ...) are useful when:

* You are building your **own helper factories** to share across a team.
* You need a special behavior that the existing factories don’t cover yet.

A good pattern:

> If you find yourself writing the same `Task` wiring several times,
> wrap it in a helper method so the next student can call one function
> instead of re‑creating the pattern.

---

## 8. Summary

* **Plants** represent mechanisms you can command with a numeric target.
* **Actuators.plant(...)** is the preferred way to create Plants from FTC
  hardware.
* **Position semantics differ**:

    * Motors + `.position(tolerance)` → feedback with encoders.
    * Servos + `.position()` → open‑loop set‑and‑hold.
* **PlantTasks** and **Tasks** provide factory helpers that build `Task`s
  for you – use them first.
* The **main loop shape** (Clock → Inputs → Tasks → Drive → Plants → Telemetry)
  stays the same for TeleOp and Autonomous.

Once you are comfortable with this Beginner’s Guide, the next steps are:

* Read the **Tasks & Macros Quickstart** for more details on the Task system.
* Read the **Shooter Case Study** for a concrete, end‑to‑end example.
* Explore other examples (`DriveTasks`, TagAim, vision) which all follow
  the same patterns described here.
