# Phoenix Beginner’s Guide

This guide is a gentle introduction to the Phoenix framework. It focuses on:

1. The **big ideas** (loop timing, inputs, tasks, plants, drive).
2. A minimal **TeleOp skeleton** that you can copy.
3. How to wire **hardware into Plants** using `FtcActuators.plant(...)`.
4. How to use **factory helpers** (`Tasks`, `PlantTasks`, `DriveTasks`) for common patterns.

If you’re new, you don’t need to know how everything works inside.
The goal is: **get a clean, non‑blocking TeleOp running quickly**.

---

## Where to look in the framework

If you’re writing robot code, you’ll spend almost all your time in a small set of packages:

* `edu.ftcphoenix.fw.actuation` — mechanisms as `Plant`s (`Plant`, `Plants`, `PlantTasks`).
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
* `edu.ftcphoenix.fw.tools.*` contains testers and copyable examples. If you just want to open the framework tester tree without writing any robot-specific glue first, run the ready-made `FW: Testers` OpMode.

If you’re curious *why* the packages are arranged this way, see [`Framework Overview`](<Framework Overview.md>) → Package structure.

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
        driveSource = new GamepadDriveSource(
                gamepads.p1().leftX(),
                gamepads.p1().leftY(),
                gamepads.p1().rightX(),
                GamepadDriveSource.Config.defaults()
        ).scaledWhen(gamepads.p1().rightBumper(), 0.35, 0.20);

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
        // Gamepad axes/buttons are Sources; they are sampled when you call get(...).
        bindings.update(clock);

        // 3) Macros (non-blocking tasks)
        macroRunner.update(clock);

        // 4) Drive (update first so rate limiting has the current dt)
        drivebase.update(clock);
        DriveSignal cmd = driveSource.get(clock).clamped();
        drivebase.drive(cmd);

        // 5) Mechanisms (Plants)
        double dt = clock.dtSec();
        shooter.update(clock);
        transfer.update(clock);
        pusher.update(clock);

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


### A principle that becomes important as robots get bigger

When your robot grows past a tiny example, do not hide button choices inside low-level framework primitives.
Keep the layers separated:

- **framework primitives** map signals, mix drive, estimate pose, or talk to hardware
- **framework lanes** own stable hardware/resource graphs like mecanum drive or localization
- **robot controls** choose sticks, buttons, slow mode, and driver/operator semantics
- **robot capabilities** expose the shared mode-neutral API that both TeleOp and Auto should use
- **robot policy** decides game-specific behavior like aiming, scoring, and macros

That separation keeps the framework reusable year to year and keeps each robot's control scheme obvious in one place.
For the full philosophy, see [`Framework Lanes & Robot Controls`](<../design/Framework Lanes & Robot Controls.md>) and [`Robot Capabilities & Mode Clients`](<../design/Robot Capabilities & Mode Clients.md>).

---

## 3. Wiring hardware as Plants with `FtcActuators.plant(...)`

To control hardware in Phoenix, you wrap it as a **Plant**.

The recommended way is to use the staged builder in
`edu.ftcphoenix.fw.ftc.FtcActuators`:

```java
private void initShooterPlants() {
    // Shooter: dual DC motors, device-managed velocity control with feedback.
    shooter = FtcActuators.plant(hardwareMap)
            .motor("shooterLeftMotor", Direction.FORWARD)
            .andMotor("shooterRightMotor", Direction.REVERSE)
            .velocity()
            .deviceManagedWithDefaults()
            .bounded(0.0, 2600.0)
            .nativeUnits()
            .velocityTolerance(100.0)
            .build();

    // Transfer: dual CR servos, power control.
    transfer = FtcActuators.plant(hardwareMap)
            .crServo("transferLeftServo", Direction.FORWARD)
            .andCrServo("transferRightServo", Direction.REVERSE)
            .power()
            .build();

    // Pusher: positional servo, commanded-position set-and-hold.
    pusher = FtcActuators.plant(hardwareMap)
            .servo("pusherServo", Direction.FORWARD)
            .position()
            .build();
}
```

### 3.1 What the builder is doing

The builder has three stages:

1. **Pick hardware**:

    * `.motor(name, direction)` then (optional) `.andMotor(name, direction)`
        * After you add a second motor, you may optionally calibrate the <i>last added</i>
          motor with `.scale(...)` / `.bias(...)` for device-managed grouped plants.
    * `.servo(name, direction)` then (optional) `.andServo(name, direction)`
    * `.crServo(name, direction)` then (optional) `.andCrServo(name, direction)`

2. **Pick the target domain**:

    * `.power()` – open-loop power (motors or CR servos).
    * `.velocity().deviceManagedWithDefaults().bounded(...).nativeUnits()` – motor velocity control.
    * `.position()` – position control.

3. **For position Plants, answer guided position questions**:

    * Motor position asks who manages the loop:
      * `.deviceManagedWithDefaults()` for FTC `RUN_TO_POSITION` defaults.
      * `.deviceManaged() ... .doneDeviceManaged()` when you want FTC motor tuning knobs.
      * `.regulated() ... .regulator(...)` when Phoenix should drive raw power from explicit feedback.
    * Position geometry asks topology and bounds:
      * `.linear()` or `.periodic(period)`
      * `.bounded(min, max)` or `.unbounded()`
    * Unit mapping/reference asks how plant units relate to native units:
      * `.nativeUnits()`, `.scaleToNative(...)`, or bounded-only `.rangeMapsToNative(...)`
      * then `.alreadyReferenced()`, `.plantPositionMapsToNative(...)`, `.assumeCurrentPositionIs(...)`, or `.needsReference(...)` when a runtime reference is required.

Then you may add plant-level modifiers like `.positionTolerance(...)`, `.rateLimit(...)`, and finish with `.build()`.

### 3.2 Position semantics: motors vs servos

The public builder surface stays parallel, but each hardware family exposes only choices that make
sense for it:

* **Motor position**:

    * `motor(...).position().deviceManagedWithDefaults()` uses FTC `RUN_TO_POSITION`.
    * `motor(...).position().deviceManaged() ... .doneDeviceManaged()` exposes optional FTC tuning knobs.
    * `motor(...).position().regulated().nativeFeedback(...).regulator(...)` uses a framework-owned regulator plus explicit native feedback.
    * Feedback-capable motor position Plants report plant-unit measurement and `atSetpoint()` status after `plant.update(clock)`.
    * `plant.reset()` clears transient controller state only. It does **not** redefine encoder zero or physical coordinate frame.

* **Regulated CR-servo position**:

    * CR servos have no device-managed position mode, so `crServo(...).position().regulated().nativeFeedback(...).regulator(...)` is required.
    * The feedback source can be an external encoder or custom source.

* **Servo position**:

    * Standard servos are command-only position outputs. Their builder exposes linear bounded position mapping only.
    * Use `.nativeUnits()` for raw servo units or `.rangeMapsToNative(...)` for logical units mapped to tuned raw endpoints.
    * `plant.hasFeedback() == false` and `plant.getMeasurement()` returns `NaN`, because the framework does not pretend a standard FTC servo has a true measured position.

Why this matters:

* For **feedback-based moves** (for example, “move arm to this angle and wait”), use a feedback
  plant: device-managed motor position/velocity or a regulated plant.
* For **simple servo motions** (pusher, claw), commanded servo position plants plus time-based waits
  are usually the right choice.

### 3.3 Device-managed vs regulated motor control

Use `deviceManagedWithDefaults()` for the common FTC motor-position path:

```java
PositionPlant arm = FtcActuators.plant(hardwareMap)
        .motor("armMotor", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(-300.0, 1200.0)
            .nativeUnits()
            .alreadyReferenced()
        .positionTolerance(20.0)
        .build();
```

Switch to a regulated path when you need an explicit feedback source or a custom regulator:

```java
PositionPlant arm = FtcActuators.plant(hardwareMap)
        .motor("armMotor", Direction.FORWARD)
        .position()
        .regulated()
            .nativeFeedback(FtcActuators.PositionFeedback.externalEncoder("armEncoder"))
            .regulator(ScalarRegulators.pid(Pid.withGains(0.006, 0.0, 0.0002)))
        .linear()
            .bounded(-300.0, 1200.0)
            .nativeUnits()
            .alreadyReferenced()
        .positionTolerance(20.0)
        .build();
```

### 3.4 Position-tolerance knobs: what they mean

For device-managed motor position plants there are **two different tolerance concepts**:

* `positionTolerance(...)`
    * Plant-level completion band used by `plant.atSetpoint()`.
    * Default: **10 ticks** for the built-in motor-position helpers.
    * This is the normal knob to use when you want to say “close enough for robot logic.”

* `devicePositionToleranceTicks(...)`
    * Optional override for the FTC motor controller's own target-position tolerance via
      `DcMotorEx.setTargetPositionTolerance(int)`.
    * Default in Phoenix: **unchanged unless you call it**.
    * Use this only when you intentionally want to change the FTC motor controller's internal
      completion threshold.

Related defaults:

* `maxPower(...)` default: **1.0**
* `velocityTolerance(...)` default for motor velocity plants: **100 plant velocity units**
* `outerPositionP(...)`, `innerVelocityPidf(...)`, and `velocityPidf(...)`: **unchanged unless set**

That separation keeps the common path simple: set the plant-level tolerance first, and only reach
for device-specific overrides when you actually need them.

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
    bindings.onRise(gamepads.p1().y(), () -> {
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

* `FtcActuators.plant(...)` to build Plants from FTC hardware.
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

* **FtcActuators.plant(...)** is the preferred way to create Plants from FTC hardware.

* **Position semantics differ**:

    * Motors + `.position().deviceManagedWithDefaults()` or `.position().regulated().nativeFeedback(...).regulator(...)` → feedback-capable position control.
    * Servos + `.position().linear().bounded(...).nativeUnits()` or `.rangeMapsToNative(...)` → open-loop set-and-hold.

* **PlantTasks** and **Tasks** provide factory helpers that build `Task`s for you.

* The **main loop shape** stays consistent:

  > Clock → Inputs → Bindings → Tasks → Drive → Plants → Telemetry

Next steps:

* Read [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) for the shared TeleOp + Auto interaction model.
* Read [`Tasks & Macros Quickstart`](<../design/Tasks & Macros Quickstart.md>) for deeper task patterns.
* Read [`Shooter Case Study & Examples Walkthrough`](<../examples/Shooter Case Study & Examples Walkthrough.md>) for an end-to-end example.
* Explore other examples (drive, tag aim, vision) — they all follow the same loop shape.
