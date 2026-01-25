# Phoenix Framework Loop Structure

This document describes the recommended structure for a Phoenix robot loop (FTC `OpMode.loop()`), and how the major framework pieces fit into it: `Gamepads`, `Bindings`, `TaskRunner`, `Plants`, `TagTarget`, `TagAim`, `DriveSource`, and `Drivebase`.

The goal is to give teams a **simple mental model**:

> **Clock → Sense → Decide → Control → Report**

If every OpMode follows this ordering, it becomes much easier to:

* Avoid subtle "stale data" bugs.
* Reason about why the robot did something on a particular loop.
* Introduce more advanced features (vision, macros, autonomous logic) without breaking existing code.

---

## 1. High-Level Loop Cycle

Each call to `loop()` should follow this order:

1. **Clock** – compute the timestep (`dtSec`).
2. **Sense** – read all inputs and sensors (gamepads, TagTarget, odometry, IMU, etc.).
3. **Decide** – compute high-level decisions (bindings, macros, state machines).
4. **Control / Actuate** – turn decisions into motor/servo commands via Plants and drivebase.
5. **Report** – send telemetry and debug information.

In pseudocode:

```java
@Override
public void loop() {
    // 1) Clock
    clock.update(getRuntime());
    double dtSec = clock.dtSec();

    // 2) Sense (inputs & sensors)
    gamepads.update(dtSec);
    scoringTarget.update();      // TagTarget → AprilTagSensor.best(...)
    // odometry.update(dtSec);
    // imuWrapper.update(dtSec);

    // 3) Decide (high-level logic)
    bindings.update(dtSec);      // may enqueue tasks, toggle modes, etc.
    macroRunner.update(clock);   // advances macros, sets plant targets
    // other state machines / planners here

    // 4) Control / Actuate (subsystems)
    DriveSignal cmd = driveWithAim.get(clock).clamped();
    drivebase.drive(cmd);
    drivebase.update(clock);

    shooter.update(dtSec);
    transfer.update(dtSec);
    pusher.update(dtSec);

    // 5) Report (telemetry & debug)
    telemetry.addData("omega", cmd.omega);
    telemetry.update();
}
```

---

## 2. Phase Details

### 1) Clock

Everything in the loop depends on a consistent sense of time. Update the `LoopClock` first so that all subsequent calls in this loop see the same timestamp and `dt`:

```java
clock.update(getRuntime());
double dtSec = clock.dtSec();
```

**Guideline:** do not call `clock.update(...)` multiple times per loop. All time-based logic should use the same `dtSec`.

---

### 2) Sense – Inputs & Sensors

"Sense" is where the robot reads the world for this loop:

* **Gamepads** (user intent):

    * `gamepads.update(dtSec);`
* **Vision** (e.g., AprilTags):

    * `scoringTarget.update();` (which internally calls `AprilTagSensor.best(...)`).
* **Other sensors**:

    * Odometry, IMU, distance sensors, etc., via their own wrappers.

Example:

```java
// Gamepad state (sticks, buttons, triggers)
gamepads.update(dtSec);

// Tag-based target tracking (TagTarget)
scoringTarget.update();

// Other sensors
// odometry.update(dtSec);
// imuWrapper.update(dtSec);
```

**Why this phase is important:**

Any later code that makes decisions based on sensors (e.g. starting a shooting macro based on the current tag distance) needs a fresh snapshot of the world. That only happens if you call things like `TagTarget.update()` before that decision code runs.

---

### 3) Decide – High-Level Logic

"Decide" is where you choose **what** the robot should do this loop, based on the inputs you just read.

This includes:

* **Bindings** (user-input–driven decisions):

    * Mapping button presses to actions.
    * Starting or cancelling macros.
    * Toggling modes (e.g., slow mode, auto-aim on/off).
* **TaskRunner / macros**:

    * High-level sequences that control Plants and other systems over time.
* **Other state machines or planners**:

    * Autonomous step logic.
    * Mode controllers.

Example:

```java
// User-input driven decisions.
bindings.update(dtSec);      // might call startShootOneBallMacro(), etc.

// Higher-level decisions / macros.
macroRunner.update(clock);   // drives PlantTasks, updates plant targets

// Other state machines/planners (if any)
// autoController.update(clock, odometry, scoringTarget.last());
```

By the end of this phase, your code should have:

* Decided which tasks/macros are active.
* Set desired targets on Plants (e.g., shooter velocity, servo positions).
* Chosen which DriveSource is currently controlling the drivetrain (e.g., base sticks vs. TagAim-wrapped drive).

**Important separation:**

* **Decide** sets *targets* and *modes*.
* **Control / Actuate** (the next phase) turns those targets into actual motor outputs.

This separation keeps it clear where the "brain" lives vs. where the "muscles" live.

---

### 4) Control / Actuate – Subsystems

In this phase, you run the controllers that translate decisions into motor & servo commands.

This is where you:

* Ask a `DriveSource` for a `DriveSignal`.
* Feed the `DriveSignal` to the `Drivebase`.
* Call `update(...)` on all Plants and subsystem controllers.

Example:

```java
// Drive: TagAim-wrapped drive source (may override omega while aim button held)
DriveSignal cmd = driveWithAim.get(clock).clamped();
drivebase.drive(cmd);
drivebase.update(clock);

// Mechanisms: shooter, transfer, pusher, etc.
shooter.update(dtSec);
transfer.update(dtSec);
pusher.update(dtSec);
```

How the pieces fit:

* `driveWithAim` is typically built via:

    * `TagAim.teleOpAim(baseDrive, aimButton, scoringTarget, config)`.
    * Internally, this uses `TagAimDriveSource` and `TagAimController`.
* Plants (e.g. `shooter`, `transfer`, `pusher`) encapsulate:

    * Control loops (e.g., PID or simple proportional control).
    * Hardware access (motors/servos via HAL adapters).

**Guideline:**

* Do not change plant targets *after* calling `plant.update()` in the same loop.
* Keep all `update(...)` calls together in this phase for readability and predictability.

---

### 5) Report – Telemetry & Debug

Finally, report what happened this loop. This phase should not change behavior; it only observes.

Typical things to report:

* Tag info: `hasTarget`, `id`, `rangeInches`, `bearingRad`, `ageSec`.
* Drive commands: axial, lateral, omega.
* Macro status: active/not, current step, target shooter velocity.
* Plant debug information (optional via `debugDump(...)`).

Example:

```java
telemetry.addLine("Drive")
        .addData("axial",   lastDrive.axial)
        .addData("lateral", lastDrive.lateral)
        .addData("omega",   lastDrive.omega);

AprilTagObservation obs = scoringTarget.last();
telemetry.addLine("Tag")
        .addData("hasTarget", obs.hasTarget)
        .addData("id",         obs.id)
        .addData("rangeIn",    obs.rangeInches)
        .addData("bearingRad", obs.bearingRad)
        .addData("ageSec",     obs.ageSec);

telemetry.addLine("Macro")
        .addData("active", macroRunner.hasActiveTask())
        .addData("status", lastMacroStatus);

telemetry.update();
```

Optional debug tracing:

```java
// Example: dump TagTarget state for logging.
scoringTarget.debugDump(debugSink, "tags.scoring");

// Example: dump TagAimDriveSource state (last bearing, last omega, etc.).
tagAimDriveSource.debugDump(debugSink, "drive.tagAim");
```

---

## 3. Where TagTarget and TagAim Fit

Tag-related classes fit naturally into this loop structure:

* **TagTarget**

    * Lives in the **Sense** phase.
    * `TagTarget.update()` calls `AprilTagSensor.best(...)` once per loop.
    * `TagTarget.last()` is used later in Decide/Control.

* **TagAim**

    * Wiring happens at init:

      ```java
      scoringTarget = new TagTarget(tagSensor, SCORING_TAG_IDS, MAX_TAG_AGE_SEC);
      driveWithAim = TagAim.teleOpAim(
              baseDrive,
              gamepads.p1().leftBumper(),
              scoringTarget
      );
      ```
    * At runtime, the **Control** phase calls `driveWithAim.get(clock)`, which:

        * Samples the current bearing via `scoringTarget.toBearingSample()`.
        * Uses `TagAimController` to compute omega.
        * Overrides only the turn component while the aim button is held.

* **Shooters and other mechanisms**

    * Use `TagTarget.last().rangeInches` in the **Decide** phase (e.g., when starting a macro) to compute desired velocities.
    * Plants then drive the hardware in the **Control** phase.

This ensures that:

* Aiming and shooter logic both see the **same** tag observation.
* Vision is read once per loop and shared across subsystems.

---

## 4. Example Skeleton TeleOp Using This Pattern

A minimal TeleOp that follows the structure:

```java
public final class MyTeleOp extends OpMode {
    private final LoopClock clock = new LoopClock();

    private Gamepads gamepads;
    private Bindings bindings;

    private MecanumDrivebase drivebase;
    private DriveSource baseDrive;
    private DriveSource driveWithAim;

    private AprilTagSensor tagSensor;
    private TagTarget scoringTarget;

    private Plant shooter;
    private final TaskRunner macroRunner = new TaskRunner();

    private DriveSignal lastDrive = new DriveSignal(0.0, 0.0, 0.0);

    @Override
    public void init() {
        gamepads = Gamepads.create(gamepad1, gamepad2);
        bindings = new Bindings();

        drivebase = Drives.mecanum(hardwareMap);
        baseDrive = GamepadDriveSource.teleOpMecanumStandard(gamepads);

        tagSensor = FtcVision.aprilTags(hardwareMap, "Webcam 1");
        scoringTarget = new TagTarget(tagSensor, SCORING_TAG_IDS, 0.5);

        driveWithAim = TagAim.teleOpAim(
                baseDrive,
                gamepads.p1().leftBumper(),
                scoringTarget
        );

        shooter = Actuators.plant(hardwareMap)
                .motor("shooterMotor", false)
                .velocity(100.0)
                .build();

        // Bindings, macros, etc.
        // bindings.onPress(...);
    }

    @Override
    public void start() {
        clock.reset(getRuntime());
    }

    @Override
    public void loop() {
        // 1) Clock
        clock.update(getRuntime());
        double dtSec = clock.dtSec();

        // 2) Sense
        gamepads.update(dtSec);
        scoringTarget.update();

        // 3) Decide
        bindings.update(dtSec);
        macroRunner.update(clock);

        // 4) Control / Actuate
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        lastDrive = cmd;

        drivebase.drive(cmd);
        drivebase.update(clock);

        shooter.update(dtSec);

        // 5) Report
        telemetry.addData("axial", lastDrive.axial);
        telemetry.addData("omega", lastDrive.omega);
        telemetry.update();
    }
}
```

This structure is intentionally repetitive: once students internalize **Clock → Sense → Decide → Control → Report**, it becomes much easier to slot new features (vision, macros, autonomous planning) into the right phase without breaking existing behavior.
