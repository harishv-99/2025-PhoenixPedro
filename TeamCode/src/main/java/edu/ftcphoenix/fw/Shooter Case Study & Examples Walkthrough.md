# Shooter Case Study & Examples Walkthrough

This document walks through the framework’s shooter examples (02–06) and explains *why* each step exists, using only real APIs from the Phoenix framework.

The examples live in `edu.ftcphoenix.fw.tools.examples.*`:

* **TeleOp_01_MecanumBasic** – baseline mecanum TeleOp (the foundation all later examples build on)
* **TeleOp_02_ShooterBasic** – basic “modes” + plants
* **TeleOp_03_ShooterMacro** – one-button macro using `TaskRunner` + `PlantTasks`
* **TeleOp_04_ShooterInterpolated** – distance → velocity via `InterpolatingTable1D`
* **TeleOp_05_ShooterTagAimVision** – AprilTag distance + DriveGuidance (manual drive + auto-omega)
* **TeleOp_06_ShooterTagAimMacroVision** – combines DriveGuidance + vision distance + macro

---

## The common pattern across all shooter examples

### 1) Wire hardware as `Plant`s (Actuators builder)

All shooter examples use `Actuators.plant(hardwareMap)` to build `Plant` instances:

```java
import edu.ftcphoenix.fw.core.hal.Direction;

Plant shooter = Actuators.plant(hardwareMap)
        .motor("shooterLeftMotor", Direction.FORWARD)
        .andMotor("shooterRightMotor", Direction.REVERSE)
        .velocity(/*toleranceNative=*/100.0)
        .build();

Plant transfer = Actuators.plant(hardwareMap)
        .crServo("transferLeftServo", Direction.FORWARD)
        .andCrServo("transferRightServo", Direction.REVERSE)
        .power()
        .build();

Plant pusher = Actuators.plant(hardwareMap)
        .servo("pusherServo", Direction.FORWARD)
        .position()
        .build();
```

Key idea: **plants only accept a scalar target** (`setTarget(...)`). Your loop (or a task) decides targets; the plant applies them when you call `update(dtSec)`.

### 2) Use `LoopClock`, update inputs, then update bindings/tasks

The examples share this “spine”:

```java
clock.update(getRuntime());
double dtSec = clock.dtSec();

gamepads.update(clock);
bindings.update(clock);        // if used
macroRunner.update(clock);      // if used

// Decide targets...

// Drive...

// Plant updates...
shooter.update(dtSec);
transfer.update(dtSec);
pusher.update(dtSec);
```

Notes:

* `Bindings.update(clock)`, `TaskRunner.update(clock)`, and `TagTarget.update(clock)` are **idempotent by `clock.cycle()`**. Calling them twice in one loop cycle becomes a no-op (safety against accidental double-updates).
* `MecanumDrivebase.update(clock)` exists to provide loop timing for optional rate limiting. Call `drivebase.update(clock)` **before** `drivebase.drive(cmd)` so the current loop’s `dt` is used (the examples follow this order). `drivebase.drive(...)` applies motor power **immediately**; `update(...)` does not move motors by itself.

---

## Example 02: Shooter Basic (modes + plants)

**File:** `TeleOp_02_ShooterBasic`

### What it teaches

* How to create `Plant`s for a shooter motor pair, transfer CR servo pair, and a pusher servo.
* How to use `Bindings.onPress(...)` to change a *mode*, then apply targets from that mode every loop.

### Structure

* Uses small enums (e.g., transfer mode and pusher mode).
* Button presses change the enums.
* Each loop converts enum → `setTarget(...)`.

### Controls note

This example keeps `teleOpMecanumStandard(...)`, so **RB remains slow mode**. To avoid a conflict, the shooter toggle is on **LB**.

---

## Example 03: Shooter Macro (Tasks + PlantTasks)

**File:** `TeleOp_03_ShooterMacro`

### What it teaches

* A “macro” is just a `Task` that *changes plant targets over time*.
* `TaskRunner` runs tasks sequentially, one at a time.
* `PlantTasks` provides *ready-made tasks* for common plant patterns.

### The macro behavior

When P1 **Y** is pressed:

1. **Spin up** shooter to a target velocity and wait for `Plant.atSetpoint()` with a timeout.
2. In parallel:

    * pulse transfer power for a short time
    * step pusher through positions using timed holds
3. **Spin down** shooter.

### Real code pattern (from the example)

```java
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

Task pusherLoad = PlantTasks.holdFor(pusher, PUSHER_POS_LOAD,  PUSHER_STAGE_SEC);
Task pusherShoot = PlantTasks.holdForThen(pusher, PUSHER_POS_SHOOT, PUSHER_STAGE_SEC, PUSHER_POS_RETRACT);

Task feedPusher = SequenceTask.of(pusherLoad, pusherShoot);
Task feedBoth   = ParallelAllTask.of(feedTransfer, feedPusher);

Task spinDown = PlantTasks.setInstant(shooter, 0.0);

Task macro = SequenceTask.of(spinUp, feedBoth, spinDown);
```

### Two rules to remember

* `PlantTasks.moveTo(...)` **requires a feedback-capable plant** (`plant.hasFeedback() == true`). That’s why the shooter is built as `.velocity(...)`.
* Timed patterns like `holdFor(...)` / `holdForThen(...)` are perfect for servo plants (no feedback).

---

## Example 04: Shooter Interpolated (manual “distance”)

**File:** `TeleOp_04_ShooterInterpolated`

### What it teaches

* How to represent calibration data as an `InterpolatingTable1D`.
* How to separate **where distance comes from** (here: D-pad) from **how it’s used** (distance → velocity).

### Real table API

The table is constructed with sorted pairs:

```java
private static final InterpolatingTable1D SHOOTER_VELOCITY_TABLE =
        InterpolatingTable1D.ofSortedPairs(
                24.0, 170.0,
                30.0, 180.0,
                36.0, 195.0,
                42.0, 210.0,
                48.0, 225.0
        );
```

Then in loop:

```java
double targetVel = SHOOTER_VELOCITY_TABLE.interpolate(distanceInches);
shooter.setTarget(shooterEnabled ? targetVel : 0.0);
```

---

## Example 05: Shooter + DriveGuidance Auto-Aim + Vision Distance

**File:** `TeleOp_05_ShooterTagAimVision`

### What it teaches

* How to get AprilTag observations through the FTC adapter: `FtcVision.aprilTags(...)`.
* How `TagTarget` tracks the “best” tag across loops with an age constraint.
* How a `DriveGuidance` plan becomes a `DriveOverlay` that overrides only **omega** while a button is held.
* How to compute shooter velocity from **tag distance** using the same interpolation-table idea from Example 04.

### Wiring

```java
AprilTagSensor tagSensor = FtcVision.aprilTags(hardwareMap, "Webcam 1");
TagTarget scoringTarget = new TagTarget(tagSensor, SCORING_TAG_IDS, MAX_TAG_AGE_SEC);

CameraMountConfig cameraMount = CameraMountConfig.of(
        /*xInches=*/6.0,
        /*yInches=*/-3.0,
        /*zInches=*/8.0,
        /*yawRad=*/0.0,
        /*pitchRad=*/0.0,
        /*rollRad=*/0.0
);

DriveSource baseDrive = GamepadDriveSource.teleOpMecanumStandard(gamepads);

// Convert TagTarget → generic robot-relative observation.
ObservationSource2d scoringObs = ObservationSources.aprilTag(scoringTarget, cameraMount);

DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .aimTo()
            // Aim at the center of whichever scoring tag is currently observed.
            .tagRelativePointInches(0.0, 0.0)
            .doneAimTo()
        .feedback()
            .observation(scoringObs)
            .doneFeedback()
        .build();

DriveSource driveWithAim = baseDrive.overlayWhen(
        () -> gamepads.p1().leftBumper().isHeld(),
        aimPlan.overlay(),
        DriveOverlayMask.OMEGA_ONLY
);
```

### Loop highlights

* Call `scoringTarget.update(clock)` once per cycle.
* Read `AprilTagObservation obs = scoringTarget.last()`.
* Use `obs.cameraRangeInches()` and `obs.cameraBearingRad()`.

This example also demonstrates computing **robot-centric bearing** that accounts for the camera offset:

```java
double robotBearingRad = obs.hasTarget
        ? CameraMountLogic.robotBearingRad(obs, cameraMount)
        : 0.0;
```

Shooter target decision:

```java
if (shooterEnabled && obs.hasTarget) {
    double targetVel = SHOOTER_VELOCITY_TABLE.interpolate(obs.cameraRangeInches());
    shooter.setTarget(targetVel);
} else {
    shooter.setTarget(0.0);
}
```

---

## Example 06: DriveGuidance Auto-Aim + Vision Distance + Macro

**File:** `TeleOp_06_ShooterTagAimMacroVision` (auto-aim is now implemented with `DriveGuidance`)

### What it teaches

* How to combine Example 03 (macro) with Example 05 (vision distance).
* How to *gate a macro* on “do we have a valid tag right now?”.

### The key idea

When P1 **Y** is pressed:

1. Read the latest tag observation from `TagTarget`.
2. If there is no tag: **don’t start the macro**.
3. If there is a tag: compute shooter velocity from the table, then build the macro using that velocity.

Real logic from the example:

```java
AprilTagObservation obs = scoringTarget.last();
if (!obs.hasTarget) {
    lastMacroStatus = "no tag: macro not started";
    return;
}

double shooterTargetVel = SHOOTER_VELOCITY_TABLE.interpolate(obs.cameraRangeInches());
Task macro = buildShootOneBallMacro(shooterTargetVel);
macroRunner.clear();
macroRunner.enqueue(macro);
```

### Macro composition (example’s version)

This example uses timed holds for each pusher stage:

```java
Task pusherLoad    = PlantTasks.holdFor(pusher, PUSHER_POS_LOAD,    PUSHER_STAGE_SEC);
Task pusherShoot   = PlantTasks.holdFor(pusher, PUSHER_POS_SHOOT,   PUSHER_STAGE_SEC);
Task pusherRetract = PlantTasks.holdFor(pusher, PUSHER_POS_RETRACT, PUSHER_STAGE_SEC);

Task feedPusher = SequenceTask.of(pusherLoad, pusherShoot, pusherRetract);
```

---

## Troubleshooting checklist when adapting these examples

* **Camera name**: examples use `"Webcam 1"`. Your Robot Configuration name must match.
* **Tag IDs**: update `SCORING_TAG_IDS` for the real game tags you care about.
* **Mount axes**: `CameraMountConfig.of(x, y, z, yaw, pitch, roll)` uses Phoenix framing:

    * +X forward, +Y left, +Z up
    * *Right* is negative Y.
* **Feedback vs no feedback**:

    * Use `PlantTasks.moveTo(...)` only for plants with feedback (motor velocity/position plants).
    * Use timed holds for servos/CR servos.

---

## What to do next

* If you like the structure of Example 06, the next extension is usually:

    * “shoot N balls” (repeat the single-ball macro in a `SequenceTask`), and/or
    * add a *min-range / max-range* check before starting the macro, and/or
    * pass `cameraMount` into `FtcVision.aprilTags(..., cfg)` via `FtcVision.Config.defaults().withCameraMount(cameraMount)` if you want the FTC SDK’s robot-pose estimation path enabled (separate from DriveGuidance).
