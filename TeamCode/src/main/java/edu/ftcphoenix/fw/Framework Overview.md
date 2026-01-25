# Framework Overview

Phoenix is a small FTC framework that helps you structure robot code around a clean, repeatable loop.

The big idea is: **advance a single `LoopClock` once per OpMode cycle**, then run everything else (inputs, bindings, tasks, drive, mechanisms) off that clock.

---

## Package structure

Phoenix is organized by **robot concepts**, not by FTC SDK details.

### Packages students use day-to-day

Most robot code should only need imports from these packages:

* `edu.ftcphoenix.fw.input` — gamepad wrappers (`Gamepads`, `GamepadDevice`, `Axis`, `Button`).
* `edu.ftcphoenix.fw.input.binding` — `Bindings`: map button edges to actions.
* `edu.ftcphoenix.fw.task` — `Task`, `TaskRunner`, `Tasks`: non-blocking macros over time.
* `edu.ftcphoenix.fw.actuation` — `Plant`, `Actuators`, `PlantTasks`: mechanisms you command with numeric targets.
* `edu.ftcphoenix.fw.drive` — `DriveSignal`, `DriveSource`, `MecanumDrivebase` (FTC-independent drive logic).
* `edu.ftcphoenix.fw.ftc` — FTC entrypoints/adapters (e.g. `FtcDrives` for drivetrain wiring).
* `edu.ftcphoenix.fw.sensing` — sensor-facing wrappers (vision, odometry, etc.).
* `edu.ftcphoenix.fw.localization` — pose estimation (AprilTags, odometry, fusion).
* `edu.ftcphoenix.fw.field` — field metadata (tag layouts, constants).

Within `drive/`, subpackages are intentionally parallel and predictable:

* `drive.source` — “where drive commands come from” (gamepad, autonomous logic).
* `drive.guidance` — driver-assist building blocks (auto-aim, “go-to point”, pose lock, etc.).
* `drive.control` — closed-loop drive behaviors/controllers/tasks (go-to-pose, heading controllers).

### Packages that are intentionally “behind the scenes”

These exist so the student-facing packages stay small and consistent:

* `edu.ftcphoenix.fw.core.*` — shared plumbing: time, math, geometry, control, debug, and the HAL.
* `edu.ftcphoenix.fw.ftc.*` — the **FTC SDK boundary** (hardware adapters, frame conversions, FTC vision plumbing). Most teams only touch a couple entrypoints like `FtcDrives`.
* `edu.ftcphoenix.fw.tools.*` — testers and examples you can copy.
* `edu.ftcphoenix.fw.legacy.*` — intentionally retained older base classes (not recommended for new code).

One important gotcha with FTC vision:

* Anything backed by a `VisionPortal` **owns the camera**. When you are done with a vision tester/OpMode,
  make sure the portal is closed so the next tester can start the camera cleanly.
  In Phoenix, `AprilTagSensor` has a `close()` method for this purpose—call it on `stop()`/BACK navigation.

One more gotcha that matters a lot for AprilTags:

### Coordinate frames and 3rd-party conventions (especially AprilTags)

Phoenix uses `Pose2d`/`Pose3d` heavily. Those are just *math objects* — they don't know what "+X" means
unless you define the frame.

Phoenix's standard convention is right-handed:

* Units: **inches** (translation) and **radians** (angles).
* Rotations:
  * yaw = rotation about **+Z** (turning left / CCW is positive)
  * pitch = rotation about **+Y**
  * roll = rotation about **+X**

For field-centric work, Phoenix uses the **FTC Field Coordinate System**:

* Origin at the field center
* +Z up
* Stand at the **Red Wall center** facing the field: +X is to your right, +Y is away from the Red Wall

AprilTags add an extra complication: FTC exposes *multiple* pose frames.

* **Game database / layout** (`AprilTagGameDatabase` → `TagLayout`)
  * Tag poses come from FTC metadata (`fieldPosition` + `fieldOrientation`).
* **Detections** (`AprilTagDetection`)
  * `rawPose` is the **native AprilTag/OpenCV camera frame** (+X right, +Y down, +Z forward).
  * `ftcPose` is a **convenience re-frame** (+X right, +Y forward, +Z up) with yaw/pitch/roll labels.

Phoenix's rule of thumb:

* If you are composing detections with the FTC game database (localization, camera mount calibration),
  use `rawPose` (then convert it into Phoenix camera axes). Mixing `ftcPose` with game database
  metadata can put you in two different coordinate systems and produce nonsense transforms.

Implementation pointers:

* `fw.ftc.FtcFrames` documents the basis transforms and exposes the conversion matrices.
* `fw.ftc.FtcVision` builds `AprilTagObservation.cameraToTagPose` from `rawPose` and converts it into
  Phoenix camera axes (+X forward, +Y left, +Z up).

Tester naming conventions (telemetry menus):

* Prefix calibration routines with `Calib:`
* Prefix localization/pose display tools with `Loc:`
* Prefix hardware sanity checks with `HW:`
* If a tester is robot-specific, make that obvious in the label (for example, append `(Robot)` or include the robot name)

Two rules of thumb:

1. If a class references FTC SDK types (`com.qualcomm.*`), it belongs in `fw.ftc` or `fw.tools`, not in the core building blocks.
2. The package tree is kept **parallel** on purpose (for example, `sensing.vision` ↔ `localization.apriltag`, and later `sensing.odometry` ↔ `localization.odometry`). That makes it easy to predict where new features should go.

---

## The layers (top → bottom)

Think of Phoenix as a few thin layers you stack:

1. **OpMode / Robot code (you)**

    * Owns the loop and decides what updates when.
2. **Input** (`fw.input`)

    * `Gamepads`, `GamepadDevice`, `Axis`, `Button`.
    * `Button` supports edge detection (`onPress`/`onRelease`), <i>and</i> a built-in
      press-to-toggle state via `Button.isToggled()` (useful when enabling drive overlays).
3. **Bindings** (`fw.input.binding`)

    * `Bindings` turns button edges into actions (often: enqueue a macro).
4. **Tasks / Macros** (`fw.task`, plus helpers in other packages)

    * `Task`, `TaskRunner`, `Tasks`, `PlantTasks`, `DriveTasks`.
5. **Drive behavior** (`fw.drive` + `fw.drive.source` + `fw.drive.guidance`)

    * `DriveSource` produces a `DriveSignal` (stick drive, driver assist overlays, etc.).
6. **Actuation**

    * Drivebase: `MecanumDrivebase`.
    * Mechanisms: `Plant`.
7. **Core HAL** (`fw.core.hal`)

    * Tiny device-neutral interfaces: `PowerOutput`, `PositionOutput`, `VelocityOutput`.
8. **FTC boundary** (`fw.ftc`)

    * `FtcHardware` wraps FTC SDK hardware into Phoenix HAL outputs.

---

## The loop clock (and why it matters)

`LoopClock` (in `fw.core.time`) tracks:

* `nowSec()` — current time
* `dtSec()` — delta time since last loop
* `cycle()` — a monotonically increasing **per-loop id**

Several Phoenix systems are **idempotent by `clock.cycle()`** (safe if accidentally called twice in the same loop), including:

* `Gamepads.update(clock)`
* `Bindings.update(clock)`
* `TaskRunner.update(clock)`
* `Button.updateAllRegistered(clock)`

This prevents bugs like “button press fired twice” or “tasks advanced twice” when helper code gets layered.

---

## Hardware and mechanisms: `Actuators`, `Plant`, and the HAL

### HAL outputs (lowest level)

Phoenix abstracts FTC hardware into small output interfaces (in `fw.core.hal`):

* `PowerOutput` — normalized power (typically `[-1, +1]`)
* `PositionOutput` — native position units (servo `0..1`, motor encoder ticks, etc.)
* `VelocityOutput` — native velocity units (e.g., ticks/sec)

### FTC boundary: `FtcHardware`

`edu.ftcphoenix.fw.ftc.FtcHardware` provides factories like:

* `FtcHardware.motorPower(hw, name, direction)`
* `FtcHardware.motorVelocity(hw, name, direction)`
* `FtcHardware.motorPosition(hw, name, direction)`
* `FtcHardware.servoPosition(hw, name, direction)`
* `FtcHardware.crServoPower(hw, name, direction)`

These return HAL outputs.

### Beginner entrypoint: `Actuators`

Most teams should **not** call `FtcHardware` directly. Use the staged builder in `Actuators`:

```java
import edu.ftcphoenix.fw.core.hal.Direction;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;

// Shooter: dual-motor velocity plant (native units) with a rate limit.
Plant shooter = Actuators.plant(hardwareMap)
        .motor("shooterLeftMotor", Direction.FORWARD)
        .andMotor("shooterRightMotor", Direction.REVERSE)
        .velocity()            // default tolerance (native units)
        .rateLimit(500.0)      // max delta in native units per second
        .build();

// Transfer: CR servo power plant.
Plant transfer = Actuators.plant(hardwareMap)
        .crServo("transferServo", Direction.FORWARD)
        .power()
        .build();

// Pusher: positional servo plant (0..1).
Plant pusher = Actuators.plant(hardwareMap)
        .servo("pusherServo", Direction.FORWARD)
        .position()
        .build();
```

**Important:** tasks can set targets on plants, but *your loop* must still call `plant.update(dtSec)` each cycle.

---

## Drive: `DriveSignal`, `DriveSource`, and `MecanumDrivebase`

### `DriveSignal` (robot-centric command)

A `DriveSignal` is **robot-centric** and follows Phoenix pose conventions:

* `axial > 0` → forward
* `lateral > 0` → left
* `omega > 0` → counter-clockwise (CCW)

### `DriveSource` (where commands come from)

A `DriveSource` produces a `DriveSignal` each loop:

* Manual TeleOp: `GamepadDriveSource`
* Assisted aiming / guidance: `DriveGuidance` (build a plan) + {@code DriveSource.overlayWhen(...) }
* Autonomous logic: anything implementing `DriveSource`

`DriveSource` also supports composition helpers (like scaling and blending) via default methods.

### `MecanumDrivebase` + `FtcDrives`

`FtcDrives.mecanum(hardwareMap)` is the beginner-friendly way to wire a mecanum drivetrain.

```java
import edu.ftcphoenix.fw.drive.*;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.input.Gamepads;

Gamepads pads = Gamepads.create(gamepad1, gamepad2);

MecanumDrivebase drivebase = FtcDrives.mecanum(hardwareMap);
DriveSource drive = GamepadDriveSource.teleOpMecanumStandard(pads);
```

**Rate limiting note:** `MecanumDrivebase` can rate-limit components using the most recent `dtSec`. Call `drivebase.update(clock)` once per loop. If you want rate limiting to use the *current* loop’s `dt`, call `update(clock)` **before** `drive(...)`.

### Spatial reasoning layers (math → predicates → controllers)

Phoenix tries to keep “spatial logic” reusable by splitting it into three layers:

1. **Spatial math** (pure geometry): `Pose2d`, `SpatialMath2d`, region/shape primitives.
2. **Spatial predicates** (answering yes/no): `RobotZones2d`, `RobotHeadings2d` (often with
   hysteresis latches).
3. **Controllers** (turn errors into commands): `DriveGuidanceSpec` + `DriveGuidancePlan` and their
   overlays/tasks/queries.

If you only want to ask “is the robot in the zone?” or “is the robot aimed?”, use layer (2). If you
want the framework to *drive*, use layer (3).

---

## Tasks and macros

### `Task` and `TaskRunner`

A `Task` is non-blocking work that progresses over multiple loop cycles.

A `TaskRunner` runs tasks **sequentially** (FIFO): start one task, update it each cycle until it completes, then move to the next.

### Factories: `Tasks`, `PlantTasks`, `DriveTasks`

Phoenix gives you factories so your code reads like intent:

* `Tasks` — general composition (`sequence`, `parallelAll`, `waitForSeconds`, `waitUntil`, `runOnce`, …)
* `PlantTasks` — patterns that command a `Plant` (`setInstant`, `holdFor`, `moveTo`, …)
* `DriveTasks` — simple patterns that command a `MecanumDrivebase` (`driveForSeconds`, `stop`, …)
* `DriveGuidanceTasks` — execute a `DriveGuidancePlan` as a Task (autonomous-style guidance)
* `GoToPoseTasks` — convenience wrappers for common go-to-pose behaviors (`goToPoseFieldRelative`, `goToPoseTagRelative`, …)

Example macro (shoot one disc):

```java
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;

private Task buildShootOneDiscMacro(Plant shooter, Plant transfer) {
    return Tasks.sequence(
            PlantTasks.setInstant(shooter, 3200.0),
            Tasks.waitUntil(shooter::atSetpoint, 1.0),
            PlantTasks.holdForThen(transfer, 1.0, 0.20, 0.0)
    );
}
```

---

## Inputs and bindings

### `Gamepads`

`Gamepads` wraps FTC `gamepad1` / `gamepad2` and exposes calibrated axes and edge-tracked buttons.

Call **once per loop**:

```java
gamepads.update(clock);
```

### `Bindings`

`Bindings` lets you map button edges to actions.

Most commonly: **enqueue a macro** on press.

```java
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.TaskRunner;

Bindings bindings = new Bindings();
TaskRunner macros = new TaskRunner();

bindings.onPress(gamepads.p1().y(), () ->
        macros.enqueue(buildShootOneDiscMacro(shooter, transfer))
);
```

Call **once per loop** (after `gamepads.update(clock)`):

```java
bindings.update(clock);
```

---

## A standard OpMode loop shape

This is the “everything has a place” pattern Phoenix is built around:

```java
@Override
public void start() {
    clock.reset(getRuntime());
}

@Override
public void loop() {
    // 1) Clock
    clock.update(getRuntime());

    // 2) Inputs
    gamepads.update(clock);

    // 3) Bindings (may enqueue macros)
    bindings.update(clock);

    // 4) Tasks / macros
    macroRunner.update(clock);

    // 5) Drive
    DriveSignal cmd = driveSource.get(clock).clamped();
    drivebase.update(clock);   // call before drive(...) if you want current-dt rate limiting
    drivebase.drive(cmd);

    // 6) Mechanisms
    double dtSec = clock.dtSec();
    shooter.update(dtSec);
    transfer.update(dtSec);

    // 7) Telemetry
    telemetry.update();
}
```

---

## Where to go next

* **Beginner’s Guide** — first setup + “how to write a Phoenix OpMode”.
* **Framework Principles** — the rules-of-thumb Phoenix expects you to follow.
* **Loop Structure** — deeper reasoning about update order and idempotency.
* **Tasks & Macros Quickstart** — how to build task graphs quickly.
* **Shooter Case Study & Examples Walkthrough** — maps concepts to real examples in `fw.tools.examples`.