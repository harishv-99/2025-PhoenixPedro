# Framework Overview

Phoenix is a small FTC framework that helps you structure robot code around a clean, repeatable loop.

The big idea is: **advance a single `LoopClock` once per OpMode cycle**, then run everything else (inputs, bindings, tasks, drive, mechanisms) off that clock.

Useful companions to this document are [`Framework Lanes & Robot Controls`](<../design/Framework Lanes & Robot Controls.md>), [`Robot Capabilities & Mode Clients`](<../design/Robot Capabilities & Mode Clients.md>), and [`Recommended Robot Design`](<../design/Recommended Robot Design.md>). Together they explain the ownership vocabulary, how to split framework lanes from robot code, how TeleOp and Auto should share mechanism APIs through robot-owned capability families, and how to choose the right internal behavior lane for a mechanism.

See the repository [`README`](<../../README.md>) for the full documentation map and suggested reading paths.

---

## Package structure

Phoenix is organized by **robot concepts**, not by FTC SDK details.

### Packages students use day-to-day

Most robot code should only need imports from these packages:

* `edu.ftcphoenix.fw.input` — gamepad wrappers (`Gamepads`, `GamepadDevice`) that expose axes as `ScalarSource` and buttons as `BooleanSource`.
* `edu.ftcphoenix.fw.input.binding` — `Bindings`: map button edges to actions.
* `edu.ftcphoenix.fw.task` — `Task`, `TaskRunner`, `Tasks`: non-blocking macros over time.
* `edu.ftcphoenix.fw.actuation` — `Plant`, `Plants`, `PlantTasks`: mechanism/runtime abstractions you command with numeric targets.
* `edu.ftcphoenix.fw.drive` — `DriveSignal`, `DriveSource`, `DriveCommandSink`, `MecanumDrivebase` (FTC-independent drive logic).
* `edu.ftcphoenix.fw.ftc` — FTC entrypoints/adapters (e.g. `FtcDrives` for drivetrain wiring).
* `edu.ftcphoenix.fw.sensing` — sensor-facing wrappers (vision, odometry, etc.).
* `edu.ftcphoenix.fw.localization` — pose estimation (AprilTags, odometry, lightweight fusion, optional EKF-style fusion).
* `edu.ftcphoenix.fw.field` — field metadata (tag layouts, constants).

For the AprilTag-specific policy around fixed vs detectable tags, see [`AprilTag Localization & Fixed Layouts`](<../drive-vision/AprilTag Localization & Fixed Layouts.md>).

For odometry + AprilTag global localization specifically, the framework now makes three roles explicit:

* `AbsolutePoseEstimator` — a field-anchored pose source such as `AprilTagPoseEstimator` or `LimelightFieldPoseEstimator`
* `MotionPredictor` — a predictor that exposes both a current pose and a latest `MotionDelta`, such as `PinpointOdometryPredictor`
* `CorrectedPoseEstimator` — a global localizer that combines one predictor with one absolute correction source

Phoenix ships both a simpler gain-based corrected estimator and an optional covariance-aware EKF-style estimator. The advanced estimator is intentionally opt-in; the simpler corrected localizer remains the default starting point.

Within `drive/`, subpackages are intentionally parallel and predictable:

* `drive.source` — “where drive commands come from” (gamepad, autonomous logic).
* `drive.guidance` — driver-assist and closed-loop drive behavior helpers (auto-aim, go-to-point, pose lock, task wrappers).

### Packages that are intentionally “behind the scenes”

These exist so the student-facing packages stay small and consistent:

* `edu.ftcphoenix.fw.core.*` — shared plumbing: time, math, geometry, control, debug, and the HAL.
* `edu.ftcphoenix.fw.ftc.*` — the **FTC SDK boundary** (hardware adapters, frame conversions, FTC vision plumbing). Most teams only touch a couple entrypoints like `FtcDrives`.
* `edu.ftcphoenix.fw.tools.*` — testers and examples you can copy.

### Tester menus and calibration flows

The tester framework is now organized around two complementary ideas:

- **category suites**, where each tester has one natural home
- **guided walkthroughs**, where a robot can present a recommended bring-up order for students

If you are trying to bring up a fresh robot, start with [`Robot Calibration Tutorials`](<../testing-calibration/Robot Calibration Tutorials.md>) and [`Guided Calibration Walkthroughs`](<../testing-calibration/Guided Calibration Walkthroughs.md>). Those docs explain both the student-facing calibration flow and the framework helpers used to build robot-specific walkthrough menus.

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

AprilTags add two separate complications: FTC exposes *multiple pose frames*, and the detector's
SDK library is not the same thing as Phoenix's fixed-field layout.

* **Detector library** (`AprilTagLibrary`)
  * Tells the processor which tags exist and how big they are.
  * May include tags that are useful to detect but are not safe localization landmarks.
* **Field-fixed layout** (`TagLayout`)
  * Tells Phoenix which tag IDs may be trusted as fixed field landmarks.
  * Tag poses usually come from FTC metadata (`fieldPosition` + `fieldOrientation`), but the
    framework curates the fixed subset explicitly.
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
* `fw.ftc.FtcGameTagLayout.currentGameFieldFixed()` builds the framework-owned official fixed-tag
  layout for the current FTC season.

For the full library-vs-layout explanation, see
[`AprilTag Localization & Fixed Layouts`](<../drive-vision/AprilTag Localization & Fixed Layouts.md>).

Tester naming conventions (telemetry menus):

* Prefix calibration routines with `Calib:`
* Prefix localization/pose display tools with `Loc:`
* Prefix hardware sanity checks with `HW:`
* If a tester is robot-specific, make that obvious in the label (for example, append `(Robot)` or include the robot name)

Two rules of thumb:

1. If a class references FTC SDK types (`com.qualcomm.*`), it belongs in `fw.ftc` or `fw.tools`, not in the core building blocks.
2. The package tree is kept **parallel** on purpose (for example, `sensing.vision` ↔ `localization.apriltag`, and later predictor-side FTC localizers ↔ corrected/absolute localization packages). That makes it easy to predict where new features should go.

---

## Framework lanes vs robot controls

Phoenix now distinguishes between three different kinds of ownership that often got blurred together in older FTC code:

- **primitives**: small reusable building blocks like `GamepadDriveSource`, `MecanumDrivebase`, `AprilTagSensor`, and `PinpointOdometryPredictor`
- **framework lanes**: stable reusable owners built from primitives, such as `FtcMecanumDriveLane`, the backend-neutral `AprilTagVisionLane` seam (commonly implemented by `FtcWebcamAprilTagVisionLane` or `FtcLimelightAprilTagVisionLane`), and `FtcOdometryAprilTagLocalizationLane`
- **field facts**: shared environment data such as `TagLayout` and `FtcGameTagLayout.currentGameFieldFixed()`
- **robot-owned capability families**: the shared mode-neutral API used by both TeleOp and Auto
- **robot-owned controls/policy**: button semantics, scoring logic, auto-aim rules, and telemetry presentation

That split is deliberate. A framework primitive should not decide that "right bumper means slow mode" any more than an estimator should decide which scoring target matters this year. Stable hardware/resource ownership belongs in reusable framework lanes. Operator semantics and game strategy belong in robot code.

See [`Framework Lanes & Robot Controls`](<../design/Framework Lanes & Robot Controls.md>) for the full role glossary and from-scratch robot build guide, [`Robot Capabilities & Mode Clients`](<../design/Robot Capabilities & Mode Clients.md>) for the shared TeleOp/Auto API pattern, then use [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) for the deeper mechanism/supervisor patterns.

## The layers (top → bottom)

Think of Phoenix as a few thin layers you stack:

1. **OpMode / Robot code (you)**

    * Owns the loop and decides what updates when.
2. **Input** (`fw.input`)

    * `Gamepads`, `GamepadDevice`, `ScalarSource`, `BooleanSource`.
    * `BooleanSource` supports edge detection (`risingEdge`/`fallingEdge`) and press-to-toggle state
      via `BooleanSource.toggled()` (useful when enabling drive overlays).
3. **Bindings** (`fw.input.binding`)

    * `Bindings` turns button edges into actions (often: enqueue a macro).
4. **Tasks / Macros** (`fw.task`, plus helpers in other packages)

    * `Task`, `TaskRunner`, `Tasks`, `PlantTasks`, `DriveTasks`.
5. **Drive behavior** (`fw.drive` + `fw.drive.source` + `fw.drive.guidance`)

    * `DriveSource` (a specialized `Source<DriveSignal>`) produces a `DriveSignal` each loop (stick drive, driver assist overlays, etc.).
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

* `Bindings.update(clock)`
* `TaskRunner.update(clock)`

This prevents bugs like “button press fired twice” or “tasks advanced twice” when helper code gets layered.

---

## Hardware and mechanisms: `FtcActuators`, `Plant`, and the HAL

### HAL outputs (lowest level)

Phoenix abstracts FTC hardware into small output interfaces (in `fw.core.hal`):

* `PowerOutput` — normalized power (typically `[-1, +1]`)
* `PositionOutput` — native position units (servo `0..1`, motor encoder ticks, etc.)
* `VelocityOutput` — native velocity units (e.g., ticks/sec)

These HAL outputs are intentionally **command-only**. Plant-level feedback lives on `Plant.getMeasurement()` and raw sensor reads live in `ScalarSource` / `FtcSensors`.

### FTC boundary: `FtcHardware`

`edu.ftcphoenix.fw.ftc.FtcHardware` provides factories like:

* `FtcHardware.motorPower(hw, name, direction)`
* `FtcHardware.motorVelocity(hw, name, direction)`
* `FtcHardware.motorPosition(hw, name, direction)`
* `FtcHardware.servoPosition(hw, name, direction)`
* `FtcHardware.crServoPower(hw, name, direction)`

These return command-only HAL outputs. Measurement lives separately as `ScalarSource` values (for example via `FtcSensors`) so internal encoders, external encoders, and derived mechanism units all share the same feedback abstraction.

### Beginner entrypoint: `FtcActuators`

Most teams should **not** call `FtcHardware` directly. Use the staged builder in `FtcActuators`:

```java
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.actuation.Plant;

// Shooter: dual-motor velocity plant with a rate limit.
Plant shooter = FtcActuators.plant(hardwareMap)
        .motor("shooterLeftMotor", Direction.FORWARD)
        .andMotor("shooterRightMotor", Direction.REVERSE)
        .velocity()            // default velocityTolerance = 100 ticks/sec
        .rateLimit(500.0)      // max delta in native units per second
        .build();

// Transfer: CR servo power plant.
Plant transfer = FtcActuators.plant(hardwareMap)
        .crServo("transferServo", Direction.FORWARD)
        .power()
        .build();

// Pusher: positional servo plant (0..1).
Plant pusher = FtcActuators.plant(hardwareMap)
        .servo("pusherServo", Direction.FORWARD)
        .position()
        .build();
```

The builder stays domain-first (`power()`, `position()`, `velocity()`) and offers advanced
strategy overrides only when you need them, for example
`motor(...).position(MotorPositionControl.regulated(...))`.

**Important:** tasks can set targets on plants, but *your loop* must still call `plant.update(clock)` each cycle.

---

## Drive: `DriveSignal`, `DriveSource`, and `MecanumDrivebase`

### `DriveSignal` (robot-centric command)

A `DriveSignal` is **robot-centric** and follows Phoenix pose conventions:

* `axial > 0` → forward
* `lateral > 0` → left
* `omega > 0` → counter-clockwise (CCW)

### `DriveSource` (where commands come from)

A `DriveSource` is the drive-specific specialization of `Source<DriveSignal>` and produces a `DriveSignal` each loop:

* Manual TeleOp: `GamepadDriveSource`
* Assisted aiming / guidance: `DriveGuidance` (build a plan) + {@code DriveSource.overlayWhen(...) }
* Autonomous logic: anything implementing `DriveSource`

`DriveSource` also supports composition helpers (like scaling and blending) via default methods.

A common beginner-to-intermediate pattern is:

```java
ReferencePoint2d speakerAim = References.relativeToTagPoint(5, 0.0, 0.0);

DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .faceTo()
            .point(speakerAim)
            .doneFaceTo()
        .resolveWith()
            .aprilTagsOnly()
            .aprilTags(tagSensor, cameraMount, 0.25)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneResolveWith()
        .build();

DriveSource driveWithAim = baseDrive.overlayWhen(
        gamepads.p1().leftBumper(),
        aimPlan.overlay(),
        DriveOverlayMask.OMEGA_ONLY
);
```

That keeps normal stick translation while guidance owns omega.

When code only needs a place to *send* drive commands, Phoenix now uses the smaller `DriveCommandSink` seam. `MecanumDrivebase` implements that interface, but route adapters can implement it too.

When code needs to follow an external route object (Pedro `PathChain`, Road Runner trajectory, or your own route type), Phoenix now provides the matching generic seam: `RouteFollower<RouteT>` in `edu.ftcphoenix.fw.drive.route`. Wrap the external follower once, then sequence it with the rest of your robot using `RouteTask` / `RouteTasks.follow(...)`.

Even in a one-module repo, keep those adapters in a library-specific edge folder/package such as `fw/integrations/pedro/`, and keep robot-specific examples in a matching robot-side folder such as `autonomous/pedro/`. The folder split does not make the dependency optional by itself, but it keeps the boundary obvious and makes later source-set or module extraction mechanical.

```java
Task auto = Tasks.sequence(
        RouteTasks.follow(pedroAdapter, outboundPath, new RouteTask.Config()),
        Tasks.runOnce(scoringSupervisor::requestSingleShot),
        RouteTasks.follow(pedroAdapter, returnPath, new RouteTask.Config())
);
```

That keeps route-library ownership outside the framework while still letting Auto reuse the same Phoenix task vocabulary as everything else.

### `MecanumDrivebase` + `FtcDrives`

`FtcDrives.mecanum(hardwareMap)` is the beginner-friendly way to wire a mecanum drivetrain.

```java
import edu.ftcphoenix.fw.drive.*;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.input.Gamepads;

Gamepads pads = Gamepads.create(gamepad1, gamepad2);

MecanumDrivebase drivebase = FtcDrives.mecanum(hardwareMap);
DriveSource drive = new GamepadDriveSource(
        pads.p1().leftX(),
        pads.p1().leftY(),
        pads.p1().rightX(),
        GamepadDriveSource.Config.defaults()
).scaledWhen(pads.p1().rightBumper(), 0.35, 0.20);
```

**Rate limiting note:** `MecanumDrivebase` can rate-limit components using the most recent `dtSec`. Call `drivebase.update(clock)` once per loop. If you want rate limiting to use the *current* loop’s `dt`, call `update(clock)` **before** `drive(...)`.

### Spatial reasoning layers (math → queries/predicates → controllers)

Phoenix tries to keep “spatial logic” reusable by splitting it into four layers:

1. **Spatial math** (pure geometry): `Pose2d`, `SpatialMath2d`, region/shape primitives.
2. **Spatial predicates** (answering yes/no): `RobotZones2d`, `RobotHeadings2d` (often with
   hysteresis latches).
3. **Spatial queries** (solving target ↔ controlled-frame relationships): `SpatialQuery`,
   `SpatialSolveLane`, `SpatialControlFrames`.
4. **Controllers / planners** (turn solved relationships into commands): `DriveGuidanceSpec` +
   `DriveGuidancePlan` and their overlays/tasks/queries.

If you only want to ask “is the robot in the zone?” or “is the robot aimed?”, use layer (2). If you
want reusable relationship solves without drivetrain policy, use layer (3). If you want the framework
to *drive*, use layer (4).

Reference-first guidance is still the intended mental model: define the meaningful point or frame
once, then let solve lanes answer it from field pose, live AprilTags, or both. `DriveGuidance` now
consumes the shared `SpatialQuery` layer internally, so the same geometry and solve-lane setup can
later be reused by mechanism planners as well.

---

## Tasks and macros

### `Task` and `TaskRunner`

A `Task` is non-blocking work that progresses over multiple loop cycles.

A `TaskRunner` runs tasks **sequentially** (FIFO): start one task, update it each cycle until it completes, then move to the next.

If you want to abort automation, prefer `cancelAndClear()` over `clear()`. `clear()` forgets the current task without calling its cancellation hook; `cancelAndClear()` lets the task stop outputs cleanly and reports `TaskOutcome.CANCELLED`.

### Factories: `Tasks`, `PlantTasks`, `DriveTasks`

Phoenix gives you factories so your code reads like intent:

* `Tasks` — general composition (`sequence`, `parallelAll`, `waitForSeconds`, `waitUntil`, `runOnce`, …)
* `PlantTasks` — patterns that command a `Plant` (`setInstant`, `holdFor`, `moveTo`, …)
* `DriveTasks` — simple patterns that command a `DriveCommandSink` (`driveForSeconds`, `stop`, …)
* `DriveGuidanceTasks` — execute a `DriveGuidancePlan` as a Task (autonomous-style guidance)
* `RouteTasks` — follow an external route through a generic `RouteFollower<RouteT>` adapter
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

`Gamepads` wraps FTC `gamepad1` / `gamepad2` and exposes calibrated axes (`ScalarSource`) and
buttons (`BooleanSource`).

There is **no global update step** for gamepads. Sources are sampled when you call `get(...)`.
If you need edges/toggles, use `risingEdge()` / `fallingEdge()` / `toggled()` (or use `Bindings`).

### `Bindings`

`Bindings` lets you map button edges to actions.

Most commonly: **enqueue a macro** on press.

```java
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.TaskRunner;

Bindings bindings = new Bindings();
TaskRunner macros = new TaskRunner();

bindings.onRise(gamepads.p1().y(), () ->
        macros.enqueue(buildShootOneDiscMacro(shooter, transfer))
);
```

Call **once per loop**:

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

    // 2) Bindings (may enqueue macros)
    // Gamepad axes/buttons are Sources; they are sampled when you call get(...).
    bindings.update(clock);

    // 3) Tasks / macros
    macroRunner.update(clock);

    // 4) Drive
    DriveSignal cmd = driveSource.get(clock).clamped();
    drivebase.update(clock);   // call before drive(...) if you want current-dt rate limiting
    drivebase.drive(cmd);

    // 5) Mechanisms
    shooter.update(clock);
    transfer.update(clock);

    // 7) Telemetry
    telemetry.update();
}
```

---

## Where to go next

* [`Beginner’s Guide`](<Beginner's Guide.md>) — first setup + “how to write a Phoenix OpMode”.
* [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) — how TeleOp and Auto should share intents, status, supervisors, and lanes.
* [`Framework Principles`](<../../Framework Principles.md>) — the rules of thumb Phoenix expects you to follow.
* [`Loop Structure`](<../core-concepts/Loop Structure.md>) — deeper reasoning about update order and idempotency.
* [`Tasks & Macros Quickstart`](<../design/Tasks & Macros Quickstart.md>) — how to build task graphs quickly.
* [`Shooter Case Study & Examples Walkthrough`](<../examples/Shooter Case Study & Examples Walkthrough.md>) — maps concepts to real examples in `fw.tools.examples`.
