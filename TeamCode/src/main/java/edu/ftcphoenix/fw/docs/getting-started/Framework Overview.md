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
* `edu.ftcphoenix.fw.input.binding` — `Bindings`: map boolean signal events and levels to actions.
* `edu.ftcphoenix.fw.task` — `Task`, `TaskRunner`, `Tasks`: non-blocking macros over time.
* `edu.ftcphoenix.fw.actuation` — `Plant`, `Plants`, `PlantTasks`: mechanism/runtime abstractions you command with numeric targets.
* `edu.ftcphoenix.fw.drive` — `DriveSignal`, `DriveSource`, `DriveCommandSink`, `MecanumDrivebase` (FTC-independent drive logic).
* `edu.ftcphoenix.fw.ftc` — FTC entrypoints/adapters (e.g. `FtcDrives` for drivetrain wiring).
* `edu.ftcphoenix.fw.ftc.ui` — telemetry UI helpers such as `SelectionMenu`, `MenuNavigator`, and `HardwareNamePicker`.
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
* `edu.ftcphoenix.fw.ftc.ui` — FTC telemetry UI helpers for menus, breadcrumbs, hardware-name pickers, and pre-start selectors.
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
    * `BooleanSource` supports edge detection (`risingEdge`/`fallingEdge`) and rise-to-toggle state
      via `BooleanSource.toggled()` (useful when enabling drive overlays).
3. **Bindings** (`fw.input.binding`)

    * `Bindings` turns boolean signal edges, changes, levels, toggles, and nudges into actions.
4. **Tasks / Macros** (`fw.task`, plus helpers in other packages)

    * `Task`, `TaskRunner`, `Tasks`, `PlantTasks`, and the exclusive Auto/test helper in `DriveTasks`.
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
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(100.0)
        .targetGuards()
            .maxTargetRate(500.0)    // max delta in plant units per second
            .doneTargetGuards()
        .targetedByDefaultWritable(0.0)
        .build();

// Transfer: CR servo power plant.
Plant transfer = FtcActuators.plant(hardwareMap)
        .crServo("transferServo", Direction.FORWARD)
        .power()
        .targetedByDefaultWritable(0.0)
        .build();

// Pusher: positional servo plant (0..1).
Plant pusher = FtcActuators.plant(hardwareMap)
        .servo("pusherServo", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .nativeUnits()
        .targetedByDefaultWritable(0.0)
        .build();
```

The builder stays domain-first (`power()`, `position()`, `velocity()`). Position Plants then ask
small guided questions about control strategy, topology, bounds, unit mapping, and reference policy.
For example, a regulated motor position path uses `motor(...).position().regulated().nativeFeedback(...).regulator(...)`
instead of hiding those choices inside a large argument object.

A direct power Plant already knows its only legal domain: normalized `[-1.0, +1.0]`. It clamps a
finite out-of-range request before calling `PowerOutput`, while the FTC adapter keeps its own clamp
as final boundary defense.

**Important:** tasks write the Plant's registered `ScalarTarget`; *your loop* must still call `plant.update(clock)` each cycle so the Plant samples that source and applies hardware guards.

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
        .solveWith()
            .aprilTagsOnly()
            .aprilTags(tagSensor, cameraMount)
            .maxAgeSec(0.25)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneAprilTagsOnly()
        .build();

DriveSource driveWithAim = baseDrive.overlayWhen(
        gamepads.p1().leftBumper(),
        aimPlan.overlay(),
        DriveOverlayMask.OMEGA_ONLY
);
```

That keeps normal stick translation while guidance owns omega.

When code only needs a place to *send* drive commands, Phoenix now uses the smaller `DriveCommandSink` seam. `MecanumDrivebase` implements that interface, but route adapters can implement it too.

Ordinary TeleOp Tasks do decision/source/Plant-target work before one final drive phase samples the
composed `DriveSource` and writes its `DriveSignal` to that sink. Do not let a Task and the final
TeleOp drive phase compete as behavior-command writers.

For a simple open-loop Auto routine or drive tester,
`DriveTasks.driveExclusivelyForSeconds(...)` may own the sink for a fixed interval, but only when it
is the sole behavior-command writer. It refreshes the sink and writes the signal every active cycle,
then stops on completion or active cancellation. If an adapter's supported lifecycle requires
updates beyond active Tasks, its composition root continues calling `update(clock)` with the shared
`LoopClock`, and the adapter deduplicates the Task's same-cycle update.

When code needs to follow an external route object (Pedro `PathChain`, Road Runner trajectory, or
your own route type), Phoenix provides the matching generic seam: `RouteFollower<RouteT>` in
`edu.ftcphoenix.fw.drive.route`. Wrap the external follower once, then sequence it with the rest of
your robot using `RouteTask` / `RouteTasks.follow(...)`.

`RouteFollower.follow(route)` returns a `RouteExecution` for that one run. Its `status()` and
active-only, idempotent `cancel()` remain bound to that execution, so an old Task cannot mistake a
replacement route for its own completion or cancel the replacement. `RouteTask` retains that exact
handle and exposes its backend-neutral result through `getRouteStatus()`.

`RouteStatus` distinguishes normal completion from follower timeout/stall, interruption,
replacement, Task timeout, active cancellation, failure, and an unknown terminal transition.
`COMPLETED` maps to Task success; follower/Task timeouts map to `TaskOutcome.TIMEOUT`; other abnormal
terminal states use the existing fail-closed `TaskOutcome.CANCELLED` compatibility bucket. Use
`getRouteStatus()` when robot policy needs the precise reason instead of reconstructing it from a
vendor busy flag.

Fixed routes use the ordinary eager `RouteTasks.follow(...)` factory. If a return or fallback route
must use live pose, vision, or another current fact, use the explicitly named
`RouteTasks.followBuiltAtStart(...)` factory instead:

```java
RouteTask<PathChain> returnToStart = RouteTasks.followBuiltAtStart(
        "returnToStart",
        pedro.driveAdapter(),
        () -> robotPaths.buildReturnFromCurrentPose(returnPose),
        routeConfig
);
```

The supplier runs exactly once at that Task's `start(clock)` boundary. Keep it quick and
non-blocking, and let robot-owned path code read current snapshots and build the vendor route. Core
Task code and the follower adapter still receive no sensor, vision, alliance, or strategy types.

A stateful external follower may need a heartbeat even after its route Task completes. Pedro, for
example, keeps hold-end control, pose updates, callbacks, and manual drive inside
`Follower.update()`. Its adapter therefore has one stable Auto composition-root owner and
deduplicates any same-cycle update made by `RouteTask` or `DriveGuidanceTask`:

```java
PedroPathingRuntime pedro = Constants.createPhoenixAutoRuntime(hardwareMap, profile);
robot.initAuto(pedro.driveAdapter(), pedro.motionPredictor());
```

The runtime owns one configured Pinpoint predictor and gives Pedro a passive localizer view of that
same predictor. Phoenix therefore owns the localization/correction update first, Pedro sees that
current-cycle pose in its downstream heartbeat, and no second Pinpoint object resets or polls the
device. Apply the route's Pedro start pose through `pedro.setStartingPose(...)` before the first
heartbeat.

Tasks still select routes and guidance; they do not become the only Pedro lifecycle owner. The
adapter also uses Pedro's immediate typed break operation for cancellation/shutdown instead of
merely storing a zero vector for a future update.

Do not call raw Pedro Follower lifecycle methods from robot routines. Starting, breaking,
replacing, manually taking over, or updating a route outside the adapter bypasses its per-execution
status and is unsupported. Route construction and read-only Follower inspection remain valid at the
Pedro boundary.

Even in a one-module repo, keep those adapters and runtime bridges in a library-specific edge
folder/package such as `fw/integrations/pedro/`, and keep robot-specific examples in a matching
robot-side folder such as `autonomous/pedro/`. The folder split does not make the dependency
optional by itself, but it keeps the boundary obvious and makes later source-set or module
extraction mechanical.

The following snippet keeps a fixed outbound route eager and builds its return from live state when
that phase starts. It shows the composition shape only; a production scoring routine must use
robot-owned policy to gate position-dependent work after each route result.

```java
Task auto = Tasks.sequence(
        RouteTasks.follow(pedro.driveAdapter(), outboundPath, new RouteTask.Config()),
        Tasks.runOnce(scoringSupervisor::requestSingleShot),
        RouteTasks.followBuiltAtStart(
                "return",
                pedro.driveAdapter(),
                () -> robotPaths.buildReturnFromCurrentPose(returnPose),
                new RouteTask.Config())
);
```

That keeps route-library ownership outside the framework while still letting Auto reuse the same Phoenix task vocabulary as everything else.

Timed open-loop drive is not a substitute for normal Pedro route movement. Use `RouteTasks` or the
guidance helpers so the adapter can preserve its supported route lifecycle and truthful terminal
status.

The truthful status does not choose robot strategy. Generic Task composition keeps its existing
semantics; deciding whether a non-completed route should continue, run a fallback, or abort belongs
in the robot's Auto routine. Do not assume a generic sequence automatically stops after an abnormal
route ending.

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

Each Task object is single-use. A repeated driver request should call the macro/builder again (or a
`Supplier<Task>` / `OutputTaskFactory`) to create a fresh graph. The runner rejects one object being
current or queued twice, and framework Tasks throw a clear error if an already-started object reaches
`start(...)` again after completion or through another runner.

The runner may start and update a new task in the same loop. Framework timed tasks anchor their
durations and timeouts to that start timestamp, so the preceding loop's `dtSec()` cannot erase a
short wait or command. Every positive-duration output or Plant request remains available to its
later realization phase for at least one loop. The exclusive timed-drive helper instead publishes
directly when its Task starts and on each active cycle, so a positive interval is observable without
a competing downstream drive writer. Zero-duration intervals may finish immediately.

Use `cancelAndClear()` to abort automation. It cooperatively cancels the active Task, if any, and
always discards every queued Task; queued Tasks receive no pre-start cancellation callback. There
is no abrupt queue-forgetting path that can silently abandon active work. Framework Task
cancellation is active-only: it is a no-op before start, terminal while active, and a no-op after
completion or when repeated. Calling `update(...)` before `start(...)` is an actionable lifecycle
error; `Tasks.noop()` is the intentional exception because it is already successfully complete when
created.

If task start, update, completion checking, or cancellation throws a `RuntimeException`, the runner
fails closed: it best-effort cancels active or partially started work, clears its queue and state,
and rethrows the original failure with any cleanup failure suppressed.

### Factories: `Tasks`, `PlantTasks`, `DriveTasks`

Phoenix gives you factories so your code reads like intent:

* `Tasks` — general composition (`sequence`, `parallelAll`, `waitForSeconds`, `waitUntil`, `runOnce`, …)
* `PlantTasks` — guided patterns that write a Plant's registered target (`write` and `move`)
* `DriveTasks` — `driveExclusivelyForSeconds(...)` for simple timed open-loop Auto/test movement when
  its Task is the sole behavior-command writer for the `DriveCommandSink`
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
            PlantTasks.move(shooter)
                    .to(3200.0)
                    .cancelTo(0.0)
                    .timeout(1.0)
                    .build(),
            PlantTasks.write(transfer)
                    .to(1.0)
                    .forSeconds(0.20)
                    .then(0.0)
                    .build()
    );
}
```

A feedback move must choose `.cancelTo(value)` or `.leaveTargetOnCancel()` immediately after
`.to(...)`. `cancelTo(...)` changes the registered request in Plant units; it does not bypass the
Plant's overlays, bounds, references, or guards and therefore is not a guaranteed hardware stop.
Robot-owned coordinated cleanup must still cancel related behavior and reset every related target.
For timed writes, `.then(value)` runs on active cancellation as well as normal completion; omitting
it or selecting `.leaveThere()` leaves the current request in place.

---

## Inputs and bindings

### `Gamepads`

`Gamepads` wraps FTC `gamepad1` / `gamepad2` and exposes calibrated axes (`ScalarSource`) and
buttons (`BooleanSource`).

There is **no global update step** for gamepads. Sources are sampled when you call `get(...)`.
If you need edges/toggles, use `risingEdge()` / `fallingEdge()` / `toggled()` (or use `Bindings`).

### `Bindings`

`Bindings` is still boolean-first, but it now has one small continuous-copy helper as well. The core names are signal-based, not button-specific:

* `onRise(...)` and `onFall(...)` run once on edges.
* `mirrorOnChange(...)` mirrors the current high/low value to a setter, including the first sample.
* `whileHigh(...)` and `whileLow(...)` run every loop while a level is active.
* `toggleOnRise(...)` and `nudgeOnRise(...)` combine an edge trigger with a stateful helper.
* `copyEachCycle(...)` forwards a scalar value every loop. This is the standard pairing with frame-valued capability commands.

Most commonly: **enqueue a macro** on rise.

```java
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.TaskRunner;

Bindings bindings = new Bindings();
TaskRunner macros = new TaskRunner();

bindings.onRise(gamepads.p1().y(), () ->
        macros.enqueue(buildShootOneDiscMacro(shooter, transfer))
);
```

For a continuous non-drive mechanism command, bind the scalar each loop instead of writing directly to a plant:

```java
bindings.copyEachCycle(
        gamepads.p2().leftY().deadbandNormalized(0.08, -1.0, 1.0),
        lift::commandManualPower
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

This is the normal TeleOp pattern: Tasks finish their decisions first, then the one final
`DriveSource` writer commands the drivebase. Do not add an exclusive `DriveTasks` interval to this
loop unless the ordinary final drive write is disabled for that interval.

---

## Where to go next

* [`Beginner’s Guide`](<Beginner's Guide.md>) — first setup + “how to write a Phoenix OpMode”.
* [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) — how TeleOp and Auto should share intents, status, supervisors, and lanes.
* [`Framework Principles`](<../../Framework Principles.md>) — the rules of thumb Phoenix expects you to follow.
* [`Loop Structure`](<../core-concepts/Loop Structure.md>) — deeper reasoning about update order and idempotency.
* [`Tasks & Macros Quickstart`](<../design/Tasks & Macros Quickstart.md>) — how to build task graphs quickly.
* [`Examples Progression & Layered Mechanisms`](<../examples/Examples Progression & Layered Mechanisms.md>) — shows how the full `TeleOp_01` → `TeleOp_09` sequence fits together.
* [`Layered Shooter Example`](<../examples/Layered Shooter Example.md>) — walks through the first explicit `Requests → Behavior → Realization` mechanism example (`TeleOp_09`).
* [`Shooter Case Study & Examples Walkthrough`](<../examples/Shooter Case Study & Examples Walkthrough.md>) — dives into the shooter + vision examples (`TeleOp_05` and `TeleOp_06`).
