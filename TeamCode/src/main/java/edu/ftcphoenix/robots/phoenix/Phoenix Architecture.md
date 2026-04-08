# Phoenix Architecture

This document explains the current Phoenix object graph and how it maps onto the framework architecture vocabulary.

The governing rule is:

> Stable hardware and resource ownership lives in framework lanes. Shared field landmarks stay separate. Operator semantics live in robot controls. Game-specific behavior lives in robot services, supervisors, and subsystems.

That split is what keeps Phoenix reusable as a template without pushing one season's strategy into the framework.

## Big-picture structure

```text
PhoenixRobot
  PhoenixProfile

  FtcMecanumDriveLane drive
  FtcAprilTagVisionLane vision
  FtcOdometryAprilTagLocalizationLane localization

  PhoenixTeleOpControls controls
  PhoenixDriveAssistService driveAssists
  ScoringTargeting targeting
  Shooter shooter
  ShooterSupervisor scoring
  TaskRunner autoRunner
  PhoenixTelemetryPresenter telemetry
```

## Role map

### Framework primitives used underneath the lanes

- `GamepadDriveSource`
- `MecanumDrivebase`
- `AprilTagSensor`
- `PinpointPoseEstimator`
- `FixedTagFieldPoseSolver`

### Framework lanes

- `FtcMecanumDriveLane`: owns mecanum hardware wiring, brake behavior, drivebase construction, and drive lifecycle
- `FtcAprilTagVisionLane`: owns the AprilTag camera rig, webcam identity, camera mount, and portal cleanup
- `FtcOdometryAprilTagLocalizationLane`: owns odometry + AprilTag localization strategy, fused estimator selection, and pose production

### Shared field facts

- `PhoenixProfile.field.fixedAprilTagLayout`

This layout is consumed by localization and targeting. It is intentionally not hidden inside the vision lane or the localization lane.

### Robot-owned objects

- `PhoenixTeleOpControls`: all TeleOp input semantics, including stick mapping and scoring buttons
- `ScoringTargeting`: selected-tag policy, auto-aim guidance, cached targeting status, and shot suggestions
- `PhoenixDriveAssistService`: robot-specific drive assists that combine manual drive, scoring state, localization, and overlays
- `Shooter`: mechanism subsystem and single writer to scoring-path plants
- `ShooterSupervisor`: policy/orchestration for scoring modes and requests
- `TaskRunner autoRunner`: autonomous task queue used when Phoenix is running Auto
- `PhoenixTelemetryPresenter`: driver-facing presentation from snapshots
- `PhoenixRobot`: composition root and loop owner

## Dependency graph

```text
PhoenixProfile
  ├─ drive       -> FtcMecanumDriveLane.Config
  ├─ vision      -> FtcAprilTagVisionLane.Config
  ├─ localization -> FtcOdometryAprilTagLocalizationLane.Config
  ├─ field       -> TagLayout
  ├─ controls    -> PhoenixTeleOpControls tuning
  ├─ driveAssist -> PhoenixDriveAssistService tuning
  ├─ shooter     -> Shooter config
  └─ autoAim     -> ScoringTargeting / scoring policy config

PhoenixRobot
  ├─ drive lane
  ├─ vision lane
  ├─ localization lane
  ├─ controls owner
  ├─ drive-assist service
  ├─ targeting service
  ├─ shooter subsystem
  ├─ scoring supervisor
  └─ telemetry presenter
```

## Why the vision/localization split matters

Older FTC code often lumps camera ownership and localization together. Phoenix now avoids that because the camera rig is not only for localization.

### `FtcAprilTagVisionLane` owns

- webcam name
- camera resolution
- camera mount
- AprilTag sensor / portal lifetime
- camera cleanup

### `FtcOdometryAprilTagLocalizationLane` owns

- Pinpoint odometry
- AprilTag-only field solver
- fused/global estimator selection
- localization update order
- fused and odometry pose outputs

### `ScoringTargeting` consumes

- the shared AprilTag sensor from the vision lane
- the camera mount from the vision lane
- the fused pose from the localization lane
- the fixed field tag layout from `PhoenixProfile.field`

That dependency structure is deliberate and matches the role vocabulary in the framework docs.


## Why `shootBraceEnabled()` no longer lives in `PhoenixRobot`

Earlier revisions left a `shootBraceEnabled()` helper and the underlying hysteresis latch inside the
robot container. That was a boundary mistake.

`shootBrace` depends on:

- driver translation intent from the controls owner
- scoring state from the scoring supervisor
- localization-backed pose lock from the drive-guidance layer

That makes it a **robot-specific drive-assist policy**, not a composition-root concern.

Phoenix now assigns that responsibility to `PhoenixDriveAssistService`, which owns:

- the shoot-brace latch and thresholds
- the pose-lock overlay used while actively shooting
- the final TeleOp drive source built from manual drive + brace + auto aim
- the read-only drive-assist status snapshot used by telemetry

That split matters because the composition root should wire objects together, not quietly hold onto
small state machines that implement match behavior.

## TeleOp controls live in one place now

Phoenix no longer spreads drive stick setup in one class and scoring bindings in another.

`PhoenixTeleOpControls` owns all of these:

- manual drive source creation
- translation-magnitude semantics shared with higher-level services
- slow mode
- auto-aim enable source
- aim override source
- intake binding
- flywheel binding
- shoot/eject hold semantics
- selected-velocity up/down bindings

`PhoenixDriveAssistService` then consumes those control-layer sources without taking ownership of the button layout itself. That keeps input semantics and drive-assist policy separate while still letting them collaborate cleanly.

## Recommended profile shape

```text
PhoenixProfile
  drive         -> FtcMecanumDriveLane.Config
  vision        -> FtcAprilTagVisionLane.Config
  localization  -> FtcOdometryAprilTagLocalizationLane.Config
  field         -> fixed AprilTag layout
  controls      -> TeleOp control tuning
  driveAssist   -> shoot-brace / drive-assist tuning
  shooter       -> mechanism config
  autoAim       -> scoring target catalog + shot model + aim tuning
  calibration   -> human acknowledgements
```

This profile shape is the template future robots should copy.

## Autonomous structure

Phoenix Auto now reuses the same targeting, shooter, and telemetry stack as TeleOp, but leaves route ownership outside the robot container. `PhoenixRobot.initAuto()` intentionally does **not** create a drivetrain lane. Instead, Auto code can plug in an external follower through the framework seams:

- `DriveCommandSink` when Phoenix only needs to command normalized drive signals (for example, the final aim turn)
- `RouteFollower<RouteT>` / `RouteTask<RouteT>` when Phoenix wants to sequence an external route object alongside waits, shots, and other tasks

The checked-in Pedro example is `autonomous/pedro/PhoenixPedroAutoTestOpMode.java`. It uses:

- `fw/integrations/pedro/PedroPathingDriveAdapter.java` to wrap a Pedro `Follower` as both a `DriveCommandSink` and a `RouteFollower<PathChain>`
- `autonomous/pedro/PedroPathingFollowers.java` to construct the follower without importing a team-specific Pedro constants class into Phoenix core
- `RouteTasks.follow(...)` for outbound and return path segments
- a mid-path Pedro callback to spin up the flywheel and refresh the target-derived shot velocity
- a separate Phoenix aim task before requesting the shot

That split matches the framework principles: the route package stays project-specific, Pedro imports stay in clearly marked edge folders, and the robot behavior still reads like normal Phoenix tasks.

## Loop order

Phoenix keeps loop order explicit inside `PhoenixRobot.updateTeleOp()`:

```text
1. localization.update(clock)
2. targeting.update(clock)
3. controls.update(clock)
4. shooterSupervisor.update(clock)
5. driveAssists.update(clock, scoringStatus)
6. drive.update(clock)
7. drive.drive(...)
8. shooter.update(clock)
9. telemetryPresenter.emitTeleOp(...snapshots...)
```

That order reflects ownership:

- lanes produce stable shared state first
- robot services and controls consume that state
- supervisors translate intent into requests
- subsystems write hardware
- presenters explain the result

Phoenix keeps `updateAuto()` just as explicit:

```text
1. localization.update(clock)
2. targeting.update(clock)
3. autoRunner.update(clock)
4. shooterSupervisor.update(clock)
5. shooter.update(clock)
6. telemetryPresenter.emitTeleOp(...with Auto snapshots...)
```

Auto uses the same scoring and targeting services, but swaps TeleOp drive-assist policy for the queued autonomous task runner.

## Anti-patterns this architecture avoids

### Putting button policy into framework primitives

Phoenix intentionally avoids helpers that bake in a specific gamepad button at the primitive layer.

### Treating localization as the owner of every camera concern

The camera rig belongs to the vision lane because other robot systems may use it too.

### Letting the robot container become a control script

`PhoenixRobot` is allowed to wire objects together and choose loop order. It should not quietly absorb detailed scoring, targeting, or drive-assist policy. Small state machines like shoot-brace latches belong in a service or supervisor that clearly owns that behavior.

## If you are building a future robot

Copy the ownership pattern, not Phoenix's season-specific behavior.

A good starting point is:

```text
MyRobot
  MyRobotProfile

  FtcMecanumDriveLane drive
  FtcAprilTagVisionLane vision
  FtcOdometryAprilTagLocalizationLane localization

  MyTeleOpControls controls
  MyDriveAssistService driveAssists
  MyTargetingService targeting
  MySubsystem subsystem
  MySupervisor supervisor
  MyTelemetryPresenter telemetry
```

Then read the framework doc [`Framework Lanes & Robot Controls`](<../fw/docs/design/Framework Lanes & Robot Controls.md>) for the full glossary and from-scratch build steps.
