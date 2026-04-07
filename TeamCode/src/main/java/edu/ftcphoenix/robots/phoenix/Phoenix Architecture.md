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
  ScoringTargeting targeting
  Shooter shooter
  ShooterSupervisor scoring
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
- `Shooter`: mechanism subsystem and single writer to scoring-path plants
- `ShooterSupervisor`: policy/orchestration for scoring modes and requests
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
  ├─ shooter     -> Shooter config
  └─ autoAim     -> ScoringTargeting / scoring policy config

PhoenixRobot
  ├─ drive lane
  ├─ vision lane
  ├─ localization lane
  ├─ controls owner
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

## TeleOp controls live in one place now

Phoenix no longer spreads drive stick setup in one class and scoring bindings in another.

`PhoenixTeleOpControls` owns all of these:

- manual drive source creation
- slow mode
- auto-aim enable source
- aim override source
- intake binding
- flywheel binding
- shoot/eject hold semantics
- selected-velocity up/down bindings

That makes it obvious where driver policy lives.

## Recommended profile shape

```text
PhoenixProfile
  drive         -> FtcMecanumDriveLane.Config
  vision        -> FtcAprilTagVisionLane.Config
  localization  -> FtcOdometryAprilTagLocalizationLane.Config
  field         -> fixed AprilTag layout
  controls      -> TeleOp control tuning
  shooter       -> mechanism config
  autoAim       -> scoring target catalog + shot model + aim tuning
  calibration   -> human acknowledgements
```

This profile shape is the template future robots should copy.

## Loop order

Phoenix keeps loop order explicit inside `PhoenixRobot.updateTeleOp()`:

```text
1. localization.update(clock)
2. targeting.update(clock)
3. controls.update(clock)
4. shooterSupervisor.update(clock)
5. drive.update(clock)
6. drive.drive(...)
7. shooter.update(clock)
8. telemetryPresenter.emitTeleOp(...snapshots...)
```

That order reflects ownership:

- lanes produce stable shared state first
- robot services and controls consume that state
- supervisors translate intent into requests
- subsystems write hardware
- presenters explain the result

## Anti-patterns this architecture avoids

### Putting button policy into framework primitives

Phoenix intentionally avoids helpers that bake in a specific gamepad button at the primitive layer.

### Treating localization as the owner of every camera concern

The camera rig belongs to the vision lane because other robot systems may use it too.

### Letting the robot container become a control script

`PhoenixRobot` is allowed to wire objects together and choose loop order. It should not quietly absorb detailed scoring or targeting policy.

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
  MyTargetingService targeting
  MySubsystem subsystem
  MySupervisor supervisor
  MyTelemetryPresenter telemetry
```

Then read the framework doc [`Framework Lanes & Robot Controls`](<../fw/docs/design/Framework Lanes & Robot Controls.md>) for the full glossary and from-scratch build steps.
