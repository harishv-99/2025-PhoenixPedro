# Phoenix Architecture

This document explains Phoenix's current object graph and how it maps onto the framework's
architecture vocabulary.

The governing rule is:

> Stable hardware/resource ownership lives in framework lanes. Shared public robot vocabulary lives
> in robot-owned capability families. Operator semantics live in TeleOp controls. Game-specific
> reasoning lives in services, supervisors, and subsystems.

That split keeps Phoenix reusable as a template without pushing one season's strategy into the
framework.

Useful framework references:

- [`Framework Lanes & Robot Controls`](<../fw/docs/design/Framework Lanes & Robot Controls.md>)
- [`Robot Capabilities & Mode Clients`](<../fw/docs/design/Robot Capabilities & Mode Clients.md>)
- [`Recommended Robot Design`](<../fw/docs/design/Recommended Robot Design.md>)

## Big-picture structure

```text
PhoenixRobot
  PhoenixProfile

  PhoenixCapabilities capabilities

  FtcMecanumDriveLane drive
  AprilTagVisionLane vision   (selected by PhoenixVisionFactory; checked-in impl = FtcWebcamAprilTagVisionLane)
  FtcOdometryAprilTagLocalizationLane localization

  PhoenixTeleOpControls controls
  PhoenixDriveAssistService driveAssists
  ScoringTargeting targeting
  Shooter shooter
  ShooterSupervisor scoring
  TaskRunner autoRunner
  PhoenixTelemetryPresenter telemetry
```

Phoenix keeps backend choice in a small robot-owned wrapper: `PhoenixProfile.VisionConfig`
plus `PhoenixVisionFactory`. That is why the rest of the graph can depend on
`AprilTagVisionLane` instead of a webcam-specific owner while Stage 1 still keeps the
checked-in implementation on `FtcWebcamAprilTagVisionLane`.

The shared mode clients are:

```text
TeleOp client:
  PhoenixTeleOpControls -> PhoenixCapabilities -> Phoenix internals

Auto client:
  autonomous OpMode / PhoenixPedroAutoPlan -> PhoenixCapabilities -> Phoenix internals
```

That is the intended parallelism. TeleOp and Auto are parallel **clients** of Phoenix, not
different APIs layered directly onto internals.

## Role map

### Framework primitives used underneath the lanes

- `GamepadDriveSource`
- `MecanumDrivebase`
- `AprilTagSensor`
- `PinpointPoseEstimator`
- `FixedTagFieldPoseSolver`

### Framework lanes

- `FtcMecanumDriveLane`: owns mecanum hardware wiring, brake behavior, drivebase construction, and drive lifecycle
- `AprilTagVisionLane` / `FtcWebcamAprilTagVisionLane`: Phoenix consumes the backend-neutral seam while the checked-in FTC-boundary implementation remains webcam-backed today
- `FtcOdometryAprilTagLocalizationLane`: owns odometry + AprilTag localization strategy, fused estimator selection, and pose production

### Shared field facts

- `PhoenixProfile.field.fixedAprilTagLayout`

This layout is consumed by localization and targeting. It is intentionally not hidden inside the
vision lane or the localization lane.

### Robot-owned capability families

- `PhoenixCapabilities.scoring()`
- `PhoenixCapabilities.targeting()`

These are Phoenix's shared mode-neutral public API families. TeleOp controls and Auto plans should
use them instead of touching `Shooter`, `ShooterSupervisor`, or `ScoringTargeting` directly.

### Robot-owned objects

- `PhoenixTeleOpControls`: all TeleOp input semantics, including stick mapping and scoring button semantics
- `PhoenixDriveAssistService`: robot-specific drive assists that combine manual drive, scoring state, localization, and overlays
- `ScoringTargeting`: selected-tag policy, auto-aim guidance, cached targeting status, and shot suggestions
- `Shooter`: mechanism subsystem and single writer to scoring-path plants
- `ShooterSupervisor`: policy/orchestration for scoring modes and requests
- `TaskRunner autoRunner`: autonomous task queue used when Phoenix is running Auto
- `PhoenixTelemetryPresenter`: driver-facing presentation from snapshots
- `PhoenixRobot`: composition root and loop owner

## Why Phoenix uses capability families

Phoenix now has an explicit `PhoenixCapabilities` layer because TeleOp and Auto both need the same
robot vocabulary, but they should not depend on raw internals.

Phoenix currently splits that vocabulary into two families because that is the clearest public split
for Decode.

### `scoring()`

Owns public actions that change scoring-path state:

- intake enable
- flywheel enable
- continuous shooting enable
- eject enable
- single-shot requests
- selected-velocity adjustments
- scoring status

### `targeting()`

Owns public targeting/aiming behavior and read-side status:

- selected target / aim status snapshots
- autonomous aim task creation

### Why this split makes sense for Decode

`scoring()` is command-heavy and mutates mechanism state.

`targeting()` is mostly about selection, aim readiness, and aim execution. Auto and telemetry often
need to read targeting status without also caring about intake/eject/flywheel requests.

That makes the split cohesive without forcing callers to depend on one giant flat robot API.

### What future robots should copy

Copy the **pattern**, not these exact family names.

Another season may use families like:

- `gamePiece()`
- `endgame()`
- `awareness()`
- `drive()`

The rule is to split by cohesive public vocabulary, not by this year's internal class names.

## Dependency graph

```text
PhoenixProfile
  ├─ drive        -> FtcMecanumDriveLane.Config
  ├─ vision       -> PhoenixProfile.VisionConfig
  ├─ localization -> FtcOdometryAprilTagLocalizationLane.Config
  ├─ field        -> TagLayout
  ├─ controls     -> PhoenixTeleOpControls tuning
  ├─ driveAssist  -> PhoenixDriveAssistService tuning
  ├─ shooter      -> Shooter config
  └─ autoAim      -> ScoringTargeting / scoring policy config

PhoenixRobot
  ├─ drive lane
  ├─ vision lane
  ├─ localization lane
  ├─ capabilities façade
  ├─ controls owner
  ├─ drive-assist service
  ├─ targeting service
  ├─ shooter subsystem
  ├─ scoring supervisor
  └─ telemetry presenter
```

## Why the vision/localization split matters

Older FTC code often lumps camera ownership and localization together. Phoenix avoids that because
the camera rig is not only for localization.

### `AprilTagVisionLane` / `FtcWebcamAprilTagVisionLane` own

- backend-specific device identity (webcam today, smart camera later)
- camera resolution / processor setup for the active backend
- camera mount
- AprilTag sensor lifetime
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

## TeleOp controls live in one place

Phoenix no longer spreads drive stick setup in one class and scoring bindings in another.

`PhoenixTeleOpControls` owns all of these:

- manual drive source creation
- translation-magnitude semantics shared with higher-level services
- slow mode
- auto-aim enable source
- aim override source
- intake/flywheel/shoot/eject button semantics
- selected-velocity nudge semantics

The important boundary change is that TeleOp binds against `PhoenixCapabilities`, not against raw
`Shooter` or `ShooterSupervisor` objects.

`PhoenixDriveAssistService` then consumes those control-layer sources without taking ownership of the
button layout itself. That keeps input semantics and drive-assist policy separate while still
letting them collaborate cleanly.

## Autonomous structure

Phoenix Auto reuses the same targeting, shooter, and telemetry stack as TeleOp, but leaves route
ownership outside the robot container.

`PhoenixRobot.initAuto()` now does three things:

1. build the shared runtime used by Auto
2. create `PhoenixCapabilities`
3. create the shared `TaskRunner autoRunner`

It intentionally does **not**:

- create a drivetrain lane for a specific route library
- choose a specific autonomous routine
- expose a pile of robot-container task helpers for one season's strategy

The checked-in Pedro example now shows the intended pattern:

- `PhoenixPedroAutoTestOpMode` owns Pedro follower construction and path creation
- `PhoenixPedroAutoPlan` is the auto-side mode client that composes tasks over `PhoenixCapabilities`
- `PhoenixRobot` owns the shared runtime and task runner
- `PedroPathingDriveAdapter` remains the framework bridge between Pedro and Phoenix seams

That split matches the framework principles: reusable bridges stay in the framework, project-specific
route setup stays in robot code, and both modes share the same capability vocabulary.

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
- controls update operator intent
- supervisors translate intent into requests
- services reshape drive behavior
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

Auto uses the same scoring and targeting services, but swaps TeleOp drive-assist policy for the
queued autonomous task runner.

## Recommended profile shape

```text
PhoenixProfile
  drive         -> FtcMecanumDriveLane.Config
  vision        -> PhoenixProfile.VisionConfig
  localization  -> FtcOdometryAprilTagLocalizationLane.Config
  field         -> fixed AprilTag layout
  controls      -> TeleOp control tuning
  driveAssist   -> shoot-brace / drive-assist tuning
  shooter       -> mechanism config
  autoAim       -> scoring target catalog + shot model + aim tuning
  calibration   -> human acknowledgements
```

This profile shape is the template future robots should copy.

## Anti-patterns this architecture avoids

### Letting the robot container become a control script

`PhoenixRobot` is allowed to wire objects together, expose `PhoenixCapabilities`, and choose loop
order. It should not quietly absorb detailed scoring, targeting, or route-specific strategy.

### Letting TeleOp or Auto touch raw internals directly

`PhoenixTeleOpControls` and auto plans should use `PhoenixCapabilities`, not `Shooter`,
`ShooterSupervisor`, or `ScoringTargeting`.

### Treating capability families like framework lanes

The capability pattern is expected every year, but the family names and splits belong to the robot,
not to the framework.

## If you are building a future robot

Copy the ownership pattern, not Phoenix's season-specific behavior.

A good starting point is:

```text
MyRobot
  MyRobotProfile

  framework lanes
  shared field facts

  MyCapabilities
  MyTeleOpControls
  MyAutoPlan / MyAutoRoutine

  MyDriveAssistService
  MyTargetingService
  MySubsystem
  MySupervisor
  MyTelemetryPresenter
```

Then read the framework docs for the full split philosophy and role glossary.
