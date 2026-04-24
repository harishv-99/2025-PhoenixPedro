# Phoenix Architecture

This document explains Phoenix's current object graph and how it maps onto the framework's
architecture vocabulary.

The governing rule is:

> Stable hardware/resource ownership lives in framework lanes. Shared public robot vocabulary lives
> in robot-owned capability families. Operator semantics live in TeleOp controls. Game-specific
> reasoning lives in cohesive robot services and subsystems.

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
  AprilTagVisionLane vision   (selected by PhoenixVisionFactory from the active profile backend)
  FtcOdometryAprilTagLocalizationLane localization

  PhoenixTeleOpControls controls
  PhoenixDriveAssistService driveAssists
  ScoringTargeting targeting
  ScoringPath scoring
  TaskRunner autoRunner
  PhoenixTelemetryPresenter telemetry
```

Phoenix keeps backend choice in a small robot-owned wrapper: `PhoenixProfile.VisionConfig`
plus `PhoenixVisionFactory`. That is why the rest of the graph can depend on
`AprilTagVisionLane` instead of a backend-specific owner even though the FTC-boundary
implementation may be a webcam lane or a Limelight lane.

The shared mode clients are:

```text
TeleOp client:
  PhoenixTeleOpControls -> PhoenixCapabilities -> Phoenix internals

Auto client:
  PhoenixAutoSpec / PhoenixPedroAutoRoutineFactory -> PhoenixCapabilities -> Phoenix internals
```

That is the intended parallelism. TeleOp and Auto are parallel **clients** of Phoenix, not
different APIs layered directly onto internals.

`PhoenixCapabilities` is now only a tiny aggregate of robot-owned capability families. It no longer has a separate delegating implementation class; `ScoringPath` implements the scoring family directly and `ScoringTargeting` implements the targeting family directly.

## Role map

### Framework primitives used underneath the lanes

- `GamepadDriveSource`
- `MecanumDrivebase`
- `AprilTagSensor`
- `PinpointOdometryPredictor`
- `FixedTagFieldPoseSolver`

### Framework lanes

- `FtcMecanumDriveLane`: owns mecanum hardware wiring, brake behavior, drivebase construction, and drive lifecycle
- `AprilTagVisionLane` plus a concrete FTC-boundary implementation such as `FtcWebcamAprilTagVisionLane` or `FtcLimelightAprilTagVisionLane`: Phoenix consumes the backend-neutral seam while `PhoenixVisionFactory` chooses the active backend
- `FtcOdometryAprilTagLocalizationLane`: owns predictor + AprilTag localization strategy, correction-source selection, corrected-estimator selection, and pose production

### Shared field facts

- `PhoenixProfile.field.fixedAprilTagLayout`

This layout is consumed by localization and targeting. It is intentionally not hidden inside the
vision lane or the localization lane.

### Robot-owned capability families

- `PhoenixCapabilities.scoring()`
- `PhoenixCapabilities.targeting()`

These are Phoenix's shared mode-neutral public API families. TeleOp controls and Auto plans should
use them instead of touching `ScoringPath` or `ScoringTargeting` directly.

### Robot-owned objects

- `PhoenixTeleOpControls`: all TeleOp input semantics, including stick mapping and scoring bindings
- `PhoenixDriveAssistService`: robot-specific drive assists that combine manual drive, scoring state, localization, and overlays
- `ScoringTargeting`: selected-tag policy, auto-aim guidance, cached targeting status, and shot suggestions
- `ScoringPath`: scoring-path mechanism owner, single writer to scoring-path plants, internally layered as inputs → execution → realization
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
  ├─ scoring      -> ScoringPath config
  ├─ autoAim      -> ScoringTargeting / scoring policy config
  └─ auto         -> Auto route/aim/wait timing

PhoenixRobot
  ├─ drive lane
  ├─ vision lane
  ├─ localization lane
  ├─ capabilities aggregate
  ├─ controls owner
  ├─ drive-assist service
  ├─ targeting service
  ├─ scoring path
  └─ telemetry presenter
```

## Why the vision/localization split matters

Older FTC code often lumps camera ownership and localization together. Phoenix avoids that because
the camera rig is not only for localization.

### `AprilTagVisionLane` plus its concrete backend owner own

- backend-specific device identity (for example webcam or Limelight)
- backend-specific processor / pipeline setup
- camera mount
- AprilTag sensor lifetime
- vision-device cleanup

### `FtcOdometryAprilTagLocalizationLane` owns

- Pinpoint odometry
- raw AprilTag field solver
- optional direct Limelight field pose
- correction-source and corrected/global estimator selection
- localization update order
- corrected and predictor pose outputs

### `ScoringTargeting` consumes

- the shared AprilTag sensor from the vision lane
- the camera mount from the vision lane
- the corrected/global pose from the localization lane
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

The important boundary is that TeleOp binds against `PhoenixCapabilities`, not against raw
`ScoringPath` or `ScoringTargeting` objects.

Phoenix still keeps drivetrain control on the dedicated drive-source path because manual drive is a
continuous stream that benefits from source composition and overlays. Non-drive continuous mechanism
controls should generally go through capabilities as frame-valued commands and be registered with
`Bindings.copyEachCycle(...)` when Phoenix adds those commands.

`PhoenixDriveAssistService` then consumes those control-layer sources without taking ownership of the
button layout itself. That keeps input semantics and drive-assist policy separate while still
letting them collaborate cleanly.

## ScoringPath internal layering

`ScoringPath` now keeps its mutable state split into the same three roles Phoenix wants future
mechanisms to follow:

- **Inputs**: held caller-owned selections and pending request counters written by capability methods
- **Execution**: priority rules, queue admission, cancellation rules, and transient shot behavior
- **Realization**: plant ownership, final actuator targets, and flywheel readback used for readiness

That internal split matters more than file count. `ScoringPath` is still one public mechanism owner,
but the code now makes it much harder to accidentally mix caller-owned intent with robot-owned queue
state or plant readback.

## Autonomous structure

Phoenix Auto reuses the same targeting, scoring path, and telemetry stack as TeleOp, but leaves route
ownership outside the robot container. The robot-owned spec/strategy objects now make the selected
match setup explicit before any route or task sequence is built.

The core Auto types are:

```text
PhoenixAutoSpec
  alliance
  startPosition
  partnerPlan
  strategy

PhoenixAutoProfiles
  spec + base profile -> Auto-specific profile snapshot, using profile-owned red/blue Auto tag ids

PhoenixAutoTasks
  reusable scoring/targeting task snippets over PhoenixCapabilities

PhoenixPedroPathFactory
  spec -> Pedro PathChain set

PhoenixPedroAutoRoutineFactory
  spec.strategy + context -> Task sequence
```

`PhoenixRobot.initAuto()` still does three things only:

1. build the shared runtime used by Auto
2. create `PhoenixCapabilities`
3. create the shared `TaskRunner autoRunner`

It intentionally does **not**:

- choose alliance color
- choose a starting side
- choose a partner-coordination strategy
- build Pedro paths
- choose a specific autonomous routine
- expose route-script helpers from the robot container

Annotated Phoenix Driver Station entries live under:

```java
edu.ftcphoenix.robots.phoenix.opmode
```

Implementation code stays by role under packages such as:

```java
edu.ftcphoenix.robots.phoenix.autonomous
edu.ftcphoenix.robots.phoenix.autonomous.pedro
edu.ftcphoenix.robots.phoenix.tester
```

That means OpModes are easy to find from the Driver Station perspective, while path factories,
routine factories, tester implementations, and robot services remain grouped by responsibility.

The checked-in Pedro routes are still placeholder integration geometry. Real alliance/start/partner
paths should be added in `PhoenixPedroPathFactory`; high-level strategy decisions should stay in
`PhoenixPedroAutoRoutineFactory`; reusable aim/shoot snippets should stay in `PhoenixAutoTasks`.

## Driver Station setup UI

Phoenix should use the framework FTC UI helpers for future pre-start selectors rather than building
selection mechanics into `PhoenixRobot` or individual OpModes. The intended split is:

- `edu.ftcphoenix.fw.ftc.ui.SelectionMenu` for one visible list of choices
- `MenuNavigator` for nested setup flows, breadcrumbs, back/home behavior, and level display
- `SelectionMenus` for compact enum-backed setup screens
- `ConfirmationScreen` for final review before building a selected Auto routine
- `SummaryScreen` for locked summaries after a selected runtime has already been initialized
- `HardwareNamePicker` for tester hardware selection, with `X` as refresh so `B` can remain
  available for back/cancel in richer flows
- Phoenix robot code for the meaning of selected values, such as alliance, start position, partner
  plan, or autonomous strategy

`PhoenixPedroAutoSelectorOpMode` is the first robot-specific user of this split. It produces a
`PhoenixAutoSpec` during INIT, delegates to the same Pedro/Phoenix lifecycle glue used by static
Auto entries, and then replaces the wizard with a locked summary so the menu cannot drift away from
the queued routine. OpMode classes should still stay thin: they choose or collect the spec, construct
`PhoenixRobot`, and enqueue the selected routine. They should not become route scripts or hardware
selection screens.

`PhoenixPedroAutoOpModeBase` also treats INIT failures as retryable setup failures, not as
half-built robot states. Before each new build attempt it clears any previous error and stops any
partial Phoenix/Pedro runtime left by an earlier failure; after a successful build it preserves the
active spec, profile, paths, robot, and adapter as one consistent set.

Pedro debug telemetry is composed into the same frame as Phoenix Auto telemetry. The OpMode adds
`auto.spec`, `auto.paths`, and Pedro pose/busy rows before `PhoenixRobot.updateAuto()` calls the
Auto telemetry presenter, so the Driver Station sees one coherent Auto status page per loop.


## Loop order

Phoenix keeps loop order explicit inside `PhoenixRobot.updateTeleOp()`:

```text
1. localization.update(clock)
2. targeting.update(clock)
3. controls.update(clock)
4. scoringPath.update(clock)
5. driveAssists.update(clock, scoringStatus)
6. drive.update(clock)
7. drive.drive(...)
8. telemetryPresenter.emitTeleOp(...snapshots...)
```

That order reflects ownership:

- lanes produce stable shared state first
- controls update operator intent
- the scoring path translates intent and writes scoring hardware
- services reshape drive behavior
- presenters explain the result

Phoenix keeps `updateAuto()` just as explicit:

```text
1. localization.update(clock)
2. targeting.update(clock)
3. autoRunner.update(clock)
4. scoringPath.update(clock)
5. telemetryPresenter.emitAuto(...with Auto task and scoring snapshots...)
```

Auto uses the same scoring path and targeting service, but swaps TeleOp drive-assist policy for the
queued autonomous task runner and reports through the Auto-specific telemetry emitter.

## Recommended profile shape

```text
PhoenixProfile
  drive         -> FtcMecanumDriveLane.Config
  vision        -> PhoenixProfile.VisionConfig
  localization  -> FtcOdometryAprilTagLocalizationLane.Config
  field         -> fixed AprilTag layout
  controls      -> TeleOp control tuning
  driveAssist   -> shoot-brace / drive-assist tuning
  scoring       -> scoring-path mechanism config
  autoAim       -> scoring target catalog + shot table + aim tuning
  auto          -> Auto route/aim/wait timing + red/blue Auto scoring tag ids
  calibration   -> human acknowledgements
```

This profile shape is the template future robots should copy.

## Anti-patterns this architecture avoids

### Letting the robot container become a control script

`PhoenixRobot` is allowed to wire objects together, expose `PhoenixCapabilities`, and choose loop
order. It should not quietly absorb detailed scoring, targeting, or route-specific strategy.

### Letting TeleOp or Auto touch raw internals directly

`PhoenixTeleOpControls` and auto plans should use `PhoenixCapabilities`, not `ScoringPath`
or `ScoringTargeting`.

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
  MyScoringPath / MySubsystem
  MyTelemetryPresenter
```

Then read the framework docs for the full split philosophy and role glossary.

## Spatial guidance and scalar setpoint planning notes

Phoenix currently uses the framework's Drive Guidance path for drivetrain-facing behaviors such as scoring aim. The same framework layer now separates three concepts that robot code should keep distinct:

1. **`SpatialQuery`** solves field/robot geometry: target points, facing errors, translation errors, and alternate solve lanes such as live AprilTags vs global localization.
2. **`DriveGuidancePlan` / `DriveGuidanceQuery`** consume spatial results and produce drivetrain omega/translation commands.
3. **`ScalarSetpointPlanner`** consumes plant-unit scalar requests and produces feasible mechanism setpoints for `Plant`s.

Phoenix scoring aim follows the drive path because the drivetrain turns the whole robot. A future turret, tray, arm, or extension should generally follow the scalar path: robot-owned calibration/reference setup converts semantic intent or spatial geometry into the plant units exposed by the mechanism, then the scalar planner handles candidates, periodic equivalents, travel range, and readiness status.

For example, a future turret with its own camera would use one spatial query with two solve lanes: a direct turret-camera AprilTag lane and a global-pose fallback lane. The turret service would map the selected facing solution into the turret `PositionPlant`'s public units, and a `ScalarSetpointPlanner` would choose a reachable setpoint under cable limits before feeding the turret `Plant`.

Calibration remains robot-owned. Homing switches, encoder zero offsets, ticks-per-turn constants, and cable-limit ranges should be established by the mechanism service and exposed through `PositionPlant`/planner-facing plant-unit measurements and `ScalarRange`s.
