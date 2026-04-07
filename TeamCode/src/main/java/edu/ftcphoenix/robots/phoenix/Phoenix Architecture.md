# Phoenix Architecture

This file explains the current Phoenix object split and, more importantly, **why** it is structured this way.

The design follows one core rule:

> Stable hardware/resource ownership belongs in framework lanes. Driver/operator semantics belong in a robot-owned controls object. Game-specific behavior belongs in robot policy and subsystems.

That rule keeps the framework reusable year to year without pushing game strategy into the framework.

## Big-picture structure

```text
PhoenixRobot
  PhoenixProfile

  FtcMecanumDriveLane drive
  FtcLocalizationLane localization

  PhoenixTeleOpControls controls
  ScoringTargeting targeting
  Shooter shooter
  ShooterSupervisor scoring
  PhoenixTelemetryPresenter telemetry
```

## What each object owns

### `FtcMecanumDriveLane`

Owns stable drive-lane concerns:

- motor wiring
- zero-power brake behavior
- `MecanumDrivebase`
- drive lifecycle

Does **not** own:

- stick mapping
- slow-mode button choices
- aim assists
- driver policy

### `FtcLocalizationLane`

Owns stable localization-lane concerns:

- Pinpoint odometry
- AprilTag camera ownership
- AprilTag-only field solve
- fused/global estimator selection
- per-loop localization updates
- vision cleanup

Does **not** own:

- scoring-tag selection
- shot decisions
- aiming policy
- match strategy

### `PhoenixTeleOpControls`

Owns all TeleOp input semantics:

- base drive-stick mapping
- slow mode
- auto-aim enable / override sources
- intake / flywheel / shoot / eject bindings
- selected-velocity adjustment bindings

This is the key cleanup from the previous design. Drive controls are no longer configured in a different place from the rest of the bindings.

### `ScoringTargeting`

Owns targeting policy:

- scoring-tag selection
- auto-aim query
- cached targeting snapshot
- range-based shot suggestion

### `ShooterSupervisor`

Owns scoring policy:

- requests
- mode transitions
- gating
- feed decisions

### `Shooter`

Owns scoring-path actuation:

- plants
- flywheel control
- feed queue
- final output writes

### `PhoenixTelemetryPresenter`

Owns driver-facing telemetry formatting and consumes snapshots rather than reaching back into live objects.

## Recommended future-robot pattern

When starting a new robot, try to keep the same three-way split:

1. **Framework lanes**
   - drive hardware lane
   - localization lane
   - any other truly stable, reusable resource owner

2. **Robot-owned controls**
   - all driver/operator semantics in one place
   - build `DriveSource`s here
   - expose the enable/override sources higher-level code needs

3. **Robot-owned mechanisms and policy**
   - subsystems write plants
   - supervisors own policy
   - targeting/strategy objects own game-specific reasoning

## Recommended profile shape

Phoenix now uses this style of profile:

```text
PhoenixProfile
  drive         -> FtcMecanumDriveLane.Config
  localization  -> FtcLocalizationLane.Config
  controls      -> PhoenixTeleOpControls.Config-ish tuning
  shooter       -> Shooter.Config
  autoAim       -> ScoringTargeting.Config-ish tuning
  calibration   -> human acknowledgements
```

That shape is a good template for future robots too.

## Anti-patterns this refactor is trying to avoid

### 1. Framework primitives that smuggle button policy

Bad example in spirit:

```text
a low-level drive helper that bakes a button choice into a primitive
```

Why it is odd:

- a low-level input primitive suddenly chooses a specific button
- drive controls become split between multiple places
- future robots inherit hidden operator assumptions

### 2. Robot container as a giant control script

If `PhoenixRobot` owns all of these directly:

- stick mapping
- button edges
- target selection
- aim gating
- telemetry formatting
- resource cleanup

then it stops being a composition root and becomes the place where every behavior change lands.

### 3. Pushing game strategy into framework lanes

The framework should know how to produce pose and how to send drive output. It should not know which tag is a scoring target or when a shooter should feed.

## The practical rule of thumb

If a new object answers one of these questions, it probably belongs in the framework:

- "Which FTC resources does this stable lane own?"
- "How is this stable lane configured?"
- "How is this stable lane updated and shut down?"

If a new object answers one of these questions, it probably belongs in robot code:

- "Which buttons do the drivers use?"
- "What does auto aim mean for this game?"
- "When is it okay to score?"
- "How should this mechanism behave in this robot?"
