# Framework Lanes & Robot Controls

This document explains one of the most important architectural boundaries in Phoenix: **what belongs in the reusable framework, and what should stay in each robot's code**.

---

## The short version

Use three layers:

1. **Primitives** — small reusable building blocks
   - examples: `GamepadDriveSource`, `MecanumDrivebase`, `AprilTagSensor`, `PinpointPoseEstimator`
2. **Framework lanes** — stable reusable owners built from primitives
   - examples: `FtcMecanumDriveLane`, `FtcLocalizationLane`
3. **Robot-owned controls and policy** — button semantics, scoring logic, target selection, telemetry presentation

A primitive should do one focused job. A lane should own a stable multi-object resource graph. Robot code should decide what the controls mean and what the game strategy is.

---

## What belongs in a framework primitive

A primitive should be reusable even when the next robot changes:

- driver slots
- which sticks are used
- which button enables slow mode
- which buttons fire macros
- whether the robot is TeleOp, assisted TeleOp, or Auto

That is why `GamepadDriveSource` now accepts **explicit axes** instead of hiding button or gamepad choices inside factory helpers.

Good primitive responsibilities:

- map axes to a `DriveSignal`
- mix mecanum wheel powers
- estimate pose from one sensor/model
- wrap FTC hardware into framework interfaces

Bad primitive responsibilities:

- "P1 RB means slow mode"
- "P2 LB enables auto aim"
- "this year's scoring target is tag 24"

Those are robot policy decisions.

---

## What belongs in a framework lane

A framework lane is appropriate when a recurring FTC subsystem has:

- several collaborating objects
- stable hardware/resource ownership
- a clear update lifecycle
- a clear cleanup lifecycle
- little or no game-specific policy

### Example: `FtcLocalizationLane`

Localization is a good lane because most FTC robots need nearly the same ownership graph:

- odometry estimator
- AprilTag sensor
- field-fixed tag layout
- AprilTag-only field solver
- fused/global estimator
- per-loop update
- camera cleanup

Those concerns are stable across seasons. What changes year to year is how the robot **uses** pose, not how the camera and odometry are owned.

### Example: `FtcMecanumDriveLane`

Drive hardware is also a good lane because it bundles:

- wiring
- zero-power brake behavior
- drivetrain construction
- update/stop lifecycle

But it does **not** decide who is driving or how slow mode works.

---

## What should stay robot-owned

Robot code should own anything that answers one of these questions:

- What do the controls mean?
- What is our match strategy?
- What target matters for this game?
- When is it safe to feed/shoot/intake?
- What status does the driver need to see?

That usually means the robot should define objects like:

- `MyRobotTeleOpControls`
- `ScoringTargeting`
- `ShooterSupervisor`
- `MyRobotTelemetryPresenter`

These are thin, explicit owners. They keep the robot's behavior understandable without teaching the framework about one specific season's control scheme.

---

## Recommended structure for a new robot

```text
MyRobot
  MyRobotProfile

  FtcMecanumDriveLane drive
  FtcLocalizationLane localization

  MyRobotTeleOpControls controls
  MyTargetingService targeting
  MyMechanismSubsystem mechanism
  MySupervisor supervisor
  MyTelemetryPresenter telemetry
```

The composition root should wire these together and own loop order. It should not silently absorb their responsibilities.

---

## A useful rule of thumb

When you are deciding where a new behavior belongs, ask:

### “Would I expect this to be the same on multiple future robots?”

- If **yes**, and it is mostly about stable hardware/resource ownership, it probably belongs in a **framework lane**.
- If **yes**, and it is one narrow behavior with no policy, it probably belongs in a **primitive**.
- If **no**, or it expresses control semantics / game logic, it belongs in **robot code**.

---

## Why Phoenix avoids giant base robot classes

A large inheritance-based `BaseCompetitionRobot` usually turns into a pile of hooks, hidden assumptions, and game-specific convenience methods. That makes it easy to blur boundaries and hard to understand who owns what.

Phoenix prefers:

- composition
- explicit owners
- small configs
- status snapshots
- framework lanes for stable reusable bundles

That keeps the architecture principled while still reducing year-to-year boilerplate.
