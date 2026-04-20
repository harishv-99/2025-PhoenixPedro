# Spatial Queries

`SpatialQuery` is the shared framework layer for **field/robot geometry**. It answers a task-space question:

> Given a target, a controlled robot frame, and one or more solve lanes, what translation or facing relationship can each lane solve this loop?

It does **not** decide which lane wins, and it does **not** command a drivetrain or mechanism. That policy belongs to consumers such as Drive Guidance or Scalar Setpoint Planning.

## When to use `SpatialQuery`

Use `SpatialQuery` directly when you need raw geometry:

- compare live AprilTag solving against localization fallback
- run a simple PID on a facing error
- inspect translation/facing solutions for telemetry
- build a robot-specific mechanism planner that needs field-relative context

Do not use `SpatialQuery` when the target is already a plant-unit value. A lift preset such as `1420 ticks` should go straight to a `ScalarSetpointPlanner` or directly to a `Plant`.

## Main vocabulary

- `TranslationTarget2d`: the point a controlled frame should move toward.
- `FacingTarget2d`: the point or heading a controlled frame should face.
- `SpatialControlFrames`: robot-relative frame providers for translation and facing.
- `SpatialSolveLane`: one strategy for solving the relationship, such as absolute pose or live AprilTags.
- `SpatialQueryResult`: ordered per-lane results from one loop.
- `TranslationSolution`: solved target point in robot and controlled-frame coordinates.
- `FacingSolution`: signed facing error in radians.

The key naming rule is:

```text
Query -> Result
Planner/Guidance -> Status
```

`SpatialQuery` produces a `SpatialQueryResult`. `DriveGuidanceQuery` and `ScalarSetpointPlanner` produce status objects because they apply policy and choose commands/setpoints.

## Builder shape

`SpatialQuery` now follows the same guided-builder rule as Drive Guidance and Scalar Setpoint
Planning: answer the required conceptual questions explicitly, and do not expose `build()` before the
query has both a target and solve lanes.

The common path builds a runtime query directly:

```java
SpatialQuery query = SpatialQuery.builder()
        .faceTo(SpatialTargets.point(References.relativeToTagPoint(20, 6.0, -1.5)))
        .controlFrames(
                SpatialControlFrames.robotCenter()
                        .withFacingFrame(robotToShooterFrame)
        )
        .solveWith(
                SpatialSolveSet.builder()
                        .aprilTags(tagSensor, cameraMount, 0.25)
                        .absolutePose(globalPoseEstimator, 0.50, 0.10)
                        .build()
        )
        .fixedAprilTagLayout(tagLayout)
        .build();
```

The first question is the target relationship:

```java
SpatialQuery facingOnly = SpatialQuery.builder()
        .faceTo(SpatialTargets.fieldHeading(Math.PI))
        .solveWith(solveSet)
        .build();

SpatialQuery translationOnly = SpatialQuery.builder()
        .translateTo(SpatialTargets.fieldPoint(48.0, 24.0))
        .solveWith(solveSet)
        .build();

SpatialQuery both = SpatialQuery.builder()
        .translateTo(SpatialTargets.fieldPoint(48.0, 24.0))
        .andFaceTo(SpatialTargets.fieldHeading(Math.PI))
        .controlFrames(frames)
        .solveWith(solveSet)
        .build();
```

`controlFrames(...)` defaults to `SpatialControlFrames.robotCenter()`. `fixedAprilTagLayout(...)` is
optional and should be supplied only when a lane or target reference needs trusted field-tag
geometry.

Use `SpatialQuerySpec.builder()` only when you need a reusable immutable description and separate
runtime query instances:

```java
SpatialQuerySpec spec = SpatialQuerySpec.builder()
        .faceTo(SpatialTargets.fieldHeading(Math.PI))
        .controlFrames(SpatialControlFrames.robotCenter())
        .solveWith(solveSet)
        .build();

SpatialQuery driveSide = SpatialQuery.from(spec);
SpatialQuery telemetrySide = SpatialQuery.from(spec);
```

Do not share one stateful `SpatialQuery` between independent owners if either owner may call
`reset()`. Share a `SpatialQuerySpec` and create independent runtime queries from it.

## Control frame vs camera frame

A control frame is the thing you are trying to move or face. A camera frame is the sensor pose used by an AprilTag lane.

For a shooter:

```java
Pose2d robotToShooterFrame = new Pose2d(8.0, 2.0, Math.toRadians(3.0));
CameraMountConfig robotToCamera = profile.vision.webcam.cameraMount;
```

`robotToShooterFrame` belongs in `SpatialControlFrames.withFacingFrame(...)`. `robotToCamera` belongs in `SpatialSolveSet.aprilTags(...)`.

For a turret-mounted camera, those frames are still separate:

```java
Pose2d robotToTurretToolZero = new Pose2d(7.0, 2.5, Math.toRadians(10.0));
TimeAwareSource<CameraMountConfig> turretCameraMount = turretCameraMountHistory;

SpatialSolveSet solveSet = SpatialSolveSet.builder()
        .aprilTags(turretCameraTags, turretCameraMount, 0.15)
        .absolutePose(globalPoseEstimator, 0.50, 0.10)
        .build();
```

The query controls the turret tool frame. The AprilTag lane uses the dynamic camera frame to understand what the camera saw.

## Timestamp-aware frames

Fast moving mechanisms need more than “current pose.” A camera frame may be old by the time the loop reads it. If a turret moved during that delay, the AprilTag lane should interpret the tag using the turret camera mount from the frame timestamp.

Phoenix therefore supports `TimeAwareSource<T>` for dynamic frames and camera mounts. Fixed frames use `RobotFrames.rigid(...)` or `TimeAwareSources.fixed(...)`. Current-only dynamic frames can use `RobotFrames.currentOnly(...)`, but moving sensors should eventually use a history-backed source.

Rule of thumb:

> Anything derived from a delayed sensor frame should carry or derive a timestamp. Any moving frame used to interpret that sensor should be sampled at that timestamp when possible.

## Selecting lane results

The base query returns every lane result. Use selectors when you want priority behavior:

```java
SpatialSolutionGate gate = SpatialSolutionGate.builder()
        .maxAgeSec(0.20)
        .minQuality(0.45)
        .build();

SpatialQueryResult result = query.get(clock);
SpatialFacingSelection facing = SpatialQuerySelectors.firstValidFacing(result, gate);
```

The same selector/gate concepts should be used by Drive Guidance and scalar setpoint request builders so “valid enough” means the same thing across the framework.

## Direct query example: simple turret visual PID

A quick visual-servo turret can use the facing error directly and skip scalar setpoint planning:

```java
SpatialQueryResult result = turretTagQuery.get(clock);
SpatialFacingSelection facing = SpatialQuerySelectors.firstValidFacing(result, gate);

if (facing != null) {
    double power = turretPid.update(facing.facingErrorRad(), clock.dtSec());
    turretMotor.setPower(power);
} else {
    turretMotor.setPower(0.0);
}
```

This is simple, but you must handle cable limits, unreachable targets, and loss behavior yourself. For a position-controlled turret, prefer the scalar setpoint planner path described in [`Mechanism Setpoint Planning.md`](<Mechanism Setpoint Planning.md>).
