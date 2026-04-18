# Spatial Queries

`SpatialQuery` is Phoenix's shared **task-space relationship solver**.

It answers questions like:

- where is this target relative to the robot's translation frame?
- how much should this aim frame rotate to face that point?
- what do different solve lanes think about the same target this loop?

That makes it the reusable layer underneath `DriveGuidance` and a good foundation for future
mechanism planners.

---

## 1. What problem does `SpatialQuery` solve?

A `SpatialQuery` separates four concerns that used to live together inside Drive Guidance:

1. **Target semantics** — what point or heading is meaningful?
2. **Controlled frames** — which robot-relative point/frame should satisfy that geometry?
3. **Solve lanes** — which backends are allowed to solve it?
4. **Per-lane results** — what each lane solved this loop

The query itself does **not** decide which result to trust or how to turn that result into a
drivetrain or mechanism command.

That policy belongs to higher-level consumers:

- `DriveGuidance` turns the query result into drivetrain translation/omega commands.
- A future turret planner can turn the same aim relationship into a turret angle target.
- A future pickup planner can turn the same translation relationship into extension / lift / wrist
  targets.

---

## 2. The main public pieces

### 2.1 Semantic targets

Use the neutral target helpers in `edu.ftcphoenix.fw.spatial`:

- `ReferencePoint2d`
- `ReferenceFrame2d`
- `References`
- `TranslationTarget2d`
- `AimTarget2d`
- `SpatialTargets`

Most teams should not implement `TranslationTarget2d` or `AimTarget2d` directly. Use the helpers in
`SpatialTargets`.

Examples:

```java
ReferenceFrame2d backdropFace = References.fieldFrame(48.0, 24.0, Math.PI);
ReferencePoint2d settlePoint = References.framePoint(backdropFace, -6.0, 0.0);

TranslationTarget2d translationTarget = SpatialTargets.point(settlePoint);
AimTarget2d aimTarget = SpatialTargets.frameHeading(backdropFace);
```

### 2.2 Controlled frames

`SpatialControlFrames` describes **which robot-relative frames are being controlled**.

There are two channels:

- `translationFrame`
- `aimFrame`

The API accepts either:

- a rigid `Pose2d`, or
- a dynamic `Source<Pose2d>` sampled once per loop

Examples:

```java
SpatialControlFrames robotCenter = SpatialControlFrames.robotCenter();
```

```java
Pose2d robotToShooter = new Pose2d(7.0, 4.5, Math.toRadians(5.0));

SpatialControlFrames shooterAim = SpatialControlFrames.robotCenter()
        .withAimFrame(robotToShooter);
```

```java
Source<Pose2d> clawLandingFrame = clock -> projectClawLandingPointOnFloor(clock);

SpatialControlFrames pickupFrames = SpatialControlFrames.robotCenter()
        .withTranslationFrame(clawLandingFrame);
```

The important rule is: the frame is always expressed as **`robot -> frame`**.

### 2.3 Solve lanes

A `SpatialSolveLane` is one strategy/backend that can solve some or all of the query.

Phoenix currently provides two reusable lanes:

- `AbsolutePoseSpatialSolveLane` — solve from any `AbsolutePoseEstimator`
- `AprilTagSpatialSolveLane` — solve directly from live `AprilTagSensor` observations, with an
  optional fixed-tag field-pose bridge when the target requires field coordinates

Those lanes are composed with `SpatialSolveSet`.

Examples:

```java
SpatialSolveSet lanes = SpatialSolveSet.builder()
        .absolutePose(globalPoseEstimator, 0.50, 0.10)
        .aprilTags(tagSensor, cameraMount, 0.25, fieldPoseSolverConfig)
        .build();
```

Because the solve set is interface-based, teams can add their own lanes later without changing the
shared query API.

### 2.4 Spec and runtime query

`SpatialQuerySpec` is the immutable description.

`SpatialQuery` is the stateful runtime owner.

```java
SpatialQuerySpec spec = SpatialQuery.builder()
        .translateTo(SpatialTargets.point(settlePoint))
        .aimTo(SpatialTargets.frameHeading(backdropFace))
        .controlFrames(SpatialControlFrames.robotCenter().withAimFrame(robotToShooter))
        .solveWith(lanes)
        .fixedAprilTagLayout(tagLayout)
        .build();

SpatialQuery query = new SpatialQuery(spec);
```

---

## 3. Sampling a query

Call `query.get(clock)` once per loop (or let a higher-level owner do it).

```java
SpatialQuerySample sample = query.get(clock);
```

A `SpatialQuerySample` contains:

- the sampled `robotToTranslationFrame`
- the sampled `robotToAimFrame`
- one `SpatialLaneResult` per solve lane

This matters for dynamic frames. `SpatialQuery` samples the control frames **once per loop** and
passes the exact same snapshot to every solve lane so you can compare lanes fairly.

---

## 4. Interpreting per-lane results

Each `SpatialLaneResult` may contain:

- a `TranslationSolution`
- an `AimSolution`
- translation-selection telemetry
- aim-selection telemetry

A lane can solve:

- only translation
- only aim
- both
- or neither

That means a higher-level consumer can choose its own policy. For example:

- `DriveGuidance` can blend between localization and live AprilTags.
- A turret planner can prefer a direct vision lane when it is valid.
- A pickup planner can ignore lanes that only solve aim.

The spatial layer intentionally does **not** hide that policy.

---

## 5. How `DriveGuidance` uses `SpatialQuery`

`DriveGuidance` now consumes the shared spatial-query layer internally.

That means:

- the geometry is authored once,
- the same solve lanes can later be reused by non-drive planners,
- and drive-specific policy stays in `DriveGuidance`.

One drive-only target still stays outside the shared query layer:

- `DriveGuidanceSpec.RobotRelativePoint`

That target is special because it is not just geometry. It means:

> capture the translation frame's field pose when guidance enables, then offset from that latched
> anchor.

That lifecycle/enable semantic is specific to drive guidance today, so it stays there until another
real consumer of the same behavior shows up.

---

## 6. Example: direct query for turret-style aim geometry

This example does **not** command a turret yet. It only shows how to reuse the shared query layer to
solve the target relationship that a future turret planner would consume.

```java
Pose2d robotToTurretZeroFrame = new Pose2d(
        6.5,
        2.0,
        Math.toRadians(15.0)
);

SpatialSolveSet lanes = SpatialSolveSet.builder()
        .absolutePose(globalPoseEstimator, 0.50, 0.10)
        .aprilTags(tagSensor, cameraMount, 0.25)
        .build();

SpatialQuery turretAimQuery = new SpatialQuery(
        SpatialQuery.builder()
                .aimTo(SpatialTargets.point(
                        References.relativeToSelectedTagPoint(scoringSelection, 0.0, 0.0)
                ))
                .controlFrames(
                        SpatialControlFrames.robotCenter().withAimFrame(robotToTurretZeroFrame)
                )
                .solveWith(lanes)
                .fixedAprilTagLayout(tagLayout)
                .build()
);

SpatialQuerySample sample = turretAimQuery.get(clock);
SpatialLaneResult localizationLane = sample.laneResult(0);
if(localizationLane.aim !=null){
double desiredAimErrorRad = localizationLane.aim.aimErrorRad;
// A future mechanism planner would turn this into a commanded turret angle.
}
```

---

## 7. Example: floor-pickup geometry with a dynamic controlled frame

For a floor pickup, the controlled point may not be welded to the chassis. It may be the projected
landing point of the claw when lowered.

```java
Source<Pose2d> clawLandingFrame = clock -> projectClawLandingPointOnFloor(clock);

SpatialQuery pickupQuery = new SpatialQuery(
        SpatialQuery.builder()
                .translateTo(SpatialTargets.point(detectedObjectReference))
                .controlFrames(
                        SpatialControlFrames.robotCenter().withTranslationFrame(clawLandingFrame)
                )
                .solveWith(lanes)
                .build()
);
```

The shared spatial layer gives you the relationship between:

- the detected object target, and
- the current projected claw landing frame

A robot-specific pickup planner can then turn that task-space error into extension / lift / wrist
setpoints.

---

## 8. Relationship to localization

Localization and spatial queries are related, but they are not the same layer.

- **localization** answers: “where is the robot on the field?”
- **spatial query** answers: “where is this target relative to this controlled frame?”

A spatial solve lane may consume localization. For example, `AbsolutePoseSpatialSolveLane` can solve
from a corrected global pose.

But a pose estimator should not generally consume `SpatialQuery`. Queries are target-dependent; pose
estimation is target-agnostic.

---

## 9. When to use what

Use `DriveGuidance` when you want the framework to output a drivetrain command.

Use `SpatialQuery` directly when you want:

- task-space geometry without drivetrain policy,
- per-lane comparison data,
- dynamic controlled-frame solving,
- or a reusable solve layer for a custom planner.

If all you need is a yes/no geometric predicate, stay even lower level and use the simpler spatial
helpers such as `RobotZones2d` or `RobotHeadings2d`.
