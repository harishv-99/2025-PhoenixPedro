# Drive Guidance

`DriveGuidance` is the drivetrain consumer of the shared spatial-query layer. It turns field/robot geometry into a `DriveSignal` overlay or autonomous task.

Use Drive Guidance when the **drivetrain** should correct translation, heading/facing, or both. Use [`Spatial Queries.md`](<Spatial Queries.md>) directly when you only want raw geometry. Use [`Mechanism Setpoint Planning.md`](<Mechanism Setpoint Planning.md>) when a **mechanism Plant** should move independently to a scalar setpoint.

## Mental model

```text
SpatialQuery
    solves target vs control-frame geometry
        ↓
DriveGuidanceCore
    applies drive policy, blending, controller tuning
        ↓
DriveGuidanceStatus / DriveSignal
        ↓
Drive overlay, task, or telemetry gate
```

`DriveGuidancePlan` is a reusable behavior description: target, control frames, solve lanes, and drive tuning. `DriveGuidanceQuery` is the runtime source used for telemetry and readiness checks. Overlays, tasks, and queries use the same underlying evaluation logic.

## Common TeleOp pattern: button-held omega override

This example keeps driver translation from the sticks, but overrides omega while the button is held so a shooter frame faces a scoring point offset from an AprilTag.

```java
Pose2d robotToShooterFrame = new Pose2d(
        8.0,                  // shooter is 8" forward of robot center
        2.0,                  // and 2" left of robot center
        Math.toRadians(3.0)   // shooter +X points 3 deg left of robot +X
);

ReferencePoint2d scoringPoint = References.relativeToTagPoint(
        20,
        6.0,   // 6" forward from the tag frame
        -1.5   // 1.5" right from the tag frame
);

DriveGuidancePlan shooterAim = DriveGuidance.plan()
        .faceTo(SpatialTargets.point(scoringPoint))
        .controlFrames(
                SpatialControlFrames.robotCenter()
                        .withFacingFrame(robotToShooterFrame)
        )
        .solveWith()
            .adaptive()
            .aprilTags(tagSensor, cameraMount, 0.25)
            .localization(globalPoseEstimator, 0.50, 0.10)
            .fixedAprilTagLayout(fixedTagLayout)
            .doneSolveWith()
        .tuning(DriveGuidancePlan.Tuning.defaults()
                .withAimKp(2.5)
                .withAimDeadbandRad(Math.toRadians(1.0)))
        .build();

DriveSource drive = DriveOverlayStack.on(manualDrive)
        .add("shooterFacing", aimButton, shooterAim.overlay(), DriveOverlayMask.OMEGA_ONLY)
        .build();
```

The important separation is:

- `robotToShooterFrame` is the controlled frame that should face the target.
- `cameraMount` is the sensor frame used by the AprilTag solve lane.
- `scoringPoint` is the semantic target, here offset from tag 20.

The camera does not need to be centered or aligned with the shooter.

## Readiness / telemetry query

A plan can create a runtime query for “are we ready?” checks:

```java
DriveGuidanceQuery shooterAimQuery = shooterAim.query();

DriveGuidanceStatus status = shooterAimQuery.get(clock);
boolean readyToShoot = status.omegaWithin(Math.toRadians(1.0));

telemetry.addData("shooterFacing.errorDeg", Math.toDegrees(status.omegaErrorRad));
telemetry.addData("shooterFacing.ready", readyToShoot);
```

`DriveGuidanceQuery` implements `Source<DriveGuidanceStatus>`, so it fits the Phoenix source graph. Create one query per independent owner because it is stateful.

## Translation + facing

A plan can solve translation, facing, or both:

```java
DriveGuidancePlan alignToSlot = DriveGuidance.plan()
        .translateTo(SpatialTargets.point(References.framePoint(slotFrame, -6.0, 0.0)))
        .faceTo(SpatialTargets.frameHeading(slotFrame))
        .controlFrames(SpatialControlFrames.robotCenter())
        .solveWith()
            .localization(globalPoseEstimator)
            .fixedAprilTagLayout(tagLayout)
            .doneSolveWith()
        .build();
```

The translation frame and facing frame may differ:

```java
SpatialControlFrames frames = SpatialControlFrames.robotCenter()
        .withTranslationFrame(robotToIntakePoint)
        .withFacingFrame(robotToShooterFrame);
```

## Relationship to `SpatialQuery`

Drive Guidance uses `SpatialQuery` internally. The same concepts are shared with mechanism planners:

```text
faceTo(...)
translateTo(...)
controlFrames(...)
solveWith(...)
fixedAprilTagLayout(...)
```

The output boundary is different:

- Drive Guidance maps spatial results to drivetrain `DriveSignal` commands.
- Scalar Setpoint Planning maps requests to caller-facing plant-unit setpoints.

The framework keeps this difference because a drivetrain command domain is known, but a mechanism may use ticks, inches, servo positions, rotations, or another scalar coordinate.

## Dynamic camera mounts

For a fixed webcam or fixed Limelight, pass a fixed `CameraMountConfig`:

```java
.solveWith()
    .aprilTags(tagSensor, fixedCameraMount, 0.25)
    .doneSolveWith()
```

For a moving camera used outside DriveGuidance, pass a timestamp-aware source through `SpatialSolveSet.aprilTags(...)`; see [`Spatial Queries.md`](<Spatial Queries.md>) and [`Mechanism Setpoint Planning.md`](<Mechanism Setpoint Planning.md>). This lets the AprilTag lane interpret delayed camera frames using the camera pose from the frame timestamp.

## Control frames and off-center facing

`SpatialControlFrames.withFacingFrame(...)` describes the frame whose +X axis should face the target. For a rigid shooter, this is often a fixed robot-relative pose. For a turret driven by its own Plant, this is usually the turret tool zero frame, not the camera frame.

```java
Pose2d robotToTurretToolZero = new Pose2d(
        7.0,
        2.5,
        Math.toRadians(10.0)
);
```

Meaning:

- the turret pivot/tool frame is 7" forward and 2.5" left of robot center
- when the turret mechanism coordinate is zero, its tool +X points 10° left of robot forward

For a turret with its own Plant, do not use Drive Guidance to turn the robot. Use `SpatialQuery` plus `ScalarSetpointPlanner` as shown in [`Mechanism Setpoint Planning.md`](<Mechanism Setpoint Planning.md>).

## When not to use Drive Guidance

Use a direct `DriveSource` when the driver or autonomous routine already knows the desired drive command. Use a direct `Plant` or `ScalarSetpointPlanner` when a mechanism should move independently of the drivetrain. Use `SpatialQuery` directly when you need geometry but want to apply your own PID or mechanism logic.
