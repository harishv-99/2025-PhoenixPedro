# Drive Guidance

`DriveGuidance` is the drivetrain consumer of the shared spatial-query layer. It turns field/robot geometry into a `DriveSignal` overlay or autonomous task.

Use Drive Guidance when the **drivetrain** should correct translation, heading/facing, or both. Use [`Spatial Queries.md`](<Spatial Queries.md>) directly when you only want raw geometry. Use [`Mechanism Target Planning.md`](<Mechanism Target Planning.md>) when a **mechanism Plant** should move independently to a scalar target.

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

## Guided builder shape

`DriveGuidance.plan()` and `PlantTargets.plan(request)` both answer required questions in order,
then enter optional tuning branches only when needed. Drive Guidance starts with a target-choice
stage; mechanism planning instead requires its fixed or live request at the factory boundary.

```text
DriveGuidance.plan()
    target question: choose the first channel with translateTo() or faceTo(), then optionally add the other with andFaceTo() or andTranslateTo()
    optional frame question: controlFrames(...)
    solve question: solveWith().localizationOnly..., aprilTagsOnly..., or adaptive...
    optional tuning: driveTuning()
    build()
```

`build()` is not visible until at least one target and one solve strategy are chosen. Target choice methods such as `point(...)`, `fieldPointInches(...)`, and `frameHeading(...)` return to the parent stage immediately because they answer exactly one choice. Adaptive-only knobs such as `translationTakeover(...)` and `omegaPolicy(...)` only appear in the adaptive branch, and multi-setting tuning branches still use explicit `done...()` methods.

## Common TeleOp pattern: button-held omega override

This example keeps driver translation from the sticks, but overrides omega while the button is held so a shooter frame faces a scoring point offset from an AprilTag.
The season-independent [framework examples index](<../examples/README.md>) places this advanced
policy after the managed drive and localization lessons; ordinary robot code keeps the guidance
plan in a robot-owned service and leaves the managed host responsible for loop and cleanup.

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
        .faceTo()
            .point(scoringPoint)
        .controlFrames(
                SpatialControlFrames.robotCenter()
                        .withFacingFrame(robotToShooterFrame)
        )
        .solveWith()
            .adaptive()
            .localization(globalPoseEstimator)
            .aprilTags(tagSensor, cameraMount)
            .localizationMaxAgeSec(0.50)
            .localizationMinQuality(0.10)
            .aprilTagMaxAgeSec(0.25)
            .fixedAprilTagLayout(fixedTagLayout)
            .doneAdaptive()
        .driveTuning()
            .aimKp(2.5)
            .aimDeadbandRad(Math.toRadians(1.0))
            .doneDriveTuning()
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

## Runtime ownership and cycle safety

`DriveGuidancePlan` is reusable configuration; each call to `overlay()`, `query()`, or
`task(driveSink, taskConfig)` creates fresh runtime state. These plan-owned methods are the public
construction paths for guidance runtimes. Give each overlay instance exactly one activation owner. In ordinary
robot code that means calling `plan.overlay()` once for each stack layer instead of retaining one
overlay and installing it in multiple layers or stacks:

```java
DriveSource drive = DriveOverlayStack.on(manualDrive)
        .add("shooterFacing", aimButton, shooterAim.overlay(), DriveOverlayMask.OMEGA_ONLY)
        .build();
```

The student-facing call is unchanged; cycle protection lives inside the framework. After a built
stack completes one evaluation, repeated reads in that cycle return the same command without
resampling its base, activation gates, or enabled overlays. A guidance runtime similarly advances
adaptive blending and controllers once. Failed evaluation is not cached as success, so a retry may
resample the graph instead of hiding the failure behind stale or null output; each stateful source
or overlay still protects its own advancing state for that retry.

An overlay-stack builder is single-use. After `build()`, do not add another layer or build another
stack from that builder. A stack also rejects the same overlay object in two layers because two
independent activation gates cannot truthfully own one overlay's enable/disable state. If two gates
mean one behavior, combine the gates; if they mean independent behaviors, create a fresh
`plan.overlay()` for each.

One guidance runtime also uses one requested `DriveOverlayMask` in a cycle. Equal same-cycle reads
return the same result. If two consumers need different masks, use the plan's natural/union mask or
create independent runtimes; asking one runtime for a second mask in that cycle fails fast instead
of advancing one controller state twice. `DriveGuidanceQuery.get(clock)` and `sample(clock)` use the
plan's natural mask, so ordinary readiness code needs no mask bookkeeping.

Overlay activation remains the stack's lifecycle: it calls `onEnable(clock)` and
`onDisable(clock)` exactly once per transition. There is intentionally no `DriveOverlay.reset()`
hook. Resetting a guidance query or re-enabling an overlay clears only that runtime's behavior and
query memory; it does not reset frame providers, solve lanes, sensors, estimators, or selected-tag
policies borrowed through the plan's reusable spatial spec. Those collaborators remain owned by
the robot services that supplied them.

## Controller tuning

`DriveGuidancePlan.Tuning` is an immutable, reusable description of how spatial error becomes a
normalized drive command. Start from `Tuning.defaults()` when robot code stores or shares the
complete value. Use the `driveTuning()` branch, as in the example above, when the answers belong
only to one plan. Both paths construct the same validated value; neither is live tuning.

| Setting | Default | Meaning | Required software domain |
| --- | ---: | --- | --- |
| `kPTranslate` | `0.05` | normalized command per inch of translation error | finite and `>= 0` |
| `maxTranslateCmd` | `0.60` | maximum normalized translation-command magnitude | finite in `[0, 1]` |
| `kPAim` | `2.50` | normalized omega command per radian of aim error | finite and `>= 0` |
| `maxOmegaCmd` | `0.80` | maximum normalized omega-command magnitude | finite in `[0, 1]` |
| `minOmegaCmd` | `0.00` | minimum normalized omega command outside the deadband | finite in `[0, maxOmegaCmd]`; a positive value also requires positive `kPAim` |
| `aimDeadbandRad` | `Math.toRadians(1.0)` | wrapped aim error that produces zero omega | finite in `[0, Math.PI]` |

Zero gains or command caps can deliberately disable their channel. Zero minimum omega disables the
stiction assist, while a deadband of pi suppresses every canonical wrapped aim error. Every immutable
intermediate must be coherent, so when reducing both nonzero omega limits, reduce `minOmegaCmd`
before reducing `maxOmegaCmd` below the old minimum.

For a positive cap, the translation controller preserves the direction of a finite error vector.
It enforces every accepted normalized cap, including zero and very small positive values, and
recovers that finite-error direction when multiplication by a very large finite gain would otherwise
overflow. This software bound does not prove that the defaults—or any other accepted values—are
physically safe or well tuned for a drivetrain. Validate direction, response, and safe limits on the
actual robot before relying on an assist.

## Autonomous Task configuration

`DriveGuidanceTask.Config` is a mutable construction input, not a live-tuning handle. A direct
`plan.task(driveSink, config)` call validates the complete input and retains a private snapshot
before it returns. Changing the same `config` afterward cannot change that Task; the new values are
used only by a later Task constructed from the edited input.

The task-level settings are separate from the controller tuning stored in the plan:

| Setting | Default | Required software domain |
| --- | ---: | --- |
| `positionTolInches` | `1.5` | finite and `>= 0` |
| `headingTolRad` | `Math.toRadians(6.0)` | finite and `>= 0` |
| `timeoutSec` | `3.0` | finite and `> 0` |
| `maxNoGuidanceSec` | `0.35` | finite and `> 0` |

`positionTolInches` and `headingTolRad` decide when the requested translation and facing work is
complete. `timeoutSec` bounds the complete Task; `maxNoGuidanceSec` bounds one consecutive interval
without a usable guidance command. Zero, a negative value, `NaN`, or infinity is not a spelling for
disabling either timeout. If `requestedMask` is `null`, the Task uses the plan's natural mask.

These checks prove only that the software configuration is coherent. They do not prove that the
tolerances or time budgets are appropriate for a particular drivetrain, that guidance is tuned
safely, or that the robot will physically reach its target.

## Known-clear rectangular parking assist

A robot-owned parking assist can reuse the ordinary field-relative Go-to-Pose Task. It must own an
explicit target translation and one or more reviewed target headings; the framework does not derive
a target or collision-free route from the parking box.

For one required orientation, put that heading directly in the target `Pose2d`. When several
orientations are acceptable, choose the nearest one exactly once from a usable current pose and
freeze it into the Task's target:

```java
double selectedHeadingRad = SpatialMath2d.nearestHeadingRad(
        fieldToRobot.headingRad,
        PARK_HEADINGS_RAD
);
Pose2d parkTarget = new Pose2d(PARK_X_INCHES, PARK_Y_INCHES, selectedHeadingRad);

Task parkTask = GoToPoseTasks.goToPoseFieldRelative(
        poseEstimator,
        drivebase,
        parkTarget,
        driveTuning,
        taskConfig
);
```

The ordered heading list belongs to the robot's configuration and must be defensively copied. Before
constructing hardware or behavior owners, check every complete candidate pose at the reviewed target
translation with a conservative `RobotFrameRectangle2d` and the authored
`AxisAlignedBoxRegion2d`; reject the configuration if any candidate is not
`rectangle.fullyInside(box, candidatePose)`. An off-center tracked origin uses
`RobotFrameRectangle2d.fromRobotFrameBoundsInches(...)` rather than pretending the origin is the
rectangle center.

Do not reselect the heading each loop near a tie. If the current pose is unavailable or non-finite,
do not start automatic motion; report the assist as unavailable and leave manual drive available.
`fullyInside(...)` reports only whether all four modeled corners are inside or on the box at that
instant. `hasAnyCornerInside(...)` answers only its literal corner question: it is not a general
overlap test and neither predicate proves support, collision clearance, occupancy, legality, or an
official score.

Use this assist only for a designated known-clear box with conservative pose and physical clearance.
A shared or partially occupied box remains a manual driver decision. The final-pose check does not
prove that the robot can turn or drive there without sweeping outside the clear area.

## Readiness / telemetry query

A plan can create a runtime query for “are we ready?” checks:

```java
DriveGuidanceQuery shooterAimQuery = shooterAim.query();

DriveGuidanceStatus status = shooterAimQuery.get(clock);
boolean readyToShoot = status.omegaWithin(Math.toRadians(1.0));

telemetry.addData("shooterFacing.errorDeg", Math.toDegrees(status.omegaErrorRad));
telemetry.addData("shooterFacing.ready", readyToShoot);
```

`DriveGuidanceQuery` implements `Source<DriveGuidanceStatus>`, so it fits the Phoenix source graph.
Create one query per independent owner because each query owns its cycle cache, blend/controller
state, and explicit reset lifecycle. The plan and its robot-owned spatial dependencies may still be
shared safely.

## Translation + facing

A plan can solve translation, facing, or both. Start with the first channel, then add the second with `andFaceTo()` or `andTranslateTo()`:

```java
DriveGuidancePlan alignToSlot = DriveGuidance.plan()
        .translateTo()
            .point(References.framePoint(slotFrame, -6.0, 0.0))
        .andFaceTo()
            .frameHeading(slotFrame)
        .controlFrames(SpatialControlFrames.robotCenter())
        .solveWith()
            .localizationOnly()
            .localization(globalPoseEstimator)
            .fixedAprilTagLayout(tagLayout)
            .doneLocalizationOnly()
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
- Mechanism Target Planning maps requests to caller-facing Plant-unit targets.

The framework keeps this difference because a drivetrain command domain is known, but a mechanism may use ticks, inches, servo positions, rotations, or another scalar coordinate.

## Dynamic camera mounts

For a fixed webcam or fixed Limelight, pass a fixed `CameraMountConfig`:

```java
.solveWith()
    .aprilTagsOnly()
    .aprilTags(tagSensor, fixedCameraMount)
    .maxAgeSec(0.25)
    .doneAprilTagsOnly()
```

For a moving camera used outside DriveGuidance, pass a timestamp-aware source through
`SpatialSolveSet.builder().aprilTags(...)`; see [`Spatial Queries.md`](<Spatial Queries.md>) and
[`Mechanism Target Planning.md`](<Mechanism Target Planning.md>). This lets the AprilTag lane
interpret delayed camera frames using the camera pose from the frame timestamp. Phoenix carries that
capture time as one `LoopTimestamp`, so camera, frame-history, spatial, and guidance code do not
separately pass or compare a raw clock epoch and timestamp.

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

For a turret with its own Plant, do not use Drive Guidance to turn the robot. Use `SpatialQuery`
plus `PlantTargets.equivalentPositionsOf(...)` for one current logical angle, or the advanced
`PlantTargets.plan(request)` path when alternative/observation metadata must be retained, as shown in
[`Mechanism Target Planning.md`](<Mechanism Target Planning.md>).

## When not to use Drive Guidance

Use a direct `DriveSource` when the driver or autonomous routine already knows the desired drive
command. Use an exact/equivalent `PlantTargetResolver` or advanced `PlantTargets.plan(request)` when a
mechanism should move independently of the drivetrain. Use `SpatialQuery` directly when you need
geometry but want to apply your own PID or mechanism logic.
