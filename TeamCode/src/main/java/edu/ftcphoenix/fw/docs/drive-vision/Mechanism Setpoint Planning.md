# Mechanism Setpoint Planning

`ScalarSetpointPlanner` sits **above a `Plant`**. It turns semantic or computed requests into a
feasible scalar setpoint in the same caller-facing units as the downstream Plant target.

It does not run the actuator PID. The `Plant` still owns low-level regulation. The planner decides
which target value is safe and meaningful to command.

```text
controller / service
    decides behavior is active
        ↓
request source
    exact, equivalent-position, explicit-period, one-of, relative, or spatial-derived request
        ↓
ScalarSetpointPlanner
    applies range, periodicity, candidate choice, loss policy
        ↓
Plant.setTarget(...)
    low-level regulation in the plant's public units
```

## Unit rule

The scalar planner is unit-agnostic. These must all use the same public plant coordinate:

- request value
- request period, when an explicit period is supplied
- measurement
- range limits
- tolerances
- planner output setpoint
- `Plant` target

For a ticks-based turret, all values may be ticks. For an inches-based extension, all values are
inches. For a servo claw built with `rangeMapsToNative(0.30, 0.80)`, robot code and planner values
can be logical `0.0..1.0` while the Plant maps those to raw servo fractions internally.

For `PositionPlant`s built through `FtcActuators`, the preferred planner setup is:

```java
ScalarSetpointPlanner planner = ScalarSetpoints.plan()
        .request(requestSource)
        .forPositionPlant(positionPlant)
        .build();
```

The staged builder asks two required questions explicitly:

1. where the request comes from (`request(...)` or `requestFromSpatial()`)
2. what scalar domain the planner commands (`forPositionPlant(...)` or `explicitDomain()`)

After those questions are answered, optional tuning lives in `policy()` and `completion()` branches instead of mixing directly into the required setup path.

That pulls the plant-unit measurement, dynamic target range, and periodicity from the plant itself.
If the plant still needs a reference, the range source is invalid and the planner blocks instead of
producing an unsafe target.

## When to use the planner

Use `ScalarSetpointPlanner` when it adds value:

- the mechanism has travel limits or homing-dependent limits
- the request may be periodic, such as a spinner or turret
- several candidates are acceptable, such as multiple tray slots
- you want consistent `atSetpoint()` vs `requestSatisfied()` status
- you need loss behavior like holding the last setpoint

Control the `Plant` directly when the request is already simple and always legal:

```java
liftPlant.setTarget(BASKET_TICKS);
liftPlant.update(clock);
```

That is fine for many beginner cases. Reach for the planner when the target is selected,
constrained, periodic, or contextual.

## Exact request: lift preset

```java
PositionPlant lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .build();

ScalarSetpointPlanner liftPlanner = ScalarSetpoints.plan()
        .request(clock -> ScalarSetpointRequest.exact("basket", BASKET_TICKS))
        .forPositionPlant(lift)
        .completion()
            .atSetpointTolerance(20.0)
            .requestSatisfiedTolerance(20.0)
            .doneCompletion()
        .build();
```

Before homing, `lift.targetRangeSource()` reports `ScalarRange.invalid("lift not homed")`, so the
planner blocks. After a homing task establishes the reference, the same planner produces bounded
plant-unit setpoints.

Loop usage:

```java
ScalarSetpointStatus s = liftPlanner.get(clock);
if (s.hasSetpoint()) {
    lift.setTarget(s.getSetpoint());
}
lift.update(clock);
```

## Equivalent-position request: free spinner or tray

A free spinner/tray can declare its own period in plant units. If robot code wants to speak in
degrees while the motor uses ticks, use `scaleToNative(...)`.

```java
PositionPlant tray = FtcActuators.plant(hardwareMap)
        .motor("tray", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .periodic(360.0)
            .unbounded()
            .scaleToNative(TRAY_TICKS_PER_DEGREE)
            .needsReference("tray index mark not found")
        .positionTolerance(2.0)
        .build();
```

After the tray is indexed, a request can say "slot 2 is at 240 degrees modulo the plant's period":

```java
ScalarSetpointPlanner trayPlanner = ScalarSetpoints.plan()
        .request(clock -> ScalarSetpointRequest.equivalentPosition("slot-2", 240.0))
        .forPositionPlant(tray)
        .policy()
            .candidatePreference(ScalarSetpointPlanner.CandidatePreference.NEAREST_TO_MEASUREMENT)
            .donePolicy()
        .build();
```

The request does not repeat the period. The plant owns that topology. If the plant were linear,
`equivalentPosition(...)` would not produce a setpoint because no plant period exists.

## One-of candidates: tray with colored artifacts

A tray service can convert robot-specific inventory into plant-unit candidates. The planner chooses
the nearest feasible representative.

```java
Source<ScalarSetpointRequest> purpleToOutput = clock -> {
    ArrayList<ScalarSetpointCandidate> candidates = new ArrayList<>();

    for (int slot = 0; slot < 3; slot++) {
        if (inventory.colorAt(slot) == ArtifactColor.PURPLE) {
            double alignDeg = trayModel.alignSlotToOutputDegrees(slot);
            candidates.add(ScalarSetpointCandidate.equivalentPosition(
                    "slot-" + slot + "-purple",
                    alignDeg
            ));
        }
    }

    return candidates.isEmpty()
            ? ScalarSetpointRequest.none("no purple artifact")
            : ScalarSetpointRequest.oneOf(candidates);
};

ScalarSetpointPlanner trayPlanner = ScalarSetpoints.plan()
        .request(purpleToOutput)
        .forPositionPlant(tray)
        .policy()
            .candidatePreference(ScalarSetpointPlanner.CandidatePreference.NEAREST_TO_MEASUREMENT)
            .donePolicy()
        .completion()
            .atSetpointTolerance(2.0)
            .doneCompletion()
        .build();
```

The planner does not know what “purple” means. It only sees plant-unit candidates.

## Spatial-derived request: turret with camera and localization fallback

A turret can use one `SpatialQuery` with two lanes:

1. live AprilTags from the turret-mounted camera
2. absolute pose / localization fallback

Then the scalar builder maps the selected facing solution into the turret Plant's public units.

```java
PositionPlant turret = FtcActuators.plant(hardwareMap)
        .motor("turret", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .periodic(360.0)
            .bounded(-225.0, 225.0)
            .scaleToNative(TURRET_TICKS_PER_DEGREE)
            .needsReference("turret not homed")
        .positionTolerance(1.5)
        .build();

SpatialSolutionGate gate = SpatialSolutionGate.builder()
        .maxAgeSec(0.20)
        .minQuality(0.45)
        .build();

ScalarSetpointPlanner turretPlanner = ScalarSetpoints.plan()
        .requestFromSpatial()
            .faceTo(SpatialTargets.point(
                    References.relativeToTagPoint(20, 6.0, -1.5)
            ))
            .controlFrames(
                    SpatialControlFrames.robotCenter()
                            .withFacingFrame(robotToTurretToolZeroFrame)
            )
            .solveWith(
                    SpatialSolveSet.builder()
                            .aprilTags(turretCameraTags, turretCameraMountHistory, 0.15)
                            .absolutePose(globalPoseEstimator, 0.50, 0.10)
                            .build()
            )
            .fixedAprilTagLayout(fixedTagLayout)
            .selectWith(gate)
            .mapToRequest((facing, clock) -> {
                double targetDeg = Math.toDegrees(facing.facingErrorRad());
                return ScalarSetpointRequest.equivalentPosition(
                        facing.sourceId(),
                        targetDeg,
                        facing.quality(),
                        facing.ageSec(),
                        facing.timestampSec()
                );
            })
            .doneRequest()
        .forPositionPlant(turret)
        .policy()
            .candidatePreference(ScalarSetpointPlanner.CandidatePreference.NEAREST_TO_MEASUREMENT)
            .lossPolicy(ScalarSetpointPlanner.LossPolicy.HOLD_LAST_SETPOINT)
            .donePolicy()
        .completion()
            .atSetpointTolerance(1.5)
            .requestSatisfiedTolerance(2.0)
            .doneCompletion()
        .build();
```

`atSetpoint()` and `requestSatisfied()` are intentionally different. If cable limits force the
setpoint to clamp, the plant may be at the clamped setpoint while the original request is not truly
satisfied.

## Translation-derived request: extension toward a pickup point

`requestFromSpatial().translateTo(...)` is for cases where field geometry should become a scalar
target through robot-specific kinematics. The staged spatial-request builder makes the target kind
explicit: choose either `faceTo(...)` or `translateTo(...)`, then provide the shared spatial solve
configuration and a matching `mapToRequest(...)` function.

```java
ScalarSetpointPlanner extensionPlanner = ScalarSetpoints.plan()
        .requestFromSpatial()
            .translateTo(SpatialTargets.point(detectedGamePiecePoint))
            .controlFrames(
                    SpatialControlFrames.robotCenter()
                            .withTranslationFrame(clawLandingFrameHistory)
            )
            .solveWith(solveSet)
            .selectWith(gate)
            .mapToRequest((translation, clock) -> {
                double extensionInches = pickupKinematics.extensionInchesFor(
                        translation.solution.frameForwardInches(),
                        translation.solution.frameLeftInches()
                );
                return ScalarSetpointRequest.exact(
                        "pickup-extension",
                        extensionInches,
                        translation.quality(),
                        translation.ageSec(),
                        translation.timestampSec()
                );
            })
            .doneRequest()
        .forPositionPlant(extension)
        .build();
```

The framework solves the spatial relationship. Robot-specific kinematics map that relationship into
plant units.

## Calibration belongs next to the mechanism

The scalar planner assumes the mechanism coordinate is meaningful or that the PositionPlant will
publish an invalid range until it becomes meaningful. A turret or lift service owns homing/indexing
Tasks and semantic goals.

```java
Task homeLift = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .thenHold(0.0)
        .failAfterSec(3.0)
        .build();
```

The timeout policy is explicit: use `failAfterSec(...)` for a bounded search or `neverTimeout()` only when another safety path is guaranteed to cancel the task.

The planner should not decide when zero is trustworthy. Homing, indexing, manual zeroing, and
semantic presets belong in the robot mechanism/service layer.

## Parallel with Drive Guidance

Both builders can consume the same spatial vocabulary:

```java
DriveGuidancePlan drivePlan = DriveGuidance.plan()
        .faceTo()
            .point(scoringPoint)
            .doneFaceTo()
        .controlFrames(SpatialControlFrames.robotCenter().withFacingFrame(robotToShooterFrame))
        .solveWith()
            .adaptive()
            .localization(globalPoseEstimator)
            .aprilTags(tagSensor, cameraMount)
            .fixedAprilTagLayout(tagLayout)
            .doneAdaptive()
        .build();
```

```java
ScalarSetpointPlanner turretPlanner = ScalarSetpoints.plan()
        .requestFromSpatial()
            .faceTo(SpatialTargets.point(scoringPoint))
            .controlFrames(SpatialControlFrames.robotCenter().withFacingFrame(robotToTurretFrame))
            .solveWith(solveSet)
            .fixedAprilTagLayout(tagLayout)
            .mapToRequest(turretFacingMapper)
            .doneRequest()
        .forPositionPlant(turret)
        .build();
```

The principled difference is the output boundary: Drive Guidance knows how to turn a facing error
into drivetrain omega. A scalar mechanism must map spatial geometry into its own public plant units.
