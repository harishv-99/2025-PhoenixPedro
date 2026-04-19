# Mechanism Setpoint Planning

`ScalarSetpointPlanner` sits **above a `Plant`**. It turns semantic or computed requests into a feasible scalar setpoint in the plant's native units.

It does not run the actuator PID. The `Plant` still owns low-level regulation. The planner decides which target value is safe and meaningful to command.

```text
controller / service
    decides behavior is active
        ↓
request source
    exact, periodic, one-of, relative, or spatial-derived request
        ↓
ScalarSetpointPlanner
    applies range, periodicity, candidate choice, loss policy
        ↓
Plant.setTarget(...)
    low-level regulation in native plant units
```

## Native units only

The scalar planner is unit-agnostic. These must all use the same coordinate system:

- request value
- request period
- measurement
- range limits
- tolerances
- planner output setpoint
- `Plant` target

For a ticks-based turret, all values are ticks. For an inches-based extension, all values are inches. For a servo plant, all values are servo units. Calibration and unit conversion happen in robot/mechanism code before the request reaches the planner.

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

That is fine for many beginner cases. Reach for the planner when the target is selected, constrained, periodic, or contextual.

## Exact request: lift preset

```java
ScalarSetpointPlanner liftPlanner = ScalarSetpoints.plan()
        .request(clock -> ScalarSetpointRequest.exact("basket", BASKET_TICKS))
        .measurement(liftTicks)
        .range(ScalarRange.bounded(LOWEST_TICKS, HIGHEST_TICKS))
        .atSetpointTolerance(20.0)
        .requestSatisfiedTolerance(20.0)
        .build();

ScalarSetpointStatus s = liftPlanner.get(clock);
if (s.hasSetpoint()) {
    liftPlant.setTarget(s.getSetpoint());
}
liftPlant.update(clock);
```

## Periodic request: free spinner

A free spinner has no cable limits. If one full revolution is `9000` ticks, the same alignment repeats every `9000` ticks.

```java
ScalarSetpointPlanner spinnerPlanner = ScalarSetpoints.plan()
        .request(clock -> ScalarSetpointRequest.periodic("align-mark", alignTicks, 9000.0))
        .measurement(spinnerTicks)
        .range(ScalarRange.unbounded())
        .candidatePreference(ScalarSetpointPlanner.CandidatePreference.NEAREST_TO_MEASUREMENT)
        .build();
```

## One-of candidates: tray with colored artifacts

A tray has three slots. Some slots may contain purple artifacts. The robot-specific tray service decides which slots are purple; the scalar planner chooses the nearest feasible slot alignment.

```java
Source<ScalarSetpointRequest> purpleToOutput = clock -> {
    ArrayList<ScalarSetpointCandidate> candidates = new ArrayList<>();

    for (int slot = 0; slot < 3; slot++) {
        if (inventory.colorAt(slot) == ArtifactColor.PURPLE) {
            double alignTicks = trayCalibration.alignSlotToOutputTicks(slot);
            candidates.add(ScalarSetpointCandidate.periodic(
                    "slot-" + slot + "-purple",
                    alignTicks,
                    trayCalibration.ticksPerRevolution()
            ));
        }
    }

    return candidates.isEmpty()
            ? ScalarSetpointRequest.none("no purple artifact")
            : ScalarSetpointRequest.oneOf(candidates);
};

ScalarSetpointPlanner trayPlanner = ScalarSetpoints.plan()
        .request(purpleToOutput)
        .measurement(trayCalibration.positionTicksSource())
        .range(ScalarRange.unbounded())
        .candidatePreference(ScalarSetpointPlanner.CandidatePreference.NEAREST_TO_MEASUREMENT)
        .atSetpointTolerance(20.0)
        .build();
```

The planner does not know what “purple” means. It only sees native-unit candidates.

## Spatial-derived request: turret with camera and localization fallback

A turret can use one `SpatialQuery` with two lanes:

1. live AprilTags from the turret-mounted camera
2. absolute pose / localization fallback

Then the scalar builder maps the selected facing solution into native turret ticks.

```java
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
            .mapFacingToRequest((facing, clock) -> {
                double targetTicks = turretCalibration.zeroTicks()
                        + turretCalibration.facingAngleRadToTicks(facing.facingErrorRad());

                return ScalarSetpointRequest.periodic(
                        facing.sourceId(),
                        targetTicks,
                        turretCalibration.ticksPerTurn(),
                        facing.quality(),
                        facing.ageSec(),
                        facing.timestampSec()
                );
            })
            .doneRequest()
        .measurement(turretCalibration.positionTicksSource())
        .range(turretCalibration.travelRangeTicksSource())
        .candidatePreference(ScalarSetpointPlanner.CandidatePreference.NEAREST_TO_MEASUREMENT)
        .lossPolicy(ScalarSetpointPlanner.LossPolicy.HOLD_LAST_SETPOINT)
        .atSetpointTolerance(25.0)
        .requestSatisfiedTolerance(35.0)
        .build();
```

Loop usage stays short:

```java
ScalarSetpointStatus s = turretPlanner.get(clock);

if (autoAimEnabled.getAsBoolean(clock) && s.hasSetpoint()) {
    turretPlant.setTarget(s.getSetpoint());
}

turretPlant.update(clock);
```

`atSetpoint()` and `requestSatisfied()` are intentionally different. If cable limits force the setpoint to clamp, the plant may be at the clamped setpoint while the original request is not truly satisfied.

## Translation-derived request: extension toward a pickup point

`requestFromSpatial().translateTo(...)` is for cases where field geometry should become a native scalar target through robot-specific kinematics.

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
            .mapTranslationToRequest((translation, clock) -> {
                double extensionTicks = pickupKinematics.extensionTicksFor(
                        translation.solution.frameForwardInches(),
                        translation.solution.frameLeftInches()
                );
                return ScalarSetpointRequest.exact(
                        "pickup-extension",
                        extensionTicks,
                        translation.quality(),
                        translation.ageSec(),
                        translation.timestampSec()
                );
            })
            .doneRequest()
        .measurement(extensionTicks)
        .range(extensionTravelRange)
        .build();
```

The framework solves the spatial relationship. Robot-specific kinematics map that relationship into plant units.

## Calibration belongs next to the mechanism

The scalar planner assumes the mechanism coordinate is already meaningful. A turret service might track:

- raw encoder ticks
- zero ticks for the turret tool frame
- ticks per full turret turn
- homed left/right cable limits
- whether travel range is valid yet

Before homing, publish `ScalarRange.invalid("turret not homed")`. After homing, publish `ScalarRange.bounded(leftTicks, rightTicks)`.

The planner should not decide when zero is trustworthy. That is a robot/mechanism service responsibility.

## Parallel with Drive Guidance

Both builders can consume the same spatial vocabulary:

```java
DriveGuidancePlan drivePlan = DriveGuidance.plan()
        .faceTo(SpatialTargets.point(scoringPoint))
        .controlFrames(SpatialControlFrames.robotCenter().withFacingFrame(robotToShooterFrame))
        .solveWith(solveSet)
        .fixedAprilTagLayout(tagLayout)
        .build();
```

```java
ScalarSetpointPlanner turretPlanner = ScalarSetpoints.plan()
        .requestFromSpatial()
            .faceTo(SpatialTargets.point(scoringPoint))
            .controlFrames(SpatialControlFrames.robotCenter().withFacingFrame(robotToTurretFrame))
            .solveWith(solveSet)
            .fixedAprilTagLayout(tagLayout)
            .mapFacingToRequest(turretNativeUnitMapper)
            .doneRequest()
        .measurement(turretTicks)
        .range(turretRange)
        .build();
```

The principled difference is the output boundary: Drive Guidance knows how to turn a facing error into drivetrain omega. A scalar mechanism must map spatial geometry into its own native plant units.
