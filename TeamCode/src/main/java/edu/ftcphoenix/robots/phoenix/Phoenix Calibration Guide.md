# Phoenix calibration guide

This is the Phoenix-specific version of the framework calibration path.

Use it when you are bringing up a fresh Phoenix robot and want the exact menu names and `PhoenixProfile` fields to touch.

## Architecture notes for the stage-1 refactor

The stage-1 cleanup intentionally split a few responsibilities that used to live together:

- `PhoenixProfile` owns robot configuration instead of the old `RobotConfig` static bag.
- `PhoenixRobot` is the composition root and loop owner.
- `PhoenixTeleOpBindings` owns button-edge and toggle semantics.
- `ShooterSupervisor` owns scoring policy and intent-level commands.
- `PhoenixTelemetryPresenter` owns driver-facing telemetry formatting.

That split matters during bring-up because tester fixes and robot fixes should usually go to the owner of the behavior, not to an unrelated helper.

## Where to start in the tester menu

Open:

- `Guide: Phoenix Calibration Walkthrough`

That walkthrough intentionally duplicates links to the real testers so students can follow one recommended path.

If you already know what you need, browse instead through:

- `Phoenix: Hardware Bring-up`
- `Phoenix: Calibration & Localization`

## Step 1: drivetrain motor direction

### Menu entry

- `HW: Drivetrain Motor Direction`

### Goal

Confirm each wheel would drive the robot forward when the tester says it should.

### Fix in code

Use the Phoenix drivetrain motor wiring/config, not a tester workaround, to correct any reversed motor.

### Tester implementation note

`DrivetrainMotorDirectionTester` is a `BaseTeleOpTester`, so telemetry should go through `ctx.telemetry` or the base helpers (`telemHeader`, `telemHint`, `telemUpdate`) rather than an OpMode field named `telemetry`.

## Step 2: camera mount

### Menu entry

- `Calib: Camera Mount (Robot)`

### Goal

Solve Phoenix's webcam pose relative to the robot.

### Paste result into

```java
PhoenixProfile.current().vision.cameraMount
```

The tester prints `CameraMountConfig.of(...)` and `CameraMountConfig.ofDegrees(...)`. Paste one of those directly.

### Phoenix notes

- the preferred camera is `PhoenixProfile.current().vision.nameWebcam`
- the walkthrough status turns `OK` once the camera mount no longer looks like the identity placeholder

## Step 3: AprilTag-only localization sanity check

### Menu entry

- `Loc: AprilTag Localization (Robot)`

### Goal

Verify that Phoenix's preferred camera, fixed-tag layout policy, and AprilTag-only solver produce a believable field pose before odometry is fused in.

### What to watch

- fresh detections
- correct selected tag ID
- stable `fieldToRobot` pose
- low sample jitter while stationary

### Phoenix notes

This tester reuses:

```java
PhoenixProfile.current().vision.nameWebcam
PhoenixProfile.current().vision.cameraMount
PhoenixProfile.current().localization.aprilTags
```

So the practice tool should match production localization math more closely.

## Step 4: Pinpoint axis directions

### Menu entry

- `Calib: Pinpoint Axis Check (Robot)`

### Goal

Verify:

- +X is forward
- +Y is left
- heading is CCW-positive

### Fix in code

Adjust:

```java
PhoenixProfile.current().localization.pinpoint
```

Specifically, correct pod direction fields before continuing.

### Record completion

After you have rerun the tester and accepted the result, set:

```java
PhoenixProfile.current().calibration.pinpointAxesVerified = true
```

## Step 5: Pinpoint pod offsets

### Menu entry

- `Calib: Pinpoint Pod Offsets (Robot)`

### Goal

Estimate the Pinpoint offsets that remove fake translation during rotation.

### Paste result into

```java
PhoenixProfile.current().localization.pinpoint
```

The tester prints the recommended:

```java
.withOffsets(forwardPodOffsetLeftInches, strafePodOffsetForwardInches)
```

### Record completion

After copying the numbers and rerunning once to confirm they are stable, set:

```java
PhoenixProfile.current().calibration.pinpointPodOffsetsCalibrated = true
```

### Phoenix notes

Phoenix enables AprilTag assist for this tester automatically once the camera mount looks solved enough to trust.

## Step 6: default global localization validation

### Menu entry

- `Loc: Pinpoint + AprilTag Fusion (Robot)`

### Goal

Validate Phoenix's default global localizer in the conditions that matter for real operation:

- tags visible at the start
- movement while vision is available
- temporary tag loss near the target
- smooth continuity while relying on odometry alone
- clean correction when tags come back

### Config involved

```java
PhoenixProfile.current().localization.pinpoint
PhoenixProfile.current().localization.aprilTags
PhoenixProfile.current().localization.pinpointAprilTagFusion
PhoenixProfile.current().vision.cameraMount
```

## Step 7: optional EKF comparison

### Menu entry

- `Loc: Pinpoint + AprilTag EKF (Robot)`

### Goal

Compare the optional covariance-aware estimator against the default fusion path.

### Do this only after

- camera mount is calibrated
- Pinpoint axis directions are verified
- Pinpoint pod offsets are measured
- the default fusion tester already looks trustworthy

### Config involved

```java
PhoenixProfile.current().localization.pinpointAprilTagEkf
PhoenixProfile.current().localization.globalEstimatorMode
```

Use the tester to compare behavior first. Only then consider changing the robot's default estimator mode.

## Quick checklist for a fresh Phoenix robot

1. drivetrain direction
2. camera mount
3. AprilTag-only localization check
4. Pinpoint axis directions
5. Pinpoint pod offsets
6. default fusion validation
7. optional EKF comparison

## Related framework docs

- [`docs/testing-calibration/Robot Calibration Tutorials.md`](<docs/testing-calibration/Robot Calibration Tutorials.md>)
- [`docs/testing-calibration/Guided Calibration Walkthroughs.md`](<docs/testing-calibration/Guided Calibration Walkthroughs.md>)
