# Phoenix calibration guide

This is the Phoenix-specific version of the framework calibration path.

Use it when you are bringing up a fresh Phoenix robot and want the exact menu names and `PhoenixProfile` fields to edit.

## Architecture notes

Phoenix now splits stable ownership this way:

- `PhoenixProfile.drive` -> `FtcMecanumDriveLane.Config`
- `PhoenixProfile.vision` -> `FtcAprilTagVisionLane.Config`
- `PhoenixProfile.localization` -> `FtcOdometryAprilTagLocalizationLane.Config`
- `PhoenixProfile.field` -> shared field facts such as the fixed AprilTag layout
- `PhoenixTeleOpControls` -> all TeleOp input semantics
- `ShooterSupervisor` -> scoring policy and intent-level requests
- `ScoringTargeting` -> selected-tag policy, cached aim status, and shot suggestions
- `Shooter` -> mechanism actuation and status
- `PhoenixRobot` -> composition root and loop owner

That split matters during bring-up because fixes should land in the owner of the behavior:

- drivetrain wiring / brake / drive tuning -> `PhoenixProfile.drive`
- webcam name / camera mount / vision portal settings -> `PhoenixProfile.vision`
- odometry tuning / AprilTag localization tuning / fusion tuning -> `PhoenixProfile.localization`
- fixed field tags or practice-field overrides -> `PhoenixProfile.field`
- button semantics / manual drive behavior -> `PhoenixTeleOpControls`
- scoring gating / requests / feed policy -> `ShooterSupervisor`
- mechanism actuation -> `Shooter`

## Where to start in the tester menu

Open:

- `Guide: Phoenix Calibration Walkthrough`

That walkthrough intentionally links to the real testers in the recommended order.

If you already know what you need, browse instead through:

- `Phoenix: Hardware Bring-up`
- `Phoenix: Calibration & Localization`

## Step 1: drivetrain motor direction

### Menu entry

- `HW: Drivetrain Motor Direction`

### Goal

Confirm each wheel would drive the robot forward when the tester says it should.

### Fix in code

Use the Phoenix drive-lane wiring config, not a tester workaround, to correct any reversed motor:

```java
PhoenixProfile.current().drive.wiring
```

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

- the preferred camera is `PhoenixProfile.current().vision.webcamName`
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
PhoenixProfile.current().vision.webcamName
PhoenixProfile.current().vision.cameraMount
PhoenixProfile.current().localization.aprilTags
PhoenixProfile.current().field.fixedAprilTagLayout
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
PhoenixProfile.current().localization.odometry
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
PhoenixProfile.current().localization.odometry
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
PhoenixProfile.current().vision.webcamName
PhoenixProfile.current().vision.cameraMount
PhoenixProfile.current().localization.odometry
PhoenixProfile.current().localization.aprilTags
PhoenixProfile.current().localization.odometryTagFusion
PhoenixProfile.current().field.fixedAprilTagLayout
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
PhoenixProfile.current().localization.odometryTagEkf
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

## Related docs

- [`Framework Lanes & Robot Controls`](<../fw/docs/design/Framework Lanes & Robot Controls.md>)
- [`Recommended Robot Design`](<../fw/docs/design/Recommended Robot Design.md>)
- [`Robot Calibration Tutorials`](<../fw/docs/testing-calibration/Robot Calibration Tutorials.md>)
- [`Guided Calibration Walkthroughs`](<../fw/docs/testing-calibration/Guided Calibration Walkthroughs.md>)
