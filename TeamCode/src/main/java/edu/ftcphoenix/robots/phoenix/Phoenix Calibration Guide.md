# Phoenix calibration guide

This is the Phoenix-specific bring-up path.

Use it when you want the exact Phoenix menu names and the exact `PhoenixProfile` fields to edit while bringing up a fresh robot.

---

## 1. Ownership map

Phoenix keeps calibration ownership intentionally clean:

- `PhoenixProfile.drive` -> drivetrain hardware/wiring/brake tuning
- `PhoenixProfile.vision` -> active vision backend plus backend-specific rig config
- `PhoenixProfile.localization` -> predictor, AprilTag-localization tuning, correction-source choice, and corrected-global estimator tuning
- `PhoenixProfile.field` -> shared field facts such as the fixed AprilTag layout
- `PhoenixCapabilities` -> shared mode-neutral robot API used by TeleOp and Auto
- `PhoenixTeleOpControls` -> TeleOp stick/button semantics
- `ShooterSupervisor` -> scoring policy and requests
- `ScoringTargeting` -> selected-tag policy and aim status
- `Shooter` -> mechanism actuation and status
- `PhoenixRobot` -> composition root and loop owner

That ownership split matters during bring-up because fixes should land in the owner of the behavior:

- drivetrain wiring / brake / drive tuning -> `PhoenixProfile.drive`
- backend choice + active camera rig -> `PhoenixProfile.vision`
- webcam name / VisionPortal-backed rig config -> `PhoenixProfile.vision.webcam`
- Limelight hardware name / pipeline / poll rate -> `PhoenixProfile.vision.limelight`
- predictor, AprilTag solve, and corrected-global localization tuning -> `PhoenixProfile.localization`
- field-fixed tag policy or practice-field overrides -> `PhoenixProfile.field`

---

## 2. Where to start in the tester menu

Recommended starting point:

- `Guide: Phoenix Calibration Walkthrough`

That walkthrough links to the real testers in the recommended order.

If you already know what you need, browse instead through:

- `Phoenix: Hardware Bring-up`
- `Phoenix: Calibration & Localization`

---

## 3. Step-by-step bring-up

### Step 1: drivetrain motor direction

Menu entry:

- `HW: Drivetrain Motor Direction`

Goal:

- confirm each wheel would drive the robot forward when the tester says it should

Fix in code:

```java
PhoenixProfile.current().drive.wiring
```

---

### Step 2: camera mount

Menu entry:

- `Calib: Camera Mount (Robot)`

Goal:

- solve the active AprilTag vision device pose relative to the robot

Paste result into:

```java
PhoenixProfile.current().vision.webcam.cameraMount    // when backend = WEBCAM
PhoenixProfile.current().vision.limelight.cameraMount // when backend = LIMELIGHT
```

The tester prints both `CameraMountConfig.of(...)` and `CameraMountConfig.ofDegrees(...)`. Paste one of those directly into the active backend config.

### How webcam and Limelight are used so far

Above the FTC boundary, Phoenix uses both backends almost the same way:

```java
AprilTagVisionLane vision = PhoenixVisionFactory.create(hardwareMap, PhoenixProfile.current().vision);

FtcOdometryAprilTagLocalizationLane localization =
        new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                PhoenixProfile.current().field.fixedAprilTagLayout,
                PhoenixProfile.current().localization
        );
```

The backend only changes which concrete AprilTag lane is created:

- `Backend.WEBCAM` -> `FtcWebcamAprilTagVisionLane`
- `Backend.LIMELIGHT` -> `FtcLimelightAprilTagVisionLane`

Phoenix can now also use Limelight's direct device field pose as an **optional** correction source through `PhoenixProfile.localization.correctionSource`, but the raw AprilTag path remains available either way. Limelight's FTC SDK exposes both fiducial-result access and direct botpose / MT2 pose access.

### Phoenix notes

- the preferred AprilTag device name is `PhoenixProfile.current().vision.activeDeviceName()`
- the walkthrough status turns `OK` once the active backend's camera mount no longer looks like the identity placeholder

---

### Step 3: AprilTag-only localization sanity check

Menu entry:

- `Loc: AprilTag Localization (Robot)`

Goal:

- verify that Phoenix's active AprilTag backend, fixed-tag layout policy, and raw AprilTag field solve produce a believable field pose before odometry is fused in

What to watch:

- fresh detections
- correct selected tag ID
- stable `fieldToRobot` pose
- low sample jitter while stationary

This tester reuses:

```java
PhoenixProfile.current().vision.activeDeviceName()
PhoenixProfile.current().vision.activeCameraMount()
PhoenixProfile.current().localization.aprilTags
PhoenixProfile.current().field.fixedAprilTagLayout
```

So the practice tool should match production AprilTag-solving math closely.

---

### Step 4: Pinpoint axis directions

Menu entry:

- `Calib: Pinpoint Axis Check (Robot)`

Goal:

- verify +X forward, +Y left, heading CCW+

Fix in code:

```java
PhoenixProfile.current().localization.predictor
```

Specifically, correct pod direction fields before continuing.

Record completion after rerunning the tester and accepting the result:

```java
PhoenixProfile.current().calibration.pinpointAxesVerified = true;
```

---

### Step 5: Pinpoint pod offsets

Menu entry:

- `Calib: Pinpoint Pod Offsets (Robot)`

Goal:

- estimate the Pinpoint offsets that remove fake translation during rotation

Paste result into:

```java
PhoenixProfile.current().localization.predictor
```

The tester prints the recommended:

```java
.withOffsets(forwardPodOffsetLeftInches, strafePodOffsetForwardInches)
```

Record completion after copying the numbers and rerunning once to confirm they are stable:

```java
PhoenixProfile.current().calibration.pinpointPodOffsetsCalibrated = true;
```

Phoenix enables AprilTag assist for this tester automatically once the active backend's camera mount looks solved enough to trust.

---

### Step 6: default corrected-global localization validation

Menu entry:

- `Loc: Pinpoint + Field Corrections (Robot)`

Goal:

- validate Phoenix's default corrected-global localizer in the conditions that matter for real operation:
  - tags visible at the start
  - movement while corrections are available
  - temporary tag loss near the target
  - smooth continuity while relying on prediction alone
  - clean correction when tags come back

Config involved:

```java
PhoenixProfile.current().vision
PhoenixProfile.current().localization.predictor
PhoenixProfile.current().localization.aprilTags
PhoenixProfile.current().localization.correctionSource
PhoenixProfile.current().localization.correctionFusion
PhoenixProfile.current().field.fixedAprilTagLayout
```

Phoenix defaults to:

- predictor = Pinpoint odometry
- absolute correction source = raw AprilTag field solve (`APRILTAG_POSE`)
- corrected estimator = gain-based fusion (`FUSION`)

### Trying direct Limelight field pose

If you want the corrected/global estimator to use Limelight's direct device field pose instead of the raw AprilTag field solve:

```java
PhoenixProfile.current().vision.backend = PhoenixProfile.VisionConfig.Backend.LIMELIGHT;
PhoenixProfile.current().localization.correctionSource.mode =
        FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.LIMELIGHT_FIELD_POSE;
PhoenixProfile.current().localization.correctionSource.limelightFieldPose.mode =
        LimelightFieldPoseEstimator.Config.Mode.BOTPOSE_MT2;
```

Start conservatively:

- keep the raw AprilTag tester available as your sanity check
- tighten `maxResultAgeSec`
- require at least 2 visible tags if direct pose looks noisy in motion
- keep motion-aware degradation enabled

Phoenix's direct Limelight path currently assumes the Limelight field map already matches the field/tag layout you intend to use.

---

### Step 7: optional EKF comparison

Menu entry:

- `Loc: Pinpoint + Field Corrections EKF (Robot)`

Goal:

- compare the optional covariance-aware corrected localizer against the default fusion path

Do this only after:

- camera mount is calibrated
- Pinpoint axis directions are verified
- Pinpoint pod offsets are measured
- the default corrected-global tester already looks trustworthy

Config involved:

```java
PhoenixProfile.current().localization.correctionEkf
PhoenixProfile.current().localization.correctedEstimatorMode
```

Use the tester to compare behavior first. Only then consider changing the robot's default corrected estimator mode.

---

## 4. The localization model Phoenix is using

Phoenix now treats localization as three different roles:

- `MotionPredictor` -> short-term motion propagation (`PinpointOdometryPredictor`)
- `AbsolutePoseEstimator` -> field-anchored absolute pose (`AprilTagPoseEstimator`, optional `LimelightFieldPoseEstimator`)
- `CorrectedPoseEstimator` -> combines a predictor with one absolute correction source (`OdometryCorrectionFusionEstimator` or `OdometryCorrectionEkfEstimator`)

That split is important because it gives Phoenix a clean extension path later.

Examples of future additions that would fit this model naturally:

- field tape / line tracking that can directly anchor robot pose -> another `AbsolutePoseEstimator`
- wheel + IMU dead-reckoning -> another `MotionPredictor`
- smarter absolute-source selection policies -> a higher-level `AbsolutePoseEstimator` wrapper

---

## 5. Quick checklist for a fresh Phoenix robot

1. drivetrain direction
2. camera mount
3. raw AprilTag localization check
4. Pinpoint axis directions
5. Pinpoint pod offsets
6. default corrected-global localization validation
7. optional EKF comparison

---

## 6. Related docs

- [`Framework Overview`](<../fw/docs/getting-started/Framework Overview.md>)
- [`AprilTag Localization & Fixed Layouts`](<../fw/docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`Framework Lanes & Robot Controls`](<../fw/docs/design/Framework Lanes & Robot Controls.md>)
- [`Robot Calibration Tutorials`](<../fw/docs/testing-calibration/Robot Calibration Tutorials.md>)
- [`Guided Calibration Walkthroughs`](<../fw/docs/testing-calibration/Guided Calibration Walkthroughs.md>)
