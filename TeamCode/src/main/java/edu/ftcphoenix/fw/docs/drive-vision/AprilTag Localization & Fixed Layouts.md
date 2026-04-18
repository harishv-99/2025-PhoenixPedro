# AprilTag Localization & Fixed Layouts

This guide explains Phoenix's AprilTag-localization policy, the difference between detector libraries and trusted field layouts, and how the framework now splits localization into **absolute pose estimators**, **motion predictors**, and **corrected/global estimators**.

The short version:

- **`AprilTagLibrary`** tells a detector which tags exist and how large they are.
- **`TagLayout`** tells Phoenix which tag IDs are trusted as fixed field landmarks.
- **`AprilTagVisionLane`** owns the FTC-side AprilTag rig and exposes a shared `AprilTagSensor`.
- **`AbsolutePoseEstimator`** answers "where is the robot on the field?"
- **`MotionPredictor`** answers both "where is the robot now?" and "how did it move since the last loop?"
- **`CorrectedPoseEstimator`** combines a motion predictor with one absolute correction source.

That split matters because a camera can be shared by localization, alignment, and future vision jobs while the localization stack remains free to choose whether it trusts a raw AprilTag solve, a direct smart-camera pose, or some future absolute field-anchor signal.

---

## 1. Detectable tags vs trusted field-fixed tags

These are not the same thing.

### 1.1 Detectable tags

The FTC SDK `AprilTagLibrary` is detector metadata:

- tag IDs
- tag sizes
- FTC-provided field metadata when available

Phoenix does **not** assume every tag in that library is safe for global localization. Some FTC seasons include tags that are useful for identification, scoring logic, or driver assists even though their exact placement is not deterministic enough to trust as a fixed field landmark.

### 1.2 Fixed field tags

A Phoenix `TagLayout` is the framework contract for **field-fixed** tags only.

Anything that promotes AprilTag observations into a field pose should use a `TagLayout`, not the raw FTC detector library. In practice that includes:

- `AprilTagPoseEstimator`
- guidance paths that temporarily solve a field pose from tags
- corrected/global localization lanes
- calibration tools that depend on field-fixed tags

For official FTC games, the intended entrypoint is:

```java
TagLayout fixedLayout = FtcGameTagLayout.currentGameFieldFixed();
```

That keeps the season-specific "which tags are really fixed?" decision in one framework-owned place.

If you intentionally work with an archived or custom library, use:

```java
FtcGameTagLayout.officialGameFieldFixed(library);
FtcGameTagLayout.fromLibraryFixedIds(library, ids);
FtcGameTagLayout.fromLibraryAllTags(library);   // escape hatch only
```

Use `fromLibraryAllTags(...)` only when you already know every tag in that library is truly fixed in the environment you are solving against.

---

## 2. Localization roles: absolute pose vs motion prediction

Phoenix now formalizes three distinct localization roles.

### 2.1 `AbsolutePoseEstimator`

An `AbsolutePoseEstimator` outputs an absolute robot pose in field coordinates.

Examples:

- `AprilTagPoseEstimator` — solves `field -> robot` from raw AprilTag observations and a trusted `TagLayout`
- `LimelightFieldPoseEstimator` — uses Limelight's direct full-field pose output as an absolute pose source
- future line/tape, landmark, beacon, or wall-anchor localizers — if they can directly answer "where is the robot on the field?"

This is the interface most consumers want:

- drive guidance
- go-to-pose tasks
- targeting
- telemetry

### 2.2 `MotionPredictor`

A `MotionPredictor` is the predictor side of localization.

It still exposes a current absolute pose estimate, but it also exposes a timestamped `MotionDelta` describing how the robot moved since the last update. That lets corrected/global estimators replay motion through delayed measurements without reverse-engineering deltas from two unrelated absolute samples.

Current example:

- `PinpointOdometryPredictor`

Future examples could include:

- a wheel + IMU dead-reckoner
- a drivetrain-state propagator
- another odometry computer that naturally outputs incremental motion

### 2.3 `CorrectedPoseEstimator`

A `CorrectedPoseEstimator` combines:

- one `MotionPredictor`
- one `AbsolutePoseEstimator` used as the absolute correction source

Phoenix currently ships two implementations:

- `OdometryCorrectionFusionEstimator` — simpler gain-based corrected localizer
- `OdometryCorrectionEkfEstimator` — optional covariance-aware corrected localizer

Both expose the same high-level contract, so robot code and tools can swap between them intentionally.

---

## 3. Vision-lane ownership

At the FTC boundary, the important seam is:

- `AprilTagVisionLane`

Standard FTC-boundary implementations are:

- `FtcWebcamAprilTagVisionLane`
- `FtcLimelightAprilTagVisionLane`

Both expose the same shared resources above the FTC boundary:

```java
AprilTagSensor tags = visionLane.tagSensor();
CameraMountConfig mount = visionLane.cameraMountConfig();
```

What changes is only how raw AprilTag observations are acquired:

- `FtcWebcamAprilTagVisionLane` uses a `WebcamName` plus FTC VisionPortal / FTC AprilTag processing.
- `FtcLimelightAprilTagVisionLane` uses a `Limelight3A`, switches to the configured AprilTag pipeline, starts polling, and adapts Limelight fiducial results into the same `AprilTagSensor` seam. Limelight also exposes direct device field pose and orientation-update APIs through the FTC SDK, which Phoenix can optionally consume through a separate absolute-pose estimator path.

The important policy is: **the camera lane owns device lifecycle and raw observations; localization owns estimation strategy.**

---

## 4. The standard corrected-localization lane

`FtcOdometryAprilTagLocalizationLane` is the standard FTC-boundary owner for the common "predictor + tags + corrected/global pose" stack.

It owns:

- one `PinpointOdometryPredictor`
- one raw `AprilTagPoseEstimator`
- optionally one `LimelightFieldPoseEstimator`
- one selected absolute correction source
- one corrected/global estimator (`OdometryCorrectionFusionEstimator` or `OdometryCorrectionEkfEstimator`)

That means one lane can expose all of these views at once:

- predictor pose
- raw AprilTag pose
- optional direct Limelight field pose
- the currently active correction estimator
- the corrected/global pose

This is exactly why the framework does **not** need a new fusion class for every sensor combination. Instead, it composes a few primitive roles cleanly.

---

## 5. Common usage patterns

### 5.1 Webcam + raw AprilTag correction

This is the most common baseline.

```java
FtcWebcamAprilTagVisionLane.Config camCfg = FtcWebcamAprilTagVisionLane.Config.defaults();
camCfg.webcamName = "Webcam 1";
camCfg.cameraMount = solvedCameraMount;

AprilTagVisionLane vision = new FtcWebcamAprilTagVisionLane(hardwareMap, camCfg);

FtcOdometryAprilTagLocalizationLane.Config locCfg =
        FtcOdometryAprilTagLocalizationLane.Config.defaults();
locCfg.predictor.hardwareMapName = "pinPoint";
locCfg.correctionSource.mode = FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
locCfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;

FtcOdometryAprilTagLocalizationLane localization =
        new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                FtcGameTagLayout.currentGameFieldFixed(),
                locCfg
        );
```

This gives you:

- Pinpoint-based motion prediction
- raw AprilTag field solves from the webcam
- corrected/global localization using the raw AprilTag solve as the absolute correction source

### 5.2 Limelight + raw AprilTag correction

If you want Limelight to behave like a smart AprilTag camera but keep Phoenix's own raw-tag pose solve as the correction source:

```java
FtcLimelightAprilTagVisionLane.Config llCfg = FtcLimelightAprilTagVisionLane.Config.defaults();
llCfg.hardwareName = "limelight";
llCfg.pipelineIndex = 0;
llCfg.pollRateHz = 100;
llCfg.cameraMount = solvedCameraMount;

AprilTagVisionLane vision = new FtcLimelightAprilTagVisionLane(hardwareMap, llCfg);

FtcOdometryAprilTagLocalizationLane.Config locCfg =
        FtcOdometryAprilTagLocalizationLane.Config.defaults();
locCfg.predictor.hardwareMapName = "pinPoint";
locCfg.correctionSource.mode = FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
locCfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
```

Everything above `AprilTagVisionLane` still consumes the same `AprilTagSensor` seam.

### 5.3 Limelight + direct field-pose correction

If you want corrected/global localization to trust the Limelight's direct full-field pose instead of Phoenix's raw-tag solve:

```java
FtcOdometryAprilTagLocalizationLane.Config locCfg =
        FtcOdometryAprilTagLocalizationLane.Config.defaults();
locCfg.predictor.hardwareMapName = "pinPoint";
locCfg.correctionSource.mode = FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.LIMELIGHT_FIELD_POSE;
locCfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
locCfg.correctionSource.limelightFieldPose.mode =
        LimelightFieldPoseEstimator.Config.Mode.BOTPOSE_MT2;
locCfg.correctionSource.limelightFieldPose.maxResultAgeSec = 0.20;
locCfg.correctionSource.limelightFieldPose.minVisibleTags = 2;
locCfg.correctionSource.limelightFieldPose.degradeWhenMoving = true;
```

This gives you two absolute pose views side by side:

- `localization.aprilTagPoseEstimator()` — Phoenix's raw-tag field solve
- `localization.limelightFieldPoseEstimator()` — Limelight's direct device field pose

and the corrected/global estimator will use the configured correction source.

### 5.4 Which one should I start with?

Recommended order:

1. Start with **webcam or Limelight raw AprilTag correction** (`APRILTAG_POSE`).
2. Verify camera mount, tag policy, and predictor quality.
3. Then try **direct Limelight field pose** (`LIMELIGHT_FIELD_POSE`) only after the raw-tag path already makes sense.
4. Compare corrected/global behavior, especially while the robot is moving.

That makes it easier to separate "camera rig / field map / mount is wrong" from "direct device pose is noisier than expected in motion."

---

## 6. Raw AprilTag solving policy

Phoenix's shared AprilTag solver does this:

- gather visible observations whose IDs are in the trusted `TagLayout`
- compute one candidate `field -> robot` pose per visible fixed tag
- weight closer / more centered tags more strongly
- prefer observation-provided field pose when it agrees with Phoenix's explicit geometry and remains plausible
- choose a consensus seed
- reject outliers
- compute one fused field pose and a quality score

That shared policy is used by:

- `AprilTagPoseEstimator`
- guidance paths that temporarily solve a field pose from tags

That keeps guidance and localization from drifting into subtly different AprilTag policies.

### Sharing solver tuning cleanly

`AprilTagPoseEstimator.Config` extends the shared fixed-tag solver config and adds camera-specific fields.

When another component should reuse the same weighting / outlier / plausibility policy, use `toSolverConfig()`:

```java
AprilTagPoseEstimator.Config tagCfg = profile.localization.aprilTags
        .toAprilTagPoseEstimatorConfig(profile.vision.activeCameraMount());

AprilTagPoseEstimator tagLocalizer = new AprilTagPoseEstimator(tags, fixedLayout, tagCfg);

DriveGuidancePlan plan = DriveGuidance.plan()
        .resolveWith()
            .adaptive()
            .localization(correctedLocalizer)
            .aprilTags(tags, profile.vision.activeCameraMount(), 0.50)
            .fixedAprilTagLayout(fixedLayout)
            .aprilTagFieldPoseConfig(tagCfg.toSolverConfig())
            .doneResolveWith()
        .build();
```

---

## 7. Corrected/global localization and latency compensation

When you combine a `MotionPredictor` with an absolute correction source, Phoenix's corrected estimators do two important reliability jobs:

- deduplicate repeated absolute measurements by measurement timestamp
- when history is available, apply the correction at the measurement timestamp and replay predictor motion forward to the current loop

That is more trustworthy than repeatedly blending the same delayed frame against "now".

Typical fusion setup:

```java
OdometryCorrectionFusionEstimator.Config fusionCfg =
        OdometryCorrectionFusionEstimator.Config.defaults();
fusionCfg.maxCorrectionAgeSec = 0.35;
fusionCfg.predictorHistorySec = 1.0;  // should cover maxCorrectionAgeSec when latency compensation is enabled

OdometryCorrectionFusionEstimator corrected =
        new OdometryCorrectionFusionEstimator(predictor, absoluteCorrection, fusionCfg);
```

Typical EKF setup:

```java
OdometryCorrectionEkfEstimator.Config ekfCfg =
        OdometryCorrectionEkfEstimator.Config.defaults();
ekfCfg.maxCorrectionAgeSec = 0.35;
ekfCfg.predictorHistorySec = 1.0;

CorrectedPoseEstimator corrected =
        new OdometryCorrectionEkfEstimator(predictor, absoluteCorrection, ekfCfg);
```

Notes:

- `predictorHistorySec` should be at least `maxCorrectionAgeSec` when latency compensation is enabled.
- corrected estimators now consume an explicit `MotionDelta` from the predictor instead of reverse-engineering motion from two unrelated pose snapshots.
- if the corrected pose is pushed back into the predictor, the estimator also rebases its predictor baseline/history so the next predictor delta does not re-apply that jump.

---

## 8. Direct Limelight field pose in motion

Some teams report that direct device field pose can degrade while the robot is moving.

Potential causes include:

- motion blur
- rolling-shutter / capture delay effects
- stale frames
- single-tag geometry sensitivity
- misconfigured camera mount or field map
- aggressive robot rotation between capture time and robot-loop time

Phoenix's direct Limelight field-pose estimator is intentionally conservative:

- freshness gating (`maxResultAgeSec`)
- visible-tag gating (`minVisibleTags`)
- optional motion-aware quality degradation (`degradeWhenMoving`)
- optional hard reject thresholds (`rejectWhenMovingTooFast`)
- optional predictor yaw feed for MegaTag2-style direct pose modes when the active SDK supports it. Limelight's FTC API exposes both direct botpose access and a robot-orientation update hook for the MT2 path.

If direct Limelight field pose is unstable while moving:

1. tighten `maxResultAgeSec`
2. require `minVisibleTags >= 2`
3. keep `degradeWhenMoving = true`
4. compare against the raw AprilTag solve in the tester
5. fall back to `APRILTAG_POSE` as the correction source until the direct path is proven trustworthy

One important ownership note: Phoenix currently assumes the Limelight device is already configured with a field map that matches the trusted `TagLayout` you intend to use. Keep those aligned whenever you use direct device field pose.

---

## 9. Future signals beyond AprilTags

The current localization model already leaves room for other pose signals.

Examples:

- field tape / field-line tracking
- walls or fixed landmarks
- overhead fiducials
- vision-detected beacons
- operator-placed anchors during setup

The rule of thumb is simple:

- if the signal directly answers "where is the robot on the field?" it should probably implement `AbsolutePoseEstimator`
- if the signal is primarily incremental motion (wheel encoders, IMU-integrated yaw, dead-reckoning) it should probably implement `MotionPredictor` or feed one

That is why the framework does **not** need a bespoke fusion class for every sensor combination. It needs a few principled roles and then clear composition.

---

## 10. Testing checklist

When AprilTag-based global localization feels wrong, work down this list:

1. camera mount solved and non-identity
2. predictor/pod offsets calibrated
3. trusted `TagLayout` matches the field you are actually on
4. raw selected-tag observations look sane in the tester
5. raw `AprilTagPoseEstimator` solves look sane before trusting the corrected/global estimator
6. `predictorHistorySec` is large enough for accepted correction age when latency compensation is enabled
7. direct Limelight field pose, if enabled, stays reasonable while the robot is moving
8. corrected/global telemetry shows accepted corrections rather than repeated rejection or duplicate-frame skipping

The intended tester progression is:

- `Loc: AprilTag Localization` first
- then `Loc: Pinpoint + Field Corrections` once camera mount and predictor calibration are trustworthy
