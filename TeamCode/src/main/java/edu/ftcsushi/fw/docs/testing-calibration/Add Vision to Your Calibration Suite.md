---
tags:
  - Test & Tune
---

# Add vision to your calibration suite

**Learning mode:** Integrated example

**Outcome:** add camera-mount, AprilTag-only, and corrected-localization checks using your robot's
authored camera and field facts. Each selected tester owns a fresh camera; none borrows the live
production camera.

**Before this page:** [Add calibration testers to your robot](<Add Calibration Testers to Your Robot.md>).
Keep its profile, fresh-factory, suite, and thin-host pattern. This optional extension needs no
powered drivetrain. Reading needs no camera; physical checks require the installed camera and
the [camera/localization runbook](<Robot Calibration Tutorials.md#camera-mount>).

## Choose one camera backend

A **backend** is the device-specific implementation behind a shared capability. A USB webcam uses
FTC camera processing; a Limelight supplies its own results. The robot still chooses a concrete
device and configuration—it does not need a second version of each calibration algorithm.

[`CalibrationRobotProfile.CameraBackend`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.CameraBackend.html>)
is an enum, a fixed list of named Java choices: `NONE`, `WEBCAM`, or `LIMELIGHT`. The checked-in
`cameraBackend` is `NONE`. Change that one canonical choice to the installed backend before building
the suite. The example constructs no camera branch while it remains `NONE`; calling a camera-only
factory without a selected backend is an error, not a request for a hidden default camera.

An **AprilTag** is a printed visual identifier whose observed geometry can help estimate position.
A **camera mount** records the lens position and orientation relative to the robot. A **field
layout** records trusted landmarks' positions; the next section explains their coordinate frames.

A Limelight **pipeline** is a processing setup on that device. Selecting pipeline `0` in Java does
not create or configure its AprilTag detector. Two timing facts matter: **receipt staleness** is
time since the Control Hub received a result; **estimated capture age** also includes the camera's
reported capture and processing delay. Reading telemetry again does not make the image newer.
The corresponding profile mapping supplies these software defaults:

| Backend | Authored mapping | Active example values |
| --- | --- | --- |
| Webcam | `webcam()` | FTC name `Webcam 1`; `640 × 480`; AprilTag processing enabled |
| Limelight | `limelight()` | FTC name `limelight`; pipeline `0`; polling request `100 Hz`; maximum Hub receipt staleness `0.25 s`; AprilTag capability enabled |
| Both | `cameraMount` and `fixedTagLayout()` | One authored camera mount and one explicit fixed-field layout |

Confirm that the physical device's selected pipeline
produces the required AprilTag results. A `100 Hz` polling request is not proof of that many fresh
camera frames each second. Hardware names, resolution and pipeline choices must match your setup.

The two profile recipes answer the same questions with different device-specific Config types.
Only the selected recipe is called; enabling AprilTag processing is explicit in each one:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.java -->
```java
public FtcWebcamVisionLane.Config webcam() {
    FtcWebcamVisionLane.Config cfg = FtcWebcamVisionLane.Config.defaults();
    cfg.webcamName = webcamName;
    cfg.cameraMount = cameraMount;
    cfg.aprilTags = FtcWebcamVisionLane.AprilTagConfig.defaults();
    return cfg;
}
```

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.java -->
```java
public FtcLimelightVisionLane.Config limelight() {
    FtcLimelightVisionLane.Config cfg = FtcLimelightVisionLane.Config.defaults();
    cfg.hardwareName = limelightName;
    cfg.cameraMount = cameraMount;
    cfg.pipelineIndex = limelightPipelineIndex;
    cfg.pollRateHz = limelightPollRateHz;
    cfg.maxResultAgeSec = limelightMaxResultAgeSec;
    cfg.aprilTags = FtcLimelightVisionLane.AprilTagConfig.defaults();
    cfg.aprilTags.pipelineIndex = limelightPipelineIndex;
    return cfg;
}
```

[`FtcWebcamVisionLane.Config`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcWebcamVisionLane.Config.html>)
and [`FtcLimelightVisionLane.Config`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcLimelightVisionLane.Config.html>)
are data. The assignments use the canonical fields `webcamName`, `limelightName`,
`limelightPipelineIndex`, `limelightPollRateHz`, and `limelightMaxResultAgeSec` shown above.

## Keep mount and field facts explicit

A **pose** contains position and facing direction. A **frame** states the origin and directions
used for its numbers. The camera's image measurements become useful field coordinates only when
the software also knows where the camera sits on the robot and where trusted tags sit on the field.

[`CameraMountConfig`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/vision/CameraMountConfig.html>)
describes the lens relative to the robot's center of rotation on the floor: X forward, Y left,
Z up, all in inches. Its `ofDegrees(...)` convenience uses yaw about Z, pitch about Y and roll about
X in degrees; the stored angles are radians. Follow the
[mount measurement procedure](<Robot Calibration Tutorials.md#camera-mount>) for the actual values.

The profile starts with `cameraMount = CameraMountConfig.identity()`: zero displacement and zero
rotation. That is a placeholder, not a measured mount. It is suitable as an unknown to solve in the
mount calibrator; it does not establish a trustworthy AprilTag-derived robot pose. Record an
accepted result in that same `cameraMount` field, rebuild, and use the configured localization
check. The separate `cameraMountAccepted` acknowledgement starts `false`; only a reviewed physical
result justifies changing it, and the assisted powered extension additionally checks it.
Set `cameraMountAccepted` back to `false` and re-review after changing the backend, physical camera,
or mount. Acceptance of the old setup does not transfer automatically to its replacement.

A detector library describes recognizable tags and
their sizes. A [`TagLayout`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/field/TagLayout.html>)
instead records the field positions of tags trusted to stay fixed. Recognizing a tag does not make
it a field landmark.

The example's `fixedTagLayout()` explicitly chooses
[`FtcGameTagLayout.currentGameFieldFixed()`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcGameTagLayout.html#currentGameFieldFixed()>).
This is a software choice of field facts, not a claim that your practice field or detector library
matches them. For a different field, change the canonical layout recipe to its independently
established fixed tags. Use [AprilTag practice setup](<../drive-vision/AprilTag Practice Setup.md>)
for a deliberate practice layout; do not mark a movable tag as fixed merely to obtain a pose.

## Pass a camera recipe, not an open camera

The selected tester may let the operator choose a hardware name. It therefore receives a function
that turns the selected name into an
[`AprilTagCameraFactory`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/AprilTagCameraFactory.html>)—a
recipe that opens a fresh camera owner later. The Java spelling
`Function<String, AprilTagCameraFactory>` means “take a text name and return a camera factory.”

[`AprilTagCameraFactories.webcam(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/AprilTagCameraFactories.html#webcam(edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane.Config)>)
and [`limelight(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/AprilTagCameraFactories.html#limelight(edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane.Config)>)
capture backend configuration; they do not open hardware at menu registration. The example builds
the chosen template from copied profile facts before saving the selected-name lambda. Each invocation copies it and
changes only the device name. A picker retry must not reread a mutable robot profile or retain an
old open lane.

Inside `visionFactoryBuilder(captured)`, the webcam branch is:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
if (captured.cameraBackend == CalibrationRobotProfile.CameraBackend.WEBCAM) {
    FtcWebcamVisionLane.Config template = captured.webcam();
    return selectedName -> {
        FtcWebcamVisionLane.Config cfg = template.copy();
        cfg.webcamName = selectedName;
        return AprilTagCameraFactories.webcam(cfg);
    };
}
```

The remaining selected backend uses the parallel Limelight recipe:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
FtcLimelightVisionLane.Config template = captured.limelight();
return selectedName -> {
    FtcLimelightVisionLane.Config cfg = template.copy();
    cfg.hardwareName = selectedName;
    return AprilTagCameraFactories.limelight(cfg);
};
```

`selectedName -> { ... }` saves a function whose input is the picker's text name. The braces hold
its later work, and `return` supplies the new factory. This changes the software name only: choosing
another physical camera does not prove it has the same measured mount or detector setup.

That shared recipe serves the configured checks below. The selected tester opens, updates and
closes its own camera owner. The other menu entries have no live camera, and a production OpMode
must not own that device simultaneously. If you deliberately provide a custom SDK tag library,
keep that borrowed library stable for the selected owner's complete lifetime, including retries.

## Add the three configured checks

[`CalibrationTesters`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.html>)
uses the same authored mount, field facts and backend recipe in these factories:

| Factory | Selected owner | What the check can establish |
| --- | --- | --- |
| `cameraMount(profile)` | [`CameraMountCalibrator`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/calibration/CameraMountCalibrator.html>) | A candidate mount from known robot placement and fixed-tag observations |
| `aprilTagLocalization(profile)` | [`AprilTagLocalizationTester`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/localization/AprilTagLocalizationTester.html>) | Detection evidence and the robot field pose computed using the rebuilt mount/layout |
| `correctedLocalization(profile)` | [`PinpointAprilTagCorrectedLocalizationTester`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/localization/PinpointAprilTagCorrectedLocalizationTester.html>) | Pinpoint motion plus accepted independent AprilTag pose corrections |

**Correction** means using an independent field observation to adjust the movement-based estimate.
The profile's `localization()` maps its own `pinpoint()` and `aprilTags()` settings into
[`FtcOdometryAprilTagLocalizationLane.Config`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/localization/FtcOdometryAprilTagLocalizationLane.Config.html>).
The example uses `APRILTAG_POSE` correction and the ordinary `FUSION` estimator for both backends.
It does not switch to Limelight's direct field-pose output or require the optional EKF comparison.

This complete AprilTag-only factory shows the configuration-to-owner connection:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
public static AprilTagLocalizationTester aprilTagLocalization(CalibrationRobotProfile profile) {
    CalibrationRobotProfile captured = captureVision(profile);
    AprilTagLocalizationTester.Config cfg = AprilTagLocalizationTester.Config.defaults();
    cfg.preferredVisionDeviceName = visionDeviceName(captured);
    cfg.visionDeviceType = visionDeviceType(captured);
    cfg.visionPickerTitle = "Select " + captured.cameraBackend;
    cfg.fixedTagLayout = captured.fixedTagLayout();
    cfg.aprilTags = captured.aprilTags();
    return new AprilTagLocalizationTester(cfg, visionFactoryBuilder(captured));
}
```

`captureVision` copies the profile and rejects `NONE` before constructing a camera Config.
`visionDeviceName` selects `webcamName` or `limelightName`; `visionDeviceType` selects `WebcamName.class`
or `Limelight3A.class`. A `.class` value describes the SDK device type the picker should enumerate;
it is not a constructed device. The mount calibrator uses the same picker/layout/builder mapping
and `cfg.maxDetectionAgeSec = captured.maxDetectionAgeSec`, without requiring an already-accepted
mount. The corrected-localization factory maps `cfg.localization = captured.localization()` into
its own tester Config and passes the same selected-backend builder.

That localization mapping preserves the ordinary correction choices explicitly:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.java -->
```java
public FtcOdometryAprilTagLocalizationLane.Config localization() {
    FtcOdometryAprilTagLocalizationLane.Config cfg =
            FtcOdometryAprilTagLocalizationLane.Config.defaults();
    cfg.predictor = pinpoint();
    cfg.estimation.aprilTags = aprilTags();
    cfg.estimation.correctedEstimatorMode =
            FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
    cfg.estimation.correctionSource.mode =
            FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
    return cfg;
}
```

The menu's vision branch registers these factories only for the selected non-`NONE` backend.
Use the same `CalibrationTesters.create(profile)` host connection from the basic lesson; no new
OpMode loop, clock, production robot, or motor is needed.

The selected vision submenu saves the same three factories as fresh recipes:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
TesterSuite suite = new TesterSuite().setTitle("Vision checks: " + captured.cameraBackend);
suite.add("Measure camera mount", "Independent known robot pose; record the answer",
        () -> cameraMount(captured));
suite.add("AprilTag localization", "Inspect the configured mount; acceptance is a human check",
        () -> aprilTagLocalization(captured));
suite.add("Pinpoint + AprilTag fusion", "Shared profile; no powered drive",
        () -> correctedLocalization(captured));
return suite;
```

## Keep age and agreement separate

The profile's `limelightMaxResultAgeSec = 0.25` maps to the backend's `maxResultAgeSec`: at most
`0.25 s` of **Control Hub receipt staleness** may confirm pipeline readiness. This asks whether a
result has arrived recently enough; it does not measure how old the image was when it arrived.

The shared `maxDetectionAgeSec = 0.35` instead limits **estimated capture age** for tag solving.
For Limelight, the adapter adds the reported capture and targeting-processing delays to receipt
staleness, then anchors that estimate in a retained frame timestamp on the shared loop clock.
For example, `0.20 s` since receipt plus `0.20 s` of those delays gives an estimated `0.40 s` image
age: it can pass the receipt-staleness check and still be too old for the solver.

These limits measure different starting moments; they are not two cutoffs on the same age.
Increasing the solver's capture-age allowance cannot recover a result unavailable because the
backend's pipeline-readiness check failed.

The AprilTag solver also checks agreement between candidate tag solves: its example outlier gates
are `18.0 in` of position difference and `25.0°` of heading difference from its consensus estimate.
An **outlier** is a candidate that disagrees too much with that estimate. Those software thresholds
are not robot accuracy guarantees or recommended driving tolerances. Change them in the canonical
`aprilTags()` mapping only after an evidence-backed decision.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.java -->
```java
public AprilTagLocalizationConfig aprilTags() {
    AprilTagLocalizationConfig cfg = AprilTagLocalizationConfig.defaults();
    cfg.maxDetectionAgeSec = maxDetectionAgeSec;
    cfg.fieldPoseSolver.outlierPositionGateInches = tagOutlierPositionGateInches;
    cfg.fieldPoseSolver.outlierHeadingGateRad = tagOutlierHeadingGateRad;
    return cfg;
}
```

[`AprilTagLocalizationConfig`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/localization/FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.html>)
keeps these shared age/solver facts separate from the camera mount and the camera's lifecycle.

Availability, age and quality are separate observations: a pose can exist and have a high quality
score yet be too old for an action. The
[localization reference](<../drive-vision/AprilTag Localization & Fixed Layouts.md>) owns the full
estimator and delayed-evidence contracts; this lesson only connects the configured owners.

## Verify the handoff

1. Select the actual backend and names; verify its detector setup and fixed-field facts.
2. Run the configured camera-mount check with known robot placement. Record and review the result.
3. Edit the canonical `cameraMount`, rebuild, and open a fresh configured AprilTag-localization
   check. An old suite cannot reload changed facts.
4. Compare its field pose at independently known placements. Only after the mount and Pinpoint
   facts are credible should you run the corrected-localization check.

The existing [runbook](<Robot Calibration Tutorials.md>) owns the physical controls, observations
and stop gates. A working detector or non-identity mount alone does not prove correct robot pose.

These are the same independent source files as the basic lesson:

- [Complete source: `CalibrationRobotProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.java>) — canonical backend, mount, field and solver facts.
- [Complete source: `CalibrationTesters.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java>) — snapshots, selected-name recipes and fresh configured checks.

Use the [basic lesson's software check](<Add Calibration Testers to Your Robot.md#inspect-the-maintained-software-example>)
to verify the maintained mappings. Its software evidence cannot establish camera identity,
pipeline readiness, measured mount, field placement or physical accuracy.

Stop here if you only need unpowered vision/localization checks. For an explicitly reviewed
drivetrain, [Enable powered calibration](<Enable Powered Calibration.md>) adds the separate motion
permission and optional assisted-pod gate.
