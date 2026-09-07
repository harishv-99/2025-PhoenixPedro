---
tags:
  - Reference
---

# Sensing, localization, and vision quick reference

## Ordinary entry points

| Need | API |
|---|---|
| adapt FTC devices into typed sources | [`FtcSensors`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcSensors.html>) |
| retain a timestamped robot-relative target | [`TargetObservation2d`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/TargetObservation2d.html>) |
| retain one bounded frame, including a confirmed empty scene | [`TargetObservations2d`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/TargetObservations2d.html>) |
| select a located target by geometry | [`TargetSelections`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/TargetSelections.html>) |
| add capture-time field coordinates | [`ObservationSources`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/ObservationSources.html>) |
| consume absolute pose evidence | [`AbsolutePoseEstimator`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/AbsolutePoseEstimator.html>) |
| retain time-addressable planar pose history | [`PlanarPoseHistory`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/PlanarPoseHistory.html>) |
| fuse motion and correction observations | [`CorrectedPoseEstimator`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/fusion/CorrectedPoseEstimator.html>) |
| own one FTC webcam/VisionPortal lifecycle | [`FtcWebcamVisionLane`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcWebcamVisionLane.html>) |
| own one explicitly switched Limelight lifecycle | [`FtcLimelightVisionLane`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcLimelightVisionLane.html>) |
| borrow tag sensing without camera-close authority | [`AprilTagVision`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/AprilTagVision.html>) |
| configure floor-object interpretation on either camera | [`FtcFloorObjectVision.Config`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcFloorObjectVision.Config.html>) |

## Remember

Sample once per cycle when stateful, preserve the capture timestamp, keep mutable vendor values at
the boundary, and separate a measurement from the robot policy that interprets it. A pose estimate
is evidence with a frame and time—not a claim of perfect field position.

Read [Locate a vision target](<../drive-vision/Vision Targets.md>), [FTC Sensors](<../ftc-boundary/FTC Sensors.md>), and
[AprilTag Localization and Fixed Layouts](<../drive-vision/AprilTag Localization & Fixed Layouts.md>).
