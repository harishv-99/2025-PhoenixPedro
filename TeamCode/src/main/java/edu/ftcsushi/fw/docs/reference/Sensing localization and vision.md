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
| consume absolute pose evidence | [`AbsolutePoseEstimator`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/AbsolutePoseEstimator.html>) |
| retain time-addressable planar pose history | [`PlanarPoseHistory`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/PlanarPoseHistory.html>) |
| fuse motion and correction observations | [`CorrectedPoseEstimator`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/fusion/CorrectedPoseEstimator.html>) |
| own one FTC webcam/VisionPortal lifecycle | [`FtcWebcamVisionPortalLane`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcWebcamVisionPortalLane.html>) |

## Remember

Sample once per cycle when stateful, preserve the capture timestamp, keep mutable vendor values at
the boundary, and separate a measurement from the robot policy that interprets it. A pose estimate
is evidence with a frame and time—not a claim of perfect field position.

Read [FTC Sensors](<../ftc-boundary/FTC Sensors.md>) and
[AprilTag Localization and Fixed Layouts](<../drive-vision/AprilTag Localization & Fixed Layouts.md>).
