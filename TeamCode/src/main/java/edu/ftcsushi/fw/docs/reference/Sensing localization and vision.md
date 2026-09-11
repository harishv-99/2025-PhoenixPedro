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
| choose a reusable geometric ranking rule | [`TargetSelectionPolicies`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/TargetSelectionPolicies.html>) |
| retain bounded recent anonymous field locations | [`FieldTargetMemory`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/FieldTargetMemory.html>) |
| rank remembered field locations near a point or current robot pose | [`FieldTargetSelectionPolicies`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/FieldTargetSelectionPolicies.html>) |
| select or hold a genuine tag ID from observed or inferred candidates | [`TagSelections`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/vision/apriltag/TagSelections.html>) |
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

The ordinary object path is `TargetSelections.fromVisibleObjects(source).freshWithinSec(age)`
followed by `choose(TargetSelectionPolicies.nearestToRobot())`. The final answer returns a
`TargetSelectionSource` directly. Tag selection additionally chooses its lifetime with
`continuous()`, `holdWhile(enabled)`, or another explicit hold terminal. Anonymous frame objects
do not gain tag-like identity merely because their selection policies use the same grammar.

For mixed-age field memory, construct one
`FieldTargetMemory.fromFieldObjects(source).retainingForSec(age).matchingWithinInches(radius).maxEntries(count)`
owner and call `update(clock)` after capture-time field projection is ready. Select with
`TargetSelections.fromRecentFieldLocations(memory.source()).choose(policy)`; selection inherits
retention unless `freshWithinSec(...)` requests a stricter bound. Only
`References.selectedFieldTargetPoint(...)` describes that selection to the spatial layer, using
`absolutePose(...)` rather than camera-only evidence. Explicit reset fences and terminal STOP
belong to the memory owner; selectors and queries borrow its view. See
[Remember recently seen ball locations](<../examples/Remember Recent Field Locations.md>) for the
complete read-only owner and its software-only limits.

Read [Locate a vision target](<../drive-vision/Vision Targets.md>), [FTC Sensors](<../ftc-boundary/FTC Sensors.md>), and
[AprilTag Localization and Fixed Layouts](<../drive-vision/AprilTag Localization & Fixed Layouts.md>).
