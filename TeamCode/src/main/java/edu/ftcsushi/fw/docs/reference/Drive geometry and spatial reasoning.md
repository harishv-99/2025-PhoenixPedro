---
tags:
  - Reference
---

# Drive, geometry, and spatial reasoning quick reference

## Ordinary entry points

| Need | API |
|---|---|
| publish robot-centric axial, lateral, and turn intent | [`DriveSignal`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/DriveSignal.html>) |
| produce drive intent from controls or policy | [`DriveSource`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/DriveSource.html>) |
| build the ordinary FTC mecanum sink | [`FtcDrives`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcDrives.html>) |
| compose selected drive corrections | [`DriveOverlays`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/DriveOverlays.html>) |
| ask a frame-explicit spatial question | [`SpatialQuery`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/SpatialQuery.html>) |
| name robot, field, and other reference frames | [`References`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/References.html>) |
| compute a robot-center destination from a located target | [`ApproachResult2d`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/ApproachResult2d.html>) |
| solve delayed robot-at-capture target feedback | [`SpatialSolveSet.builder().observedPoints()`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/SpatialSolveSet.html>) |

## Remember

Sushi uses robot `+X` forward, `+Y` left, `+Z` up; positive yaw and omega are counter-clockwise;
distances are inches and angles are radians. A DriveSource produces intent and one sink performs
the final write.

Read [Drive Guidance](<../drive-vision/Drive Guidance.md>),
[Spatial Queries](<../drive-vision/Spatial Queries.md>), and
[Field-relative Drive](<../examples/Field-relative Drive.md>).
