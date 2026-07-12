# Pedro integration

This folder is Phoenix's explicit boundary for Pedro Pathing. Core framework packages stay free of
`com.pedropathing.*`; reusable adapter, field-transform, and runtime ownership code lives here;
robot-specific paths and strategies remain under `robots/phoenix/autonomous/pedro/`.

## Production ownership

Phoenix Pedro Auto builds one `PedroPathingRuntime` through the project-specific constants factory:

```java
PedroPathingRuntime runtime =
        Constants.createPhoenixAutoRuntime(hardwareMap, selectedProfile);

robot.initAuto(runtime.driveAdapter(), runtime.motionPredictor());
runtime.setStartingPose(paths.pedroStartPose);
```

The runtime contains exactly one of each production owner:

- one profile-configured `PinpointOdometryPredictor`, which alone acquires, configures, resets,
  polls, and rebases Pinpoint;
- one passive Pedro `Localizer`, which converts and consumes the predictor's immutable current-cycle
  pose/velocity/physical-heading snapshot without polling hardware;
- one valid native Pedro mecanum drivetrain and `Follower`;
- one `PedroPathingDriveAdapter`, which owns the Follower heartbeat and final drivetrain stop.

Build production paths with `runtime.pathBuilder()`. Pedro 2.1.2's no-argument
`Follower.pathBuilder()` reads a process-wide mutable constraint default; the runtime instead passes
an independent copy of its validated `PathConstraints` to every builder. The pinned Pedro
`PathBuilder.build()` also replaces explicit per-path constraints with that global default while
constructing a `PathChain`; the runtime contains this vendor defect and restores both its checked
defaults and any deliberate per-path overrides. Robot path code keeps Pedro's normal fluent builder
API and does not need a repair step.

`PhoenixRobot` receives only backend-neutral `DriveCommandSink` and `MotionPredictor` seams. It does
not learn Pedro route types or configuration. Auto loop order remains:

```text
Clock -> Phoenix localization/correction -> targeting -> Pedro heartbeat
      -> Auto Tasks -> scoring -> telemetry
```

The adapter prepares the passive localizer with the shared `LoopClock` immediately before its one
vendor heartbeat. The localizer fails closed unless the Pinpoint snapshot belongs to that exact
cycle. Route and guidance Tasks may call the adapter's update hook too; `LoopClock.cycle()`
deduplication makes those later calls no-ops.

## Pose contract

Phoenix Decode explicitly selects `PedroFieldTransform.decodeInvertedFtc()`. It inverse-tests pose
origin, axes, absolute heading, and velocity rotation rather than relying on Pedro 2.1.2's broken
FTC inverse transform. Phoenix values stay in the current FTC field frame, inches, radians, and
CCW-positive yaw; Pedro values are explicitly tagged `PedroCoordinates`.

Apply the route start pose through `PedroPathingRuntime.setStartingPose(...)` before the first
heartbeat. Raw Follower pose/IMU resets are rejected because they bypass Phoenix correction history
and timing. By default, accepted AprilTag corrections are pushed into the shared predictor before
the Pedro heartbeat, so path following and targeting see the same corrected pose in that cycle.
Disabling predictor push remains the explicit targeting-only policy; Phoenix telemetry displays raw
predictor pose, corrected global pose, and their drift.

## Configuration and tuning tools

The project factory derives motor names/directions and Pinpoint name/offsets/resolution/directions/
yaw scalar from the selected defensive `PhoenixProfile` snapshot. `Constants` retains Pedro-only
follower/controller/drivetrain/path tuning. Invalid names, duplicate motors, unsupported reset
assumptions, and non-finite tuning fail during INIT with a setting-specific message.

Pedro's generated `Tuning` menu and standalone `PedroTest` use
`Constants.createToolOnlyNativeFollower(hardwareMap)`. That clearly named tool path uses Pedro's
native Pinpoint lifecycle because those vendor tools call only `Follower.update()`. Never pass the
tool-only follower to Phoenix Auto; it is not a second production option.

## Related docs

- [`../../docs/core-concepts/Loop Structure.md`](<../../docs/core-concepts/Loop Structure.md>)
- [`../../docs/design/Recommended Robot Design.md`](<../../docs/design/Recommended Robot Design.md>)
- [`../../docs/drive-vision/AprilTag Localization & Fixed Layouts.md`](<../../docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
