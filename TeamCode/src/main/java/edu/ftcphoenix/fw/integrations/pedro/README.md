# Pedro integration

This folder is Phoenix's explicit boundary for Pedro Pathing. Core framework packages stay free of
`com.pedropathing.*`; reusable adapter, field-transform, and runtime ownership code lives here;
robot-specific paths and strategies remain under the owning robot package. Phoenix production paths
live under `robots/phoenix/autonomous/pedro/`; the independent beginner reference lives under
`robots/examples/pedro/`.

## Small compiling Auto reference

Start with [`Pedro Autonomous Reference`](<../../docs/examples/Pedro Autonomous Reference.md>) when
you need the complete lifecycle without the Phoenix season graph. Its five robot-code files show one
fixed practice path, a Plant-backed capability Task, explicit success/timeout/cancellation policy,
one recurring follower heartbeat, deterministic stop, and a thin FTC OpMode.

Four reference files are independent of Phoenix season code. The fifth,
`PhoenixBasicPedroAutoExample`, is a disabled host that uses this repository's real Phoenix profile
only to compile-check the physical construction and FTC INIT/START/loop/STOP boundary. A new robot
replaces that host with its own verified runtime and mechanism wiring and adapts the concrete
capability/routine/root types to its own mechanisms; it must not copy the Phoenix hardware profile.
The guide counts all five robot-code files and documents the exact edit points, portability
boundaries, and hardware checks that compilation cannot perform.

## Production ownership

Phoenix Pedro Auto builds one `PedroPathingRuntime` through the project-specific constants factory:

```java
PedroPathingRuntime runtime =
        Constants.createPhoenixAutoRuntime(hardwareMap, selectedProfile);

robot.initAuto(runtime.driveAdapter(), runtime.motionPredictor());
runtime.setStartingPose(paths.pedroStartPose);
```

Successful runtime construction proves only this integration graph's configuration and ownership
contracts. It does not decide whether robot-specific calibration acknowledgements, field facts, or
route geometry are ready for a match; keep that combined arming policy in the robot mode client.

The runtime contains exactly one of each production owner:

- one profile-configured `PinpointOdometryPredictor`, which alone acquires, configures, resets,
  polls, and rebases Pinpoint;
- one passive Pedro `Localizer`, which converts and consumes the predictor's immutable current-cycle
  pose/velocity/physical-heading snapshot without polling hardware;
- one valid native Pedro mecanum drivetrain and `Follower`;
- one `PedroPathingDriveAdapter`, which owns the Follower heartbeat, per-route terminal
  classification, and final drivetrain stop.

Build production paths with `runtime.pathBuilder()`. Pedro 2.1.2's no-argument
`Follower.pathBuilder()` reads a process-wide mutable constraint default; the runtime instead passes
an independent copy of its validated `PathConstraints` to every builder. The pinned Pedro
`PathBuilder.build()` also replaces explicit per-path constraints with that global default while
constructing a `PathChain`; the runtime contains this vendor defect and restores both its checked
defaults and any deliberate per-path overrides. Robot path code keeps Pedro's normal fluent builder
API and does not need a repair step.

Build fixed geometry eagerly during the mode client's pre-start construction and pass it to the
ordinary `RouteTasks.follow(...)` path.
When a return or fallback must begin at the follower's live pose, keep that decision in the
robot-owned path factory and pass a quick lambda to `RouteTasks.followBuiltAtStart(...)`. The lambda
runs exactly once when its Route Task starts, after the composition root's current-cycle
localization and Pedro heartbeat. It may use the runtime's supported read-only Follower inspection,
but it must still build through `runtime.pathBuilder()` and must not call raw Follower lifecycle
methods.

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

Every `follow(pathChain)` returns a backend-neutral `RouteExecution` for that exact run. During the
owned heartbeat, the adapter retains the expected path identity and classifies the transition while
Pedro's progress and endpoint evidence still exists. It reports `COMPLETED` only when the original
final path reaches its parametric end and its endpoint velocity, translation, and heading
constraints pass. A stuck segment or follower endpoint timeout reports
`FOLLOWER_TIMEOUT_OR_STALL`; adapter/callback stop or manual takeover reports `INTERRUPTED`; a
supported later follow reports `REPLACED`; and a detectable unexplained terminal transition reports
`UNKNOWN_TERMINAL` rather than guessed success. In particular, a non-parametric segment advance
stops the whole route as a stall instead of silently skipping ahead. Raw lifecycle calls remain
unsupported because they can erase the evidence needed for truthful classification.

`RouteTask` adds the Task-owned `TASK_TIMEOUT` and `CANCELLED` distinctions and exposes the final
backend-neutral value through `getRouteStatus()`. Its cancellation uses that one execution handle,
so late cleanup of an older Task cannot stop a replacement route.

That status is a fact, not a recovery decision. A robot routine must retain the exact `RouteTask`,
gate position-dependent scoring on its completed status, and explicitly choose continue, fallback,
or abort for every other result. Phoenix's conservative routine bounds its complete outbound-plus-
scoring pre-park policy with `Tasks.withTimeout(...)`, then permits one live-pose park after normal
completion, a local follower/Task timeout, or successful match-time cleanup. It aborts on
interruption, replacement, cancellation, failure, unknown termination, direct root cancellation,
or failed cleanup. The park is outside the timer, so starting it early cannot cause a second park at
the threshold. Cleanup stays in the robot capability layer; the Pedro adapter neither chooses game
strategy nor writes mechanism state.

Keep `RouteTask`'s local timeout when its exact `TASK_TIMEOUT` fact matters. A generic outer timeout
uses ordinary child cancellation and therefore leaves the route child `CANCELLED` while the wrapper
reports `TIMEOUT`; it cannot substitute for route-specific terminal classification.

`getLatestRouteStatus()` provides the newest backend-neutral value for Driver Station telemetry.
Code making a decision about one particular route should retain that route's `RouteTask` or
`RouteExecution` instead of consulting the mutable latest-route view.

Raw Follower lifecycle calls are unsupported in production Phoenix Auto. Do not call
`followPath(...)`, `breakFollowing()`, `startTeleopDrive()`, `update()`, or pose-reset methods on the
Follower from robot code; those calls bypass heartbeat ownership or the per-execution status.
Build routes through `runtime.pathBuilder()`, set the initial pose through the runtime, command
routes/guidance through the adapter, and use adapter/robot stop. Read-only Follower inspection at
the integration boundary remains valid.

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

The robot mode client may apply the same declared start during INIT and reapply it at the exact FTC
START boundary, provided neither call occurs after the first heartbeat. This is a software
coordinate rebase and publication check, not an independent observation of physical field
placement. Show the expected physical start with an explicit Pedro-field label to the drive team,
or use a separately trustworthy field-absolute observation if an automatic placement check is
required.

## Configuration and tuning tools

The project factory derives motor names/directions and Pinpoint name/offsets/resolution/directions/
yaw scalar from the selected defensive `PhoenixProfile` snapshot. `Constants` retains Pedro-only
follower/controller/drivetrain/path tuning. Invalid names, duplicate motors, unsupported reset
assumptions, and non-finite tuning fail during pre-start construction with a setting-specific
message.
Human acknowledgements such as verified pod directions or calibrated offsets remain robot-owned
readiness facts; numeric construction validation must not silently claim that those procedures were
performed.

Pedro's generated `Tuning` menu and standalone `PedroTest` use
`Constants.createToolOnlyNativeFollower(hardwareMap)`. That clearly named tool path uses Pedro's
native Pinpoint lifecycle because those vendor tools call only `Follower.update()`. Never pass the
tool-only follower to Phoenix Auto; it is not a second production option.

## Related docs

- [`../../docs/examples/Pedro Autonomous Reference.md`](<../../docs/examples/Pedro Autonomous Reference.md>)
- [`../../docs/core-concepts/Loop Structure.md`](<../../docs/core-concepts/Loop Structure.md>)
- [`../../docs/design/Recommended Robot Design.md`](<../../docs/design/Recommended Robot Design.md>)
- [`../../docs/drive-vision/AprilTag Localization & Fixed Layouts.md`](<../../docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
