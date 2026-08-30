# Timestamped adaptive collection

**Audience:** teams that already have a managed Pedro robot and need one vision-selected collection
attempt. Start with the [Pedro autonomous reference](<Pedro Autonomous Reference.md>) if route
lifecycle and exact route outcomes are new.

This optional case study teaches how to turn one delayed Limelight detector frame into one bounded,
explainable collection attempt. Its adaptive vision, path, and attempt roles are robot-example code;
their timestamp alignment deliberately demonstrates the optional framework `PlanarPoseHistory`
capability. The case study is not a new Phoenix framework lane, a complete autonomous cycle, or a
drop-in OpMode.

## What the example proves

The example follows one chain of evidence:

```text
confirmed camera frame + its capture timestamp
        -> immutable angle snapshot
        -> exact or safely interpolated planar pose-history lookup
        -> capture-time floor projection
        -> selected collection band or typed unavailable reason
        -> one selected or explicit-fallback Pedro route
        -> route + cancellation-safe intake
        -> exact exit reason
        -> optional live-pose return
```

Three example-owned roles keep that chain visible:

| Role | Owns | Does not own |
|---|---|---|
| [`AdaptiveCollectionVisionService`](<../../../robots/examples/pedro/adaptive/AdaptiveCollectionVisionService.java>) | The Limelight resource, immediate SDK-value copying, capture-time history lookup, floor projection, deterministic band selection, and one cached immutable decision. | Localization or history heartbeat/reset, route geometry, inventory meaning, or intake hardware. |
| [`AdaptiveCollectionPaths`](<../../../robots/examples/pedro/adaptive/AdaptiveCollectionPaths.java>) | Phoenix-to-Pedro target conversion, live Pedro start-pose snapshots, selected/fallback/return geometry, and two native semantic callbacks. | Camera interpretation, route execution policy, inventory, or follower lifecycle. |
| [`AdaptiveCollectionAttempt`](<../../../robots/examples/pedro/adaptive/AdaptiveCollectionAttempt.java>) | One fresh Task graph, milestone/inventory exit policy, exact collection and return status, and conditional return. | A scheduler, repeated cycles, match-time policy, parking, or mechanism realization. |

An adopting robot should reuse its existing intake capability and semantic inventory source. The
case study uses `BasicPedroAutoMechanism` only to keep the companion Task concrete and
cancellation-safe.

## Declare the history owner before vision

The existing Pedro/localization service remains the one heartbeat and lifecycle owner. Give it one
[`PlanarPoseHistory`](<../../localization/PlanarPoseHistory.java>) over the authoritative high-rate
trajectory estimator. At START it resets the history, applies the starting pose, updates
localization, and records the resulting current sample. Each active LOOP updates localization and
immediately records before downstream services. At STOP it stops its owned resources and resets the
concrete history. Do not register the history as a second `RobotProgram.Service`, and do not ask the
vision service to update or reset it.

The relevant addition inside that existing service is deliberately small:

```java
// Constructed once by the composition root and retained by the existing localization service.
PlanarPoseHistory.Config poseHistoryConfig = PlanarPoseHistory.Config.defaults();
PlanarPoseHistory poseHistory = new PlanarPoseHistory(
        runtime.motionPredictor(), poseHistoryConfig);

// Existing localization service START:
poseHistory.reset();
applyStartingPose.run();
localization.update(clock);
poseHistory.recordCurrent(clock);
autoDrive.update(clock);

// Existing localization service LOOP:
localization.update(clock);
poseHistory.recordCurrent(clock);
autoDrive.update(clock);

// Existing localization service STOP, after stopping its owned resources:
poseHistory.reset();
```

`retentionSec` must retain the earlier bracket endpoint for the oldest frame this consumer accepts,
while `maxSamples` remains the independent hard memory bound. The interpolation time, translation,
and yaw limits should reject a bracket that the adopting robot cannot treat as credible motion
evidence. These values are robot configuration, not Limelight or route policy.

Then declare vision after that existing service and give it only the stable read-only lookup
projection. Set `pathsConfig.fieldTransform` to the same immutable transform used when the adopting
robot configured its Pedro runtime; the current defaults both select `decodeInvertedFtc()`:

```java
// The adopting composition root has already registered its Pedro/localization service.
AdaptiveCollectionVisionService vision = program.service(
        new AdaptiveCollectionVisionService(
                hardwareMap,
                poseHistory.lookupSource(),
                visionConfig));

AdaptiveCollectionPaths paths = new AdaptiveCollectionPaths(runtime, pathsConfig);
AdaptiveCollectionAttempt attempt = new AdaptiveCollectionAttempt(
        vision,
        paths,
        inventoryFull,
        intake,
        attemptConfig);

program.rootTask(attempt.task());
program.presenter((clock, telemetry) -> present(telemetry, attempt.status()));
```

`RobotProgram` starts the already-declared localization service before vision, updates services in
that same order, cancels the root Task before stopping resources, and closes services in reverse
declaration order. `task()` returns the attempt's one stable single-use root. Construct a fresh
`AdaptiveCollectionAttempt` for another attempt.

The four mutable `Config` values are authoring drafts. Each long-lived owner snapshots the fields
it retains. `defaults()` supplies finite software examples only; it does not claim that a pipeline,
camera mount, field target, callback point, timeout, or route is physically correct.

## Why the frame timestamp matters

A Limelight result can arrive after the camera exposure that produced it. The service therefore
retains the frame's existing `LoopTimestamp`; it never substitutes the loop time at which robot
code read the result. A frame is unavailable when that timestamp is absent, belongs to a prior
clock reset, is materially in the future, or exceeds `maxFrameAgeSec`. Supplying a timestamp from a
different `LoopClock` is a wiring error rather than an unavailable observation.

FTC detector objects remain inside the robot/FTC edge. The service immediately copies only the two
principal-pixel angles needed by this case. Limelight reports horizontal positive right and
vertical positive down, so the Phoenix camera-frame ray is:

```text
(forward, left, up) = (1, -tan(horizontalAngle), -tan(verticalAngle))
```

The configured robot-to-camera pose and the history's field-to-robot pose at exposure time rotate
and translate that ray. Because the maintained history is planar, the service deliberately lifts
its `Pose2d` into a `Pose3d` with z, pitch, and roll equal to zero before composing the calibrated
3D camera mount. That one configured mount must remain fixed relative to the robot; an articulated
or moving camera needs its own timestamp-aware mount transform and is outside this example service.
The selector rejects non-finite values, parallel/upward rays, non-positive floor intersections, and
points outside the inclusive collection box. It then considers fixed-width Y bands, choosing the
band containing the most projected points and breaking a tie by the lower band start. Vendor list
order therefore cannot change the result.

The immutable `Decision` reports the frame timestamp and age, detector/projectable/in-box counts,
and either guarded selected-band accessors or one guarded `UnavailableReason`. If calculation
reached the pose query, `hasPoseLookup()` is true and `poseLookup()` returns that exact immutable
lookup, including its exact/interpolated kind or typed unavailable reason. Frame-level rejection
can happen before a query, so `poseLookup()` is guarded separately. No selected target is
represented by `-1`, `NaN`, `null`, or another value that could accidentally be clamped into a
valid route. A custom history projection that returns a lookup for any timestamp other than the
requested exposure is a wiring error; pass the stable `PlanarPoseHistory.lookupSource()` projection.
`AdaptiveCollectionAttempt.Status` retains that exact `Decision`, so a presenter can
report the same lookup kind or unavailable reason without resampling history.

## History-backed moving projection

`PlanarPoseHistory` records the authoritative estimator's final as-published pose once per managed
cycle. It preserves exact endpoints and otherwise interpolates field X/Y, shortest-path yaw, and
minimum endpoint quality only when the bracket remains within every configured bound. A later
correction does not rewrite earlier samples: this is an as-published trajectory, not a retrospective
smoother.

The vision service queries that history with the frame's original `LoopTimestamp`. It accepts exact
and interpolated results, so remaining stationary between exposure and selection is no longer a
precondition. It never extrapolates, clamps, selects the nearest pose, or substitutes the current
pose. Empty, evicted, reset/discontinuous, excessive-time, excessive-translation,
excessive-yaw, and after-latest outcomes therefore select the explicit fallback route and remain
inspectable through the retained lookup.

The history's read-only `lookupSource()` cannot clear its owner; its `reset()` is deliberately a
no-op. Every starting-pose application and coordinate rebase must pass through the authoritative
trajectory estimator so its segment identity changes, and the existing localization service must
own the concrete history reset at START and STOP. An out-of-band rebase or incorrect service order
breaks that ownership contract; vision cannot infer or repair it.

## One route decision, built once

When the attempt starts, it freezes exactly one cached `Decision`. The collection
`RouteTask` uses `RouteTasks.followBuiltAtStart(...)`, so its supplier reads one live Pedro start
pose and builds one path exactly once:

- a selected decision uses the configured collection X and heading plus the selected band center Y;
- an unavailable decision uses `fallbackFieldToRobotPose` explicitly.

The path publishes `safeToLeave` and `nearEnd` through native Pedro callbacks. Those names are
robot meaning. The attempt never treats raw parametric progress as general framework policy, and a
callback never starts or cancels another route.

The collection route and bounded intake Task are companions of one `Tasks.waitUntil(...)` deadline.
On every deadline sample, exit precedence is:

1. the exact collection route has reached a terminal status;
2. `nearEnd` is latched;
3. `safeToLeave` is latched and the cached inventory source reports full.

Consequently, fullness before the safe point does not exit, and the safe point alone does not
exit. When a semantic condition wins, the attempt records `NEAR_END` or
`INVENTORY_FULL_AFTER_SAFE` before `Tasks.parallelDeadline(...)` cancels the exact route and intake
companions. The retained collection route may then report `CANCELLED`; that does not erase the
intentional exit reason that caused the cancellation.

## Exact outcomes and conditional return

`Status` keeps the frozen decision, semantic milestones, cached inventory fact, collection exit,
and collection/return route statuses separate. Follower timeout/stall, Task timeout, interruption,
replacement, cancellation, failure, and unknown terminal state remain distinct.

Only these collection exits permit the return route:

- `ROUTE_COMPLETED`
- `NEAR_END`
- `INVENTORY_FULL_AFTER_SAFE`

The return is another timed start-built Route Task. It samples the live Pedro pose only after the
collection phase has finished cancelling its companions. An abnormal route ending, construction
failure, or direct outer cancellation does not manufacture a return.

The aggregate root `TaskOutcome` and the route facts answer different questions. A successful
policy/no-op branch can leave the root successful even when an exact route status explains why no
return ran. Present `Status.exitReason()`, `collectionRouteStatus()`, and
`returnRouteStatus()` when strategy needs that truth; never replace them with the adapter's latest
route status after a newer route starts.

## What software tests cannot establish

The hardware-free tests prove copying, frame timestamp gates, exact and interpolated history
consumption, retained typed history gaps, geometry rejection, deterministic selection, fallback
reachability, exactly-once construction, milestone precedence, cancellation identity, intake
cleanup, outcome retention, and conditional return. They do not establish:

- the selected Limelight detector pipeline or its behavior on the installed firmware;
- camera height, orientation, rigidity, or principal-angle signs on the physical setup;
- estimator accuracy, camera/loop time synchronization, or floor-projection accuracy;
- collision-free paths or correct callback placement;
- sensor timing, pickup reliability, cycle time, or match benefit; or
- physical drivetrain, intake, and camera cleanup on STOP.

Validate those facts on the adopting robot with conservative motion, clear space, and an operator
ready to stop it. Repetition, attempt-count/match-time limits, recovery, and parking remain outer
Auto strategy rather than hidden behavior in this one-attempt example.

## Related reading

- [Pedro autonomous reference](<Pedro Autonomous Reference.md>)
- [Spatial queries and timestamp-aware frames](<../drive-vision/Spatial Queries.md>)
- [Pedro integration contract](<../../integrations/pedro/README.md>)
- [Tasks and Macros](<../design/Tasks & Macros Quickstart.md>)
- [Loop Structure](<../core-concepts/Loop Structure.md>)
