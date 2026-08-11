# Phoenix Architecture

This document describes the production framework-based robot under
`edu.ftcphoenix.robots.phoenix`. It is the complete reference design for applying the framework's
ownership, lifecycle, capability, targeting, and autonomous contracts to one competition robot.

The repository-wide [`Framework Principles`](<../../fw/Framework Principles.md>) remain the primary
design authority. Use the package [`README`](<README.md>) for a shorter map of what to read and
where to make common robot changes.

## Big picture

Phoenix separates five concerns:

```text
mode client
  -> robot capability vocabulary
  -> robot policy/services
  -> mechanism realization
  -> framework/FTC/vendor boundaries
```

- `PhoenixRobot` is the composition root. It constructs and privately retains the complete
  mechanism graph for one mode and declares its managed lifecycle roles.
- `PhoenixCapabilities` is the shared, mode-neutral intent/status vocabulary used by TeleOp and
  Auto.
- `PhoenixTeleOpControls` owns driver/operator meanings.
- `PhoenixAlliance` is the mode-neutral match-alliance fact shared by TeleOp and Auto.
- `PhoenixAutoSpec`, path factories, and routine factories own the remaining autonomous selection
  and strategy.
- `ScoringTargeting` publishes targeting facts and guidance.
- `ScoringPath` owns scoring policy, requests, feed queues, final Plant resolvers, Plant update
  order, and stop.
- `RobotProgram` owns FTC lifecycle forwarding, one `LoopClock`, bindings, Tasks, outputs,
  presenters, telemetry commit, and fail-stop cleanup.

FTC SDK details stay in `edu.ftcphoenix.fw.ftc`. Pedro types stay in the narrow integration and
Phoenix path edge. Capability and strategy code do not depend on raw FTC devices or Followers.

## One managed OpMode grammar

TeleOp and Auto are parallel clients of `FtcRobotOpMode`/`RobotProgram`. Annotated OpModes do not
manually forward FTC callbacks.

TeleOp has one override:

```java
@Override
protected void configure(RobotProgram program) {
    new PhoenixTeleOpProgram(this, program, PhoenixAlliance.RED);
}
```

The entry makes its standalone default explicit. The package-private program owner copies
`PhoenixProfile`, installs one visible `PhoenixTeleOpPrestart`, constructs `PhoenixRobot`, and calls
`declareTeleOp(program, prestart.eligibleScoringTagIds())`. It then consumes the optional match
snapshot after localization exists, using its alliance only to seed the still-editable INIT menu.
The prestart's value at FTC START is the sole frozen targeting-eligibility authority.

Every production or diagnostic Phoenix-season Auto extends `PhoenixAutoOpMode` and returns only a
setup:

```java
@Override
protected PhoenixAutoSetup autoSetup() {
    return PhoenixAutoSetup.fromFixedSpec(
            PhoenixAutoSpec.audienceSafe(PhoenixAlliance.RED),
            PhoenixReadiness.AutoPurpose.MATCH_AUTO
    );
}
```

The parallel selector factory is
`PhoenixAutoSetup.fromInitSelection(defaultSpec, purpose)`. Both feed the same internal
`PhoenixAutoProgram` declaration. There is no raw Phoenix Auto OpMode base, private Auto clock,
private Auto runner, manual telemetry commit, or public start/update/stop alias on `PhoenixRobot`.
The disabled `robots.examples.pedro.BasicPedroAutoExample` is a generic new-robot reference using
the same managed `FtcRobotOpMode` grammar; it is not a Phoenix-season entry or alternate Phoenix
hardware graph.

## Construction and ownership

An ordinary Phoenix mechanism constructor receives `HardwareMap` plus data-only configuration,
defensively snapshots that configuration, constructs and privately owns its final resolver/Plant
graph, and owns update/stop. The OpMode does not prebuild or inject Plants.

`PhoenixTeleOpControls` constructs its stable driver and operator sources without registering
behavior. After `PhoenixCapabilities` exists, `PhoenixRobot` calls
`controls.bind(program.callbackBindings(), capabilities)` exactly once. This explicit boundary keeps
callback-graph mutation out of constructors while the managed program retains the only binding
heartbeat.

`PhoenixRobot` is constructed once for one mode. `declareTeleOp(...)` or `declareAuto(...)` may be
called once. Both modes retain the complete defensive `PhoenixProfile`; neither constructs an
alliance-filtered hardware profile. Their prestart owners instead supply a source that becomes one
singleton scoring-tag set after their `PhoenixAlliance` freezes. The ordinary Phoenix Auto host
supplies always-enabled aim and no override, while the same single `declareAuto(...)` boundary
retains clock-aware enable and override sources for an advanced direct assembly; neither choice
owns a second lifecycle.

Auto construction happens once during FTC INIT:

1. invalidate an old Phoenix match handoff and snapshot `PhoenixProfile`;
2. register one data-only `PhoenixAutoPrestart`;
3. construct one `PedroPathingRuntime`;
4. register the final-pose-plus-frozen-alliance match handoff transaction when applicable;
5. transfer the Pedro drive owner into Phoenix's managed service and construct vision,
   localization, targeting, and scoring once;
6. declare one root Task;
7. register additive presenters.

A construction/configuration failure is terminal for that OpMode instance. Correct the problem and
restart the OpMode. The selector never retries or replaces a hardware graph.

## Prestart selection and readiness

`RobotProgram` accepts at most one aggregated `Prestart` role:

```java
public interface Prestart {
    void update(LoopClock clock);
    StartDisposition freezeForStart();
}
```

Phoenix has two package-private prestart owners. TeleOp shows one RED/BLUE screen with the entry's
visible RED default. Gamepad 1's D-pad directly edits the highlighted draft during INIT—there is no
`A` confirmation—and a fresh Auto handoff may seed—but never freeze—the
same visible draft. Auto selects only alliance, start position, and strategy, then shows a read-only
summary. It has no unused partner-plan field and no second confirmation state; FTC START is the sole
freeze boundary for both modes.

These owners contain only menu bindings, selected data, previews, and readiness. They own no camera,
motor, Plant, Pedro heartbeat, or stop callback.

During INIT the order is clock, prestart, presenters, commit. At START, prestart freezes once before
the clock reset:

- `READY` starts services, root Task, and outputs normally.
- `BLOCKED` starts none of those and continues only clock plus presenters.

`PhoenixReadiness.allianceScoringTarget(...)` gives both modes the same selected-alliance catalog
and fixed-layout validation. `PhoenixReadiness.pedroAuto(...)` adds Auto purpose, calibration,
starting-contract, and route-maturity checks. Target visibility and game-piece uncertainty are not
readiness. Current competition routes are explicitly `INTEGRATION_ONLY`, so match entries remain
fail-closed; the named Pedro test entry may run them with a persistent warning.

## Root and route construction

Fixed route geometry is built eagerly when its spec is known. The INIT selector declares one lazy
root:

```java
program.rootTask(Tasks.buildAtStart(
        "phoenix.selectedAuto",
        () -> buildRoutine(prestart.frozenSpec())
));
```

`Tasks.buildAtStart(...)` defers only a fresh single-use Task graph. It is not a hardware or
resource factory. It invokes its supplier once at its own start boundary, retains the child before
starting it, and preserves child failures/cancellation/outcome.

Later route phases whose geometry depends on current pose use
`RouteTasks.followBuiltAtStart(...)`. That narrower factory samples live geometry once when that
route Task starts. `PhoenixPedroPathFactory` remains the one owner of Pedro geometry, route labels,
declared starting poses, and route maturity.

Each route start has one retained execution identity. The integration classifies endpoint success,
follower timeout/stall, interruption, replacement, failure, and unknown terminal state. `RouteTask`
retains that integration classification and adds Task-owned `TASK_TIMEOUT`, `CANCELLED`,
or fail-closed `FAILED` when applicable; `FAILED` can originate at either boundary. Robot routine
policy—not a generic Task sequence—decides continue, fallback, or abort.

## Pedro and localization

`PedroPathingRuntime` owns one `PinpointOdometryPredictor` and exposes a passive Pedro localizer
view of its snapshots. No second Pinpoint owner polls or resets the device.

At a permitted START:

```text
freeze spec
-> reset shared LoopClock
-> apply frozen Pedro starting pose
-> vision readiness
-> localization
-> targeting
-> Pedro heartbeat
-> root Task start + first update
-> scoring output
```

The adapter keeps one stable heartbeat throughout the active OpMode, even while a mechanism/wait
Task runs. Route and guidance Tasks may call the same hook; the adapter deduplicates every call by
`clock.cycle()`, including vendor-hidden updates.

The adapter's `stop()` applies physical zero immediately, including after reentrant callbacks. It
does not merely stage zero for a future follower update.

## Targeting eligibility

Phoenix keeps the complete configured scoring-target catalog and fixed tag layout. This allows all
appropriate fixed tags to support pose solving and field localization.

`ScoringTargeting` additionally receives one robot-owned
`Source<Set<Integer>> eligibleScoringTagIds`:

- TeleOp supplies the singleton mapped from its START-frozen alliance.
- Auto supplies the same singleton shape mapped from its START-frozen alliance.

At the first targeting update of a session, the owner validates and defensively freezes that set,
then builds one exact selector, fixed-layout subset, offset map, and guidance plan before reading the
sensor. Null, empty, null-member, unknown-ID, or missing fixed-layout entries fail fast. Inactive
catalog entries therefore cannot enter the current mode's plan. Eligibility is applied before
candidate preview and sticky selection, so an opposite-alliance tag cannot become either mode's
scoring target. This does not filter localization: its solver still owns the complete fixed tag
layout. `reset()` begins a new targeting session and requires a fresh drive graph.

Target visibility is handled inside the routine. The scoring attempt waits for its bounded
`waitForTargetSec`; timeout is retained truthfully, and `PhoenixPedroPreParkTask` selects the
explicit return/park fallback. No pre-reset INIT timestamp crosses the START clock epoch.

## Capabilities and scoring

`PhoenixCapabilities` contains the common vocabulary used by both mode clients. Controls and Auto
routines request intent through capability families; neither reaches into `ScoringPath`,
`ScoringTargeting`, or raw Plants.

`ScoringPath` is the single owner for:

- intake/eject/flywheel/shoot intent;
- the feed queue and active feed pulse;
- target overlays and flywheel target selection;
- flywheel readiness prediction;
- final scoring Plant resolvers and update order;
- transient rollback and terminal stop.

Feedback-aware Tasks name both command target and Plant because they write the target while the
Plant supplies completion feedback/provenance. Ordinary exact mechanisms retain only their Plant
and use its stable `commandTarget()` when constructing a command.

## Loop and telemetry order

The active Auto order is:

```text
LoopClock
-> vision/localization/targeting/Pedro service
-> bindings
-> Tasks
-> scoring output
-> presenters
-> one telemetry commit
```

TeleOp uses the same framework phases with its TeleOp services, controls, scoring output, and final
source-driven drive declaration. Services publish upstream facts. Tasks/controls change intent.
Outputs perform final physical realization. Presenters read retained snapshots only and never
clear or commit telemetry.

Optional `LoopPhaseProfiler` markers are observational. They do not add a heartbeat or include FTC
telemetry commit latency in a claimed robot phase.

## Cleanup and match handoff

`RobotProgram` first marks the program terminal, then performs:

```text
cancel Tasks
-> clear bindings
-> stop outputs in declaration order
-> stop services in reverse declaration order
```

For Auto, scoring therefore stops before the managed service writes Pedro zero, resets targeting,
and closes vision. Repeated/reentrant STOP is inert. Lifecycle `RuntimeException` preserves the
first failure and suppresses later cleanup failures; `Error` is not caught.

Match Auto declares one typed `stopHandoff(capture, publish, invalidate)` transaction. It
invalidates immediately. Only normal STOP from `ACTIVE` captures the cached final pose together
with the `PhoenixAlliance` frozen by that Auto. Publication occurs only after all cleanup succeeds.
Blocked, pre-start, failed, test, cleanup-failure, and publication-failure paths publish nothing and
invalidate.

TeleOp consumes a fresh snapshot once during INIT after localization exists. Its pose restores
localization and its alliance seeds the still-editable TeleOp draft. The operator may change that
draft before START. Missing, stale, or already-consumed snapshots leave the normally initialized
pose and visible RED draft unchanged.

## Profile shape

`PhoenixProfile` is data-only and copied by long-lived owners. Major sections include:

- `drive`: drivetrain hardware and scaling;
- `vision`: selected backend and camera rig;
- `localization`: predictor, AprilTag solve, and corrected-estimator policy;
- `field`: fixed field facts and tag layout;
- `autoAim`: alliance-to-scoring-tag mapping, complete target catalog, and targeting tolerances;
- `auto`: route/scoring budgets and fallback timing;
- `scoring`: mechanism hardware, Plants, readiness, and feed timing;
- `controls` and `driveAssist`: operator mapping and assist policy;
- `calibration`: explicit calibration acknowledgements.

Alliance, route choice, and strategy are not mutable profile filters. Auto carries all three in its
frozen `PhoenixAutoSpec`; TeleOp freezes its own `PhoenixAlliance` through
`PhoenixTeleOpPrestart`.

## Anti-patterns

Do not:

- put control scripts or route selection inside `PhoenixRobot`;
- construct a second Phoenix/Pedro hardware graph after INIT selection;
- override raw FTC callbacks in a Phoenix Auto entry;
- keep a private Auto clock, runner, telemetry commit, or cleanup path;
- make route Tasks the only Pedro heartbeat owner;
- filter the complete profile to choose an alliance target;
- let opposite-alliance observations enter preview/sticky target selection;
- make sensor visibility a structural START-readiness requirement;
- infer route success from a vendor idle flag;
- let presenters advance state;
- let Auto or TeleOp reach raw Plants or vendor Followers.

## Coordinate contract

Phoenix robot frame is right-handed: `+X` forward, `+Y` left, `+Z` up. Yaw and omega are
counter-clockwise positive. Distances are inches and angles are radians unless a name explicitly
states otherwise. Pedro/FTC conversions belong at the integration boundary.
