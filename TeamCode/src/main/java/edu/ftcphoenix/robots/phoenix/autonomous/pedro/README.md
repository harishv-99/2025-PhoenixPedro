# Phoenix Pedro autonomous structure

This package owns Phoenix's Pedro-specific path and routine implementation. Annotated Driver
Station entries remain in `edu.ftcphoenix.robots.phoenix.opmode`; they delegate here instead of
becoming strategy scripts.

Current roles:

- `PhoenixPedroPathFactory` builds Pedro `PathChain` values through the runtime's checked
  `pathBuilder()`, returns the explicitly named `pedroStartPose`, and owns current-pose route
  construction for the selected `PhoenixAutoSpec`.
- `PhoenixPedroAutoContext` carries the selected spec, profile snapshot, Phoenix capabilities,
  drive adapter, path factory, and fixed path set into routine factories.
- `PhoenixPedroAutoRoutineFactory` maps strategy ids to Phoenix Tasks with explicit route-result
  policy.
- `PhoenixPedroAutoOpModeBase` owns shared FTC/Pedro/Phoenix construction and lifecycle glue.
- Concrete selector/static OpModes choose or collect a spec, then delegate.

## Construction and ownership

`PhoenixPedroAutoOpModeBase` performs the production setup once during INIT:

1. derive an Auto-specific defensive `PhoenixProfile` snapshot;
2. ask project `Constants` for one complete `PedroPathingRuntime`;
3. pass `runtime.driveAdapter()` and `runtime.motionPredictor()` into `PhoenixRobot.initAuto(...)`;
4. build fixed paths and apply `paths.pedroStartPose` through `runtime.setStartingPose(...)`;
5. build and install the selected root routine over `PhoenixCapabilities` and the same drive
   adapter.

The runtime owns one profile-configured Pinpoint predictor, one passive Pedro localizer view, one
native Pedro mecanum/Follower graph, and one adapter. Phoenix owns localization/correction before
the adapter heartbeat and owns the adapter's final stop. The OpMode never calls raw
`Follower.update()`, constructs a second Pinpoint localizer, or stops the adapter separately.

The adapter remains in `PhoenixPedroAutoContext` because route and guidance Tasks command it through
`RouteFollower<PathChain>` and `DriveCommandSink`. That command access is not another lifecycle
owner. Same-cycle Task update hooks are harmless because the adapter deduplicates by the shared
`LoopClock.cycle()`.

Each `RouteFollower.follow(...)` call returns a `RouteExecution` tied to that route alone. The
adapter classifies completion, follower timeout/stall, interruption, replacement, failure, or an
unknown terminal transition during its owned heartbeat; `RouteTask.getRouteStatus()` preserves that
backend-neutral reason. Task timeout and active cancellation remain distinct statuses, and cleanup
of an old Task cannot cancel a newer route.

Fixed robot routes continue to use the short `RouteTasks.follow(adapter, route, cfg)` call. Robot
routines must not start, break, replace, manually take over, update, or reset pose through the raw
Pedro Follower; those lifecycle calls bypass the adapter's ownership and truthful status. Route
building and read-only integration inspection remain supported.

The placeholder outbound route is fixed and remains eager. The return helper uses
`RouteTasks.followBuiltAtStart(...)` with a lambda to `PhoenixPedroPathFactory`, so the factory reads
and snapshots the current Pedro pose only when that phase begins, then builds the concrete return
through the checked runtime builder. The generic route Task never sees the Follower, vision state,
or strategy, and a failed build cannot start the adapter.

## Pre-park budget, route, and scoring policy

The integration reports route facts; `PhoenixPedroAutoRoutineFactory` owns their strategy meaning.
The checked-in strategies treat outbound plus scoring as one pre-park Task A, wrap A in
`Tasks.withTimeout(A, profile.auto.parkTakeoverElapsedSec)`, and put one live-pose park B after that
timed region. If A finishes early, B starts immediately. If A is still active at the configured
25-second default, the wrapper actively cancels A and permits B only after A's safety cleanup
returns successfully. Once B starts, no timer can interrupt or restart it.

The Pedro heartbeat runs before the Task phase. If it—or the scoring child—has already produced a
terminal result on the exact cutoff cycle, pre-park preserves that result before applying ordinary
cancellation. A valid success or local timeout remains park-eligible; interruption, replacement,
failure, cancellation, unknown, or malformed lifecycle evidence suppresses B.

Aim/shoot runs only after outbound `COMPLETED`. An outbound `FOLLOWER_TIMEOUT_OR_STALL` or
`TASK_TIMEOUT` skips scoring and permits B as a truthful local-timeout fallback. `INTERRUPTED`,
`REPLACED`, `CANCELLED`, `FAILED`, and `UNKNOWN_TERMINAL` suppress B. Direct routine cancellation,
FTC STOP, or a lifecycle/cleanup failure likewise aborts without launching recovery.

The scoring attempt retains unavailable-target, aim, and shot-drain timeouts instead of converting
them to success. On scoring timeout, Phoenix cancels only that attempt's transient shot, disables
the flywheel request enabled by the outbound route, and still attempts the normal live-pose return.
Cancellation-like scoring results abort. Return/fallback timeout remains a timeout; other abnormal
return results remain cancellation-like and do not start another replacement route.

Cleanup stays inside the capability boundary. On a natural PHX-03 ending, the scoring attempt owns
its transient shot request and pre-park disables the flywheel request enabled by this routine. At
the match cutoff or another active pre-park cancellation, pre-park additionally cancels its active
route/aim/scoring child, clears the Auto root's transient and held scoring requests, and applies
immediate drive zero before B may start. Every safety action is attempted even if an earlier action
fails; a failure suppresses B. The policy never writes a Plant or hardware device directly.

The recurring Auto order is:

```text
Phoenix localization/correction -> targeting -> Pedro adapter heartbeat
-> Auto TaskRunner -> scoring -> telemetry
```

The passive Pedro localizer verifies the predictor snapshot belongs to that exact Phoenix cycle.
The default correction policy pushes accepted AprilTag corrections into the shared predictor, so
Pedro path control and Phoenix targeting see one corrected pose in the same heartbeat.

## Exact START, INIT retry, and telemetry

The base installs one root routine during INIT. At FTC START, `PhoenixRobot.startAny(...)` resets the
one shared clock and `startAuto()` immediately starts that installed timed root. The private
pre-park Task arms there but does not start outbound work until the first later Auto Task phase,
after localization, targeting, and the Pedro heartbeat. This measures the match budget from the
real START boundary without moving the robot before the normal loop order is ready.

The base is retry-safe during INIT. A failed attempt asks the partially initialized `PhoenixRobot`
to stop every retained owner, clears runtime references, records an actionable error, and allows a
later selector confirmation to rebuild cleanly. Successful initialization keeps the selected spec,
profile, paths, robot, and adapter as one consistent graph.

Pedro debug rows are added before `PhoenixRobot.updateAuto()` emits the standard Auto block. The
Driver Station therefore gets one coherent frame containing spec/path labels, Pedro pose/busy state,
the backend-neutral `route.status`, Task/scoring/targeting status, raw predictor pose, corrected
global pose, and pose drift. Phoenix retains the installed root after completion; its dynamic name
and outcome distinguish normal continuation, local timeout, match-time cutoff, suppressed park,
active park, and the final park result after the mutable latest-route row changes.

## Where students add real Auto behavior

The checked-in geometry is still the small placeholder integration route. Replace geometry branches
inside `PhoenixPedroPathFactory` with real alliance/start/partner paths. Keep fixed routes eager;
for live-pose or live-vision geometry, add a narrow path-factory method and pass it to
`followBuiltAtStart(...)`. Keep high-level strategy selection in
`PhoenixPedroAutoRoutineFactory`, and keep reusable scoring/targeting snippets in
`PhoenixAutoTasks`. Routine Tasks continue using the existing adapter and `PhoenixCapabilities`;
they do not need a separate localization API.

Use `Tasks.withTimeout(...)` for a parent-owned hard budget around a complete Task graph. Keep a
route's own timeout when `RouteStatus.TASK_TIMEOUT` and route-specific cancellation matter; the
outer wrapper uses ordinary cancellation and may report `TIMEOUT` while its retained route or
pre-park child reports `CANCELLED`.

For bounded work that should exist only while a route is active, compose it as
`Tasks.parallelDeadline(routeTask, companionTask)`. The route owns completion and outcome; a
started companion is asked to cancel when the route ends. The companion must own safe active
cancellation—do not use an `enable → wait → disable` sequence whose final step can be skipped.
Persistent intake, flywheel, scoring, and aim requests remain capability/service state. The current
placeholder routes do not require a deadline companion yet.

Truthful route status supplies the fact for Phoenix's robot-owned continue/fallback/abort decision;
it does not choose that policy. The existing routine factory applies the conservative mapping above.
When adding a strategy that intentionally continues after an interruption, state that different
mapping in the routine instead of changing the integration or assuming a generic Task sequence
short-circuits.
