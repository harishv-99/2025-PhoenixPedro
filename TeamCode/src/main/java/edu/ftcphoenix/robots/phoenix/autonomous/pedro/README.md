# Phoenix Pedro autonomous structure

This package owns Phoenix's Pedro-specific path and routine implementation. Annotated Driver
Station entries remain in `edu.ftcphoenix.robots.phoenix.opmode`; they delegate here instead of
becoming strategy scripts.

Current roles:

- `PhoenixPedroPathFactory` builds Pedro `PathChain` values through the runtime's checked
  `pathBuilder()` and returns the explicitly named `pedroStartPose` for the selected
  `PhoenixAutoSpec`.
- `PhoenixPedroAutoContext` carries the selected spec, profile snapshot, Phoenix capabilities,
  drive adapter, and built path set into routine factories.
- `PhoenixPedroAutoRoutineFactory` maps strategy ids to Phoenix Task sequences.
- `PhoenixPedroAutoOpModeBase` owns shared FTC/Pedro/Phoenix construction and lifecycle glue.
- Concrete selector/static OpModes choose or collect a spec, then delegate.

## Construction and ownership

`PhoenixPedroAutoOpModeBase` performs the production setup once during INIT:

1. derive an Auto-specific defensive `PhoenixProfile` snapshot;
2. ask project `Constants` for one complete `PedroPathingRuntime`;
3. pass `runtime.driveAdapter()` and `runtime.motionPredictor()` into `PhoenixRobot.initAuto(...)`;
4. build paths and apply `paths.pedroStartPose` through `runtime.setStartingPose(...)`;
5. build and enqueue the selected routine over `PhoenixCapabilities` and the same drive adapter.

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

Robot routines continue to use the short `RouteTasks.follow(adapter, route, cfg)` call. They must not
start, break, replace, manually take over, update, or reset pose through the raw Pedro Follower;
those lifecycle calls bypass the adapter's ownership and truthful status. Route building and
read-only integration inspection remain supported.

The recurring Auto order is:

```text
Phoenix localization/correction -> targeting -> Pedro adapter heartbeat
-> Auto TaskRunner -> scoring -> telemetry
```

The passive Pedro localizer verifies the predictor snapshot belongs to that exact Phoenix cycle.
The default correction policy pushes accepted AprilTag corrections into the shared predictor, so
Pedro path control and Phoenix targeting see one corrected pose in the same heartbeat.

## INIT retry and telemetry

The base is retry-safe during INIT. A failed attempt asks the partially initialized `PhoenixRobot`
to stop every retained owner, clears runtime references, records an actionable error, and allows a
later selector confirmation to rebuild cleanly. Successful initialization keeps the selected spec,
profile, paths, robot, and adapter as one consistent graph.

Pedro debug rows are added before `PhoenixRobot.updateAuto()` emits the standard Auto block. The
Driver Station therefore gets one coherent frame containing spec/path labels, Pedro pose/busy state,
the backend-neutral `route.status`, Task/scoring/targeting status, raw predictor pose, corrected
global pose, and pose drift.

## Where students add real Auto behavior

The checked-in geometry is still the small placeholder integration route. Replace geometry branches
inside `PhoenixPedroPathFactory` with real alliance/start/partner paths. Keep high-level strategy
selection in `PhoenixPedroAutoRoutineFactory`, and keep reusable scoring/targeting snippets in
`PhoenixAutoTasks`. Routine Tasks continue using the existing adapter and `PhoenixCapabilities`;
they do not need a separate localization API.

Truthful route status supplies the fact needed for a later robot-owned continue/fallback/abort
decision; it does not choose that policy. Phoenix routines must add that policy explicitly, so do
not assume generic Task sequences automatically stop after every abnormal route ending.
