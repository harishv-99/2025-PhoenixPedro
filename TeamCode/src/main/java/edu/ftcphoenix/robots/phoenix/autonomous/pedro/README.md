# Phoenix Pedro autonomous structure

This folder contains Pedro-specific autonomous implementation code. Annotated Driver Station entries
live in `edu.ftcphoenix.robots.phoenix.opmode`; this package owns the Pedro path/routine pieces that
those entries call.

Current files:

- `PhoenixPedroPathFactory` builds Pedro `PathChain` objects from a robot-owned `PhoenixAutoSpec`.
- `PhoenixPedroAutoContext` carries the selected spec, profile snapshot, capabilities, drive adapter,
  and built path set into routine factories.
- `PhoenixPedroAutoRoutineFactory` maps strategy ids to Phoenix task sequences.

The related annotated OpModes are in `robots.phoenix.opmode`:

- `PhoenixPedroAutoOpModeBase` owns shared FTC/Pedro/Phoenix lifecycle glue.
- `PhoenixPedroAutoTestOpMode` keeps the original 12-inch integration test available.
- `PhoenixPedroAutoSelectorOpMode` uses framework UI helpers to build a `PhoenixAutoSpec` during INIT, then locks the screen after successful Phoenix/Pedro initialization.
- `PhoenixRedAudienceSafeAuto` and `PhoenixBlueAudienceSafeAuto` are static safe-match entries.

The intended split is:

- `PhoenixRobot` owns shared runtime, capability families, the Auto task runner, and the recurring
  lifecycle of one required backend-neutral Auto `DriveCommandSink`.
- `PhoenixAutoSpec` owns the chosen match setup: alliance, start position, partner plan, and strategy.
- `PhoenixAutoProfiles` derives an Auto-specific profile snapshot before constructing `PhoenixRobot`, using profile-owned red/blue Auto scoring tag ids.
- `PhoenixPedroPathFactory` owns Pedro geometry and placeholder path callbacks.
- `PhoenixPedroAutoRoutineFactory` owns strategy-to-task-sequence mapping.
- `PhoenixPedroAutoOpModeBase` constructs the team-configured follower and adapter, supplies the
  adapter once with `PhoenixRobot.initAuto(adapter)`, and then delegates its heartbeat and final
  stop to the robot.
- Concrete OpModes choose or collect a spec, then delegate. They do not update or stop the adapter
  separately.

The adapter remains in `PhoenixPedroAutoContext` because route and guidance Tasks command it through
`RouteFollower<PathChain>` and `DriveCommandSink`. That command access does not create another
lifecycle owner. `PhoenixRobot.updateAuto()` advances the adapter after localization/targeting and
before the Auto task runner. The adapter deduplicates update requests by the shared
`LoopClock.cycle()`, so Task-local update hooks cannot create a second Pedro heartbeat in the same
loop.

`PhoenixPedroAutoOpModeBase` is retry-safe during INIT. If selector confirmation fails partway
through Phoenix or Pedro construction, the base class asks the active `PhoenixRobot` lifecycle owner
to stop its retained adapter and all other partially-created runtime owners, clears runtime
references, records the error, and allows a later confirmation attempt to rebuild from a clean
state. Successful initialization still locks the selector summary so driver-visible choices cannot
drift away from the queued routine.

Pedro debug rows are added before `PhoenixRobot.updateAuto()` emits the standard Phoenix Auto block.
That keeps `auto.spec`, `auto.paths`, Pedro pose/busy state, task status, scoring status, targeting
status, and pose estimates in one telemetry update instead of splitting the Driver Station display
across two frames.

The recurring Auto order is intentionally one-owner and explicit:

```text
Phoenix localization -> targeting -> Pedro adapter heartbeat -> Auto TaskRunner -> scoring -> telemetry
```

This item establishes only heartbeat and stopped-state ownership. Pedro drivetrain construction,
Pinpoint/localization ownership, coordinate conversion, and pose-correction authority remain
separate `PEDRO-02` work; path and routine structure should not work around those undecided
boundaries.

The checked-in geometry is still a placeholder based on the old 12-inch Pedro integration path. Real
alliance/start/partner routes should replace the branches inside `PhoenixPedroPathFactory`, while
reusable scoring snippets should stay in `PhoenixAutoTasks` and high-level strategy mapping should
stay in `PhoenixPedroAutoRoutineFactory`.
