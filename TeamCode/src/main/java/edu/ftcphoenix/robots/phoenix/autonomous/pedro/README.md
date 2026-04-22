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
- `PhoenixPedroAutoSelectorOpMode` uses framework UI helpers to build a `PhoenixAutoSpec` during INIT.
- `PhoenixRedAudienceSafeAuto` and `PhoenixBlueAudienceSafeAuto` are static safe-match entries.

The intended split is:

- `PhoenixRobot` owns shared runtime, capability families, and the Auto task runner.
- `PhoenixAutoSpec` owns the chosen match setup: alliance, start position, partner plan, and strategy.
- `PhoenixAutoProfiles` derives an Auto-specific profile snapshot before constructing `PhoenixRobot`.
- `PhoenixPedroPathFactory` owns Pedro geometry and placeholder path callbacks.
- `PhoenixPedroAutoRoutineFactory` owns strategy-to-task-sequence mapping.
- OpModes choose or collect a spec, then delegate.

The checked-in geometry is still a placeholder based on the old 12-inch Pedro integration path. Real
alliance/start/partner routes should replace the branches inside `PhoenixPedroPathFactory`, while
reusable scoring snippets should stay in `PhoenixAutoTasks` and high-level strategy mapping should
stay in `PhoenixPedroAutoRoutineFactory`.
