# Phoenix production robot

This package is the repository's complete production example of a robot built on the Phoenix
framework. It is useful after the beginner course because it shows how the same framework pieces
fit together at competition scale.

If you are building your first mechanism, start at the canonical
[`Phoenix framework documentation hub`](<../../fw/docs/README.md>). Framework and robot design
choices must follow the [`Framework Principles`](<../../fw/Framework Principles.md>).

## Read this package in order

1. [`Phoenix Architecture`](<Phoenix Architecture.md>) explains the complete ownership graph,
   managed lifecycle, loop order, readiness rules, and TeleOp/Auto relationship.
2. [`PhoenixProfile.java`](<PhoenixProfile.java>) contains the data-only hardware and policy
   configuration that long-lived owners defensively copy.
3. [`Phoenix Calibration Guide`](<Phoenix Calibration Guide.md>) explains how to replace checked-in
   placeholders with measurements and record the required acknowledgements.
4. [`Phoenix Pedro autonomous`](<autonomous/pedro/README.md>) explains route ownership, INIT
   selection, route outcomes, and match handoff.

## Owner map

| Owner | Responsibility |
|---|---|
| [`PhoenixRobot.java`](<PhoenixRobot.java>) | Constructs the robot graph and declares its managed lifecycle roles. It is not a control script. |
| [`PhoenixCapabilities.java`](<PhoenixCapabilities.java>) | Defines the mode-neutral robot actions and status shared by TeleOp and Auto. |
| [`PhoenixTeleOpControls.java`](<PhoenixTeleOpControls.java>) | Maps driver and operator inputs to capability requests. |
| [`ScoringPath.java`](<ScoringPath.java>) | Realizes scoring intent, owns its final Plant graph, feed policy, and output order. |
| [`ScoringTargeting.java`](<ScoringTargeting.java>) | Selects an eligible scoring target and publishes targeting facts and guidance. |
| [`PhoenixReadiness.java`](<PhoenixReadiness.java>) | Decides whether a selected mode may start from configuration and route-maturity facts. |

`PhoenixRobot` wires these owners together. Controls and autonomous routines use
`PhoenixCapabilities`; they do not reach into mechanisms, Plants, FTC devices, or Pedro followers.

## Mode entries

- [`PhoenixTeleOp.java`](<opmode/PhoenixTeleOp.java>) is the competition TeleOp entry. Its INIT
  screen selects the alliance used for scoring-target eligibility.
- [`PhoenixRedAudienceSafeAuto.java`](<opmode/PhoenixRedAudienceSafeAuto.java>) and
  [`PhoenixBlueAudienceSafeAuto.java`](<opmode/PhoenixBlueAudienceSafeAuto.java>) are fixed-spec
  match Auto entries.
- [`PhoenixPedroAutoSelectorOpMode.java`](<opmode/PhoenixPedroAutoSelectorOpMode.java>) selects
  alliance, start position, and strategy during INIT.
- [`PhoenixPedroAutoTestOpMode.java`](<opmode/PhoenixPedroAutoTestOpMode.java>) is the explicitly
  named integration-test entry for checked-in Pedro geometry.
- [`PhoenixTestersOpMode.java`](<opmode/PhoenixTestersOpMode.java>) exposes calibration and hardware
  tests without creating a second production robot architecture.

The competition TeleOp and Auto entries use the managed `FtcRobotOpMode`/`RobotProgram` lifecycle.
Each entry chooses a setup; the program owns the clock, lifecycle phases, Tasks, outputs, telemetry
commit, and fail-stop cleanup. `PhoenixTestersOpMode` is the deliberate exception: it extends
`FtcTeleOpTesterOpMode`, whose tester host owns the shared clock, tester lifecycle, and fail-stop
cleanup without creating a `RobotProgram`.

## Where to make a change

| Goal | Change here |
|---|---|
| Rename hardware or enter a measured constant | [`PhoenixProfile.java`](<PhoenixProfile.java>) |
| Change a button's meaning | [`PhoenixTeleOpControls.java`](<PhoenixTeleOpControls.java>) |
| Add mode-neutral robot intent | [`PhoenixCapabilities.java`](<PhoenixCapabilities.java>) and its owning mechanism |
| Change scoring realization or safety behavior | [`ScoringPath.java`](<ScoringPath.java>) |
| Change target-selection or drive-assist policy | [`ScoringTargeting.java`](<ScoringTargeting.java>) or [`PhoenixDriveAssistService.java`](<PhoenixDriveAssistService.java>) |
| Change route geometry | [`PhoenixPedroPathFactory.java`](<autonomous/pedro/PhoenixPedroPathFactory.java>) |
| Change autonomous strategy | [`PhoenixPedroAutoRoutineFactory.java`](<autonomous/pedro/PhoenixPedroAutoRoutineFactory.java>) |
| Change what blocks START | [`PhoenixReadiness.java`](<PhoenixReadiness.java>) |
| Add read-only Driver Station information | [`PhoenixTelemetryPresenter.java`](<PhoenixTelemetryPresenter.java>) |

Keep the change with the owner of that decision. `PhoenixRobot` should change only when the
composition or lifecycle graph changes.

## Readiness and physical validation

The checked-in competition routes are marked `INTEGRATION_ONLY`. Match Auto entries intentionally
remain blocked until their geometry is calibrated, validated on the physical robot, and promoted
to `MATCH_READY`. The Pedro test entry can run integration geometry while keeping a persistent test
warning visible.

Compilation and fake tests cannot validate wiring, motor direction, camera placement, odometry,
mechanism limits, traction, or route clearance. Follow the calibration guide, test conservatively,
and record only acknowledgements supported by physical evidence.
