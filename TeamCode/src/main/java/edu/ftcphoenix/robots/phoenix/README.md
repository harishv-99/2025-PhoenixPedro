# Phoenix production robot

This package is the repository's complete production example of a robot built on the Phoenix
framework. It is useful after
[Phoenix in one picture](<../../fw/docs/getting-started/Framework Overview.md>) and the relevant
[Phoenix topic](<../../fw/docs/getting-started/Beginner's Guide.md>) because it shows how the same
framework pieces fit together at competition scale.

If you are building your first mechanism, start at the canonical
[`Phoenix framework documentation hub`](<../../fw/docs/README.md>). Framework and robot design
choices must follow the [`Framework Principles`](<../../fw/Framework Principles.md>).

## Read this package in order

1. [`Phoenix Architecture`](<Phoenix Architecture.md>) explains the complete ownership graph,
   managed lifecycle, loop order, readiness rules, and TeleOp/Auto relationship.
2. [`PhoenixProfile.java`](<PhoenixProfile.java>) assembles one fresh, data-only current
   configuration graph whose active slices are synchronously handed to their long-lived owners.
3. [`Phoenix Calibration Guide`](<Phoenix Calibration Guide.md>) explains how to replace checked-in
   placeholders with measurements and record the required acknowledgements.
4. [`Phoenix Pedro autonomous`](<autonomous/pedro/README.md>) explains route ownership, INIT
   selection, route outcomes, and match handoff.

## Owner map

| Owner | Responsibility |
|---|---|
| [`PhoenixRobot.java`](<PhoenixRobot.java>) | Constructs the robot graph and declares its managed lifecycle roles. It is not a control script. |
| [`PhoenixCapabilities.java`](<PhoenixCapabilities.java>) | Defines the mode-neutral robot actions and capability-owned status snapshots shared by TeleOp and Auto. |
| [`PhoenixTeleOpControls.java`](<PhoenixTeleOpControls.java>) | Maps driver and operator inputs to capability requests. |
| [`PhoenixScoring.java`](<scoring/PhoenixScoring.java>) | Owns scoring requests, feed policy, all four scoring Plants, update order, status production, and stop. |
| [`PhoenixTargeting.java`](<scoring/PhoenixTargeting.java>) | Selects an eligible scoring target and publishes targeting facts and guidance. |
| [`PhoenixReadiness.java`](<PhoenixReadiness.java>) | Decides whether a selected mode may start from configuration and route-maturity facts. |

`PhoenixRobot` wires these owners together. Controls and autonomous routines use
`PhoenixCapabilities`; they do not reach into mechanisms, Plants, FTC devices, or Pedro followers.
Its only public constructor is `PhoenixRobot(HardwareMap)`. TeleOp supplies its one local
`PhoenixProfile`, both Gamepads, and targeting-eligibility source to `declareTeleOp(...)`; Auto
supplies its local profile and Auto-only runtime roles to `declareAuto(...)`. The root retains no
aggregate profile or dormant Gamepad dependency.

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
- [`PhoenixPanelsTuningOpMode.java`](<opmode/PhoenixPanelsTuningOpMode.java>) is the dedicated
  **Phoenix: Tuning (Panels)** flywheel workflow. It opens the tuner directly—there is no tester
  menu to navigate; see the
  [`control tuning runbook`](<../../fw/docs/testing-calibration/Control Tuning Workflow.md>).

The competition TeleOp and Auto entries use the managed `FtcRobotOpMode`/`RobotProgram` lifecycle.
Each entry chooses a setup; the program owns the clock, lifecycle phases, Tasks, outputs, telemetry
commit, and fail-stop cleanup. `PhoenixTestersOpMode` is the deliberate exception: it extends
`FtcTeleOpTesterOpMode`, whose tester host owns the shared clock, tester lifecycle, and fail-stop
cleanup without creating a `RobotProgram`. The Panels tuning entry is the same kind of explicit
tester-host exception: the framework tuner owns one fresh flywheel Plant, requires exactly one
client, and terminally stops/restores on disconnect or failure. The Plant comes from
`PhoenixScoring`'s canonical flywheel recipe but is never shared with production. Production TeleOp
and Auto never read its Configurable draft values.

## Where to make a change

| Goal | Change here |
|---|---|
| Change drivetrain motor names, directions, or brake policy | `PhoenixDriveConfiguration.current()` in [`PhoenixDriveConfiguration.java`](<PhoenixDriveConfiguration.java>) |
| Change Pinpoint, AprilTag correction, Fusion, or EKF policy | `PhoenixLocalizationConfiguration.current()` in [`PhoenixLocalizationConfiguration.java`](<PhoenixLocalizationConfiguration.java>) |
| Record reviewed Pinpoint axes or pod-offset evidence | `PhoenixCalibrationConfiguration.current()` in [`PhoenixCalibrationConfiguration.java`](<PhoenixCalibrationConfiguration.java>) |
| Change the fixed FTC field tag layout | `FtcGameTagLayout.currentGameFieldFixed()` in the framework FTC boundary |
| Select/configure the active vision backend | `PhoenixVisionFactory.Config.defaults()` in [`PhoenixVisionFactory.java`](<PhoenixVisionFactory.java>) |
| Change a button's meaning or drive-control tuning | `PhoenixTeleOpControls.Config.defaults()` in [`PhoenixTeleOpControls.java`](<PhoenixTeleOpControls.java>) |
| Change drive-assist policy | `PhoenixDriveAssistService.Config.defaults()` in [`PhoenixDriveAssistService.java`](<PhoenixDriveAssistService.java>) |
| Change scoring hardware, bounds, timing, or controller values | `PhoenixScoring.Config.defaults()` in [`PhoenixScoring.java`](<scoring/PhoenixScoring.java>) |
| Tune flywheel velocity control | Open **Phoenix: Tuning (Panels)**, correlate trials by session/segment ID, then copy accepted controller readback into `PhoenixScoring.Config.defaults()` |
| Change target catalog, alliance mapping, or aim policy | `PhoenixTargeting.Config.defaults()` in [`PhoenixTargeting.java`](<scoring/PhoenixTargeting.java>); edit shot rows in `PhoenixShotVelocityCalibration.currentTable()` |
| Change autonomous timing/budgets | `PhoenixAutoConfig.defaults()` in [`PhoenixAutoConfig.java`](<PhoenixAutoConfig.java>) |
| Add mode-neutral robot intent | [`PhoenixCapabilities.java`](<PhoenixCapabilities.java>) and its owning mechanism |
| Change scoring realization or safety behavior | [`PhoenixScoring.java`](<scoring/PhoenixScoring.java>) |
| Change target-selection or drive-assist policy | [`PhoenixTargeting.java`](<scoring/PhoenixTargeting.java>) or [`PhoenixDriveAssistService.java`](<PhoenixDriveAssistService.java>) |
| Change route geometry | [`PhoenixPedroPathFactory.java`](<autonomous/pedro/PhoenixPedroPathFactory.java>) |
| Change autonomous strategy | [`PhoenixPedroAutoRoutineFactory.java`](<autonomous/pedro/PhoenixPedroAutoRoutineFactory.java>) |
| Change what blocks START | [`PhoenixReadiness.java`](<PhoenixReadiness.java>) |
| Add read-only Driver Station information | [`PhoenixTelemetryPresenter.java`](<PhoenixTelemetryPresenter.java>) |

Keep the change with the owner of that decision. `PhoenixRobot` should change only when the
composition or lifecycle graph changes.

`PhoenixProfile.current()` is the sole public profile factory and returns a new graph every time.
For a one-run override, assign it once, edit that local value, and pass the same value to the mode
declaration. Calling `current()` again does not recover an earlier edit. There is intentionally no
aggregate `PhoenixProfile.defaults()` or `PhoenixProfile.copy()`; each runtime owner captures and
validates only the slice active in that mode or selected backend.

## Readiness and physical validation

The checked-in competition routes are marked `INTEGRATION_ONLY`. Match Auto entries intentionally
remain blocked until their geometry is calibrated, validated on the physical robot, and promoted
to `MATCH_READY`. The Pedro test entry can run integration geometry while keeping a persistent test
warning visible.

Compilation and fake tests cannot validate wiring, motor direction, camera placement, odometry,
mechanism limits, traction, or route clearance. Follow the calibration guide, test conservatively,
and record only acknowledgements supported by physical evidence. The ordinary TeleOp and Auto
programs do perform one centralized pre-effect collision check between the four drive motor names
and scoring's intake/flywheel names, but that software check cannot establish physical identity or
safe behavior.
