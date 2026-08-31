# Phoenix Application Instructions

This file applies to the complete Phoenix production-application subtree. It supplements the
repository-root instructions without weakening the Sushi Framework Principles.

## Primary design authority

Before changing Phoenix code, read completely:

1. `TeamCode/src/main/java/edu/ftcsushi/fw/Framework Principles.md`
2. The relevant guide under `TeamCode/src/main/java/edu/ftcsushi/fw/docs/`
3. `TeamCode/src/main/java/edu/ftcsushi/robots/phoenix/Phoenix Architecture.md`

Keep application implementation, package-local documentation, Javadocs, and tests synchronized.

## Application bubble

- Keep Phoenix identity, hardware facts, match policy, configuration, legacy implementation, and
  application-only vendor tooling inside this main package and its matching test package.
- Phoenix may depend on Sushi core and explicit FTC/vendor integration edges. Framework code,
  shared tooling, and independent examples must never depend back on Phoenix.
- Do not add a generic forwarding package or compatibility alias for application-owned code.

## Phoenix-specific expectations

- Keep `PhoenixRobot` a composition root and lifecycle/loop owner, not a control script.
- Keep `PhoenixProfile` data-only and defensively copy configuration for long-lived owners.
- Keep drivetrain, AprilTag vision, localization, and field-layout ownership distinct. Phoenix may
  select a concrete vision backend, while consumers depend on the backend-neutral lane interface.
- TeleOp must map gamepads in `PhoenixTeleOpControls` and call robot-owned capability families rather
  than reaching into `PhoenixScoring`, `PhoenixTargeting`, or raw Plants.
- Auto and TeleOp are parallel clients of the same `PhoenixCapabilities` vocabulary. Keep alliance,
  route selection, Pedro paths, and routine composition outside `PhoenixRobot`.
- Keep scoring's intent, execution policy, and hardware realization separated. Preserve the single
  owner for feed queues, target overlays, flywheel readiness, and scoring Plant update order.
- Keep targeting and drive-assist decisions in their robot-owned services; do not move season-specific
  scoring or aim policy into reusable framework lanes.
- Keep ordinary OpModes thin: choose configuration/specification and declare the robot's roles only
  through `configure(RobotProgram)`. `FtcRobotOpMode` owns the final FTC lifecycle callbacks,
  managed loop phases, and cleanup. An explicitly custom host is the advanced exception.
