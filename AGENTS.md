# Codex Repository Instructions

This file applies to the entire repository. More-specific `AGENTS.md` files may add local guidance,
but they must not weaken the framework principles below.

## Primary design authority

Before changing framework or modern Phoenix robot code, read:

1. `TeamCode/src/main/java/edu/ftcphoenix/fw/Framework Principles.md`
2. The relevant guide under `TeamCode/src/main/java/edu/ftcphoenix/fw/docs/`
3. For Phoenix changes, `TeamCode/src/main/java/edu/ftcphoenix/robots/phoenix/Phoenix Architecture.md`

Treat **Framework Principles.md as a design requirement, not optional background reading**. After
the user's explicit instructions, it is the repository's primary authority for architecture, API
shape, lifecycle, naming, documentation, and extension decisions. Do not work around a principle
silently. If a requested change appears to conflict with it, explain the conflict and choose the
smallest principle-consistent design, or ask for direction when the choice would materially change
the requested result.

Keep implementation, Javadocs, Markdown guides, and examples synchronized. Documentation is part of
the framework API.

## Repository map

- `edu.ftcphoenix.fw`: reusable Phoenix framework code.
- `edu.ftcphoenix.fw.ftc`: the FTC SDK boundary and stable FTC resource owners.
- `edu.ftcphoenix.fw.integrations`: narrow third-party integration edges, such as Pedro Pathing.
- `edu.ftcphoenix.fw.tools`: examples, testers, and calibration support.
- `edu.ftcphoenix.robots.phoenix`: the modern framework-based Phoenix robot and reference design.
- `org.firstinspires.ftc.teamcode.robots` and FTC controller samples: legacy or sample code. Do not
  use these as architectural templates for new framework-based work.

## Non-negotiable framework rules

- Keep robot behavior non-blocking. Do not add sleeps, busy waits, or long-running loops to TeleOp
  or Auto behavior. Represent work over time with cooperative `Task`s updated by the main loop.
- Use one `LoopClock` heartbeat. Advance it exactly once per OpMode cycle; other components read it
  but never advance their own competing timebase.
- Preserve per-cycle semantics. Stateful sources, bindings, task runners, and similar components
  must be safe against repeated sampling or updating in the same `clock.cycle()` where required.
- Maintain clear ownership:
  - primitives own one narrow reusable capability;
  - framework lanes own stable reusable multi-object resource/lifecycle graphs;
  - field facts remain separate from sensor ownership and game strategy;
  - robot capability families expose shared mode-neutral intent and status;
  - controls own operator meanings;
  - subsystems/realization own final Plant target sources and Plant update order;
  - supervisors and services own robot-specific policy and coordination;
  - presenters only format snapshots for humans;
  - the composition root wires objects and makes loop order explicit.
- Keep FTC SDK and vendor details at explicit boundaries. Core framework logic should depend on
  Phoenix abstractions, not directly on `com.qualcomm.*` or third-party route-library types.
- Keep Plants source-driven. Each Plant has one final `PlantTargetSource`; compose behavior with
  `PlantTargets.exact(...)`, `overlay(...)`, or `plan(...)`, then let the Plant apply hardware
  bounds, references, and guards. Do not introduce competing imperative writers.
- Prefer framework task factories (`Tasks`, `PlantTasks`, `DriveTasks`, guidance/route task helpers)
  over hand-written task state machines unless a new state machine is genuinely needed.
- Keep drive intent and actuation separate: `DriveSource` produces robot-centric `DriveSignal`s;
  overlays reshape selected components; a `DriveCommandSink` performs the final write.
- Preserve the documented coordinate contract: robot frame `+X` forward, `+Y` left, `+Z` up;
  yaw/omega counter-clockwise positive; distances in inches; angles in radians. Put frames and
  units in names where ambiguity is possible, and convert third-party conventions at boundaries.
- Fail fast with actionable configuration errors. Prefer staged builders and types that prevent
  invalid combinations over hidden precedence or late generic failures.
- Prefer one clear API over parallel legacy paths. Breaking changes are acceptable when they make
  the framework more coherent; update all in-repository callers and documentation together.

## Phoenix-specific expectations

- Keep `PhoenixRobot` a composition root and lifecycle/loop owner, not a control script.
- Keep `PhoenixProfile` data-only and defensively copy configuration for long-lived owners.
- Keep drivetrain, AprilTag vision, localization, and field-layout ownership distinct. Phoenix may
  select a concrete vision backend, while consumers depend on the backend-neutral lane interface.
- TeleOp must map gamepads in `PhoenixTeleOpControls` and call robot-owned capability families rather
  than reaching into `ScoringPath`, `ScoringTargeting`, or raw Plants.
- Auto and TeleOp are parallel clients of the same `PhoenixCapabilities` vocabulary. Keep alliance,
  route selection, Pedro paths, and routine composition outside `PhoenixRobot`.
- Keep scoring's intent, execution policy, and hardware realization separated. Preserve the single
  owner for feed queues, target overlays, flywheel readiness, and scoring Plant update order.
- Keep targeting and drive-assist decisions in their robot-owned services; do not move season-specific
  scoring or aim policy into reusable framework lanes.
- Keep OpModes thin: choose configuration/specification, construct the runtime, forward FTC lifecycle
  calls, and clean up owned resources.

## Change workflow

1. Inspect the relevant implementation, Javadocs, Markdown, examples, and current callers before
   designing a change.
2. Make the smallest coherent change that preserves the ownership model; avoid unrelated cleanup.
3. Update affected documentation and examples in the same change as an API or behavior change.
4. Verify in proportion to risk with the narrowest relevant Gradle compile/test tasks and targeted
   static checks. Report anything that could not be verified on robot hardware.
5. Preserve existing user changes and avoid modifying generated, vendored, SDK sample, or legacy
   files unless the task explicitly requires it.

