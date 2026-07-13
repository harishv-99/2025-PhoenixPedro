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
- A timed Task or timed Task phase starts at its own `clock.nowSec()` boundary. Never charge it the
  `dtSec()` interval that elapsed before it started, and keep every positive-duration command
  observable by the documented downstream loop phase at least once.
- Treat every `Task` instance as single-use. A task may enter `start(clock)` once; framework tasks
  must fail fast on another start before repeating child, controller, or hardware side effects.
  Build a fresh task from a macro method, `Supplier<Task>`, or `OutputTaskFactory` whenever behavior
  should run again, and reject obvious duplicate child identities when composing task graphs.
- Keep Task cancellation active-only and idempotent: cancellation before `start(clock)` has no
  effect, active cancellation is terminal, and terminal/repeated cancellation does nothing. Reject
  direct updates before start; `Tasks.noop()` is the intentional terminal-at-construction exception.
  A total runner abort or lifecycle failure must best-effort cancel its current Task and clear
  pending work; never expose a drop-without-cancel queue operation.
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
  bounds, references, and guards. Direct power Plants own the normalized `[-1, +1]` target range.
  Reject static guard fallbacks outside any Plant's declared range and keep every final guarded
  target finite and inside that range. A feedback move must explicitly choose whether active
  cancellation writes a caller-selected target or leaves its persistent request unchanged. Do not
  introduce competing imperative writers or claim that Task cancellation bypasses the source graph.
- Prefer framework task factories (`Tasks`, `PlantTasks`, `DriveTasks`, guidance/route task helpers)
  over hand-written task state machines unless a new state machine is genuinely needed.
- Keep drive intent and actuation separate: `DriveSource` produces robot-centric `DriveSignal`s;
  overlays reshape selected components; a `DriveCommandSink` performs the final write.
- When a third-party follower's supported lifecycle requires updates beyond active route/guidance
  Tasks, give it one stable composition-root heartbeat throughout the relevant OpMode loop. Its
  adapter must deduplicate same-cycle Task/root calls, count vendor-hidden updates, and make
  `stop()` apply physical zero immediately (including after reentrant callbacks), rather than
  merely stage zero for later.
- Keep route completion truthful and scoped to one start. A route integration returns a per-start
  execution whose terminal status and cancellation cannot drift to a replacement route. Never infer
  success solely from a vendor's idle/not-busy flag; classify and retain endpoint completion,
  follower timeout/stall, interruption, replacement, failure, and unknown terminal state at the
  integration boundary.
- Build fixed routes eagerly. When route geometry depends on current pose, vision, or another live
  fact, resolve it exactly once through the explicitly named start-time Route Task factory. Keep
  construction quick and non-blocking, keep interpretation in robot-owned path code, and leave the
  follower untouched if construction fails.
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
