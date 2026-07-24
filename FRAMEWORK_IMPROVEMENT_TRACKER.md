# Framework Improvement Tracker

Last updated: 2026-07-23

This file tracks proposed Phoenix framework improvements. It is deliberately a planning document:
an item being listed here does **not** mean its current proposed solution has been approved. Each
item must pass the decision gate below before implementation begins.

## Design authority and goal

Every change must follow, in order:

1. The user's instructions for the specific change.
2. [`AGENTS.md`](AGENTS.md).
3. [`Framework Principles.md`](TeamCode/src/main/java/edu/ftcphoenix/fw/Framework%20Principles.md).
4. The relevant framework guide, Javadocs, examples, and current callers.

The goal is not merely to make framework internals more abstract. The goal is to make normal robot
code easy for students to read and create while the framework owns reusable complexity, safety,
lifecycle, timing, and hardware-boundary behavior.

Non-negotiable evaluation criteria:

- Keep the common robot call site short and obvious.
- Prefer one concept and one normal path over parallel convenience APIs.
- Do not add a generic `BaseRobot` or move season-specific vocabulary into the framework.
- Preserve one `LoopClock`, source-driven Plants, one final actuator writer, and explicit ownership.
- Fail early with a message that tells a student what to change.
- Add an abstraction only when it prevents a real misuse or removes repeated complexity.
- Keep implementation, Javadocs, Markdown guides, and compiling examples synchronized.

## Status legend

- **Proposed**: problem appears real; solution has not passed the decision gate.
- **Researching**: current behavior, callers, and alternatives are being checked.
- **Ready**: decision is recorded, scope is bounded, and tests are specified.
- **In progress**: implementation of this item only is underway.
- **Verifying**: code is complete and focused verification is running.
- **Done**: implementation, callers, documentation, and verification are complete.
- **Deferred**: deliberately postponed with a reason recorded.

## Hardware-unavailable execution policy

As of 2026-07-19, no representative robot hardware is available. Do not substitute fake-backed
tests, SDK source inspection, or configuration metadata for a physical measurement required to
select a production design or claim completion.

- `SOURCE-03`, `SENSOR-01`, `PERF-01`, `PERF-03`, `CHECK-01`, and `SAFE-04` are **Deferred** under
  their current completion contracts because each requires recorded signal, controller-performance,
  assembled-robot, or representative-hardware evidence.
- A later decision gate may propose a narrower conservative software contract that makes physical
  observation adopting-robot validation rather than completion evidence. That is a material design
  change and still requires explicit user approval under the evidence gate.
- Hardware that would merely increase confidence in an already testable software-seam contract does
  not block other items. Such work may proceed only when its completion claim explicitly stops at
  that seam and preserves physical validation as optional adoption evidence.
- `AUTO-01` is also **Deferred**, for a different reason: the second materially different bounded
  Auto caller needed to justify a reusable abstraction does not yet exist. Do not manufacture that
  caller merely to advance the tracker.
- While this policy is active, select the highest-priority item whose current behavior, design
  choice, and completion contract can all be established through source/caller inspection,
  deterministic tests, documentation, and compilation. Do not advance an item whose current
  production contract still depends on unavailable adopting-robot evidence.

## Required decision gate for every item

Before editing framework code, Codex must update that item's decision record with:

1. **Confirmed behavior**: reproduce the issue with a focused test or a minimal traced call path.
2. **Current callers**: find every in-repository caller and identify the student-facing common path.
3. **Alternatives considered**: include no change/documentation-only and the smallest local fix;
   consider a new interface or type only when it provides a meaningful correctness benefit.
4. **Simplicity comparison**: compare call-site length, number of concepts students must learn,
   discoverability, error quality, ownership clarity, and implementation complexity.
5. **Chosen design**: record why it is the smallest robust, principle-consistent approach.
6. **Rejected designs**: record why plausible alternatives were rejected so the same debate is not
   repeated without new evidence.
7. **Verification plan**: name the unit tests, compile checks, caller migrations, docs, and examples
   required for completion.

Only then may the item's status move to **Ready**. Implement one Ready item at a time. Do not include
adjacent cleanup unless it is required to keep the repository compiling and documented.

## Recommended implementation order

| Order | ID | Item | Status | Current leading hypothesis (not yet a decision) |
|---:|---|---|---|---|
| 1 | TEST-01 | Pure framework test harness | Done | Establish fake-clock and pure unit-test support before behavior changes. |
| 2 | PHX-01 | Phoenix shutdown ordering | Done | Make shutdown one idempotent, correctly ordered owner operation. |
| 3 | SAFE-01 | Final Plant target invariant | Done | Validate finite/range-safe output after all dynamic guards. |
| 4 | SAFE-02 | Power Plant applied-target truth | Done | Clamp normalized power in the Plant and report the clamp. |
| 5 | TASK-01 | Timed-task start semantics | Done | Measure elapsed time from task start, not the preceding loop interval. |
| 6 | TASK-02 | Task reuse contract | Done | Enforce the documented single-use contract consistently. |
| 7 | TASK-03 | Clear and cancellation semantics | Done | Remove unsafe drop-without-cancel behavior and make Plant cancellation explicit. |
| 8 | PEDRO-01 | Follower heartbeat and stopped-state ownership | Done | Advance one Pedro follower once per Auto loop outside individual Tasks; Tasks select behavior but do not own the heartbeat. |
| 9 | PEDRO-02 | Pedro drivetrain, localization, and pose authority | Done | Build one valid Pedro runtime with one hardware/localization owner and an explicit Pedro-to-Phoenix field-pose contract. |
| 10 | ROUTE-02 | Truthful route terminal status | Done | Do not infer route success solely from `!isBusy()`; preserve completion, interruption, timeout, and failure meaning. |
| 11 | ROUTE-01 | Start-time route construction | Done | Resolve a route once when its Task starts so live pose and vision can select geometry safely. |
| 12 | FIELD-01 | Explicit alliance field transforms | Deferred | Revisit with real routes: share only genuinely symmetric geometry, while keeping alliance-specific points and complete routes explicit. |
| 13 | DRIVE-01 | Drive task ownership | Done | Keep an exclusive Auto-only timed sink Task, after Pedro's outer-loop ownership is made correct. |
| 14 | TASK-04 | Parallel deadline composition | Done | Add one cancellation-safe deadline primitive only if route-plus-companion callers justify it. |
| 15 | PHX-03 | Explicit Auto route-failure policy | Done | Keep generic sequence semantics and make continue/fallback/abort policy visible in the robot routine. |
| 16 | PHX-04 | Match-time fallback takeover | Done | Bound only pre-park work, then start one live-pose park after early completion or a successful match-time cutoff. |
| 17 | PHX-02 | Phoenix runtime readiness | Done | Validate calibration, Pedro construction, routes, alliance facts, and required services before enabling assists/Auto. |
| 18 | EXAMPLE-02 | Compiling Pedro autonomous reference | Done | Show one small real path, capability Tasks, one follower heartbeat, explicit fallback, and a thin OpMode. |
| 19 | CTRL-01 | Final scalar-regulator output constraints | Done | Constrain the final composed regulator command with one factory-only decorator, without adding flywheel policy or another Plant-builder path. |
| 20 | SAFE-03 | Regulated Plant actuator-command truth | Done | Make every normalized regulated-Plant command finite, bounded, and truthful before the defensive hardware adapter. |
| 21 | SOURCE-02 | Derived rate from sampled scalar position | Done | Derive units-per-second from any position `ScalarSource` in core; keep encoder hardware reads at the FTC boundary. |
| 22 | FTC-01 | FTC raw motor-power run-mode ownership | Done | Make the FTC motor-power boundary consistently own `RUN_WITHOUT_ENCODER` without another student-facing choice. |
| 23 | API-03 | PID and linear-PIDF configuration ownership | Done | Keep plain PID error-centric and add one factory-only standard PIDF regulator that owns all four gains and live tuning as one validated update. |
| 24 | TUNE-01 | Live tuning to checked-in profile | Done | Document one explicit test-mode PIDF tune/apply/record workflow; add no tuner API because API-03 exposes the complete gain update and realization already owns outer reset. |
| 25 | ROUTE-03 | Factory-only route Task configuration | Done | Use one named factory-only layer with direct bounded or explicit no-Task-timeout policy. |
| 26 | ACT-01 | FTC actuator-group identity validation | Done | Private SDK-equivalent actuator/mecanum validation reviewed, verified, and approved on 2026-07-17. |
| 27 | COMMON-01 | Cleanup action aggregation | Done | The stateless cleanup-action primitive and five bounded migrations are implemented, verified, and approved; the generic INIT runtime remains deferred. |
| 28 | TESTER-01 | Tester child lifecycle fail-stop | Done | Approved fail-stop policies are implemented, verified, and approved without changing valid public call sites. |
| 29 | AUTO-01 | Compact bounded Auto continuation | Deferred | Wait for a second materially different real bounded-Auto caller; do not infer an API from PHX-04 alone. |
| 30 | SOURCE-03 | Composable scalar measurement conditioning | Deferred | Wait for recorded signal traces before choosing a public filtering algorithm or latency contract. |
| 31 | MATCH-01 | Explicit Auto-to-TeleOp handoff | Done | Complete; physical pose accuracy remains adopting-robot validation rather than a software-contract claim. |
| 32 | DRIVE-02 | Shared drivetrain actuator handoff | Deferred | Wait for a real PTO-equipped adopting robot; preserve the approved single-owner design constraints without inventing hardware-specific APIs. |
| 33 | VISION-01 | Shared webcam and Limelight vision ownership | Done | Parallel webcam/Limelight ownership, readiness, lifecycle recovery, migrations, documentation, and tests were approved on 2026-07-20. |
| 34 | SENSOR-01 | Motor-current sensing | Deferred | Current completion requires controller polling measurements; revisit via a narrower conservative seam only through a new approved decision gate. |
| 35 | INPUT-01 | Safe contextual control activation | Done | Approved robot-first contextual controls, shared registration surface, lifecycle-safe tester handoffs, and regression coverage. |
| 36 | HAPTIC-01 | Driver haptic feedback boundary | Done | Approved fixed-pulse sink, truthful FTC conversion, focused coverage, and full software verification; physical rumble behavior remains adopting-robot validation. |
| 37 | PERF-01 | FTC hub bulk-cache ownership | Deferred | Wait for real hub-I/O benchmarks before changing the SDK automatic-caching default. |
| 38 | PERF-02 | Loop phase diagnostics | Done | Approved and reviewed sequential profiler, bounded atomic statistics, off-by-default Phoenix telemetry integration, focused/full verification; enabled hardware overhead remains adopting-robot validation. |
| 39 | PERF-03 | Contract-safe hardware write deduplication | Deferred | Wait for per-adapter measurements and controller/watchdog evidence before suppressing writes. |
| 40 | TARGET-01 | Lazy Plant target overlay selection | Done | Two-pass lazy target resolution, measured-hold re-entry, truthful diagnostics, documentation, and 22 focused tests are complete and approved. |
| 41 | TARGET-02 | Candidate freshness | Done | Timestamp-canonical observed/timeless APIs, derived age, fail-closed metadata handling, diagnostics, docs, and tests were reviewed and approved on 2026-07-23. |
| 42 | TIME-01 | Epoch-safe LoopClock timestamps | Done | Typed timestamps, reset-safe portable timing, monotonic cycle identity, caller migration, documentation, software verification, and Android Studio review are complete. |
| 43 | VISION-02 | Stable AprilTag observation timestamps | Done | Stable backend-owned frame timestamps, exact propagation, fail-closed reset/replay handling, synchronized documentation, verification, and Android Studio review are complete. |
| 44 | TARGET-03 | Periodic planner complexity | Proposed | Replace range iteration with constant-time candidate mathematics. |
| 45 | CYCLE-01 | Stateful drive-source cycle safety | Proposed | Memoize stateful composition once per `clock.cycle()` and propagate reset deliberately. |
| 46 | CYCLE-02 | Localization cycle safety | Proposed | Guard predictors/estimators against duplicate same-cycle updates. |
| 47 | SOURCE-01 | Boolean composition sampling | Proposed | Sample both operands once per cycle before combining stateful results. |
| 48 | API-01 | Writable Plant command binding | Proposed | Keep one simple `Plant` if builder provenance can prevent silent no-op writes. |
| 49 | API-02 | Feedback tolerance choice | Proposed | Ask for tolerance after unit mapping; retain an explicitly named native default only if useful. |
| 50 | API-04 | Binding execution order | Proposed | Preserve declaration order unless explicit phases are proven necessary. |
| 51 | API-05 | One beginner drive entry point | Proposed | Teach the lane as the robot-facing path and keep the raw factory as a lower-level tool. |
| 52 | COMMON-02 | Telemetry commit ownership | Proposed | Renderers add data; the composition root commits once. |
| 53 | CHECK-01 | Staged whole-robot system check | Deferred | Meaningful Phoenix thresholds, hazardous-motion confirmation, and physical safe-state evidence require the assembled robot. |
| 54 | EXAMPLE-01 | Compiling modern starter robot | Proposed | Add a small multi-file reference, not an inheritance framework. |
| 55 | EXAMPLE-03 | Advanced moving-target reference | Proposed | Prove progress-triggered scoring and a bounded moving turret without putting game physics in the framework. |
| 56 | BOUNDARY-01 | FTC boundary enforcement | Proposed | Fix existing import leaks, then add a focused forbidden-import check. |
| 57 | DOC-01 | Stale and non-compiling documentation | Proposed | Correct loop/API examples and validate links/examples where practical. |
| 58 | CI-01 | Framework verification in CI | Proposed | Run focused unit tests, TeamCode compilation, docs checks, and boundary checks. |
| 59 | CLEAN-01 | Alias and risky convenience cleanup | Proposed | Remove only APIs proven redundant or unsafe by caller search. |
| 60 | SAFE-04 | PowerOutput failure cleanup and seam truth | Deferred | Current completion requires representative actuator observation; a narrower software-seam contract needs a new approved decision gate. |
| 61 | CAL-01 | Calibration-search power validation | Proposed | Reject invalid normalized search power before stopping normal output or changing calibration state. |
| 62 | CAL-02 | Position-calibration reference validity | Proposed | Validate calibration reference and hold answers at the boundary that owns their units and lifecycle. |
| 63 | MAP-01 | FTC actuator mapping-domain validation | Proposed | Validate finite child transforms and raw actuator domains before command mapping can be silently clamped. |
| 64 | RANGE-01 | ScalarRange construction validity | Proposed | Define and enforce finite, half-bounded, and unbounded range construction without allowing `NaN`. |
| 65 | FTC-02 | Device-managed controller configuration validation | Proposed | Validate FTC PIDF/P, maximum-power, and related staged answers before SDK access or mode changes. |
| 66 | CONFIG-01 | Owner-configuration snapshot audit | Deferred | Revisit specific owners only when a traced caller shows mutation drift or invalid retained state. |

The completed order was intentionally front-loaded with testability, robot lifecycle, actuator
safety, deterministic Task behavior, Pedro ownership, truthful route outcomes, and the reusable
pieces already proven by Bettabot's shooter. The remaining order was re-audited on 2026-07-17
against the still-current Summer26/Bettabot commit `4eed9d6c` and the then-current,
pre-COMMON-01 626-line Pedro reference rather than against short outer calls.

The next decision-gate cluster selected on 2026-07-17 was TUNE-01, ROUTE-03, ACT-01, and COMMON-01.
TUNE-01 had the
largest potential shooter payoff after API-03, but it must prove a net reduction in all tuner and
OpMode robot code rather than merely introduce a short framework call. ROUTE-03 split the
factory/configuration portion out of broad CLEAN-01 because the pre-change simple route needed a
mutable one-field `RouteTask.Config`, nullable defaults, and a public construction family that
duplicates the `RouteTasks` facade. ACT-01 is a small but concrete Bettabot simplification:
`FtcActuators` already knows the complete two-motor group and can own blank/duplicate identity
validation before lookup, allowing an adopting Bettabot to delete its duplicate checks. COMMON-01
is promoted to measure the repeated lifecycle shell exposed by EXAMPLE-02; it must not turn that
evidence into a base OpMode or hide loop order.

AUTO-01 and SOURCE-03 remain high-value but explicitly evidence-gated. AUTO-01 needs a second
materially different bounded routine—ideally Bettabot's first real route—before extracting a
continuation abstraction. SOURCE-03 now has a second real flywheel caller in Cuttlefish, but still
needs representative traces before selecting a filter whose delay could help or harm PIDF control.
They follow the actionable research cluster instead of pretending missing physical/routine evidence
is implementation-ready. The remaining unrelated items retain their prior relative order.

The Pedro review added two runtime-ownership gates before DRIVE-01:
the checked-in Auto must first have one continuous follower heartbeat and one valid drivetrain/
localization authority. The Cuttlefish review then added route truth and deferred construction ahead
of the higher-level Auto policies that depend on them. The expanded Worlds-source review added one
mode-boundary handoff and one missing FTC sensor primitive, while deliberately leaving robot power,
jam, calibration, and match strategy out of the framework. The Bettabot review then separated an
intentional final control-command constraint from universal regulated-output safety, and separated
generic rate estimation from optional signal conditioning rather than naming either source
abstraction after one encoder. Later API cleanup should not begin until tests protect the core
semantics it depends on.

## External competition capability benchmark (2026-07-11)

The tracker was re-reviewed against the public Worlds-season source for
[6165 MSET Cuttlefish Decode at commit `1a9ff399`](https://github.com/6165-MSET-Cuttlefish/Decode/tree/1a9ff399298a95639c08daf0434463d9b035d383),
not just against framework style preferences. The team's
[official 2025 results page](https://ftc-events.firstinspires.org/2025/team/6165) confirms that this
is competition-proven code worth treating as a capability benchmark. The goal is not to copy its
architecture or season vocabulary. The goal is for Phoenix to express the same useful robot
behaviors with fewer lifecycle hazards and a smaller student-facing programming surface.

| Cuttlefish capability/evidence | Phoenix assessment | Tracker consequence |
|---|---|---|
| One configured Pedro `Follower` is built with mecanum and Pinpoint configuration, then advanced by the outer OpMode loop independently of route scheduling ([Constants](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/Constants.java), [EnhancedOpMode](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/EnhancedOpMode.java)). | This validates Phoenix's intended single-owner direction, but the checked-in Phoenix Pedro runtime does not yet satisfy it. | Keep PEDRO-01 and PEDRO-02 ahead of DRIVE-01. |
| Routes can be constructed when execution begins from the live follower pose and current vision selection ([PathActionBuilder](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/pathaction/PathActionBuilder.java), [Far Auto](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Far.java)). | Phoenix's eager `RouteTask` cannot naturally express live start geometry without rebuilding the surrounding routine. | Add ROUTE-01. |
| Route progress callbacks coordinate intake, turret, and scoring, while the scheduler also handles replacement, built-in path interruption, and manual skipping ([Close Auto](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Close.java), [PathActionScheduler](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/pathaction/PathActionScheduler.java)). | Pedro already supplies useful progress callbacks, and Phoenix Tasks can own companion work. The real gap is that `!isBusy()` alone cannot distinguish success from interruption or failure. | Add ROUTE-02; retain TASK-04 and PHX-03. Do not add a second scheduler or generic callback DSL. |
| A global match-time rule can preempt whatever Auto phase is active and start a park route from the current pose ([Far Auto](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Far.java)). | A route-local branch or deadline child does not by itself express one bounded pre-park phase followed by one live-pose park cleanly. | Add PHX-04 as robot-owned policy around one generic timeout decorator and one root Task graph. |
| Red/blue geometry is derived from one field-pose definition ([FieldPose](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/FieldPose.java)). | Phoenix currently makes alliance-specific path duplication too attractive. | Add FIELD-01, with Phoenix coordinates and no global alliance state. |
| A PTO reuses drivetrain motors and encoders for the lift/endgame, with explicit mode changes and TeleOp drive suppression ([RobotActions](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/RobotActions.java), [Endgame](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Endgame.java)). | The standard Phoenix drive lane hides the shared actuator/readback seam, so a robot can do this only by bypassing normal ownership. This is distinct from timed drive Tasks. | Add DRIVE-02; do not broaden DRIVE-01 into resource arbitration. |
| Layered controls suppress held-button edges on activation and add rate-limited gamepad rumble ([TeleOp](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tele/Tele.java), [LayeredGamepad](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/input/LayeredGamepad.java)). | Flat Phoenix controls remain the beginner default, but advanced robots need optional safe contextual activation and a separate driver-feedback output boundary. | Add INPUT-01 and HAPTIC-01. |
| Custom vision processors, manual bulk-cache clearing, output write caching, phase profiling, Dashboard tuning, and a staged whole-robot check are all used in normal development ([ClusterDetectionProcessor](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/ClusterDetectionProcessor.java), [WriteCache](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/hardware/WriteCache.java), [LoopProfiler](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LoopProfiler.java), [SystemCheck](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/SystemCheck.java)). | Phoenix has useful device testers and stable profile snapshots, but lacks reusable ownership for custom VisionPortal lifecycles, measured loop diagnostics, a safe live-to-checked-in tuning path, and a staged robot check. | Add VISION-01, PERF-01, PERF-02, PERF-03, TUNE-01, and CHECK-01 as separate decision gates. |
| Moving-target turret lead and magazine sorting combine pose/velocity/sensor facts with bounded hardware realization ([Context](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Context.java), [Turret](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Turret.java)). | Existing Phoenix spatial queries, timestamped sources, robot-owned services/supervisors, capabilities, and bounded Plants can express these behaviors cleanly. The missing piece is a compact compiling proof. | Add EXAMPLE-03; do not add projectile physics, game-piece sorting, or scoring vocabulary to the framework. |

Cuttlefish's asynchronous action helper runs robot behavior on background coroutines. Phoenix should
not copy that execution model: cooperative Tasks on the OpMode heartbeat preserve FTC hardware
thread ownership and make cancellation and loop order visible. Its standalone `DecodeSim` also
duplicates a simplified robot rather than executing the production ownership graph, so simulator
work remains deferred until a fake hardware boundary can exercise real Phoenix composition.

### Additional public Worlds-source review

FIRST does not publish one objective first-through-fifth robot ranking. For a reproducible "top"
set, this review first used the official 2026 Championship
[winning and finalist alliance awards](https://ftc-events.firstinspires.org/2025/FTCCMP1/awards).
The two fielded finalist pairs are also visible in
[Final 1](https://ftc-events.firstinspires.org/2025/FTCCMP1/playoff/16/1) and
[Final 2](https://ftc-events.firstinspires.org/2025/FTCCMP1/playoff/16/2). A focused GitHub and team-
site search did not find public, season-matching DECODE robot source for the six awarded alliance
teams (30030, 21087, 11228, 18270, 20265, and 7172). The tracker therefore does not pretend that a
different public repository belongs to a championship finalist.

The code comparison instead uses three additional public repositories with exact snapshots from
the Worlds event, selected for verifiable Worlds performance and usable production source. These
are evidence-bearing public-source benchmarks, not a replacement placement ranking:

| Public Worlds source | Student-facing shape and competition evidence | Phoenix consequence |
|---|---|---|
| [21936 SaMoTech at its final Worlds commit `c8ff047c`](https://github.com/SaMoTechRobotics/FTC-Decode/tree/c8ff047c4245f79d934e622c05481269b5cc42da), ranked 10th at [Edison](https://ftc-events.firstinspires.org/2025/FTCCMP1EDIS/rankings) | Named routines are readable arrays of semantic [`AutoStep`s](https://github.com/SaMoTechRobotics/FTC-Decode/blob/c8ff047c4245f79d934e622c05481269b5cc42da/TeamCode/src/main/java/auto/config/AutoStep.java), and live vision/pose can generate the next pickup path. Internally, [`BaseAuto`](https://github.com/SaMoTechRobotics/FTC-Decode/blob/c8ff047c4245f79d934e622c05481269b5cc42da/TeamCode/src/main/java/auto/config/BaseAuto.java) runs blocking helpers on a background thread, while [`Robot`](https://github.com/SaMoTechRobotics/FTC-Decode/blob/c8ff047c4245f79d934e622c05481269b5cc42da/TeamCode/src/main/java/robot/Robot.java) owns additional blocking loops and timer callbacks. | Preserve the short semantic plan through capability/route Tasks, ROUTE-01/02, PHX-03/04, and PERF-02. Do not copy the thread, blocking loops, timer callbacks, or a season-wide Auto DSL. |
| [19411 Tech Tigers at Worlds commit `0bec90bb`](https://github.com/techtigers-ftc/decode/tree/0bec90bb2523368c122d8d8361b56261d85cdfe2), ranked 14th at [Jackson](https://ftc-events.firstinspires.org/2025/FTCCMP1JACK/rankings) | A leaf Auto such as [`RedNearTripleGate`](https://github.com/techtigers-ftc/decode/blob/0bec90bb2523368c122d8d8361b56261d85cdfe2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/RedNearTripleGate.java) supplies a compact list of cycle facts. That simplicity hides a large [`AutoOpMode`](https://github.com/techtigers-ftc/decode/blob/0bec90bb2523368c122d8d8361b56261d85cdfe2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutoOpMode.java) and fixed-shape [`StateMachineBuilder`](https://github.com/techtigers-ftc/decode/blob/0bec90bb2523368c122d8d8361b56261d85cdfe2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configurators/StateMachineBuilder.java). | Phoenix should keep the small specification but express the graph with normal route and capability Task factories. This independently supports DRIVE-01, TASK-04, route outcome policy, whole-routine park policy, FIELD-01, performance tooling, and system checks; it does not justify a base OpMode or generic state-machine language. |
| [25609 LOAD Robotics at Worlds commit `35a74bfb`](https://github.com/LOAD-Robotics/Decode-Robot-Code/tree/35a74bfba7d5cc9eeede5ecfdd71123869bd4f17), ranked 16th at [Goodall](https://ftc-events.firstinspires.org/2025/FTCCMP1GOOD/rankings) | [`Auto_Main_`](https://github.com/LOAD-Robotics/Decode-Robot-Code/blob/35a74bfba7d5cc9eeede5ecfdd71123869bd4f17/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOADCode/Main_/Auto_/Auto_Main_.java) uses especially readable Pedro/NextFTC semantic sequences and selectable partner-specific plans. [`Commands`](https://github.com/LOAD-Robotics/Decode-Robot-Code/blob/35a74bfba7d5cc9eeede5ecfdd71123869bd4f17/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOADCode/Main_/Hardware_/Commands.java) also races routes against timeout/stall and constructs a late park after a global time limit. | This is close to the desired Phoenix routine surface: one init specification plus a readable `Tasks.sequence(...)` of route and capability factories. Truthful route results, start-time park geometry, one deadline primitive, and one Auto supervisor should replace loose race semantics without adding NextFTC/Ivy beside Phoenix Tasks. |

Across all three new repositories, motor current is a first-class behavior input: SaMoTech estimates
intake state from it; LOAD detects intake jams and protects turret homing; Tech Tigers observes
subsystem load before robot-owned drive derating. Phoenix documentation already names current spikes
as useful sensor facts, but `FtcSensors` has no current source. That repeated gap adds SENSOR-01. It
does **not** justify a universal power manager: thresholds, priorities, jam recovery, homing, and
brownout policy remain robot-owned.

All three also deliberately carry match state from Auto into TeleOp: pose in every case, plus turret
offset, calibration, game-piece inventory, motif, or alliance where useful. Their static fields or
string-keyed global stores work, but invite stale values and casts. That repeated mode-boundary need
adds MATCH-01 for one typed immutable handoff, while the snapshot fields remain robot-specific.

The expanded review also reinforces TASK-04, ROUTE-01/02, PHX-03/04, FIELD-01, PERF-02, TARGET-02,
CYCLE-02, CHECK-01, and EXAMPLE-02/03. One LOAD articulated-camera localization implementation is
not enough evidence for another framework lane; revisit dynamic localization camera mounts only if
a second concrete robot cannot use the existing time-aware mount and estimator boundaries cleanly.
No reviewed code justifies a second scheduler, generic command requirements, global mutable robot
state, a generated Auto language, or background hardware behavior.

### Bettabot shooter review (2026-07-15)

The tracker was also reviewed against every Java file under `edu.ftcphoenix.robots.betta` in
[Hansika1098/Summer26 at commit `4eed9d6c`](https://github.com/Hansika1098/Summer26/tree/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta).
All 1,047 lines count as robot code for this assessment, including tuning, telemetry, composition,
controls, and unused Auto scaffolding; a short outer Plant call does not by itself make the complete
robot implementation simple.

| Bettabot evidence | Phoenix assessment | Tracker consequence |
|---|---|---|
| [`CappedFlywheelRegulator`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L77-L136) wraps the complete PID-plus-feedforward result because `Pid` output limits apply before the feedforward decorator. | An intentional narrower command range is a reusable control-law composition operation. It does not belong in a flywheel class, FTC motor adapter, or another regulated-Plant builder branch. Zero-target coast/reset remains robot policy. Separately, every normalized regulated output needs truthful universal `[-1,+1]` safety even when a caller does not request a narrower range. | Add CTRL-01 for one factory-only policy constraint and SAFE-03 for universal regulated-command safety/truth; explicitly evaluate composition order and anti-windup implications. |
| [`ThroughBoreVelocitySource`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L30-L75) directly reads FTC position and differences consecutive samples to avoid a reported high-rate velocity-counter problem. | Hardware acquisition belongs in `fw.ftc`, but position-to-rate estimation is generic core Source logic that can consume motor, external, analog, absolute, or simulated position sources. Filtering is another generic transform and must not be hidden inside an encoder model or an automatic velocity fallback. | Add SOURCE-02 for generic derived-rate estimation and SOURCE-03 for independently composable scalar conditioning. Require hardware comparison against direct SDK velocity before changing the recommended encoder-feedback path. |
| [`BettaDashboardControls`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaDashboardControls.java#L18-L100) synchronizes mutable live values, while [`BettaShooter`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L320-L361) repeats extensive configuration validation. | Live tuning is useful but should not become production configuration authority. Repeated finite/range/name validation should fail at the earliest fully informed framework or robot-owner boundary. | Add Bettabot evidence to TUNE-01 and broaden API-03's validation audit; do not add a framework shooter, tuning singleton, or generic robot base. |
| [`BettaAutoTasks`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/autonomous/BettaAutoTasks.java) contains only three one-shot shooter requests, the [Betta Pedro guide](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/autonomous/pedro/README.md) describes future wiring, and [`PedroTest`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/PedroTest.java) is entirely commented out with an empty state update. | Bettabot has no enabled production Auto to copy or evaluate. The next useful proof is a small non-Phoenix consumer of the completed Pedro runtime, Route Tasks, capability Tasks, truthful fallback, and one visible heartbeat—not another handwritten state machine or a copy of Phoenix's season graph. | Prioritize EXAMPLE-02 first and require its decision gate to compare the complete example robot-code surface against Bettabot's current scaffold. Reassess AUTO-01 only after this example and a materially different real bounded routine provide its required evidence. |

The intended measurement layering is explicit: an FTC adapter acquires and memoizes a raw hardware
reading; core Sources perform reusable unit mapping, position-to-rate estimation, and optional
filtering; a Plant or robot service consumes the resulting Source; robot code owns what the value
means and how behavior responds. SOURCE-02 and SOURCE-03 remain independent decision gates:
SOURCE-02 must not depend on optional filtering, while SOURCE-03 may decorate either raw or derived
measurements. Each item must audit the existing `ScalarSource` construction layer independently and
avoid adding redundant factory and instance-method spellings.

Summer26 currently carries an older copied framework boundary: its
[`PedroPathingDriveAdapter`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/fw/integrations/pedro/PedroPathingDriveAdapter.java)
predates the completed PEDRO-01/02 and ROUTE-01/02 ownership and outcome work. Before a production
Bettabot Auto is enabled, that downstream copy must be synchronized with the current framework
baseline. This is an adoption prerequisite, not evidence for a second Pedro API or a new framework
distribution abstraction from one consumer. If another robot repeats material copy/sync drift,
open a separate decision gate for the smallest supported distribution path.

### Bettabot-focused remaining-work audit (2026-07-17)

Summer26 `master` was rechecked and still points to the pinned `4eed9d6c` revision used above. The
completed CTRL-01, SAFE-03, SOURCE-02, FTC-01, and API-03 work already owns most reusable PIDF
shooter complexity; the completed Pedro/Route/Task/PHX/EXAMPLE-02 sequence already establishes the
safe route foundation. The remaining priority therefore targets concrete residual robot code,
without reopening those designs:

| Priority | Item | Concrete Bettabot benefit | Gate that remains |
|---:|---|---|---|
| 1 | TUNE-01 | Potentially replaces the roughly 100-line two-way Dashboard synchronization owner with one explicit four-gain tune, validate, apply/reset, and record workflow. | Compare the complete tuner plus OpMode surface after API-03 against a small robot-local solution. This repository currently uses Pedro Panels and has no FTC Dashboard dependency; do not add a vendor dependency or generic tuning language unless total robot code and concepts decrease. |
| 2 | ROUTE-03 | Makes each fixed or start-time Pedro route choose its name, route source, follower, and timeout directly through one factory layer instead of constructing a mutable one-field config or discovering parallel constructors. | Audit every factory, constructor, default/named overload, config caller, and return type. This is a major public-API decision and requires approval before implementation. |
| 3 | ACT-01 | Would let an adopting Bettabot delete blank and duplicate left/right flywheel motor-name checks after the fully informed grouped-actuator boundary owns them, without changing the valid staged call. | Preserve intentional external-feedback reuse of a powered motor's own port and avoid a global hardware registry. |
| 4 | COMMON-01 | The pre-COMMON-01 independent Pedro reference proved that 426 of its 626 source lines were in the composition root and FTC host rather than paths, capabilities, or the semantic routine. | Separate genuinely repeated INIT/retry/partial-cleanup mechanics from required robot ownership. Split another narrowly named item if recurring Auto-root lifecycle is proven but does not fit this initialization scope. |
| 5 | AUTO-01 | Could prevent copying Phoenix's large bounded-pre-park continuation state machine when Bettabot adds a match-time park policy. | EXAMPLE-02 supplies one reference, but a second materially different bounded routine is still required. A simple one-route Auto should not wait for this item or justify an Auto DSL. |
| 6 | SOURCE-03 | Could improve position-derived flywheel feedback and avoid a future robot-local filter. | Cuttlefish supplies the second real caller, not the correct algorithm. Require real traces and compare noise rejection against control delay under irregular loop periods; never add implicit smoothing. |

**Manual review (2026-07-17):** the user approved this revised Bettabot-focused priority order
with “Priority order looks good.” This authorizes publication of this tracker-only plan; it does
not approve any proposed item design or start TUNE-01.

SAFE-04 remains useful grouped-output failure safety, but it removes no Bettabot code and requires
representative physical validation. API-02 already matches Bettabot's explicit RPM tolerance call;
MAP-01 targets mappings Bettabot does not use; FTC-02 owns device-managed SDK PIDF rather than
Bettabot's external-feedback software PIDF. PERF/CHECK/DOC/CI work can improve diagnosis, readiness,
and maintenance later, but none is a more direct next simplification. FIELD-01 and EXAMPLE-03 remain
advanced geometry/moving-target work rather than prerequisites for a simple first route.

### Recommended autonomous ownership structure

The competition review supports this student-facing Auto shape after its prerequisite items land:

1. A thin OpMode/specification chooses alliance, start, routine, delay, and explicitly allowed
   fallback policy. It constructs the runtime, forwards FTC lifecycle calls, and closes it.
2. One Pedro integration owner builds the configured Follower, owns Pinpoint/localization and the
   Phoenix-coordinate pose/motion bridge, advances Pedro once per loop, and performs the final drive
   stop. Routine code never owns another Follower heartbeat.
3. A robot path factory owns field geometry and Pedro types. It can return fixed routes or a
   start-time supplier using current pose/vision. Pedro parametric callbacks may request a narrow
   robot capability; they do not write mechanisms or run a second scheduler.
4. Robot capability families provide fresh single-use Tasks and persistent mode-neutral intent for
   intake, scoring, endgame, and other mechanisms. Services/supervisors own season calculations;
   source-driven Plants remain the final bounded hardware realization.
5. A robot routine factory composes readable `Tasks.sequence(...)`, the justified deadline helper,
   route Tasks, capability Tasks, and explicit route-outcome branches. It contains strategy but no
   hardware-map calls, raw motors, or follower update loop.
6. One robot Auto supervisor observes the shared match budget and can perform the explicit
   cancel -> safe-capabilities -> fresh deferred park takeover. It uses the same TaskRunner rather
   than another command system.
7. At mode transition, the Auto composition root may publish one immutable robot-defined handoff
   snapshot before owned resources close. TeleOp consumes it once or uses its explicit configured
   fallback; no hardware object, Task, follower, or vendor pose crosses the boundary.

The canonical active loop remains visible: advance `LoopClock` once; clear optional manual bulk
caches; sample inputs and advance the one Pedro/localization owner once; update robot services and
the TaskRunner; realize each Plant once in documented order; render diagnostics; commit telemetry
once. This keeps
the simple routine call site while preserving the Framework Principles' single heartbeat, one final
writer, and explicit lifecycle ownership.

## Task definitions and decision records

### TEST-01 - Pure framework test harness

- **Problem to confirm:** the framework has substantial pure stateful logic but no focused unit-test
  suite or fake-clock regression coverage.
- **Decision question:** can tests live cleanly under `TeamCode/src/test`, or would a small pure-Java
  module materially reduce Android/FTC friction without increasing repository complexity?
- **Leading hypothesis:** start in `TeamCode/src/test`; extract a module only if the Android test
  environment prevents fast pure tests.
- **Completion:** fake-clock fixture plus representative tests compile and run; test commands are
  documented. No production behavior changes.
- **Decision record (2026-07-10):**
  - **Confirmed behavior:** `TeamCode` had no local unit-test source set content or test dependency.
    Its pure framework packages contain deterministic logic that can be exercised without a robot,
    but there was no executable regression harness for it.
  - **Current callers:** this change does not migrate production or student robot callers. Tests
    exercise the existing framework APIs through their public contracts.
  - **Alternatives considered:** no change/documentation-only, a custom assertion runner, Android
    instrumentation tests, JUnit 5, Robolectric/mocking, local JUnit 4 tests under `TeamCode/src/test`,
    and a new pure-Java framework module.
  - **Simplicity comparison:** local JUnit 4 tests add one test-scoped dependency and use Android
    Studio's standard gutter/Gradle test workflow. A new module would require moving or duplicating
    production sources and introduce a repository boundary unrelated to robot code. Instrumentation
    would require a device; JUnit 5, Robolectric, mocks, or a custom runner add setup without helping
    these pure tests.
  - **Chosen design:** use JUnit 4.13.2 in `TeamCode/src/test`. Add a test-only controlled-clock
    fixture that advances the real final `LoopClock`; do not change the production clock API merely
    for testing. Add narrow representative tests for clock behavior, source memoization, and
    `TaskRunner` same-cycle idempotency.
  - **Rejected designs:** defer a pure-Java module until measured test latency/dependency friction or
    the independently reviewed FTC-boundary work justifies it. Do not enable Android stub default
    returns because that could hide an accidental SDK dependency in code intended to be pure. Do not
    encode known disputed behavior from later tracker items as expected behavior in this baseline.
  - **Verification plan:** run `:TeamCode:testDebugUnitTest`, confirm the new tests are discoverable in
    Android Studio, document the command in maintainer guidance, and verify that no production Java
    file changes as part of TEST-01.
  - **Automated verification:** `:TeamCode:testDebugUnitTest` passed on 2026-07-10 with three tests,
    zero failures, and zero errors. The scope check found no production Java changes. Android Studio
    inspection was confirmed by the user on 2026-07-10.

### PHX-01 - Phoenix shutdown ordering

- **Problem to confirm:** shared shutdown clears `scoringPath` before the later general stop can stop
  it, potentially leaving owned behavior active.
- **Decision question:** should Phoenix expose one idempotent `stop()`/`close()` operation, or can the
  existing mode-specific methods be made safe without duplicating ordering?
- **Leading hypothesis:** one idempotent owner-level shutdown is simpler and harder for thin OpModes
  to call incorrectly.
- **Completion:** every initialized resource is stopped exactly once for TeleOp, Auto, partial init,
  repeated stop, and init failure.
- **Decision record (2026-07-10, approved):**
  - **Confirmed behavior:** both `PhoenixTeleOp.stop()` and the Pedro Auto cleanup call the
    mode-specific stop first. That stop clears `scoringPath`, so the following `stopAny()` cannot
    call `ScoringPath.stop()`. This affects normal TeleOp, normal Auto, and Auto initialization
    failures that occur after Phoenix construction but during follower/path/routine setup.
  - **Current callers:** only `PhoenixTeleOp` and `PhoenixPedroAutoOpModeBase` use the three public
    Phoenix stop methods. The Pedro drive adapter remains owned and stopped separately by the Auto
    mode client.
  - **Alternatives considered:** reverse the two calls; move scoring cleanup into
    `stopSharedRuntime()`; make both mode-specific stops comprehensive; keep public lifecycle phases
    with stricter documentation; add a mode enum, cleanup registry, `AutoCloseable`, or framework
    lifecycle abstraction; or expose one complete `stop()` operation.
  - **Simplicity comparison:** reversing calls or repairing the shared helper leaves an order-sensitive
    two-call protocol that future thin OpModes can repeat incorrectly. Two comprehensive mode methods
    duplicate one object-lifetime operation and make partial initialization choose a mode. A mode
    enum, public lifecycle interface, cleanup registry, or `close()` alias adds concepts without a
    second lifetime. One `stop()` matches FTC vocabulary and gives students one obvious call.
  - **Chosen design:** replace `stopTeleOp()`, `stopAuto()`, and `stopAny()` with one public,
    idempotent `PhoenixRobot.stop()`. Internally cancel the Auto runner and TeleOp behavior owners,
    stop scoring and drive sinks while their references are still available, reset targeting, close
    vision last, and clear reference-only state. Move each field to a local and null it before its
    cleanup call so repeated/reentrant stops act exactly once. Continue after individual cleanup
    failures, then rethrow the first failure with later failures suppressed.
  - **Initialization failure policy:** wrap `initTeleOp()` and the explicit-source `initAuto(...)` so
    a construction failure invokes the same idempotent stop, attaches any cleanup failure to the
    original exception, and rethrows the original cause. Successfully constructed owners are then
    cleaned even when a later owner or telemetry step fails.
  - **Rejected designs:** do not keep deprecated aliases because all callers are in-repository and
    breaking changes are allowed for one coherent API. Do not add `AutoCloseable`, a mode state
    machine, a generic framework helper, mocking dependency, injection graph, or test-only lifecycle
    interface for this bounded robot fix; `COMMON-01` separately evaluates reusable initialization
    ceremony.
  - **Implementation refinement:** a package-private pure-Java `PhoenixShutdown` executor enforces
    ordered, one-shot, best-effort action execution without becoming a cleanup registry or public
    robot concept. The complete owner-to-action mapping remains visible in `PhoenixRobot.stop()`.
  - **Verification plan:** migrate both callers and lifecycle documentation; confirm no references to
    the removed methods remain; unit-test the internal executor's declared ordering, missing-owner,
    repeated-stop, cleanup-failure, and post-stop-init behavior; run `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac`; and inspect the concrete acquisition/cleanup matrix for
    no-init, TeleOp, Auto, and partial-init paths. Direct JVM construction of the concrete FTC Phoenix
    graph would require disproportionate production seams, so perform a later on-robot check that
    drive and scoring outputs reach zero and vision can reinitialize after STOP.
  - **Approval checkpoint:** the user approved this public API reduction on 2026-07-10.
  - **Automated verification:** `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` passed on 2026-07-10. Six tests ran with zero failures and
    zero errors, including three focused shutdown tests. Repository search found no executable
    callers of the removed lifecycle methods, and three independent diff reviews found no lifecycle,
    API, documentation, or test issues. The user confirmed Android Studio inspection on 2026-07-10.
    Actual motor-zero and vision-reinitialization behavior should also be confirmed during the next
    on-robot bring-up.

### SAFE-01 - Final Plant target invariant

- **Problem to confirm:** static clamping happens before dynamic guards, so a guard fallback may
  become the actual hardware command without final finite/range validation.
- **Alternatives to compare:** final re-sanitize/re-clamp; validate fallback only at construction;
  make guard output a constrained value type; or both build-time validation and final defense.
- **Leading hypothesis:** validate static fallback configuration early and retain a cheap final
  Plant-level finite/range check as defense in depth.
- **Completion:** no Plant command can leave its declared legal range or become non-finite after any
  guard; requested/applied status explains the result.
- **Decision record (2026-07-10):**
  - **Confirmed behavior:** `MappedPositionPlant` and `MappedVelocityPlant` clamp the sampled target
    before `PlantTargetGuards.apply(...)`, but a fallback, hold-last value, or rate limiter can then
    replace that candidate and reach the output without a final finite/range check. For example, a
    bounded `[10, 20]` Plant starts with an applied-target field of `0`; a rejecting first-cycle
    hold-last guard can therefore return `0`. A finite static fallback may also lie outside the
    consuming Plant's range, and finite limiter inputs can overflow to a non-finite result.
  - **Current callers:** the three in-repository guard-application paths are the shared low-level
    `Plants` implementation, `MappedPositionPlant`, and `MappedVelocityPlant`. FTC staged builders
    ultimately use the mapped Plants; no Phoenix robot behavior currently calls
    `fallbackTargetUnless(...)` directly. The normal student-facing builder syntax does not need to
    change.
  - **Alternatives considered:** leave behavior unchanged and document the hazard; validate only
    static fallback configuration; add only a final re-sanitize/re-clamp; make guard output a new
    constrained target type; or combine build-time fallback validation with one internal final
    Plant-target defense.
  - **Simplicity comparison:** documentation alone does not enforce hardware safety. Validation-only
    cannot protect dynamic hold-last or limiter state, while final-defense-only silently accepts a
    fallback typo that can be explained when the Plant is built. A constrained value type would add
    another concept and builder plumbing yet still require runtime checks. The combined design adds
    no robot-code calls or public interfaces and gives students an actionable build error.
  - **Chosen design:** when a mapped Plant binds guards to its configured plant-unit range, validate
    that the range can contain finite commands and that every static fallback lies inside it. Route
    all three internal Plant pipelines through one package-private final-safety helper after dynamic
    guards. Preserve an unchanged guard status when its result is safe; otherwise retain the
    previous finite command (clamped into a valid range), report the correction with an existing
    non-accepted status, and reconcile rate-limiter state to the value hardware actually receives.
    If loop time becomes non-finite, the limiter holds that value until a finite time baseline has
    been re-established instead of bypassing its rate on the recovery cycle.
    Temporarily unreferenced position Plants continue to stop their normal output without advancing
    dynamic guards.
  - **Rejected designs:** do not expose a new constrained target/result type or add another staged
    builder choice because neither simplifies normal robot code nor removes the runtime postcondition.
    Do not rely only on output-adapter clamping because `getAppliedTarget()` and status must describe
    the Plant command. Do not add normalized `[-1, 1]` power semantics here; that remains SAFE-02.
    Native-unit mapping overflow and broader servo/owner validation remain API-03.
  - **Verification plan:** add focused pure tests for mapped position and velocity fallback rejection,
    safe fallback status, first-cycle hold-last repair, limiter-state reconciliation, non-finite
    requested/guard output handling, non-finite clock recovery, exact boundary-clamp status, and the
    shared low-level `Plants` path. Confirm a finite power target outside `[-1, 1]` remains unchanged
    in this item. Run `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac`, search all guard-application callers, update Javadocs and
    the actuator/Plant guide, inspect the focused diff, and request Android Studio verification.
  - **Implementation:** mapped position and velocity Plants now reject unusable configured ranges
    and out-of-range static fallbacks at build. All three framework guard-application paths use one
    package-private final-safety helper that retains a finite in-range recovery target, reports the
    correction, and reconciles limiter state. `SlewRateLimiter` safely holds through a non-finite
    clock sample and baseline recovery. The staged student builder API is unchanged, and finite
    power remains unbounded at the Plant layer pending SAFE-02.
  - **Automated verification (2026-07-10):** `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` pass. Twenty-four tests across eight suites ran with zero
    failures or errors. Targeted coverage includes both mapped Plant types, the shared lower-level
    Plant path, fallback validation/status, exact range boundaries and signed zero, initial
    hold-last repair, limiter overflow/state recovery, non-finite clock recovery, and explicit
    SAFE-02 exclusion. Repository search finds one remaining `PlantTargetGuards.apply(...)` call,
    inside the shared final-safety helper, and no Phoenix/tool fallback callers to migrate.
    `git diff --check` passes, and three independent final diff reviews found no remaining code,
    test, documentation, scope, or student-simplicity issues. The user confirmed Android Studio
    inspection on 2026-07-10. Physical actuator behavior should also be observed during the next
    normal on-robot bring-up.

### SAFE-02 - Power Plant applied-target truth

- **Problem to confirm:** a Power Plant can report an applied value outside `[-1, 1]` while the FTC
  adapter silently sends a different clamped value.
- **Alternatives to compare:** Plant-level normalized range invariant; output-level status feedback;
  builder-injected range; or rejecting out-of-range commands.
- **Leading hypothesis:** normalized power is a semantic Plant constraint, so clamp and report it at
  the Plant level while retaining hardware-boundary defense.
- **Completion:** `getAppliedTarget()` equals the command received by the output for finite values,
  with explicit status for clamping and safe behavior for NaN/infinity.
- **Decision record (2026-07-10):**
  - **Confirmed behavior:** `FtcActuators...power()` delegates to `Plants.PowerPlant`, whose target
    context is currently unbounded. A finite request of `2.0` therefore remains
    `getAppliedTarget() == 2.0` with `ACCEPTED` status and is passed to `PowerOutput.setPower(2.0)`.
    Standard FTC motor and CR-servo adapters then clamp/cache `1.0`, so Plant telemetry and the
    hardware-boundary command disagree. The SAFE-01 exclusion test intentionally captures the
    current unbounded behavior.
  - **Current callers:** production and modern examples reach `Plants.power(...)` through the same
    staged `FtcActuators` path: three Phoenix scoring Plants, four drivetrain-direction tester
    Plants, and four framework examples. Their intended logical targets are normalized already;
    direct lower-level `Plants.power(...)` callers are tests. Group scale/bias intentionally maps a
    logical group target into per-child commands and is not another Plant applied target.
  - **Alternatives considered:** document the mismatch without changing behavior; give PowerPlant a
    fixed semantic normalized range; read a possibly clamped value back from `PowerOutput`; ask for a
    range in the staged builder; inject a range only in `FtcActuators`; reject dynamic out-of-range
    requests; or add a constrained public power value type.
  - **Simplicity comparison:** a fixed internal range adds no student calls or concepts and gives
    target-aware sources the true legal domain before the write. Output readback creates a second
    truth channel after hardware has already been called and is ambiguous for grouped/mapped
    outputs. A builder range asks students a redundant question with only one correct answer, while
    runtime rejection can crash ordinary loop-time composition where saturation is conventional.
  - **Chosen design:** only the direct logical PowerPlant owns a private fixed `[-1.0, +1.0]`
    `ScalarRange` in its `PlantTargetContext`. Reuse SAFE-01's final-safety path so finite dynamic
    requests and guard output clamp before `setPower(...)`. Report `CLAMPED_TO_RANGE` when that
    clamp remains the active final transform; a later rate limit, interlock, or fallback may report
    its more specific status instead. Validate a constant guard fallback against the fixed range
    when the PowerPlant is constructed. Keep the FTC adapter clamp as cheap boundary defense. This
    changes no public interface, builder stage, or normal robot call site and is not a major API
    decision.
  - **Rejected designs:** do not add PowerOutput result/status feedback, a public range choice, or a
    parallel FTC-only semantic path. Do not reject dynamic power saturation or add a constrained
    value type. Do not change drive normalization, regulated position/velocity controller output,
    calibration-search output, or grouped child scale/bias mapping; those do not represent a direct
    PowerPlant's applied target. Raw HAL/regulator NaN sanitation is a separate hardware-boundary
    concern and is not claimed as fixed by this item.
  - **Verification plan:** replace the SAFE-01 power exclusion test with positive/negative clamp and
    output-equality assertions; cover exact boundaries/in-range acceptance, unavailable NaN/infinity
    sources, normalized target context, guard candidate/rate semantics, valid fallback status, and
    actionable out-of-range fallback rejection. Move the existing unbounded limiter-overflow
    regression from `Plants.power(...)` to `Plants.position(...)` so SAFE-01 coverage remains real.
    Run `:TeamCode:testDebugUnitTest` and `:TeamCode:compileDebugJavaWithJavac`, search all power
    callers, synchronize Javadocs/principles/guides, inspect the focused diff, and request Android
    Studio verification. FTC/group boundary behavior remains for normal on-robot observation.
  - **Implementation:** direct `Plants.power(...)` Plants now publish the fixed normalized
    `[-1.0, +1.0]` range to target sources and reuse the SAFE-01 final-safety pipeline before calling
    `PowerOutput`. Finite requests and dynamic guard output saturate to that range; unavailable
    non-finite sources retain a finite safe command; and an out-of-range static fallback fails when
    the Plant is constructed with the guard name, value, Plant, and legal range in the message. The
    FTC adapter remains a defensive boundary clamp. No public interface, staged-builder step,
    Phoenix call site, or normal student call shape changed.
  - **Automated verification (2026-07-10):** `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` pass. Thirty-two tests across eight suites ran with zero
    failures or errors. Focused coverage verifies positive and negative saturation, exact boundaries
    and in-range acceptance, normalized target context, first-cycle and subsequent NaN/infinity
    safety, pre-guard normalization, fallback validation/status, and rate limiting toward a clamped
    boundary; the SAFE-01 unbounded limiter-overflow regression remains covered through a position
    Plant. Repository search confirms production constructs direct power Plants only through
    `FtcActuators`; the three Phoenix, four tester, and four modern-example call sites need no
    migration. `git diff --check` passes, and three independent final reviews found no remaining
    code, test, documentation, scope, or student-simplicity issues. The user confirmed Android
    Studio inspection on 2026-07-10. Physical actuator behavior should also be observed during the
    next normal on-robot bring-up.

### TASK-01 - Timed-task start semantics

- **Problem to confirm:** tasks started by `TaskRunner.update()` immediately consume the loop's
  pre-start `dtSec`, allowing a short positive-duration output to finish before it is observed.
- **Alternatives to compare:** absolute `startSec`; skip elapsed time on the start cycle; runner-level
  start-phase separation; or an explicit minimum one-observation contract.
- **Leading hypothesis:** absolute elapsed time from `clock.nowSec()` is easiest to reason about, but
  output tasks must additionally guarantee a positive-duration command is observable.
- **Completion:** focused tests cover zero duration, duration below one loop, normal duration, large
  first-loop `dt`, and duplicate same-cycle updates.
- **Decision record (2026-07-10):**
  - **Confirmed behavior:** `LoopClock.dtSec()` describes the interval ending at the current loop,
    but `TaskRunner.update(...)` starts a newly dequeued task and immediately updates it with that
    same clock. A `0.02` second task first scheduled on a loop whose preceding `dtSec()` is `0.10`
    therefore consumes `0.10` seconds before it has existed. A timed Plant write can run its start
    and finish callbacks before the later Plant phase samples the requested target, while an
    `OutputForSecondsTask` can become inactive before `OutputTaskRunner` exposes its output.
    `GatedOutputUntilTask` does not consume the task-start delta in WAIT, but it leaves idle output
    on the gate-open cycle and can restore idle on its first RUN update, so a positive pulse shorter
    than one loop can still be invisible. Existing task tests cover only runner cycle idempotency.
  - **Current callers:** the affected common paths are `Tasks.waitForSeconds(...)`, timed
    `Tasks.waitUntil(...)`, `DriveTasks.driveForSeconds(...)`, `ScalarTasks` and `PlantTasks` timed
    writes, `PlantTasks.move(...)` timeout/stability, `Tasks.outputForSeconds(...)`, and the preferred
    `Tasks.outputPulse(...)` builder. Phoenix Auto owns a `TaskRunner` and timed waits/guidance;
    Phoenix `ScoringPath` owns the production feed-pulse queue. Modern examples exercise feedback
    moves, timed Plant writes, direct output pulses, and guided output pulses. Direct timed-task
    constructors are otherwise framework internals, tests, or documentation examples. No caller
    syntax needs to change.
  - **Alternatives considered:** leave the behavior unchanged and document the scheduling detail;
    fix only disappearing output tasks; skip the first update in each timed task; separate
    `TaskRunner` start and update into different cycles; introduce a marker/interface or public
    timer type; or keep same-cycle scheduling while measuring every task-owned interval from its
    actual absolute start time and separately guaranteeing positive output observability.
  - **Simplicity comparison:** documentation cannot make an erased command safe. An output-only fix
    leaves waits, timeouts, feedback stability, and drive commands pre-charged. Blindly skipping a
    first update is wrong when direct code starts a task in one cycle and first updates it in the
    next. Runner-level separation is a smaller edit but adds one-loop latency to every condition,
    route, guidance, and custom task, makes correctness depend on one scheduler, and still does not
    publish a gated pulse on its gate-open cycle. Private start timestamps add no student concept or
    call-site step and follow the existing `Task.start(...)` contract.
  - **Chosen design:** preserve `TaskRunner`'s same-cycle start/update behavior. Capture
    `clock.nowSec()` whenever a task-owned timed interval actually begins and derive elapsed time
    from that timestamp for `RunForSecondsTask`, `OutputForSecondsTask`, `WaitUntilTask` timeout,
    gated RUN/COOLDOWN phases, `PlantTasks.MoveTask` timeout/stability, and
    `DriveGuidanceTask`'s no-guidance interval. Existing absolute-time route, calibration, and
    guidance hard timeouts remain unchanged. A positive required output window must expose its run
    command to the documented downstream output/Plant phase for at least one runner cycle; zero
    duration remains immediate with no pulse. Preserve condition-before-timeout precedence and the
    existing inclusive duration boundary. This adds no public API, builder step, or robot-code
    concept and is not a major API decision.
  - **Rejected designs:** do not change the global runner lifecycle, add a timed-task marker, or
    publish a new timer abstraction for this local invariant. Do not modify `LoopClock`, generic
    debouncers/rate limiters, shared drive-guidance blend timing, task reuse, cancellation/clear
    policy, drive ownership, or external route-follower timing; those are separate lifecycle or
    cycle-safety concerns. Handle feedback-move stability locally instead of broadening
    `DebounceBoolean` semantics. Preserve the unchanged shared clock passed to an advanced
    `RunForSecondsTask.onUpdate` callback: if its first callback shares the start cycle,
    `dtSec()` still predates the task, so callback-owned timers must anchor to `nowSec()` as the
    Task contract documents. In-repository callbacks only reassert values and do not integrate it.
  - **Verification plan:** add focused `ManualLoopClock` tests for zero and sub-loop durations,
    normal/exact-boundary duration, a large delta already present at start, and duplicate same-cycle
    updates. Cover callback ordering, timed waits and condition precedence, direct and guided output
    visibility through `OutputTaskRunner`, zero-duration idle behavior, sequence/parallel child
    starts, timed scalar/Plant writes, feedback-move timeout/stability, drive command visibility,
    and the guidance no-command timeout. Run `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac`; search all timed-task callers; synchronize Framework
    Principles, Javadocs, task/output guides, beginner/overview timing notes, and examples where
    prose changes; inspect the focused diff; then request Android Studio verification.
  - **Implementation:** `TaskRunner` retains its existing same-cycle start/update lifecycle. Core
    waits, callback runs, direct output tasks, and timed wait-until tasks now derive elapsed time
    from `nowSec()` captured at task start. Guided output tasks independently anchor RUN and
    COOLDOWN; a positive required run publishes on the gate-open cycle, a sensor already done with
    no minimum completes at idle, and a zero run emits no output while any configured cooldown
    still applies. Feedback moves anchor timeout at task start and stability at the first reached
    observation, resetting stability when feedback leaves the target. Guidance's consecutive
    no-command timeout has its own start/recovery anchor. No public signature, builder, runner
    ordering, or robot call site changed.
  - **Automated verification (2026-07-10):** `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` pass. Fifty-five tests across eleven suites ran with zero
    failures or errors, including twenty-three new focused tests. Coverage includes large pre-start
    deltas; zero, sub-loop, normal, and exact-boundary durations; same-cycle idempotency; callback
    ordering; wait condition precedence; direct/guided output visibility; done-at-gate behavior;
    zero-run cooldown; sequence/parallel children; timed Plant writes; drive command visibility;
    feedback-move timeout/stability/reset/tie semantics; and guidance-loss recovery. Repository
    search finds no remaining task-owned elapsed timer in the scoped task, Plant-move, or guidance
    paths that accumulates `dtSec()`, and all production/example caller syntax remains unchanged.
    `git diff --check` passes. Three independent final reviews found no remaining correctness,
    Framework-Principles, API-scope, documentation, test, or student-simplicity issue after their
    cleanup findings were addressed. The user confirmed Android Studio inspection on 2026-07-10.
    Timing behavior should also be observed during the next normal on-robot bring-up.

### TASK-02 - Task reuse contract

- **Problem to confirm:** built-in Tasks disagree about whether a second `start()` restarts, no-ops,
  or partially reuses children despite documentation declaring Tasks single-use.
- **Alternatives to compare:** enforce single-use with actionable exceptions; make every task safely
  restartable; or move repeated behavior to `Supplier<Task>` factories.
- **Leading hypothesis:** single-use plus factories is the clearest lifecycle and avoids retaining
  partially consumed task graphs.
- **Completion:** all built-ins and composites follow one documented rule; repeated start/enqueue has
  deterministic tests and a useful error.
- **Decision record (2026-07-10):**
  - **Confirmed behavior:** built-in Task instances currently have three incompatible second-start
    behaviors. `InstantTask`, `RunForSecondsTask`, and `ParallelAllTask` silently no-op;
    `WaitUntilTask`, `OutputForSecondsTask`, `GatedOutputUntilTask`, `SequenceTask`, the anonymous
    `Tasks.branchOnOutcome(...)` and scalar-set tasks, `PlantTasks.MoveTask`, calibration search,
    `RouteTask`, and `DriveGuidanceTask` reset/restart. A reused sequence or outcome branch can
    partially restart because its children independently no-op, restart, or remain terminal.
    `Tasks.noop()` is terminal even before `start()`. `TaskRunner.enqueue(...)` accepts the same
    identity more than once, so misuse may be silent, partial, or delayed until a task graph runs.
    Existing tests use each instance once and do not define repeated-start behavior.
  - **Current callers:** repository production and modern examples already follow fresh-instance
    patterns. Phoenix Auto creates and enqueues one routine graph; TeleOp 03/06 rebuild macro graphs
    for each activation; TeleOp 07 creates a new direct output task per request; Phoenix
    `ScoringPath` and TeleOp 09 use `OutputTaskFactory`/factory methods; and `TaskBindings` already
    accepts `Supplier<Task>` or value-to-Task factories. No executable caller re-enqueues or directly
    restarts a Task instance, so the common student call sites need no migration.
  - **Alternatives considered:** keep the mixed behavior and document it; make every Task safely
    restartable; make a second start idempotent; enforce reuse only in `TaskRunner`; retain every
    ever-enqueued identity; replace or overload enqueue with supplier-only APIs; add a public marker,
    lifecycle/base class, reset method, or general `TaskFactory`; or enforce a one-shot lifecycle in
    each framework Task while using existing suppliers/factories for repetition.
  - **Simplicity comparison:** documentation or silent idempotence preserves skipped/partial macros.
    Universal restartability requires every child graph, source, route follower, controller,
    hardware owner, outcome, and cancellation state to reset perfectly, yet still cannot safely put
    one object under parallel owners. Supplier-only enqueue adds ceremony to ordinary one-shot Auto,
    while an overload creates two normal paths. A permanent runner identity ledger can cover custom
    Tasks but retains every fresh pulse/macro for the runner lifetime. Local one-shot guards change
    no normal robot call, and existing `Supplier<Task>`/macro builders plus `OutputTaskFactory`
    already make repetition explicit and discoverable.
  - **Chosen design:** define one public contract: a Task instance may enter `start(clock)` once.
    Every framework built-in, anonymous Task, and composite records that attempt before child or
    hardware side effects; another start while active or after any terminal outcome throws an
    actionable `IllegalStateException` naming the Task and instructing the caller to create a fresh
    instance with a builder/macro method, `Supplier<Task>`, or `OutputTaskFactory`. Keep repeated
    `cancel()` and terminal `update()` behavior unchanged. `TaskRunner.enqueue(Task)` remains the
    single enqueue API but rejects the same identity when it is already current or queued, without
    retaining an unbounded history. Sequence, parallel, and outcome-branch construction reject
    obvious duplicate child identities before any child starts; nested aliases still fail through
    the child's own start guard. Repetition continues through existing factories—no new public
    lifecycle type, base class, reset API, factory type, builder step, or enqueue overload.
  - **Rejected designs:** do not make Task instances restartable, because stateful child/resource
    graphs cannot offer that guarantee through the current narrow interface. Do not silently ignore
    a second start, since it hides skipped behavior. Do not enforce only at the runner, because
    direct/composite/cross-runner starts bypass it; do not keep a permanent/weak global ownership
    registry. Do not remove the convenient direct `enqueue(Task)` path or add a parallel supplier
    overload. Direct cancel-before-first-start and update-before-start policy remain TASK-03 scope;
    this item is narrowly about start/re-enqueue reuse and leaves those first-use effects unchanged.
    Correct `SequenceTask.fromSuppliers(...)` documentation to say its suppliers run once while
    constructing one single-use sequence; it is not a restart mechanism.
  - **Verification plan:** add focused tests for direct second start while active and after terminal
    completion across instant/noop, timed wait/run, direct/gated output, scalar/Plant tasks,
    calibration, route, guidance, sequence, parallel, and outcome branch families. Test immediate
    identity rejection for a duplicate queued/current Task; delayed built-in rejection after
    completion and across runners; duplicate composite children; partial-progress/completed
    composite restart; a repeated backlog supplier returning one instance; and successful execution
    of two genuinely fresh supplier/`OutputTaskFactory` results. Assert error messages identify the
    Task and explain the fresh-instance fix. Preserve all first-run, TASK-01 timing, outcome, and
    same-cycle tests; run `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac`; search callers; synchronize Framework Principles,
    Javadocs, task/output guides, beginner/overview docs, calibration guidance, and repository
    instructions; inspect the focused diff; then request Android Studio verification.
  - **Approval gate:** although signatures and normal call sites remain unchanged, this deliberately
    changes the public lifecycle semantics of every framework Task and composite. Treat it as a
    major API-semantic decision and obtain user approval before moving to **In progress**.
  - **Approval:** the user approved the TASK-02 design on 2026-07-10.
  - **Implementation (2026-07-10):** every framework-owned Task implementation now records a
    private start attempt before its action, child, controller, source reset, target write, route,
    guidance, or calibration side effects and throws an actionable single-use error on another
    start. `TaskRunner` identity-rejects a duplicate current/queued object without retaining
    history. Sequence, parallel, and outcome-branch construction reject direct child aliases, while
    nested aliases fall through to the leaf guard. Start/reuse exceptions still propagate with the
    runner's existing state; this item does not guess whether a partially started graph should be
    cancelled or dropped, because that failure-cleanup policy belongs with TASK-03. Existing macro
    methods, `Supplier<Task>`, and `OutputTaskFactory` remain the repetition path; no public
    lifecycle type, reset method, marker, builder step, enqueue overload, or normal Phoenix caller
    migration was added. Framework Principles, repository instructions, Javadocs,
    task/output/beginner/overview guides, and calibration guidance now state the same start-attempt
    contract without deciding TASK-03's cancel-before-first-start policy.
  - **Automated verification (2026-07-10):** four new suites add 32 tests across the core Task,
    actuation/calibration, route, and guidance families. They cover active, terminal, failed, and
    partial second starts; guard-before-side-effect behavior; direct and nested composite aliases;
    duplicate queued/current identities; completed and cross-runner reuse; eager/fresh suppliers
    and output factories; repeated and invalid backlog suppliers; repeated cancellation/terminal
    updates; and preserved cancel-before-first-start and TASK-01 timing behavior.
    `:TeamCode:testDebugUnitTest` passes all 87 tests in 15 suites with zero
    failures, errors, or skips, and `:TeamCode:compileDebugJavaWithJavac` succeeds. Static inspection
    finds a guard in every framework-owned `start(LoopClock)` implementation, modern Phoenix/example
    callers already construct fresh tasks, and `git diff --check` plus the trailing-whitespace scan
    pass. Only the repository's existing JDK 21/source-8 and deprecation warnings remain. Independent
    reviews corrected three cancellation-wording overstatements, the current-versus-active wording,
    and an attempted failed-start cleanup that would have preempted TASK-03's safety decision; final
    re-review reports no remaining scoped issue.
  - **Manual verification:** the user confirmed the Android Studio review on 2026-07-10. No
    robot-hardware validation is required for this misuse contract; normal behavior call sites and
    hardware commands are unchanged.

### TASK-03 - Clear and cancellation semantics

- **Problem to confirm:** `TaskRunner.clear()` can discard an active task without `cancel()`, while a
  canceled Plant move may leave a persistent target commanding motion.
- **Alternatives to compare:** remove `clear()`; redefine it as cancel-and-clear; add
  `clearPending()`; and make Plant cancellation policy explicit or choose one universal safe policy.
- **Leading hypothesis:** keep `cancelAndClear()` as the obvious destructive operation, add
  `clearPending()` only if a real caller needs it, and make mechanism cancellation behavior explicit
  at task construction rather than globally guessing what is safe.
- **Completion:** no queue operation silently abandons active output; tests cover cancel before
  start, during motion, after completion, and repeated cancellation.
- **Decision record (2026-07-11):**
  - **Confirmed behavior:** `TaskRunner.clear()` clears its queue and drops its current Task without
    calling `cancel()`; `OutputTaskRunner.clear()` exposes the same operation and immediately
    reports idle output. A feedback move writes a persistent `ScalarTarget` when it starts, but
    `PlantTasks.MoveTask.cancel()` only changes outcome/completion state, so the Plant continues
    requesting that move target. `PlantTasksTimingTest` explicitly preserves this today even when
    `.thenTarget(...)` is configured. The focused baseline `PlantTasksTimingTest` and
    `TaskRunnerTest` run passes all seven tests with zero failures, errors, or skips.
  - **Lifecycle gaps confirmed:** framework Tasks disagree about cancellation before `start()`:
    some make the task terminal, some reopen and run later, some do nothing, and calibration,
    route, or guidance Tasks may call an external cleanup operation before acquiring the resource.
    Direct `update()` before `start()` likewise auto-starts some Tasks while other Tasks no-op or
    mutate default state. `TaskRunner.update()` leaves a Task current, its follow-up queue intact,
    and the cycle marked updated when `start()`, `update()`, or `isComplete()` throws. A throwing
    cancellation hook also prevents today's `cancelAndClear()` from clearing the queue. TASK-02
    deliberately deferred these first-use and failed-start policies to this item.
  - **Current callers:** there are no executable in-repository callers of either runner's
    `clear()` and no direct caller of `cancelCurrent()`. Every real total-abort path already uses
    `cancelAndClear()`: Phoenix Auto shutdown, `ScoringPath`'s feed queue, and modern examples 03,
    06, and 09. Modern Phoenix has no raw `PlantTasks.move(...)` caller. The student-facing move
    callers are framework examples, tests, and guides. The six compact `moveTo*` helpers have no
    executable caller outside `PlantTasks` itself. Examples 03 and 06 also say their timed transfer
    pulse stops, but omit `.then(0.0)`, so normal completion currently leaves shoot power requested;
    their owner-level cancellation methods correctly reset every related mechanism target.
  - **Alternatives considered:** retain/document or deprecate abrupt `clear()`; redefine it as
    cancellation; remove it; add `clearPending()`; make cancellation before start a terminal state,
    an error, or an active-only operation; retain automatic update-start behavior or reject it;
    preserve, drop, or fail-stop a runner after lifecycle exceptions; and use a universal zero,
    current measurement, previous target, `Plant.stop()`, `.thenTarget(...)`, an optional fixed
    cancellation target, a required two-choice builder stage, a public policy type, or an arbitrary
    callback for Plant move cancellation.
  - **Simplicity comparison:** removing `clear()` subtracts two unsafe public methods and leaves the
    already-used `cancelAndClear()` as the one obvious total abort; no caller needs
    `clearPending()`. Active-only cancellation needs no extra Task state or robot call: before start
    there is no acquired resource to release, while queued work is withdrawn by its runner.
    Making pre-start cancellation terminal and silently skipping it could let a sequence feed after
    a cancelled prerequisite; throwing on the later first start adds another lifecycle state.
    Rejecting update-before-start gives advanced callers one actionable rule without changing
    runner-managed robot code. For a Plant move, one required autocomplete-guided cancellation
    choice adds one meaningful line but no named policy object; making it optional would preserve
    the unsafe omission, while a policy interface/callback or adjacent-`double` helper overloads add
    more concepts. Removing the compact helpers leaves one clear move API instead of a bypass around
    that safety question.
  - **Chosen runner and Task lifecycle:** remove public `clear()` from both runners, add no
    `clearPending()`, retain the distinct `cancelCurrent()`, and keep `cancelAndClear()` as the
    normal total abort. Framework Task cancellation before the first `start()` is a side-effect-free
    no-op; cancellation while active is terminal; cancellation after completion and repeated
    cancellation are no-ops. A direct framework Task `update()` before start fails with an
    actionable lifecycle error rather than auto-starting or silently mutating state;
    `Tasks.noop()` is the intentional terminal-at-construction exception. On a Task
    lifecycle `RuntimeException`, a runner detaches and best-effort cancels the current/partially
    started Task, clears all pending work, resets its owned state, and rethrows the original failure
    with a cleanup failure suppressed. Total abort must leave the runner empty even when cancellation
    throws or re-enters it; a failed `cancelCurrent()` also clears pending work rather than allowing
    a follow-up to run after failed cleanup. `OutputTaskRunner` invalidates cached output and reports
    its configured idle value after every abort/failure path.
  - **Chosen Plant move API:** after `.to(target)`, require exactly one staged answer:
    `.cancelTo(target)` writes that finite Plant-unit request on active cancellation, or
    `.leaveTargetOnCancel()` deliberately performs no cancellation write and warns that motion may
    continue. Either answer returns the existing optional stability/timeout/completion step.
    `.thenTarget(...)` remains success/timeout behavior because a shooter commonly must keep speed
    after successful readiness but command zero only when cancelled. Cancellation never writes a
    move or cancellation target before start; active cancellation establishes terminal state before
    its one cancellation write; terminal/repeated cancellation never writes again. Remove the six
    compact `moveTo`, `moveToStable`, and `moveToThen` overloads because they cannot express the
    required choice without hidden policy or ambiguous arguments. Do not add a Plant capability
    interface, Plant-builder setting, public policy enum, or arbitrary cleanup callback.
  - **Ownership boundary:** `cancelTo(...)` changes the registered behavior request; it does not
    imperatively stop hardware. The next Plant update still owns final resolution, overlays may mask
    the request, and Plant bounds/guards may transform it. Cancelling one active child also cannot
    undo persistent requests left by earlier completed macro children. Robot capability/subsystem
    owners must still cancel queues, disable overlays, and reset every related mechanism request for
    coordinated or emergency shutdown. API-01 still owns writable-target provenance.
  - **Rejected/deferred scope:** do not apply `.thenTarget(...)` on cancellation because it cannot
    express different success and cancellation behavior. Do not guess zero, measurement, previous
    target, or `Plant.stop()` because none is safe across position and velocity Plants and a direct
    stop is overwritten by the source on the next loop. Do not make `ensureBacklog()` transactional
    in this item: factory/admission atomicity is separable from active cancellation, cannot roll back
    factory side effects, and its current partial-admission behavior is already explicit in a
    TASK-02 test. Do not add a failure outcome enum or promise cleanup for arbitrary custom Tasks
    whose default cancellation hook owns no cleanup.
  - **Verification plan:** add focused runner tests for active/queued/terminal/repeated cancellation,
    queued discard without pre-start hooks, cancellation-hook failure and reentrancy, start/update/
    completion-check failure cleanup, suppressed cleanup exceptions, and OutputTaskRunner idle-cache
    recovery. Replace the TASK-02 compatibility assertions with a common pre-start/no-op,
    update-before-start rejection, active cancellation, terminal cancellation, and repeated
    cancellation matrix across core, composite, scalar/Plant, calibration, route, and guidance Task
    families. Test both required Plant cancellation choices, finite-value validation, exactly-once
    writes, a throwing cancellation target, success/timeout independence, and timed-write `.then(...)`
    cancellation. Migrate the two compiling shooter examples, all affected Javadocs, Framework
    Principles, task/output/beginner/overview/target-planning guides, and every compact-helper
    reference. Run `:TeamCode:testDebugUnitTest`, `:TeamCode:compileDebugJavaWithJavac`, XML result
    counts, exhaustive caller searches, `git diff --check`, trailing-whitespace checks, adversarial
    reviews, and Android Studio inspection. On-robot observation should confirm cancellation requests
    pass through real overlays/guards and coordinated shutdown still stops scoring and drive.
  - **Approval gate:** this design removes public runner and Plant convenience APIs and changes the
    lifecycle contract of every framework Task. It remains consistent with the leading hypothesis,
    but it is a major API/semantic decision and requires explicit user approval before moving to
    **In progress**.
  - **Approval:** the user approved the TASK-03 design on 2026-07-11.
  - **Implementation (2026-07-11):** removed the unsafe `clear()` operation from both runners and
    added no replacement drop-without-cancel path. `cancelCurrent()` now acts only on a genuinely
    active Task, while total abort and lifecycle `RuntimeException` paths fail closed, withstand
    throwing/reentrant cleanup, and do not recursively advance replacement work. Output queues
    invalidate scalar and active-source caches on every abort/failure boundary and keep diagnostics
    live without propagating Task lifecycle failures. Framework-owned Tasks now share active-only,
    idempotent cancellation and actionable update-before-start behavior, with the documented
    terminal-at-construction `Tasks.noop()` exception. Composite, source, target, calibration,
    route, and guidance callbacks preserve terminal cancellation even when they re-enter an owning
    runner. Feedback moves now require `.cancelTo(...)` or `.leaveTargetOnCancel()` immediately
    after `.to(...)`; `.thenTarget(...)` remains success/timeout-only, and all six policy-bypassing
    compact `moveTo*` helpers are gone. Examples 03 and 06 explicitly stop their transfer pulse,
    and Framework Principles, repository instructions, Javadocs, guides, and examples describe the
    same lifecycle and ownership boundary.
  - **Automated verification (2026-07-11):** `:TeamCode:testDebugUnitTest` passes all 135 tests in
    17 suites with zero failures, errors, or skips, and `:TeamCode:compileDebugJavaWithJavac`
    succeeds. Focused tests cover pre-start/active/terminal/repeated cancellation; throwing and
    reentrant abort/fail-stop cleanup; same-cycle output and active-source invalidation; diagnostic
    failure handling; composite cleanup and start/update reentrancy; calibration acquisition;
    route/guidance cleanup; both Plant move cancellation policies; finite validation; throwing and
    reentrant target writes; and success/timeout independence. Exhaustive searches find no caller
    of the removed runner or compact Plant APIs and no feedback-move chain missing its explicit
    cancellation choice. `git diff --check` passes. Three independent adversarial reviews corrected
    callback reentrancy, composite cleanup, cached-source, diagnostic, acquisition-order, and
    documentation issues; final re-review reports no remaining scoped finding. Only the existing
    JDK 21/source-8 and deprecation warnings remain.
  - **Manual verification:** the user confirmed the Android Studio review on 2026-07-11. On-robot
    observation is recommended during the next normal bring-up to confirm that cancellation
    requests pass through the real Plant overlays/guards and that coordinated scoring/drive
    shutdown still reaches safe hardware output.

### PEDRO-01 - Follower heartbeat and stopped-state ownership

- **Problem to confirm:** Phoenix advances the Pedro `Follower` only from active route/guidance
  Tasks, while Pedro's supported lifecycle advances one follower every OpMode loop independently of
  the scheduled command. A route that transitions into Pedro hold-end becomes `isBusy() == false`,
  so `RouteTask` completes and stops calling `follower.update()`. A following mechanism/wait Task
  therefore loses hold and pose updates. In manual/guidance mode, `setTeleOpDrive(0, 0, 0)` changes
  requested vectors but does not itself apply zero motor output, so the current adapter's `stop()`
  may leave the last nonzero output applied when the next Task does not update Pedro.
- **External evidence:** Pedro's
  [official Auto example](https://pedropathing.com/docs/pathing/examples/auto) calls
  `follower.update()` once every loop separately from scheduler execution. The
  [v2.1.2 Follower](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.2/core/src/main/java/com/pedropathing/follower/Follower.java)
  applies drivetrain output inside `update()`, continues hold-point control after `isBusy()` becomes
  false, and stops immediately through `breakFollowing()`. Worlds-level Pedro/Ivy code follows the
  same pattern: one recurring robot/follower periodic owner and route commands that only start and
  observe path completion. Cuttlefish makes this separation explicit by calling
  [`follower.update()` from its outer loop](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/EnhancedOpMode.java)
  independently of `PathActionScheduler.update()`.
- **Alternatives to compare:** retain Task-owned updates and force hold-end off; call raw
  `follower.update()` globally while leaving Task updates unchanged; make `drive()`/`stop()` force
  hidden follower updates; or introduce one small Pedro runtime/lane that owns the Follower
  heartbeat, immediate stop, and cycle deduplication while Tasks only select route/manual/hold
  behavior.
- **Leading hypothesis:** one Pedro integration owner, called by the Auto composition root before
  the Phoenix TaskRunner, should advance the single Follower exactly once per OpMode cycle whether a
  route, guidance, mechanism, wait, or hold phase is active. Route and drive Tasks may request
  behavior but must not create a second heartbeat. Adapter cancellation/stop should use the pinned
  typed Pedro stop operation and satisfy `DriveCommandSink.stop()` immediately. Do not add Ivy or a
  second scheduler to Phoenix robot code.
- **Completion:** tests or a version-pinned integration harness cover exactly one follower update per
  cycle; route -> hold-end -> mechanism wait; route completion/cancellation/timeout; guidance ->
  scoring wait; next-route transition; immediate stop; repeated shutdown; and callback/update
  reentrancy. The route -> manual transition is included explicitly because Pedro's
  `startTeleopDrive()` may perform an update internally; switching modes must not advance Pedro
  twice in one Phoenix cycle. The common routine call site stays unchanged, and documentation shows
  the one explicit Pedro loop owner.
- **Decision record (2026-07-11):**
  - **Confirmed pinned behavior:** Phoenix pins Pedro FTC/core `2.1.2`. In that version,
    `followPath(...)` starts a route but does not advance it; `Follower.update()` is the recurring
    pose, callback, hold, and drivetrain heartbeat. Natural hold-end completion sets
    `isBusy() == false` while hold control still requires later updates. `setTeleOpDrive(...)` only
    stores requested vectors, `startTeleopDrive()` performs a hidden `Follower.update()`, and
    `breakFollowing()` synchronously reaches Pedro's mecanum stop and writes zero to all four drive
    motors. Phoenix does not override Pedro's default automatic hold-end setting. The current
    adapter's `follow(route, false)` accidentally calls the default-hold overload, its reflective
    compatibility paths are unnecessary against the exact pin, and its manual-mode `stop()` stores
    zero without applying another heartbeat.
  - **Confirmed failure paths:** `RouteTask` is currently the only route heartbeat owner. Once a
    route reaches hold-end it observes `!isBusy()`, completes, and a following mechanism or wait
    Task leaves Pedro pose and hold correction frozen while the last motor output remains applied.
    `DriveGuidanceTask` likewise updates Pedro before storing its next manual vector; success,
    timeout, or cancellation can then leave the previous nonzero output applied throughout the
    following scoring wait. Adding an unconditional root update without deduplication would create
    two updates on route/guidance cycles and can create a third-party hidden update during the
    route-to-manual transition. A callback can also reenter cancellation from inside
    `Follower.update()` and Pedro's enclosing update can otherwise write motion again after the
    cancellation stop.
  - **Current callers and lifecycle:** `PedroPathingDriveAdapter` is the one object shared by
    `RouteTask` and `DriveGuidanceTask`; Phoenix constructs it only in
    `PhoenixPedroAutoOpModeBase`, passes it through `PhoenixPedroAutoContext` and the routine
    factory, and currently advances it only through those active Tasks. `PhoenixRobot.updateAuto()`
    owns the stable localization -> targeting -> TaskRunner -> scoring loop, while the OpMode owns a
    separate adapter stop. `PhoenixPedroPathFactory` also performs one raw `follower.update()`
    during INIT after setting the start pose. The concrete selector/static Auto entries and their
    routine call sites need no change. Pedro tuning/sample OpModes own raw Followers independently
    and are not modern Phoenix integration callers. Affected API documentation includes the Pedro
    adapter and integration guide, `DriveCommandSink`, route/guidance Javadocs, Framework Overview,
    Robot Capabilities & Mode Clients, Recommended Robot Design, Phoenix Architecture, and the
    Phoenix Pedro guide.
  - **Alternatives considered:** document the current behavior or force hold-end off; retain
    Task-owned heartbeats; add a raw global `follower.update()` beside Task updates; make
    `drive()`/`stop()` perform hidden updates; pass the adapter into every `updateAuto(...)` call;
    expose the robot's mutable `LoopClock`; split Task-facing no-op views from a second Pedro
    runtime/lane; add a generic external-route host or another scheduler; or make the existing
    adapter cycle-aware and give it to the Phoenix Auto composition root at initialization.
  - **Simplicity comparison:** documentation and disabling hold do not repair stale pose or stopped
    output. Task ownership fails during ordinary mechanism/wait phases, while raw parallel updates
    duplicate stateful third-party work. Imperative hidden updates make output timing depend on
    which method happened to be called and conflict with Pedro's own hidden transition update.
    Passing the sink on every loop repeats a stable ownership choice and leaves shutdown split
    between owners; exposing the clock weakens the one-heartbeat boundary. A new Pedro lane or
    Task-specific view set could make command timing more specialized, but adds objects and concepts
    before PEDRO-02 decides drivetrain/localization authority. Reusing the existing adapter and
    existing backend-neutral `DriveCommandSink` seam adds no concept to routine code and leaves the
    normal student sequence unchanged.
  - **Chosen integration ownership:** evolve `PedroPathingDriveAdapter` into the sole cycle-aware
    owner of its Follower. Supply it as the required backend-neutral Auto drive owner when
    initializing `PhoenixRobot`; the robot retains it, advances it with the shared clock after
    localization/targeting and before `autoRunner`, and stops it in the same idempotent shutdown
    graph after behavior cancellation. Remove the parallel no-drive Phoenix Auto initialization
    path rather than create two normal lifecycle choices. Pedro types, route geometry, alliance,
    and routine selection remain outside `PhoenixRobot`; concrete Auto routines continue using the
    same adapter-backed `RouteTasks` and guidance factories.
  - **Chosen adapter lifecycle:** make `update(clock)` idempotent by `clock.cycle()` and record the
    cycle/in-progress state before entering Pedro so same-cycle Task calls and callback reentry
    cannot repeat the heartbeat. Use the pinned typed `followPath(route, holdEnd)`,
    `startTeleopDrive()`, `setTeleOpDrive(...)`, and `breakFollowing()` APIs and remove reflection.
    Stage a manual-mode request so `startTeleopDrive()` runs only inside the owned heartbeat and its
    internal vendor update counts as that cycle's one update; retain the requested command for the
    next heartbeat instead of forcing a second update. `stop()` and route cancellation replace any
    pending motion with a zero-manual stopped request and call `breakFollowing()` immediately; a
    stop requested reentrantly is enforced again after the active update returns, and the next
    owned heartbeat enters/stays in stable zero-manual mode rather than resurrecting a retained
    route. If the owned vendor update throws, best-effort break the follower before rethrowing. Fix
    explicit `holdEnd == false`, and remove the raw INIT update from `PhoenixPedroPathFactory`
    because `setStartingPose(...)` establishes the starting pose without consuming a runtime
    heartbeat.
  - **Preserved generic behavior and deferred scope:** keep `RouteFollower.update(...)`,
    `RouteTask`, `DriveCommandSink.update(...)`, and `DriveGuidanceTask` semantics unchanged so
    non-Pedro integrations can still own Task-local work; the Pedro adapter makes their later
    same-cycle calls no-ops. Do not add Ivy, a second scheduler, a background thread, a generic
    external-route runtime, or a public heartbeat interface. Do not decide PEDRO-02's drivetrain,
    Pinpoint, localization, field transform, or pose-correction authority; on-robot validation of a
    fully configured Follower remains dependent on that item. Do not change truthful route terminal
    outcomes (ROUTE-02) or drive-task ownership (DRIVE-01) here.
  - **Verification plan:** add a version-pinned Pedro harness with fake drivetrain/localization and
    focused adapter/Phoenix lifecycle tests for one update per clock cycle, same-cycle root plus
    RouteTask/guidance calls, hold-end through mechanism waits, explicit no-hold, route completion,
    timeout and cancellation, route-to-manual transition including Pedro's hidden update, guidance
    to scoring wait, immediate physical zero, stable stopped heartbeats, next-route startup,
    repeated shutdown/INIT cleanup, callback reentry, reentrant stop, and thrown update cleanup.
    Verify the required loop order and unchanged routine surface; synchronize every affected
    Javadoc/guide; run `:TeamCode:testDebugUnitTest` and `:TeamCode:compileDebugJavaWithJavac`, count
    XML results, search for every raw follower update and old Auto-init caller, run
    `git diff --check`, perform adversarial reviews, and request Android Studio inspection. Real
    drivetrain/hold behavior should be observed on hardware after PEDRO-02 supplies the valid
    production runtime.
  - **Approval gate:** the leading hypothesis remains the smallest robust design and ordinary Auto
    routine code stays unchanged. Requiring Phoenix Auto initialization to receive and own the
    external drive lifecycle is nevertheless a major public lifecycle/API decision, so explicit
    user approval is required before moving this item to **In progress** or editing Java code.
  - **Approval:** the user approved the PEDRO-01 design on 2026-07-11.
  - **Implementation (2026-07-11):** `PedroPathingDriveAdapter` is now the one cycle-aware owner of
    its pinned Pedro `Follower`. A private follower/manual-pending/manual state machine records
    `LoopClock.cycle()` before entering Pedro, counts `startTeleopDrive()`'s hidden vendor update as
    the transition heartbeat, stages manual vectors without forcing another update, keeps route
    hold/pose updates alive outside route Tasks, and uses typed Pedro 2.1.2 APIs instead of
    reflection. Explicit `holdEnd == false` now reaches the exact boolean overload. Cancellation,
    stop, route-start failure, and update failure stage stable zero-manual state and call
    `breakFollowing()` immediately; callback-time stops are reasserted after the enclosing vendor
    update, repeated owner stops retry the physical break, and the original failure retains any
    cleanup failure as suppressed. Route starts are rejected from inside an active heartbeat, and
    callback-initializer stop/manual requests deliberately win over the outer route start.
  - **Phoenix ownership and API:** Phoenix Auto initialization now requires a backend-neutral
    `DriveCommandSink`. `PhoenixRobot` retains it, updates it after localization/targeting and before
    the Auto runner, and stops it after behavior cancellation as part of the existing one-shot
    shutdown graph. The Pedro OpMode constructs the follower/adapter before
    `initAuto(adapter)` and no longer owns a parallel adapter stop. `PhoenixShutdown` now also
    rejects repeated or TeleOp/Auto cross-initialization before an active ownership graph can be
    overwritten. The path factory sets the starting pose without consuming a raw INIT update.
    Concrete route/routine factories, contexts, annotated Auto entries, and student task sequences
    remain unchanged; Pedro drivetrain/localization/pose authority remains PEDRO-02 scope.
  - **Documentation:** Framework Principles, repository instructions, Loop Structure, Framework
    Overview, Recommended Robot Design, Robot Capabilities & Mode Clients, drive/route/guidance
    Javadocs, both Pedro guides, Phoenix Architecture, and Phoenix lifecycle/path Javadocs now state
    the same qualified rule: only an integration whose supported lifecycle needs updates beyond an
    active Task gets a persistent root heartbeat, same-cycle Task calls deduplicate, and stop means
    immediate physical output rather than a staged zero request.
  - **Automated verification (2026-07-11):**
    `:TeamCode:testDebugUnitTest` and `:TeamCode:compileDebugJavaWithJavac` pass. The XML reports 152
    tests across 18 suites with zero failures, errors, or skips. Sixteen new version-pinned adapter
    tests instantiate Pedro core 2.1.2's real `Follower`, paths, callbacks, and guidance with fake
    `Localizer`/`Drivetrain` boundaries. They cover root plus RouteTask/guidance same-cycle
    deduplication; hold-end through a root-only wait; explicit no-hold; normal completion, timeout,
    cancellation, next-route recovery, and route-start failure; the hidden manual transition;
    guidance success followed by root-only scoring-wait cycles; immediate/stable/repeated stop;
    callback update reentry, callback-initializer stop, callback route-start rejection, and
    post-callback final break; failed-cycle consumption; and primary/suppressed failure plus later
    owner-stop retry. Four Phoenix lifecycle tests cover declared shutdown order, best-effort error
    aggregation, post-stop rejection, and repeated/cross-mode initialization without losing the
    original graph.
  - **Static and external-boundary verification:** exhaustive searches find the one production
    adapter construction and `initAuto(adapter)` call, no no-argument Auto initializer, no separate
    OpMode adapter stop, no reflective/legacy Pedro compatibility path, and no raw Follower update
    in modern Phoenix code outside the adapter. Source inspection confirms the explicit Phoenix
    order `localization -> targeting -> autonomousDrive -> autoRunner -> scoring` and shutdown order
    `cancel behavior -> stop scoring -> stop autonomous drive -> release support`. The pinned Pedro
    2.1.2 FTC `Mecanum.breakFollowing()` source writes zero to every motor synchronously before
    selecting FLOAT. `git diff --check` and the changed/untracked trailing-whitespace scan pass.
    Three independent final reviews report no remaining correctness, lifecycle, Framework
    Principles, student-simplicity, API-scope, documentation, or test-validity issue after their
    findings were corrected. Existing JDK 21/source-8 and deprecation warnings remain unchanged.
    The fake drivetrain proves that the adapter reaches Pedro's typed stop and leaves its boundary
    at zero; actual FTC motor-controller output, hold behavior, and the production Follower graph
    still require on-robot observation after PEDRO-02 supplies the valid drivetrain/localization
    runtime.
  - **Manual verification:** the user confirmed the Android Studio review on 2026-07-11. No
    PEDRO-01 hardware test is required before publication because PEDRO-02 still owns the unresolved
    production drivetrain/localization construction; perform the recorded physical hold and motor-
    zero observations after that runtime is valid.

### PEDRO-02 - Pedro drivetrain, localization, and pose authority

- **Problem to confirm:** checked-in `Constants.createFollower(...)` supplies only path constraints;
  its declared mecanum and Pinpoint configs are not passed to `FollowerBuilder`, so the pinned Pedro
  runtime does not currently have a valid configured drivetrain/localizer. Enabling the commented
  Pinpoint path naively would let both Pedro and `FtcOdometryAprilTagLocalizationLane` own/update the
  same Pinpoint hardware. Pedro pathing/debugging and Phoenix targeting would also consume separate
  poses in different field-coordinate conventions without an explicit synchronization, correction,
  or drift policy.
- **External evidence:** Pedro's
  [setup guide](https://pedropathing.com/docs/pathing/tuning/setup) requires drivetrain construction,
  its [custom-localizer guide](https://pedropathing.com/docs/pathing/custom/localizer) puts hardware
  ownership in one localizer, and its
  [coordinate guide](https://pedropathing.com/docs/pathing/reference/coordinates) requires explicit
  conversion between Pedro and FTC field frames. Cuttlefish provides a concrete valid caller by
  wiring both
  [mecanum and Pinpoint configuration into one Follower](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/Constants.java),
  then using the same owner's pose and motion in pathing and shot-on-the-move calculations.
- **Alternatives to compare:** Phoenix owns Pinpoint and feeds Pedro through a custom localizer;
  Pedro owns Pinpoint and exposes pose/deltas through a Phoenix predictor adapter; deliberately use
  separate sensors/estimators with synchronization and drift diagnostics; or create a Pedro-specific
  integration lane that owns construction plus the explicit pose bridge. Compare whether AprilTag
  corrections should affect targeting only or also reset/correct Pedro's path pose.
- **Leading hypothesis:** do not allow two implicit owners. Build one Pedro Auto integration lane
  around a valid pinned Follower configuration and one odometry hardware owner, then expose an
  explicit converted field-pose and coherent timestamped motion/delta contract to Phoenix consumers.
  The decision gate must choose which side owns Pinpoint only after tracing Pedro's custom-localizer/
  reset APIs and Phoenix correction requirements; student routine code should not see that choice.
- **Completion:** `FollowerBuilder.build()` succeeds with the real drivetrain/localizer; exactly one
  owner initializes, updates, resets, and closes each hardware resource; Pedro and Phoenix start
  poses map through a tested named transform; path following and targeting consume a coherent pose,
  translational velocity/delta, and angular velocity/delta snapshot; correction/reset/drift behavior
  is documented and observable; invalid or placeholder config fails during INIT with an actionable
  readiness message.
- **Decision record (2026-07-11):**
  - **Confirmed behavior:** the checked-in `Constants.createFollower(...)` calls only
    `pathConstraints(...)`. In pinned Pedro 2.1.2, `FollowerBuilder` leaves both `localizer` and
    `drivetrain` null and performs no build validation. `build()` passes those nulls to `Follower`;
    `PoseTracker` immediately calls `localizer.resetIMU()`, and `Follower.breakFollowing()` also
    immediately reaches the drivetrain. The current production factory therefore cannot construct
    a usable follower and fails with a vendor null dereference rather than an actionable INIT error.
    Adding both missing builder calls would construct a follower, but the native Pedro
    `PinpointLocalizer.update()` polls Pinpoint inside every `Follower.update()`, while Phoenix's
    existing localization lane independently constructs, configures, resets, and polls another
    `PinpointOdometryPredictor` for the same hardware name. That is two implicit owners. The two
    stacks also publish different poses: Pedro's 144-inch field with origin `(72, 72)` and Phoenix's
    current FTC field frame.
  - **Pinned lifecycle trace:** Pedro's native Pinpoint localizer owns hardware configuration, pose,
    instantaneous translational/angular velocity, unwrapped total heading, start/current pose
    mutation, IMU reset, and tuning accessors. `Follower.update()` couples its localizer update to
    path control and the final drivetrain write. `Follower.setPose(...)` is not a safe same-cycle
    correction primitive in 2.1.2 because `PoseTracker.setPose(...)` does not invalidate its cached
    current pose. Pedro provides no timestamped AprilTag-fusion API, but its supported custom-
    localizer seam is `FollowerBuilder.setLocalizer(...)`. Phoenix's existing
    `PinpointOdometryPredictor` already owns the desired hardware config, timestamped pose/delta,
    reset path, correction replay, and push-back behavior; it currently lacks only the same-sample
    raw velocity and unwrapped-heading snapshot required to satisfy Pedro's passive `Localizer`
    contract without guessing from a correction-bearing pose delta.
  - **Current callers and common path:** `Constants.createFollower(hardwareMap)` is called by the
    generated `Tuning` menu, the standalone `PedroTest`, and
    `PhoenixPedroAutoOpModeBase`. Phoenix's annotated Autos all flow through that base; student
    route code lives in `PhoenixPedroPathFactory`, and routine code continues to use the existing
    adapter through `RouteTask`, guidance Tasks, and `PhoenixCapabilities`. The base is the only
    production setup call site. `PhoenixRobot`, both Pedro guides, framework drive/route examples,
    telemetry, and the localization tester are the other affected API/documentation consumers.
  - **Alternatives considered:** leave the invalid runtime and document it; add only
    `.mecanumDrivetrain(...)` and `.pinpointLocalizer(...)`; let Phoenix own Pinpoint and implement a
    passive Pedro `Localizer`; let Pedro own Pinpoint and expose a Phoenix predictor/reset view;
    use separate physical localization systems with synchronization/drift diagnostics; add a new
    generic combined drive/localization runtime interface or value; or keep the ownership choice
    implicit and infer a predictor from the drive sink at runtime.
  - **Simplicity comparison:** documentation cannot make the current factory run, and the two-line
    builder fix hides duplicate resets/polls and disagreeing field frames. Native Pedro ownership is
    the lower-code runner-up and keeps generated tuning unchanged, but it makes the Follower write
    drive output before Phoenix localization/correction, applies a correction to path control one
    cycle later, moves Auto calibration out of the established Phoenix lane, and needs a vendor
    PoseTracker cache/offset workaround for same-cycle route starts. A passive Pedro view over the
    Phoenix owner adds integration code but no route/routine concept: one same-cycle kinematic
    snapshot supplies the exact velocity/heading data, the current localization-first loop stays
    true, and TeleOp/Auto use one checked-in Pinpoint configuration. Standalone Pedro tuning is a
    clearly named tool-only native factory derived from that same physical config, not a second
    production choice. Two sensors add hardware and drift policy. A generic combined runtime
    interface adds a concept to every route integration without improving the Phoenix common path.
  - **Chosen hardware and integration ownership:** construct one Pedro Auto integration lane around
    one `PinpointOdometryPredictor`, one passive Pedro `Localizer` view, one valid native Pedro
    mecanum drivetrain, one Follower, and the existing `PedroPathingDriveAdapter`. The predictor is
    the sole object that acquires, configures, resets, polls, and rebases Pinpoint. Its immutable
    same-cycle kinematic snapshot adds field-frame X/Y velocity, angular velocity, unwrapped physical
    heading, cycle, and timestamp alongside its existing pose/delta. The passive Pedro view never
    polls hardware: it verifies that the predictor has a current snapshot, converts pose and
    velocity explicitly, and supplies Pedro's read/reset/start contract through the lane. The
    adapter remains the sole Follower heartbeat and drivetrain-stop owner. Do not create a second
    Pinpoint localizer, expose the raw driver, add a broad velocity interface to core, use reflection,
    or select ownership with `instanceof`.
  - **Phoenix API and loop ownership:** replace the one-argument Auto initialization path with the
    explicit backend-neutral pair `robot.initAuto(adapter, pedroRuntime.motionPredictor())`; the
    advanced overload receives the same predictor before its aim-enable/override sources. Do not
    retain the old normal path. `FtcOdometryAprilTagLocalizationLane` gains an injected-
    `MotionPredictor` construction path for Auto while its ordinary FTC/TeleOp construction still
    creates the same configured `PinpointOdometryPredictor`. Preserve the current explicit Auto
    order: `localization/predictor/correction -> targeting -> owned Pedro heartbeat -> TaskRunner ->
    scoring -> telemetry`. The passive Pedro localizer consumes the already-current snapshot, so
    Follower control sees an accepted pushed correction in that same heartbeat. Phoenix remains
    unaware of Pedro types, and route geometry/strategy remains outside `PhoenixRobot`.
  - **One configuration story:** the project-specific factory must build the complete production
    lane and validate it before calling the unvalidated vendor builder. Motor names/directions and
    Pinpoint name/offsets/resolution/directions/yaw scalar come from the selected defensive
    `PhoenixProfile` snapshot; `Constants` retains only Pedro-specific follower/controller/path
    tuning. Standalone Pedro `Tuning`/`PedroTest` use an explicitly named tool-only native-localizer
    factory derived from the same profile, while Phoenix production has one lane factory. Reject
    blank/duplicate motor names, a blank Pinpoint name, a missing field transform, non-finite
    tuning/offsets, unsupported reset assumptions, and failed hardware lookup with a message naming
    the exact setting. Broader calibration acknowledgements and route readiness remain PHX-02.
  - **Coordinate, time, and motion contract:** require one named, inverse-tested Pedro-to-Phoenix
    field transform when the lane is built; Phoenix Decode selects the inverted FTC convention
    explicitly. Never infer the season frame or trust the pinned `FTCCoordinates`/
    `InvertedFTCCoordinates` implementation without regression tests—the 2.1.2 source applies the
    same signed quarter-turn in both conversion directions. Convert pose origin, axes, and absolute
    heading at the integration boundary; rotate velocity components without applying field-origin
    translation. Phoenix snapshots use inches, radians, CCW-positive yaw, and the shared
    `LoopClock` cycle/time. The first predictor sample and every deliberate pose rebase publish
    `MotionDelta.none(...)`; raw same-sample velocity/physical heading remain correction-spike-free.
    Name vendor-frame values such as `pedroStartPose` explicitly.
  - **Start, correction, and drift policy:** starting pose assignment goes through the integration
    lane before the first heartbeat so the predictor/global initializer and passive Pedro view share
    one pose. The lane consumes only Pedro's constructor-triggered duplicate `resetIMU()` after the
    predictor's controlled INIT reset; later reset attempts either use one coordinated lane operation
    or fail with an instruction naming it, never silently no-op. By default the existing corrected-
    estimator option `enablePushCorrectedPoseToPredictor = true` pushes each accepted filtered
    correction through the shared predictor, so the passive view gives path following the corrected
    pose in the same downstream heartbeat. Keeping it false is the explicit advanced targeting-only
    policy; telemetry then shows raw predictor pose, corrected global pose, and drift. Start/reset/
    correction rebases clear the motion delta while preserving or explicitly resetting physical
    velocity and accumulated heading; they must not look like robot motion.
  - **Rejected designs:** reject the local two-builder-call fix because it leaves two hardware
    owners; reject native Pedro production ownership because it weakens Phoenix's established
    localization-first graph and same-cycle correction for less internal code but no routine-code
    gain; reject an active/passive mode hidden inside one standalone follower factory; reject
    independent estimators because Phoenix has no requirement worth their hardware/drift complexity;
    reject a generic combined runtime interface because two existing narrow seams express the
    production dependency; and reject legacy overloads/runtime inference because they allow the
    invalid ownership graph to return.
  - **Verification plan:** use the real pinned Follower with fake localizer/drivetrain boundaries
    plus focused adapter/localization/Phoenix tests. Cover valid construction and each actionable
    config failure; exactly one Pinpoint configuration, controlled INIT reset and poll plus one
    drivetrain writer; a passive localizer that never polls; first/next/reset kinematic snapshots,
    raw velocities, physical total heading, timestamps, translation/yaw deltas, wraparound, invalid
    poses, and correction rebasing; named Pedro-to-
    Phoenix round trips for center, axes, headings, velocity, and Decode's inverted FTC frame;
    default correction-to-path behavior, targeting-only drift, and a route starting in the same
    cycle as a correction; starting-pose synchronization; INIT retry/shutdown; and unchanged routine
    Task call sites. Migrate `Constants`, `Tuning`, `PedroTest`, the Phoenix Auto base/path factory,
    telemetry, localization tester, all Javadocs/guides/examples, and exhaustive caller searches.
    Run `:TeamCode:testDebugUnitTest`, `:TeamCode:compileDebugJavaWithJavac`, XML result counts,
    `git diff --check`, static ownership/frame searches, adversarial review, and Android Studio
    inspection. On robot, confirm valid INIT, coordinate signs from known poses, one Pinpoint update
    per cycle, path response to an accepted correction, targeting-only drift telemetry, hold behavior,
    and immediate stopped motor output.
  - **Approval gate:** the leading hypothesis remains viable and deliberately left the Pinpoint
    owner open, so this is not a material departure from the tracker. The design nevertheless changes
    Phoenix's public Auto initialization and establishes cross-library pose/correction authority.
    It is a major API/lifecycle decision and requires explicit user approval before moving this item
    to **In progress** or editing Java code.
  - **Approval:** the user approved the PEDRO-02 design on 2026-07-11.
  - **Implementation (2026-07-12):** production Phoenix Pedro Auto now constructs one
    `PedroPathingRuntime` containing one profile-configured `PinpointOdometryPredictor`, one passive
    Pedro localizer, one native mecanum/Follower graph, and the existing cycle-aware
    `PedroPathingDriveAdapter`. The predictor is the sole physical Pinpoint owner. Its immutable
    `PinpointKinematicSnapshot` publishes one cycle/time-tagged field pose, raw field velocity,
    angular velocity, and unwrapped physical heading; polling is same-cycle idempotent, and pose
    rebases clear correction-shaped deltas while retaining physical motion values. The passive
    localizer never touches hardware, consumes only the current Phoenix cycle, converts through the
    inverse-tested named Decode field transform, permits the one pinned Follower-constructor reset,
    and rejects later uncoordinated pose/reset/update calls with actionable errors.
  - **Phoenix ownership and configuration:** `PhoenixRobot.initAuto(...)` now requires the explicit
    backend-neutral pair `DriveCommandSink` plus `MotionPredictor`; the injected predictor feeds the
    normal localization lane before targeting and the adapter heartbeat. The project `Constants`
    factory derives physical motor and Pinpoint configuration from a defensive `PhoenixProfile`,
    retains Pedro-only controller/path tuning, validates configuration before hardware use, and
    separates the production runtime from the clearly named tool-only native Follower used by
    generated Pedro tuning and `PedroTest`. The Auto base applies the explicitly named
    `pedroStartPose` before the first heartbeat, keeps route strategy outside `PhoenixRobot`, and
    preserves `localization/correction -> targeting -> Pedro heartbeat -> TaskRunner -> scoring ->
    telemetry`. Telemetry now exposes raw/global translation and heading drift.
  - **Pinned Pedro path-constraint containment:** final audit found that Pedro 2.1.2 first reads a
    process-wide default in its no-argument path builder and then, even when explicit constraints
    are supplied, `PathBuilder.build()` replaces every path's constraints with that same global
    default through `PathChain`. `PedroPathingRuntime.pathBuilder()` now contains both vendor defects
    in a private version-pinned subclass: it owns copied defaults, preserves deliberate per-path
    overrides across `build()`, retains Pedro's braking-start semantics, and fails fast if the
    pinned construction behavior changes. Student path code keeps the normal
    `runtime.pathBuilder()...build()` shape; it does not mutate global state or require a repair
    step.
  - **Documentation:** framework overview/design/loop/localization guides, drive and route
    Javadocs, both Pedro guides, Phoenix Architecture and calibration guidance, the localization
    tester, Auto base/path factory, and telemetry documentation now describe the same owner, field,
    timing, correction, start-pose, tuning-tool, and shutdown contracts. All local links in the 11
    modified Markdown files resolve.
  - **Automated verification (2026-07-12):** `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` pass. XML reports 179 tests across 24 suites with zero
    failures, errors, or skips. New focused tests cover Pinpoint configuration and same-cycle
    kinematic snapshots; pose/velocity/heading transforms and round trips; the real pinned Follower
    with passive-localizer reset, heartbeat, stale-sample, rebase, and raw-mutation contracts;
    runtime validation and actionable failures; profile-to-mecanum/Pinpoint mapping; and copied
    builder defaults plus multiple paths and per-path constraint overrides. Existing JDK
    21/source-8 and SDK deprecation warnings remain unchanged.
  - **Static and independent verification:** exhaustive caller searches find one production runtime
    factory and adapter construction, no legacy one-argument Auto initializer, no native Pedro
    Pinpoint owner in production Auto, and no raw production Follower heartbeat outside the adapter;
    raw updates/localizer construction remain only in the explicitly tool-only Pedro programs.
    `git diff --check` and the changed/untracked trailing-whitespace scan pass. Independent reviews
    confirmed the pinned PathChain overwrite from source and bytecode, then found no blocking
    correctness, ownership, Framework Principles, documentation, or student-simplicity issue in the
    contained workaround or the completed PEDRO-02 graph.
  - **Manual verification:** the user approved the Android Studio review on 2026-07-12. On-robot
    observation remains recommended during the next normal bring-up: confirm successful INIT with
    the selected profile; known-pose X/Y/heading signs; one physical Pinpoint poll per loop;
    same-cycle path response to an accepted pushed correction; understandable targeting-only drift
    when push-back is disabled; route hold behavior; and immediate/stable motor zero on cancellation
    and stop.

### ROUTE-02 - Truthful route terminal status

- **Problem to confirm:** `RouteTask` currently converts any post-update `!follower.isBusy()` state
  into `TaskOutcome.SUCCESS`. A follower may instead become not busy because it reached the endpoint,
  tripped a path timeout/stall safeguard, was broken by a callback or obstacle policy, was replaced,
  or failed internally. PHX-03 cannot choose a safe continue/fallback/abort policy if those states
  are collapsed before robot strategy sees them.
- **External evidence:** Cuttlefish deliberately skips routes after its bonk detector fires, replaces
  active paths, attaches velocity/parametric break conditions, and performs a match-time takeover.
  Its
  [path scheduler](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/pathaction/PathActionScheduler.java)
  also demonstrates the danger of reconstructing terminal meaning from loose booleans: deferred
  paths and hold-end stopping follow different branches and can be stopped or restarted
  inconsistently.
- **Expanded Worlds evidence:** SaMoTech's intake route can end because it filled, timed out, stalled,
  or invoked obstacle recovery. LOAD races `FollowPath` against a delayed low-velocity condition,
  which otherwise makes an interrupted/stalled route indistinguishable from success. Tech Tigers
  detects some drive timeouts but maps several of them back into ordinary state-complete events.
  Three different implementations therefore reconstruct strategy from terminal side effects; the
  route integration boundary needs to preserve the reason once.
- **Alternatives to compare:** document that `!isBusy()` means success; add a route endpoint
  predicate in each robot routine; extend `RouteFollower` with a small status/terminal-result
  contract; keep status Pedro-specific in its integration lane; or broaden generic `TaskOutcome`.
  Compare which layer can reliably distinguish a route generation and terminal reason without
  exposing vendor state to routine code.
- **Leading hypothesis:** the integration owner that starts and stops a route should retain its
  generation and terminal reason, and `RouteTask` should consume one narrow backend-neutral route
  result. The normal follow call should stay short. Do not add generic `FAILED` outcomes or a large
  route event model unless the decision gate proves existing Task outcomes cannot carry the required
  policy safely.
- **Completion:** focused tests distinguish natural endpoint completion, follower timeout/stall,
  callback break, external replacement, Task timeout, active cancellation, and unknown terminal
  state; stale status from a prior route cannot complete a new one; the typed result needed by
  PHX-03 is available without implementing that later robot-policy item here; and telemetry names
  the terminal reason without leaking Pedro types.
- **Decision record (2026-07-12):**
  - **Confirmed framework defect and baseline:** `RouteTask.update(...)` advances its follower and
    converts every post-update `!isBusy()` into `TaskOutcome.SUCCESS`. The generic seam supplies no
    route identity or terminal reason, and its unscoped `cancel()` can stop a replacement route if
    an older Task is cleaned up late. The focused pre-change baseline passes 23 tests across
    `RouteTaskSingleUseTest` and `PedroPathingDriveAdapterTest` with zero failures, errors, or skips;
    one generic test explicitly protects the current ambiguous `busy == false -> SUCCESS` behavior.
  - **Confirmed pinned Pedro behavior:** Phoenix pins Pedro core/FTC `2.1.2`. Its one
    `Follower.update()` heartbeat advances a path segment when either the segment reaches its
    parametric end or a private stuck timer expires. On the final segment it becomes not busy when
    either endpoint velocity/translation/heading constraints pass or a private endpoint timeout
    expires. `breakFollowing()`, hold-end completion, callback interruption, manual takeover, and
    route replacement also clear the same busy flag, while clearing the private timeout/stuck
    evidence. Therefore terminal meaning must be classified and retained by the adapter during its
    owned heartbeat; it cannot be reconstructed later from `isBusy()`.
  - **Current callers and migration surface:** `PedroPathingDriveAdapter` is the only production
    `RouteFollower`; Phoenix's four Auto strategies reach it through two short helpers in
    `PhoenixPedroAutoRoutineFactory`. Other implementations are focused test fakes. A coherent
    breaking seam change therefore has a small migration surface, and the normal student call can
    remain `RouteTasks.follow("outbound", adapter, path, cfg)`.
  - **Alternatives considered:** document that not-busy means success; repeat an endpoint predicate
    in each routine; retain a mutable current-route status/generation on `RouteFollower`; add a
    second optional status interface beside `isBusy()`; keep truth Pedro-specific; expose generation
    counters to routine code; or add generic `TaskOutcome.FAILED` plus new aggregation and branching
    semantics. Documentation and call-site predicates cannot observe cleared callback/stall state.
    Pedro-only or optional status leaves the generic route seam dishonest. A mutable current status
    lets a newer run satisfy or be cancelled by an older Task. Exposed counters and a global Task
    outcome expansion add concepts and framework-wide policy that ROUTE-02 does not need.
  - **Chosen public contract:** replace `RouteFollower.follow(R)`, `isBusy()`, and global `cancel()`
    with `RouteExecution follow(R)` plus the existing optional `update(clock)` heartbeat. The small
    backend-neutral execution handle exposes only `status()` and active-only, idempotent `cancel()`.
    Its private identity binds status and cancellation to exactly one start, so an adapter can mark
    an older execution `REPLACED` without exposing a generation counter and cancellation of that old
    handle cannot stop the new route. Do not retain parallel legacy methods or add a builder.
  - **Chosen status vocabulary:** one backend-neutral `RouteStatus` enum carries `NOT_STARTED`,
    `ACTIVE`, `COMPLETED`, `FOLLOWER_TIMEOUT_OR_STALL`, `INTERRUPTED`, `REPLACED`, `TASK_TIMEOUT`,
    `CANCELLED`, `FAILED`, and `UNKNOWN_TERMINAL`. The combined follower timeout/stall status is the
    smallest truthful policy category because Pedro can prove an abnormal constraint/stuck ending
    but does not expose which private timer won in every final-loop edge case. Adapter-owned stop,
    callback break, and manual takeover use `INTERRUPTED`; another supported follow uses `REPLACED`;
    and caught start/update exceptions record `FAILED` before the existing fail-closed rethrow.
    Unsupported raw vendor lifecycle calls bypass those truthful guarantees and remain unsupported;
    detectable unexplained transitions are retained as `UNKNOWN_TERMINAL` rather than guessed.
  - **Chosen Task behavior:** `RouteTask` retains the exact execution, exposes
    `getRouteStatus()`, and uses its own `TASK_TIMEOUT`/`CANCELLED` result before cancelling the
    execution. `RouteTasks.follow(...)` returns `RouteTask<R>` rather than hiding it as `Task`; this
    remains source-compatible with ordinary Task sequences and gives later PHX-03 policy typed
    access without a cast. `COMPLETED` maps to `TaskOutcome.SUCCESS`, follower/Task timeout maps to
    `TIMEOUT`, and interrupted/replaced/failed/unknown terminals map to the existing fail-closed
    `CANCELLED` compatibility bucket while their precise route status remains available. Do not add
    generic `FAILED` or change sequence/parallel/branch policy in this item.
  - **Pedro classification rule:** explicit adapter lifecycle reasons win. Around the one owned
    heartbeat, retain the expected chain/final path and inspect segment progress plus final physical
    velocity, translation, and heading constraints. A non-parametric segment advance caused by the
    latched stuck condition terminalizes and stops the route as `FOLLOWER_TIMEOUT_OR_STALL` rather
    than silently skipping ahead. Final completion is `COMPLETED` only when the original final path
    reached its parametric end and all endpoint constraints pass; otherwise a follower-driven end is
    `FOLLOWER_TIMEOUT_OR_STALL`. If constraints become true in the same heartbeat as Pedro's hidden
    timeout, only the achieved state is observable, so report `COMPLETED`. Unexpected route/path
    identity changes or untagged not-busy transitions become `REPLACED` or `UNKNOWN_TERMINAL`, never
    success. Raw Follower lifecycle mutation remains unsupported; route building/read-only
    inspection through the exposed follower stays valid.
  - **Framework Principles and student simplicity:** route behavior remains non-blocking and uses
    the existing one cycle-deduplicated Pedro heartbeat. Vendor inspection stays inside the explicit
    integration boundary, routine strategy receives only Phoenix types, and common Auto code gains
    no new line or concept. The added execution/status concepts are paid only by framework and
    integration authors, where they replace ambiguous global state with one clear API.
  - **Scope boundary:** ROUTE-02 will expose truthful data and backend-neutral telemetry, but will
    not implement PHX-03 continue/fallback/abort policy, ROUTE-01 deferred construction, a callback
    DSL, a second scheduler, or changes to generic Task composition semantics.
  - **Verification plan:** replace the ambiguous generic baseline with tests for every status,
    single-use/pre-start/idempotent cancellation, Task timeout, stale-result isolation, and old-handle
    cancellation after replacement. Extend the real pinned-Pedro harness for natural hold/no-hold
    completion, impossible endpoint constraints, intermediate/final stall, callback interruption,
    manual takeover, replacement, start/update failure, unknown mutation, immediate physical stop,
    and same-cycle root/Task deduplication. Compile all four unchanged short Phoenix routine shapes,
    expose the latest generic status in Auto telemetry, synchronize route/Pedro/Phoenix docs, run
    the full TeamCode unit suite and debug Java compile, inspect callers and raw follower lifecycle
    calls, check documentation links and diffs, and request Android Studio review. Deliberate
    obstruction, endpoint timeout, and status labels still require on-robot observation.
  - **Approval gate:** the leading hypothesis remains the smallest robust direction, refined with a
    per-execution handle so generation mechanics never reach students. Replacing the public
    `RouteFollower` lifecycle is nevertheless a major API decision, so explicit user approval is
    required before this item moves to **In progress** or any Java implementation is edited.
  - **Approval:** the user approved the ROUTE-02 design on 2026-07-12.
  - **Implementation (2026-07-13):** `RouteFollower.follow(...)` now returns one public
    `RouteExecution` for that exact start, with a backend-neutral `RouteStatus` snapshot and
    active-only cancellation. The old follower-global busy/cancel methods are removed rather than
    retained as a parallel path. `RouteTasks.follow(...)` returns the typed `RouteTask<R>` while
    preserving the same one-line call and normal use as a `Task`.
  - **Task lifecycle and outcome handling:** `RouteTask` retains its exact execution, samples it
    before and after the optional follower heartbeat, and exposes `getRouteStatus()`. A stale Task
    cannot heartbeat or cancel a replacement, and cancellation first preserves a terminal reason
    already observed by the root heartbeat. `COMPLETED` maps to `SUCCESS`; follower/Task timeout
    maps to `TIMEOUT`; interruption, replacement, cancellation, failure, and unknown terminal state
    map to the existing fail-closed `CANCELLED` bucket without losing the precise route status.
    Synchronous-start/status contract violations and start/update/status failures fail closed with
    exact-handle best-effort cleanup while retaining the primary exception.
  - **Pedro implementation:** `PedroPathingDriveAdapter` retains one per-start execution and
    classifies final completion, endpoint timeout/stall, non-parametric segment stall, adapter or
    callback interruption, replacement, detected unexplained transitions, and caught failures
    around its one cycle-deduplicated heartbeat. Endpoint classification mirrors pinned Pedro 2.1.2
    using the final Path's live constraints and Pedro's cached closest-point heading goal. Route
    start and heartbeat work are fail-closed transactions; invalid/partial starts, vendor getter
    failures, same-cycle reentry, callback-initializer reentry, callback stop, and cleanup failure
    retain truthful status and immediate/stable zero without allowing an old handle to stop a newer
    route. Phoenix Auto telemetry now includes the latest backend-neutral `route.status`.
  - **Documentation and callers:** Framework Principles, repository instructions, route/Task
    Javadocs, framework design/getting-started/loop/capability guides, the Pedro guide, Phoenix
    Architecture, and the Phoenix Pedro Auto guide describe the same per-start identity, heartbeat,
    raw-lifecycle boundary, status mapping, and failure-policy boundary. Structural route-to-score
    examples explicitly warn that generic sequences do not choose continue/fallback/abort policy.
    Phoenix's four strategies retain their short existing helper call shape; no route policy,
    deferred route construction, callback DSL, or second scheduler was added.
  - **Automated verification (2026-07-13):** `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` pass. XML reports 221 tests across 25 suites with zero
    failures, errors, or skips. The focused ROUTE-02 suites pass 65 tests: 7 single-use tests, 26
    generic route-status/lifecycle tests, and 32 real pinned-Pedro adapter tests. Existing JDK
    21/source-8 and SDK deprecation warnings remain unchanged.
  - **Static and independent verification:** caller searches find the Pedro adapter as the only
    production `RouteFollower`, no legacy busy/cancel route seam, and no production raw Pedro
    lifecycle calls outside the adapter/runtime boundary; raw calls remain in generated tuning and
    tool-only Pedro programs. `git diff --check`, the changed/untracked trailing-whitespace scan,
    and local-link checks for all 10 changed Markdown files pass. Independent core API and
    documentation/caller reviews found no blocking Framework Principles, scope, lifecycle,
    documentation, or student-simplicity issue. A separate audit against cached pinned Pedro 2.1.2
    source confirmed the endpoint/stall rules, cached heading behavior, hold-end heartbeat,
    immediate-zero break behavior, failure cleanup, replacement identity, and reentrancy handling.
  - **Manual verification:** the user approved the Android Studio review on 2026-07-13. On-robot
    observation remains recommended for a normal route, deliberate obstruction/stall, impossible
    endpoint constraints, callback/manual interruption, immediate and stable motor zero, hold-end
    behavior, and understandable Driver Station `route.status` labels.

### ROUTE-01 - Start-time route construction

- **Problem to confirm:** `RouteTask` stores an already-built route. Competitive Autos often need to
  choose a target from INIT vision and construct the next path from the follower's actual pose at
  the moment that phase begins, after earlier path error or a global fallback decision. Prebuilding
  every variant or rebuilding the whole routine duplicates strategy and makes stale start geometry
  easy to use.
- **External evidence:** Cuttlefish's
  [deferred path builder](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/pathaction/PathActionBuilder.java)
  resolves suppliers when a path action starts, and its
  [Far Auto](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Far.java)
  builds cycle and park geometry from live pose and vision state.
- **Expanded Worlds evidence:** SaMoTech generates an intake route from the current follower pose and
  live vision selection when that step executes. LOAD's match-time leave wrapper demonstrates the
  opposite failure mode: it asks for a fallback after the timer wins, but assembles the park path
  from follower pose while the wrapper is built, so the encoded start can already be stale.
- **Alternatives to compare:** prebuild every route; rebuild the containing routine; write a custom
  Pedro Task at each call site; add an eager route-start-pose correction; or add one explicit
  start-time `Supplier<R>` route factory. Also compare one general `Tasks.defer(Supplier<Task>)`
  lifecycle wrapper: it may avoid repeating deferred-child semantics, but it should not broaden the
  beginner API without a non-route caller. Compare generic `RouteTasks` ownership with a Pedro-only
  helper and define how construction failures become visible.
- **Leading hypothesis:** add an explicitly named deferred/generated route factory that resolves its
  supplier exactly once at the Task's own `start(clock)` boundary and then uses the normal route
  lifecycle. Geometry creation and sensor-to-target decisions remain robot-owned path-factory work;
  core Tasks see no Pedro type and gain no Auto DSL.
- **Completion:** tests cover sampling current pose/vision exactly once at start, sequence start in a
  sub-loop, null/throwing factories, cancellation, route timeout, truthful terminal status, and
  debug naming. A compiling example shows both eager fixed geometry and a current-pose fallback
  without exposing the raw follower throughout the routine.
- **Decision record (2026-07-13):**
  - **Confirmed behavior and baseline:** `PhoenixPedroAutoOpModeBase` calls
    `PhoenixPedroPathFactory.build(...)` during INIT, that factory builds both the outbound and
    return `PathChain`, and `PhoenixPedroAutoRoutineFactory` later gives those stored objects to
    `RouteTasks.follow(...)`. `RouteTask` requires and retains one concrete route in its constructor;
    its own `start(clock)` delays only `follower.follow(route)`, not geometry construction. A return
    or fallback route therefore keeps the INIT-time start pose even after earlier path error,
    interruption, correction, or strategy changes. `SequenceTask.fromSuppliers(...)` does not solve
    this because it deliberately invokes its suppliers while constructing the sequence. The focused
    baseline on 2026-07-13 passes all 33 existing generic route tests (7 single-use and 26 status/
    lifecycle tests) with zero failures, errors, or skips.
  - **Current callers and common path:** `PedroPathingDriveAdapter` is the only production
    `RouteFollower`. The only production `RouteTasks.follow(...)` calls are the short outbound and
    return helpers in `PhoenixPedroAutoRoutineFactory`; all four Phoenix strategies reuse those two
    helpers. There is no framework-tool or modern-example route caller. `PhoenixPedroPathFactory`
    already owns Pedro geometry and the checked `PedroPathingRuntime.pathBuilder()`, while the
    runtime explicitly permits read-only Follower inspection at that boundary. Routine code should
    continue seeing only a path-factory method reference, the adapter, and Phoenix Tasks—not a raw
    Follower or path builder.
  - **Alternatives considered:** keep all geometry eager and document the stale-pose risk; prebuild
    every pose/vision variant; rebuild the containing routine; mutate a route's start pose when it
    begins; write a custom Pedro Task at each caller; add a Pedro-only deferred helper; add a same-
    name `follow(..., Supplier<R>, ...)` overload; add a public `RouteFactory<R>` or staged builder;
    add `Function<LoopClock, R>`; add a general `Tasks.defer(Supplier<Task>)`; or add one explicitly
    named route-specific start-time factory using the standard `Supplier` type.
  - **Simplicity comparison:** fixed geometry should remain the ordinary one-line
    `RouteTasks.follow(adapter, route, cfg)` path. A dynamic route needs only one method reference or
    lambda, not another Task wrapper, builder, or public factory type. A same-name Supplier overload
    hides the timing difference and becomes ambiguous for null or a route type that itself implements
    `Supplier`. `Tasks.defer(...)` adds a general wrapper lifecycle without another demonstrated
    caller, makes the route call more deeply nested, and normally hides the typed `RouteTask` needed
    for precise `RouteStatus` policy. Passing `LoopClock` into a new factory concept is unnecessary:
    the composition root updates localization, targeting, and the Pedro heartbeat before Auto Tasks,
    so a robot-owned path factory can read the current-cycle snapshots when its supplier runs.
  - **Chosen public API:** keep both existing eager `RouteTasks.follow(...)` overloads unchanged and
    add parallel named/default `RouteTasks.followBuiltAtStart(...)` overloads that accept
    `Supplier<? extends R>` and still return `RouteTask<R>`. The name exposes the only semantic
    difference; `followAtStart` would be misleading because eager Route Tasks also begin following
    at start, while `deferred` is jargon and `generated` omits when generation occurs. Do not add a
    public supplier constructor, `RouteFactory` type, builder, or second Task class; `RouteTasks`
    uses an internal/package-private construction path in the existing `RouteTask`.
    ROUTE-03 later superseded the default/named/config construction spellings while preserving this
    eager-versus-start-time capability distinction.
  - **Chosen lifecycle, failure, and debug contract:** validate the supplier when the Task is built,
    record the Task's single start attempt before invoking it, and call it exactly once at that
    Task's own `start(clock)` boundary. Never invoke it during task construction, pre-start
    cancellation, queue removal, update, status reads, or debug output. A null result or factory
    `RuntimeException` makes the Task terminal with `RouteStatus.FAILED` and the existing fail-closed
    `TaskOutcome.CANCELLED`, leaves the follower untouched, and throws an actionable error naming the
    Task while preserving the original cause. A second start cannot call the supplier again. If
    cancellation re-enters while the supplier is resolving, cancellation remains terminal and the
    Task must not start the follower afterward. Once a non-null route exists, the existing exact
    `RouteExecution`, timeout, heartbeat, status, and cancellation semantics remain unchanged. Debug
    output reports eager versus built-at-start resolution and a pending route class without sampling
    the supplier; suppliers must remain quick and non-blocking.
  - **Phoenix/Pedro ownership and compiling proof:** keep one fixed outbound path eager. Retain the
    existing robot-owned `PhoenixPedroPathFactory` in `PhoenixPedroAutoContext`, add its narrow
    `buildReturnFromCurrentPose(...)` operation, and have the routine pass a lambda or method
    reference for that operation to `followBuiltAtStart(...)`. The path factory alone reads the
    current Pedro pose through the runtime's supported read-only inspection and builds through
    `PedroPathingRuntime.pathBuilder()`; remove the stale prebuilt return path when it has no caller.
    Live vision interpretation remains a robot path-factory/service responsibility over a current
    immutable selection snapshot; no Pedro, vision, alliance, or game-strategy type enters core
    route APIs. Raw Follower lifecycle calls remain unsupported.
  - **Rejected and deferred scope:** do not add general deferred Task composition, a second scheduler,
    a route-progress DSL, route-failure policy, match-time takeover, or alliance transforms in this
    item. Those broader behaviors remain TASK-04, PHX-03, PHX-04, and FIELD-01. Do not change
    `RouteFollower.follow(R)` to accept factories: the integration should continue receiving one
    already-resolved route and owning only its concrete per-start execution.
  - **Verification plan:** add focused generic tests for no early sampling; live pose/vision values
    sampled exactly once at a direct and sequence sub-loop start; eager behavior unchanged; null and
    throwing suppliers; second start after factory failure; pre-start, active, repeated, and
    reentrant cancellation; timeout and every truthful terminal `RouteStatus`; and debug output
    before/after resolution without side effects. Add a compiling Phoenix Pedro example/test with
    an eager outbound route plus a current-pose return/fallback built through the checked runtime
    builder. Search every route caller and raw Follower lifecycle call; synchronize RouteTask/
    RouteTasks Javadocs, Framework Principles, Framework Overview, Recommended Robot Design, loop
    guidance, both Pedro guides, and Phoenix Architecture; run the focused route/Pedro suites, full
    `:TeamCode:testDebugUnitTest`, `:TeamCode:compileDebugJavaWithJavac`, XML counts, link/diff/
    whitespace checks, independent reviews, and Android Studio inspection. On robot, observe that a
    deliberately displaced/interrupted outbound route produces a return path whose first point is
    the live pose and that cancellation still reaches immediate stable zero.
  - **Approval gate:** the leading hypothesis remains the smallest robust design; the
    `followBuiltAtStart(...)` name is only a clarity refinement. This adds a public route-task API and
    start-time callback lifecycle, so explicit user approval is required before moving to
    **In progress** or editing Java implementation.
  - **Approval:** the user approved the ROUTE-01 design on 2026-07-13.
  - **Implementation (2026-07-13):** `RouteTasks` now provides named and default
    `followBuiltAtStart(...)` overloads while both eager `follow(...)` overloads remain unchanged.
    The existing `RouteTask` owns the internal resolution path: it samples the supplier once after
    recording its single start attempt, retains the concrete route, reports pending/resolved debug
    state without sampling, and fails closed before touching the follower when construction returns
    null or throws. Active, repeated, and reentrant cancellation preserve the exact per-start
    execution and truthful terminal status, including cancellation that occurs before
    `RouteFollower.follow(...)` returns its handle.
  - **Phoenix/Pedro proof:** `PhoenixPedroAutoContext` now retains the robot-owned path factory. The
    placeholder outbound path remains eagerly built during INIT; the return helper uses
    `followBuiltAtStart(...)` and `PhoenixPedroPathFactory.buildReturnFromCurrentPose(...)` to read
    the current Pedro pose exactly once and build through the checked runtime builder. The obsolete
    prebuilt `returnPath` was removed. Non-finite endpoints fail before Pedro receives them, and an
    already-at-target translation uses Pedro's point path rather than a zero-length line that would
    produce a non-finite closest-point parameter in pinned Pedro 2.1.2.
  - **Automated verification (2026-07-13):** the focused
    `RouteTaskStartTimeConstructionTest`, `RouteTaskSingleUseTest`, `RouteTaskStatusTest`, and
    `PhoenixPedroPathFactoryTest` suites pass 52 tests with zero failures, errors, or skips. The full
    `:TeamCode:testDebugUnitTest` task passes 240 tests across 27 suites with zero failures, errors,
    or skips, and `:TeamCode:compileDebugJavaWithJavac` succeeds. Existing JDK 21/source-8 and FTC SDK
    deprecation warnings remain unchanged.
  - **Static and independent verification:** searches find one eager outbound production caller,
    one built-at-start return caller, one supported read-only current-pose read in the Phoenix path
    factory, no stale production `returnPath`, and no raw Pedro lifecycle call in Phoenix robot code.
    `git diff --check`, a trailing-whitespace scan of all 17 changed/untracked files, and all 44 local
    links across the nine changed Markdown files pass. Independent core, documentation/caller, and
    pinned-Pedro audits found and drove fixes for reentrant cancellation before an exact execution
    handle existed, cancellation status being overwritten by a later start failure, one stale eager
    documentation example, and Pedro's zero-length-line non-finite geometry. Final re-reviews sign
    off with no actionable Framework Principles, lifecycle, ownership, documentation, or student-
    simplicity findings.
  - **Manual verification:** the user approved the Android Studio review on 2026-07-13. On robot,
    deliberately displace or interrupt the outbound phase and confirm the return path begins at the
    live pose, test the already-at-return-pose case, and confirm active cancellation still applies
    immediate stable zero.

### ROUTE-03 - Factory-only route Task configuration

- **Problem to confirm:** the route family currently exposes four `RouteTasks` factories, two
  equivalent public eager `RouteTask` constructors, nullable implicit defaults, and a mutable
  `RouteTask.Config` whose only answer is `timeoutSec`. A basic route creates a config, mutates that
  one field, and passes it immediately; many tests pass `null`. This makes a simple Pedro route
  learn a configuration noun and parallel construction layer without adding route capability.
  The current `timeoutSec > 0` check also silently treats `NaN`, negative infinity, zero, and
  negative values as no timeout, while positive infinity effectively never expires.
- **Current caller evidence:** `BasicPedroAutoRoutine` constructs a fresh config solely to set a
  four-second timeout. Phoenix's `PhoenixAutoTasks.routeConfig(...)` likewise creates a fresh
  one-field config from the profile for each eager or start-time route call. Repository search found
  no modern production caller that stores, shares, composes, or independently validates the config.
  `RouteTask` itself must remain a public retained return type because callers use its precise
  per-start `RouteStatus`; that does not justify public constructors.
- **Alternatives to compare:** keep every path and improve documentation; remove constructors but
  retain mutable/null config; replace config with an immutable options value or builder; answer the
  sole timeout directly on each factory; expose a separately named no-timeout choice; retain the
  numeric `<= 0` sentinel; use generic `Tasks.withTimeout(...)`; or require a named route
  factory with no unnamed/default sibling. Compare default omission, finite-positive timeout,
  zero/negative/no-timeout, non-finite rejection, route-specific `TASK_TIMEOUT` truth, named
  diagnostics, call-site concepts, eager/start-time API parallelism, compatibility, and whether
  each remaining overload provides distinct value.
- **Leading hypothesis:** keep `RouteTasks` as the sole public construction layer and `RouteTask` as
  the status-bearing return type. If the complete caller audit confirms that timeout is the only
  configuration answer and is never reused, delete the public constructors, mutable config, and
  nullable default path without deprecation; answer timeout policy directly and keep eager and
  built-at-start factories parallel. Require a deliberate, validated distinction between a finite
  timeout and no timeout instead of letting non-finite values become accidental sentinels. Do not
  replace route-local timeout semantics with a generic wrapper that loses the retained route
  terminal reason, and do not add a staged builder or options wrapper used only inline.
- **Student-facing target:** a fixed route and a start-time route should each read as one direct,
  parallel factory call whose arguments are the actual robot decisions: diagnostic name, follower,
  concrete route or route factory, and timeout. The decision gate must explicitly decide whether an
  unnamed/default overload has distinct beginner value or is another learning path.
- **Completion:** exact API-surface tests enforce the chosen sole construction layer; every
  production, example, test, Javadoc, and Markdown caller migrates together; route timeout,
  no-timeout, default, zero/negative, and invalid non-finite semantics are explicit and tested;
  cancellation, status, single-use, and start-time construction behavior remain unchanged; the
  Pedro reference visibly loses config ceremony. No robot hardware validation is required for this
  pure API cleanup.
- **Decision record (2026-07-17):**
  - **Confirmed construction and timeout behavior:** `RouteTasks` exposes four public factories and
    `RouteTask` exposes two equivalent public eager constructors, for six public construction
    spellings. The built-at-start implementation is already internal. A nullable config silently
    creates a fresh ten-second default. The constructor retains a caller's mutable `Config`
    reference, and `update(...)` plus `debugDump(...)` reread its field, so mutation after
    construction can change an active Task's deadline and reported configuration. No caller uses
    that live-mutation behavior deliberately. The current `timeoutSec > 0.0` check treats zero,
    negative values, `NaN`, and negative infinity as no timeout while positive infinity is an
    effective no-timeout sentinel. The timeout is checked after execution status, the follower
    heartbeat, and a second status read, using strict `elapsed > timeoutSec`.
  - **Complete caller audit:** repository search found 59 executable route-Task constructions:
    50 through `RouteTasks` (three production/example calls and 47 test calls) and nine direct
    public constructors. Modern production/example code has
    only three: the named eager `BasicPedroAutoRoutine` route and Phoenix's named eager outbound
    plus named built-at-start return. Production code never uses an unnamed factory or direct
    constructor. The unnamed overloads exist only in nine core-test calls; direct constructors
    exist only in route/Pedro tests. Phoenix retains the concrete public `RouteTask<R>` result and
    reads `getRouteStatus()` in its routine coordinators, so the type must remain public even though
    its constructors need not be.
  - **Configuration storage and reuse audit:** executable code never stores a `RouteTask.Config` in
    an owner field, shares one between Tasks, composes it, or independently validates it.
    `BasicPedroAutoRoutine` creates, mutates, and immediately passes one local config.
    `PhoenixAutoTasks.routeConfig(...)` wraps the profile's already-owned scalar timeout in a fresh
    config for each of its two callers. Tests similarly create one-use values; only an illustrative
    Javadoc snippet reuses one config. The wrapper therefore answers no second question and provides
    no reusable value semantics. Merely adding `defaults()` or a defensive copy would repair config
    convention but retain the unnecessary student-facing noun.
  - **Alternatives and simplicity comparison:** keeping the current API or changing documentation
    alone preserves six spellings, nullable policy, and mutation risk. Removing constructors while
    retaining `Config`, or replacing it with an immutable options object, builder, `OptionalDouble`,
    or timeout-policy type still makes students construct a one-answer wrapper for immediate
    consumption. A hidden ten-second overload saves one argument but hides a robot safety decision;
    a zero, negative, or non-finite sentinel is short but ambiguous and silently accepts mistakes.
    `Tasks.withTimeout(...)` represents a caller-owned outer budget: it returns a generic `Task` and
    cancels the child through ordinary cancellation, so it cannot replace a retained route-local
    `RouteStatus.TASK_TIMEOUT`. Requiring a diagnostic name adds no production ceremony because
    every modern caller already supplies one, makes retained route results actionable, and removes
    unnamed/default construction paths.
  - **Chosen public API:** keep `RouteTasks` as the only public construction layer and keep
    `RouteTask<R>` public solely as the status-bearing result, with no public constructors or public
    config type. Expose exactly four parallel factories:
    `follow(debugName, follower, route, taskTimeoutSec)`,
    `followWithoutTaskTimeout(debugName, follower, route)`,
    `followBuiltAtStart(debugName, follower, routeFactory, taskTimeoutSec)`, and
    `followBuiltAtStartWithoutTaskTimeout(debugName, follower, routeFactory)`. The ordinary bounded
    route remains the shortest path. The longer exceptional method name is deliberate: it disables
    only `RouteTask`'s deadline and does not promise that a follower cannot report
    `FOLLOWER_TIMEOUT_OR_STALL`. Eager/start-time argument order and bounded/unbounded naming remain
    parallel inside the family.
  - **Validation and timing semantics:** require a non-null, nonblank diagnostic name and reject
    every timed value that is not finite and greater than zero at factory construction, before a
    route supplier or follower can run. The error must include the rejected value and point to the
    corresponding `WithoutTaskTimeout` factory. Internally retain final timeout-enabled state and a
    primitive value rather than a public sentinel or mutable alias. Start timing at the Task's own
    `start(clock)` boundary as today. At update, preserve the current evidence order—status,
    heartbeat, status, deadline—so confirmed follower completion or failure wins; if the execution
    is still active at `elapsed >= taskTimeoutSec`, retain `TASK_TIMEOUT`. The inclusive boundary
    matches the framework's other maximum-budget Tasks and avoids an unintended extra loop.
  - **Student/Bettabot effect:** the basic route loses the config declaration and mutation and
    becomes one call such as
    `RouteTasks.follow("BasicPracticeRoute", routes, practiceRoute, 4.0)`. Phoenix passes
    `profile.auto.routeTimeoutSec` directly and deletes `PhoenixAutoTasks.routeConfig(...)`.
    Because the basic routine branches only on the broad Task outcome, it also stores that result as
    `Task` and drops the `RouteTask` import; Phoenix correctly retains `RouteTask<PathChain>` where
    its coordinator reads precise status. A future Bettabot Pedro routine learns route identity,
    route source timing, and timeout policy, but no separate config type or null/sentinel rule. This
    is a modest route-composition simplification; it does not claim to remove Pedro geometry,
    lifecycle, or autonomous strategy work.
  - **Rejected and bounded scope:** do not retain deprecated constructors, aliases, unnamed
    overloads, or `Config`; do not add a builder, options value, hidden default, sentinel, generic
    timeout substitution, route DSL, or new autonomous policy. `RouteExecution`, `RouteStatus`,
    cancellation, single-use behavior, start-time supplier resolution, follower ownership, and
    exact per-start identity remain unchanged. `DriveGuidanceTask.Config` answers several guidance
    questions and is outside this one-item change; route naming is intentionally stricter because
    robot policy retains and branches on each route's precise terminal fact.
  - **Verification plan:** add a focused public-surface test proving the four factories, no public
    `RouteTask` constructor, and no `RouteTask.Config`. Cover blank names; zero, negative, `NaN`,
    and both infinities; validation before supplier/follower side effects; bounded and explicitly
    unbounded eager/start-time routes; the exact inclusive boundary; and follower completion at the
    boundary taking precedence. Migrate all 59 executable calls and delete
    `PhoenixAutoTasks.routeConfig(...)`. Re-run the existing route status, single-use, start-time,
    Pedro adapter, Phoenix routine, and basic example tests; run the full TeamCode unit suite and
    Java compile; then search for stale constructors/config/null sentinels and synchronize
    `RouteTask`/`RouteTasks` Javadocs, framework/Pedro guides, Phoenix Architecture, and the compiling
    Pedro reference. No hardware validation is required for this pure construction/validation
    change, but Android Studio review remains the manual approval point.
  - **Approval gate:** the factory-only/direct-timeout leading hypothesis remains viable. The exact
    four-method public surface, required names, explicit no-Task-timeout names, and inclusive
    deadline are major API decisions, so implementation must not begin until the user approves this
    ROUTE-03 design.
  - **Approval:** the user approved the ROUTE-03 design on 2026-07-17.
  - **Implementation (2026-07-17):** `RouteTasks` is now the sole public construction layer and
    exposes exactly the approved four named factories for eager/start-time and bounded/explicitly
    unbounded routes. `RouteTask` retains its public status-bearing type but has no public
    constructor or config type. Bounded factories validate a nonblank diagnostic name and a finite,
    positive Task timeout before route-factory or follower side effects; unbounded factories retain
    the stable `.timeoutSec` debug key with the value `none`. The deadline is inclusive while the
    existing follower-status evidence order, cancellation, single-use, and start-time resolution
    contracts remain intact.
  - **Robot-code effect:** `BasicPedroAutoRoutine` now supplies its timeout directly and stores the
    result as the broad `Task` it actually consumes, removing four lines plus the `RouteTask` concept
    from that beginner routine. Phoenix passes its profile-owned timeout directly and deletes
    `PhoenixAutoTasks.routeConfig(...)`, while correctly retaining `RouteTask<PathChain>` in the
    coordinator that reads precise route status. The complete independent Pedro reference is now
    626 source lines: 200 semantic path/routine/capability lines and 426 composition-root/FTC-host
    lines, compared with the recorded 630-line pre-ROUTE-03 baseline.
  - **Automated verification (2026-07-17):** the focused route-core suite passes 58 tests with zero
    failures, errors, or skips. `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` succeed; the final XML result contains 53 suites and 520
    tests with zero failures, errors, or skips. The only compiler diagnostics are the existing
    Java-21/source-8 warnings and existing FTC deprecation note.
  - **Static and documentation verification:** all 59 executable pre-change construction sites
    migrated. Searches find no external direct constructor, config, nullable-timeout, or removed
    factory use; the only `new RouteTask` calls are the two package-private internal construction
    paths. Exact reflection tests enforce the four public factories, no public constructor, and no
    public config type. `git diff --check`, trailing-whitespace checks across all 20 changed and
    untracked files, and all 52 local Markdown links in the seven changed Markdown files pass.
    Framework/Pedro guides, Phoenix Architecture, Javadocs, examples, and historical tracker records
    are synchronized.
  - **Independent review (2026-07-17):** separate core-lifecycle, public-API/simplicity, and
    documentation/principles reviews signed off on the settled tree. Earlier review findings drove
    preservation of the stable `.timeoutSec` debug key, corresponding invalid-timeout recovery
    guidance, exact bounded/unbounded debug assertions, current reference counts, and clearer
    historical tracker framing. The final reviewers report no remaining concrete finding.
  - **Android Studio audit point (2026-07-17):** ROUTE-03 is **Verifying** and intentionally remains
    unstaged and uncommitted. Inspect the four-method `RouteTasks` surface, the hidden `RouteTask`
    construction path, direct timeout use in the beginner and Phoenix routines, exact-boundary and
    invalid-input tests, and the synchronized guide examples. This is a pure API and Task-timing
    change; source inspection and unit tests are sufficient, and no robot-hardware result is
    claimed. Approval authorizes finalization/publication of ROUTE-03 only and does not start
    another item.
  - **Manual verification (2026-07-17):** the user reviewed the ROUTE-03 implementation in Android
    Studio and approved it with `ROUTE-03 looks good`. ROUTE-03 is now **Done**; this approval
    authorizes Gate 3 finalization, publication, and merge for ROUTE-03 only, not work on the next
    tracker item.

### FIELD-01 - Explicit alliance field transforms

- **Problem to confirm:** real autonomous routes can contain field-symmetric poses, but a
  mirrored route is only a nominal geometric baseline. Phoenix must eventually make shared geometry
  easy without implying that Red and Blue must use identical tuned endpoints, control points,
  constraints, or route topology.
- **External evidence:** Cuttlefish's
  [`FieldPose`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/FieldPose.java)
  derives a nominal alliance variant, but its competition Auto also uses alliance-specific
  endpoints, an extra Blue control point, different heading interpolation timing, and different
  mechanism offsets. This demonstrates the student-facing benefit and the required escape hatch:
  share only the geometry that is truly symmetric. Phoenix should still avoid Cuttlefish's global
  color state and use Phoenix's documented frame.
- **Alternatives to compare:** continue explicit red/blue path definitions; use Pedro's color helper
  directly; store alliance in global mutable context; add robot-local coordinate helper methods; or
  add a small immutable Phoenix-coordinate field transform driven by explicit field facts.
- **Reopen hypothesis:** after real Phoenix routes exist, first use an optional robot-local
  Phoenix-coordinate symmetry helper for the poses/control points that are actually shared. Keep
  ordinary named alliance-specific points or complete route builders for tested differences, then
  convert to Pedro at the integration boundary. Promote only repeated, stable transform math into
  the framework; do not add season route names, alliance strategy, or a generic route-tuning DSL.
- **Completion after reopening:** the real route set demonstrates which geometry is shared and which
  is deliberately alliance-specific. Focused tests cover the selected pose/heading symmetry,
  identity behavior, transform-before-Pedro conversion, and an explicit side-specific point or
  route. On-robot validation confirms both alliances independently; no test claims that geometric
  symmetry guarantees identical physical tracking.
- **Decision record (2026-07-13):**
  - **Confirmed behavior:** the original problem statement is not true of the current repository.
    `PhoenixPedroPathFactory` builds one documented twelve-inch Pedro integration placeholder. Its
    only alliance geometry branch keeps x/y unchanged and selects heading `0` for Red or `pi` for
    Blue; there are no duplicated Red/Blue route definitions or control-point sets to consolidate.
    `Phoenix Architecture.md` and the Phoenix Pedro README explicitly defer real geometry to the
    future. `Pose2d.then(...)` is an orientation-preserving rigid transform, so it cannot represent
    a reflection without a new mathematical concept. The runtime-selected `PedroFieldTransform`
    remains a vendor-coordinate boundary, not an alliance policy object.
  - **Current callers:** exhaustive modern-framework/Phoenix searches find the one placeholder
    heading branch above, alliance-specific scoring-tag selection in `PhoenixAutoProfiles`, thin
    Red/Blue OpModes that only choose `PhoenixAutoSpec.Alliance`, and selector/telemetry display.
    No production route, framework tool, or compiling example currently repeats alliance pose,
    heading, vector, or control-point transformation.
  - **Competition and vendor evidence:** at Cuttlefish's audited Worlds commit `1a9ff399`,
    `FieldPose.ColorPose(...)` nominally mirrors x and heading for Blue using its configured
    141.5-inch field width.
    Its [`Close` Auto](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Close.java)
    nevertheless uses distinct Red/Blue intake endpoints, adds a Blue-only curve control point,
    changes Red/Blue heading interpolation timing, and selects different shooter offsets; its
    [`Far` Auto](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Far.java)
    also applies a side-specific live control-point correction. Pinned Pedro 2.1.2 similarly
    provides only [`Pose.mirror(...)`](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.2/core/src/main/java/com/pedropathing/geometry/Pose.java)
    for one pose in Pedro coordinates, with a 141.5-inch default, rather than a route-level
    equivalence guarantee. The need for an explicit side-specific escape hatch is therefore
    demonstrated by the strongest external example. Its source does not establish whether each
    difference came from hardware, field interaction, camera/mechanism geometry, or strategy.
  - **Alternatives considered:** duplicate every route explicitly; expose Pedro's `Pose.mirror()`;
    use a global current-alliance value; add a private Phoenix helper now; add an immutable public
    `FieldTransform2d` plus point/vector types now; or defer extraction and use a hybrid when real
    geometry arrives. A future hybrid may transform shared canonical poses while ordinary robot
    code replaces a point, changes a constraint/control-point count, or selects a separate route.
  - **Simplicity comparison:** explicit separate routes add no concept and are easiest to tune when
    their topology differs, but unnecessarily duplicated shared geometry can drift. A public type
    today would add symmetry kind, axis/origin, canonical-side, point-versus-vector, heading-wrap,
    and adjustment-order contracts without removing one current repeated call. A private helper
    beside the first real route keeps the common call site short and lets the actual route reveal
    the smallest useful contract. A generic post-transform tuning/override layer would add more
    vocabulary than an ordinary named branch and is not justified.
  - **Chosen design and reason for deferral:** do not implement a public framework transform against
    placeholder geometry. Reopen FIELD-01 when a real route set proposes shared alliance geometry,
    not merely because EXAMPLE-02 contains one explicit route. Define canonical geometry only where
    field symmetry is real, apply an explicit
    robot-local transform selected from `PhoenixAutoSpec`, permit named per-alliance points or whole
    routes without ceremony, and convert Phoenix coordinates to Pedro last. Systematic drivetrain,
    localization, or follower error should first be corrected in calibration/tuning because hiding
    it in alliance field facts would conflate desired geometry with tracking compensation and tend
    to duplicate that compensation across routes. Promote a small backend-neutral transform into
    `fw` only after a second in-repository robot/backend repeats the same stable math or a concrete
    frame mistake shows that centralization prevents misuse. Any task that adds shared alliance
    geometry or an alliance transform must first move FIELD-01 from `Deferred` to `Researching` and
    rerun this decision gate; do not introduce transform logic silently under another item. A
    single-alliance or explicitly separate-alliance example route does not reopen this task.
  - **Rejected designs:** do not force one transformed route for both alliances; do not wrap or
    teach Pedro's coordinate-specific mirror as the framework authority; do not use global alliance
    state; do not add `Vec2`, arbitrary matrices, a transform-composition DSL, or a route-tuning DSL
    before callers require them. A completely separate alliance route remains a supported
    robot-owned choice rather than a design failure.
  - **Verification on reopening:** trace every real pose/control-point caller; add pure tests for
    the exact selected identity/symmetry math, finite inputs, heading normalization, boundaries,
    and applying the mapping twice when the selected symmetry is involutive; test
    Phoenix-transform-before-Pedro conversion; compile one shared route portion plus one explicit
    alliance-specific difference; run the full unit/compile/documentation/static checks; and tune
    and validate both alliance routes separately on the physical robot.
  - **Approval:** the user approved this deliberate deferral on 2026-07-13. No framework or Phoenix
    Java implementation was made; the recorded reopen gate prevents transform logic from being
    introduced silently with the first real route or EXAMPLE-02.

### DRIVE-01 - Drive task ownership

- **Problem to confirm:** timed drive tasks write only on start even though the sink command is
  current-loop state and the normal drive pipeline may overwrite it immediately.
- **Alternatives to compare:** reassert direct sink ownership every Auto loop; represent all drive
  tasks as sources; create a drive-output runner; or delete ambiguous helpers.
- **Leading hypothesis:** provide an explicitly exclusive Auto task that refreshes the sink, while
  TeleOp macros remain source/overlay proposals consumed by the single final writer.
- **Completion:** ownership and loop order are obvious at call sites; watchdog-style sinks receive a
  command each required loop; TeleOp retains one final drive writer.
- **Decision record (2026-07-11):**
  - **Confirmed behavior:** `DriveTasks.driveForSeconds(...)` builds a `RunForSecondsTask` whose
    start callback calls `sink.drive(signal)`, whose per-loop callback is `null`, and whose finish
    callback calls `sink.stop()`. The nonzero command is therefore written exactly once rather than
    refreshed during the timed interval. `DriveCommandSink` permits both immediate sinks and
    heartbeat-driven adapters that retain a request for a later owned update; every current concrete
    sink retains its last request in some form. The confirmed contract mismatch is therefore not a
    current watchdog failure: the helper never calls the `update(clock)` hook that the sink Javadoc
    says timed-drive Tasks call, and its mode-neutral name does not reveal that a direct sink Task
    must exclusively own behavior commands. The focused
    `TimedTaskStartSemanticsTest.publicDriveTaskCommandSurvivesItsSubLoopStartCycle` baseline passes
    one test with zero failures/errors/skips while explicitly asserting that the drive count stays
    at one; this confirms rather than protects against the defect.
  - **Loop and caller trace:** the documented TeleOp loop and compiling examples 03 and 06 run Tasks
    before one final `DriveSource -> DriveCommandSink` write. Phoenix TeleOp likewise has one final
    source-driven writer, although it does not currently expose a general macro runner. A direct
    Task write in either pattern is therefore overwritten later in the same cycle and its later
    `stop()` would be overwritten too. Phoenix Auto has no competing behavior-command phase: its
    active Auto Task owns drive behavior, while `PhoenixRobot` now supplies the stable external-sink
    lifecycle heartbeat before the Task runner. Existing `DriveGuidanceTask` and `RouteTask` model
    the same exclusive behavior-command path. `MecanumDrivebase` latches immediate motor power and
    has a no-op update; the Pedro adapter retains manual requests and deduplicates the composition
    root's update from any same-cycle Task-facing update.
  - **Current callers:** there is no production Phoenix, framework-tool example, Auto, or TeleOp
    caller of `DriveTasks`. The only executable caller is the baseline unit test. Student-facing
    references are concentrated in `DriveTasks`, `DriveCommandSink`, `Tasks`, Framework Principles,
    the Beginner's Guide, Framework Overview, Loop Structure, and Tasks & Macros Quickstart.
    `driveInstant(...)` has no caller outside its declaration, and `DriveTasks.stop(...)` appears
    only in one documentation sketch.
  - **Alternatives considered:** document the current latch behavior; silently refresh under the
    existing mode-neutral name; expose timed drive entirely as a `DriveSource`; add a drive-specific
    output Task and runner; delete all direct drive helpers; or keep one explicitly exclusive direct
    sink Task and delete one-shot helpers whose ownership/duration cannot be portable.
  - **Simplicity comparison:** a single exclusive helper remains one short line in an Auto
    `Tasks.sequence(...)`, composes with mechanism, guidance, and route Tasks, and adds no concept to
    the student path. A source-only timed behavior needs separate lifecycle/enable state before it
    can be sequenced. A drive-output runner adds another Task type, runner, active-source contract,
    and cross-runner coordination even though no current caller needs a timed TeleOp drive macro.
    Deleting every helper pushes a beginner's simple timed Auto move into a raw Task or blocking
    code. Keeping the old name or compatibility aliases leaves an attractive API that still looks
    safe inside TeleOp even though another writer wins.
  - **Chosen public API:** replace `driveForSeconds(...)` with the ownership-explicit
    `DriveTasks.driveExclusivelyForSeconds(sink, signal, durationSec)`. Do not retain a deprecated
    alias. Remove `driveInstant(...)`: one nonzero write either latches indefinitely or is
    overwritten immediately, so it has no portable Task meaning. Remove `DriveTasks.stop(...)` as
    well: the timed/guidance Tasks own their cleanup, robot owners call `DriveCommandSink.stop()`
    directly for shutdown, and an advanced sequence can deliberately use
    `Tasks.runOnce(sink::stop)` without presenting a mode-neutral drive macro to beginners. The
    resulting `DriveTasks` surface has one safe, discoverable purpose.
  - **Chosen ownership rule:** the exclusive helper is for an Auto-style runner or tester where the
    Task is the only behavior command writer for that sink while active. A composition root may and,
    for a stateful external adapter such as Pedro, must remain the stable lifecycle-heartbeat owner.
    Each unexpired active cycle asks `sink.update(clock)` once and, if that callback leaves the Task
    active, calls `sink.drive(signal)` once. A sink lane with an outer once-per-cycle heartbeat must
    make the same-cycle update idempotent; the Task-local call is not a substitute for that lifecycle
    owner. A positive
    duration publishes its command from `start(clock)` during the TaskRunner's start cycle, then
    deduplicates the runner's same-cycle `update(clock)`; normal completion and active cancellation
    stop exactly once. A zero duration publishes no motion and stops immediately. TeleOp macros and
    assists remain `DriveSource`/overlay proposals consumed by the composition root's one final
    writer; they must not use the exclusive helper before that writer.
  - **Chosen lifecycle implementation:** use a private purpose-specific Task behind the factory,
    not a new public type or builder. It anchors elapsed time at its own `clock.nowSec()` boundary,
    deduplicates by `clock.cycle()`, and becomes terminal before invoking `stop()`. It checks for
    reentrant cancellation after `sink.update(clock)` so cancellation cannot stop and then allow a
    stale nonzero write. This small internal state machine avoids `RunForSecondsTask`'s unavoidable
    update-then-stop write on the expiry cycle and its callback reentrancy gap while keeping robot
    code to one factory call. Reject non-finite or negative durations and null sink/signal inputs
    with actionable errors; do not add a public ownership policy type.
  - **External architecture re-review (2026-07-11):** BettaFish's Road Runner trajectory action,
    Pedro's official Follower/Ivy model, Astra Machina 16010's Worlds Pedro code, Golden Fish 13093's
    Worlds-finalist manual drive loop, and Cuttlefish's outer-loop Follower owner all recompute or
    reassert the active drivetrain output every loop under one drive owner. The successful
    command-based examples compose short semantic sequences/parallel groups; the manual FSM remains
    competitive but carries substantially more duplicated state and timing plumbing. Cuttlefish's
    PTO further reinforces the word **exclusive**: a timed drive Task must never overlap the owner
    that has reassigned those motors to endgame. This strengthens the exclusive per-loop command and
    source-driven TeleOp split. It does not justify a drive-output runner, Ivy dependency, generic
    Task resource scheduler, or making timed open-loop drive the normal Auto path; route Tasks
    remain the common autonomous movement primitive, and shared-actuator handoff stays in DRIVE-02.
    The expanded Worlds set leaves that judgment unchanged. Tech Tigers' `FollowerCommand` computes
    and writes drive output every scheduler tick, while SaMoTech and LOAD use Pedro routes as the
    normal Auto movement inside short semantic plans. SaMoTech's blocking drive helpers demonstrate
    the lifecycle hazards Phoenix Tasks should remove; LOAD's readable route/capability sequences
    demonstrate that doing so need not make the student routine longer. None of the three needs a
    general direct-drive runner or makes timed open-loop drive the normal autonomous API.
  - **Pedro prerequisite discovered by the re-review:** Pedro requires an outer-loop Follower
    heartbeat, immediate stopped-state behavior, a valid drivetrain/localizer, and explicit
    coordinate/pose authority. PEDRO-01 and PEDRO-02 were therefore completed ahead of DRIVE-01.
    The current Phoenix Auto root now owns the recurring adapter heartbeat, and the adapter makes
    Task/root calls same-cycle idempotent. DRIVE-01 must preserve that ownership and must not claim
    to implement or replace Pedro hold-end, pose updates, or stopped-output behavior.
  - **Rejected/deferred scope:** do not convert route followers or autonomous guidance into
    `DriveSource`s; their direct lifecycle ownership is already explicit. Do not add a generic or
    drive-specific output runner until a real recurring TeleOp timed-drive use case demonstrates
    that the extra abstraction is simpler than robot-owned source/overlay state. Do not add runtime
    resource arbitration for parallel Tasks in this item; exclusive ownership is a composition-root
    rule shared with existing guidance and route Tasks. Do not change `DriveSignal` range policy or
    unrelated stale documentation beyond text needed to make drive ownership and loop order true.
  - **Verification plan:** replace the bug-preserving assertion with focused tests for
    update-before-drive ordering, one refresh per active cycle, same-cycle idempotency, start inside
    a sequence, positive/zero duration, expiry, active/repeated cancellation, reentrant cancellation
    during sink update, drive/update failures, exactly-once stop, and suppressed cleanup failures
    through `TaskRunner`. Add an explicit competing-final-writer test or traced example showing why
    the exclusive helper is not a TeleOp proposal. Migrate every API/Javadoc/guide reference, verify
    exhaustive searches find no removed calls, then run `:TeamCode:testDebugUnitTest`,
    `:TeamCode:compileDebugJavaWithJavac`, XML result counts, `git diff --check`, adversarial review,
    and Android Studio inspection. Generic watchdog/recording-sink behavior belongs in DRIVE-01;
    Pedro hold, heartbeat, cancellation, and stopped-output behavior remain PEDRO regression scope,
    not functionality supplied by DRIVE-01.
  - **Pre-implementation revalidation (2026-07-13):** the complete caller, lifecycle, sink, Phoenix
    loop, Pedro, test, Javadoc, and guide audit confirms the leading API and ownership split remain
    the smallest principle-consistent design. There are still no production, Phoenix, tool,
    modern-example, or OpMode callers to migrate; only the bug-preserving unit test and current
    documentation use the old factories. Deleting every timed helper would make basic direct-drive
    Auto/test code harder, while a timed `DriveSource`, output runner, public ownership policy, or
    resource scheduler would add student-facing machinery without a caller. The private Task must
    publish `sink.update(clock)` then `sink.drive(signal)` from `start(clock)` for every positive
    duration, recording the cycle before callbacks so the runner's same-cycle update is a no-op.
    This is required when a sequence starts the drive child at the end of its own update; waiting
    until the next cycle could let a short command expire without ever being published. The Task
    then refreshes once per later unexpired cycle, checks expiry before another write, rechecks
    terminal state after a reentrant sink update, and becomes terminal before exactly-once stop
    cleanup. Documentation must describe **exclusive** as behavior-command ownership, retain the
    independent root heartbeat for stateful adapters, and keep this helper out of the ordinary
    TeleOp source/final-writer path. No DRIVE-01 Java implementation was made during this gate.
  - **Approval gate:** this confirms the tracker's leading ownership split, but it renames/removes
    public drive-task APIs and defines a new exclusive lifecycle contract. It is therefore a major
    API decision and requires explicit user approval before moving to **In progress**.
  - **Approval:** the user approved the DRIVE-01 design on 2026-07-13.
  - **Implementation (2026-07-13):** `DriveTasks` now exposes one public
    `driveExclusivelyForSeconds(...)` factory backed by a private single-use Task. Every positive
    interval publishes from its own start boundary, refreshes at most once per unexpired clock cycle,
    checks expiry before another write, and stops exactly once after becoming terminal. Active
    cancellation is idempotent; update-before-start and second-start misuse fail actionably; zero
    duration stops without publishing motion. The old `driveForSeconds(...)`, `driveInstant(...)`,
    and `DriveTasks.stop(...)` paths were removed rather than retained as aliases. TeleOp remains one
    source-driven final writer, while adapters whose supported lifecycle requires updates beyond
    active Tasks retain a shared-clock composition-root heartbeat with same-cycle deduplication.
  - **Automated verification (2026-07-13):** `DriveTasksTest` has 19 passing tests for start/update
    order, refresh and deduplication, sequence timing, exact/overshoot expiry, positive/zero duration,
    lifecycle misuse, cancellation/reentrancy, callback failures, exactly-once stop, runner cleanup,
    suppressed cleanup failures, and the competing-writer trace. The retained
    `TimedTaskStartSemanticsTest` has 14 passing tests. The full
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` run succeeded: 28 suites,
    258 tests, zero failures, errors, or skips. Only the repository's existing Java 8-on-JDK-21
    deprecation warnings remain.
  - **Static and independent verification (2026-07-13):** exhaustive current source, test, guide,
    example, tool, and OpMode searches find no removed API call; the remaining old names are only the
    historical decision evidence in this tracker. The public `DriveTasks` surface contains only the
    approved factory. `git diff --check`, changed-file trailing-whitespace checks, and changed-
    Markdown local-link checks pass. Three independent adversarial reviews covered lifecycle/Pedro
    compatibility, Framework Principles/student-facing documentation, and API/test/caller scope.
    Their wording and reentrant-update test findings were resolved, and the final review has no
    blocking finding.
  - **Manual verification (2026-07-13):** the user confirmed the Android Studio review and approved
    DRIVE-01. No robot-hardware run is claimed; a later low-power direct-drive Auto/test check can
    confirm command refresh and immediate stop on the team's real sink, while normal Pedro route
    movement remains covered by its route and heartbeat lifecycle rather than this helper.

### TASK-04 - Parallel deadline composition

- **Problem to confirm:** Phoenix has `sequence(...)` and `parallelAll(...)`, but a realistic Auto
  often needs one route to determine phase completion while an intake, aiming preparation, or other
  companion behavior runs alongside it and is cancelled when the route ends. `parallelAll(...)`
  cannot express that ownership: a longer companion prevents phase completion. BettaFish's
  long-lived shooter/context updates and latched intake command are instead evidence for held
  capability or service state, not forever Tasks. Cuttlefish provides the concrete reusable caller:
  a bounded mechanism action whose lifetime is owned by one path segment.
- **Expanded Worlds evidence:** Tech Tigers uses a `ParallelRaceGroup` where transfer completion
  ends a phase and cancels perpetual shooter preparation; LOAD uses races mainly to express route
  timeouts, stall exits, and bounded mechanism work. Phoenix should normally express the perpetual
  shooter preparation as robot-owned state/service behavior, and should express route timeout and
  stall status at the route boundary. Those callers therefore do not justify a symmetric
  arbitrary-race API.
- **Alternatives to compare:** require every companion to be finite; set/clear capability state
  before and after the route; use Pedro parametric callbacks; create robot-specific phase macros;
  add one `Tasks.parallelDeadline(deadline, companions...)` primitive; or copy Ivy's full race,
  deadline, repeat, loop, requirements, and priority model.
- **Leading hypothesis:** persistent baseline mechanisms should remain source/capability state, not
  forever Tasks. If real Phoenix route-plus-finite-companion callers remain after that simplification,
  add one small deadline composition whose deadline child controls completion and whose still-active
  companions receive normal active cancellation. Add symmetric race/repeat/requirements only after
  separate caller evidence; do not import Ivy or create a second scheduler.
- **Completion:** the common Auto phase is one readable factory call; start, same-cycle completion,
  deadline success/timeout/cancellation, companion early completion, one best-effort cancel attempt
  per selected direct child, throwing/reentrant cancellation, duplicate identities, outcomes, and
  single-use behavior are specified and tested. Documentation distinguishes held capability state
  from bounded companion Tasks.
- **Decision record (2026-07-13):**
  - **Confirmed behavior and minimal trace:** `ParallelAllTask` starts every child, updates every
    incomplete child, and becomes complete only after `allFinished()` observes every child complete.
    Therefore `Tasks.parallelAll(route, companion)` correctly remains active after `route` completes
    whenever `companion` is still active; a containing sequence cannot advance and no ownership edge
    cancels the companion. This is not an `all` bug, and changing `parallelAll(...)` would silently
    break its existing meaning.
  - **Current callers:** exhaustive framework, Phoenix, tool, test, and documentation searches find
    no production Phoenix caller of `Tasks.parallelAll(...)`. The tool examples use `ParallelAllTask`
    for two finite actions and genuinely require all-of completion. The current Phoenix Pedro Auto
    routines are sequence-only placeholders. Phoenix's existing Pedro progress callback that enables
    the flywheel is correctly persistent scoring state, not a deadline companion, so there is no
    in-repository production call to migrate or force-fit.
  - **Competition caller classification:** all three
    [BettaFish route-plus-transfer phases](https://github.com/7641MSETBettafish/BettafishDecode/blob/1fb81edcd777b42652ead60302e7bd0f6ea8cedf/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/Autoblue.java)
    use an outer all-of `ParallelAction`; the third additionally bounds `transfer.run()` with a
    4.5-second `RaceAction`. Its forever shooter/context updates map to services, while its one-shot
    intake command maps to persistent held capability/source state. Tech Tigers' perpetual shooter
    preparation likewise maps to state/service ownership, and LOAD's principal races map to truthful
    route termination, bounded waits, or the separate whole-routine takeover item. The narrow
    positive evidence is Cuttlefish: its fidelity port's
    [`Close`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/f9005c2e1f5f7e523d415f69a670b7e2f737f4f1/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Close.java)
    queues `flickExtakeThenOn(100)` twice with `actionDuring(...)`; each runs with the next real path
    segment. Its
    [`PathActionScheduler`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/f9005c2e1f5f7e523d415f69a670b7e2f737f4f1/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/scheduler/PathActionScheduler.java)
    cancels every active during-action when the segment advances. That repeated route-scoped
    cancellation edge is a real deadline ownership pattern, but the Cuttlefish action does not itself
    guarantee safe actuator restoration when cancelled. Its 100 ms action will normally finish before
    a multi-second path, so the edge matters chiefly on early timeout, skip, or failure; this is
    deliberately narrow evidence. Phoenix can express the ownership without copying Cuttlefish's
    second scheduler, while requiring its companion Task to own safe cancellation cleanup; planned
    EXAMPLE-02 needs that cleanup-safe route-plus-intake shape.
  - **Alternatives considered:** make all companions finite and use `parallelAll(...)`; place
    capability set/clear calls before and after a route; use Pedro progress callbacks; hand-write a
    robot-specific phase Task; add one deadline composition; or import a full race/deadline/repeat/
    requirements scheduler. Requiring a guessed companion duration changes when the phase completes.
    A trailing clear action still runs after an ordinary terminal route outcome, but is skipped if
    the enclosing sequence is actively cancelled or a lifecycle exception fail-stops its runner.
    Pedro callbacks are useful progress events but couple ownership to one vendor and do not provide
    general cancellation cleanup. A custom phase Task repeats subtle lifecycle code in robot code.
  - **Simplicity comparison:** the selected common call is one line beside the existing all-of form:
    `Tasks.parallelDeadline(followRoute, runIntake)`. Students learn one distinction: `parallelAll`
    waits for every child, while `parallelDeadline` ends with its first, named deadline child. The
    word `deadline` describes ownership rather than elapsed seconds; the parameter Javadoc will make
    that explicit. The implementation type remains package-private, with no public constructor,
    builder, list/supplier overload, resource declaration, priority, or scheduler. Held flywheel,
    intake, aiming, and similar baseline requests remain capability/service state instead of Tasks.
  - **Chosen public API:** add only
    `Tasks.parallelDeadline(Task deadline, Task... companions)`. The deadline controls group
    completion and the group's natural outcome; companions may finish early without completing the
    group. Zero companions are valid and behave like a structured wrapper around the deadline, which
    keeps dynamically assembled varargs unsurprising. Reject a null deadline, null companion array,
    null companion element, and direct identity reuse across the deadline and companion positions
    with role/index-specific guidance to build fresh single-use Tasks. Defensively copy the varargs
    array so later caller mutation cannot alter the graph. Do not add a `primary` alias or expose
    `ParallelDeadlineTask` publicly.
  - **Chosen lifecycle:** the package-private Task is single-use, rejects update-before-start, and
    keeps pre-start and terminal cancellation side-effect-free. Consume the wrapper's one start
    attempt before any child callback, and record each direct child's start attempt before invoking
    that callback so partial-start failure remains cleanable. Start the deadline once; if it reports
    complete immediately after that required `start(clock)` call, validate and capture its outcome
    and do not start companions. Otherwise start distinct companions in declaration order.
  - **Update and termination order:** on each update, check deadline completion first; if active,
    update it once, then check again. This prevents an extra update when an outer heartbeat completed
    it between cycles. If the deadline completes, validate and capture its outcome before making the
    group terminal, then best-effort cancel companions before any companion receives another update
    that cycle. Apply the same check-before-update rule to companions, skipping those already
    complete. Re-check terminal/deadline state after every callback so reentrant completion or
    cancellation stops further starts or updates.
  - **Cleanup contract:** active direct cancellation makes the group terminal first, then makes one
    best-effort `cancel()` call on every start-attempted direct child, including the deadline. Natural
    deadline completion makes the group terminal first, then makes that call on every start-attempted
    companion. Cleanup never calls `isComplete()`, because that may be the hook that failed; the Task
    contract makes terminal child cancellation a no-op, so only active children perform cleanup.
    Try every selected direct child even when one throws, then rethrow the first runtime failure with
    later failures suppressed. Hidden aliases may route more than one cancel call to a shared leaf,
    so leaf idempotence remains required and global exactly-once cleanup is not claimed.
  - **Outcome contract:** while active, the group reports `NOT_DONE`; direct cancellation reports
    `CANCELLED`. Natural completion exposes the deadline's `SUCCESS`, `TIMEOUT`, `CANCELLED`, or
    `UNKNOWN` result and does not let companion outcomes overwrite the owner result. A completed
    deadline that returns null or `NOT_DONE` is a malformed custom Task and causes an actionable
    `IllegalStateException`; a throwing outcome query also propagates. Outcome capture occurs before
    terminalizing so `TaskRunner` fail-stop can still cancel the deadline and companions if capture
    fails. Companion cleanup failure is thrown after all cleanup attempts, while the graph remains
    terminal for runner fail-stop handling.
  - **Companion responsibility:** the composition can only invoke each Task's `cancel()` hook; it
    cannot infer mechanism cleanup. Every route-scoped companion must therefore already be
    cancellation-safe. In particular, `sequence(enable, wait, disable)` is unsafe as a companion
    because active sequence cancellation skips the later disable child. Persistent requests should
    remain capability/service state; a genuinely bounded route-scoped behavior should be exposed by
    a robot Task/macro whose own active cancellation restores its safe or caller-selected state.
  - **Rejected/deferred scope:** do not change `parallelAll(...)`; do not add symmetric `race(...)`,
    repeat/forever Tasks, requirements, priorities, implicit actuator arbitration, or another runner.
    Do not turn persistent capability requests into cancellation-owned Tasks. Do not add route
    library types or route timeout policy to this generic helper. Do not add recursive Task-graph
    introspection solely to detect a shared leaf hidden inside different composites: the leaf's
    single-use guard rejects its second start, and runner fail-stop cleans the attempted graph.
    Reconsider any broader primitive only with a separate repeated caller and decision gate.
  - **Verification plan:** add focused pure tests for validation and zero companions; start/update
    order; immediate and same-cycle deadline completion; companion early completion; deadline
    success/timeout/cancellation/unknown outcomes plus null/`NOT_DONE`/throwing outcome fail-stop;
    completion between cycles without a deadline update; pre-start, active, repeated, and terminal
    cancel; partial-start and lifecycle failures; one best-effort cancel attempt per selected direct
    child with suppressed failures; reentrant cancellation/completion; direct duplicate identities
    and hidden nested aliases; caller mutation of the original companion array; update-before-start;
    second-start rejection including after a failed first start; and stable deadline/companion debug
    output. Update `Tasks`/internal Javadocs, the Task guide, framework overview/principles where the
    composition vocabulary is listed, and the Phoenix Auto guide without fabricating a production
    caller. Then run `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac`, inspect XML test totals, run changed-file/static/Markdown
    checks and exhaustive caller searches, and request Android Studio review. No robot-hardware
    claim is required for this pure composition primitive.
  - **Approval gate:** this adds a public composition API and defines cancellation/outcome ordering,
    so it is a major API decision. Stop at **Ready** for explicit user approval before changing Java,
    tests, or framework documentation.
  - **Approval:** the user approved the TASK-04 design on 2026-07-14.
  - **Implementation (2026-07-14):** `Tasks.parallelDeadline(deadline, companions...)` is now the
    only public addition, backed by one package-private `ParallelDeadlineTask`. It starts and updates
    the deadline first, skips companion starts after immediate deadline completion, lets companions
    finish early without ending the group, captures the deadline's validated terminal outcome, and
    terminalizes before deterministic best-effort cleanup. Direct cancellation includes every
    start-attempted child; natural completion includes only start-attempted companions. Both paths
    retain the first cleanup failure and suppress later ones without querying completion during
    cleanup. Direct aliases are rejected before effects, the varargs array is copied, partial starts
    remain cleanable, and reentrant callbacks cannot advance later children after termination.
  - **Documentation and caller disposition:** public/internal Javadocs, Framework Principles, the
    Beginner and Framework Overview guides, Tasks & Macros Quickstart, Recommended Robot Design,
    `RouteTask`, Phoenix Architecture, and the Phoenix Pedro Auto guide now distinguish all-of from
    deadline-owned composition and persistent capability/service state from bounded companions.
    They make companion-owned cancellation cleanup explicit and do not fabricate a production
    caller; the current Phoenix routes remain sequence-only placeholders.
  - **Automated verification (2026-07-14):** the focused `ParallelDeadlineTaskTest` passes 14 tests
    covering validation/copying, zero companions and all valid outcomes, deadline-first order,
    immediate/between-cycle/same-cycle completion, early companions, reentrancy, direct and natural
    cleanup failure suppression, partial starts, lifecycle and outcome fail-stop, single-use,
    hidden aliases, and stable debug prefixes. The full
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` run succeeds; XML reports 272
    tests across 29 suites with zero failures, errors, or skips. Only the existing JDK 21/source-8
    and SDK deprecation warnings remain.
  - **Static and independent verification (2026-07-14):** caller/type searches confirm that only
    `Tasks` constructs the package-private implementation, there is no Phoenix production caller to
    migrate, and no route/vendor type entered the generic API. `git diff --check`, all changed and
    untracked file trailing-whitespace checks, changed-Markdown local-link checks, forbidden-import/
    blocking-call searches, and visibility checks pass. Three independent reviews covered lifecycle
    and TaskRunner failure behavior, API/test/caller simplicity, and documentation consistency. The
    documentation findings were resolved; lifecycle re-review and an independent focused test run
    found no defect. One static-review concern expected an integer debug count, but Java selects
    `DebugSink.addData(String, double)` for that primitive value, and the passing focused XML confirms
    the documented/tested double value.
  - **Initial manual-verification gate:** superseded when the API-family review reopened TASK-04 for
    the factory-only design recorded below. Final user approval is recorded at the end of this item.
  - **Second API-family review (2026-07-14):** after the user questioned whether matching
    `of(...)` factories would improve the API or merely duplicate it, the construction paths and
    their callers were audited from scratch. Modern Phoenix production code and lifecycle tests use
    the `Tasks` facade. Direct concrete `of(...)` use is limited to six calls in two disabled teaching
    tools, plus class Javadocs and the facade's own delegation. The public List constructors have no
    external in-repository caller; `SequenceTask.fromSuppliers(...)` is used only by two tests and
    eagerly invokes its suppliers, so it does not provide deferred or repeatable behavior; and
    `SequenceTask.getCurrentTask()` has no external caller and exposes a mutable implementation
    detail that `debugDump(...)` already makes observable for humans.
  - **Reconsidered alternatives:** concrete factories only would replace the established student
    namespace with three imports and would remain inconsistent with factory-only composites such as
    `Tasks.branchOnOutcome(...)`. Keeping both layers preserves compatibility, but offers two
    spellings for identical behavior and no demonstrated advanced capability. Making composition
    factory-only through `Tasks` provides one discoverable namespace and makes all supported public
    siblings parallel at the one public construction layer.
  - **Revised chosen design:** expose composition only as `Tasks.sequence(...)`,
    `Tasks.parallelAll(...)`, and `Tasks.parallelDeadline(...)`. Make `SequenceTask`,
    `ParallelAllTask`, and `ParallelDeadlineTask` package-private implementation types; remove the
    public concrete constructors and `of(...)` factories; remove the unused eager
    `SequenceTask.fromSuppliers(...)` convenience and public current-child accessor; and migrate the
    two teaching tools and all documentation to the facade. Preserve the complete lifecycle,
    cancellation, outcome, validation, and debug contracts in `Tasks` Javadocs. Do not add a List
    overload until a real caller demonstrates that it improves robot code.
  - **Simplicity and compatibility judgment:** this is the shortest common robot-code path, reduces
    autocomplete choices, and reconciles the principles of one obvious way and parallel sibling
    APIs without inventing an "advanced" layer that exposes no useful advanced behavior. It is a
    source- and binary-breaking change for unknown external callers of the concrete types. The
    in-repository migration is small and mechanical, FTC projects rebuild from source, and Framework
    Principles explicitly permits coherent breaking cleanup; copied external code would need to
    replace concrete construction with the corresponding `Tasks.*` call.
  - **Principle and workflow correction:** retain the existing parallel-API principle, but clarify
    that an API family should choose one supported public construction layer and keep siblings
    parallel within it; a second public layer is justified only by a distinct capability. Update the
    framework-improvement skill's decision and adversarial-review gates to inventory every sibling
    construction path and record whether each path provides distinct value. This targeted checklist
    is warranted because the existing general principles did not prevent the duplicate-layer
    recommendation; the skill's trigger and UI metadata do not change.
  - **Verification delta:** migrate and statically audit all concrete composition callers; move the
    full public contracts to `Tasks`; keep package-private white-box lifecycle tests where useful;
    validate the edited skill; and rerun the focused and full TeamCode checks plus documentation and
    public-API searches.
  - **Reopened approval gate:** hiding existing public composition types materially changes the
    approved API and expands TASK-04's migration scope. TASK-04 remains **Ready** and no Java,
    principle, guide, or skill implementation edit should occur until the user explicitly approves
    this factory-only composition design.
  - **Factory-only API approval:** the user approved the revised composition design on 2026-07-14.
    TASK-04 was moved to **In progress**. Implementation is limited to the three composition factories,
    their internal implementation types, directly affected callers/tests/documentation, the
    principle clarification, and the corresponding improvement-skill audit rule.
  - **Factory-only implementation (2026-07-14):** `Tasks.sequence(...)`,
    `Tasks.parallelAll(...)`, and `Tasks.parallelDeadline(...)` are now the sole public construction
    layer for generic sequence/all/deadline composition and all return `Task`. Their concrete
    implementations are package-private, with package-private constructors only. The redundant
    public constructors, `of(...)` factories, eager `fromSuppliers(...)` convenience, and mutable
    current-child accessor were removed. In-repository examples, tests, Javadocs, and guides now use
    only `Tasks.*`; the three internal types identify themselves to `TaskRunner` by those public
    factory names, so telemetry and duplicate-enqueue errors do not teach inaccessible class names.
  - **Principle and workflow implementation:** Framework Principles now requires one supported
    public construction layer per API family and parallel sibling operations within that layer; a
    second layer or overload must provide distinct demonstrated value rather than another spelling.
    The framework-improvement skill now inventories facade factories, concrete constructors,
    `of(...)` methods, overloads, and return types, then requires removal/migration or a bounded,
    explicitly justified compatibility disposition for every redundant layer. Its adversarial gate
    repeats that audit and forbids symmetry-only expansion. The official skill validator passes, and
    a forward test correctly rejected retaining the redundant concrete layer without that explicit
    disposition.
  - **Final automated verification (2026-07-14):** the focused deadline, cancellation, and
    single-use contract suites pass after adding coverage for ordinary update order, deadline-start
    failure cleanup, completion during companion start, fresh supplier-built factory graphs, public
    factory debug identities, and duplicate-enqueue diagnostics. The full
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` run succeeds; XML reports 277
    tests across 29 suites with zero failures, errors, or skips. Only the existing JDK 21/source-8
    and SDK deprecation warnings remain.
  - **Final static and adversarial verification (2026-07-14):** bytecode inspection confirms the
    three concrete composition classes are not public and that `Tasks` exposes exactly the three
    intended `Task`-returning sequence/all/deadline factories. Exhaustive searches find no stale
    concrete imports or removed factories; only `Tasks` constructs the internal classes.
    `git diff --check`, changed/untracked-file whitespace checks, changed-Markdown local-link checks,
    and skill validation pass. Independent lifecycle, API/caller, documentation, and workflow
    reviews found and then confirmed fixes for three test-coverage gaps, overly broad overload
    wording, an omitted composition helper in two guides, public diagnostic naming, and fresh-graph
    regression coverage; their final re-reviews report no remaining concrete defect.
  - **Factory-only manual verification (2026-07-14):** the user reviewed the implementation in
    Android Studio and approved it with `TASK-04 factory-only API looks good`. TASK-04 is **Done**.
    No robot-hardware claim is needed for this pure generic composition/API change; future
    production companion Tasks still need mechanism-specific cancellation verification.

### PHX-03 - Explicit Auto route-failure policy

- **Problem to confirm:** `SequenceTask` deliberately advances after a child timeout and only reports
  the aggregate timeout when the whole sequence ends. Phoenix's current routines directly sequence
  outbound route -> aim/shoot -> return, so an outbound route timeout can still trigger scoring from
  the wrong location. Pedro's completion guidance also recommends explicit time/failsafe policy for
  paths that cannot satisfy endpoint constraints. Cuttlefish's bonk handling intentionally breaks a
  path and continues to a different phase, further showing that the robot must state what each
  non-normal route ending means. ROUTE-02 must land first so that policy receives truthful status.
- **Alternatives to compare:** accept aggregate sequence behavior; change every generic sequence to
  short-circuit; wrap each route with existing `Tasks.branchOnOutcome(...)`; add a generic
  success-only sequence; or add a small robot-owned route helper that chooses continue, fallback
  park, or abort for each strategy.
- **Leading hypothesis:** preserve generic `Tasks.sequence(...)` because timeout is not universally
  fatal. Make route outcome policy visible in `PhoenixPedroAutoRoutineFactory`, using existing
  outcome branching first and extracting a tiny Phoenix helper only if several routines repeat the
  same shape. Fallback selection remains robot strategy, not framework route policy.
- **Completion:** every route timeout/cancellation has an explicit continue/fallback/abort result;
  no routine silently aims or shoots after a failed prerequisite; the fallback is visible in task
  telemetry; tests cover outbound failure, scoring failure, return failure, cancellation, and a
  successful routine without changing generic sequence semantics.
- **Decision record (2026-07-14):**
  - **Confirmed failure path:** all four strategies built by
    `PhoenixPedroAutoRoutineFactory` currently have the same unconditional
    `outbound route -> aimAndShootOne -> return/park route -> disable flywheel` sequence.
    `RouteTask` truthfully maps `COMPLETED` to `SUCCESS`, the two timeout statuses to `TIMEOUT`,
    and interruption/replacement/cancellation/failure/unknown terminal statuses to `CANCELLED`.
    `SequenceTask` deliberately advances after every completed child, aggregates only child
    `TIMEOUT`, and can therefore erase a child `CANCELLED` into final `SUCCESS`. A non-throwing
    abnormal outbound result consequently starts position-dependent aiming and may request a shot
    from the wrong location. This is distinct from direct cancellation of the whole routine, which
    does stop later children, and from a thrown lifecycle/integration failure, which uses the
    `TaskRunner` fail-stop path.
  - **Persistent-state and return-path evidence:** the fixed outbound Pedro path has a 50-percent
    progress callback that captures the suggested velocity and enables the held flywheel request.
    A later route timeout or interruption can therefore leave scoring intent active. The return
    route has the same policy gap: every terminal status advances to the trailing flywheel-disable
    Task, cancellation-like statuses may be reported as success, and a thrown start-time path-build
    failure skips that trailing Task until robot shutdown. Safe policy must cancel pending shots and
    clear held scoring requests through `PhoenixCapabilities`, not write hardware directly.
  - **Production and modern-caller inventory:** `PhoenixPedroAutoOpModeBase` is the only production
    caller of `PhoenixPedroAutoRoutineFactory.build(...)`. The two static safe Autos select
    `SAFE_PRELOAD`; the Pedro test OpMode selects `PEDRO_INTEGRATION_TEST`; and the selector exposes
    all four strategy ids, including `PRELOAD_AND_PARK` and `PARTNER_AWARE_CYCLE`. All four reach the
    same repeated routine shape. No production Phoenix class currently reads a per-start
    `RouteTask.getRouteStatus()` for policy; the OpMode only displays the adapter's mutable
    latest-route status. Existing route/Pedro tests prove the lower-level statuses and lifecycle,
    while `PhoenixPedroPathFactoryTest` covers only return geometry. There is no current Phoenix
    routine-policy or `PhoenixAutoTasks` unit test.
  - **Public construction-path audit:** generic sequential composition has one supported public path,
    `Tasks.sequence(Task...) -> Task`; `SequenceTask` is package-private and has no public constructor
    or `of(...)` alias. Outcome branching likewise has one public path,
    `Tasks.branchOnOutcome(Task, Task, Task) -> Task`. `RouteTasks` has a justified two-by-two family:
    eager versus explicitly start-time-built geometry, each with default versus diagnostic-name
    overloads, and all four return the status-bearing `RouteTask<R>`. Phoenix has one public routine
    factory, `PhoenixPedroAutoRoutineFactory.build(ctx) -> Task`; its strategy builders remain
    private. `PhoenixCapabilities(Scoring, Targeting)` is the capability aggregate's sole public
    construction path, while `PhoenixRobot.capabilities()` acquires the robot-owned aggregate rather
    than constructing an alternative. `Targeting.aimTask(...) -> Task` is the capability leaf
    factory. `PhoenixAutoTasks.aimAndShootOne(...)` and `disableFlywheel(...)` are distinct semantic
    macro factories, while `routeConfig(...)` and `aimConfig(...)` were config adapters at this
    gate; the utility constructor is private and there are no constructor/`of(...)` aliases.
    PHX-03 adds no parallel public macro, constructor, builder, overload, or route API: both new
    implementations stay behind the existing public routine/macro factories. ROUTE-03 later removed
    the one-field `routeConfig(...)` adapter while leaving `aimConfig(...)` unchanged.
  - **Adjacent API disposition:** the audit also found that `RouteTask` still has two public eager
    constructors which duplicate `RouteTasks.follow(...)`, while start-time construction is
    deliberately factory-only, and that `RouteTask.Config` still uses implicit construction/null
    defaults rather than the usual explicit owner-config vocabulary. Neither issue is required to
    fix robot route policy. Removing or normalizing those public paths here would be an unrelated
    breaking framework migration, contrary to the one-item scope, when PHX-03 needs no such change.
    Do not add supplier constructors merely for symmetry. The existing `CLEAN-01` gate now explicitly
    records both findings and must perform its own caller/compatibility decision before migrating or
    removing the constructors or normalizing the config construction/default path.
    ROUTE-03 later completed that audit and removed the duplicated constructors, config, nullable
    defaults, and unnamed route factories.
  - **Alternatives considered:** leave the sequence and improve only documentation; globally make
    `sequence(...)` short-circuit; add a second generic success-only sequence; nest only the existing
    `branchOnOutcome(...)`; add cancelled/status overloads to generic `Tasks`; add a route-status DSL
    or public Phoenix Auto builder; or centralize the repeated lifecycle in one private robot-owned
    coordinator. Documentation alone leaves the unsafe behavior. Changing or duplicating generic
    sequence semantics makes timeout fatal where recovery may be correct. Existing outcome branching
    has no branch for a child `CANCELLED`, collapses precise route reasons, cannot distinguish direct
    wrapper cancellation from an observed abnormal route, and does not retain useful fallback debug
    state. A generic cancelled/status API still cannot own Phoenix scoring cleanup, while coupling
    `Tasks` to route or season policy violates the framework boundary. A public Auto DSL adds concepts
    before Phoenix has real varied routines that justify one.
  - **Simplicity comparison:** the selected design adds no concept to the public student API and keeps
    each strategy as one named semantic routine factory. Students continue using `RouteTasks`,
    `PhoenixAutoTasks`, and `PhoenixCapabilities`; the subtle status, cancellation, cleanup, and
    telemetry state machine is implemented and tested once. The private routine code will explicitly
    name that scoring runs only after a completed outbound route and that a timeout chooses the
    current-pose return/park fallback. This is shorter and harder to misuse than deeply nested
    outcome branches, while preserving the generic framework's one factory-only composition layer.
  - **Chosen ownership and implementation shape:** keep `Tasks.sequence(...)`, `TaskOutcome`,
    `RouteTask`, and `RouteTasks` semantics unchanged. Use two cohesive package-private state
    machines behind existing factories, rather than one monolith or hidden nested branches: a
    Phoenix scoring-attempt Task behind `PhoenixAutoTasks.aimAndShootOne(...)`, and one Phoenix/Pedro
    routine coordinator for the repeated outbound, scoring, and return/fallback phases. The latter is
    exposed only through private helpers in `PhoenixPedroAutoRoutineFactory`. The private
    `followOutbound(...)` and `followReturn(...)` helpers will return `RouteTask<PathChain>` rather
    than erasing the typed per-start status to `Task`. Both helpers depend on capabilities and
    backend-neutral Task/route types, never on a raw Pedro `Follower`, and remain single-use,
    non-blocking, and on the shared `LoopClock`. This is two internal lifecycle owners with distinct
    jobs, not a second public construction layer or an Auto DSL.
  - **Chosen current strategy policy:** all four checked-in placeholder strategies use the same
    conservative mapping, stated in the routine factory rather than the integration:
    - outbound `COMPLETED`: run the scoring attempt, then build the return/park route from the live
      pose;
    - outbound `FOLLOWER_TIMEOUT_OR_STALL` or `TASK_TIMEOUT`: make scoring safe, skip aiming/shooting,
      and attempt the live-pose return/park route as the fallback;
    - outbound `INTERRUPTED`, `REPLACED`, `CANCELLED`, `FAILED`, or `UNKNOWN_TERMINAL`: make scoring
      safe and abort without starting another route;
    - scoring success: disable the route-owned flywheel request after the requested shot drains, then
      return/park; scoring timeout: let the scoring attempt cancel its own pending shot, disable the
      route-owned flywheel request, retain the degraded outcome, and return/park; scoring
      cancellation/unknown: clean only those same owned requests and abort;
    - return/fallback completion: finish safely; return timeout/stall: finish with `TIMEOUT`; every
      other abnormal return status: finish with `CANCELLED` and no replacement route.
    A future strategy that intentionally interrupts a route, such as a bonk transition, must state a
    different robot-owned mapping explicitly; it must not change the integration's status meaning.
  - **Scoring-attempt truth:** update the existing `PhoenixAutoTasks.aimAndShootOne(...)` behavior,
    without adding another public construction path. Its package-private scoring-attempt Task owns
    the wait-for-target, capture-velocity, aim, request-shot, and wait-for-shot phases directly, so
    unavailable target, aim timeout, and shot-drain timeout remain `TIMEOUT` rather than being
    converted to success by a no-op handler; child cancellation/unknown outcomes likewise remain
    visible instead of being erased by an inner sequence. Each failure skips every dependent shot
    action that has not safely begun, and timeout/cancellation after a shot request best-effort
    cancels that Task's own transient shot. This lets the routine make and test an honest scoring-
    failure decision while keeping the same short public call.
  - **Lifecycle, outcome, and cleanup contract:** consume the coordinator's one start attempt before
    any child or capability effect, reject update-before-start and direct child aliases, and start at
    most one phase child at a time. Snapshot the exact terminal status from each retained RouteTask;
    never decide from `getLatestRouteStatus()`. Direct active cancellation terminalizes first,
    cancels only the active child, disables the route-owned flywheel request once, and never launches
    fallback; an active scoring child performs its own transient-shot cleanup. Lifecycle
    exceptions continue to propagate to `TaskRunner` fail-stop; its best-effort wrapper cancellation
    still invokes each owner's idempotent cleanup. A successful fallback does not erase the triggering
    timeout; a later cancellation-like failure takes precedence. Cleanup is deliberately limited to
    state these Tasks create: the scoring attempt owns pending transient shots, and the coordinator
    owns disabling the flywheel request enabled by its outbound path callback. It does not clear
    intake, continuous-shooting, or eject requests that another future strategy/companion may own.
    Each cleanup owner terminalizes first, attempts every one of its selected child/owned cleanup
    actions even if an earlier action throws, and rethrows the first runtime failure with later ones
    suppressed. No Plant, device, or raw follower write is added.
  - **Error quality:** null or aliased direct roles, update-before-start, a second start attempt, and a
    completed child with a null/`NOT_DONE` outcome fail with the routine/scoring phase and role named,
    plus guidance to create a fresh Task graph from the existing factory. Malformed terminal route
    state names the retained route Task/status rather than falling through to a guessed policy.
  - **Telemetry contract:** Phoenix's current Auto presenter emits the current Task's dynamic
    `getDebugName()` and outcome, but does not call `debugDump()`. The coordinator's debug name will
    therefore include its routine label, active phase, selected `FALLBACK` or `ABORT` decision, and
    retained triggering route status while that policy remains current; in particular, a live-pose
    fallback remains visible after the adapter's mutable latest-route row changes to the fallback
    route. `debugDump()` additionally exposes the accumulated outcome and active child for explicit
    debug/test callers, but PHX-03 does not add presenter history or claim that an immediate terminal
    abort is retained after the runner detaches it.
  - **Bounded implementation and documentation scope:** change only the Phoenix scoring helper,
    package-private scoring-attempt Task, Phoenix Pedro routine factory/private coordinator, focused
    unit tests, and the route-policy text in Framework Principles, Phoenix Architecture, Phoenix
    Pedro/integration guidance, and any modern
    route example that still implies a generic sequence short-circuits. Do not implement PHX-04's
    match-time preemption, a new scheduler, new route geometry, a generic race/success-only Task,
    route-constructor cleanup, readiness checks, or a public autonomous DSL.
  - **Verification plan:** add pure tests with fake route executions, scoring Tasks/capabilities, and
    a manual loop clock for outbound success; both timeout sources; every cancellation-like status;
    skipped scoring on failed prerequisite; target/aim/shot scoring failure; normal and degraded
    return; return/fallback timeout and cancellation; direct cancellation in every phase; no fallback
    after direct cancel; owned-cleanup ordering, idempotence, and failure suppression; immediate
    completion and loop ordering; child lifecycle/configuration failures; duplicate identities;
    update before
    start; second-start rejection; retained outcomes; and debug phase/status/decision rows. Verify all
    four strategy ids use the explicit common policy and return fresh Task graphs. Run focused tests,
    full `:TeamCode:testDebugUnitTest`, `:TeamCode:compileDebugJavaWithJavac`, XML totals,
    `git diff --check`, changed-file whitespace/link/import/blocking-call checks, and exhaustive caller
    searches. Android Studio review should inspect the policy table in code and simulate or force a
    route/scoring failure; deliberate on-robot route obstruction, skipped-shot behavior, immediate
    flywheel stop, live-pose fallback, and Driver Station telemetry remain the hardware validation.
  - **Hypothesis and approval gate:** this preserves generic sequence behavior and follows the
    leading hypothesis's explicitly allowed small Phoenix helper after four repeated callers proved
    it necessary. The research refines the hypothesis because bare `branchOnOutcome(...)` is not a
    complete solution, but it does not move policy into the framework or add a public API. The change
    still defines major autonomous lifecycle, failure, and held-mechanism safety semantics, and it
    changes the observable outcome of `aimAndShootOne(...)`; stop at **Ready** for explicit user
    approval before editing Java, tests, or framework/robot documentation. `Approve PHX-03 design`
    authorizes only the package-private scoring/coordinator implementations, the exact status table,
    owned transient-shot/flywheel cleanup, dynamic current-task-name telemetry, focused tests, and
    synchronized documentation described above. It does not authorize PHX-04, any public framework
    API, route/config cleanup, publication/merge, or another tracker item.
  - **Approval:** the user approved the PHX-03 design on 2026-07-14. Implementation is limited to
    the package-private scoring attempt and route-policy coordinator, the recorded current-strategy
    status mapping, owned cleanup, dynamic current-task telemetry, focused tests, and synchronized
    documentation. PHX-04 and every adjacent API cleanup remain out of scope.
  - **Implementation (2026-07-14):** `PhoenixAutoTasks.aimAndShootOne(...)` now constructs one
    package-private scoring-attempt Task that requires target selection, aim, and shot-drain success
    in order; retains timeout/cancellation/unknown outcomes; and cancels only its owned transient
    shot after an abnormal post-request ending. `PhoenixPedroAutoRoutineFactory.build(...)` now
    constructs one package-private Phoenix/Pedro coordinator for every strategy. It applies the
    approved outbound/scoring/return status mapping, builds the live-pose return only after policy
    selects it, retains earlier timeout degradation, gives later cancellation-like results
    precedence, disables the route-owned flywheel request once, and exposes the current
    phase/decision/trigger through its dynamic Task name. Direct cancellation terminalizes first,
    cancels only the active child, performs owned cleanup, and never starts recovery. Generic Task,
    RouteTask, and RouteTasks behavior remains unchanged.
  - **Public API and simplicity verification:** both lifecycle owners and their constructors are
    package-private implementation types. Bytecode inspection confirms neither class is public;
    exhaustive production construction search finds exactly one scoring construction inside
    `PhoenixAutoTasks` and one routine construction inside the private helper behind
    `PhoenixPedroAutoRoutineFactory`. Direct constructor use elsewhere is same-package white-box
    testing only. The public factory matrix covers all four `PhoenixAutoStrategyId` values and
    repeated builds, proving fresh coordinator plus outbound/scoring/return child identities. No
    public builder, overload, route-policy type, scheduler, PHX-04 takeover behavior, or second
    student-facing construction path was added.
  - **Automated verification (2026-07-14):** the focused scoring suite reports 21 tests, the routine
    policy suite 27 tests, and the public factory matrix one test, all with zero failures, errors, or
    skips. Coverage includes the complete route-status matrix, retained timeout and cancellation
    precedence, skipped scoring after failed prerequisites, immediate-complete children,
    active-only/direct/reentrant cancellation, exact-once capability actions, cleanup ordering and
    suppression, malformed children, dynamic telemetry, `TaskRunner` fail-stop after scoring-start
    and live-return-build failures, all four strategies, and fresh graphs. The final
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` run succeeds; XML reports 326
    tests across 32 suites with zero failures, errors, or skips. Only the existing JDK 21/source-8
    and FTC SDK deprecation warnings remain.
  - **Documentation, static, and adversarial verification (2026-07-14):** Framework Principles,
    Phoenix Architecture, Pedro integration/Phoenix guidance, Task/capability guides, and modern
    route examples now consistently state that integrations report facts while robot routines own
    continue/fallback/abort policy and owned mechanism cleanup. Searches find no remaining modern
    generic route-then-score sequence or stale claim that Phoenix policy is missing. All nine
    changed Markdown files have valid local links; `git diff --check`, untracked-file whitespace,
    forbidden boundary import, blocking-call, and construction-path checks pass. Independent
    lifecycle, policy, API, documentation, and student-simplicity reviews found and drove fixes for
    terminal-child updates plus reentrant double-start/repeated-action cases; the final settled-tree
    rereview reports no remaining concrete defect.
  - **Android Studio / robot verification gate:** before publication, PHX-03 was held at
    **Verifying** for review of the two public factory calls, the private status table, truthful
    scoring outcomes, and the Driver Station Task-name phase/decision/trigger. Hardware validation
    should deliberately obstruct an outbound route and a scoring attempt, confirm that the shot is
    skipped or cancelled as recorded, confirm prompt flywheel shutdown, and confirm that only
    timeout selects the live-pose return. No robot-hardware result is claimed yet.
  - **Manual verification (2026-07-14):** the user reviewed the PHX-03 implementation in Android
    Studio and approved it with `PHX-03 looks good`. PHX-03 is **Done**. The hardware checks above
    remain recommended during normal robot bring-up and are not claimed by this approval.

### PHX-04 - Match-time fallback takeover

- **Problem to confirm:** a competitive Auto may need to abandon whatever phase is active when the
  remaining match budget reaches a threshold, put mechanisms in a known safe state, and generate a
  park route from the current pose. A route-local timeout or branch does not preempt a mechanism
  phase elsewhere in the routine, while wrapping the whole routine in an ordinary deadline does not
  by itself sequence the safe-state transition and replacement park exactly once.
- **External evidence:** Cuttlefish's
  [Far Auto time override](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Far.java)
  interrupts its active path/action sequence after the global threshold and starts a park path from
  the live follower pose. This is effective strategy but depends on scheduler-global mutation that
  Phoenix should keep behind a robot-owned policy owner.
- **Expanded Worlds evidence:** LOAD wraps the selected whole routine in a timed leave command, while
  SaMoTech tracks expected/actual step duration and Tech Tigers repeats park-time checks in selected
  drive states. These are three variations of the same match-wide policy. One robot Auto supervisor
  is clearer than scattered state checks, but the takeover threshold and park choice remain routine
  facts.
- **Alternatives to compare:** repeat raw time checks inside every routine phase; use only a route
  timeout; make a timer the TASK-04 deadline; wrap `A + B` in a timed override; add a generic race;
  add a Java-like asynchronous `finally`; or compose one generic timeout decorator around pre-park
  work A and put one park B after that timed region. Compare early A completion, B already running at
  25 seconds, direct FTC STOP, cleanup failure, route-policy truth, and student-facing API size.
- **Leading hypothesis:** keep one scheduler and one `LoopClock`. Add one factory-only generic
  `Tasks.withTimeout(Task, double)` decorator, bound only the Phoenix pre-park work A, and let one
  private Phoenix policy coordinator start one live-pose park B after A completes normally or is
  successfully cancelled at the match threshold. Once B starts, no timer owns it. Match threshold,
  park choice, mechanism safety, and route interpretation remain robot/routine facts.
- **Completion:** tests cover early A completion, timeout during every pre-park phase, B beginning
  before 25 seconds without interruption or restart, exactly-once B construction/start, active-child
  cleanup failure, abnormal PHX-03 route outcomes, safe mechanism requests, truthful route status,
  unavailable park geometry, repeated loop calls, direct cancellation, and shutdown. Telemetry
  distinguishes normal continuation, local degradation, match-time cutoff, suppressed park, park
  construction failure, and final park outcome.
- **Decision record (2026-07-14):**
  - **Current behavior and caller trace:** Phoenix owns one shared `LoopClock`, one private
    `TaskRunner`, and one continuously updated Pedro drive adapter. `startAny(...)` resets the clock
    at FTC START, but `startAuto()` is currently empty and the queued routine does not start until
    the first regular Auto loop. `PhoenixPedroAutoOpModeBase` is the only production installer and
    every concrete/selector OpMode converges on its one `PhoenixPedroAutoRoutineFactory.build(ctx)`
    call. No code currently observes a whole-match time budget. Route and scoring timeouts are only
    local facts, so a healthy route, a `parallelDeadline(...)` companion, or a mechanism wait can
    still consume the remaining time and prevent the routine's normal park/return from starting.
  - **A/B boundary correction after user review:** the current PHX-03 routine already contains
    outbound, scoring, and return/park. Calling that entire graph the primary in a timed override is
    wrong: if its normal park B begins at 20 seconds, the still-armed 25-second override cancels and
    restarts B. Calling only pre-park work A the primary is also insufficient for the proposed
    `timedOverride(A, 25, B)`, because B would not run when A finishes early. The timed region must be
    exactly A, with B outside it and started once after an allowed A ending.
  - **Why existing composition alone is insufficient:** repeating clock checks in every phase
    duplicates policy and can miss a new phase; a route timeout cannot preempt scoring or companion
    work; and neither orientation of `parallelDeadline(timer, A)` works. A timer deadline delays B
    until 25 seconds even when A finishes early, while an A deadline cannot be preempted by timer
    completion. `sequence(withTimeout(A, 25), B)` is the correct generic execution shape, but bare
    `SequenceTask` deliberately advances after any non-throwing child completion and does not retain
    the exact PHX-03 route decision. Phoenix therefore needs a private outcome-policy coordinator,
    not another public composition spelling.
  - **Generic public construction-path audit:** `sequence(...)`, `parallelAll(...)`,
    `parallelDeadline(...)`, and `branchOnOutcome(...)` are factory-only generic compositions in
    `Tasks`; they accept fresh child `Task` instances and return `Task`, while their implementations
    and constructors are package-private. Add exactly one supported sibling:
    `Tasks.withTimeout(Task task, double timeoutSec)`. Back it with one package-private
    `TimeoutTask`; do not expose a constructor, concrete type, `of(...)`, builder, Task default
    method, `Supplier<Task>` overload, match-time alias, or second factory layer. Task-first argument
    order parallels `waitUntil(condition, timeoutSec)`, and the conventional decorator name says
    only what the framework owns.
  - **Generic timeout contract:** require a non-null Task and a finite, nonnegative duration. The
    duration begins at the wrapper's own `clock.nowSec()` start boundary, never from preceding
    `dtSec()`. Zero completes immediately with `TIMEOUT` without starting the child. A positive
    duration starts the child at that boundary and captures an immediate terminal outcome. On each
    later update, an already-terminal child wins, including at the exact threshold; otherwise
    `elapsed >= timeoutSec` wins before another child update. A natural completion retains the
    child's validated terminal outcome. A framework timeout actively cancels the child and reports
    `TIMEOUT` only after cancellation returns and the child is terminal. A throwing, nonterminal, or
    malformed cancellation fails closed and cannot release a continuation. Direct wrapper
    cancellation is active-only, reports `CANCELLED`, and never changes into a timeout. Single-use,
    update-before-start, same-cycle, reentrant-callback, and retained-debug rules match the other
    generic compositions.
  - **Built-in timeout audit and ownership rule:** do not remove task-local timeouts as a consequence
    of adding the decorator. They own different semantics: `PlantTasks.move(...).timeout(...)`
    applies its success/timeout `.thenTarget(...)` rather than its cancellation target;
    `RouteTask`'s route-factory-supplied local timeout records `RouteStatus.TASK_TIMEOUT` and uses
    timeout-specific route cancellation; output-pulse maximums begin at the gated RUN phase and may
    continue through
    cooldown; calibration's staged `failAfterSec(...)` forces an explicit search-safety choice; and
    guidance's no-guidance window resets when usable guidance returns. The common
    `Tasks.waitUntil(condition, timeoutSec)` overload also remains: it is materially simpler and
    samples its condition before its local timeout at the exact boundary. Fixed-duration actions
    are successful timed commands, not timeouts. `DriveGuidanceTask.Config.timeoutSec` is the only
    superficially replaceable whole-task cap, but it remains an operation-owned safe default; its
    removal would be a separate API decision, not PHX-04 cleanup.
  - **Timeout scope rule for documentation:** use a task-local timeout when the operation owns
    phase timing, safe terminal behavior, or a truthful domain status. Use
    `Tasks.withTimeout(...)` when a parent owns a hard budget around an otherwise complete Task or
    composite graph. The decorator reaches its deadline through the child's ordinary active
    `cancel()` path; it cannot invoke that child's internal timeout transition. The wrapper may
    therefore report `TIMEOUT` while the retained child reports `CANCELLED`. Nested local and outer
    limits are valid when they represent different scopes—such as a four-second route safeguard
    inside a 25-second pre-park budget—but should not duplicate the same policy. When the local
    timeout reason matters, configure the outer budget to expire later.
  - **Concrete generic code shape and `finally` boundary:** a custom robot that truly wants B after
    every non-throwing terminal result of A writes:

    ```java
    Task auto = Tasks.sequence(
            Tasks.withTimeout(A, 25.0),
            B
    );
    ```

    If A finishes at 18 seconds, B starts then. If A remains active at 25 seconds, the decorator
    cancels A safely and B starts once. If B began earlier, the completed decorator no longer has a
    timer that can interrupt it. This is a bounded continuation, not Java `finally`: direct
    cancellation/FTC STOP or a lifecycle/cleanup exception cancels the graph and must not start an
    asynchronous driving Task. Mandatory physical cleanup belongs in A's active `cancel()` path.
  - **Phoenix public API and one-root ownership:** retain the tested `TaskRunner` only as
    `PhoenixRobot`'s private, single-root lifecycle driver. Replace `enqueueAuto(Task)` with the
    one-shot, INIT-only `installAutoRoutine(Task)` and remove public mutable `autoRunner()`; caller
    search found no compatibility need outside the shared Auto base. Installation rejects null, a
    second/late install, start without an installed routine, and repeated start. The supported
    public installation call remains one line:
    `robot.installAutoRoutine(PhoenixPedroAutoRoutineFactory.build(ctx))`. Do not add a timed install
    overload, public supervisor/status type, or public `PhoenixAutoTasks.withMatchTimePark(...)` that
    duplicates the one generic timing construction path. This does not make the private Phoenix
    coordinators free of robot-code maintenance cost; everything under `edu.ftcphoenix.robots.phoenix`
    is robot code for simplicity review.
  - **Phoenix routine composition and PHX-03 preservation:**
    `PhoenixPedroAutoRoutineFactory.build(ctx)` keeps its public `Task` return type and remains the
    only supported public construction entry point. Internally, split the current private
    coordinator into
    (1) one pre-park policy Task A for outbound plus scoring, (2)
    `Tasks.withTimeout(A, profile.auto.parkTakeoverElapsedSec)`, (3) one live-start return/park
    `RouteTask` B, and (4) one private Phoenix A-to-B policy coordinator. The coordinator starts B
    once after A/bounded-A `SUCCESS` or `TIMEOUT`, including the match cutoff. It does not start B
    after PHX-03's interruption, replacement, failure, cancellation, or unknown endings, after a
    lifecycle/cleanup failure, or after direct root cancellation. This private gate is required
    because generic `sequence(...)` must not choose season route-recovery policy.
  - **One park and live geometry:** the initial checked-in `parkTakeoverElapsedSec` is `25.0`, and
    Phoenix rejects a non-finite or non-positive match threshold even though the generic decorator
    defines zero. B is one fresh Task built through `RouteTasks.followBuiltAtStart(...)`. Its object
    may be allocated during routine construction, but its supplier samples current pose and builds
    geometry exactly once only when B starts. The same B serves normal early completion, a local
    outbound/scoring timeout, and the global cutoff; there is no separate emergency park to restart
    it. Threshold and destination remain profile/routine facts, and Pedro types remain at the
    integration edge.
  - **Exact FTC START and first-loop ordering:** `startAuto()` starts the installed timed root through
    the private runner immediately after `startAny(...)` resets the shared clock, so 25 seconds is
    measured from the exact FTC START boundary rather than the first loop. Generic timed composition
    starts its primary normally. The private Phoenix routine's `start(clock)` becomes an arm-only
    boundary that records its cycle without starting outbound behavior; its same-cycle update is a
    no-op. Its normal child starts on the first later Auto Task phase, after localization, targeting,
    and the continuously owned Pedro heartbeat. This preserves generic Task semantics and Phoenix's
    established loop order without a competing clock.
  - **Phoenix cancellation is the safety boundary:** pre-park A owns cleanup required before B. Its
    active `cancel()` terminalizes first; best-effort cancels its active route/scoring phase; clears
    only Auto-owned transient scoring, feed, intake, aim, and flywheel requests; and immediately
    stops the drive adapter. It attempts every safety action even if an earlier one fails and
    propagates one aggregate with later failures suppressed. The timeout decorator becomes terminal
    only after that cleanup returned and A is terminal, so the private coordinator cannot start B
    after failed cleanup. Direct robot cancellation performs the same cleanup but terminalizes the
    outer coordinator first, so it never selects B. No generic cleanup callback or Phoenix
    capability enters `Tasks`.
  - **Truthful terminal state:** `TimeoutTask.debugDump(...)` retains elapsed/threshold state, whether
    its own timeout fired, and the child snapshot after completion. The private Phoenix coordinator
    retains whether A ended normally, locally timed out, or was cut off by match time, plus B's exact
    `RouteStatus` for success, endpoint timeout/stall, interruption, replacement, cancellation,
    failure, and unknown. A successful B after any allowed A timeout remains overall `TIMEOUT`; a
    cancellation-like B result takes precedence as `CANCELLED`; and B never triggers another
    fallback. Phoenix retains the installed root for terminal telemetry. No public timeout concrete
    type or status model is justified.
  - **Framework Principles and simplicity check:** the design is cooperative and non-blocking; uses
    one heartbeat, clock, runner, drive owner, and root Task graph; preserves single-use,
    active-only cancellation, start-boundary timing, and source/capability ownership; keeps generic
    lifecycle in `fw.task` and robot policy in Phoenix; and removes two scheduler-facing Phoenix
    paths in favor of one install path. API parallelism exists at the supported factory layer:
    generic compositions and decorators are factory-only siblings in `Tasks`, while Phoenix uses
    that layer instead of exposing another spelling. A routine installer still selects a spec and
    calls one factory, but that short entry point is not by itself evidence that the complete Phoenix
    robot implementation is simple. `AUTO-01` will evaluate whether the private coordinator's
    lifecycle ceremony can be reduced without moving match-specific policy into the framework.
  - **Rejected public paths and deferred scope:** do not implement this as a public
    `MatchTimeAutoTask`, retain `timedOverride(...)`, add `timeoutThen(...)`, call an asynchronous
    continuation `finally`, put Phoenix/Pedro/safety callbacks in `Tasks`, add a generic race or
    `takeOverWhen(...)`, add an eager-plus-supplier overload pair, expose an `of(...)`/constructor,
    add a second scheduler/clock, or add a Phoenix timing alias merely for symmetry. `deadlineThen`
    would conflict with the established parallel-deadline term. A future condition-driven takeover,
    true cleanup hook, or public typed status requires an independent caller and decision gate. Do
    not change generic sequence outcome policy, add real season park geometry, readiness checks,
    mode handoff, or another tracker item.
  - **Bounded implementation scope:** add the one `Tasks.withTimeout(...)` facade method,
    package-private implementation, focused generic tests, and synchronized Task principles/guides;
    change only the Phoenix Auto profile snapshot, private pre-park/final routine lifecycle,
    routine factory/private live-pose park helper, root installation/telemetry path, focused Phoenix
    tests, and synchronized Phoenix/Pedro documentation.
  - **Verification plan:** generic tests cover null/non-finite/negative construction and zero;
    just-before, exact, and after-threshold timing; immediate/early child completion; child
    success/timeout/cancellation/unknown outcomes; timeout cancellation that succeeds, throws,
    returns nonterminal, or re-enters; child start/update/completion/outcome failures; direct
    cancellation before start, while active, and after terminal; single-use, update-before-start,
    same-cycle/reentrant calls, and retained debug state. Phoenix tests cover early A completion;
    match cutoff during outbound, deadline companions, scoring, and mechanism waits; B already
    active at 25 seconds without cancellation/restart; exact FTC START with no INIT/first-loop
    charge; every PHX-03 cancellation-like result with B suppressed; every best-effort safety action
    and suppression order; cleanup/drive-stop failure with B unstarted; direct cancellation in A and
    B; live pose sampled once; truthful B results; shutdown; duplicate install/start; all strategy
    ids and fresh identities; retained telemetry; and unchanged Pedro heartbeat/immediate-zero
    behavior. Run focused suites, full `:TeamCode:testDebugUnitTest`,
    `:TeamCode:compileDebugJavaWithJavac`, XML counts, construction-path/caller/import/blocking
    checks, links, `git diff --check`, independent lifecycle/API/simplicity reviews, and Android
    Studio inspection. On robot, tune the 25-second threshold with real paths, interrupt each major
    pre-park phase near it, let normal B cross the threshold, and confirm safe mechanisms, immediate
    zero, one live-pose park, and truthful Driver Station status.
  - **Approval gate:** this now adds one public generic composition API, removes two public Phoenix
    scheduling paths, splits the private PHX-03 routine at the A/B boundary, changes Phoenix root
    cancellation/start semantics, and replaces the previously proposed timed override. Stop at
    **Ready**. `Approve PHX-04 bounded-pre-park design` authorizes only the exact factory-only
    `Tasks.withTimeout(...)` API, private Phoenix A-to-B integration, failure rules, telemetry,
    tests, and documentation above; it does not authorize PHX-02, publication, or adjacent cleanup.
  - **Approval (2026-07-14):** the user approved the bounded-pre-park design with
    `Approve PHX-04 bounded-pre-park design`. Implementation is limited to the exact generic
    timeout decorator, private Phoenix A-to-B policy and safety boundary, exact Auto root
    installation/start behavior, focused tests, telemetry, and synchronized documentation recorded
    above. PHX-02, adjacent cleanup, staging, publication, and merge remain out of scope.
  - **Implementation (2026-07-14):** added the sole public construction path
    `Tasks.withTimeout(Task, double)` backed by a package-private, single-use `TimeoutTask`. The
    decorator measures from its own start timestamp, gives an already-terminal child precedence at
    the exact boundary, uses ordinary active cancellation when its hard budget wins, and fails
    closed without releasing continuation after throwing, nonterminal, or malformed cleanup.
  - **Phoenix integration (2026-07-14):** split the private Pedro routine into bounded outbound plus
    scoring A and one live-start return/park B outside the timer. `PhoenixProfile.AutoConfig` now
    carries the copied `25.0`-second default, the routine preserves PHX-03 continue/fallback/abort
    policy, and match cutoff performs the full capability-owned scoring/drive cleanup before B.
    A persistent-heartbeat route result or scoring result that becomes terminal on the exact cutoff
    cycle is observed first, so cancellation-like or malformed evidence suppresses B instead of
    being mistaken for a normal cutoff. B is built and started at most once and cannot be interrupted
    or restarted by the completed A timer.
  - **Root lifecycle and telemetry (2026-07-14):** Phoenix now exposes one INIT-only
    `installAutoRoutine(Task)` path while its `TaskRunner` stays private behind
    `AutoRoutineLifecycle`. FTC START is captured before last-chance initialization, resets the one
    shared clock, and starts the installed root immediately at the reset cycle identity (cycle zero
    under the then-current clock contract; TIME-01 later made reset identities monotonic); physical
    outbound work remains arm-only until the first later ordered Auto loop. The installed root is retained after
    terminal completion, obsolete queue telemetry was removed, and routine name/outcome plus the
    private policy trigger retain normal, cutoff, suppressed, construction-failure, and final-route
    state.
  - **Automated verification (2026-07-14):** `:TeamCode:testDebugUnitTest` and
    `:TeamCode:compileDebugJavaWithJavac` pass. The final XML contains **35 suites / 350 tests / 0
    failures / 0 errors / 0 skipped**. PHX-04-focused coverage comprises 15 generic timeout tests, 6
    one-root lifecycle tests, 2 profile-copy tests, 26 Phoenix routine-policy tests, and 2 routine
    factory tests; the 27-test `RouteTaskStatusTest` suite also passes with the live terminal-status
    observation regression. Coverage includes exact-cutoff route/scoring races, a real
    `parallelDeadline(...)` mechanism-wait/companion graph, reentrant policy callbacks, malformed
    lifecycle snapshots, construction failure, best-effort cleanup, direct cancellation, live-pose
    single construction, and all allowed/aborting route outcomes.
    The same two Gradle checks were rerun after final approval on 2026-07-15 and passed with the same
    **35 suites / 350 tests / 0 failures / 0 errors / 0 skipped** result.
  - **Static and documentation verification (2026-07-14):** construction-path search finds
    `new TimeoutTask(...)` only in `Tasks`; no timeout aliases, public Phoenix runner/queue escape
    hatches, or obsolete queue telemetry remain. No sleeps or busy waits were added (the one new
    `while` is a bounded in-call drain of already-completed private phases). `git diff --check`
    passes, and local links validate in all 9 changed Markdown files. Framework Principles, generic
    guides, Pedro integration guidance, Phoenix architecture, examples, Javadocs, and selector help
    are synchronized.
  - **Independent review (2026-07-14):** lifecycle, API/simplicity, test-coverage, exact-boundary,
    and final-diff reviews were completed. They identified and the implementation now resolves stale
    queue terminology, missing beginner documentation, park-construction failure telemetry, an
    exact-cutoff route/scoring race, terminal-plus-`NOT_DONE` scoring evidence, and lost PHX-03
    reentrant/malformed regressions. The final reviewer reported no unresolved PHX-04 blocker.
  - **Android Studio / hardware audit point (2026-07-14, before approval):** status was
    **Verifying** and no files were staged or committed. Robot-hardware validation still needs to
    tune the threshold with real paths and confirm Pedro immediate physical zero, safe scoring/feed
    requests near cutoff, one feasible live-pose park, an already-running park crossing 25 seconds
    without restart, and truthful Driver Station routine status.
  - **Manual approval and publication authorization (2026-07-15):** the user accepted the Android
    Studio/manual audit with `PHX-04 looks good` and authorized proceeding. The user also clarified
    that “robot code” includes every class under `edu.ftcphoenix.robots.phoenix`; the one-line public
    installer therefore must not be used as a proxy for total student/season maintenance burden.
    That follow-up is tracked separately as `AUTO-01`. Hardware validation remains recommended and
    is not represented as completed.

### PHX-02 - Phoenix runtime readiness

- **Problem to confirm:** placeholder routes, uncalibrated localization values, missing alliance tag
  facts, unavailable services, or an incompletely configured Pedro `FollowerBuilder` may still allow
  assists/Auto to attempt initialization. The Pedro review found declared drivetrain/Pinpoint
  configs that are not actually supplied to `Constants.createFollower(...)`. Cuttlefish also checks
  the live initialized follower pose against the selected route start and emits an unmistakable
  warning when they disagree, a useful readiness check that Phoenix currently lacks.
- **Alternatives to compare:** robot-owned validation report; scattered constructor checks; disabling
  individual features; or framework-generic validation primitives.
- **Leading hypothesis:** Phoenix owns the actual readiness rules, while the Pedro integration owner
  reports whether its pinned drivetrain/localizer and coordinate contract are complete. Readiness
  compares the selected route start with the live pose under explicit distance/heading tolerances
  and a documented warn-versus-refuse-to-arm policy. A tiny generic validation report is worth
  extracting only if another framework caller needs the same aggregation behavior; active staged
  hardware motion remains CHECK-01, not INIT validation.
- **Completion:** unsafe modes fail before start with actionable telemetry; valid partial-feature
  configurations remain possible when explicitly selected; placeholder routes, invalid Pedro
  construction, and a selected-start/first-route mismatch cannot be armed as a match Auto; expected
  physical placement is shown without claiming that a software pose rebase measured it.
- **Decision record (2026-07-15): Ready, with a material correction to the leading start-pose
  hypothesis.** This is a major Phoenix lifecycle and robot-facing policy decision, so no Java,
  tests, or API/documentation implementation may begin until the user explicitly approves the
  design below.
  - **Confirmed current behavior:** every checked-in strategy currently builds the same documented
    twelve-inch Pedro integration placeholder. `SAFE_PRELOAD`, `PRELOAD_AND_PARK`, and
    `PARTNER_AWARE_CYCLE` are nevertheless shown as selectable match strategies; both static
    audience-side entries install the placeholder; and the shared base plus `PhoenixRobot` emit
    variations of “auto ready.” A production-looking OpMode can therefore arm test geometry.
    `PhoenixProfile` also has zero Pinpoint pod offsets and
    `pinpointPodOffsetsCalibrated == false`, but neither calibration acknowledgement participates in
    match arming or TeleOp-assist availability. Finally, `PhoenixAutoProfiles` silently retains the
    complete scoring-target catalog when the selected alliance tag is absent, which can leave the
    other alliance's target eligible instead of reporting the bad config.
  - **What earlier work already solved:** PEDRO-02 made
    `Constants.createPhoenixAutoRuntime(...)` and `PedroPathingRuntime.create(...)` all-or-throw
    production construction paths. They validate the selected drivetrain/Pinpoint physical config,
    predictor, Follower, mecanum constants, path constraints, coordinate transform, hardware
    construction, and passive-localizer contract. The passive localizer also requires a finite
    start pose before its first heartbeat and verifies that the predictor publishes the requested
    rebase. Phoenix service constructors already fail initialization when required vision,
    localization, targeting, or scoring owners cannot be built, and the shared Auto base refuses to
    call `startAuto()` after that failure. PHX-02 will consume these truthful construction facts; it
    will not add a redundant `PedroPathingRuntime.isReady()`, duplicate vendor validators, or
    optional/no-op service graph.
  - **Current callers and ordinary student path:** `PhoenixPedroAutoOpModeBase` is the sole shared
    build/start path for the two static match entries, the match selector, and the explicit Pedro
    test entry. `PhoenixAutoProfiles.profileFor(...)` is called only from that base.
    `PhoenixPedroPathFactory.build(...)` feeds the routine factory and its focused tests. TeleOp
    assists are wired once in `PhoenixRobot.initTeleOp()`. The normal student-facing match OpMode
    remains a tiny `autoSpec()` implementation; students will not construct reports, validators, or
    builders. The selector and shared base will render and enforce the same robot-owned facts.
    The public-path audit found no PHX-02 cleanup need in `PhoenixRobot`'s distinct constructor or
    `initAuto` convenience/configuration overloads, the single production Pedro runtime factory,
    the distinct tool-only Follower path, or eager versus start-time route factories. CLEAN-01 owns
    the already-recorded `RouteTask` construction cleanup.
  - **Framework Principles and simplicity comparison:** one centralized robot policy gives one
    answer to “may this behavior run?” while the existing component owners continue answering “was
    my configuration constructed correctly?” Required warnings stay in normal Driver Station
    telemetry, not `debugDump()`. Ordinary match code gains no call or concept; the only extra
    concept visible in robot internals is the necessary distinction between match-ready and
    integration-only routes. This is simpler and more truthful than scattered exceptions, silent
    no-op capabilities, a generic feature matrix, or a second framework validation DSL. Manual
    TeleOp remains usable when localization calibration is incomplete, but localization-dependent
    assists are visibly unavailable rather than silently applying questionable corrections.
  - **Chosen report shape:** add one Phoenix-owned, immutable, factory-only readiness result; do not
    put it in `fw`. Its ordered issues have a stable id, `WARNING` or `BLOCKING` severity, an
    actionable message, and remediation text; the result exposes whether the requested behavior is
    allowed. There is no public constructor, builder, validator interface, telemetry dependency, or
    generic framework `ValidationReport`. One Phoenix policy factory evaluates TeleOp pose assists
    and Pedro Auto from the relevant immutable profile/spec/route facts. Presenters and the shared
    Auto base format that snapshot for humans. Extract a generic report only after a second
    non-Phoenix production caller demonstrates compatible semantics.
  - **Typed route maturity and one source of truth:** give the Pedro path owner a typed route fact
    such as `MATCH_READY` versus `INTEGRATION_ONLY`; never infer safety from a label containing
    “placeholder.” The same path-factory-owned lookup used by `build(...)` supplies selector
    availability and the built `Paths` snapshot, so strategy enum tags, selector rules, and geometry
    cannot become parallel authorities. The current twelve-inch geometry is
    `INTEGRATION_ONLY`. It blocks all match entries, including START-without-confirmation in the
    selector. It may run only from the explicitly marked Pedro integration-test entry, with an
    unmistakable persistent `TEST` warning; that test strategy is disabled in the match selector.
    Real geometry must be added and deliberately marked match-ready before a static or selected
    match Auto can arm.
  - **Calibration and partial-mode policy:** match Auto requires both existing human
    acknowledgements: verified Pinpoint axes and calibrated pod offsets. Unverified axes also block
    a moving Pedro integration test because the response direction may be unsafe; incomplete pod
    offsets are a persistent warning in that explicit test mode so the diagnostic route remains
    usable for bounded integration work. In TeleOp, either incomplete acknowledgement leaves manual
    drive and mechanisms available while gating both localization-dependent auto-aim and shoot-brace
    overlays off and printing the exact calibration action required. This is the supported
    partial-feature case. PHX-02 will not introduce nullable/optional Phoenix capabilities, pretend
    missing hardware is present with no-ops, or block merely because no AprilTag happens to be
    visible during INIT. Only the selected alliance's target is required; an absent inactive-
    alliance target does not block that spec.
  - **Alliance and field-fact policy:** the selected red/blue scoring tag id must exist in
    `autoAim.scoringTargets` and in the configured fixed `TagLayout` before the Auto-specific profile
    is constructed. Missing data is one actionable blocker naming the exact profile field and tag
    id; the full-catalog fallback is removed. `PhoenixAutoProfiles.profileFor(...)` retains its one
    transformation role and also fails fast when called directly with the same invalid selected
    fact. Broad profile-null/field validation remains API-03, and backend streaming/health policy
    remains VISION-01 rather than being invented here.
  - **Corrected start-pose decision:** the tracker previously proposed comparing the selected route
    start to a “live” follower pose under configurable tolerances. In Phoenix, the current
    `setStartingPose(...)` deliberately rebases the only Pinpoint predictor to that selected pose
    and immediately asserts software equality. Comparing the resulting values would therefore
    always pass and cannot prove where the robot is physically sitting. Cuttlefish obtains a
    meaningful relative check only by assigning a separate setup pose, repeatedly polling during
    INIT, and expecting the drive team to move the robot from that setup pose to the route start.
    Phoenix already builds after the exact spec is selected. Adding that second pose, operator
    movement procedure, pre-START Pedro heartbeat, and a new `LoopClock` phase-rebase API would add
    substantial lifecycle and student complexity; moving or lifting the robot could still make the
    relative odometry claim wrong. It is not the best general Phoenix path.
  - **Truthful replacement start contract:** keep one declared selected start pose, require the
    first fixed route geometry to start there under a tight structural epsilon at the Pedro
    boundary, retain the integration's existing rebase/publication assertion, and show the expected
    field start prominently throughout INIT. Reapply that same selected pose at FTC START before
    the first Pedro heartbeat, then start the installed root only while the retained report is
    allowed. This gives one software coordinate authority and avoids charging or skipping an INIT
    interval without changing `LoopClock` cycle semantics. It does **not** claim to verify physical
    field placement; the drive team must place the robot at the displayed start. A future automatic
    placement gate requires a fresh independently trustworthy field-absolute observation, such as a
    calibrated AprilTag pose, and a separate decision about unavailable/stale observation policy.
  - **Arming, status, and error contract:** `PhoenixRobot.initAuto(...)` reports only that its owned
    services initialized; it must not claim the whole selected Auto is ready. The shared Pedro base
    retains the readiness result, expected start, and exact initialization failure. It installs and
    starts no root when any blocker remains, rechecks the retained arm decision at START so selector
    bypass cannot move the robot, and labels `BLOCKED`, `TEST`, `WARN`, and `READY` distinctly in
    always-on telemetry. Remove the universal “check drive/Pinpoint” suffix from unrelated failures.
    INIT cleanup remains best-effort, but a cleanup failure is retained/suppressed alongside the
    original cause instead of disappearing. Active staged hardware motion remains CHECK-01.
  - **Rejected alternatives:** documentation-only leaves placeholder match OpModes runnable;
    scattered constructor checks cannot aggregate route maturity, calibration, and alliance policy;
    silently substituting no-op services hides capability loss and complicates every caller; a
    generic framework report has no second compatible runtime caller; tester-only
    `CalibrationStatus` does not represent multi-issue arming policy; a generic optional-feature
    matrix has no concrete Phoenix caller and would spread branching through capabilities and
    controls; a post-rebase pose comparison is tautological; and Cuttlefish's separate setup-pose
    INIT heartbeat is a valid team-specific operating procedure but is not simpler or more truthful
    for Phoenix's already-selected spec. Do not change the generic Task, route, Pedro heartbeat, or
    clock APIs in PHX-02.
  - **Bounded implementation scope:** change only the small Phoenix readiness policy/result,
    Phoenix calibration-dependent TeleOp-assist wiring/status telemetry, Auto profile selection,
    Pedro path maturity/start validation, the shared selector/base/test-entry arming path, focused
    Phoenix tests, and synchronized Framework Principles/Phoenix/Pedro/calibration documentation.
    Do not add real season route geometry, active hardware checks, generic framework validation,
    optional service graphs, live-vision arming, unrelated profile validation, or another task.
  - **Verification plan:** add pure readiness-matrix tests for every current strategy and run
    purpose; placeholder match blocking; the explicit test escape hatch; axes/offset combinations;
    manual TeleOp versus assist availability; selected versus inactive alliance facts; absent
    catalog/layout entries; deterministic issue ordering and immutability. Add profile tests proving
    valid red/blue filtering and actionable missing-target failure. Extend path tests for maturity,
    nonempty/finite geometry, declared-start/first-point translation and wrapped-heading equality,
    and structural mismatch refusal. Add focused arming/lifecycle coverage proving a blocker never
    installs or starts the root, START cannot bypass the selector, test-only state stays visible,
    the start pose is applied before the first heartbeat, no route callback or actuator starts in
    INIT, retry clears stale reports, and failure cleanup preserves the primary cause. Verify
    TeleOp's manual path remains live while both pose assists are gated. Re-run all Pedro runtime,
    passive-localizer, adapter, route, routine, PHX-03/PHX-04 timing tests; full
    `:TeamCode:testDebugUnitTest`; `:TeamCode:compileDebugJavaWithJavac`; XML totals; caller,
    raw-Follower-lifecycle, blocking-call, link, and `git diff --check` audits.
  - **Android Studio and hardware audit point:** inspect that static match entries and the selector
    show `BLOCKED` with the exact route/calibration remedy, that only the dedicated integration-test
    entry can show `TEST`, and that normal static OpModes still contain only `autoSpec()`. On the
    robot, prove blocked match entries cannot move or spin mechanisms; confirm manual TeleOp remains
    controllable while unavailable assists stay off; verify calibrated real geometry can arm; and
    check that the explicitly Pedro-field-labeled expected pose matches physical placement. The last
    check is an operator validation, not an automated absolute-position claim.
  - **Approval gate:** `Approve PHX-02 readiness design` authorizes only the exact robot-owned report,
    route-maturity/test policy, calibration-dependent TeleOp gating, alliance/start-contract checks,
    fail-closed arming/error telemetry, tests, and synchronized documentation above. It does not
    authorize a Cuttlefish-style setup-pose heartbeat, a `LoopClock` API change, generic framework
    validation, real route geometry, CHECK-01, publication/merge, or another tracker item.
  - **Approval (2026-07-15):** the user approved the design with
    `Approve PHX-02 readiness desing` (interpreted as the requested PHX-02 readiness-design
    approval). Implementation is limited to the exact scope and exclusions above.
  - **Implementation (2026-07-15):** added the immutable factory-only `PhoenixReadiness` policy,
    exact selected-alliance profile validation/filtering, path-factory-owned `RouteAvailability`,
    structural declared-start validation, calibration-dependent TeleOp pose-assist gating, and one
    shared Pedro Auto arming/start/error path for static entries, the selector, and the dedicated
    integration test. All checked-in geometry remains deliberately `INTEGRATION_ONLY`, so every
    match entry is blocked and only the visibly named Pedro test entry may run it with persistent
    `TEST` warnings. The selected Pedro start is reapplied before the shared clock/root start, and
    telemetry names its physical-placement value `auto.expectedPhysicalStartPedro` with explicit
    Pedro-field inches/degrees. Construction/start failures retain their exact cause; cleanup
    failures are suppressed, displayed with restart guidance, and prevent competing-owner retries.
    No real route geometry, live-vision gate, optional capability graph, generic validation API,
    hardware-motion check, or adjacent tracker item was added.
  - **Adversarial review (2026-07-15):** independent Auto, TeleOp/readiness, Pedro-documentation, and
    final Framework-Principles reviews found and resolved route/spec mismatch risk, raw-versus-gated
    auto-aim status, inactive-alliance layout coverage, invalid/overflowing placeholder distance,
    stale selector reports, retained cleanup failures, conflicting reinitialization, INIT/START
    documentation drift, ambiguous field-frame telemetry, and unnecessarily broad readiness/test
    hooks. The final review found no remaining blocker; no extra public or protected student
    extension seam was added for tests.
  - **Automated verification (2026-07-15):** Android Studio's bundled JDK completed
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` successfully. The generated XML
    contains 39 suites / 380 tests / 0 failures / 0 errors / 0 skipped. Focused coverage includes 9
    readiness-policy, 5 Auto-profile, 10 Pedro-path, 2 TeleOp drive-assist, 7 shared Auto OpMode, and
    7 passive-localizer tests. It proves selector START cannot bypass a blocker, repeated pre-heartbeat
    start-pose publication is allowed, START reapplies pose before clock/root start with no INIT
    behavior, ordinary retry refreshes state, and primary/cleanup failures remain truthful. Static
    audits found no changed production sleep/busy-wait pattern or raw Follower lifecycle owner;
    changed Markdown local links resolve; `git diff --check` and trailing-whitespace checks over all
    modified/untracked files pass. The only compiler output is the repository's existing Java 8 on
    JDK 21 deprecation warning. Robot hardware behavior remains unverified until the user audit.
  - **Android Studio audit requested (2026-07-15):** inspect `PhoenixReadiness`,
    `PhoenixPedroPathFactory.RouteAvailability`, `PhoenixPedroAutoOpModeBase`, the selector/test
    entries, TeleOp drive-assist gating, and always-on telemetry. Confirm current match entries and
    selector choices show `BLOCKED` without constructing/moving hardware; the dedicated Pedro test
    alone shows `TEST`; static OpModes still only provide `autoSpec()`; manual TeleOp remains
    available while both pose assists report unavailable; and the expected physical start is
    explicitly labeled as Pedro-field coordinates. Hardware validation is useful for the no-motion
    block and manual-drive/gated-assist behavior, but flipping placeholder routes to match-ready is
    intentionally outside PHX-02.
  - **Manual approval (2026-07-15):** after the Android Studio audit point and readiness-workflow
    walkthrough, the user replied `PHX-02 looks good`, approving this exact implementation for
    publication and merge. No robot-hardware validation was reported; all current match routes
    remain deliberately blocked until real geometry is implemented and validated.

### MATCH-01 - Explicit Auto-to-TeleOp handoff

- **Problem to confirm:** FTC starts TeleOp with a new OpMode and robot runtime, but a competitive
  robot often needs the Auto's final pose or calibration state. Without an explicit boundary,
  students reach for public static fields, string-keyed global maps, or disk persistence. Those
  approaches make stale match data, unchecked casts, and ownership/clear timing easy to miss.
- **External evidence:** SaMoTech carries final pose and turret offset through
  [`PoseStorage`](https://github.com/SaMoTechRobotics/FTC-Decode/blob/c8ff047c4245f79d934e622c05481269b5cc42da/TeamCode/src/main/java/robot/chassis/PoseStorage.java).
  LOAD carries final pose and alliance through public static fields consumed by TeleOp. Tech Tigers
  publishes pose, game-piece inventory, motif, and calibration flags through a string-keyed
  `RobotSaveState` and restores them with casts/fallback exceptions. Three unrelated Worlds robots
  therefore need the boundary even though their payload fields differ.
- **Alternatives to compare:** no framework change plus a documented robot-local static holder; an
  immutable Phoenix-specific holder; a tiny typed process-local FTC handoff; any suitable facility
  already present in the pinned SDK; a string/object blackboard; or serialized preferences/files.
  Compare missing, wrong-type, stale, already-consumed, process-restart, diagnostic/test OpMode, and
  Auto-init/stop-failure behavior. Do not assume a cross-mode `LoopClock` instance exists.
- **Leading hypothesis:** the robot defines one immutable data-only snapshot containing only the
  facts its next mode may accept. A narrow FTC-boundary, typed, one-shot carrier publishes that
  snapshot from the Auto composition root and lets TeleOp consume it or use an explicit robot-owned
  fallback. It records enough producer/freshness metadata to reject stale or wrong-type data and
  has an explicit clear operation. Keep hardware objects, Tasks, vendor poses, services, and season
  strategy out of the carrier; convert poses at the publishing boundary. Prefer a robot-named
  wrapper at the call site, and do not introduce a global robot-state map or event bus.
- **Completion:** tests cover publish/consume once, missing data, explicit fallback, wrong type,
  stale data, repeated Auto/TeleOp/test OpModes in one app process, explicit clear, process-local
  limitations, concurrent misuse, Auto cleanup failure, and defensive immutability. Phoenix keeps
  the required clear, publish, and consume boundaries obvious and centralized in its shared
  lifecycle owners, and ordinary robots that do not carry state gain no required setup.
- **Decision record (2026-07-19):**
  - **Confirmed FTC process boundary:** the pinned FTC SDK 11.1.0 explicitly supports process-local
    state between OpModes. `OpMode` declares a public static `HashMap<String, Object> blackboard`,
    and the repository's official disabled `ConceptBlackboard` sample names Auto-to-TeleOp as a use
    case. The sample also documents the exact hazards MATCH-01 must remove: callers choose string
    keys and casts, values persist when an OpMode is re-entered, and all data disappears on a robot
    reset, app restart, or code download. The SDK does not clear the map automatically.
    `OpModeManagerImpl` stops the old user OpMode before swapping to and initializing the new one,
    and ordinary annotated OpModes are newly constructed. A user `stop()` failure can still be
    recorded while the requested next OpMode is initialized, so publishing uncertain state before
    cleanup is not safe. There is no typed Pedro handoff facility or other SDK owner with freshness,
    one-shot consumption, and lifecycle policy.
  - **Current Phoenix callers:** the concrete match producers are
    `PhoenixRedAudienceSafeAuto`, `PhoenixBlueAudienceSafeAuto`, and
    `PhoenixPedroAutoSelectorOpMode`; all share `PhoenixPedroAutoOpModeBase`.
    `PhoenixPedroAutoTestOpMode` is an explicitly non-match purpose and must clear but never publish.
    `PhoenixBasicPedroAutoExample` and `PhoenixTestersOpMode` are diagnostic/example entries that
    must invalidate a pending Phoenix match handoff. `PhoenixTeleOp` is the one consumer.
    The selector overrides `init()` and currently does not call the base implementation, so the
    common clear cannot be placed in the base without also making that lifecycle delegation
    explicit. Repository search found no existing modern Phoenix/framework carrier, preference,
    file, blackboard, or mutable public pose-storage path.
  - **Current pose and ownership seam:** Auto's retained `PedroPathingRuntime.motionPredictor()`
    exposes the same backend-neutral predictor that Phoenix localization and Pedro's passive
    localizer share. Its cached `PoseEstimate` is already in the Phoenix field frame; no Pedro
    `Pose`, follower read, or vendor conversion is needed. TeleOp constructs its localization owner
    during `PhoenixRobot.initTeleOp()`, after which the robot can apply an accepted planar
    `Pose2d` through the existing `CorrectedPoseEstimator.setPose(...)` seam before FTC START or the
    first loop. That seam rebases the owned predictor as part of localization ownership. The
    handoff must not expose the lane or resetter from `PhoenixRobot`.
  - **Payload audit:** Phoenix currently has one fact that the next mode actually consumes: the
    final valid, finite Phoenix field-to-robot planar pose. Alliance and checked-in calibration
    flags are available during Auto, but TeleOp has no current consumer and those flags are
    configuration rather than live calibration observations. The robot-specific payload will
    therefore be one deeply immutable data-only snapshot containing only `Pose2d`. It will not
    carry Tasks, hardware, services, queues, capabilities, route status, strategy, vendor types,
    `LoopClock`, or a `PoseEstimate` timestamp whose epoch belongs to the old OpMode.
  - **Alternatives and simplicity comparison:**

    | Approach | Ordinary robot code | Safety/lifecycle result | Decision |
    |---|---:|---|---|
    | Documentation plus a public robot static field | Short initially, but every team writes read/null/clear rules | No type token, freshness, atomic consume, or shared failure policy | Reject |
    | Phoenix-only private holder | Smallest one-robot change | Correctable, but repeats the same one-shot age/type/concurrency mechanism already needed by several reviewed teams | Keep only as a thin robot wrapper |
    | FTC `OpMode.blackboard` | Raw `put/get/remove` plus keys and casts | Public mutable map, key collisions, Blocks access, no synchronization, freshness, ownership, or automatic clear | Use as lifecycle evidence, not storage |
    | Typed process-local carrier plus robot wrapper | One private channel declaration; lifecycle calls live in the shared robot base/wrapper | Centralizes type, age, consume-once, clear, concurrency, and diagnostics while payload/policy remain robot-owned | Choose |
    | Staged exact producer/consumer class lists | Adds publisher/consumer class lists and a stage for every new OpMode | Excludes named tests, but cannot prove Auto and TeleOp were adjacent; existing robot purpose and private wrapper already own eligibility | Reject |
    | Lifecycle listener that observes every intervening OpMode | Adds manager/activity registration and listener cleanup | Can detect adjacency but couples a tiny value handoff to internal FTC lifecycle plumbing | Reject |
    | Preferences, files, or serialization | Adds schema/I/O/recovery work | Persists stale match state across the wrong lifetime | Reject |
    | Global registry, string map, or event bus | Adds a second robot-state architecture | Broad unchecked ownership for a one-slot use case | Reject |

  - **Chosen framework API:** add one stateful
    `edu.ftcphoenix.fw.ftc.FtcAutoToTeleOpHandoff<T>` with exactly one public construction path:

    ```java
    FtcAutoToTeleOpHandoff<MatchSnapshot> handoff =
            FtcAutoToTeleOpHandoff.create(
            "Phoenix Auto to TeleOp",
            MatchSnapshot.class,
            60.0
    );
    ```

    The constructor is private. There is no plural factory facade, `of(...)`, builder, overload,
    registry, string key, no-age path, `peek()`, or persistent backing. The three factory answers
    have distinct value: a diagnostic channel name, a runtime payload type token, and a finite
    positive maximum process-monotonic age in seconds. Its only state operations are
    `clear()`, `publishFromAuto(OpMode producer, T payload)`, and
    `consumeForTeleOp(OpMode consumer)`. Producer and consumer objects are used only to record
    concrete class diagnostics and are never retained or treated as authentication.
  - **Consume result and state contract:** `consumeForTeleOp(...)` returns one privately
    constructed, immutable `ConsumeResult<T>` with a status of `DELIVERED`, `MISSING`, `STALE`, or
    `ALREADY_CONSUMED`, the payload only for `DELIVERED`, and bounded producer/age diagnostics when
    they exist. The carrier is a single linearizable slot backed by an isolated atomic state, not
    the SDK blackboard. `clear()` idempotently opens a fresh empty cycle. One valid publish is
    accepted only into that cleared cycle; null, wrong raw type, duplicate, or late publication
    fails before replacing state. The first consume atomically closes the cycle even when it finds
    missing or stale data, so a concurrent late publisher cannot leave data for a later TeleOp.
    Exactly one concurrent consumer can receive a value; later consumers see
    `ALREADY_CONSUMED`. Stale data is discarded. A package-private injected nanosecond clock makes
    the exact-age and regressing-clock behavior deterministic in tests; production uses
    `System.nanoTime()`, never either OpMode's `LoopClock` or wall time.
  - **Immutability boundary:** the generic carrier can validate the declared runtime class but
    cannot deep-copy arbitrary `T`; its Javadocs require an immutable payload and must not claim
    otherwise. Phoenix satisfies that contract with its private immutable match snapshot and the
    already immutable `Pose2d`. This keeps the generic API honest while preventing a public
    Phoenix snapshot/config noun that no caller needs to construct.
  - **Chosen Phoenix wrapper:** one robot-named `PhoenixMatchHandoff` privately owns the static final
    generic channel and the 60-second robot policy. It exposes lifecycle-named operations to the
    Phoenix OpModes but never exposes the channel or payload. It validates `PoseEstimate.hasPose`
    and finite x/y/heading before creating the snapshot, translates the generic consume result into
    a small Phoenix restore status, and delegates an accepted pose to a package-private
    `PhoenixRobot` initialization seam. The carrier itself does not authenticate match roles:
    `PhoenixPedroAutoOpModeBase` remains the owner of the existing `MATCH_AUTO` versus
    `PEDRO_INTEGRATION_TEST` policy. This is simpler and more truthful than maintaining exact
    concrete-class allowlists that still cannot prove mode adjacency.
  - **Auto lifecycle policy:** clear the channel at the beginning of every Phoenix Pedro Auto
    `init()`. The selector will call `super.init()` first; its `buildRobotInInit() == false` keeps
    construction deferred while sharing the clear. Record a separate successfully-started flag
    only after both Phoenix start calls return; the existing `startBoundaryReached` flag is too
    early. At `stop()`, a successfully started match Auto captures one immutable candidate from the
    predictor's already-cached estimate before owners are cleared. It performs no new hardware
    update after the SDK's failsafe. It then attempts normal robot cleanup and clears its owner
    references. Only when capture and cleanup both succeed does it publish, as the final operation.
    A blocked/init-only/test Auto, start failure, invalid pose, capture failure, or cleanup failure
    leaves the channel empty; failures retain their primary/suppressed ordering. Publication before
    cleanup is explicitly rejected because a failed Auto could otherwise leak a snapshot into the
    requested TeleOp.
  - **TeleOp and diagnostic lifecycle policy:** `PhoenixTeleOp.init()` first constructs and
    initializes its ordinary localization graph, then performs exactly one robot-wrapper restore
    before START. `DELIVERED` applies the pose; `MISSING`, `STALE`, and `ALREADY_CONSUMED` visibly
    keep the normally initialized TeleOp localizer pose. This is the explicit fallback. Do not add
    a zero-pose sentinel or new profile field because no input proves a different standalone
    TeleOp field pose. If application fails after initialization, the OpMode stops the constructed
    robot and preserves the failure. The Phoenix tester host and disabled basic Pedro example clear
    pending state during INIT so they cannot sit between a match Auto and TeleOp unnoticed.
    Process/classloader restart naturally produces `MISSING`; no recovery from disk is attempted.
  - **Student-facing and total robot-code effect:** the generic mechanism is common framework code,
    while Phoenix still contains the robot facts and lifecycle policy. This does not pretend all
    robot code becomes one line: the shared Auto base owns a clear and a publish boundary,
    `PhoenixTeleOp` owns one restore boundary, and the two diagnostic hosts own invalidation.
    Students adding a normal Phoenix match Auto leaf still write only its existing `autoSpec()`
    because the repeated lifecycle is centralized in the base. A different robot opts in by
    defining one immutable snapshot and one private wrapper; a robot with no handoff adds nothing.
  - **Framework Principles result:** the design keeps FTC process behavior at `fw.ftc`, payload and
    role policy in the robot, pose realization in the localization-owning composition root, and
    vendor/strategy types outside the carrier. It provides one public construction layer and three
    parallel state operations, fails fast on invalid configuration/misuse, preserves explicit
    lifecycle calls and fallback, and does not infer physical pose accuracy or mode adjacency from
    a class name. No Framework Principles or framework-improvement skill change is needed: their
    existing ownership, one-construction-path, parameter-value, fail-fast, and evidence rules
    directly selected this design.
  - **Bounded implementation scope:** add only the generic carrier/result, the Phoenix private
    snapshot/wrapper, the package-private robot pose-restore seam, the shared Auto/selector,
    TeleOp, tester, and basic-example lifecycle integration, and directly affected documentation
    and tests. Do not add persistent storage, SDK-blackboard wrapping, exact OpMode allowlists,
    lifecycle listeners, a global state service, multiple payloads, alliance/calibration transfer,
    public localization access, route changes, or another tracker item.
  - **Software-only verification plan:** pure carrier tests cover factory validation; correct and
    raw wrong-type publication; duplicate/late publish; explicit clear; exact freshness boundary;
    stale, missing, already-consumed, regressing monotonic time, and process-local semantics;
    linearizable publish/consume/clear races and one receiving consumer per cleared cycle; and
    producer/consumer objects not being retained. Phoenix tests cover immutable finite-pose
    validation, all three production Auto
    paths, test/example/tester invalidation, selector base-init delegation, blocked/no-start/failed-
    start behavior, capture-before-cleanup and publish-after-cleanup ordering, primary/suppressed
    cleanup failures, repeated Auto/TeleOp sequences, single TeleOp restore before first update,
    existing-pose fallback, and failed-restore cleanup. Re-run the full TeamCode unit suite and FTC
    Java compile; audit the public surface, blackboard absence, vendor-type boundary, callers,
    Javadocs, the FTC-boundary guide, mode-client guide, and Phoenix Architecture.
    No robot hardware is required to verify this process/lifecycle contract. Tests can prove the
    existing pose-reset seam receives the accepted value; they must not claim that Auto's physical
    pose estimate was accurate or that a physical TeleOp drivetrain starts at that field pose.
  - **Approval gate:** the leading typed, one-shot, process-local hypothesis remains viable, with
    two reviewed refinements: use an isolated atomic slot instead of the SDK's raw blackboard, and
    keep ordinary TeleOp localization as the explicit missing/stale fallback instead of inventing a
    configured pose. Because the factory, consume-result statuses, publish-last Auto ordering, and
    Phoenix restore lifecycle are major API/behavior decisions, implementation must not begin until
    the user approves this MATCH-01 design.
  - **Public-scope refinement and approval (2026-07-19):** when the user asked whether the carrier
    had uses beyond Auto-to-TeleOp, the caller audit found no second proven lifecycle. Mechanism
    offsets, inventory, alliance, or detected game facts are possible **payloads** for the same
    match boundary only when the next TeleOp actually consumes them; durable calibration belongs in
    configuration/persistence, and same-OpMode state belongs in the robot runtime. The approved
    public type is therefore narrowed from the broader draft `FtcModeHandoff<T>` to
    `FtcAutoToTeleOpHandoff<T>`, with role-named publish/consume operations. This changes no state,
    freshness, Phoenix-wrapper, or verification decision and avoids advertising a general global
    mode-state mechanism. The user then replied `Approve MATCH-01 design`, authorizing Gate 2
    implementation of this narrowed design and this item only.
  - **Implementation (2026-07-20):**
    - Added the factory-only `FtcAutoToTeleOpHandoff<T>` with one isolated atomic state, explicit
      `clear()`, role-named publish/consume operations, runtime type validation, process-monotonic
      freshness, immutable `ConsumeResult<T>`, and `DELIVERED`, `MISSING`, `STALE`, and
      `ALREADY_CONSUMED` statuses. Primitive type tokens now fail during construction rather than
      creating a carrier that can never accept a boxed value.
    - Added `PhoenixMatchHandoff`, whose private immutable snapshot contains only a finite planar
      Phoenix field pose. Its private channel and 60-second policy remain behind Phoenix lifecycle
      names; no Pedro, hardware, Task, service, strategy, or localization-owner type crosses the
      boundary.
    - The shared Pedro Auto base clears during INIT, retains the exact runtime motion-predictor
      reference, marks success only after both START calls, captures its cached immutable pose before
      shutdown, and publishes only after successful shutdown. Blocked, test-purpose, never-started,
      failed-start, failed-capture, invalid-pose, and failed-cleanup runs leave the channel empty;
      repeated/reentrant stop cannot erase a successful publication. The selector now explicitly
      delegates base INIT, while the Phoenix tester and disabled basic example invalidate match
      state.
    - `PhoenixTeleOp` constructs the ordinary localization graph, performs the one-shot restore
      before START, and fail-stops an initialized robot if application fails. Normal controls help,
      pose-assist readiness, and the handoff result/fallback are staged into one committed INIT
      telemetry frame so FTC auto-clear cannot hide either set of guidance.
    - Added the FTC-boundary handoff guide and synchronized the docs hub, FTC-boundary index,
      mode-client guide, Phoenix Architecture, Javadocs, and examples with the exact process,
      ownership, fallback, failure-ordering, and physical-accuracy limits.
  - **Adversarial review and corrections (2026-07-20):** independent core/concurrency,
    Phoenix-lifecycle, and documentation/API reviews found and resolved four material edges before
    this gate. Construction now rejects primitive type tokens; `ConsumeStatus` and the tracker's
    wrong-type wording are synchronized; deterministic clear/publish/consume regressions prove an
    old atomic identity cannot cross a cleared cycle and a retried publication receives fresh time;
    and TeleOp now commits one combined INIT frame instead of replacing readiness help with a second
    auto-cleared frame. The guide points to the complete Phoenix failure-ordering reference.
    Final independent re-reviews found no remaining core, lifecycle, documentation, scope, or
    student-facing API issue. A large fake hardware-runtime abstraction was deliberately not added
    solely to invoke successful `PhoenixTeleOp.init()` on the host; the narrow restore,
    presentation, and failure seams plus production call-order audit cover that boundary without
    enlarging robot code.
  - **Automated verification (2026-07-20):**
    - The six focused MATCH-01 suites pass **47/47**: `FtcAutoToTeleOpHandoffTest` 18,
      `PhoenixMatchHandoffTest` 9, `PhoenixPedroAutoOpModeBaseTest` 14, `PhoenixTeleOpTest` 2,
      `PhoenixBasicPedroAutoExampleTest` 3, and `PhoenixTestersOpModeTest` 1, with zero failures,
      errors, or skips.
    - `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` passes: **63 suites,
      628 tests**, zero failures, errors, or skips. Output contains only the repository's existing
      Java 21/source-target 8 deprecation warning.
    - `git diff --check`, all-changed-file trailing-whitespace checks, and changed-Markdown
      relative-link checks pass. The bounded scope is 20 files: eight production Java files, six
      test files, five guides/indexes, and this tracker. Compiled public-surface inspection confirms
      the generic carrier has one public factory plus only `clear`, `publishFromAuto`, and
      `consumeForTeleOp`; the Phoenix wrapper exposes only its three lifecycle operations. Static
      audits found no SDK-blackboard access, persistent/global registry, vendor type at the generic
      boundary, new sleep/busy wait, or retained producer/consumer `OpMode` field.
  - **Gate 2 verification stop (2026-07-20):** MATCH-01 is **Verifying** and paused for Android
    Studio review. Inspect the generic carrier/result, Phoenix wrapper, Auto stop ordering, selector
    base-init delegation, TeleOp pre-START restore/fallback and combined INIT telemetry frame,
    diagnostic invalidation, focused tests, and the new FTC-boundary guide. No robot hardware is
    required to verify the software process/lifecycle contract. The tests prove that an accepted
    pose reaches the existing reset seam; they do not claim that Auto's physical pose estimate was
    accurate on a real field. No later tracker item was started.
  - **Manual verification (2026-07-20):** the user reviewed MATCH-01 in Android Studio and replied
    `MATCH-01 looks good`. MATCH-01 is **Done**. This approval authorizes publication and merge of
    this item; no robot-hardware validation was reported or required for its process/lifecycle
    contract. The user's separate `Next task` instruction authorizes beginning the next eligible
    decision gate only after this reviewed change is merged.

### DRIVE-02 - Shared drivetrain actuator handoff

- **Problem to confirm:** some competitive robots mechanically reuse drivetrain motors and encoders
  through a PTO for a lift or endgame mechanism. `FtcMecanumDriveLane` deliberately hides its four
  motor writers/readbacks, so a Phoenix robot currently must either bypass the lane, create a second
  competing owner, or make a mechanism pretend to be a `DriveSource`. None preserves one obvious
  final actuator owner.
- **External evidence:** Cuttlefish's
  [PTO transition](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/RobotActions.java)
  resets drivetrain motor modes/encoders and engages the PTO, while its
  [Endgame](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Endgame.java)
  reads the shared encoders and writes lift output through the drivetrain owner. TeleOp explicitly
  suppresses normal driving during the handoff.
- **Alternatives to compare:** expose raw motors from the standard lane; build separate drive and lift
  Plants over the same motors; model lift as a drive signal; create a robot-specific shared hardware
  owner; add an explicit multi-mode extension seam to the standard lane; or add generic resource
  arbitration. Include motor run-mode/encoder reset, servo PWM, follower stop, and failed-transition
  behavior in the comparison.
- **Leading hypothesis:** one robot-owned FTC realization should retain exclusive ownership of every
  shared motor and encoder and expose explicit mutually exclusive `DRIVE` and `ENDGAME` capability
  modes. The transition stops Pedro before changing hardware modes and fails closed before enabling
  the new writer. Extract a narrow reusable lane/seam only if caller analysis shows it keeps the
  ordinary four-motor drive path just as simple; do not add raw-motor getters or generic Task
  resource priorities.
- **Completion:** tests/fakes prove exactly one writer in each mode; no same-cycle stale drive after
  PTO engagement; ordered follower stop, zero output, motor-mode/encoder transition, and PTO actuation;
  safe rollback on failure; repeated/idempotent transitions; explicit drive reinitialization; useful
  readback for the endgame Plant; and unchanged student code for robots without shared actuators.
- **Decision gate (2026-07-20; implementation deferral approved by the user):**
  - **Confirmed current behavior:** the ordinary FTC path constructs one
    `FtcMecanumDriveLane(hardwareMap, profile.drive)`. The lane privately retains a
    `MecanumDrivebase`; every drive command immediately fans out through the four raw-power outputs,
    and `stop()` zeros them but does not latch the lane disabled. Phoenix TeleOp then unconditionally
    performs its final `drive.update(clock)` and `drive.drive(command)` after controls and mechanism
    policy. A PTO transition that only calls `stop()` earlier in that cycle can therefore be followed
    by a stale final drive write that reacquires `RUN_WITHOUT_ENCODER`.
  - **Pedro trace:** production Pedro constructs its own native mecanum writer inside
    `PedroPathingRuntime`. `PhoenixRobot.updateAuto()` calls the retained adapter heartbeat before the
    Auto Task root on every loop. `PedroPathingDriveAdapter.stop()` correctly interrupts the current
    route and applies immediate/final zero, including callback reentry, but a later heartbeat is still
    a vendor drivetrain lifecycle call. A shared owner must fence both future heartbeats and route
    starts before changing motor ownership; stopping the adapter once is not arbitration.
  - **Reachable competition evidence:** the previously audited Decode repository is no longer
    publicly reachable at its pinned commit. Its public
    [`summer-2026` fidelity port](https://github.com/6165-MSET-Cuttlefish/summer-2026/tree/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd)
    preserves the relevant shape:
    [`RobotActions`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode/RobotActions.java#L178-L205)
    resets the front drive encoders, engages the PTO, waits, and enables full lift;
    [`Endgame`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Endgame.java#L217-L241)
    reads those shared encoders and submits paired lift targets to the
    [`Drivetrain`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Drivetrain.java#L109-L164),
    which remains the physical motor writer; and
    [`Tele`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tele/Tele.java#L112-L123)
    suppresses ordinary drive while the PTO/full lift is active. This proves the use case, but not
    one universal motor subset, sign convention, encoder-reset policy, servo/PWM sequence, settling
    time, reversibility, or physical rollback contract.
  - **What not to copy from the competition example:** Cuttlefish's root order is
    [`readModules -> follower.update -> gameLoop -> Actions.update -> writeModules`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/core/EnhancedOpMode.java#L213-L253).
    Suppressing the later manual `drive()` call does not clear the drivetrain's retained targets, so
    the preceding command can still be written during the 300 ms PTO-settle interval until full-lift
    control replaces it. That is a source-derived risk, not evidence of a field failure. The example
    also has no explicit physical zero/Pedro stop before reset and no transactional fail-closed state.
    Phoenix should support the capability without copying those lifecycle gaps.
  - **Current callers:** no production, Phoenix, modern example, or tool runtime shares drivetrain
    motors with another mechanism. Phoenix has no endgame/PTO profile, subsystem, capability, or
    control. `DrivetrainMotorDirectionTester` owns four separate test Plants in an isolated tester and
    is not a handoff caller.
  - **Supported public construction layers:**
    - `FtcMecanumDriveLane(HardwareMap, Config)` is the one recommended robot-facing lane and owns
      standard wiring, brake setup, drivebase lifecycle, and debug delegation.
    - `FtcDrives.mecanum(...)` is the lower-level FTC factory. Its overloads create a private
      coordinated four-output group that preflights every raw-power mode before requested motion.
      Whether its many beginner-facing overloads should remain is API-05, not DRIVE-02.
    - `new MecanumDrivebase(PowerOutput, PowerOutput, PowerOutput, PowerOutput, Config)` is the
      existing advanced injection seam. It deliberately leaves custom multi-output coordination and
      lifecycle to its caller.
    - Public `FtcHardware.motorPower(...)` creates one raw-power adapter; the coordinated group
      factory is package-private. Combining four independent public adapters does not preserve the
      standard factory's all-motor preflight guarantee.
    - `FtcActuators.plant(...)` can intentionally reuse configured motor names under one higher-level
      robot lifecycle owner, but it does not arbitrate a drive lane and a mechanism Plant.
      `PositionPlant.establishReferenceAt(...)` can establish a lift coordinate without resetting a
      hardware encoder, so a framework PTO API must not assume `STOP_AND_RESET_ENCODER`.
  - **Student-facing comparison (each bracket is a new conceptual answer):**

    | Approach | Ordinary/PTO call-site shape | Concepts and ownership result |
    |---|---|---|
    | No framework change for ordinary robots | `new FtcMecanumDriveLane(hardwareMap, profile.drive)` `[hardware registry] [drive config]` | Current one-line path remains clear. |
    | Raw lane getters | `drive.motor(FRONT_LEFT).setMode(...)` `[wheel identity] [SDK mode] [reset order] [direction] [writer suppression]` | Short syntax exposes the hardest lifecycle decisions and breaks lane encapsulation. |
    | Separate drive and lift Plants | Build both over the same names `[two owners] [activation order]` | Looks familiar but permits the later drive/heartbeat writer to steal the motors back. |
    | Lift as a `DriveSource` | Encode lift power as axial/lateral/omega `[fake drive meaning]` | Hides contention by lying about the command domain and loses Plant range/reference/readiness. |
    | New shared-hardware handle/seam | `shared = FtcDrives.sharedMecanum(...)`; then build drive and lift views `[handle] [views] [mode API] [transition policy]` | Adds a second construction layer but still cannot own robot-specific PTO mechanics unless it grows into the robot owner. |
    | Robot-owned shared realization | One `SharedDriveEndgame` owns both private realizations; TeleOp/Auto use `mobility()` and `endgame()` capabilities `[robot topology] [explicit mode intent]` | Preserves one final writer and keeps strategy out of framework, but the realization remains meaningful robot code rather than complexity that can honestly disappear. |
    | Generic claims/priorities | `resource.claim(owner, priority)` `[resource] [claim] [lease] [priority] [transition callback]` | Adds a resource scheduler yet still cannot define the physical PTO sequence or Pedro callback boundary. |

  - **Framework Principles result:** the leading ownership direction is still correct: one
    robot-owned FTC realization should own every shared motor, encoder, PTO actuator, mode transition,
    and final write branch. Its internal lifecycle should use explicit `DRIVE`, transition,
    `ENDGAME`, and fail-closed states; capabilities expose mode-neutral intent. It must gate the
    TeleOp final sink or Pedro heartbeat/route seam before stopping and zeroing hardware, establish a
    software encoder reference unless the real mechanism proves a hardware reset is required, actuate
    and settle the PTO non-blockingly, and enable only the selected writer. A failed prerequisite
    leaves both behavior writers disabled and output zero; it must not claim rollback to drive unless
    every physical restore step succeeds.
  - **Why implementation should not start now:** Phoenix has no adopting PTO robot, and representative
    hardware is unavailable. The exact shared motors, motor directions, drive/follower owner, encoder
    reference policy, PTO servo/PWM behavior, settle phases, safe failure state, and return-to-drive
    behavior are all part of the production contract. Inventing them would manufacture season
    hardware inside Phoenix. A partial injected handle, raw getters, or generic arbitration would add
    student-facing concepts without closing those correctness gaps.
  - **Approved disposition:** **Deferred**. The user agreed that Phoenix should not solve a
    specialized hardware problem whose correct solution may change with the robot's construction.
    Resume when a real adopting robot and its safe transition specification are available. Implement
    the first complete owner in robot code; extract only a narrow correctness-critical or repeated
    framework seam after caller evidence identifies one, while preserving the ordinary one-line lane
    path.
  - **Future verification plan:** fake-owner tests must prove the drive/route/heartbeat fence occurs
    before hardware transition, exactly one writer is enabled, no stale same-cycle command escapes,
    transitions and cancellation are idempotent, and any failure remains zero/disabled. FTC fake
    tests must prove all-motor zero and mode preflight ordering, reference/reset policy, and explicit
    drive reinitialization. Task tests must prove non-blocking PTO settling and safe cancellation.
    Pedro tests must prove no heartbeat or route start reaches the follower in endgame mode. Compile
    the adopting robot and preserve the ordinary lane caller. Physical validation must still verify
    PTO engagement, servo/PWM behavior, encoder signs, settling, load behavior, and the actual
    fail-safe state; fake tests cannot establish those facts.

### VISION-01 - Shared webcam and Limelight vision ownership

- **Problem to confirm:** Phoenix's reusable FTC vision owners are AprilTag-specific. A robot that
  adds a color/cluster/game-piece `VisionProcessor` must repeat webcam/portal construction, processor
  enablement, streaming state, failure reporting, and close/retry behavior or incorrectly force
  robot-specific results through `AprilTagVisionLane`. A Limelight robot has a parallel ownership
  problem, but not the same processing model: one physical device may serve object, motif, and
  localization pipelines, and a requested pipeline is not immediately safe to interpret.
- **External evidence:**
  - The original private Cuttlefish `Decode` link used in the first audit is no longer accessible.
    Its public
    [`WebcamSession`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpenCVPipelines/WebcamSession.java#L28-L64)
    fidelity-port equivalent owns lookup, asynchronous open, pipeline/stream start, Dashboard
    streaming, update, and close around a robot-owned EasyOpenCV algorithm. This corroborates the
    reusable webcam-lifecycle problem, but it does not by itself prove a VisionPortal-specific API.
  - Tech Tigers' Worlds
    [`LimelightSubsystem`](https://github.com/techtigers-ftc/decode/blob/0bec90bb2523368c122d8d8361b56261d85cdfe2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/LimelightSubsystem.java#L72-L165)
    shares one Limelight between artifact and AprilTag work. It reads the latest result, requests the
    pipeline implied by the new semantic state, and then interprets that pre-switch result without
    checking its pipeline index or age. That is concrete evidence for framework-owned
    requested-versus-confirmed pipeline state and stale-result protection; its artifact/motif
    vocabulary remains robot-owned.
  - FTC documents VisionPortal processor enablement, camera streaming, and final close as distinct
    resource levels. Limelight's official FTC guide documents `pipelineSwitch(...)` as
    fire-and-forget and directs callers to confirm `LLResult.getPipelineIndex()`; results also expose
    staleness. Limelight and HuskyLens do not use VisionPortal.
- **Alternatives to compare:** keep every custom device owner robot-owned; generalize
  `AprilTagVisionLane`; expose raw `VisionPortal`/`Limelight3A` from factories; add only a webcam
  portal owner; create one generic cross-backend owner; create parallel backend-specific owners; or
  define generic detection/result maps. Confirm whether existing `FtcVision` already removes enough
  ceremony and whether a common status interface/value has a real caller before adding it.
- **Original leading hypothesis:** if at least two real callers share the lifecycle, add one small
  `fw.ftc` resource owner for camera/processor construction, enable/disable state, readiness
  diagnostics, retry, and close. Algorithms and immutable timestamped result snapshots stay
  robot-owned and typed; framework routines never depend on `VisionProcessor`, OpenCV, or an
  arbitrary generic detection map.
- **Completion:** focused fake/FTC-boundary tests cover both owners' configuration and partial
  construction, readiness diagnostics, close after failure, repeated close, and fresh reconstruction.
  Webcam tests cover registered-processor enablement and stream state. Limelight tests cover polling
  order, rejected pipeline requests, disconnected/fake results, requested-versus-observed pipeline,
  post-switch generation, switch-away/switch-back, freshness, same-cycle sampling, and the
  distinction between an operational pipeline and “no target.” Existing AprilTag lanes use the same
  physical owner rather than acquiring a competing camera; ordinary Phoenix construction remains
  unchanged. A compiling custom-vision example keeps algorithms and immutable typed snapshots
  robot-owned, and no SDK/vendor type leaks into core Tasks or robot strategy.
- **Decision record (2026-07-20): Ready for major API approval; no implementation code changed.**
  - **Confirmed current behavior and callers:**
    - `FtcVision.aprilTags(...)` constructs and hides one `VisionPortal` plus one
      `AprilTagProcessor`. `FtcWebcamAprilTagVisionLane` can close the returned sensor but cannot
      attach a custom processor, control an attached processor, or report the portal's asynchronous
      camera state. A robot that opens another portal risks competing ownership of the same webcam.
    - `FtcLimelightAprilTagVisionLane` gets one `Limelight3A`, sets the poll rate, requests a
      pipeline, starts polling, and ignores the boolean pipeline-switch result. Its sensor accepts
      fiducials without confirming running/connected state, the requested pipeline, or a post-switch
      frame, and the lane exposes the mutable raw device as an escape hatch.
    - FTC SDK 11.1 `Limelight3A.getLatestResult()` synthesizes an empty result when no real result
      has arrived. That synthetic object is timestamped at creation and defaults to pipeline zero, so
      non-nullness, apparent freshness, or pipeline index alone can falsely make a disconnected
      pipeline-zero camera look ready. Readiness must require running and connected state too.
    - VisionPortal can run several attached processors concurrently and toggles each processor for a
      subsequent frame. Limelight runs one onboard pipeline at a time; a successful
      `pipelineSwitch(...)` request is not proof that subsequent reads belong to it. These are
      different mechanisms, not two implementations of one enable operation.
    - Phoenix, localization testers, and calibration tools already consume the backend-neutral
      `AprilTagVisionLane`; selectable testers already own retry correctly by closing a confirmed
      lane and factory-opening a fresh one. That retry policy should not move inside a camera owner.
  - **Framework Principles and student-facing comparison:**

    | Approach | Simplicity and principle result |
    |---|---|
    | Keep custom ownership in every robot | No new framework type, but repeats correctness-sensitive lifecycle and leaves the Tech Tigers stale-pipeline hazard for each student team to rediscover. Rejected. |
    | Generalize `AprilTagVisionLane` into `VisionLane<T>` or a generic detection map | Makes unrelated processor/pipeline/result concepts appear interchangeable, leaks vendor or untyped result vocabulary upward, and makes the ordinary path harder to learn. Rejected. |
    | Return a raw `VisionPortal` or `Limelight3A` from a factory | Saves construction lines but does not establish one owner, readiness, generation/freshness, or safe close. It also preserves competing imperative access. Rejected. |
    | Add only a VisionPortal owner | Solves the webcam symptom but does not support the clarified next-season Limelight requirement or one-device multi-pipeline use. Rejected. |
    | One cross-backend device owner | Would need conditional APIs for simultaneous processor enablement versus exclusive pipeline selection and would create false parallelism. Rejected. |
    | Parallel concrete owners with one narrow readiness value | Keeps vendor mechanics at the FTC boundary, preserves one physical owner, keeps ordinary AprilTag construction stable, and lets robot code expose one small typed semantic capability. Chosen. |

  - **Chosen public shape:**
    1. Add `FtcWebcamVisionPortalLane`, a concrete FTC-boundary owner constructed with the complete
       set of fresh `VisionProcessor` instances. It owns webcam lookup, portal construction,
       registered-processor enable/disable, stream state, camera-control access, diagnostics, and
       idempotent close. It does not expose the raw portal and cannot add processors after build.
    2. Add `FtcLimelightVisionLane`, a separate concrete FTC-boundary owner for hardware lookup,
       poll/start/stop order, requested pipeline, request success/failure, cycle-stable result
       sampling, requested-versus-observed pipeline, post-request generation, freshness,
       diagnostics, and idempotent close. It exposes narrow operations needed by supported
       integrations, such as robot-orientation input, rather than the raw mutable `Limelight3A`.
    3. Add one small immutable `VisionReadiness` value used by `AprilTagVisionLane` and the two
       concrete owners. It reports component readiness and an actionable reason; it is not a
       generic detection/result interface and does not claim that a target is visible.
       `AprilTagVisionLane` gains a readiness query so Phoenix and the existing backend-selectable
       testers have a real common caller.
    4. Make `FtcWebcamAprilTagVisionLane` and `FtcLimelightAprilTagVisionLane` specialized forms of
       their corresponding concrete owner, with one underlying device and one idempotent close.
       The standard public construction remains:

       ```java
       AprilTagVisionLane vision =
               PhoenixVisionFactory.create(hardwareMap, profile.vision);
       ```

       A robot that needs AprilTags plus a custom processor/pipeline retains that one specialized
       owner and exposes only its own immutable typed vision snapshot to strategy. It does not open a
       second camera owner.
    5. Keep backend realization explicit inside the robot's vision boundary. A robot-owned
       `MyVision` capability/factory maps semantic modes such as `GAME_PIECES` and `APRIL_TAGS` to
       either processor enablement or a Limelight pipeline request. The rest of Auto/TeleOp sees
       only typed immutable `MyVisionSnapshot` data; it does not see `VisionProcessor`, `LLResult`,
       OpenCV, or numeric pipeline indices.
    6. Treat retry as close plus fresh construction. A closed VisionPortal cannot reopen, and FTC
       SDK processor identities are single-attachment objects, so retry must create fresh processors
       and a fresh owner. Do not add a mutable `retry()` method or ownership flags.
    7. Remove the overlapping `Tags.aprilTags(...)` / `FtcVision.aprilTags(...)` public construction
       path and migrate its two examples to the explicit AprilTag lane owner. Keep only internal
       conversion/build support needed by the lanes. The deferred `AprilTagVisionLaneFactory`
       remains because “open a fresh configured lane later” is a distinct tester capability, not a
       second spelling of ordinary construction.
  - **Readiness and freshness contract:**
    - Webcam device readiness means the portal is streaming and the required processor is enabled;
      target availability remains the processor's result concern. Because `VisionProcessor` has no
      standard result API, each robot-owned processor/adapter must publish an immutable timestamped
      snapshot and invalidate or reject stale output after disable/re-enable. The framework will not
      invent a generic result map to guess that policy.
    - Limelight readiness means polling is running, the device is connected, the pipeline request
      was accepted, and a result newer than that request confirms the requested pipeline within the
      configured freshness bound. `LLResult.isValid()` means a target is available; it is not
      pipeline/session readiness. A generation/baseline check is required so switching away and back
      cannot revive a cached result merely because its numeric pipeline index matches again.
    - Limelight's SDK pipeline request is a synchronous HTTP operation even though its effect is
      asynchronous. Call it only once on an actual semantic transition and never in an every-loop
      setter. Do not add an unverified background thread or promise non-blocking vendor behavior
      without hardware/thread-safety evidence; the software contract instead guarantees no waits,
      retries, or result consumption loops and refuses results until settling is confirmed.
  - **Implementation and verification scope after approval:** add the two owners, readiness value,
    fakeable package-private device seams, specialized AprilTag composition, controlled Limelight
    result/orientation access, focused tests, one custom webcam/Limelight structure example, and
    synchronized Javadocs/vision/lane/Phoenix docs. Update all in-repository callers and delete the
    duplicate construction path in the same change. Use software tests to establish lifecycle,
    ordering, generation, and failure contracts; report that actual camera-open time, pipeline-switch
    latency, image quality, and physical detection performance still require adopting-robot
    validation.
  - **Decision-gate result:** this materially differs from the original single-owner leading
    hypothesis. The smallest correct design is two parallel backend-specific owners, not one owner
    with webcam/Limelight conditionals. It is also a major public ownership/readiness decision, so
    implementation must stop here until the user approves this revised design.
  - **Approval:** the user approved the VISION-01 parallel-owner design on 2026-07-20.
  - **Implementation record (2026-07-20):**
    - Added `FtcWebcamVisionPortalLane` and `FtcLimelightVisionLane` as parallel FTC-boundary
      owners, plus the narrow immutable `VisionReadiness` value. The webcam owner fixes the complete
      processor set at construction and owns processor/stream controls and terminal close. The
      Limelight owner owns polling, pipeline requests, generation-aware same-cycle sampling,
      requested-versus-observed confirmation, freshness, and terminal close without exposing the
      raw device.
    - Rebuilt the webcam and Limelight AprilTag lanes on those physical owners. The specialized
      webcam lane can enable or disable its built-in AprilTag processor without exposing that SDK
      processor, so one robot-owned semantic capability can switch between AprilTags and other
      registered processors. `PhoenixVisionFactory.create(...)` remains the ordinary one-line
      Phoenix construction path.
    - Removed the overlapping `FtcVision.aprilTags(...)` and `Tags.aprilTags(...)` construction
      paths and migrated their callers. Borrowed `AprilTagSensor` views no longer pretend to own or
      close the physical camera.
    - Added a compiling custom-vision example in which Auto/TeleOp consume one robot-owned semantic
      interface and immutable timestamped snapshots while webcam processor toggles and Limelight
      pipeline indices remain inside backend realizations. Transition failures are terminal and
      cleanup preserves the primary failure; replacement is allowed only after close succeeds.
    - Phoenix and the selectable vision/calibration tools now report component readiness separately
      from target visibility, retain actionable causes, close and detach failed owners, permit INIT
      reselection only after confirmed cleanup, and require an OpMode restart after cleanup failure.
      A factory/constructor failure that arrives with a suppressed rollback failure conservatively
      blocks replacement even when no owner reference was published; this keeps the call sites safe
      without adding another public failure-marker API. Phoenix Auto applies the same rule.
    - Both concrete owners distinguish close attempted, close succeeded, and close failed. A thrown
      Limelight pipeline transition terminalizes that owner until its one cleanup attempt, and all
      close/replacement diagnostics remain truthful under repeated or reentrant calls.
    - Corrected the FTC webcam mount boundary from Phoenix robot axes into FTC robot-placement and
      optical-camera axes, removed the redundant webcam SDK pitch-offset setting, and added
      matrix-based regression coverage. Limelight snapshots defensively copy poses and reject the
      SDK's exact all-zero missing-pose sentinel; the field estimator now returns current-loop
      no-pose timestamps instead of non-finite values.
  - **Automated verification (2026-07-20):**
    - Focused owner/readiness/lifecycle suite: 55 tests (`VisionReadinessTest` 3,
      `FtcWebcamVisionPortalLaneTest` 10, `FtcLimelightVisionLaneTest` 12,
      `SelectableVisionTesterLifecycleTest` 15, and `PhoenixPedroAutoOpModeBaseTest` 15), with 0
      failures, 0 errors, and 0 skipped.
    - Full `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac`: 66 suites and 662
      tests, with 0 failures, 0 errors, and 0 skipped; compilation succeeded. Output contained only
      the existing Java 21/source-8 and FTC app-shell deprecation warnings.
    - `git diff --check`, an untracked-file trailing-whitespace scan, and targeted stale-API/retry
      searches passed. Independent adversarial reviews exercised ownership, recovery, freshness,
      frame conversion, API scope, documentation, and test validity; their findings were resolved
      before this audit point.
  - **Android Studio audit point (completed):** the two backend-specific owners, specialized
    AprilTag composition, custom semantic example, Phoenix/tool readiness presentation, and focused
    tests were presented while status was `Verifying`. At that audit point, no VISION-01 file had
    been staged, committed, pushed, or merged. Actual camera open/stream behavior, processor frame
    timing, Limelight pipeline-switch latency, image quality, physical pose accuracy, detections,
    and USB/device close behavior remain adopting-robot hardware validation rather than claims
    established by these software tests.
  - **Manual verification (2026-07-20):** the user reviewed the implementation in Android Studio
    and approved it with `VISION-01 looks good`.

### SENSOR-01 - Motor-current sensing

- **Problem to confirm:** `FtcSensors` exposes battery voltage plus motor position and velocity, but
  not `DcMotorEx` current. Robot code that needs a jam, hard-stop, load, or homing fact must therefore
  retain the raw SDK motor and remember units/sampling behavior, even though the framework guide
  already recommends current spikes as a useful sensor condition.
- **External evidence:** SaMoTech samples intake current in
  [`Intake`](https://github.com/SaMoTechRobotics/FTC-Decode/blob/c8ff047c4245f79d934e622c05481269b5cc42da/TeamCode/src/main/java/robot/intake/Intake.java).
  LOAD uses current both for intake jam recovery in
  [`Commands`](https://github.com/LOAD-Robotics/Decode-Robot-Code/blob/35a74bfba7d5cc9eeede5ecfdd71123869bd4f17/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOADCode/Main_/Hardware_/Commands.java)
  and as a turret-homing safeguard in
  [`Turret`](https://github.com/LOAD-Robotics/Decode-Robot-Code/blob/35a74bfba7d5cc9eeede5ecfdd71123869bd4f17/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOADCode/Main_/Hardware_/Actuators_/Turret.java).
  Tech Tigers observes drive, intake, and shooter load before applying its robot-specific policy in
  [`CurrentManagementCalculator`](https://github.com/techtigers-ftc/decode/blob/0bec90bb2523368c122d8d8361b56261d85cdfe2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/CurrentManagementCalculator.java).
- **Alternatives to compare:** leave direct SDK reads in each robot; add only
  `FtcSensors.motorCurrentAmps(...)`; expose readback from a combined actuator owner; use the SDK's
  motor current-alert facility; add a sampled multi-motor load snapshot; or create a framework power
  manager. Measure current-read cost and behavior under AUTO versus MANUAL bulk caching, and define
  unsupported/unavailable readings before choosing more than one simple source.
- **Leading hypothesis:** add `DcMotorEx` and named-hardware overloads returning a
  cycle-memoized `ScalarSource` in explicit amps, matching the existing position/velocity source
  pattern. Robot code composes it with existing `above(...)`/hysteresis/filter primitives and owns
  the threshold, persistence, jam recovery, calibration, and subsystem priority. A read-only source
  never becomes another motor writer. Do not add a new current interface, universal limit, hidden
  automatic derating, or current-policy methods to `Plant`.
- **Completion:** tests/fakes cover one hardware sample per `LoopClock.cycle()`, repeated consumers,
  explicit amp units, reset, invalid/null configuration, unavailable/non-finite readings, and debug
  output; hardware measurements record polling cost and SDK/controller assumptions. Examples show
  one jam condition and one `PositionCalibrationTasks` stop condition while keeping recovery policy
  in the robot capability/supervisor. Existing robots need no migration.
- **Pause record (2026-07-19):** **Deferred while representative controller measurements are
  unavailable.** Fake-backed tests can prove a narrow current-source API, but the current completion
  contract also requires polling-cost and SDK/controller behavior evidence. A later Gate 1 may ask
  the user to approve a conservative software-seam contract that moves those measurements to
  adopting-robot validation; do not make that reclassification silently.

### INPUT-01 - Safe contextual control activation

- **Problem to confirm:** advanced TeleOp code sometimes changes the meaning of controls by robot
  mode. Simply enabling a new group while a button is held can manufacture a fresh press edge, and
  enabling a continuous manual control while a stick/trigger is displaced can command motion
  without a fresh neutral-to-active gesture. Recreating `Bindings` or scattering mode checks also
  makes ownership and precedence hard to audit.
- **External evidence:** Cuttlefish's
  [`LayeredGamepad`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/input/LayeredGamepad.java)
  primes newly activated controls from their current physical state so held buttons do not appear as
  new presses. Its TeleOp uses this for distinct normal, manual, and endgame mappings. The original
  pinned repository is no longer readable, but Cuttlefish's public
  [`summer-2026` import](https://github.com/6165-MSET-Cuttlefish/summer-2026/commit/1c9bfb2d170f89c009a97b1a059aaccf9aa0fb53)
  identifies itself as the robot-agnostic framework from that exact Decode revision. Its imported
  [`LayerStack`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/1c9bfb2d170f89c009a97b1a059aaccf9aa0fb53/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/input/LayerStack.java)
  selects exactly one keyed layer rather than implementing overlap or priority, while its imported
  [`LayeredGamepad`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/1c9bfb2d170f89c009a97b1a059aaccf9aa0fb53/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/input/LayeredGamepad.java)
  suppresses one refresh, primes Boolean edge baselines, and then accepts held levels and displaced
  analog values. Phoenix should express that accept-current behavior without duplicating an FTC
  gamepad facade, while also offering the stricter neutral-rearm policy for hazardous controls.
- **Alternatives to compare:** retain explicit mode guards in each control callback; construct one
  independent `Bindings` object per mode; add enable/disable to individual bindings; add a small
  Boolean-only contextual group plus a scalar source transform; put parallel Boolean and scalar
  declarations on one context with an explicit activation policy; restrict all binding APIs to new
  neutral-aware control-source subtypes; duplicate every reusable helper overload for root/context;
  expose one narrow registration-only capability shared by root/context; or build a general
  input-layer stack with priorities.
  Compare held buttons, analog neutral thresholds, transformed and sensor sources, inactive-output
  ownership, duplicate writers, precedence, and the flat common path.
- **Selected hypothesis after Gate 1:** keep today's flat declarations as the default and add one
  optional, root-created `Bindings.ControlContext` for repeated contextual controls. Its declaration
  surface is parallel with root `Bindings`, including `copyEachCycle(...)`, and construction
  requires one explicit `ActivationPolicy`: `REARM_AFTER_NEUTRAL` or `ACCEPT_CURRENT`. Root
  `Bindings` and `ControlContext` implement one narrow public `BindingRegistrar` declaration
  capability so reusable helpers accept either through one signature without owning update/clear or
  duplicating overload families. Each registered Boolean or scalar binding applies the context
  policy independently while sharing one context-activation snapshot.
  Contextual scalar copy is deliberately limited to final command streams where exact zero is the
  safe inactive command; there must still be only one scalar-copy registration for a final command
  across root bindings and all contexts. Preserve the deliberate startup-recentering policy and
  document its operator precondition. Mutually exclusive activation predicates can model the
  exactly-one-active mode layers evidenced by Cuttlefish, but no implicit priority, fallthrough,
  automatic subsystem arbitration, or Task ownership enters the framework.
- **Completion:** tests cover first activation/reactivation while a digital control is pressed,
  release/repress, fall/mirror/level/toggle/nudge behavior, both activation policies, multiple
  enabled contexts, activation changes during callbacks, repeated same-cycle root updates, and
  Task-binding enqueue without implicit cancellation. Scalar tests cover activation while
  neutral/displaced under both policies, neutral-then-move, inactive/unarmed zero output,
  deactivation/reactivation, non-finite input, no inactive upstream sampling, and a continuous
  zero-safe command. Migrate the proven framework tester handoffs, keep normal Phoenix TeleOp
  behavior unchanged, and document callback guards, control contexts, navigator-versus-context
  selection, exclusive mode-context use, the one-final-command-writer rule, and the startup-neutral
  requirement. True priority/fallthrough, dynamic child/session ownership, non-zero inactive
  policies, persistent-intent cleanup beyond mirror/copy neutralization, and Task cancellation
  remain separately owned.
- **Decision record (2026-07-21):**
  - **Confirmed traced behavior:** every `Bindings.onRise(...)` registration creates an independent
    `signal.risingEdge()` tracker. In several hardware testers, the picker registers `A` first and
    an active-control callback registers the same raw `A` later. On one physical press, the picker
    callback resolves hardware and changes `ready` to true; the later independent edge tracker sees
    the same rise, observes the newly true gate, and may enable the selected actuator in that same
    `Bindings.update(clock)`. `DcMotorPositionTester` can therefore toggle `RUN_TO_POSITION` from
    the selection press before its downstream INIT update. `DcMotorPowerTester` already records the
    selection cycle and rejects controls during that cycle, confirming the intended one-boundary
    safety rule. `TesterSuite` also constructs and initializes a fresh selected child during its
    menu-`A` callback, then invokes that child's loop in the same shared cycle. Fresh child edge
    trackers establish a safe baseline, but `mirrorOnChange`, levels, and scalar copies intentionally
    accept an initial value: selecting `DrivetrainMotorDirectionTester` with `A` can therefore mirror
    that held selection input into its back-left motor target during the same INIT cycle. Separately,
    `active.and(button)` manufactures a false-to-true result when a mode is
    enabled while the physical button is already held, while conditionally updating independent
    `Bindings` objects leaves their edge state stale across reactivation.
  - **Current callers and common path:** `PhoenixTeleOpControls` uses one flat `Bindings` and has no
    contextual remapping, so the beginner path must not change. `SelectionMenu`, `MenuNavigator`,
    and related UI helpers keep raw edges sampled and guard inside callbacks; that remains the
    smallest safe pattern for one or two conditional actions. Repeated picker-versus-active control
    groups exist in `DcMotorPositionTester`, `DcMotorVelocityTester`, `DcMotorPowerTester`,
    `CrServoPowerTester`, `ServoPositionTester`, `NormalizedColorSensorTester`, and the Pinpoint plus
    AprilTag fusion tester. These are real modern callers, not legacy robot code, and they establish
    the need for one reusable activation boundary. `SelectionMenu`, `HardwareNamePicker`,
    `MenuNavigator`, `ScalarTuner`, `IntTuner`, and `TaskBindings` currently accept concrete
    `Bindings`; without a common declaration capability, their existing helper calls cannot be
    reused with a context. `PhoenixPedroAutoSelectorOpMode` already proves the distinct picker-flow
    pattern: one `MenuNavigator` binds Up/Down/Select once and dispatches those stable meanings to
    the current `MenuScreen` while successive choices push new pickers. The original pinned
    Cuttlefish repository is no longer readable, but the exact-revision framework import above
    verifies its one-selected-layer facade, activation suppression, Boolean priming, and immediate
    post-prime analog behavior.
  - **Small-case comparison:** one or two mappings should remain a single continuously sampled edge
    with visible robot-owned dispatch, for example
    `bindings.onRise(operator.a(), () -> { if (mode == ENDGAME) endgame.deploy(); });`. This adds no
    concept and does not convert a previously held button into a later press. A controls owner may
    use one `switch (mode)` inside that callback when the same physical input has mutually exclusive
    meanings. Callers must still decide what simultaneous mode-selection and action inputs mean;
    the optional context gives repeated mappings one atomic activation snapshot.
  - **Chosen parallel API:** add one root-created context type and one construction method, with no
    public context constructor, alias factory, or policy-default overload:
    `Bindings.ControlContext endgameControls = bindings.contextWhen(endgameMode, Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);`.
    The policy argument is mandatory because accepting a held/nonzero control is motion-relevant;
    neither a hidden default nor two factory spellings improve the call. A context exposes the same
    applicable declaration names and argument order as root `Bindings`: `onRise`, `onFall`,
    `mirrorOnChange`, `whileHigh`, `whileLow`, both `toggleOnRise` forms, `nudgeOnRise`, and
    `copyEachCycle`. Extract those declaration methods once into public `BindingRegistrar`,
    implemented by both root `Bindings` and `ControlContext`. The interface has one distinct
    capability: reusable helpers can register mappings without receiving the root's heartbeat,
    clear, or context-construction ownership. Change helper parameter types rather than adding
    root/context overload pairs; ordinary calls continue passing `Bindings`, while contextual calls
    pass the context they already created. Root `Bindings` remains the simple always-live declaration
    surface.
    `bindings.update(clock)` remains the only heartbeat and dispatcher; a context exposes no
    independent `update`. `TaskBindings.of(registrar, runner)` remains its one factory and accepts
    either root or context through that same declaration capability; it only enqueues a fresh Task
    after an accepted event. Activation stays source-driven through the `BooleanSource` passed to
    `contextWhen(...)`; do not add imperative `enable()`/`disable()` calls whose mid-dispatch timing
    would need a second lifecycle rule.
  - **Activation policies:** `REARM_AFTER_NEUTRAL` and `ACCEPT_CURRENT` are context-wide choices applied
    independently by each registration; there is no global latch that waits for every control in a
    context to be neutral at once. Both policies use an effect-free activation frame.
    `REARM_AFTER_NEUTRAL` arms a final Boolean only after observing false and a scalar copy only after
    observing finite exact zero; a neutral activation-frame sample may arm for the following cycle.
    A held true/nonzero value remains suppressed until that registration independently reaches its
    neutral state, and the neutralizing sample itself creates no fall, mirror, level, toggle, nudge,
    or nonzero copy effect. `ACCEPT_CURRENT` records current Boolean baselines without synthesizing
    an initial rise or toggle, and accepts a finite scalar value, on the effect-free activation
    frame. Beginning on the following cycle, held levels and mirrored state are visible, a later
    release from an accepted high may produce a real fall, and a still-displaced finite scalar may
    pass. This intentionally covers Cuttlefish's post-prime held/analog behavior without making it
    the safer default.
  - **Contextual scalar contract:** `ControlContext.copyEachCycle(source, consumer)` is only for a
    conditioned final command stream where exact zero is the safe inactive command. It does not
    sample its upstream source while inactive and writes zero while inactive, on the activation
    frame, while `REARM_AFTER_NEUTRAL` is unarmed, or for a non-finite sample. Under
    `REARM_AFTER_NEUTRAL`, non-finite input disarms the registration; under `ACCEPT_CURRENT`, a later
    finite value may resume because accepting current values is the explicit policy. Each final
    scalar command/sink must have exactly one `copyEachCycle` registration across root bindings and
    all contexts. Servo-position hold, a non-zero stow target, mode-dependent arbitration of the
    same command, and other fallback policies remain one robot-owned composed source registered at
    the root. Do not also expose public `ControlContext.rearmFromZero(...)` or
    `ScalarSource.rearmFromZeroWhen(...)`; either would restore a second spelling for contextual
    continuous binding.
  - **Activation and rearm contract:** the root snapshots every context's activation once before any
    binding callback and uses those snapshots through all Boolean and scalar dispatch phases, so a
    mode changed by a callback takes effect on the next loop rather than changing the meaning of the
    current controls frame midway through dispatch. Every registration
    owns its own edge/rearm state; a held `A` must not prevent an unrelated released `B` or neutral
    stick from becoming usable. The final value supplied to a Boolean binding defines its contract:
    false is neutral. Thus a derived `axis.above(threshold)` may use below-threshold false as its
    intentional neutral state. If robot policy truly requires a physical return to a narrower
    analog-neutral region, robot code first builds a final Boolean whose false state expresses that
    policy; the generic context does not infer physical displacement from a transformed Boolean.
    Toggle state persists across inactive intervals and neither policy overwrites it from a held
    activation-frame input.
  - **Deactivation, overlap, and ownership:** deactivation disarms event/level/toggle/nudge
    registrations without manufacturing callbacks. Contextual scalar copies continue publishing
    zero while inactive. Contextual `mirrorOnChange` publishes its effective neutral false once on
    its initial inactive update and once when an active true mirror deactivates; it does not publish
    false every cycle or turn deactivation into `onFall`/`whileLow`. These two continuous mirrors
    therefore cannot leave a stale non-neutral request. Deactivation does not cancel an
    active or queued Task, reset toggle state, or perform robot/subsystem cleanup beyond those
    declared neutral mirrors. A mode transition that requires other effects calls the robot
    capability/supervisor abort or the correctly owned runner operation explicitly. More than one
    context may be active and every eligible action runs; there is no hidden winner. Mutually
    exclusive predicates derived from one robot mode can model Cuttlefish-style exactly-one-active
    control layers. Overlap, per-control fallthrough, input consumption, and priority are not
    inferred. Like scalar copy, one persistent state setter must not be mirrored by competing root
    or context registrations. `Bindings.clear()` invalidates its old contexts so later declarations on them fail
    fast; callers retain ownership of broader capability cleanup when rebuilding bindings.
  - **Why parallel scalar declaration is bounded:** moving `copyEachCycle(...)` onto the context is
    safe only because choosing that method asserts a specific final-command contract: exact zero is
    safe, and the parent root publishes it while the context cannot pass a live value. This is not a
    generic inactive fallback for every `ScalarSource`. A servo position, non-zero hold/stow target,
    or two mode-dependent sources for one final command must be composed once in robot code and
    registered once at the root. Because Java `DoubleConsumer` method references do not provide a
    stable sink identity, the framework cannot reliably detect two registrations for the same final
    command; the one-final-copy rule is therefore an explicit ownership contract, not declaration
    order as hidden arbitration.
  - **Gamepad neutral contract:** `GamepadDevice` consciously captures current stick/trigger
    positions as logical zero during construction or explicit `calibrate()`, compensating for
    spring wear and controller center offset; its configurable deadband then maps nearby readings to
    exact zero. Preserve that behavior. Because software cannot distinguish drift from a deliberately
    held input at that instant, the documented precondition is that every analog control is at its
    intended physical neutral whenever the wrapper is constructed or recalibrated. Per-use behavior
    deadbands may still be applied before contextual scalar copy; the conditioned source must map
    the intended neutral region to exact zero before the context applies its activation policy.
  - **Why generic sources do not expose a neutral range:** `ScalarSource` intentionally represents
    gamepad inputs, battery voltage, distance, analog voltage, encoder position/velocity, Plant
    targets and errors, controller outputs, and Task output streams. Many have no neutral, while
    others have a tolerance that changes by use: zero encoder position is a reference, zero error is
    only “at target” under a caller-selected tolerance, and a distance sensor's inactive range
    depends on the behavior. Adding optional neutral metadata would make every transform (`scaled`,
    `clamped`, `ratePerSecond`, rate limiting, fallback/hold, and conditional selection) preserve,
    change, or discard a safety claim it often cannot prove. `BooleanSource` has the same issue:
    false is a value, not universal proof of physical release. Instead, an adapter/source pipeline
    that is intentionally used as a control signal normalizes its inactive meaning to exact zero or
    false before contextual declaration. A sensor with an intrinsic baseline may be
    calibrated/conditioned to zero and then use the same narrow context; non-zero or use-specific
    neutral policy stays explicit at its interpretation boundary. Restricting all `Bindings` methods to a
    `ScalarControlSource`/`BooleanControlSource` subtype was also considered: it gives contextual
    code a compile-time marker, but flat bindings do not need neutral metadata, existing sensor/UI/
    software signals are legitimate binding inputs, ordinary transforms would either lose the
    subtype or require a parallel transform hierarchy, and the subtype still cannot prove what a
    consumer must do while inactive. Do not add a broad `neutralRange()` contract or neutral-aware
    source hierarchy without a real caller that canonical zero/false cannot express.
  - **Rejected alternatives:** callback guards are retained for small cases but repeat the mode on
    every declaration and can observe a gate changed by an earlier same-cycle callback; independent
    `Bindings` objects are unsafe after skipped sampling and add multiple heartbeats; per-binding
    enable overloads multiply every sibling API; a Boolean source-level gate cannot represent an
    inactive third state, so `whileLow` would run while disabled and deactivation could synthesize a
    fall; a Boolean-only context plus `rearmFromZero(...)` and root scalar copy is visibly
    non-parallel and leaves two spellings for continuous contextual binding; an unrestricted
    context scalar copy that guesses arbitrary neutral/fallback policy would be unsafe, so the
    selected method is explicitly zero-neutral and retains the one-final-copy ownership rule;
    per-binding policy overloads repeat one mode-level decision and make a context internally
    inconsistent; a policy-default overload hides a motion-relevant decision, while two named
    context factories duplicate one construction path; one global all-controls-neutral latch lets
    an unrelated held input block the entire context;
    restricting `Bindings` to new control-source subtypes makes the flat API pay for contextual
    metadata and either loses that type through ordinary transforms or recreates a parallel source
    hierarchy; changing or deleting deliberate startup recentering loses useful hardware-error
    compensation; recreating or resetting bindings loses registrations/toggle state or propagates
    reset into shared upstream sources; a general layer/priority stack hides precedence and imports
    much more machinery than Phoenix needs.
  - **Student-facing and Framework Principles result:** flat robot code remains exactly one
    `Bindings`, its current declarations, and one update. An advanced controls owner pays for one
    clearly named control context and one mode source, then writes ordinary binding lines without a
    repeated guard or repeated mode predicate. The mode and cleanup meaning stay in robot
    controls/capabilities, the root owns the single input heartbeat, no FTC or subsystem type enters
    core binding dispatch, and no API infers analog facts it cannot prove. The context name reflects
    that one activation boundary covers Boolean events and zero-neutral scalar commands. The parent
    root still owns the only heartbeat/dispatcher, while the context owns one coherent declaration
    surface and `TaskBindings` preserves the applicable event vocabulary. The student writes, for
    example,
    `manualControls.copyEachCycle(operator.leftY().deadbandNormalized(0.08, -1.0, 1.0), lift::commandManualPower);`.
    The required policy at context construction makes held-input behavior visible without another
    wrapper, builder, repeated mode predicate, or scalar binding path.
  - **Context count and exclusivity:** a robot with no conditional control group creates no context;
    root `Bindings` remains its complete controls API. One optional group needs only one context,
    alongside any always-available root mappings. Multiple contexts may overlap when they command
    independent meanings and sinks. When contexts are being used as mutually exclusive layers that
    remap the same physical controls or capabilities, robot code derives mutually exclusive
    activation sources from its mode; the framework does not require global exclusivity or choose a
    winner when predicates overlap. In every case the robot still calls only the parent
    `bindings.update(clock)` once per cycle.
  - **Tester ownership and simplification:** do not create or retain one parent-owned context per
    dynamically selected `TeleOpTester`. `TesterSuite` and `HardwareSelectingTester` already own a
    stronger boundary: a factory returns one fresh child with its own root `Bindings`, and the child
    is stopped and discarded before replacement. A parent context would retain callbacks and child
    or hardware references, would not release resources, and would weaken that lifecycle. Contexts
    instead simplify stable modes inside one tester. A picker may remain one continuously sampled,
    callback-guarded menu, while a repeated selected-device group becomes
    `bindings.contextWhen(BooleanSource.of(() -> ready), REARM_AFTER_NEUTRAL)` and registers its
    tuner/action bindings without repeated `if (!ready) return` checks or selection-cycle fields.
    Use the test's existing lifecycle contract when choosing `ready` versus
    `ready && opModeStarted`; INPUT-01 must not silently move a tester's allowed controls between
    INIT and RUN. A directly selected tester whose mirror/level/scalar controls must reject the held
    suite-selection input uses one always-enabled neutral-rearm context for those action controls.
    Context deactivation still does not replace immediate hardware zero, mode restoration, BACK,
    `stop()`, or child fail-stop cleanup.
  - **Picker and navigator boundary:** when Up/Down/Select keep the same semantic meaning while only
    the visible list changes, bind `MenuNavigator` once and let each `MenuScreen` receive those
    stable events. A selection callback pushes or replaces the next picker; continuously sampled
    edge state means a held navigation button does not become a new press on the next screen. One
    context per picker would add registrations, retain dynamically replaced screens, and duplicate
    dispatch already owned by the navigator. Wrap the navigator as a whole in one context only when
    the entire wizard is optional relative to other robot controls. Use distinct contexts when the
    controls' meanings actually change—for example from picker navigation to actuator commands—not
    merely because the recipient list changed.
  - **Reusable-helper boundary:** change the first parameter of menu, picker, navigator, tuner, and
    Task-binding registration helpers from concrete `Bindings` to `BindingRegistrar`; do not add
    sibling `ControlContext` overloads. Retain a continuously sampled callback guard only where it
    is deliberately the smaller one-screen/one-action policy. Repeated hazardous action groups use
    a context and should not also repeat an `active` predicate inside every helper callback. The
    stateful `ScalarTuner`/`IntTuner` axis-update methods are not ordinary scalar-copy bindings: they
    update persistent targets only while displaced (and integer nudge also consumes `dtSec`). Keep
    their explicit activity gate in this item rather than pretending contextual zero-copy has the
    same meaning; do not pass a persistent tuner target setter to contextual `copyEachCycle`.
  - **Bounded implementation scope:** change only `Bindings`, the narrow `TaskBindings` adapter,
    directly needed tester registration helpers/callers, focused tests, Javadocs, the sources/loop
    and controls-owner guides, and this tracker. Add only `BindingRegistrar`, `ControlContext`, its
    required `ActivationPolicy`, the parallel zero-neutral contextual `copyEachCycle`, and the
    helper/TaskBindings parameter migrations needed by this task; do not add a transform to
    `ScalarSource` or `ControlContext`, alter
    flat binding execution order, begin API-04, add a global mode enum or priority stack, add an
    arbitrary contextual scalar fallback API, change `GamepadDevice` calibration behavior, change
    stateful tuner-axis semantics, replace fresh tester-child lifecycle with contexts, change
    TaskRunner cancellation, or add season-specific control policy to Phoenix. Keep
    `PhoenixTeleOpControls` behavior unchanged.
  - **Software verification plan:** add focused binding tests for inactive silence, effect-free
    activation under both policies, held activation, false rearm, accept-current baselines, fresh
    rise/fall, mirror neutralization and both levels, toggle persistence, nudge, independent per-
    registration rearm, two active contexts, activation mutation during a callback, root same-cycle
    idempotency, shared predicate sample count, clear/context invalidation, and flat regressions. Add
    TaskBindings tests proving the one `BindingRegistrar` factory accepts root/context, fresh Task
    factories enqueue only after accepted events, and context deactivation does not cancel
    shared-runner work. Add compile/static helper checks proving menu, picker, navigator, and tuner
    helpers accept either registrar without sibling overloads. Add fake-backed tester regressions
    proving selection `A` cannot also enable or mutate the selected device, selecting the
    drivetrain-direction tester with held `A` cannot mirror a motor command, and BACK/reselection
    rearms safely. Preserve `TesterSuite` fresh-child/stop behavior and verify a navigator screen
    transition does not double-deliver or manufacture a held navigation edge. Add contextual scalar
    tests for inactive zero without upstream sampling, effect-free activation, displaced activation
    under both policies, zero-then-later movement, deactivation/reactivation, policy-specific
    non-finite recovery, same-cycle idempotency, and continuous copy into both frame-valued and
    persistent test consumers. Audit every root/context scalar copy to preserve one registration per
    final command. Run the focused suites, full TeamCode unit tests, FTC Java
    compile, public-surface/static caller audits, documentation link checks, and `git diff --check`;
    no robot hardware is required for the input-state contract. Real-controller deadband adequacy
    remains adoption validation.
  - **Approval gate:** this is a major public API and lifecycle decision that materially revises the
    previous split design. One `Bindings.ControlContext` now owns a fully parallel declaration
    surface for Boolean bindings and zero-neutral scalar copy, with one required context-wide
    `ActivationPolicy`; root and context share the registration-only `BindingRegistrar` capability
    so reusable helpers keep one signature while the parent root still owns dispatch and the single
    heartbeat. Mutually exclusive contexts may represent exactly-one-active mode layers, but stable
    picker-to-picker navigation stays one `MenuNavigator`, dynamically selected testers remain fresh
    lifecycle-owned children, and there is no implicit priority or duplicate-sink arbitration. The
    deliberate gamepad startup recentering remains intact with an explicit neutral-at-construction
    precondition. Gate 1 is complete and the item is Ready, but implementation must not begin until
    the user approves this final INPUT-01 context-and-binding-registrar design. No framework or robot
    code has been changed at this gate.
  - **Approval (2026-07-21):** the user approved the final INPUT-01
    context-and-binding-registrar design. Implementation is limited to the recorded context
    semantics, common registration capability, directly required helper/tester migrations, focused
    tests, and synchronized documentation; this approval does not authorize INPUT-02 or another
    tracker item.
  - **Implementation record (2026-07-21):**
    - Added the registration-only `BindingRegistrar` capability, implemented by the existing root
      `Bindings` and its new root-created `Bindings.ControlContext`. The root remains the only
      heartbeat, clear owner, and context factory. `TaskBindings` keeps one `of(...)` factory that
      accepts the shared registrar; no root/context overload family or imperative context lifecycle
      was added.
    - Implemented the approved `REARM_AFTER_NEUTRAL` and `ACCEPT_CURRENT` policies with one
      pre-callback activation snapshot per context, effect-free activation, independent Boolean and
      scalar registration state, persistent contextual toggle state, independent nudge inputs,
      mirror neutralization, exact-zero contextual scalar safety, policy-specific non-finite
      recovery, inactive upstream silence, and root same-cycle idempotency. Ordinary root binding
      phase order and behavior remain covered by regressions.
    - Changed reusable picker, menu, navigator, tuner, and Task-binding registration helpers to
      accept `BindingRegistrar` through their existing API shapes. Removed the tuner button
      helpers' now-redundant trailing activity predicate while preserving the distinct stateful
      `updateFromAxis(clock, active)` contract.
    - Migrated stable picker-to-live action groups in the modern hardware, calibration, and
      localization testers to neutral-rearming contexts without replacing fresh selected-child
      ownership, BACK, stop, resource release, or hardware fail-stop behavior. The real Phoenix
      drivetrain-direction tester now suppresses held suite-selection input and reports applied
      Plant targets rather than raw held-button demand.
    - The lifecycle audit found that a context alone could not clean persistent tester intent.
      `CrServoPowerTester` now stops, disarms, and resets its target for selection, BACK, and stop.
      `ServoPositionTester` snapshots the newly selected device's finite SDK command (not physical
      feedback), remains write-disabled until a fresh enable, and never transfers the prior servo's
      retained command. `PinpointPodOffsetCalibrator` keeps its B abort on root bindings so a vision
      failure cannot hide the stop action while a drive phase continues.
    - Updated the Framework Principles, public Javadocs, loop/source guidance, controls-owner guide,
      FTC UI guide, and testing guide. The primary explanation is now ordinary robot TeleOp use:
      always-eligible root mappings plus optional normal/endgame or manual-mode contexts, with
      tester navigation retained only as a secondary lifecycle example.
  - **Adversarial audit (2026-07-21):** initial independent reviews found incomplete reactivation
    coverage, reduced public method Javadocs, retained actuator requests and inversion across device
    reselection, a vision-dependent abort binding, a synthetic drivetrain regression that did not
    exercise the real tester, misleading raw-input telemetry, and wording that implied one
    group-wide rearm latch or described raw rather than effective contextual Task input. Each
    finding was corrected before this audit point. The resulting API still has one
    context factory, one shared registration capability, one root heartbeat, no hidden priority or
    cancellation, and no change to ordinary `PhoenixTeleOpControls` behavior.
  - **Automated verification (2026-07-21):**
    - Focused INPUT-01 matrix: 31 tests across `BindingsControlContextTest` (12),
      `BindingsRootRegressionTest` (2), `TaskBindingsContextTest` (3),
      `ContextualUiBindingTest` (2), `ContextualHardwareTesterSafetyTest` (3), and the existing
      `TesterMenuLifecycleTest` (9), with 0 failures, 0 errors, and 0 skipped. These include real
      fake-backed CRServo/Servo reselection and the actual Phoenix drivetrain tester entered through
      `TesterSuite`, not only a synthetic binding example.
    - Full `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac`: 71 suites and 684 tests,
      with 0 failures, 0 errors, and 0 skipped; compilation succeeded. Output contained only the
      existing Java 21/source-8 and FTC app-shell deprecation warnings.
    - `git diff --check`, all-untracked trailing-whitespace scanning, changed-Markdown local-link
      checks, stale/duplicate helper-signature searches, one-factory public-surface searches, and
      external case-study-reference checks passed. `PhoenixTeleOpControls` is unchanged.
  - **Manual verification (2026-07-21):** the user reviewed the presented INPUT-01 implementation
    and robot-first documentation at the Android Studio audit point and replied `INPUT-01 looks
    good`. No controller or robot hardware was available; real gamepad deadband adequacy and
    physical device behavior remain adopting-robot validation rather than claims made by the
    software tests.

### HAPTIC-01 - Driver haptic feedback boundary

- **Confirmed problem:** competition robot controls use gamepad rumble for short readiness,
  intake-full, relocalization, mode, warning, and endgame cues. Phoenix can read controller input
  through framework Sources but has no corresponding output seam. A robot-owned controls class must
  therefore retain an FTC `Gamepad`, express durations in SDK milliseconds, and couple otherwise
  portable event policy to an asynchronous, controller-specific SDK command.
- **Confirmed external evidence:** the original Cuttlefish Decode snapshot is no longer publicly
  readable, so this gate rechecked its accessible
  [public fidelity port at pinned commit `42d1ce8`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tele/Tele.java).
  It sends only fixed-duration full-strength pulses: driver-one warning activation, intake-full, and
  relocalization cues; driver-two mode, turret-hold, sorting-layer, and endgame cues. Its two helper
  methods retain one last-accepted timestamp per controller and drop calls inside a 250 ms window.
  Some state conditions call those helpers every loop, so that cooldown is compensating for
  state-based requests rather than adding a hardware capability.
- **Confirmed FTC SDK behavior:** local RobotCore 11.1 sources and the bundled
  `ConceptGamepadRumble` sample show that a new rumble effect displaces the active effect and enters
  a one-element evicting outgoing queue. `stopRumble()` queues another command; it is not a confirmed
  immediate physical stop. `isRumbling()` is only a wall-clock estimate. The SDK exposes no reliable
  controller-support or delivery query: controller `id`/`type` can be unknown for an attached idle
  gamepad, and documented controller models have different or absent rumble hardware. The core
  contract must therefore be command-only and best-effort.
- **Current caller and ownership audit:** `GamepadDevice` is an input/calibration owner for axes and
  buttons and already has an FTC import that BOUNDARY-01 must resolve. `Gamepads` exposes
  `create(Gamepad, Gamepad)`, the redundant `of(...)` alias, and a public wrapped-device
  constructor. Twelve modern callers use `Gamepads.create` (Phoenix, the Phoenix Auto selector,
  `BaseTeleOpTester`, and nine TeleOp examples); none uses `of`, constructs a wrapped gamepad
  directly, or requests rumble. `PhoenixTeleOpControls` has one composition-root caller and owns
  operator meaning, while `PhoenixRobot` has the raw gamepads at its FTC construction boundary and
  owns cleanup. HAPTIC-01 can add output without changing any existing input constructor or caller.
- **Side-by-side alternatives and student-visible concepts:** the gate compared complete creation
  and event calls, not only the shortest leaf method:

  | Approach | Representative robot code | Student-visible decisions and result |
  |---|---|---|
  | Direct FTC / documentation only | `gamepad1.rumble(200);` | One line, but every event chooses an SDK recipient, milliseconds, and SDK effect semantics; controls remain FTC-coupled and harder to fake. |
  | Add output to `GamepadDevice` | `gamepads.p1().pulse(1.0, 0.20);` | Short, but makes an input/calibration object also command output, deepens BOUNDARY-01's existing leak, and gives selectors/testers an output capability they did not ask for. |
  | Robot-local adapter | `driverHaptics.pulse(1.0, 0.20);` | Can be clean for one robot, but repeats the same stable SDK conversion, validation, and stop contract in each adopting robot. |
  | Separate framework sink and FTC factory | `HapticSink driverHaptics = FtcHaptics.gamepad(gamepad1);` then `driverHaptics.pulse(1.0, 0.20);` | Chooses the recipient and SDK boundary once; each semantic event supplies only normalized strength and seconds. Core controls are fakeable and SDK-free. |
  | Reusable pattern value | `driverHaptics.play(Haptics.pulse(1.0, 0.20));` | Adds a pattern type, factory, nested call, and storage concept. The real callers use only immediate fixed pulses and do not store, pass, compose, or sequence effect values, so the extra layer has no current payoff. |
  | Sink-wide cooldown decorator | `Haptics.cooldown(driverHaptics, clock, 0.25)` | Shortens repeated state calls but can drop an urgent warning because an ordinary cue fired first. Per-event keys, priorities, and bypass paths grow into the rejected notification router. |
  | Generic event bus | `feedback.publish(DRIVER, INTAKE_FULL);` | Makes the leaf call shortest by moving season meaning, recipient, pattern, interruption, and priority policy into a much larger hidden framework owner. |
- **Selected public API:** add one SDK-free `HapticSink` with exactly
  `pulse(double strength, double durationSec)` and `stop()`. Add one public construction path,
  `FtcHaptics.gamepad(Gamepad)`, returning the interface through a private adapter. There is no
  public adapter constructor, `create`/`of` alias, staged builder, pattern class, dual-motor channel
  overload, continuous-effect API, `isActive`, availability query, success result, silent-sink
  factory, or method on `GamepadDevice`/`Gamepads`. The two scalar arguments are the complete inline
  answers for the one evidenced command and do not justify another value type.
- **Pulse and validation contract:** strength is normalized, finite, and in `(0, 1]`; duration is
  finite and positive seconds. Zero is not a second spelling for `stop()`. The FTC adapter sends the
  same effective normalized strength to both SDK rumble channels so robot code does not choose
  controller-specific motors and one-channel controllers still receive channel one. It rounds a
  valid positive strength below the SDK's first nonzero command level up to that level, rounds
  every positive sub-millisecond request up to one millisecond, and rejects durations that cannot
  fit the SDK's finite millisecond range before queuing an effect. The SDK retains only the latest
  undelivered request; once delivered, a later pulse intentionally replaces an earlier one. A normal
  return means only that the SDK request was accepted locally; it does not prove support, delivery,
  completion, or physical intensity.
- **Cooldown and event policy:** do not add a haptic-specific cooldown wrapper. Ordinary cues are
  requested on semantic transitions, for example
  `bindings.onRise(intakeFull, () -> driverHaptics.pulse(1.0, 0.50));`, rather than by writing the
  same state every loop. Noisy Boolean facts may use the existing Source debounce transforms. If
  one particular active state genuinely needs a periodic reminder, its controls/driver-feedback
  owner owns one existing `Cooldown`; a supervisor or service may supply the state or policy that
  owner consumes. That explicit per-event policy cannot suppress an unrelated urgent cue. A safety
  warning may deliberately replace an ordinary pulse immediately.
- **Lifecycle and ownership:** the FTC composition root constructs one sink per intended recipient,
  passes those sinks to robot controls or a dedicated driver-feedback owner that owns cue mapping,
  and best-effort calls each sink's `stop()` during total cleanup. Supervisors/services may supply
  the status or robot policy that owner consumes. Repeated `stop()` calls are safe, but the contract
  describes an SDK stop request rather than immediate physical acknowledgement. Calls remain on the
  ordinary OpMode loop thread. The sink owns no heartbeat, `LoopClock`, timer, scheduler, background
  thread, event queue, retry, or priority. When multiple policies share a controller, that one
  controls/feedback owner coordinates their meaning; framework dispatch does not guess a winner.
- **Framework Principles and simplicity result:** the core capability owns one narrow reusable
  command; the FTC factory owns SDK conversion; robot controls or a dedicated driver-feedback owner
  owns cue mapping, recipient, interruption, and any repeat policy; and the composition root owns
  construction and cleanup.
  Flat robots that need no rumble gain no object or heartbeat. An adopting robot replaces raw SDK
  event calls with one fakeable method and seconds used everywhere else in Phoenix. Output does not
  contaminate the input Source graph, and HAPTIC-01 neither preserves nor broadens BOUNDARY-01's
  existing SDK leak. This is the smallest design that supports all observed Cuttlefish behavior.
- **Rejected or deferred scope:** do not add rumble to `GamepadDevice`; do not infer support from
  gamepad metadata; do not claim delivery or physical stop; do not add a blanket cooldown,
  notification bus, scoring vocabulary, LED/audio abstraction, custom SDK effect escape hatch,
  multistep pattern DSL, or Phoenix season cue merely to manufacture an in-repository production
  caller. A future real caller that must store or sequence patterns requires a new decision gate.
  Deliberately disabling haptics remains visible robot policy (omit/guard the binding); an
  unsupported controller safely produces no physical feedback without an unreliable automatic
  detection branch.
- **Bounded implementation scope:** add only the core sink, one FTC facade/private adapter, focused
  tests, package/public Javadocs, robot-first controls and loop/lifecycle guidance, any directly
  affected documentation index, and this tracker. Existing Phoenix behavior and every `Gamepads`
  construction path remain unchanged. Do not begin BOUNDARY-01 or clean its aliases in this item.
- **Software verification plan:** use SDK-backed unit tests to verify null rejection, both-channel
  strength mapping, seconds-to-milliseconds rounding, finite/range/overflow validation before SDK
  effects, latest-request replacement, independent driver/operator queues, and safe repeated stop
  requests. Add fake call-site tests for edge-driven cues and an event-specific existing
  `Cooldown` whose ordinary reminder cannot block a direct urgent pulse. Add static checks proving
  the core haptic package imports no FTC SDK type and that there is one public FTC construction
  path with no public adapter. Run focused tests, full TeamCode unit tests, FTC Java compilation,
  changed-Markdown link checks, and `git diff --check`. Software tests cannot prove controller
  support, delivery, felt strength, physical stop latency, or actual OpMode-thread use; those remain
  adopting-robot/hardware validation and must be documented as such.
- **Approval gate (2026-07-21):** this is a new public API and therefore a major decision. Gate 1 is
  complete and HAPTIC-01 is Ready, but implementation must not begin until the user approves the
  factory-only fixed-pulse sink design. No framework or robot implementation code has been changed
  at this gate.
- **Approval (2026-07-21):** after explicitly reconsidering multistep pattern support, the user
  chose the simpler fixed-pulse design with “Keep it simple for now and proceed with your
  suggestion.” Implementation is limited to the recorded `HapticSink`/`FtcHaptics` API, fixed-pulse
  semantics, tests, and synchronized documentation; this approval does not authorize pattern
  support, BOUNDARY-01, or another tracker item.
- **Implementation record (2026-07-21):**
  - Added the SDK-free `HapticSink` command seam with only `pulse(strength, durationSec)` and
    repeat-safe, best-effort `stop()`, plus package Javadocs that keep cue mapping in robot controls
    or a dedicated driver-feedback owner. The interface owns no clock, heartbeat, scheduler,
    availability state, pattern value, cooldown, or event vocabulary.
  - Added the sole FTC construction path, `FtcHaptics.gamepad(Gamepad)`, returning `HapticSink`
    through a private adapter. It rejects null gamepads, invalid strength/duration, and
    unrepresentable duration before an SDK effect; sends one effective strength to both channels;
    raises every valid positive sub-level strength to the SDK's first nonzero level; rounds every
    positive sub-millisecond duration up to one millisecond; and delegates stop to the SDK's queued
    zero/zero continuous request. No existing `GamepadDevice`, `Gamepads`, Phoenix constructor, or
    robot behavior changed.
  - Added SDK-queue tests for the exact public surface, sole factory/private adapter, validation
    ordering, strength/channel quantization, minimum positive strength, duration rounding and exact
    overflow boundary, latest-undelivered replacement, recipient isolation, and repeated stop.
    SDK-free usage tests exercise real `Bindings` edge dispatch and a per-event existing `Cooldown`
    that cannot suppress a direct urgent cue.
  - Updated the robot architecture guide, loop guide, and framework package overview. The docs show
    one recipient-specific sink per controller, controls/driver-feedback cue ownership, edge-driven
    ordinary cues, explicit event-specific repeat policy, best-effort aggregate cleanup, no extra
    loop phase, and the FTC runtime/Driver Station/controller limits on support and delivery.
- **Adversarial audit (2026-07-21):** independent correctness, public-API/simplicity, SDK, and
  documentation reviews repeated the construction-path and Framework Principles checks. They
  confirmed that the two public types have distinct value and found no reason for a duration-only
  overload, public adapter, `create`/`of` alias, pattern object, cooldown decorator, silent sink,
  availability query, or event bus. Findings corrected before this audit point were a null-test
  exception mismatch, an imprecise Driver Station event-loop attribution, a cleanup example that
  did not initially aggregate two stop failures, ownership wording that let supervisors choose
  controller cues, and a valid tiny strength that the SDK would otherwise quantize to known zero.
  Rechecks found no remaining issue.
- **Automated verification (2026-07-21):**
  - Focused `FtcHapticsTest` and `HapticSinkUsageTest`: 13 tests, 0 failures, 0 errors, and 0 skipped.
  - Full `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac`: 73 suites and 697 tests,
    with 0 failures, 0 errors, and 0 skipped; TeamCode compilation succeeded. Output contained only
    the existing Java 21/source-8 and FTC app-shell deprecation warnings.
  - `git diff --check`, trailing-whitespace scanning including untracked files, changed-Markdown
    local-link checks, core FTC-import checks, one-factory/private-adapter public-surface searches,
    and framework-code/docs case-study-reference checks passed.
- **Hardware-validation boundary and audit point (2026-07-21):** software inspection can prove the
  queued SDK command, validation, conversion, ownership, and replacement semantics; it cannot prove
  that a particular controller supports rumble, receives a request, produces a distinguishable
  strength, completes a duration, or stops with a particular latency. Those remain adopting-robot
  validation when hardware is available. The implementation was marked Verifying and paused for the
  user's Android Studio review; nothing beyond HAPTIC-01 was started.
- **Manual verification (2026-07-22):** the user reviewed HAPTIC-01 in Android Studio and replied
  `HAPTIC-01 looks good`. HAPTIC-01 is **Done** and publication is authorized for this item only. No
  controller hardware was available, so physical rumble support, feel, delivery, duration, and stop
  latency remain adopting-robot validation rather than software-test claims.

### PERF-01 - FTC hub bulk-cache ownership

- **Problem to confirm:** Phoenix has no explicit owner for Lynx bulk-caching policy. On sensor-heavy
  robots, SDK automatic behavior may be sufficient, while manual caching can reduce repeated hub I/O
  only if every hub is configured and cleared exactly once per OpMode cycle. Scattered clears make
  same-cycle readings inconsistent.
- **External evidence:** Cuttlefish configures all Lynx modules for manual bulk caching during init
  and clears them once at the beginning of every outer loop in
  [`EnhancedOpMode`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/EnhancedOpMode.java).
- **Alternatives to compare:** rely on SDK `AUTO`; configure manual caching directly in Phoenix;
  create a tiny optional `fw.ftc` cycle owner; or hide it in a base OpMode/lifecycle framework.
  Measure real loop I/O before selecting a default and specify INIT, repeated same-cycle calls, and
  shutdown restoration.
- **Leading hypothesis:** retain SDK automatic caching for the beginner path unless measurements show
  a meaningful benefit. If manual mode is justified, one optional FTC composition-root owner discovers
  the hubs, configures them once, and clears them idempotently by `LoopClock.cycle()`. No subsystem
  clears a cache and no base OpMode, global singleton, or loop-rate sleep is added.
- **Completion:** benchmark results and hardware assumptions are recorded; fake/FTC tests cover all
  hubs, one clear per cycle, duplicate calls, INIT/active transitions, partial failure, and stop;
  documentation places the owner at one explicit loop phase; robots that do not opt in need no extra
  concepts.
- **Pause record (2026-07-19):** **Deferred while real hub-I/O benchmarks are unavailable.** Fakes
  can verify one-clear-per-cycle mechanics but cannot establish that manual caching has enough
  benefit to justify a new owner or changing the beginner default.

### PERF-02 - Loop phase diagnostics

- **Confirmed problem:** `LoopClock.update(runtime)` samples once at the outer loop boundary, so
  `dtSec()` reports the preceding top-of-loop-to-top-of-loop period. Its value is intentionally
  frozen for the rest of that cycle and cannot attribute current-cycle work. Framework examples
  can render that overall period through `LoopClock.debugDump(...)`, but modern `PhoenixRobot` does
  not currently present it and has no way to distinguish whether vision readiness, localization,
  targeting, controls, scoring/Plant realization, drive, Auto Tasks, or telemetry owned a slow
  cycle. Those useful boundaries are inside `PhoenixRobot.updateTeleOp()` and `updateAuto()`, so an
  OpMode-local stopwatch can measure only the whole call. Students otherwise repeat ad hoc timing
  and aggregation code or remove behavior experimentally.
- **Complete current caller and ownership audit:** the repository has thirteen modern production or
  example `LoopClock` owners and one supported construction path, `new LoopClock()`. Phoenix advances
  its private clock in `updateAny(runtime)` before either mode-specific update. The FTC tester host,
  the basic Pedro example, the Phoenix INIT selector, and framework examples also expose explicit
  root-level order, but no production or test source contains a reusable named-phase profiler.
  Existing `System.nanoTime()`, `System.currentTimeMillis()`, and `ElapsedTime` uses are either
  vendor/freshness boundaries, legacy Pedro tuning, or SDK samples; none is a supported loop
  diagnostic. `DebugSink` is output-only, `PhoenixTelemetryPresenter` owns Phoenix formatting and
  the single telemetry commit, and COMMON-02 separately owns any future telemetry-commit change.
- **Confirmed external evidence:** the original Cuttlefish Decode snapshot is no longer publicly
  readable, so this gate rechecked its accessible
  [public fidelity port at pinned commit `42d1ce8`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/telemetry/LoopProfiler.java).
  Its sequential `start()`/`mark(name)` path demonstrates the value of high-level named boundaries,
  while its additional token path explicitly permits double counting on a repeated leave, its name
  map is unbounded, and reporting allocates and sorts. SaMoTech's pinned
  [`TaskTimer`](https://github.com/SaMoTechRobotics/FTC-Decode/blob/c8ff047c4245f79d934e622c05481269b5cc42da/TeamCode/src/main/java/util/TaskTimer.java)
  uses a global static map, wall-clock milliseconds, paired start/end calls, and direct logging. A
  real SaMoTech drive caller starts `Update Field Poses` twice without ending it, illustrating the
  fragility of paired calls. The useful shared evidence is phase observation, not either project's
  global state, paired/token API, output coupling, or another scheduler.
- **Side-by-side alternatives and student-visible code:**

  | Approach | Representative composition-root code | Result |
  |---|---|---|
  | Documentation/manual timing | `long start = System.nanoTime(); ...` around every phase | Adds no framework type, but every robot must choose a clock, convert units, pair reads, aggregate, bound history, handle missing calls, and format output. Rejected. |
  | Phoenix-local helper | `phoenixProfiler.finishPhase("localization");` | Can solve Phoenix, but repeats the same diagnostic mechanics in every future robot, framework example, and tester. Rejected. |
  | Extend `LoopClock` | advance or resample it after each phase | Corrupts the one behavioral heartbeat and changes the meaning of `nowSec()`/`dtSec()` inside one cycle. Rejected. |
  | Paired scopes/tokens | `try (Phase ignored = phases.begin("localization")) { ... }` | Doubles or nests instrumentation, adds allocation/lifetime semantics, and creates unfinished, double-close, inclusive/exclusive, and exception questions that flat root phases do not need. Rejected. |
  | Callback wrapper | `phases.measure("localization", () -> localization.update(clock));` | Hides the visible loop order behind callbacks and needs `Runnable`, value-returning, checked-failure, and capture variants. Rejected. |
  | Enum-keyed generic profiler | `phases.finishPhase(Phase.LOCALIZATION);` | Prevents spelling mistakes and predeclares bounds, but makes each adopting robot define an enum and carry a generic snapshot solely to label development diagnostics. Stable literal names plus a hard registry cap provide the needed safety with less student API. Rejected for this first API. |
  | Sequential framework profiler | `phases.finishPhase("localization");` after the visible update | One extra boundary line per useful high-level phase; common timing, aggregation, recovery, bounds, snapshots, and debug output move into the framework. Selected. |
- **Selected public API and one construction layer:** add one final SDK-free
  `fw.core.debug.LoopPhaseProfiler`. Its only public construction path is
  `LoopPhaseProfiler.create(boolean enabled)`; there is no public constructor, `of(...)` alias,
  enabled/disabled factory pair, builder, public time-source parameter, or runtime `setEnabled`.
  The ordinary surface is `startCycle(LoopClock)`, `finishPhase(String)`,
  `finishCycle(LoopClock)`, `snapshot()`, `debugDump(DebugSink, String)`, and an inactive-only
  `reset()` for an explicitly new measurement window or reuse across a deliberate `LoopClock`
  reset. `LoopPhaseProfiler.Snapshot` exposes enabled/completed/incomplete/last-cycle facts; latest,
  mean, and maximum observed-total and unattributed durations; and a stable immutable
  `List<LoopPhaseProfiler.PhaseTiming>`. Each phase entry exposes only its name, sample count,
  last-observed cycle, and latest/mean/maximum duration. Every duration accessor has a `Sec` suffix.
  Snapshot and phase-entry constructors remain private. Disabled creation returns a shared inert
  implementation/state and a shared empty snapshot; every disabled operation returns before
  validation, stopwatch reads, lookup, allocation, telemetry output, or mutation.
- **Timing and one-clock contract:** enabled profiling owns a private diagnostic-only
  `System.nanoTime()` stopwatch, hidden behind a package-private deterministic test seam. It uses the
  supplied `LoopClock` only for owner identity and `cycle()` correlation. It never advances or
  resets that clock, exposes its stopwatch to robot behavior, sleeps, schedules, rate-limits, or
  supplies a behavioral timestamp. Durations are monotonic elapsed wall time, not CPU time or a
  complete FTC callback period: the observed total excludes work before `startCycle` and after
  `finishCycle`, includes OS scheduling pauses and profiler overhead, and is not synchronized with
  sensor timestamps. `finishPhase` samples once on entry to close the named phase and again after
  its internal lookup/accounting to begin the next phase, so observer bookkeeping is not silently
  charged to the following robot owner; that gap remains visible in unattributed time. Public
  duration names and debug keys use explicit seconds.
- **Sequential phase and lifecycle contract:** `finishPhase("localization")` attributes the interval
  since `startCycle` or the preceding `finishPhase` to the just-completed phase. Repeating a name in
  one cycle sums its segments into one phase sample; conditional absence does not add a false zero
  sample; first-observed name order stays stable. There is deliberately no nested phase API. A
  second same-cycle start or retry, an orphan finish, a different clock, an invalid name, a time
  regression, or a same-cycle duplicate completion fails fast with an actionable error and discards
  the active scratch sample. If a prior cycle was left unfinished because observed robot work threw,
  the next later cycle discards and counts that incomplete cycle before starting normally, so
  diagnostics do not permanently poison recovery. Because `LoopClock` exposes no reset generation,
  callers must pair every deliberate clock reset with an inactive profiler `reset()`; numeric cycle
  comparison catches ordinary duplicate/non-advancing use but cannot prove that a reset followed by
  several clock updates never occurred. `reset()` while a cycle is active fails fast; an inactive
  reset clears measurements, known names, and clock/cycle binding so the next cycle establishes a
  fresh observation window.
- **Bounded aggregation and snapshots:** phase totals remain in per-cycle scratch state and commit
  atomically only after a valid `finishCycle`; no malformed partial cycle becomes plausible history.
  The profiler retains a fixed maximum of 32 distinct stable names and constant-size online latest,
  mean, maximum, sample-count, and last-observed-cycle statistics for each phase, the total observed
  span, and unattributed/observer time. It retains no per-cycle history, percentile, threshold, or
  arbitrary rolling-window policy. The 33rd name fails actionably and advises against dynamic names.
  `snapshot()` allocates only on request, returns immutable copies in stable order, never reads the
  stopwatch, and cannot change an active cycle; old snapshots remain unchanged. `debugDump(...)`
  follows the framework's nullable-sink/stable-key convention and renders only completed data.
- **Phoenix integration and student simplicity:** wire the profiler at the `PhoenixRobot`
  composition root behind one private, off-by-default diagnostic constant, and mark its existing
  high-level TeleOp/Auto phases without adding a `PhoenixRobot` constructor, profile field, base
  OpMode, or subsystem instrumentation. When enabled, a retained `FtcTelemetryDebugSink` adds the
  last completed profiler state to the existing Phoenix telemetry frame before the one existing
  presenter commit; the just-measured telemetry phase therefore becomes visible on the next frame.
  The default-disabled robot still uses the identical explicit loop order and gains no time reads,
  allocation, extra telemetry row, scheduler, or heartbeat. Future robots need only the same one
  creation line and one marker after each phase they actually want to distinguish.
- **Rejected and deferred scope:** do not add tracing/Dashboard dependencies, a second telemetry
  commit, automatic reflection/instrumentation, framework-defined season phase names, a public
  monotonic clock, nested or asynchronous spans, thread safety, callbacks, scopes, budgets, alerts,
  percentiles, rolling history, CPU/GC claims, or profiler-owned loop rate. Do not instrument every
  subsystem or migrate unrelated examples/testers in this item. A richer trace is a separate need,
  not a second spelling layered onto this API.
- **Bounded implementation scope:** add the core profiler and immutable nested snapshots, focused
  unit tests with a package-private fake monotonic source, off-by-default Phoenix root markers and
  same-frame debug rendering, synchronized Framework Principles/loop/architecture guidance, and this
  tracker only. No hardware resource or behavior path changes.
- **Software verification plan:** tests cover exact sequential attribution and explicit units;
  repeated and conditionally absent names; stable order; latest/mean/max/count totals; atomic
  unfinished-cycle discard and recovery; every lifecycle misuse; clock reset/rebind; equal,
  regressing, negative-origin, and wrapping nano values; stopwatch call counts; zero disabled source
  calls/state/output/allocation paths; exactly 32 versus 33 names; constant registry/storage over a
  long run; immutable prior snapshots; nullable/stable debug output; and a simulated Phoenix-shaped
  slow phase without sleeping or changing behavior. Static checks prove no FTC/Android/Telemetry
  dependency in the core type and one public construction path. Run focused tests, full TeamCode
  unit tests, FTC Java compilation, changed-Markdown link checks, and `git diff --check`. Software
  tests cannot establish enabled Control Hub overhead or timing resolution, so the feature remains
  opt-in and those measurements remain adopting-robot validation rather than a completion blocker.
- **Decision record (2026-07-22):** the leading lightweight named-phase hypothesis remains viable;
  no material design reversal is needed. Gate 1 selects the one sequential string-boundary API,
  narrows the original nested-phase wording to explicit rejection/detection, and replaces arbitrary
  retained history with bounded online statistics. This is the smallest design that preserves
  visible student loop order, the one-`LoopClock` behavioral truth, diagnostic/output ownership, and
  reusable framework value. PERF-02 is Ready, but it introduces a public API and instrumentation
  lifecycle, so implementation must not begin until the user explicitly approves this design. No
  framework or robot implementation code has been changed at this gate.
- **Approval (2026-07-22):** the user explicitly approved the PERF-02 sequential phase-profiler
  design. Implementation is limited to the recorded core diagnostic, bounded completed snapshots,
  off-by-default Phoenix composition-root markers/debug rendering, focused tests, and synchronized
  documentation. This approval does not authorize PERF-01, PERF-03, or another tracker item.
- **Implementation record (2026-07-22):**
  - Added the final SDK-free `LoopPhaseProfiler` with the sole public
    `create(boolean enabled)` construction path, private monotonic `System.nanoTime()` source, and
    package-private deterministic test seam. The public loop surface remains the approved flat
    `startCycle(clock)` / `finishPhase(name)` / `finishCycle(clock)` sequence plus distinct
    inactive `reset()`, completed immutable `snapshot()`, and standard `debugDump(...)` capabilities.
    No public stopwatch, constructor, alias, scope, callback, scheduler, or telemetry dependency was
    added.
  - Implemented a fixed 32-name registry with primitive constant-size scratch/statistic storage,
    repeated-name summation, conditional-phase omission, stable first-committed order, online
    latest/average/maximum values, explicit seconds, total/unattributed observations, and immutable
    on-request copies. Disabled creation returns a shared inert profiler/snapshot before validation,
    time reads, lookup, allocation, output, or mutation.
  - Made cycle completion atomic: only a valid `finishCycle` commits statistics. A later cycle
    discards and counts unfinished work, newly encountered names roll back with that discarded
    cycle, same-cycle retries remain rejected after any failed attempt, and obvious clock/cycle/time
    misuse fails actionably. Since resettable numeric cycle values cannot prove a hidden earlier
    reset, the public contract truthfully requires callers to reset the inactive profiler beside
    every deliberate `LoopClock.reset()` rather than claiming universal inference.
  - Added one private false `ENABLE_LOOP_PHASE_PROFILING` constant to `PhoenixRobot`, one profiler,
    and one retained telemetry/null debug sink without changing constructors or `PhoenixProfile`.
    TeleOp and Auto mark their existing high-level synchronous ownership boundaries; the profiler
    resets immediately before the shared clock at FTC START. Completed debug rows join the existing
    Driver Station frame before the sole presenter commit, while the just-measured telemetry phase
    appears on the next frame. `visionReadiness` is deliberately named so it does not claim to time
    asynchronous camera work.
  - Updated the Framework Principles, loop guide, and Phoenix Architecture with the one-clock
    observer rule, copy-pastable retained-sink example, explicit enablement, previous-frame display,
    measurement-window statistics, paired-reset contract, use cases, and wall-time limitations.
- **Adversarial audit (2026-07-22):** independent API/simplicity, correctness/test, and documentation
  reviews reconfirmed one public construction layer, distinct public capabilities, a core package
  with no FTC/Android dependency, fixed literal Phoenix markers, one telemetry commit, and inert
  default behavior. Initial review found that names first seen in an incomplete cycle could consume
  hidden registry capacity, a failed observation could be retried in the same `LoopClock` cycle,
  reset detection wording claimed more than numeric cycle input can prove, Phoenix documentation
  conflated preceding latest data with measurement-window average/max and a full FTC callback, and
  the loop example left its debug sink unexplained. The implementation, regressions, tracker, and
  guides were corrected; focused re-reviews found no remaining issue.
- **Automated verification (2026-07-22):**
  - Focused `LoopPhaseProfilerTest`: 24 deterministic tests, 0 failures, 0 errors, and 0 skipped.
    Coverage includes exact attribution, repeated/conditional phases, online statistics, stable
    order, atomic incomplete recovery and registry rollback, same-cycle retry rejection, explicit
    reset/rebind, equal/regressing/negative-origin/wrapping nanos, exact stopwatch call counts,
    shared disabled behavior, the 32/33 boundary, 10,000-cycle constant storage, immutable old
    snapshots, every stable debug key, a Phoenix-shaped slow owner, and the one-factory surface.
  - Full `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac`: 74 suites and 721 tests,
    with 0 failures, 0 errors, and 0 skipped; compilation succeeded. Output contained only the
    existing Java 21/source-8 and FTC app-shell deprecation warnings.
  - `git diff --check`, tracked/untracked trailing-whitespace scanning, changed-Markdown local-link
    validation, core FTC/Android/Telemetry import checks, one-public-factory/no-public-constructor
    checks, and the Phoenix off-by-default flag check passed.
- **Hardware boundary:** no robot hardware is required to review the API, loop order, output
  ownership, or deterministic accounting. Software tests do not measure enabled profiler overhead,
  timer resolution, OS scheduling, or real Driver Station update cost on a Control Hub. Profiling
  therefore remains off by default, and those physical measurements remain adopting-robot
  validation rather than claims made by PERF-02.
- **Manual verification (2026-07-22):** the user reviewed PERF-02 in Android Studio and replied
  `PERF-02 looks good`. PERF-02 is **Done** and publication is authorized for this item only. No
  hardware was available; enabled Control Hub overhead, timer resolution, and physical Driver
  Station cadence remain adopting-robot validation.

### PERF-03 - Contract-safe hardware write deduplication

- **Problem to confirm:** FTC adapters may resend an unchanged motor/servo command every cycle even
  when the controller already holds it. Avoiding redundant bus writes can improve loop time, but a
  generic cache can suppress a required zero, run-mode transition, watchdog refresh, or truthful
  Plant applied-target update.
- **External evidence:** Cuttlefish wraps motor output with
  [`WriteCache`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/hardware/WriteCache.java).
  That proves the optimization is useful enough to evaluate, not that its threshold and latching
  assumptions satisfy every Phoenix actuator contract.
- **Alternatives to compare:** no change; exact deduplication inside individual FTC adapters;
  tolerance-based motor/servo wrappers; cache at Plant output; or rely on SDK/controller behavior.
  Benchmark each affected hardware type and trace stop, rail/clamp, mode/configuration, watchdog,
  reconnect, and telemetry semantics separately.
- **Leading hypothesis:** make no framework API change without measured benefit. If justified, keep
  caching at the narrow hardware boundary with device-specific invalidation and force-write rules;
  Plants still evaluate and report every cycle, normalized limits remain exact, and safety stop/zero
  is never skipped merely because a stale cache matches.
- **Completion:** before/after measurements are recorded; tests cover first write, unchanged write,
  tiny changes, zero/stop, limits, mode/config changes, invalidation/reconnect, write failure, NaN
  rejection, and Plant applied-target truth. Each supported adapter opts in explicitly; no generic
  source or Task behavior changes.
- **Pause record (2026-07-19):** **Deferred while per-adapter hardware measurements and controller
  behavior evidence are unavailable.** Fake tests cannot justify suppressing a real write or prove
  watchdog, reconnect, and configuration invalidation rules. Keep existing writes unchanged.

### TUNE-01 - Live tuning to checked-in profile

- **Confirmed problem:** live editing is useful while tuning, but mutable Dashboard/Panels fields
  are not checked-in production configuration. Reading them continuously from a match owner can
  change behavior between cycles, exposes a multi-field candidate before the robot has deliberately
  accepted it, and gives the framework no truthful way to know whether accepted values were copied
  into source control. A complete tuning workflow therefore needs an explicit candidate snapshot,
  apply boundary, safe mechanism policy, accepted-value report, and manual record/restart step.
- **Confirmed current behavior:**
  - `PidfRegulator.setGains(kP, kI, kD, kF)` already validates the complete candidate before
    changing any gain. A rejected non-finite candidate leaves all four applied gains unchanged.
    A valid candidate preserves controller history until the robot resets its outermost retained
    regulator composition.
  - The smallest correct post-API-03 apply operation is already:

    ```java
    flywheelPidf.setGains(candidateKP, candidateKI, candidateKD, candidateKF);
    flywheelRegulator.reset();
    ```

    The robot realization owns `flywheelRegulator` because it alone knows whether PIDF is wrapped
    by output limiting, voltage compensation, or another stateful robot policy. Reset does not
    itself command or stop the actuator; the Plant applies the changed law on its next ordinary
    update.
  - Summer26 `master` still points to the reviewed `4eed9d6c` commit. Its 1,047 Java lines under
    `edu.ftcphoenix.robots.betta` are distributed across eight files. The current tuning workflow
    touches seven of them: the 123-line `BettaDashboardControls`, the 202-line composition root,
    52-line TeleOp host, 39-line capability aggregate, 362-line shooter owner, 132-line profile,
    and 84-line controls owner. Only the 53-line Auto task file is unrelated.
  - The 123-line Dashboard class is not 123 lines of reusable PIDF infrastructure. It declares and
    mirrors six vendor statics (enable, target RPM, and four gains), arbitrates Dashboard changes
    against gamepad state, initializes and stops the flywheel request, publishes command telemetry,
    and retains six previous values. Enable, target selection, safe arming, and gamepad precedence
    are Bettabot policy. API-03 already makes the reusable separate-`Pid`/captured-`kF`,
    four-finite-check, and partial-update burden removable.
  - Bettabot currently applies a changed four-gain tuple from production TeleOp and has no explicit
    record/commit step or separate tuning-mode readiness boundary. Cuttlefish independently proves
    that live mutable configuration is useful, but it is not a second Phoenix
    `PidfRegulator` tune/apply/reset/record caller and therefore does not prove a common standard-
    PIDF tester lifecycle.
- **Complete current caller inventory:**
  - No checked-in Phoenix robot, framework tool, or modern compiling example constructs a
    `PidfRegulator`; current production Java references are the regulator and factory
    implementations themselves. `PidfRegulatorTest`, `ScalarRegulatorsApiTest`, and
    `RegulatedPlantSafetyTest` are the concrete executable callers.
  - The Beginner's Guide, FTC Actuators & Plants, Layered Shooter Example, and Recommended Robot
    Design show the retained PIDF plus outer-regulator reset. They do not yet describe explicit
    apply, safe arming, rejected-candidate reporting, rollback choice, profile recording, and
    production restart as one workflow.
  - Bettabot is the only reviewed Phoenix-style adopting caller. Its Dashboard dependency is
    enabled while this repository has Panels through Pedro and no FTC Dashboard dependency.
    Both vendors expose static configuration fields, but their refresh and copy-back behavior is
    vendor-specific; neither supplies a framework-owned atomic four-value candidate or proof that
    source was committed.
- **Public construction, storage, and sibling-family audit:**
  - `ScalarRegulators.pidf(double, double, double, double) -> PidfRegulator` is the sole public
    standard-PIDF construction layer. `PidfRegulator` has no public constructor or class-local
    factory.
  - `PidfRegulator.setGains(...) -> PidfRegulator` is the retained capability used for a live
    complete update. Its four getters expose applied truth. `ScalarRegulator.reset() -> void` is a
    separate composition-lifecycle operation whose correct receiver is robot-owned.
  - No current caller stores, shares, composes, or independently validates a `PidfGains`,
    tuning-session, or profile-assignment object. The robot profile and UI already own the four
    values in their respective domains. Adding an immediately consumed wrapper would repeat those
    answers and reopen the API-03 decision that deliberately rejected such a type.
  - `ScalarTuner` is a one-number gamepad UI component constructed directly for hardware testers.
    Four instances would still require gain ranges/steps, selection/apply choreography, mechanism
    target/enable policy, telemetry, and profile-field names. It is not a missing PIDF tuple owner.
  - `TeleOpTester` and `FtcTeleOpTesterOpMode` already provide a generic explicit tester lifecycle.
    A robot can use that host for a local tuning mode without another framework base class. The
    framework has no public PIDF-tester factory or second PIDF action layer to make parallel.
- **Ordinary student-call comparison:**

  | Design | Ordinary apply site | Conceptual decisions still supplied by robot code |
  |---|---|---|
  | Checked-in profile only | No live apply; rebuild from four checked-in gains. | Four gains and normal mechanism safety. Safest and smallest, but redeploys every trial. |
  | Robot-local explicit tuning mode (selected) | `pidf.setGains(p, i, d, f);` then `outer.reset();` | Four candidate values, explicit APPLY event, safe target/arming policy, the outer composition owner, accepted/last-known-good values, and robot profile field names. |
  | `PidfTuningSession`/`PidfTuners.apply(...)` | One helper call after constructing or passing another owner. | Every decision above plus a tuning-session/helper noun; it removes at most the visible reset statement and cannot infer its correct receiver. |
  | Framework PIDF Plant tester | A builder/factory supplied with PIDF, outer regulator, Plant, writable target, setpoint range/steps, profile labels, and lifecycle hooks. | Every mechanism-specific safety and recording decision, while also granting a second object direct Plant lifecycle ownership. |
  | Dashboard or Panels adapter | Vendor annotations/statics plus an adapter call. | The same six Bettabot meanings and safe mechanism policy, with a second public path or a dependency that one of the two repositories does not carry. |
- **Alternatives rejected:**
  - Documentation-only warnings without a concrete workflow were rejected because they leave the
    current immediate-production-static pattern looking endorsed.
  - Continuously reading mutable fields in production was rejected because there is no explicit
    candidate acceptance, tuple publication contract, match isolation, or source-control truth.
  - Rebuilding a regulator, Plant, subsystem, or robot owner was rejected because a retained
    `PidfRegulator` already updates the standard law without replacing hardware/lifecycle owners or
    losing unrelated configuration.
  - A `PidfGains` value, `PidfTuningSession`, `PidfTuners.apply(...)`, or
    `setGainsAndReset(outer, ...)` convenience was rejected because it wraps the existing two-line
    boundary, repeats four profile answers, cannot discover the outer composition, and has only one
    adopting caller.
  - A full generic PIDF Plant tester was rejected because the framework would need target units,
    safe range and step policy, enable/stop semantics, Plant ownership, mechanism telemetry, and
    profile field names. Those extra answers make the common robot call longer and give a tool
    competing knowledge of realization lifecycle.
  - Dashboard-only, Panels-only, dual-vendor, reflection, and generic live-config adapters were
    rejected. Static annotation discovery and refresh behavior are vendor-specific, and a generic
    facade does not remove the robot's fields or meanings.
  - Automatic profile mutation, Java-source rewriting, a cross-OpMode dirty singleton, and a
    framework match-readiness claim were rejected because runtime code cannot prove that a student
    copied, reviewed, committed, and restarted with a value. A match mode that never reads live
    tuning state needs no artificial readiness flag.
  - Automatic rollback around arbitrary outer decorators was rejected as an untruthful
    transaction. Invalid candidates already leave gains unchanged. A valid but poor candidate is
    rolled back by explicitly reapplying the tester's last accepted or checked-in four-gain tuple;
    any apply/reset/update failure must make the robot-local tuning mode stop its owned mechanism.
- **Selected design:** add no framework runtime type, factory, vendor adapter, dependency, tuning
  singleton, profile writer, or robot base class. Add one vendor-neutral documentation pattern for
  a dedicated tuning/tester mode:
  1. Production TeleOp and Auto construct only from a checked-in defensive profile snapshot and do
     not read live tuning statics.
  2. The tuning mode starts with a zero/disabled request, uses the robot's existing target and
     complete-output bounds, and requires explicit arming.
  3. An explicit APPLY event causes the OpMode thread to read each UI value once, call
     `PidfRegulator.setGains(...)`, and reset the robot-owned outermost regulator before the next
     Plant update. The UI publication mechanism, not Phoenix, must make that candidate coherent;
     Phoenix does not claim cross-thread tuple atomicity.
  4. Rejection is displayed without changing the accepted tuple. Runtime apply/reset/update failure
     disables and stops the tuning mechanism. Rollback is an explicit reapply of a known-good
     tuple, not an inferred hardware transaction.
  5. Telemetry prints the accepted four profile assignments. The student copies them into the
     robot profile, commits the source, stops the tuning mode, and starts a fresh production mode.
- **Bettabot effect:** TUNE-01 itself adds no new shortening API because the post-API-03 reusable
  apply site is already two statements with no extra noun. Bettabot can make its match code safer
  by moving Dashboard fields and tuning arbitration out of `BettaTeleOp` into a dedicated local
  tuning mode, then deleting the Dashboard field/update/stop coupling from production. The exact
  replacement line count depends on Bettabot's chosen UI and safe flywheel controls, so the
  framework must not claim that all 123 Dashboard-owner lines disappear. The decision gate failed
  the required proof that another framework abstraction reduces total robot code and concepts.
- **Framework Principles check:** this keeps one PIDF construction/update surface, leaves outer
  composition and Plant lifecycle with realization, keeps vendor types outside core, makes the
  tester/match boundary explicit, preserves checked-in profile snapshots, and refuses to add an
  abstraction from one caller. The Framework Principles should record the tune -> explicitly
  apply -> record -> restart rule so future tuning features cannot silently turn mutable globals
  into production authority.
- **Bounded documentation implementation after approval:**
  - Add a short vendor-neutral live-PIDF tuning guide under `fw/docs/testing-calibration`, link it
    from that section's README and the existing PIDF examples, and keep every example project-
    neutral.
  - Add the live-tuning authority rule to Framework Principles and tighten `PidfRegulator`
    Javadocs around one-loop candidate snapshots, explicit apply, outer reset, and production
    isolation.
  - Change no runtime Java behavior or public API, add no dependency, and do not add or modify
    Phoenix/Bettabot robot code.
- **Verification plan:** rerun the existing focused API-03 PIDF and regulated-Plant safety suites
  plus TeamCode Java compilation; confirm the public surface is unchanged; verify documentation
  links, code snippets, whitespace, and final newlines; and scan framework source/docs for
  Dashboard, Panels, or external-project coupling. No hardware claim is required because the
  selected change defines a software ownership workflow and adds no actuator behavior.
- **Decision record (2026-07-17):** **Ready for explicit approval, with a material change from the
  leading hypothesis.** The audit did not prove that a new optional tuning owner reduces complete
  robot code. API-03 already owns the reusable validated four-gain update, while the remaining
  candidate UI, arming, target, rollback choice, telemetry, and profile names are robot policy.
  The user approved the documentation-only design with
  `Approve TUNE-01 documentation-only design` on 2026-07-17. Implementation is limited to the
  Framework Principle, vendor-neutral guide and links, and `PidfRegulator` Javadocs recorded above.
- **Implementation and automated verification (2026-07-17):**
  - Added the vendor-neutral `Software PIDF Tuning Workflow.md`, linked it from the testing and
    calibration index plus all four existing standard-PIDF explanations, and added Framework
    Principles section 3.4.5. Production modes use checked-in defensive snapshots; the dedicated
    tuning mode owns explicit arm/apply/stop, and accepted getter values are manually recorded
    before a fresh production start.
  - Tightened `PidfRegulator` Javadocs without changing runtime Java or the public API. The guide
    distinguishes complete-candidate rejection from an outer-reset failure, guards the following
    owner/Plant update, preserves the primary failure through best-effort zero/stop cleanup, and
    emits locale-independent round-trip `double` assignments.
  - `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` passed: 52 suites, 512 tests,
    0 failures, 0 errors, and 0 skipped. Focused evidence includes 18 `PidfRegulatorTest`, 4
    `ScalarRegulatorsApiTest`, and 9 `RegulatedPlantSafetyTest` cases. Only the repository's existing
    Java 8 source/target deprecation warnings were emitted.
  - All relative links in the eight affected Markdown files resolve; code fences are balanced;
    affected files have final newlines and no trailing whitespace; `git diff --check` passes; the
    tracker still has 63 sequential unique items with one matching section each. An affected-Java
    diff scan found zero non-comment changed lines.
  - No external-project reference appears in the affected framework files or elsewhere under
    `edu.ftcphoenix.fw`. A whole-framework tuning-vendor word scan found only two pre-existing
    generic `dashboard` comments in `Pid.java`, with no vendor type or dependency; this item adds no
    Dashboard/Panels coupling.
  - No robot-hardware claim is made or needed because TUNE-01 changes only ownership guidance,
    Javadocs, and navigation.
- **Manual verification (2026-07-17):** the user reviewed the implementation in Android Studio and
  approved it with `TUNE-01 looks good`. Status is **Done**; no hardware validation was required.

### TARGET-01 - Lazy Plant target overlay selection

- **Problem to confirm:** resolving shadowed layers can trigger stateful capture such as
  `holdMeasuredTargetOnEntry()` even when that layer is not selected.
- **Alternatives to compare:** top-down lazy resolution; explicit selection lifecycle hooks; or a new
  selected-only capture source.
- **Leading hypothesis:** lazy highest-priority-first resolution best matches overlay semantics and
  avoids hidden side effects without exposing lifecycle complexity to robot code.
- **Completion:** shadowed target producers are not sampled; every activation gate remains a truthful
  once-per-cycle input; fallback/`addIfAvailable(...)` behavior stays explicit; and measured-hold
  re-entry behavior is covered by tests and documentation.
- **Decision record (2026-07-22):** **Ready for explicit approval.** The leading lazy-selection
  direction remains correct, with two required internal refinements: activation gates must be
  sampled separately from target producers, and both measured-hold implementations must recognize
  a sampling gap as a new entry. This changes observable overlay and hold lifecycle semantics, so no
  framework implementation may be edited until the user approves this design.
  - **Confirmed current behavior:** `OverlayTargetSource.resolve(...)` first resolves the base, then
    walks layers in declaration order. Every enabled lower target therefore runs before a later,
    higher-priority winner. A base `holdMeasuredTargetOnEntry(...)` can latch measurement `A` while
    a higher override actually wins, then later return stale `A` when the override ends instead of
    capturing current measurement `B`. A lower normal `add(...)` layer that returns unavailable also
    breaks the loop before a valid later/higher layer is considered, contradicting the documented
    “later enabled layers have higher priority” rule. There is no focused overlay test today.
  - **Public construction-path audit:** overlay construction has one public layer:
    `PlantTargets.overlay(...)` returns `OverlayBuilder`, repeatable `add(...)` and
    `addIfAvailable(...)` calls declare layers, and `build()` returns the final
    `PlantTargetSource`. The `double`, `ScalarSource`, and `PlantTargetSource` overloads are useful
    lifts of the same concept rather than competing arbitration APIs. Normal `add(...)` and
    `addIfAvailable(...)` remain deliberately distinct because “required target” and “try then fall
    through” are different failure policies. The direct
    `PlantTargets.holdMeasuredTargetOnEntry(...)` factory and planner
    `whenUnavailable().holdMeasuredTargetOnEntry(...)` answer are both supported and must have
    parallel entry meaning. No constructor, `of(...)`, second overlay builder, or selected-only
    target path exists.
  - **Builder-reuse and redundant-layer audit:** every repository caller builds one overlay inline;
    none retains `OverlayBuilder`, calls `build()` twice, or mutates it after build. `add(...)` is a
    genuinely repeatable layer declaration, not a staged single-answer question. TARGET-01 will not
    invent builder sealing/copying semantics or reset children on selection changes. The unrelated
    `fromScalar(...)` alias and other public-surface cleanup remain in CLEAN-01; this item will not
    add siblings to them or remove them opportunistically.
  - **Current callers:** compiled production/reference code has five overlay construction sites:
    three one-layer feed-pulse overlays in Phoenix `ScoringPath`, one in
    `TeleOp_07_SupervisorPoseMechanism`, and one in
    `TeleOp_09_LayeredShooterMechanism`. Their bases are pure current-value sources and their output
    queues are explicitly updated before Plant update, so selected output is unchanged. No compiled
    Java caller currently uses `addIfAvailable(...)` or the standalone measured-hold source.
    Multi-layer priority, explicit fall-through, and measured-hold examples are nevertheless part
    of the supported public contract in `Mechanism Target Planning.md`; the FTC Plant, output queue,
    quickstart, layered-shooter, supervisor, and Phoenix architecture guides also teach overlays.
  - **Ordinary robot code stays the same:** students still decide only the total base, each behavior
    name/gate/target, declaration order, and—only when genuinely needed—whether failure should
    explicitly fall through:

    ```java
    PlantTargetSource feederTarget = PlantTargets.overlay(baseTarget)
            .add("feedPulse", feedPulseQueue.activeSource(), feedPulseQueue)
            .add("eject", ejectRequested, -1.0)
            .build();
    ```

    No lifecycle call, wrapper type, selector object, or second target spelling is added.
  - **Alternatives compared:** documentation-only cannot prevent hidden capture, shadowed failures,
    or the lower-unavailable priority defect. A local change only to measured hold leaves all other
    stateful target producers and priority failure intact. A fully lazy highest-to-low scan of both
    gates and targets is shorter internally, but it would pause lower stateful activation signals,
    debounce/edge progression, and `OutputTaskRunner.activeSource()` updates while a higher layer is
    active; an old pulse could then resume unexpectedly after the override. Robot-local nested
    `choose(...)` or negated gates repeats arbitration and fall-through policy. Public
    `onSelected`/`onDeselected` hooks, an overlay lifecycle interface, `.lazy()` modes, or a new
    selected-only hold source add concepts and shared-source ownership questions. Automatically
    calling child `reset()` when shadowed can clear planners or cancel/empty queue owners and is not
    a selection operation.
  - **Chosen overlay algorithm:** first sample every memoized activation gate exactly once in
    declaration order and retain that per-loop snapshot. Then inspect layers from last to first and
    resolve target producers only as needed. The first enabled producer whose plan has a target
    wins. An enabled normal `add(...)` target that is unavailable terminates with that explicit
    failure; an enabled `addIfAvailable(...)` target records fall-through and continues to the next
    lower layer. Resolve
    the base only when no layer wins or blocks. Lower targets and the base are therefore not sampled
    under a higher winner, while activation facts and queue heartbeats remain current. Selection for
    the loop is frozen before target resolution, avoiding target-side effects that change a later
    gate during the same arbitration pass.
  - **Measured-hold entry semantics:** for the standalone hold, “entry” means its first resolution
    after construction/reset or after at least one `LoopClock.cycle()` in which it was not resolved.
    Continuous-cycle and repeated same-cycle resolutions retain one capture; selection after a
    sampling gap captures the then-current measurement, or the caller's finite fallback if feedback
    is unavailable. The planner's `HOLD_MEASURED` unavailable policy uses the same sampling-gap rule
    for a new unavailable entry, without clearing its unrelated hold-last/request history. General
    target children are never reset merely because another layer wins.
  - **Diagnostics and failure truth:** reset each overlay layer's per-resolution diagnostic state
    before arbitration and distinguish disabled, enabled-but-shadowed, selected,
    explicitly-fell-through, and required-but-unavailable outcomes without resolving a target for
    telemetry. Existing target/gate exceptions remain failures rather than becoming silent
    fall-through. An unavailable lower layer cannot mask a valid higher winner because it is not
    selected; an unavailable selected normal layer cannot silently expose a lower command.
  - **Framework Principles assessment:** this keeps one final `PlantTargetSource`, one Plant sample
    and writer, explicit priority/fall-through, and the single `LoopClock` heartbeat. It preserves
    per-cycle progression for activation sources while making conditional target branches genuinely
    lazy, keeps state and reset ownership in their existing owners, and moves reusable arbitration
    complexity into the framework with no added student-facing noun or call.
  - **Bounded implementation scope:** change only private overlay/hold state in `PlantTargets`, the
    conditional-resolution contract in `PlantTargetSource` Javadocs, focused framework tests, the
    Plant-target section of Framework Principles, `Mechanism Target Planning.md`, and the stale
    “base always runs” wording in `Supervisors & Pipelines.md`. Inspect other overlay snippets for
    semantic accuracy but do not rewrite valid call sites or Phoenix robot behavior. Do not absorb
    TARGET-02 freshness, TARGET-03 candidate mathematics, SOURCE-01 Boolean-combinator semantics,
    CYCLE-01 whole-source memoization, CLEAN-01 aliases, builder-reuse policy, or selection-driven
    queue reset. The audit also found that generic memoized sources mark a cycle before an upstream
    sample succeeds; same-cycle retry behavior is a separate source fail-stop candidate and is not
    changed under TARGET-01.
  - **Verification plan:** add a focused `PlantTargetsOverlayTest` covering all gates sampled once,
    base/lower target laziness, later-layer priority over a lower unavailable layer, required
    unavailable blocking, `addIfAvailable(...)` fall-through chains, base-only fallback, frozen
    gate snapshots, reset propagation, and truthful changing diagnostics. Cover standalone and
    planner measured holds for first capture, continuous cycles, repeated same-cycle resolution,
    shadow/re-entry recapture, unavailable feedback fallback, and explicit reset. Add one real Plant
    integration assertion that only the selected requested target is applied. Run the focused suite,
    full `:TeamCode:testDebugUnitTest`, `:TeamCode:compileDebugJavaWithJavac`, exhaustive caller/API
    searches, documentation-link/whitespace checks, and `git diff --check`. No hardware evidence is
    required because this is deterministic source-selection and lifecycle behavior.
  - **Approval gate:** this preserves every public call signature but changes observable sampling,
    priority, failure shielding, and measured-hold re-entry semantics. Stop before implementation
    and request explicit approval of the TARGET-01 two-phase lazy-overlay design.
  - **Design approval (2026-07-22):** the user approved the TARGET-01 two-phase lazy-overlay design
    with `Approve TARGET-01 two-phase lazy overlay design`. Gate 2 implementation is authorized for
    this item only.
  - **Gate 2 implementation (2026-07-22):**
    - `PlantTargets.overlay(...)` now clears per-overlay diagnostic state, samples every memoized
      activation gate once in declaration order, then resolves target producers from last/highest
      priority to first/lowest. A selected normal unavailable layer blocks; an
      `addIfAvailable(...)` layer records fall-through; the base resolves only when no layer wins or
      blocks. Gate and selected-target exceptions preserve the original failure and never trigger a
      fallback target.
    - Overlay diagnostics retain the existing winner/enabled/fell-through/plan keys and now report
      base/target sampling plus disabled, shadowed, selected, fell-through, required-unavailable,
      gate-failed, and target-failed outcomes. Mutable diagnostic snapshots are owned by each built
      overlay rather than the reusable layer specification. Successful resolution adds no new
      per-loop diagnostic object or dynamic error-message allocation.
    - The standalone measured hold recaptures after a resolution-cycle gap; same-cycle and
      consecutive-cycle resolutions retain one capture. Planner `HOLD_MEASURED` applies the same
      re-entry rule without clearing candidate or hold-last history. Explicit reset forces a fresh
      entry, and the affected stateful sources fail fast when the required `LoopClock` is missing.
    - Synchronized `PlantTargetSource`/`PlantTargets` Javadocs, Framework Principles,
      `Mechanism Target Planning.md`, and `Supervisors & Pipelines.md`. All existing Phoenix and
      example construction calls remain unchanged; no public/protected production signature or
      robot behavior file changed.
  - **Automated verification (2026-07-22):**
    - Focused `PlantTargetsOverlayTest`: 22 tests, 0 failures, 0 errors, and 0 skipped. Coverage
      includes gate order/memoization, frozen snapshots, lazy winner/base behavior, required failure,
      successful and chained optional fall-through, exception shielding/diagnostics, reset
      propagation, standalone/planner measured-hold entry and fallback, hold-last isolation, and a
      real power-Plant application assertion.
    - Full `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` passed: 75 XML suites,
      743 tests, 0 failures, 0 errors, and 0 skipped. Output contained only the repository's existing
      Java-8-on-JDK-21 source/target and deprecated FTC controller sample warnings.
    - `git diff --check`, changed-file trailing-whitespace/final-newline checks, Markdown fence and
      relative-link checks, exhaustive overlay caller search, and an affected-Java public/protected
      signature diff all passed. Three independent final reviews of implementation/performance,
      test validity, and documentation/API simplicity reported no remaining blocker.
    - No robot hardware evidence is required or claimed because the changed contract is fully
      deterministic source selection, loop-cycle re-entry, and diagnostics behavior.
  - **Manual verification scope:** inspect the two-pass loops, `add(...)` versus
    `addIfAvailable(...)` branches, measured-hold gap/reset state, per-overlay debug states, focused
    tests, and synchronized docs in Android Studio. Confirm that the five existing Phoenix/tool
    overlay calls are unchanged. No physical robot run is needed for this item.
  - **Manual verification (2026-07-22):** the user reviewed TARGET-01 in Android Studio and replied
    `TARGET-01 looks good`. TARGET-01 is **Done**, and this approval authorizes Gate 3 finalization,
    publication, and merge for this item only. No robot-hardware validation was required or claimed.

### TARGET-02 - Candidate freshness

- **Problem to confirm:** a cached candidate with a fixed `ageSec` can remain fresh forever even when
  it also supplies an old timestamp.
- **Alternatives to compare:** compute age entirely from timestamp; use the maximum of reported and
  clock-derived age; or require one canonical freshness representation.
- **Leading hypothesis:** validate quality/age and compute a conservative effective age while keeping
  compatibility with candidates that cannot provide timestamps.
- **Completion:** stale cached candidates fail their gates; invalid quality, age, and timestamp values
  fail early or follow one documented unavailable policy.
- **Decision record (2026-07-22):**
  - **Confirmed behavior:** every metadata-bearing `PlantTargetRequest` factory delegates to the
    matching `PlantTargetCandidate` factory. Candidate construction validates target value and an
    explicit period, but stores `quality`, `ageSec`, and `timestampSec` unchanged. The planner's
    `candidatePassesGate(...)` compares only `candidate.quality` and the fixed
    `candidate.ageSec`; `timestampSec` is never read. For example, one cached request with
    `ageSec = 0.02`, `timestampSec = 1.0`, and a configured maximum age of `0.20` is still selected
    when `clock.nowSec() == 10.0`, because only `0.02 <= 0.20` is evaluated. The resulting
    `PlantTargetPlan` also copies `0.02`, so diagnostics incorrectly continue to report it as young.
  - **Malformed-metadata path:** positive-infinite quality and negative age can currently pass;
    timestamp infinities are accepted and ignored; a `NaN` age is ignored entirely when the optional
    maximum-age gate is absent; and `minQuality(...)` accepts `NaN`, infinities, negative values, or
    values above one. If no candidate passes, the planner already routes through the required
    `whenUnavailable()` answer: report unavailable, fixed fallback, hold last, or hold measured.
    That existing policy is the correct fail-closed boundary for bad live observation data.
  - **Current callers:** exhaustive production, Phoenix, tool, modern-example, test, and Markdown
    searches found no Phoenix/tool/example Java caller of `PlantTargets.plan()`,
    `PlantTargetRequest`, or `PlantTargetCandidate`. Phoenix Architecture describes a future turret
    use. The student-facing executable-shaped calls are in `Mechanism Target Planning.md`; its
    spatial example forwards `facing.quality()`, `facing.ageSec()`, and
    `facing.timestampSec()`. Existing tests contain three planner calls for TARGET-01 measured/last
    hold lifecycle only; none exercises candidate metadata or the acceptance branch.
  - **Construction-path and distinct-capability audit:**
    - `PlantTargets.plan()` is the only planner construction entry. Its staged path is
      `request(Source<PlantTargetRequest>)`, one of four direct preference answers, one of two direct
      unreachable answers, the optional multi-setting `accept()` branch, and one of four terminal
      unavailable answers. There is no public planner constructor or `of(...)` factory.
    - `Source<PlantTargetRequest>` and `PlantTargetRequest` have independent value: robot services
      can store, compose, and repeatedly evaluate a request source outside the one builder step.
      They are not immediately consumed selector wrappers, so replacing `request(...)` with several
      stage-specific target methods would lose dynamic multi-candidate composition.
    - `PlantTargetRequest.*` is the concise one-candidate construction layer.
      `PlantTargetCandidate.*` is the necessary multi-candidate layer used with `oneOf(...)`; a
      dynamic inventory caller stores candidates in a list before constructing the request. The
      varargs and `List` request forms respectively support literal and accumulated candidate sets.
      These layers therefore provide distinct capabilities and are retained.
    - Exact, equivalent-position, periodic, relative-equivalent, and relative-periodic candidates
      have simple factories; every sensed form except exact `relative(...)` also has a
      quality/age/timestamp overload. A relative exact observation currently cannot participate in
      freshness/quality gating at all. Parallel overloads on `PlantTargetCandidate.relative(...)`
      and `PlantTargetRequest.relative(...)` provide that missing capability without a new type or
      construction layer.
  - **Ordinary call-site comparison:**
    - **Recommended conservative design (unchanged existing call):**
      `PlantTargetRequest.equivalentPosition(sourceId, targetDeg, quality, reportedAgeSec,
      observationTimestampSec)`. The student supplies one candidate family/value and forwards three
      observation facts. The planner configuration separately supplies the two policy answers once:
      maximum accepted age and minimum accepted quality. Reported age is acquisition/solve latency
      at publication; timestamp is stable observation identity in the `LoopClock` timebase and lets
      the framework continue aging a cached result. They are independent evidence, not two policy
      answers.
    - **Timestamp-only candidate:** shortens the advanced factory by one scalar, but age-only sensors
      must synthesize timestamps in robot code and sources that cannot prove a timestamp lose their
      usable age fact. It also discards the conservative check when reported latency and clock math
      disagree.
    - **`CandidateFreshness`/metadata wrapper:** names the alternatives but adds a public noun that
      ordinary code constructs only to pass immediately, makes candidate lists longer, and does not
      improve ownership or correctness beyond the planner-local rule.
    - **Separate age-only and timestamp-only factory families:** creates parallel spellings and asks
      robot code to branch when a source sometimes has a timestamp. It cannot use both facts
      conservatively.
  - **Alternatives considered:**
    - Documentation-only or continuing to trust `ageSec` leaves the confirmed stale-cache failure.
    - A smallest local `max(ageSec, nowSec - timestampSec)` comparison fixes the main trace but is
      incomplete by itself: malformed metadata can still pass, future timestamps can rejuvenate a
      result, `minQuality` remains invalid, and selected-plan diagnostics remain misleading.
    - Timestamp-only canonical freshness is mathematically attractive but makes existing age-only
      sources harder to adapt and moves timebase conversion into robot code.
    - Tracking timestamp-less candidates by object identity, candidate ID, or first-seen time was
      rejected because none proves that two returned candidates represent the same observation.
    - Throwing while constructing a candidate was rejected for live metadata. A bad sensor sample
      should reject that candidate and allow another candidate or the already-required unavailable
      policy to keep the mechanism safe; it should not crash an OpMode. Invalid builder tuning is
      programmer configuration and should still fail immediately with an actionable exception.
    - Renaming or restructuring the established `accept()` branch was rejected as unrelated API
      churn. It already has distinct value as a multi-setting optional tuning branch.
  - **Chosen design:**
    1. Keep all existing request/candidate calls and planner stages. Add only the two missing
       `relative(id, delta, quality, ageSec, timestampSec)` overloads, one at each justified
       construction layer.
    2. Validate each candidate at planner resolution before geometry: quality must be finite and in
       `[0, 1]`; reported age must be finite and non-negative; timestamp must be `NaN` for unknown or
       finite in the current `LoopClock` timebase. A timestamp more than `1e-6` seconds in the future
       is invalid; a difference within that numeric tolerance contributes zero derived age.
    3. When a timestamp is known, compute
       `effectiveAgeSec = max(reportedAgeSec, max(0, clock.nowSec() - timestampSec))`. With a `NaN`
       timestamp, use the reported age. Apply the configured maximum to that effective age with the
       current inclusive boundary; apply minimum quality inclusively. Invalid, stale, or low-quality
       candidates are skipped, so a later valid candidate remains eligible. If none wins, use the
       caller's existing explicit unavailable policy with an actionable first-rejection reason.
    4. Validate `minQuality(...)` as finite and in `[0, 1]`; retain the existing finite,
       non-negative validation for `maxRequestAgeSec(...)`.
    5. Preserve the candidate's quality and original timestamp in `PlantTargetPlan`, but make
       `selectedAgeSec()` report the effective age actually used by selection. Use an internal plan
       construction path rather than adding another public metadata type or changing the existing
       public `PlantTargetPlan.planned(...)` signature.
    6. A metadata-free candidate remains ordinary current robot intent (`quality = 1`, age zero,
       unknown timestamp). With no configured maximum age, the planner does not invent a staleness
       limit, though it still rejects malformed metadata and reports effective age. A timestamp-less
       observation remains compatible, but its producer must update reported age whenever it is
       returned; the framework cannot truthfully age a cached value that supplies no capture time.
    7. Planner/source reset remains paired with a deliberate `LoopClock` reset. TARGET-02 will not
       invent clock-rollback recovery, change same-cycle memoization, alter candidate preference/tie
       ordering, change overlay fall-through, or implement TARGET-03 periodic mathematics.
  - **Framework Principles and simplicity check:** the planner owns current-loop freshness because
    it alone has both `LoopClock` and acceptance policy; producers own immutable observation facts;
    and the existing unavailable branch owns safe behavior after rejection. The ordinary simple
    factories remain two arguments, the advanced spatial mapping remains the same five-argument
    forwarding call, and no new public noun or builder question is introduced. Runtime data fails
    closed without hiding other valid candidates; configuration fails fast. Diagnostics report the
    fact actually used. The two added relative overloads satisfy a real missing capability rather
    than symmetry alone.
  - **Bounded implementation scope:** update `PlantTargetCandidate`, `PlantTargetRequest`,
    `PlantTargetPlan`, and planner internals/Javadocs in `PlantTargets`; add one focused
    `PlantTargetsPlannerFreshnessTest`; synchronize the concise Plant-target section in Framework
    Principles and the freshness contract/spatial example in `Mechanism Target Planning.md`.
    Phoenix robot Java, tools, existing examples, spatial/source ownership, other freshness APIs,
    and later tracker items remain unchanged.
  - **Verification plan:** focused tests will cover a cached timestamped candidate aging through the
    exact inclusive boundary; reported-versus-derived conservative maxima in both directions;
    timestamp-absent fresh/stale compatibility; no configured age limit; quality thresholds;
    invalid quality/age/timestamp and future timestamps; stale/invalid-first plus valid-later
    selection; declaration-order ties; all four unavailable policies and fresh/stale recovery;
    same-cycle memoization, next-cycle recomputation, reset, zero-delta and large time jumps;
    truthful selected effective age/original timestamp; builder validation; and both new relative
    overloads. Then run the focused suite, full TeamCode unit tests and compilation, public-signature
    and exhaustive caller audits, Markdown links/fences, whitespace checks, and `git diff --check`.
    No robot hardware evidence is needed because the contract is deterministic loop-clock and value
    selection behavior.
  - **Approval gate:** candidate acceptance semantics and the relative metadata overloads are public
    API decisions. Stop before implementation and request explicit approval of the TARGET-02
    conservative-freshness design.

- **Reopened decision (2026-07-22; supersedes the conservative-freshness choice above):** the user
  challenged why TARGET-02 should retain timestamp-less age. A second exhaustive source/caller/API
  audit found no concrete reason to do so, and the leading hypothesis therefore changed materially.
  The earlier approval request is withdrawn; this revised design requires fresh approval.
  - **Capability evidence:** no production, Phoenix, tool, example, or test Java caller constructs a
    metadata-bearing target candidate/request. The documented spatial path already receives a
    `LoopClock`-domain timestamp. `PoseEstimate` requires a timestamp; the absolute-pose and AprilTag
    spatial lanes forward timestamps; webcam detections expose a frame acquisition time; and the
    Limelight owner has both a stable result timestamp/identity and the loop clock. A source that
    natively reports age can convert it once, while creating its immutable snapshot, with
    `observedAtSec = clock.nowSec() - sampledAgeSec` and retain that timestamp thereafter.
  - **Why age-only is rejected:** age is relative to the instant at which it was sampled. If a
    producer caches a candidate whose fixed age is `0.02`, the planner cannot distinguish that old
    observation from a newly sampled one. Recomputing `now - age` in each consumer merely gives the
    cached observation a new timestamp every loop. Candidate identity, ID, object identity, and
    first-seen tracking also cannot prove whether two values are the same observation. The source or
    adapter that owns acquisition is the only layer able to answer that question once and retain it.
  - **Ordinary robot-code comparison:** persistent/current robot intent stays concise and has no
    freshness metadata:

    ```java
    PlantTargetRequest.equivalentPosition("slot-2", 240.0);
    ```

    An observation-derived request says so explicitly and forwards one freshness fact rather than
    two overlapping values:

    ```java
    PlantTargetRequest.observedEquivalentPosition(
            facing.sourceId(), targetDeg, facing.quality(), facing.timestampSec());
    ```

    The acceptance policy likewise describes what it limits:

    ```java
    .accept()
        .maxObservationAgeSec(0.20)
        .minQuality(0.45)
        .doneAccept()
    ```

  - **Revised chosen design:**
    1. Keep the existing short geometry factories for timeless/current robot intent. Replace every
       public quality/age/timestamp overload with a parallel `observed...` factory that accepts only
       quality and one stable observation timestamp in the current `LoopClock` timebase. Provide the
       same six pairs on both justified construction layers: exact, equivalent position, explicit
       periodic, relative exact, relative equivalent position, and relative explicit periodic.
       This both supplies the currently missing observed-relative-exact capability and keeps the
       `PlantTargetRequest` one-candidate convenience parallel with `PlantTargetCandidate` lists.
    2. Mark observation-derived candidates internally. Do not use `NaN` alone to distinguish them
       from timeless intent: an invalid timestamp supplied to an `observed...` factory must be
       rejected, not silently reclassified as timeless. Remove candidate `ageSec`; do not retain or
       deprecate the old age-bearing overloads because there is no repository migration cost or
       demonstrated external compatibility requirement.
    3. At planner resolution, require observed quality to be finite and in `[0, 1]` and the timestamp
       to be finite and no more than `1e-6` seconds in the future. Derive age exactly once as
       `max(0, clock.nowSec() - observationTimestampSec)`. Apply minimum quality and renamed
       `maxObservationAgeSec(...)` gates inclusively. Invalid, stale, or low-quality observations
       skip to the next candidate; if none wins, the existing explicit `whenUnavailable()` policy
       remains the single fail-closed answer. Invalid builder thresholds still fail immediately.
    4. Timeless candidates have implicit quality `1.0` and are not subject to an observation-age
       limit. A valid old observation is eligible when the caller deliberately configures no maximum
       observation age; the framework does not invent that policy. Malformed observed metadata is
       always unavailable, even without an optional gate.
    5. `PlantTargetPlan.selectedAgeSec()` reports the selection-time age derived by the planner and
       `selectedTimestampSec()` preserves the observation timestamp; both report `NaN` when timing
       does not apply to a timeless candidate. Make candidate-plan construction framework-internal
       so custom sources cannot create a `PLANNED_CANDIDATE` plan while bypassing the one freshness
       calculation. Public custom sources retain exact, fallback, hold, and unavailable plans; public
       candidate selection has one supported route through `PlantTargets.plan()`.
    6. Keep source/adapter timestamp anchoring as an ownership contract, not robot-code boilerplate.
       TARGET-02 will document that an age-native reusable owner converts age once when publishing a
       snapshot. It will not add a metadata wrapper, parallel age/timestamp factory families,
       candidate identity heuristics, an implicit default staleness limit, or unrelated vision API
       restructuring.
  - **Alternatives reconsidered:** keeping both age and timestamp makes every sensed call answer the
    same freshness question twice and asks the planner to reconcile disagreement that the producer
    should resolve. A timestamp-only same-name overload is shorter in autocomplete but obscures the
    important distinction between timeless intent and observed data; explicit `observed...` names
    retain the same number of factory paths while making misuse visible. Throwing from an
    `observed...` factory for malformed live metadata would force robot code to prevalidate or catch
    sensor faults, so runtime metadata instead follows the planner's safe unavailable policy. A
    `CandidateFreshness` wrapper adds a noun used only to pass two scalars immediately.
  - **Framework Principles and simplicity check:** this design answers freshness once at the owner
    that can prove acquisition time, uses the framework's single `LoopClock` timebase, does not infer
    observation identity, deletes overlapping legacy paths, keeps sibling factories parallel, and
    leaves timeless robot intent at its existing two/three-argument call. Sensor-to-target robot code
    becomes one scalar shorter than the current documentation and cannot accidentally rely on a
    non-aging age value when it uses the explicitly observed path.
  - **Revised implementation/verification scope:** update `PlantTargetCandidate`,
    `PlantTargetRequest`, candidate-plan construction, planner internals/Javadocs in `PlantTargets`,
    Framework Principles, and `Mechanism Target Planning.md`; add a focused
    `PlantTargetsPlannerFreshnessTest`. Verify observed/timeless factories and their parallel API,
    cached timestamp aging through the inclusive boundary, invalid/future timestamps, quality,
    later-candidate recovery, all unavailable policies, same-cycle/reset behavior, truthful plan
    diagnostics, builder validation, caller/signature audits, focused/full TeamCode tests and
    compilation, documentation fences/links, and whitespace. No robot hardware evidence is needed.
  - **Revised approval gate:** this removes and renames public factories/policy and narrows public
    candidate-plan construction. Per the framework-improvement workflow, stop before implementation
    and request explicit approval with `Approve TARGET-02 timestamp-canonical freshness design`.
  - **Design approval (2026-07-22):** the user replied
    `Approve TARGET-02 timestamp-canonical freshness design`. This authorizes Gate 2 implementation
    of the revised bounded scope above for TARGET-02 only; it does not authorize TARGET-03.
  - **Implementation (2026-07-22):** the approved timestamp-canonical design is complete.
    - `PlantTargetCandidate` and `PlantTargetRequest` now expose the same six ordered
      timeless/`observed...` factory pairs. Observed factories accept geometry followed by quality
      and one stable `LoopClock` timestamp. Age fields and every age-bearing overload were deleted,
      and the missing observed-relative-exact path was added at both justified construction layers.
    - The planner distinguishes observed candidates internally, validates live quality/timestamp
      metadata, derives age from the current clock, applies inclusive minimum-quality and renamed
      `maxObservationAgeSec(...)` tuning, preserves declaration-order scoring, skips rejected
      candidates, and reports the first actionable rejection only if no candidate wins. Invalid
      tuning still fails immediately.
    - Candidate-plan construction is package-private and used only by `PlantTargets`; public custom
      sources retain distinct exact/fallback/hold/unavailable plan factories. Selected observation
      diagnostics retain timestamp and derived age, while timeless candidates and all non-observation
      plans report `NaN` timing rather than a misleading zero.
    - Framework Principles, the mechanism-planning guide, maintainer builder notes, Javadocs, and the
      spatial example now use the observed/timeless vocabulary and one-time timestamp anchoring rule.
  - **Adversarial/API audit:** independent reviews found and resolved non-observation plans claiming
    zero age, one reversed sibling-factory declaration order, and imprecise unavailable-policy
    wording. A separate existing upstream risk remains: `AprilTagDetections.frameTimestampSec(...)`
    may re-anchor an age-only cached snapshot. That is not repairable inside the target planner and
    was intentionally excluded by the approved scope, so proposed high-priority VISION-02 now owns
    the acquisition/vision-boundary decision rather than silently expanding TARGET-02.
  - **Automated verification (2026-07-22):** `PlantTargetsPlannerFreshnessTest` has **13 tests** and,
    together with the 22 existing overlay regression tests, reports **35 tests, 0 failures, 0 errors,
    0 skipped**. The complete `:TeamCode:testDebugUnitTest` result reports **756 tests, 0 failures,
    0 errors, 0 skipped** across 76 suites. `:TeamCode:compileDebugJavaWithJavac` passes. Exhaustive
    public-factory/caller searches found six parallel pairs per retained layer, exactly one internal
    `PlantTargetPlan.planned(...)` caller, no age-bearing target factories, and no old
    `maxRequestAgeSec(...)` use. Markdown fences, changed-file trailing whitespace,
    and `git diff --check` pass. Gradle emits only the existing Java 8 source/target deprecation and
    FTC sample deprecation warnings.
  - **Hardware scope:** no physical run is useful for this item; candidate acceptance is deterministic
    `LoopClock` value-selection behavior. VISION-02 separately owns truth at webcam/Limelight
    acquisition boundaries.
  - **Android Studio audit point:** inspect the observed/timeless factory pairs, planner validation
    and age derivation, plan diagnostics, focused tests, and synchronized guide/principle wording.
    Stop without staging, committing, publishing, or starting VISION-02 until the user replies
    `TARGET-02 looks good`.
  - **Manual verification (2026-07-23):** the user reviewed TARGET-02 in Android Studio and replied
    `TARGET-02 looks good`. TARGET-02 is **Done**, and this approval authorizes Gate 3 finalization,
    publication, and merge for this item only. It does not authorize starting VISION-02.

### TARGET-03 - Periodic planner complexity

- **Problem to confirm:** candidate enumeration between bounds can take unbounded loop time for a
  large range and small period.
- **Alternatives to compare:** direct floor/ceil/round candidate calculation for each preference, or
  a strictly bounded local enumeration around the ideal index.
- **Leading hypothesis:** constant-time analytic candidates are both simpler and safer.
- **Completion:** results match existing intended preferences at boundaries and ties without loops
  proportional to the target range.
- **Decision record:** _Pending._

### CYCLE-01 - Stateful drive-source cycle safety

- **Problem to confirm:** repeated same-cycle reads of stateful overlays/guidance may advance blend or
  other state more than once, and reset propagation is inconsistent.
- **Alternatives to compare:** memoize every stateful wrapper internally; require callers to wrap with
  `memoized()`; or centralize sampling in the lane.
- **Leading hypothesis:** abstractions that own state must protect their own per-cycle contract;
  requiring students to remember an external wrapper is too fragile.
- **Completion:** repeated reads in one cycle are observationally identical and reset ownership is
  explicit, tested, and non-destructive to shared owners.
- **Decision record:** _Pending._

### CYCLE-02 - Localization cycle safety

- **Problem to confirm:** duplicate predictor/estimator updates in one cycle can consume sensor deltas
  twice or replace real motion with a zero delta.
- **Alternatives to compare:** guards at every stateful owner; one guard at the public lane; timestamp
  deduplication; or a combination at integration boundaries.
- **Leading hypothesis:** public owners should be idempotent by cycle, with timestamp checks as
  defense against duplicated sensor samples.
- **Completion:** predictor, fusion estimator, and lane tests cover duplicate cycle and duplicate
  timestamp behavior without suppressing legitimate next-cycle updates.
- **Decision record:** _Pending._

### SOURCE-01 - Boolean composition sampling

- **Problem to confirm:** Java short-circuiting prevents a stateful right-hand source from being
  sampled, delaying debounce/readiness chains or missing edges.
- **Alternatives to compare:** eager sample both operands; document that stateful operands must be
  memoized/sampled separately; or expose distinct eager composition methods.
- **Leading hypothesis:** Boolean source composition should sample each operand once per cycle because
  that is the least surprising framework-level behavior.
- **Completion:** stateful operands advance once per cycle regardless of the left value; truth-table
  behavior remains unchanged.
- **Decision record:** _Pending._

### API-01 - Writable Plant command binding

- **Problem to confirm:** a Plant built from an arbitrary final `PlantTargetSource` may register a
  `ScalarTarget` that the final graph does not actually read, so a Plant task can silently write to a
  disconnected command.
- **Why the prior interface proposal is not accepted yet:** `WritablePlant` and `FeedbackPlant` could
  catch misuse at compile time, but they also add types to staged builder return values and concepts
  to beginner code. The Framework Principles explicitly reject capability splitting done only for
  mechanical SOLID compliance.
- **Alternatives to compare:**
  1. Keep one `Plant`; allow writable tasks only when the builder itself created or directly bound
     the exact `ScalarTarget`.
  2. Keep one `Plant`; introduce a small command-binding value produced by a target graph builder so
     provenance is provable for composed sources.
  3. Make task factories take an explicit `ScalarTarget`, using the Plant only for feedback/status.
  4. Add capability interfaces such as `WritablePlant`/`FeedbackPlant` and propagate them through the
     staged builder.
- **Leading hypothesis:** prefer option 1 for the common path and option 2 only if composed writable
  overlays need it. Use capability interfaces only if caller analysis shows that provenance cannot
  be made safe without them. Do not make students select or cast Plant subtypes.
- **Completion:** a writable task cannot compile/build against a disconnected command target; normal
  beginner examples still declare and use `Plant` with no extra ceremony; complex composition has
  one documented path.
- **Decision record:** _Pending._

### API-02 - Feedback tolerance choice

- **Problem to confirm:** position and velocity tolerance defaults are expressed in public Plant
  units, so mapping ticks to inches/degrees/RPM can make a magic default meaningless.
- **Alternatives to compare:** require tolerance after mapping; derive a native tolerance then map it;
  retain defaults only for native units; or remove tolerance from construction and require it in
  feedback tasks.
- **Leading hypothesis:** ask once after units are known, with an explicit native-units shortcut if it
  demonstrably improves the beginner path without hiding a safety/completion decision.
- **Completion:** every feedback Plant has an intentional, unit-documented tolerance and examples use
  meaningful robot units.
- **Decision record:** _Pending._

### API-03 - PID and linear-PIDF configuration ownership

- **Confirmed problem:** `Pid` currently accepts `NaN` and infinity for every gain and accepts
  non-finite or reversed explicit integral/output limits. Invalid gains can therefore produce
  non-finite commands during the loop, while reversed limits are silently reordered by
  `MathUtil.clamp(...)`. The numeric `ScalarRegulators.pidf(controller, kF)` overload also accepts a
  non-finite static feedforward coefficient. A final Plant or output guard can stop an invalid
  command, but that is too late to explain which controller setting was wrong.
- **Reopened ownership problem (2026-07-16):** Bettabot uses the function PIDF overload solely to
  capture a mutable robot-owned `velocityKf`. The framework therefore cannot validate or atomically
  update the complete four-gain candidate, and Bettabot retains a field, lambda, validation, and
  update-order policy for a standard linear `kF * setpoint` controller. The user's question exposed
  that this is an accidental escape hatch, not evidence that live linear `kF` belongs in robot code.
  Plain `Pid` still cannot own `kF` because its narrow `update(error, dtSec)` contract never receives
  the setpoint; a setpoint-aware `ScalarRegulator` can.
- **Current caller evidence:** modern in-repository callers already use
  `Pid.withGains(...).setIntegralLimits(...).setOutputLimits(...)`. The only direct
  `new Pid(...)` production call is internal to `ScalarControllers.proportional(...)`. Bettabot's
  pinned
  [`BettaShooter`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L209-L271)
  repeats finite checks for all four gains, retains a separate mutable `kF`, and coordinates a
  three-gain `Pid` update with that field during live tuning. This is reusable standard PIDF
  validity and ownership, not shooter policy.
- **Complete modern caller inventory:** Phoenix production currently has no direct `Pid` caller.
  `TeleOp_08_LiftExternalSensorControl` is the sole compiling tool example; it uses the factory and
  an output-limit setter. `ScalarRegulatorsTest` is the sole current unit-test caller and uses the
  same factory plus the function PIDF overload. The Beginner's Guide, FTC Actuators & Plants,
  Recommended Robot Design, Layered Shooter Example, and the `Pid`, `ScalarControllers`,
  `ScalarRegulator`, and `ScalarRegulators` Javadocs all use or describe the factory form.
  `ScalarControllers` contains the only implementation call to the public constructor.
  Repository-wide caller search found no stored PID parameter wrapper and no other direct
  construction path.
- **Public construction and reuse audit:**
  - `new Pid(kP, kI, kD)` and `Pid.withGains(kP, kI, kD) -> Pid` are two spellings of the same
    construction. The constructor has no distinct capability or modern direct caller; there is no
    `of(...)` factory or builder.
  - `Pid.setGains(...)` is not a duplicate construction path. Bettabot's live-tuning workflow needs
    to update the controller object already retained by its regulator, so the operation remains for
    PID-only callers.
  - `ScalarRegulators.pid(PidController) -> ScalarRegulator` remains the custom error-controller
    adapter. `setpointFeedforward(ScalarRegulator, DoubleUnaryOperator) -> ScalarRegulator` remains
    the explicit advanced decorator for nonlinear, dynamic, or custom-regulator feedforward.
  - The current `pidf(PidController, double|DoubleUnaryOperator)` overloads do not retain distinct
    value after the standard four-gain factory exists. The numeric form creates a second spelling of
    standard linear PIDF; the function form is exactly
    `setpointFeedforward(pid(controller), function)`. Both will be deleted without deprecation.
  - `voltageCompensated(...)` and `outputLimited(...)` remain distinct outer decorators. They can
    wrap PID, standard PIDF, or custom regulators and preserve visible inside-to-outside command
    ordering.
  - `ScalarControllers.pid(ScalarSource|double, ScalarSource, PidController)` and
    `proportional(ScalarSource|double, ScalarSource, double)` return a cycle-memoized
    `ScalarSource`. This is a different lifecycle/output seam from `ScalarRegulator`; the
    constant-setpoint overload removes a commonly repeated `ScalarSource.constant(...)` without
    adding ambiguity.
  - `PidController` is a deliberately stored/composed interface for custom stateful error
    controllers. It is not a staged-builder parameter or a second PID-construction spelling.
  - FTC `velocityPidf(...)` and `innerVelocityPidf(...)` staged answers remain distinct: they program
    an SDK motor controller and select a device-managed Plant path. API-03 owns Phoenix software
    regulation; FTC-02 separately owns validation at that hardware boundary.
  - Bettabot demonstrably retains the standard PIDF object across Plant composition, live tuning,
    reset, status, and diagnostics. A specialized public return type therefore has independent
    capability value. Its nonpublic constructor does not create a second public construction layer.
  - No caller independently stores or reuses a PID/PIDF gains parameter bundle outside its existing
    robot profile. A new `Pid.Config`, `PidfGains`, constrained-number type, or builder would wrap
    four conventional arguments only to pass them immediately into the factory or setter.
- **Alternatives compared:**
  - Documentation-only warnings were rejected because invalid static values would still enter the
    loop and every robot would still need to remember the same checks.
  - The smallest local fix—validate only `Pid.withGains(...)`—was rejected as incomplete: it leaves
    the equivalent constructor, live setter, explicit limit setters, and numeric PIDF overload as
    holes in the same controller/regulator family.
  - Robot-owner-only validation was rejected because every robot would repeat the same finite-gain
    and ordered-limit checks, and a simpler robot could omit them.
  - Plant or hardware-boundary validation alone was rejected because it loses the setting name and
    fails only after the control loop starts.
  - One broad validation utility or constrained numeric/config type was rejected because it hides
    ownership and adds student-facing nouns without demonstrated reuse.
  - Putting `kF` on `Pid` or widening `PidController.update(...)` was rejected because an
    error-centric controller never receives the setpoint needed for `kF * setpoint`.
  - Keeping Bettabot's captured lambda was rejected because the robot must then own a mutable field,
    validation, and partial-update ordering for a standard framework control law.
  - A mutable coefficient Source/handle was rejected because PID and `kF` would still have separate
    owners and could not be validated as one update. Rebuilding the regulator or Plant was rejected
    because it complicates hardware/lifecycle ownership and discards state.
  - A public PIDF interface plus private implementation was rejected because `ScalarRegulator`
    already is the custom-control-law extension seam. A public final retained capability with a
    nonpublic constructor parallels final `Pid` implementing `PidController`, uses one less public
    abstraction, and guarantees the built-in validation contract.
  - Validating servo mappings, calibration, motor-group topology, FTC device-managed tuning, and
    every owner config in the same item was rejected by the scope gate. Those answers have different
    units, policies, and earliest fully informed owners.
- **Selected public API:**
  - Make the `Pid` constructor private. `Pid.withGains(kP, kI, kD)` is the only public PID
    construction path; no deprecated constructor or compatibility alias remains.
  - Add exactly
    `ScalarRegulators.pidf(double kP, double kI, double kD, double kF) -> PidfRegulator`
    for the standard Phoenix software control law
    `PID(setpoint - measurement) + kF * setpoint`.
  - `PidfRegulator` is a public final class implementing `ScalarRegulator`, with a nonpublic
    constructor and no class-local factory. The retained type exposes only
    `setGains(kP, kI, kD, kF)`, `setIntegralLimits(min, max)`,
    `setPidOutputLimits(min, max)`, and current-gain accessors `getKP/getKI/getKD/getKF` beyond the
    inherited regulation, reset, and debug capabilities.
  - Rename `Pid.getkP/getkI/getkD` to `getKP/getKI/getKD` for parallel spelling and delete the old
    methods instead of adding aliases. `Pid.setGains(...)`, `setIntegralLimits(...)`,
    `setOutputLimits(...)`, and `getIntegral()` remain distinct PID-only capabilities.
  - Delete both old `ScalarRegulators.pidf(PidController, double|DoubleUnaryOperator)` overloads.
    Advanced custom or nonlinear feedforward remains explicit through
    `setpointFeedforward(ScalarRegulators.pid(customController), function)`. Add no
    `ScalarControllers.pidf(...)` sibling merely for symmetry.
- **Control and limit semantics:**
  - Standard PIDF computes `error = setpoint - measurement`, evaluates the internal PID with
    `clock.dtSec()`, computes `feedforward = kF * setpoint`, and returns
    `pidOutput + feedforward`. Signed finite gains are valid. `kF` is command per plant-setpoint
    unit; it is not a static-friction, acceleration, gravity, or FTC SDK motor-controller term.
  - `PidfRegulator.setPidOutputLimits(...)` deliberately limits only `P + I + D` before adding
    feedforward. `ScalarRegulators.outputLimited(...)` still limits the complete regulator passed to
    it, including PIDF, voltage compensation, and any inner robot policy. These are distinct
    decisions: with PID output `-2.0` and feedforward `+1.5`, an inner `[-1,+1]` limit produces
    `+0.5`, while omitting it produces `-0.5` before any outer `[0,0.65]` limit.
  - All gains must be finite. All explicit integral and output bounds must be finite and ordered.
    Integral bounds must also include zero so `reset()` can truthfully clear the integral
    contribution; one-sided, symmetric, and asymmetric ranges containing zero remain valid, with
    `[0,0]` the only equal-bound case.
  - A rejected factory or setter validates every argument before mutation. A valid narrower
    integral range immediately reclamps an existing finite integral contribution. Gain changes do
    not otherwise reset integral/derivative history.
  - A four-gain update is all-or-nothing with respect to validation in the single-thread OpMode loop;
    it is not a cross-thread atomicity guarantee. TUNE-01 must snapshot Dashboard fields and apply
    the candidate on the loop boundary.
  - After live tuning, robot code resets the outermost retained regulator composition, not merely
    the inner PIDF handle, so voltage sources, limit/debug state, and every delegating wrapper reset
    together. Reset preserves gains and limits and writes no hardware.
  - PIDF does not memoize same-cycle calls or add setpoint/measurement substitution. It passes
    `clock.dtSec()` to the existing `Pid`, whose timing semantics treat a non-positive step as zero
    and advance integral/derivative terms only for a positive step. Integral and PID-output limits
    saturate finite values only; they must not turn non-finite dynamic input or arithmetic overflow
    into a plausible boundary command. Existing `ScalarRegulator` invocation semantics remain;
    regulated Plants retain SAFE-03 fail-stop/reset defense, and direct regulator callers own their
    final output boundary.
- **Student-facing comparison:**

  | Design | Extra student-owned concepts beyond the four gains |
  |---|---|
  | Captured lambda (current Bettabot) | Separate `Pid`, mutable `velocityKf`, lambda, four finite checks, partial-update ordering, and manual status ownership. |
  | Mutable coefficient Source/handle | Two mutable owners and no one-step four-gain validation. |
  | Rebuild regulator/Plant | Replacement/delegation lifecycle and lost state. |
  | `PidfGains` wrapper | Another value/factory copied from an existing robot profile, without independent reuse. |
  | Factory-owned `PidfRegulator` (selected) | One retained capability only when live tuning/status needs it; static callers may store it as `ScalarRegulator`. |

  Bettabot's PIDF portion becomes:

  ```java
  pidf = ScalarRegulators.pidf(
          cfg.velocityKp, cfg.velocityKi, cfg.velocityKd, cfg.velocityKf)
          .setIntegralLimits(-cfg.integralOutputLimit, cfg.integralOutputLimit)
          .setPidOutputLimits(-1.0, 1.0);
  ```

  Its retained outer `regulator` continues to compose that PIDF with Bettabot's explicit
  zero-target coast/reset policy and the framework's complete-command output limit (and voltage
  compensation if configured); API-03 does not disguise that robot policy as a generic helper.

  Its live update becomes:

  ```java
  pidf.setGains(kP, kI, kD, kF);
  regulator.reset();
  ```

  This deletes Bettabot's retained plain `Pid`, mutable `velocityKf`, captured lambda, four generic
  finite checks, and split gain-update order. The Dashboard/profile still own candidate values and
  checked-in configuration; status can read the four applied gains from `PidfRegulator`. Motor
  identity/directions, encoder choice and mapping, target/default/maximum/tolerance relationships,
  maximum-power/no-reverse/coast policy, and reset timing remain robot-owned.
- **Framework Principles check:** the design keeps `Pid` error-centric, puts setpoint feedforward at
  the `ScalarRegulator` layer that has the required input, keeps `ScalarRegulators` as the sole
  public PIDF construction layer, deletes overlapping overloads, and adds no inline-only config
  wrapper. The retained public final type is justified by Bettabot's real storage, tuning, status,
  reset, and composition use. Optional PID-contribution and complete-output limits remain
  explicitly named rather than hidden behind precedence.
- **Named follow-ups:** ACT-01 owns FTC group identity/topology; CAL-01 owns normalized
  calibration-search power before lifecycle side effects; CAL-02 owns calibration reference/hold
  validity; MAP-01 owns FTC actuator transforms and raw native domains; RANGE-01 owns core range
  construction; and FTC-02 owns SDK device-managed PIDF/P and maximum-power answers. CONFIG-01 is
  Deferred until a concrete caller proves owner-snapshot drift. Robot-specific cross-field
  relationships stay in each robot owner.
- **Verification required:**
  - Add focused `Pid` tests for every non-finite gain position, valid signed gains,
    non-finite/reversed bounds, the integral-must-include-zero contract, one-sided/asymmetric/equal
    zero limits, rejected-setter all-or-nothing behavior, immediate integral reclamping, reset, and
    unchanged control/debug math.
  - Add focused `PidfRegulator` tests for every invalid gain at construction and live update, valid
    signed gains, four-gain all-or-nothing behavior, same-error/different-setpoint formula evidence,
    first/zero/positive-dt behavior, integral and PID-output limits, the inner-versus-outer limit
    distinction, gain persistence across reset, outer-decorator reset propagation, live updates
    through retained compositions, repeated same-cycle calls, and debug/current-gain truth.
  - Add API-surface/reflection tests proving `Pid` has no public constructor, `PidfRegulator` is
    public/final with no public constructor or class-local factory, exactly one public
    `ScalarRegulators.pidf(...)` remains with four `double` parameters and a `PidfRegulator` return,
    removed PIDF overloads and old `getkP/getkI/getkD` names are absent, and the selected methods are
    present.
  - Migrate the arbitrary-feedforward regulator test to explicit
    `setpointFeedforward(pid(controller), function)` and retain coverage that the advanced path is
    distinct. Search the entire repository for removed constructors, overloads, and getter names.
  - Synchronize `Pid`, `PidfRegulator`, `ScalarRegulator`, `ScalarRegulators`, and
    `ScalarControllers` Javadocs plus the Beginner's Guide, FTC Actuators & Plants, Recommended
    Robot Design, and Layered Shooter Example. The existing Framework Principles already state the
    required fail-fast, one-public-layer, API-parallelism, and no-inline-wrapper rules. The
    implementation review must add the newly exposed finite-only policy-limit invariant to those
    principles so a bound cannot hide invalid control math; the improvement skill already requires
    the adversarial correctness/safety review that found it and needs no workflow amendment.
  - Run focused tests, the full TeamCode unit suite and Java compilation, inspect XML result counts,
    and run public-surface/caller/documentation and whitespace checks. This pure software
    control-law/configuration change needs no robot-hardware claim.
- **Diagnostics:** errors must name the PID or PIDF factory/setter, each offending setting and
  supplied value, and the finite/order/contains-zero rule. PIDF debug output will expose current
  gains plus last setpoint, measurement, error, PID output, feedforward output, and combined output;
  reset clears last-sample diagnostics without hiding configuration. Generic PID/PIDF cannot
  truthfully name a motor or physical unit, so robot owners retain those contextual messages where
  they express robot policy.
- **Decision record (2026-07-16):** **Ready for explicit approval after the user-requested reopened
  audit.** The original earliest-owner hypothesis remains valid, but the improved design now gives
  standard live linear `kF` the same framework ownership as the PID gains at the correct
  setpoint-aware layer. It deliberately replaces the first-pass validation-only proposal. This is a
  major public API decision: it adds the retained `PidfRegulator` type, removes both old PIDF
  overloads and the public `Pid` constructor, renames PID gain getters, and strengthens integral
  limit semantics. Approve with `Approve API-03 framework-owned PIDF design`; implementation must
  not start before approval.
- **Approval checkpoint (2026-07-16):** the user approved the framework-owned PIDF design and its
  breaking public-API cleanup. API-03 is now **In progress**; implementation remains limited to the
  selected API, caller/documentation migrations, focused tests, and required verification.
- **Implementation checkpoint (2026-07-16):** implemented the approved factory-only API. `Pid` now
  has one public construction path through `withGains(...)`, validates finite gains and explicit
  finite/ordered limits at construction and live mutation, requires integral limits to contain
  zero, and uses the parallel `getKP/getKI/getKD` names. The new public final
  `PidfRegulator` is constructed only by
  `ScalarRegulators.pidf(kP, kI, kD, kF)`, owns all four gains and their one-step live update, and
  exposes separately named integral and inner-PID output limits. Both legacy PIDF overloads were
  deleted; arbitrary feedforward remains the explicit
  `setpointFeedforward(pid(customController), function)` composition.
- **Adversarial refinement (2026-07-16):** correctness review found that the pre-existing `Pid`
  clamp order could convert infinity or arithmetic overflow into a finite limit, hiding invalid
  control math from SAFE-03's Plant fail-stop boundary. `Pid` now applies integral and PID-output
  policy limits only to finite values, so non-finite measurements and P/I/D overflow propagate to
  the owning output boundary. Framework Principles, Javadocs, guides, and direct plus regulated-
  Plant tests now state and enforce this finite-only limiting rule. The framework-improvement skill
  already mandates the adversarial correctness/safety review that found the issue, so no skill
  change is needed.
- **Automated verification (2026-07-16):** the focused API-03 suites pass 58 tests
  (`PidTest` 16, `PidfRegulatorTest` 18, `ScalarRegulatorsApiTest` 4,
  `ScalarRegulatorsTest` 11, and `RegulatedPlantSafetyTest` 9) with zero failures, errors, or skips.
  `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` succeeds; its XML contains
  52 suites and 512 tests with zero failures, errors, or skips. Exact reflection checks enforce the
  approved public surface. Repository scans found no removed constructor/overload/getter production
  caller and no Summer26/Bettabot reference in framework source, tests, or framework documentation.
  `git diff --check` and changed/untracked-file whitespace/final-newline checks are clean. Three
  independent adversarial reviews report no remaining API/principles, correctness/safety, or
  tests/documentation finding.
- **Android Studio audit point (2026-07-16):** API-03 is **Verifying** and intentionally remains
  unstaged and uncommitted. Inspect the factory-only construction surface, four-gain live update,
  inner-versus-complete limit naming, invalid-value fail-fast behavior, non-finite fail-stop
  propagation, synchronized examples, and focused tests. This is a pure software control-law
  change; source inspection and unit tests are sufficient, with no robot-hardware claim. User
  approval authorizes finalization/publication of API-03 only and does not start another item.
- **Manual verification (2026-07-17):** the user reviewed the API-03 implementation in Android
  Studio and approved it with `API-03 looks good`. API-03 is now **Done**; this approval authorizes
  Gate 3 finalization, publication, and merge for API-03 only, not work on the next tracker item.

### API-04 - Binding execution order

- **Problem to confirm:** bindings are grouped by type internally, so execution precedence differs
  from declaration order when different binding kinds affect the same intent.
- **Alternatives to compare:** one ordered binding list; explicit phases/priorities; or reject multiple
  writers to one target.
- **Leading hypothesis:** declaration order is easiest to teach, but conflicting intent ownership
  should be checked before relying on order as policy.
- **Completion:** precedence is deterministic, visible at the call site, and covered across binding
  kinds without changing per-cycle edge semantics.
- **Decision record:** _Pending._

### API-05 - One beginner drive entry point

- **Problem to confirm:** guides present both a raw drivebase factory and a lane, and still teach a
  no-op `drivebase.update(clock)` as if it owns rate-limiter timing.
- **Alternatives to compare:** make the lane the sole robot-facing entry point; restore meaningful
  drivebase update behavior; or clearly separate beginner and low-level APIs.
- **Leading hypothesis:** teach `FtcMecanumDriveLane` for modern robots, retain a clearly lower-level
  factory for tools/custom owners, and remove obsolete update calls.
- **Completion:** one complete beginner loop compiles and matches actual timing/ownership behavior.
- **Decision record:** _Pending._

### COMMON-01 - Cleanup action aggregation

- **Problem to confirm:** Phoenix Auto and framework testers repeat retry, error reporting, partial
  construction cleanup, and initialization state.
- **New Pedro evidence (2026-07-17, current after ROUTE-03):** the completed independent Pedro
  reference totals 626 source lines. Its semantic path/routine/capability files are 200 lines, while
  `BasicPedroAutoRobot` and `PhoenixBasicPedroAutoExample` contribute 426 lines for the composition
  root and FTC host. Much of that shell is required ownership, but INIT failure handling, partial
  construction cleanup, one-shot lifecycle guards, and best-effort stop aggregation are concrete
  candidates to compare against Phoenix Auto and existing testers. A short routine method alone
  does not make the adopting robot simple.
- **Alternatives to compare:** a small generic init/cleanup owner; local shared private helpers; an
  FTC-specific lane; a separate narrowly scoped Auto-runtime shell; or no extraction until another
  production robot needs it. Compare the entire adopting OpMode and composition-root surface, not
  only the extracted helper call.
- **Leading hypothesis:** extract only the lifecycle state machine and cleanup guarantees already
  repeated in at least three callers. Do not include robot capabilities, configuration policy, or
  OpMode inheritance. Keep one explicit clock/localization/Pedro/Task/Plant loop order. If recurring
  active Auto-root lifecycle is real but does not fit an initialization helper without hiding
  ownership, split another focused tracker item rather than silently broadening COMMON-01.
- **Completion:** callers become shorter without hiding loop order or resource ownership; partial
  failure and repeated cleanup are tested; the decision record counts removed versus introduced
  robot-code lines and concepts across the complete Pedro reference.
- **Decision record (2026-07-17):**
  - **Confirmed decision-gate reference cost:** the pre-implementation Pedro reference was 626
    source lines:
    98 in `BasicPedroAutoMechanism`, 52 in `BasicPedroAutoPaths`, 50 in
    `BasicPedroAutoRoutine`, 209 in `BasicPedroAutoRobot`, and 217 in
    `PhoenixBasicPedroAutoExample`. The semantic capability/path/routine files are 200 lines; the
    composition root and FTC host are 426 lines. The loop itself is not accidental boilerplate:
    `BasicPedroAutoRobot.update(...)` must keep
    `clock -> localization -> Pedro heartbeat -> TaskRunner -> mechanism Plant` explicit. The
    mechanically repeated portion is narrower: ordered best-effort cleanup plus preservation of a
    primary failure and later suppressed cleanup failures was estimated to remove 62-70 net
    robot-code lines across the basic root, mechanism, and host before implementation. The exact
    completed measurement below is 60 lines, or about 9.6% of the complete reference.
  - **Full caller and behavior audit:** only `PhoenixPedroAutoOpModeBase` has the complete proposed
    retry state machine. It distinguishes a readiness blocker from a construction failure, retains
    the exact `PhoenixAutoSpec`, treats another request for the same successful spec as idempotent,
    rejects replacement with a different spec, permits another INIT attempt after ordinary clean
    failure, and permanently blocks retry after uncertain cleanup. The independent Pedro host is a
    one-shot FTC lifecycle that rethrows INIT failure; `BasicPedroAutoRobot` is a one-shot
    START/update/fail-stop owner; and `PhoenixRobot` permits one mode initialization and one
    terminal idempotent stop. These are not three copies of one retry policy.
  - **Tester and tool audit:** `FtcTeleOpTesterOpMode` owns one whole-OpMode tester but has no
    retained retry contract. `TesterSuite` and `HardwareSelectingTester` own replaceable child
    sessions, not a whole Auto runtime; in particular, `HardwareSelectingTester` can lose a local
    tester whose `init(...)` or `start()` threw before `active` was assigned. Three vision testers
    repeat resource-specific ready/error/reselection code, but currently swallow cleanup failure
    and permit retry, unlike the fail-closed Phoenix Auto contract. These are concrete tester-safety
    follow-ups, not evidence that one generic INIT state machine should silently choose retry,
    replacement, presentation, or cleanup-failure policy for every caller.
  - **Other cleanup callers audited:** `RouteExecution.failClosed(...)`,
    `FtcHardware.acquireRequiredModeIfNeeded(...)`, and `PhoenixScoringAttemptTask.cancel()` contain
    mechanically equivalent attempt-all/first-failure aggregation, and modern framework TeleOp
    examples contain several simpler sequential multi-owner `stop()` methods. They confirm that the
    narrow primitive is reusable, but remain deliberate non-migrations in this bounded first use so
    COMMON-01 does not become a repository-wide cleanup rewrite. `DcMotorPowerTester` also preserves
    failures while restoring physical zero and motor mode, but deliberately wraps some failures
    with tester-specific remediation. Other Task, route, and regulated-Plant implementations make
    outcome- or state-dependent cleanup decisions; converting those non-equivalent state machines
    would change their safety or lifecycle contract. None of these callers supplies a second copy
    of the complete generic INIT/retry policy.
  - **Broader-use and naming audit:** appropriate uses extend beyond a composition root's final
    `stop()`: partial-construction rollback, Task cancellation, route fail-closed cancellation,
    multi-motor zeroing after configuration failure, Plant output/reset recovery, and tester
    teardown all need the same attempt-all/retain-first mechanism. They are nevertheless all
    cleanup or compensating actions. No ordinary forward-operation caller was found where continuing
    after a failed prerequisite is generally safe. A generic `Actions`, `BestEffortActions`,
    `RuntimeFailures`, or `Exceptions` facade would therefore advertise a broader and potentially
    unsafe policy. `LifecycleCleanup` is also too narrow and can sound like it owns lifecycle state
    or idempotence. Use the plural static facade `CleanupActions`; it describes caller-owned
    cleanup operations without claiming to own their lifecycle.
  - **Existing public construction paths:** `PhoenixRobot` has two public constructors,
    `initAny()`, `initTeleOp()`, two `initAuto(...)` overloads, and one public `stop()`; its
    `PhoenixShutdown` state helper is deliberately package-private. `PedroPathingRuntime` has one
    public static `create(...)` factory and a private constructor, and cleans its own partially
    created vendor graph. `BasicPedroAutoRobot` has one public wiring constructor plus a
    package-private fake-test constructor. Testers use the existing `TeleOpTester` lifecycle,
    `FtcTeleOpTesterOpMode#createTester()`, the `TesterSuite` constructor/fluent registration path,
    and the `HardwareSelectingTester` constructor. There is no existing framework runtime slot,
    cleanup registry, generic lifecycle interface, or second public construction layer to extend.
  - **Why the documented supplier-based guard is not sound:** the conceptual
    `InitRuntimeGuard.builder(...).buildWith(Supplier<T>)...` in
    `Future Robot Boilerplate Helpers.md` cannot clean hardware or another resource created before
    a throwing supplier returns `T`. Fixing that requires callers either to introduce a new wrapper
    composition root before acquisition or to register every partial owner in a cleanup scope and
    transfer/disarm ownership as larger owners are built. The latter adds a public resource
    transaction, alias/transfer rules, reverse-versus-explicit cleanup-order choices, and
    double-stop hazards. Its `cleanupWith(...)` and `describe(...)` parameters would be constructed
    inline for one builder step rather than stored, reused, composed, shared, or independently
    validated, so staged parameter types or selector wrappers would add public nouns without
    distinct value.
  - **Side-by-side ordinary robot call sites and decisions:**

    | Approach | Ordinary owner code | Conceptual decisions supplied by the student |
    |---|---|---|
    | No framework change | `failure = attempt(failure, runner::cancelAndClear);` repeated for each owner, followed by `if (failure != null) throw failure;` plus a separate primary-failure bridge | terminal/idempotent state; every owner; action order; first-failure retention; later suppression; self-suppression; primary propagation |
    | Robot-local private helper | the same `attempt(...)` calls, with the helper copied into each new robot root/host | every decision above; the mechanics merely move to another robot file |
    | Proposed `CleanupActions` | `CleanupActions.attemptAll(runner::cancelAndClear, mechanisms::stop, drive::stop);` | terminal/idempotent state; the three real owners; their safety order. The framework owns only attempt-all and exception aggregation |
    | Generic `InitRuntimeGuard<T>` | `guard.tryInitialize(this::buildRuntime)` plus a cleanup callback or registration scope | runtime identity; construction/initialization boundary; partial-owner registration and transfer; same/different-spec replacement; retry eligibility; cleanup-failure policy; error presentation; STOP behavior |
    | FTC/Auto base shell | `final class MyAuto extends FrameworkAutoOpMode { ...hooks... }` | short leaf code, but hidden callback/loop order plus hook choices for readiness, telemetry, routes, services, mechanisms, and cleanup |

    The no-change and local-helper shapes therefore keep about seven lifecycle/error decisions in
    robot code. The proposed static call reduces that to owner identity/order plus the owner's
    existing terminal flag. The generic guard adds decisions that the current callers answer
    differently, and a base shell hides required ownership rather than eliminating it.
  - **Chosen materially narrower design:** add one FTC-independent, static-only public primitive,
    `edu.ftcphoenix.fw.core.lifecycle.CleanupActions`, with no public constructor, builder, retained
    resource list, or parallel instance API. `attemptAll(Runnable...)` executes the caller-listed
    actions in order, attempts every action after a `RuntimeException`, then throws the first failure
    with later failures suppressed. “Attempt” is intentional: it promises that every eligible
    cleanup is invoked, not that every cleanup succeeds. The
    `RuntimeException attemptAllAfterFailure(RuntimeException primaryFailure, Runnable...)` method
    attempts every action in order, attaches each cleanup failure directly to the already-owned
    primary, and returns that **same exception object** so the common rethrow remains one explicit
    expression:

    ```java
    catch (RuntimeException failure) {
        throw CleanupActions.attemptAllAfterFailure(
                failure,
                mechanism::stop,
                drive::stop
        );
    }
    ```

    The supplied failure is therefore the attachment target, not a signal that the helper secretly
    throws it. A caller may instead wrap the returned primary as an actionable cause, retain it for
    telemetry, or ignore the return while continuing to reference the same mutated object. Making
    the helper always throw was rejected because `PedroPathingRuntime` deliberately wraps its
    enriched construction failure, and returning a new aggregate would lose primary identity.
    Eliminating this method in favor of catching `attemptAll(...)` would create a nested suppression
    graph (`primary -> first cleanup -> later cleanup`) rather than attaching every cleanup failure
    directly and visibly to the true primary.
    A null primary is rejected before cleanup because there is no failure to preserve. A null
    action array passed explicitly to `attemptAll(...)` is rejected; when an existing primary is
    supplied, the equivalent array error is attached to that primary. An individual null action is
    treated as an actionable indexed cleanup failure at its position and later actions are still
    attempted. “Distinct” means only that an exception is never suppressed onto itself by identity;
    the helper does not globally deduplicate a later exception instance. It catches
    `RuntimeException`, not `Error`.
  - **What students still decide:** owners retain their own exact-once or terminal flag, detach
    references before reentrant cleanup when required, and list the real actions in their required
    safety order. For the basic Auto root, the complete normal call remains visibly
    `CleanupActions.attemptAll(runner::cancelAndClear, mechanism::stop, drive::stop)`. During partial
    host construction, the host still explicitly chooses mechanism cleanup **or** raw Plant cleanup
    according to which owner exists, then asks the helper to preserve those failures on the
    construction exception. The helper never owns a `LoopClock`, OpMode callback, runtime slot,
    retry decision, resource registry, telemetry frame, route policy, or robot capability.
  - **Framework Principles and simplicity result:** this introduces one public noun with a distinct
    safety capability instead of a second spelling of runtime construction. It keeps all actions
    and their order visible, preserves the original actionable failure, contains no FTC/vendor
    dependency, and removes exception-graph mechanics from robot code without hiding the heartbeat
    or hardware ownership. The full reference still teaches idempotent fail-stop cleanup as one of
    its approximately 15 concepts; it should not claim that a helper removes the need to choose
    owners or safety order.
  - **Rejected designs:** documentation-only/no change leaves a real repeated safety mechanism in
    robot code; private helpers do not reduce adopting-robot code; a public
    `InitRuntimeGuard<T>` lacks enough equivalent callers and cannot own an unreturned partial
    graph; a cleanup-registration transaction adds more ownership concepts than it removes; making
    every owner `AutoCloseable` does not fit the non-lexical FTC lifecycle or explicit stop order;
    promoting `PhoenixShutdown` would incorrectly export Phoenix's one-init policy; an FTC lane,
    generic Auto runtime, scheduler, or base OpMode would hide lifecycle/loop ownership and absorb
    robot-specific readiness, telemetry, and strategy.
  - **Bounded implementation scope:** if approved, add only the static cleanup primitive and focused
    tests; migrate the equivalent aggregation in `BasicPedroAutoRobot`,
    `BasicPedroAutoMechanism`, `PhoenixBasicPedroAutoExample`, PhoenixRobot's
    initialization-failure bridge, and the matching single-cleanup construction bridge in
    `PedroPathingRuntime` where semantics remain identical. Keep `PhoenixShutdown.run(...)`
    package-private and unchanged because it deliberately accepts absent/null owner actions as part
    of a larger exact-once lifecycle; the proposed public primitive treats an individual null
    action as an error. Do not change Auto retry/spec/readiness policy, tester selection behavior,
    Task cleanup state machines, route behavior, loop order, or any later tracker item. Correct
    `Future Robot Boilerplate Helpers.md` so the speculative generic guard is explicitly deferred,
    and synchronize the Pedro reference and robot-design cleanup guidance with the real API and
    exact line counts.
  - **Verification plan:** add pure tests for action order, all-actions-after-failure, first/later
    failure identity and suppression order, return identity from `attemptAllAfterFailure(...)`, an
    action throwing the already-owned primary, invalid primary/action inputs, and calls with no
    actions. Rerun `PhoenixShutdownTest`,
    `BasicPedroAutoRobotTest`, `BasicPedroAutoMechanismTest`,
    `PhoenixBasicPedroAutoExampleTest`, `PhoenixPedroAutoOpModeBaseTest`, and
    `PedroPathingRuntimeValidationTest`; then run the complete TeamCode unit suite and Java compile
    with Android Studio's JBR. Recount the five Pedro files, search for a new generic INIT guard,
    resource registry, hidden OpMode/base shell, or changed loop order, check documentation links
    and whitespace, and report that hardware cleanup ordering itself remains unverified without the
    robot.
  - **Design approval (2026-07-18):** the user approved the `CleanupActions` design after the API was
    refined to `attemptAll(...)` and returned-primary `attemptAllAfterFailure(...)`. Implementation
    is limited to the primitive, the five bounded migrations, focused tests, synchronized
    documentation, and exact post-change line counts.
  - **Implementation (2026-07-18):** added the static-only, FTC-independent `CleanupActions`
    primitive with only the two approved public methods. It preserves caller order and exception
    identity, attempts later actions after a `RuntimeException`, attaches later failures in order,
    avoids self-suppression, treats null actions as indexed cleanup failures, and propagates
    `Error` immediately. The equivalent aggregation in `BasicPedroAutoRobot`,
    `BasicPedroAutoMechanism`, `PhoenixBasicPedroAutoExample`, PhoenixRobot's two initialization
    failure bridges, and `PedroPathingRuntime` now uses that primitive. No loop order, owner
    eligibility, idempotence flag, partial-construction selection, retry/readiness policy,
    `PhoenixShutdown`, Task state machine, or public robot construction path changed.
  - **Implementation measurement (2026-07-18):** the five-file Pedro reference is now exactly 566
    source lines: 85 in `BasicPedroAutoMechanism`, 52 in `BasicPedroAutoPaths`, 50 in
    `BasicPedroAutoRoutine`, 171 in `BasicPedroAutoRobot`, and 208 in
    `PhoenixBasicPedroAutoExample`. That is 60 fewer lines than the 626-line pre-COMMON-01
    reference, a reduction of about 9.6%. The four independent reference classes total 358 lines;
    the Phoenix-specific FTC host contributes 208. Ownership, cleanup order, the explicit
    `clock -> localization -> Pedro heartbeat -> TaskRunner -> mechanism Plant` update order, and
    all robot-specific construction policy remain visible.
  - **Automated verification (2026-07-18):** the seven focused cleanup, migrated-caller, Phoenix
    shutdown/host, and Pedro-runtime suites pass 36 tests with zero failures, errors, or skips. The
    final `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` run succeeds with 55 XML
    suites and 554 tests, zero failures, zero errors, and zero skips. Diagnostics are limited to the
    repository's existing Java-21/source-8 and FTC deprecation warnings.
  - **Static, documentation, and adversarial verification:** exact line recounting confirms the
    566-line reference and 60-line reduction. Exhaustive caller/API searches confirm one new public
    static facade, exactly two approved methods, exactly five production migrations, and no generic
    INIT guard, registry, base OpMode, alias, overload, or parallel construction path. Framework
    Principles, robot-design guidance, the Pedro reference, and the future-helper guidance describe
    the same ownership and `RuntimeException`/`Error` contract. Framework docs contain no downstream
    case-study project reference; local Markdown links, `git diff --check`, and affected-file
    trailing-whitespace checks pass. Independent API/simplicity, correctness/lifecycle, docs, and
    final-scope reviews found no blocking issue after their count, wording, suppression, and
    adversarial-test findings were addressed.
  - **Gate 3 / verification status (2026-07-18):** implementation and automated verification are
    complete. No robot-hardware result is claimed; physical cleanup ordering remains an on-robot
    observation even though this change preserves the existing owner order and adds no new hardware
    behavior.
  - **Manual verification (2026-07-18):** the user reviewed COMMON-01 in Android Studio and replied
    `COMMON-01 looks good`. COMMON-01 is **Done**. This approval authorizes publication and merge for
    COMMON-01 only; TESTER-01 and every later tracker item remain untouched.

### TESTER-01 - Tester child lifecycle fail-stop

- **Problem to confirm:** tester owners can lose a partially initialized child or replace a child
  after cleanup failed, leaving hardware ownership uncertain. `HardwareSelectingTester` constructs
  a candidate in a local variable and assigns `active` only after `init(...)` and `start()` return;
  if either callback throws, the candidate is no longer reachable for `stop()`. `TesterSuite`,
  `FtcTeleOpTesterOpMode`, and the selectable vision testers also need an explicit audit of failure
  and replacement behavior across every child lifecycle callback.
- **Why it matters:** test OpModes intentionally exercise real hardware. A failed child must not
  leave a motor, camera, or vendor resource active while the UI selects or creates a replacement.
  The safe behavior should be fail-stop and actionable without adding a generic robot runtime or
  hiding tester selection from students.
- **Framework Principle target:** stable resource ownership, exact and idempotent cleanup,
  best-effort cancellation after lifecycle failure, fail-fast actionable errors, and one clear
  public lifecycle path.
- **Alternatives to compare:**
  1. correct each existing tester owner locally, retaining the candidate before invoking lifecycle
     callbacks and blocking replacement when cleanup outcome is uncertain;
  2. extract a tester-specific child-session owner only if the complete lifecycle state machine is
     repeated after the local corrections are written and tested;
  3. reuse the narrow lifecycle-cleanup aggregation primitive for exception preservation while
     keeping ownership and retry policy local;
  4. introduce the previously proposed generic INIT/runtime guard;
  5. document the risk without changing behavior.
- **Leading hypothesis:** first fix the existing tester owners at their real ownership boundaries.
  Use the COMMON-01 cleanup primitive, if approved and applicable, only for attempt-all exception
  aggregation. Extract a tester-session helper only if the completed fixes reveal multiple
  structurally identical child state machines; do not generalize this into an Auto/robot runtime.
- **Student-facing simplicity gate:** ordinary tester creation and selection must not gain a
  builder, cleanup registry, ownership-transfer token, or second public construction path. The
  tester should either be visibly active or visibly failed closed; students should not decide
  exception-suppression mechanics.
- **Completion:** fake-backed tests cover a null/throwing factory, `init`, `start`, `initLoop`, and
  `loop` failure; prove that every reachable partial child is stopped at most once, cleanup failures
  are preserved and block unsafe replacement, BACK and repeated STOP remain safe, and no callback
  runs on a terminal child. Update tester documentation and examples if the observable lifecycle
  contract changes.
- **Decision record (2026-07-18):**
  - **Confirmed traced failures:** `HardwareSelectingTester.enter(...)` keeps a newly returned
    tester only in a local variable until both `init(...)` and an optional `start()` return. Either
    callback can therefore throw after acquiring hardware, after which the catch block drops the
    only reference, clears the picker, and permits a replacement without ever invoking
    `stop()`. `TesterSuite` retains its candidate before `init(...)`, but an `init`, `start`,
    `initLoop`, `loop`, or BACK failure leaves contradictory menu/active state and can permit a
    replacement or another callback on the failed child. Both owners clear `active` only after
    `stop()` returns, so a failed or reentrant stop can be invoked repeatedly.
    `FtcTeleOpTesterOpMode` likewise retains its root before `init(...)`, but every forwarded
    lifecycle failure leaves that root callable and repeated FTC `stop()` calls repeat the child
    stop. Its null `createTester()` result merely disables the OpMode after telemetry rather than
    failing with an actionable configuration error. These direct call traces close the problem
    confirmation gate without touching hardware.
  - **Selectable-resource audit:** `CameraMountCalibrator`,
    `AprilTagLocalizationTester`, and `PinpointAprilTagFusionLocalizationTester` each allow a
    failed `AprilTagVisionLane.close()` to be discarded or followed by another selection, so the
    previous camera's ownership can be uncertain while a replacement is opened. Their candidate
    lane must be retained as soon as `open(...)` returns, detached before one close attempt, and
    allowed to be replaced only when that close returned normally. A factory whose `open(...)`
    throws before returning still owns rollback of anything it acquired; no caller-side session
    can clean an object that was never returned.
  - **Complete public construction and caller audit:** the normal menu path remains
    `new TesterSuite().add("Name", Tester::new)`. Eight direct construction sites, including
    `CalibrationWalkthroughBuilder`, use that path across nested framework and Phoenix menus.
    `FtcTeleOpTesterOpMode` has one protected `createTester()` hook and exactly the framework and
    Phoenix production hosts. `HardwareSelectingTester` has one public constructor and four
    `StandardTesters` factories. The ready-made `StandardTesters` and Phoenix factories supply
    menu content; they are not parallel lifecycle APIs. No `TesterSuite.of(...)`, second host,
    lifecycle builder, registry, or session API exists.
  - **Parameter-type and API-value audit:** `Supplier<TeleOpTester>` has distinct value because a
    menu entry must create a fresh inactive tester each time it is entered.
    `Function<String, TeleOpTester>` has different value because the runtime-selected hardware
    name configures that fresh child. The advanced `MenuItem<Supplier<TeleOpTester>>` overload has
    distinct stable-id, tag, help, and disabled-item value. Keep all three capabilities, but reject
    a null enabled factory or null returned child with the selected tester/hardware name in the
    message. Document that a factory constructs an inactive child, performs no unreturnable
    acquisition, and returns a fresh non-null instance; hardware acquisition belongs in
    `init(...)` or a later owned phase.
  - **Alternatives and simplicity comparison:**

    | Approach | Valid robot/tester call | New student concepts | Ownership and failure result |
    |---|---|---:|---|
    | Documentation only | unchanged | 0 | Leaves the proven lost-child, repeated-stop, and unsafe-replacement paths |
    | Local owner corrections | unchanged | 0 | Each existing owner visibly retains one child and chooses its own retry/presentation policy |
    | Package-private session for the two menu owners | unchanged | 0 | Centralizes their identical active/retry/blocked mechanism while each owner keeps its menu/picker presentation |
    | Public or all-owner tester session | adds another lifecycle API | 1 or more | The one-shot host, replaceable menu child, and vision resource do not share one complete policy |
    | Put state in `BaseTeleOpTester` | subclass hooks appear unchanged | hidden self-ownership | Cannot govern arbitrary `TeleOpTester` implementations or the owner's replacement decision |
    | Generic INIT/runtime guard | adds a guard/builder and cleanup transfer rules | several | Cannot clean resources acquired before a throwing supplier returns and must guess retry, replacement, and UI policy |

    Local corrections therefore add no public method, builder, state token, registry, or second
    construction path. The only student-visible change is a phase- and tester-named error plus
    explicit “restart the OpMode” guidance when cleanup is uncertain.
  - **Chosen child-owner lifecycle:** `TesterSuite` and `HardwareSelectingTester` repeat the same
    replaceable child state machine closely enough to justify one package-private
    `TesterChildSession` in their existing package. It owns only the child slot, active/empty/
    cleanup-blocked/terminal state, lifecycle forwarding, detach-before-stop, exact-once cleanup,
    and primary/suppressed exception preservation. It does **not** own a factory, selected
    hardware name, menu/picker state, telemetry wording, `LoopClock`, or FTC OpMode lifecycle, and
    is not a public or protected API. Each existing owner keeps those presentation and selection
    decisions. A non-null child is retained before its first callback. On a child
    `RuntimeException`, the session first makes that child terminal and detaches it, invokes
    `stop()` at most once, preserves the callback failure as primary, and attaches a stop failure
    as suppressed. Confirmed cleanup lets the suite or selector show the error and accept a
    **fresh** selection; cleanup failure latches a visible failed-closed state, invokes no child or
    factory again, consumes BACK so a parent suite cannot replace the uncertain subtree, and
    instructs the operator to restart the OpMode. Normal unhandled BACK uses the same
    detach-before-stop rule; handled BACK leaves the child active. Explicit owner STOP
    terminalizes first, stops an active child once, and makes repeated or reentrant STOP a no-op.
  - **FTC-host policy remains intentionally different:** `FtcTeleOpTesterOpMode` is a one-shot root,
    not a retry menu. A null/throwing `createTester()` is terminal and actionable. Once a root is
    returned it is retained before `init(...)`; a `RuntimeException` from `init`, `name`,
    `initLoop`, `start`, or `loop` detaches and stops it once, preserves any stop failure, and
    rethrows so the SDK records the failure. Later lifecycle calls cannot reach that terminal root.
    The existing one-clock reset/update order is unchanged. As with `CleanupActions`, this contract
    catches `RuntimeException`, not `Error`.
  - **Selectable-vision policy:** correct the three audited selectable vision owners locally rather
    than making a public generic resource session. If setup fails after `open(...)` returned a
    lane, detach and close that lane once while preserving the setup failure. Confirmed close
    permits the picker to retry; close failure is retained, blocks another open, and displays a
    restart instruction. BACK and final STOP use detach-before-close and never retry the same lane.
    This is the same safety invariant as child ownership, but its camera-specific readiness and
    picker presentation remain with the tester that owns them.
  - **Framework Principles result:** the design keeps one existing tester lifecycle and one normal
    construction path, leaves the FTC and camera boundaries explicit, retains fresh-child and
    cleanup ownership at the composition point, fails with actionable context, and puts no
    lifecycle decisions into ordinary Phoenix robot/tester code. The public `TeleOpTester`
    contract must say that `stop()` can be called once after a partially completed or throwing
    `init()`/`start()`, so implementations clean only successfully acquired fields. That is a
    material lifecycle semantic even though method signatures and valid call sites do not change.
  - **Rejected designs:** do not duplicate the proven menu-child state machine after the audit has
    shown two structurally identical owners. Also do not broaden that narrow package-private helper
    into a public or protected child session, lifecycle builder,
    ownership-transfer token, cleanup registry, generic Auto/runtime guard, or another
    `TeleOpTester` construction facade. Do not make the child own its own replacement policy.
    Do not use `CleanupActions` as if it owned terminal state; it is useful only where a concrete
    tester has multiple independent cleanup actions. Do not catch `Error` or retry a stop whose
    result is already uncertain.
  - **Bounded implementation scope:** add only the package-private child-session implementation and
    change `HardwareSelectingTester`, `TesterSuite`, `FtcTeleOpTesterOpMode`, the three named
    selectable vision testers, their directly affected Javadocs/guides, and focused fake-backed
    tests. Preserve all existing valid constructors, menu-registration methods, factories, loop
    order, hardware choices, and Phoenix caller code.
    The audit also found separate internal-resource issues: the configured
    `PinpointPodOffsetCalibrator` does not stop its optional drive, multi-motor tester cleanup can
    stop after the first failure, several raw-device testers discard restoration failures, and a
    Limelight lane may acquire before construction returns. Those are not structurally the same
    child-session change and are deliberately **not** claimed fixed by TESTER-01; they require a
    separate decision gate or the appropriate later low-level resource item rather than silently
    expanding this task.
  - **Verification plan:** add a pure `TesterChildSessionTest`, plus `TesterSuiteTest`,
    `HardwareSelectingTesterTest`, and `FtcTeleOpTesterOpModeTest` fake children covering factory
    throw/null, failures in `init`,
    `start` both before and after FTC START, `initLoop`, `loop`, `name`, and BACK; successful and
    throwing `stop`; original/suppressed exception identity; safe retry only after confirmed
    cleanup; nested failed-closed BACK; repeated/reentrant STOP; no callback on a terminal child;
    happy-path lifecycle mapping; and unchanged one-heartbeat behavior. Add focused fake
    `AprilTagVisionLane` cases for setup failure after open, close failure, confirmed retry, BACK,
    and repeated STOP in each selectable owner. Run those suites, the existing tester/calibration
    suites, `:TeamCode:testDebugUnitTest`, and `:TeamCode:compileDebugJavaWithJavac`; search every
    constructor/caller again, inspect docs/examples for one-path consistency, and report that
    physical motor/camera cleanup remains unverified without robot hardware.
  - **Design approval (2026-07-18):** the user approved the TESTER-01 fail-stop design.
    Implementation is limited to the package-private menu-child session, the distinct one-shot host
    and three selectable-vision owners, synchronized tester lifecycle documentation, and focused
    fake-backed verification. The adjacent resource-cleanup findings and every later tracker item
    remain untouched.
  - **Implementation (2026-07-18):**
    - Added one package-private `TesterChildSession` used only by `TesterSuite` and
      `HardwareSelectingTester`. It retains a child before callbacks, detaches before one cleanup
      attempt, preserves primary/suppressed runtime failures, permits a fresh selection only after
      confirmed cleanup, and latches cleanup-blocked or terminal state without adding a public API.
    - Made `FtcTeleOpTesterOpMode` a terminal one-shot owner across factory, `init`, `name`,
      `initLoop`, `start`, and `loop` failures. Reentrant or repeated STOP cannot transfer or
      reacquire ownership, invoke a later callback, or stop the same root twice.
    - Applied the approved local policy to `CameraMountCalibrator`,
      `AprilTagLocalizationTester`, and `PinpointAprilTagFusionLocalizationTester`: a returned lane
      is retained immediately, cleanup is detach-first and exact-once, a failed close blocks another
      open, and a terminal STOP request cannot be cleared by a reentrant close callback.
    - Kept all valid constructors, menu factories, hardware choices, and Phoenix callers unchanged.
      Updated `TeleOpTester`/base Javadocs and the FTC UI/testing guides with the fresh inactive
      factory, partial-init cleanup, safe-retry, and restart-after-uncertain-cleanup contracts.
  - **Adversarial review and corrections (2026-07-18):** independent lifecycle and test reviews
    found three boundary cases before the verification gate. The FTC root now rechecks terminal
    state and retained identity after reentrant factory, initialization, naming, and telemetry
    callbacks. The three vision owners now preserve terminal precedence if `close()` reenters
    owner STOP. Nested `TesterSuite` instances now deduplicate parent-delegated and locally bound
    BACK handling in one shared-clock cycle, and clear that cache when START resets the root clock
    and reuses cycle numbers. Focused regressions exercise each path. Final independent re-review
    found no remaining material lifecycle, scope, or coverage issue.
  - **Automated verification (2026-07-18):**
    - Focused lifecycle suites pass **36/36**: `FtcTeleOpTesterOpModeTest` 13,
      `TesterChildSessionTest` 7, `TesterMenuLifecycleTest` 9, and
      `SelectableVisionTesterLifecycleTest` 7, with zero failures, errors, or skips.
    - `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` passes: **59 suites,
      590 tests**, zero failures, errors, or skips. Output contains only the repository's existing
      Java 21/source-target 8 deprecation warnings.
    - `git diff --check` and trailing-whitespace checks pass. The approved scope is 16 files:
      nine production Java files, four test files, two guides, and this tracker. A signature scan
      found no added or changed public/protected construction method. The existing eight
      `TesterSuite` constructions, four `HardwareSelectingTester` constructions, and two FTC-host
      subclasses still compile without caller changes.
  - **Gate 2 verification stop (2026-07-18):** TESTER-01 was moved to **Verifying** and paused for
    Android Studio review. Fake-backed tests prove the framework's ownership, callback, exception,
    and exact-once cleanup decisions. With no robot hardware available, they do not claim that a
    physical motor or camera actually stopped after an SDK/vendor failure; that remains optional
    adoption validation. No later tracker item was started.
  - **Manual verification (2026-07-19):** the user reviewed TESTER-01 in Android Studio and replied
    `TESTER-01 looks good`. TESTER-01 is **Done**. This approval authorizes publication and merge for
    TESTER-01 only; COMMON-02 and every later tracker item remain untouched.

### COMMON-02 - Telemetry commit ownership

- **Problem to confirm:** some presenters call `telemetry.update()` while framework renderers do not,
  making composition and ordering inconsistent.
- **Alternatives to compare:** presenters never commit; a `TelemetryFrame` owner; or explicitly
  commit in every presenter with isolated telemetry instances.
- **Leading hypothesis:** presenters render only and the composition root commits once per loop.
- **Completion:** adding a presenter cannot erase or prematurely commit another presenter's data;
  lifecycle is documented in the canonical loop.
- **Decision record:** _Pending._

### CHECK-01 - Staged whole-robot system check

- **Problem to confirm:** Phoenix has strong individual device testers and PHX-02 can report static
  readiness, but there is no one pre-match procedure that safely exercises the assembled robot in
  stages, records pass/warn/fail evidence, asks for human confirmation where measurement is
  impossible, and guarantees cleanup after cancellation or failure.
- **External evidence:** Cuttlefish's non-blocking
  [`SystemCheck`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/SystemCheck.java)
  advances through sensor, vision, odometry, drivetrain, turret, shooter, intake, magazine, and
  manual-confirmation stages, then consolidates results and cleans up outputs.
- **Alternatives to compare:** keep separate tester OpModes; write one Phoenix-only check directly;
  compose existing Tasks in a robot check specification; add a small staged check/result runner; or
  reflect over hardware/subsystems automatically. Separate passive readiness from motion-producing
  checks and define explicit operator confirmation before each hazardous stage.
- **Leading hypothesis:** Phoenix owns the check list, expected ranges, capability actions, and safe
  state because those are robot facts. Reuse existing tester lifecycle and Task cancellation. Extract
  only a small stage/result/report/presenter abstraction if it removes repeated sequencing and
  cleanup without hiding which hardware will move; do not add reflection, hardware auto-discovery,
  or a generic `BaseRobot`.
- **Completion:** a check can run, skip, retry, cancel, and stop from every stage; motion stages require
  visible consent; every active capability returns safely after success, failure, exception, or
  OpMode stop; results include measured value, expectation, severity, and actionable advice; PHX-02
  may reference the latest report without running hardware automatically; and a fake-HAL test covers
  the complete sequence.
- **Pause record (2026-07-19):** **Deferred while the assembled Phoenix robot is unavailable.** A
  generic stage/report runner is software-testable, but meaningful robot thresholds, hazardous-motion
  confirmations, and claims that every physical capability reaches its safe state cannot be guessed.
  Revisit with the robot, or propose a separately approved infrastructure-only contract that makes
  the complete Phoenix checklist adopting-robot work.

### EXAMPLE-01 - Compiling modern starter robot

- **Problem to confirm:** current beginner material jumps from a flat OpMode to a large layered
  example without a small compiling composition-root/capabilities reference.
- **Alternatives to compare:** multi-file example package; repository template; code generator; or a
  generic base robot.
- **Leading hypothesis:** a small compiling multi-file reference is simplest to copy and understand.
  A generator or base class is unjustified until repeated manual use proves a need.
- **Completion:** one profile, mechanism, capability family, controls owner, composition root, thin
  TeleOp, and tiny Auto; no vision or season strategy.
- **Decision record:** _Pending._

### EXAMPLE-02 - Compiling Pedro autonomous reference

- **Problem to confirm:** Phoenix has a large season-specific Pedro graph and placeholder paths, but
  no small compiling reference that teaches the complete supported lifecycle. A student cannot yet
  see, in one bounded example, where Pedro poses live, who updates the Follower, how route Tasks
  compose with capability Tasks, how route failure branches, and which owner stops hardware.
- **External evidence:** LOAD's Worlds routines show the target readability directly: init selects
  alliance/plan, and each routine reads as a short semantic sequence of follow-path, intake, and
  shoot factories. SaMoTech's named `AutoStep[]` plans and Tech Tigers' short
  `CycleConfiguration[]` leaf OpModes reach a similar surface but require much larger hidden custom
  machinery. Phoenix should achieve that short surface with the ordinary framework vocabulary
  rather than teaching a generated plan language or fixed-cycle state builder.
- **Alternatives to compare:** rely on Pedro's Ivy example; expand Markdown snippets only; make the
  full Phoenix Decode Auto the starter; add a code generator/base OpMode; or add one small compiling
  Pedro-specific robot-side example after PEDRO-01/02 establish the correct integration contract.
- **Leading hypothesis:** add a compact multi-file example and guide, not a new Auto DSL or base
  robot. It should show one configured follower/integration owner, a tiny immutable path set, one
  reusable mechanism capability Task, a readable routine composition, explicit route fallback, a
  thin OpMode, one LoopClock/follower heartbeat, and deterministic cancellation. Keep Pedro types at
  the integration/path boundary and use Phoenix Tasks rather than teaching Ivy beside them. Keep
  four reference classes independent of Phoenix-season services so the example cannot hide those
  dependencies or count only its outer call site as robot simplicity.
- **Completion:** the example compiles against the pinned Pedro version, runs in a pure/fake adapter
  test where practical, contains no placeholder hardware config, and documents the exact files a
  student normally edits for a new alliance/path/routine. Android Studio and on-robot walkthroughs
  confirm the normal programming path is shorter and clearer than copying the full Phoenix season
  graph or assembling a generated state-machine scaffold. Record total robot-code files, lines, and
  concepts—not only the leaf routine call site—and identify the framework/runtime prerequisites a
  different robot must satisfy before adopting it. After their prerequisite items land, an advanced
  companion section demonstrates one
  alliance transform, a route supplier resolved at start, one Pedro progress callback that requests
  a robot capability, deadline-bounded intake, truthful route branching, vision fallback, and a
  match-time park takeover without introducing a second scheduler or Auto DSL.
- **Decision record (2026-07-15):**
  - **Confirmed gap:** the checked-in production Pedro entry is
    `PhoenixPedroAutoOpModeBase`, whose lifecycle is correct but whose useful behavior is spread
    across the Phoenix profile, readiness, capabilities, path factory, routine factory, routine
    coordinator, and bounded-pre-park coordinator. The seven principal Auto files alone total more
    than 2,000 source lines, before the rest of the Phoenix mechanism and localization graph is
    counted. Existing framework documentation contains isolated route/runtime snippets, and the
    disabled framework examples are flat TeleOps; neither is a compiling end-to-end Pedro Auto.
    The problem is therefore an example and portability gap, not evidence for another scheduler or
    framework lifecycle abstraction.
  - **Existing API and caller audit:** `PedroPathingRuntime.create(...)` already creates the one
    validated drivetrain/localization graph; `pathBuilder()`, `setStartingPose(...)`, and
    `driveAdapter()` expose the supported path, START-pose, route, recurring-heartbeat, and physical
    stop boundaries. `RouteTasks.follow(...)` creates an eager fixed-route Task, and its retained
    execution maps `COMPLETED` to `SUCCESS`, follower/Task timeout to `TIMEOUT`, and interruption,
    replacement, cancellation, failure, or unknown termination to `CANCELLED`.
    `Tasks.branchOnOutcome(...)` can therefore run a capability Task only after confirmed route
    success, run one explicitly named safe fallback only after timeout, and abort without starting
    either continuation after cancellation-like endings. `TaskRunner`, `PlantTasks`, and the
    adapter's same-cycle heartbeat deduplication cover the remaining lifecycle. Current production
    construction uses the project `Constants.createPhoenixAutoRuntime(...)` factory; there is no
    independent robot-side caller. The two public eager `RouteTask` constructors and mutable/
    nullable route config are existing API debt already assigned to CLEAN-01, not a reason for
    EXAMPLE-02 to add another spelling.
  - **Alternatives considered:** expand Markdown snippets only; teach Pedro/Ivy beside Phoenix
    Tasks; reduce the existing Phoenix season Auto to a nominal starter; copy another robot's
    unverified constants into this repository; publish placeholder hardware names in a disabled
    OpMode; add a generic base OpMode, code generator, Auto DSL, or new Pedro facade; provide only a
    constructor-injected test fixture; or combine an independent small example with one explicit
    host wiring boundary. Documentation alone cannot compile-check lifecycle. Phoenix's graph hides
    the very adoption work being taught. A second scheduler/facade or generated plan language adds
    concepts without removing a current limitation. A test-only fixture does not prove FTC
    INIT/START/loop/STOP. Placeholder names are still unsafe when disabled. Copying another robot's
    values would create a second configuration authority and could publish hardware directions or
    tuning that have not been verified for the configuration-owning robot.
  - **Framework Principles and simplicity comparison:** the selected design has one `LoopClock`, one
    Pinpoint owner, one stable Pedro heartbeat, one final drivetrain writer, source-driven mechanism
    output, fresh single-use capability Tasks, explicit outcome policy, and active-only idempotent
    cancellation. It keeps Pedro types in the host/path/integration edge, keeps route geometry and
    Auto policy in robot code, and keeps the annotated OpMode as lifecycle forwarding. The routine
    uses only `RouteTasks.follow(...)`, `Tasks.branchOnOutcome(...)`, and a named capability Task;
    there is no custom Task state machine, base class, generated state, resource/requirements
    system, or public framework API. The implementation audit corrected the original seven-bucket
    estimate: the full reference—not only its short routine—is five Java files, 630 source lines,
    and about 15 concrete concepts once hardware/runtime wiring, FTC lifecycle, composition,
    localization, heartbeat, route execution, Task lifecycle, Plant realization, telemetry, and
    fail-stop cleanup are counted separately.
  - **Chosen basic-reference structure:** add the independent package
    `edu.ftcphoenix.robots.examples.pedro` with (1) `BasicPedroAutoMechanism`, one small Plant-backed
    capability whose factory always returns a fresh cancellation-safe Task; (2)
    `BasicPedroAutoPaths`, one declared Pedro start pose and one eagerly built short practice path;
    (3) `BasicPedroAutoRoutine`, the approximately 15-25-line semantic route/success/timeout graph;
    and (4) `BasicPedroAutoRobot`, the composition root that owns the clock, predictor update,
    adapter heartbeat, runner, mechanism update, and idempotent best-effort stop. Add (5) one
    `@Disabled` Phoenix-hosted test OpMode that constructs those four objects using
    `PhoenixProfile.current()` and `Constants.createPhoenixAutoRuntime(...)` only at the physical
    wiring boundary; it must not instantiate or delegate to `PhoenixRobot`, Phoenix readiness,
    targeting, scoring services, paths, routines, or season strategy. Reuse one real Phoenix
    mechanism mapping from the profile rather than duplicating a name or tuning value. A new robot
    replaces this one host file with its own verified runtime/mechanism construction while retaining
    the lifecycle/routine shape.
  - **Chosen lifecycle and policy:** construct hardware and the fixed path during INIT without
    starting behavior. At FTC START, apply the declared Pedro pose once, reset the shared clock at
    that exact boundary, and enqueue one fresh root routine. Each loop advances
    `clock -> predictor -> driveAdapter heartbeat -> TaskRunner -> mechanism Plant -> telemetry`;
    the route Task may also call the adapter, whose same-cycle guard prevents a second vendor
    update. The fixed route is the deadline-like operation by itself: confirmed completion runs one
    capability action, a truthful timeout runs a safe mechanism fallback, and all cancellation-like
    terminal statuses abort. STOP first cancels and clears the runner, then restores/stops the
    mechanism and immediately stops the Pedro adapter, attempting every cleanup action once even if
    an earlier one fails. No raw `Follower.update()`, `breakFollowing()`, or competing pose owner is
    exposed in the independent package.
  - **Documentation and adoption proof:** add `fw/docs/examples/Pedro Autonomous Reference.md` and
    link it from the examples and Pedro integration indexes. The guide names the ordinary edit
    points: verified hardware/tuning in the host factory, geometry/start pose in `Paths`, semantic
    ordering/timeouts/failure policy in `Routine`, and robot capability realization only when the
    robot does not already have one. It must count all example files/lines/concepts and explain that
    another robot should keep its existing capability Tasks, provide its own paths and routine, and
    wire one thin OpMode/composition root against the completed PEDRO/ROUTE/TASK baseline. Hardware
    names, directions, localization calibration, constraints, and tuning remain owned and verified
    by the adopting robot; none are copied into this reference.
  - **Bounded scope:** this item teaches one fixed practice route and one explicit timeout fallback.
    It does not reopen deferred FIELD-01, add alliance transforms, live-pose construction, vision,
    progress callbacks, deadline-bounded intake, or match-time park takeover. Those belong in the
    already recorded advanced companion after their stated prerequisites, not silently in the
    beginner example. It also does not fix CLEAN-01 route-factory/config debt or extract AUTO-01
    lifecycle ceremony from one example.
  - **Verification plan:** compile TeamCode against pinned Pedro 2.1.2; run the full unit suite; add
    fake-focused tests for START-pose/clock boundaries, exactly one root heartbeat per cycle, route
    success, timeout fallback, cancellation-like abort, direct cancellation, mechanism safe target,
    idempotent best-effort STOP, and INIT-to-STOP forwarding. Statically verify that the independent
    example package imports no `edu.ftcphoenix.robots.phoenix` classes and does not call raw Follower
    lifecycle APIs. Perform a portability audit that identifies the exact framework/runtime
    prerequisites and host-factory replacement required by another robot. In
    Android Studio, review all five Java files and the guide rather than only the routine. Physical
    testing remains a deliberately enabled, low-speed practice-path walkthrough on the
    configuration-owning robot after motor directions, Pinpoint calibration, clear space, and an
    immediate STOP are verified.
  - **Approval (2026-07-15):** the user approved this example ownership/lifecycle design. The
    implementation remains limited to the five-file basic reference, its focused tests and guide,
    and the documented portability audit; no later tracker item or advanced Auto feature is
    included.
  - **Implementation (2026-07-15):** added four Phoenix-season-independent classes under
    `edu.ftcphoenix.robots.examples.pedro`: a writable-Plant intake capability, one fixed 12-inch
    Pedro practice path, an outcome-aware route/capability routine, and the explicit Auto
    composition root. Added the disabled `PhoenixBasicPedroAutoExample` under the Phoenix OpMode
    package as the sole real-hardware host. START applies the declared pose, resets one clock, and
    enqueues without advancing the route; each loop owns
    `clock -> localization -> Pedro heartbeat -> TaskRunner -> Plant`; STOP best-effort cancels and
    clears work, idles/stops the mechanism, and physically stops drive. No framework API, Auto DSL,
    base class, custom Task state machine, placeholder hardware name, or advanced Auto feature was
    added.
  - **Simplicity and boundary audit (2026-07-15):** the exact production counts are 98 lines for
    `BasicPedroAutoMechanism`, 52 for `BasicPedroAutoPaths`, 54 for
    `BasicPedroAutoRoutine`, 209 for `BasicPedroAutoRobot`, and 217 for the Phoenix host: 630 total,
    with 413 in the four independent classes. This is materially smaller than the Phoenix season
    graph but is not honestly a one-line or tiny complete robot. The semantic routine remains about
    20 lines; most bulk is explicit lifecycle and best-effort cleanup. Compressing it with lambdas
    would not remove concepts, while extracting a public lifecycle/cleanup helper from one example
    would be a major API decision without enough evidence. Keep this result as concrete AUTO-01
    evidence. The four independent classes import no Phoenix season type; the concrete routine/root
    intentionally name the example capability, so another robot adapts the pattern rather than
    pretending the class is a drop-in generic base.
  - **Adversarial corrections (2026-07-15):** review made the retained `PathChain` accessor
    package-private, documented the idle-at-routine-entry invariant for cancellation-like aborts,
    ensured partial INIT failure stops a Plant even if capability construction did not take
    ownership, and made host telemetry failures fail-stop the root while preserving cleanup
    failures as suppressed exceptions. The guide reports all 630 lines and 15 concepts, does not
    claim superiority over raw generated scaffolding by line count, and distinguishes reusable
    framework guarantees from robot-owned route and mechanism policy.
  - **Portability audit (2026-07-15):** the example has no dependency on another robot project or
    its hardware constants. An adopting robot keeps its existing capability Tasks, syncs the
    supported PEDRO/ROUTE/TASK baseline, and adapts the example's paths/routine/root/host pattern.
    Its owners must supply and verify motor names and directions, localization calibration, Pedro
    constraints, and follower tuning before enabling Auto. This repository does not claim that a
    successful source-level adaptation proves another robot's physical configuration.
  - **Automated verification (2026-07-15):** Android Studio's bundled JBR completed
    `:TeamCode:compileDebugJavaWithJavac` and the focused EXAMPLE-02 tests, then
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac`. The full result is 389 tests,
    0 failures, 0 errors, and 0 skipped; 9 tests directly cover capability cancellation, truthful
    route success/timeout/cancellation policy, START and first-loop ordering, exact-once
    best-effort STOP, FTC lifecycle forwarding, and telemetry-failure fail-stop. Static checks found
    no Phoenix imports in the four independent classes, no raw/blocking follower lifecycle calls,
    no direct framework Task construction, no trailing whitespace, and no broken local links in the
    four affected documentation files. The only compiler output was the repository's existing
    Java-8-on-JDK-21 deprecation warning.
  - **Android Studio audit point:** inspect all five production Java files and
    `fw/docs/examples/Pedro Autonomous Reference.md`, not only the short routine. Confirm the
    declared `(24, 24, 0)` to `(36, 24, 0)` Pedro practice geometry, success/timeout/cancellation
    policy, enqueue-only START, loop ownership order, best-effort STOP, the disabled real-profile
    host boundary, complete line/concept accounting, and candid portability limits.
    Physical drive/Pinpoint/intake behavior remains unverified; deliberately enable this test only
    after directions, calibration, conservative constraints, clear space, and immediate STOP are
    checked on the configuration-owning robot.
  - **Manual verification (2026-07-15):** the user confirmed the five-file implementation in
    Android Studio and approved publication after the framework documentation and this item record
    were made project-neutral. No production Java behavior changed during that wording adjustment.
    Robot-hardware validation remains deliberately outstanding as documented above.

### AUTO-01 - Compact bounded Auto continuation

- **Problem to confirm:** PHX-04 keeps the supported public installation entry point short, but its
  two private Phoenix bounded-pre-park/coordinator Tasks contain 1,012 source lines. Because all code
  under `edu.ftcphoenix.robots.phoenix` is robot code, a new season or robot may otherwise copy a
  large amount of single-use, same-cycle/reentrant, outcome-validation, cancellation, cleanup
  aggregation, one-continuation handoff, and telemetry machinery merely to express its own park
  policy. Determine how much is essential robot strategy and how much is repeated lifecycle ceremony.
- **Prerequisite evidence:** complete or prototype `EXAMPLE-02` as a genuinely independent compact
  Pedro Auto, then compare at least one materially different bounded-continuation routine. Count the
  robot-code concepts and lines each approach requires; a short outer factory call is not sufficient
  evidence of simplicity.
- **Reprioritization state (2026-07-17):** EXAMPLE-02 recorded the full 630-line independent
  reference baseline before ROUTE-03; route-config cleanup reduced it to 626 lines, and COMMON-01
  cleanup aggregation has since reduced the current reference to 566 lines. The first prerequisite
  remains closed. Bettabot still has only one-shot shooter Tasks and no real bounded Pedro routine,
  so the second caller needed to distinguish reusable continuation mechanics from Phoenix's match
  policy does not yet exist. Keep this item high for the first competition Auto, but do not
  implement it for an ordinary single-route routine or manufacture a generic Auto DSL from the
  remaining evidence.
- **Alternatives to compare:** retain the tested explicit pattern; simplify only the Phoenix private
  Tasks; document a copyable template; add a robot-local helper; reuse or reshape existing
  `sequence(...)`/`branchOnOutcome(...)` composition; extract a narrow lifecycle utility; or add a
  generic factory-only composition only if multiple real callers prove identical semantics. Compare
  one continuation identity, truthful retained outcomes, cleanup ownership, reentrant/same-cycle
  behavior, diagnostic quality, and the number of concepts a student must maintain.
- **Leading hypothesis:** do not add a framework API from PHX-04 alone. Use the independent examples
  to identify repeated lifecycle/outcome mechanics, then extract only the smallest proven common
  shell. Keep route-status meaning, park eligibility, match threshold, mechanism cleanup, and
  recovery policy robot-owned. Do not assume a public Auto DSL, abstract base class, generic cleanup
  callback, or match-specific framework API is the answer.
- **Completion:** a new-season reference expresses bounded pre-park work, allowed terminal results,
  safe cancellation, and one live-start park in compact, auditable robot code without copying a large
  lifecycle state machine. Tests retain exact START timing, reentry and same-cycle safety,
  cleanup-failure suppression, exactly-once continuation, and truthful final outcomes. If no
  abstraction is simpler than a tested private template, record that evidence-based no-change result
  explicitly rather than adding another API spelling.
- **Pause record (2026-07-19):** **Deferred for a second materially different real bounded-Auto
  caller.** This is a caller-evidence gate, not a hardware gate. EXAMPLE-02 closes the independent
  reference prerequisite, but Phoenix remains the only caller with the complete bounded-continuation
  lifecycle. Resume when another real robot routine needs the behavior; do not manufacture a generic
  Auto DSL or count a one-route routine as evidence.

### EXAMPLE-03 - Advanced moving-target reference

- **Problem to confirm:** Phoenix architecture documents describe spatial targeting and bounded
  realization, but there is no small compiling reference proving that a competition-style moving
  turret/shot-on-the-move service and route-progress scoring event fit the existing source, Plant,
  capability, and Task vocabulary. Without that proof, students may copy mutable global calculations
  or conclude the framework needs season-specific shooter APIs.
- **External evidence:** Cuttlefish computes lead from robot pose, translational/angular motion, and
  target geometry in
  [`Context`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Context.java),
  applies a bounded turret realization in
  [`Turret`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Turret.java),
  and uses Pedro parametric callbacks to request mechanism actions while routes continue.
- **Alternatives to compare:** rely on architecture prose; put generic projectile/lead equations in
  the framework; create shooter/turret capability interfaces; add a standalone tested cookbook; or
  extend the compiling Pedro example with one advanced robot-owned service and fake-HAL realization.
- **Leading hypothesis:** add a bounded compiling/tested advanced example, not new game vocabulary.
  A robot-owned service combines timestamped field/motion facts into a desired target; a periodic
  source and `PlantTargets.plan(...)` feed one bounded PositionPlant; readiness/status stays in a
  robot capability; and a Pedro progress callback only requests that capability. Physics policy,
  target choice, and fallback remain in the example robot.
- **Completion:** fake-HAL tests cover motion compensation inputs, stale/unavailable localization,
  reachable/unreachable target range, bounded output, fallback, readiness, route callback
  idempotency, cancellation, and safe stop. The guide identifies the few student edit points and
  explains why no direct writer, background coroutine, projectile framework, or scoring API is
  required.
- **Decision record:** _Pending._

### BOUNDARY-01 - FTC boundary enforcement

- **Problem to confirm:** FTC SDK imports exist outside the documented `fw.ftc`/tools boundary, and
  module layout does not enforce the distinction.
- **Alternatives to compare:** move the few leaks and add a static import rule; Gradle source sets;
  separate core/FTC modules; or tolerate documented exceptions.
- **Leading hypothesis:** fix imports and add a focused check first. Split modules only if testability
  or dependency isolation provides enough benefit to offset student/maintainer complexity.
- **Completion:** every exception is removed or explicitly justified, and CI prevents accidental new
  core-to-SDK dependencies.
- **Decision record:** _Pending._

### DOC-01 - Stale and non-compiling documentation

- **Problem to confirm:** some guides/Javadocs reference no-op or nonexistent APIs, omit required
  clock updates, or contain broken links.
- **Decision question:** which examples can be compiled directly, and which need lightweight link or
  snippet validation?
- **Leading hypothesis:** fix factual errors immediately, then compile a small set of canonical
  examples rather than building a complex documentation system.
- **Completion:** canonical loop, task, Plant, and drive examples match real APIs; repository Markdown
  links pass a focused checker.
- **Decision record:** _Pending._

### CI-01 - Framework verification in CI

- **Problem to confirm:** existing workflows do not compile or test framework changes.
- **Alternatives to compare:** unit tests plus TeamCode compile; full FTC build; docs/boundary checks;
  and local-only scripts if hosted CI cannot legally obtain dependencies.
- **Leading hypothesis:** use the narrowest reliable unit-test and TeamCode compile jobs, then add
  cheap docs and import checks.
- **Completion:** the documented local commands and hosted checks agree; failures are actionable.
- **Decision record:** _Pending._

### CLEAN-01 - Alias and risky convenience cleanup

- **Problem to confirm:** aliases and adjacent-`double` overloads create parallel learning paths;
  level-triggered task bindings may enqueue unbounded work.
- **Decision question:** which APIs have no callers or demonstrably unsafe semantics, and which names
  genuinely improve discoverability?
- **Leading hypothesis:** remove only after caller search and after the preferred path is documented;
  prioritize unsafe task-enqueue helpers over cosmetic naming cleanup.
- **Completion:** one obvious documented path remains, all callers migrate, and no unrelated naming
  churn is bundled with behavioral work.
- **Scope split (2026-07-17):** public `RouteTask` construction and its one-field/null configuration
  path moved to focused ROUTE-03 so a Pedro simplification is not bundled with unrelated aliases or
  binding behavior.
- **Decision record:** _Pending._

### CTRL-01 - Final scalar-regulator output constraints

- **Problem to confirm:** Phoenix can limit a `PidController`'s contribution, but later
  `ScalarRegulator` decorators can add feedforward or voltage compensation after that limit. A robot
  that needs an intentional final command range must therefore write a custom wrapper or rely on the
  actuator's broader defensive saturation, which neither expresses the requested range nor reports
  the constrained control-law result.
- **External evidence:** Bettabot's
  [`CappedFlywheelRegulator`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L77-L136)
  adds about sixty lines to constrain the complete PIDF command to `[0, maximumOutputPower]` and
  expose useful tuning diagnostics. Its zero-setpoint reset/coast behavior is additional
  flywheel-specific policy and is not evidence for a universal stop rule.
- **Alternatives to compare:** keep custom robot regulators; document careful decorator ordering;
  use `Pid.setOutputLimits(...)`; rely on final FTC `[-1, +1]` saturation; add a regulated-Plant
  builder stage; add one factory-only `ScalarRegulator` decorator; or make saturation visible to a
  controller for anti-windup. Trace every current regulator construction path and distinguish final
  output limiting from integral limiting, non-finite-output defense, Plant target bounds, and
  actuator saturation. SAFE-03 separately owns the universal normalized-command invariant for
  regulated Plants that do not opt into a narrower policy constraint.
- **Leading hypothesis:** add one factory-only `ScalarRegulators.outputLimited(...)` decorator, with
  finite ordered bounds, delegated reset, and debug fields for unconstrained
  output, applied output, and whether limiting occurred. It composes explicitly around PIDF or
  voltage compensation and adds no public concrete class, duplicate instance spelling, Plant-builder
  branch, flywheel vocabulary, automatic zero-setpoint reset, or claim of saturation-aware
  anti-windup.
- **Anti-windup boundary:** an outer decorator cannot generically feed its saturation state back into
  an arbitrary inner regulator. Compare existing integral limits/reset, controller-specific
  conditional integration or back-calculation, and a richer regulator contract using focused
  saturation/recovery evidence. The leading hypothesis is to document that final limiting is not
  automatic anti-windup rather than complicating every `ScalarRegulator` from Bettabot's currently
  zero-`kI` case.
- **Completion:** pure tests cover exact boundaries, positive/negative saturation, decorator order
  with PIDF and voltage compensation, repeated updates, reset propagation, debug state, invalid
  bounds, and non-finite inner output. A regulated-Plant test with a fake `PowerOutput` proves the
  selected limit reaches the output and its diagnostics agree. Javadocs and the shooter examples show
  one obvious composition path and clearly separate generic limiting from robot-owned disable/coast
  policy; hardware output adapters retain cheap defensive saturation.
- **Decision record (2026-07-15):**
  - **Confirmed behavior and minimal trace:** `Pid.update(...)` clamps only its own `P + I + D`
    result. `ScalarRegulators.pidf(...)` then adds setpoint feedforward, and
    `voltageCompensated(...)` can scale that combined result afterward. For example, a PID limited to
    `[-0.65, +0.65]` returns `0.65`; `0.20` feedforward makes the regulator result `0.85`; a `1.30`
    voltage multiplier then makes it `1.105`. Neither the PID limit nor an eventual FTC
    `[-1, +1]` clamp expresses an intentional complete-command range such as `[0, 0.65]`. The issue
    is therefore confirmed by the traced current call order without inventing a new Plant behavior.
  - **Current callers and construction paths:** `ScalarRegulators` is already the sole public
    construction facade for built-in Plant regulators: `pid(...)`, the function and linear-gain
    `pidf(...)` overloads, `setpointFeedforward(...)`, and `voltageCompensated(...)`; every concrete
    implementation is private. The three student-facing regulated `FtcActuators` paths (motor
    velocity, motor position, and CR-servo position) all accept an already composed
    `ScalarRegulator` and pass it to `MappedVelocityPlant` or `MappedPositionPlant`. The lower-level
    `Plants.positionFromPower(...)` and `velocityFromPower(...)` paths do the same. The only compiled
    in-repository call site is the external-sensor lift example, which uses a plain PID regulator and
    needs no migration. There is no current Phoenix-season caller. The FTC actuator guide's
    PIDF-plus-voltage flywheel is the canonical documentation caller to update.
  - **Public construction-layer audit:** retain `ScalarRegulator` as the custom-control-law extension
    seam and retain `ScalarRegulators` as the only built-in construction/decorator layer. The
    `pidf(controller, double kF)` overload has distinct student value because it removes a lambda for
    the common linear case. `Pid.setOutputLimits(...)` remains a distinct controller-contribution
    limit when PID is the complete control law; it is not another spelling for complete-regulator
    limiting. `ScalarControllers` remains a distinct cycle-memoized `ScalarSource` family and must
    not gain a sibling limiter. The audit also found that public `new Pid(kP, kI, kD)` and
    `Pid.withGains(...)` are equivalent construction spellings with no in-repository constructor
    caller. CTRL-01 did not bundle that unrelated deletion; API-03's later full
    controller-construction audit now owns the factory-only migration instead of CLEAN-01.
  - **Alternatives considered:** keep robot-local wrappers; document ordering only; use
    `Pid.setOutputLimits(...)`; rely on FTC saturation; add a limit to each regulated-Plant builder;
    publish a concrete/status-bearing limiter type; add one factory-only generic decorator; or add a
    controller feedback contract for saturation-aware anti-windup. No change leaves each robot to
    repeat validation, clamp math, reset/debug delegation, and state reporting. Documentation and
    PID limits cannot enforce a range after later decorators. FTC saturation is broader, late, and
    cannot express asymmetric or intentionally reduced ranges. A Plant-builder branch duplicates
    the same operation across three staged builders, hides ordering, and omits lower-level users.
    Generic anti-windup cannot be inferred backward through arbitrary feedforward and scaling and is
    unsupported by the current zero-`kI` evidence.
  - **Simplicity comparison:** the normal public call adds one discoverable outer factory around the
    already composed regulator:

    ```java
    ScalarRegulator regulator = ScalarRegulators.outputLimited(
            ScalarRegulators.voltageCompensated(pidf, voltage, 13.0, 9.0, 1.4),
            0.0,
            maximumOutputPower);
    ```

    Students learn one distinction: a PID output limit bounds the PID contribution, while
    `outputLimited(...)` bounds everything inside it. They do not learn a public limiter class,
    status interface, builder branch, bounds object, instance/default-method twin, or symmetric-max
    overload. The composition order remains visible at the call site and construction failures name
    the invalid argument.
  - **Chosen public API:** add only
    `ScalarRegulators.outputLimited(ScalarRegulator inner, double minOutput, double maxOutput)`,
    returning the declared `ScalarRegulator` interface and backed by one private implementation.
    Bounds are inclusive, finite, and ordered with `minOutput <= maxOutput`; equal bounds are valid,
    reversed bounds are rejected rather than silently swapped, and bounds are not restricted to
    `[-1, +1]` because regulator command units are generic. The name deliberately omits `final`:
    another output-changing decorator placed outside it could change the result, so documentation
    requires the intentional complete-command limit to be outermost.
  - **Update, failure, reset, and diagnostics contract:** each call invokes the inner regulator
    exactly once with the supplied operands; there is no same-cycle cache because repeated calls may
    intentionally use different setpoints or measurements. A finite result is constrained by exact
    range comparison, with exact-boundary values reported as not limited. `NaN` or either infinity
    from the inner regulator throws an actionable `IllegalStateException` rather than converting a
    numerical failure into a possibly dangerous command. `reset()` delegates once and clears local
    numeric diagnostics to `NaN`. Stable debug fields report `minOutput`, `maxOutput`,
    `lastUnconstrainedOutput`, `lastOutput`, and `lastOutputLimited`, then nest the inner regulator's
    diagnostics under `.inner`. The factory exposes no typed `wasLimited()` accessor; limiting is
    tuning/debug information unless a future real operational caller proves otherwise.
  - **Ownership and safety boundary:** this decorator owns one optional control-law policy range. It
    does not change Plant target bounds or `Plant.getAppliedTarget()`, does not guarantee downstream
    grouped/scaled child commands, does not replace FTC adapter defense, and does not make universal
    finite normalized-output truth optional. SAFE-03 retains those Plant/output responsibilities.
    It also does not provide saturation-aware integral control: callers retain explicit integral
    limits and robot-owned controller reset/enable policy.
  - **Summer26/Bettabot simplification:** the current 60-line `CappedFlywheelRegulator` combines two
    different responsibilities. CTRL-01 moves reusable bounds validation, clamp math, unconstrained/
    applied/limited bookkeeping, limiter debug keys, and the custom `wasLimited()` machinery into
    the framework. Exact current behavior still requires a small robot-owned
    `CoastingFlywheelRegulator`: when the flywheel setpoint is non-positive it resets its inner
    regulator and returns exactly zero; otherwise it delegates. A `[0, maximum]` limit alone is not
    equivalent because positive integral history or a negative/noisy velocity measurement can still
    request positive power at a zero setpoint. Keeping that 29-line policy wrapper inside the new
    outer `outputLimited(...)` call reduces the complete custom class and call-site burden by about
    30 lines. Moving the non-operational `powerLimited` field and its presenter plumbing to the
    standard regulator/Plant debug output removes about five more, reducing the pinned robot Java
    surface from 1,047 to about 1,012 physical lines. PID/PIDF tuning, maximum-power configuration,
    feedback units, Plant wiring, and exact coast/reset policy correctly remain robot code.
  - **Rejected and deferred designs:** do not add a public limiter class or diagnostic subtype merely
    to preserve one telemetry boolean; do not add `ScalarRegulator.outputLimited(...)`, an `of(...)`
    alias, a max-absolute overload, a range value type, or three Plant-builder spellings. Do not make
    zero-setpoint coast/reset automatic: zero velocity may mean active holding/braking for another
    mechanism. A generic conditional-activation decorator is not justified by this one policy
    caller. Do not modify PID construction aliases, controller parameter validation, universal
    regulated-Plant command truth, signal filtering, or tuning workflow in this item.
  - **Verification plan:** add pure `ScalarRegulatorsTest` coverage for null input; every non-finite
    and reversed bound; equal bounds; exact boundaries and one-ULP excursions; positive/negative
    limiting; PID-limit-then-feedforward order; correct and deliberately incorrect voltage/
    constraint order; all non-finite inner results; repeated same-cycle calls; exact reset
    delegation; null-safe debug; stable fields; and nested diagnostics. Add one focused fake-output
    case to `MappedVelocityPlantSafetyTest` proving the selected regulator result reaches the
    semantic `PowerOutput` seam while the Plant's applied target remains in mechanism units. Update
    `ScalarRegulator`/`ScalarRegulators` Javadocs, the generic FTC actuator/flywheel guide, the layered
    shooter guide, and the stable Framework Principles distinction among controller contribution,
    complete-regulator policy, and Plant/output safety. Framework documentation must remain project-
    neutral. Run the focused tests, full `:TeamCode:testDebugUnitTest`, and
    `:TeamCode:compileDebugJavaWithJavac`; inspect XML totals, caller/API searches, changed Markdown
    links, and `git diff --check`; then request Android Studio review. Robot-hardware verification
    remains downstream: an adopting robot must check zero and maximum boundaries, no reverse drive,
    reset-clean re-enable behavior, diagnostics, and its owning lifecycle's stop response after the
    deliberate non-finite-result exception.
  - **Approval gate:** this adds a public regulator-composition API and fixes its ordering, numerical
    failure, reset, and diagnostics contract. It is therefore a major API decision. Stop at
    **Ready** for explicit user approval before editing Java, tests, or framework documentation.
  - **Approval (2026-07-15):** the user approved the factory-only
    `ScalarRegulators.outputLimited(inner, minOutput, maxOutput)` design, including outermost
    composition, finite ordered bounds, fail-fast non-finite output, delegated reset/debugging, no
    typed limiting-status API, and the robot-owned zero-setpoint coast/reset boundary.
  - **Implementation (2026-07-15):** `ScalarRegulators` now exposes the approved single
    `outputLimited(inner, minOutput, maxOutput)` factory backed by a private decorator. Construction
    rejects null inner regulators, non-finite bounds, and reversed bounds with argument-specific
    values; equal bounds remain valid. Each update delegates exactly once, constrains a finite result
    by exact inclusive comparison, records stable nested diagnostics, and throws with remediation
    guidance after retaining a non-finite result. Reset delegates once and clears limiter state. No
    concrete/status type, instance/default-method spelling, overload, bounds object, Plant-builder
    branch, automatic coast/reset policy, or anti-windup contract was added.
  - **Documentation and focused tests (2026-07-15):** `Pid`, `ScalarRegulator`, and
    `ScalarRegulators` Javadocs plus Framework Principles and the generic actuator/layered-shooter
    guides now distinguish PID contribution limits, outermost complete-regulator policy, and
    Plant/output command safety while retaining a short beginner PID example. Framework docs remain
    project-neutral. A new `ScalarRegulatorsTest` has 11 focused cases covering configuration,
    boundaries, ordering, repeated calls, failure diagnostics, reset, and debug nesting; the existing
    mapped-velocity safety suite adds one semantic-output integration case. The focused XML total is
    21 tests with zero failures, errors, or skips.
  - **Automated verification (2026-07-15):** Android Studio's bundled JBR completed the focused
    regulator/mapped-velocity suites and then
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac`. The full XML result is 44
    suites and 401 tests with zero failures, errors, or skips. The only compiler output is the
    repository's existing Java-8-on-JDK-21 source/target deprecation warning.
  - **Static and independent verification (2026-07-15):** `git diff --check`, an affected-file
    trailing-whitespace scan including the untracked new test, the new same-document Markdown anchor,
    exhaustive `ScalarRegulator` factory/implementation/caller searches, and project-neutrality
    searches all pass. Three independent reviews covered API construction-layer scope, numerical and
    reset/debug correctness, test validity, Framework Principles, student simplicity, and
    documentation. The reviews corrected a weak decorator-order test, aligned actionable error text,
    restored the short beginner Javadoc example, and removed a duplicated guide snippet; final review
    found no blocker. The intentional remaining boundary is explicit: `outputLimited(...)` owns no
    hardware and therefore cannot itself zero a previously commanded actuator after throwing on a
    non-finite inner result. The owning lifecycle and SAFE-03 retain stop/failure and universal
    regulated-command responsibilities.
  - **Android Studio audit point:** inspect `ScalarRegulators.outputLimited(...)` and its private
    implementation, the `Pid`/`ScalarRegulator` Javadocs, the three-layer rule in Framework
    Principles, the actuator and layered-shooter examples, `ScalarRegulatorsTest`, and the added
    mapped-velocity integration case. Confirm that the limit is outermost after PIDF/voltage
    compensation, equal/inclusive bounds and non-finite failures read clearly, diagnostics distinguish
    unconstrained from applied output, and no public concrete type, status interface, Plant-builder
    spelling, zero-setpoint policy, or anti-windup claim was introduced. This repository has no
    production hardware caller, so robot testing is not required for this pure decorator. When an
    adopting flywheel migrates, separately verify exact coast/reset policy, zero and maximum output,
    no reverse drive, clean re-enable, diagnostics, and owning-lifecycle cleanup on the real robot.
  - **Manual verification (2026-07-15):** the user confirmed the CTRL-01 implementation in Android
    Studio and approved publication. No robot-hardware run is claimed for this pure decorator; the
    downstream adopting-robot checks remain as documented above.

### SOURCE-02 - Derived rate from sampled scalar position

- **Problem to confirm:** Phoenix exposes raw position and direct SDK velocity sources but has no
  reusable way to derive rate from position samples. Bettabot therefore embeds FTC SDK access,
  sample history, timing, reset, and same-cycle deduplication in a robot-specific class. The same
  need can apply to any encoder or position sensor whose direct velocity is absent, overflows, is
  too quantized, or has unsuitable semantics.
- **External evidence:** Bettabot's
  [`ThroughBoreVelocitySource`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L30-L75)
  differences `getCurrentPosition()` using `LoopClock.dtSec()` to avoid a reported REV velocity-
  counter limitation. The implementation is not actually Through Bore-specific, and its timing is
  only correct while it is sampled every cycle. Cuttlefish independently uses the same acquisition
  direction in its Decode shooter: its
  [`Shooter`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Shooter.java)
  reads `getCurrentPosition()`, divides the position delta by measured elapsed time and 8192
  counts/revolution, then applies explicit low-pass stages. Its checked-in
  [`decode.xml`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/res/xml/decode.xml)
  puts the encoder-bearing left flywheel on hardware-counted Expansion Hub port 3. The user's report
  that this external Through Bore path worked is useful field-shaped adoption evidence, but the
  public snapshot contains no retained tachometer comparison or lost-edge result and does not by
  itself certify exact-stack accuracy.
- **Layer boundary to preserve:** `FtcSensors` owns device lookup, raw cycle-memoized readings, and
  any confirmed SDK/controller representation correction such as a hardware counter-width defect.
  Core Source composition owns device-independent delta-over-elapsed-time estimation; a generic
  periodic-position contract, if justified, is distinct from correcting a particular SDK counter.
  SOURCE-03 owns optional smoothing or outlier conditioning. Direct
  `nativeFeedback(ScalarSource)` on the already-domain-specific regulated builder stage remains the
  advanced Plant seam; no immediately consumed feedback-selector wrapper is retained.
- **Alternatives to compare:** retain one custom class per robot; always trust `DcMotorEx.getVelocity()`;
  add a Through Bore or FTC-only helper; derive rate as a generic core Source transform; expose a
  separate estimator object; or add a Plant-feedback-only option. Compare actual SDK velocity with
  position-derived results on supported hubs, motors, internal/external quadrature encoders, and
  relevant loop rates. Define first-sample, skipped-cycle, duplicate-cycle, reset, non-finite,
  counter-rollover, wrapped/absolute-position, discontinuity, and direction/unit semantics before
  choosing the API.
- **Original hardware decision gate, now adoption validation:** record direct SDK velocity and position-derived
  velocity on the pinned SDK and actual REV hardware across the Through Bore encoder's expected
  counts/revolution, RPM, direction, stop, and representative loop-rate envelope. Determine whether
  any discrepancy is acquisition/counter representation, generic estimation, or filtering. Put a
  correction only in that owning layer. This exact-stack run remains necessary before claiming
  Bettabot hardware accuracy or match readiness, but after the approved contract reclassification
  it no longer blocks the generic framework transform or the staged external-feedback path.
- **Leading hypothesis:** core owns one resettable, cycle-idempotent position-to-rate calculation that
  accepts any `ScalarSource` in caller-chosen position units and returns the same units per second.
  It uses elapsed time between its own accepted samples rather than assuming it was sampled during
  the immediately preceding loop. `FtcSensors` continues to own raw SDK position and direct-velocity
  acquisition. Make the already-domain-specific regulated builder stage the only public feedback-
  selection layer: it exposes direct `internalEncoder(...)`, `averageInternalEncoders()`,
  `externalEncoder(...)`, and advanced `nativeFeedback(ScalarSource)` answers. Delete the redundant
  public `PositionFeedback`/`VelocityFeedback` wrapper factories. On the velocity stage,
  `externalEncoder(...)` deterministically resolves continuous FTC position followed by the generic
  rate transform, while internal encoders retain direct SDK velocity for the reviewed built-in motor
  encoders. Add no runtime strategy inference, second external-encoder selector, or parallel core
  factory spelling. Do not identify the core estimator with Through Bore hardware, silently replace
  the raw direct SDK source globally, or hide smoothing inside either path.
- **Completion:** fake-clock tests cover startup, normal and irregular intervals, skipped and repeated
  cycles, reset/restart, zero or invalid elapsed time, direction, unit mapping, non-finite samples,
  wrap/discontinuity policy, and debug output. FTC-boundary tests cover signed 32-bit rollover and
  staged internal/direct versus external/derived selection. Documentation demonstrates the same
  estimator with at least two position-source kinds and shows one concise regulated-Plant
  integration without leaking FTC types into core. Exact-stack usable range, count retention, and
  control performance remain explicitly deferred adopting-robot evidence rather than framework
  completion claims.
- **Decision record (direct staged API and observed-position contract approved 2026-07-16):**
  - **Current callers and public layers:** repository search found no second handwritten derivative;
    Bettabot is the concrete caller. The three regulated feedback stages already establish velocity,
    motor position, or CR-servo position before asking for feedback. The public `PositionFeedback`
    and `VelocityFeedback` objects are constructed only to be passed immediately into those stages;
    no repository caller stores or reuses one, and their `fromSource(ScalarSource)` methods add no
    dimensional type safety because `ScalarSource` itself is unit-agnostic. They are therefore a
    redundant public construction layer. `ScalarSource` instance methods remain the one-input core
    composition layer, `FtcSensors` owns reusable raw FTC acquisition, and lower-level regulated
    Plants consume a final measurement Source.
  - **Supplied-source capability and disposition:** an arbitrary native measurement remains a
    distinct advanced capability for an analog tachometer, vendor device, simulated source, fused
    measurement, or explicitly composed/conditioned signal that has no FTC motor-port encoder name.
    Removing that capability would force such callers to abandon the staged builder. It does not,
    however, justify public `fromSource(...)` wrapper factories. Each domain-specific feedback stage
    should accept `nativeFeedback(ScalarSource)` directly. The concrete position example using an
    analog lift-height source migrates mechanically to that method. A named `externalEncoder(...)`
    remains the ordinary acquisition answer; overloading it with an arbitrary scalar would misname
    non-encoder measurements and make it unclear whether the value is position or already rate.
  - **Pinned-SDK representation finding:** this repository pins FTC SDK 11.1.0. Inspection of its
    `Hardware` source JAR traces Lynx motor velocity through `LynxGetBulkInputDataResponse`, whose
    protocol payload and Java storage are signed 16-bit values; motor position is signed 32-bit.
    `LynxDcMotorController` returns that velocity without host-side overflow correction. Direct
    velocity can therefore represent only `-32768..32767` counts/second. Source inspection proves
    that a higher count rate cannot be represented truthfully, but does not prove whether a given
    hub firmware wraps, saturates, or fails another way; that observed behavior remains part of the
    hardware run.
  - **Encoder-spec comparison:** the relevant distinction is count rate at the hub port, not the
    motor brand or the output-shaft counts/revolution by itself.
    - A [REV HD Hex](https://docs.revrobotics.com/duo-control/sensors/encoders/motor-based-encoders)
      has 28 counts per motor revolution at 6000 RPM. Its 20:1 and 40:1 gearboxes multiply output
      counts/revolution while reducing output RPM by the same factor, so each configuration reaches
      only about 2800 counts/second at free speed. The REV Core Hex's 288 output counts/revolution at
      125 RPM is about 600 counts/second. Both have wide signed-16 velocity margin.
    - A current [goBILDA 5203 Yellow Jacket](https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/)
      likewise has 28 counts per encoder-shaft revolution at 6000 RPM, or about 2800
      counts/second. Gearbox examples preserve approximately that count rate: the official
      [19.2:1 model](https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/)
      is 537.7 output counts/revolution at 312 RPM, or about 2796 counts/second.
    - Both [REV Through Bore V1](https://www.revrobotics.com/content/docs/REV-11-1271-DS.pdf)
      and [V2](https://www.revrobotics.com/rev-11-3174/) specify 8192 incremental counts/revolution
      and a 10000 RPM sensor limit. On the hub's signed-16 velocity representation, positive direct
      velocity reaches its limit at about 240 RPM. Bettabot's configured 1000 RPM default is about
      136533 counts/second and its 5000 RPM maximum is about 682667 counts/second, so raw
      `getVelocity()` cannot represent either. The sensor's own maximum rating would be about
      1365333 counts/second; REV does not publish a matching maximum count rate for the hub encoder
      port, so successful high-rate position capture must be measured rather than inferred. FIRST's
      [Control/Expansion Hub guidance](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips.html)
      says ports 0 and 3 are hardware-counted while ports 1 and 2 are software-counted, and names the
      high-resolution Through Bore as a device that should use port 0 or 3 to avoid missed counts.
      A claimed high-rate path must therefore use and record port 0 or 3.
  - **Position-API finding:** `DcMotor.getCurrentPosition()` is the pinned SDK's ordinary public
    position API. On a Lynx hub it returns the same signed 32-bit position field available from
    `LynxModule.BulkData.getMotorCurrentPosition(...)`; direct bulk-data access supplies neither a
    wider counter nor a more precise value. `DcMotorEx.getVelocity(AngleUnit)` only converts the
    same signed-16 direct velocity and is not an alternative acquisition. Consecutive accepted
    `getCurrentPosition()` readings can infer interval-average velocity without that signed-16
    range limit, but no SDK method can prove that the selected port captured every electrical edge.
    Correct wiring on hardware-counted port 0/3, coherent bulk-cache lifecycle, a safely bounded
    interval, and comparison with an independent reference remain necessary.
  - **Layer conclusion:** keep motor/encoder lookup, direction normalization, raw position and raw
    velocity, quadrature-port behavior, and any proven Lynx signed-16 correction in `FtcSensors`.
    Keep delta-position divided by elapsed sample time device-independent in core. Caller-selected
    CPR/unit scaling remains ordinary Source composition, and robot configuration owns which sensor
    is physically attached and what its value means. A wrapped absolute-angle source must be
    explicitly unwrapped by a layer that knows its period before generic differentiation. SOURCE-03
    continues to own optional smoothing/outlier handling; neither acquisition path may hide it.
  - **Construction-time dispatch:** after `.position().regulated()` or
    `.velocity().regulated()`, the staged builder itself provides the required distinction; neither
    the regulator implementation nor a sampled value must be inspected. On a velocity stage,
    `internalEncoder(...)` resolves to direct SDK velocity and `externalEncoder(...)` resolves to a
    rollover-aware FTC position source followed by the generic elapsed-time rate transform. On a
    position stage, the same encoder-named methods resolve position. `nativeFeedback(source)` means
    the advanced Source already reports the native measurement required by that stage and is passed
    unchanged. `MappedVelocityPlant.update(clock)` already supplies the shared `LoopClock`, and its
    reset already resets measurement history, so no new heartbeat, constructor-time sample, or public
    lifecycle method is required. The regulator remains hardware-agnostic: it receives the final
    native measurement. A physically external encoder must use `externalEncoder(...)` even when it
    occupies the powered motor's encoder channel or shares its configured hardware name; the SDK
    cannot verify that physical fact.
  - **Plant-facade refinement after user review:** the existing
    velocity-stage `externalEncoder(name[, direction])` answer should mean "obtain trustworthy native
    velocity from this external incremental encoder," not "call `DcMotorEx.getVelocity()`." Its
    current private resolver does the latter, but the builder and `MappedVelocityPlant` already own
    construction, measurement reset, sampling, unit conversion, and regulator delivery, so they can
    hide the stateful position-derived source without adding robot code. Keep the velocity-stage
    `internalEncoder(...)` answer on direct SDK velocity for the reviewed low-count-rate motor
    encoders. Keep direct `nativeFeedback(source)` as the advanced already-domain-specific seam.
    This materially refines the earlier assumption that the Plant layer added no distinct value and
    removes the redundant selector-object layer instead of adding another API.
  - **Feedback API alternatives:** retaining the two wrapper families preserves explicit nouns but
    makes students restate the domain already chosen by `.position()`/`.velocity()` and exposes
    public selector objects with no demonstrated reuse. Replacing them with one public
    `EncoderFeedback` object removes one repeated noun but still creates an immediately consumed
    factory object and cannot honestly represent a non-encoder native source. Accepting only a raw
    `ScalarSource` would force ordinary callers to know FTC acquisition, direction, internal-group
    selection, and direct-versus-derived behavior. The smallest coherent design is therefore direct
    answers on the staged feedback question. Motor position and velocity stages expose parallel
    `internalEncoder()`, `internalEncoder(name)`, `averageInternalEncoders()`,
    `externalEncoder(name[, direction])`, and `nativeFeedback(source)` methods. CR-servo position
    exposes only external encoder and native-source answers; adding impossible internal choices for
    visual symmetry is rejected. Delete the old wrapper classes rather than retaining parallel
    spellings or deprecations.
  - **Approach comparison:** direct SDK velocity remains the simplest recommended acquisition for
    the reviewed built-in motor encoders and must not be replaced globally. It is structurally
    unusable for Bettabot's high-rate Through Bore. A generic elapsed-time position derivative is
    the smallest reusable way to remove Bettabot's robot-owned history/timing code and is the fixed
    default behind `externalEncoder(...)`. `DcMotor.getMotorType()` does expose configured
    ticks/revolution and maximum RPM, but the SDK says that type is assigned in the robot
    configuration and may later be changed; the pinned SDK contains no Through Bore motor type, and
    its unspecified motor type contains plausible legacy Tetrix values rather than an unknown
    marker. It therefore cannot identify an external encoder or prove direct velocity safe. Plant
    target bounds likewise describe legal commands rather than guaranteed measured speed.
    A position-assisted signed-16 branch correction might retain better hub-timed resolution, but
    it depends on undocumented wrap and sampling semantics and can choose an adjacent
    `65536`-counts/second branch during acceleration, long loops, stops, resets, or reversal. It is
    rejected as the leading/default design unless hardware evidence first shows that plain derived
    feedback is unusable and that the correction is branch-stable. Runtime fallback based on a
    suspicious raw value is unsafe because an overflowed value can look plausible. A student-facing
    `DIRECT`/`DERIVED`/`AUTO` choice, Through Bore-named core type, duplicate static and instance
    factories, second external-encoder selector, and another Plant feedback spelling are therefore
    rejected as misleading or redundant.
  - **Approved core semantics:** use one
    resettable, cycle-idempotent `ScalarSource` instance transform returning caller position units
    per second. Sample upstream once per new `clock.cycle()` and use differences between the
    transform's own accepted `clock.nowSec()` samples, never the preceding loop's `dtSec()`. A first
    finite sample establishes a baseline and returns a documented bootstrap `0.0`; reset propagates
    upstream and clears history. Same-cycle calls return the cached result. Zero elapsed time keeps
    the prior result without consuming the position delta, regressing time rebaselines, and a
    non-finite sample/result returns `NaN` without poisoning the last valid baseline. The core type
    assumes linear unwrapped position and must not guess counter width, periodic shortest paths,
    discontinuities, CPR, direction, or filtering.
  - **Prior algorithm approval and reopened API gate (2026-07-16):** the user approved the fixed
    internal-direct/external-derived acquisition policy, then identified that the public
    `VelocityFeedback` wrapper repeats the velocity domain and that `fromSource(...)` adds ceremony.
    That observation materially improves the public API and reopened the required major-design stop.
    The user approved the direct staged feedback API on 2026-07-16. The measurement-only comparison
    was then added to the existing generic motor-power tester and reviewed before the hardware gate
    was reconsidered.
  - **API-review workflow lesson:** `Framework Principles.md` now requires each conceptual builder
    question to be answered once, treats an immediately consumed selector/parameter wrapper as a
    public construction layer, and permits that type only when callers meaningfully store, reuse,
    compose, share, or independently validate it. The framework-improvement skill now performs that
    parameter-type audit explicitly and has a separate evidence gate for decisions that source or
    documentation inspection cannot prove. It keeps the item `Researching`, separates diagnostic
    tooling from production defaults, requires Android Studio safety review before a physical run,
    and reopens design approval if measured evidence contradicts an assumption.
  - **Measurement-only tooling (2026-07-16):** the existing generic `HW: DcMotor Power` tester now
    keeps its ordinary display concise and adds an opt-in Y capture. While capture is active, an
    acquisition guard reads the selected motor port at most once per shared loop cycle, including a
    cached failure. On a matched REV module, one explicit fresh bulk read feeds both public motor
    getters from a coherent snapshot; an original `OFF` cache mode is changed to `MANUAL` only while
    those getters consume the snapshot and is restored immediately, while `MANUAL`/`AUTO` are left
    unchanged. Module, controller, firmware, original cache mode, port 0/3 eligibility, coherence,
    and restoration truth are logged rather than inferred. A package-private tools-only helper uses
    elapsed accepted `clock.nowSec()` samples, same-cycle caching, signed 32-bit rollover-aware
    deltas, explicit warmup, zero-time retention, time-regression rebaseline, and reset at
    device/lifecycle boundaries; it is deliberately not the proposed production `ScalarSource`.
    The tester locks actuation during INIT, requires a fresh A arm after Driver Station START and
    every device selection, prevents a stale target from reaching a new motor, temporarily selects
    `RUN_WITHOUT_ENCODER` for open-loop testing, and commands zero before restoring the prior mode.
    Locale-stable rows distinguish the command held before measurement from the command actually
    issued afterward, report every unavailable cycle explicitly, and include one non-blocking
    right-bumper sample gap. No filter, signed-16 correction, sleep, FtcSensors/Plant API, or
    production feedback selection was added. The generic guide documents fixture/tachometer safety,
    the exact port and stack, coherent acquisition, raw-log preservation, and the distinction between
    configured motor metadata and physical encoder identity without naming a downstream robot
    project.
  - **Measurement-tool applicability:** the pinned Bettabot code obtains its external reading through
    `leftMotorName`, matching the one selected motor channel that the power tester both drives and
    observes. The hardware run must still confirm that the Through Bore is physically wired to that
    selected channel. If it is actually on a separate encoder-only channel, stop: a two-device
    actuator/measurement tester is a material UI and ownership change requiring another design gate.
  - **Measurement-stage automated verification (2026-07-16):** nine focused pure tests pass for
    bootstrap, normal/irregular intervals, duplicate cycles, zero/regressing/non-finite time, reset,
    forward/reverse signed position rollover, and unavailable direct velocity with valid derived
    output. The complete `:TeamCode:testDebugUnitTest` suite passes 433 tests with zero failures,
    errors, or skips, and `:TeamCode:compileDebugJavaWithJavac` succeeds against the pinned FTC SDK.
    `git diff --check`, untracked-file whitespace checks, and no-sleep/no-loop diagnostic checks pass.
    Independent adversarial reviews closed same-press selection/arming, stale-target transfer,
    INIT actuation, motor-mode restoration, fail-stop cleanup, coherent REV bulk acquisition,
    one-acquisition-per-cycle, log truth, and student-facing wording findings. The remaining physical
    hardware evidence is intentionally not claimed by these automated checks.
  - **Measurement-tool manual audit (2026-07-16):** the user reviewed the diagnostic controls,
    safety/lifecycle behavior, coherent-acquisition path, and recorded output in Android Studio and
    replied `SOURCE-02 measurement tool looks good`. This approves the physical evidence run only;
    it is not final SOURCE-02 approval and does not authorize production builder/core changes.
  - **No-hardware and Cuttlefish reassessment (approved 2026-07-16):** the user does not
    have access to the exact hardware and pointed to Cuttlefish's working external Through Bore
    implementation as an additional hint. Cuttlefish independently chose position-delta over actual
    elapsed time at 8192 counts/revolution, bootstraps the first sample at zero, keeps smoothing as
    explicit later stages, configures the encoder-bearing motor on hardware-counted port 3, and
    bounds final motor power. This strengthens the practicality and ownership case for hiding the
    derived measurement below robot behavior. It does not establish the encoder version, retained
    count accuracy, direct-versus-derived error, reference-instrument agreement, or the exact
    firmware/loop conditions, and its per-loop filters and lifecycle assumptions should not be
    copied into the generic estimator.
    - The recommended response is a formal Gate 1 reclassification, not an evidence waiver. Narrow
      SOURCE-02's production promise to a deterministic rate from the continuous positions the FTC
      SDK actually reports. Keep physical encoder identity, counts/revolution/unit mapping,
      direction, wiring, appropriate high-rate port, maximum usable rate, and regulator tuning as
      adoption facts. Track the exact-stack tachometer/count-retention run separately and do not
      claim that a finite derived result proves every physical edge was captured.
    - Under that narrower contract, indefinite framework deferral adds no safety: the signed-16
      direct velocity range is already proven structurally insufficient for this 8192-count encoder,
      while two robot implementations repeat position history and timing. The approved staged API,
      generic elapsed-time transform, FTC signed-32 continuity handling, exhaustive fake-clock/
      rollover tests, port 0/3 guidance, existing finite normalized command boundary, and explicit
      unvalidated-hardware wording form the conservative best-effort implementation. No automatic
      direct/derived fallback, signed-16 branch correction, hidden filter, or reliability claim is
      justified.
    - Cuttlefish also explicitly selects `RUN_WITHOUT_ENCODER` before its raw-power
      shooter loop. Phoenix's general FTC `motorPower(...)` adapter currently leaves motor run mode
      unchanged. That is a broader regulated-motor lifecycle issue, not permission to silently
      expand SOURCE-02; decide it through its own major gate before claiming that every regulated
      motor path owns raw-power mode. Feedback-read/non-finite-measurement cleanup is likewise a
      broader regulated-Plant safety question and must not be hidden inside this source transform.
    - **Approval and implementation status (2026-07-16):** the user replied
      `Approve reclassifying SOURCE-02 hardware evidence as adoption validation and proceeding with
      the conservative observed-position implementation.` This formally limits the framework claim
      to the SDK-observed position stream, preserves the exact-stack run below as separate adoption
      validation, and authorizes the already approved core/staged implementation. SOURCE-02 passed
      `Ready` and entered implementation. It does not authorize FTC motor run-mode changes,
      filtering, feedback-failure semantics, or a hardware-accuracy claim.
  - **Production implementation (2026-07-16):** `ScalarSource.ratePerSecond()` now owns one
    resettable, per-cycle-idempotent elapsed-time derivative with the approved bootstrap, skipped-
    cycle, zero/regressing-time, non-finite, reset, and diagnostic semantics. The FTC boundary adds
    package-private signed-32 modular position continuity for the staged external-encoder path.
    Regulated motor velocity, motor position, and CR-servo position builders now expose direct,
    parallel feedback answers; the redundant public `PositionFeedback` and `VelocityFeedback`
    selector families are removed. Internal motor velocity remains direct SDK velocity, external
    incremental-encoder velocity is observed position followed by the generic rate transform, and
    advanced `nativeFeedback(source)` remains an unchanged already-domain-specific seam. All
    in-repository callers, Javadocs, guides, and examples use the one staged API. No run mode,
    filtering, automatic fallback, feedback-validity policy, or hardware-accuracy inference was
    added.
  - **Bettabot simplification:** the framework path replaces Bettabot's robot-owned
    `ThroughBoreVelocitySource` state machine—SDK lookup, prior position/time history, reset, and
    sample arithmetic—with the ordinary builder answer `.externalEncoder(leftMotorName)`. Bettabot
    still owns the meaningful configuration: physical encoder name and direction, counts-per-
    revolution mapping, regulator/tuning, legal target range, target source, and mechanism policy.
    The next prioritized `FTC-01` gate separately decides raw-power run-mode ownership so this
    simplification is not presented as a complete flywheel lifecycle yet.
  - **Automated verification (2026-07-16):** focused suites cover the generic rate transform and FTC
    staged/rollover boundary. Android Studio's bundled JBR completed
    `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac`; the full XML result is 48 suites
    and 452 tests with zero failures, errors, or skips. The only compiler output is the repository's
    existing Java-8-on-JDK-21 source/target deprecation warning.
  - **Static and independent verification (2026-07-16):** `git diff --check`, an untracked-file
    trailing-whitespace scan, stale feedback-wrapper searches, project-neutrality searches, and
    no-blocking-loop checks pass. Builder-level fake-hardware tests prove that the public internal
    velocity answer reads only SDK velocity, the external velocity answer reads only position and
    derives elapsed-time rate, reset restores bootstrap behavior, and motor/CR position answers stay
    position-based. Three independent reviews covered numerical/timing/reset behavior, signed-32
    rollover, staged construction, API parallelism, error quality, documentation claims, scope, and
    Framework Principles. They found no remaining blocker after the builder-dispatch coverage was
    strengthened.
  - **Android Studio audit point:** inspect `ScalarSource.ratePerSecond()`, the three regulated
    feedback-step interfaces and their builder implementations in `FtcActuators`, the package-private
    continuous position source in `FtcSensors`, `FtcExternalEncoderFeedbackTest`, and the feedback
    section in `FTC Actuators & Plants.md`. Confirm that the ordinary velocity call reads
    `.regulated().externalEncoder(name).regulator(...)`, internal and external acquisition choices
    are deterministic, CR servos expose no impossible internal answer, reset/bootstrap and hardware-
    accuracy limits are clear, and no mode selector, filter, automatic fallback, or public feedback-
    wrapper type was added. Also inspect `FTC-01` below and confirm it—not SOURCE-02—owns the next
    Bettabot motor-mode decision. No robot hardware is required for this code/API audit; the exact-
    stack encoder run remains deferred adopting-robot validation.
  - **Manual verification (2026-07-16):** the user reviewed SOURCE-02 in Android Studio and replied
    `SOURCE-02 looks good`. This approves the generic estimator, direct staged feedback API,
    conservative SDK-observed-position contract, synchronized documentation/tests, and publication.
    It does not close or weaken the separately documented exact-stack adopting-robot validation, and
    it does not authorize starting `FTC-01` as part of this item.
  - **Deferred adopting-robot evidence point:** when Bettabot hardware is available, first confirm
    that the Through Bore is physically connected to the selected powered motor's encoder channel.
    If the actuator and encoder require different configured device entries, the existing tester
    needs a separate UI/ownership decision before that run. This validation is not a production-code
    prerequisite under the approved observed-position contract.
  - **Evidence still required before claiming Bettabot hardware accuracy or match readiness:** compare raw SDK velocity and elapsed-time
    position-derived velocity against an independent tachometer on Bettabot's actual Through Bore,
    Control/Expansion Hub and firmware, hardware-counted encoder port 0 or 3, SDK 11.1.0, and
    production bulk-caching mode. Record same-cycle time, position, and both rates at stop; both
    directions; below/around 240 RPM; the expected direct-velocity wrap boundaries; 1000 RPM; and the
    highest safely fixture-contained operating point up to the claimed 5000 RPM. Include spin-up/down,
    coast, reversal, typical/fast/slow loops, and one skipped/long loop. The run must verify that
    position capture does not lose counts at the supported maximum and that the derived interval-
    average feedback is accurate and responsive enough for closed-loop regulation. Reconsider a
    measurement-only signed-16 correction only if the plain derived path fails that performance gate;
    it is not a co-equal production candidate by default. Until this adoption gate closes, do not
    claim that Phoenix, the SDK, or the configured hub port guarantees physically correct Through
    Bore velocity at Bettabot's operating speeds.

### FTC-01 - FTC raw motor-power run-mode ownership

- **Confirmed problem:** `FtcHardware.motorPower(...)` currently configures direction and forwards
  `setPower(...)` without establishing the run mode that gives that command raw/open-loop meaning.
  Under the pinned FTC SDK 11.1.0, `RUN_WITHOUT_ENCODER` sends constant power,
  `RUN_USING_ENCODER` turns `setPower(...)` into a hub-managed velocity target,
  `RUN_TO_POSITION` uses the stored position target and treats power as unsigned maximum effort,
  and `STOP_AND_RESET_ENCODER` does not actuate from a power write. A same-OpMode owner transition
  can therefore put a Phoenix raw-power command beneath a second controller, send it toward a stale
  target, or make it ineffective.
- **Pinned-SDK correction to the original hypothesis:** normal REV/Lynx OpMode initialization does
  not carry the prior user OpMode's motor mode forward. Before user `init()`,
  `OpModeManagerImpl` resets Lynx controllers and `LynxDcMotorController.initializeHardware()`
  selects `RUN_WITHOUT_ENCODER`, float/zero. The confirmed problem is still real for same-OpMode
  transitions, calibration search, competing owners, non-Lynx/custom reset behavior, and an adapter
  whose own semantic contract is unspecified; the decision must not claim ordinary cross-OpMode
  inheritance on the pinned stack.
- **Why it is prioritized for Bettabot:** SOURCE-02 makes the external Through Bore feedback concise,
  while CTRL-01 and SAFE-03 centralize complete-regulator limits and normalized command safety.
  Those improvements do not make the flywheel controller authoritative if its motor-power boundary
  can still use an FTC PID mode. Bettabot should not need raw `DcMotorEx` access or a surrounding
  `setMode(...)` ritual to make the ordinary regulated builder correct.
- **Completion:** focused fake-motor tests cover every modern starting mode, construction versus
  first-write timing, exact zero/mode/verification/request ordering, already-correct and repeated
  writes, transition failures, single and grouped mode preflight, stop/reset, calibration search,
  direct power, regulated position/velocity, simple mecanum drive, and handoff to device-managed
  control. Javadocs and the FTC actuator guide state which boundary owns mode, the difference
  between `setPower(0.0)` and lifecycle `stop()`, and the no-restore/no-reset policy. The complete
  unit suite and FTC compile pass. Representative hardware observation remains adoption evidence;
  software verification must not claim independent firmware acknowledgement or physical motion.
- **Decision record (2026-07-16):**
  - **Minimal reproduced trace:** all compiled framework motor-power paths reach
    `FtcHardware.motorPower(...)`. Its current `setPower(...)` only clamps and delegates. FTC
    `DcMotorImpl.setPower(...)` consults the current mode; in `RUN_TO_POSITION` it makes the power
    non-negative, while the REV/Lynx controller sends a target-velocity command for both PID modes
    and a constant-power command only for `RUN_WITHOUT_ENCODER`. Device-managed position
    calibration is a concrete in-repository failure: `beginCalibrationSearch(...)` first stops its
    normal output, that stop selects `RUN_USING_ENCODER`, and the current search output then writes
    "open-loop" power without leaving the velocity PID mode. The SDK's mode transition also
    preserves and reissues the prior power, so a safe transition must submit zero before selecting
    raw mode; selecting mode first is not sufficient.
  - **Current callers and public construction layers:** the only compiled callers of
    `FtcHardware.motorPower(...)` are `FtcActuators` and `FtcDrives`. They cover motor
    `.power()`, framework-regulated motor position and velocity, motor position calibration search,
    the standard mecanum lane/examples/calibrator, Phoenix's intake and drivetrain tester, and the
    external-sensor regulated example. No in-repository robot directly calls the low-level factory,
    although `PowerOutput` Javadoc teaches that public use. `FtcHardware.motorPosition(...)` and
    `motorVelocity(...)` already assert their device-managed modes when commanding. Generic core
    `PowerOutput` remains a hardware-neutral command seam.
  - **Student-facing simplicity and parallelism:** the Beginner's Guide already defines motor
    `.power()` as open-loop normalized power and `.regulated()` as Phoenix driving raw power.
    Consequently the run mode is not another conceptual student choice: it follows from the target
    domain and loop owner already selected; selecting external rather than internal feedback does
    not change the powered motor's required raw-power mode. The public call sites remain unchanged. Making
    FTC-specific `motorPower(...)`, `motorVelocity(...)`, and `motorPosition(...)` each establish
    the mode required by their names is more parallel and discoverable than adding a regulated-only
    exception. Bettabot's normal builder therefore remains the complete construction; FTC-01 removes
    no additional robot class, but prevents a hidden hardware prerequisite and avoids future
    `DcMotorEx.setMode(...)` setup code.
  - **Chosen ownership boundary and command lifecycle:** `FtcHardware.motorPower(...)` will mean an
    FTC raw/open-loop power output and will own `RUN_WITHOUT_ENCODER` whenever
    `setPower(...)` asserts a command. Construction continues to resolve the motor and set direction
    only; it does not steal mode from an unused output. Before each command, the adapter checks the
    current SDK-reported mode. If it is already `RUN_WITHOUT_ENCODER`, it submits the requested power
    directly. Otherwise it commands zero in the current mode, selects and verifies
    `RUN_WITHOUT_ENCODER`, and only then submits the requested power. This is a conditional mode
    assertion, not a blind `setMode(...)` on every loop; on Lynx, the mode read uses the same
    freshness cache that `DcMotorImpl.setPower(...)` already consults. A transition or verification
    failure best-effort commands zero, preserves the primary failure, suppresses cleanup failures,
    and throws an actionable motor/current-mode/required-mode error. It never resets encoder
    position.
  - **Stop, reset, calibration, and handoff:** the motor-power adapter overrides `stop()` to command
    zero directly without acquiring or restoring a mode. A normal raw owner therefore usually
    leaves `RUN_WITHOUT_ENCODER` selected at zero; if another owner already selected a different
    mode, stop does not steal it back. Plant `reset()` remains state-only and sends no hardware or
    mode command. Calibration search asserts raw mode on its search command; the next
    device-managed position command reasserts `RUN_TO_POSITION`. Device-managed velocity likewise
    retains `RUN_USING_ENCODER`. A deliberate shared-actuator handoff must stop and disable the old
    updater before enabling the new one. Every command asserts its own mode, but FTC-01 does not
    make simultaneous writers valid.
  - **External-encoder channel boundary:** run mode follows the actuation path, not the feedback
    source. A powered motor controlled by a Phoenix regulator therefore uses
    `RUN_WITHOUT_ENCODER` whether feedback comes from its internal encoder, an external encoder on
    the same configured motor channel, or a separate encoder-only channel. A separate external
    channel is measurement-only: `FtcSensors` reads position and must not set its power, select its
    mode, or reset it as a side effect of `externalEncoder(...)`. Normal pinned Lynx OpMode init
    leaves that channel in a non-resetting `RUN_WITHOUT_ENCODER` mode. If adopting robot code
    deliberately enters `STOP_AND_RESET_ENCODER`, that owner must explicitly leave reset mode before
    expecting measurements; Phoenix must not hide that lifecycle mistake by mutating a sensor
    channel. FTC-01 changes only the powered `motorPower(...)` output.
  - **Grouped-mode safety and item boundaries:** framework-created motor groups will resolve their
    members before command side effects and use the smallest package-private preflight coordination
    needed to zero/select/verify every child before any requested nonzero child write. A
    mode-acquisition failure therefore cannot make an earlier child begin the requested group
    motion. Software fan-out is still not atomic. SAFE-04 retains ordinary child command/stop
    failure cleanup and cached seam-truth semantics after mode preflight; FTC-01 must not absorb
    that separate public contract. DRIVE-02 retains PTO/shared-drivetrain arbitration and suppression
    of the old updater. CR servos, generic custom `PowerOutput`s, lower-level pure Plants, encoder
    identity, zero-power brake/coast policy, and device-managed tuning are unchanged.
  - **Alternatives rejected:** documentation-only or robot-owned `setMode(...)` leaves a common FTC
    correctness rule in every robot; a new public run-mode question, enum, wrapper, or factory adds
    a redundant way to restate the already selected power/control domain; a regulated-only wrapper
    leaves direct `.power()`, mecanum, and calibration search contradicting their documented
    open-loop meaning; construction-only selection can be invalidated by a later legitimate
    device-managed phase and causes unused outputs to claim hardware; blindly setting mode on every
    loop causes needless mode transitions; rejecting every incompatible starting mode makes normal
    handoff harder without improving ownership; restoring a prior PID mode can revive stale
    controller state; and a global actuator registry/resource scheduler is disproportionate here
    and belongs, if justified, to DRIVE-02.
  - **Verification plan:** add a dedicated ordered-event `DcMotorEx` probe rather than extending the
    SOURCE-02 feedback test. Verify no mode/power write at construction; all four current starting
    modes; zero before every required transition; no requested command before verified raw mode; no
    encoder reset; already-raw and repeated-write behavior; stop without mode acquisition/restoration;
    transition/readback/cleanup failures and exception suppression; mode changes between writes;
    all-child group preflight before requested nonzero fan-out; regulated fail-stop compatibility;
    direct and regulated `FtcActuators` paths; device-managed calibration and return; external
    feedback on the powered channel and on a distinct encoder-only channel; no mode, power, or reset
    write to the distinct measurement channel; and `FtcDrives` stop/handoff semantics. Update
    `FtcHardware`/`PowerOutput` Javadocs, Framework
    Overview, Beginner's Guide, and FTC Actuators & Plants together. Run focused tests,
    `:TeamCode:testDebugUnitTest`, `:TeamCode:compileDebugJavaWithJavac`, `git diff --check`, and
    no-sleep/no-busy-loop checks.
  - **Approval gate:** approved by the user on 2026-07-16. The approved design deliberately broadens
    the original regulated-only leading hypothesis to every standard FTC motor-power path while
    keeping the student-facing builders unchanged.
  - **Implementation record (2026-07-16):**
    - `FtcHardware.motorPower(...)` now returns a named FTC raw-power adapter. Construction still
      resolves the motor and configures direction without reading/writing mode or power. Each
      explicit power command checks the current SDK-reported mode; an incompatible mode uses
      zero, `RUN_WITHOUT_ENCODER`, verification, then the requested clamped power. Acquisition
      failures request a best-effort zero and retain primary and suppressed cleanup failures in an
      actionable exception. Lifecycle `stop()` writes zero without mode acquisition/restoration,
      and no raw-power path resets encoder position.
    - A package-private FTC motor-group coordinator resolves every named child before direction
      configuration and preflights every child's raw mode before requested power fan-out. A
      preflight failure best-effort zeros the complete group without submitting requested motion.
      Fixed-order batch state is invalidated by interruption, unexpected ordering, and lifecycle
      stop. Public `FtcActuators` and `FtcDrives` APIs remain unchanged; generic `PowerOutput`,
      CR-servo groups, and custom output construction do not gain an FTC mode concept.
    - `FtcActuators` uses the coordinated output group for direct power, regulated
      position/velocity, and device-managed position calibration-search paths.
      `FtcDrives` uses the same coordination for standard mecanum construction.
      `MecanumDrivebase.stop()` now delegates to each output's lifecycle `stop()` hook, allowing a
      deliberate later owner's mode to remain selected while physical zero is requested.
    - `PowerOutput` Javadocs and Framework Overview, Beginner's Guide, and FTC Actuators & Plants
      document hardware-neutral versus FTC-specific ownership, active zero versus lifecycle stop,
      calibration handoff, internal/external feedback independence, separate encoder-channel
      read-only behavior, and the advanced manual-group responsibility. No student-facing builder
      call or Bettabot robot class changed. Per user direction, FTC-01 contains no compatibility
      path for deprecated SDK run-mode aliases and does not call the deprecated
      `DcMotor.RunMode.migrate()` API; its contract and tests use the four current SDK modes.
  - **Automated verification (2026-07-16):**
    - Dedicated `FtcMotorPowerRunModeTest`: 21 tests, 0 failures, 0 errors, 0 skipped. Coverage
      includes all four current SDK starting modes, construction timing, explicit zero,
      stop/reset, mode drift, exact transition ordering, verification and cleanup failures,
      exception suppression, all-child group preflight/failure cleanup, complete-group resolution,
      direct and regulated position/velocity paths, same-channel and distinct external feedback,
      raw/device-managed velocity handoff, calibration search/return, regulated fail-stop, and
      standard mecanum drive/stop.
    - `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` passed under the Android
      Studio JBR. Generated XML reports 473 tests, 0 failures, 0 errors, and 0 skipped. Output
      retains the project-wide Java 8 source/target warning under JDK 21 and the existing FTC Robot
      Controller app-shell note; neither comes from modern `edu.ftcphoenix` code or an FTC-01
      compatibility path.
    - `git diff --check`, an explicit trailing-whitespace scan including the untracked focused test,
      and targeted no-sleep/no-busy-loop scans all passed. Independent API/Framework-Principles and
      SDK/lifecycle adversarial reviews report no remaining actionable finding.
  - **Manual verification (2026-07-16):** the user reviewed FTC-01 in Android Studio and confirmed
    that it looks good. Software tests cannot prove independent hub-firmware acknowledgement,
    absence of a physical transition pulse under every electrical condition, or correct mechanism
    motion. Representative robot observation remains adoption validation when hardware becomes
    available, not a blocker for this conservative boundary contract.

### SOURCE-03 - Composable scalar measurement conditioning

- **Problem to confirm:** finite differencing can amplify encoder quantization and loop jitter, while
  analog, distance, current, and other scalar measurements may contain spikes or noise. Phoenix has
  generic mapping, clamping, hold-last, rate-limiting, and boolean stability helpers, but no clearly
  documented generic numeric smoothing or outlier-rejection path. Putting a filter inside a Through
  Bore adapter or derived-velocity helper would couple reusable signal conditioning to one device
  and prevent deliberate filtering before versus after estimation.
- **External evidence:** Bettabot currently uses one-cycle position differencing without filtering in
  [`ThroughBoreVelocitySource`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L30-L75).
  Cuttlefish independently derives flywheel rate from position and then applies multiple explicit
  low-pass accumulator stages in its
  [`Shooter`](https://github.com/6165-MSET-Cuttlefish/summer-2026/blob/42d1ce8ffe3dd62187b93771f1a5455c85fff6cd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Shooter.java).
  That supplies a second real caller, but its per-loop alpha values are not a documented
  elapsed-time/time-constant contract and do not prove which filter Phoenix should use. Collect real
  flywheel traces before selecting an algorithm.
- **Composition gap to audit:** the simple regulated-Plant
  `.externalEncoder(name, direction)` path deliberately hides rollover-aware position acquisition,
  and rate derivation. The surrounding regulated actuator path owns the powered motor's
  `RUN_WITHOUT_ENCODER` mode; a separate encoder-only port remains read-only. Advanced
  `nativeFeedback(ScalarSource)` can accept a composed signal, but the exact rollover-aware
  continuous-position source used by the simple path is currently package-private. The decision
  gate must determine whether real callers need a distinct advanced composable acquisition seam,
  while keeping the ordinary external-encoder call unchanged. Do not add implicit filtering or a
  second ordinary feedback selector merely to expose internals.
- **Alternatives to compare:** document robot-owned custom filters; use FTC SDK filter utilities at
  the boundary; add an exponential/time-constant low-pass decorator; add fixed-sample or time-window
  mean/median filters; reject implausible deltas; estimate rate with windowed regression; or provide
  a broad configurable filter interface. Compare noise rejection, phase delay, irregular-loop
  behavior, allocation, tuning concepts, and whether each candidate offers distinct value beside
  existing Source composition.
- **Leading hypothesis:** if measurements confirm a repeated need, add only the smallest named
  filter decorator or decorators to the generic scalar Source layer. Each has explicit time/window
  semantics, is cycle-idempotent and resettable, and can be composed either before SOURCE-02's rate
  estimate or after it. Do not add a generic "filtered encoder velocity," implicit default smoothing,
  one opaque filter with many modes, or parallel factory/instance spellings without distinct value.
- **Completion:** deterministic tests cover startup, reset, duplicate cycles, irregular `dt`, step and
  ramp response, representative noise/outliers, non-finite samples, latency, and debug state.
  Documentation explains the semantic difference between filtering position before differentiation
  and filtering derived velocity afterward, gives selection guidance backed by recorded data, and
  keeps sensor meaning and response policy in robot code.
- **Pause record (2026-07-19):** **Deferred while recorded representative signal traces are
  unavailable.** Synthetic data can test a chosen filter but cannot select the public algorithm,
  time/window semantics, or acceptable latency. Resume with raw timestamped traces or an approved
  narrower no-API/documentation result; do not hide an unvalidated default in the encoder boundary.

### SAFE-03 - Regulated Plant actuator-command truth

- **Problem to confirm:** `PowerOutput` is a normalized `[-1,+1]` command seam, but regulated
  position/velocity Plants currently store and forward the raw regulator result while FTC adapters
  may clamp a different value. Group scale/bias wrappers also report the pre-transform request while
  child adapters may saturate. Debug output can therefore disagree with the command that reached
  hardware, and non-finite or excessive commands travel farther than necessary before defensive
  handling. SAFE-02 intentionally fixed only direct power Plants and deferred regulated outputs.
- **External evidence:** Bettabot's
  [`CappedFlywheelRegulator`](https://github.com/Hansika1098/Summer26/blob/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790/TeamCode/src/main/java/edu/ftcphoenix/robots/betta/BettaShooter.java#L77-L136)
  prevents this mismatch for one flywheel while also applying its narrower `[0, maximum]` policy.
  CTRL-01 owns that optional policy range; SAFE-03 asks what every normalized regulated Plant must
  guarantee even when no optional decorator is used.
- **Alternatives to compare:** document that every regulator must self-limit; install CTRL-01
  implicitly; clamp/sanitize in each regulated Plant; centralize one final regulated-output safety
  helper; change `PowerOutput.setPower(...)` to return what was applied; read
  `getCommandedPower()` after every write; or add a constrained command/status type. Trace legacy and
  mapped position/velocity Plants, search/homing power, grouped and scaled outputs, CR servos, direct
  pure-framework callers, debug/status consumers, and stop/failure behavior before selecting scope.
- **Leading hypothesis:** because `PowerOutput` semantically consumes normalized power, apply one
  shared finite/`[-1,+1]` invariant immediately before each regulated Plant's final write and retain
  both requested regulator output and the normalized command submitted by a normally returning
  top-level output call in diagnostics. Keep FTC adapter saturation as defense in depth. Do not require
  students to remember CTRL-01 for universal safety, change mechanism targets or
  `Plant.getAppliedTarget()`, or conflate child scale/bias configuration validity with the central
  regulator command; API-03 separately audits those mappings.
- **Completion:** focused fake-output tests cover mapped and lower-level regulated position/velocity
  paths, exact boundaries, positive/negative saturation, NaN/infinity, grouped/scaled outputs,
  search/homing commands where in scope, stop/reset/failure, and agreement between diagnostics and
  the command submitted by a normally returning top-level output call. Javadocs clearly distinguish
  mechanism target, raw regulator result, submitted normalized command, per-child mapped command,
  and physical feedback;
  existing finite in-range behavior and CTRL-01's narrower explicit policy remain unchanged.
- **Decision record (2026-07-15):**
  - **Confirmed behavior and minimal trace:** all regulated paths currently call
    `regulator.update(...)`, cache that raw result, and pass it unchanged to
    `PowerOutput.setPower(...)`. The lower-level `Plants.positionFromPower(...)` and
    `velocityFromPower(...)` factories share `Plants.AbstractRegulatedPlant`; mapped position and
    velocity each duplicate the same sequence. Their stable raw-result debug keys are respectively
    `.output`, `.lastRegulatorOutput`, and `.regulatorOutput`. A finite `1.2` therefore reaches a
    custom output as `1.2`; a standard FTC motor or CR-servo adapter later clips it to `1.0`, so the
    Plant and boundary diagnostics disagree. FTC's current clamp maps positive/negative infinity to
    `+1.0`/`-1.0` and leaves `NaN` as `NaN`. If a regulator throws instead of returning—specifically,
    CTRL-01 rejects a non-finite inner result—the Plant leaves the previously commanded hardware
    output active unless an outer owner happens to stop it.
  - **Current callers and construction paths:** the normal student path is the staged
    `FtcActuators` builder. Regulated motor velocity uses `MappedVelocityPlant`; regulated motor
    position and regulated CR-servo position use `MappedPositionPlant`. The builder requires
    default scale/bias for regulated groups, so its supported group path fans out one identical
    logical command. The only compiled modern repository caller is
    `TeleOp_08_LiftExternalSensorControl`; current Phoenix production code has no regulated Plant.
    The public mapped builders and `Plants.positionFromPower(...)`/`velocityFromPower(...)` remain
    distinct lower-level paths for custom HAL outputs, and no in-repository production caller uses
    those factories directly.
  - **Public construction-layer audit:** retain the staged
    `FtcActuators.plant(...).motor/crServo(...).velocity/position().regulated()...` path as the one
    beginner layer because it owns FTC device lookup, direction, grouping, feedback selection, and
    guided validation. Retain `MappedPositionPlant.regulated(PowerOutput, ScalarSource,
    ScalarRegulator)` and `MappedVelocityPlant.regulated(...)` as the advanced non-FTC/custom-HAL
    boundary because their builders uniquely own plant/native unit mapping, declared target ranges,
    position topology/reference, optional calibration output, and typed mapped-Plant results.
    `Plants.positionFromPower(...)` and `velocityFromPower(...)` are older minimal native-unit
    ingredient factories returning generic `Plant`; their behavior overlaps an identity-configured
    mapped builder, they have no in-repository caller, and they are not a second recommended robot
    path. SAFE-03 must still protect them while CLEAN-01 performs the separate compatibility/external-
    caller gate for eventual removal rather than bundling an unrelated breaking deletion into a
    safety change. Retain `ScalarRegulator` as the custom control-law seam, `ScalarRegulators` as the
    only built-in regulator factory, and `PowerOutput` as the normalized custom-output seam. Add no
    overload, builder question, public implementation, result type, getter, or status interface.
  - **Alternatives considered:** no change or documentation-only self-limiting; implicitly wrap every
    regulator with CTRL-01's `outputLimited(..., -1.0, +1.0)`; add a clamp and failure handling in
    each of the three Plant implementations; centralize only a stateless clamp; add one private
    stateful regulated-power channel; fix only `FtcHardware`; change `PowerOutput.setPower(...)` to
    return an applied value; read `getCommandedPower()` after the write; expose a constrained power
    value, Plant getter, or command-status type; reject all finite saturation; silently convert
    numerical failures to zero; or retain the previous command.
  - **Simplicity comparison:** documentation and self-limiting make every student remember a safety
    wrapper. An implicit CTRL-01 decorator conflates optional robot policy with universal output
    safety and still cannot stop hardware when its inner regulator throws. Three local clamps or a
    stateless helper duplicate raw/normalized diagnostics and fail-stop/reset behavior. A return
    value, readback, constrained type, getter, or builder step adds public concepts but still cannot
    prove grouped-child or physical actuator response. One package-private channel changes no robot
    call, keeps autocomplete unchanged, and removes the repeated safety/lifecycle state machine from
    all three implementations.
  - **Chosen internal design:** add one package-private actuation helper,
    `RegulatedPowerChannel`, that owns one stable `ScalarRegulator` -> normalized command ->
    `PowerOutput` graph. `Plants.AbstractRegulatedPlant`, `MappedPositionPlant`, and
    `MappedVelocityPlant` delegate regulator evaluation, final normalization, write bookkeeping,
    reset, stop, failure cleanup, and common debug state to it. It does not memoize by loop cycle:
    repeated calls may intentionally carry different setpoints or measurements. Plant target
    resolution, unit mapping, target guards, `getAppliedTarget()`, `PlantTargetStatus`, tolerance,
    and update order remain unchanged; only the internal evidence that the last regulated actuation
    completed normally becomes an explicit prerequisite for completion reporting.
  - **Finite update contract:** invoke the regulator exactly once. Preserve finite values already in
    the inclusive `[-1.0, +1.0]` range, including exact boundaries and signed zero. Saturate a finite
    excursion once at this final seam, forward only the normalized value, and distinguish submitted
    from saturated-and-submitted results in diagnostics. Normal return from the `void` output call is
    not a hardware acknowledgement. Saturation is normal actuator-domain behavior, so it does not
    throw or reset the regulator and does not claim generic anti-windup. CTRL-01 remains the explicit
    outermost policy when a robot wants a narrower or asymmetric range.
  - **Numerical and write-failure contract:** never forward `NaN` or either infinity. Record a
    returned non-finite raw value, best-effort stop the output first, reset the regulator, and throw
    an actionable `IllegalStateException` identifying the setpoint, measurement, result, and Plant
    implementation/control path. That synthetic exception remains primary and any stop/reset failure
    is suppressed on it. If regulator evaluation or `PowerOutput.setPower(...)` throws a
    `RuntimeException`, perform the same best-effort stop/reset cleanup and rethrow that original
    exception as primary with cleanup failures suppressed. Catch no `Error`. Silently zero-and-
    continue would hide broken control math; retaining the previous command would leave a known
    powered failure state.
  - **Reset, stop, and Plant-status contract:** maintain one internal completion-valid condition (or
    equivalent protected hook) used by both `atTarget()` and `atTarget(value)`. Clear it before every
    regulated actuation attempt, reset, and explicit stop; set it only after the complete regulator/
    output update returns normally. This prevents preserved matching target/status/measurement facts
    from making either completion overload return true after a failed stop. `reset()` resets
    controller and raw-computation state but performs no hardware write: retain the last normally
    submitted normalized command and report `RESET_WITHOUT_WRITE`, or
    `RESET_FAILED_WITHOUT_WRITE` before rethrowing a reset failure. `stop()` attempts every distinct
    owned output stop before regulator reset and always attempts all cleanup, preserving the first
    exception and suppressing later ones.
    If all required output stop calls return normally, command `0.0` is truthfully submitted and each
    Plant applies its existing successful-stop target/status semantics even if regulator reset later
    fails; the combined status records that reset failure. If any required output stop throws, mark
    command truth unknown, keep `atTarget()` false, reset local guards/search ownership best-effort,
    but preserve the prior public `getAppliedTarget()`, target plan, and `PlantTargetStatus` rather
    than falsely reporting `STOPPED`; then propagate the failure. The same conditional Plant-status
    rule applies when a regulator/write failure triggers fail-stop cleanup: only a normally returning
    stop may apply successful-stop semantics. `MappedPositionPlant` must identity-deduplicate its
    regulated and search output when shared, while still attempting both when they are distinct.
  - **Diagnostics and truth boundary:** add consistent private debug state at each Plant prefix for
    raw `.regulatorOutput`, seam-level `.normalizedPowerCommand`, and a
    `.regulatedPowerStatus` that distinguishes not-yet-updated, submitted, saturated-and-submitted,
    reset-without-write/reset-failed, stop-submitted/stop-failed, and regulator/non-finite/output
    failure outcomes—including whether fail-stop and regulator reset separately succeeded. Preserve
    the lower-level `.output` and
    mapped-position `.lastRegulatorOutput` raw aliases because Framework Principles require stable
    debug keys; mapped velocity's existing `.regulatorOutput` is already canonical. Continue nesting
    the regulator at `.regulator`. A normalized command is only the value submitted by a top-level
    `PowerOutput` operation that returned normally; a `void` normal return is not acceptance or
    readback. It is not a per-child transformed command, SDK readback, measured actuator power,
    voltage, velocity, or proof that hardware physically moved. Command value and later controller-
    reset outcome remain separate facts: a submitted zero stays truthful if reset then fails. No
    operational getter or public status type is added.
  - **Grouped, scaled, and FTC boundary:** the staged regulated group path accepts only default child
    mappings, so its normal successful fan-out receives the central normalized command unchanged.
    Advanced lower-level callers may supply a `PowerOutput` that scales, biases, clips, or otherwise
    interprets that command; SAFE-03 deliberately reports only the configured top-level seam. It
    does not read `getCommandedPower()` after a side effect or claim atomic group writes. Standard FTC
    finite saturation remains defense in depth. Raw-HAL non-finite handling, cache-after-success,
    and best-effort cleanup after a partial composite write are broader `PowerOutput` concerns now
    tracked separately as SAFE-04.
  - **Calibration-search scope:** `MappedPositionPlant`'s search power is open-loop configuration,
    not a regulator result, and the same search path is also installed for device-managed motor
    position Plants. Routing it through `RegulatedPowerChannel` would therefore mix two different
    responsibilities and broaden this item silently. SAFE-03 leaves valid search behavior unchanged.
    API-03 now explicitly tracks finite/range validation at `PositionCalibrationTasks.withPower(...)`
    and the direct `beginCalibrationSearch(...)` boundary; SAFE-04 owns output-failure cleanup rather
    than pretending one regulated-command value solves it.
  - **CTRL-01 and Summer26/Bettabot effect:** SAFE-03 removes no additional Bettabot class or line
    after CTRL-01; the projected robot surface remains about 1,012 Java lines. Bettabot still needs
    `outputLimited(..., 0.0, maximumOutputPower)` for its intentional no-reverse/narrower policy and
    its small robot-owned zero-setpoint coast/reset wrapper. PID/PIDF tuning, feedback units, Plant
    wiring, readiness, maximum-power configuration, and enable/reset policy also remain robot facts.
    The improvement is safety and mental-burden reduction: simpler robots need no redundant generic
    `[-1,+1]` regulator wrapper merely to protect normalized hardware, diagnostics no longer hide FTC
    clipping, and a CTRL-01/non-finite regulator failure best-effort stops the previous flywheel
    command before propagating. Existing PID output limits should remain when they intentionally
    shape that controller contribution; SAFE-03 is not evidence to remove tuning policy.
  - **Rejected and deferred designs:** do not add another public API, builder branch, constrained
    number, Plant command getter/status, or implicit regulator decorator. Do not replace the stable
    mechanism-target meaning of `getAppliedTarget()`/`PlantTargetStatus`, and do not rename existing
    debug keys. Do not move per-child scale/bias validation from API-03, calibration configuration
    into regulated control, raw output-adapter/composite failure semantics from SAFE-04, or robot
    enable/coast/hold policy into the framework. Do not claim physical output truth or saturation-
    aware anti-windup.
  - **Verification plan:** add pure fake-output coverage for all four public control paths: lower-level
    regulated position/velocity and mapped regulated position/velocity. Cover interior values,
    signed zero, exact `+/-1` boundaries, one-ULP and larger positive/negative saturation, all three
    non-finite values, CTRL-01 narrower pass-through and thrown failure, repeated calls, reset without
    a hardware write, reset failure after a prior command, successful/throwing stop, successful stop
    followed by reset failure, regulator/write/cleanup exceptions, suppressed-error ordering,
    prior-`atTarget() == true` and `atTarget(value) == true` invalidation, conditional public target/
    status behavior after successful versus failed fail-stop, shared-versus-distinct search-output
    identities, and raw/normalized/status debug truth.
    Audit default regulated group construction and custom scaled-output semantics without claiming
    child truth. Preserve existing target/range/guard tests and the compiling lift caller. Update
    `PowerOutput`/`ScalarRegulator` and regulated factory/class Javadocs, Framework Principles, and
    `FTC Actuators & Plants`; framework documentation remains project-neutral. Run the focused tests,
    full `:TeamCode:testDebugUnitTest`, `:TeamCode:compileDebugJavaWithJavac`, caller/debug-key/doc-link
    searches, XML-result inspection, and `git diff --check`, then request Android Studio review.
    Real motor/CR-servo saturation and fail-stop behavior remain required observations during normal
    adopting-robot bring-up; no hardware result will be claimed from unit tests.
  - **Approval gate:** the leading hypothesis remains the smallest student-facing design and no
    public call changes, but finite out-of-range values sent to custom outputs change and non-finite,
    regulator, output-write, and stop failures gain a new fail-stop/reset/rethrow lifecycle. Treat
    that as a major safety-semantic decision. `Approve SAFE-03 fail-stop design` authorizes only this
    recorded scope; implementation must not begin before explicit approval.
  - **Approval (2026-07-16):** the user approved the recorded design with
    `Approve SAFE-03 fail-stop design`. Implementation is limited to the private shared regulated-
    power channel, conditional Plant completion/stop truth, stable diagnostics, focused tests, and
    synchronized project-neutral documentation above. SAFE-04, API-03 calibration validation,
    publication, and every later tracker item remain out of scope.
  - **Implementation (2026-07-16):** added the package-private `RegulatedPowerChannel` and routed
    all four regulated Plant paths through it: lower-level position/velocity-from-power and mapped
    position/velocity. The shared channel evaluates the regulator once, preserves finite in-range
    values, saturates finite excursions at the final normalized-power seam, rejects all non-finite
    results, and centralizes raw/normalized diagnostics plus best-effort stop-then-reset cleanup.
    No public type, builder question, overload, getter, or robot-code call changed.
  - **Lifecycle and truth implementation (2026-07-16):** regulated completion is invalidated before
    every actuation, reset, and stop and becomes valid only after a complete update returns normally.
    Runtime regulator/output failures retain their primary exception and suppress cleanup failures
    in attempt order. Reset retains the last normally submitted normalized command without writing.
    Stop marks zero only when every relevant top-level output stop returns normally; otherwise the
    owning Plant preserves its prior applied target/plan/status while keeping both completion forms
    false. Mapped position stops a shared regulated/search output once and stops distinct outputs
    before regulator reset. Calibration-search command validation and raw/composite output behavior
    remain deferred to API-03 and SAFE-04.
  - **Documentation and diagnostics (2026-07-16):** synchronized Framework Principles, Plant,
    `ScalarRegulator`, `PowerOutput`, mapped/lower regulated Javadocs, Sources and Signals, and FTC
    Actuators & Plants. Stable raw aliases remain; the canonical private debug fields are
    `.regulatorOutput`, `.normalizedPowerCommand`, and `.regulatedPowerStatus`, with `.regulator`
    still nested. Documentation explicitly limits command truth to operations performed by the
    regulated boundary and does not claim calibration-search, per-child, SDK, or physical truth.
  - **Automated verification (2026-07-16):** focused SAFE-03 tests passed with 23 tests, zero
    failures/errors/skips (`RegulatedPowerChannelTest`: 15; `RegulatedPlantSafetyTest`: 8). The final
    full `:TeamCode:testDebugUnitTest` run passed with 424 tests across 46 suites and zero
    failures/errors/skips; XML results were inspected. `:TeamCode:compileDebugJavaWithJavac` passed.
    `git diff --check`, a trailing-whitespace scan including new files, regulated-path delegation,
    public-surface, project-reference, and command-readback searches passed. Gradle emitted only the
    existing Java 21/source-8 deprecation warnings.
  - **Adversarial review (2026-07-16):** the failure-semantics review found no remaining production
    correctness issue after the distinct-output stop-order refinement. Its inverse companion-stop
    failure test gap and calibration-search diagnostic wording gap were both fixed and reverified.
    The independent simplicity review found no student-facing API expansion, parallel construction
    path, project-specific framework reference, or unapproved scope expansion.
  - **Gate 3 / verification status (2026-07-16):** implementation and automated verification are
    complete; SAFE-03 is **Verifying** pending the user's Android Studio review. No robot-hardware
    result is claimed. Do not stage, commit, publish, merge, mark Done, or start SAFE-04 until the
    user explicitly approves this audit point.
  - **User verification (2026-07-16):** the user completed the Android Studio review and replied
    `SAFE-03 looks good`. SAFE-03 is **Done**. Robot-hardware saturation and fail-stop observations
    remain part of normal adopting-robot bring-up rather than a claim from this unit-tested change.

### SAFE-04 - PowerOutput failure cleanup and seam truth

- **Problem to confirm:** the standard FTC motor and CR-servo adapters cache a command before the SDK
  write succeeds and their finite clamp still forwards `NaN`. The internal scaled/grouped outputs
  likewise cache before child writes, and a child exception aborts the remaining write or stop loop.
  `getCommandedPower()` can therefore imply success after a failed write, while a composite may leave
  only some children stopped. SAFE-03 can best-effort call the configured top-level output but cannot
  make that operation atomic or prove child/physical state.
- **Alternatives to compare:** retain expert-only raw behavior; validate every caller; strengthen only
  FTC adapters; cache only after success; make composite writes/stops best-effort across every child;
  expose per-child results; change `PowerOutput` to return a result; or add a transactional output
  abstraction. Reproduce first/middle/last-child failures and distinguish the logical seam command,
  child commands, SDK acceptance, and physical response before changing the contract.
- **Leading hypothesis:** keep the narrow `PowerOutput` interface and its seam-relative cached
  command. Require finite normalized input; for a non-finite raw hardware request, best-effort submit
  zero and then throw. Retain finite saturation at concrete hardware adapters and cache a command only
  after its top-level operation returns normally. If a composite child write throws, stop issuing the
  requested nonzero command immediately, best-effort stop every child (including children not yet
  written), preserve the original write failure, and suppress stop failures. If a child stop throws,
  continue stop attempts for all remaining children, preserve the first stop failure, and suppress
  later ones. Set `getCommandedPower()` to `NaN` after any partial/failed write or stop and document
  that as unknown seam state; this public semantic change requires SAFE-04's own approval gate.
  Document that software fan-out is not atomic. Add no physical-readback claim, transaction API, or
  public per-child status unless a real operational caller proves it is needed.
- **Boundary with other items:** API-03 owns construction-time scale/bias and calibration-power
  validity; PERF-03 owns optional successful-write deduplication. SAFE-04 owns only defensive
  low-level write/stop failure semantics and truthful cache state.
- **Completion:** focused throwing-output/SDK-adapter tests cover non-finite raw commands, exact and
  excessive finite values, success-only caching, failures before and after a child side effect,
  first/middle/last-child write and stop failures, immediate nonzero-write abandonment, all-child
  best-effort stop cleanup, suppressed exceptions, `NaN` unknown-cache state, repeated stop, and
  truthful top-level versus child diagnostics. Javadocs state exactly what
  `getCommandedPower()` can and cannot prove; physical behavior is checked on representative motor,
  CR-servo, and grouped hardware before claiming completion.
- **Pause record (2026-07-19):** **Deferred under the current representative-hardware completion
  gate.** Fault-injected fakes can prove the software seam, but the present item explicitly requires
  physical actuator observation. A later Gate 1 may propose and seek approval for a narrower
  fail-stop software contract that disclaims atomic and physical truth; until then, do not claim
  completion from fake outputs alone.

### ACT-01 - FTC actuator-group identity validation

- **Problem to confirm:** `FtcActuators` rejects null group names and directions, but blank names and
  duplicate configured names within one motor/standard-servo/CR-servo group can survive until SDK
  lookup. A duplicate group member can configure and command the same SDK object twice. Bettabot
  consequently repeats blank/distinct shooter-motor name checks that the complete group
  specification already knows.
- **Priority evidence (2026-07-17):** unlike the remaining broad configuration audits, this change
  identifies concrete downstream branches that an adopting Bettabot can delete after migration:
  its left/right blank and duplicate-name checks move to the existing fully informed grouped-
  actuator boundary. The student-facing valid staged call remains unchanged, and no hardware
  measurement is needed to decide ownership.
- **Alternatives to compare:** validate the existing private group specification before any lookup;
  keep robot-owner checks; add public motor/group identity values; validate only at SDK lookup; or
  add a global cross-Plant hardware registry. Preserve the FTC-01 use case where an external encoder
  intentionally names a powered motor's own channel, and distinguish duplicates inside one
  actuator group from reuse across independent owners.
- **Leading hypothesis:** keep the current fluent staged API byte-for-byte for valid callers. Validate
  nonblank and unique names once the group identity specification is complete and before
  resolving/configuring hardware. Add no public group/spec type or global registry. MAP-01 owns
  scale/bias and native-domain mathematics.
- **Completion:** focused fake-HardwareMap tests prove a freshly supplied invalid group has no lookup,
  direction, mode, tuning, or command side effects, while a rejected add through a retained stage
  alias causes no additional effects and does not mutate the earlier valid group; errors identify
  the group and offending name. Valid direct-power, device-managed, regulated, same-motor external-
  feedback, standard-servo, CR-servo, FTC mecanum, and Pedro mecanum cases remain supported.
- **Decision record (2026-07-17):**
  - **Confirmed `FtcActuators` behavior:** its one public construction layer is
    `FtcActuators.plant(hardwareMap)` followed by the staged motor, standard-servo, or CR-servo
    answers. All concrete builders and the shared member `Spec` are private. That `Spec` currently
    null-checks `name` and `direction` but accepts blank and repeated names. Motor power/velocity/
    position, standard-servo position, and CR-servo power/position then resolve those names through
    separate paths. Device-managed motor and servo/CR group paths can configure an earlier child
    before a later invalid SDK lookup; a repeated name can resolve one SDK object more than once,
    set its direction or tuning more than once, and submit multiple ordered commands to it.
  - **Why validation cannot wait for final build:** motor and CR-servo `.power()` resolve their
    outputs immediately, and regulated feedback answers can resolve an encoder before `build()`.
    Public staged-builder objects are also mutable aliases: a caller can retain the group step,
    create a downstream velocity/position builder that retains the same private parent list, and
    then call another `andMotor(...)`, `andServo(...)`, or `andCrServo(...)`. A one-time transition
    check can therefore be bypassed unless the group is copied/sealed or every later resolution
    revalidates it. Repository callers all use ordinary inline chains, so a private validate-before-
    add invariant is both smaller and stronger than snapshots, sealing, or repeated late checks.
  - **FTC SDK identity fact:** this repository pins FTC SDK `RobotCore` 11.1.0. Inspection of the
    pinned `HardwareMap.get(...)`, `tryGet(...)`, and insertion bytecode confirms that the SDK calls
    `String.trim()` and then performs an ordinary case-sensitive map lookup. The configuration-
    identity key for this change is therefore `name.trim()` with case-sensitive equality:
    `"left"` and `" left "` are the same requested device, while `"left"` and `"Left"` remain
    distinct. Preserve the caller's raw string for lookup and diagnostics; do not lowercase,
    Unicode-normalize, or claim that different keys prove different physical ports.
  - **Repository caller audit:** there are 16 executable `andMotor(...)`/`andCrServo(...)` group
    additions across Phoenix, modern framework tools, and focused tests; there is currently no
    executable standard-servo group caller, but that parallel public family is documented and
    supported. Phoenix `ScoringPath` owns one two-CR-servo transfer group. Shooter examples own
    device-managed motor groups and CR-servo power groups. Motor singles, servo singles, and the
    existing external-feedback paths remain relevant preserved cases. Within `FtcActuators`, no
    caller stores, shares, composes, or independently validates a public group-specification value,
    and no such public value exists; every actuator group is supplied inline from literals,
    constants, or profile strings.
  - **Bettabot evidence and exact simplification:** at pinned Summer26 commit
    [`4eed9d6c`](https://github.com/Hansika1098/Summer26/commit/4eed9d6c7c93c5e2b65bdbc78463ad0be0e87790),
    `BettaShooter.validateConfig(...)` repeats three conditions—left name null/blank, right name
    null/blank, and exact left/right equality—across nine source lines before supplying the same
    names to its grouped `FtcActuators` call. An adopting Bettabot may delete those branches after
    moving or eliminating its two optional raw telemetry `hardwareMap.get(...)` calls so that the
    validated Plant construction is the first name-consuming boundary. ACT-01 does not remove the
    remaining robot-owned RPM, tolerance, counts-per-revolution, gain, or output-power validation,
    and it does not claim that a short Plant call represents the whole robot subsystem.
  - **Public-layer and sibling audit:** `FtcHardware` and `FtcSensors` are distinct low-level
    command/source capabilities rather than duplicate Plant construction layers; their public
    single-device factories have no within-group identity question and remain outside this item.
    The audit did find one genuine sibling group: all eight public `FtcDrives.mecanum(...)` overloads
    converge on the custom-name overload and package-private `FtcHardware.motorPowerGroup(...)`;
    `FtcMecanumDriveLane` owns that construction for its lane. Unlike `FtcActuators`,
    `FtcDrives.MecanumWiringConfig` is deliberately a reusable public wiring value with
    `defaults()`/`copy()` and profile/tool/calibrator callers. Its four names currently reach the
    group helper, which interleaves validation with lookup and accepts blank or SDK-trim-equivalent
    duplicates. `FtcDrives.setZeroPowerBehavior(...)` consumes the same wiring value directly.
    Pedro's `PedroPathingRuntime.create(...)` consumes reusable vendor `MecanumConstants`; its
    independent preflight already rejects blank and exact duplicate names, but its used-name set
    does not yet use the SDK's trimmed identity key.
  - **Alternatives compared:** documentation alone and robot-local checks leave double resolution/
    commands possible and repeat framework-owned knowledge. Validation only inside SDK lookup is too
    late, misses several group paths, and may initialize/configure hardware first. A final-build or
    domain-transition-only check is bypassable through retained stage aliases unless the builder is
    copied or sealed. A public `HardwareName`, group spec, builder, or registry adds a student-facing
    concept even though callers only provide inline strings; a global/per-`HardwareMap` registry
    also invents release/handoff lifecycle and would reject legitimate independent builds and
    feedback sharing. Java object-identity comparison requires lookup and still cannot prove
    physical-port identity under arbitrary SDK/vendor aliases.
  - **Recommended `FtcActuators` design:** keep every public nested interface, method, argument,
    return type, and valid fluent call unchanged. Use one shared private validate-and-add path for
    `motor`/`andMotor`, `servo`/`andServo`, and `crServo`/`andCrServo`. Before mutating the private
    spec list, validate in the existing observable order—null name, null direction, trim-empty
    name, duplicate trimmed key, then mutation—and reject a case-sensitive trimmed key already used
    by that one homogeneous commanded group. Preserve the existing null exception types/messages
    and failed-add non-mutation. Internal named-encoder membership should compare the same SDK-
    equivalent key, so harmless surrounding whitespace does not create a second identity rule.
  - **Recommended mecanum parity:** because API parallelism and the same double-command hazard make
    an FtcActuators-only fix incomplete, prevalidate the complete package-private motor-power group
    before its first `HardwareMap` lookup and apply the same check before direct mecanum zero-power-
    behavior configuration. Make Pedro's existing duplicate-name preflight use the same trimmed,
    case-sensitive key. Keep any shared FTC helper package-private; add no public name/group API.
    This is a bounded expansion from the original FtcActuators-only leading hypothesis and is the
    reason implementation requires explicit approval.
  - **Ownership and escape hatches preserved:** uniqueness is scoped only to commanded members of
    one actuator or drivetrain group. A regulated Plant may deliberately use
    `.externalEncoder(...)` or an internal-encoder selector naming one selected motor, including the
    Bettabot left flywheel's encoder channel. Separate read-only feedback may be shared, and
    separately constructed Plants/modes may reuse one configured name; ACT-01 does not make
    simultaneous writers valid or add hidden ownership state. Different trimmed keys are accepted
    without claiming they are different physical hardware.
  - **Error contract:** blank/duplicate failures occur at the earliest fully informed group
    boundary, before the rejected member enters the private spec or that add can cause any new
    lookup/configuration effect. A caller may already have resolved the earlier valid group through
    a retained public stage alias; validation cannot erase that prior work, but rejection neither
    mutates that group nor adds another side effect. Messages identify `FtcActuators`, FTC motor-
    power/mecanum, or Pedro ownership as applicable; name the actuator family/member or configured
    field; show the offending raw/effective name and earlier conflict where useful; and tell the
    student to use a distinct configured hardware name. Missing and wrong-type real devices remain
    ordinary SDK lookup failures after the complete name specification passes.
  - **Bounded implementation scope:** limit production edits to private FTC group-name validation,
    `FtcActuators`' shared add path/internal selected-motor comparison, the package-private motor
    group, direct mecanum zero-power configuration, and Pedro's existing name-set key. Synchronize
    only affected Javadocs, Framework Principles, the beginner/overview hardware sections, the FTC
    actuator-group guide, and relevant Pedro/drive guidance. Do not change robot behavior code,
    public construction types, low-level single-device factories, mapping mathematics (MAP-01),
    motor-mode ownership, controller policy, or physical hardware discovery.
  - **Verification plan:** add focused fake-`HardwareMap` coverage for first/later blank names and
    exact/trim-equivalent duplicates across motor power/velocity/position, standard-servo position,
    and CR-servo power/position. For each `FtcActuators` family, cover first/later null name and null
    direction with the existing exception contract and failed-add non-mutation. For the package-
    private motor group, cover null/empty/mismatched lists and a later null name/direction before any
    lookup. For fresh invalid groups, assert zero lookup, direction, mode, power, position, velocity,
    `setPositionPIDFCoefficients`, `setVelocityPIDFCoefficients`,
    `setTargetPositionTolerance`, and `setZeroPowerBehavior` side effects. Use a fake
    `HardwareMap` that emulates SDK-trimmed lookup; cover actionable messages, accepted case-
    distinct configured keys, valid surrounding whitespace, and a retained stage alias whose
    earlier valid resolution remains unchanged with no additional effect after a rejected add.
    Preserve the same powered motor as external velocity feedback and external position feedback—
    including a trim-equivalent spelling—canonical `internalEncoder(String)` selection, direct-
    power, device-managed, regulated, servo, CR-servo, and separate-Plant reuse cases. Cover all
    converged FtcDrives construction plus direct zero-power configuration and Pedro trimmed-key
    parity. Run the focused FTC/Pedro suites, full `:TeamCode:testDebugUnitTest`,
    `:TeamCode:compileDebugJavaWithJavac`, XML counts, exhaustive caller/API searches,
    documentation-link/diff/whitespace checks, independent reviews, and Android Studio inspection.
    This is a string/specification safety contract; no physical robot run is needed.
  - **Approval gate:** the private add-time FtcActuators invariant preserves and simplifies the
    leading hypothesis, but extending SDK-equivalent group validation to the supported FTC/Pedro
    mecanum siblings materially broadens the original scope. It is also a public invalid-input
    timing/behavior change across supported construction paths. Do not edit Java implementation
    until the user explicitly approves this expanded ACT-01 design.
  - **Design approval (2026-07-17):** user approved the expanded ACT-01 design, including private
    SDK-equivalent validation for `FtcActuators`, FTC mecanum owners, and Pedro's existing mecanum
    preflight. Gate 2 implementation is authorized for this item only.
  - **Gate 2 implementation (2026-07-17):**
    - Added one package-private FTC name-group helper that uses the SDK's trimmed, case-sensitive
      identity key while preserving raw lookup strings and exposing no student-facing type.
      `FtcActuators` now routes motor, standard-servo, and CR-servo additions through one private
      null-name → null-direction → blank-name → duplicate-name → mutation invariant. Its named
      internal-encoder selectors use the same identity key.
    - `FtcHardware.motorPowerGroup(...)` now validates the complete list shape and every
      name/direction before its first lookup. `FtcDrives`' eight factory overloads continue to
      converge there, while direct zero-power/brake configuration prevalidates the same reusable
      wiring names. `FtcMecanumDriveLane` inherits the check through its existing owner path.
    - Pedro's existing `MecanumConstants` preflight now stores trimmed case-sensitive keys, so a
      trim-equivalent duplicate fails before vendor drivetrain construction. No public method,
      argument, return type, staged answer, or valid robot call changed.
    - Synchronized Javadocs, Framework Principles, the beginner/overview/FTC actuator guides, the
      Pedro lifecycle reference, and Pedro integration guidance. Framework documentation contains
      no downstream case-study project reference.
  - **Automated verification (2026-07-17):**
    - Focused ACT-01 suites passed: `FtcActuatorGroupIdentityValidationTest` 18/18 and
      `PedroPathingRuntimeValidationTest` 5/5, with zero failures, errors, or skips.
    - Full
      `:TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` completed successfully:
      54 XML suites, 539 tests, zero failures, zero errors, and zero skips. Output contained only
      the repository's existing Java-8-on-JDK-21 source/target warnings and deprecated FTC controller
      sample note.
    - `git diff --check` and a changed-plus-untracked trailing-whitespace scan passed. Exhaustive
      caller/construction searches confirmed the private helper boundary, both package-private motor-
      group callers, all converged FtcDrives paths, Pedro preflight, preserved feedback sharing, and
      no added public/protected production signature. Independent implementation/API, tests, and
      documentation reviews reported no remaining findings.
  - **Manual verification scope:** inspect the private validation/error messages, unchanged
    staged call sites, FtcDrives/Pedro parity, same-channel feedback exception, tests, and
    synchronized docs in Android Studio. No physical robot run is required for this configuration-
    string contract; a robot run cannot prove that differently named configuration entries identify
    different physical devices.
  - **Manual verification (2026-07-17):** user reviewed the ACT-01 implementation in Android Studio
    and confirmed that it looks good. Gate 3 finalization and publication are authorized for this
    item only.

### CAL-01 - Calibration-search power validation

- **Problem to confirm:** `PositionCalibrationTasks.withPower(...)` accepts non-finite or
  out-of-range normalized power, while `MappedPositionPlant.beginCalibrationSearch(...)` can stop
  normal output and change calibration state before a bad value reaches the hardware adapter.
  `NaN` can pass through the current clamp.
- **Alternatives to compare:** documentation only; task-factory validation only; direct Plant
  validation only; both earliest staged validation and defensive direct-boundary validation; a new
  normalized-power value type; or an additional builder stage.
- **Leading hypothesis:** keep `.withPower(double)` and the existing calibration lifecycle API.
  Require finite inclusive `[-1, +1]` power immediately at the task stage and defensively at the
  direct Plant boundary before any stop, state change, or write. Do not invent a wrapper for one
  inline answer or impose a robot-specific gentle/nonzero/direction policy.
- **Completion:** tests cover every boundary value and failure before side effects, direct and
  task-driven entry, active cancellation/return behavior, and actionable normalized-power
  diagnostics. Documentation distinguishes framework validity from the robot's safe homing power.
- **Decision record:** _Pending; split from API-03 and SAFE-03's open-loop calibration boundary on
  2026-07-16._

### CAL-02 - Position-calibration reference validity

- **Problem to confirm:** position reference and hold answers such as
  `plantPositionMapsToNative(...)`, `assumeCurrentPositionIs(...)`, and the post-search hold target
  have multiple public entry paths and may accept non-finite values before calibration state changes.
  Unlike normalized search power, their units and valid range depend on the configured Plant map.
- **Alternatives to compare:** validate each staged answer immediately; validate only when the full
  Plant range/map is known; add a reusable calibration-reference value; rely on final target guards;
  or keep direct advanced-Plant and beginner-facade paths with parallel checks. Inventory every path
  and determine which boundary first knows the value's declared units and legal range.
- **Leading hypothesis:** preserve direct answer methods and validate at the earliest fully informed
  owner, before mutating calibration state. Add no wrapper if callers use the answer inline, and
  keep runtime target guards as separate defense.
- **Completion:** every public calibration-reference/hold path has parallel finite/range semantics,
  actionable plant-unit diagnostics, and tests proving rejection before lifecycle or hardware side
  effects.
- **Decision record:** _Pending; split from CAL-01 during API-03 review on 2026-07-16._

### MAP-01 - FTC actuator mapping-domain validation

- **Problem to confirm:** standard-servo endpoint maps and grouped child transforms may compute raw
  values outside the SDK's `[0, 1]` servo domain and then be silently clamped, making commanded and
  applied position meaning diverge. Non-finite affine inputs or arithmetic overflow can likewise
  survive partial checks.
- **Alternatives to compare:** finite scale/bias checks in the existing staged answers; raw-domain
  validation only in standard-servo builders; require callers to pre-clamp endpoints; report
  applied native clamp; or add endpoint/mapping value objects. Inventory direct mapped-Plant and
  FtcActuators paths, but keep generic `ScalarRange` construction in RANGE-01.
- **Leading hypothesis:** validate finite affine parameters at their existing FTC staged answers and
  enforce `[0, 1]` only at the standard-servo boundary that knows that native domain. Preserve final
  SDK clamping as defense, and add no wrapper unless a mapping is demonstrably stored or shared.
  Calibration reference/offset answers remain exclusively in CAL-02 even when they contribute to a
  later affine map.
- **Completion:** every accepted endpoint/child transform is finite and maps the declared Plant range
  into its native domain without silent configuration clamp; tests cover negative scale, endpoints,
  group transforms, overflow, and command/applied truth.
- **Decision record:** _Pending; split from API-03 on 2026-07-16._

### RANGE-01 - ScalarRange construction validity

- **Problem to confirm:** `ScalarRange.bounded(min, max)` checks ordering but can admit `NaN` because
  comparisons with `NaN` are false. Other framework code intentionally uses unbounded ranges, so a
  blanket finite-only rule could remove a useful capability or create multiple spellings for the
  same range.
- **Alternatives to compare:** require finite `bounded(...)` endpoints and keep one explicit
  `unbounded()` factory; permit intentional half-bounded infinities; add named lower/upper-bounded
  factories; validate only when a Plant consumes the range; or introduce constrained endpoint
  values. Audit all range factories, constructors, and callers before deciding.
- **Leading hypothesis:** reject `NaN` at construction and expose each supported bounded/unbounded
  shape through one obvious factory. Do not use FTC servo `[0, 1]` policy in this core value.
- **Completion:** the accepted finite, half-bounded, and unbounded contracts are explicit, every
  caller has one construction path, and tests cover `NaN`, infinities, equal/reversed endpoints,
  containment, and Plant-target safety integration.
- **Decision record:** _Pending; split from MAP-01 during API-03 review on 2026-07-16._

### FTC-02 - Device-managed controller configuration validation

- **Problem to confirm:** FTC device-managed velocity/position PIDF or P coefficients and maximum
  power are staged separately from framework `Pid`, but several paths accept non-finite values and
  excessive maximum power that is later clamped. SDK access or run-mode changes are too late to
  explain the configuration error.
- **Alternatives to compare:** validate each existing staged answer; add an immutable reusable FTC
  tuning value; defer to the SDK; clamp silently; or replace device-managed control with framework
  regulation. Compare actual callers to determine whether numeric overloads or a tuning bundle have
  independent reuse value.
- **Leading hypothesis:** preserve both device-managed and framework-regulated paths because they
  are distinct capabilities. Validate finite SDK-controller settings and normalized maximum power
  at the existing staged answer before lookup/mode mutation; keep hardware saturation as defense and
  avoid a tuning wrapper used only inline.
- **Completion:** focused tests cover all device-managed motor/CR-servo paths, invalid values before
  SDK side effects, boundary values, diagnostics with units, and unchanged valid student calls.
- **Decision record:** _Pending; split from API-03 on 2026-07-16._

### CONFIG-01 - Owner-configuration snapshot audit

- **Remaining evidence gap:** the API-03 audit found that `DriveGuidanceTask.Config` is mutable and
  may be retained directly, while many other long-lived owners copy private config. Source shape
  alone does not prove that a caller mutates it after construction or that a defensive copy,
  immutable value, or different public answer would simplify guidance code.
- **Resolved route half:** ROUTE-03 proved that `RouteTask.Config` was only a mutable, immediately
  consumed one-field wrapper. It removed that type, its retained alias, and non-positive sentinel
  semantics in favor of validated direct factory answers. Route configuration is no longer an open
  part of CONFIG-01.
- **Why Deferred:** “make all configs immutable” would still combine unrelated owners and add
  wrappers or withers without traced robot-code benefit. Resume only for the named
  `DriveGuidanceTask.Config` owner after a production, Phoenix, tool, or test caller demonstrates
  mutation drift or an invalid retained-state failure; then compare an owner-specific defensive
  copy, immutable reusable value, direct staged answer, and no-change design.
- **Decision record (updated 2026-07-17):** **Deferred for concrete DriveGuidance caller evidence.**
  This is an audit note, not an implementation-ready umbrella task.

### TIME-01 - Epoch-safe LoopClock timestamps

- **Problem to confirm:** a scalar timestamp in the `LoopClock.nowSec()` coordinate does not retain
  which `LoopClock` or deliberate clock-reset epoch produced it. Asking infrastructure consumers to
  carry a second raw `epoch` value would duplicate pairing, validation, freshness, and comparison
  logic throughout vision, localization, spatial solving, target planning, and history-backed
  sources. Resetting `LoopClock` also returns `cycle()` to zero, so a cycle-memoized owner can return
  a pre-reset cached value before it gets a chance to reject that value's timestamp.
- **Decision record (2026-07-23):** **Ready; major API approval required before implementation.**
  This item was split out of the VISION-02 implementation preflight after the reset-safety audit
  proved that the same invariant applies beyond camera ownership. VISION-02 remains a separate
  camera-frame acquisition item and waits for this reusable timing foundation.
  - **Confirmed behavior:** `LoopClock.reset(...)` currently resets both time and `cycle()` and can
    reset them to exactly the values they already have. A retained scalar timestamp therefore
    cannot distinguish `reset(0.0)` at cycle zero from no reset. More importantly, many stateful
    owners memoize only by the scalar `clock.cycle()`: an exact pre-/post-reset cycle collision can
    return the old snapshot without running any timestamp check. The current public timing spine
    also repeatedly implements `nowSec - timestampSec`, `Math.max(storedAge, now - timestamp)`,
    finiteness checks, future-time tolerance, and bare-double ordering/deduplication. These are the
    same rules, not independent robot policy.
  - **Current callers and escaped values:** the audit found portable LoopClock-domain timestamp
    scalars across approximately 30 main framework files. They escape acquisition owners through
    `PoseEstimate`, `MotionDelta`, `PinpointKinematicSnapshot`, `TimeAwareSource`, AprilTag values,
    `FacingSolution`, `TranslationSolution`, TARGET-02 candidates/requests/plans, correction
    histories, and diagnostics. A vision-only timestamp wrapper would immediately lose its reset
    identity when converted to `PoseEstimate` or a target request, while a public raw epoch would
    make every one of these values carry two arguments that could be accidentally mismatched.
    Task-local start/deadline seconds do not escape their one Task lifecycle and are not part of
    this problem. FTC/vendor capture clocks and the process-monotonic Auto-to-TeleOp handoff are also
    separate domains and remain boundary-owned scalars until deliberately translated.
  - **Ordinary robot-code comparison:** robot code should pass one value, not an epoch and a scalar.
    A TARGET-02 call remains the same conceptual size:

    ```java
    return PlantTargetRequest.observedEquivalentPosition(
            facing.sourceId(),
            targetDeg,
            facing.quality(),
            facing.timestamp()
    );
    ```

    A custom age-native sensor owner anchors its fact once with
    `clock.timestampSecondsAgo(frameAgeSec)`. Consumers ask the retained value for
    `timestamp.ageSec(clock)`, `timestamp.isFresh(clock, maxAgeSec)`, or the signed duration
    `newerTimestamp.secondsSince(olderTimestamp)`; they never read, store, or compare an epoch.
    Existing high-level selection builders continue to own freshness policy and add no
    student-facing answer.
  - **Alternatives considered:**
    - Documentation-only or retaining raw `double timestampSec` values leaves reset aliasing,
      repeated comparison code, and age/timestamp drift intact.
    - A public `long LoopClock.epoch()` is the smallest camera-owner patch, but it exposes two
      independently passable primitives. It cannot keep an escaped pose, spatial result, or target
      candidate paired correctly and it does not identify timestamps from two different clocks.
    - An opaque epoch token still requires every value and method to carry a two-argument pair; it
      prevents numeric epoch mistakes but not separation or repetitive comparisons.
    - A camera-only frame-time wrapper fixes one boundary but creates a parallel time type and loses
      its invariant at the existing localization, spatial, and target-planning conversions.
    - Reset callbacks or clearing every consumer from the composition root cannot invalidate
      already copied immutable facts and adds lifecycle bookkeeping to robot code.
    - Detecting reset from time/cycle regression misses exact same-value resets. Making only
      `cycle()` monotonic prevents cache collision but does not make two timestamp scalars
      comparable across a reset.
    - Passing a complete clock snapshot with time, delta, cycle, and reset identity gives captured
      measurements unrelated heartbeat fields and makes every data value larger conceptually.
    - A public `timestampAtSec(double)` factory is not required by a current production producer.
      It could incorrectly bless an arbitrary or prior-epoch scalar as belonging to the current
      clock. Current samples, delayed camera/sensor facts, and propagation are fully covered by
      “now,” “seconds ago,” and forwarding an existing typed value; tests do not justify a
      production escape hatch.
    - Java `Comparable` is not appropriate because timestamps from different clocks or reset epochs
      have no truthful total ordering.
  - **Chosen timestamp design:**
    1. Add one final immutable core value, `LoopTimestamp`. It privately retains its originating
       `LoopClock` identity, reset epoch, and finite seconds coordinate. There is no public
       constructor, `of(...)`, raw epoch accessor, or public scalar timestamp accessor. A valid
       timestamp is created only by the clock that owns its timebase through
       `nowTimestamp()` or `timestampSecondsAgo(...)`. The latter is the explicit boundary path for
       an age-native measurement and rejects non-finite or negative age. An existing captured fact
       is propagated unchanged rather than reconstructed from a scalar.
       `LoopTimestamp.unavailable()` is the one distinct no-fact sentinel used instead of `null`
       when a containing value has no truthful measurement time. A no-result value may still carry
       `nowTimestamp()` when its own contract explicitly timestamps the production of that result;
       it must not pretend that time is an underlying measurement time. A structural
       `isAvailable()` distinguishes only the sentinel and does not claim freshness or a current
       epoch.
    2. Put the reusable interpretation on `LoopTimestamp`: `ageSec(clock)` and
       `isFresh(clock, maxAgeSec)` centralize finite/non-negative validation, the TARGET-02
       one-microsecond future tolerance, inclusive freshness boundaries, clock identity, and reset
       epoch. Unavailable or prior-epoch values fail closed (`NaN` age and not fresh); passing a
       different `LoopClock` is an actionable wiring error. One signed `secondsSince(...)` operation
       owns pairwise duration, ordering, deduplication, and interpolation without exposing a scalar
       coordinate or adding redundant before/after methods. It returns `NaN` for unavailable or
       non-current/different-epoch values so an ordinary reset invalidates retained history without
       crashing the loop, and throws an actionable error for different clock ownership. Callers
       apply their algorithm-specific tolerance to the signed duration; the TARGET-02 one-microsecond
       future tolerance belongs only to age/freshness against the current clock.
    3. Replace portable public `double timestampSec` and canonical stored-age pairs with the one
       typed value. `TimeAwareSource.getAt(...)`, localization measurements/deltas, history keys,
       spatial solutions/selections, generic observations, and TARGET-02's observed candidate path
       carry `LoopTimestamp`. Derived `ageSec` remains only as an on-demand diagnostic snapshot when
       it has a distinct documented meaning; it is not a second canonical freshness input. Delete
       the replaced scalar APIs and migrate all in-repository callers together rather than retaining
       deprecated parallel paths.
    4. Keep raw FTC/vendor timestamps private to their boundary. The boundary computes a truthful
       coordinate or age, asks its current `LoopClock` to create one `LoopTimestamp`, and thereafter
       forwards that value unchanged. Lifecycle-local Task timers remain primitive seconds because
       they never cross an epoch or public measurement boundary.
  - **Chosen cycle design:** preserve `cycle()` as the stable per-cycle identity, but make it
    monotonically increasing for the lifetime of one `LoopClock`. Every explicit `reset(...)`
    advances it immediately instead of returning it to zero, even when the reset time is unchanged;
    this makes post-reset reads miss every pre-reset cycle cache before the next update. The first
    implicit `update(...)` initializes directly as cycle one rather than invoking public-reset
    semantics. `cycle()` is an identity, not an elapsed-loop count. This does not replace explicit
    state resets or permit switching among multiple `LoopClock` objects; the one-stable-clock
    ownership principle remains.
  - **Framework Principles and simplicity check:** one clock still owns the heartbeat and all
    translations; SDK/vendor details stay at explicit boundaries; reusable timing correctness lives
    in core instead of robot or season code; and students carry one named value whose type prevents
    mismatched primitives. The wrapper is justified by concrete cross-layer reuse and misuse
    prevention, unlike the previously rejected camera-only noun. The common call remains one timing
    argument and the advanced sensor-owner call becomes shorter than separately constructing an
    epoch/timestamp pair.
  - **Bounded implementation scope:** add `LoopTimestamp` and its `LoopClock` factories; change
    `LoopClock.reset(...)` cycle identity; migrate the portable timing spine named above, its current
    production callers, tests, Javadocs, examples, and relevant framework guides; and update
    Framework Principles with the epoch-safe captured-time and monotonic-cycle rules. VISION-02 does
    not implement webcam/Limelight frame identity, latency anchoring, or camera result propagation
    in this item; it consumes the completed type afterward. Do not start TARGET-03 or the broader
    CYCLE-01/CYCLE-02 state-update audits.
  - **Verification plan:** extend `LoopClockTest` for construction, first implicit update, normal
    updates, reset-to-the-same-value, repeated resets, immediate post-reset identity, and monotonic
    cycle behavior. Add `LoopTimestampTest` for both clock factories, unavailable values, current age,
    inclusive freshness, tolerated and materially future values, prior epochs, different clocks,
    ordering, and durations. Add at least one exact-cycle-reset cache regression and migrate focused
    target-planning, generic-observation, spatial, localization, predictor/fusion, and time-aware
    history tests. Run exhaustive public-signature/caller searches for portable
    `double timestampSec` and redundant canonical `ageSec`, the focused suites, full TeamCode unit
    tests, TeamCode Java compilation, Markdown checks, and `git diff --check`. No hardware is needed
    to prove these deterministic value, reset, and cache contracts.
  - **Approval gate:** this is a cross-framework breaking API migration and changes public
    `LoopClock.cycle()` reset semantics. Per the framework-improvement workflow, stop before
    implementation and request explicit approval with
    `Approve TIME-01 epoch-safe LoopTimestamp prerequisite`.
  - **Design approval (2026-07-23):** the user replied
    `Approve TIME-01 epoch-safe LoopTimestamp prerequisite`. This authorizes Gate 2 implementation
    of TIME-01 only. VISION-02 remains paused until this prerequisite is implemented, reviewed,
    published, and merged.
  - **Implementation (2026-07-23):** core now provides one immutable `LoopTimestamp` created only by
    `LoopClock.nowTimestamp()` or the explicit age-native boundary
    `LoopClock.timestampSecondsAgo(...)`. It privately retains clock identity, reset epoch, and its
    finite coordinate; `ageSec(...)`, `isFresh(...)`, and `secondsSince(...)` centralize reset,
    clock-wiring, inclusive freshness, future-tolerance, duration, ordering, and deduplication
    behavior without exposing a raw epoch or seconds accessor. `LoopClock.cycle()` is now monotonic
    for one clock lifetime and advances immediately on every explicit reset, including reset to the
    same numeric time.
  - **Portable timing migration and simplicity (2026-07-23):** TARGET-02 candidates/requests/plans,
    generic observations, pose estimates, motion deltas, Pinpoint snapshots, time-aware sources,
    spatial solutions/selections, correction histories/diagnostics, AprilTag portable values, FTC
    estimator boundaries, Pedro localization, current callers, examples, and telemetry now pass one
    typed timestamp. Replaced raw scalar APIs were deleted rather than deprecated. Derived age
    remains only where it is a current diagnostic or an explicit policy input. Ordinary robot code
    still supplies one timing argument, while reusable sensor owners translate a native age once;
    students never carry or compare a separate epoch.
  - **Reset-safety refinements (2026-07-23):** cycle caches miss immediately after reset;
    `Source`/`ScalarSource` last-valid holds fail closed instead of rejuvenating retained values;
    position-derived rate feedback re-establishes its baseline; spatial frame/mount history and
    AprilTag selection reject unavailable, stale, or prior-epoch observations before history lookup;
    and localization fusion/EKF history ordering, interpolation, replay, and duplicate handling use
    the typed comparison. Freshness-source factories now reject invalid maximum-age configuration at
    construction rather than waiting for a loop sample.
  - **VISION-02 boundary (2026-07-23):** this prerequisite deliberately does not implement stable
    webcam/Limelight frame identity, camera-latency anchoring, or the approved geometry-only
    AprilTag frame factory. Current FTC adapters translate their existing age facts into
    `LoopTimestamp`; VISION-02 remains **Ready** and paused until TIME-01 is reviewed, published, and
    merged.
  - **Automated verification (2026-07-23):** the focused TIME-01/core migration set passes 70 tests,
    including exact same-time reset invalidation, different-clock failures, inclusive/future
    freshness, exact-cycle cache invalidation, time-aware history, target planning, spatial history,
    AprilTag values, fusion/EKF interpolation and replay, and Pedro passive localization. The final
    unfiltered `:TeamCode:compileDebugJavaWithJavac :TeamCode:testDebugUnitTest` run passes with
    **83 suites / 801 tests / 0 failures / 0 errors / 0 skipped**. Public-signature/caller scans find
    no remaining portable `double timestampSec` APIs, old timestamp getters, or downstream
    `now - sampledAge` reconstruction; the remaining raw FTC/vendor and match-handoff time domains
    stay at their documented boundaries. Changed-Markdown fence and local-link checks pass, and
    `git diff --check` passes. Independent timing/core, localization/fusion, spatial/observation,
    API/documentation, and adversarial reviews found no remaining priority issue.
  - **Android Studio audit point (2026-07-23):** implementation and deterministic software
    verification were presented for user inspection. No robot hardware is required for this
    value/reset/cache contract; physical camera timing remains VISION-02 adoption validation.
  - **Manual verification (2026-07-23):** the user replied `TIME-01 looks good`, approving the
    implementation for Gate 3 publication and merge. TIME-01 is **Done**; this approval does not
    authorize starting VISION-02 or another tracker item.

### VISION-02 - Stable AprilTag observation timestamps

- **Problem to confirm:** TARGET-02's adversarial review found that
  `AprilTagDetections.frameTimestampSec(clock)` derives `clock.nowSec() - ageSec` on every call when
  the snapshot has no explicit timestamp. Re-reading one cached age-only detections object can
  therefore assign the same frame a newer timestamp every loop. `AprilTagSpatialSolveLane` consumes
  that helper, while current FTC webcam and Limelight adapters publish age-only detection snapshots.
- **Alternatives to compare:** require every detections snapshot to carry a stable `LoopClock`
  timestamp; let each FTC owner stamp once using camera-frame/result identity; retain age-only
  snapshots but make timestamp-dependent consumers reject them; or add cache/identity inference in
  the spatial consumer. Audit webcam acquisition time, Limelight result identity,
  localization, custom sensors, tests, and every public detections factory before choosing.
- **Leading hypothesis:** acquisition owners have the facts needed to anchor time. The webcam owner
  should translate frame acquisition time once; the Limelight owner should latch one timestamp per
  stable result identity; immutable detection snapshots should retain it; and downstream getters
  should return the retained timestamp rather than re-anchoring age. Keep age only as a diagnostic
  if it still has distinct value, and do not move vendor identity logic into spatial or robot code.
- **Completion:** repeatedly consuming one cached frame preserves one timestamp and its derived age
  grows; new frames/results receive new timestamps; webcam, Limelight, localization, spatial, and
  no-result paths remain truthful; and target-planner examples receive stable timestamps without
  student bookkeeping.
- **Decision record (2026-07-23):** **Ready; major API approval required before implementation.**
  This supersedes the pending hypothesis above while preserving its acquisition-owner direction.
  TARGET-02 deliberately fixed only the candidate/planner boundary; VISION-02 owns the upstream
  camera-frame contract.
  - **Confirmed behavior:** `AprilTagDetections` currently stores both a creation-time `ageSec` and
    an optional `timestampSec`. Its public age-only factories leave the timestamp unknown, and
    `frameTimestampSec(clock)` then computes `clock.nowSec() - ageSec` every time it is called. For
    one cached snapshot with `ageSec = 0.10`, calls at loop times `5.0` and `6.0` therefore report
    timestamps `4.90` and `5.90` for the same camera frame. The opposite error exists in
    `isFresh(...)`, `forId(...)`, `freshObservations(...)`, `freshMatching(...)`, and
    `visibleIds(...)`: they compare the fixed creation-time age without a clock, so that same
    snapshot can remain fresh forever. The public dual age/timestamp factory can also accept two
    contradictory answers to one freshness question.
  - **Traced consequences:** `AprilTagSpatialSolveLane` is the only timestamp-fallback caller, then
    separately copies frozen detection age into its solutions. `AprilTagPoseEstimator` filters by
    frozen age, rebuilds an age-only detections object, and recomputes `now - age`, discarding even
    an explicit upstream timestamp. `AprilTagSources`, `TagSelections`, and `SpatialQuerySupport`
    all gate through the same clock-less helpers. A cached frame can consequently defeat tag
    selection expiry, dynamic camera-mount history lookup, corrected-localization timestamp
    deduplication/replay, spatial freshness, and the stable timestamp expected by TARGET-02.
    `LimelightFieldPoseEstimator` and `CustomVisionOwnershipExample` independently repeat
    `clock.nowSec() - result.ageSec()`, so fixing only the AprilTag value object would leave the
    direct Limelight path wrong.
  - **Current construction paths and ordinary robot code:** the public detections surface is
    `none()`, `none(ageSec)`, `of(ageSec, observations)`, and
    `of(ageSec, timestampSec, observations)`; there is no public constructor. In-repository
    nonempty snapshots are created only by the FTC webcam and Limelight adapters, while custom
    `AprilTagSensor` implementations are the advanced external extension seam. Normal robot code
    constructs a vision lane, borrows its `tagSensor()`, and usually configures
    `TagSelections...freshWithinSec(...)`; that student-facing path will not gain another object or
    builder answer. Direct raw inspection already has the shared clock for `tags.get(clock)` and
    will pass that same clock when asking for age/freshness.
  - **Acquisition evidence:** the pinned FTC SDK documents webcam
    `frameAcquisitionNanoTime`/`CameraFrame.getCaptureTime()` in the Control Hub
    `System.nanoTime()` domain, and the AprilTag processor assigns the one capture value to every
    detection from that processed frame. It is therefore a stable webcam frame identity, while the
    owner must translate its origin into the `LoopClock` domain rather than treating raw nanoseconds
    as loop seconds. For Limelight, the pinned SDK sets `controlHubTimeStamp` when it receives/parses
    a result and calculates staleness from that wall-clock receipt; that is not camera capture time.
    The same SDK exposes Limelight-local result timestamp `ts`, capture latency `cl`, and targeting
    latency `tl`. The
    [Limelight JSON results specification](https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification)
    defines `ts` as milliseconds from camera boot, `cl` as the delay from exposure to tracking, and
    `tl` as tracking latency. Use the stable camera-local value as the primary result identity and
    the Control Hub receipt value only when that local identity is unavailable, not as a claimed
    exposure timestamp.
  - **Ordinary call-site comparison:** the most common selection call stays unchanged:

    ```java
    TagSelectionSource scoringTag = TagSelections.from(vision.tagSensor())
            .among(SCORING_TAG_IDS)
            .freshWithinSec(0.25)
            .choose(TagSelectionPolicies.closestRange())
            .continuous()
            .build();
    ```

    Direct inspection becomes explicit about the one loop heartbeat, without a second freshness
    value:

    ```java
    AprilTagDetections detections = vision.tagSensor().get(clock);
    Set<Integer> visible = detections.visibleIds(clock, 0.25);
    AprilTagObservation tag = detections.forId(clock, 5, 0.25);
    double ageSec = tag.frameAgeSec(clock);
    ```

    An advanced sensor owner creates complete tag geometry without repeating timing for every tag,
    then publishes its one proven frame fact once:

    ```java
    List<AprilTagObservation> observations = new ArrayList<>();
    observations.add(AprilTagObservation.target(tagId, cameraToTagPose));
    observations.add(AprilTagObservation.target(
            secondTagId, secondCameraToTagPose, fieldToRobotPose));
    return AprilTagDetections.fromFrame(frameTimestamp, observations);
    ```

    This is one scalar shorter than the current dual factory and cannot silently reinterpret an old
    `of(ageSec, observations)` call as a timestamp call. A missing lookup uses
    `AprilTagObservation.noTarget()`; a trustworthy frame that saw no tags uses
    `AprilTagDetections.fromFrame(frameTimestamp, Collections.emptyList())`.
  - **Alternatives considered:**
    - Documentation-only or no change leaves both reproduced cache failures.
    - Stamping only the two FTC adapters is the smallest local patch, but leaves public age-only
      custom sensors, frozen selection helpers, direct Limelight consumers, and contradictory
      age/timestamp construction intact. It fixes current owners without making the framework seam
      safe.
    - Keeping stored age as a public diagnostic still gives independently retained snapshots and
      observations a value that stops aging. Naming it `sampledAgeSec` would be honest but adds a
      second timing fact that no current caller needs once a timestamp exists.
    - Inferring identity or caching timestamps in spatial/localization consumers puts vendor-frame
      knowledge in the wrong layer and cannot prove whether two new wrapper objects represent one
      observation.
    - Rejecting timestamp-less values only in timestamp-dependent consumers fails safely but
      unnecessarily disables existing selection/localization behavior and preserves the unsafe
      construction path.
    - A camera-specific timestamp/identity wrapper adds a noun used only to pass one scalar and a
      list, then loses its identity when the result becomes a pose or target candidate. TIME-01's
      generic `LoopTimestamp` is different: the cross-layer audit proved that it owns the same
      reset/comparison invariant across vision, localization, spatial solving, and target planning.
      A mutable frame builder can prevent an unattached observation from existing, but makes an
      advanced adapter learn builder mutability/reuse rules and changes its existing list-building
      loop without improving the final immutable contract. Geometry-only observation factories plus
      one frame factory preserve the current extension shape with fewer arguments; the frame factory
      owns attachment and rejects accidental restamping.
    - VISION-02 does not redesign the generic `TargetObservation2d` API. The subsequently completed
      TIME-01 prerequisite made that value timestamp-canonical across the framework, so the
      AprilTag adapter now forwards the exact retained `LoopTimestamp`; it does not derive a new
      creation-time age or restamp the observation.
  - **Chosen public value design:**
    1. Make `AprilTagDetections` store one available `LoopTimestamp frameTimestamp` plus its
       immutable observation list. Keep `none()` for “no trustworthy processed frame” and add one
       explicitly named `fromFrame(frameTimestamp, observations)` factory; an empty list represents
       a trustworthy processed frame with no usable tags. Remove `ageSec`, generic `timestampSec`, both age-only
       factories, the dual factory, and the `frameTimestampSec(clock)` fallback rather than
       deprecating parallel paths.
    2. Derive `frameAgeSec(clock)` from the current shared clock. Make every freshness/selection
       helper accept that clock and fail closed for unavailable, non-finite, or materially future
       timestamps. A derived getter is not a second stored freshness representation.
    3. Replace the public observation factories with the complete parallel geometry family
       `noTarget()`, `target(id, cameraToTagPose)`, and
       `target(id, cameraToTagPose, fieldToRobotPose)`. A target made by these factories is valid
       tag geometry but has no freshness claim until a detections frame owns it; its frame timestamp
       is unknown and its freshness helpers fail closed. `fromFrame(...)` validates that every list
       member is a real target, then attaches the one frame timestamp to immutable observation
       copies. A selected observation retained from that snapshot therefore provides
       `frameTimestamp()`, `frameAgeSec(clock)`, and `isFresh(clock, maxAgeSec)` without asking an
       adapter to repeat the timestamp for every tag. Passing an observation already attached to the
       same exact frame timestamp is allowed; passing one attached to a different frame throws an
       actionable error rather than rejuvenating it. `noTarget()` is a result sentinel, is rejected
       inside a detections list, carries no frame timestamp, and is never fresh.
    4. Rename the canonical factory instead of changing the meaning of
       `of(double, List<AprilTagObservation>)`; external age-based code must fail at compile time
       with an actionable migration rather than compile with an age misread as a timestamp.
  - **Chosen acquisition and propagation design:**
    1. Add one small package-private FTC timestamp anchor/bridge, not a robot-facing abstraction.
       It maps an age-native vendor result into one `LoopTimestamp` once per stable vendor identity
       and returns that same retained value for repeats. A deliberate `LoopClock`
       epoch reset invalidates cached output before any same-cycle return and blocks that retained
       vendor identity; the owner publishes unavailable until it observes a genuinely new vendor
       frame/result and anchors that new identity in the new clock domain. Ordinary source reset
       without a clock-epoch change preserves identity history and must not make the same vendor
       frame newly observed.
    2. The webcam owner keys by processor-data generation plus `frameAcquisitionNanoTime`, computes
       the System-nano-to-LoopClock translation once, and derives all later age from the retained
       timestamp. Mixed usable frame timestamps violate the documented one-frame snapshot and fail
       the whole snapshot closed; a same-generation regressing acquisition time also fails closed.
       Empty SDK detections remain unavailable because the FTC processor supplies no empty-frame
       capture identity.
    3. The generic Limelight owner captures SDK `ts`, `cl`, and `tl` in addition to its stable
       Control Hub receipt time. It keys a result by pipeline generation and Limelight-local
       timestamp, using the stable receipt identity only when the SDK supplies no usable local
       identity. On first publication it anchors the best available approximate exposure time as
       current loop time minus receipt staleness, capture latency, and targeting latency; repeats
       retain that exact timestamp. Pipeline changes clear the result-identity epoch. Invalid timing
       fails closed. This is intentionally described as the best SDK-supported estimate, not proof
       of physical exposure time or network latency.
    4. `FtcLimelightVisionLane.ResultSnapshot` exposes the one retained
       `frameTimestamp()` needed by semantic adapters and direct localization. Remove its public
       relative `ageSec()` because it freezes when the immutable result is retained. The absolute
       Control Hub receipt time is a distinct transport/diagnostic fact rather than a second camera
       freshness answer, so retain it under the explicit name
       `resultReceivedAtControlHubMillis()` and delete the ambiguous old accessor. Receipt staleness
       remains owner-internal readiness policy. A confirmed processed no-target result becomes a
       timestamped empty detections frame, while an unready/no-result state remains `none()`.
    5. Replace the public camera-mount conversion with
       `CameraMountLogic.robotObservation2d(obs, mount, clock)`. It rejects unframed, invalid, or
       materially future observation timing and forwards the observation's exact retained
       `LoopTimestamp` into the timestamp-canonical generic `TargetObservation2d`. Migrate
       `ObservationSources` and every caller; do not keep a parallel clock-less overload.
    6. Migrate pose estimation, spatial solving, tag sources/selections, direct Limelight
       localization, tools, Phoenix telemetry, and the custom-vision example to forward the retained
       timestamp or derive current age from it. No robot/Phoenix behavior owns a frame identity or
       calls `now - sampledAge`.
  - **Framework Principles and simplicity check:** acquisition owners keep FTC/vendor identity and
    clock-origin conversion at explicit boundaries; all behavior uses the one `LoopClock`; immutable
    snapshots remain safe when cached; spatial/localization layers consume backend-neutral frame
    facts; and robot code learns one freshness representation rather than age plus timestamp. The
    normal `TagSelections` builder is unchanged, direct inspection adds only the clock already in
    scope, advanced custom sensors answer frame time once, invalid live timing fails closed, and no
    camera-specific wrapper or vendor type leaks into core.
  - **Bounded implementation scope:** update `AprilTagDetections`, `AprilTagObservation`,
    `AprilTagSensor`, `AprilTagSources`, tag selection result/sources, camera-mount observation
    conversion, FTC webcam support, the generic and AprilTag Limelight owners, AprilTag pose
    estimation, AprilTag spatial solving/support, direct Limelight field-pose estimation, affected
    framework tools/examples, Phoenix telemetry sentinels, and no-op sensor tests. Synchronize
    Framework Principles, AprilTag localization, Spatial Queries, custom vision ownership, shooter
    walkthrough/Javadocs, and all affected examples. Do not redesign generic observation timing,
    change vision ownership/lifecycle beyond timestamp cache invalidation, or begin TARGET-03.
  - **Verification plan:** add focused core tests proving one cached detections object and one
    independently retained observation keep an exact timestamp while derived age grows; inclusive
    freshness boundaries; invalid/future metadata; timestamped-empty versus unavailable snapshots;
    and cached `TagSelections` expiry. Extend `FtcWebcamVisionPortalLaneTest` for same-frame repeats,
    new frames, mixed/regressing/future frames, generation changes, ordinary source reset, and a
    clock-epoch reset that remains unavailable until a genuinely new frame.
    Extend `FtcLimelightVisionLaneTest` for same/new result identity, missing-local-identity receipt
    fallback, capture plus targeting latency, pipeline changes, a clock reset that rejects the
    retained result, invalid timing, no-target frames, same-cycle memoization, and exact
    direct-estimator propagation. Add
    core observation-construction tests for all three replacement factories, frame attachment,
    same-frame reuse, different-frame rejection, and no-target-list rejection. Add/extend AprilTag
    pose/spatial tests to prove the original timestamp reaches localization and dynamic mount
    history. Then run the
    focused suites, full TeamCode unit tests, TeamCode Java compilation, exhaustive caller and
    public-signature audits, Markdown link/fence checks, changed-file whitespace/newline checks, and
    `git diff --check`.
  - **Hardware/evidence scope:** webcam timestamp-domain identity is documented by the pinned FTC
    SDK; Limelight result identity and latency fields are documented by the pinned SDK and vendor
    result specification. Deterministic cache/propagation behavior therefore needs no hardware to
    complete. A real device remains adoption validation for firmware behavior across no-target
    frames, camera reboot, and pipeline changes, and for quantifying transport/exposure error; this
    item will not claim that software tests measured physical capture accuracy.
  - **Approval gate:** this deletes/renames public snapshot and Limelight-result APIs and changes
    freshness helpers to require `LoopClock`. Per the framework-improvement workflow, stop before
    implementation and request explicit approval with
    `Approve VISION-02 timestamp-canonical frame design`.
  - **Design approval (2026-07-23):** the user replied
    `Approve VISION-02 timestamp-canonical frame design`. This authorizes Gate 2 implementation of
    the bounded timestamp-canonical frame design for VISION-02 only; it does not authorize
    TARGET-03 or any other tracker item.
  - **Gate 2 preflight reopening (2026-07-23):** implementation has not started. The acquisition
    audit found that the approved clock-reset rule cannot be implemented exactly from the current
    public `LoopClock` facts. `reset(...)` returns `cycle()` to zero and may set `nowSec()` to the
    same value it already had, so a retained frame owner cannot always distinguish a deliberate
    reset from an unchanged clock. In particular, cycle/time-regression heuristics cannot detect
    `reset(0.0)` while the clock is already at cycle zero and time zero. Claiming exact epoch
    invalidation while using that heuristic would violate fail-closed timing and truthful API
    principles.
    - **Alternatives reconsidered:** infer resets from cycle/time regression (no API change but an
      unprovable same-value hole); add camera-specific `onClockReset()` calls (student/composition
      root bookkeeping and parallel lifecycle drift); change `cycle()` so it never resets (larger
      breaking semantic change for every cycle-aware owner); or expose the timebase identity that
      `LoopClock` already owns.
    - **Superseded supplemental proposal (not approved):** add one read-only
      `long LoopClock.epoch()` fact. A newly
      constructed clock begins in epoch zero; every explicit `reset(...)` advances the epoch even
      when time and cycle values happen to be unchanged. First-use implicit initialization through
      `update(...)` remains epoch zero. The FTC timestamp anchor retains both the `LoopClock` object
      identity and `epoch()`; any change invalidates and blocks the old vendor frame identity until
      a genuinely new frame/result arrives. `epoch()` is diagnostic/infrastructure state beside
      `cycle()`, not a second clock, timer, or robot-facing reset call.
    - **Original simplicity assessment:** ordinary robot, camera, selection, spatial, localization,
      and target-planner code adds no call. The one new concept appears only to reusable owners that
      retain facts across deliberate clock resets. It is smaller and more truthful than callbacks,
      heuristics, or changing cycle semantics, and it preserves one `LoopClock` heartbeat.
    - **Verification addition:** extend `LoopClockTest` for construction, implicit first update,
      repeated explicit resets including reset-to-the-same-value, and unchanged epoch across normal
      updates; make both camera-owner reset tests use the exact same-value case.
    - **Withdrawn supplemental approval gate:** the raw `epoch()` proposal was not approved. A
      follow-up consumer audit found that timestamps leave camera ownership through localization,
      spatial, and target-planning values, where a raw epoch would have to be copied and compared
      repeatedly. The exact-reset audit also found cycle-only cache collisions that a timestamp
      accessor does not prevent. TIME-01 records the replacement `LoopTimestamp` plus monotonic-cycle
      design and requires its own major-API approval. VISION-02 stays **Ready**, retains the already
      approved camera-frame behavior, and waits for TIME-01 before implementation; do not use
      `Approve VISION-02 LoopClock epoch accessor`.
  - **Gate 2 prerequisite satisfied and implementation resumed (2026-07-23):** TIME-01 is complete,
    reviewed, published, and merged. Its `LoopTimestamp` contract and monotonic cycle identity close
    the preflight reset/cache gap without exposing a raw epoch or adding camera-specific reset calls.
    The approved VISION-02 design remains the smallest principle-consistent approach: acquisition
    owners anchor one timestamp per stable vendor frame/result identity, core snapshots attach that
    timestamp once to geometry-only observations, and consumers derive age from the shared clock.
    Implementation is therefore proceeding under the existing approval with no material design
    change and remains bounded to VISION-02 only.
  - **Implementation (2026-07-23):** the portable AprilTag snapshot now has one
    `AprilTagDetections.fromFrame(LoopTimestamp, List)` construction path. Its geometry-only
    `AprilTagObservation.target(...)` family receives that exact timestamp on immutable copies;
    attached observations cannot be restamped, `none()` means no trustworthy processed frame, and
    a timestamped empty list means a trustworthy no-target frame. Freshness, tag selection,
    AprilTag pose estimation, spatial conversion, and target-observation conversion all derive age
    from or forward that retained timestamp without reconstructing it at consumption time.
  - **Acquisition-boundary implementation (2026-07-23):** one package-private constant-memory FTC
    timestamp anchor maps a stable vendor identity to a `LoopTimestamp` once. The webcam adapter
    keys processor generation plus acquisition nanoseconds and rejects mixed, invalid, future, and
    regressing frame timing. The Limelight adapter prefers the camera-local result timestamp,
    falls back to stable Control Hub receipt identity, estimates exposure time from receipt
    staleness plus capture/targeting latency, isolates pipeline generations, and distinguishes a
    confirmed no-target result from unavailable data. Both adapters reject retained identities
    after clock resets and use monotonic vendor-identity gates to reject older replay across
    repeated resets. `ResultSnapshot.frameTimestamp()` is the semantic timing fact; the renamed
    `resultReceivedAtControlHubMillis()` remains diagnostic transport metadata. The direct
    Limelight pose estimator now requires a positive freshness limit and applies it consistently.
  - **Student-facing simplicity:** ordinary webcam and Limelight robot code remains
    `visionLane.tagSensor()` followed by normal tag selection; it does not manage frame IDs,
    latencies, timestamps, or reset epochs. Only an advanced custom `AprilTagSensor` adapter names
    one acquisition timestamp once with `fromFrame(frameTimestamp, observations)`, instead of
    repeating timing on every observation. The Framework Principles, AprilTag/localization and
    spatial guides, custom-owner example, shooter walkthrough, Phoenix architecture, and public
    Javadocs now state that contract explicitly.
  - **Adversarial review corrections (2026-07-23):** three independent read-only reviews found and
    closed the remaining propagation-test, future-sample, primary Limelight identity, public
    Javadoc, custom-adapter wording, tracker-refinement, zero-freshness, and unbounded reset-history
    gaps. The final code audit reports no remaining P1/P2 issue.
  - **Automated verification (2026-07-23):** the seven focused timestamp suites report **57 tests,
    0 failures, 0 errors, 0 skipped**, covering portable frame/observation values, camera-mount and
    localization propagation, the shared anchor, both FTC camera backends, and spatial history.
    The complete `:TeamCode:testDebugUnitTest` suite reports **86 suites / 821 tests, 0 failures,
    0 errors, 0 skipped**, and `:TeamCode:compileDebugJavaWithJavac` succeeds. Exhaustive removed-API
    and caller scans are clean, changed Markdown fences and links are valid, and `git diff --check`
    passes. Gradle emits only the existing Java 8 source/target and FTC sample deprecation warnings.
  - **Hardware scope:** no camera hardware is needed to validate the deterministic identity,
    immutability, reset, propagation, or freshness contract. A real webcam and Limelight remain
    adoption validation for firmware behavior across no-target frames, device reboot, and pipeline
    changes, and for measuring the accuracy of the best-effort exposure-time estimate; this item
    does not claim that software tests measured physical capture time.
  - **Android Studio audit point (2026-07-23):** inspect the portable factories and exact timestamp
    propagation, the shared bounded anchor, webcam and Limelight identity/reset handling, the
    focused tests, and synchronized principle/guide wording. No files are staged, committed,
    pushed, or merged. Stop before another item until the user replies `VISION-02 looks good`.
  - **Manual verification (2026-07-23):** the user reviewed the implementation in Android Studio
    and replied `VISION-02 looks good`. VISION-02 is **Done**, and this approval authorizes Gate 3
    staging, publication, and merge for this item only; it does not authorize starting TARGET-03.

## Explicitly deferred architectural ideas

These are not implementation tasks without new evidence:

- Generic `BaseRobot` inheritance.
- Framework-owned shooter/intake/scoring capability families.
- A mandatory split into many Plant capability interfaces.
- A large framework module reorganization before focused tests exist.
- Setup/code-generation wizards before the compiling starter has been used and evaluated.
- A generic route host or season strategy layer based only on Phoenix's current needs.
- A monolithic autonomous path/action DSL, generic route-progress callback language, or wrapper over
  every Pedro feature. Keep Pedro geometry/constraints/callbacks at the path boundary and use Phoenix
  Tasks/capabilities for robot meaning.
- An Ivy/Phoenix command bridge or two schedulers in one OpMode; Phoenix Tasks remain the one robot
  behavior vocabulary unless a concrete integration cannot be expressed safely through a leaf
  adapter.
- Background coroutine/thread execution for FTC hardware behavior; cooperative Tasks remain owned by
  the OpMode heartbeat.
- Generic Task resource requirements, priorities, suspension, or arbitration based only on command-
  framework parity. Explicit composition and one visible drive owner remain simpler until a real
  Phoenix caller requires dynamic conflict resolution.
- Framework projectile physics, shooter/magazine sorting policy, or arbitrary generic vision-result
  maps. Robot services and typed snapshots already provide the correct ownership boundary.
- A framework-wide electrical power allocator or automatic motor-current response. SENSOR-01 may
  expose the missing measurement; each robot still owns its thresholds, subsystem priorities,
  derating, jam recovery, and calibration policy.
- A persistent/string-keyed global robot-state blackboard. MATCH-01 evaluates only a typed,
  short-lived Auto-to-TeleOp handoff with explicit fallback and clearing.
- A new articulated-camera localization lane based on one LOAD implementation. Reconsider only if a
  second robot cannot express its timestamped mount/measurement through the existing time-aware
  camera-mount and localization boundaries.
- A standalone simulator that duplicates a simplified robot. Reconsider simulation when a fake FTC
  hardware boundary can execute the production Phoenix composition, lifecycle, and Task graph.

## Recommended Codex workflow

The safest workflow is one Codex task and one branch per tracker item. This keeps each design review,
diff, verification result, and rollback boundary small.

1. Start with the first item in the recommended order that is neither **Done** nor **Deferred**, on
   its own `codex/` branch.
2. Ask Codex to perform the decision gate first and update this tracker. It should stop for direction
   if the chosen design materially differs from the leading hypothesis.
3. After the decision record is accepted, implement only that ID, update callers/docs/examples, run
   focused verification, and mark it **Verifying** in the same branch.
4. Review the Android Studio audit point. Only after the implementation is approved, mark the item
   **Done**, publish and merge it, then start the next item. Reorder the queue when evidence from a
   completed item changes later assumptions.
5. Use a separate task/branch for Phoenix-only fixes even when they were discovered during framework
   work.

Suggested prompt for each new Codex task:

> Implement `<ID>` from `FRAMEWORK_IMPROVEMENT_TRACKER.md`, and nothing else. First perform the
> documented decision gate: confirm the behavior and callers, compare the smallest alternatives
> against `Framework Principles.md`, and update the item's decision record. Preserve the simplest
> student-facing robot API. If the best design materially differs from the leading hypothesis, stop
> after the decision record and ask me before implementation. Otherwise implement it, update all
> affected Javadocs/guides/examples, run focused tests and compilation, mark only that item Verifying,
> and stop for my Android Studio review before publication.

For the current Codex conversation, the same process can be followed sequentially if branch-per-task
is inconvenient. The important constraints are one tracker ID at a time, a visible design decision
before code, and a review checkpoint before continuing.
