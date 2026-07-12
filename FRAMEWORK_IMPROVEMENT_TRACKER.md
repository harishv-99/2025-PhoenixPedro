# Framework Improvement Tracker

Last updated: 2026-07-12

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
| 10 | ROUTE-02 | Truthful route terminal status | Proposed | Do not infer route success solely from `!isBusy()`; preserve completion, interruption, timeout, and failure meaning. |
| 11 | ROUTE-01 | Start-time route construction | Proposed | Resolve a route once when its Task starts so live pose and vision can select geometry safely. |
| 12 | FIELD-01 | Explicit alliance field transforms | Proposed | Define field geometry once and apply a named, tested transform instead of duplicating red/blue path code. |
| 13 | DRIVE-01 | Drive task ownership | Ready | Keep an exclusive Auto-only timed sink Task, after Pedro's outer-loop ownership is made correct. |
| 14 | TASK-04 | Parallel deadline composition | Proposed | Add one cancellation-safe deadline primitive only if route-plus-companion callers justify it. |
| 15 | PHX-03 | Explicit Auto route-failure policy | Proposed | Keep generic sequence semantics and make continue/fallback/abort policy visible in the robot routine. |
| 16 | PHX-04 | Match-time fallback takeover | Proposed | Let one robot-owned Auto supervisor safely cancel any active phase and start a fresh park route. |
| 17 | PHX-02 | Phoenix runtime readiness | Proposed | Validate calibration, Pedro construction, routes, alliance facts, and required services before enabling assists/Auto. |
| 18 | MATCH-01 | Explicit Auto-to-TeleOp handoff | Proposed | Carry one typed immutable robot snapshot across the FTC mode boundary without string maps or stale globals. |
| 19 | DRIVE-02 | Shared drivetrain actuator handoff | Proposed | Preserve one motor writer when a PTO reuses drivetrain motors for an endgame mechanism. |
| 20 | VISION-01 | Custom VisionPortal ownership | Proposed | Reuse camera/processor lifecycle without forcing robot-specific detections through AprilTag APIs. |
| 21 | SENSOR-01 | Motor-current sensing | Proposed | Expose cycle-memoized current in amps as a Source; keep jam, homing, and power-budget policy robot-owned. |
| 22 | INPUT-01 | Safe contextual control activation | Proposed | Support optional control modes without turning held controls into phantom press edges. |
| 23 | HAPTIC-01 | Driver haptic feedback boundary | Proposed | Expose small rate-safe rumble output while controls retain the meaning of each notification. |
| 24 | PERF-01 | FTC hub bulk-cache ownership | Proposed | Evaluate one optional, cycle-idempotent manual bulk-cache heartbeat against SDK automatic caching. |
| 25 | PERF-02 | Loop phase diagnostics | Proposed | Measure named loop phases with a lightweight diagnostic that does not own timing or sleep. |
| 26 | PERF-03 | Contract-safe hardware write deduplication | Proposed | Add write caching only where measurements justify it and stop/Plant truth remain exact. |
| 27 | TUNE-01 | Live tuning to checked-in profile | Proposed | Keep production snapshots stable while providing an explicit, optional live-tuning workflow. |
| 28 | TARGET-01 | Lazy Plant target overlay selection | Proposed | Resolve the selected highest-priority layer first and avoid sampling shadowed layers. |
| 29 | TARGET-02 | Candidate freshness | Proposed | Compute effective age from the loop clock and timestamp, with validation. |
| 30 | TARGET-03 | Periodic planner complexity | Proposed | Replace range iteration with constant-time candidate mathematics. |
| 31 | CYCLE-01 | Stateful drive-source cycle safety | Proposed | Memoize stateful composition once per `clock.cycle()` and propagate reset deliberately. |
| 32 | CYCLE-02 | Localization cycle safety | Proposed | Guard predictors/estimators against duplicate same-cycle updates. |
| 33 | SOURCE-01 | Boolean composition sampling | Proposed | Sample both operands once per cycle before combining stateful results. |
| 34 | API-01 | Writable Plant command binding | Proposed | Keep one simple `Plant` if builder provenance can prevent silent no-op writes. |
| 35 | API-02 | Feedback tolerance choice | Proposed | Ask for tolerance after unit mapping; retain an explicitly named native default only if useful. |
| 36 | API-03 | Servo and owner-config validation | Proposed | Reject invalid configuration at build/owner construction with device-specific messages. |
| 37 | API-04 | Binding execution order | Proposed | Preserve declaration order unless explicit phases are proven necessary. |
| 38 | API-05 | One beginner drive entry point | Proposed | Teach the lane as the robot-facing path and keep the raw factory as a lower-level tool. |
| 39 | COMMON-01 | Initialization runtime helper | Proposed | Extract only repeated retry/error/cleanup ceremony; avoid a robot base class. |
| 40 | COMMON-02 | Telemetry commit ownership | Proposed | Renderers add data; the composition root commits once. |
| 41 | CHECK-01 | Staged whole-robot system check | Proposed | Compose a safe pre-match check from robot capabilities without hardware reflection or a base robot. |
| 42 | EXAMPLE-01 | Compiling modern starter robot | Proposed | Add a small multi-file reference, not an inheritance framework. |
| 43 | EXAMPLE-02 | Compiling Pedro autonomous reference | Proposed | Show one small real path, capability Tasks, one follower heartbeat, explicit fallback, and a thin OpMode. |
| 44 | EXAMPLE-03 | Advanced moving-target reference | Proposed | Prove progress-triggered scoring and a bounded moving turret without putting game physics in the framework. |
| 45 | BOUNDARY-01 | FTC boundary enforcement | Proposed | Fix existing import leaks, then add a focused forbidden-import check. |
| 46 | DOC-01 | Stale and non-compiling documentation | Proposed | Correct loop/API examples and validate links/examples where practical. |
| 47 | CI-01 | Framework verification in CI | Proposed | Run focused unit tests, TeamCode compilation, docs checks, and boundary checks. |
| 48 | CLEAN-01 | Alias and risky convenience cleanup | Proposed | Remove only APIs proven redundant or unsafe by caller search. |

The order is intentionally front-loaded with testability, robot lifecycle, actuator safety, and
deterministic Task behavior. The Pedro review added two runtime-ownership gates before DRIVE-01:
the checked-in Auto must first have one continuous follower heartbeat and one valid drivetrain/
localization authority. The Cuttlefish review then added route truth and deferred construction ahead
of the higher-level Auto policies that depend on them. The expanded Worlds-source review added one
mode-boundary handoff and one missing FTC sensor primitive, while deliberately leaving robot power,
jam, calibration, and match strategy out of the framework. Later API cleanup should not begin until
tests protect the core semantics it depends on.

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
| A global match-time rule can preempt whatever Auto phase is active and start a park route from the current pose ([Far Auto](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/Far.java)). | A route-local branch or deadline child does not by itself express a one-shot whole-routine takeover cleanly. | Add PHX-04 as robot-owned policy over the existing TaskRunner. |
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
  state; stale status from a prior route cannot complete a new one; PHX-03 branches on truthful data;
  and telemetry names the terminal reason without leaking Pedro types.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

### FIELD-01 - Explicit alliance field transforms

- **Problem to confirm:** Phoenix's path selector describes alliance mirroring, but the current path
  factory manually branches between red and blue coordinates. That duplicates field facts and lets
  headings, control points, and target poses drift apart as routes evolve.
- **External evidence:** Cuttlefish's
  [`FieldPose`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/auto/FieldPose.java)
  defines one pose and derives the alliance variant, demonstrating the student-facing benefit even
  though Phoenix should avoid its global color state and use Phoenix's documented frame.
- **Alternatives to compare:** continue explicit red/blue path definitions; use Pedro's color helper
  directly; store alliance in global mutable context; add robot-local coordinate helper methods; or
  add a small immutable Phoenix-coordinate field transform driven by explicit field facts.
- **Leading hypothesis:** define a tiny backend-neutral 2D pose/vector transform with explicit field
  dimensions/origin and no global alliance. Robot path factories select the transform, apply it to
  poses/headings/control points, and convert to Pedro only at the integration boundary. Do not add
  season route names or alliance strategy to the framework.
- **Completion:** tests cover pose, vector, heading, and control-point transformation; reflection
  boundaries and heading normalization; applying the alliance transform twice where mathematically
  applicable; explicit no-transform/default behavior; and Pedro boundary conversion. One route is
  defined once for both alliances in the compiling Auto reference.
- **Decision record:** _Pending._

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
    refreshed during the timed interval. `DriveCommandSink.drive(...)` defines its command as
    current-loop state and exposes `update(clock)` for per-loop housekeeping, so the helper does not
    satisfy the generic sink contract. The focused
    `TimedTaskStartSemanticsTest.publicDriveTaskCommandSurvivesItsSubLoopStartCycle` baseline passes
    one test with zero failures/errors/skips while explicitly asserting that the drive count stays
    at one; this confirms rather than protects against the defect.
  - **Loop and caller trace:** the documented TeleOp loop runs Tasks before one final
    `DriveSource -> DriveCommandSink` write. Phoenix and compiling examples 03 and 06 follow that
    order, so a direct Task write is overwritten later in the same cycle and its later `stop()` is
    overwritten too. Phoenix Auto has no competing final drive phase: its active Auto Task owns the
    drive resource. Existing `DriveGuidanceTask` and `RouteTask` already model that exclusive path by
    updating their sink/follower each active loop. `MecanumDrivebase` happens to latch immediate
    motor power and has a no-op update, which masks the bug when uncontested; the Pedro adapter needs
    `update(clock)` to advance its follower and receives no separate update from Phoenix Auto.
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
  - **Chosen ownership rule:** the exclusive helper is for an Auto-style runner or tester that is
    the only owner updating and commanding that sink while the Task is active. Each unexpired active
    cycle asks `sink.update(clock)` and then calls `sink.drive(signal)` exactly once. A stable sink
    lane that already owns a once-per-cycle heartbeat must make the same-cycle update idempotent;
    the Task-local call is not a substitute for an outer lifecycle owner such as Pedro requires. A
    positive duration publishes its command during the TaskRunner's start cycle; normal completion
    and active cancellation stop exactly once. A zero duration publishes no motion and stops
    immediately. TeleOp macros and assists remain `DriveSource`/overlay proposals consumed by the
    composition root's one final writer; they must not use the exclusive helper before that writer.
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
  - **Pedro dependency discovered by the re-review:** the current Phoenix adapter does not yet meet
    the lifecycle assumed by this generic design. Pedro requires an outer-loop Follower heartbeat,
    immediate stopped-state behavior, a valid drivetrain/localizer, and explicit coordinate/pose
    authority. Those are now tracked as PEDRO-01 and PEDRO-02 ahead of DRIVE-01. DRIVE-01 must not
    claim to fix Pedro hold-end, pose updates, or stopped output by itself.
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
    Pedro hold, heartbeat, cancellation, and stopped-output hardware checks belong in PEDRO-01 after
    PEDRO-02 supplies a valid runtime.
  - **Approval gate:** this confirms the tracker's leading ownership split, but it renames/removes
    public drive-task APIs and defines a new exclusive lifecycle contract. It is therefore a major
    API decision and requires explicit user approval before moving to **In progress**.

### TASK-04 - Parallel deadline composition

- **Problem to confirm:** Phoenix has `sequence(...)` and `parallelAll(...)`, but a realistic Auto
  often needs one route to determine phase completion while an intake, aiming preparation, or other
  companion behavior runs alongside it and is cancelled when the route ends. `parallelAll(...)`
  cannot express that ownership: a persistent companion prevents completion. The reviewed
  BettaFish Auto contains exactly this trap with Road Runner Actions that always return running.
  Cuttlefish provides two more concrete caller shapes: a route paired with a bounded asynchronous
  intake action, and a routine that is cancelled when the match-time park deadline wins.
- **Expanded Worlds evidence:** Tech Tigers uses a `ParallelRaceGroup` where transfer completion
  ends a phase and cancels perpetual shooter preparation; LOAD uses races mainly to express route
  timeouts, stall exits, and bounded mechanism work. The first is a true deadline caller. The latter
  should prefer truthful route timeout/status and bounded wait helpers, so it does not justify a
  symmetric arbitrary-race API.
- **Alternatives to compare:** require every companion to be finite; set/clear capability state
  before and after the route; use Pedro parametric callbacks; create robot-specific phase macros;
  add one `Tasks.parallelDeadline(primary, companions...)` primitive; or copy Ivy's full race,
  deadline, repeat, loop, requirements, and priority model.
- **Leading hypothesis:** persistent baseline mechanisms should remain source/capability state, not
  forever Tasks. If real Phoenix route-plus-finite-companion callers remain after that simplification,
  add one small deadline composition whose primary child controls completion and whose still-active
  companions receive normal active cancellation. Add symmetric race/repeat/requirements only after
  separate caller evidence; do not import Ivy or create a second scheduler.
- **Completion:** the common Auto phase is one readable factory call; start, same-cycle completion,
  primary success/timeout/cancellation, companion early completion, exactly-once companion cleanup,
  throwing/reentrant cancellation, duplicate identities, outcomes, and single-use behavior are
  specified and tested. Documentation distinguishes held capability state from bounded companion
  Tasks.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

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
  timeout; wrap the entire routine in TASK-04 and place park after it; add generic Task race/preempt
  primitives; or add one Phoenix Auto supervisor that owns the one-shot takeover over the existing
  `TaskRunner`. Compare how normal routine completion, Task cleanup failures, and safe mechanism
  requests are handled.
- **Leading hypothesis:** keep one scheduler and one `LoopClock`. A robot-owned Auto supervisor reads
  the shared match time, calls the TaskRunner's total cancel-and-clear path once, requests safe
  capability state, creates a fresh deferred park Task from the current pose, and starts it through
  the same runner. The time threshold, park choice, and mechanism policy remain routine/robot facts;
  no generic framework emergency strategy or hidden background timer is added.
- **Completion:** tests cover takeover during routes, companion work, mechanism waits, and idle/end
  state; exactly-once triggering at the threshold; active-child cancellation and cleanup failure;
  safe mechanism requests; truthful route failure; unavailable park geometry; repeated loop calls;
  and shutdown. Telemetry distinguishes normal completion, time takeover, fallback construction
  failure, and park outcome.
- **Decision record:** _Pending._

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
  construction, and an unsafe selected-start/live-pose mismatch cannot be armed as a match Auto.
- **Decision record:** _Pending._

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
  snapshot from the Auto composition root and lets TeleOp consume it or use an explicit configured
  fallback. It records enough producer/freshness metadata to reject stale or wrong-mode data and
  has an explicit clear operation. Keep hardware objects, Tasks, vendor poses, services, and season
  strategy out of the carrier; convert poses at the publishing boundary. Prefer a robot-named
  wrapper at the call site, and do not introduce a global robot-state map or event bus.
- **Completion:** tests cover publish/consume once, missing data, explicit fallback, wrong type,
  stale data, repeated Auto/TeleOp/test OpModes in one app process, explicit clear, process-local
  limitations, concurrent misuse, Auto cleanup failure, and defensive immutability. The Phoenix
  Auto and TeleOp each need one obvious handoff call, and ordinary robots that do not carry state
  gain no required setup.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

### VISION-01 - Custom VisionPortal ownership

- **Problem to confirm:** Phoenix's reusable FTC vision owners are AprilTag-specific. A robot that
  adds a color/cluster/game-piece `VisionProcessor` must repeat webcam/portal construction, processor
  enablement, streaming state, failure reporting, and close/retry behavior or incorrectly force
  robot-specific results through `AprilTagVisionLane`.
- **External evidence:** Cuttlefish's Far Auto creates and closes a custom cluster-detection portal,
  and its
  [`ClusterDetectionProcessor`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/ClusterDetectionProcessor.java)
  publishes detection facts that affect route construction. The lifecycle is reusable; the cluster
  vocabulary and OpenCV algorithm are not. Tech Tigers also shares one physical Limelight among
  object, motif, and localization pipelines. Its robot-specific pipeline meanings should not be
  generalized, but the decision gate must specify one-device ownership, enable/switch settling,
  readiness, and stale-result behavior rather than treating a requested pipeline as immediately
  usable.
- **Alternatives to compare:** keep every custom portal robot-owned; generalize the AprilTag lane;
  expose raw `VisionPortal` from a factory; create a small FTC portal resource owner around supplied
  processors; or define generic detection/result interfaces. Confirm whether existing `FtcVision`
  already removes enough ceremony before adding another public type.
- **Leading hypothesis:** if at least two real callers share the lifecycle, add one small `fw.ftc`
  resource owner for camera/processor construction, enable/disable state, readiness diagnostics,
  retry, and close. Algorithms and immutable timestamped result snapshots stay robot-owned and typed;
  framework routines never depend on `VisionProcessor`, OpenCV, or an arbitrary generic detection
  map.
- **Completion:** a fake or FTC-boundary test covers partial construction, processor enable/disable,
  streaming/readiness, retry, close after failure, repeated close, and stale-result policy. AprilTag
  code remains coherent, a compiling custom processor example has one clear owner, and no SDK/vendor
  type leaks into core Tasks or robot strategy.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

### INPUT-01 - Safe contextual control activation

- **Problem to confirm:** advanced TeleOp code sometimes changes the meaning of controls by robot
  mode. Simply enabling a new group while a button is held can manufacture a fresh press edge and
  trigger a dangerous action; recreating `Bindings` or scattering mode checks also makes ownership
  and precedence hard to audit.
- **External evidence:** Cuttlefish's
  [`LayeredGamepad`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/input/LayeredGamepad.java)
  primes newly activated controls from their current physical state so held buttons do not appear as
  new presses. Its TeleOp uses this for distinct normal, manual, and endgame mappings.
- **Alternatives to compare:** retain explicit mode guards in each control callback; construct one
  independent `Bindings` object per mode; add enable/disable to individual bindings; add a small
  ordered contextual group with rearm semantics; or build a general input-layer stack with
  priorities. Compare held buttons, analog neutral thresholds, precedence, and the flat common path.
- **Leading hypothesis:** keep today's flat declarations as the default. If repeated Phoenix callers
  justify extraction, allow an optional robot-owned binding group/context whose activation samples
  current controls and requires release/neutral before its edge bindings rearm. Declaration order
  stays visible; no global control-mode state, implicit priority stack, or automatic subsystem
  arbitration enters the framework.
- **Completion:** tests cover activation/deactivation while pressed, release/repress, analog neutral,
  toggle and hold bindings, multiple enabled contexts, same-cycle transitions, and cancellation of
  context-owned Tasks. A normal TeleOp remains unchanged, while an advanced example makes each
  mode's meanings and precedence obvious in one controls owner.
- **Decision record:** _Pending._

### HAPTIC-01 - Driver haptic feedback boundary

- **Problem to confirm:** Cuttlefish uses gamepad rumble for readiness, intake-full, relocalization,
  mode, warning, and endgame events. Phoenix controls can read through framework sources but have no
  corresponding narrow output seam, so robot code must retain raw FTC `Gamepad` references and
  repeatedly implement controller selection and cooldown policy.
- **External evidence:** Cuttlefish's
  [TeleOp haptic helpers](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tele/Tele.java)
  rate-limit rumble independently for each driver and use it across many meaningful robot events.
- **Alternatives to compare:** keep direct FTC calls in the OpMode; add rumble methods to
  `GamepadDevice`; define a tiny core-facing haptic sink with one FTC adapter; put rate limiting in
  every controls owner; or add a generic notification/event bus. Coordinate this with BOUNDARY-01 so
  fixing output does not preserve an existing SDK import leak.
- **Leading hypothesis:** expose one small haptic output contract with an FTC adapter and a reusable
  cooldown/debounce wrapper only if it shortens real callers. Robot controls/presenters retain the
  meaning, pattern choice, and recipient of each signal. Do not create a framework notification bus,
  background thread, or scoring-specific feedback API.
- **Completion:** tests cover per-controller cooldown, immediate safety warnings where selected,
  repeated events, disabled/unavailable rumble, stop, and loop-thread ownership. The common call site
  is one obvious feedback request and core robot policy has no FTC SDK dependency.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

### PERF-02 - Loop phase diagnostics

- **Problem to confirm:** Phoenix can report overall loop timing but cannot identify whether input,
  localization, services, Tasks, Plant realization, vision, or telemetry owns a slow cycle. Students
  therefore troubleshoot performance with ad hoc timers or by commenting out behavior.
- **External evidence:** Cuttlefish's
  [`LoopProfiler`](https://github.com/6165-MSET-Cuttlefish/Decode/blob/1a9ff399298a95639c08daf0434463d9b035d383/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LoopProfiler.java)
  records named segment durations and surfaces bottlenecks during robot development. SaMoTech's
  [`TaskTimer`](https://github.com/SaMoTechRobotics/FTC-Decode/blob/c8ff047c4245f79d934e622c05481269b5cc42da/TeamCode/src/main/java/util/TaskTimer.java)
  independently times named chassis, shooter, turret, indicator, and drawing phases, while its Auto
  records expected versus actual semantic-step duration. This supports observation and snapshots,
  not another scheduler or clock.
- **Alternatives to compare:** document manual timing; add local timers only to Phoenix; create a
  lightweight named-phase diagnostic; integrate with telemetry/tracing libraries; or let a profiler
  own the loop clock and rate. Measure allocation and telemetry overhead before choosing defaults.
- **Leading hypothesis:** an optional diagnostic may observe a supplied monotonic time source and
  aggregate named phase timings, but it never advances `LoopClock`, sleeps, schedules work, or hides
  loop order. The composition root marks only useful high-level boundaries, and presenters decide
  when to render a snapshot.
- **Completion:** tests cover phase order, repeated names, nested/unfinished phases, cycle rollover,
  bounded allocation/history, disabled overhead, and immutable snapshots. A Phoenix diagnostic
  example identifies a simulated slow phase without changing behavior or the one-clock contract.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

### TUNE-01 - Live tuning to checked-in profile

- **Problem to confirm:** competition robots benefit from live Dashboard tuning, but Phoenix profiles
  are intentionally data-only and defensively copied by long-lived owners. Directly reading mutable
  static `@Config` fields throughout production would make behavior change mid-cycle, bypass staged
  validation, and leave the tuned values outside version-controlled robot configuration.
- **External evidence:** Cuttlefish exposes many mechanism, control, and Auto parameters through FTC
  Dashboard configuration during development. Its use demonstrates the speed benefit while also
  showing why mutable globals should not become Phoenix's production configuration authority.
- **Alternatives to compare:** edit/redeploy checked-in profiles only; let production owners read
  mutable Dashboard statics continuously; rebuild individual owners from a tuning snapshot; provide
  dedicated calibration/tester bridges; or emit/copy validated values into the profile. Consider
  which parameter changes are safe while hardware is active.
- **Leading hypothesis:** keep production Phoenix owners on immutable defensive snapshots. An
  optional FTC/Dashboard tuning owner is used only by explicit tuning/tester modes, validates a
  complete candidate, applies it at a visible rebuild boundary or through an already-safe calibrated
  source, and reports values in a form that can be copied into the checked-in profile. No hidden
  mutable global becomes a match dependency.
- **Completion:** tests cover invalid/non-finite edits, atomic snapshot/apply, active-output safety,
  rollback, rebuild/close ordering, and emitted profile values. Documentation gives students one
  repeatable tune -> validate -> record -> production workflow, and match OpModes fail readiness if
  they depend on unrecorded live state.
- **Decision record:** _Pending._

### TARGET-01 - Lazy Plant target overlay selection

- **Problem to confirm:** resolving shadowed layers can trigger stateful capture such as
  `holdMeasuredTargetOnEntry()` even when that layer is not selected.
- **Alternatives to compare:** top-down lazy resolution; explicit selection lifecycle hooks; or a new
  selected-only capture source.
- **Leading hypothesis:** lazy highest-priority-first resolution best matches overlay semantics and
  avoids hidden side effects without exposing lifecycle complexity to robot code.
- **Completion:** shadowed sources are not sampled; fallback/add-if-available behavior stays explicit;
  re-entry behavior is covered by tests and documentation.
- **Decision record:** _Pending._

### TARGET-02 - Candidate freshness

- **Problem to confirm:** a cached candidate with a fixed `ageSec` can remain fresh forever even when
  it also supplies an old timestamp.
- **Alternatives to compare:** compute age entirely from timestamp; use the maximum of reported and
  clock-derived age; or require one canonical freshness representation.
- **Leading hypothesis:** validate quality/age and compute a conservative effective age while keeping
  compatibility with candidates that cannot provide timestamps.
- **Completion:** stale cached candidates fail their gates; invalid quality, age, and timestamp values
  fail early or follow one documented unavailable policy.
- **Decision record:** _Pending._

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

### API-03 - Servo and owner-config validation

- **Problem to confirm:** invalid servo endpoints, transformed values, non-finite limits, or negative
  magnitude settings may survive construction and be silently clamped or misapplied later.
- **Alternatives to compare:** builder-stage validation, owner constructor validation, constrained
  config value objects, and hardware-boundary defense.
- **Leading hypothesis:** validate once at the earliest fully informed builder/owner boundary and keep
  cheap final hardware defense; avoid new public value types unless several APIs share the invariant.
- **Completion:** errors name the device, setting, units, supplied value, and legal range.
- **Decision record:** _Pending._

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

### COMMON-01 - Initialization runtime helper

- **Problem to confirm:** Phoenix Auto and framework testers repeat retry, error reporting, partial
  construction cleanup, and initialization state.
- **Alternatives to compare:** a small generic init/cleanup owner; local shared private helpers; an
  FTC-specific lane; or no extraction until another production robot needs it.
- **Leading hypothesis:** extract only the lifecycle state machine and cleanup guarantees already
  repeated in at least three callers. Do not include robot capabilities, configuration policy, or
  OpMode inheritance.
- **Completion:** callers become shorter without hiding loop order or resource ownership; partial
  failure and repeated cleanup are tested.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

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
  machinery. The Phoenix example should achieve the short surface with the ordinary framework
  vocabulary rather than teaching a generated plan language or fixed-cycle state builder.
- **Alternatives to compare:** rely on Pedro's Ivy example; expand Markdown snippets only; make the
  full Phoenix Decode Auto the starter; add a code generator/base OpMode; or add one small compiling
  Pedro-specific robot-side example after PEDRO-01/02 establish the correct integration contract.
- **Leading hypothesis:** add a compact multi-file example and guide, not a new Auto DSL or base
  robot. It should show one configured follower/integration owner, a tiny immutable path set, one
  reusable mechanism capability Task, a readable routine composition, explicit route fallback, a
  thin OpMode, one LoopClock/follower heartbeat, and deterministic cancellation. Keep Pedro types at
  the integration/path boundary and use Phoenix Tasks rather than teaching Ivy beside them.
- **Completion:** the example compiles against the pinned Pedro version, runs in a pure/fake adapter
  test where practical, contains no placeholder hardware config, and documents the exact files a
  student normally edits for a new alliance/path/routine. Android Studio and on-robot walkthroughs
  confirm the normal programming path is shorter and clearer than copying the full Phoenix season
  graph. After their prerequisite items land, an advanced companion section demonstrates one
  alliance transform, a route supplier resolved at start, one Pedro progress callback that requests
  a robot capability, deadline-bounded intake, truthful route branching, vision fallback, and a
  match-time park takeover without introducing a second scheduler or Auto DSL.
- **Decision record:** _Pending._

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
- **Decision record:** _Pending._

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

1. Start with the next incomplete item in the recommended order, currently `PEDRO-01`, on its own
   `codex/` branch.
2. Ask Codex to perform the decision gate first and update this tracker. It should stop for direction
   if the chosen design materially differs from the leading hypothesis.
3. After the decision record is accepted, implement only that ID, update callers/docs/examples, run
   focused verification, and mark it Done in the same branch.
4. Review and merge that change before starting the next item. Reorder the queue when evidence from a
   completed item changes later assumptions.
5. Use a separate task/branch for Phoenix-only fixes even when they were discovered during framework
   work.

Suggested prompt for each new Codex task:

> Implement `<ID>` from `FRAMEWORK_IMPROVEMENT_TRACKER.md`, and nothing else. First perform the
> documented decision gate: confirm the behavior and callers, compare the smallest alternatives
> against `Framework Principles.md`, and update the item's decision record. Preserve the simplest
> student-facing robot API. If the best design materially differs from the leading hypothesis, stop
> after the decision record and ask me before implementation. Otherwise implement it, update all
> affected Javadocs/guides/examples, run focused tests and compilation, and mark only that item Done.

For the current Codex conversation, the same process can be followed sequentially if branch-per-task
is inconvenient. The important constraints are one tracker ID at a time, a visible design decision
before code, and a review checkpoint before continuing.
