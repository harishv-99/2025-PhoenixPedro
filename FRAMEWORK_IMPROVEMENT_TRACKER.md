# Framework Improvement Tracker

Last updated: 2026-07-11

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
| 8 | DRIVE-01 | Drive task ownership | Proposed | Separate exclusive Auto sink ownership from TeleOp drive-source proposals. |
| 9 | TARGET-01 | Lazy Plant target overlay selection | Proposed | Resolve the selected highest-priority layer first and avoid sampling shadowed layers. |
| 10 | TARGET-02 | Candidate freshness | Proposed | Compute effective age from the loop clock and timestamp, with validation. |
| 11 | TARGET-03 | Periodic planner complexity | Proposed | Replace range iteration with constant-time candidate mathematics. |
| 12 | CYCLE-01 | Stateful drive-source cycle safety | Proposed | Memoize stateful composition once per `clock.cycle()` and propagate reset deliberately. |
| 13 | CYCLE-02 | Localization cycle safety | Proposed | Guard predictors/estimators against duplicate same-cycle updates. |
| 14 | SOURCE-01 | Boolean composition sampling | Proposed | Sample both operands once per cycle before combining stateful results. |
| 15 | API-01 | Writable Plant command binding | Proposed | Keep one simple `Plant` if builder provenance can prevent silent no-op writes. |
| 16 | API-02 | Feedback tolerance choice | Proposed | Ask for tolerance after unit mapping; retain an explicitly named native default only if useful. |
| 17 | API-03 | Servo and owner-config validation | Proposed | Reject invalid configuration at build/owner construction with device-specific messages. |
| 18 | API-04 | Binding execution order | Proposed | Preserve declaration order unless explicit phases are proven necessary. |
| 19 | API-05 | One beginner drive entry point | Proposed | Teach the lane as the robot-facing path and keep the raw factory as a lower-level tool. |
| 20 | COMMON-01 | Initialization runtime helper | Proposed | Extract only repeated retry/error/cleanup ceremony; avoid a robot base class. |
| 21 | COMMON-02 | Telemetry commit ownership | Proposed | Renderers add data; the composition root commits once. |
| 22 | EXAMPLE-01 | Compiling modern starter robot | Proposed | Add a small multi-file reference, not an inheritance framework. |
| 23 | BOUNDARY-01 | FTC boundary enforcement | Proposed | Fix existing import leaks, then add a focused forbidden-import check. |
| 24 | DOC-01 | Stale and non-compiling documentation | Proposed | Correct loop/API examples and validate links/examples where practical. |
| 25 | CI-01 | Framework verification in CI | Proposed | Run focused unit tests, TeamCode compilation, docs checks, and boundary checks. |
| 26 | CLEAN-01 | Alias and risky convenience cleanup | Proposed | Remove only APIs proven redundant or unsafe by caller search. |
| 27 | PHX-02 | Phoenix runtime readiness | Proposed | Validate calibration, routes, alliance facts, and required services before enabling assists/Auto. |

The order is intentionally front-loaded with testability, a known robot lifecycle defect, actuator
safety, and deterministic task behavior. Later API cleanup should not begin until tests protect the
core semantics it depends on.

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

### DRIVE-01 - Drive task ownership

- **Problem to confirm:** timed drive tasks write only on start even though the sink command is
  current-loop state and the normal drive pipeline may overwrite it immediately.
- **Alternatives to compare:** reassert direct sink ownership every Auto loop; represent all drive
  tasks as sources; create a drive-output runner; or delete ambiguous helpers.
- **Leading hypothesis:** provide an explicitly exclusive Auto task that refreshes the sink, while
  TeleOp macros remain source/overlay proposals consumed by the single final writer.
- **Completion:** ownership and loop order are obvious at call sites; watchdog-style sinks receive a
  command each required loop; TeleOp retains one final drive writer.
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

### PHX-02 - Phoenix runtime readiness

- **Problem to confirm:** placeholder routes, uncalibrated localization values, missing alliance tag
  facts, or unavailable services may still allow assists/Auto to initialize.
- **Alternatives to compare:** robot-owned validation report; scattered constructor checks; disabling
  individual features; or framework-generic validation primitives.
- **Leading hypothesis:** Phoenix owns the actual readiness rules. A tiny generic validation report is
  worth extracting only if another framework caller needs the same aggregation behavior.
- **Completion:** unsafe modes fail before start with actionable telemetry; valid partial-feature
  configurations remain possible when explicitly selected.
- **Decision record:** _Pending._

## Explicitly deferred architectural ideas

These are not implementation tasks without new evidence:

- Generic `BaseRobot` inheritance.
- Framework-owned shooter/intake/scoring capability families.
- A mandatory split into many Plant capability interfaces.
- A large framework module reorganization before focused tests exist.
- Setup/code-generation wizards before the compiling starter has been used and evaluated.
- A generic route host or season strategy layer based only on Phoenix's current needs.

## Recommended Codex workflow

The safest workflow is one Codex task and one branch per tracker item. This keeps each design review,
diff, verification result, and rollback boundary small.

1. Start with `TEST-01` on a branch such as `codex/test-01-framework-harness`.
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
