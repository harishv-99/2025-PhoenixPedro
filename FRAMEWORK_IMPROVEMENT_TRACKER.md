# Framework Improvement Tracker

Last updated: 2026-07-10

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
| 1 | TEST-01 | Pure framework test harness | Proposed | Establish fake-clock and pure unit-test support before behavior changes. |
| 2 | PHX-01 | Phoenix shutdown ordering | Proposed | Make shutdown one idempotent, correctly ordered owner operation. |
| 3 | SAFE-01 | Final Plant target invariant | Proposed | Validate finite/range-safe output after all dynamic guards. |
| 4 | SAFE-02 | Power Plant applied-target truth | Proposed | Clamp normalized power in the Plant and report the clamp. |
| 5 | TASK-01 | Timed-task start semantics | Proposed | Measure elapsed time from task start, not the preceding loop interval. |
| 6 | TASK-02 | Task reuse contract | Proposed | Enforce the documented single-use contract consistently. |
| 7 | TASK-03 | Clear and cancellation semantics | Proposed | Remove unsafe drop-without-cancel behavior and make Plant cancellation explicit. |
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
- **Decision record:** _Pending._

### PHX-01 - Phoenix shutdown ordering

- **Problem to confirm:** shared shutdown clears `scoringPath` before the later general stop can stop
  it, potentially leaving owned behavior active.
- **Decision question:** should Phoenix expose one idempotent `stop()`/`close()` operation, or can the
  existing mode-specific methods be made safe without duplicating ordering?
- **Leading hypothesis:** one idempotent owner-level shutdown is simpler and harder for thin OpModes
  to call incorrectly.
- **Completion:** every initialized resource is stopped exactly once for TeleOp, Auto, partial init,
  repeated stop, and init failure.
- **Decision record:** _Pending._

### SAFE-01 - Final Plant target invariant

- **Problem to confirm:** static clamping happens before dynamic guards, so a guard fallback may
  become the actual hardware command without final finite/range validation.
- **Alternatives to compare:** final re-sanitize/re-clamp; validate fallback only at construction;
  make guard output a constrained value type; or both build-time validation and final defense.
- **Leading hypothesis:** validate static fallback configuration early and retain a cheap final
  Plant-level finite/range check as defense in depth.
- **Completion:** no Plant command can leave its declared legal range or become non-finite after any
  guard; requested/applied status explains the result.
- **Decision record:** _Pending._

### SAFE-02 - Power Plant applied-target truth

- **Problem to confirm:** a Power Plant can report an applied value outside `[-1, 1]` while the FTC
  adapter silently sends a different clamped value.
- **Alternatives to compare:** Plant-level normalized range invariant; output-level status feedback;
  builder-injected range; or rejecting out-of-range commands.
- **Leading hypothesis:** normalized power is a semantic Plant constraint, so clamp and report it at
  the Plant level while retaining hardware-boundary defense.
- **Completion:** `getAppliedTarget()` equals the command received by the output for finite values,
  with explicit status for clamping and safe behavior for NaN/infinity.
- **Decision record:** _Pending._

### TASK-01 - Timed-task start semantics

- **Problem to confirm:** tasks started by `TaskRunner.update()` immediately consume the loop's
  pre-start `dtSec`, allowing a short positive-duration output to finish before it is observed.
- **Alternatives to compare:** absolute `startSec`; skip elapsed time on the start cycle; runner-level
  start-phase separation; or an explicit minimum one-observation contract.
- **Leading hypothesis:** absolute elapsed time from `clock.nowSec()` is easiest to reason about, but
  output tasks must additionally guarantee a positive-duration command is observable.
- **Completion:** focused tests cover zero duration, duration below one loop, normal duration, large
  first-loop `dt`, and duplicate same-cycle updates.
- **Decision record:** _Pending._

### TASK-02 - Task reuse contract

- **Problem to confirm:** built-in Tasks disagree about whether a second `start()` restarts, no-ops,
  or partially reuses children despite documentation declaring Tasks single-use.
- **Alternatives to compare:** enforce single-use with actionable exceptions; make every task safely
  restartable; or move repeated behavior to `Supplier<Task>` factories.
- **Leading hypothesis:** single-use plus factories is the clearest lifecycle and avoids retaining
  partially consumed task graphs.
- **Completion:** all built-ins and composites follow one documented rule; repeated start/enqueue has
  deterministic tests and a useful error.
- **Decision record:** _Pending._

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
