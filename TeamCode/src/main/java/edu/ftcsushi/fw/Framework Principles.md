---
tags:
  - Advanced
---

# Framework Principles

Sushi optimizes for **student simplicity without hiding truth**. Code should make the safe,
ordinary choice obvious, and names, ownership, lifecycle, documentation, and errors should all tell
the same story.

This document is the design authority for the framework. It states the small set of principles that
new APIs and robot designs must preserve. Detailed recipes belong in the
[`documentation hub`](<docs/README.md>) and exact method behavior belongs in Javadocs.

## The six principles

| Principle | Question it answers |
| --- | --- |
| **One obvious path** | What should an ordinary robot author do? |
| **One owner** | Who is responsible for this state, resource, or decision? |
| **One heartbeat** | When does work advance, and in what order? |
| **One realization path** | How does intent become the final hardware command? |
| **Truthful boundaries** | What is actually known, valid, or complete here? |
| **One current, proven story** | Can a student learn and trust the supported design? |

These principles are deliberately related. One obvious path is easier when every resource has one
owner. One owner is easier to reason about with one heartbeat. One realization path makes hardware
effects visible. Truthful boundaries keep simplicity from becoming hidden behavior. Current docs,
examples, and tests prove that the story is real.

---

## 1. One obvious path

An ordinary problem gets one ordinary solution. Advanced seams may exist for genuinely different
hosts or integrations, but they must be named and documented as exceptions rather than presented as
parallel beginner choices.

### Implications

- Ordinary FTC robot code uses `FtcRobotOpMode` and declares one framework-created `RobotProgram` in
  `configure(program)`. The program owns lifecycle callbacks and loop phases. A custom/manual host is
  an advanced seam for a materially different lifecycle, not a second robot recipe.
- A controls constructor establishes stable input sources without registering behavior. Ordinary
  controls declare their meanings exactly once through an explicit
  `bind(program.callbackBindings(), capability)` call, with the callback surface first and the
  capability it drives second.
- Use `program.callbackBindings()` for synchronous callbacks and `program.taskBindings()` for fresh
  queued Tasks. Their parallel names expose the different downstream behavior.
- Prefer one public construction layer. Remove overlapping legacy paths when a breaking change makes
  the framework clearer; update every in-repository caller, example, and guide together.
- Put FTC hardware construction behind the FTC boundary. Ordinary mechanisms construct private
  Plants through `FtcActuators.plant(hardwareMap)`. The hardware-neutral
  `Plants.fromOutputs()` seam is for tests, portable hosts, and custom adapters. Both expose the
  same conceptual Plant grammar; the separate entry points represent different available evidence,
  not two competing designs.
- Adapt ordinary values with `Source.of(...)`, primitive clockless leaves with
  `ScalarSource.of(...)` or `BooleanSource.of(...)`, and constants with `Source.constant(...)`.
  Direct `Source` implementation is reserved for a named framework or integration abstraction that
  adds a real domain contract.
- Prefer framework factories such as `Tasks`, `ScalarTasks`, `SemanticScalarTasks`, `DriveTasks`,
  route helpers, and output task helpers. Write a custom state machine only when the factories
  cannot express the required lifecycle.
- Staged builders ask each conceptual question once and use types to prevent invalid combinations.
  Use a `done...()` step only when it closes a branch containing several related settings. Optional
  tuning must never conceal a required safety or cancellation decision.
- Parallel concepts use parallel nouns, verbs, prefixes, and argument order. A reader should infer a
  relationship from names without memorizing exceptions.
- Model one domain as one coherent family: use the same ordered vocabulary, units, equations,
  construction order, and lifecycle semantics; name a difference only when ownership or available
  evidence truly differs.
- Configuration follows one vocabulary:
  - `Owner.Config.defaults()` is a valid software baseline, never a claim of reviewed physical safety.
  - `identity()` names an identity transform; `zero()` names zero geometry or value.
  - immutable-style variations use `withX(...)` and `withoutX()`.
  - ordinary features use positive `enableX`; rare explicit permissions may use `allowX`.
  - keep an owner's configuration constructor private when callers should start from its named
    factories rather than assemble partially initialized state.
  - long-lived owners defensively snapshot configuration. Live tuning is a separate, explicit
    workflow, not mutation of retained configuration.

Simplicity is not the fewest possible classes or characters. It is the fewest concepts a student
must keep in mind to make the correct choice.

---

## 2. One owner

Every mutable state, lifecycle, hardware resource, policy decision, and final write has one clear
owner. Other objects request, observe, or compose; they do not quietly become peer owners.

### Robot ownership

- The composition root constructs objects and makes update order explicit. It does not contain the
  robot's control script.
- Capability families expose mode-neutral robot intent and status. TeleOp and Auto are parallel
  clients of that vocabulary.
- Controls own operator meanings. They call capabilities rather than reach into paths, services, or
  raw Plants.
- Mechanisms and subsystems own their private Plants, final target resolvers, Plant update order,
  and stop behavior.
- Supervisors and services own robot-specific policy and coordination upstream of mechanisms.
- Presenters observe snapshots and add rows. They do not make behavior decisions, clear telemetry,
  or commit a shared telemetry frame.
- Profiles and configuration objects are data-only. Long-lived owners copy what they retain.

An ordinary mechanism constructor receives `HardwareMap` plus data-only configuration, constructs
and privately owns its complete resolver/Plant graph, and owns update and stop. The composition root
constructs the mechanism; it does not prebuild and inject peer Plant and target dependencies. A
completed-Plant constructor is an explicitly labeled hardware-neutral test, custom-adapter,
portable-host, or advanced-assembly seam and receives the Plant alone.

An exclusive diagnostic or tuning OpMode may give a framework workflow factory one function that
creates a **fresh** Plant from the same canonical private recipe used by production. This is an
explicitly advanced assembly seam, not another ordinary mechanism constructor or command path. The
workflow becomes the returned Plant's sole heartbeat and lifecycle owner; production and diagnostic
Plant instances are never shared. Prefer a framework-owned workflow factory when it can own the
draft, validation, evidence, and cleanup so robot code declares only its device, safe test range,
and canonical Plant recipe.

For an ordinary exact mechanism, retain the Plant and use its stable, side-effect-free
`commandTarget()` when creating a command or Task. Keep a separate target only when it has an
independent shared, composed-graph, or target-only policy role. Feedback-aware `ScalarTasks` names
both target and Plant because the target receives the request while the Plant supplies completion
feedback and provenance. A read-only or planned realization requires no command: retain its Plant
and do not manufacture a command target that the graph does not own.

When a capability's public request has named semantic meaning such as `Height`, `Mode`, or a
semantic pose, one mechanism owner maps that value forward to the numeric Plant command. Every
direct control and Task path publishes through that same command owner; no raw numeric Task may
bypass it, and status must not reverse-infer the semantic value from a double. Use one
`SemanticScalarCommand<S>` to validate and publish the named value with its mapped finite scalar,
using `forEnum(initial).map(...).build()` when a fixed enum table should fail fast on incomplete
coverage and `create(initial, mapper)` when the mapping is computed or the semantic type is not an
enum. Bind an exact owner directly with the Plant builder's `targetExactlyFrom(command)` answer;
use `PlantTargets.equivalentPositionsOf(command)` explicitly when a periodic position should select
an interchangeable physical representative. Its composed
semantic/Plant snapshot invalidates prior arrival evidence synchronously, including when a new
request repeats the same name and scalar. The semantic owner deliberately exposes no
`ScalarTarget`; use `ScalarTasks` directly only when the scalar is the complete capability request
rather than one representation of richer named intent. Use `SemanticScalarTasks` for immediate,
timed, or feedback-aware deferred semantic requests; it publishes through that same command owner
and names the feedback Plant explicitly when completion depends on one observer's evidence.

Use `Plant.snapshot()` for the common immutable requested/applied target, resolution/status,
feedback, measurement, error, and arrival facts; a `PositionPlant` covariantly adds its coordinate,
range, and reference facts. This is a capture of cached public state, not another update, hardware
poll, atomic publication, or cross-thread synchronization contract. Numeric power and velocity
mechanisms use the same scalar snapshot—do not manufacture a `VelocityPlant` merely to rename the
same evidence. A grouped device-managed Plant's snapshot preserves its inverse-mapped aggregate
mean; a grouped regulated Plant's snapshot preserves the one scalar feedback source explicitly
configured for that Plant. Both preserve the Plant's own `atTarget` contract. Per-member balance,
settling, debounce, game-piece, and readiness policy remain in a capability-owned status that
composes the generic Plant facts.

When generic snapshot navigation would leak framework vocabulary into ordinary capability clients,
return a thin capability-owned status view. That view holds one immutable generic snapshot and
delegates domain-named accessors; it does not maintain a parallel cached status, mirror fields in
`update()` or `stop()`, or offer primitive construction that can fabricate an incoherent semantic
and numeric pair. It may expose the underlying Plant snapshot as an explicitly advanced diagnostic
escape hatch.

### Boundary ownership

- The reusable framework core is the part of `edu.ftcsushi.fw` outside the explicit
  `fw.ftc`, `fw.integrations`, and `fw.tools` edges. It depends only on core Sushi contracts; it
  does not import FTC/Android/vendor types, those edge packages, or robot application packages.
- FTC SDK and Android adapters live under `edu.ftcsushi.fw.ftc`; narrow third-party bridges live
  under `edu.ftcsushi.fw.integrations`; FTC-bound examples, testers, and calibration hosts live
  under `edu.ftcsushi.fw.tools`. `edu.ftcsushi.robots` is the application edge that composes
  those adapters with core capabilities and season-specific policy.
- Dependencies point from an explicit edge into the reusable core, never from the core back out to
  an edge. A production-source boundary test enforces that direction. Test sources may use SDK
  stubs or fakes to prove an edge adapter without making those types a production core dependency.
- An abstraction exposes the smallest capability it can truthfully promise. Do not invent a
  universal interface for devices whose lifecycle or evidence is fundamentally different.
- Introduce a framework lane only when a recurring, stable multi-object resource graph has a shared
  lifetime, a concrete lifecycle hazard, meaningful configuration differences, a narrow boundary,
  simpler adopters, and a testable contract. Do not add a generic lane registry or `updateAll()`.
- Field facts, sensor ownership, and game strategy remain separate. Vision backends may differ while
  robot consumers depend on a small semantic, backend-neutral lane.

Clear ownership is the practical form of SOLID used by Sushi: cohesive objects, narrow
interfaces, explicit dependencies, and no mechanical fragmentation that makes robot code harder.

---

## 3. One heartbeat

One `LoopClock` defines time and cycle identity. The managed host advances it exactly once per FTC
cycle; every other component reads it. Nothing sleeps, busy-waits, or runs a private competing loop.

### Managed lifecycle

The ordinary active order is:

```text
Clock -> Services -> Bindings -> Tasks -> Outputs/Drive -> Presenters -> one telemetry commit
```

- INIT advances the clock, runs an optional data-only `Prestart`, and presents status without
  actuating hardware.
- START freezes `Prestart` exactly once, resets the same clock, starts services, starts and first
  updates the optional root Task, then realizes outputs once. Every positive-duration request is
  therefore observable by the downstream phase before it can expire.
- BLOCKED keeps services, bindings, Tasks, and outputs inert while clock and presenters may continue.
- STOP or a runtime failure best-effort cancels active work, clears pending work, stops owned outputs
  and resources, and prevents further ordinary commands. A handoff may publish cached state only
  after complete normal ACTIVE-stop cleanup; every other path invalidates it.
- A `Prestart` owns selection data, not hardware/resource lifecycle or a stop hook. Use
  `Tasks.buildAtStart(...)` when a Task depends on frozen selection; do not defer hardware graph
  construction through it.

### Per-cycle behavior

- A stateful value source publishes at most one successful observation for one `clock.cycle()`.
  It commits its cache only after the complete operation succeeds, so a same-cycle read may retry
  after an exception. User-supplied mapping functions, predicates, and reducers are side-effect-free
  value operations; stateful decorators own and document their transactional state.
- An effectful update owner claims the cycle before polling hardware, writing vendor state, or
  advancing an irreversible controller. A repeat after success is a no-op; a repeat after failure
  rethrows the retained failure instead of replaying an uncertain effect. Reentrant update is an
  error.
- Bindings observe control contexts and registrations in documented declaration order. This is
  deterministic sequencing, not priority or command arbitration; one robot owner still composes
  competing intent.
- Reset changes cycle identity and clears state according to ownership. It does not invent resets
  for borrowed collaborators or allow pre-reset cached observations to masquerade as current.

### Tasks and time

- Every `Task` instance is single-use and records its start attempt before starting a child,
  invoking a controller, or producing a hardware side effect. Repetition comes from a macro method,
  `Supplier<Task>`, `OutputTaskFactory`, or bounded `Tasks.repeatWhileSuccessful(...)` composition;
  every path creates fresh child identities, and compositions reject obvious identity reuse.
- Bounded successful repetition samples admission before each proposed child, including the first,
  and starts at most one fresh child per shared clock cycle. Only an exact successful child permits
  another admission decision; a false decision or the configured limit completes successfully,
  while any other valid terminal child outcome remains exact. Admission is a soft start policy and
  never interrupts an active child.
- A timed Task or timed phase starts at its own `clock.nowSec()` boundary. It never consumes the
  `dtSec()` interval from before it started.
- A timed scalar Task owns its request for the interval. It reasserts a superseded numeric request;
  a semantic timed Task retains one exact request identity while uncontested and publishes a fresh
  occurrence when reclaiming ownership, never an old identity with stale arrival evidence.
- Cancellation before start has no effect. Active cancellation is terminal and idempotent;
  terminal or repeated cancellation does nothing. `Tasks.noop()` is the intentional
  terminal-at-construction exception.
- Framework Tasks reject a direct `update(clock)` before their first `start(clock)`. A runner may
  update while idle and starts its queued head during that update. Its total abort cancels the
  current Task, discards pending Tasks, clears its state, and never exposes a drop-without-cancel
  queue operation.
- Composition states lifetime and outcome policy explicitly. `sequence(...)` starts a later child
  only after exact `SUCCESS`; `sequenceOnCompletion(...)` is the exceptional recovery/repair form
  that may continue after any valid natural terminal outcome while retaining the first non-success
  result. Direct cancellation never runs a later child. `parallelAll(...)` waits for every child
  and succeeds only when every child succeeds; matching abnormal outcomes remain exact and mixed
  abnormal outcomes become `UNKNOWN`. `parallelDeadline(deadline, companions...)` lets the
  deadline own group lifetime and cancels started companions. Mandatory cleanup belongs in the
  active owner's cancellation behavior or persistent capability state.
- A timeout reports an outcome; it does not silently choose recovery. When a required continuation
  must take over at a reserved elapsed boundary, put every preceding Task inside `withTimeout(...)`
  and use `sequenceOnCompletion(...)` for the continuation outside that timeout. The direct timed
  child must become terminal through its cooperative cancellation path before the continuation
  starts, and every nested Task is required to honor active-cancellation terminality. A propagated
  lifecycle/cleanup failure, direct cancellation of the outer sequence/root, or STOP never launches
  the continuation. This is not `finally`; branch on retained results when success and failure must
  continue differently.
- A feedback move explicitly chooses whether active cancellation writes a caller-selected finite
  target or leaves its persistent request unchanged. That request still travels through the source
  graph; cancellation never writes hardware directly. A feedback move publishes once and does not
  fight a later request. Success or timeout does not hide a terminal command; compose any required
  outcome-dependent follow-up explicitly.

When a supported third-party follower requires updates beyond an active route Task, one stable
composition-root heartbeat updates it throughout the relevant loop. An adapter reached by both
root and Task deduplicates by cycle and counts vendor-hidden updates. Its `stop()` applies physical
zero immediately, including after reentrant callbacks; staging zero for a future cycle is not stop.

---

## 4. One realization path

Behavior expresses intent. One visible graph resolves that intent into one final hardware command.
Tasks, controls, queues, calibration tools, and safety policy do not create competing imperative
writers.

### Plants

Each Plant has one final `PlantTargetResolver`:

```text
command or planned intent
        -> exact / equivalent positions / overlay / plan
        -> guards, references, and hardware bounds
        -> one Plant update
        -> actuator output
```

- Use `PlantTargets.exact(...)`, `equivalentPositionsOf(...)`, `overlay(...)`, or the advanced
  `plan(...)` to compose intent. The Plant applies final guards, references, and bounds.
- Use the vocabulary **command -> requested target -> applied target -> actuator output**. Do not
  collapse those distinct facts into a vague `target` or `power` claim.
- Direct-power Plants own the normalized target range `[-1, +1]`. A static guard fallback and every
  final guarded target must be finite and inside the declared Plant range.
- The FTC standard-servo native `[0, 1]` SDK envelope is an adapter command domain, not evidence
  that a mechanism may safely travel through that entire interval. An ordinary named-position
  servo mechanism uses a normalized mechanism coordinate such as `CLOSED = 0.0` and `OPEN = 1.0`
  (or meaningful physical units), then explicitly maps its Plant bounds to human-reviewed,
  backed-off native endpoints. Plant snapshots report that mechanism coordinate; native endpoint
  facts remain configuration. A normalized `0.5` is halfway through the configured command
  interval, not proof of halfway physical linkage travel or shaft feedback. Raw-native Plant units
  require their own mechanism-specific safe bounds; neither the SDK envelope nor `defaults()`
  supplies that review.
- A Task changes a request; the mechanism's ordinary output phase updates the Plant. A calibration
  Task may stage a temporary command-preserving calibration mode, and an output queue may propose
  an overlay value, but the mechanism remains the one Plant heartbeat and final writer. Any
  post-reference semantic request goes through that mechanism's normal setter after exact search
  success.
- Open-loop Plants may hold a command but cannot claim sensor-based arrival. A direct scalar
  feedback move names the exact graph-owned command target it writes and the feedback Plant that
  proves completion; a named semantic move instead calls its mechanism setter and waits on the
  owner's coherent request/arrival status.
- `Plant.stop()` is the one final Plant lifecycle operation. It latches the Plant terminal, invokes
  the realization's natural stop, and makes later updates inert before resolver, plan, feedback,
  guard, controller, or hardware work. It does not rewrite the command target or reset the resolver
  graph; construct a fresh Plant for another lifetime. During an active match, request zero power,
  zero velocity, or a position hold through the existing source graph rather than stopping the
  Plant.
- Robot shutdown separately cancels coordinated Tasks, queues, services, and other owned work, then
  stops each Plant. Cancelling one Task cannot erase unrelated persistent requests, and terminal
  Plant stop does not need to discover or disable every kind of upstream resolver.

### Drive

Drive uses the parallel grammar:

```text
DriveSource -> DriveSignal -> optional overlays/guidance -> DriveCommandSink
```

The source produces robot-centric intent; overlays reshape selected components; one sink performs
the final write. A direct timed Drive Task is appropriate only when it is the sole behavior writer
for that sink. Route and guidance integrations select behavior through Sushi seams without
turning a vendor follower into a second final writer.

One realization path makes interlocks and diagnostics inspectable: the framework can distinguish
what was requested, what policy allowed, and what hardware received.

---

## 5. Truthful boundaries

Validate a fact where it becomes knowable, and claim only what the available evidence proves.
Sushi rejects invalid configuration at the first boundary that owns it. Runtime Plants expose
finite range clamping and loss of target evidence through explicit status rather than hiding either.

### Validation and physical truth

- Configuration builders reject non-finite values, invalid ranges, impossible combinations, and
  unsafe static fallbacks at the layer that owns the rule. A bounded Plant may explicitly clamp a
  finite runtime request to its declared range and report `CLAMPED_TO_RANGE`, unless a later guard
  supplies a more specific resolution status. A non-finite runtime target becomes observable
  `TARGET_UNAVAILABLE`, and the Plant retains or recovers a safe finite applied target according to
  its contract. Hardware-adapter clamping is final defense in depth, not permission for upstream
  invalid commands.
- Bounds are stated in the units and domain owned by that layer. Scaling does not silently change
  the meaning of previously declared bounds.
- Construction proves that software configuration is coherent; it does not prove readiness,
  calibration, physical placement, wiring, or safe power. `defaults()` is not a hardware safety
  review.
- A generic actuator bring-up tool may establish the exact FTC direction, submitted command, or
  encoder sample it observed. It must not relabel command cache as physical feedback, discover a
  limit by collision/stall, choose robot-owned Plant units, or claim that captured endpoints remain
  safe under production load.
- Requested, applied, measured, ready, and complete are separate facts. An open-loop command cannot
  claim physical arrival, and a software pose rebase cannot claim that the robot moved.
- Cleanup helpers may aggregate best-effort actions and exceptions, but they do not choose resources,
  authorize recovery, or prove rollback.

### Time, frames, and observations

- A captured observation carries one clock-created `LoopTimestamp`, which keeps clock identity,
  reset epoch, and time together. A boundary anchors an age-native vendor result once and retains
  that timestamp while the result identity repeats; it never refreshes cached age as though the
  observation were new.
- Put frames, units, and signs in names when ambiguity is possible. Sushi uses robot `+X` forward,
  `+Y` left, `+Z` up; positive yaw/omega is counter-clockwise; distance is inches and angles are
  radians. Convert vendor conventions at boundaries.
- Data spanning frames uses explicit transform names such as `fieldToRobotPose` and
  `robotToCameraPose`; vague `pose`, `x`, or `heading` names are insufficient when the frame matters.

### Routes and integrations

- One route start owns one execution identity. Its retained status and cancellation cannot drift to
  a replacement route.
- Vendor idle or not-busy is not proof of endpoint success. The integration classifies normal
  endpoint completion, follower timeout/stall, interruption, replacement, failure, and unknown
  terminal state while the evidence is available. `RouteTask` preserves that classification and
  adds `TASK_TIMEOUT`, `CANCELLED`, or fail-closed `FAILED` when the Task boundary owns the ending;
  `FAILED` may therefore originate at either boundary. Robot strategy explicitly maps those facts
  to continue, fallback, or abort.
- Build fixed route geometry eagerly. Geometry based on live pose, vision, or another current fact
  resolves exactly once in an explicitly named start-time route Task factory. Construction is quick
  and non-blocking, interpretation remains robot-owned, and failure leaves the follower untouched.

### Diagnostics and errors

- Exceptions name the invalid fact and the correction. Reject impossible configuration at build
  time when possible.
- `toString()` gives a compact identity/configuration description for a stable value.
  `debugDump(...)` reports cached or already-published live diagnostic state with stable keys. It
  never blocks, samples a behavior-changing source, advances a filter, updates hardware, or commits
  telemetry.
- Required driver status is not hidden in optional diagnostics. Presenters add to one frame; the
  frame owner commits once.

Truthfulness includes uncertainty. Software tests can prove software contracts; only controlled
hardware checks can prove direction, motion, tuning, wiring, and mechanism safety.

---

## 6. One current, proven story

Documentation is part of the API. A student should find one present-state explanation that agrees
with code completion, Javadocs, examples, exceptions, and tests.

### Implications

- [`docs/README.md`](<docs/README.md>) is the canonical documentation hub. Root and package READMEs
  orient readers and route them there instead of maintaining competing maps.
- Teach by progressive disclosure. First contact may assume basic Java, `if` statements, methods,
  FTC gamepad fields, and either familiar FTC loop spelling; it must not assume lambdas, callback
  registration, Tasks, or framework-specific nouns. Start with an FTC-loop bridge and a required
  software-only tour, then offer goal-based Build recipes, deeper concepts, examples, reference,
  and maintainer material.
- Before the first API that saves code for later, contrast a method call that runs now with a
  function registered during setup and explain exactly when that saved function runs. Introduce a
  plain action description before its framework noun. First-contact diagrams use those plain
  actions; a nearby explanation may then map them to exact API terms.
- Markdown is the one authored source for narrative guides and any generated documentation site.
  Javadocs remain the exact method-level API contract. Generated HTML must consume those sources,
  not become another hand-maintained copy.
- Describe the supported current state. Historical decisions, migration notes, rejected alternatives,
  and future backlogs do not belong in a beginner path or normative guide.
- Compiling examples demonstrate the supported path. Small snippets explain one idea and link to the
  complete source rather than quietly inventing a second architecture.
- Every class and non-trivial method has useful Javadocs. Document units, frames, preconditions,
  side effects, lifecycle, failure behavior, and same-cycle semantics where relevant.
- API and behavior changes update implementation, Javadocs, guides, examples, and callers in the same
  coherent change.
- Verification is proportional to risk: targeted tests first, then the narrowest relevant clean
  compile/test suite. Report separately what cannot be verified without the robot.

Good documentation is an implication of simplicity: if the API cannot be explained with one
consistent vocabulary and one ordinary example, the design probably still contains competing ideas.

---

## Design checklist

Before adding or changing a framework concept, answer:

1. **Obvious path:** Is there one ordinary student-facing way to do this? If an advanced seam is
   necessary, is its different evidence or lifecycle explicit?
2. **Owner:** Who owns the state, resource, policy, update, stop, and final write? Are any peers
   competing for the same responsibility?
3. **Heartbeat:** Which managed phase advances it? Is same-cycle behavior and failure replay defined?
4. **Realization:** Can every hardware effect be traced through one final source/Plant or drive path?
5. **Truth:** Which boundary knows each range, unit, frame, timestamp, readiness, and completion fact?
   What happens when evidence is missing or invalid?
6. **Proof:** Do present-state Javadocs, guides, examples, diagnostics, and tests tell the same story?

If an answer is unclear, simplify ownership or naming before introducing another abstraction.

## Where exact contracts live

| Topic | Detailed guide |
| --- | --- |
| Beginner FTC-loop bridge | [`How Sushi runs your code`](<docs/getting-started/Framework Overview.md>) |
| Exact managed phases and custom-host exception | [`Loop Structure`](<docs/core-concepts/Loop Structure.md>) |
| Sources, caching, and signals | [`Sources and Signals`](<docs/core-concepts/Sources and Signals.md>) |
| FTC construction and Plant grammar | [`FTC Actuators & Plants`](<docs/ftc-boundary/FTC Actuators & Plants.md>) |
| Target resolution and guards | [`Mechanism Target Planning`](<docs/drive-vision/Mechanism Target Planning.md>) |
| Tasks, timing, composition, and cancellation | [`Tasks and Macros`](<docs/design/Tasks & Macros Quickstart.md>) |
| Output queues and source proposals | [`Output Tasks & Queues`](<docs/design/Output Tasks & Queues.md>) |
| Robot ownership and framework lanes | [`Framework Lanes & Robot Controls`](<docs/design/Framework Lanes & Robot Controls.md>) |
| Capabilities shared by TeleOp and Auto | [`Robot Capabilities & Mode Clients`](<docs/design/Robot Capabilities & Mode Clients.md>) |
| Drive guidance and coordinate contracts | [`Drive Guidance`](<docs/drive-vision/Drive Guidance.md>) |
| Spatial observations | [`Spatial Queries`](<docs/drive-vision/Spatial Queries.md>) |
| Pedro route lifecycle | [`Pedro integration`](<integrations/pedro/README.md>) |
| Software route-status checkpoint | [`Inspect one Pedro route's software outcome`](<docs/build/First Pedro Auto.md>) |
| Actuator facts and safe endpoints | [`Actuator Bring-up`](<docs/testing-calibration/Actuator Bring-up.md>) |
| Tuning and calibration evidence | [`Control Tuning Workflow`](<docs/testing-calibration/Control Tuning Workflow.md>) |
| Documentation and maintainer practices | [`Maintainer Notes`](<docs/maintainers/Maintainer Notes.md>) |

Javadocs on the referenced types are authoritative for exact signatures, argument contracts, and
failure behavior.
