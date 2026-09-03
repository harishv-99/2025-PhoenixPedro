---
tags:
  - Advanced
---

# Loop Structure

Sushi assumes that one OpMode loop is the “heartbeat” of the robot. In ordinary FTC robot code,
`FtcRobotOpMode` and its framework-created `RobotProgram` own that heartbeat; robot code declares
the owners that participate in it.

This document explains:

* the recommended **update order**,
* why certain components are **idempotent by cycle**,
* and how to avoid common FTC loop pitfalls (double-updates, hidden time steps, and accidental blocking).

---

## 1. The Sushi loop contract

Sushi code is designed around a single contract:

> **One OpMode loop cycle advances the robot once.**

Everything else should be driven from that cycle.

This keeps behavior predictable and makes it much easier to reason about timing.

---

## 2. The recommended loop order

The ordinary managed ordering is:

> **Clock → Services → Bindings → Tasks → Outputs/Drive → Presenters → Telemetry commit**

Services are upstream owners such as sensing, localization, and a required vendor heartbeat.
Outputs are downstream realization owners such as Plant-backed mechanisms. The one optional
source-driven drive joins output declaration order; declare it before mechanisms when that is the
robot's intended realization order.

### 2.1 Why this order?

**Clock first**

* Many subsystems depend on `dtSec()` and `cycle()`.
* Updating the clock once defines “this loop cycle” for every component.

**Services before Bindings**

* If bindings depend on sensor signals (distance thresholds, vision targets, etc.), update their
  owning services first.
* Gamepad axes/buttons are exposed as `ScalarSource`/`BooleanSource` and are sampled when you call
  `get(...)`.
* Edge/toggle trackers (e.g., `risingEdge()`, `toggled()`) must be sampled each loop to avoid
  missing transitions. `Bindings.update(clock)` does that sampling for the bindings you register.

**Bindings before Tasks**

* Bindings typically enqueue tasks (macros).
* If you enqueue in this cycle, you usually want the runner to see it immediately.

**Tasks before outputs and drive**

* Ordinary Tasks are the “decision layer”: they update behavior sources or request Plant targets.
  A position-calibration Task may also manage a temporary search lifecycle, but it never calls
  `plant.update(clock)`.
* The downstream Outputs/Drive phase is the “actuation layer” that consumes those decisions.
* Each mechanism or subsystem remains the sole heartbeat owner for its private Plants, including
  while one of those Plants is in calibration-search mode.
* In TeleOp, the one `program.drive(...)` declaration is the final behavior-command writer at its
  output-order position. Tasks must not also write imperatively to the same drive sink.

**Outputs and drive keep declaration order**

* Each output declaration identifies one final realization owner and one intentional place in the
  downstream phase.
* `program.drive(source, sink)` calls the sink heartbeat, samples the final source, rejects any
  non-finite component, clamps finite components, and writes that command at its declaration
  position.
* A mechanism remains the sole updater of its private Plants; the program does not discover or
  register raw Plants.

**Telemetry last**

* Telemetry is slow and should never influence state updates.

---

## 3. The canonical managed program

The compiling [`StarterTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.html>) shows the
ordinary complete host. It overrides one configuration method, not five FTC lifecycle methods:

```java
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

public final class MyTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        ShooterMechanism shooter = program.output(
                new ShooterMechanism(hardwareMap, profile.shooter));
        IntakeMechanism intake = program.output(
                new IntakeMechanism(hardwareMap, profile.intake));

        MyControls controls = new MyControls(new GamepadDevice(gamepad1));
        controls.bind(program.callbackBindings(), shooter, intake);
        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, profile.drive));

        program.presenter((clock, telemetry) -> {
            telemetry.addData("dtSec", clock.dtSec());
            telemetry.addData("shooter.ready", shooter.status().ready());
        });
    }
}
```

`GamepadDevice` is the FTC input adapter; after construction, controls and drive code consume only
its Sushi sources. Its `fw.ftc.input` boundary location adds no construction ceremony: ordinary
robot code uses `new GamepadDevice(gamepad1)`.

The program owns one private `LoopClock`, `Bindings`, and `TaskRunner`. It may also own one
aggregated, data-only `RobotProgram.Prestart`. INIT advances the clock, updates that prestart role,
then runs presenters; services, bindings, Tasks, drive, and outputs do not actuate. START freezes
prestart exactly once before resetting the clock. `READY` starts services, starts and first-updates
the optional root Task, then realizes outputs once. `BLOCKED` starts none of them and continues only
the clock and presenters so the reason stays visible.
That exact-start realization keeps a positive-duration request observable even if the first active
loop is delayed. Each active loop follows the fixed order above and commits telemetry exactly once.

STOP and every caught lifecycle `RuntimeException` use one terminal cleanup order:

```text
cancel Tasks -> clear bindings -> stop outputs in declaration order -> stop services in reverse order
```

The first failure remains primary and later cleanup failures are suppressed. Repeated or reentrant
STOP is inert. `Error` is not caught.

If a selected root depends on data frozen at START, use
`Tasks.buildAtStart(name, taskSupplier)`. It defers only one Task graph; hardware/resource owners
still belong in `configure(program)`. A typed `program.stopHandoff(capture, publish, invalidate)`
supports Auto-to-TeleOp transfer without a custom FTC lifecycle: capture occurs only on normal
ACTIVE STOP, publication waits for successful cleanup, and every other path invalidates.

The curated ordinary examples use this managed host exclusively. A genuinely custom host is an
advanced exception, not a second robot recipe. Its eligibility and complete lifecycle contract live
in [`Advanced host ownership`](<../maintainers/Maintainer Notes.md#11-advanced-host-ownership>).
Do not copy an active-loop excerpt without its construction and STOP ownership.

That is the normal TeleOp ownership model: Tasks finish their decision/source/Plant-target work,
then the downstream Outputs/Drive phase visits declarations in order. The one drive declaration
samples the composed `DriveSource` and writes its sink, while each mechanism performs the sole
update of its private Plants. A calibration-search Task can stage or release temporary search mode
during the Task phase; it does not add another Plant update.

For a simple open-loop Auto routine or drive tester with no competing final writer,
`DriveTasks.driveExclusivelyForSeconds(...)` can own the sink for an interval. It calls the sink's
update hook and writes the requested signal every active cycle, then stops on completion or active
cancellation. Do not also run the `driveSource` write shown above while that exclusive Task is
active. If an adapter's supported lifecycle requires updates beyond active Tasks, its composition
root continues calling `update(clock)` with the shared `LoopClock`, and the adapter deduplicates the
Task's same-cycle update. Use route or guidance Tasks, not timed open-loop drive, for normal Pedro
route movement.

### 3.1 Haptic commands do not add a loop phase

A `HapticSink` is command-only. A controls binding or dedicated driver-feedback owner calls
`pulse(...)` when its event occurs; the sink has no `update(clock)` method, background thread, or
separate heartbeat. A supervisor or service may supply status or robot policy, but it does not make
the low-level sink own cue mapping. Use `Bindings.onRise(...)` for an ordinary state-change cue. If
the robot intentionally repeats one reminder while a condition stays true, make that repeat policy
visible with a dedicated `Cooldown` for that event rather than calling `pulse(...)`
unconditionally every loop.

In a managed program, register one robot-owned feedback owner as an output. It privately retains
the haptic sinks, exposes semantic pulse methods to controls, has an intentionally no-op
`update(clock)` because the sinks are command-only, and stops every sink from its `stop()` method.
That is a truthful lifecycle/output role, not a fake heartbeat. A deliberate custom host may retain
and stop its sinks directly; ordinary code relies on the program to reach the registered owner's
cleanup path.
For the `FtcHaptics` adapter, both pulse and stop are queued, best-effort SDK commands. The queue
retains only the latest undelivered request, and a delivered request displaces the current effect;
neither call reports physical delivery, and controller support varies.

### 3.2 Advanced manual bulk caching is a first service

Ordinary programs leave the FTC SDK's module cache mode untouched. A robot that has separately
measured and selected REV/Lynx `MANUAL` caching can opt into its lifecycle owner at the very top of
`configure(...)`:

```java
program.service(FtcBulkCaching.manual(hardwareMap)); // declare this service first
```

The factory discovers modules during configuration/INIT but performs no cache operation then. At
START the first service captures every actual prior mode before mutation, attempts the ordered
`MANUAL` mode pass, and begins initial invalidation only if every mode write succeeds. In each
claimed active cycle it runs after the clock advances and before later services, making at most one
owner-issued clear attempt per module and exactly one per module on a successful pass. Successful
START already owns that cycle, so the same-cycle service update is a no-op.

Because services stop in reverse declaration order, literal first placement also keeps the owner
active until dependent services have stopped. Its terminal cleanup first attempts to invalidate
every captured module, then attempts to restore every exact prior mode. It does not and cannot
restore the prior retained packet or command histories. The current `RobotProgram` API does not
enforce first placement; the composition root must make it true, and no other cache owner or direct
cache-management caller may coexist with the service.

This is an advanced opt-in, not a default phase or a Sushi adoption. It supplies lifecycle order,
not freshness, validity, transaction-count, timing, or performance guarantees. See
[`FTC manual bulk caching`](<../ftc-boundary/FTC Manual Bulk Caching.md>) for the exact eligible reads,
failure semantics, cleanup contract, and tester incompatibility.

---

## 4. Idempotency: “safe if called twice”

Sushi is used in real teams with multiple layers of helper code:

* testers and debug menus
* robot frameworks wrapping robot frameworks
* subsystems that try to “helpfully” update inputs or tasks

To avoid brittle behavior, several Sushi components are **idempotent by `clock.cycle()`**.

That means:

* If the component was already updated during the current cycle, additional calls do nothing.
* `cycle()` is an identity, not a count of elapsed loops. It stays monotonic for the lifetime of one
  clock and advances immediately at every explicit `clock.reset(...)`, so a post-reset read cannot
  reuse a pre-reset cache entry even when the reset time is unchanged.

### 4.1 Sources, edges, and memoization are cycle-idempotent

Sushi uses `LoopClock.cycle()` to make many source wrappers safe to read multiple times in the
same loop.

Examples:

* `ScalarSource.memoized()` / `BooleanSource.memoized()`
* `BooleanSource.risingEdge()` / `fallingEdge()`
* `BooleanSource.toggled()`
* drive rate limiters, conditional overlays, built overlay stacks, and guidance runtimes

These wrappers publish at most one successful advance per cycle <em>when sampled</em>. They reject
recursive sampling. A failed value attempt leaves the wrapper's prior committed state and cycle
unchanged, so a later nonrecursive same-cycle call may retry instead of receiving a plausible stale,
null, zero, or false result. Once one attempt succeeds, repeated reads return that exact result. If
you never sample an edge/toggle source during a cycle, it cannot observe that transition.

Sampling a `BooleanSource.and(...)` or `or(...)` composite observes the left operand first. If that
succeeds, it observes the right operand once regardless of the left Boolean value, then combines
their values. A decisive left value therefore does not freeze a stateful right operand. This does
not make the source graph push-based: if the composite is not sampled, neither operand is sampled
through that graph. Conditional producers such as `BooleanSource.choose(...)` remain intentionally
branch-lazy.

The state owner provides that protection. Robot code does not need to remember an outer
`memoized()` call around a stateful drive composition. Repeated reads of a conditional overlay or
built overlay stack in one cycle return the same successful command and do not resample the base,
activation gates, or active overlays. Guidance advances its blend/controllers once and accepts only
one requested mask for a runtime in that cycle; use the plan's natural/union mask or an independent
runtime if two consumers genuinely need different masks. Pure transforms such as drive scaling own
no advancing behavior state and need no extra cache.

Cycle caches become valid only after a complete sample succeeds. If an upstream source, gate, or
overlay throws, the owner does not publish stale or null output as that cycle's result. A source
that reaches an arbitrary stateful scalar controller retains a controller-thrown exception for the
rest of that cycle instead of replaying an invocation whose internal mutation cannot be rolled
back; failures while reading its inputs still occur before that attempt boundary and remain
retryable.

An explicit `clock.reset(...)` advances the cycle identity, so the next sample cannot reuse a
pre-reset cache entry. It does not reset a source or any controller state. A structural source
graph's explicit `reset()` clears its own local state and propagates through wrapped source/gate
children. Reset happens while the graph is inactive: a structural wrapper rejects overlap, resets
children first, and clears local state only after every child succeeds. A child failure may leave
earlier child resets observable, but generic source code does not claim rollback. In contrast,
`SpatialQuery.reset()` is query-local: frame providers, solve lanes, sensors,
estimators, and selection policies supplied through its reusable spec remain owned by their
composition roots.

For synchronous control callbacks, ordinary robot code registers through
`program.callbackBindings()` and `RobotProgram` performs the one sample/update each active loop.
Only an explicit custom binding owner calls `Bindings.update(clock)` directly.

### 4.2 Localization publishes one snapshot per cycle

Call the robot's localization lane once from its robot-owned service in the Services phase. Sushi
localization predictors, absolute estimators, corrected estimators, and the owning FTC lane claim
their first update attempt for that `clock.cycle()`. A successful repeat leaves the exact snapshot
unchanged and does not poll hardware, solve tags, write Limelight orientation, advance a filter, or
traverse the lane again.
Reentrant update is a lifecycle error. If an attempt throws, a same-cycle repeat rethrows that
failure rather than retrying effects that may already have happened.

Cycle identity and measurement identity solve different problems. Corrected estimators also retain
the end timestamp of predictor motion already consumed, so a source that holds the same
`MotionDelta` into another cycle cannot move the fused pose twice. A zero-time predictor sample
publishes no usable delta and keeps its accepted motion baseline for the next positive-time sample.
Robot code supplies neither guard: it still writes only:

```java
localization.update(clock);
PoseEstimate pose = localization.globalEstimator().getEstimate();
```

Use the one non-null shared `LoopClock`. Complete camera processor/pipeline transitions before the
localization phase; a change after this cycle's snapshot was published is reflected next cycle.

### 4.3 Bindings own one effectful attempt per cycle

`Bindings.update(clock)` claims the cycle before sampling a context activation, binding source, or
callback. After a successful traversal, another call in the same cycle is a no-op. A recursive
update is rejected before it can start another traversal.

If an activation, source, or callback throws a `RuntimeException`, the traversal stops. Bindings do
not roll back already sampled source state or callback effects. Another call in that same cycle
rethrows the exact same exception without sampling or dispatching again. A different cycle may
attempt the existing graph again from the state it actually reached; only an explicit custom owner
should choose that recovery policy. The ordinary `FtcRobotOpMode` path instead becomes terminal,
cancels Tasks, clears bindings, stops outputs and services, and rethrows the original failure.

Optional `Bindings.ControlContext` objects are declaration groups, not additional heartbeat owners.
Do not update them separately; the parent `bindings.update(clock)` samples every context's
activation once in context-creation order before sampling any binding source or dispatching any
callback, then uses those snapshots for the complete input frame. If a callback changes a mode
used by a context, the new eligibility starts on the next loop. The root's same-cycle idempotency
covers both root and contextual registrations.

After the activation prepass, the root visits root and contextual registrations once in their
global declaration order across binding kinds. Any source samples, neutral outputs, or callbacks
produced by those visits follow that order; ordinary binding sources are not pre-snapshotted.
Declaration order is sequencing only, not priority or final-output arbitration. Register controls,
create contexts, and clear/rebuild the graph during initialization or between updates—attempting a
structural change while `Bindings.update(clock)` is running fails before changing the graph.
`Bindings.clear()` is an explicit rebuild boundary: it invalidates old contexts, removes
registrations, and resets the retained cycle attempt or failure. It cannot undo source state or
external effects produced before a failed traversal stopped.

### 4.4 TaskRunner is idempotent

`TaskRunner.update(clock)` will not advance the current task twice in one loop cycle.

This is critical because advancing tasks twice effectively doubles loop speed and breaks timeouts.

### 4.5 Stateful external followers need a persistent owner

A route Task is active while its per-route `RouteExecution` reports `RouteStatus.ACTIVE`. Some
vendor followers still need updates after that execution becomes terminal—for example, Pedro
hold-end control and pose tracking continue while a later mechanism or wait Task runs.

Give such an adapter one stable composition-root heartbeat every Auto loop. If `RouteTask` or
`DriveGuidanceTask` can also reach the same update hook, make the adapter idempotent by
`clock.cycle()` so the root and Task calls still produce exactly one vendor update. Vendor methods
that secretly perform an update during a mode transition must count as that cycle's heartbeat.

The managed basic Pedro reference makes the persistent ownership visible. Its Auto service owns
the following upstream order:

```text
Clock → Localization → Pedro heartbeat → Auto Tasks → Mechanism Plants → Telemetry
```

`BasicPedroAuto` supplies only `configure(program)`; inherited final FTC callbacks own the
shared clock and managed lifecycle. At START the program resets its clock, then the service applies
the declared Pedro start pose before the order above. The complete hardware graph and fixed route
are constructed once during configuration and are never retried or replaced inside the same
OpMode. A larger robot may insert vision or targeting services before the follower heartbeat while
preserving the same one-owner, declaration-ordered service phase.

The production Pedro runtime shares the localization phase's one Pinpoint predictor. Its Pedro
`Localizer` is passive: the downstream heartbeat verifies and consumes that current-cycle snapshot
instead of polling odometry again. Accepted corrections pushed into the predictor are therefore
visible to path control in the same heartbeat.

One service therefore owns localization before the recurring Pedro heartbeat; the program runs
that service before its root Task and downstream mechanism output.

The heartbeat precedes the Task runner so the adapter can classify and retain route completion,
timeout/stall, interruption, replacement, or an unknown terminal transition before the Task reads
its `RouteExecution`. A route selected by the runner begins advancing on the next loop. A Pedro
manual-mode transition uses that next loop's vendor-hidden zero update, then applies its retained
nonzero command on the following heartbeat. This small, predictable staging delay is preferable to
a hidden second follower update.

If a route is built with `RouteTasks.followBuiltAtStart(...)`, its supplier runs exactly once when
that Route Task starts in the Task-runner phase. It can therefore snapshot the localization,
targeting, or vision facts already refreshed earlier in the same cycle. Keep the supplier quick and
non-blocking; the resulting route starts in that phase and begins advancing on the next owned
follower heartbeat.

---

## 5. Where time comes from (and where it must not come from)

Sushi is intentionally strict about time:

* `LoopClock.update(getRuntime())` defines `dtSec()`.
* Tasks and stateful source wrappers such as drive rate limiters use `dtSec()`.

Avoid these patterns:

* calling `System.nanoTime()` or `ElapsedTime` deep inside tasks to “self-time”
* calling `getRuntime()` inside multiple subsystems and letting each compute its own dt

If you need time, take it from the `LoopClock`.

### 5.1 Captured measurement time

When a measurement time must cross component boundaries, keep it as one `LoopTimestamp`:

```java
LoopTimestamp capturedNow = clock.nowTimestamp();
LoopTimestamp delayedFrame = clock.timestampSecondsAgo(frameAgeSec);

double ageSec = delayedFrame.ageSec(clock);
boolean usable = delayedFrame.isFresh(clock, 0.20);
double spacingSec = capturedNow.secondsSince(delayedFrame);
```

The value privately retains its clock and reset epoch as well as its coordinate. Consumers do not
store or compare a separate epoch, and there is intentionally no raw timestamp accessor. A reset
invalidates older timestamps automatically; an unavailable or prior-epoch timestamp returns `NaN`
age and is not fresh. Passing a timestamp to a different `LoopClock` is a wiring error.

Use `timestampSecondsAgo(...)` once where an age-native sensor or vendor result enters the
framework, then retain and forward that timestamp. Recomputing `now - cachedAge` each time a cached
result is sampled would incorrectly make it appear newly captured. `LoopTimestamp.unavailable()`
is the explicit non-null value when no truthful measurement time exists. Raw vendor/FTC clock
coordinates stay private to their boundary until translated; lifecycle-local Task start/deadline
seconds do not need this wrapper when they never escape that Task.

### 5.2 Optional loop-phase diagnostics

When one managed owner is slow and you need to find its internal cost, that service, output, or
presenter may retain a `LoopPhaseProfiler` and mark stable, high-level boundaries inside its own
`update(clock)` or presentation call. This is owner-internal instrumentation, not another
heartbeat: `RobotProgram` still advances one `LoopClock` and calls the owner in its normal phase.
The profiler hides its own monotonic stopwatch, never advances `LoopClock`, and must never supply a
value that changes Tasks, controllers, targets, guards, or other robot behavior.

At the start of that owner's one call for the cycle, call `startCycle(clock)`. After each stable
internal stage, call `finishPhase(name)` with a fixed literal name; it measures the interval since
`startCycle(clock)` or the preceding `finishPhase(...)`. Call `finishCycle(clock)` before returning
from the owner. Retain the profiler instead of allocating it per update. It deliberately has no
nested-span, callback, scheduler, sleep, or loop-rate API.

This pattern covers only the work between that owner's markers. A composition-root-retained
profiler may also span markers placed in several of its own registered roles, but the elapsed span
then includes any private `RobotProgram` work between those markers and cannot truthfully attribute
that time to a framework phase. It still cannot mark private transitions outside those roles or
Driver Station commit latency. Profiling across the complete managed phase schedule is possible
only inside an already-approved advanced host whose materially different lifecycle independently
justifies that ownership. Profiling alone is not a reason to recreate FTC callbacks; use the
[`Advanced host ownership`](<../maintainers/Maintainer Notes.md#11-advanced-host-ownership>)
contract before retaining such a host.

Keep profiling disabled outside a deliberate diagnostic run. When it is enabled, `snapshot()` and
`debugDump(...)` report only fully completed spans, so a dump made during one active span still
describes the preceding completed one. Select `debugDump(...)` at the call site only when that
diagnostic is enabled, and use stable literal names such as `"poll"`, `"estimate"`, and `"publish"`;
do not generate one name per device, route, target, or cycle.

The reported seconds are monotonic elapsed wall time. They can include operating-system scheduling
pauses and profiler overhead; they are not CPU time and do not cover work before `startCycle` or
after `finishCycle`. If the same profiler has already collected data when its owning lifecycle
deliberately resets its `LoopClock`, call `reset()` at that boundary while no profiler span is
active.

---

## 6. Direct drive and vendor heartbeats

`MecanumDrivebase.drive(signal)` sends the normalized command to its coordinated outputs
**immediately**. It has no direct-drive heartbeat and does not own rate limiting. Put shaping in the
source graph instead:

```java
DriveSource shaped = manual.rateLimited(4.0, 4.0, 6.0);

// Once in the explicit drive phase:
drivebase.drive(shaped.get(clock).clamped());
```

The generic `DriveCommandSink.update(clock)` hook still matters for Pedro and other stateful vendor
adapters whose supported lifecycle requires a stable composition-root heartbeat. Call it for that
adapter every relevant loop and let the adapter deduplicate same-cycle Task/root calls. Do not copy
that rule onto a direct `MecanumDrivebase` merely for API symmetry.

---

## 7. Plants: update is mandatory

Plants are stateful actuators. Tasks typically request targets or manage a temporary calibration
recipe, but those decisions do not apply “by magic.”

Each mechanism or subsystem must call each of its private Plants exactly once in its downstream
Plant phase:

```java
plant.update(clock);
```

The Task, control, or composition layer must not call that Plant a second time.

Typical pattern:

* construct and store Plants privately in the mechanism owner
* advance Tasks before the mechanism phase
* update each private Plant exactly once from that owner

For a position-calibration search, this order is observable and intentional. The Task phase acquires
the temporary search, samples the cue, establishes the reference when found, and releases the
search on success, timeout, or active cancellation. If the search remains active, the later Plant
phase submits its staged power exactly once. If the cue is already true, the Task releases the
search before that phase, so no search-power command is submitted. The cue is sampled before the
timeout check, so it also wins when both become true at the same boundary. A successful
search, timeout, and cancellation all preserve the existing persistent command. If exact success
should select a named semantic request, the enclosing mechanism Task calls its normal setter as the
next exact-success sequence step; that immediate continuation starts before the same downstream
Plant update. In every case the Plant phase evaluates the appropriate owner-held state once.

---

## 8. Telemetry and debug output

Telemetry should not control robot behavior.

Treat one Driver Station update as a composed frame:

1. Required presenters and renderers add the mode, safety, remediation, and high-level status rows
   drivers need during normal operation.
2. The composition root may add one or more explicitly selected optional diagnostics.
3. The owner of that complete frame calls `telemetry.update()` exactly once.

Additive presenters and renderers do not call `clear()`, `clearAll()`, or `update()`. This lets an
Auto prestart owner, route adapter, robot presenter, and diagnostic contributor share one frame without their
call order accidentally hiding or prematurely committing another contributor's rows. An ordinary
`RobotProgram` owns the final INIT, blocked, and active commits. A custom portable host or dedicated
tester may still commit an exclusive screen when it is the complete-frame owner.

If you need structured optional debug output, prefer debug sinks:

* many subsystems expose `debugDump(dbg, prefix)`

`DebugSink` is pull-based and output-only. It does not select topics, filter keys, clear telemetry,
or commit a frame. Select the relevant top-level owner at the composition-root call site and skip
the dump entirely when that diagnostic is disabled. This avoids both an unwieldy all-object dump
and the traversal/formatting work that a no-op sink alone cannot prevent. Required telemetry stays
on the always-on presenter path rather than depending on `debugDump(...)`.

A dump should normally report cached or already-published state. It must not update behavior,
sample a Source, advance a filter, or perform blocking or expensive acquisition just to explain the
robot. An FTC or vendor boundary owner may make a documented cheap, side-effect-free status read
when the backend exposes no retained equivalent, but that read must not acquire a new result or
change robot behavior.

The curated examples and subsystem lab cards keep telemetry specific to the lesson. Experiment code
prints only program-known commands, measurements, timing, errors, and outcomes; externally observed
physical results remain operator records. Adopted match code should enable only the presenters
relevant to the current investigation.

Use a consistent prefix so logs are easy to scan, for example:

* `drive.*`
* `shooter.*`
* `tasks.*`

---

## 9. Common loop mistakes

### Mistake: blocking waits

Bad:

```java
while (!shooter.atTarget()) { }
```

Good:

* use `ScalarTasks.set(target, value).untilReachedBy(plant).cancelTo(...).build()` (or deliberately
  choose `.leaveRequestOnCancel()`) or use `Tasks.waitUntil(...)`, then declare it through
  `program.rootTask(...)` or `program.taskBindings()`. An explicitly owned custom/private runner is
  the advanced alternative.

### Mistake: missing edges by not sampling

Edge/toggle trackers like `risingEdge()` and `toggled()` only advance when they are sampled.

If you create a `BooleanSource` edge/toggle and then <em>don’t read it every loop</em>, you can miss transitions that happened in between.

Fix: make sure edge/toggle sources are sampled once per loop (e.g., by wiring them into `Bindings.update(clock)`, a drive pipeline, or telemetry that runs every loop).

### Mistake: double-running task updates

Ordinary robot code never advances the program's private runner. In a deliberate custom host, if two
helpers both call `macroRunner.update(clock)`, Sushi prevents double-advancement in the same cycle,
but the custom host should still make exactly one update call in its explicit Task phase.

### Mistake: making a route Task the only follower heartbeat

If a vendor follower owns pose, hold, callbacks, or final drive output after a route execution
becomes terminal, do not stop updating it when `RouteTask` completes. Keep the composition-root
heartbeat running and let the adapter deduplicate the Task-facing update. Do not infer success from
the vendor becoming not busy; use the retained `RouteExecution.status()` instead.

---

## 10. Summary

Sushi’s loop structure is intentionally boring:

* one clock
* one update order
* no hidden time steps

For ordinary robot code, declare roles once and let `RobotProgram` keep:

> Clock → Services → Bindings → Tasks → Outputs/Drive → Presenters → Telemetry commit

An advanced host is a narrow maintainer exception, not an alternate spelling of ordinary robot
code. It must satisfy the complete
[`Advanced host ownership`](<../maintainers/Maintainer Notes.md#11-advanced-host-ownership>) contract.

That keeps the rest of the framework predictable.
