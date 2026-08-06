# Loop Structure

Phoenix assumes that your OpMode loop is the “heartbeat” of the robot.

This document explains:

* the recommended **update order**,
* why certain components are **idempotent by cycle**,
* and how to avoid common FTC loop pitfalls (double-updates, hidden time steps, and accidental blocking).

---

## 1. The Phoenix loop contract

Phoenix code is designed around a single contract:

> **One OpMode loop cycle advances the robot once.**

Everything else should be driven from that cycle.

This keeps behavior predictable and makes it much easier to reason about timing.

---

## 2. The recommended loop order

Phoenix’s preferred ordering is:

> **Clock → Sensors → Bindings → Tasks → Drive → Plants → Telemetry**

### 2.1 Why this order?

**Clock first**

* Many subsystems depend on `dtSec()` and `cycle()`.
* Updating the clock once defines “this loop cycle” for every component.

**Sensors before Bindings**

* If bindings depend on sensor signals (distance thresholds, vision targets, etc.), update those sensors first.
* Gamepad axes/buttons are exposed as `ScalarSource`/`BooleanSource` and are sampled when you call
  `get(...)`.
* Edge/toggle trackers (e.g., `risingEdge()`, `toggled()`) must be sampled each loop to avoid
  missing transitions. `Bindings.update(clock)` does that sampling for the bindings you register.

**Bindings before Tasks**

* Bindings typically enqueue tasks (macros).
* If you enqueue in this cycle, you usually want the runner to see it immediately.

**Tasks before Drive and Plants**

* Ordinary Tasks are the “decision layer”: they update behavior sources or request Plant targets.
  A position-calibration Task may also manage a temporary search lifecycle, but it never calls
  `plant.update(clock)`.
* The Drive/Plants phases are the “actuation layer” that consumes those decisions.
* Each mechanism or subsystem remains the sole heartbeat owner for its private Plants, including
  while one of those Plants is in calibration-search mode.
* In TeleOp, the Drive phase is the one final behavior-command writer. Tasks must not also write
  imperatively to the same drive sink.

**Drive before Plants**

* Drive is often the most time-sensitive and can benefit from being applied early.
* Keeping one explicit drive phase makes the final `DriveSource` sample and hardware write easy to
  find. Stateful source wrappers obtain timing from the shared clock when that graph is sampled.

**Telemetry last**

* Telemetry is slow and should never influence state updates.

---

## 3. A canonical loop template

Below is the active-cycle portion of a typical Phoenix loop. The compiling
[`StarterTeleOp`](<../../../robots/examples/starter/StarterTeleOp.java>) and
[`StarterRobot`](<../../../robots/examples/starter/StarterRobot.java>) show the structured host and
composition-root form. For flat teaching OpModes, compare the complete
[`TeleOp_03_ShooterMacro`](<../../tools/examples/TeleOp_03_ShooterMacro.java>) and the smaller
drive-only [`TeleOp_01_MecanumBasic`](<../../tools/examples/TeleOp_01_MecanumBasic.java>).

```java
@Override
public void start() {
    clock.reset(getRuntime());
}

@Override
public void loop() {
    // 1) Clock
    clock.update(getRuntime());

    // 2) Sensors (optional)
    // scoringTarget.update(clock);

    // 3) Bindings (may enqueue macros)
    // Gamepad axes/buttons are Sources; they are sampled when you call get(...).
    bindings.update(clock);

    // 4) Tasks / macros
    macroRunner.update(clock);

    // 5) Drive
    DriveSignal cmd = driveSource.get(clock).clamped();
    drivebase.drive(cmd);      // applies motor power immediately

    // 6) Plants (mechanisms)
    double dtSec = clock.dtSec();
    shooter.update(clock);
    transfer.update(clock);
    pusher.update(clock);

    // 7) Telemetry
    telemetry.addData("dtSec", dtSec);
    telemetry.update();
}
```

This excerpt intentionally omits construction and FTC STOP. In structured robot code, the thin
OpMode calls its composition root's idempotent `stop()` once; that root cancels its runners and
stops every mechanism, drive, haptic, vision, and vendor resource whose lifecycle it owns. In a
flat teaching OpMode, the OpMode itself must cancel its active runner and stop every Plant, drive
sink, and other resource it constructed. The linked compiling examples show both complete cleanup
shapes; do not copy only the active-loop excerpt as a complete owner.

That is the normal TeleOp ownership model: Tasks finish their decision/source/Plant-target work,
then the one final Drive phase samples the composed `DriveSource` and writes the sink, and each
mechanism performs the sole downstream update of its private Plants. A calibration-search Task can
stage or release temporary search mode during the Task phase; it does not add another Plant update.

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

During OpMode cleanup, the owner that constructed each haptic sink calls `stop()`. A composition
root may retain and stop the sink directly, or it may invoke cleanup on a robot-owned feedback
owner that privately retains the sink; either way, the complete robot root owns reaching that
cleanup path.
For the `FtcHaptics` adapter, both pulse and stop are queued, best-effort SDK commands. The queue
retains only the latest undelivered request, and a delivered request displaces the current effect;
neither call reports physical delivery, and controller support varies.

---

## 4. Idempotency: “safe if called twice”

Phoenix is used in real teams with multiple layers of helper code:

* testers and debug menus
* robot frameworks wrapping robot frameworks
* subsystems that try to “helpfully” update inputs or tasks

To avoid brittle behavior, several Phoenix components are **idempotent by `clock.cycle()`**.

That means:

* If the component was already updated during the current cycle, additional calls do nothing.
* `cycle()` is an identity, not a count of elapsed loops. It stays monotonic for the lifetime of one
  clock and advances immediately at every explicit `clock.reset(...)`, so a post-reset read cannot
  reuse a pre-reset cache entry even when the reset time is unchanged.

### 4.1 Sources, edges, and memoization are cycle-idempotent

Phoenix uses `LoopClock.cycle()` to make many source wrappers safe to read multiple times in the
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

For buttons, the most common way to ensure sampling is to register a binding and call `Bindings.update(clock)` every loop.

### 4.2 Localization publishes one snapshot per cycle

Call the robot's localization lane once in the Sensors phase. Phoenix localization predictors,
absolute estimators, corrected estimators, and the owning FTC lane claim their first update attempt
for that `clock.cycle()`. A successful repeat leaves the exact snapshot unchanged and does not poll
hardware, solve tags, write Limelight orientation, advance a filter, or traverse the lane again.
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

### 4.3 Bindings are idempotent

`Bindings.update(clock)` is guarded so it will not fire actions twice in a cycle.

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

Phoenix Pedro Auto uses this explicit order:

```text
Clock → Localization → Targeting → Pedro heartbeat → Auto Tasks → Scoring Plants → Telemetry
```

The production Pedro runtime shares the localization phase's one Pinpoint predictor. Its Pedro
`Localizer` is passive: the downstream heartbeat verifies and consumes that current-cycle snapshot
instead of polling odometry again. Accepted corrections pushed into the predictor are therefore
visible to path control in the same heartbeat.

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

Phoenix is intentionally strict about time:

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

When the whole loop is slow and you need to find the owner, the composition root may opt into a
`LoopPhaseProfiler`. This is a diagnostic observer, not another behavioral clock. It hides its own
monotonic stopwatch, never advances `LoopClock`, and must never supply a value that changes Tasks,
controllers, targets, guards, or other robot behavior.

Keep the normal loop order visible and mark only stable, high-level boundaries. `finishPhase(name)`
names the interval that just completed, starting at `startCycle(clock)` or the preceding
`finishPhase(...)`. The profiler deliberately has no nested-span, callback, scheduler, sleep, or
loop-rate API.

```java
private static final boolean DEBUG_LOOP_PHASES = false;

private final LoopPhaseProfiler loopPhases =
        LoopPhaseProfiler.create(DEBUG_LOOP_PHASES);
private DebugSink debugSink = NullDebugSink.INSTANCE;

@Override
public void init() {
    // Retain one output adapter; do not allocate it inside loop().
    debugSink = DEBUG_LOOP_PHASES
            ? new FtcTelemetryDebugSink(telemetry)
            : NullDebugSink.INSTANCE;
}

@Override
public void start() {
    clock.reset(getRuntime());
    loopPhases.reset();
}

@Override
public void loop() {
    clock.update(getRuntime());
    loopPhases.startCycle(clock);

    localization.update(clock);
    loopPhases.finishPhase("localization");

    bindings.update(clock);
    loopPhases.finishPhase("bindings");

    macroRunner.update(clock);
    loopPhases.finishPhase("tasks");

    drivebase.drive(driveSource.get(clock).clamped());
    loopPhases.finishPhase("drive");

    shooter.update(clock);
    loopPhases.finishPhase("plants");

    // During this cycle, this displays the previous completed cycle.
    // Select the diagnostic at the call site so the disabled path does no dump work.
    if (DEBUG_LOOP_PHASES) {
        loopPhases.debugDump(debugSink, "loopPhases");
    }
    telemetry.update(); // the complete-frame owner commits once
    loopPhases.finishPhase("telemetry");

    loopPhases.finishCycle(clock);
}
```

Leave `DEBUG_LOOP_PHASES` false during ordinary robot operation. When it is true,
`snapshot()` and `debugDump(...)` report only fully completed cycles. Because telemetry is rendered
before its own phase can finish, that phase first appears on the next telemetry frame. Use stable
literal names such as `"localization"` and `"tasks"`; do not generate one name per device, route,
or target.

The reported seconds are monotonic elapsed wall time. They can include operating-system scheduling
pauses and profiler overhead; they are not CPU time and do not cover work before `startCycle` or
after `finishCycle`, so they are not the complete FTC callback period. If the same profiler has
already collected data when the composition root deliberately resets its `LoopClock`, call
`loopPhases.reset()` at that boundary while no profiler cycle is active, as the example does.

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
`holdAfterReference(value)` writes the graph-owned command before the same downstream Plant update;
`resumeTargeting()`, timeout, and cancellation preserve the existing command. In every case the
Plant phase still evaluates the appropriate owner-held state once.

---

## 8. Telemetry and debug output

Telemetry should not control robot behavior.

Treat one Driver Station update as a composed frame:

1. Required presenters and renderers add the mode, safety, remediation, and high-level status rows
   drivers need during normal operation.
2. The composition root may add one or more explicitly selected optional diagnostics.
3. The owner of that complete frame calls `telemetry.update()` exactly once.

Additive presenters and renderers do not call `clear()`, `clearAll()`, or `update()`. This lets an
Auto host, route adapter, robot presenter, and diagnostic contributor share one frame without their
call order accidentally hiding or prematurely committing another contributor's rows. During the
active robot loop, the composition root normally owns that final commit. An outer OpMode still owns
and commits frames that do not enter the active loop, such as INIT selection, construction failure,
or start-error pages. A dedicated tester may likewise commit its exclusive screen when it is the
complete-frame owner.

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

The disabled `TeleOp_01` through `TeleOp_06` teaching examples intentionally enable several verbose
dumps when a student explicitly launches one; they expose the related framework surfaces together
for exploration. Treat that as tutorial instrumentation, not a match-robot default. Adopted and
match robot code should default optional topics off and enable only the owner or owners relevant to
the current investigation.

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
  choose `.leaveTargetOnCancel()`) or use `Tasks.waitUntil(...)`, then run it in a `TaskRunner`.

### Mistake: missing edges by not sampling

Edge/toggle trackers like `risingEdge()` and `toggled()` only advance when they are sampled.

If you create a `BooleanSource` edge/toggle and then <em>don’t read it every loop</em>, you can miss transitions that happened in between.

Fix: make sure edge/toggle sources are sampled once per loop (e.g., by wiring them into `Bindings.update(clock)`, a drive pipeline, or telemetry that runs every loop).

### Mistake: double-running task updates

If two helpers both call `macroRunner.update(clock)`, Phoenix prevents double-advancement in the same cycle — but the better design is still: update it exactly once, in your main loop.

### Mistake: making a route Task the only follower heartbeat

If a vendor follower owns pose, hold, callbacks, or final drive output after a route execution
becomes terminal, do not stop updating it when `RouteTask` completes. Keep the composition-root
heartbeat running and let the adapter deduplicate the Task-facing update. Do not infer success from
the vendor becoming not busy; use the retained `RouteExecution.status()` instead.

---

## 10. Summary

Phoenix’s loop structure is intentionally boring:

* one clock
* one update order
* no hidden time steps

Stick to:

> Clock → Sensors → Bindings → Tasks → Drive → Plants → Telemetry

and the rest of the framework will behave predictably.
