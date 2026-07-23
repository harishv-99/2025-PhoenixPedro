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
* Gamepad axes/buttons are exposed as {@code ScalarSource}/{@code BooleanSource} and are sampled when you call {@code get(...)}.
* Edge/toggle trackers (e.g., {@code risingEdge()}, {@code toggled()}) must be sampled each loop to avoid missing transitions.
  {@code Bindings.update(clock)} does that sampling for the bindings you register.

**Bindings before Tasks**

* Bindings typically enqueue tasks (macros).
* If you enqueue in this cycle, you usually want the runner to see it immediately.

**Tasks before Drive and Plants**

* Ordinary Tasks are the “decision layer”: they update behavior sources or request Plant targets.
* The Drive/Plants phases are the “actuation layer” that consumes those decisions.
* In TeleOp, the Drive phase is the one final behavior-command writer. Tasks must not also write
  imperatively to the same drive sink.

**Drive before Plants**

* Drive is often the most time-sensitive and can benefit from being updated early.
* More importantly: `MecanumDrivebase.update(clock)` provides loop timing to its rate limiters.

**Telemetry last**

* Telemetry is slow and should never influence state updates.

---

## 3. A canonical loop template

Below is a typical Phoenix loop (TeleOp or Auto). This is intended as a reference pattern.

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
    drivebase.update(clock);   // ensure rate limiting uses current dt
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

That is the normal TeleOp ownership model: Tasks finish their decision/source/Plant-target work,
then the one final Drive phase samples the composed `DriveSource` and writes the sink.

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

During OpMode cleanup, the composition root calls `stop()` on each haptic sink it constructed.
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

### 4.1 Sources, edges, and memoization are cycle-idempotent

Phoenix uses {@link edu.ftcphoenix.fw.core.time.LoopClock#cycle()} to make many source wrappers safe to read multiple times in the same loop.

Examples:

* `ScalarSource.memoized()` / `BooleanSource.memoized()`
* `BooleanSource.risingEdge()` / `fallingEdge()`
* `BooleanSource.toggled()`

These wrappers only advance once per cycle <em>when sampled</em>. If you never sample an edge/toggle source during a cycle, it cannot observe that transition.

For buttons, the most common way to ensure sampling is to register a binding and call `Bindings.update(clock)` every loop.

### 4.2 Bindings are idempotent

`Bindings.update(clock)` is guarded so it will not fire actions twice in a cycle.

Optional `Bindings.ControlContext` objects are declaration groups, not additional heartbeat owners.
Do not update them separately; the parent `bindings.update(clock)` samples every context's
activation once before dispatching any callbacks and uses that snapshot for the complete input
frame. If a callback changes a mode used by a context, the new eligibility starts on the next loop.
The root's same-cycle idempotency covers both root and contextual registrations.

### 4.3 TaskRunner is idempotent

`TaskRunner.update(clock)` will not advance the current task twice in one loop cycle.

This is critical because advancing tasks twice effectively doubles loop speed and breaks timeouts.

### 4.4 Stateful external followers need a persistent owner

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
* Tasks and drive rate limiters use `dtSec()`.

Avoid these patterns:

* calling `System.nanoTime()` or `ElapsedTime` deep inside tasks to “self-time”
* calling `getRuntime()` inside multiple subsystems and letting each compute its own dt

If you need time, take it from the `LoopClock`.

### 5.1 Optional loop-phase diagnostics

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

    drivebase.update(clock);
    drivebase.drive(driveSource.get(clock).clamped());
    loopPhases.finishPhase("drive");

    shooter.update(clock);
    loopPhases.finishPhase("plants");

    // During this cycle, this displays the previous completed cycle.
    loopPhases.debugDump(debugSink, "loopPhases");
    telemetry.update();
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

## 6. Rate limiting and the drive update call

`MecanumDrivebase` can rate-limit axial/lateral/omega based on the most recent `update(clock)`.

Guideline:

* If you use rate limiting, call:

  ```java
  drivebase.update(clock);
  drivebase.drive(signal);
  ```

Calling `drive(signal)` before `update(clock)` means the rate limiter uses the previous cycle’s dt.

Also remember the semantics:

* `drivebase.drive(...)` sends power commands to the hardware **immediately**.
* `drivebase.update(clock)` only provides timing (`dtSec`) for optional rate limiting; it does not move motors by itself.

---

## 7. Plants: update is mandatory

Plants are stateful actuators. Tasks typically set targets on plants, but targets do not apply “by magic.”

Your loop must call:

```java
plant.update(clock);
```

for each plant you care about.

Typical pattern:

* store plants as fields
* update them every loop

---

## 8. Telemetry and debug output

Telemetry should not control robot behavior.

If you need structured debug output, prefer Phoenix debug sinks:

* many subsystems expose `debugDump(dbg, prefix)`

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

* use `PlantTasks.move(plant).to(...).cancelTo(...).build()` (or deliberately choose
  `.leaveTargetOnCancel()`) or use `Tasks.waitUntil(...)`, then run it in a `TaskRunner`.

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
