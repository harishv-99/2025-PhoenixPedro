# Tasks and Macros

This is the detailed guide to Sushi **Tasks**: non-blocking behaviors that continue over several
robot loops. If this is your first Task, read
[`Tasks and autonomous`](<../getting-started/learn-sushi/Tasks and Autonomous.md>) before using
this reference.

Tasks are used for:

* TeleOp **macros** (e.g., shooting sequences).
* **Autonomous routines** built out of reusable pieces.

We assume you already have an ordinary robot owner wired like the
[`Basic Mechanisms Robot`](<../getting-started/Basic Mechanisms Robot.md#complete-source-and-owner-map>):

* An `FtcRobotOpMode` receiving one framework-created `RobotProgram`.
* Gamepad meanings declared through `program.callbackBindings()` or `program.taskBindings()`.
* Drive and Plant-owning mechanisms declared as the program's downstream outputs.

Everything here is **non-blocking**: there is no `sleep()` and no long-running `while` loop that
stalls TeleOp. Spatial zones and heading predicates are a separate topic in
[`Spatial Queries`](<../drive-vision/Spatial Queries.md>).

---

## 1. The big picture: Tasks coordinate behavior

Sushi Task behavior is built around three ideas:

1. **Tasks** – small, reusable behaviors that run over time.
2. **Plants** – things that accept a numeric target (motors, servos, etc.).
3. **TaskRunner** – drives a queue of tasks every loop; an ordinary `RobotProgram` owns it
   privately.

You describe *what* should happen as a graph of tasks, and the framework figures out *when* each piece runs.

Typical ordinary TeleOp declaration:

```java
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

public class MyTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        ScoringMechanism scoring = program.output(
                new ScoringMechanism(hardwareMap, profile.scoring));
        GamepadDevice driver = new GamepadDevice(gamepad1);

        program.taskBindings().onRise(driver.y(), scoring::createShootOneTask);
        program.drive(driveSource(driver), FtcDrives.mecanum(hardwareMap, profile.drive));
    }
}
```

`GamepadDevice` belongs to the FTC input boundary because its constructor accepts the SDK gamepad.
The Task and binding APIs see ordinary Sushi sources, and robot code uses the direct
`new GamepadDevice(gamepad1)` construction call.

This is a complete FTC lifecycle shape: the framework starts and advances its private runner after
bindings, then updates declared outputs and commits presenters. STOP and runtime failure actively
cancel the runner before outputs stop. The managed
[`ReferenceLauncherMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.html>)
shows a fresh outcome-aware macro without exposing custom-host ceremony.

In ordinary TeleOp, Tasks do decision work, update sources, request Plant targets, or manage a
temporary calibration-search recipe. They run before the one final `DriveSource` writer and the
mechanism-owned Plant phase shown above; they do not write imperatively to the drive sink or call a
Plant's `update(clock)` themselves.

The rest of this guide is about how to create those `Task` objects in a friendly way.

---

## 2. Core interfaces (Task, TaskRunner, TaskOutcome)

### 2.1 `Task`

At its core, a task looks like this:

```java
public interface Task {
    void start(LoopClock clock);
    void update(LoopClock clock);
    void cancel();
    boolean isComplete();

    TaskOutcome getOutcome();
}
```

Rules:

* `start(...)` and `update(...)` must **return quickly** (no blocking).
* A `Task` object is **single-use**: it may enter `start(...)` once. To repeat behavior, call the
  macro/builder again or use a `Supplier<Task>` so each run receives a fresh object. For bounded
  clock-aware repetition, `Tasks.repeatWhileSuccessful(...)` invokes such a factory for each
  admitted child. Framework Tasks throw a clear error on a second start; custom Task
  implementations must honor the same contract.
* Use fields inside the task to remember your own state.
* `isComplete()` becomes `true` when the task is done.
* `cancel()` is the cooperative early-stop hook. Before `start(...)` it is a side-effect-free
  no-op; while active it makes the Task terminal; after completion and on repeated calls it is a
  no-op. Every custom Task must implement this method so cancellation policy is explicit at
  compile time, including an intentional no-op for a Task that cannot remain active.
* Calling `update(...)` directly before `start(...)` is a lifecycle error. Framework Tasks report
  it with an actionable exception instead of silently starting or mutating state. `Tasks.noop()`
  is the intentional exception because it is already successfully complete when created.
* `getOutcome()` is part of every Task implementation. A simple Task that deliberately does not
  track a richer result may return `TaskOutcome.UNKNOWN`. Outcome-aware Tasks normally return
  `NOT_DONE` while active, then a terminal value such as `SUCCESS`, `TIMEOUT`, or `CANCELLED`.
* If a custom task owns a duration or timeout, capture `clock.nowSec()` when that interval starts.
  On a first `update(...)` that shares the start cycle, the current `dtSec()` began before
  `start(...)` and must not be counted as task runtime. The advanced `RunForSecondsTask.onUpdate`
  callback receives that unchanged clock too, so callback-owned timers follow the same rule.

### 2.2 `TaskRunner`

`TaskRunner` is a simple scheduler:

* It keeps an internal queue of `Task`s.
* You call `runner.enqueue(task)` to add new work.
* It rejects the same object if that identity is already current or queued.
* Each loop you call `runner.update(clock)`, and it:

    * Starts the next task when the previous one completes.
    * Calls `start(...)` once and then `update(...)` every loop.

The first `update(...)` may happen immediately after `start(...)` in the same runner call. Sushi's
timed factories account for that: a long loop immediately before scheduling does not consume the
new task's duration. Positive-duration output and Plant requests remain available to their later
realization phase for at least one loop. The exclusive timed-drive helper instead publishes directly
when its Task starts and on each active cycle, so a positive interval is observable without a
competing downstream drive writer. A zero-duration interval itself is immediate, though an
explicitly configured follow-up or cooldown still runs.

Ordinary robot code does not construct or subclass `TaskRunner`; it declares an Auto root with
`program.rootTask(...)` and input-created work with `program.taskBindings()`. A custom host,
framework tool, or mechanism that genuinely owns a private bounded queue may construct a runner and
must own its complete cancellation lifecycle.

Inside an explicitly owned custom/private runner, use `cancelAndClear()` for driver override, mode
switches, route interruption, and other total aborts. The managed program calls it automatically at
terminal shutdown. It asks the active Task to stop, if there is one, and always discards every
queued Task.
Queued Tasks have not acquired resources and do not receive a pre-start cancellation callback.
There is no abrupt queue-forgetting operation that can silently abandon the active Task.

The runners also fail closed on lifecycle `RuntimeException`s. If starting, updating, checking
completion, or cancelling active work throws one, the runner detaches and best-effort cancels that
work, clears all pending work, resets its state, and rethrows the original exception. A cleanup
exception is attached as suppressed rather than replacing the original failure.

Aborting a runner or cancelling a Task does not make a started Task reusable. If the behavior is
requested later, enqueue a new Task built by the same macro method or factory.

Advanced explicit-host example:

```java
bindings.onRise(gamepads.p1().b(), runner::cancelAndClear);
```

That calls the active Task's cooperative cancellation path first, then forgets any queued follow-up
work. The runner is empty afterward even if the cancellation hook fails.

---

## 3. Start with the factories: `Tasks` helpers

For most code, you should use the **factory methods** in `edu.ftcsushi.fw.task.Tasks` instead of directly `new`‑ing task classes.

This keeps robot code readable:

```java
Task auto = Tasks.sequence(
    // Wait until shooter is spun up.
    Tasks.waitUntil(() -> shooterReady()),

    // This simple Auto is the only drive-command writer while the interval runs.
    DriveTasks.driveExclusivelyForSeconds(drivebase, forwardSignal, 0.8),

    // Small pause before the next step.
    Tasks.waitForSeconds(0.5)
);
```

Common factories (high‑level view):

* **Instant behavior**

    * `Tasks.runOnce(Runnable action)` – run once in `start(...)`, then complete.
    * `Tasks.noop()` – do nothing and complete immediately.

* **Time‑based waits**

    * `Tasks.waitForSeconds(double seconds)` – wait a fixed duration.

* **Condition‑based waits**

    * `Tasks.waitUntil(BooleanSupplier condition)` – wait until a boolean becomes true.
    * `Tasks.waitUntil(BooleanSupplier condition, double timeoutSec)` – wait until true or time out.

* **Composition**

    * `Tasks.repeatWhileSuccessful(String debugName, int maxIterations,
      BooleanSource mayStartIteration, Supplier<? extends Task> taskFactory)` – admit and run at
      most that many fresh Tasks, continuing only after exact success.
    * `Tasks.sequence(Task... steps)` – run tasks in order, starting the next only after exact
      success.
    * `Tasks.sequenceOnCompletion(Task... steps)` – explicitly run a later recovery or repair step
      after any valid natural terminal outcome.
    * `Tasks.parallelAll(Task... steps)` – run tasks in parallel, wait for all, and succeed only
      when all succeed.
    * `Tasks.parallelDeadline(Task deadline, Task... companions)` – run bounded companions only
      while one named Task is active; that deadline determines completion and outcome.
    * `Tasks.withTimeout(Task task, double timeoutSec)` – impose one hard outer time budget on a
      complete Task or Task graph.
    * `Tasks.branchOnOutcome(Task move, Task onSuccess, Task onTimeout)` – select only the exact
      success or timeout branch; cancellation and unknown outcomes fail closed.

> `Tasks.*` is the one public construction layer for generic composition and ordinary leaf Tasks,
> including `runOnce(...)`, `waitUntil(...)`, `sequence(...)`, `sequenceOnCompletion(...)`,
> `parallelAll(...)`,
> `parallelDeadline(...)`, `repeatWhileSuccessful(...)`, `withTimeout(...)`, and
> `branchOnOutcome(...)`.
> `RunForSecondsTask` remains directly constructible because its start/update/finish callback
> capability is distinct from a fixed wait.

### 3.1 Choose how a sequence advances

For ordinary dependent steps, use `sequence(...)`:

```java
Task score = Tasks.sequence(
        lift.home(),
        lift.moveTo(BasicLift.Height.HIGH),
        Tasks.runOnce(() -> claw.setState(BasicClaw.State.OPEN))
);
```

The child Tasks are constructed when this graph is built, but they start one at a time. Only exact
`SUCCESS` starts the next child. `TIMEOUT`, naturally reported `CANCELLED`, or `UNKNOWN` stops the
sequence immediately and becomes its exact outcome. This makes the short form safe for normal
prerequisite chains. A child that succeeds immediately may start the next child in the same
lifecycle call; a newly started positive-duration child is not updated again by that same call.

Use `sequenceOnCompletion(...)` only when a later recovery, takeover, or repair step intentionally
follows a valid natural abnormal result:

Assume `boundedPrePark` owns an explicit hard budget and `park` is the required takeover attempt:

```java
Task attemptParkAfterBoundedWork = Tasks.sequenceOnCompletion(boundedPrePark, park);
```

This form runs later children after `SUCCESS`, `TIMEOUT`, natural `CANCELLED`, or `UNKNOWN`, then
retains the first non-success outcome after the later work settles. Direct cancellation of the
outer graph and lifecycle failures remain terminal and never start a later child. It is not Java
`finally`; mandatory safety still belongs in the active Task's cancellation path, a persistent safe
request, or owner `stop()`.

Both sequence factories receive eagerly constructed, single-use child Tasks. Use
`Tasks.buildAtStart(...)` when construction itself needs a live pose, vision result, earlier outcome,
or another fact that is knowable only when that phase starts.

When success and timeout require different actions, use `branchOnOutcome(...)`. Exact `SUCCESS`
selects the success branch and exact `TIMEOUT` selects the timeout branch. `CANCELLED` and `UNKNOWN`
start neither branch. The selected branch's validated result becomes the wrapper result, so a
successful timeout branch can handle a timeout without erasing the retained domain fact.

### 3.2 Choose the parallel owner explicitly

Use `parallelAll(...)` when every child is required before the group can finish. It waits for every
child rather than failing fast. It reports `SUCCESS` only when every child succeeds; one kind of
non-success result remains exact, while mixed non-success kinds report `UNKNOWN`. Use
`parallelDeadline(...)` when one Task owns the lifetime of bounded companion work:

```java
Task collectAlongRoute = Tasks.parallelDeadline(
        followRoute,
        collectWhileMoving
);
```

The first argument is called the deadline because it owns completion, not because it must be a
timer. When `followRoute` ends, its outcome becomes the group's outcome and the composite asks the
start-attempted companions to cancel before they receive another update. A companion that finishes
early does not end the route.

Each companion must already make active cancellation safe. Do not use
`Tasks.sequence(enable, wait, disable)` as a companion: cancelling that sequence skips `disable`.
Build a bounded robot macro whose own `cancel()` restores its caller-selected state, or keep a
long-lived flywheel/intake/aim request as ordinary capability or service state.

Route failure policy is still outside generic composition. `Tasks.sequence(...)` now stops unless a
child reports exact `SUCCESS`, but broad `TaskOutcome` values do not replace a route's precise
status or choose a fallback. Keep the returned `RouteTask`; use `branchOnOutcome(...)` or a
robot-owned routine when timeout should start a named fallback or several route statuses need
different strategy. Direct cancellation never starts the fallback. Cleanup belongs to the phase
that created the request and goes through robot capabilities, not direct Plant or hardware writes.

### 3.3 Admit bounded fresh attempts between iterations

Use `repeatWhileSuccessful(...)` when one policy wants a bounded series of fresh attempts and can
decide whether another attempt may start:

```java
Task attempts = Tasks.repeatWhileSuccessful(
        "adaptiveCollection.attempts",
        MAX_ATTEMPTS,
        clock -> clock.nowSec() < LATEST_NEW_ATTEMPT_SEC
                && (latestAttempt == null || latestAttemptPermitsAnother()),
        this::buildFreshAttemptTask
);
```

The admission source is checked once before every proposed child, including the first. If the first
decision is false, the wrapper completes with `SUCCESS` without invoking the factory. A true
decision invokes the factory once and starts its fresh child. After that child reports exact
`SUCCESS`, the wrapper waits for a later `LoopClock.cycle()` before it may check admission for the
next child. Reaching `MAX_ATTEMPTS` or receiving a false later decision also completes the wrapper
with `SUCCESS`.

Only exact child `SUCCESS` permits another decision. A child ending with `TIMEOUT`, `CANCELLED`, or
`UNKNOWN` stops the wrapper and becomes its exact outcome; there is no next condition sample or
factory call. A terminal child reporting `null` or `NOT_DONE`, a null/reused/self child from the
factory, or a lifecycle callback that throws is a contract failure. The wrapper fails closed and
does not admit another attempt.

The wrapper invokes at most one factory and starts at most one child in a shared clock cycle. That
remains true when the parent starts and is first-updated in the same cycle, when a child completes
inside `start(...)`, and on repeated or reentrant calls. A repeated same-cycle update is a no-op; a
reentrant lifecycle callback fails closed instead of creating or restarting a child. Active
cancellation makes the wrapper terminal and cancels only its current start-attempted child;
cancellation before start, after completion, or while no child is active has no child side effects.

This is a **soft admission gate**. It can refuse a new attempt based on time, inventory, or the
latest attempt's exact robot-owned status, but it cannot interrupt an attempt that is already
running. It is deliberately not an arbitrary retry, active-child race, or `finally` facility:
unsuccessful children are not restarted, and takeover/continuation policy stays explicit in the
surrounding Task graph.

### 3.4 Put a required continuation outside its hard time budget

Use `withTimeout(...)` when a parent owns a hard budget around one complete Task or composite. A
bounded Auto normally combines the soft latest-start rule above with a distinct hard takeover:

```java
Task preParkWork = Tasks.sequence(preload, attempts);
Task boundedPrePark = Tasks.withTimeout(preParkWork, PARK_TAKEOVER_ELAPSED_SEC);

RouteTask<MyRoute> park = RouteTasks.followBuiltAtStart(
        "park",
        routeFollower,
        parkPaths::buildFromCurrentPose,
        PARK_ROUTE_TIMEOUT_SEC
);

Task auto = Tasks.sequenceOnCompletion(boundedPrePark, park);
program.rootTask(auto);
```

The timeout contains **all** pre-park work, including preload, so its elapsed budget begins with
the root at FTC START. If that graph finishes early, the sequence starts park immediately. If it is
still active, the first lifecycle call at or after `PARK_TAKEOVER_ELAPSED_SEC` cancels the active
pre-park graph and starts park only after cancellation returns and the direct timed child reports
terminal. Every nested Task is still required to honor the framework rule that active cancellation
makes it terminal. This is a software guarantee at that cooperative Task boundary, not proof that
the physical robot will finish parking by the end of the match.

Park is outside the timeout, so the old pre-park cutoff cannot cancel or restart it. The
`followBuiltAtStart(...)` supplier reads the live pose and builds exactly one park route when that
Route Task starts; do not wrap it in another `Tasks.buildAtStart(...)` layer.

The explicit `sequenceOnCompletion(...)` advances to park after every valid natural
`boundedPrePark` outcome. That includes early success and timeout takeover, and is appropriate only
when the robot's declared strategy is to attempt park in every such case. It retains the timeout as
the enclosing sequence outcome even if park later succeeds. Direct cancellation of the outer
sequence—including FTC STOP—never starts park. A lifecycle or cleanup exception propagated by the
pre-park graph also fails closed and suppresses park. A custom nested Task that silently returns
from cancellation while remaining active violates the Task contract; a terminal composite cannot
prove that hidden descendant state. This is a bounded continuation, not Java `finally`; mandatory
physical cleanup belongs in the active owner's cancellation behavior or persistent capability
state.

Keep references to `boundedPrePark`, the repetition Task, the latest attempt's exact status, and the
park `RouteTask`. The completion-continuing sequence retains its first abnormal Task outcome, but
that result is still too coarse to replace the attempt exit reason, the timeout wrapper diagnostics,
or the park's exact `RouteStatus`.

If only success should continue, use ordinary `sequence(...)`. If different domain results need
different continuations, keep the timed A/B shape inside a robot-owned policy coordinator or an
explicit outcome branch. Direct cancellation still must not launch fallback.

An outer timeout is intentionally different from operation-owned timeouts. A route-local timeout
can retain `RouteStatus.TASK_TIMEOUT`; a feedback move can apply its timeout target; a gated output
can time only its RUN phase. `withTimeout(...)` instead uses the child's normal cancellation path,
so the wrapper may report `TIMEOUT` while the retained child reports `CANCELLED`. Keep both when
they protect different scopes, but do not configure two copies of the same policy.

---

## 4. Mechanisms: `ScalarTasks` for common patterns

When the capability request is itself a number, such as flywheel velocity or lift position, start
with `edu.ftcsushi.fw.actuation.ScalarTasks`. A simple exact mechanism retains one Plant variable
and creates its command inline:

The builder below is a construction-time excerpt from the mechanism owner. In ordinary FTC robot
code, that constructor receives `HardwareMap` and its active data-only config, copies and validates
the complete snapshot before its own hardware lookup, and uses the copied hardware name and
direction. The composition root owns cross-owner policy and constructs the mechanism, not this raw
Plant. See the compiling
[`BasicPedroAutoMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/pedro/capability/intake/BasicPedroAutoMechanism.html>)
for a complete numeric-target owner. This pattern is intentionally separate from the Starter
intake's named `Mode` vocabulary.

```java
private final Plant roller;

// In the RollerMechanism constructor:
this.roller = FtcActuators.plant(hardwareMap)
        .motor(snapshot.motorName, snapshot.direction)
        .power()
        .targetFromNewCommand(0.0)
        .build();
```

The source-driven plant rule is:

```text
Task changes the Plant's stable ScalarTarget
    ↓
Plant invokes its final PlantTargetResolver to select the requested target
    ↓
Plant bounds and target guards select the applied target
    ↓
Plant sends the actuator command; a feedback-capable Plant can also report physical arrival
```

`roller.commandTarget()` returns that same stable target without sampling or touching hardware. Use
it directly for immediate numeric behavior (`roller.commandTarget().set(...)`) and deferred Task
behavior (`ScalarTasks.set(roller.commandTarget(), ...)`). A feedback branch validates that the
supplied Plant follows that exact target, so a mismatched standalone or graph-owned target fails
when the Task is constructed rather than silently waiting on the wrong mechanism.

Calling `ScalarTasks.set(...)` or `build()` does not change the target. The built Task writes when
it starts. Each `build()` creates a fresh single-use Task, so retain the Plant and rebuild the Task
or macro for every later run. Keep a separately named `ScalarTarget` only when it is standalone,
shared, owned by target-only policy, or useful while assembling a composed target graph. If that
target is the Plant's complete exact graph, bind it through
`targetFromResolver(PlantTargets.exact(target))`.

This direct path is for a scalar-complete request: the number itself is the public meaning. If a
capability instead names intent with `Height`, `Mode`, or another semantic value, its mechanism
owns one `SemanticScalarCommand` that maps and atomically publishes the named/numeric pair. Direct
methods and Tasks call its semantic setter and read the composed semantic/Plant snapshot; the helper
exposes no `ScalarTarget`, so raw `ScalarTasks` cannot change the number behind the owner's status.

#### Calibration-search handoff

`PositionCalibrationTasks.search(positionPlant)` is a narrow Task recipe for homing or indexing.
It acquires and releases the temporary search mode, samples the cue, establishes the reference, and
requests the temporary output stop; it never updates the Plant or reads or writes its persistent
command. In an ordinary program, the private runner phase comes before the registered mechanism
output, which advances its private Plants once. A custom host must preserve that same
Task-before-mechanism order explicitly.

`.withPower(...)` requires a finite normalized command in the inclusive `[-1.0, +1.0]` range and
rejects `NaN`, infinities, and overshoot at that builder step. It does not clamp a bad recipe into a
different command. This validates the number's shape only; robot code must still choose a safe
magnitude and direction and verify the cue and mechanism setup.

The plant-unit answer to `establishReferenceAt(...)` must also be finite. It rejects `NaN` or
infinity at that builder step without clamping or overwriting an earlier accepted answer. A
reference defines the coordinate map and need not lie inside the Plant's target range; it is not a
target command.

Success, timeout, and active cancellation all release temporary search ownership while preserving
the persistent command and final resolver. Post-reference policy belongs to the mechanism that owns
the request. A named-height lift can use ordinary exact-success composition:

```java
Task search = PositionCalibrationTasks.search(lift)
        .withPower(homingPower)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .failAfterSec(homingTimeoutSec)
        .build();

return Tasks.sequence(
        search,
        Tasks.runOnce(() -> setHeight(Height.STOWED)));
```

There is no initial STOWED write: temporary search ownership already suspends normal target
realization. Exact success starts the semantic setter in the same lifecycle callback before the
downstream Plant phase. Timeout and cancellation skip it and retain the prior—or any during-search
superseding—coherent semantic/numeric request. These numeric checks do not prove the robot is
physically at the declared reference or that its later request is safe.

### 4.1 Guided writes for time-based commands

Use `ScalarTasks.set(target, value).forSeconds(...)` for open-loop pulses and timed target writes.
These work whether or not a Plant has feedback.

```java
Task rollerPulse = ScalarTasks.set(roller.commandTarget(), +1.0)
        .forSeconds(0.7)
        .then(0.0)
        .build();
```

Behavior:

* At start: write `+1.0` to the Plant's command target.
* For `0.7` seconds: keep writing `+1.0`.
* When time elapses: write `0.0` once and complete.

The `.then(0.0)` also runs if this timed write is cancelled while active. That makes the command
target return to zero; the Plant still resolves that request on its next update. If the Task is
still queued and never started, discarding it has no target side effects.

The `0.7` seconds begin when this task starts. Even if the preceding loop was unusually long, the
new `+1.0` target is still available for the following `plant.update(clock)` call before a positive
duration may finish.

Do not use a numeric target to reconstruct a named capability state. The Starter intake publishes
its requested `Mode` and mapped power through one `SemanticScalarCommand`, and builds its timed
action with `RunForSecondsTask` callbacks that call the same setter. This keeps
the capability-shaped `status().mode()` and `status().requestedPower()` truthful while the one-line
robot call remains
`intake.collectForSeconds(durationSec)`.

To hold and leave the target there:

```java
Task ensureSpinUp = ScalarTasks.set(shooter.commandTarget(), SHOOTER_VELOCITY_NATIVE)
        .forSeconds(0.5)
        .leaveThere()
        .build();
```

The ending choice is required: `.leaveThere()` keeps the held request after normal completion and
active cancellation, while `.then(...)` writes the selected final request in both cases.

To set once and finish immediately:

```java
Task stopShooter = ScalarTasks.set(shooter.commandTarget(), 0.0).build();
```

### 4.2 Guided feedback moves

Continue the same `ScalarTasks.set(...)` path with `.untilReachedBy(plant)` when a Task should wait
for physical arrival.

```java
Task spinUp = ScalarTasks.set(shooter.commandTarget(), SHOOTER_VELOCITY_NATIVE)
        .untilReachedBy(shooter)
        .cancelTo(0.0)
        .timeout(1.5)
        .build();
```

Behavior:

* On start: write the requested value to the Plant's command target.
* Each loop: require the move's logical command path to be selected and check physical arrival at
  the target resolved from it.
* For an ordinary exact source, that is equivalent to
  `plant.atTarget(SHOOTER_VELOCITY_NATIVE)`.
* For `PlantTargets.equivalentPositionsOf(...)`, a logical value such as `20` may resolve to a
  physical target such as `380`; the same Task waits for `380` without changing its logical
  `set(turretTarget, 20)` call.
* If a behavior overlay, clamp, fallback, hold, or target guard wins, the task does **not** complete
  early—even when that path has the same numeric logical value.
* If the timeout elapses first, the task completes with `TaskOutcome.TIMEOUT`.
* If actively cancelled, the shooter command target changes to `0.0` once.

For readiness that must remain stable for a short period:

```java
Task spinUpStable = ScalarTasks.set(shooter.commandTarget(), SHOOTER_VELOCITY_NATIVE)
        .untilReachedBy(shooter)
        .leaveTargetOnCancel()
        .stableFor(0.15)
        .timeout(1.5)
        .build();
```

Every feedback move must choose one cancellation behavior immediately after
`.untilReachedBy(plant)`:

* `.cancelTo(value)` writes that finite value, in the Plant's units, to its command target when
  an active move is cancelled.
* `.leaveTargetOnCancel()` deliberately leaves the move request unchanged, so motion may continue.

Neither choice is a direct hardware stop. The Plant's next update still resolves overlays and
applies bounds, references, and guards. The robot owner must still cancel related queues, disable
overlays, and reset every mechanism request needed for coordinated shutdown.

To request a final target after success or timeout:

```java
Task spinAndStop = ScalarTasks.set(flywheel.commandTarget(), SHOT_RPM)
        .untilReachedBy(flywheel)
        .cancelTo(STOPPED_RPM)
        .timeout(1.0)
        .thenTarget(STOPPED_RPM)
        .build();
```

Here the numeric RPM is the complete request. `.thenTarget(...)` handles success or timeout, while
`.cancelTo(...)` independently handles active cancellation.

> If `untilReachedBy(...)` receives an open-loop Plant or a Plant commanded by a different
> `ScalarTarget`, construction throws an actionable exception. Use the timed branch for open-loop
> behavior.

### 4.3 The same path for standalone scalar targets

Because the API is target-rooted, the same `ScalarTasks` entry also works when the writable value is
not attached to a Plant, such as a behavior variable or a command target that feeds a larger source
graph.

```java
Task setShotVelocity = ScalarTasks.set(selectedVelocity, 3200.0).build();
```

---


## 5. TeleOp macros: putting it together

A typical TeleOp macro flows like this:

1. Driver presses a button.
2. A macro `Task` is created (often a `Tasks.sequence(...)`).
3. `program.taskBindings()` enqueues it into the private `TaskRunner`.
4. Each active program loop advances it before downstream outputs.

### 5.1 Wiring a simple shooter macro

Inside the shooter mechanism, retain the two simple Plants as private fields:

* `shooter` – a velocity command and feedback Plant for the flywheel.
* `transfer` – a power command and Plant that feeds discs.

A semantic Task factory on that owner could look like:

```java
private final Plant shooter;
private final Plant transfer;

public Task createShootOneDiscTask() {
    Task spinUp = ScalarTasks.set(shooter.commandTarget(), SHOOTER_VELOCITY_NATIVE)
            .untilReachedBy(shooter)
            .cancelTo(0.0)
            .timeout(SHOOTER_SPINUP_TIMEOUT_SEC)
            .build();

    Task feed = ScalarTasks.set(transfer.commandTarget(), TRANSFER_POWER_SHOOT)
            .forSeconds(TRANSFER_PULSE_SEC)
            .then(0.0)
            .build();

    Task holdBeforeSpinDown = ScalarTasks.set(shooter.commandTarget(), SHOOTER_VELOCITY_NATIVE)
            .forSeconds(SHOOTER_SPINDOWN_HOLD_SEC)
            .leaveThere()
            .build();

    Task spinDown = ScalarTasks.set(shooter.commandTarget(), 0.0).build();

    return Tasks.sequence(
        spinUp,
        feed,
        holdBeforeSpinDown,
        spinDown
    );
}
```

Bind a button to enqueue this macro:

```java
program.taskBindings().onRise(shootButton, scoring::createShootOneDiscTask);
```

`scoring` is the semantic mechanism/capability visible to controls; its raw Plants stay private.
The program owns binding dispatch, Task advancement, and then `scoring.update(clock)` in its output
phase.

---

## 6. Autonomous routines with Tasks

The same patterns work in Autonomous: build one fresh root graph during configuration and declare
it with `program.rootTask(auto)`. The framework starts and first-updates it at the exact FTC START
boundary, then owns its later advancement and cancellation.

For a simple open-loop Auto or drive tester, the exclusive timed-drive helper is appropriate only
when no route Task, guidance Task, or separate final `DriveSource` loop is also writing behavior
commands to the same sink:

```java
Task auto = Tasks.sequence(
    // 1. Drive off the starting line.
    DriveTasks.driveExclusivelyForSeconds(drivebase, forwardSignal, 1.0),

    // 2. Wait until shooter is ready.
    Tasks.waitUntil(() -> shooterReady()),

    // 3. Shoot three discs with a macro.
    scoring.createShootThreeDiscTask()
);

program.rootTask(auto);
```

`driveExclusivelyForSeconds(...)` refreshes the sink and writes the requested signal on every active
cycle, then stops the sink when the interval completes or is actively cancelled. If an adapter's
supported lifecycle requires updates beyond active Tasks, its composition root continues calling
`update(clock)` with the shared `LoopClock`; same-cycle deduplication makes the Task's call safe.
Timed open-loop drive is not normal Pedro route movement—use `RouteTasks` or guidance Tasks for
those supported lifecycles.

Because everything is non‑blocking, your loop can also update telemetry, vision, etc., while the auto sequence runs.

---

## 7. When to use a lower-level or custom Task

For **most** robots, you only need:

* `Tasks.*` factory methods.
* `ScalarTasks.set(...)` for write-once, timed, and feedback-aware scalar-complete commands.
* `DriveTasks.driveExclusivelyForSeconds(...)` only for simple Auto/test movement with exclusive
  drive-sink ownership.

Use the matching `Tasks` helper for ordinary leaf behavior rather than directly constructing its
implementation class. One public generic leaf remains because it exposes a distinct capability:

* `RunForSecondsTask` – when you want full control over what happens during the time window (custom callbacks each loop).

Even inside a team-specific helper factory, compose child Tasks through `Tasks.*`, such as
`sequence(...)`, `sequenceOnCompletion(...)`, `parallelAll(...)`, `parallelDeadline(...)`,
`repeatWhileSuccessful(...)`, `withTimeout(...)`, or `branchOnOutcome(...)`. Implement
`Task` directly only when the behavior genuinely needs a new state machine rather than another
spelling of existing composition.

A good rule of thumb:

> Try the factories (`Tasks`, `ScalarTasks`, and the narrowly scoped `DriveTasks` helper) first. If you find yourself rewriting the same pattern many times using raw `Task` classes, wrap it in a new helper method so the next student can just call the helper.

---

## 8. Output tasks and queues

Sometimes you want **continuous logic** (e.g. staging balls, holding a gate, keeping a mechanism ready)
*and* short **pulses** (feed one ball, spit out wrong color, etc.).

If both pieces of code try to own the same target variable, they will fight.

Sushi provides a clean target-resolver ownership pattern:

- **`OutputTask`** — a `Task` that produces a scalar output (`getOutput()`).
- **`OutputTaskRunner`** — runs `OutputTask`s sequentially and exposes the active output as a `ScalarSource`. Use `cancelAndClear()` when you need to abort the active output task cleanly. This is the right default for feed queues, pulse queues, and “repeat while held” helpers because it lets the current task stop cooperatively, always clears the queue, and returns the source to its configured idle output even when task lifecycle cleanup fails.

That lets your mechanism decide the final Plant target in one place. The queue, resolver, and Plant
are long-lived fields. Build the resolver and Plant once in the mechanism constructor; never build
the graph in the loop:

```java
// Mechanism fields
private final OutputTaskRunner feederQueue = Tasks.outputQueue(0.0);
private final OutputTaskFactory feedOne;
private final Plant transferShooterPlant;

// In the mechanism constructor, after copying and validating config, then creating inputs:
FeederConfig snapshot = Objects.requireNonNull(config, "config").copy();
this.feedOne = Tasks.outputPulse("feedOne")
        .startWhen(fireAllowed)
        .runOutput(0.90)
        .until(ballLeftGate)
        .minRunSec(0.05)
        .maxRunSec(0.30)
        .build();
PlantTargetResolver finalTarget = PlantTargets.overlay(0.0)
        .add("feedPulse", feederQueue.activeSource(), feederQueue)
        .build();

this.transferShooterPlant = FtcActuators.plant(hardwareMap)
        .crServo(snapshot.transferServoName, snapshot.transferDirection)
        .power()
        .targetFromResolver(finalTarget)
        .build();

// In the mechanism update:
// Keep one "feedOne" buffered while the driver requests shooting.
// The task waits in WAIT until fireAllowed is true.
feederQueue.whileHigh(
        clock,
        requestShoot,
        1,
        feedOne
);
feederQueue.update(clock);
transferShooterPlant.update(clock);
```

Here `FeederConfig` is illustrative robot code, not a framework type; `snapshot` is the mechanism's
defensive, owner-validated copy of that data-only config. A compiling default is not physical
hardware or safe-motion evidence.
`fireAllowed`, `requestShoot`, and `ballLeftGate` are long-lived mechanism/supervisor inputs created
during construction; only their sampling, queue advancement, and Plant update happen each cycle.

For the full design rationale and more examples, see [`Output Tasks & Queues`](<Output Tasks & Queues.md>) and [`Recommended Robot Design`](<Recommended Robot Design.md>).

---

## 9. Common gotchas

* **Do not block** inside `Task.start(...)` or `Task.update(...)`.

    * Never call `Thread.sleep(...)` or spin in `while(!condition)` loops.
    * Use `Tasks.waitForSeconds(...)` or `Tasks.waitUntil(...)` instead.

* **Do not save and restart a Task object.**

    * Task instances are single-use, including sequences and parallel groups.
    * Save a macro method, `Supplier<Task>`, or `OutputTaskFactory` and ask it for a fresh task each
      time instead.
    * For bounded successful repetition, pass that fresh factory to
      `Tasks.repeatWhileSuccessful(...)`; do not return the same Task identity twice.

* **Do not confuse admission time with takeover time.**

    * A `repeatWhileSuccessful(...)` condition can prevent another child from starting, but it
      cannot cancel the child already running.
    * Put all work that must yield by a hard cutoff inside `withTimeout(...)`, then put a required
      continuation after that wrapper in the outer sequence.

* **Give every runner exactly one lifecycle owner.**

    * An ordinary `RobotProgram` owns its private runner automatically.
    * A deliberate custom host or private bounded-queue owner must call its runner once per loop
      and cancel it on every shutdown/failure path.

* **Use the feedback branch only with the Plant that follows that target.**

    * `ScalarTasks.set(target, value).untilReachedBy(plant)` requires feedback and validates that
      `plant.commandTarget() == target`.
    * For servos and other open-loop outputs, use `.build()` for one write or
      `.forSeconds(...)` with an explicit `.then(...)`/`.leaveThere()` ending.

* **Do not bypass a named semantic request.**

    * If status names a `Height`, `Mode`, or semantic pose, call the mechanism's one setter from
      both direct behavior and Tasks, then wait on the mechanism's capability-shaped status. The
      mechanism may retain the exact request returned by its private `SemanticScalarCommand` at
      Task start so same-valued supersession cannot complete old work; ordinary clients should not
      have to navigate that backing request identity.
    * Use `ScalarTasks` directly only when the number itself is the complete request.

* **Be intentional about completion and cancellation targets.**

    * `.forSeconds(...).leaveThere().build()` keeps the same target after the timer.
    * `.forSeconds(...).then(...)` sets a different final target after time elapses and on active
      cancellation.
    * Every feedback move requires `.cancelTo(...)` or `.leaveTargetOnCancel()` immediately after
      `.untilReachedBy(...)`; `.thenTarget(...)` remains the success/timeout choice.

---

## 10. Summary

* **`Task`** is the basic unit of non‑blocking behavior over time.
* **`TaskRunner`** manages a queue of tasks; ordinary robot code reaches the program-owned runner
  through `rootTask(...)` and `taskBindings()` rather than constructing it.
  `cancelAndClear()` is the total-abort operation and lifecycle failures clear owned work before
  they are rethrown.
* **`Tasks` factories** (`runOnce`, `waitForSeconds`, `waitUntil`, `sequence`,
  `sequenceOnCompletion`, `parallelAll`, `parallelDeadline`, `repeatWhileSuccessful`,
  `withTimeout`, `noop`, ...)
  are the main building blocks you should reach for first. Normal prerequisite chains use
  `sequence`; only intentional recovery/repair continuation uses `sequenceOnCompletion`.
* **`ScalarTasks.set(target, value)`** is the direct deferred-write path when the request itself is
  numeric: build it directly, add a timed branch, or add a feedback-aware branch.
* **`DriveTasks.driveExclusivelyForSeconds(...)`** provides simple timed open-loop Auto/test movement
  when its Task is the sole behavior-command writer for the drive sink.
* Ordinary leaf Tasks and generic composition use `Tasks.*`; `RunForSecondsTask` remains public for
  its distinct start/update/finish callback capability.
* TeleOp macros and Autonomous routines both use the same task patterns; only the triggers change (buttons vs. init/start).

Once you are comfortable with these patterns, you can layer in more advanced pieces like vision, DriveGuidance (auto-aim / go-to), and interpolated shooter speeds – they all compose naturally on top of the same Task/Plant/Drive structure.
