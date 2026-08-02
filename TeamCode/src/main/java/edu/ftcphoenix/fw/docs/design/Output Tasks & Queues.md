# Output Tasks & Queues

Phoenix has two common ways to express mechanism behavior over time:

1. **Tasks that change a persistent scalar command target** (`ScalarTasks`)
2. **Tasks that produce a temporary scalar output** (`OutputTask` + `OutputTaskRunner`)

This document is about the second pattern. Use it when a short behavior should temporarily influence a Plant target without becoming a second Plant writer.

For the broader robot-design context, read [`Recommended Robot Design`](<Recommended Robot Design.md>) and [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>).

---

## 1. The core idea

A source-driven Plant invokes one final `PlantTargetResolver` each loop:

```text
behavior sources / queues / overlays
        ↓
one final PlantTargetResolver
        ↓
Plant.update(clock)
```

An `OutputTask` does **not** write a Plant. It proposes a temporary scalar output. The subsystem then uses `PlantTargets.overlay(...)` to decide whether that output overrides the normal baseline target. A temporary conditional layer never becomes the Plant's command target; only a `ScalarTarget` carried by the overlay base does.

That keeps the target-resolver ownership rule intact:

```text
many things may propose target values
one PlantTargetResolver arbitrates
one Plant consumes the final target
```

---

## 2. `OutputTask`: a Task that proposes a scalar output

An `OutputTask` is still a normal non-blocking Phoenix `Task`:

- `start(clock)` once
- `update(clock)` once per loop
- `isComplete()` ends it

The object itself is single-use once it has attempted `start(clock)`. Do not put an already-started
or completed pulse object back into a queue; create a fresh pulse from its `OutputTaskFactory`.
Reusing the same object fails with a clear error instead of silently skipping or partially
restarting output behavior.

It also exposes:

- `getOutput()` — the scalar output for this loop

Timed output phases measure from the loop timestamp at which that phase begins, not from the
preceding loop's `dtSec()`. When a positive-duration pulse's start gate opens, its run output is
available to the downstream overlay/Plant phase in that loop before the pulse can finish. A
zero-duration run is an immediate no-output operation; any configured cooldown still follows it.

Common examples:

- feed one game piece into a shooter
- pulse a transfer servo
- run an eject motor for a short window
- wait until a readiness gate opens, then run an output until a sensor changes

---

## 3. `OutputTaskRunner`: queue + scalar source

`OutputTaskRunner` runs `OutputTask`s sequentially and also implements `ScalarSource`.

```java
OutputTaskRunner feederQueue = Tasks.outputQueue(0.0); // idle output
```

The queue can be sampled anywhere a `ScalarSource` is expected:

```java
ScalarSource queuedOutput = feederQueue;
BooleanSource queueActive = feederQueue.activeSource();
```

Call `feederQueue.update(clock)` once per loop before updating Plants that depend on its current output. Sampling `feederQueue.getAsDouble(clock)` also advances the queue, but explicit `update(clock)` keeps loop order easier to read.

---

## 4. Standard realization pattern: base target + queued override

Use `PlantTargets.overlay(...)` when a queued output should temporarily override a baseline target.
In ordinary robot code, the queue and Plant are private fields of one mechanism; the resolver and
Plant below are constructed once inside that mechanism's `HardwareMap` + config constructor. They
are not rebuilt in the loop and are not assembled by the composition root.

```java
// Mechanism fields
private final OutputTaskRunner feederQueue = Tasks.outputQueue(0.0);
private final Plant transfer;

// Mechanism constructor
ScalarSource baseTransferTarget = ScalarSource.of(() -> stagingEnabled ? 0.20 : 0.0);

PlantTargetResolver finalTransferTarget = PlantTargets.overlay(baseTransferTarget)
        .add("feedPulse", feederQueue.activeSource(), feederQueue)
        .build();

transfer = FtcActuators.plant(hardwareMap)
        .crServo("transfer", Direction.FORWARD)
        .power()
        .targetFromResolver(finalTransferTarget)
        .build();
```

Here the supplied resolver has no command target: its base is a read-only `ScalarSource`, and the
queue supplies an enabled overlay layer. `targetFromResolver(...)` also accepts framework graphs
whose base carries a `ScalarTarget`; those Plants expose that recognized command identity through
`commandTarget()`.

Then the loop remains simple:

```java
feederQueue.update(clock);
transfer.update(clock);
```

The queue proposes. `PlantTargets.overlay(...)` arbitrates. The Plant invokes the final target resolver.

---

## 5. Guided pulse recipes

A pulse is common enough that Phoenix provides a staged builder:

```java
OutputTaskFactory feedOne = Tasks.outputPulse("feedOne")
        .startWhen(canShootNow)
        .runOutput(0.90)
        .forSeconds(0.12)
        .cooldownSec(0.05)
        .build();
```

The builder asks the stable robot questions in order:

```text
When may this pulse start?
What output does it produce?
How does it end?
Does it need cooldown time?
```

It returns an `OutputTaskFactory`, not a single task, because queued tasks are single-use. The
mechanism/supervisor that owns the private queue also retains this factory and calls
`feedOne.create()` or `feedOne.get()` for every enqueue; each call creates a fresh pulse task.

### Sensor-ended pulse

When a sensor can prove that the mechanism finished moving a game piece, use `until(...)` with a max-run safety cap:

```java
BooleanSource ballAtGate = gateDistanceCm
        .hysteresisBelow(6.0, 7.0)
        .debouncedOnOff(0.05, 0.05)
        .memoized();

BooleanSource ballLeftGate = ballAtGate.fallingEdge();

OutputTaskFactory feedOne = Tasks.outputPulse("feedOne")
        .startWhen(canShootNow.and(ballAtGate))
        .runOutput(0.90)
        .until(ballLeftGate)
        .minRunSec(0.05)
        .maxRunSec(0.30)
        .cooldownSec(0.05)
        .build();
```

The max-run cap is required for sensor-ended pulses so a failed or disconnected sensor cannot leave an output running forever.

### Timed fallback

When there is no reliable done sensor, use a timed pulse:

```java
OutputTaskFactory feedOne = Tasks.outputPulse("feedOne")
        .startWhen(canShootNow)
        .runOutput(0.90)
        .forSeconds(0.12)
        .build();
```

`Tasks.outputForSeconds(...)` still exists for low-level one-off tasks, but `Tasks.outputPulse(...)` is the preferred student-facing pattern because it captures start gates, output, ending policy, and cooldown in one guided builder.

For a positive `.forSeconds(...)` duration, the run output is observable in at least the gate-open
loop even when the next loop arrives after the requested duration. Time elapsed before the gate
opened never shortens the pulse.

---

## 6. Repeating while a request is held

Inside the mechanism or supervisor that owns the private queue, use queue-level `whileHigh(...)` /
`whileLow(...)` to keep a bounded backlog while a semantic request signal has the desired level.
Controls set the request through a robot-owned method; they do not receive the queue.

```java
private boolean continuousFeedRequested;
private final BooleanSource requestShoot =
        BooleanSource.of(() -> continuousFeedRequested);

public void setContinuousFeedRequested(boolean requested) {
    continuousFeedRequested = requested;
}

public void requestSingleFeed() {
    feederQueue.enqueue(feedOne.create());
}

// In the owning mechanism/supervisor update:
feederQueue.whileHigh(
        clock,
        requestShoot,
        1,       // keep exactly one active-or-queued pulse while the request is high
        feedOne  // OutputTaskFactory creates fresh tasks
);
```

Important separation:

- The `requestShoot` signal says whether the driver or auton wants shots.
- The pulse's `startWhen(...)` gate says when feeding is actually safe.
- The final `PlantTargets.overlay(...)` says how the active pulse affects the Plant target.

When `requestShoot` goes low, `whileHigh(...)` cancels and clears the queue. This prevents old pulses from firing after the operator changed modes.

`whileLow(...)` is the exact signal-level mirror: maintain backlog while the signal is low, cancel and clear while it is high. Phoenix uses `high`/`low` vocabulary consistently with input bindings (`onRise`, `onFall`, `whileHigh`, `whileLow`).

---

## 7. TeleOp and autonomous reuse

TeleOp and Auto reuse the same semantic capability methods and status. The mechanism/supervisor
keeps readiness Sources, pulse factories, and its queue private.

TeleOp maps a trigger to the held request:

```java
bindings.mirrorOnChange(
        gamepads.p2().rightTrigger().above(0.50),
        scoring::setContinuousFeedRequested);
```

Autonomous can request exactly one pulse through the same capability vocabulary:

```java
Task feedOneTask = Tasks.runOnce(scoring::requestSingleFeed);
```

Autonomous can also wait on the capability's status snapshot:

```java
Task waitForReady = Tasks.waitUntil(() -> scoring.status().canShoot(), 2.0);
```

No duplicate sensor or queue logic is needed.

---

## 8. Aborting an output queue

When a driver lets go of a trigger, changes mode, or an autonomous step is interrupted, prefer:

```java
public void cancelTransientActions() {
    feederQueue.cancelAndClear();
}
```

That is implementation inside the queue owner. TeleOp and Auto call
`scoring.cancelTransientActions()` rather than manipulating `feederQueue` directly.

`cancelAndClear()` is the total-abort operation: it cooperatively cancels the active output Task,
always discards every queued Task, invalidates any cached active output, and reports the queue's
configured idle value. Queued Tasks have not started and receive no cancellation callback. There is
no abrupt queue-forgetting operation that can silently abandon active work.

The same fail-stop rule applies if start, update, completion checking, or cancellation throws a
`RuntimeException`. The runner best-effort cancels active or partially started work, clears all
owned work and cached output, then rethrows the original failure with any cleanup failure
suppressed.

Typical abort situations:

- the driver released the trigger that requested repeated pulses
- the robot switched modes and a queued pulse is no longer safe
- a supervisor detected that the mechanism is jammed and wants the active task to stop cleanly

---

## 9. When to use ScalarTasks vs Output tasks

Use **ScalarTasks** when:

- one task should change a Plant's persistent command target, or an intentionally standalone/shared
  `ScalarTarget`
- the task may wait until its logical command path wins and Plant feedback confirms physical arrival
- the behavior is naturally “move this mechanism target and wait”

For an ordinary exact Plant, write
`ScalarTasks.set(arm.commandTarget(), SCORE).untilReachedBy(arm)...build()`. The Plant accessor
returns its stable command without sampling hardware or changing state; the feedback branch validates
that the Plant follows that exact command. Keep a separately named `ScalarTarget` when it is useful
on its own or while assembling a shared, overlay, equivalent-position, or advanced target graph.

Use **OutputTaskRunner** when:

- continuous baseline behavior and short temporary outputs both influence the same final Plant target
- you need sensor-gated pulses that work in TeleOp and auton
- a single logical pulse should fan out into multiple Plants through scaled overlay layers

Rule of thumb:

```text
ScalarTasks.set(...) changes a command target.
OutputTaskRunner proposes a temporary output.
PlantTargets.overlay(...) decides the final Plant target.
```
