# Output Tasks & Queues

Phoenix has two common ways to express mechanism behavior over time:

1. **Tasks that write a Plant's registered `ScalarTarget`** (`PlantTasks`)
2. **Tasks that produce a temporary scalar output** (`OutputTask` + `OutputTaskRunner`)

This document is about the second pattern. Use it when a short behavior should temporarily influence a Plant target without becoming a second Plant writer.

For the broader robot-design context, read [`Recommended Robot Design`](<Recommended Robot Design.md>) and [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>).

---

## 1. The core idea

A source-driven Plant follows one final `ScalarSource` target each loop:

```text
behavior sources / queues / overlays
        ↓
one final ScalarSource
        ↓
Plant.update(clock)
```

An `OutputTask` does **not** write a Plant. It proposes a temporary scalar output. The subsystem then uses a source-composition tool such as `ScalarOverlayStack` to decide whether that output overrides the normal baseline.

That keeps the single-writer rule intact:

```text
many things may propose target values
one ScalarSource arbitrates
one Plant consumes the final target
```

---

## 2. `OutputTask`: a Task that proposes a scalar output

An `OutputTask` is still a normal non-blocking Phoenix `Task`:

- `start(clock)` once
- `update(clock)` once per loop
- `isComplete()` ends it

It also exposes:

- `getOutput()` — the scalar output for this loop

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

Use `ScalarOverlayStack` when a queued output should temporarily override a baseline target.

```java
OutputTaskRunner feederQueue = Tasks.outputQueue(0.0);

ScalarSource baseTransferTarget = ScalarSource.of(() -> stagingEnabled ? 0.20 : 0.0);

ScalarSource finalTransferTarget = ScalarOverlayStack.on(baseTransferTarget)
        .add("feedPulse", feederQueue.activeSource(), feederQueue)
        .build();

Plant transfer = FtcActuators.plant(hardwareMap)
        .crServo("transfer", Direction.FORWARD)
        .power()
        .targetedBy(finalTransferTarget)
        .build();
```

Then the loop remains simple:

```java
feederQueue.update(clock);
transfer.update(clock);
```

The queue proposes. The overlay arbitrates. The Plant follows the final source.

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

It returns an `OutputTaskFactory`, not a single task, because queued tasks are single-use. Each call to `feedOne.create()` or `feedOne.get()` creates a fresh pulse task.

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

---

## 6. Repeating while a request is held

Use queue-level `whileHigh(...)` / `whileLow(...)` to keep a bounded backlog while a request signal has the desired level.

```java
BooleanSource requestShoot = gamepads.p2().rightTrigger().above(0.50);

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
- The final `ScalarOverlayStack` says how the active pulse affects the Plant target.

When `requestShoot` goes low, `whileHigh(...)` cancels and clears the queue. This prevents old pulses from firing after the operator changed modes.

`whileLow(...)` is the exact signal-level mirror: maintain backlog while the signal is low, cancel and clear while it is high. Phoenix uses `high`/`low` vocabulary consistently with input bindings (`onRise`, `onFall`, `whileHigh`, `whileLow`).

---

## 7. TeleOp and autonomous reuse

Because readiness, sensors, and queue outputs are all Sources, TeleOp and autonomous code can reuse the same pieces.

TeleOp can maintain a queue while a trigger is held:

```java
feederQueue.whileHigh(clock, requestShoot, 1, feedOne);
```

Autonomous can enqueue exactly one pulse:

```java
feederQueue.enqueue(feedOne.create());
```

Autonomous can also wait on the same readiness signal:

```java
Task waitForReady = Tasks.waitUntil(canShootNow, 2.0);
```

No duplicate sensor logic is needed.

---

## 8. Aborting an output queue

When a driver lets go of a trigger, changes mode, or an autonomous step is interrupted, prefer:

```java
feederQueue.cancelAndClear();
```

Use plain `clear()` only when you intentionally want to forget queue state without invoking cancellation hooks. Most robot code should reach for `cancelAndClear()`.

Typical abort situations:

- the driver released the trigger that requested repeated pulses
- the robot switched modes and a queued pulse is no longer safe
- a supervisor detected that the mechanism is jammed and wants the active task to stop cleanly

---

## 9. When to use PlantTasks vs Output tasks

Use **PlantTasks** when:

- one task should change a Plant's registered writable target
- the task may wait on Plant feedback using `plant.atTarget(value)`
- the behavior is naturally “move this mechanism target and wait”

Use **OutputTaskRunner** when:

- continuous baseline behavior and short temporary outputs both influence the same final Plant target
- you need sensor-gated pulses that work in TeleOp and auton
- a single logical pulse should fan out into multiple Plants through scaled overlay layers

Rule of thumb:

```text
PlantTasks change a command target.
OutputTaskRunner proposes a temporary output.
ScalarOverlayStack decides the final Plant target.
```
