# Output Tasks & Queues

Phoenix has two common ways to express "behavior over time":

1. **Tasks that directly write Plants** (`PlantTasks`)
2. **Tasks that *produce an output* you apply in your loop** (`OutputTask` + `OutputTaskRunner`)

This document describes option (2), which is the recommended pattern when you want:

For the broader robot-design context, read [`Recommended Robot Design`](<Recommended Robot Design.md>) and [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>).


- sensor‑gated, reactive automation in **TeleOp** (intake/shooter pipelines, staged feeding)
- the *same* logic to work in **autonomous** without duplicating sensor rules
- to avoid multiple pieces of code "fighting" by writing targets to the same Plant

---

## 1. The problem: multiple writers fight

In a real robot, you usually have:

- a **continuous base behavior** (hold something, stage balls forward, keep a mechanism ready)
- occasional **short overrides** (pulse a feeder, spit out a bad game piece, jog a transfer)

If both behaviors directly call `plant.setTarget(...)`, you get conflicts:

- your base loop sets target to 0
- a macro sets target to +1
- base loop runs again and immediately sets it back to 0

To solve this cleanly, Phoenix introduces **output‑producing tasks**.

---

## 2. `OutputTask`: a Task that proposes a scalar output

An `OutputTask` is still a normal `Task`:

- `start(clock)` once
- `update(clock)` once per loop
- `isComplete()` ends it

…but it also exposes:

- `double getOutput()` — the output value for this loop

**Key idea:** the task does **not** write hardware. It only proposes an output.

---

## 3. `OutputTaskRunner`: TaskRunner + output, exposed as a ScalarSource

`OutputTaskRunner` runs `OutputTask`s sequentially (FIFO), using the same scheduling rules as `TaskRunner`.

It also implements `ScalarSource`, so you can treat its output like any other signal:

```java
OutputTaskRunner feederQueue = new OutputTaskRunner(0.0); // idle output

// Later: enqueue a pulse
feederQueue.enqueue(Tasks.outputForSeconds("feed", 0.9, 0.12));

// In your loop
feederQueue.update(clock);
double feederOverride = feederQueue.getAsDouble(clock);
```

### Why this matters

You can now enforce a **single writer** for the Plant:

- base target comes from continuous logic (Sources)
- override target comes from an OutputTaskRunner
- one place decides the final target

---

## 4. Common pattern: base target + queued override

A typical plant update becomes:

```java
// 1) Update your queues first
feederQueue.update(clock);

// 2) Compute a base target (continuous behavior)
double base = 0.0; // e.g. staging logic

// 3) Apply a priority rule
// (You can use hasActiveTask(), or the signal-friendly activeSource() helper.)
boolean overrideActive = feederQueue.activeSource().getAsBoolean(clock);
double finalTarget = overrideActive ? feederQueue.getAsDouble(clock) : base;

transferShooterPlant.setTarget(finalTarget);
```

That priority rule is explicit, debuggable, and easy to change.

---

## 5. Gated feeding: sensors optional

Phoenix includes a generic gating task:

- `GatedOutputUntilTask`

It has three phases:

1. WAIT: output idle until `startWhen` becomes true
2. RUN: output run value until `doneWhen` is true (and min time elapsed)
3. COOLDOWN (optional): output idle for a short cooldown

### Sensor-based example (distance sensor at the shooter gate)

```java
BooleanSource shooterReady = PlantSources.atSetpoint(shooterPlant).debouncedOn(0.15);
BooleanSource aimLocked = ...; // from your auto-aim error source + hysteresis/debounce
BooleanSource fireAllowed = shooterReady.and(aimLocked);

BooleanSource ballAtGate = gateDistanceCm
        .hysteresisBelow(6.0, 7.0)
        .debouncedOnOff(0.05, 0.05)
        .memoized();

// "done" when the ball leaves the gate
BooleanSource ballLeftGate = ballAtGate.fallingEdge();

OutputTask feedOne = Tasks.gatedOutputUntil(
        "feedOne",
        fireAllowed.and(ballAtGate),   // startWhen
        ballLeftGate,                  // doneWhen
        0.90,                           // run output
        0.05,                           // min run sec
        0.30                            // max run sec (safety cap)
);

feederQueue.enqueue(feedOne);
```

### Fast repeat shooting: keep one "feed" buffered while held

To shoot quickly while maintaining accuracy, a common pattern is:

1. driver holds a trigger to request shots
2. you <b>only feed</b> when (a) aimed and (b) shooter is stably at speed and (c) a ball is present
3. you keep exactly <b>one</b> feed task buffered so the next shot starts immediately

Phoenix provides a small helper on {@code OutputTaskRunner}:

```java
BooleanSource requestShoot = gamepads.p2().rightTrigger().above(0.50);

// Readiness lives inside the task. That way, the task can sit in WAIT without output,
// and you still keep one "feedOne" buffered and ready to run.
BooleanSource canShootNow = fireAllowed.and(ballAtGate);
BooleanSource ballLeftGate = ballAtGate.fallingEdge();

feederQueue.whileTrue(
        clock,
        requestShoot,
        1, // keep exactly one task buffered
        () -> Tasks.gatedOutputUntil(
                "feedOne",
                canShootNow,   // startWhen
                ballLeftGate,  // doneWhen
                0.90,
                0.05,
                0.30
        )
);

// Note: whileTrue(...) only manages the backlog. When the request goes false it uses cancelAndClear()
// so the active output task can stop cleanly before the queue is emptied.
// Your loop still needs to advance the queue each cycle by calling:
//   feederQueue.update(clock)
// or by sampling its output via feederQueue.getAsDouble(clock).
```

This keeps your loop logic small and prevents "queue spam" (hundreds of pulses queued up)
when a trigger is held.

### Sensorless fallback (no done condition)

```java
OutputTask feedPulse = Tasks.outputForSeconds("feedPulse", 0.9, 0.12);
feederQueue.enqueue(feedPulse);
```

---

## 6. Auton reuse: wait on the same signals

Because readiness and ball‑present logic are `BooleanSource`s, auton can wait on them without
re‑implementing them:

```java
Task waitForReady = Tasks.waitUntil(fireAllowed, 2.0);
```

And auton can request shots by enqueueing the same `OutputTask`s your TeleOp automation uses.

---

## 6.5 Aborting an output queue

When a driver lets go of a trigger, changes mode, or an autonomous step is interrupted, prefer:

```java
feederQueue.cancelAndClear();
```

Use plain `clear()` only when you intentionally want to forget queue state without invoking
task cancellation hooks. Most robot code should reach for `cancelAndClear()`.

Typical abort situations:

- the driver released the trigger that requested repeated pulses
- the robot switched modes and a queued pulse is no longer safe
- a supervisor detected that the mechanism is jammed and wants the active task to stop cleanly

---

## 7. When to use PlantTasks vs Output tasks

Use **PlantTasks** when:

- a macro "owns" a plant for a while
- you are writing a simple autonomous sequence
- it is okay for a task to directly set targets

Use **OutputTaskRunner** when:

- you have continuous logic + short overrides
- you need sensor‑gated pulses that work in TeleOp and auton
- you want to avoid multi‑writer conflicts

