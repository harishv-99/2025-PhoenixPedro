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

* Tasks are the “decision layer” that sets targets.
* Drive/Plants are the “actuation layer” that consumes the latest targets.

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

### 4.3 TaskRunner is idempotent

`TaskRunner.update(clock)` will not advance the current task twice in one loop cycle.

This is critical because advancing tasks twice effectively doubles loop speed and breaks timeouts.

---

## 5. Where time comes from (and where it must not come from)

Phoenix is intentionally strict about time:

* `LoopClock.update(getRuntime())` defines `dtSec()`.
* Tasks and drive rate limiters use `dtSec()`.

Avoid these patterns:

* calling `System.nanoTime()` or `ElapsedTime` deep inside tasks to “self-time”
* calling `getRuntime()` inside multiple subsystems and letting each compute its own dt

If you need time, take it from the `LoopClock`.

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

* use `PlantTasks.move(plant).to(...).build()` or `Tasks.waitUntil(...)` and run it in a `TaskRunner`.

### Mistake: missing edges by not sampling

Edge/toggle trackers like `risingEdge()` and `toggled()` only advance when they are sampled.

If you create a `BooleanSource` edge/toggle and then <em>don’t read it every loop</em>, you can miss transitions that happened in between.

Fix: make sure edge/toggle sources are sampled once per loop (e.g., by wiring them into `Bindings.update(clock)`, a drive pipeline, or telemetry that runs every loop).

### Mistake: double-running task updates

If two helpers both call `macroRunner.update(clock)`, Phoenix prevents double-advancement in the same cycle — but the better design is still: update it exactly once, in your main loop.

---

## 10. Summary

Phoenix’s loop structure is intentionally boring:

* one clock
* one update order
* no hidden time steps

Stick to:

> Clock → Sensors → Bindings → Tasks → Drive → Plants → Telemetry

and the rest of the framework will behave predictably.
