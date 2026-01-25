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

> **Clock → Inputs → Bindings → Tasks → Drive → Plants → Telemetry**

### 2.1 Why this order?

**Clock first**

* Many subsystems depend on `dtSec()` and `cycle()`.
* Updating the clock once defines “this loop cycle” for every component.

**Inputs before Bindings**

* Button edges (`onPress()`, `onRelease()`) are computed during input update.
* Bindings should read those edges after they’ve been refreshed.

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

    // 2) Inputs
    gamepads.update(clock);

    // 3) Bindings (may enqueue macros)
    bindings.update(clock);

    // 4) Tasks / macros
    macroRunner.update(clock);

    // 5) Drive
    DriveSignal cmd = driveSource.get(clock).clamped();
    drivebase.update(clock);   // ensure rate limiting uses current dt
    drivebase.drive(cmd);      // applies motor power immediately

    // 6) Plants (mechanisms)
    double dtSec = clock.dtSec();
    shooter.update(dtSec);
    transfer.update(dtSec);
    pusher.update(dtSec);

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

### 4.1 Buttons are globally polled once per cycle

Buttons use a global registry and an update gate:

* `Button.updateAllRegistered(clock)` updates edge state once per cycle.

`Gamepads.update(clock)` calls this for you.

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
plant.update(clock.dtSec());
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
while (!shooter.atSetpoint()) { }
```

Good:

* use `PlantTasks.moveTo(...)` or `Tasks.waitUntil(...)` and run it in a `TaskRunner`.

### Mistake: consuming button edges in two places

If you call `Button.updateAllRegistered(clock)` manually *and* also call `gamepads.update(clock)`, that’s okay because it’s idempotent — but you should still treat `gamepads.update(clock)` as the canonical input update.

### Mistake: double-running task updates

If two helpers both call `macroRunner.update(clock)`, Phoenix prevents double-advancement in the same cycle — but the better design is still: update it exactly once, in your main loop.

---

## 10. Summary

Phoenix’s loop structure is intentionally boring:

* one clock
* one update order
* no hidden time steps

Stick to:

> Clock → Inputs → Bindings → Tasks → Drive → Plants → Telemetry

and the rest of the framework will behave predictably.
