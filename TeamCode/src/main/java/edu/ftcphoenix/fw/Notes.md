# Phoenix Notes, Legacy, and Migration Guide

This document collects **advanced notes**, **legacy details**, and **migration tips** for the Phoenix framework.

It is mainly for mentors and framework maintainers. New students should not need anything here to write robot code.

---

## 1. Purpose of this document

Over time, Phoenix has evolved:

* Older versions used base classes like `PhoenixTeleOpBase`, `PhoenixAutoBase`, and `Subsystem`.
* There were experiments with concepts like `Stage`, `MacroSlot`, and more opinionated schedulers.
* The input layer changed from `DriverKit` to a simpler `Gamepads` + `Bindings` design.

Current docs and examples use a **PhoenixRobot + thin OpMode + Tasks + Plants** pattern. This document:

* Explains what the older pieces were trying to do.
* Clarifies which parts are kept only for **backward compatibility**.
* Provides concrete migration recipes.

If you are working only with new code, you can safely treat this file as optional reading.

---

## 2. Legacy base classes: PhoenixTeleOpBase, PhoenixAutoBase, Subsystem

### 2.1 What they were

Earlier Phoenix versions encouraged teams to:

* Extend `PhoenixTeleOpBase` for TeleOp.
* Extend `PhoenixAutoBase` for Autonomous.
* Organize robot code into `Subsystem` classes.

These base classes handled:

* Some lifecycle wiring.
* Common configuration and debugging.
* A coarse form of subsystem separation.

### 2.2 Why we moved away

Over time, several issues became clear:

1. **OpModes were harder for students to reason about**

    * Important behavior was inherited from base classes rather than written explicitly.
    * Understanding control flow required reading multiple layers of inheritance.

2. **Subsystems encouraged over‑fragmentation**

    * Small teams often ended up with many small classes.
    * Logic and state were spread out, making it harder to see “the whole robot.”

3. **Tight coupling to framework base classes**

    * OpModes that extended framework classes were less flexible.
    * Swapping in new patterns (e.g., different schedulers) was harder.

### 2.3 Current recommendation

* New code should use:

    * A **single season robot class** (e.g., `PhoenixRobot`) that composes plants, drive, sensors, bindings, and tasks.
    * Plain FTC OpModes (`OpMode` / `LinearOpMode`) that construct and delegate to the robot class.
* `PhoenixTeleOpBase`, `PhoenixAutoBase`, and `Subsystem` remain for **backward compatibility** and for old examples that haven’t been migrated.

If you are maintaining old code, it is fine to keep it working as is. For new students and new robots, prefer the newer pattern documented in **Beginner’s Guide** and **Framework Overview**.

---

## 3. Legacy Stage / MacroSlot / "safe idle" ideas

### 3.1 Stage

There was an experimental "Stage" concept intended to:

* Represent high‑level modes or phases of robot behavior.
* Encapsulate their own logic, tasks, and permitted actions.

In practice, this overlapped heavily with what **Tasks** already provide:

* Multi‑step behavior over time.
* Clear lifecycle (`start` / `update` / `isFinished`).

The extra abstraction layer made it harder to understand where logic lived, especially for beginners.

### 3.2 MacroSlot and safe idle

The idea behind `MacroSlot` (and similar constructs) was roughly:

* Provide a dedicated place to run a single macro at a time.
* Enforce certain safety rules when no macro is running ("safe idle").

These concerns are now better expressed as:

* A plain `TaskRunner` that owns the currently running Task.
* Normal robot logic that defines what "idle" means for plants (e.g., zero power) when no task is active.
* Interlock / safety behavior implemented directly in **plant wrappers** (e.g., `InterlockPlant`) or in the robot logic.

### 3.3 Current status

* Stage / MacroSlot abstractions are **not recommended** for new code.
* They may still appear in older files or experiments; treat them as historical.
* Prefer:

    * Plain `Task` + `TaskRunner` for scheduling.
    * Plant wrappers (rate limit, interlock) for safety.
    * Clear robot logic for normal vs. macro behavior.

---

## 4. DriverKit → Gamepads and Bindings

### 4.1 What DriverKit tried to do

`DriverKit` was an earlier attempt to:

* Encapsulate player devices (P1, P2).
* Provide helper methods for common control patterns.

Over time, it became clear that it overlapped with the concept of:

* A **global Gamepads manager** (`Gamepads`), and
* Per‑robot **Bindings** for input mapping.

### 4.2 Current approach

The framework now uses:

* `Gamepads` – updates and exposes `GamepadDevice` objects (P1, P2, etc.).
* `Bindings` – attaches actions to button events (`onPress`, `onRelease`, `whileHeld`, etc.).

This is simpler and more explicit:

* Control mappings live in one place (`configureBindings()` on your robot).
* There is a clear separation between "what buttons exist" and "what they do."

### 4.3 Migration notes

* Replace calls that used `DriverKit.Player` with `Gamepads.player1()`, `Gamepads.player2()`, etc.
* Where `DriverKit` offered convenience wrappers, consider whether they are still needed:

    * If yes, recreate them as small helper methods in your **robot class**, not in a global driver utility.
    * If no, delete them.

---

## 5. Migration recipes

This section gives concrete suggestions for moving older code to the newer patterns.

### 5.1 From Subsystem to PhoenixRobot

**Old pattern:**

* Separate `Subsystem` classes for drive, shooter, arm, etc.
* TeleOp/Auto extend `PhoenixTeleOpBase` / `PhoenixAutoBase` and own instances of each subsystem.

**New pattern:**

* A single season robot class (e.g., `PhoenixRobot`), which:

    * Has fields for drive, shooter, arm, etc.
    * Wires hardware, plants, bindings, and tasks in one place.
    * Exposes `updateTeleOp(dt)` and `updateAuto(dt)`.

**Migration strategy:**

1. **Create PhoenixRobot** and move subsystem fields into it as normal fields.
2. **Inline simple subsystems** directly into PhoenixRobot:

    * If a subsystem class is thin, fold its fields and methods in.
3. For more complex subsystems, you may keep them as plain helper classes (no base `Subsystem`), owned by PhoenixRobot.
4. Change TeleOp/Auto to:

    * Extend `OpMode` / `LinearOpMode`.
    * Construct PhoenixRobot in `init()`.
    * Call `robot.updateTeleOp(dt)` / `robot.updateAuto(dt)` from `loop()`.

### 5.2 From PhoenixTeleOpBase / PhoenixAutoBase to thin OpModes

1. Copy any robot‑specific fields from the base class into your robot class or OpMode.
2. Replace base class lifecycle methods with explicit logic:

    * `init()` → construct `FtcHardware`, `DebugSink`, and `PhoenixRobot`.
    * `loop()` → update clock, inputs, bindings, then call `updateTeleOp(dt)`.
3. Remove inheritance from `PhoenixTeleOpBase` / `PhoenixAutoBase` once you’ve replicated the needed behavior.

In many cases, this simplifies the file rather than making it longer.

### 5.3 From Stage / MacroSlot to TaskRunner

1. Identify where a Stage or MacroSlot was acting like a scheduler for single macros.
2. Introduce a `TaskRunner` field in your robot class.
3. Replace Stage/MacroSlot calls with:

    * `taskRunner.start(task);`
    * `taskRunner.update(dt);`
4. If "safe idle" behavior is needed, express it directly:

    * Default plant targets set in `updateTeleOp(dt)` when no macro is running.
    * Or use wrappers like `InterlockPlant` for safety conditions.

### 5.4 From DriverKit to Gamepads + Bindings

1. Replace any `DriverKit.Player` usage with `GamepadDevice` from `Gamepads`.
2. Move control mapping logic into a `configureBindings()` method on your robot.
3. Use `bindings.update()` from your OpMode `loop()`.

---

## 6. Miscellaneous advanced notes

### 6.1 Multiple TaskRunners

The recommended starting point is **one TaskRunner per robot**. In rare cases it may be useful to have more than one, e.g.:

* A separate TaskRunner dedicated to a long‑running climb sequence.
* Another TaskRunner for smaller, interruptible macros.

If you go this route:

* Be explicit about which TaskRunner owns which plants.
* Document how interruptions should behave.

### 6.2 Idle behavior and safety

The framework does not enforce a single global notion of "safe idle".

Instead:

* Each robot is expected to define what "idle" means for its plants and drivebase.
* Safety features (e.g., not running shooter and intake in conflicting directions) are best expressed as:

    * interlocks in plant wrappers, or
    * explicit logic in the robot’s `update…` methods.

This keeps policy in robot code rather than baking it into the framework.

### 6.3 Versioning and file organization

If you maintain both legacy and new examples:

* Consider grouping legacy examples under a separate package (e.g., `examples.legacy`) or clearly marking them with comments.
* Update docs to always point new students to the modern examples first.

---

## 7. Summary

* Phoenix has evolved from base‑class/subsystem patterns to a **composition‑based robot class** with **thin OpModes**.
* Older constructs like `PhoenixTeleOpBase`, `PhoenixAutoBase`, `Subsystem`, `Stage`, `MacroSlot`, and `DriverKit` remain primarily for backward compatibility.
* New code should favor:

    * `PhoenixRobot` (or similar) as the central composition point.
    * `Gamepads` + `Bindings` for input.
    * `Plant` + `Task` + `TaskRunner` for actuation and behavior.

Use this document when you’re touching older files, need to understand historical context, or are planning larger refactors.


## Principles for evaluating output:

- Simplicity of robot-specific implementation is paramount.
    * Beginner path: Have simple and opinionated design that can be used very easily
    * Advanced path: Power users should be able to configure the details if they want
- Consistent. Framework should be consistent in the philosophy it follows
- Solve for the 80%. Framework should work well for *common* FTC use cases, including from those
  used by other FTC robots.
- Overall simplicity is important. Code should be as complex as it needs to be to solve the problem
  elegantly, but no more complex
- No duplication. Code should be reused as much as possible and there should be no duplication of
  logic.
- Structured organization. Classes should be named for clarity and the package structure should make
  purpose clear.
- Limited awareness. Each layer should serve a purpose and its awareness of the system should be
  limited as much as possible, but enough to do the work.
- Document the code with javadocs. Write comments that explain:
    * The framework contract / requirements, and assumptions
    * When and how to use classes
    * When to extend the class or not
- Telemetry should aid debugging. Add telemetry instrumentation for classes for easy debugging
  later.

## Ensure the package has quality documentation in addition to the javadocs

- One canvas that includes:
    * Framework philosophy and separation of logic into layers
    * System architecture guidelines
    * Happy-path example snippets
- Example files that are to be placed in an "examples" package appropriately:
    * Examples of how to create the common FTC use cases
    * Examples of how to integrate with RoadRunner
    * Examples of how to use the more complex features of the framework

## Principles to use when generating output

- Create a point of view using best practices
- Play the devils advocate and evaluate how the framework will not work, and resolve issues.
- It is okay to propose new interface and break legacy code as long as we are confident that the
  result is better.
- When we find a problem, think through whether the problem can be generalized and we solve for a
  bigger issue than the specifically identified issue alone.