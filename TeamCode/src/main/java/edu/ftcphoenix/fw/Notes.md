# Phoenix Notes and Legacy

This document is for mentors and framework maintainers.
Most students should not need anything here to write robot code.

---

## 1. Legacy code (`edu.ftcphoenix.fw.legacy`)

Phoenix historically provided framework base classes for OpModes (and a small subsystem interface):

* `PhoenixTeleOpBase`
* `PhoenixAutoBase`
* `Subsystem`

These are kept **intentionally** in `edu.ftcphoenix.fw.legacy.*` as a reference and an occasional escape hatch.

### 1.1 Current recommendation (new code)

For new robots and new students, prefer the “thin OpMode + explicit loop” pattern:

* A plain FTC OpMode (`OpMode` or `LinearOpMode`).
* A single robot container class (your season robot) that owns:

  * `LoopClock`
  * `Gamepads` and `Bindings`
  * drive (`DriveSource` → `MecanumDrivebase`)
  * plants (`Plant` via `Actuators.plant(...)`)
  * one or more `TaskRunner`s for macros

And follow the standard loop order:

> **Clock → Inputs → Bindings → Tasks → Drive → Plants → Telemetry**

This keeps control flow visible, avoids inheritance surprises, and aligns with the rest of the Phoenix docs.

### 1.2 What the legacy base classes are for

The legacy base classes exist because sometimes a team wants:

* a quick way to stand up a TeleOp/Auto without repeating boilerplate, or
* a “house style” that enforces the Phoenix loop ordering.

They are **not** the recommended teaching path, which is why they are:

* in the `legacy/` package, and
* marked `@Deprecated`.

If you choose to use them anyway, treat them as a convenience wrapper around the same loop contract (they should not change the fundamentals).

---

## 2. Advanced notes

### 2.1 Multiple `TaskRunner`s

The recommended starting point is **one `TaskRunner` per robot**.

Use multiple runners only when it genuinely reduces complexity, for example:

* one runner dedicated to a long, non-interruptible sequence (like an endgame climb), and
* one runner for small, interruptible TeleOp macros.

If you do this, be explicit about **ownership**:

* Which runner is allowed to command which plants/drive behaviors.
* What happens when a macro is interrupted (what targets are left behind).

### 2.2 Idle behavior and safety

Phoenix does not enforce a single global notion of “safe idle.”

Instead, keep it explicit in your robot logic:

* define what each mechanism should do when no macro is running
* reset targets intentionally

When it helps clarity, use plant wrappers:

* `RateLimitedPlant` to smooth target changes
* `InterlockPlant` to enforce simple safety rules

### 2.3 FTC boundary rule (for maintainers)

As a best practice, keep FTC SDK types (`com.qualcomm.*`) inside:

* `edu.ftcphoenix.fw.ftc.*` (the adapter/boundary layer), and
* `edu.ftcphoenix.fw.tools.*` (testers/examples that necessarily depend on OpModes)

This keeps the student-facing building blocks (`actuation/drive/input/task/...`) easier to reason about and easier to test in isolation.
