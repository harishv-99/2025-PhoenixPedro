# Framework Principles

This document explains the design principles behind the Phoenix framework: **why the APIs look the way they do**, and the usage patterns the framework is optimized for.

If you want to *get running* first, read:

1. **Beginner’s Guide** – loop shape + how to wire Plants.
2. **Tasks & Macros Quickstart** – how to build macros using factory helpers.
3. **Shooter Case Study & Examples Walkthrough** – a complete example tied to real code.

---

## 1. High-level goals

Phoenix is designed around a few core goals:

1. **Non-blocking by design**

   No `sleep(...)`, no long `while (...)` loops inside TeleOp/Auto. Anything that takes time is expressed as a **Task** that advances once per loop.

2. **Clear separation of concerns**

   Robot code is written in terms of a few narrow building blocks:

    * **Drive behavior**: `DriveSource` → `DriveSignal` → `MecanumDrivebase`
    * **Mechanisms**: `Plant`
    * **Behavior over time**: `Task` / `TaskRunner`

   FTC SDK specifics live in the `fw.ftc` boundary layer (and `fw.tools`), not in your robot logic.

3. **Beginner-friendly, mentor-powerful**

   Students primarily use:

    * `Actuators.plant(hardwareMap) ... build()` (to create Plants)
    * `PlantTasks`, `Tasks`, `DriveTasks` (to create Tasks)

   Mentors can go deeper (HAL, adapters, custom Tasks) when needed.

4. **Composable building blocks**

   Drive logic, plants, and tasks are intentionally decoupled so you can swap components:

    * Use `GamepadDriveSource` today, add a `DriveGuidance` overlay tomorrow.
    * Keep the same shooter macro while changing how distance is estimated.


5. **One loop, one heartbeat**

   Phoenix assumes a single loop heartbeat (`LoopClock`) that everything else uses.
   This enables robust button edge detection, predictable task timing, and consistent rate limiting.

6. **Docs are part of the API**

   Phoenix treats documentation as a first-class feature: Javadocs should be “mouse-over quality,”
   and the Markdown guides should stay in sync with the real APIs and examples.

   Phoenix also treats **debuggability** as a first-class feature: a good `debugDump(...)` (and in some cases a good `toString()`) is **live documentation**.

   **Rule of thumb:**

   * Use **`toString()`** for small, mostly-immutable *value objects* and configs where a compact one-line representation is helpful (examples: `Pose2d`, `Pose3d`, `DriveSignal`, `ChassisSpeeds`, `CameraMountConfig`, small `Owner.Config` objects).
   * Use **`debugDump(DebugSink dbg, String prefix)`** for loop-updated and/or stateful objects (things that have `update(...)`, own hardware, or own other objects): `Plant`, `PoseEstimator`, `DriveSource` / `DriveOverlay`, `Task`s, controllers, subsystems.

   **When both exist:**

   * `toString()` should answer “what is this object?” (identity + key config).
   * `debugDump()` should answer “what is this object doing right now?” (live state: current targets, errors, timers, modes, enable flags).

   **`debugDump(...)` conventions:**

   * Accept a nullable sink and do nothing if `dbg == null`.
   * Use stable keys: `prefix + ".fieldName"` (avoid changing key names).
   * Keep it safe to call every loop: no exceptions, no blocking, no expensive work.
   * Prefer delegating: call `child.debugDump(dbg, prefix + ".child")` for owned components.

   **`toString()` conventions:**

   * Keep it short (single line).
   * Avoid huge nested output; if it wants to be multi-line or you want nested structure, prefer `debugDump(...)`.

   **Debug vs required telemetry:**

   * `debugDump(...)` is for diagnostics and understanding internal state. It should be safe to disable (for performance or to reduce telemetry noise).
   * Driver-facing / “required for normal operation” telemetry (mode, safety warnings, high-level state the drivers rely on) should **not** depend on the debug pipeline.
     In FTC OpModes, print that information via the FTC `Telemetry` API directly (or via a dedicated always-on status channel).

7. **Fail fast with actionable errors**

   When something is misconfigured, Phoenix should throw early (often at build-time) with an
   error message that tells a student what to change. Avoid silent no-ops.

8. **Principle-driven evolution (breaking changes are OK)**

   Phoenix optimizes for a coherent, principle-driven API surface — not strict backwards compatibility.

   * If a better design exists, it’s okay to make a breaking change.
   * Prefer **deleting** legacy paths instead of accumulating deprecations.
     Keep deprecated code only when there is a concrete reason (external consumers, season support,
     migration cost that truly can’t be paid immediately, etc.).
   * Aim for **one obvious way** to do something. If two APIs overlap, choose one and remove the other.
   * Keep things **parallel** across similar objects:
     consistent capabilities, consistent method names, consistent argument order, and consistent nouns.

---

## 2. Layering: from hardware to behavior

Phoenix is built in layers. Most robot code should live near the **top**.

### 2.1 Hardware abstraction (HAL)

At the bottom is a tiny hardware abstraction layer:

* `PowerOutput` – normalized power (typically `-1..+1`).
* `PositionOutput` – position in native units (servo `0..1`, encoder ticks, etc.).
* `VelocityOutput` – velocity in native units (ticks/sec, etc.).

The FTC adapter `edu.ftcphoenix.fw.ftc.FtcHardware` wraps FTC SDK devices into these outputs.

Most robot code should **not** use these directly.

### 2.2 Plants

A **Plant** is the low-level sink you command with a scalar target.

Key methods (see `edu.ftcphoenix.fw.actuation.Plant`):

* `setTarget(double)` / `getTarget()`
* `update(double dtSec)`
* `stop()`
* `atSetpoint()` and `hasFeedback()`
* optional `reset()` and `debugDump(...)`

A Plant may be open-loop (power, servo set-and-hold) or closed-loop (motor position/velocity with feedback).

### 2.3 The beginner entrypoint: `Actuators.plant(...)`

`edu.ftcphoenix.fw.actuation.Actuators` is the recommended way to create Plants from FTC hardware.

```java
import edu.ftcphoenix.fw.core.hal.Direction;

Plant shooter = Actuators.plant(hardwareMap)
        .motor("shooterLeftMotor", Direction.FORWARD)
        .andMotor("shooterRightMotor", Direction.REVERSE)
        .velocity()   // uses default tolerance in native velocity units
        .build();

Plant transfer = Actuators.plant(hardwareMap)
        .crServo("transferLeftServo", Direction.FORWARD)
        .andCrServo("transferRightServo", Direction.REVERSE)
        .power()
        .build();

Plant pusher = Actuators.plant(hardwareMap)
        .servo("pusherServo", Direction.FORWARD)
        .position()   // servo position set-and-hold (open-loop)
        .build();
```

The builder is staged on purpose:

1. **Pick hardware**: `motor` (optional `andMotor`), `servo` (optional `andServo`), `crServo` (optional `andCrServo`)
2. **Pick control type**: `power()`, `velocity()` / `velocity(tol)`, `position()` / `position(tol)`
3. **Optional modifiers**: `rateLimit(maxDeltaPerSec)`, then `build()`

Internally, Phoenix also has `Plants` factory helpers, but student code should typically prefer `Actuators`.

---

## 3. Drive: sources, signals, and the drivebase

Phoenix drive is split into two parts:

### 3.1 `DriveSource` produces a `DriveSignal`

A `DriveSource` converts “intent” into a robot-centric `DriveSignal`:

* manual TeleOp: `GamepadDriveSource.teleOpMecanumStandard(pads)`
* assisted drive: `DriveGuidance` overlays (auto-aim, go-to-point, pose lock, etc.)
* autonomous logic: any custom `DriveSource`

### 3.2 `DriveSignal` sign conventions are a contract

`DriveSignal` is robot-centric and aligned with Phoenix pose conventions (+X forward, +Y left):

* `axial > 0` → forward
* `lateral > 0` → left
* `omega > 0` → CCW (turn left)

Driver-facing conversions (e.g., “stick right means strafe right”) happen **at the input boundary** (see `GamepadDriveSource`), not scattered through control code.

### 3.3 `MecanumDrivebase` applies the command

`MecanumDrivebase` mixes a `DriveSignal` into four wheel powers.

If you use rate limiting, call `update(clock)` **before** `drive(signal)` so it uses the most recent `dtSec`:

```java
DriveSignal s = driveSource.get(clock).clamped();

drivebase.update(clock);
drivebase.drive(s);
```

Configuration is via `MecanumDrivebase.Config`. The drivebase makes a **defensive copy** of the config at construction time, so mutating the config object later won’t change an already-created drivebase.



### 3.4 Config objects

Phoenix uses two common patterns for configuration:

1. **Owner-scoped configs** live as a nested `Owner.Config` class.

   * Use this when the config is only meaningful to that one class.
   * Examples: `MecanumDrivebase.Config`, `FtcVision.Config`, `PinpointPoseEstimator.Config`.
   * These are usually simple mutable data objects: start from `defaults()`, tweak the fields you care about, then pass the config into the owner.
   * Owners should defensively copy configs at construction time when later mutation would be surprising.

2. **Semantic config wrappers** are top-level `*Config` classes.

   * Use this when the config represents a real thing with meaning across multiple systems, and the type name should communicate that meaning.
   * Example: `CameraMountConfig` (robot→camera extrinsics).
   * These are often immutable value objects with `of(...)` / `identity()` factory helpers.

Rule of thumb: if you find yourself creating a top-level `XConfig` that is only ever used to configure `X`, it probably wants to be `X.Config` instead.


#### 3.4.1 Factory naming: `defaults()` vs `identity()` vs `zero()`

Phoenix uses these names intentionally:

- **`defaults()`** means “a reasonable starting configuration for a component.”
  It’s used for `Owner.Config` classes that collect tuning / hardware options.
  The values are not “correct for your robot,” they’re just safe to start from.
  Example: `PinpointPoseEstimator.Config.defaults()`.

- **`identity()`** means “the identity transform.”
  It’s used when the object conceptually *is a transform between frames* and there is a meaningful
  identity element (“no translation, no rotation”).
  Example: `CameraMountConfig.identity()` means the camera is assumed to be at the robot origin and
  aligned with the robot axes — useful as a placeholder, but you should replace it with a calibrated
  mount for real accuracy.

- **`zero()`** is used for geometry primitives where the identity transform is also the “all zeros” value.
  Example: `Pose2d.zero()` / `Pose3d.zero()` represent “no translation, no rotation.”

Rule of thumb: if the thing you’re constructing is *behavior/tuning*, call it `defaults()`.
If it’s *pure geometry*, prefer `zero()` / `identity()` depending on what’s idiomatic for that type.


#### 3.4.2 Constructing configs: prefer `defaults()` over `new`

For **owner-scoped configs** (`Owner.Config`), Phoenix intentionally standardizes on a factory method:

- Create configs with `Owner.Config.defaults()`
- Do **not** instantiate them directly with `new Owner.Config()`

Why this matters:

- It keeps call sites uniform and easy to grep.
- It lets us change how defaults are produced later (copy-on-write, versioned defaults, etc.) without changing user code.
- It avoids the "half the code uses `new`, half uses `defaults()`" drift that makes refactors noisy.

Implementation rule:

- `Owner.Config` (and other owner-scoped nested `*Config` classes, like `FtcDrives.MecanumWiringConfig`) constructors should be `private`, and each config should provide `public static ... defaults()`.


#### 3.4.3 Naming config helpers: `withX(...)` / `withoutX()`

Most Phoenix config tuning is done by directly setting public fields:

```java
MecanumDrivebase.Config cfg = MecanumDrivebase.Config.defaults();
cfg.maxOmega = 0.8;
cfg.maxAxial = 1.0;
```

Sometimes, a config wants convenience helpers (usually when toggling a feature requires setting multiple related fields).
When Phoenix provides those helpers, the naming convention is:

- `withX(...)` to enable/apply something
- `withoutX()` to disable/clear something

Phoenix avoids `useX(...)` in config APIs. If you see a `useX(...)` method, it should be renamed to `withX(...)` for consistency.



#### 3.4.4 Boolean naming in configs

Phoenix tries to make boolean flags read clearly at the call site (especially inside `if (...)`).

**Rule:** In config objects, booleans should usually be named as *feature toggles*:

- Prefer `enableX` for “turn this behavior on/off.”
- Avoid `useX` (it's ambiguous and tends to drift into inconsistent meaning).
- Avoid negative booleans like `disableX` when possible — prefer a positive `enableX` and flip the default if needed.

Examples:

- ✅ `enableAprilTagAssist`
- ✅ `enableInitializeFromVision`
- ✅ `enableResetOnInit`
- ❌ `useAprilTagsAssist`
- ❌ `disableVisionInit`

If a boolean is genuinely a *permission* (“this action is allowed”), `allowX` is acceptable — but try to keep that rare and obvious.


### 3.5 Direction and naming conventions

Phoenix coordinates and sign conventions are a contract.

Importantly, Phoenix uses **multiple frames**:

* **Robot / mechanism frames (robot-centric):** Phoenix uses a right-handed frame with
  **+X forward**, **+Y left**, **+Z up**.
  * `DriveSignal` is defined in this robot-centric frame.
  * Camera mount extrinsics (`robotToCamera`) are expressed in this robot-centric frame.

* **Field frame (field-centric):** Phoenix uses the **FTC Field Coordinate System** for the
  current season.
  * The origin is the center of the field on the floor.
  * **+Z is up**.
  * The meaning of **+X and +Y is season-dependent** (diamond vs square vs inverted square)
    and is defined by FTC official docs.
  * When in doubt, use the FTC reference-frame definition: a person standing at the center
    of the Red Wall looking in — **+X is to their right** and **+Y is away from the Red Wall**.

Shared conventions (all frames):

* Distances are in **inches**.
* Angles are in **radians**.
* Heading/yaw is **CCW-positive** about **+Z**.

**Naming guideline:** prefer *directional words* (`Forward`, `Left`, `Up`, etc.) when a value is not literally a `Pose2d` component.

For example, library boundaries often use parameter names like `xOffset` / `yOffset` that are defined in that library’s local frame. In Phoenix code, prefer names like `offsetLeftInches`, `offsetForwardInches`, or `forwardPodOffsetLeftInches` to make the meaning explicit.
---

## 4. Tasks and macros

### 4.1 Cooperative tasks

A `Task` is a cooperative unit of work driven by the main loop:

* `start(LoopClock clock)` – called once
* `update(LoopClock clock)` – called each cycle while running
* `isComplete()` – true when finished
* `getOutcome()` – optional richer completion info (`TaskOutcome`)

### 4.2 The `TaskRunner`

`TaskRunner` runs tasks sequentially (FIFO). Tasks are assumed to be **single-use**.

A key design choice: `TaskRunner.update(clock)` is **idempotent by `clock.cycle()`**. If nested code accidentally calls `update()` twice in the same loop cycle, tasks do not advance twice.

### 4.3 Prefer factory helpers

Robot code should rarely implement raw tasks directly. Prefer:

* `Tasks.*` for generic composition (`sequence`, `parallelAll`, `waitForSeconds`, `waitUntil`, ...)
* `PlantTasks.*` for commanding Plants (`setInstant`, `holdFor`, `moveTo`, ...)
* `DriveTasks.*` for drive behaviors

---

## 5. Feedback vs open-loop (and why `hasFeedback()` exists)

Phoenix distinguishes:

* **Feedback-capable plants** – implement a meaningful `atSetpoint()` and override `hasFeedback()` to return `true` (motor position/velocity plants).
* **Open-loop plants** – do not expose sensor-based completion and leave `hasFeedback() == false` (power plants, servo set-and-hold plants).

Important consequence:

* `PlantTasks.moveTo(...)` / `moveTo(..., timeout)` / `moveToThen(...)` **require** `plant.hasFeedback() == true` and will throw if used on an open-loop plant.
* Time-based helpers like `holdFor(...)` / `holdForThen(...)` work on any Plant.

This makes it hard to accidentally write a “wait for setpoint” macro on a mechanism that has no feedback.

---

## 6. LoopClock: the per-cycle truth

Phoenix expects:

1. **Advance the clock once per OpMode cycle** (`clock.update(getRuntime())`).
2. **Everything else reads the clock** (dt, cycle id) but does not advance time.

Why the cycle id matters:

* Button edge detection, bindings, task runners, and similar systems need a clear definition of “one loop cycle.”
* Phoenix uses `LoopClock.cycle()` as that identity.

Several core systems are **idempotent by cycle**:

* `Button.updateAllRegistered(clock)`
* `Gamepads.update(clock)`
* `Bindings.update(clock)`
* `TaskRunner.update(clock)`

Idempotency prevents subtle bugs when code is layered (menus, testers, helpers) and multiple layers try to “helpfully” update the same system.

---

## 7. Nomenclature and coordinate conventions

Phoenix is designed so you can read code and know:

* what frame a value is in,
* what units it uses,
* what sign it means.

### 7.1 Frame must appear in the name

If a value depends on a frame, the frame name should appear in the identifier.

Examples:

* `fieldToRobotPose`, `fieldToRobotTargetPose`
* `robotDriveSignal`
* `cameraBearingRad`, `cameraForwardInches`, `robotLeftInches`

Avoid ambiguous names like `pose`, `targetPose`, `x`, `y`, `heading` when the frame is not obvious.

In particular, **data holders should not expose a raw `pose` field** if the frame matters. Prefer names like
`fieldToRobotPose`, `robotToCameraPose`, `cameraToTagPose`, etc.

### 7.2 Transform naming: `fromToToPose` (optionally `p`-prefixed in adapters)

For rigid transforms (`Pose2d` / `Pose3d`) that represent relationships between frames, Phoenix consistently uses:

* `cameraToTagPose`
* `robotToCameraPose`
* `fieldToRobotPose`

In **Phoenix core code**, the `p` prefix is usually unnecessary (everything is already in Phoenix framing), and it
creates noisy method APIs. Prefer names like `robotToTagPose(...)` rather than `pRobotToTag(...)`.

In **adapter code** where multiple coordinate systems coexist (FTC SDK vs Phoenix), it *can* be helpful to prefix
Phoenix-framed values with `p` (for example, `pFieldToRobotPose`) so it’s obvious which convention you’re in.

Rule of thumb: if you see `pAtoB.then(pBtoC)`, the result should be `pAtoC`.

This convention is also called out explicitly in the FTC adapter boundary (`edu.ftcphoenix.fw.ftc.FtcFrames`).

### 7.3 Units must appear in the name

Prefer suffixes like: `Inches`, `Rad`, `Deg`, `Sec`, `PerSec`, `PerSec2`.

Examples:

* `headingRad`, `omegaRadPerSec`
* `maxSpeedInchesPerSec`, `timeoutSec`

### 7.4 Signs are part of the contract

Phoenix uses right-handed conventions:

* +X forward, +Y left, +Z up
* yaw CCW-positive
* `DriveSignal.omega > 0` turns left

Convert “driver intuition” at the boundaries (for example, in `GamepadDriveSource`), not throughout the codebase.

---

## 8. Recommended usage pattern

A typical OpMode loop follows this shape:

> Clock → Inputs → Bindings → Tasks → Drive → Plants → Telemetry

In code (conceptually):

```java
clock.update(getRuntime());

gamepads.update(clock);
bindings.update(clock);

macroRunner.update(clock);

drivebase.update(clock);
drivebase.drive(driveSource.get(clock).clamped());

shooter.update(clock.dtSec());
transfer.update(clock.dtSec());
```

The **Loop Structure** document dives deeper into why this order matters.

---

## 9. Extending Phoenix

Phoenix is designed to be extended safely:

* New `DriveSource` implementations (assist drive, path following, etc.)
* New Plant types (custom control or interlocks)
* New task factories (team-specific high-level macros)

When extending:

* Keep SDK- or vendor-specific calls in adapters.
* Preserve per-cycle semantics (do not “secretly” advance time or consume edges).
* Keep interfaces narrow (`Plant`, `DriveSource`, `Task`) so systems remain composable.

---

## 10. Documentation standards

Phoenix code is meant to be read by students **during build season** under time pressure.
Javadocs aren’t “nice to have” — they are part of the API.

### 10.1 Javadocs rules of thumb

**Every class should be documented (public or not).**

- **Top-level types:** start with a one-sentence summary of what the type is for.
- **Non-public / internal types:** still add a short comment that answers “why does this exist?”
  and (if relevant) what invariant it relies on.
- If the type depends on **units** or a **coordinate frame**, say so up front.

**Every non-trivial method should be documented.**

- Start with a short verb phrase: “Create…”, “Update…”, “Return…”, “Apply…”, etc.
- Document **units and frames** for parameters and return values (use suffixes like
  `Inches`, `Rad`, `Sec`, etc. and explain when the frame is not obvious).
- Document **preconditions** and what happens when they are violated (`@throws` with a helpful message).
- Document **side effects** (what gets mutated / what gets cached).
- For per-loop methods (`update`, `get(clock)`, etc.), document **call order** and whether
  behavior is **idempotent by cycle**.

**Examples are encouraged when they prevent misuse.**

- Prefer small, copy-pastable snippets using `<pre>{@code ... }</pre>`.
- When the “right” usage is not obvious (builders, overlays, transforms), include at least one example.

**Link to the right things.**

- Use `{@link ...}` to connect concepts and reduce duplicated explanations.
- Use `{@code ...}` for code-ish names.

### 10.2 Error messages are documentation

Phoenix frequently teaches through its exceptions.

- Prefer errors like: “field heading targets require fieldPose(...) feedback” over generic
  `NullPointerException` / “invalid state.”
- If a configuration can never work, throw **at build time** rather than silently producing
  “no output” at runtime.

### 10.3 Keep docs and code in sync

- If an API changes, update **Javadocs**, **Markdown docs**, and **examples** in the same change.
- Avoid stale comments (“TODO update later”) in student-facing areas — they become misinformation.
