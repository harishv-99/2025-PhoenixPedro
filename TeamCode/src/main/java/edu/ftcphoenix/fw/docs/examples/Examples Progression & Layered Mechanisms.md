# Examples Progression & Layered Mechanisms

This guide explains how the runnable framework examples fit together as one learning sequence.

The short version is:

- `TeleOp_01` through `TeleOp_04` are the beginner ramp,
- `TeleOp_05` and `TeleOp_06` are the shooter + vision case study,
- `TeleOp_07` and `TeleOp_08` show the newer mechanism-architecture patterns.

If you are reading the examples to understand the framework's layering philosophy, the most important files are now:

- `TeleOp_07_SupervisorPoseMechanism`
- `TeleOp_08_LiftExternalSensorControl`

Those two examples are not "off theme." They are the place where the examples stop being just shooter variations and start showing how robot-owned mechanism structure should look.

---

## 1. Read the examples as a progression, not as isolated files

### Example 01: `TeleOp_01_MecanumBasic`

This is the drive-stream baseline.

It teaches:

- `LoopClock` update order,
- gamepad → drive-source flow,
- drivebase update + drive,
- and the basic one-loop heartbeat used across the framework.

This example intentionally stays flat and does not try to introduce held/frame/pending mechanism inputs yet.

### Example 02: `TeleOp_02_ShooterBasic`

This is the first mechanism example.

It keeps the drive path simple and adds a small scoring mechanism with direct mode selection. The value of Example 02 is not a full layering story yet; it is showing how to turn FTC hardware into plants and how to map simple controls into those plants without losing the basic loop shape.

### Example 03: `TeleOp_03_ShooterMacro`

This adds non-blocking macro behavior.

The core lesson is that timed sequences should become tasks/macros rather than ad-hoc sleeps or hand-written timers spread through the OpMode.

### Example 04: `TeleOp_04_ShooterInterpolated`

This adds data-driven calibration. Instead of hardcoding one shooter speed, the example uses a small interpolation table.

That prepares you for later examples where the selected target, distance estimate, or calibration curve may vary at runtime.

### Examples 05-06: shooter + vision

These are the detailed shooter examples documented in the separate shooter case study.

- `TeleOp_05_ShooterTagAimVision` adds tag selection, aim assist, and shooter speed from vision range.
- `TeleOp_06_ShooterTagAimMacroVision` layers a shooting macro on top of the same selector.

Read [`Shooter Case Study & Examples Walkthrough.md`](<Shooter Case Study & Examples Walkthrough.md>) for the full reasoning behind those two examples.

### Examples 07-08: mechanism layering

These are the architecture bridge from simple examples to real robot code.

They are where the framework's newer vocabulary becomes visible:

- held state,
- pending work,
- execution/policy,
- and plant-owned realization.

---

## 2. Why Examples 07 and 08 matter

The framework now encourages a simple mental model inside one mechanism owner:

1. caller-owned inputs or selections are remembered,
2. robot-owned execution decides what should happen now,
3. plant-owned realization computes and applies final targets.

The examples do **not** force that model with a lot of ceremony. Instead, they show it progressively.

- Example 07 is a stepping stone: supervisor + subsystem, with one place that owns policy and one place that owns the plant.
- Example 08 is the cleanest simple example of a held selection feeding a regulated plant with external feedback.

That is why the examples section should be broader than "shooter docs." Shooter remains one important case study, but the final examples are teaching robot structure more directly.

---

## 3. Example 07: supervisor + subsystem, with a temporary override

`TeleOp_07_SupervisorPoseMechanism` shows a very common robot pattern:

- the driver requests one of a few semantic poses,
- the subsystem remembers that pose,
- a short temporary override can win for a moment,
- and the subsystem remains the single writer to the hardware plant.

The example is intentionally small, but it shows three important ideas.

### 3.1 Pose requests are remembered state

The requested pose is not a one-cycle signal. It is state that stays active until the next request replaces it.

That makes discrete-pose mechanisms feel stable and predictable.

### 3.2 Timed special behavior should not overwrite the base request forever

The "pulse open" command is modeled as a short-lived override. It temporarily wins, then the mechanism naturally falls back to the remembered base pose.

This is exactly the kind of situation where output queues or task runners help: execution owns the timing, and the plant owner still remains the final writer.

### 3.3 The subsystem is still the single writer

Even though a supervisor is involved, the supervisor does not write the hardware directly. The subsystem still computes the final target and updates the plant.

That matches the framework principle of keeping one plant owner per mechanism.

---

## 4. Example 08: held selection + regulated realization

`TeleOp_08_LiftExternalSensorControl` is the example to read when you want the cleanest introduction to the newer mechanism-input vocabulary without a lot of scaffolding.

It has three especially useful ideas.

### 4.1 The requested lift height is a held selection

The driver taps A/B/Y to choose a desired lift height.

That choice is remembered in a `HeldValue<Double>`. This is a good example of a value that belongs to caller-owned input memory:

- it is not a pending action queue,
- it is not a per-cycle manual command,
- it is simply the currently selected target.

### 4.2 Execution is trivial here, and that is okay

Not every mechanism needs a fancy execution layer.

Example 08 intentionally shows that the execution step can be almost a pass-through when the behavior is simple:

- read the selected height,
- decide that the lift should hold that height,
- send that semantic target to the plant.

This is still consistent with the layered model. The middle layer is just small because the example is simple.

### 4.3 Realization owns the regulated plant

The lift plant is built from:

- a raw motor-power output,
- an external analog position sensor,
- a PID regulator,
- and bounded native units.

The plant then owns the actual feedback loop and target tracking. That keeps the hardware-side details in the realization layer instead of scattering them through the OpMode.

---

## 5. Where held, frame, and pending fit in the examples

The examples now line up like this:

- **Held** values show up naturally in Examples 07 and 08 for remembered pose/height selections.
- **Frame** values are best for manual non-drive commands that must be refreshed each loop. None of the current example OpModes needs a strong frame-based mechanism command yet, but the pattern is ready for future examples such as manual lift jog or arm power override.
- **Pending** work is represented by `RequestCounter`. Example 07 hints at this pattern in the supervisor comments and in the way temporary override behavior is separated from remembered pose. More advanced robot code will often drain pending requests into execution state or queued tasks.

Drive remains the main exception: raw manual drivetrain control is still best expressed as a continuous drive-source stream rather than a capability-style frame value.

---

## 6. Recommended reading order from here

If you are new to the framework:

1. read `TeleOp_01` and `TeleOp_02`,
2. skim `TeleOp_03` and `TeleOp_04`,
3. read the shooter case study for `TeleOp_05` and `TeleOp_06`,
4. then study `TeleOp_07` and `TeleOp_08` as the bridge to real robot architecture.

If you are already comfortable with the basics and want the architectural takeaways quickly:

1. read `TeleOp_07_SupervisorPoseMechanism`,
2. read `TeleOp_08_LiftExternalSensorControl`,
3. then map those ideas onto your robot-owned capability and mechanism code.
