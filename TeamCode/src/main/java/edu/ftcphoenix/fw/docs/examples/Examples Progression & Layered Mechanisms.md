# Examples Progression & Layered Mechanisms

This guide explains how the runnable framework examples fit together as one learning sequence.

The short version is:

- `TeleOp_01` through `TeleOp_04` are the beginner ramp,
- `TeleOp_05` and `TeleOp_06` are the shooter + vision case study,
- `TeleOp_07` and `TeleOp_08` are the bridge into mechanism-layering ideas,
- and `TeleOp_09` is the first example that makes the three internal layers explicit.

If you are reading the examples to understand the framework's layering philosophy, the most
important files are now:

- `TeleOp_07_SupervisorPoseMechanism`
- `TeleOp_08_LiftExternalSensorControl`
- `TeleOp_09_LayeredShooterMechanism`

Those final examples are not “off theme.” They are the place where the examples stop being only
shooter variations and start showing how robot-owned mechanism structure should look.

---

## 1. Read the examples as a progression, not as isolated files

### Example 01: `TeleOp_01_MecanumBasic`

This is the drive-stream baseline.

It teaches:

- `LoopClock` update order,
- gamepad → drive-source flow,
- drivebase update + drive,
- and the basic one-loop heartbeat used across the framework.

This example intentionally stays flat and does not try to introduce held/frame/pending mechanism
inputs yet.

### Example 02: `TeleOp_02_ShooterBasic`

This is the first mechanism example.

It keeps the drive path simple and adds a small scoring mechanism with direct mode selection. The
value of Example 02 is not a full layering story yet; it is showing how to turn FTC hardware into
plants and how to map simple controls into those plants without losing the basic loop shape.

### Example 03: `TeleOp_03_ShooterMacro`

This adds non-blocking macro behavior.

The core lesson is that timed sequences should become tasks/macros rather than ad-hoc sleeps or
hand-written timers spread through the OpMode.

### Example 04: `TeleOp_04_ShooterInterpolated`

This adds data-driven calibration. Instead of hardcoding one shooter speed, the example uses a
small interpolation table.

That prepares you for later examples where the selected target, distance estimate, or calibration
curve may vary at runtime.

### Examples 05-06: shooter + vision

These are the detailed shooter examples documented in the separate shooter case study.

- `TeleOp_05_ShooterTagAimVision` adds tag selection, aim assist, and shooter speed from vision
  range.
- `TeleOp_06_ShooterTagAimMacroVision` layers a shooting macro on top of the same selector.

Read [`Shooter Case Study & Examples Walkthrough.md`](<Shooter Case Study & Examples Walkthrough.md>)
for the full reasoning behind those two examples.

### Examples 07-09: mechanism layering

These are the architecture bridge from simple examples to real robot code.

They are where the framework's newer vocabulary becomes visible:

- held state,
- frame-valued manual commands,
- pending work,
- behavior/execution logic,
- and plant-owned realization.

---

## 2. Why Examples 07, 08, and 09 matter

The framework now encourages a simple mental model inside one mechanism owner:

1. caller-owned requests or selections are remembered,
2. robot-owned behavior decides what should happen now,
3. plant-owned realization computes and applies final targets.

The examples introduce that model progressively instead of forcing full ceremony everywhere.

- Example 07 is a stepping stone: supervisor + subsystem, with one place that owns policy and one
  place that owns the plant.
- Example 08 shows that a regulated plant can be the realization layer even when behavior is
  almost a pass-through.
- Example 09 is the first explicit version where the code literally has <b>Requests</b>,
  <b>Behavior</b>, and <b>Realization</b> as separate internal owners.

That is why the examples section should be broader than “shooter docs.” Shooter remains one
important case study, but the final examples are teaching robot structure more directly.

---

## 3. Example 07: supervisor + subsystem, with a temporary override

`TeleOp_07_SupervisorPoseMechanism` shows a very common robot pattern:

- the driver requests one of a few semantic poses,
- the subsystem remembers that pose,
- a short temporary override can win for a moment,
- and the subsystem owns the final target source and Plant update order.

The example is intentionally small, but it shows three important ideas.

### 3.1 Pose requests are remembered state

The requested pose is not a one-cycle signal. It is state that stays active until the next request
replaces it.

That makes discrete-pose mechanisms feel stable and predictable.

### 3.2 Timed special behavior should not overwrite the base request forever

The “pulse open” command is modeled as a short-lived override. It temporarily wins, then the
mechanism naturally falls back to the remembered base pose.

This is exactly the kind of situation where output queues or task runners help: behavior owns the
short-lived execution, and the plant owner still owns the final target source and update order.

### 3.3 The subsystem still owns the target sources and Plant update order

Even though a supervisor is involved, the supervisor does not command hardware directly. The
subsystem still computes the final target source and updates the Plant.

That matches the framework principle of keeping one Plant/source owner per mechanism.

---

## 4. Example 08: held selection + regulated realization

`TeleOp_08_LiftExternalSensorControl` is the example to read when you want the cleanest
introduction to the newer mechanism-input vocabulary without a lot of scaffolding.

It has three especially useful ideas.

### 4.1 The requested lift height is a held selection

The driver taps A/B/Y to choose a desired lift height.

That choice is remembered in a `HeldValue<Double>`. This is a good example of a value that belongs
to caller-owned request memory:

- it is not a pending action queue,
- it is not a per-cycle manual command,
- it is simply the currently selected target.

### 4.2 Behavior is trivial here, and that is okay

Not every mechanism needs a fancy behavior layer.

Example 08 intentionally shows that the behavior step can be almost a pass-through when the
mechanism is simple:

- read the selected height,
- decide that the lift should hold that height,
- send that semantic target to the plant.

This is still consistent with the layered model. The middle layer is just small because the example
is simple.

### 4.3 Realization owns the regulated plant

The lift plant is built from:

- a raw motor-power output,
- an external analog position sensor,
- a PID regulator,
- and bounded native units.

The plant then owns the actual feedback loop and target tracking. That keeps the hardware-side
details in the realization layer instead of scattering them through the OpMode.

---

## 5. Example 09: explicit requests, behavior, and realization

`TeleOp_09_LayeredShooterMechanism` is the copyable architecture example.

It deliberately combines the three common request shapes in one mechanism owner:

- <b>Held</b>: flywheel enabled + selected flywheel velocity.
- <b>Frame</b>: manual feed power, refreshed every loop through `Bindings.copyEachCycle(...)`.
- <b>Pending</b>: shot requests stored in a `RequestCounter` until behavior consumes them.

Then it makes the next two layers explicit.

### 5.1 Requests only remember caller intent

The request layer does not know anything about timing, pulse duration, or final target-source composition.

Its only job is to remember:

- what the caller wants held,
- what the caller commanded this frame,
- and how many shots are still pending.

### 5.2 Behavior owns the interesting part

The behavior layer decides:

- whether pending shots are blocked by manual feed,
- whether the flywheel is ready for a shot,
- when a shot request becomes an active feed pulse,
- whether the feed should currently be idle, manual, or pulsing,
- and whether the flywheel should stay spun up because work is still pending.

That is the heart of the mechanism. It is also the layer where timing belongs.

### 5.3 Realization stays boring on purpose

The realization layer is intentionally boring:

- it takes the chosen flywheel target and feed power,
- writes them to the plants,
- updates the plants,
- and exports a small readback snapshot for the next loop.

That simplicity is a feature. It keeps the target-source ownership rule obvious.

For a deeper walkthrough of Example 09, read
[`Layered Shooter Example.md`](<Layered Shooter Example.md>).

---

## 6. Where held, frame, and pending fit in the examples

The examples now line up like this:

- <b>Held</b> values show up naturally in Examples 07, 08, and 09 for remembered pose, height,
  flywheel, and selected-velocity state.
- <b>Frame</b> values show up explicitly in Example 09 for the manual feed command that must be
  refreshed each loop.
- <b>Pending</b> work is represented by `RequestCounter` in Example 09. The behavior layer drains
  that counter one shot at a time when the mechanism is ready.

Drive remains the main exception: raw manual drivetrain control is still best expressed as a
continuous drive-source stream rather than a capability-style frame value.

---

## 7. Recommended reading order from here

If you are new to the framework:

1. read `TeleOp_01` and `TeleOp_02`,
2. skim `TeleOp_03` and `TeleOp_04`,
3. read the shooter case study for `TeleOp_05` and `TeleOp_06`,
4. then study `TeleOp_07`, `TeleOp_08`, and `TeleOp_09` as the bridge to real robot
   architecture.

If you are already comfortable with the basics and want the architectural takeaways quickly:

1. read `TeleOp_07_SupervisorPoseMechanism`,
2. read `TeleOp_08_LiftExternalSensorControl`,
3. then read `TeleOp_09_LayeredShooterMechanism` and
   [`Layered Shooter Example.md`](<Layered Shooter Example.md>).
