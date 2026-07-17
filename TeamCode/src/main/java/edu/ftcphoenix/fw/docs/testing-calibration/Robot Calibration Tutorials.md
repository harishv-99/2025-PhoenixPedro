# Robot calibration tutorials

This is the framework's start-to-finish bring-up path for the core calibration steps that most FTC robots need before localization and driver assists are trustworthy.

The framework now supports two complementary ways to reach this material:

- a **guided walkthrough menu** inside the tester tree for students who want one recommended order
- a **category-based tester tree** for teams who already know which individual tool they want

That split is intentional. The walkthrough is allowed to duplicate entries. Individual testers should still have one clear home in the category menus.

## Before you start

Use a robot that is mechanically assembled enough to roll freely, with the final odometry pods, camera, and drivetrain wiring already installed.

For the best experience:

- use a fully charged battery
- put the robot on reasonably flat flooring
- make sure the camera can see tags clearly
- bring a laptop open to `RobotConfig` so you can paste values immediately
- change one thing at a time, then rerun the relevant tester

## Mechanism position references

### Why this matters

A position Plant can have a clean public coordinate even when the raw hardware coordinate is awkward.
A lift may use plant units of inches or ticks above the bottom, while the motor encoder starts at an
arbitrary raw count. A tray may use degrees modulo one rotation, while a painted mark establishes
where phase zero is. Standard servos may use logical `0.0..1.0` even though the useful raw servo
range is `0.30..0.80`.

Phoenix keeps those ideas separate:

```text
raw/native hardware coordinate
    -> reference + unit mapping
    -> public plant coordinate
    -> PlantTargets.plan()
    -> PositionPlant target source
```

`Plant.reset()` should not redefine physical zero. Homing, indexing, manual zeroing, and static
endpoint scaling belong in the position-Plant reference/mapping layer and the robot mechanism
service that decides when to run it.

### Common initialization choices

Use `alreadyReferenced()` when the selected native coordinate is already meaningful in the plant
coordinate. Examples: a standard servo using raw `0..1`, an absolute/source measurement already in
degrees, or a simulator source already in plant units.

Use `plantPositionMapsToNative(plantPosition, nativePosition)` when the scale and one offset point are known in
code. Example: arm degrees mapped to encoder ticks with a measured zero tick.

Use `assumeCurrentPositionIs(value)` only when the robot is physically placed at a known pose before
init. Example: the lift is manually collapsed before the match, so the first encoder sample becomes
plant position `0.0`.

Use `needsReference(reason)` when the mechanism must find a switch, index mark, hard stop, or custom
sensor condition before position targets are safe.

### Runtime homing/indexing task

A reference search is a normal non-blocking `Task`:

```java
Task homeLift = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .holdAfterReference(0.0)
        .failAfterSec(3.0)
        .build();
```

Build a fresh search Task for every homing attempt. A search Task that has begun is not restarted;
the same builder recipe can create the next attempt.

For an indexer or tray, the condition can be a color detector, magnet sensor, beam break, or custom
BooleanSource:

```java
Task indexTray = PositionCalibrationTasks.search(tray)
        .withPower(0.12)
        .until(paintedMarkSeen)
        .establishReferenceAt(0.0)
        .stopAfterReference()
        .failAfterSec(5.0)
        .build();
```

Every reference search must explicitly choose timeout behavior. Prefer `failAfterSec(...)`; use `neverTimeout()` only when a driver button, scheduler, or other safety interlock is guaranteed to cancel the task.

For periodic Plants, `establishReferenceAt(...)` treats the supplied value as a reference within the
period and preserves the nearest unwrapped equivalent. That makes repeated index marks useful for
small drift corrections during a match.

### What “good” looks like

- before reference, the Plant reports an invalid target range with a clear reason such as `lift not homed`
- the homing/indexing task has timeout and cancellation behavior
- after reference, the public measurement matches the physical mechanism coordinate
- presets, Plant target requests, and telemetry all use plant units rather than raw hardware surprises

### Do not move on if

- the mechanism can command outside its safe travel range
- raw encoder offsets leak into presets throughout robot code
- a periodic mechanism resets its unwrapped position to zero every time an index mark appears
- drivers need to remember raw servo endpoint values instead of logical mechanism positions

## Drivetrain motor direction

### Why this comes first

Before you trust odometry or autonomous motion, each drivetrain motor should contribute in the direction you think it does. This is the fastest possible sanity check after wiring a fresh robot.

### Tester

- Framework generic tools: `HW: DcMotor Power`
- Robot-specific shortcut when available: `HW: Drivetrain Motor Direction`

### Procedure

1. Use the robot-specific drivetrain direction tester if your project provides one.
2. Command each wheel individually.
3. Confirm the intended “forward” test really tries to drive the robot forward.
4. Fix any reversed drivetrain motor config before continuing.

### Good result

A student can answer, without hesitation, “yes, each wheel does the expected thing.”

### Do not move on if

- one wheel spins opposite the others for the same commanded motion
- drivetrain motor names are still uncertain
- your only explanation is “mecanum is confusing” rather than a config fix

## High-resolution external encoder velocity comparison

### Why this is a separate hardware check

A quadrature encoder fundamentally supplies position changes. FTC hardware and the SDK may also
report a device-timed velocity, but that representation can have a smaller numeric range than the
position counter. A high-count-rate external encoder therefore needs evidence from the exact hub,
firmware, port, SDK, and loop configuration before either reading becomes the production default.
Motor configuration metadata is not proof of which physical encoder is connected.

`HW: DcMotor Power` includes a measurement-only comparison for this purpose. For safe open-loop
power testing it temporarily selects `RUN_WITHOUT_ENCODER`, then restores the motor's prior mode
after commanding zero when the tester stops or returns to the picker. It does not filter either
reading, correct an apparent velocity wrap, or change any Plant feedback API.

### Safety and setup

- Mechanically fixture the mechanism, guard every rotating part, and begin at zero power.
- Selecting a motor always resets the target and leaves output disarmed. The A press that chooses a
  motor cannot also arm it; release A, inspect the selection, then press A again deliberately.
- Use an independent tachometer with known accuracy; neither SDK reading is an independent truth.
- Select the configured motor whose own encoder port carries the external encoder. This tester does
  not compare a separately selected encoder-only port while driving a different motor.
- For a high-rate quadrature encoder on a REV hub, use encoder port **0 or 3**. Those ports are
  hardware-counted; FIRST warns that the software-counted ports 1 and 2 can miss counts from a
  high-count-rate encoder. See the current
  [FIRST Control and Expansion Hub guidance](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips.html)
  and record the exact port.
- Record the SDK version, hub model and firmware, bulk-caching mode, encoder version and counts per
  revolution, battery voltage, and tachometer model before the run.

### Procedure

1. Run `FW: Testers`, open `Framework: Hardware Testers`, and select `HW: DcMotor Power`.
2. Choose the motor/encoder entry, then start the OpMode from Driver Station with the power target
   still at zero. Output remains disarmed until you deliberately press A.
3. In Android Studio Logcat, filter for the tag `PhoenixEncoderVelocity`.
4. Press Y to start capture. The first position-derived value deliberately reports unavailable
   until two positive-time samples exist. Confirm telemetry says the matched REV snapshot is
   coherent and `Row eligible for tachometer comparison` says `YES`. This means the row has the
   required measurement mechanics; only the independent tachometer comparison can establish
   accuracy, so do not use an ineligible row or the label alone to decide production policy.
5. Press A to arm, then increase power gradually. Hold each safe test point long enough to record
   the tachometer, then capture acceleration, coast-down, reversal, and both rotation directions.
   Include points below, near, and above any suspected direct-velocity representation boundary.
6. Press right bumper once during a steady point to skip exactly one comparison sample. The OpMode
   and motor command continue normally; the following accepted sample spans the longer interval.
   Never create a long sample with `sleep(...)` or a blocked loop.
7. Press B to command zero and keep capturing until the mechanism has stopped. Press Y again to end
   the capture, then save the filtered Logcat output.

Each capture begins with an `ENCODER_VELOCITY_META` row containing the selected connection, controller,
port, matched REV module address/serial/firmware, original bulk-caching mode, motor run modes,
direction, and configured motor-type values. Those configured motor-type values are labeled
metadata, not physical encoder identification.

On a matched REV module, each accepted loop explicitly obtains one fresh bulk snapshot. If the
module's mode is `OFF`, the tester switches to `MANUAL` only while both public motor getters consume
that snapshot, then restores `OFF`; existing `MANUAL` or `AUTO` settings are left unchanged. Each
data row records whether the snapshot was coherent and the original mode was preserved. This keeps
the direct and position readings comparable without silently changing the production stack's
bulk-caching policy.

Each `ENCODER_VELOCITY_DATA` row records the session, motor name, loop cycle/time, enabled state,
target power, the command held before measurement, the command issued afterward, position,
rollover-aware position delta, accepted sample interval, both velocities, their difference,
availability flags, snapshot/bulk-mode evidence, port eligibility, and status. An
`ENCODER_VELOCITY_ERROR` row makes an unavailable cycle explicit, `ENCODER_VELOCITY_SKIPPED`
identifies the deliberate one-sample gap, and `ENCODER_VELOCITY_END` closes a capture. Both
velocities are in ticks per second. Convert the tachometer reading using:

```text
expected ticks/second = tachometer RPM * encoder counts/revolution / 60
```

Compare steady-state accuracy and sign as well as spin-up, spin-down, reversal, stop, ordinary loop
intervals, and any observed long loop. Compile success and plausible-looking telemetry are not
hardware validation. Preserve the raw log: filtering, smoothing, or a signed-velocity correction is
a separate design decision that must not be inferred from one display value. Keep captures short
and evaluate the recorded loop intervals because per-cycle Logcat output can itself affect timing.

## Camera mount

### Why this matters

Every AprilTag field-pose solve depends on `robot -> camera` extrinsics. If the camera mount is still left at an identity placeholder, tag localization may appear to work while quietly producing the wrong pose.

### Tester

- `Calib: Camera Mount`
- robot-specific variant if your project preselects a vision backend or tag-localizer config

### What you are solving

You tell the tester where the robot is on the field, the tester observes a known tag, and it solves for the camera pose relative to the robot.

### Procedure

1. Place the robot in a pose you can describe confidently in the FTC field frame.
2. Open `Calib: Camera Mount`.
3. Choose the active vision device if a picker appears.
4. Select the visible tag ID.
5. Adjust the known robot pose until it matches the real robot position and heading.
6. Hold the robot still and capture several samples.
7. Paste the printed `CameraMountConfig.ofDegrees(...)` value into `RobotConfig`.
8. Rerun the tester once after pasting to confirm the new config behaves the same way.

### What “good” looks like

- the solved mount translation is physically plausible for where the camera really sits
- repeated samples cluster closely
- `Sample vs avg mount` stays small when the robot is still
- `Avg residual` and the range check look reasonable instead of exploding

### Common mistakes

- using the wrong tag ID
- typing the wrong robot field pose
- mixing up field axes or heading sign
- trying to calibrate while the robot is moving
- pasting the printed value into the wrong robot config field

### Record this result in code

Update the robot's camera mount config immediately. Do not leave a “paste later” sticky note for this step.

## AprilTag-only localization check

### Why this matters

Do not jump straight to odometry fusion. First verify that tags alone are being detected and that the field pose solve is sane.

### Tester

- `Loc: AprilTag Localization`

### Procedure

1. Run the tester after the camera mount has been pasted into config.
2. Confirm the selected vision device is correct.
3. Start in `ANY` mode to verify fresh detections exist.
4. Switch to `SINGLE` mode when you want to inspect one tag at a time.
5. Look at the solved `fieldToRobot` pose while the robot is still.
6. Capture a few samples and inspect the mean and standard deviation.

### What “good” looks like

- fresh detections appear without long gaps
- the selected tag matches what the camera is actually seeing
- the pose estimate is roughly correct in both translation and heading
- captured samples show low jitter while the robot is stationary

### Do not move on if

- detections are intermittent for no clear reason
- the solved pose is mirrored, rotated, or offset by a large amount
- the camera mount still looks like the identity placeholder

## Pinpoint axis directions

### Why this matters

Odometry sign mistakes poison every later localization step. Fix them before tuning offsets.

### Tester

- `Calib: Pinpoint Axis Check`

### Procedure

1. Zero the tester.
2. Start the forward sample, push the robot forward by hand, then stop the sample.
3. Start the left sample, push the robot left by hand, then stop the sample.
4. Start the rotation sample, rotate the robot CCW by hand, then stop the sample.
5. Apply the tester's suggested config fixes if any sign is wrong.
6. Rerun until all three directions read correctly.

### What “good” looks like

- forward motion produces positive X
- left motion produces positive Y
- CCW rotation produces positive heading

### Record this result in code

Set your robot-side explicit verification flag once the axes have been checked on real hardware. The walkthrough menu uses that acknowledgement to know this step was deliberately completed.

## Pinpoint pod offsets

### Why this matters

Axis directions only fix signs. Pod offsets fix the geometry. Leaving offsets at `0 / 0` makes rotation drift look like translation.

### Tester

- `Calib: Pinpoint Pod Offsets`

### Prerequisites

Run this after:

- camera mount is solved
- AprilTag-only localization looks believable
- Pinpoint axis directions are verified

### Procedure

1. Start from a still robot.
2. Use a manual sample or an auto sample, depending on whether the project provides drivetrain wiring.
3. Rotate roughly 180 degrees in place.
4. Let the tester compute the recommended offsets.
5. Paste the printed `.withOffsets(...)` numbers into your Pinpoint config.
6. Rerun the tester and confirm the recommendation stabilizes instead of wandering wildly.

### What “good” looks like

- repeated runs converge on similar offsets
- the last sample heading change is large enough for a stable solve
- the recommended offsets are physically plausible
- later fused localization no longer “slides” during turns with no real translation

### Common mistakes

- rotating too little
- rotating almost exactly 360 degrees, which makes the solve ill-conditioned
- treating real floor slip as an odometry-config problem
- skipping the camera mount step and then expecting AprilTag assist to rescue the solve

### Record this result in code

Paste the new offsets immediately and set your robot-side “offsets calibrated” acknowledgement flag when you are satisfied with the result.

## Pinpoint plus field corrections

### Why this matters

This is the first true global-localization validation pass. At this point you are no longer asking whether each subsystem works in isolation. You are checking whether motion prediction and the chosen absolute correction source agree enough to trust the combined pose.

### Tester

- `Loc: Pinpoint + Field Corrections`

### Procedure

1. Start with the robot where tags are visible.
2. Compare the predictor pose, the raw AprilTag pose, the active correction pose, and the corrected pose.
3. Drive around while tags are visible and confirm the fused pose stays stable.
4. Move closer to a target or rotate so tags disappear.
5. Confirm the fused pose continues smoothly on odometry alone.
6. Bring tags back into view and watch the fused estimator correct itself cleanly.

### What “good” looks like

- odom, vision, and fused estimates are broadly consistent
- the corrected pose does not jump unpredictably when corrections are enabled
- the fused pose keeps updating smoothly when tags disappear temporarily
- absolute corrections improve the estimate instead of fighting it

### Do not move on if

- fusion only looks good when the robot is perfectly still
- turning in place introduces obvious translation drift
- the camera mount or Pinpoint offsets are still known-bad

## Optional EKF comparison

### Why this is optional

The covariance-aware EKF-style estimator is intentionally not the first thing teams should tune. It is easier to debug the simpler fusion estimator first, then compare the EKF once the hardware calibration is already credible.

### Tester

- `Loc: Pinpoint + Field Corrections EKF`

### Procedure

1. Get the default fusion tester into a trustworthy state first.
2. Run the EKF tester on the same path.
3. Compare correction behavior, lag, and stability.
4. Only keep the EKF if it is clearly helping for your robot and floor conditions.

### What “good” looks like

- the EKF agrees with the simpler fusion path most of the time
- innovations are reasonable instead of constantly huge
- the estimated uncertainty behaves like a useful readiness signal, not noise

## How this maps to the tester menus

The intended navigation pattern is:

1. **Guided walkthrough** when starting from a fresh robot
2. **Category menus** once you already know which system you are working on

That keeps the walkthrough student-friendly without turning the entire tester tree into one long duplicated checklist.
