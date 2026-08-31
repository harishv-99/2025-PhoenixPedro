# Robot calibration tutorials

This is the framework's start-to-finish path for the calibration steps most FTC robots need before
mechanisms, localization, and driver assists are trustworthy.

Start new actuators through one ordinary device-first path:

- run either **FW: Testers (Driver Station) → HW: Actuator Bring-up** or **FW: Testers (Panels) →
  HW: Actuator Bring-up**;
- isolate one configured device;
- establish its direction and, when appropriate, already-backed-off safe endpoint evidence;
- copy those facts into robot configuration; and
- verify them again through the real mechanism or drivetrain owner.

Read [`Actuator bring-up`](<Actuator Bring-up.md>) before first motion. A robot-specific guided
walkthrough may order later integration and localization checks, but it should reuse this hardware
fact rather than invent another generic actuator workflow. Direct controller/encoder experiments
live under **Advanced: Hardware Diagnostics**.

## Before you start

Use a robot that is mechanically assembled enough to roll freely, with the final odometry pods, camera, and drivetrain wiring already installed.

For the best experience:

- use a fully charged battery
- put the robot on reasonably flat flooring
- make sure the camera can see tags clearly
- bring a laptop open to `RobotConfig` so you can paste values immediately
- change one thing at a time, then rerun the relevant tester

Vision/calibration tools take one explicit data-only Config plus, when they use AprilTags, one
backend-neutral lane-factory builder. The tool validates and snapshots its active data and fixed-tag
layout before it uses its child context or opens hardware; the builder separately captures the
webcam or Limelight recipe and must stay stable for the tool's lifetime and clean picker retries.
Facts that depend on the returned lane—its actual subtype, mount/sensor accessors, and asynchronous
readiness—can only be checked after `open()`. A null contract fact or a `RuntimeException` from those
checks detaches the published lane and closes it exactly once when cleanup succeeds; `NOT_READY` is
normal pending state, not a failure. An `Error` propagates immediately without promised cleanup. If
the lane remains published and STOP is later invoked, that boundary closes the still-retained owner.

The snapshotted layout display deliberately keeps the FTC game-policy summary plus captured IDs and
poses, rather than retaining the mutable source layout for richer per-key debug rows. An empty layout
is preserved as empty: raw detections can remain visible, but no fixed-layout calibration sample or
AprilTag correction is invented. Software defaults establish a valid authoring grammar; they do not
prove the selected camera, mount, printed tag size, field placement, Pinpoint geometry, or drivetrain.

## Actuator direction and safe endpoints

The canonical runbook is [`Actuator bring-up`](<Actuator Bring-up.md>). It distinguishes motor
ticks, standard-servo `0.0..1.0` command endpoints, physical safe travel, Plant units/bounds,
mapping, and runtime reference. Use it before the reference choices below.

The generic wizard can report direction alone, or direction plus two human-approved endpoints for a
bounded DC motor or standard servo. It cannot automatically discover hard stops, infer CR-servo
position, choose meaningful Plant units, establish an incremental encoder's durable zero, or tune
PIDF. After direction, feedback, bounds, and stop behavior are established, use the separate
[`control tuning workflow`](<Control Tuning Workflow.md>).

## Production mechanism and timed-behavior check

The generic wizard establishes one device fact. It does not prove that the production profile,
mechanism owner, capability meaning, Task ending, and lifecycle stop work together. Perform this
separate integration check before relying on a mechanism in TeleOp or Auto.

### Before enabling motion

- Review the exact hardware names, directions, bounds, powers or limits, sensor polarity, and stop
  policy used by the selected production owner. Checked-in defaults and successful Config
  validation are software evidence, not physical evidence.
- If the profile has an explicit motion permission such as `allowIntakeMotion`, leave it false while
  editing and set it true only after that subsystem's complete configuration has been reviewed.
  Keep unrelated subsystem motion permissions false for an isolated check.
- Remove game pieces, fixture the robot, unload the mechanism when appropriate, and keep people,
  hair, tools, clothing, and wires outside its full motion path. Never hold or stall a powered shaft,
  roller, wheel, or gearbox by hand.
- Do not validate a bounded arm, lift, or other hard-stop mechanism by copying an unbounded power
  example. Use its real feedback, bounds, references, guards, support, and owner-specific policy.
- Begin with conservative power and duration. Select only the intended OpMode and assign one person
  to FTC STOP with immediate access to robot power.

### Procedure

1. With the mechanism clear, press INIT. It must not begin ordinary motion. If it does, press STOP,
   remove motion power, and inspect wiring, the selected OpMode, competing owners, and local changes.
2. Press START and request one small semantic action, such as collect, eject, raise, or launch. Verify
   that the physical direction matches the capability meaning rather than compensating with a
   scattered negative command.
3. For timed behavior, verify the requested action becomes observable, lasts for the reviewed
   interval or terminal condition, and reaches its documented completion request. Do not block the
   loop with `sleep(...)` to create or measure the interval.
4. Cancel one active behavior through its supported control when that is part of the design, and
   verify its documented cancellation request. Then press FTC STOP and verify the mechanism's owner
   immediately applies its physical stop path without needing another ordinary loop update.
5. Only after the unloaded, conservative check succeeds should the team repeat under expected load
   and record computed versus operator-observed evidence using the
   [`subsystem experiment`](<../examples/Subsystem Experiments.md>) lab card.

### Do not move on if

- INIT causes unexplained motion;
- the mechanism moves opposite its semantic request, crosses reviewed travel, or contacts an
  obstruction;
- a timed action never reaches its documented ending;
- cancellation or FTC STOP leaves unsafe output; or
- the only evidence is a permission boolean, successful build, unit test, or plausible telemetry.

## Mechanism position references

### Why this matters

A position Plant can have a clean public coordinate even when the raw hardware coordinate is awkward.
A lift may use plant units of inches or ticks above the bottom, while the motor encoder starts at an
arbitrary raw count. A tray may use degrees modulo one rotation, while a painted mark establishes
where phase zero is. Standard servos may use logical `0.0..1.0` even though the useful raw servo
range is `0.30..0.80`.

Sushi keeps those ideas separate:

```text
raw/native hardware coordinate
    -> reference + unit mapping
    -> public plant coordinate
    -> PlantTargets exact/equivalent/advanced plan
    -> PositionPlant invokes its target resolver
```

There is no catch-all Plant reset. `Plant.stop()` ends that Plant instance and must not redefine
physical zero. Homing, indexing, manual zeroing, and static endpoint scaling belong in the
position-Plant reference/mapping layer and the robot mechanism service that decides when to run it.

### Common initialization choices

Use `alreadyReferenced()` when the selected native coordinate is already meaningful in the plant
coordinate. Examples: a standard servo using raw `0..1`, an absolute/source measurement already in
degrees, or a simulator source already in plant units.

Use `plantPositionMapsToNative(plantPosition, nativePosition)` when the scale and one offset point are known in
code. Example: arm degrees mapped to encoder ticks with a measured zero tick.

Both values must be finite. Sushi rejects `NaN` and infinity immediately rather than clamping
either coordinate. The plant position is a coordinate-map anchor, not a target request, so it need
not lie inside the Plant's legal target range.

Use `assumeCurrentPositionIs(value)` only when the robot is physically placed at a known pose before
init. Its plant-unit answer must be finite. Example: the lift is manually collapsed before the
match, so the first finite encoder sample becomes plant position `0.0`; a non-finite sample leaves
the reference pending.

Use `needsReference(reason)` when the mechanism must find a switch, index mark, hard stop, or custom
sensor condition before position targets are safe.

### Runtime homing/indexing task

A reference search is a normal non-blocking `Task`:

### Critical code

Replace the demonstration mechanism, cue, powers, holds, and timeout with reviewed robot facts.

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...inside the mechanism's fresh homing-task factory...
Task homeLift = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .holdAfterReference(0.0)
        .failAfterSec(3.0)
        .build();
```

`.withPower(...)` requires a finite normalized command in the inclusive `[-1.0, +1.0]` range. It
rejects `NaN`, infinities, and overshoot immediately instead of clamping them into a different
search. Passing that check does not make the recipe mechanically safe: verify the magnitude,
direction, cue polarity/behavior, hard stops, and clearance on the actual robot.

`establishReferenceAt(...)` also requires a finite plant-unit coordinate and rejects `NaN` or
infinity at the recipe step, before search lifecycle effects. It is a reference anchor rather than
a target, so it is not clamped to `targetRange()`. `holdAfterReference(...)` requires a separate
finite plant-unit command. A finite hold still goes through the complete target resolver, range,
overlays, and guards; choose a deliberately safe in-range value when the mechanism should hold that
exact position predictably.

Build a fresh search Task for every homing attempt. A search Task that has begun is not restarted;
the same builder recipe can create the next attempt.

Advance that Task from the runner before the mechanism's normal update. The Task owns the temporary
search lifecycle, cue, reference, timeout, and handoff, but it never calls `plant.update(clock)`.
The mechanism remains the sole Plant heartbeat owner, so its one downstream update either submits
the staged search command or returns through the normal target resolver after the Task releases the
search.

For an indexer or tray, the condition can be a color detector, magnet sensor, beam break, or custom
BooleanSource:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...inside the mechanism's fresh indexing-task factory...
Task indexTray = PositionCalibrationTasks.search(tray)
        .withPower(0.12)
        .until(paintedMarkSeen)
        .establishReferenceAt(0.0)
        .resumeTargeting()
        .failAfterSec(5.0)
        .build();
```

**What to notice**

- Each attempt builds a fresh single-use `Task`; the mechanism remains the sole Plant heartbeat owner.
- The search stages temporary raw output and must end with an explicit hold or resume policy.
- Timeout/cancellation stops and releases the search without redefining the persistent target graph.
- Finite software validation does not prove switch polarity, physical zero, clearance, or safe power.

**Key APIs**

- `PositionCalibrationTasks.search(plant)` — starts the non-blocking reference-task recipe.
- `until(BooleanSource)` — supplies the independently owned reference cue.
- `establishReferenceAt(...)` — anchors the public coordinate at the cue sample.
- `holdAfterReference(...)` / `resumeTargeting()` — explicitly chooses the post-reference request policy.
- `failAfterSec(...)` — gives the search a bounded lifetime.

`resumeTargeting()` preserves the Plant's persistent command and final target resolver. It requests
a nonterminal stop of the temporary raw output and releases the search; it does not call terminal
`Plant.stop()`. The downstream Plant phase immediately evaluates the unchanged graph. By contrast,
`holdAfterReference(value)` writes that graph-owned command before releasing
the search. The full resolver still runs afterward, so an enabled overlay may select another
target. Timeout and active cancellation request the same stop and release the search without
changing the persistent command.

Every reference search must explicitly choose timeout behavior. Prefer `failAfterSec(...)`; use `neverTimeout()` only when a driver button, scheduler, or other safety interlock is guaranteed to cancel the task.

For periodic Plants, the Task's clocked `establishReferenceAt(...)` uses the cue cycle's current
native sample, treats the supplied value as a reference within the period, and preserves the nearest
unwrapped equivalent from that sample. That makes repeated index marks useful for small drift
corrections during a match. When the current plant estimate is finite, the Plant rejects a
non-finite final nearest-equivalent result without committing that reference. Reference commit also
requires every endpoint and derived measurement of the complete candidate bounded affine map to be
finite. An FTC raw-domain check then runs before each realized command whose native offset depended
on that runtime reference. An unbounded core Plant-to-native conversion is checked individually
before applied state or output, while the later FTC child/domain layer checks all children before
the first child write.

### What “good” looks like

- before reference, the Plant reports an invalid target range with a clear reason such as `lift not homed`
- the homing/indexing task has timeout and cancellation behavior
- the mechanism, not the calibration Task, remains the only owner of the Plant update heartbeat
- after reference, the public measurement matches the physical mechanism coordinate
- the finite software reference has been checked against an independent physical pose or cue; a
  numeric validation result alone does not prove that correspondence
- presets, command targets, Plant target requests, and telemetry all use plant units rather than raw
  hardware surprises
- one periodic command uses `PlantTargets.equivalentPositionsOf(...)`; multiple alternatives or
  observation metadata use the advanced `PlantTargets.plan(request)` path

### Do not move on if

- the mechanism can command outside its safe travel range
- raw encoder offsets leak into presets throughout robot code
- a periodic mechanism resets its unwrapped position to zero every time an index mark appears
- drivers need to remember raw servo endpoint values instead of logical mechanism positions

## Drivetrain direction and integration

### Why this comes first

Before you trust odometry or autonomous motion, each drivetrain motor should contribute in the direction you think it does. This is the fastest possible sanity check after wiring a fresh robot.

### Testers

- Raw configured-device fact: `HW: Actuator Bring-up`
- Robot-specific check when available: `HW: Configured Drivetrain Verification`
- Final check: the production TeleOp with all wheels safely raised

### Before enabling motion

- Review all drivetrain hardware names and directions, the zero-power brake choice, and conservative
  axial, lateral, and turn limits. If the profile has an explicit drive-motion permission, leave it
  false while editing and set it true only after that complete review.
- Raise and secure every drive wheel for the first integrated test. Keep mechanisms empty and clear,
  and assign one person to FTC STOP with immediate access to robot power.
- Release both sticks and every trigger before INIT and reconfirm them before START. Sushi
  `GamepadDevice` uses the current axis positions as its neutral baseline when it is constructed, so
  a held control can teach the wrong baseline.

### Procedure

1. In the generic wizard, command each configured motor individually and decide which FTC
   `Direction` makes its positive rotation contribute to robot-forward motion.
2. Copy those direction facts into the robot profile and rebuild. Fix the profile rather than
   compensating with scattered negative powers.
3. If the project provides configured-drivetrain verification, run it to confirm the profile and
   hardware-name wiring select the expected wheel one at a time.
4. With every control neutral and all wheels raised, press INIT. No ordinary drive motion should
   occur. Reconfirm neutral controls, press START, then test forward, strafe, and turn separately at
   small input. Test any slow-mode control before increasing the ordinary limits.
5. Release the controls and verify all drive motors command zero. Press FTC STOP and verify zero
   again.
6. Remove motion power before lowering the stopped robot. Never lower or carry it while the OpMode is
   active. Clear an open floor area, restore power, and start a fresh INIT/START with controls
   neutral.
7. Repeat only small, separate forward, strafe, and turn requests on the floor. Increase one limit at
   a time only after controlled tests establish that the current value is safe.

### Good result

A student can answer, without hesitation, “yes, each wheel does the expected thing.”

### Do not move on if

- one wheel spins opposite the others for the same commanded motion
- drivetrain motor names are still uncertain
- the robot moves during INIT or with neutral controls
- releasing the controls or FTC STOP does not produce the expected physical stop
- your only explanation is “mecanum is confusing” rather than a config fix

## High-resolution external encoder velocity comparison

### Why this is a separate hardware check

A quadrature encoder fundamentally supplies position changes. FTC hardware and the SDK may also
report a device-timed velocity, but that representation can have a smaller numeric range than the
position counter. A high-count-rate external encoder therefore needs evidence from the exact hub,
firmware, port, SDK, and loop configuration before either reading becomes the production default.
Motor configuration metadata is not proof of which physical encoder is connected.

The advanced motor-power diagnostic includes a measurement-only comparison for this purpose. For safe open-loop
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

1. Run either tester entry, open `Advanced: Hardware Diagnostics`, and select the motor
   power/encoder evidence diagnostic.
2. Choose the motor/encoder entry, then start the OpMode with the power target still at zero.
   Output remains disarmed until you deliberately press A.
3. In Android Studio Logcat, filter for the tag `SushiEncoderVelocity`.
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

On a matched REV module, each accepted loop explicitly requests one bulk snapshot and reads
that motor port's position and direct velocity from the same packet. The tester applies the same
configured-direction plus motor-type-orientation normalization as the FTC motor getters, so both
values use the public motor coordinate. It observes but never changes the module's `OFF`, `MANUAL`,
or `AUTO` caching mode. The explicit snapshot clears that module's current bulk cache before its
transaction attempt; a normally returned real or fake response refills it. Use this isolated
diagnostic as the only hub-cache owner in the OpMode. In particular, it is
incompatible with `FtcBulkCaching.manual(hardwareMap)` and must not run beside that service or any
other code that sets modes, clears caches, or calls `getBulkData()`. The tester runs through the
separate `FtcTeleOpTesterOpMode` host, which has no `RobotProgram` service phase; do not install or
simulate the manual-cache service there. Ordinary getters are consumers, not competing owners. Each
data row records snapshot coherence and that the configured mode was preserved. See
[`FTC manual bulk caching`](<../ftc-boundary/FTC Manual Bulk Caching.md>) for the exclusive managed
owner contract. Here coherence means only that both decoded values came from one returned packet;
it does not establish freshness, validity, or a physically simultaneous hardware observation.

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

1. Keep the robot still until the tester reports Pinpoint `READY`, then zero the tester.
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
2. During successful ordinary INIT, confirm the tool configures and polls evidence without commanding
   drive power. Failed-init rollback and an explicit STOP may still command physical zero.
3. Press Driver Station START; this is the first ordinary drive-zero command boundary.
4. Use a manual sample or an auto sample, depending on whether the project provides drivetrain wiring.
5. Rotate roughly 180 degrees in place. Automatic motion requires exact current-cycle Pinpoint
   `READY` pose and velocity evidence and fail-stops when that evidence disappears.
6. Let the tester compute the recommended offsets.
7. Paste the two printed offset field assignments into your Pinpoint config.
8. Rerun the tester and confirm the recommendation stabilizes instead of wandering wildly.

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

Use **HW: Actuator Bring-up** for ordinary motor/servo hardware facts, **Framework: Calibration &
Localization** for camera/pose work, and **Advanced: Hardware Diagnostics** only for a distinct
controller or measurement investigation. A robot-specific walkthrough may order those checks for
one robot, but it does not create a second generic actuator path.
