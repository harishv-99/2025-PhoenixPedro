---
tags:
  - Test & Tune
---

# Robot calibration tutorials

**Learning mode:** Operational runbook

Use this runbook to establish one physical fact at a time, record it, and then prove that the
production robot actually consumes the recorded configuration.

**Before reading:** use [the tester console guide](<Using the Tester Console.md>) for the menu and
control vocabulary. Reading needs no hardware. Choose only the procedure for the fact you need;
this is not a requirement to calibrate every device in order.

## Choose your stage

| Stage | Use it when | Outcome |
| --- | --- | --- |
| **1. Probe and record** | You have the robot, even if you cannot edit its source. | Run the framework-only testers, isolate one fact, and record the exact observation or suggested value. |
| **2. Rebuild and verify** | The robot profile owner can edit/deploy and the project supplies a fresh configured verifier for this fact. | Put the accepted fact in the canonical robot profile, rebuild, and verify it with that fresh robot-configured tester and then the production owner. |
| **3. Investigate an advanced question** | The ordinary path is already credible and you have a specific reason to go deeper. | Compare encoder representations, enable powered/vision-assisted pod calibration, compare an EKF, or construct a guided suite. |

The rookie first path is Stage 1 followed by a handoff to the Stage-2 owner. Source access is
optional for Stage 1 only. If you cannot edit and rebuild—or the project does not yet supply the
configured verifier—preserve the recorded evidence for the profile owner and stop before claiming
production verification. This page does not generate a robot-specific verifier from generic
defaults.

## What the framework-only testers know

For camera/position procedures, a **pose** is position plus facing direction. A **frame** specifies
the origin and axis directions used for those numbers; **yaw** is horizontal turning angle.
**Localization** estimates the robot's pose. **Odometry** estimates movement from sensors such as
encoder-equipped tracking wheels (pods); Pinpoint is the odometry computer used by these testers.
An **AprilTag layout** records known field positions of the square visual markers. An **identity
camera mount** assumes zero offset and rotation between robot and camera; it is not a measurement.

Run either **FW: Testers (Driver Station)** or **FW: Testers (Panels)**. Under **Framework:
Calibration & Localization**, the generic `StandardTesters` entries are independent fact probes:
every selection reconstructs a fresh tester `Config` from framework defaults.

- Generic vision entries use the selected webcam or Limelight name, the current-game fixed-tag
  layout, and an identity camera mount. The camera-mount calibrator can still measure the mount
  because that mount is its unknown, but the generic AprilTag and corrected-localization entries do
  not receive a mount you measured in another screen.
- Generic Pinpoint entries replace only `hardwareMapName` with the selected device name. They retain
  the defaults: `0.0 / 0.0` pod offsets, both encoder directions `FORWARD`, the
  `goBILDA_4_BAR_POD` resolution, factory yaw calibration (`yawScalar = null`), and quality `0.75`.
  The generic pod-offset entry has no drive and no AprilTag-assist factory.
- `StandardTesters` entries do not persist or propagate results: no generic entry writes a file,
  changes the robot profile, sets an acknowledgement, or passes its result to the next entry. They
  cannot verify production configuration.

The required handoff is **record -> rebuild -> fresh robot-configured tester -> verify**:

1. **Record** the exact displayed value, suggested assignment, device name, conditions, and the
   observations that justified accepting it.
2. **Rebuild** after the profile owner copies that fact into the canonical robot configuration.
3. **Open a fresh robot-configured tester supplied by the robot project.** Its suite factory creates
   a new owner and that owner snapshots its supplied configuration; an already-open tester cannot
   reload a changed profile.
4. **Verify** the rebuilt value through that configured tester, then through the real mechanism,
   drivetrain, or localization owner. A repeatable generic probe alone is not this verification.

Read [`Actuator bring-up`](<Actuator Bring-up.md>) before first motion. Direct controller/encoder
experiments and custom suite construction are optional advanced work, not prerequisites for the
ordinary calibration path.

## Before you start

For a physical procedure, install the final hardware that procedure measures. Drivetrain/odometry
checks need a robot that rolls freely; camera checks need the installed camera and known tags.
An actuator-only check does not require odometry pods or a camera.

Prepare an evidence sheet or, when source is available, the canonical robot profile. Also:

- use a fully charged battery
- put the robot on reasonably flat flooring
- make sure the camera can see tags clearly
- change one thing at a time, then rerun the relevant tester
- keep one person at FTC STOP whenever a powered workflow is active

Each vision tester owns the selected camera until BACK or STOP closes it. Wait while readiness says
`WAITING`; no visible tag is a different fact from camera readiness. If cleanup becomes uncertain,
stop and restart the OpMode rather than selecting another owner. An empty fixed layout may still show
raw detections, but it cannot produce a fixed-layout mount sample or field-pose correction.

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

For the complete first mechanism, read [Establish a lift reference](<../build/Referenced Lift.md>).
The [optional implementation details](<#optional-reference-implementation-details>) below explain
other reference strategies after the ordinary physical procedures.

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


## Camera mount

### Why this matters

A camera usually sits away from the robot's chosen origin and faces its own direction.
The **camera mount** describes that position and orientation relative to the robot; the exact term
is `robot -> camera` **extrinsics**. The solver needs this relationship to convert what the camera
sees into the robot's field pose. An unmeasured identity placeholder can produce a plausible but
wrong answer. Correctness means matching the physical installation, not merely using nonzero values.

### Framework-only entries and active defaults

- `Calib: Camera Mount (Webcam)`
- `Calib: Camera Mount (Limelight)`

Both entries use the current-game fixed layout and accept detections no older than `0.35 s`. The
known `fieldToRobotPose` starts at `(0, 0, 0)`, the first fixed-layout tag ID is initially selected,
and quick-edit mode starts with fine steps of `0.25 in` and `0.5°`; START switches to coarse steps of
`1.0 in` and `2.0°`. The generic webcam lane uses `640 x 480`. The generic Limelight lane requests
pipeline `0` at `100 Hz` and requires a result no older than `0.25 s` to confirm transport
readiness. Those are software defaults, not proof that the installed camera or pipeline is correct.

The generic lane's identity mount does not contaminate this solve: `robotToCameraPose` is the fact
being measured, so the calibrator deliberately does not use the lane's configured mount as an
input. It still does not write the measured answer anywhere.

### What you are solving

You tell the tester where the robot is on the field, the tester observes a known tag, and it solves for the camera pose relative to the robot.

**Spread** describes how far repeated answers differ from one another. A **residual** is the
remaining mismatch when a solved answer is checked against the observed geometry. **Range** is
distance to the tag. Compare these with independently measured geometry and your team's criterion;
a small spread alone can mean a repeatably wrong setup.

### Procedure

1. Place the robot in a pose you can describe confidently in the FTC field frame.
2. Open the exact `(Webcam)` or `(Limelight)` entry for the installed backend. In the device picker,
   Dpad Up/Down highlights, A chooses, and X refreshes. Wait for vision readiness.
3. In quick mode, Y increments and X decrements the tag ID. Dpad Left/Right edits known robot X,
   Dpad Up/Down edits known robot Y, LB adds yaw, and RB subtracts yaw. START selects the fine or
   coarse increments listed above. Right-stick click optionally enters field-by-field edit mode.
4. Match the displayed known robot pose and selected tag to the physical setup. Hold the robot still
   and press A several times to capture samples; B clears a bad set.
5. Record the printed `CameraMountConfig.ofDegrees(...)`, the selected device and tag, the physical
   pose, sample count, sample-to-average spread, residual, and range comparison.
6. Put the accepted mount in the canonical robot profile, rebuild, and open a fresh
   robot-configured AprilTag-localization tester. Verify the field pose there. Reopening the generic
   camera calibrator can check repeatability, but cannot prove that production consumed the value.

### What “good” looks like

- the solved mount translation is physically plausible for where the camera really sits
- repeated samples cluster closely
- `Sample vs avg mount` stays small when the robot is still
- `Avg residual` and the range check look reasonable instead of exploding

The tool defines no universal pass threshold for spread or residual. Its only large-mount warning is
triggered when translation magnitude exceeds `36 in`; that warning is a setup diagnostic, not an
acceptance limit. The team must compare the solve with measured geometry and repeatable samples.

### Common mistakes

- using the wrong tag ID
- typing the wrong robot field pose
- mixing up field axes or heading sign
- trying to calibrate while the robot is moving
- pasting the printed value into the wrong robot config field

### Record this result in code

Update the robot-owned camera mount profile when source is available. Otherwise preserve the exact
record for that owner; do not mark the production camera calibrated from the generic screen alone.

## AprilTag-only localization check

### Why this matters

First verify the tag observations and robot field-pose estimate on their own. **Fusion** combines
movement estimates with external position observations; it cannot repair a wrongly configured
camera mount just by combining more data.

### Framework-only entries and active defaults

- `Loc: AprilTag Localization (Webcam)`
- `Loc: AprilTag Localization (Limelight)`

The tester starts in `ANY` mode and accepts a detection frame up to `0.35 s` old. Its default
fixed-tag solver prefers an SDK-provided robot pose only when it agrees with the explicit geometry
solve within `8 in` and `12°`, and rejects multi-tag outliers beyond `18 in` or `25°`. These are
diagnostic software defaults, not robot-specific acceptance criteria.

The framework-only entries open a newly constructed identity-mount lane. They can prove that the
selected backend produces fresh raw detections and that fixed-layout metadata is present. Their
`fieldToRobot` answer cannot verify the mount you recorded or the production localization config.
Judge field pose only in a fresh robot-configured tester after the rebuild handoff.

The standard menu help says “Verify AprilTag detections and the field pose solve.” In this generic
entry, **verify** means inspect the default diagnostic path; it does not mean verify a recorded
mount or robot profile. Treat detection freshness and layout membership as the generic evidence.

Read **age** as time since observation, **range** as distance, and **bearing** as direction toward
the tag in the named frame. The **mean** is the average sampled pose; **standard deviation**
summarizes how much samples vary around that average. These describe the recorded samples, not
proof of absolute accuracy. Use the team's stated acceptance limits, not a universal number.

### Procedure

1. Open the exact `(Webcam)` or `(Limelight)` entry. In the picker, Dpad Up/Down highlights, A
   chooses, and X refreshes; wait for readiness.
2. Leave the initial `ANY` mode active to confirm fresh detections from fixed-layout tags. START
   toggles between `ANY` and `SINGLE`.
3. In `SINGLE`, Dpad Right or Y increments the tag ID; Dpad Left or X decrements it. Confirm the ID,
   age, range, bearing, and layout membership match the setup.
4. Press A several times while the robot is still to capture pose samples; B clears them. Inspect
   mean and standard deviation. BACK closes this camera owner and returns to the picker.
5. If this is the generic entry, record detection evidence and stop short of accepting field pose.
   After recording the mount in source and rebuilding, repeat with a fresh robot-configured tester
   and compare `fieldToRobot` with an independently known robot pose.

### What “good” looks like

- fresh detections appear without long gaps
- the selected tag matches what the camera is actually seeing
- in the robot-configured pass, the pose estimate is correct in translation and heading within the
  team's stated criterion
- captured configured-pass samples satisfy the team's stated stationary-jitter criterion

### Do not move on if

- detections are intermittent for no clear reason
- the solved pose is mirrored, rotated, or offset by a large amount
- a robot-configured lane still reports the camera mount as the identity placeholder
- the only apparently good field pose came from the framework-only identity-mount entry

## Pinpoint axis directions

### Why this matters

Odometry sign mistakes poison every later localization step. Fix them before tuning offsets.

### Tester

- `Calib: Pinpoint Axis Check`

The sample controls are exact: A toggles forward, Y toggles left, B toggles CCW rotation, and X
resets pose and clears every result. A translation sample must cover at least `6 in`; the rotation
sample must cover at least `20°`. The tester also requires `READY` Pinpoint pose evidence from the
current cycle before it accepts a sample.

### Procedure

1. Keep the robot still until the tester reports Pinpoint `READY`, then press X.
2. Press A, push the robot forward by hand at least `6 in`, then press A again.
3. Press Y, push the robot left by hand at least `6 in`, then press Y again.
4. Press B, rotate the robot CCW by hand at least `20°`, then press B again.
5. Record each delta and suggested config assignment. Put accepted changes in the robot Pinpoint
   profile, rebuild, then repeat all three samples in a fresh robot-configured tester.

### What “good” looks like

- forward motion produces positive X
- left motion produces positive Y
- CCW rotation produces positive heading

### Record this result in code

Set a robot-owned verification acknowledgement only after the rebuilt, robot-configured pass has
checked all three axes on real hardware. The generic tester neither sets nor persists that flag.

## Pinpoint pod offsets

### Why this matters

Axis directions only fix signs. Pod offsets fix the geometry. Leaving offsets at `0 / 0` makes rotation drift look like translation.

### Tester

- `Calib: Pinpoint Pod Offsets`

The framework-only entry can be the rookie manual path only when the installed pods use the default
`goBILDA_4_BAR_POD` resolution and the already-verified forward/lateral encoder directions are both
`FORWARD`. It actively applies those reconstructed defaults, not the profile values found by the
previous screen. If either fact differs, do not accept a generic offset solve; start with a fresh
robot-configured calibrator built from the reviewed profile.

The generic entry supplies neither a mecanum drive config nor a vision factory. The robot cannot
power itself and AprilTag assist is off. X resets pose and clears results, A advances the manual
sample, and B aborts; Y reports unavailable. The solve requires
`4 * sin(deltaHeading / 2)^2 >= 0.5`, roughly `45°` away from a degenerate `0°`/`360°` result, while
the tool recommends a turn near `180°`.

### Prerequisites

Run the manual path after Pinpoint axis directions and pod resolution are verified. Before using the
generic entry, confirm they match its two `FORWARD` directions and `goBILDA_4_BAR_POD` resolution;
otherwise use the robot-configured entry. The run requires current-cycle Pinpoint `READY` pose and
velocity evidence, clear floor space, and an independent way to judge whether the robot returned to
its starting position. A camera mount is not required unless a robot-specific advanced tester
enables AprilTag assist.

### Procedure

1. Confirm the selected generic-or-configured path satisfies the direction/resolution gate above.
   During INIT, start from a still robot and wait for current-cycle Pinpoint `READY` pose and
   velocity, then press X.
2. Press Driver Station START, then A to start the sample. Rotate the unpowered robot by hand
   roughly `180°` in place.
3. Press A to finish rotation and enter the default recenter phase. Physically translate the robot
   back to its starting point without adding rotation, then press A again to compute. Press B at
   any time to abort.
4. Record both recommended offset assignments and the observed heading change. Repeat the sample;
   reject a result that does not stabilize or fails the solve gate.
5. Put the accepted offsets in the canonical Pinpoint profile, rebuild, and run a fresh
   robot-configured pod-offset tester. Confirm the current offsets shown there are the rebuilt
   values and that a repeat sample produces only a small, stable recommendation.

### What “good” looks like

- repeated runs converge on similar offsets
- the last sample heading change is large enough for a stable solve
- the recommended offsets are physically plausible
- later fused localization no longer “slides” during turns with no real translation

### Common mistakes

- rotating too little
- rotating almost exactly 360 degrees, which makes the solve ill-conditioned
- treating real floor slip as an odometry-config problem
- accepting a recommendation without rebuilding and confirming the configured current offsets

### Record this result in code

Set a robot-owned “offsets calibrated” acknowledgement only after the rebuilt configured pass is
accepted. The generic tester prints assignments but persists neither the offsets nor the flag.

## Pinpoint plus field corrections

### Why this matters

This is the first true global-localization validation pass. At this point you are no longer asking whether each subsystem works in isolation. You are checking whether motion prediction and the chosen absolute correction source agree enough to trust the combined pose.

### Framework-only entries and baseline

- `Loc: Pinpoint + Field Corrections (Webcam)`
- `Loc: Pinpoint + Field Corrections (Limelight)`

`StandardTesters` selects raw AprilTag pose correction and the simpler `FUSION` estimator. The
baseline accepts AprilTag detections up to `0.50 s` old, but fusion corrections only up to `0.25 s`
old and quality at least `0.05`; position/heading gains are `0.25 / 0.35`, jump gates are `24 in` and
`60°`, and latency compensation retains `1.0 s` of predictor history. Corrections begin enabled.
These are software tuning defaults, not proof that they are appropriate for a robot.

The framework-only entries still combine an identity camera mount with newly reconstructed
Pinpoint defaults, changing only the selected device names. They are useful for inspecting the
independent streams and controls, but they cannot perform this section's production validation.
Use a fresh robot-configured tester after mount, directions, and offsets have been recorded and the
project rebuilt.

This screen may remind you to run **Calib: Camera Mount** or **Calib: Pinpoint Pod Offsets**. Those
messages name missing prerequisites; returning to this same generic entry still reconstructs
defaults and does not adopt the result. Rebuild and use the robot-configured tester for the final
check.

### Procedure

1. In the fresh robot-configured tester, start still where fixed tags are visible and wait for
   Pinpoint `READY` plus a plausible active correction.
2. Compare predictor, raw AprilTag, active correction, and corrected pose. START toggles only the
   raw preview between `ANY FIXED` and `SINGLE RAW PREVIEW`; Dpad Left/Right or Y/X changes its tag
   ID.
3. Press B to disable and re-enable correction while prediction continues. Watch accept/reject and
   replayed/non-replayed counts. Non-replayed includes direct corrections and supported
   motion-aligned projections; it does not promise a current pose. Check pose age separately
   from quality, and do not infer acceptance from a visually smooth number alone.
4. When the independent physical pose is known, A snaps the corrected estimator to the current
   active correction. RB instead rebases software pose to `(0,0,0)`; neither button claims that the
   robot physically moved.
5. Move the unpowered robot by hand along a controlled path with tags visible, temporarily hidden,
   then visible again. Verify smooth odometry-only prediction through the gap and bounded correction
   on reacquisition against the team's stated accuracy and jump criteria.

### What “good” looks like

- odom, vision, and fused estimates are broadly consistent
- the corrected pose does not jump unpredictably when corrections are enabled
- the fused pose keeps updating smoothly when tags disappear temporarily
- absolute corrections improve the estimate instead of fighting it

### Do not move on if

- fusion only looks good when the robot is perfectly still
- turning in place introduces obvious translation drift
- the camera mount or Pinpoint offsets are still known-bad

## Optional reference implementation details

This is source-level depth, not part of the rookie probe-and-record path. Read the
[referenced lift](<../build/Referenced Lift.md>) first; its reference, encoder, and Task vocabulary
is required here. Advanced periodic maps are detailed in
[FTC Actuators and Plants](<../ftc-boundary/FTC Actuators & Plants.md>).

### Common initialization choices

Use `alreadyReferenced()` when the selected measured native coordinate is already meaningful in
the plant coordinate. Examples: an absolute/source measurement already in degrees, or a modeled
simulation source already in plant units. A standard servo has command mapping instead, not this
measured-reference builder stage.

Use `plantPositionMapsToNative(plantPosition, nativePosition)` when the scale and one offset point are known in
code. Example: arm degrees mapped to encoder ticks with a measured zero tick.

Both values must be finite. Sushi rejects `NaN` and infinity immediately rather than clamping
either coordinate. The plant position is a coordinate-map anchor, not a target request, so it need
not lie inside the Plant's legal target range.

Use `assumeCurrentPositionIs(value)` only when the robot is physically placed at a known pose before
init. Its plant-unit answer must be finite. Example: the lift is manually collapsed before the
match, so the first finite encoder sample becomes plant position `0.0`; a non-finite sample leaves
the reference pending.

Use `needsReference(reason)` when the mechanism must find a switch, index mark, or reviewed custom
sensor condition before position targets are safe.

### Runtime homing/indexing task

A reference search is a normal non-blocking `Task`:

### Critical code

Replace the demonstration mechanism, cue, power, reference, and timeout with reviewed robot facts.

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...inside the mechanism's fresh homing-task factory...
Task search = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .failAfterSec(3.0)
        .build();

return Tasks.sequence(
        search,
        SemanticScalarTasks.set(heightCommand, Height.STOWED).build());
```

`.withPower(...)` requires a finite normalized command in the inclusive `[-1.0, +1.0]` range. It
rejects `NaN`, infinities, and overshoot immediately instead of clamping them into a different
search. Passing that check does not make the recipe mechanically safe: verify the magnitude,
direction, cue polarity/behavior, hard stops, and clearance on the actual robot.

`establishReferenceAt(...)` also requires a finite plant-unit coordinate and rejects `NaN` or
infinity at the recipe step, before search lifecycle effects. It is a reference anchor rather than
a target, so it is not clamped to `targetRange()`.

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
        .failAfterSec(5.0)
        .build();
```

**What to notice**

- Each attempt builds a fresh single-use `Task`; the mechanism remains the sole Plant heartbeat owner.
- The search stages temporary raw output and always preserves the persistent target graph.
- Success, timeout, and cancellation stop and release the search without changing its command.
- A success-only semantic continuation publishes through the mechanism's command owner; timeout
  and cancellation skip it and retain the latest coherent request.
- Finite software validation does not prove switch polarity, physical zero, clearance, or safe power.

**Key APIs**

- `PositionCalibrationTasks.search(plant)` — starts the non-blocking reference-task recipe.
- `until(BooleanSource)` — supplies the independently owned reference cue.
- `establishReferenceAt(...)` — anchors the public coordinate at the cue sample.
- `failAfterSec(...)` — gives the search a bounded lifetime.
- `Tasks.sequence(...)` / `SemanticScalarTasks.set(...).build()` — adds mechanism-owned semantic
  policy after exact success.

The search preserves the Plant's persistent command and final target resolver on every terminal
path. It requests a nonterminal stop of the temporary raw output and releases the search; it does
not call terminal `Plant.stop()`. The downstream Plant phase immediately evaluates the unchanged
graph. In the homing macro above, exact search success publishes the semantic request before that
Plant phase. Timeout and active cancellation stop the sequence before the request and retain the
prior—or any during-search superseding—semantic/numeric request. There is no need to preselect
STOWED before search because temporary search ownership already suspends target realization.

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

## High-resolution external encoder velocity comparison

**Optional advanced route:** the ordinary calibration procedures above do not require this section.
Use it only when you need evidence about direct versus position-derived velocity
from a high-count-rate external encoder.

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

## Optional advanced: powered and vision-assisted pod offsets

This is not supplied by the generic `StandardTesters` pod-offset entry. A robot-specific factory
must deliberately provide a complete mecanum config; an AprilTag lane-factory builder is a second,
independent option. Review motor names/directions and clear a large floor area before enabling it.

When those branches are active, the tester defaults to right-stick manual rotation scale `0.60`,
automatic omega `0.35`, a `180°` target, and left-stick recenter scale `0.60`. With drive and vision
assist, start/end tag searches default on at omega `+0.25`, require `3` stable frames, and allow up
to `4π` radians before the sample and `2π` afterward; automatic compute and post-turn recenter also
default on. These commands and limits are active powered values, not reviewed safe values.

Each automatic rotation phase also has its own elapsed-time limit. Set
`PinpointPodOffsetCalibrator.Config.automaticPhaseTimeoutSec` on the tool Config before constructing
the tester; its software default is `10.0` seconds, and a configured drive requires a finite value
greater than zero. Without a drive, this setting is ignored. A fresh timer starts when each
start-tag search, Y-initiated sample turn, or
end-tag search begins. It is not one shared budget for the complete sample. A start-tag search is
powered and timed even when A requested the following manual sample. Hand rotation, stick-driven
sample rotation, and manual recentering are not subject to this automatic-phase timer.

If a serviced loop reaches the deadline, the tester requests zero and discards the attempt before
another sensor poll or queued A/Y/X action can advance it. It retains the timeout reason and does
not produce an offset recommendation from that attempt. Inspect the cause, keep the robot still,
and retry only with a fresh button press in a later loop after the required evidence is ready.
This is a **cooperative timeout**: it is checked when the OpMode loop runs, not by an independent
hardware watchdog. It cannot interrupt a blocked loop or prove stopping distance; `10.0` seconds
is not a physically validated safe duration. Review the limit for the actual commands and clear
floor area, and keep a person at FTC STOP.

Successful ordinary INIT may configure the drive and poll Pinpoint/vision, but does not call the
ordinary drive command path. Driver Station START sends the first explicit zero. Y begins the
configured automatic turn; A begins or advances a manual sample, RightStickX rotates during its
powered rotation phase, LeftStick translates during recenter, and B aborts to zero. B also discards
A/Y/X actions queued in that same cycle, so another button cannot restart or advance the aborted
attempt. Every motion path requires current-cycle Pinpoint `READY` pose and velocity and aborts if
that evidence disappears. Vision assist disables itself when the opened lane still reports an
identity mount.

## Optional EKF comparison

### Why this is optional

An **EKF** (extended Kalman filter) combines estimates using a model of uncertainty. **Covariance**
describes uncertainty and how errors vary together; an **innovation** is the difference between a
new observation and its prediction. This comparison needs the optional
[localization explanation](<../drive-vision/AprilTag Localization & Fixed Layouts.md>) and credible
hardware calibration. Debug the simpler fusion estimator first; this is not an opening lesson.

`StandardTesters` does **not** register an EKF entry. A robot-specific suite must construct a
corrected-localization tester with `GlobalEstimatorMode.EKF` and the robot's complete configured
profile; its menu wording belongs to that suite. There is no generic EKF menu label.

### Procedure

1. Get the default fusion tester into a trustworthy state first.
2. Run the EKF tester on the same path.
3. Compare correction behavior, lag, and stability.
4. Only keep the EKF if it is clearly helping for your robot and floor conditions.

### What “good” looks like

- the EKF agrees with the simpler fusion path most of the time
- innovations are reasonable instead of constantly huge
- the estimated uncertainty behaves like a useful readiness signal, not noise

## Optional advanced: guided suite construction

The rookie path does not require a custom tester registry. Teams that already own checked-in robot
profiles and fresh robot-configured tester factories can order those existing facts with
[`Guided calibration walkthroughs`](<Guided Calibration Walkthroughs.md>). The walkthrough adds
status and ordering; it does not persist results or replace the record → rebuild → fresh configured
tester → verify handoff.

## How this maps to the tester menus

Use **HW: Actuator Bring-up** for ordinary motor/servo hardware facts, **Framework: Calibration &
Localization** for camera/pose work, and **Advanced: Hardware Diagnostics** only for a distinct
controller or measurement investigation. A robot-specific walkthrough may order those checks for
one robot, but it does not create a second generic actuator path.
