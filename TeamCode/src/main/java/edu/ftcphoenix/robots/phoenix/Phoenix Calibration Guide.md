# Phoenix calibration guide

This is the Phoenix-specific bring-up path.

Use it when you want the exact Phoenix menu names and the exact `PhoenixProfile` fields to edit while bringing up a fresh robot.

---

## 1. Ownership map

Phoenix keeps calibration ownership intentionally clean:

- `PhoenixProfile.drive` -> drivetrain hardware/wiring/brake tuning
- `PhoenixProfile.vision` -> active vision backend plus backend-specific rig config
- `PhoenixProfile.localization` -> predictor, AprilTag-localization tuning, correction-source choice, and corrected-global estimator tuning
- `PhoenixProfile.field` -> shared field facts such as the fixed AprilTag layout
- `PhoenixCapabilities` -> shared mode-neutral robot API and public status snapshots used by TeleOp and Auto
- `PhoenixTeleOpControls` -> TeleOp stick/button semantics
- `.scoring.PhoenixScoring` -> scoring requests, policy, all scoring Plants, update order, and status production
- `.scoring.PhoenixTargeting` -> selected-tag policy, aim status, and shot suggestions
- `PhoenixReadiness` -> immutable mode-specific calibration/field/route warnings and blockers
- `PhoenixRobot` -> composition root; declares managed TeleOp or Auto roles
- `RobotProgram` -> managed mode clock, phase order, prestart policy, telemetry commit, and cleanup

That ownership split matters during bring-up because fixes should land in the owner of the behavior:

- drivetrain wiring / brake / drive tuning -> `PhoenixProfile.drive`
- backend choice + active camera rig -> `PhoenixProfile.vision`
- webcam name / VisionPortal-backed rig config -> `PhoenixProfile.vision.webcam`
- Limelight hardware name / pipeline / poll rate -> `PhoenixProfile.vision.limelight`
- predictor, AprilTag solve, and corrected-global localization tuning -> `PhoenixProfile.localization`
- field-fixed tag policy or practice-field overrides -> `PhoenixProfile.field`


### Actuator mapping facts

Phoenix's shooter-transfer CR-servo group keeps one
`PhoenixProfile.scoring.shooterTransferLeftScale` value. It is a configured dimensionless speed ratio
for the last-added left child, after the two `Direction` values account for physical mounting. There
is no transfer bias field: normalized power zero must map to exact zero for both CR servos, while a
nonzero additive offset would make an active zero command disagree with stop.

The framework validates this scale as finite and preflights the complete `[-1.0, +1.0]` group image
before fresh hardware effects. That numeric proof does not establish the correct physical ratio;
confirm direction, relative speed, loading, and transfer behavior on the robot. If a mechanism
needs additive or nonlinear per-wheel intent, model that pair inside its owner
with two private Plants rather than hiding another command in an actuator map.


---

## Mechanism position calibration pattern

Phoenix mechanism services should use the framework position-Plant vocabulary when a mechanism needs
software limits, logical units, homing, or indexing. The service owns the robot meaning of the
positions; the framework Plant owns the public plant coordinate and the hardware-native mapping.

Recommended builder flow inside the owning mechanism/service constructor:

`PhoenixRobot` should pass that owner `HardwareMap` plus its validated `PhoenixProfile` slice. The
owner constructs and keeps the Plant; `PhoenixRobot` should not construct the Plant and inject it
back into the owner. The compact fragment below omits that surrounding constructor so the
calibration stages remain visible.

```java
// Inside the mechanism constructor; lift is a private field.
this.lift = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .deviceManaged()
        .nonPeriodic()
            .bounded(0.0, 18.0)              // plant units: inches
            .scaleToNative(TICKS_PER_INCH)   // native units: encoder ticks
            .needsReference("lift not homed")
        .positionTolerance(0.10)             // plant units
        .targetFromNewCommand(0.0)
        .build();
```

Use these rules when adding Phoenix mechanisms:

- **Plant units** are the units Phoenix services, presets, scalar planners, and telemetry should
  speak in. They may be inches, degrees, logical servo units, or encoder ticks.
- **Native units** are only mentioned by APIs with `Native` in the name, such as
  `scaleToNative(...)`, `rangeMapsToNative(...)`, `plantPositionMapsToNative(...)`, or native FTC tuning
  methods like `devicePositionToleranceTicks(...)`.
- Use `alreadyReferenced()` only when the selected native coordinate is already meaningful at robot
  init. For a relative motor encoder on a lift, prefer `needsReference(...)` plus a calibration task,
  or `assumeCurrentPositionIs(...)` when the robot is physically placed at a known pose before init.
  The latter uses the first finite native sample; a non-finite sample leaves the reference pending.
- Every explicit plant/native reference value must be finite and is rejected rather than clamped.
  A reference is a coordinate anchor, not a target command, so its plant value need not lie inside
  the Plant's target range.
- A Plant has no catch-all reset; runtime homing/indexing is an explicit `Task`, while
  `Plant.stop()` permanently ends that Plant instance.

Example homing task:

```java
Task homeLift = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .holdAfterReference(0.0)
        .failAfterSec(3.0)
        .build();
```

`.withPower(...)` accepts only a finite normalized command in the inclusive `[-1.0, +1.0]` range
and rejects `NaN`, infinities, and overshoot immediately rather than clamping them. Phoenix's
mechanism service must still choose and validate the safe magnitude, direction, cue behavior, and
mechanical setup for its hardware.

The plant-unit reference supplied to `establishReferenceAt(...)` and the separate post-success
command supplied to `holdAfterReference(...)` must also be finite. Each rejects `NaN` or infinity
at its builder step without clamping or overwriting an earlier accepted answer. The reference is not
range-limited; the finite hold remains a normal logical command that the complete resolver, range,
overlays, and target guards may transform or clamp. Choose a deliberately safe in-range hold when
the mechanism should predictably hold that exact position.

Create a fresh search Task for each homing attempt. Search Task objects follow the framework's
single-use lifecycle and are not restarted after they have begun.

For a Phoenix mechanism, `RobotProgram` advances Tasks before calling the owning
mechanism's update in both TeleOp and Auto. The search Task owns its cue, reference, timeout, and
handoff recipe but never calls `lift.update(clock)`; the mechanism remains the sole Plant heartbeat
owner. If the search is still active, that one downstream update submits its staged power. If the
Task releases the search first, the same phase evaluates the normal final resolver.

`holdAfterReference(0.0)` changes the Plant's graph-owned command before that handoff; the complete
resolver can still mask or transform it. Use `resumeTargeting()` when success should preserve the
existing persistent command and unchanged resolver. That option does not leave the continuously
updated Plant disabled. Timeout and active cancellation also request the same internal,
nonterminal temporary-output stop and release the search without changing the persistent command;
none of those calibration handoffs calls terminal `Plant.stop()`.

The timeout policy is explicit: use `failAfterSec(...)` for a bounded search or `neverTimeout()` only when another safety path is guaranteed to cancel the task.

For periodic mechanisms such as trays or turrets, `establishReferenceAt(...)` establishes the
reference modulo the Plant period and preserves the nearest equivalent unwrapped position from the
cue cycle's current native sample when the Plant was already referenced. When the current plant
estimate is finite, the Plant rejects a non-finite final nearest-equivalence result before committing
the new reference. The complete candidate bounded affine map and derived public measurement must
also remain finite before reference state changes. Each later realized FTC command still passes its
raw-domain check. An unbounded core Plant-to-native conversion is checked individually before
applied-target state or output.

These finite-value checks do not prove the physical pose, mapping scale/sign, cue, travel, or hold
is correct or safe. The Phoenix mechanism owner must validate those facts on its robot.

## 2. Where to start in the tester menu

Recommended starting point:

- `Guide: Phoenix Calibration Walkthrough`

That walkthrough links to the real testers in the recommended order.

If you already know what you need, browse instead through:

- `Phoenix: Configured Hardware Verification`
- `Phoenix: Calibration & Localization`

---

## 3. Step-by-step bring-up

### Step 1: raw actuator direction and optional endpoints

Menu entry:

- `HW: Actuator Bring-up`

Goal:

- isolate one configured motor or servo, establish its tested FTC `Direction`, and capture only
  already-backed-off safe native endpoints when the mechanism is genuinely bounded

Follow the canonical [`actuator bring-up runbook`](<../../fw/docs/testing-calibration/Actuator Bring-up.md>).
Copy its result into `PhoenixProfile` or the owning mechanism config, then rebuild. The tool does not
edit Phoenix code, choose Plant units, establish an incremental encoder's runtime zero, or tune
PIDF.

---

### Step 2: configured drivetrain verification

Menu entry:

- `HW: Configured Drivetrain Verification`

Goal:

- verify that the motor names and directions already stored in the Phoenix profile select the
  expected wheel and make positive power contribute to robot-forward motion

Before this Phoenix-specific check, complete the preceding canonical
[`HW: Actuator Bring-up`](<../../fw/docs/testing-calibration/Actuator Bring-up.md>) tool to establish
each raw motor's FTC `Direction`. Copy those facts into the profile and rebuild. The configured
drivetrain verifier does not discover direction and does not edit the profile.

Fix in code:

```java
PhoenixProfile.current().drive.wiring
```

Run the verification safely:

1. Raise and secure the chassis so all four wheels are clear. Keep people, wires, and loose items
   away from the wheels.
2. Enter the tester. INIT is presentation-only and never updates or writes its drivetrain Plants.
3. Press Driver Station START, then release A/B/X/Y once so the controls arm from neutral.
4. Hold exactly one mapped button: X = front-left, Y = front-right, A = back-left, B = back-right.
   Releasing the button commands every motor to zero. Holding two or more mapped buttons also
   commands every motor to zero.
5. Confirm that only the named wheel turns and that its positive motion would contribute to
   robot-forward travel. STOP commands all four motors to zero.

This isolated test proves only that the checked-in Phoenix names/directions reach the expected
motors under controlled software conditions. Real hardware observation is still required. For the
final drivetrain integration verification, keep every wheel raised, run the production Phoenix
TeleOp, and test forward, strafe, and turn separately. That final check exercises the real TeleOp
controls, mecanum mixing, configured drivetrain, and output path together.

---

### Step 3: flywheel velocity control

OpMode:

- **Phoenix: Tuning (Panels)**

This OpMode opens the flywheel tuner directly; there is no tester menu or second selection.

Goal:

- tune the FTC device-managed velocity controller through a fresh flywheel Plant built from
  `PhoenixScoring`'s canonical production recipe, with the same bounds, mapping, feedback, and
  update path but a separate exclusive lifetime

Before INIT, confirm the shooter motor's direction and encoder sign, secure the robot, clear the
firing path, and connect exactly one Panels client. Keep physical access to robot power; browser
STOP is not an emergency stop. Zero or multiple clients, a disconnect, or an input failure ends the
run closed, and reconnecting does not rearm it.

The ordinary test loop is:

```text
edit controller.* plus experiment.targetVelocity and experiment.autoStopAfterSec
-> Panels Update All
-> A
-> observe the numbered segment and charts
```

Repeat that loop while the wheel is running for a hot target/controller comparison without
deliberately requesting zero. Press B when you want zero and a cold baseline. The first A and every
A after B wait until measured speed is finite and the Plant truthfully reports `atTarget(0.0)`;
zero command is not proof that inertia has stopped the wheel. For Phoenix, that arrival check uses
the checked-in `velocityToleranceNative`. `autoStopAfterSec > 0` requests zero after that segment's
duration, while exactly `0` runs until B, BACK, disconnect, failure, or OpMode stop.

Panels Update All changes only the browser draft. Wait for it to finish and verify every displayed
active field before pressing A. The OpMode loop then snapshots the ordered map after a short
quiescence interval; Configurables has no atomic batch marker, so that delay is a filter rather than
an atomicity guarantee. Invalid or still-changing values reject the whole candidate and leave a
running segment unchanged. Read telemetry as separate facts: browser draft, captured request, exact
controller readback, immutable segment history, target/measurement/error graphs, and response
metrics. Transition labels say whether controller, target, both, or neither changed; each history
record still retains the complete accepted request.

For distance-to-basket trials, use the displayed short session ID and segment ID in a lab sheet:

```text
distanceIn | sessionId | segmentId | targetVelocity | shots | makes | accepted | notes
```

Phoenix does not infer a made shot from controller error and does not store robot-specific metadata
inside the tuner. Record several shots for one segment, then press A for a new segment when target
or gains change.

After the team reviews the trials, sort only the accepted finite rows by strictly increasing
distance and replace the checked-in Phoenix shot table:

```java
PhoenixProfile.current().autoAim.shotVelocityTable =
        InterpolatingTable1D.ofSortedPairs(
                28.0, 1500.0,
                36.0, 1430.0,
                50.0, 1450.0);
```

Each adjacent pair is `(rangeInches, flywheelVelocityNative)`. The table owns validation of every
finite value and rejects duplicate or out-of-order distances when Phoenix constructs the profile;
do not add a profile-local validation loop. During a match, a fresh tag alone does not prove that
its derived range is finite. Phoenix publishes a shot suggestion only when the table result is
finite, so unavailable live geometry cannot masquerade as a clamped endpoint shot.

The FTC controller may quantize gains, so copy the displayed **readback values** into:

```java
PhoenixProfile.current().scoring.applyFlywheelVelocityPIDF = true;
PhoenixProfile.current().scoring.flywheelVelKp = /* displayed readback */;
PhoenixProfile.current().scoring.flywheelVelKi = /* displayed readback */;
PhoenixProfile.current().scoring.flywheelVelKd = /* displayed readback */;
PhoenixProfile.current().scoring.flywheelVelKf = /* displayed readback */;
```

Then stop the tuner, review and commit those profile edits, rebuild, and start a fresh production
TeleOp or Auto. Production never reads Configurables. BACK, STOP, disconnect, or failure
terminally stops the flywheel Plant and best-effort restores the controller configuration captured
for the tuning session, but no software restore can undo physical motion or prove controller
history. Validate hot-transition dips, spikes, oscillation, shot behavior, and safe targets on the
robot. See the complete [`control tuning workflow`](<../../fw/docs/testing-calibration/Control Tuning Workflow.md>).

---

### Step 4: camera mount

Menu entry:

- `Calib: Camera Mount (Robot)`

Goal:

- solve the active AprilTag vision device pose relative to the robot

Paste result into:

```java
PhoenixProfile.current().vision.webcam.cameraMount    // when backend = WEBCAM
PhoenixProfile.current().vision.limelight.cameraMount // when backend = LIMELIGHT
```

The tester prints both `CameraMountConfig.of(...)` and `CameraMountConfig.ofDegrees(...)`. Paste one of those directly into the active backend config.

### How webcam and Limelight are used so far

For Phoenix's current AprilTag use case, both backends expose the same narrow seam:

```java
AprilTagVisionLane vision = PhoenixVisionFactory.create(hardwareMap, PhoenixProfile.current().vision);

FtcOdometryAprilTagLocalizationLane localization =
        new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                PhoenixProfile.current().field.fixedAprilTagLayout,
                PhoenixProfile.current().localization
        );
```

That constructor is the normal TeleOp/calibration path and creates the profile-configured Pinpoint
predictor. Phoenix Pedro Auto instead maps the relevant profile slice to data and crosses the sole
runtime hardware boundary:

```java
PedroPathingRuntime runtime = PedroPathingRuntime.create(
        hardwareMap,
        Constants.phoenixAutoRuntimeConfig(profile));
```

The pure mapper snapshots the predictor, drivetrain wiring, and brake choice and combines them with
fresh checked-in Pedro tuning; it constructs no hardware. The runtime validates and owner-copies
the complete graph before hardware lookup or its non-blocking Pinpoint reset request, then passes
its sole predictor into `FtcOdometryAprilTagLocalizationLane.withPredictor(...)`. Phoenix must not
construct Pedro's native Pinpoint localizer as a second production owner.

Pedro's generated tuning menu and `PedroTest` use a package-local tool-only native factory instead.
That exclusive OpMode graph derives the same hardware name, offsets, resolution, directions, and
yaw scalar from the profile, but owns Pedro's native `PinpointLocalizer` and raw Follower heartbeat.
It is not a production runtime option and must never coexist with Phoenix Auto. The separate public
completed-Follower adapter constructor is only for an advanced custom/portable host that has
already constructed the vendor graph and will route its lifecycle through the adapter; it acquires no
Pinpoint hardware.

The backend only changes which concrete AprilTag lane is created:

- `Backend.WEBCAM` -> `FtcWebcamAprilTagVisionLane`
- `Backend.LIMELIGHT` -> `FtcLimelightAprilTagVisionLane`

The selected concrete owner validates and snapshots its full backend Config before device lookup;
the inactive profile branch is not opened or treated as calibrated. Profile copies keep a custom
FTC webcam tag library as borrowed draft data, while the active webcam owner canonicalizes its
units and deep-snapshots its mutable metadata. Change the checked-in profile and restart the OpMode
to adopt different metadata or solver tuning; neither is a live-tuning surface.

Phoenix's tester factories map the relevant profile facts into one fresh tool Config and pass the
active backend recipe separately as a `Function<String, AprilTagVisionLaneFactory>`. That function
captures the selected webcam or Limelight template immediately; a later picker retry does not reread
the broad mutable profile. The tester snapshots the fixed layout, while the backend Config remains
the sole camera-mount and detector-library owner. Keep any borrowed custom SDK tag library stable for
the complete tester lifetime and every clean retry.

Intrinsic layout, AprilTag policy, predictor, source-selection, and selected Fusion/EKF errors fail
before portal or Pinpoint effects. Actual returned-lane subtype, accessors, and readiness can only be
checked after open. Non-null `NOT_READY` remains pending; a null contract fact or `RuntimeException`
closes the published lane once when cleanup succeeds. An `Error` propagates immediately and leaves
no cleanup guarantee. If the lane remains published and STOP is later invoked, that boundary closes
the still-retained owner. The layout display retains the FTC policy summary and
captured IDs/poses, deliberately not a mutable source alias for richer per-key rows. An empty layout
stays empty and cannot produce a fixed-layout sample or raw-AprilTag field correction. These software
checks still do not prove the physical camera, mount, library, field placement, or calibration.

That does not make the devices identical. A webcam portal may run its construction-time processor
set concurrently; a Limelight runs one onboard pipeline and must confirm a fresh result after each
requested change. Phoenix displays `vision.componentReadiness` and `vision.readinessReason` every
loop independently of target visibility. A ready lane may legitimately see no tags.

Custom multi-purpose vision keeps the concrete advanced owner in a robot realization:
`FtcWebcamVisionPortalLane` maps semantic modes to processor enablement, while
`FtcLimelightVisionLane` maps them to one pipeline request at each transition. Auto and TeleOp
should consume one robot-owned immutable timestamped snapshot rather than FTC or Limelight result types.

Phoenix can use Limelight's direct device field pose as an **optional** correction source through
`PhoenixProfile.localization.estimation.correctionSource`, while the raw AprilTag path remains
available.
Limelight's FTC SDK exposes both fiducial-result access and direct botpose / MT2 pose access.

### Phoenix notes

- the preferred AprilTag device name is `PhoenixProfile.current().vision.activeDeviceName()`
- the walkthrough status turns `OK` once the active backend's camera mount no longer looks like the identity placeholder

---

### Step 5: AprilTag-only localization sanity check

Menu entry:

- `Loc: AprilTag Localization (Robot)`

Goal:

- verify that Phoenix's active AprilTag backend, fixed-tag layout policy, and raw AprilTag field solve produce a believable field pose before odometry is fused in

What to watch:

- fresh detections
- correct selected tag ID
- stable `fieldToRobot` pose
- low sample jitter while stationary

This tester reuses:

```java
PhoenixProfile.current().vision.activeDeviceName()
PhoenixProfile.current().vision.activeCameraMount()
PhoenixProfile.current().localization.estimation.aprilTags
PhoenixProfile.current().field.fixedAprilTagLayout
```

So the practice tool should match production AprilTag-solving math closely.

---

### Step 6: Pinpoint axis directions

Menu entry:

- `Calib: Pinpoint Axis Check (Robot)`

Goal:

- verify +X forward, +Y left, heading CCW+

Keep the robot stationary while the constructor-requested Pinpoint reset is calibrating. The tester
will not accept a sample until its cached device status is `READY` and a measured pose is available;
there is no blocking reset delay hidden in the OpMode.

Fix in code:

```java
PhoenixProfile.current().localization.predictor
```

Specifically, correct pod direction fields before continuing.

The encoder-resolution field is one tagged choice, so a custom value cannot silently override a
second preset field. The default is the goBILDA 4-bar pod. Select exactly one form:

```java
PinpointOdometryPredictor.Config predictor =
        PhoenixProfile.current().localization.predictor;
predictor.encoderResolution =
        PinpointOdometryPredictor.EncoderResolution.forGoBildaPod(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

// Or, for a genuinely custom encoder/pod whose calibration has been reviewed:
predictor.encoderResolution =
        PinpointOdometryPredictor.EncoderResolution.ticksPerInch(reviewedTicksPerInch);
```

Passing software validation proves only that the selected value is representable by the Pinpoint
driver. Confirm the physical pod model or measured ticks-per-inch on the robot.

Likewise, the Pedro runtime's full Config validation can prove only a complete, finite, internally
coherent captured graph. It cannot prove the selected ports, motor directions, pod placement,
reset-to-READY behavior, follower stability, field alignment, route clearance, stopping distance,
or physical STOP. Those facts require the corresponding tester and a controlled robot run.

Record completion after rerunning the tester and accepting the result:

```java
PhoenixProfile.current().calibration.pinpointAxesVerified = true;
```

Until this acknowledgement is true, Phoenix keeps TeleOp auto-aim and shoot-brace unavailable and
blocks every Pedro Auto, including the dedicated integration test. Manual TeleOp drive and
mechanisms remain available, with the required tester named in Driver Station telemetry.

---

### Step 7: Pinpoint pod offsets

Menu entry:

- `Calib: Pinpoint Pod Offsets (Robot)`

Goal:

- estimate the Pinpoint offsets that remove fake translation during rotation

Automatic and stick-driven calibration motion fail-stops whenever the current Pinpoint poll lacks
`READY` pose and velocity evidence. If that happens, keep the robot still and restart the sample
after the status returns to `READY`.

Paste result into:

```java
PhoenixProfile.current().localization.predictor
```

The tester prints the two recommended field assignments:

```java
predictor.forwardPodOffsetLeftInches = forwardPodOffsetLeftInches;
predictor.strafePodOffsetForwardInches = strafePodOffsetForwardInches;
```

Record completion after copying the numbers and rerunning once to confirm they are stable:

```java
PhoenixProfile.current().calibration.pinpointPodOffsetsCalibrated = true;
```

Phoenix enables AprilTag assist for this tester only once the active backend's camera mount looks
solved enough to trust. The assist then uses the same checked-in fixed layout, detection-age limit,
and fixed-tag solver policy as production localization rather than a tool-local fallback.

During successful ordinary INIT the tester may configure the drivetrain and poll/reset Pinpoint, but
it does not read/write drive mode or power. Driver Station START issues the first ordinary zero;
only a later RUN loop with exact current-cycle `READY` pose and velocity evidence may command motion.
A failed-init rollback or STOP-before-START may still command physical zero as cleanup.

Until this acknowledgement is true, match Auto remains blocked and both pose-dependent TeleOp
assists remain unavailable. The explicitly named Pedro integration-test OpMode may still run after
the axes check, but it shows a persistent uncalibrated-offset warning. That exception supports
bounded calibration work; it does not make the route or localization match-ready.

### Readiness shown by production OpModes

Phoenix TeleOp and Auto both require their selected alliance scoring tag in
`PhoenixProfile.autoAim.scoringTargets` and `PhoenixProfile.field.fixedAprilTagLayout`. Match Auto
additionally requires route geometry deliberately classified `MATCH_READY` by
`PhoenixPedroPathFactory`. Current checked-in placeholder geometry is `INTEGRATION_ONLY`, so static
match entries and the selector correctly show `BLOCKED` even after localization calibration is
complete. The dedicated Pedro test entry alone can show `TEST` and run that geometry.

Auto INIT displays the exact expected Pedro-field x/y/heading (inches/degrees) for physical
placement. Setting the Pedro start pose rebases the software coordinate system; it does not sense
where the chassis is on the field. Place the robot at the displayed pose before START.

---

### Step 8: default corrected-global localization validation

Menu entry:

- `Loc: Pinpoint + Field Corrections (Robot)`

Goal:

- validate Phoenix's default corrected-global localizer in the conditions that matter for real operation:
  - tags visible at the start
  - movement while corrections are available
  - temporary tag loss near the target
  - smooth continuity while relying on prediction alone
  - clean correction when tags come back

Config involved:

```java
PinpointAprilTagCorrectedLocalizationTester.Config.defaults()
PhoenixProfile.current().vision
PhoenixProfile.current().localization.predictor
PhoenixProfile.current().localization.estimation.aprilTags
PhoenixProfile.current().localization.estimation.correctionSource
PhoenixProfile.current().localization.estimation.correctionFusion
PhoenixProfile.current().field.fixedAprilTagLayout
```

Phoenix defaults to:

- predictor = Pinpoint odometry
- absolute correction source = raw AprilTag field solve (`APRILTAG_POSE`)
- corrected estimator = gain-based fusion (`FUSION`)

### Trying direct Limelight field pose

If you want the corrected/global estimator to use Limelight's direct device field pose instead of the raw AprilTag field solve:

```java
PhoenixProfile.current().vision.backend = PhoenixProfile.VisionConfig.Backend.LIMELIGHT;
PhoenixProfile.current().localization.estimation.correctionSource.mode =
        FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.LIMELIGHT_FIELD_POSE;
PhoenixProfile.current().localization.estimation.correctionSource.limelightFieldPose.mode =
        LimelightFieldPoseEstimator.Config.Mode.BOTPOSE_MT2;
```

Start conservatively:

- keep the raw AprilTag tester available as your sanity check
- tighten `maxResultAgeSec`
- require at least 2 visible tags if direct pose looks noisy in motion
- keep motion-aware degradation enabled

Phoenix's direct Limelight path assumes the Limelight field map matches the field/tag layout you
intend to use.

---

### Step 9: optional EKF comparison

Menu entry:

- `Loc: Pinpoint + Field Corrections EKF (Robot)`

Goal:

- compare the optional covariance-aware corrected localizer against the default fusion path

Do this only after:

- camera mount is calibrated
- Pinpoint axis directions are verified
- Pinpoint pod offsets are measured
- the default corrected-global tester already looks trustworthy

Config involved:

```java
PhoenixProfile.current().localization.estimation.correctionEkf
PhoenixProfile.current().localization.estimation.correctedEstimatorMode
```

Use the tester to compare behavior first. Only then consider changing the robot's default corrected estimator mode.

---

## 4. The localization model Phoenix is using

Phoenix treats localization as three different roles:

- `MotionPredictor` -> short-term motion propagation (`PinpointOdometryPredictor`)
- `AbsolutePoseEstimator` -> field-anchored absolute pose (`AprilTagPoseEstimator`, optional `LimelightFieldPoseEstimator`)
- `CorrectedPoseEstimator` -> combines a predictor with one absolute correction source (`OdometryCorrectionFusionEstimator` or `OdometryCorrectionEkfEstimator`)

That split keeps each extension at the role whose contract it satisfies.

The raw AprilTag estimator composes camera freshness/mount data with a
`FixedTagFieldPoseSolver.Config`. Phoenix targeting constructs its own configured
`FixedTagFieldPoseSolver` from the same profile policy before building guidance plans. The two
runtime owners therefore use the same authored policy without sharing a mutable Config.

Other additions that fit this model include:

- field tape / line tracking that can directly anchor robot pose -> another `AbsolutePoseEstimator`
- wheel + IMU dead-reckoning -> another `MotionPredictor`
- smarter absolute-source selection policies -> a higher-level `AbsolutePoseEstimator` wrapper

---

## 5. Quick checklist for a fresh Phoenix robot

1. establish raw drivetrain motor direction with `HW: Actuator Bring-up`
2. run Phoenix configured-drivetrain verification with all wheels raised
3. run the production TeleOp raised-wheel forward/strafe/turn integration check
4. tune the installed flywheel through `Phoenix: Tuning (Panels)`, copy controller readback, and
   adopt reviewed finite distance/velocity rows
5. camera mount
6. raw AprilTag localization check
7. Pinpoint axis directions
8. Pinpoint pod offsets
9. default corrected-global localization validation
10. confirm TeleOp reports pose assists `READY`
11. confirm the intended Auto has calibrated field facts and `MATCH_READY` geometry
12. optional EKF comparison

---

## 6. Related docs

- [`Framework Overview`](<../../fw/docs/getting-started/Framework Overview.md>)
- [`AprilTag Localization & Fixed Layouts`](<../../fw/docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`Framework Lanes & Robot Controls`](<../../fw/docs/design/Framework Lanes & Robot Controls.md>)
- [`Robot Calibration Tutorials`](<../../fw/docs/testing-calibration/Robot Calibration Tutorials.md>)
- [`Guided Calibration Walkthroughs`](<../../fw/docs/testing-calibration/Guided Calibration Walkthroughs.md>)

## Periodic mechanism calibration pattern

An independently rotating turret or tray uses the framework's periodic mechanism pattern.

A mechanism service should establish a calibrated native coordinate before resolving
`PlantTargets.equivalentPositionsOf(...)` commands or advanced `PlantTargets.plan(request)` requests:

- **measurement source**: the value the `Plant` understands, such as encoder ticks
- **zero/reference**: the raw encoder value corresponding to the mechanism's meaningful zero
- **period**: for repeated orientations, such as ticks per turret revolution or tray revolution
- **range**: the advanced Plant/planner protocol uses `ScalarRange.bounded(finiteMin, finiteMax)`
  for a finite travel window, `ScalarRange.boundedFrom(finiteMin)` or
  `ScalarRange.boundedTo(finiteMax)` for a genuine one-sided limit, and
  `ScalarRange.unbounded()` for a free spinner; every supplied endpoint is finite, never an infinity
  sentinel
- **validity**: before homing completes, publish `ScalarRange.invalid("not homed")` as runtime
  unavailability so the target resolver uses its explicit `whenUnavailable()` policy instead of
  producing an unsafe request; no range shape ever makes `NaN` or infinity a legal target

For a turret with a camera, the camera mount may be timestamp-aware. A moving camera should be represented by a history-backed source so delayed AprilTag frames are interpreted using the camera pose from the frame timestamp, not the current turret pose after the mechanism has moved.

See the framework docs:

- `fw/docs/drive-vision/Spatial Queries.md`
- `fw/docs/drive-vision/Mechanism Target Planning.md`
- `fw/docs/drive-vision/Drive Guidance.md`
