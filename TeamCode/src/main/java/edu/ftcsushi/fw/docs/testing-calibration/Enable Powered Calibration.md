---
tags:
  - Test & Tune
---

# Enable powered calibration

**Learning mode:** Integrated example

**Outcome:** deliberately enable a reviewed drivetrain for pod-offset calibration, with unassisted
and AprilTag-assisted checks kept as separate choices.

**Before this page:** [Add calibration testers to your robot](<Add Calibration Testers to Your Robot.md>).
For assistance only, also read [Add vision to your calibration suite](<Add Vision to Your Calibration Suite.md>).
The unassisted branch requires no camera. Reading needs no hardware; powering the robot requires
[actuator bring-up](<Actuator Bring-up.md>) and the
[powered/assisted physical runbook](<Robot Calibration Tutorials.md#optional-advanced-powered-and-vision-assisted-pod-offsets>).

## Review motion before constructing its owner

The basic manual tester contains no drive. A powered calibration tester is different: it owns and
can command all four drivetrain motors. A boolean is a Java true/false choice; the example's
`poweredMotionReviewed` starts `false`. It records a human review, not a framework safety verdict.

Before changing it, establish the motor names/directions, Pinpoint axes and pod model, tested
command envelope, clear floor area, start/abort controls, and a person responsible for physical FTC
STOP. Record the reviewed configuration revision. A successful build or a value named `defaults()`
does not authorize motion on your robot.

[`CalibrationTesters.poweredPodOffsets(profile)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.html#poweredPodOffsets(edu.ftcsushi.robots.examples.calibration.CalibrationRobotProfile)>)
checks that review before constructing a drive Config or tester:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
public static PinpointPodOffsetCalibrator poweredPodOffsets(CalibrationRobotProfile profile) {
    CalibrationRobotProfile captured = capture(profile);
    requireReviewedMotion(captured);
    return new PinpointPodOffsetCalibrator(poweredPodConfig(captured), null);
}
```

`requireReviewedMotion` is a private example check that throws an actionable error when the flag is
false. Calling this factory directly cannot bypass the menu's review condition. The final `null`
means this is deliberately unassisted: no camera is constructed even when the profile selects one.

## Map the same drivetrain facts the robot uses

[`CalibrationRobotProfile.mecanum()`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.html#mecanum()>)
returns a fresh [`FtcDrives.MecanumConfig`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcDrives.MecanumConfig.html>).
Mecanum means the four-wheel drive arrangement used by this example. Another robot must supply
facts supported by the selected tester, not a fabricated four-motor mapping for unrelated hardware.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.java -->
```java
public FtcDrives.MecanumConfig mecanum() {
    FtcDrives.MecanumConfig cfg = FtcDrives.MecanumConfig.defaults();
    cfg.wiring.frontLeftName = frontLeftMotorName;
    cfg.wiring.frontRightName = frontRightMotorName;
    cfg.wiring.backLeftName = backLeftMotorName;
    cfg.wiring.backRightName = backRightMotorName;
    cfg.wiring.frontLeftDirection = frontLeftMotorDirection;
    cfg.wiring.frontRightDirection = frontRightMotorDirection;
    cfg.wiring.backLeftDirection = backLeftMotorDirection;
    cfg.wiring.backRightDirection = backRightMotorDirection;
    cfg.enableZeroPowerBrake = enableZeroPowerBrake;
    return cfg;
}
```

The canonical example fields name `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, and
`backRightMotor`. The framework's
[`Direction`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/hal/Direction.html>)
selects the logical motor sign. Left motors start with `Direction.FORWARD`, right motors with `Direction.REVERSE`;
those are illustrative logical motor-sign choices, not physical evidence. `enableZeroPowerBrake`
is `true`. Establish your own assignments through the
[drivetrain procedure](<Robot Calibration Tutorials.md#drivetrain-direction-and-integration>),
then edit the existing robot configuration these mappings read. Do not make separate test-only
wiring that disagrees with production.

## Make every active command choice visible

A **normalized command** uses the range `-1` to `+1`, not inches per second or measured turning
speed. A scale multiplies the requested command. `omega` names rotation: positive is
counterclockwise. The example keeps the drive mixer's axial, lateral and turn scales at `1.0`, so
the tester's settings below are the explicit additional scales—not an unexplained second set of
reductions.

| Profile field or tester choice | Example value | Effect |
| --- | --- | --- |
| `manualOmegaScale` | `0.20` | Scales right-stick rotation during the manual rotation phase |
| `autoOmegaCmd` | `0.20` | Automatic-turn command magnitude; not measured angular speed |
| `targetTurnRad` | `Math.PI` | Automatic target of 180° counterclockwise |
| `automaticPhaseTimeoutSec` | `10.0 s` | Shared-clock elapsed-time bound for each automatic phase |
| `recenterTranslationScale` | `0.20` | Scales left-stick translation while recentering |
| `enableAutoTagSearchAtStart` / `enableAutoTagSearchAtEnd` | `false` / `false` | Disables automatic rotating searches for a missing tag |
| `autoComputeAfterAutoSample` | `true` | An assisted automatic sample computes at turn completion when its matched end is available |
| `enablePostRotateRecenter` | `true` | Retains the separate manual recenter phase when immediate assisted computation does not apply |

These are software illustrations, not universal safe powers or time limits. `Math.PI` is Java's
constant for π radians, or 180°. Review the profile's command and time fields for the actual robot
before setting `poweredMotionReviewed = true`.

The shared powered Config recipe makes the drive and search decisions explicit:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
cfg.pinpoint = captured.pinpoint();
cfg.mecanum = captured.mecanum();
cfg.manualOmegaScale = captured.manualOmegaScale;
cfg.autoOmegaCmd = captured.autoOmegaCmd;
cfg.targetTurnRad = captured.targetTurnRad;
cfg.automaticPhaseTimeoutSec = captured.automaticPhaseTimeoutSec;
cfg.recenterTranslationScale = captured.recenterTranslationScale;
cfg.enableAutoTagSearchAtStart = false;
cfg.enableAutoTagSearchAtEnd = false;
return cfg;
```

The two `true` choices in the table are retained
[`PinpointPodOffsetCalibrator.Config`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/calibration/PinpointPodOffsetCalibrator.Config.html>)
defaults. If a reviewed experiment changes them, set those exact Config fields inside this recipe
before construction; the old mutable draft does not retune a running tester.

## Keep assisted and unassisted attempts distinct

`CalibrationTesters.create(profile)` omits the powered submenu while `poweredMotionReviewed` is
false. Once reviewed, it registers **Powered pod offsets**, whose first choice is **Unassisted
powered pod offsets**. The separate **Assisted powered pod offsets** entry appears only when a
camera backend is selected, `cameraMountAccepted` is true, and the mount passes the framework's
non-identity check.

The direct assisted factory enforces the same conditions before making its active Config:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
CalibrationRobotProfile captured = capture(profile);
requireReviewedMotion(captured);
requireVision(captured);
if (!captured.cameraMountAccepted
        || !CalibrationChecks.canUseAprilTagAssist(captured.cameraMount)) {
    throw new IllegalStateException("Assisted pod offsets require cameraMountAccepted and "
            + "an accepted non-identity cameraMount; record, rebuild, and verify the mount first");
}
```

[`CalibrationChecks.canUseAprilTagAssist(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/calibration/CalibrationChecks.html#canUseAprilTagAssist(edu.ftcsushi.fw.sensing.vision.CameraMountConfig)>)
recognizes a likely identity placeholder; it cannot prove that a mount was measured correctly.
That is why this example also requires the separate human acknowledgement. A missing permission,
camera, or suitable accepted mount rejects the request; it never silently falls back to an
unassisted tester. If this example cannot express a reviewed setup, stop rather than changing an
acknowledgement just to make the guard pass.

Reset `cameraMountAccepted` to `false` and repeat the mount review whenever the backend, physical
camera, or mount changes. The software does not know that an old acknowledgement belongs to a
different physical setup.

After the guard, the assisted factory adds the same selected-name, device-type and fixed-layout
mapping taught in the vision lesson, `cfg.aprilTags = captured.aprilTags()`, and the captured
`visionFactoryBuilder(captured)` instead of null. No live camera or production drive is injected.

Assistance compares a tag observation with raw odometry from that same **capture time**—when the
image was taken, not when the result arrived. The calibrator owns bounded odometry history for this
matching. Its software defaults are `0.50 s` retention, `128` samples, at most `0.10 s` between
interpolated endpoints, `12.0 in` translation and π/2 radians of yaw change. The example retains
those `Config.assistOdometryHistory` defaults; the separate tag-age limit comes from the profile's
`maxDetectionAgeSec = 0.35`. See the
[capture-time explanation and bounds](<Robot Calibration Tutorials.md#optional-advanced-powered-and-vision-assisted-pod-offsets>)
before interpreting a result. Missing history does not become a usable endpoint merely because a
tag is young enough.

## Understand what remains powered

Disabling tag searches does **not** disable automatic sampling. After START and the required
current-cycle Pinpoint `READY` evidence, Y still requests the automatic 180° turn. Manual rotation
uses the right stick, and the optional recenter phase uses the left stick; these are powered motor
commands, not the hand-only procedure from the basic lesson.

For the example's assisted Y sample, a usable capture-matched start is required before motion.
With a usable matched end at turn completion, `autoComputeAfterAutoSample = true` computes the
recommendation immediately; it does not require manual recentering and a final A. Missing start or
required end evidence discards the assisted attempt because both searches are off. An assisted
manual sample, or an explicitly changed auto-compute choice, can instead enter the optional
recenter phase; its final A must obtain a matched end or discard the attempt.

The unassisted entry keeps its manual recenter-and-A computation path even after a Y-triggered
turn. These differences are shown by the active screen; follow the physical runbook rather than
assuming every mode uses the same completion gesture.

The `10.0 s` automatic-phase bound is checked cooperatively by the shared loop. It is not a hardware
watchdog, a bound on manual stick-controlled phases, or proof of physical stopping distance.
Successful ordinary INIT does not command drive motion, but failure cleanup may request zero.
B aborts the attempt; BACK cleans up the selected child; FTC STOP remains the emergency action.
The adopting team's stop plan must cover loop delays and hardware failures.

## Verify before accepting a result

Keep the basic host and menu connection; no additional FTC loop is needed. Rebuild after reviewing
the canonical configuration, then use the separately named powered entry and its
[physical procedure](<Robot Calibration Tutorials.md#optional-advanced-powered-and-vision-assisted-pod-offsets>).
Record the attempt's mode, accepted evidence and output, not just a plausible offset number.
Edit accepted offsets in the canonical robot configuration, rebuild, and verify through a fresh
configured tester and the production owner.

- [Complete source: `CalibrationRobotProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationRobotProfile.java>) — reviewed wiring, command choices and acknowledgements.
- [Complete source: `CalibrationTesters.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java>) — pre-construction guards and separate fresh-owner recipes.

The [basic lesson's software check](<Add Calibration Testers to Your Robot.md#inspect-the-maintained-software-example>)
can establish configuration propagation and rejection before construction. It cannot establish
correct motor direction, safe motion, camera placement, calibration accuracy or physical STOP.
Use [Guided calibration walkthroughs](<Guided Calibration Walkthroughs.md>) only if you want an
ordered view of the same configured checks; a menu status must never authorize motion.
