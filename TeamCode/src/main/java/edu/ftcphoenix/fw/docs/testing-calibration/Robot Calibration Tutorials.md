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
