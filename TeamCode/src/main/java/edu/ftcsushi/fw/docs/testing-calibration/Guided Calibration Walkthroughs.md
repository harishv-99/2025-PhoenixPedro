---
tags:
  - Test & Tune
---

# Guided calibration walkthroughs

**Learning mode:** Architecture reference

**Before this page:** complete the reading path in
[Add calibration testers to your robot](<Add Calibration Testers to Your Robot.md>). It supplies the
checked-in robot profile and fresh configured tester factories used here. This optional page adds
only ordering and completion status; the camera-free example is enough.

The framework home works without a custom walkthrough: use
[`Robot Calibration Tutorials`](<Robot Calibration Tutorials.md>) to probe and record facts even
when source is unavailable.

A guided suite adds robot-specific order and status around existing testers. It is useful only when
it reads that robot's checked-in configuration or adds a real configured-system check; it does not
save calibration results or replace the canonical `HW: Actuator Bring-up` workflow.

## Design rules

These are the framework rules the walkthrough helpers are built around.

### One implementation for each fact

Do not build another motor-power or servo-position screen for a robot project. Reuse the canonical
actuator wizard for raw configured-device facts, then add only genuinely robot-specific checks such
as configured drivetrain verification or a complete mechanism test.

A walkthrough may point to the same factory because its job is ordering, but it must not fork the
controls, safety behavior, or evidence contract.

### Status belongs near the walkthrough step

Walkthrough steps can show a small `OK` / `TODO` tag plus a one-line reason. The builder evaluates
that status while it builds the suite. Opening or completing a tester does not mutate the profile or
refresh the tag: record the result, edit the profile, rebuild, and start a newly built suite.

Use [Keep one calibration record](<Robot Calibration Tutorials.md#keep-one-calibration-record>) to
connect those edits to the reviewed validation and acceptance evidence. Deploying a proposed value
or seeing `OK` in this menu is not physical acceptance, and the external record does not update
suite status automatically.

### Robot code should stay thin

The framework should own the generic menu/status mechanics. Robot code should mainly supply:

- which testers exist for that robot
- the current robot config objects
- any explicit human-acknowledgement booleans

For a vision-backed step, map only the relevant robot facts into a fresh tester Config and pass the
backend-neutral vision-factory builder separately. Capture the selected webcam/Limelight template
when building that function; do not let a later picker callback reread a broad mutable robot profile.
The suite stores a `Supplier<TeleOpTester>`, so every selection must create a fresh inactive owner
with its own Config/layout snapshot. A borrowed custom SDK tag library must remain stable for that
owner's full lifetime and any clean retry.

For a motor-capable calibration step such as Pinpoint pod offsets, teach the lifecycle boundary too:
successful ordinary INIT may configure hardware and collect evidence but does not command the drive;
START/RUN and cleanup STOP are the command boundaries. A failed-init rollback can still write
physical zero before START.

## Framework helpers

### `CalibrationStatus`

[`CalibrationStatus`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/calibration/CalibrationStatus.html>)
is a small immutable status value: once created, its completion flag and reason do not change.

Use it when you want a step to answer two questions:

- is this complete enough to move on?
- what one-line message should the menu show?

### `CalibrationChecks`

[`CalibrationChecks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/calibration/CalibrationChecks.html>)
supplies shared **heuristics**: simple clues in configuration, not physical measurements.

Examples:

- does a `CameraMountConfig` still look like the identity placeholder?
- do Pinpoint offsets still look like `0 / 0`?
- has a robot-side explicit verification flag been set?

The point is not to make the framework magically know everything. The point is to keep the obvious, repeated heuristics out of every robot project.

### `CalibrationWalkthroughBuilder`

[`CalibrationWalkthroughBuilder`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/calibration/CalibrationWalkthroughBuilder.html>)
collects ordered steps and produces a normal
[`TesterSuite`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/TesterSuite.html>).
It adds these calibration-specific choices:

- steps are shown in the order you add them
- tracked steps can show `OK` / `TODO`
- the first incomplete tracked step is selected by default
- status tags are passed to the shared `SelectionMenu` item model instead of being embedded in labels
- robot projects do not have to hand-roll the menu boilerplate

## Map one checked-in fact

The maintained
[`CalibrationTesters.guidedWalkthrough(profile)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.html#guidedWalkthrough(edu.ftcsushi.robots.examples.calibration.CalibrationRobotProfile)>)
uses the same profile and factories from the basic integration lesson. This camera-free beginning
copies the facts and adds two steps; it creates no tester or hardware during registration:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
CalibrationRobotProfile captured = capture(profile);
CalibrationWalkthroughBuilder guide =
        new CalibrationWalkthroughBuilder("Robot calibration walkthrough");
guide.addStep("Verify Pinpoint axes", "Hand motion; record, rebuild, and verify",
        () -> CalibrationChecks.pinpointAxes(captured.pinpointAxesVerified),
        () -> axisDirections(captured));
guide.addStep("Verify Pinpoint offsets", "Status is not physical evidence",
        () -> CalibrationChecks.pinpointOffsets(
                captured.pinpoint(), captured.pinpointOffsetsVerified),
        () -> manualPodOffsets(captured));
```

`addStep(label, help, status, testerFactory)` saves two functions after the text: a status reader
and a fresh tester recipe. The `() ->` syntax has the same saved-function meaning as menu
registration in the basic lesson. The builder calls the status reader while building the suite;
the suite calls the tester recipe only after selection. Neither is a new thread.

`pinpointAxesVerified` and `pinpointOffsetsVerified` start `false`. A person edits them only after
accepting the rebuilt robot-configured result. The offset helper also considers nonzero offsets a
clue; an `OK` tag therefore does not always mean that explicit human verification happened. The
default `0 / 0` offsets and false flags yield incomplete status. No tag authorizes powered motion.

When a camera backend is selected, the same method additionally registers mount measurement,
configured AprilTag verification and corrected-localization comparison using the
[vision lesson's factories](<Add Vision to Your Calibration Suite.md>). The mount-status helper
only notices a non-identity value; it does not prove the mount is physically correct. The
verification factory creates an AprilTag-localization tester, not another mount calibrator or the
production camera owner.

The method finishes by returning the ordinary suite:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java -->
```java
return guide.build();
```

To select this ordered view, have your thin host's `createTester()` return
`CalibrationTesters.guidedWalkthrough(CalibrationRobotProfile.current())` instead of the basic
`CalibrationTesters.create(...)` result. Do not add another clock or FTC loop. The built suite keeps
its status snapshot; record the result, edit the profile, rebuild, and create a fresh suite to see
changed acknowledgements.

[Complete source: `CalibrationTesters.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/CalibrationTesters.java>)
contains the complete method and the real factories it uses. The
[basic integration lesson](<Add Calibration Testers to Your Robot.md>) owns the profile and host
assembly; this page adds ordering and status only.

### Ownership checklist

- **Profile:** one robot-owned data profile is authoritative for the camera mount, Pinpoint config,
  and explicit human acknowledgements.
- **Status:** use `CalibrationChecks.cameraMount(...)`, `pinpointAxes(...)`, or
  `pinpointOffsets(...)` only for the one fact they describe. A heuristic or boolean is not fresh
  hardware evidence.
- **Factory:** every `Supplier<TeleOpTester>` returns a fresh inactive tester whose Config is mapped
  from the profile once. Do not cache a tester, lane, Plant, or drivetrain owner.
- **Lifecycle:** the selected tester acquires hardware during its own init and remains the exclusive
  heartbeat/cleanup owner until BACK or STOP. Production does not run beside it.
- **Persistence:** the tester prints or displays evidence; a human records and reviews it, the
  profile owner edits source, and the team rebuilds before a fresh configured verification run.

## Where robot-specific status should live

Use a mixed strategy.

### Framework-owned heuristics

Put repeated, generic checks into `CalibrationChecks`.

Good examples:

- identity camera mount detection
- default Pinpoint offset detection
- “can AprilTag assist reasonably be enabled?”

### Robot-owned acknowledgements

Keep explicit human judgement in the robot project.

Good examples:

- `pinpointAxesVerified`
- `pinpointPodOffsetsCalibrated`

Those are not purely mechanical truths. They are declarations that somebody actually ran the tester and accepted the result.

## Documentation pattern

A calibration system is easiest to learn when it is documented in two directions:

- **from the beginning**: one ordered tutorial covering the whole bring-up path
- **from the system**: links near a specific subsystem that jump directly to the relevant calibration step

The framework docs follow that structure:

- [`Robot Calibration Tutorials`](<Robot Calibration Tutorials.md>) for the full ordered path
- [`Actuator Bring-up`](<Actuator Bring-up.md>) for the canonical generic hardware workflow
- subsystem docs should link back into the relevant section of that tutorial when calibration matters

## Menu wording recommendations

Use labels that make the role obvious:

- `HW:` for quick hardware sanity checks
- `Calib:` for steps that produce numbers or a verified configuration state
- `Loc:` for validation of a localizer or pose-estimation pipeline
- `Guide:` for the deliberate walkthrough entrypoint

Short labels matter. Students should be able to find the right tester from across the room on the
selected telemetry console.
