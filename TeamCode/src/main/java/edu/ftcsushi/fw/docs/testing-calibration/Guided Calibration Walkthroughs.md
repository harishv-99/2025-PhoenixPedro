---
tags:
  - Test & Tune
---

# Guided calibration walkthroughs

**Learning mode:** Architecture reference

This is optional architecture for a team that already owns a checked-in robot profile and fresh
robot-configured tester factories. The framework home works without it: use
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

A tiny immutable status object used by walkthrough menus.

Use it when you want a step to answer two questions:

- is this complete enough to move on?
- what one-line message should the menu show?

### `CalibrationChecks`

Shared heuristics for common calibration questions.

Examples:

- does a `CameraMountConfig` still look like the identity placeholder?
- do Pinpoint offsets still look like `0 / 0`?
- has a robot-side explicit verification flag been set?

The point is not to make the framework magically know everything. The point is to keep the obvious, repeated heuristics out of every robot project.

### `CalibrationWalkthroughBuilder`

A builder that produces a normal `TesterSuite`, but with a few calibration-specific opinions baked in:

- steps are shown in the order you add them
- tracked steps can show `OK` / `TODO`
- the first incomplete tracked step is selected by default
- status tags are passed to the shared `SelectionMenu` item model instead of being embedded in labels
- robot projects do not have to hand-roll the menu boilerplate

## Map one checked-in fact

The fragment below uses the real helper signatures for one fact. It is deliberately **not a complete
robot registry**: the robot project must supply its own checked-in mount and a factory that creates
an AprilTag-localization tester whose lane was built from that robot profile.

<!-- teaching-shape -->
```java
static TesterSuite cameraMountWalkthrough(
        CameraMountConfig checkedInCameraMount,
        Supplier<TeleOpTester> freshConfiguredAprilTagLocalizationTester) {
    CalibrationWalkthroughBuilder guide =
            new CalibrationWalkthroughBuilder("Robot calibration");
    guide.addStep(
            "Verify configured camera mount",
            "After rebuild, compare field pose through the robot-configured lane.",
            () -> CalibrationChecks.cameraMount(checkedInCameraMount),
            freshConfiguredAprilTagLocalizationTester);
    return guide.build();
}
```

`CalibrationChecks.cameraMount(...)` supplies the real `CalibrationStatus` heuristic used for the
menu tag. It notices an identity placeholder in the checked-in profile; it does not prove the
camera's physical mount. `addStep(label, help, status, testerFactory)` stores the real
`Supplier<TeleOpTester>`; the suite calls it only when the operator selects the step. That supplier
must return a new inactive AprilTag-localization tester every time, not another mount calibrator, a
retained tester, or the production camera owner.

The profile owner passes the stable, checked-in `CameraMountConfig`. The tester factory separately
maps that same mount and the other current robot vision facts into a fresh tester Config and a
backend-neutral lane factory. That is what makes the selected check capable of verifying the
rebuilt profile. The walkthrough reads both; it owns neither profile mutation nor result
persistence.

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
