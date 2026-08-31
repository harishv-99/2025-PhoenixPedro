# Guided calibration walkthroughs

The tester framework separates:

- **one canonical generic tool per hardware fact**, such as `HW: Actuator Bring-up`; and
- **robot-specific guided suites**, which order those facts with configured integration and
  localization checks.

The framework home is already a short start-here path: actuator bring-up, calibration/localization,
and advanced diagnostics. A robot walkthrough is useful only when it adds that robot's checked-in
configuration, status, and recommended order.

## Design rules

These are the framework rules the walkthrough helpers are built around.

### One implementation for each fact

Do not build another motor-power or servo-position screen for a robot project. Reuse the canonical
actuator wizard for raw configured-device facts, then add only genuinely robot-specific checks such
as configured drivetrain verification or a complete mechanism test.

A walkthrough may point to the same factory because its job is ordering, but it must not fork the
controls, safety behavior, or evidence contract.

### Status belongs near the walkthrough step

Students should not have to remember which calibration has already been completed. Walkthrough steps can show a small `OK` / `TODO` tag plus a one-line reason.

### Robot code should stay thin

The framework should own the generic menu/status mechanics. Robot code should mainly supply:

- which testers exist for that robot
- the current robot config objects
- any explicit human-acknowledgement booleans

For a vision-backed step, map only the relevant robot facts into a fresh tester Config and pass the
backend-neutral vision-factory builder separately. Capture the selected webcam/Limelight template
when building that function; do not let a later picker callback reread a broad mutable robot profile.
The suite stores a `Supplier<TeleOpTester>`, so each entry creates a fresh owner with one immutable
Config/layout snapshot. A borrowed custom SDK tag library must remain stable for that owner's full
lifetime and any clean retry.

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

## Typical pattern for a robot project

A robot project should usually add two required kinds of entry, plus one optional kind, beside the
framework's canonical actuator tool:

1. a **guided calibration walkthrough**
2. a **robot-specific calibration/localization category**
3. optionally, a **robot-specific configured-system verification** when it proves something the raw
   device wizard cannot

## Example

```java
public final class ExampleRobotTesters {

    public static void register(TesterSuite suite) {
        suite.add(
                "Guide: Example Calibration Walkthrough",
                "Recommended bring-up order for a fresh robot.",
                ExampleRobotTesters::createWalkthrough
        );

        suite.add(
                "Example: Calibration & Localization",
                "Robot-configured calibration tools.",
                ExampleRobotTesters::createCalibrationSuite
        );

    }

    public static TesterSuite createWalkthrough() {
        CalibrationWalkthroughBuilder guide = new CalibrationWalkthroughBuilder("Example Calibration Walkthrough");

        guide.addStep(
                "HW: Actuator Bring-up",
                "Establish one configured device's direction and optional safe endpoints.",
                StandardTesters::createActuatorBringUp
        );

        guide.addStep(
                "Calib: Camera Mount",
                "Solve and paste RobotConfig.Vision.cameraMount.",
                ExampleRobotTesters::cameraMountStatus,
                ExampleRobotTesters::cameraMountCalibrator
        );

        guide.addStep(
                "Calib: Pinpoint Axis Check",
                "Verify +X forward, +Y left, heading CCW+.",
                ExampleRobotTesters::pinpointAxesStatus,
                ExampleRobotTesters::pinpointAxisCheck
        );

        guide.addStep(
                "Calib: Pinpoint Pod Offsets",
                "Estimate and paste Pinpoint pod offsets.",
                ExampleRobotTesters::pinpointOffsetsStatus,
                ExampleRobotTesters::pinpointPodOffsets
        );

        return guide.build();
    }
}
```

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
