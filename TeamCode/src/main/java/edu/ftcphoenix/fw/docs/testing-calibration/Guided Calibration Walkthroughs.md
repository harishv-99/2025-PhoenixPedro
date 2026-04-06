# Guided calibration walkthroughs

The tester framework now supports a cleaner split between:

- **category suites**, where every tester has one obvious home
- **guided walkthrough suites**, where a robot can present a recommended bring-up order for students

This avoids turning the top-level tester menu into a long flat list while still preserving a “start here” path for a fresh robot.

## Design rules

These are the framework rules the walkthrough helpers are built around.

### One natural home for each tester

Individual testers should live in category menus such as:

- hardware bring-up
- calibration and localization
- robot-specific system groups

That is where students and mentors browse once they already know what they want.

### Duplication is only intentional in walkthroughs

A walkthrough may repeat links to those same testers because its job is different: it is a teaching path, not a taxonomy.

### Status belongs near the walkthrough step

Students should not have to remember which calibration has already been completed. Walkthrough steps can show a small `OK` / `TODO` tag plus a one-line reason.

### Robot code should stay thin

The framework should own the generic menu/status mechanics. Robot code should mainly supply:

- which testers exist for that robot
- the current robot config objects
- any explicit human-acknowledgement booleans

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
- robot projects do not have to hand-roll the menu boilerplate

## Typical pattern for a robot project

A robot project should usually expose three entry points:

1. a **guided calibration walkthrough**
2. a **robot-specific calibration/localization category**
3. a **robot-specific hardware bring-up category**

That is the pattern Phoenix now uses.

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

        suite.add(
                "Example: Hardware Bring-up",
                "Robot-specific hardware sanity checks.",
                ExampleRobotTesters::createHardwareSuite
        );
    }

    public static TesterSuite createWalkthrough() {
        CalibrationWalkthroughBuilder guide = new CalibrationWalkthroughBuilder("Example Calibration Walkthrough");

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

The framework docs now follow that structure:

- [`Robot Calibration Tutorials`](<Robot Calibration Tutorials.md>) for the full ordered path
- subsystem docs should link back into the relevant section of that tutorial when calibration matters

## Menu wording recommendations

Use labels that make the role obvious:

- `HW:` for quick hardware sanity checks
- `Calib:` for steps that produce numbers or a verified configuration state
- `Loc:` for validation of a localizer or pose-estimation pipeline
- `Guide:` for the deliberate walkthrough entrypoint

Short labels matter. Students should be able to find the right tester from across the room on the Driver Station screen.
