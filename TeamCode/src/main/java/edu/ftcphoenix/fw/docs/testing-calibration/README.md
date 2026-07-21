# Testing & calibration

Use this section when bringing up a robot, checking hardware directions, establishing mechanism
position references, tuning a software PIDF controller, calibrating camera mounts for either webcam
or Limelight AprilTag rigs, and building guided walkthrough testers.

If you copied only `fw` into a project, you can launch the framework-owned tester tree directly from the ready-made Driver Station OpMode `FW: Testers` (`edu.ftcphoenix.fw.tools.tester.opmode.FrameworkTestersOpMode`). Robot projects that already have their own configured tester home can keep embedding the shared framework categories through `StandardTesters.register(...)`.

## Tester lifecycle and recovery

Register tester factories normally:

```java
new TesterSuite()
        .add("HW: My Mechanism", MyMechanismTester::new);
```

Each factory must return a fresh, non-null, inactive tester. Construct configuration in the factory,
then acquire hardware in `init(...)` or a later owned lifecycle phase. If a factory acquires a
resource before it returns a tester, that factory must also clean the resource before throwing;
the suite cannot stop an object it never received.

The runner retains a returned tester before invoking its lifecycle. If `init(...)`, `start()`, an
update, or BACK handling fails with a runtime exception, the runner makes that child terminal and
attempts `stop()` once. Confirmed cleanup returns a suite or hardware selector to its menu with an
actionable error. Failed cleanup blocks another tester or device from being selected because the
previous hardware ownership is uncertain; stop and restart the OpMode before continuing.

Implement `stop()` so it is safe after partial initialization: clean only fields whose resources
were successfully acquired. The runner will not repeatedly call `stop()` after it throws.

For vision, retain and close the concrete webcam or Limelight lane owner rather than closing a
borrowed processor/sensor view. After close succeeds, a retry creates a fresh owner; a webcam retry
also needs fresh `VisionProcessor` instances because processors belong to the portal graph built
around them. If close fails, follow the fail-stop rule above and restart the OpMode instead of
opening a replacement. Display `VisionReadiness` separately from whether a target is currently
visible.

## Read in this order

1. [`Robot Calibration Tutorials.md`](<Robot Calibration Tutorials.md>)
2. [`Guided Calibration Walkthroughs.md`](<Guided Calibration Walkthroughs.md>)
3. [`Software PIDF Tuning Workflow.md`](<Software PIDF Tuning Workflow.md>)
4. [`../drive-vision/AprilTag Practice Setup.md`](<../drive-vision/AprilTag Practice Setup.md>)
5. [`../ftc-boundary/FTC Sensors.md`](<../ftc-boundary/FTC Sensors.md>)

## Good companions

- [`../drive-vision/AprilTag Localization & Fixed Layouts.md`](<../drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`../design/Recommended Robot Design.md`](<../design/Recommended Robot Design.md>)
