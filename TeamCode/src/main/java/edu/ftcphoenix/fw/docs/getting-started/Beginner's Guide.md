# Phoenix beginner course

## Goal

Build confidence with one continuous starter example: compile Phoenix, run one Plant-backed
mechanism in a simple Auto, add managed TeleOp drive and controls, and understand the Task you
observed. A separate optional track then explains a Pedro reference without making route following
part of the first-robot prerequisite.

**Time:** About 2–3 hours of software work for the four core lessons, plus the time your team needs
for careful hardware checks. Pedro setup, localization, tuning, and physical validation are a
separate project.

**Prerequisites:**

- basic Java classes, methods, and enums;
- an FTC Android Studio project that opens successfully;
- access to the Robot Controller configuration when you begin the hardware lessons; and
- an adult or experienced student supervising first motion tests.

**Files for this lesson:**

- [`starter/`](<../../../robots/examples/starter/>) — the continuing TeleOp and simple Auto robot;
- [`pedro/`](<../../../robots/examples/pedro/>) — the fixed-route Pedro reference;
- [`FtcRobotOpMode.java`](<../../ftc/FtcRobotOpMode.java>) and
  [`RobotProgram.java`](<../../ftc/RobotProgram.java>) — the managed runtime surface.

## Course checkpoints

| Step | Lesson | You are done when… |
|---:|---|---|
| 1 | [`Build and Run`](<Build and Run.md>) | The TeamCode module compiles and its unit tests pass. |
| 2 | [`Your first mechanism`](<First Mechanism.md>) | The one-motor starter Auto collects for 0.75 seconds and returns to stopped. |
| 3 | [`Your first TeleOp`](<First TeleOp.md>) | The starter drives, slows with right bumper, and maps A/B/X to intake meanings. |
| 4 | [`Your first Task and Auto`](<First Task and Auto.md>) | You can explain and safely adapt the non-blocking timed behavior already observed. |

Do one checkpoint at a time. Commit or otherwise save a known-good state before changing hardware
configuration, directions, or path geometry.

After checkpoint 4, the core beginner course is complete. Continue with the optional
[`Pedro software walkthrough`](<First Pedro Auto.md>) if route integration is relevant; it remains
disabled and requires no physical Pedro setup.

## The files form one robot

You do not need to read all seven starter files at once. Each lesson opens only the next layer:

```text
StarterProfile                     names, directions, and powers
       |
       +--> StarterAuto            first one-motor FTC entry
       |          |
       |          v
       |     StarterRobot.declareAuto(...)
       |          |
       |          v
       |     StarterIntake --> StarterIntakeMechanism
       |                         private Plant and safe stop
       |
       +--> StarterTeleOp         adds drive and controls
                  |
                  v
             StarterRobot.declareTeleOp(...)
                  |
                  +--> same StarterIntake capability
                  +--> StarterTeleOpControls and final drive
```

The split keeps the OpModes short without hiding robot behavior in a robot-specific base class.

## Safety contract for the course

Before setting `hardwareConfigurationReviewed = true` or removing `@Disabled`:

- match every configured name to the Robot Controller configuration;
- review every configured direction and write down the expected motion;
- use conservative mechanism powers and explicit conservative drive scales;
- keep people, wires, game pieces, and tools outside moving mechanisms;
- leave enough clear floor space for any drive or Pedro test; and
- keep one operator ready to press STOP.

Enabling makes the first supervised motion test possible; it does not complete the physical review.
During that test, keep wheels or mechanisms safely unloaded, verify every direction, and prove that
FTC STOP removes motion before lowering the robot or increasing a limit.

Compilation and unit tests verify software contracts. They cannot verify wiring, polarity,
traction, calibration, physical placement, or safe travel.

## How to read each lesson

Every lesson gives you:

1. one visible goal;
2. the exact checked-in source files to open;
3. a short sequence of changes or observations;
4. a checkpoint you can test;
5. common failure messages and fixes; and
6. one link to the next lesson.

Code blocks are excerpts from the checked-in examples or use the same public API those examples
compile against. Follow the linked complete source whenever an excerpt omits imports, configuration,
or surrounding ownership code.

## Expected checkpoint

Before moving on, you should know that the course uses only the managed path:

```java
public final class MyMode extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        // Construct robot owners and declare their roles here.
    }
}
```

You will not create a `LoopClock`, `TaskRunner`, manual FTC loop, or second hardware writer.

## Common problems

**“The source looks larger than the code shown in a lesson.”**

The complete source also contains configuration validation, safe cleanup, telemetry, and focused
test seams. Learn the highlighted path first; return to those details when you own that subsystem.

**“When should I continue to Pedro?”**

Finish the four core checkpoints first. Pedro builds on the same Task, Auto, ownership, and
safe-stop vocabulary. The optional lesson is software-first and does not ask a beginner to enable
route motion. A later physical test is a separate project that requires an installed, localized,
calibrated, and tuned robot.

**“Should I copy code from the FTC SDK sample packages?”**

Use code under `edu.ftcphoenix.robots` as the Phoenix reference. FTC SDK sample packages do not
demonstrate this managed ownership model.

**Next:** [`Build and run Phoenix`](<Build and Run.md>)
