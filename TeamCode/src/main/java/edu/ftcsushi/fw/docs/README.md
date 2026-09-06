---
tags:
  - Get Started
---

# Guide map

This page is the complete map; it is not a reading assignment. First learn how Sushi relates to the
FTC loop by reading. Then choose the robot outcome you need. A drive student does not need
to build a lift, and a claw student does not need to finish the drive lesson.

## New to Sushi? Start small

1. [How Sushi runs your code](<getting-started/Framework Overview.md>) — connect the FTC loop you
   know to code that runs now and code saved to run later.
2. [Take the first software tour](<getting-started/First Software Tour.md>) — compare a sensor
   observation, one button press, and one action that continues across loops.
3. [Choose a build](<build/README.md>) — follow the small steps to TeleOp and basic Auto, or choose
   the independent outcome you need. Feedback, Pedro, and vision are optional later topics.

Reading needs no setup and includes expected results. If you want to run software checks or author
your own robot, [set up and verify](<getting-started/Build and Run.md>) first. The Build home explains
how to work in your own package and check your implementation, not just the maintained answer.
Hardware is a separate supervised path; no lesson requires an unrelated mechanism.

## Choose by outcome

| I want to… | Go to… |
|---|---|
| turn one switch reading into cached telemetry | [Read a switch](<build/Read a Switch.md>) |
| make the first bounded drivetrain command | [Drive with a gamepad](<build/First Drive.md>) |
| run a continuous motor with named intent | [Run a named intake](<build/Continuous Intake.md>) |
| map OPEN, HALF, and CLOSED to configured servo endpoint candidates | [Open and close a claw](<build/Named Claw.md>) |
| establish encoder zero from a bottom switch | [Establish a lift reference](<build/Referenced Lift.md>) |
| move a referenced lift and wait for fresh feedback | [Move a referenced lift](<build/Move a Referenced Lift.md>) |
| command one motor's velocity and observe arrival | [Reach one flywheel velocity](<build/Single Flywheel Velocity.md>) |
| combine continuous drive and button-controlled intake | [Combine drive and intake](<build/Combine Drive and Intake.md>) |
| run one fresh timed behavior when Auto starts | [Run one timed Auto](<build/Run One Timed Auto.md>) |
| sequence feedback-aware lift Tasks without blocking the loop | [Sequence an autonomous](<build/First Autonomous.md>) |
| compile one Pedro route and inspect its truthful software outcome | [Inspect Pedro route status](<build/First Pedro Auto.md>) |
| understand where a piece of code belongs | [Choose a learning question](<getting-started/Beginner's Guide.md>) |
| run the first Test & Tune software experiment | [Hardware-free Reference Scenarios](<examples/Hardware-free Reference Scenarios.md>) |
| operate the ready Driver Station or Panels tester console | [Using the tester console](<testing-calibration/Using the Tester Console.md>) |
| bring up an actuator without assuming its safe range | [Actuator bring-up](<testing-calibration/Actuator Bring-up.md>) |
| establish one camera, odometry, or localization fact | [Robot calibration](<testing-calibration/Robot Calibration Tutorials.md>) |
| tune one controller with a bounded experiment | [Control tuning](<testing-calibration/Control Tuning Workflow.md>) |
| design another useful software or hardware test | [How to test a Sushi component](<testing-calibration/How to test a Sushi component.md>) |
| study a less-common composition | [Advanced patterns](<advanced/README.md>) |
| look up an exact framework family | [Reference](<reference/README.md>) |
| recover from an observed problem | [Common problems](<troubleshooting/Common Problems.md>) |

## What each area is for

- **Get Started** connects familiar FTC loops to Sushi with a short readable tour and optional setup.
- **Learn** explains one framework idea without making you assemble a robot at the same time.
- **Build** gives one focused, compiling authority per ordinary outcome.
- **Test & Tune** starts with one software experiment, then opens the console for one bounded
  bring-up, calibration, or tuning question.
- **Advanced** contains optional patterns with additional ownership or evidence requirements.
- **Reference** organizes exact vocabulary and links every API name to generated Javadocs.

## Search all guides

The site search is global, not limited to the selected tab. That is intentional: you should not
need to know which area owns a term before searching for it. Filter results by the one area tag on
each page. Search titles use both the framework term and the likely goal—for example, “lift
reference,” “move,” and “flywheel velocity.” For exact classes, members, signatures, or overloads, use
[Search API types and members](<https://harishv-99.github.io/2025-PhoenixPedro/api/>).

The checked-in guides, compiling examples, Javadocs, and tests are one current documentation
authority. [Framework Principles](<../Framework Principles.md>) governs framework and maintained
example changes.
