---
tags:
  - Get Started
---

# Sushi documentation

Sushi helps an FTC team turn intent into bounded hardware commands with one managed heartbeat. You
do not need to read the framework from beginning to end. Choose the result you need, finish its
software checkpoint, and cross the stated hardware gate before integrating it into the robot.

## Your shortest path to a working robot

1. [Set up and verify Sushi](<getting-started/Build and Run.md>).
2. [Drive with a gamepad](<build/First Drive.md>) if your current outcome needs a drivetrain.
3. Read [Sushi in one picture](<getting-started/Framework Overview.md>) to recognize the owners.
4. Start the cumulative actuator path with the self-contained
   [continuous intake](<build/Continuous Intake.md>).
5. Add only the next decision you need: [named servo positions](<build/Named Claw.md>),
   [a lift reference](<build/Referenced Lift.md>),
   [a feedback-aware lift move](<build/Move a Referenced Lift.md>), then
   [one motor's velocity](<build/Single Flywheel Velocity.md>).
6. Cross each lesson's isolated **Test & Tune** gate before integrating its capability.
7. [Combine drive and intake](<build/Combine Drive and Intake.md>) to learn the first multi-owner
   TeleOp composition.
8. [Run one timed root Task](<build/Run One Timed Auto.md>), then
   [sequence real capability Tasks](<build/First Autonomous.md>) after their individual evidence
   gates pass.

## Choose by outcome

| I want to… | Go to… |
|---|---|
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
| understand where a piece of code belongs | [Choose a concept](<getting-started/Beginner's Guide.md>) |
| design a useful software or hardware test | [How to test a Sushi component](<testing-calibration/How to test a Sushi component.md>) |
| bring up an actuator without assuming its safe range | [Actuator bring-up](<testing-calibration/Actuator Bring-up.md>) |
| study a less-common composition | [Advanced patterns](<advanced/README.md>) |
| look up an exact framework family | [Reference](<reference/README.md>) |
| recover from an observed problem | [Common problems](<troubleshooting/Common Problems.md>) |

## What each area is for

- **Get Started** establishes the project and the six-part mental map.
- **Learn** explains one framework idea without making you assemble a robot at the same time.
- **Build** gives one focused, compiling authority per ordinary outcome.
- **Test & Tune** separates software evidence from facts only hardware can establish.
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
