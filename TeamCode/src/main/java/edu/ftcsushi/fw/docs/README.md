---
tags:
  - Get Started
---

# Guide map

This page is the complete map; it is not a reading assignment. First learn how Sushi relates to the
FTC loop with software only. Then choose the robot outcome you need. A drive student does not need
to build a lift, and a claw student does not need to finish the drive lesson.

## New to Sushi? Follow these three pages

1. [How Sushi runs your code](<getting-started/Framework Overview.md>) — connect the FTC loop you
   know to code that runs now and code saved to run later.
2. [Set up and verify the Sushi project](<getting-started/Build and Run.md>) — prove the project
   builds before adding robot hardware.
3. [Take the first software tour](<getting-started/First Software Tour.md>) — compare driving,
   one button press, and one action that continues across loops.

After that required software path, choose one Build recipe below. Each recipe separates a software
checkpoint from an isolated hardware check. Within the actuator series, knowledge builds in order
from intake to claw, lift, and velocity, but the hardware fixtures stay focused and independent.

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
| understand where a piece of code belongs | [Choose a learning question](<getting-started/Beginner's Guide.md>) |
| design a useful software or hardware test | [How to test a Sushi component](<testing-calibration/How to test a Sushi component.md>) |
| bring up an actuator without assuming its safe range | [Actuator bring-up](<testing-calibration/Actuator Bring-up.md>) |
| study a less-common composition | [Advanced patterns](<advanced/README.md>) |
| look up an exact framework family | [Reference](<reference/README.md>) |
| recover from an observed problem | [Common problems](<troubleshooting/Common Problems.md>) |

## What each area is for

- **Get Started** connects familiar FTC loops to Sushi and verifies the project without hardware.
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
