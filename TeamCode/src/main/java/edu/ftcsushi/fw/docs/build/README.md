---
tags:
  - Build
---

# Choose one robot outcome to build

Each recipe below is an independent checkpoint. You do not need a lift to learn a claw, and you do
not need a drivetrain connected while bringing up either mechanism.

## Recommended route

1. [Drive with a gamepad](<First Drive.md>).
2. Read the short [mechanism anatomy](<../getting-started/learn-sushi/Robot Roles.md>).
3. Choose **one** mechanism:
   [continuous intake](<Continuous Intake.md>), [named claw](<Named Claw.md>), or
   [referenced lift](<Referenced Lift.md>).
4. Run that recipe's software checkpoint.
5. Cross only its linked physical bring-up gate.
6. Integrate the proven capability into your TeleOp.
7. [Sequence capability Tasks in Auto](<First Autonomous.md>).

Pedro Pathing is independent of the mechanism route. Add it when you are ready to
[follow one route and inspect its result](<First Pedro Auto.md>).

## Pick by the evidence you have

| Hardware behavior | Start with | What software can establish |
|---|---|---|
| drivetrain power | [First drive](<First Drive.md>) | source-to-sink composition and configured limits |
| continuous motor power | [Named intake](<Continuous Intake.md>) | named intent and heartbeat-applied output |
| servo positions without feedback | [Named claw](<Named Claw.md>) | normalized mapping; not physical arrival |
| motor position with switch and encoder | [Referenced lift](<Referenced Lift.md>) | reference/move decisions from injected evidence |
| several proven capabilities | [First autonomous](<First Autonomous.md>) | Task sequencing and outcome gates |
| route follower integration | [First Pedro Auto](<First Pedro Auto.md>) | route construction and terminal classification |

Every recipe links exact generated API documentation from type names and labels GitHub links as
**Complete source**. The short excerpts explain the idea; the compiling source remains authoritative.
