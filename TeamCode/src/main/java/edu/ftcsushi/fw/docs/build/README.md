---
tags:
  - Build
---

# Build actuator knowledge one outcome at a time

The actuator lessons are one cumulative path. Start with the intake even if your robot does not
need one: it is the smallest complete example of a named capability, data-only configuration,
private Plant, managed output, controls, software evidence, and an isolated hardware gate. Each
later lesson assumes that vocabulary and teaches only the next hardware or evidence decision.

## Before the actuator path

1. [Set up and verify Sushi](<../getting-started/Build and Run.md>).
2. [Drive with a gamepad](<First Drive.md>) if you need the drivetrain path. Drive is independent
   of the actuator sequence below.

## Cumulative actuator path

1. [Run a continuous intake by name](<Continuous Intake.md>) — build the complete first actuator
   slice around normalized motor power.
2. [Move a claw through named positions](<Named Claw.md>) — add a bounded logical coordinate and
   map it to configured standard-servo endpoint candidates.
3. [Establish a lift reference](<Referenced Lift.md>) — add encoder units, an active-low switch,
   and a non-blocking reference search.
4. [Move the referenced lift and wait for feedback](<Move a Referenced Lift.md>) — add named
   positions and a feedback-aware Task whose outcome depends on fresh evidence.
5. [Command one flywheel velocity](<Single Flywheel Velocity.md>) — use a numeric command because
   velocity itself is the complete capability request and observe controller feedback separately.

Stop after the outcome your robot needs, but read the lessons in order so a later page can say
“same owner and heartbeat as before” instead of introducing a second architecture.

## Combine proven capabilities

After the relevant isolated software and hardware gates pass, [sequence capability Tasks in
Auto](<First Autonomous.md>). Pedro Pathing is a separate integration path; add it when you are
ready to [follow one route and inspect its result](<First Pedro Auto.md>).

## Choose by the new decision

| Outcome | First page that teaches it | New evidence or mapping |
|---|---|---|
| drivetrain power | [First drive](<First Drive.md>) | source-to-sink composition and configured limits |
| continuous motor power | [Continuous intake](<Continuous Intake.md>) | named intent and heartbeat-applied output |
| standard-servo positions | [Named claw](<Named Claw.md>) | bounded logical-to-native mapping without arrival feedback |
| encoder zero from a switch | [Referenced lift](<Referenced Lift.md>) | reference validity from authored switch evidence |
| bounded motor position | [Move a referenced lift](<Move a Referenced Lift.md>) | fresh position feedback and Task outcome |
| one motor velocity | [Single flywheel velocity](<Single Flywheel Velocity.md>) | requested, applied, measured, and arrived velocity |
| several proven capabilities | [First autonomous](<First Autonomous.md>) | Task sequencing and outcome gates |
| route follower integration | [First Pedro Auto](<First Pedro Auto.md>) | route construction and terminal classification |

Every recipe links exact generated API documentation and labels GitHub links as **Complete
source**. Displayed Java is copied from those compiling authorities; the explanation tells you why
each piece exists.
