---
tags:
  - Build
---

# Build one robot outcome at a time

The knowledge is cumulative; the hardware fixtures are intentionally independent. Start with the
intake even if your robot does not need one: it is the smallest complete example of a named
capability, data-only configuration, private Plant, managed output, controls, software evidence,
and an isolated hardware gate. A later lesson may assume that vocabulary, but it does not make you
carry unrelated motors and servos into its focused test.

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
“same owner and heartbeat as before” instead of introducing a second architecture. When a page
switches example packages, its **Put this focused fixture into your robot** section shows the
profile, output, controls, presenter, and managed-host connections you must carry over.

The Pedro route page is deliberately different: it is explicitly labeled a blocked
software-boundary checkpoint, not a reconstruction-grade hardware recipe. Its current lesson ends
at exact route-attempt classification because the first conservative physical gate cannot yet be
authorized through the managed integration.

## Put the pieces together

After both focused slices pass their gates, [combine drive and intake in one TeleOp](<Combine Drive and Intake.md>).
That lesson shows how one composition root shares a gamepad while preserving a continuous drive
path and callback-driven mechanism intent.

For Auto, first [run one timed root Task](<Run One Timed Auto.md>). Then
[sequence capability Tasks](<First Autonomous.md>) and let exact outcomes decide whether later work
may start. Pedro Pathing is a separate integration path; add it when you are ready to
[verify one fixed route's software outcome](<First Pedro Auto.md>).

## Where each concept first appears

| Concept you need | First Build lesson | What that lesson makes explicit |
|---|---|---|
| continuous gamepad axes | [First drive](<First Drive.md>) | `GamepadDevice` → `DriveSource` → one managed drive sink; no callback binding |
| synchronous button meaning | [Continuous intake](<Continuous Intake.md>) | `CallbackBindings.onRise(...)` calls a capability setter |
| capability, profile, Plant, status, presenter | [Continuous intake](<Continuous Intake.md>) | the complete ordinary actuator ownership chain |
| bounded named servo positions | [Named claw](<Named Claw.md>) | logical `[0, 1]` mapped to configured native endpoint candidates; no arrival claim |
| Boolean sensor, polarity, and debounce | [Referenced lift](<Referenced Lift.md>) | switch evidence establishes a reference through a fresh Task |
| direct request versus wait-for-feedback Task | [Move a referenced lift](<Move a Referenced Lift.md>) | completion and cancellation policy use coherent semantic/numeric status |
| numeric velocity feedback | [Single flywheel velocity](<Single Flywheel Velocity.md>) | requested, applied, measured, arrival, timeout, and cancel-to-stop |
| several owners in one TeleOp | [Combine drive and intake](<Combine Drive and Intake.md>) | shared controls, declaration order, output/presenter ownership, collision checks, and STOP |
| one behavior over time in Auto | [Run one timed Auto](<Run One Timed Auto.md>) | fresh root Task, START boundary, duration, and cancellation-safe persistent request |
| Task composition and outcome gates | [First autonomous](<First Autonomous.md>) | sequence first; parallel work only after the basic outcome path is clear |

Sensor-to-status observation without motion is a separate later Build slice. Prestart selection,
services, field-relative drive, vision, Pedro internals, and experiments remain later outcomes rather
than prerequisites hidden inside these lessons.

Every recipe links exact generated API documentation and labels GitHub links as **Complete
source**. Displayed Java is copied from those compiling authorities; the explanation tells you why
each piece exists.
