---
tags:
  - Build
---

# Build one robot outcome at a time

Read this course with basic Java and FTC experience; Sushi concepts and unfamiliar Java syntax are
explained where they first become useful. Reading is a complete learning path: no installation,
code changes, test run, or matching hardware is required. The short
[first software tour](<../getting-started/First Software Tour.md>) previews a changing sensor fact,
one button press, and one timed action before these pages explain their complete owners.

## Choose how to follow a lesson

Each page states its knowledge prerequisites, the production idea, and the expected observations
from a maintained software scenario. Read the code and predict those observations, then compare
your reasoning with the explanation on the page. You have finished the reading checkpoint when you
can explain which owner changes each fact and why the next loop or action changes it again.

Optionally [set up and verify the project](<../getting-started/Build and Run.md>) to run the supplied
scenario. A passing test supplies software evidence for its stated question; reading an expected
result does not claim that you ran it. You may also author the focused slice in your own robot
package using the guidance below. A physical check is a separate, optional activity that requires
the page's complete hardware procedure.

## Understand one part

The knowledge is cumulative; the hardware fixtures are intentionally independent. Read the lessons
in order to learn the ordinary ownership pattern without assembling an ever-growing example robot.

1. [Read a switch and show its state](<Read a Switch.md>) — one digital input becomes an explicit,
   debounced fact in cached status. No actuator is involved.
2. [Run a continuous intake by name](<Continuous Intake.md>) — one button changes a persistent
   request; the mechanism owns the private Plant and final motor write.
3. [Move a claw through named positions](<Named Claw.md>) — reuse that ownership pattern while
   mapping logical positions into configured servo endpoints, without claiming arrival feedback.

The intake is the complete first actuator lesson. It adds a command and hardware output to the
earlier observation vocabulary. The claw keeps that path and adds one position-mapping decision.

## Compose a TeleOp and a basic Auto

Drive is independent of the actuator fixtures. Its knowledge prerequisite is understanding a
reusable current-value reader; it does not require an intake, claw, lift, or flywheel.

1. [Drive with a gamepad](<First Drive.md>) — current stick values reach one managed drivetrain sink
   continuously, rather than through button callbacks.
2. [Combine drive and intake in one TeleOp](<Combine Drive and Intake.md>) — two already-explained
   owners share a gamepad and one managed lifecycle.
3. [Run one timed root Task in Auto](<Run One Timed Auto.md>) — reuse the intake capability, start at
   FTC START, finish after the stated duration, and handle early STOP.

At this point you can explain a complete managed TeleOp and a basic timed mechanism Auto. The Auto
does not drive a route. Route following and additional feedback mechanisms are separate choices,
not prerequisites for reaching this checkpoint.

## Add feedback when your robot needs it

These are optional continuation lessons. Each names the prior concepts it uses; read those
explanations as needed without building the earlier hardware.

1. [Establish a lift reference](<Referenced Lift.md>) — reuse switch polarity and debounce to
   establish encoder zero through cooperative homing.
2. [Move the referenced lift and wait for feedback](<Move a Referenced Lift.md>) — distinguish a
   persistent request from a Task that waits for fresh arrival evidence.
3. [Sequence capability Tasks in Auto](<First Autonomous.md>) — admit the next lift action only
   after exact success; retain abnormal outcomes and cancellation.
4. [Command one flywheel velocity](<Single Flywheel Velocity.md>) — use a numeric request because
   velocity itself is the complete intent, with controller feedback and explicit cancellation.

Pedro is an optional integration branch:
[inspect one fixed route's software outcome](<First Pedro Auto.md>). That page is explicitly a
blocked software-boundary checkpoint. It does not authorize physical route motion or supply a
complete hardware recipe while the managed route power-limit boundary remains unresolved.
Choose other [Advanced patterns](<../advanced/README.md>) individually when a robot requirement
needs them.

## Optional: author the slice in your robot { #author-in-your-robot }

The main code for your robot belongs under
`TeamCode/src/main/java/edu/ftcsushi/robots/myrobot/`, with package
`edu.ftcsushi.robots.myrobot`. Replace `myrobot` with your team's robot name. This is a sibling of
`examples` and other robot application packages, not a subpackage of them. A separate scratch robot can use another
sibling such as `edu.ftcsushi.robots.practicebot`; creating one is optional.

Keep that robot's configuration, controls, capability owners, presenters, and OpModes together.
Begin with the one slice you understand. Its maintained **Main** manifest identifies the pieces to
adapt, while the displayed construction, registration, and status code explains how they connect.
Use your robot's package and imports throughout; a package-private controls owner must remain in
the package of the clients that use it. Keep the examples as the compiling reference rather than
making your robot depend on example classes or another robot application's code.

Make one small change at a time; stop after any step and compare your code with the explained path:

1. Add the data-only configuration and the one owner for this outcome. State the hardware facts
   that owner retains and validate them before lookup.
2. Add the source or named request and its cached status. Predict which fields can change before
   the next managed update and which require that update.
3. Connect that owner to the managed program, then add only the required controls and presenter.
   Keep its update and stop responsibilities visible; do not add another FTC loop.
4. Adapt the matching software scenario so it constructs your owner and, when the question covers
   integration, your declaration path. Change one of your configuration values and its expected
   observation together to check that the scenario actually exercises your code.

Your tests belong under `TeamCode/src/test/java/edu/ftcsushi/robots/myrobot/`, using the matching
robot package. Test-only clocks and hardware probes stay in test sources, never in robot main
code. The supplied **Test** manifests and commands run the maintained examples, not your new robot.
Update the test's package, owner imports, construction calls, and Gradle selector. If it uses
package-private scenario support, adapt that support into the matching test package as well; do not
make example internals public simply to borrow them. Reuse the shared test-only clock and device
probes. Merely renaming a selector or running an unchanged reference test does not verify your
owner. A deliberate temporary wrong expected value should fail the matching assertion; restore the
correct expectation before retaining the test.

Keep each physical motion permission false and teaching OpModes disabled while authoring. Merely
copying a reviewed software shape does not review your wiring, power, travel, or stop behavior.
Complete only the isolated hardware gate for the mechanism you actually intend to operate.

## Where each concept first appears

| Concept you need | First Build lesson | What that lesson makes explicit |
|---|---|---|
| sensor value, polarity, and debounce | [Read a switch](<Read a Switch.md>) | one sampled fact, cached status, and a presenter that does not resample |
| synchronous button meaning | [Continuous intake](<Continuous Intake.md>) | a saved callback calls a capability setter on one rise |
| capability, profile, Plant, and output | [Continuous intake](<Continuous Intake.md>) | a named request reaches one private final writer |
| bounded named servo positions | [Named claw](<Named Claw.md>) | logical coordinate and native endpoints; no arrival claim |
| continuous gamepad axes | [First drive](<First Drive.md>) | a controls-owned source reaches one managed drive sink |
| several owners in one TeleOp | [Combine drive and intake](<Combine Drive and Intake.md>) | shared inputs, explicit output order, cached presentation, and STOP |
| one behavior over time in Auto | [Run one timed Auto](<Run One Timed Auto.md>) | fresh root Task, START boundary, duration, and cancellation |
| direct request versus feedback Task | [Move a referenced lift](<Move a Referenced Lift.md>) | coherent request/arrival evidence and an explicit cancellation choice |

Use [Learn](<../getting-started/Beginner's Guide.md>) for an on-demand concept explanation and
[Reference](<../reference/README.md>) for exact API lookup. Complete-source links supply package,
import, and other mechanical details; the important ownership, active values, and heartbeat
connections remain explained in each lesson.
