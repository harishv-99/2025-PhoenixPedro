---
tags:
  - Learn
---

# Choose a Plant from the outcome you need

**Learning mode:** Decision guide

**Question:** Which ordinary Plant shape matches this actuator and the evidence it can provide?

A **Plant** is the mechanism-owned object that turns one held request into one final actuator
command on the shared heartbeat. Start from the hardware outcome below, then follow the linked
Build lesson. The lesson teaches the required builder stages in context; this page is not a catalog
of every API branch.

## Start with the outcome

| I need to… | Start here | What is new in that shape |
|---|---|---|
| run one motor forward, reverse, or stopped | [Continuous intake](<../../build/Continuous Intake.md>) | normalized power plus named semantic intent |
| move one standard servo among named positions | [Named claw](<../../build/Named Claw.md>) | bounded logical coordinate mapped to configured native endpoint candidates; no arrival feedback |
| discover where a motor-position coordinate begins | [Establish a lift reference](<../../build/Referenced Lift.md>) | encoder scale, reference requirement, switch source, and non-blocking search |
| move within a referenced motor-position coordinate | [Move a referenced lift](<../../build/Move a Referenced Lift.md>) | named target, cached measurement, tolerance, and feedback-aware Task |
| request one motor speed and observe feedback | [Single flywheel velocity](<../../build/Single Flywheel Velocity.md>) | bounded numeric command, velocity feedback, and cancellation-to-zero choice |
| drive two flywheels together but require both to be ready | [Paired flywheel velocity](<../../advanced/Paired Flywheel Velocity.md>) | grouped actuation plus independent member evidence |
| choose the nearest legal full-turn position | [Periodic turret position](<../../advanced/Periodic Turret Position.md>) | explicit equivalent-position selection inside physical bounds |

Read the first five rows in order if Plants are new. Paired velocity and periodic position are
Advanced branches for requirements that the ordinary single-actuator lessons do not have.

## The contract shared by every row

```text
capability request -> one mechanism-owned target -> private Plant
                   -> mechanism update(shared clock) -> one final actuator write
```

- An ordinary mechanism receives `HardwareMap` plus data-only configuration, copies what it keeps,
  constructs its private Plant, and owns update and terminal stop.
- Plant construction validates and connects the object graph. `build()` does not move hardware.
- Controls and Tasks change requests. They do not become peer hardware writers or run private loops.
- The managed output phase updates each declared mechanism. Status reads cached facts without
  polling hardware again.
- Software-valid defaults prove only that configuration is coherent. Direction, endpoints,
  reference cues, conversion, tuning, load response, and physical STOP still need isolated checks.

## Choose semantic or numeric intent

Use a semantic command when the robot request has a name such as `COLLECT`, `OPEN`, or `LOW`. One
mechanism maps that name forward to its numeric Plant target, and status retains both facts.

Use a numeric command when the scalar itself is the complete public request, such as flywheel
velocity in ticks per second or turret angle in radians. Do not add names that hide relevant
numeric meaning, and do not expose raw numbers when robot code actually means a named behavior.

## Keep evidence names honest

| Fact | What it answers |
|---|---|
| semantic request | Which named behavior did a client request? |
| requested target | Which number did the command source request? |
| applied target | Which final target survived bounds and guards? |
| measurement | Which sensor-backed value did the Plant cache? |
| `atTarget` / ready | Does the configured controller evidence meet its criterion? |
| physical result | Did the mechanism actually collect, clear, lift, or score? |

An applied target is not a measurement. Controller arrival is not proof of a successful game
action. Open-loop motor power and standard-servo position provide submitted-command evidence, not
physical arrival.

## Go deeper only when the requirement needs it

- [FTC Actuators and Plants](<../../ftc-boundary/FTC Actuators & Plants.md>) — exhaustive staged
  construction, lifecycle, and adapter contracts
- [Mechanism target planning](<../../drive-vision/Mechanism Target Planning.md>) — overlays,
  guards, equivalent positions, and advanced plans
- [Tasks and autonomous](<Tasks and Autonomous.md>) — cooperative work over time
- [Learn Sushi topic guide](<../Beginner's Guide.md>) — choose another concept
