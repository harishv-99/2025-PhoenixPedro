---
tags:
  - Learn
---

# Choose a Plant from the outcome you need { #plant }

**Learning mode:** Decision guide

**Question:** Which ordinary Plant shape matches this actuator and the evidence it can provide?

!!! info "New concept: Plant"

    A **Plant** is the mechanism-owned path that turns one held request into one final actuator
    command for a motor or servo. The mechanism updates it once per managed loop and owns its shutdown.

Choose the outcome below, then follow its linked Build explanation. This is an on-demand decision
guide, not a required actuator tour; no installation, test run, or hardware is needed to read it.
Each lesson teaches its builder stages in context, rather than cataloging every API branch.

A read-only [switch observation](<../../build/Read a Switch.md>) does not need a Plant: a Plant
exists to realize an actuator request, not merely to format sensor status.

## Start with the outcome

| I need to… | Start here | What is new in that shape |
|---|---|---|
| run one motor forward, reverse, or stopped | [Continuous intake](<../../build/Continuous Intake.md>) | power as a fraction of the command range, selected through named [intent](<Controls and Intent.md#intent>) |
| move one standard servo among named positions | [Named claw](<../../build/Named Claw.md>) | translate the lesson's closed-to-open scale into reviewed servo commands; no measurement of arrival |
| discover where a motor-position coordinate begins | [Establish a lift reference](<../../build/Referenced Lift.md>) | use a switch to give counted encoder movement a known zero |
| move within a referenced motor-position coordinate | [Move a referenced lift](<../../build/Move a Referenced Lift.md>) | use a [Task](<../Framework Overview.md#task>) to wait until a new measurement is close enough to the request |
| request one motor speed and observe feedback | [Single flywheel velocity](<../../build/Single Flywheel Velocity.md>) | compare requested speed with measured speed; choose the request on cancellation |
| drive two flywheels together but require both to be ready | [Paired flywheel velocity](<../../advanced/Paired Flywheel Velocity.md>) | grouped actuation plus independent member evidence |
| choose the nearest legal full-turn position | [Periodic turret position](<../../advanced/Periodic Turret Position.md>) | explicit equivalent-position selection inside physical bounds |

Start with the intake row if Plants are new. Claw mapping adds one position decision; lift and
velocity are optional feedback extensions whose pages name the knowledge they need. Paired
velocity and periodic position are Advanced branches with additional evidence requirements.

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

**Semantic** means carrying a robot meaning. Use a semantic command when the robot request has a name such as `COLLECT`, `OPEN`, or `LOW`. One
mechanism maps that name forward to its numeric Plant target, and status retains both facts.

Use a numeric command when the scalar (one number) itself is the complete public request, such as flywheel
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

**Feedback** is a measurement returned from the mechanism. **Open-loop** means this command path
does not use such a measurement to establish arrival. The optional feedback lessons explain
controllers and acceptable error before asking you to configure them.

## Go deeper only when the requirement needs it

- [FTC Actuators and Plants](<../../ftc-boundary/FTC Actuators & Plants.md>) — exhaustive staged
  construction, lifecycle, and adapter contracts
- [Mechanism target planning](<../../drive-vision/Mechanism Target Planning.md>) — overlays,
  guards, equivalent positions, and advanced plans
- [Tasks and autonomous](<Tasks and Autonomous.md>) — cooperative work over time
- [Learn Sushi topic guide](<../Beginner's Guide.md>) — choose another concept
