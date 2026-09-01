# Sushi documentation

Sushi helps an FTC team build non-blocking TeleOp and Auto programs from a small set of ideas:
sources describe values, bindings turn controls into intent, Tasks coordinate behavior over time,
Plants realize mechanism targets, and one managed program owns the loop.

## New to Sushi

1. [Set up and verify the project](<getting-started/Build and Run.md>).
2. [Build the Basic Mechanisms robot](<getting-started/Basic Mechanisms Robot.md>) from first drive
   through a referenced lift, claw, semantic tests, integrated TeleOp, and Auto.
3. Read [Sushi in one picture](<getting-started/Framework Overview.md>) when you need the managed
   execution model.
4. [Choose a Sushi topic](<getting-started/Beginner's Guide.md>) when the current checkpoint needs a
   deeper answer. The topic pages are lookups, not a required course sequence.

Detailed hardware procedures live under [Testing and calibration](<testing-calibration/README.md>).

## Find an answer

| Goal | Start here |
|---|---|
| Get a standard mecanum robot under joystick control | [Build the Basic Mechanisms robot](<getting-started/Basic Mechanisms Robot.md>) |
| Understand how the framework executes | [Sushi in one picture](<getting-started/Framework Overview.md>) |
| Exercise a real mechanism and Plant without hardware | [Run the Basic Mechanisms software checkpoint](<getting-started/Basic Mechanisms Robot.md#software-checkpoint-request-heartbeat-recorded-output>) |
| Move from requirements through robot evidence one checkpoint at a time | [Build the Basic Mechanisms robot](<getting-started/Basic Mechanisms Robot.md>) |
| Learn one framework area in depth | [Choose a Sushi topic](<getting-started/Beginner's Guide.md>) |
| Decide where new robot code belongs | [From requirement to robot](<getting-started/learn-sushi/From Requirement to Robot.md>) |
| Remember the ordinary API shape | [Sushi Cheat Sheet](<reference/Sushi Cheat Sheet.md>) |
| Look up a Sushi term | [Glossary](<reference/Glossary.md>) |
| Fix something that is not working | [Common Problems](<troubleshooting/Common Problems.md>) |
| Build a mechanism or choose a Plant recipe | [FTC Actuators and Plants](<ftc-boundary/FTC Actuators & Plants.md>) |
| Build a macro or Auto routine | [Tasks and Macros](<design/Tasks & Macros Quickstart.md>) |
| Understand sources, edges, or signal shaping | [Sources and Signals](<core-concepts/Sources and Signals.md>) |
| Structure a larger robot | [Architecture roles and controls](<design/Framework Lanes & Robot Controls.md>) |
| Bring up or calibrate hardware | [Testing and calibration](<testing-calibration/README.md>) |
| Add drive guidance or localization | [Drive and vision](<drive-vision/README.md>) |
| Build a Pedro autonomous | [First Pedro Auto](<getting-started/First Pedro Auto.md>) |
| Maintain or extend the framework | [Maintainers](<maintainers/README.md>) |

## Browse by purpose

### Learn and program a team robot

- [Examples](<examples/README.md>) — the Basic Mechanisms course, Reference case study, and focused integrations
- [Reference](<reference/README.md>) — short lookups and exact current vocabulary
- [Troubleshooting](<troubleshooting/README.md>) — symptom-led recovery

### Add or validate a feature

- [Core concepts](<core-concepts/README.md>) — loop and source semantics
- [Design](<design/README.md>) — Tasks, capabilities, supervisors, and output queues
- [FTC boundary](<ftc-boundary/README.md>) — gamepad, actuator, sensor, UI, and handoff APIs
- [Drive and vision](<drive-vision/README.md>) — spatial queries, guidance, and localization
- [Testing and calibration](<testing-calibration/README.md>) — route software evidence into physical bring-up and controller evidence
- [Pedro Pathing integration](<../integrations/pedro/README.md>) — the narrow vendor boundary

### Maintain the framework

- [Framework Principles](<../Framework Principles.md>) — architecture and API design authority
- [Maintainer notes](<maintainers/Maintainer Notes.md>) — tests and extension notes

## Example labels

- **Copyable starter** — the smallest ordinary managed robot shape.
- **Teaching reference** — a larger coherent robot to study in layers, not copy wholesale.
- **Focused example** — one optional composition or integration.

The checked-in guides, compiling examples, Javadocs, and tests are the current documentation
authority.
