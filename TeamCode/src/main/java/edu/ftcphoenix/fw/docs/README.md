# Phoenix documentation

Phoenix helps an FTC team build non-blocking TeleOp and Auto programs from a small set of ideas:
sources describe values, bindings turn controls into intent, Tasks coordinate behavior over time,
Plants realize mechanism targets, and one managed program owns the loop.

## New to Phoenix

1. Read [Phoenix in five minutes](<getting-started/Framework Overview.md>) for the mental model.
2. Follow [Learn Phoenix](<getting-started/Beginner's Guide.md>) to trace the Starter and Reference
   robots from source without needing their hardware.
3. Choose a [role path](<getting-started/learn-phoenix/Role Paths.md>) for the part of the team robot
   you are working on.

For project setup, use [Build and run Phoenix](<getting-started/Build and Run.md>). Physical device
work starts separately at [Testing and calibration](<testing-calibration/README.md>).

## Find an answer

| Goal | Start here |
|---|---|
| Understand how the framework fits together | [Learn Phoenix](<getting-started/Beginner's Guide.md>) |
| Decide where new robot code belongs | [From requirement to robot](<getting-started/learn-phoenix/From Requirement to Robot.md>) |
| Remember the ordinary API shape | [Phoenix Cheat Sheet](<reference/Phoenix Cheat Sheet.md>) |
| Look up a Phoenix term | [Glossary](<reference/Glossary.md>) |
| Fix something that is not working | [Common Problems](<troubleshooting/Common Problems.md>) |
| Build a mechanism or choose a Plant recipe | [FTC Actuators and Plants](<ftc-boundary/FTC Actuators & Plants.md>) |
| Build a macro or Auto routine | [Tasks and Macros](<design/Tasks & Macros Quickstart.md>) |
| Understand sources, edges, or signal shaping | [Sources and Signals](<core-concepts/Sources and Signals.md>) |
| Structure a larger robot | [Architecture roles and controls](<design/Framework Lanes & Robot Controls.md>) |
| Bring up or calibrate hardware | [Testing and calibration](<testing-calibration/README.md>) |
| Add drive guidance or localization | [Drive and vision](<drive-vision/README.md>) |
| Build a Pedro autonomous | [First Pedro Auto](<getting-started/First Pedro Auto.md>) |
| Understand the production Phoenix robot | [Phoenix production reference](<../../robots/phoenix/README.md>) |
| Maintain or extend the framework | [Maintainers](<maintainers/README.md>) |

## Browse by purpose

### Learn and program a team robot

- [Getting started](<getting-started/README.md>) — source-based common path and setup
- [Examples](<examples/README.md>) — copyable Starter, Reference case study, and focused integrations
- [Reference](<reference/README.md>) — short lookups and exact current vocabulary
- [Troubleshooting](<troubleshooting/README.md>) — symptom-led recovery

### Add or validate a feature

- [Core concepts](<core-concepts/README.md>) — loop and source semantics
- [Design](<design/README.md>) — Tasks, capabilities, supervisors, and output queues
- [FTC boundary](<ftc-boundary/README.md>) — gamepad, actuator, sensor, UI, and handoff APIs
- [Drive and vision](<drive-vision/README.md>) — spatial queries, guidance, and localization
- [Testing and calibration](<testing-calibration/README.md>) — physical bring-up and controller evidence
- [Pedro Pathing integration](<../integrations/pedro/README.md>) — the narrow vendor boundary

### Maintain the framework

- [Framework Principles](<../Framework Principles.md>) — architecture and API design authority
- [Maintainer notes](<maintainers/Maintainer Notes.md>) — tests and extension notes
- [Phoenix Architecture](<../../robots/phoenix/Phoenix Architecture.md>) — production robot ownership

## Example labels

- **Copyable starter** — the smallest ordinary managed robot shape.
- **Teaching reference** — a larger coherent robot to study in layers, not copy wholesale.
- **Focused example** — one optional composition or integration.
- **Production reference** — a real robot with season-specific hardware and strategy.

The checked-in guides, compiling examples, Javadocs, and tests are the current documentation
authority.
