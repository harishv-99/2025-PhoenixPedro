---
tags:
  - Advanced
---

# Advanced Sushi patterns

Start here only when the ordinary Build recipes cannot express the evidence or ownership your robot
needs. Each topic adds one specific idea; none replaces the managed program, one heartbeat, one
final writer, or fresh single-use Tasks.

## Mechanisms with richer target choices

- [Paired flywheel velocity](<Paired Flywheel Velocity.md>) — one grouped velocity command, two
  independent measurements, and readiness that requires both.
- [Periodic turret position](<Periodic Turret Position.md>) — one requested angle may have several
  equivalent physical representatives; choose the nearest legal one.
- [Mechanism target planning](<../drive-vision/Mechanism Target Planning.md>) — overlays and planned
  alternatives when exact or periodic commands are insufficient.
- [Output Tasks and queues](<../design/Output Tasks & Queues.md>) — temporary output ownership for
  genuinely time-shaped mechanism behavior.

## Larger robot graphs

- [Architecture roles and controls](<../design/Framework Lanes & Robot Controls.md>)
- [Capabilities shared by TeleOp and Auto](<../design/Robot Capabilities & Mode Clients.md>)
- [Supervisors and pipelines](<../design/Supervisors & Pipelines.md>)
- [Subsystem experiments](<../examples/Subsystem Experiments.md>)

## Drive, localization, and integration

- [Field-relative drive](<../examples/Field-relative Drive.md>)
- [Drive, spatial reasoning, and vision](<../drive-vision/README.md>)
- [Timestamped adaptive collection](<../examples/Timestamped Adaptive Collection.md>)
- [Pedro Pathing integration contract](<../../integrations/pedro/README.md>)

Framework maintainers should also read the [Framework Principles](<../../Framework Principles.md>)
and [Maintainer Notes](<../maintainers/Maintainer Notes.md>).
