# Role paths

**Question:** Which Sushi material should I read for the robot work in front of me?

Use this router after the shared [Sushi overview](<../Framework Overview.md>). These are focused
reading paths, not requirements to run or modify an example.

| Current job | Start here | Continue only as needed |
|---|---|---|
| Controls or drive | [Controls and intent](<Controls and Intent.md>) | [Sources and Signals](<../../core-concepts/Sources and Signals.md>) for conditioning; [Framework Lanes and Robot Controls](<../../design/Framework Lanes & Robot Controls.md>) for coordinated policy. |
| Mechanism or Plant | [Plants and hardware](<Plants and Hardware.md>) | Smallest owner: [`StarterIntakeMechanism.java`](<../../../../robots/examples/starter/capability/intake/StarterIntakeMechanism.java>); bounded position and launcher scale: `ReferenceLiftMechanism` and `ReferenceLauncherMechanism`; detailed grammar: [FTC Actuators and Plants](<../../ftc-boundary/FTC Actuators & Plants.md>). |
| Tasks or Auto | [Tasks and autonomous](<Tasks and Autonomous.md>) | [Tasks and Macros](<../../design/Tasks & Macros Quickstart.md>); use [First Pedro Auto](<../First Pedro Auto.md>) only for a robot that actually uses Pedro Pathing. |
| Vision or guidance | [Drive and Vision](<../../drive-vision/README.md>) | Add AprilTags, localization, spatial queries, or guidance only for a concrete requirement; the Reference robot does not pretend to own them. |
| Testing or calibration | [Evidence and experiments](<Evidence and Experiments.md>) | [Subsystem Experiments](<../../examples/Subsystem Experiments.md>), then [Actuator Bring-up](<../../testing-calibration/Actuator Bring-up.md>) before motion or [Control Tuning Workflow](<../../testing-calibration/Control Tuning Workflow.md>) for measured tuning. |
| Designing a subsystem | [From requirement to robot](<From Requirement to Robot.md>) | [Robot Capabilities and Mode Clients](<../../design/Robot Capabilities & Mode Clients.md>) and the one closest Reference capability. |

## Focused satellites

Some examples deliberately sit outside the common robot path:

- [Field-relative Drive](<../../examples/Field-relative Drive.md>) teaches an optional input
  transformation plus its Service and Prestart roles. It is not built into Sushi or required for
  robot-centric mecanum.
- [Pedro Autonomous Reference](<../../examples/Pedro Autonomous Reference.md>) owns third-party
  follower lifecycle and route outcomes; do not move that policy into a generic composition root.
- [Timestamped Adaptive Collection](<../../examples/Timestamped Adaptive Collection.md>) is an
  optional one-attempt satellite for delayed detector frames, start-time Pedro geometry, semantic
  milestones, inventory-gated exit, and exact collection/return outcomes. It is not a required
  vision layer or a complete Auto cycle.
- `ReferenceParkingPlan` is a controller-free geometry helper, not behavior executed by Reference
  Auto.
- `ReferenceTestersOpMode` uses the specialized tester lifecycle. A tester must create its own
  exclusive hardware owner and never share the production mechanism instance.

These are satellites because they provide distinct capabilities, not missing layers every robot
must add.

## BIOBUZZ lookup

The [BIOBUZZ capability map](<../../examples/BIOBUZZ Capability Map.md>) points speculative season
needs to the main examples and guides. It does not establish game rules, a hardware design, physical
criteria, or a new framework API.

## Check your route

**Do you need to read every satellite before programming a basic robot?**

No. Start with the common owner for your current job and add a focused satellite only when the
robot has its demonstrated requirement.

**Return to:** [Choose a Sushi topic](<../Beginner's Guide.md>)
