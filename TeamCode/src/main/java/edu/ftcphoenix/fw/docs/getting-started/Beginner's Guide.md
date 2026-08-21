# Learn Phoenix

This self-guided path explains how Phoenix robot code fits together. You do not need the example
robot, and you are not expected to enable an OpMode, edit the examples, or build a complete robot
alone. Read the small excerpts, follow each intent or evidence flow, and use only the relevant
patterns in your team's shared robot.

**Audience:** FTC students who know basic Java classes, methods, and enums.

**Time:** About two hours for the common path. Role paths are optional follow-up reading.

## The learning contract

The course starts with the small Starter robot, then reveals the Reference robot one layer at a
time. They are not competing architectures:

- **Starter** is the smallest ordinary Phoenix shape and the easiest place to see an entire flow.
- **Reference** is a larger teaching robot that adds a referenced lift, a velocity launcher,
  non-blocking macros, status, and an experiment.
- **Focused examples and guides** cover integrations the Reference robot does not own.

The checked-in configurations are compiling placeholders, not facts about your team's hardware.
Software can explain ownership and behavior; it cannot prove wiring, safe motion, calibration, or
physical performance.

## Common path

Read these chapters in order. Each one has a bounded source trace, questions with answers, and a
clear statement of what to copy, adapt, or leave framework-owned.

| Step | Chapter | You should be able to explain |
|---:|---|---|
| 1 | [Robot roles](<learn-phoenix/Robot Roles.md>) | Why the profile, OpMode, composition root, capability, and mechanism have different jobs. |
| 2 | [Controls and intent](<learn-phoenix/Controls and Intent.md>) | How a semantic gamepad action reaches a capability without controls owning hardware. |
| 3 | [Plants and hardware](<learn-phoenix/Plants and Hardware.md>) | How one mechanism turns a request into one final hardware realization path. |
| 4 | [Tasks and autonomous](<learn-phoenix/Tasks and Autonomous.md>) | How fresh, non-blocking Tasks coordinate behavior and retain truthful outcomes. |
| 5 | [Evidence and experiments](<learn-phoenix/Evidence and Experiments.md>) | How electrical observations become semantic status without inventing physical conclusions. |
| 6 | [From requirement to robot](<learn-phoenix/From Requirement to Robot.md>) | Where a new team-robot requirement belongs and which owners must collaborate. |

The goal is understanding, not producing a modified example. The final chapter lets you check your
design reasoning on paper before your team changes robot code.

## Choose the path relevant to your role

After the common path, use [Role paths](<learn-phoenix/Role Paths.md>) to continue with controls and
drive, mechanisms and Plants, Tasks and autonomous, vision/localization/guidance, or testing and
experiments. Read only the path that answers the problem in front of you.

## Where the concepts really live

The Reference robot is the common case study, not a kitchen-sink robot. This coverage map keeps the
examples truthful:

| Concept | Teaching source |
|---|---|
| Minimal managed robot and direct-power mechanism | Starter robot |
| Profile, root, capabilities, mechanisms, controls, Tasks, status, presenters | Reference robot |
| Referenced lift, velocity launcher, transfer overlay, locked experiment | Reference robot |
| Service, Prestart, and field-relative composition | Field-relative focused example |
| Parking geometry policy | `ReferenceParkingPlan`; it is not executed by Reference Auto |
| Pedro follower lifecycle and route outcomes | Pedro focused example and tutorial |
| AprilTags, localization, and guidance | Canonical guides and production references |
| Haptics and supervisors | Optional guides, Javadocs, and production references |

## If you are working with real hardware

Conceptual understanding and hardware validation are separate jobs. Start with
[Build and run Phoenix](<Build and Run.md>) for project setup, then use the
[testing and calibration hub](<../testing-calibration/README.md>) and
[actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) before enabling motion. A software
example never authorizes physical movement.

## Common questions

**Should I copy the Reference robot?**

No. Copy or adapt a small ownership pattern only when it matches a real team requirement. Keep the
team's capability names, configuration, physical limits, and evidence truthful to its robot.

**Do I need to understand every framework package first?**

No. Complete the common path, then use one role path. Advanced integrations should appear only when
the robot has the corresponding need.

**Must I run these examples to finish the course?**

No. The course is source-based. Physical bring-up is a separate supervised team activity.

**Next:** [Robot roles](<learn-phoenix/Robot Roles.md>)
