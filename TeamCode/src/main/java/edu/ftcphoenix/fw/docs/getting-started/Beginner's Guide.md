# Choose a Phoenix topic

Start with [Phoenix in one picture](<Framework Overview.md>). After you can trace the Starter A
button from gamepad intent to the mechanism-owned hardware command, choose only the topic that
answers your current question. These are independent deep dives, not required course chapters.

| When you need to understand… | Read |
|---|---|
| Why profiles, composition roots, capabilities, mechanisms, and presenters have different jobs | [Robot roles](<learn-phoenix/Robot Roles.md>) |
| How buttons and drive sticks acquire semantic meanings | [Controls and intent](<learn-phoenix/Controls and Intent.md>) |
| How one request reaches one final actuator writer | [Plants and hardware](<learn-phoenix/Plants and Hardware.md>) |
| How behavior waits, coordinates, times out, or runs in Auto without blocking | [Tasks and autonomous](<learn-phoenix/Tasks and Autonomous.md>) |
| What software can report and what a physical experiment must establish | [Evidence and experiments](<learn-phoenix/Evidence and Experiments.md>) |
| How to place a new team requirement in the architecture | [From requirement to robot](<learn-phoenix/From Requirement to Robot.md>) |

The small **Starter** robot shows the complete ordinary shape. The larger **Reference** robot is a
case study for a referenced lift, velocity launcher, macros, status, and experiments. They use the
same architecture; Reference is not a second starter and should not be copied wholesale.

## Choose by team role

[Role paths](<learn-phoenix/Role Paths.md>) routes controls, mechanisms, Auto, vision, and testing
work to the relevant examples and detailed guides. Read a deeper reference only when the robot has
that need.

## Keep the evidence boundary visible

Checked-in profiles compile, but they are not reviewed facts about your robot. Source reading can
explain ownership and execution. It cannot prove electrical wiring, motor direction, safe motion,
calibration, or physical performance.

If your next job touches a device, first use [Build and run Phoenix](<Build and Run.md>), then follow
the [testing and calibration](<../testing-calibration/README.md>) runbook under team supervision.
None of these conceptual topics requires you to edit an example, build the project, or have matching
hardware.
