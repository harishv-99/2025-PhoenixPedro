# Choose a Sushi topic

**Learning mode:** Router

Use this page to choose a buildable module or focused reference.

First [get your first robot driving](<First Sushi Robot Code.md>) with one complete TeleOp. Then use
the seven checkpoints in [Build a robot step by step](<Build a Robot Step by Step.md>) as the
primary course. This page is its topic lookup: choose only the deep dive that answers the current
question. Use [Sushi in one picture](<Framework Overview.md>) when you need the continuous Starter
execution trace; it is not a prerequisite. The course's first software mechanism checkpoint links to
[Test a mechanism without hardware](<Test a Mechanism Without Hardware.md>).

| When you need to understand… | Read |
|---|---|
| Why profiles, composition roots, capabilities, mechanisms, and presenters have different jobs | [Robot roles](<learn-sushi/Robot Roles.md>) |
| How buttons and drive sticks acquire semantic meanings | [Controls and intent](<learn-sushi/Controls and Intent.md>) |
| How one request reaches one final actuator writer | [Plants and hardware](<learn-sushi/Plants and Hardware.md>) |
| How behavior waits, coordinates, times out, or runs in Auto without blocking | [Tasks and autonomous](<learn-sushi/Tasks and Autonomous.md>) |
| What software can report and what a physical experiment must establish | [Evidence and experiments](<learn-sushi/Evidence and Experiments.md>) |
| How to place a new team requirement in the architecture | [From requirement to robot](<learn-sushi/From Requirement to Robot.md>) |
| How vision observations, robot pose, and drive guidance stay separate | [Drive and vision](<../drive-vision/README.md>) |

The small **Starter** robot shows the complete ordinary shape. The larger **Reference** robot is a
case study for a referenced lift, velocity launcher, macros, status, and experiments. They use the
same architecture; Reference is not a second starter and should not be copied wholesale.

## Keep the evidence boundary visible

Checked-in profiles compile, but they are not reviewed facts about your robot. Source reading can
explain ownership and execution. It cannot prove electrical wiring, motor direction, safe motion,
calibration, or physical performance.

If your next job touches a device, first use [Build and run Sushi](<Build and Run.md>), then follow
the [testing and calibration](<../testing-calibration/README.md>) runbook under team supervision.
None of these conceptual topics requires you to edit an example, build the project, or have matching
hardware.
