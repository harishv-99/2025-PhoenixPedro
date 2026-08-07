# Examples & case studies

Use this section when you want concrete, runnable framework examples plus the design reasoning
behind them.

The examples folder now has five complementary docs. Start with the managed starter for ordinary
robot code; the numbered examples keep their phases explicit as focused framework lessons:

- a complete `FtcRobotOpMode`/`RobotProgram` starter shared by TeleOp and Auto,
- a progression guide for the full `TeleOp_01` → `TeleOp_09` sequence,
- a dedicated walkthrough for the first explicit layered mechanism example,
- a shooter-focused case study for the AprilTag aim-assist examples,
- and a complete small Pedro autonomous reference using that same managed lifecycle.

## Read in this order

1. [`Modern Starter Robot.md`](<Modern Starter Robot.md>)
2. [`Examples Progression & Layered Mechanisms.md`](<Examples Progression & Layered Mechanisms.md>)
3. [`Layered Shooter Example.md`](<Layered Shooter Example.md>)
4. [`Shooter Case Study & Examples Walkthrough.md`](<Shooter Case Study & Examples Walkthrough.md>)
5. [`Pedro Autonomous Reference.md`](<Pedro Autonomous Reference.md>)
6. [`../../tools/examples/`](<../../tools/examples/>)

[`CustomVisionOwnershipExample.java`](../../tools/examples/CustomVisionOwnershipExample.java) in
that source folder shows the advanced ownership pattern:
one robot-owned semantic interface and immutable timestamped snapshot, with separate webcam-processor and
Limelight-pipeline realizations. It is intentionally not another numbered full OpMode.

## Pair with

- [`../design/README.md`](<../design/README.md>)
- [`../getting-started/Beginner's Guide.md`](<../getting-started/Beginner's Guide.md>)
