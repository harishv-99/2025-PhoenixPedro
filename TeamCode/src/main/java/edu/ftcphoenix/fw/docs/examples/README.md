# Examples

Examples have different jobs. Check the classification before deciding whether to copy the file or
study only the focused idea.

| Classification | Example | Use it for |
|---|---|---|
| **Copyable starter** | [`Modern Starter Robot`](<Modern Starter Robot.md>) and its [`StarterTeleOp.java`](<../../../robots/examples/starter/StarterTeleOp.java>) entry | The ordinary managed TeleOp/Auto structure for a new robot. |
| **Managed drive lesson** | [`Field-relative Drive`](<Field-relative Drive.md>) and its [`FieldRelativeDriveExample.java`](<../../../robots/examples/fieldrelative/FieldRelativeDriveExample.java>) entry | Explicit station-defined driver up using either the Hub IMU or a full pose estimator. |
| **Optional Pedro integration reference** | [`Pedro Autonomous Reference`](<Pedro Autonomous Reference.md>) and its [`BasicPedroAutoExample.java`](<../../../robots/examples/pedro/BasicPedroAutoExample.java>) entry | A small managed Auto with one Pedro boundary, studied after the guided software lesson. |
| **Concept labs** | Start with [`TeleOp_01_MecanumBasic.java`](<../../tools/examples/TeleOp_01_MecanumBasic.java>), then follow the `TeleOp_02` through `TeleOp_09` progression | Focused framework behavior with explicit lifecycle phases visible. Do not copy their manual host as the ordinary robot architecture. |
| **Advanced case study** | [`Layered Shooter Example`](<Layered Shooter Example.md>) | Held, frame-valued, and pending requests inside one mechanism owner. |
| **Advanced case study** | [`Shooter Case Study & Examples Walkthrough`](<Shooter Case Study & Examples Walkthrough.md>) | AprilTag selection, shared aim intent, and shooter guidance. |
| **Production reference** | [`Phoenix robot`](<../../../robots/phoenix/README.md>) | A complete season robot with real policy, readiness, localization, scoring, and Pedro routes. |

[`Examples Progression & Layered Mechanisms`](<Examples Progression & Layered Mechanisms.md>) is an
index to the concept labs. Its explicit clock and FTC callbacks are useful when studying the phases
or building a genuinely custom host; ordinary code under `edu.ftcphoenix.robots` uses
`FtcRobotOpMode` and `RobotProgram`.

If this is your first Phoenix robot, follow the
[`beginner course`](<../getting-started/Beginner's Guide.md>) before browsing the advanced examples.

[Back to the Phoenix docs home](<../README.md>)
