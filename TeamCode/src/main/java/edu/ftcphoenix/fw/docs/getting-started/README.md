# Getting started

Phoenix has one ordinary robot-programming path:

1. extend `FtcRobotOpMode`;
2. declare the robot once in `configure(RobotProgram program)`; and
3. let the framework run bindings, Tasks, outputs, drive, telemetry, and cleanup.

You do not forward FTC lifecycle callbacks or construct a clock or Task runner in ordinary robot
code.

## Learn the framework from source

Start with [Phoenix in five minutes](<Framework Overview.md>), then follow
[Learn Phoenix](<Beginner's Guide.md>). The self-guided walkthrough begins with one small Starter
flow and reveals the Reference robot one layer at a time. It requires no matching robot, example
execution, or individual build project.

The common path covers robot roles, controls and intent, Plants and hardware ownership, Tasks and
Auto, evidence and experiments, and the workflow from a requirement to team robot code. Afterward,
the [role paths](<learn-phoenix/Role Paths.md>) route you to only the controls, mechanisms, Auto,
vision, or testing material relevant to your work.

## Set up or work with physical hardware

[Build and run Phoenix](<Build and Run.md>) explains project setup and software verification. When
the team is ready to work with real devices, use the
[testing and calibration hub](<../testing-calibration/README.md>) and
[actuator bring-up](<../testing-calibration/Actuator Bring-up.md>). Source understanding does not
authorize motion or prove wiring, direction, calibration, safe limits, or physical performance.

The older First Mechanism, First TeleOp, and First Task URLs remain as compatibility pages. They
point separately to the conceptual chapter and the physical runbook rather than defining a second
course.

## Optional integration tutorial

[First Pedro Auto](<First Pedro Auto.md>) is a source-only tutorial for the dedicated managed Pedro
example. It is not part of the common path and does not authorize a physical route test.

## Keep these three rules visible

- Robot behavior must not sleep or wait in a blocking loop. Use a fresh `Task` for work over time.
- A mechanism privately owns its Plants and exposes robot meanings such as `collect()` or
  `launchOne()`.
- Software evidence and physical evidence are different; claim only what the available source or
  measurement proves.

**Next:** [Phoenix in five minutes](<Framework Overview.md>)
