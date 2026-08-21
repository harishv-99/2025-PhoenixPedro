# Getting started

Phoenix has one ordinary robot-programming path:

1. extend `FtcRobotOpMode`;
2. declare the robot once in `configure(RobotProgram program)`; and
3. let the framework run bindings, Tasks, outputs, drive, telemetry, and cleanup.

You do not forward FTC lifecycle callbacks or construct a clock or Task runner in ordinary robot
code.

## Learn the framework from source

Start with [Phoenix in one picture](<Framework Overview.md>). It follows one Starter button through
the managed lifecycle and the mechanism-owned hardware-command path. Reading it requires no
matching robot, example execution, edit, or individual build project.

After [setting up and verifying the project](<Build and Run.md>), complete
[your first Phoenix robot code](<First Phoenix Robot Code.md>). It copies the one canonical Starter
into a team package, changes one button meaning, and proves the change with a hardware-free test.
The exercise reveals one owner at a time; it does not require reading all seven Starter files first.

Next, [test its mechanism without hardware](<Test a Mechanism Without Hardware.md>). That second
software lesson keeps the production `HardwareMap + Config` constructor and private Plant while a
test-only hardware registry records its actuator command. Then [choose a Phoenix topic](<Beginner's Guide.md>)
for the problem in front of you. These pages are optional deep dives rather than a required
sequence. [Role paths](<learn-phoenix/Role Paths.md>) provides a more detailed map.

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
example. It is an optional integration topic and does not authorize a physical route test.

## Keep these three rules visible

- Robot behavior must not sleep or wait in a blocking loop. Use a fresh `Task` for work over time.
- A mechanism privately owns its Plants and exposes robot meanings such as `setMode(COLLECT)`,
  `collectForSeconds(...)`, or `launchOne()`.
- Software evidence and physical evidence are different; claim only what the available source or
  measurement proves.

**First-contact guide:** [Phoenix in one picture](<Framework Overview.md>)

**First coding exercise:** [Your first Phoenix robot code](<First Phoenix Robot Code.md>)

**First mechanism scenario:** [Test a mechanism without hardware](<Test a Mechanism Without Hardware.md>)
