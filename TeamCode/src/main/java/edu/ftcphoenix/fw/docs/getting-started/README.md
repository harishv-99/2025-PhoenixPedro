# Getting started

This is the shortest supported path from a clean checkout to a Phoenix TeleOp and Auto. The core
course uses the checked-in starter robot throughout, so every core lesson points to Java that the
project compiles and tests.

Phoenix has one ordinary robot-programming path:

1. extend `FtcRobotOpMode`;
2. declare the robot once in `configure(RobotProgram program)`; and
3. let the framework run bindings, Tasks, outputs, drive, telemetry, and cleanup.

You do not forward FTC lifecycle callbacks or construct a clock or Task runner in ordinary robot
code.

## Orientation

Read these two pages before the numbered lessons:

- [`Framework Overview.md`](<Framework Overview.md>) — understand Phoenix in five minutes.
- [`Beginner's Guide.md`](<Beginner's Guide.md>) — see the complete course and checkpoints.

## Core beginner course

1. [`Build and Run.md`](<Build and Run.md>) — build the project before connecting motion hardware.
2. [`First Mechanism.md`](<First Mechanism.md>) — use the actuator bring-up wizard, copy one tested
   motor direction, and run a safe Auto checkpoint.
3. [`First TeleOp.md`](<First TeleOp.md>) — add conservatively limited drive motors and map A/B/X
   controls.
4. [`First Task and Auto.md`](<First Task and Auto.md>) — run timed behavior without blocking.

These four lessons use one continuous starter robot.

## Optional next track

[`First Pedro Auto.md`](<First Pedro Auto.md>) is a software-first route-following walkthrough for
students who have finished the core course. It switches to the dedicated Pedro reference, remains
`@Disabled`, and explains the separate configuration and calibration required before any later
physical test. It is not another required starter lesson.

The core source starts at [`StarterAuto.java`](<../../../robots/examples/starter/StarterAuto.java>).
The optional Pedro track starts at its separate
[`BasicPedroAutoExample.java`](<../../../robots/examples/pedro/BasicPedroAutoExample.java>) entry.

## Keep these three rules visible

- Robot behavior must not sleep or wait in a blocking loop. Use a `Task` for work over time.
- A mechanism privately owns its Plants and exposes robot meanings such as `collect()` or
  `collectForSeconds(...)`.
- Establish one actuator's direction and optional safe endpoints with
  [`HW: Actuator Bring-up`](<../testing-calibration/Actuator Bring-up.md>), then verify names,
  directions, limits, physical placement, and immediate STOP behavior in the production owner. A
  successful build cannot prove those facts.

## After the course

Use the main [`Phoenix docs hub`](<../README.md>) to find mechanism, controls, vision, localization,
calibration, and design references. Those pages explain deeper behavior but are not prerequisites
for the first robot.

**Next:** [`Phoenix in five minutes`](<Framework Overview.md>)
