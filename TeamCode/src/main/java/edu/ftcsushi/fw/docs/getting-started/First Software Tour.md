---
tags:
  - Get Started
---

# First software tour

This is the required first pass after [building the project](<Build and Run.md>). It uses three
maintained examples to answer one question at a time, entirely in software. Keep the example
OpModes disabled; you do not need a Control Hub, gamepad, motor, or matching drivetrain.

For each step, read only the linked **First pass** section, write down your prediction, run its one
focused test, and compare the result. The longer parts of each Build lesson are for when you choose
to build that mechanism.

## 1. Read a current value every loop

Open [First Drive: First pass—current values every loop](<../build/First Drive.md#first-pass-current-values-every-loop>).

**Predict:** if the software gamepad moves from centered to stick-up between two loops, will the
saved reader return its old number or the new reading?

Run the maintained drive scenario:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.firstdrive.FirstDriveSoftwareScenarioTest
```

The test keeps the production reader, coordinate mapping, drive caps, loop host, and STOP cleanup.
It replaces the controller and motors with software records. A pass proves the sampled drive
values, capped motor commands, and submitted STOP zeros—not physical wheel direction or motion.
You can also inspect the
[Complete source: `FirstDriveSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java>).

## 2. Run one function for one press

Open [Continuous Intake: First pass—run a function once per press](<../build/Continuous Intake.md#first-pass-run-a-function-once-per-press>).

**Predict:** across four samples—released, pressed, still pressed, released—how many times should
the saved A-button function run? Which named intake request should that accepted press select?

Run the controls-only scenario:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterFirstLessonTest
```

The test keeps the production gamepad adapter, controls, and saved rules, but replaces the intake
with a recorder. A pass proves that a press calls the short function once, holding and releasing do
not repeat it, and no timed work is created. It does not prove a motor command or intake motion.
See [Complete source: `StarterFirstLessonTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterFirstLessonTest.java>).

## 3. Advance bookmarked work across loops

Open [Run one timed Auto: First pass—work that continues across loops](<../build/Run One Timed Auto.md#first-pass-work-that-continues-across-loops>).

**Predict:** should INIT start collection? What request should be active at START, just before 0.75
seconds, and at 0.75 seconds? If FTC STOP arrives early, should the routine keep running?

Run the timed Auto scenario:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.opmode.StarterTimedAutoSoftwareScenarioTest
```

The test keeps the production routine factory, robot declaration, Task, intake mechanism, and
managed loop while replacing hardware and time with deterministic software versions. A pass proves
the software request at each tested boundary, early cancellation, and fresh single-use work. It
cannot prove real loop timing, motor motion, or a safe duration. See the
[Complete source: `StarterTimedAutoSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java>).

## Completion check

Match each need to one shape before continuing:

| Need | Shape |
| --- | --- |
| “What is the stick value now?” on every loop | continuously sampled value |
| “Run this short function once when A is pressed” | one-press callback |
| “Start now, remember progress, and finish on later loops” | multi-loop Task |

You are ready when you can explain why the first keeps sampling, the second runs once per rise, and
the third keeps a bookmark without a thread or sleep.

Hardware is optional for this tour. Each full Build lesson has a separate isolated hardware gate
for a team that owns the matching mechanism and is ready for supervised checks.
Software success does not grant permission to enable motion.

Continue to the [Guide map](<../README.md>) to choose a robot outcome or deeper explanation. Use
[Learn one Sushi idea](<Beginner's Guide.md>) when a concept question appears.
