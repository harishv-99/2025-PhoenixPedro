---
tags:
  - Get Started
---

# Set up and verify the Sushi project

**Learning mode:** Operational runbook

Complete these checks on the existing project; this page
does not ask you to author a robot subsystem.

**Before this page:** read [How Sushi runs your code](<Framework Overview.md>) so the later
`FtcRobotOpMode` and `configure(...)` names connect to a familiar FTC loop.

## Optional: new to FTC deployment?

Start with FIRST's official guides for
[`configuring the robot hardware`](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/getting_started/getting-started.html)
and
[`building, installing, and running an Android Studio OpMode`](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html).
Those guides are useful before a later hardware run. A Robot Controller, Driver Station, and
deployed OpMode are not prerequisites for this software checkpoint or the first software tour.

## Goal

Prove that the project and TeamCode module build before changing robot configuration or enabling
motion.

**Time:** 15–30 minutes. The first Gradle sync may take longer.

**Prerequisites:**

- Android Studio Ladybug 2024.2 or later;
- Git, or a ZIP download tool, to obtain this repository as a complete project; and
- any network access required for the first Gradle dependency download.

**Files for this checkpoint:**

- public repository [`README.md`](https://github.com/harishv-99/2025-PhoenixPedro#readme) — project
  and FTC SDK setup;
- [First software tour](<First Software Tour.md>) — the next required route after this setup
  checkpoint, with all teaching OpModes still disabled.

**Safety:** Keep the course OpModes `@Disabled` during this lesson. A software build requires no
robot motion.

## 1. Get and open the complete project

If the project is not already on the computer, clone this Sushi repository:

```powershell
git clone https://github.com/harishv-99/2025-PhoenixPedro.git
```

You may instead download a ZIP of that same Sushi repository and extract it. Do not substitute a
clean `FIRST-Tech-Challenge/FtcRobotController` checkout; it contains the upstream FTC SDK but not
Sushi.

Open the resulting `2025-PhoenixPedro` repository root in Android Studio, not the `TeamCode` folder
by itself. Wait for Gradle sync and indexing to finish.

In the Project view, confirm that you can reach:

```text
TeamCode/src/main/java/edu/ftcsushi/fw
TeamCode/src/main/java/edu/ftcsushi/robots
```

Framework code lives under `fw`. Robot-specific code and the course examples live under `robots`.

## 2. Compile TeamCode

Open the Android Studio terminal at the repository root and run:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

This page shows the Windows command used by this repository's team. On macOS or Linux, replace
`.\gradlew.bat` with `./gradlew` and keep the remaining arguments unchanged. The same substitution
applies to other repository commands.

Wait for `BUILD SUCCESSFUL`. Warnings are not the same as compilation failures; read the final
result and the first actual error if the build fails.

## 3. Run the software tests

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest
```

These tests use software or recording stand-ins in place of gamepads, motors, sensors, and
telemetry. That lets the maintained production code run and lets a test inspect the commands it
submitted without a robot. The stand-ins contain no physics model, so they do not prove wiring,
direction, real motion, load, or stopping distance. The tests do not move hardware.

## 4. Find the learning sources

Open [`FtcRobotOpMode`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcRobotOpMode.html>)
and [`RobotProgram`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html>)
from Android Studio or the generated API reference. Every Build recipe links one disabled, compiling
OpMode and labels its GitHub link **Complete source**.

The `@Disabled` annotation keeps a teaching OpMode off the Driver Station menu. An ordinary Sushi
OpMode extends `FtcRobotOpMode` and overrides `configure(RobotProgram program)`. Sushi calls that
method once during FTC INIT so the robot can connect its controls and robot parts; afterward,
`FtcRobotOpMode` supplies the repeated FTC loop and STOP cleanup.

Do not remove `@Disabled` merely to follow the source-based walkthrough. A later physical run must
first review every hardware name, direction, mechanism power, drive scale, and drive brake choice
required by that mode. The checked-in profile is software-valid so it compiles, but its motion
permissions remain false. If a required permission is still false, the example refuses to create
that motion-capable hardware owner and reports what must be reviewed. This is “fail closed”: when
permission is missing, the program refuses motion instead of guessing that it is safe. Defaults are
examples, not proof that the configuration or motion is correct for your robot.

Optional later step: the ready-made hardware tester consoles are separate from the disabled course
examples. Read the [`testing console guide`](<../testing-calibration/README.md>) before any
supervised hardware work.

## 5. Deploy without enabling the course examples

If a Robot Controller is available, use the team's normal Android Studio deployment process to
install the project. Confirm that the Driver Station connects and lists the team's already-enabled
OpModes. The course examples remain absent because they are disabled.

This separates project/deployment problems from hardware-configuration problems.

## Expected checkpoint

- Gradle sync finishes.
- `:TeamCode:compileDebugJavaWithJavac` reports `BUILD SUCCESSFUL`.
- `:TeamCode:testDebugUnitTest` reports `BUILD SUCCESSFUL`.
- Android Studio can navigate to `FtcRobotOpMode`, `RobotProgram`, and the focused example sources.
- No example OpMode has been enabled and no hardware has moved.

## Common problems

**Gradle says the current directory is not a project.**

Open the Sushi repository root—the folder containing `gradlew`, `gradlew.bat`, `settings.gradle`,
and `TeamCode`—and run the command there.

**Java or Gradle uses the wrong JDK.**

Set Android Studio's Gradle JDK to its bundled JDK, then sync again. Run the command from Android
Studio's terminal so it uses the same project environment.

**Dependency resolution fails.**

Confirm the machine can reach the configured repositories, retry Gradle sync, and preserve the
first useful network or repository error rather than repeatedly changing source code.

**Tests pass, so can I assume the robot is safe?**

No. Tests cannot inspect the Robot Controller configuration, wiring, motor polarity, mechanism
travel, or available floor space. Use the testing and calibration runbooks when the team begins
that separate work.

## Continue with the first software tour

With the software baseline green, follow the [First software tour](<First Software Tour.md>) while
the teaching OpModes remain disabled. It introduces live drive values, one button press, and one
timed Auto entirely through maintained software scenarios. After that tour, [choose the next Build
outcome](<../build/README.md>) your robot needs and follow that lesson's separate hardware gate only
when the matching hardware is ready.
