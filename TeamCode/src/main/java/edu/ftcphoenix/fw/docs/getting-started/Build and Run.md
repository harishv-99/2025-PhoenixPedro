# Build and run Phoenix

## Goal

Prove that the project and TeamCode module build before changing robot configuration or enabling
motion.

**Time:** 15–30 minutes. The first Gradle sync may take longer.

**Prerequisites:**

- Android Studio Ladybug 2024.2 or later;
- Git, or a ZIP download tool, to obtain this repository as a complete project; and
- any network access required for the first Gradle dependency download.

**Files for this lesson:**

- public repository [`README.md`](https://github.com/harishv-99/2025-PhoenixPedro#readme) — project
  and FTC SDK setup;
- [`StarterTeleOp.java`](<../../../robots/examples/starter/opmode/StarterTeleOp.java>) — the first course
  OpMode;
- [`StarterAuto.java`](<../../../robots/examples/starter/opmode/StarterAuto.java>) — the later simple Auto.

**Safety:** Keep the starter OpModes `@Disabled` during this lesson. A software build requires no
robot motion.

## 1. Get and open the complete project

If the project is not already on the computer, clone this Phoenix repository:

```powershell
git clone https://github.com/harishv-99/2025-PhoenixPedro.git
```

You may instead download a ZIP of that same Phoenix repository and extract it. Do not substitute a
clean `FIRST-Tech-Challenge/FtcRobotController` checkout; it contains the upstream FTC SDK but not
Phoenix.

Open the resulting `2025-PhoenixPedro` repository root in Android Studio, not the `TeamCode` folder
by itself. Wait for Gradle sync and indexing to finish.

In the Project view, confirm that you can reach:

```text
TeamCode/src/main/java/edu/ftcphoenix/fw
TeamCode/src/main/java/edu/ftcphoenix/robots
```

Framework code lives under `fw`. Robot-specific code and the course examples live under `robots`.

## 2. Compile TeamCode

Open the Android Studio terminal at the repository root and run:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

The course shows the Windows command used by this repository's team. On macOS or Linux, replace
`.\gradlew.bat` with `./gradlew` and keep the remaining arguments unchanged. The same substitution
applies to later lessons.

Wait for `BUILD SUCCESSFUL`. Warnings are not the same as compilation failures; read the final
result and the first actual error if the build fails.

## 3. Run the software tests

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest
```

These tests exercise the managed runtime, starter ownership, Tasks, Plants, and Pedro reference with
software fakes. They do not move hardware.

## 4. Find the course examples

Open the two starter entry files. Both contain `@Disabled`, so they compile but do not appear on the
Driver Station yet:

```java
@TeleOp(name = "FW Starter: TeleOp", group = "FW Examples")
@Disabled
public final class StarterTeleOp extends FtcRobotOpMode {
    // ...
}
```

Do not remove `@Disabled` until the later lessons have reviewed every hardware name, direction,
mechanism power, drive scale, and drive brake choice required by that mode. The checked-in profile
is software-valid so it compiles, but `allowIntakeMotion` and `allowDriveMotion` are both false.
Those defaults are examples, not proof that the configuration or motion is correct for your robot.

The ready-made tester suite is separate from the disabled starter. It has exactly two entries:
**FW: Testers (Driver Station)** for physical-gamepad input and **FW: Testers (Panels)** for
Panels virtual-gamepad input. They run the same testers and controls and show the same telemetry on
both consoles; choose one input owner for the whole run. The next lesson uses their shared **HW:
Actuator Bring-up** wizard to establish one motor's direction at low power before the starter owns
that motor. Read the [`testing console guide`](<../testing-calibration/README.md>) before choosing
the Panels entry.

## 5. Deploy without enabling the starter

If a Robot Controller is available, use the team's normal Android Studio deployment process to
install the project. Confirm that the Driver Station connects and lists the team's already-enabled
OpModes. The starter remains absent because it is disabled.

This separates project/deployment problems from hardware-configuration problems.

If this is your first FTC setup, use FIRST's official guides for
[`configuring the robot hardware`](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/getting_started/getting-started.html)
and
[`building, installing, and running an Android Studio OpMode`](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html).
Phoenix begins after those FTC controller and deployment steps are working.

## Expected checkpoint

- Gradle sync finishes.
- `:TeamCode:compileDebugJavaWithJavac` reports `BUILD SUCCESSFUL`.
- `:TeamCode:testDebugUnitTest` reports `BUILD SUCCESSFUL`.
- Android Studio can navigate to `FtcRobotOpMode`, `RobotProgram`, and the starter examples.
- You can find **FW: Testers (Driver Station)** and **FW: Testers (Panels)** for the next supervised
  hardware lesson.
- No course OpMode has been enabled and no hardware has moved.

## Common problems

**Gradle says the current directory is not a project.**

Open the Phoenix repository root—the folder containing `gradlew`, `gradlew.bat`, `settings.gradle`,
and `TeamCode`—and run the command there.

**Java or Gradle uses the wrong JDK.**

Set Android Studio's Gradle JDK to its bundled JDK, then sync again. Run the command from Android
Studio's terminal so it uses the same project environment.

**Dependency resolution fails.**

Confirm the machine can reach the configured repositories, retry Gradle sync, and preserve the
first useful network or repository error rather than repeatedly changing source code.

**Tests pass, so can I assume the robot is safe?**

No. Tests cannot inspect the Robot Controller configuration, wiring, motor polarity, mechanism
travel, or available floor space. The next lesson treats those as explicit checks.

**Next:** [`Your first mechanism`](<First Mechanism.md>)
