# Sushi framework

Sushi is an FTC robot framework built into an FTC SDK project. It organizes the same repeated work
you already know from `while (opModeIsActive())` or `loop()`: read controls, advance unfinished
actions, update hardware, show telemetry, and clean up at STOP.

## Start here

Follow one source-based beginner path:

1. Read [`How Sushi runs your code`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/Framework Overview.md>)
   to connect Sushi to a familiar FTC loop and learn when saved functions run.
2. Use [`Set up and verify the Sushi project`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/Build and Run.md>)
   for project setup and software verification.
3. Take the required [`First software tour`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/First Software Tour.md>)
   without enabling hardware.
4. Open the complete [`Guide map`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/README.md>) and choose
   the robot outcome or deeper explanation you need.

The map routes to the focused [`Build recipes`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/build/README.md>).
Drive is independent; actuator knowledge builds from intake toward claw, lift, and velocity.

Each Build lesson links the exact compiling mechanism, OpMode, and software scenario that proves
its current contract. Framework changes follow the
[`Framework Principles`](<TeamCode/src/main/java/edu/ftcsushi/fw/Framework Principles.md>).

### Later reference shortcuts

- [`Sushi Cheat Sheet`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/reference/Sushi Cheat Sheet.md>)
- [`Common Problems`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/troubleshooting/Common Problems.md>)

## Searchable documentation site

When GitHub Pages is enabled, the documentation workflow publishes the
[generated Sushi documentation site](https://harishv-99.github.io/2025-PhoenixPedro/) from
`master`. It presents the same checked-in Markdown plus generated framework Javadocs with
responsive navigation and local search. Markdown and Javadocs remain authoritative; never edit
`build/docs-site`.

See the [`Maintainer Notes`](<TeamCode/src/main/java/edu/ftcsushi/fw/docs/maintainers/Maintainer Notes.md>)
for local preview, the strict combined-site build, and the renderer-upgrade procedure. Ordinary
robot builds do not require Python or Zensical.

## Get this Sushi project

Clone this repository:

```powershell
git clone https://github.com/harishv-99/2025-PhoenixPedro.git
```

Open the resulting `2025-PhoenixPedro` repository root in Android Studio—the folder containing
`gradlew`, `settings.gradle`, and `TeamCode`. Do not open `TeamCode` by itself. A ZIP download of
this same repository also works when Git is unavailable.

Build and test TeamCode from the repository root:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest
```

On macOS or Linux, use `./gradlew` in place of `.\gradlew.bat`.

## FTC SDK foundation

This repository retains the official FTC Robot Controller project structure and dependencies for
the DECODE (2025–2026) season. Android Studio Ladybug 2024.2 or later is required by this SDK
baseline.

Use FIRST's official documentation for control-system setup, Robot Controller configuration, SDK
samples, and platform release information:

- [FTC documentation](https://ftc-docs.firstinspires.org/)
- [FTC Android Studio tutorial](https://ftc-docs.firstinspires.org/en/latest/programming_resources/android_studio_java/Android-Studio-Tutorial.html)
- [Official FTC Robot Controller repository](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- [Official FTC SDK releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases)

Those upstream download links create a clean FTC SDK project without Sushi. Use the Sushi clone
instructions above when following this repository's guides.
