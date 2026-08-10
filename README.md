# Phoenix framework

Phoenix is a non-blocking FTC robot framework built into an FTC SDK project. Ordinary robot code
declares mechanisms, controls, Tasks, drive, and telemetry once; the managed runtime owns FTC
lifecycle and loop order.

## Start here

There is one beginner path:

1. Open the canonical [`Phoenix documentation hub`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/README.md>).
2. Read [`Phoenix in five minutes`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/getting-started/Framework Overview.md>).
3. Review the [`beginner course checklist`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/getting-started/Beginner's Guide.md>).
4. Complete the four numbered lessons, beginning with
   [`Build and run Phoenix`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/getting-started/Build and Run.md>).

The compiling [`modern starter robot`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/examples/Modern Starter Robot.md>)
is the ordinary managed TeleOp-and-Auto example. Framework changes follow the
[`Framework Principles`](<TeamCode/src/main/java/edu/ftcphoenix/fw/Framework Principles.md>).

### Quick links after the course

- [`Phoenix Cheat Sheet`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/reference/Phoenix Cheat Sheet.md>)
- [`Common Problems`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/troubleshooting/Common Problems.md>)
- [`Phoenix production robot`](<TeamCode/src/main/java/edu/ftcphoenix/robots/phoenix/README.md>)

## Searchable documentation site

When GitHub Pages is enabled, the documentation workflow publishes the
[generated Phoenix documentation site](https://harishv-99.github.io/2025-PhoenixPedro/) from
`master`. It presents the same checked-in Markdown plus generated framework Javadocs with
responsive navigation and local search. Markdown and Javadocs remain authoritative; never edit
`build/docs-site`.

See the [`Maintainer Notes`](<TeamCode/src/main/java/edu/ftcphoenix/fw/docs/maintainers/Maintainer Notes.md>)
for local preview, the strict combined-site build, and the renderer-upgrade procedure. Ordinary
robot builds do not require Python or Zensical.

## Get this Phoenix project

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

Those upstream download links create a clean FTC SDK project without Phoenix. Use the Phoenix clone
instructions above when following this repository's course.
