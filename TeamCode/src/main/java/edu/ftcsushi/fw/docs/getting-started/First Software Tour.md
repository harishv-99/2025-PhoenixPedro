---
tags:
  - Get Started
---

# First software tour

After [How Sushi runs your code](<Framework Overview.md>), compare three small kinds of work.
Reading is a complete path: no installation, code changes, test run, or robot is required.
Read each linked **First pass**, predict the result, then compare below. The full Build lesson can wait.

**Optional: run the examples.** First [set up and verify](<Build and Run.md>), then use the command
at each stop. Keep OpModes disabled; no hardware is needed. Commands run reference code, not your code.

## 1. Observe a switch every loop

Open [Read a switch: First pass—observations every loop](<../build/Read a Switch.md#first-pass-observations-every-loop>).

**Predict:** when the switch changes, does the saved reader change, or its next value? Should a
brief pressed reading immediately become an accepted press?

**Expected behavior:** the same reader reports the new raw value on the next active loop. A brief
press is visible as raw input but is rejected by the lesson's filter. Two 0.011-second sampled
pressed intervals exceed its 0.02-second threshold. Displaying cached telemetry does not read
again. INIT reports “not observed,” not “clear.”

Optional software check:

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicsensing.BasicSwitchSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicsensing.BasicSwitchSoftwareScenarioTest
    ```

The test keeps the real sensor adapter, filter, service, presenter, and managed loop. Only the
digital channel, telemetry destination, and time are software stand-ins. A pass proves this sampled
software timeline and lifecycle, not wiring, physical contact, or a universally suitable filter.
See [Complete source: `BasicSwitchSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchSoftwareScenarioTest.java>).

## 2. Run one function for one press

Open [Continuous Intake: First pass—run a function once per press](<../build/Continuous Intake.md#first-pass-run-a-function-once-per-press>).

**Predict:** across four samples—released, pressed, still pressed, released—how many times should
the saved A-button function run? Which named intake request should that accepted press select?

**Expected behavior:** one call selects `COLLECT` when A changes from released to pressed. Holding
or releasing it adds no call. The short saved function finishes in that loop; it does not start
timed work.

Optional controls-only check:

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterFirstLessonTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterFirstLessonTest
    ```

The test keeps the gamepad adapter, controls, and saved rules; a recorder replaces the intake.
A pass proves the callback count and absence of timed work, not a motor command or motion.
See [Complete source: `StarterFirstLessonTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterFirstLessonTest.java>).

## 3. Advance bookmarked work across loops

Open [Run one timed Auto: First pass—work that continues across loops](<../build/Run One Timed Auto.md#first-pass-work-that-continues-across-loops>).

**Predict:** should INIT start collection? What request should be active at START, just before 0.75
seconds, and at 0.75 seconds? If FTC STOP arrives early, should the routine keep running?

**Expected behavior:** INIT does not start collection. START requests `COLLECT`; just before 0.75
seconds it is still selected. At 0.75 seconds the request becomes `STOPPED`. Early FTC STOP cancels
the unfinished action and stops the output. These are requests and software commands, not proof
that a motor moved for an exact physical duration.

Optional timed Auto check:

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.opmode.StarterTimedAutoSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.opmode.StarterTimedAutoSoftwareScenarioTest
    ```

The test keeps the routine factory, declaration, Task, mechanism, and managed loop; software replaces
hardware and time. A pass proves the tested requests, cancellation, and fresh single-use work—not
real timing, motion, or a safe duration. See
[Complete source: `StarterTimedAutoSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java>).

## Completion check

Match each need to one shape before continuing:

| Need | Shape |
| --- | --- |
| “What does the switch report now?” on every loop | continuously sampled value |
| “Run this short function once when A is pressed” | one-press callback |
| “Start now, remember progress, and finish on later loops” | multi-loop Task |

Explain why the first keeps sampling, the second runs once per rise, and the third keeps a bookmark
without a thread or sleep.

You can answer these questions by reading; running the tests is not a graduation requirement.
Each full Build lesson has a separate isolated hardware gate
for a team that owns the matching mechanism and is ready for supervised checks.
Software success does not grant permission to enable motion.

Continue to [Choose a build](<../build/README.md>) for the path from one observation to TeleOp and
basic Auto. It also explains optional small-step authoring in your own robot package. Use
[Learn one Sushi idea](<Beginner's Guide.md>) for a concept question, or the [Guide map](<../README.md>)
to find a later topic.
