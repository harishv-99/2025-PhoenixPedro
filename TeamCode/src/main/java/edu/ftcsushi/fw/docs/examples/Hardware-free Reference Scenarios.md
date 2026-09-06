---
tags:
  - Test & Tune
---

# Choose a hardware-free scenario by question

A scenario is useful only when its boundary matches the question. Keep the production owner of
that question, replace the external world it observes or commands, advance the normal heartbeat,
and state what the resulting evidence cannot establish.

**Before this page:** complete [Build and Run](<../getting-started/Build and Run.md>) so the repository
builds and the Android Studio terminal is open at its root.

This page explains those boundaries beside the first experiment. Read
[How to test a Sushi component](<../testing-calibration/How to test a Sushi component.md>) afterward
when you want to design another scenario.

## Start here: ask for intake, then advance one heartbeat

`StarterMechanismLessonTest` is the maintained first software experiment. It uses the real starter
intake mechanism and its real Plant, but records the motor command in software instead of requiring
a robot. Run it from the repository root:

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterMechanismLessonTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterMechanismLessonTest
    ```

A passing test answers one deliberately small question:

- **Question:** Does requesting `COLLECT` reach the configured motor power on the mechanism's next
  normal output update, rather than writing hardware immediately?
- **Keep real:** the production `StarterIntakeMechanism`, its configuration, named-mode mapping, and
  Plant update path. These are the owners whose behavior the question asks about.
- **Replace:** only the FTC motor with a recording `MotorProbe`; a manual clock supplies a
  deterministic cycle. Neither replacement contains motor physics.
- **Observe:** the requested mode, zero motor writes before the update, then cached and recorded
  power after the update.
- **Cannot conclude:** the real motor's direction, motion, clearance, current, behavior under load,
  or stopping time.

In the excerpt, **ARRANGE** constructs the production owner around the recorder, **REQUEST** changes
the named intent, and **HEARTBEAT** runs the normal output update once. The `assertEquals(...)`
lines are the **ASSERT** step: before the heartbeat they require zero applied power and zero writes;
after it they require the configured `collectPower` at both software observations.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterMechanismLessonTest.java -->
```java
// ARRANGE: production configuration and mechanism with only the FTC motor replaced.
StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
config.collectPower = 0.37;
config.ejectPower = -0.22;

FtcTestHardware hardware = new FtcTestHardware();
FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
StarterIntakeMechanism intake = new StarterIntakeMechanism(hardware, config);
ManualLoopClock time = new ManualLoopClock();

// REQUEST: semantic intent changes immediately; hardware has not been updated yet.
intake.setMode(StarterIntake.Mode.COLLECT);
assertEquals(StarterIntake.Mode.COLLECT, intake.status().mode());
assertEquals(0.0, intake.status().appliedPower(), 0.0);
assertEquals(0, motor.powerWrites());

// HEARTBEAT: the same production update path maps the request and writes the motor.
intake.update(time.clock());
assertEquals(config.collectPower, intake.status().appliedPower(), 0.0);
assertEquals(config.collectPower, motor.power(), 0.0);
```

**Read the causal chain:** `setMode(COLLECT)` changes the named request but produces no motor write;
the one explicit `update(...)` then resolves that request and records `0.37` through the production
Plant path.

**Proves:** the configured semantic mapping, requested-versus-applied order, and submitted software
command agree in this scenario.

**Does not prove:** the physical motor moved, moved in the intended direction, or is safe.

**Next gate:** learn how to choose one input owner and stop safely in
[Using the tester console](<../testing-calibration/Using the Tester Console.md>), then use
[actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) for a supervised low-power
direction check. Reading the
[Complete source: `StarterMechanismLessonTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterMechanismLessonTest.java>)
is optional; the experiment, boundary, command, and evidence are all stated here.

## Choose the next scenario by question

The starter intake above is the beginner entry. The matrix keeps later choices together; entries
marked **Advanced** assume the ordinary one-owner and one-heartbeat path is already familiar.

| Question | Production owner kept real | Focused scenario | Physical gate still required |
|---|---|---|---|
| Do stick axes keep Sushi's coordinate signs? | `GamepadDevice` + `GamepadDriveSource` | [First drive](<../build/First Drive.md#software-checkpoint-sticks-have-one-coordinate-meaning>) | motor direction and motion |
| Does named intake intent reach one motor command? | `StarterIntakeMechanism` + Plant | [Continuous intake](<../build/Continuous Intake.md#software-checkpoint-request-first-apply-on-heartbeat>) | intake direction, load, and stop |
| Can one managed TeleOp serve continuous drive and callback-driven intake? | `StarterRobot` + production controls/outputs | [Combine drive and intake](<../build/Combine Drive and Intake.md#software-checkpoint-one-managed-cycle-serves-both-outcomes>) | safe simultaneous motion and STOP |
| Do named claw positions map through configured endpoint candidates? | `BasicClawMechanism` + servo Plant | [Named claw](<../build/Named Claw.md#software-checkpoint-normalized-half-derives-from-two-endpoints>) | linkage clearance and arrival |
| Does debounced switch evidence establish encoder zero? | `BasicLiftMechanism` + calibration Task + Plant | [Lift reference](<../build/Referenced Lift.md#software-checkpoint-authored-switch-evidence-controls-homing>) | switch placement, scale, direction, and homing travel |
| Does fresh encoder evidence complete one selected height request? | `BasicLiftMechanism` + semantic Task + Plant | [Referenced lift move](<../build/Move a Referenced Lift.md#software-checkpoint-fresh-encoder-evidence-completes-the-move>) | conversion, tuning, loaded stability, and safe travel |
| Does cancelling one velocity Task request zero through the normal heartbeat? | `BasicFlywheelMechanism` + scalar Task + Plant | [Single flywheel velocity](<../build/Single Flywheel Velocity.md>) | direction, encoder scale, PIDF tuning, loaded speed, and coast-down |
| Does one timed root begin at START and select safe intent on completion/cancel? | `StarterAuto` + production intake Task/Plant | [Run one timed Auto](<../build/Run One Timed Auto.md#software-checkpoint-time-begins-at-start>) | duration under load, direction, and physical stop |
| Does successful prerequisite evidence admit Auto work in the authored order? | `BasicAutoRoutines` + Task graph | [First autonomous](<../build/First Autonomous.md#software-checkpoint-success-admits-the-next-work>) | homing/move timing, lift clearance, and STOP |
| Must both flywheels be ready? | `ReferenceFlywheelMechanism` + grouped Plant | **Advanced:** [paired flywheel](<../advanced/Paired Flywheel Velocity.md>) | balance under load and tuning |
| Which full-turn turret equivalent is legal and nearest? | `ReferencePeriodicTurretMechanism` + resolver | **Advanced:** [periodic turret](<../advanced/Periodic Turret Position.md>) | zero, cable bounds, and collision |
| Is sensor-derived inventory published only after update? | `ReferenceInventoryStatusService` | [Complete source: `ReferenceInventorySoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/inventory/ReferenceInventorySoftwareScenarioTest.java>) | sensor placement and game-piece detection |
| Does one retained Pedro execution classify honestly? | `RouteTask` + route boundary | [First Pedro Auto](<../build/First Pedro Auto.md#software-checkpoint-completion-needs-endpoint-evidence>) | localization, route accuracy, and stop |

The linked tests are small teaching scenarios. The First Autonomous scenario keeps the real
lift-only routine and uses only framework-built recording Tasks to show that home admits HIGH and
HIGH admits STOWED only on exact success; its second method makes timeout and cancellation suppress
that later request. Its neighboring
[`BasicAutoRoutinesTest`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutinesTest.java>)
is supplied maintainer evidence for the optional lift-and-claw parallel capstone and broader
fresh-Task behavior. Other broad contract tests beside teaching scenarios serve the same role:
students may run them before they are ready to author complete abnormal-outcome matrices.

## Advanced worked example: paired velocity needs paired evidence

This Reference example is useful after the starter experiment; it combines a grouped Plant with
independent per-motor readiness evidence and is not the beginner starting point.

- **Question:** Can an average velocity hide one slow wheel and one fast wheel?
- **Keep real:** the production mechanism, grouped velocity Plant, status publication, Task, and
  loop order.
- **Replace:** only the two FTC motors with recording devices whose measurements the test controls.
- **Observe:** the grouped Plant fact, two independent measurements, `ready()`, and Task outcome.
- **Cannot conclude:** motor direction, physical balance under load, tuning, release, or scoring.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelSoftwareScenarioTest.java -->
```java
// HEARTBEAT: the production owner writes both motors and publishes one complete status.
scenario.flywheels.update(scenario.time.clock());
ReferenceFlywheels.Status unbalanced = scenario.flywheels.status();
assertEquals(1000.0, scenario.left.commandedVelocityTicksPerSec(), EPSILON);
assertTrue("the grouped mean is at target", unbalanced.plantSnapshot().atCommandTarget());
assertFalse("independent member evidence prevents a false ready claim", unbalanced.ready());

// REQUEST: every factory call creates a fresh single-use request-and-wait Task.
Task waitForBoth = scenario.flywheels.setVelocityTask(1000.0, 1.0);
assertNotSame(waitForBoth, scenario.flywheels.setVelocityTask(1000.0, 1.0));
```

**Read the causal chain:** the test gives one wheel `800` ticks/s and the other `1200` while the
request is `1000`; their grouped mean looks correct, but the production status retains both member
errors and refuses readiness.

**Proves:** one grouped command does not erase the independent feedback needed by the capability's
readiness claim, and every wait request creates a fresh Task.

**Does not prove:** either physical wheel is stable, balanced, safe, or capable of launching a game
piece.

**Next gate:** run the focused mechanism at a guarded low velocity, observe both real encoders under
representative load, and tune only after directions and immediate STOP are established.

## Running another scenario

Use the fully qualified test class shown by its page or source link:

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheelSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheelSoftwareScenarioTest
    ```

Then write down the causal chain in one sentence and the physical fact that must be checked next.
If either answer is unclear, narrow the test question before adding more fake behavior.
