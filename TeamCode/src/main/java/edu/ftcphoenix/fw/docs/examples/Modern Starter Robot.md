# Modern starter robot

Use this example after the flat Beginner's Guide skeleton when you want the smallest compiling
Phoenix robot structure shared by TeleOp and Auto. It has one direct mecanum drive, one intake
mechanism, one mode-neutral intake capability, one controls owner, and one composition root. It has
no vision, localization, route library, season strategy, presenter, supervisor, service, base
OpMode, generator, or extra capability aggregate.

All seven files live in
[`edu.ftcphoenix.robots.examples.starter`](<../../../robots/examples/starter/>). They are robot code,
not new framework APIs.

## Read the seven files

Read them in this order:

| File | Job | What a student normally edits |
|---|---|---|
| [`StarterProfile.java`](<../../../robots/examples/starter/StarterProfile.java>) | Data-only, defensively copied hardware and tuning configuration | Fill the deliberately blocked physical configuration in `current()` only after checking the adopting robot. |
| [`StarterIntake.java`](<../../../robots/examples/starter/StarterIntake.java>) | The one mode-neutral capability family and its small status vocabulary | Rename the capability and modes when the robot's mechanism meaning differs. |
| [`StarterIntakeMechanism.java`](<../../../robots/examples/starter/StarterIntakeMechanism.java>) | Owns the intake Plant, final target resolver, update order, bounded intake Task, and stop command | Change the Plant realization when the mechanism hardware or safe behavior changes. |
| [`StarterTeleOpControls.java`](<../../../robots/examples/starter/StarterTeleOpControls.java>) | Owns button meanings, bindings, slow mode, and the final manual `DriveSource` | Change driver/operator meanings here, not in framework primitives or the robot root. |
| [`StarterRobot.java`](<../../../robots/examples/starter/StarterRobot.java>) | Composition root for validation, construction, one clock, mode lifecycle, loop order, telemetry commit, Task runner, and cleanup | Add a new owner only when the robot gains a real new capability or lifecycle need. |
| [`StarterTeleOp.java`](<../../../robots/examples/starter/StarterTeleOp.java>) | Disabled thin FTC TeleOp client | Normally change only the Driver Station name and selected profile. |
| [`StarterAuto.java`](<../../../robots/examples/starter/StarterAuto.java>) | Disabled tiny Auto client using one fresh bounded intake Task | Replace the one routine expression with the adopting robot's small Task composition. |

The seven-file split is intentional. A single large OpMode would collapse the boundaries this
example exists to teach. A generic base robot, template, or generator would hide or duplicate the
same ownership before repeated robot use proves that another abstraction removes real work.

## Configure before constructing hardware

`StarterProfile.current()` is deliberately not runnable as checked in. Its physical fields remain
unset or use sentinel values, and `hardwareConfigurationReviewed` remains false. Before the
disabled TeleOp is enabled, edit that one profile method with the adopting robot's reviewed:

- four configured drive motor names and directions,
- intake motor name and direction,
- finite, nonzero, distinct collect and eject powers in `[-1.0, 1.0]`, and
- `hardwareConfigurationReviewed = true` acknowledgement.

The exact edit locations inside `current()` are:

| Configuration fact | Profile field |
|---|---|
| Drive motor names | `drive.wiring.frontLeftName`, `frontRightName`, `backLeftName`, `backRightName` |
| Drive motor directions | `drive.wiring.frontLeftDirection`, `frontRightDirection`, `backLeftDirection`, `backRightDirection` |
| Intake motor identity | `intake.motorName`, `intake.direction` |
| Intake action powers | `intake.collectPower`, `intake.ejectPower` |
| Human review acknowledgement | `hardwareConfigurationReviewed` |

The tiny Auto constructs only the shared intake, so its validation does not require drive values.
It still requires the reviewed intake name, direction, powers, and acknowledgement. TeleOp requires
both the complete drive configuration and the shared intake configuration.

`StarterRobot` copies and validates the complete mode-required profile before its first
`HardwareMap` lookup or output. An incomplete profile therefore produces one actionable
configuration error instead of partially acquiring hardware. Setting the acknowledgement records
that a human reviewed the configuration; it does not prove wiring, direction, power safety,
traction, or mechanical motion.

Keep configuration facts in `StarterProfile`. Keep button meanings, Task ordering, and update order
in code. Do not copy Phoenix season values into another robot or treat compilation as hardware
validation.

## The shared capability call path

`StarterIntake` is the public, mode-neutral vocabulary. The Plant stays private to
`StarterIntakeMechanism`, and neither mode client writes a target directly.

TeleOp follows this path:

```text
StarterTeleOp
  -> StarterRobot.initTeleOp(...)
  -> StarterTeleOpControls bindings
  -> StarterIntake.setMode(...)
  -> StarterIntakeMechanism
  -> intake Plant
```

The ordinary control calls remain semantic:

```java
bindings.onRise(
        driver.a(),
        () -> intake.setMode(StarterIntake.Mode.COLLECT)
);
bindings.onRise(
        driver.b(),
        () -> intake.setMode(StarterIntake.Mode.EJECT)
);
bindings.onRise(
        driver.x(),
        () -> intake.setMode(StarterIntake.Mode.STOPPED)
);
```

Auto uses that same capability object rather than reaching into the mechanism:

```java
robot.initAuto();
robot.installAutoRoutine(robot.intake().collectForSeconds(0.75));
```

`collectForSeconds(...)` returns a fresh single-use Task. Its normal completion and active
cancellation both restore the stopped request, while the root continues updating the downstream
Plant once per Auto loop. A repeated run asks the capability for another fresh Task.

There is deliberately no one-member `StarterCapabilities` aggregate. `StarterIntake` already gives
both clients the one cohesive family they need. Add an aggregate when a second independent family
creates useful grouping, not merely to mirror a larger robot.

## Thin mode clients

Both OpModes only choose configuration, construct the root, choose a mode, and forward lifecycle.
Their INIT calls are direct:

```java
robot = new StarterRobot(hardwareMap, telemetry, StarterProfile.current());
robot.initTeleOp(gamepad1);           // TeleOp uses one driver
// or:
robot.initAuto();                     // Auto
```

The starter has one driver, so it does not wrap or pass `gamepad2`. Add a second
`GamepadDevice` only when the robot actually has second-operator meanings.

The remaining lifecycle is equally direct:

```text
INIT   -> construct StarterRobot, then initTeleOp(...) or initAuto()
START  -> robot.start(getRuntime())
loop   -> robot.update(getRuntime())
STOP   -> robot.stop()
```

`StarterAuto` additionally installs its fresh root Task during INIT. It does not add a separate
routine class for one expression. Introduce a robot-owned Auto plan/routine when path selection,
outcome policy, or several named routines make that owner useful.

## Loop and telemetry ownership

The composition root makes the two active-loop orders visible:

```text
TeleOp: clock -> controls/bindings -> direct drive write -> intake Plant -> status rows -> commit
Auto:   clock -> TaskRunner -> intake Plant -> status rows -> commit
```

The root advances its one `LoopClock` once per active cycle. Controls own manual drive shaping; the
root performs the final direct-drive write. The intake mechanism remains the only Plant target and
update owner. A Task changes the retained capability request before the downstream Plant phase, so
each positive-duration command is observable.

The root writes `intake.mode` and `intake.appliedTargetPower`; Auto adds `auto.idle`. The applied
target is the Plant's cached final target after guards, not motor or mechanism readback. The root
then calls `Telemetry.update()` once for the complete active-loop frame. It has no presenter because
these few direct rows do not justify another owner. Extract an additive presenter when several
callers need the same formatted snapshots or the required page becomes substantial. Keep optional
`debugDump(...)` calls explicitly selected at the root; do not dump every object every frame or use
a presenter to resample live state.

## Cleanup ownership

`StarterRobot.stop()` is idempotent and best-effort attempts every owner it constructed:

1. cancel the current Auto Task and clear pending work when the runner exists,
2. request zero and stop the intake Plant/output, and
3. stop the direct drive owner when TeleOp constructed it.

A root-owned configuration or construction failure, start failure, or active-loop failure enters
the same fail-stop path, preserving the original failure and attaching later cleanup failures.
Programming precondition errors such as installing a second Auto routine are rejected without
changing an otherwise valid lifecycle. The OpModes do not duplicate these stops or expose a
drop-without-cancel runner operation. A normally returning stop establishes an immediate zero
submission at the output seam; physical motion must be checked on the robot.

## Adapting the example

For an ordinary new robot:

1. complete and physically review `StarterProfile.current()`;
2. rename `StarterIntake` and its modes to the robot's real capability vocabulary;
3. keep final target resolvers, Plant construction, and Plant update order in the mechanism owner;
4. map gamepads only in `StarterTeleOpControls`;
5. let both TeleOp and Auto call the same capability;
6. keep new policy in a supervisor/service only when that policy actually exists; and
7. keep the root's construction, loop phases, telemetry commit, and cleanup explicit.

The two OpModes remain `@Disabled` until the adopting team verifies the intake name, direction,
polarity and power, clear space, and immediate STOP behavior on the actual robot. Before enabling
TeleOp, also verify all four drive names and directions with the wheels safely raised. Software
tests can prove ownership and cancellation semantics, but not those physical facts.

## Related reading

- [`../getting-started/Beginner's Guide.md`](<../getting-started/Beginner's Guide.md>)
- [`../design/Framework Lanes & Robot Controls.md`](<../design/Framework Lanes & Robot Controls.md>)
- [`../design/Robot Capabilities & Mode Clients.md`](<../design/Robot Capabilities & Mode Clients.md>)
- [`../core-concepts/Loop Structure.md`](<../core-concepts/Loop Structure.md>)
- [`Examples Progression & Layered Mechanisms.md`](<Examples Progression & Layered Mechanisms.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
