---
tags:
  - Learn
---

# Robot roles

**Question:** Which object should own this state, resource, or decision?

This is an on-demand ownership reference, not a second course. The
[intake](<../../build/Continuous Intake.md>) and
[combined TeleOp](<../../build/Combine Drive and Intake.md>) lessons show the complete connections.
No installation, code changes, test execution, or hardware is needed to read this page.

## Separate construction from later work

Before the code, read the [FTC-loop bridge](<../Framework Overview.md>) for object construction and
the managed entry method. A **profile** holds configuration choices. The **composition root** is
the object that constructs robot parts and connects them; it does not decide their behavior.
The ordinary OpMode chooses the profile and asks that object to connect its owners:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
}
```

`configure(program)` runs during INIT. `StarterRobot` constructs and declares the software graph;
it does not run the control script. The framework-created `RobotProgram` remembers those
declarations and later advances them in the managed lifecycle. Neither the OpMode nor composition
root needs its own loop.

**Key APIs:** [`FtcRobotOpMode`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcRobotOpMode.html>)
receives the FTC lifecycle;
[`RobotProgram`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html>)
owns the declared phases and cleanup. The
[Complete source: `StarterTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java>)
supplies package and imports.

## Who owns what?

**Cached** means saved from the most recent update. A **snapshot** is a fixed record of those
facts. A **capability** exposes the requests and status a robot part offers to both TeleOp and Auto.

| Role | Owns | Does not own |
| --- | --- | --- |
| OpMode | mode selection and one `configure(program)` entry | a private loop or STOP sequence |
| Profile/configuration | device names, directions, bounds, and tuning data | hardware or runtime behavior |
| Composition root | construction, relationships, and declaration order | button meanings or scripted behavior |
| Capability | mode-neutral requests and status | FTC device details |
| Controls | operator meanings | hardware construction or Auto strategy |
| Observation service | its input resource and cached observations | a presenter's polling or an implied action policy |
| Auto routine | fresh capability Tasks and outcome-dependent sequence | final actuator writes |
| Mechanism/output | private Plants, update order, cached status, and stop | gamepad meanings |
| Presenter | formatting already-computed snapshots | decisions or telemetry commits |
| `RobotProgram` | one heartbeat, managed phases, and cleanup | season-specific meanings |

These are responsibilities, not a requirement to create ten classes. A focused sensor can have one
observation owner and a short presenter. An intake exposes only the capability its clients need.
Split another owner when there is a separate responsibility, not to satisfy a package diagram.

The [switch lesson](<../../build/Read a Switch.md>) shows a service publishing an input fact.
The [intake lesson](<../../build/Continuous Intake.md>) adds a command and a private
[Plant](<Plants and Hardware.md#plant>). Reading status in either presenter does not sample
hardware again.

## Locate the boundary in a loop

The ordinary active order is:

```text
Clock -> Services -> Bindings -> Tasks -> Outputs/Drive -> Presenters -> one telemetry commit
```

A clock heartbeat identifies this cycle. Services refresh observations before behavior uses them;
bindings and Tasks express requests; mechanisms realize outputs; presenters display cached facts.
An example declares only the roles it needs. `RobotProgram` updates declared mechanisms;
mechanisms own the order of their own private Plants.

At FTC STOP, the managed host cancels work and stops resource/output owners. The exact failure and
cleanup contract belongs in [Loop Structure](<../../core-concepts/Loop Structure.md>).

## Add larger roles only when needed

Use [Robot Capabilities and Mode Clients](<../../design/Robot Capabilities & Mode Clients.md>) for
larger shared TeleOp/Auto vocabulary, [Framework Lanes and Robot Controls](<../../design/Framework Lanes & Robot Controls.md>)
for detailed ownership, and [Supervisors and Pipelines](<../../design/Supervisors & Pipelines.md>) for
robot-specific coordination. Return to [the concept index](<../Beginner's Guide.md>) for another
question.
