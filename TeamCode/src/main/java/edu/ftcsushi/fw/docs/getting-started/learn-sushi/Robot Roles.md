# Robot roles

**Learning mode:** Architecture reference

The excerpts explain ownership; use the Starter
buildable module for all files needed to author the graph.

**Question:** Where does each part of robot code belong?

Sushi assigns each decision and resource one owner. You can learn that structure by reading the
Starter robot; no hardware setup or code changes are required.

## Start with the OpMode

### Critical code

[`StarterTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java>) contains the
ordinary entry point:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
}
```

**What to notice**

- The OpMode chooses the profile and mode; it does not become the composition root.
- `StarterRobot` wires owners, while `RobotProgram` advances them.

**Key APIs**

- `FtcRobotOpMode.configure(...)` — the one ordinary FTC composition entry.
- `RobotProgram` — managed declarations, heartbeat, telemetry commit, and cleanup.

The OpMode chooses the profile and mode, then delegates **code composition**. This is different
from electrical wiring: the profile names configured FTC devices, while
[`StarterRobot`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java>) constructs the
software owners and declares their relationships once.

```text
OpMode -> profile + composition root
                       |-- intake output
                       |-- control bindings
                       |-- drive source + sink
                       `-- presenter

RobotProgram later advances those declared roles in the managed lifecycle.
```

The composition root connects the object graph; it is not the robot's control script. After
`configure(program)` returns, `RobotProgram` owns the clock, recurring phases, telemetry commit,
and cleanup.

## Who owns what?

| Role | Owns | Does not own |
| --- | --- | --- |
| OpMode | Mode selection and one `configure(program)` entry | A private loop or STOP sequence |
| Profile/configuration | Data such as device names, directions, bounds, and tuning | Hardware or runtime behavior |
| Composition root | Construction, relationships, and declaration order | Button meanings or scripted behavior |
| Capability | Mode-neutral requests and status | FTC device details |
| Controls | Operator meanings | Hardware construction or Auto strategy |
| Auto routine | Fresh capability Tasks and outcome-dependent sequence | Final actuator writes |
| Mechanism/output | Private Plants, final targets, update order, cached status, and stop | Gamepad meanings |
| Presenter | Formatting already-computed snapshots | Decisions or telemetry commits |
| `RobotProgram` | One heartbeat, managed roles, and cleanup | Season-specific meanings |

A **Plant** is the mechanism-owned object that resolves a requested actuator target and performs
the final hardware write. [Plants and hardware](<Plants and Hardware.md>) follows that path.

These are ownership boundaries, not a reason to create extra classes. The Starter exposes one
[`StarterIntake`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>)
capability directly because that is all its clients need.

## How the pattern scales

The [`ReferenceRobot`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/robot/ReferenceRobot.java>) adds lift
and launcher capability families without changing the roles. TeleOp controls and
[`ReferenceAutoRoutines`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/autonomous/ReferenceAutoRoutines.java>)
call the same mode-neutral capabilities; their mechanisms still privately own hardware realization.
Its multi-family [`ReferenceCapabilities`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/robot/ReferenceCapabilities.java>)
handoff is useful to that Auto routine, but it is not a registry every robot must copy.

## Check your understanding

**A team changes which button requests LOW. What changes?** The controls owner; LOW remains the
same capability meaning.

**Who calls a declared mechanism's `update(clock)` each active cycle?** `RobotProgram`, not the
OpMode or composition root.

**Where does the update order for two private Plants live?** In the one mechanism that owns them.

## Go deeper when needed

- [Controls and intent](<Controls and Intent.md>) — operator meanings and drive intent
- [Robot capabilities and mode clients](<../../design/Robot Capabilities & Mode Clients.md>) —
  larger TeleOp/Auto designs
- [Framework lanes and robot controls](<../../design/Framework Lanes & Robot Controls.md>) —
  detailed ownership rules
- [Learn Sushi topic guide](<../Beginner's Guide.md>) — choose another topic
