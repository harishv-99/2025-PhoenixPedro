# Plants and hardware

**Question this chapter answers:** How does a semantic request become one final hardware command,
and what evidence can robot code truthfully report?

**Reading time:** about 20 minutes

A mechanism owns the whole realization path for its actuators. It accepts capability intent,
retains private Plants, updates them in the managed output phase, publishes cached status, and
terminally stops them during cleanup. This chapter explains that path from source; it requires no
example hardware or physical motion.

A **Plant** is the mechanism-owned object that resolves one requested actuator target, applies its
bounds and feedback rules, and performs the final hardware write. Students request robot behavior;
the mechanism's Plant owns how that request reaches the actuator.

## Source map

| Read | Look for |
| --- | --- |
| [`StarterIntakeMechanism.java`](<../../../../robots/examples/starter/capability/intake/StarterIntakeMechanism.java>) | The smallest command-backed direct-power Plant |
| [`StarterIntake.java`](<../../../../robots/examples/starter/capability/intake/StarterIntake.java>) | A truthful applied-target status field |
| [`ReferenceLiftMechanism.java`](<../../../../robots/examples/reference/capability/lift/ReferenceLiftMechanism.java>) | Bounded position, units, reference state, feedback, and a cached snapshot |
| [`ReferenceLauncherMechanism.java`](<../../../../robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.java>) | Paired velocity actuation, several owned Plants, and readiness evidence |
| [`FTC Actuators and Plants`](<../../ftc-boundary/FTC Actuators & Plants.md>) | The complete staged construction and lifecycle contract |

## The one realization path

The Starter intake is the smallest useful trace:

```text
StarterIntake.setMode(COLLECT)          semantic request
                 |
                 v
StarterIntakeMechanism                 sole actuator owner
                 |
                 v
Plant command target                   persistent requested power
                 |
                 v
final target resolver + Plant bounds   applied target
                 |
                 v
Plant.update(shared clock)              one hardware write path
```

There is no second writer for an emergency, a Task, or a button. Temporary and guarded behavior
must compose into the Plant's final target resolver, and the Plant remains the only object that
writes its actuator.

## The smallest Plant

`StarterIntakeMechanism` receives a `HardwareMap` and data-only configuration. After copying and
validating every retained field, it constructs its private Plant:

```java
plant = FtcActuators.plant(
                Objects.requireNonNull(hardwareMap, "hardwareMap is required")
        )
        .motor(motorName, direction)
        .power()
        .targetFromNewCommand(STOPPED_POWER)
        .build();
collectPower = copiedCollectPower;
ejectPower = copiedEjectPower;
```

Read the builder as a series of answered questions:

1. Which FTC resource owns the hardware lookup?
2. Which motor and logical direction are used?
3. Is the public target power, position, or velocity?
4. Where does the final target come from?

Direct power Plants always own normalized target range `[-1, +1]`. The initial command target is
zero. Constructing the graph does not itself command motion; the managed output heartbeat later
realizes the target.

The capability method does not write hardware. `setMode(...)` converts a semantic mode to one
persistent request and writes the Plant's command target. On the next output phase, `plant.update`
resolves and applies that request.

The Starter reports exactly what it knows:

```java
@Override
public Status status() {
    double requestedPower = plant.commandTarget().get();
    return new Status(
            modeFor(requestedPower),
            plant.getAppliedTarget());
}
```

`appliedTargetPower` is the Plant's cached final target after resolution and guards. It is **not**
motor power readback and does not prove that the mechanism moved.

## Request, applied target, measurement, result

These terms answer different questions:

| Evidence | Question answered | Example |
| --- | --- | --- |
| Semantic request | What did the client ask the robot to do? | `StarterIntake.Mode.COLLECT` or `ReferenceLift.Height.HIGH` |
| Requested target | What numeric value did the command source request? | lift command target in inches |
| Applied target | What final target survived resolution, bounds, and guards this cycle? | `plant.getAppliedTarget()` |
| Measurement | What feedback did the Plant cache from its sensor/controller path? | lift position or flywheel velocity |
| `atTarget` / ready | Does cached controller evidence meet its configured tolerance? | launcher velocity within tolerance |
| Physical result | Did the game piece move or score successfully? | operator observation or separate sensor evidence |

Do not rename one kind of evidence to imply another. A target is not a measurement, and controller
readiness is not proof of a successful game action.

## Position adds coordinates and reference

The Reference lift uses inches as its public coordinate while the motor still uses encoder ticks.
Its Plant construction keeps all coordinate and control decisions together:

```java
lift = FtcActuators.plant(map)
        .motor(c.motorName, c.direction)
        .position()
        .deviceManaged()
        .nonPeriodic()
        .bounded(0.0, c.maximumHeightIn)
        .scaleToNative(c.ticksPerIn)
        .needsReference("reference lift has not been homed")
        .positionTolerance(c.toleranceIn)
        .outputPowerLimitedTo(c.maximumPower)
        .targetFromNewCommand(c.stowedHeightIn)
        .build();
```

That graph states:

- public positions are non-periodic inches in `[0, maximumHeightIn]`;
- the adapter maps inches to native encoder ticks;
- position meaning is unavailable until a homing Task establishes a reference;
- target tolerance is expressed in public inches; and
- the normal device-managed position output has an explicit power limit.

`needsReference(...)` does not home the mechanism and does not invent a zero. The separate
non-blocking `home()` Task searches for the bottom switch, establishes position `0.0`, and then
holds the stowed target. Behavior over time belongs in the next chapter; the Plant continues to own
the coordinate and final output.

After its Plant update, the mechanism publishes one cached snapshot:

```java
@Override
public void update(LoopClock clock) {
    lift.update(clock);
    lastStatus = new Status(
            requestedHeight,
            lift.commandTarget().get(),
            lift.getMeasurement(),
            lift.isReferenced(),
            lift.atTarget());
}
```

The order matters. Presenters and higher-level clients read `lastStatus`; they do not resample the
encoder or advance the Plant themselves.

## One mechanism may own several Plants

The Reference launcher owns a paired-motor velocity Plant, independent evidence for each wheel, a
transfer Plant, a release-servo Plant, an object sensor, and a queue that temporarily overlays
transfer power. Those objects form one cohesive mechanism because their ordering and cleanup are
coupled. The grouped Plant remains the one atomic command and stop owner; its public status adds the
per-wheel evidence needed to judge both wheels truthfully.

Its update order is:

```text
advance temporary transfer request
    -> update the grouped flywheel Plant
    -> update transfer and release Plants
    -> observe owned sensor/controller evidence
    -> replace one immutable Status snapshot
```

The transfer queue updates before the transfer Plant samples its final resolver. The flywheel Plant
updates before the target and two same-cycle wheel measurements are copied into status. This is one
heartbeat with an explicit within-owner order, not several independent actuator loops. How those
private observations are acquired is mechanism implementation detail, not another student-facing
command path.

Each wheel must have a finite measurement within the copied ticks-per-second tolerance. Aggregate
`ready` is true only when the requested target is positive and both `leftAtTarget` and
`rightAtTarget` are true. It is therefore false while the launcher is idle at zero; equal and
opposite wheel errors cannot disappear into an average. The object sensor is separate evidence.
None of those facts proves that an object moved or entered a field goal.

## Active idle is not terminal stop

A mechanism commonly needs two different endings:

- **Active-match abort/idle cleanup** changes commands to configured reusable targets. The Reference
  launcher invalidates older launch work, requests zero flywheel velocity and the retracted release
  position, and clears temporary transfer work. A following normal output phase realizes those
  requests.
- **Terminal stop** calls `Plant.stop()` during total cleanup. A stopped Plant instance is not
  restarted; a later OpMode constructs a new graph.

The mechanism owns both meanings because it knows every private Plant and queue. `RobotProgram`
calls the declared output's `stop()` during cleanup. Controls and presenters never perform partial
hardware shutdown. At FTC STOP, cancellation may change persistent requests, but terminal output
cleanup follows and no later normal Plant update is promised. In particular, a position/servo
Plant performs its realization's natural stop; do not claim that STOP physically applies a newly
requested retracted position unless the real hardware path proves that behavior.

## Trace it

### 1. Where should a team add a maximum lift power?

**Answer:** to the lift's data-only configuration and Plant construction, as the Reference lift
does with `outputPowerLimitedTo(c.maximumPower)`. It does not belong in a button callback.

### 2. Which object should read the lift encoder for its ordinary runtime status?

**Answer:** the lift Plant through its configured feedback path. The mechanism copies the cached
measurement after `lift.update(clock)`; presenters consume the snapshot.

### 3. A temporary feed pulse and a persistent zero target both affect one transfer servo. Should two owners write it?

**Answer:** no. Compose the temporary request into the one final target resolver, then let the one
transfer Plant perform the final write. The Reference launcher uses an overlay for that purpose.

## Predict it

### `StarterIntake.Status.appliedTargetPower()` reports `0.20`. Does that prove the motor turned?

**Prediction:** no.

**Answer:** it proves the Plant cached `0.20` as its final applied target. Direct-power hardware
readback and physical motion are different evidence and are not claimed by this status.

### Both launcher measurements are within tolerance at a positive target, so `ready` is true. Did a game piece score?

**Prediction:** that cannot be concluded.

**Answer:** readiness is computed flywheel-controller evidence. Scoring is an external result that
an operator may observe or another explicitly owned sensor may report.

### Who advances the transfer queue before the transfer Plant samples it?

**Prediction:** the launcher mechanism.

**Answer:** `ReferenceLauncherMechanism.update(clock)` owns both operations and their order. The
composition root declares that mechanism as one output; it does not reach inside the owner.

## Copy, adapt, and leave behind

**Copy this structure:** ordinary mechanisms receive `HardwareMap` plus their configuration,
defensively snapshot it, privately construct final Plants, own update order and cached status, and
terminally stop every owned resource.

**Adapt these decisions:** actuator kind, units, bounds, reference policy, tolerances, targets,
guards, controller choice, and the exact evidence the team's mechanism can truthfully publish.

**Do not copy as physical facts:** the Reference motor names, directions, ticks-per-inch values,
powers, velocities, servo positions, limits, tolerances, or timing. Do not expose private Plants so
controls or Auto can become competing writers.

**Previous:** [Controls and intent](<Controls and Intent.md>)

**Next:** [Tasks and autonomous](<Tasks and Autonomous.md>)

For the full Plant grammar, see
[`FTC Actuators and Plants`](<../../ftc-boundary/FTC Actuators & Plants.md>).
