# FTC Auto-to-TeleOp handoff

`FtcAutoToTeleOpHandoff<T>` carries one immutable, short-lived robot-owned snapshot from a normally
stopped match Auto to the next TeleOp. The transfer is best-effort, process-local,
single-consumer, and freshness-bounded. It does not transfer hardware owners, Tasks, commands,
estimate quality, route strategy, or vendor objects.

The framework owns only the typed process-local carrier and the managed STOP transaction. The
robot owns the payload type, which OpModes may publish and consume it, how a delivered payload is
applied, and what every non-delivery status means.

## Robot-owned carrier

Keep one carrier private behind a robot-owned wrapper. The payload must itself be immutable or a
defensive snapshot because the carrier stores the supplied reference rather than deep-copying an
arbitrary `T`:

```java
final class MatchSnapshot {
    final Pose2d finalFieldPose;
    final Alliance alliance;

    MatchSnapshot(Pose2d finalFieldPose, Alliance alliance) {
        this.finalFieldPose = finalFieldPose; // Pose2d is immutable
        this.alliance = alliance;
    }

    static MatchSnapshot fromCachedEstimate(PoseEstimate estimate, Alliance alliance) {
        if (!estimate.hasPose) {
            throw new IllegalStateException("No final field pose is available for handoff");
        }
        return new MatchSnapshot(estimate.toPose2d(), alliance);
    }
}

final class MatchHandoff {
    private static final FtcAutoToTeleOpHandoff<MatchSnapshot> SLOT =
            FtcAutoToTeleOpHandoff.create(
                    "match snapshot",
                    MatchSnapshot.class,
                    10.0);

    static void clear() {
        SLOT.clear();
    }

    static void publishFromAuto(OpMode auto, MatchSnapshot snapshot) {
        SLOT.publishFromAuto(auto, snapshot);
    }

    static FtcAutoToTeleOpHandoff.ConsumeResult<MatchSnapshot> consumeForTeleOp(
            OpMode teleOp) {
        return SLOT.consumeForTeleOp(teleOp);
    }
}
```

The wrapper is the policy boundary. It can keep the carrier inaccessible to diagnostic, practice,
or unrelated OpModes even though those classes run in the same Robot Controller process.

## Managed Auto transaction

Clear first, before constructing any owner that could fail, then register one typed STOP
transaction while declaring the match Auto:

```java
@Override
protected void configure(RobotProgram program) {
    MatchHandoff.clear();

    // Construct and declare the match-Auto owners here.
    program.stopHandoff(
            () -> MatchSnapshot.fromCachedEstimate(
                    localization.getEstimate(),
                    prestart.frozenSpec().alliance),
            snapshot -> MatchHandoff.publishFromAuto(this, snapshot),
            MatchHandoff::clear
    );
}
```

The first line prevents a configuration failure before registration from exposing a previous
match's still-fresh snapshot. Registration invalidates again when the managed transaction becomes
owned. On normal STOP from `ACTIVE`,
`RobotProgram` marks itself terminal, captures already-cached facts, performs its complete cleanup,
and publishes only if every cleanup action succeeds. The capture callback must not poll hardware,
advance localization, or mutate robot behavior.

No value is captured or published when the program stops before START, is `BLOCKED`, or fails
during configuration, START, loop, presentation, or telemetry commit. The clear-first step covers
failures before registration; after registration, every non-publication path invalidates. Capture,
publication, and cleanup failures preserve the first failure and attach later cleanup failures in
execution order. `Error` is not caught.

The cleanup order remains:

```text
capture cached snapshot
-> cancel root/queued Tasks
-> clear bindings
-> stop outputs in declaration order
-> stop services in reverse declaration order
-> publish snapshot (only after complete success)
```

Capture happens after the program becomes terminal but before cleanup clears state. Publication
happens only after Tasks, outputs, services, and every other managed owner have stopped cleanly.

## TeleOp consume and fallback

Construct and register the new TeleOp robot normally, then consume once during INIT while its
localization and editable prestart owners can still accept a seed:

```java
FtcAutoToTeleOpHandoff.ConsumeResult<MatchSnapshot> result =
        MatchHandoff.consumeForTeleOp(this);

if (result.status() == FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED) {
    MatchSnapshot snapshot = result.payloadOrNull();
    poseResetter.setPose(snapshot.finalFieldPose);
    prestart.seedAlliance(snapshot.alliance);
}
```

`DELIVERED` is the only status with a payload. `MISSING`, `STALE`, and `ALREADY_CONSUMED` must leave
the normally initialized TeleOp state unchanged or take another explicit robot-owned fallback.
Keep the result visible in INIT telemetry when drivers need to know whether restoration occurred.
Reject or avoid restore after START if localization or prestart data may no longer be changed
through that seam.

Treat restored selection as a seed unless the robot deliberately owns a stronger contract. A
driver-editable draft may still change before FTC START; the handoff should not become a hidden
targeting or control command.

## Limits

The snapshot lives only inside the Robot Controller process. Restarting the app, reloading the
classloader, or redeploying loses it. Its age uses `System.nanoTime()` because Auto and TeleOp do
not share one `LoopClock`.

The age window is a delivery guard, not proof that the physical robot stayed at the captured pose.
Moving the robot between OpModes, changing field origin, or changing localization conventions can
make a fresh payload semantically invalid. When a payload cannot be trusted, clear or ignore it and
use the robot's normal TeleOp initialization.
