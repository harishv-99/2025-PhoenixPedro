# FTC Auto-to-TeleOp handoff

FTC creates a new OpMode and a new robot runtime for TeleOp. A robot that needs one fact from the
completed autonomous period—most commonly its final field pose—therefore needs an explicit boundary
between those two runtimes.

Use `FtcAutoToTeleOpHandoff<T>` for that narrow case. It is a typed, process-local, one-shot carrier:

```java
private static final FtcAutoToTeleOpHandoff<MatchSnapshot> AUTO_TO_TELEOP =
        FtcAutoToTeleOpHandoff.create(
                "MyRobot Auto to TeleOp",
                MatchSnapshot.class,
                60.0
        );
```

`MatchSnapshot` is a robot-owned, immutable data value. Include only facts the next TeleOp actually
accepts, such as a final field pose or a final measured mechanism offset. Do not put hardware
objects, Tasks,
services, mutable queues, vendor follower types, or season strategy in it.

## Why this is not a general state store

The pinned FTC SDK exposes `OpMode.blackboard`, a public string-to-object map intended to persist
values between OpModes. It confirms that this process lifetime is supported, but it leaves key
spelling, casts, clearing, freshness, concurrency, and ownership to robot code.

Phoenix does not wrap or use that map. `FtcAutoToTeleOpHandoff` owns one isolated slot and provides:

- a runtime payload type check in addition to Java generics;
- an explicit maximum age measured with a process-monotonic clock;
- atomic one-shot consumption;
- an explicit `clear()` boundary for a new Auto attempt;
- distinct delivered, missing, stale, and already-consumed results.

It intentionally supports only Auto-to-TeleOp semantics. Same-OpMode state belongs in the active
robot runtime. Durable calibration belongs in checked-in configuration or deliberately chosen
persistent storage. Do not use this type as a global robot-state map or event bus.

## Auto lifecycle

Auto needs two visible boundaries because capture and cleanup have different safety requirements:

1. Call `clear()` at the beginning of Auto `init()`. This prevents an older run from being accepted.
2. After a successfully started match Auto, capture one immutable candidate from already-owned,
   cached state before cleanup clears those owners.
3. Stop the robot and its resources.
4. Only if capture and cleanup both succeed, call `publishFromAuto(...)` as the final operation.

Conceptually:

```java
@Override
public void init() {
    AUTO_TO_TELEOP.clear();
    // Build this Auto.
}

@Override
public void stop() {
    MatchSnapshot candidate = captureFinalSnapshot();
    robot.stop();
    AUTO_TO_TELEOP.publishFromAuto(this, candidate);
}
```

Real robot code must also handle partial initialization and preserve primary and cleanup failures.
If capture, start, or cleanup fails, leave the handoff empty. Do not publish before cleanup: FTC can
continue to the requested TeleOp after recording an Auto `stop()` failure, and that TeleOp must not
receive uncertain state.

Capture cached state rather than performing a new hardware update during `stop()`. FTC applies its
hardware failsafe before invoking the user stop callback.

For a failure-complete repository reference, follow `PhoenixMatchHandoff`,
`PhoenixPedroAutoOpModeBase.stop()`, and `PhoenixTeleOp.init()` together. They keep the robot-owned
payload policy, Auto cleanup ordering, TeleOp restore, and fail-stop behavior in their proper
owners.

## TeleOp lifecycle

Construct the new TeleOp robot and its localization owner first. Then consume exactly once, before
FTC START and before the first robot update:

```java
robot.initTeleOp();

FtcAutoToTeleOpHandoff.ConsumeResult<MatchSnapshot> result =
        AUTO_TO_TELEOP.consumeForTeleOp(this);

if (result.status()
        == FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED) {
    robot.applyInitialMatchSnapshot(result.payloadOrNull());
} else {
    // Explicit robot-owned fallback: keep the normally initialized TeleOp state.
}
```

The example application method is robot-owned; the framework carrier does not know how to mutate a
localizer or mechanism. If application fails after the robot was initialized, stop that robot before
reporting the failure.

A missing, stale, or already-consumed result is normal recovery information, not permission to
invent a pose. Phoenix keeps its normally initialized TeleOp localization state as the fallback.
Another robot may choose a different fallback only when it has a trustworthy input for that choice.

## Exact state contract

- A new carrier starts empty.
- `clear()` idempotently opens a fresh empty Auto-to-TeleOp cycle.
- One valid `publishFromAuto(...)` may fill that cycle.
- The first `consumeForTeleOp(...)` closes the cycle even if it finds no value or discards a stale
  value. A late publisher cannot leave data for a later TeleOp.
- At most one concurrent consumer receives the payload.
- A second publish without another `clear()` is a lifecycle error.
- The exact maximum-age boundary is accepted; older or apparently time-regressing data is stale.
- Producer and consumer OpMode objects provide class names for diagnostics only. They are not
  retained and do not authenticate the caller.

The carrier retains the payload reference; it cannot deep-copy arbitrary `T`. Publish a deeply
immutable snapshot.

## Process and physical limits

The value exists only in the current Robot Controller app process and classloader. A robot reset,
app restart, or code download produces a normal missing result. The carrier does not serialize or
recover anything.

A delivered pose proves only that the software value crossed the process boundary and was supplied
to the receiving robot owner. It does not prove that the Auto estimate matched the robot's physical
field pose. Physical localization accuracy remains an adopting-robot validation.
