# FTC Auto-to-TeleOp handoff

Phoenix transfers one immutable match snapshot from a normally stopped match Auto to the next
TeleOp. It contains the final field pose plus the `PhoenixAlliance` frozen by Auto at FTC START. The
transfer is best-effort, process-local, single-consumer, and freshness-bounded. It does not transfer
hardware owners, Tasks, a targeting command, estimate quality, route strategy, or vendor objects.

## Managed Auto transaction

The Auto declaration registers one typed transaction:

```java
program.stopHandoff(
        () -> new MatchHandoffCapture(
                pedro.motionPredictor().getEstimate(),
                prestart.frozenSpec().alliance),
        capture -> PhoenixMatchHandoff.publishFromAuto(
                this,
                capture.finalPose,
                capture.alliance),
        PhoenixMatchHandoff::clear
);
```

`MatchHandoffCapture` is Phoenix Auto's private immutable pair of cached pose estimate and frozen
alliance; it is not a framework payload type.

Registration invalidates any older snapshot immediately. On normal STOP from `ACTIVE`,
`RobotProgram` marks itself terminal, captures the already-cached estimate, performs its complete
cleanup, and publishes only if every cleanup action succeeds.

No value is captured or published when the program stops before START, is `BLOCKED`, fails during
configuration/START/loop/presentation/commit, or is a diagnostic/test entry. Capture,
publication, and cleanup failures preserve the first failure and attach later cleanup failures in
execution order; all non-publication paths invalidate. `Error` is not caught.

The cleanup order remains:

```text
cancel root/queued Tasks
-> clear bindings
-> stop outputs in declaration order
-> stop services in reverse declaration order
-> publish captured match snapshot (only after success)
```

For Phoenix Auto the program marks itself terminal before capture, so no later loop phase can
advance. It then captures the already-cached pose and the prestart's START-frozen alliance before
cleanup cancels the root, stops scoring, writes Pedro physical zero, resets targeting, and closes
vision. Publication happens only after all of those cleanup actions succeed.

## TeleOp restore

TeleOp first installs one visible RED-default alliance selector and declares its normal localization
graph. It then attempts one consume during INIT:

```java
PhoenixTeleOpPrestart prestart = program.prestart(
        new PhoenixTeleOpPrestart(profile, gamepad1, PhoenixAlliance.RED));
PhoenixRobot robot = new PhoenixRobot(
        hardwareMap, telemetry, gamepad1, gamepad2, profile);
robot.declareTeleOp(program, prestart.eligibleScoringTagIds());

PhoenixMatchHandoff.RestoreResult restore =
        PhoenixMatchHandoff.restoreForTeleOp(
                this,
                robot,
                prestart::seedDraftFromAuto);
program.presenter(prestart::present);
program.presenter(robot.teleOpPresenter(restore));
```

`RESTORED` applies the pose and moves the visible alliance draft to Auto's frozen alliance. The
alliance is only a seed: gamepad 1's D-pad directly selects RED or BLUE during INIT, with no `A`
pseudo-confirmation. FTC START freezes that TeleOp choice, and its mapped singleton scoring-tag set
is the sole targeting-eligibility authority. The handoff never writes targeting directly.

`MISSING`, `STALE`, and `ALREADY_CONSUMED` leave the normally initialized TeleOp pose and visible RED
draft unchanged, and remain visible in INIT telemetry. Restore after START fails because
localization may no longer be rebased and prestart data may no longer be edited through this seam.

## Limits

The snapshot lives only inside the Robot Controller process. Restarting the app loses it. The age
window is a delivery guard, not a claim that the physical robot stayed at the captured pose. Moving
the robot between OpModes, changing field origin, or using a different localization convention
makes the pose component semantically invalid even if the snapshot is still fresh. When that pose
cannot be trusted, skip or clear the handoff and select the TeleOp alliance directly.
