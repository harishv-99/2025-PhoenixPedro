# Phoenix Pedro autonomous

Phoenix Auto is an ordinary managed `FtcRobotOpMode` program. Static match entries, the INIT
selector, and the Pedro integration test all extend `PhoenixAutoOpMode` and differ only in the
`PhoenixAutoSetup` they return.

```java
@Autonomous(name = "Phoenix Red Audience Safe", group = "Phoenix")
public final class PhoenixRedAudienceSafeAuto extends PhoenixAutoOpMode {
    @Override
    protected PhoenixAutoSetup autoSetup() {
        return PhoenixAutoSetup.fromFixedSpec(
                PhoenixAutoSpec.audienceSafe(PhoenixAlliance.RED),
                PhoenixReadiness.AutoPurpose.MATCH_AUTO
        );
    }
}
```

The parallel selector factory is
`PhoenixAutoSetup.fromInitSelection(defaultSpec, purpose)`. No Phoenix Auto entry overrides raw FTC
`init`, `init_loop`, `start`, `loop`, or `stop` callbacks.

## One construction path

`PhoenixAutoProgram` performs the internal declaration once during INIT:

1. invalidate an old Phoenix match handoff and copy the complete `PhoenixProfile`;
2. register one data-only `PhoenixAutoPrestart`;
3. construct one validated `PedroPathingRuntime`;
4. register the final-pose-plus-frozen-alliance match handoff transaction when applicable;
5. declare Phoenix vision, localization, targeting, Pedro heartbeat, and scoring owners;
6. build a fixed routine eagerly, or declare one `Tasks.buildAtStart(...)` root for an INIT
   selection;
7. register additive presenters.

The hardware graph is never reconstructed inside the same OpMode. A construction/configuration
error requires correcting the configuration and restarting the OpMode; it is not a selector retry.

INIT selection changes only `PhoenixAutoSpec` data: alliance, start position, and strategy. The
last choice opens a read-only summary; it does not create a second confirmation state. On FTC START,
`PhoenixAutoPrestart` freezes the final spec exactly once and returns `READY` or `BLOCKED`. A blocked
program keeps services, Tasks, bindings, and outputs inert while its clock and presenters continue,
so the Driver Station retains the actionable readiness message.

## Readiness and target eligibility

`PhoenixReadiness.pedroAuto(...)` blocks purpose mismatches, required calibration failures,
missing selected target/catalog/layout facts, and routes that are not match-ready. Current checked-in
competition routes remain `INTEGRATION_ONLY`, so match entries intentionally block. The explicitly
named Pedro test entry may run that geometry with a persistent test warning.

Target visibility and game-piece uncertainty are not structural readiness. The routine waits for
the configured bounded interval; if a target is still unavailable, the scoring phase reports its
timeout and `PhoenixPedroPreParkTask` takes the explicit return/park fallback. A pre-reset INIT
timestamp is never carried across the START clock epoch.

Constructing the Pinpoint owner issues one mandatory reset request and returns immediately; it does
not sleep for calibration. Keep the robot stationary until a later poll reports exact `READY` and
publishes finite measured pose and velocity. The Pedro heartbeat fail-stops drive output whenever
that same-cycle evidence is absent, and its diagnostic reports the predictor's cached device status
without performing a second hardware poll.

Phoenix constructs targeting from the full defensive profile. TeleOp and Auto each supply the same
singleton shape mapped from their own START-frozen `PhoenixAlliance`. Eligibility is validated and
applied before preview/sticky selection, so an opposite-alliance tag can still support localization
through the complete fixed layout but cannot become either mode's scoring target.

## Geometry and routines

`PhoenixPedroPathFactory` is the only owner of Pedro path geometry and route-maturity facts.
`PhoenixPedroAutoRoutineFactory` maps a frozen `PhoenixAutoSpec` to one Task graph over
`PhoenixCapabilities`.

Fixed entries build fixed paths eagerly. A selector defers only its root Task construction with
`Tasks.buildAtStart(...)`, after the spec freezes. Later route phases that depend on live pose keep
using `RouteTasks.followBuiltAtStart(...)`; this is a different, narrower decision boundary.

Route outcomes remain truthful. Endpoint success may proceed to position-dependent scoring.
Timeout or other configured uncertainty may choose the explicit park fallback. Cancellation-like
outcomes abort and never start a fallback as a side effect of cancellation.

## Managed order

At a permitted START:

```text
freeze setup
-> reset shared LoopClock
-> apply frozen Pedro start pose
-> vision readiness
-> localization
-> targeting
-> Pedro heartbeat
-> root Task start + first update
-> scoring output
```

Each active loop is:

```text
LoopClock
-> vision/localization/targeting/Pedro service
-> bindings
-> Tasks
-> scoring output
-> presenters
-> one telemetry commit
```

STOP cancels the root, stops scoring, writes Pedro physical zero, resets targeting, and closes
vision. Match Auto uses `RobotProgram.stopHandoff(...)`: it captures the cached pose and the
START-frozen alliance only on normal ACTIVE STOP, publishes only after every cleanup action
succeeds, and invalidates stale state on blocked, failed, pre-start, test, or publication-failure
paths.
