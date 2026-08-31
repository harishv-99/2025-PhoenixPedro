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

1. invalidate an old Phoenix match handoff, create one fresh `PhoenixProfile.current()` graph, run
   the centralized drive-versus-scoring motor-ownership preflight, and raw-copy the Auto policy;
2. register one data-only `PhoenixAutoPrestart`;
3. construct the resource-only `PhoenixRobot(hardwareMap)` root;
4. construct one validated `PedroPathingRuntime`;
5. register the final-pose-plus-frozen-alliance match handoff transaction when applicable;
6. declare Phoenix vision, localization, targeting, Pedro heartbeat, and scoring owners;
7. build a fixed routine eagerly, or declare one `Tasks.buildAtStart(...)` root for an INIT
   selection;
8. register additive presenters.

The runtime construction in step 4 has one effect boundary:

```java
PedroPathingRuntime runtime = PedroPathingRuntime.create(
        hardwareMap,
        Constants.phoenixAutoRuntimeConfig(
                profile.localization.predictor,
                profile.drive.wiring,
                profile.drive.enableZeroPowerBrake));
```

`Constants.phoenixAutoRuntimeConfig(predictor, wiring, enableZeroPowerBrake)` is pure. It raw-copies
only those three inputs, combines them with fresh checked-in Pedro follower/constraint tuning and
the field transform, and returns an independent `PedroPathingRuntime.Config`. It accepts and retains
no aggregate `PhoenixProfile`, creates no hardware, and does not make unrelated profile sections
part of Pedro configuration validation.

`PedroPathingRuntime.create(...)` raw-copies and validates the complete captured Config before
hardware lookup, the Pinpoint reset request, motor output, Follower construction, or Pedro-static
mutation. It then owns independent retained vendor values, so later edits to the profile or tuning
draft cannot alter the running graph. The runtime exports narrow predictor, adapter, path-builder,
starting-pose, and cached-pose operations rather than its mutable Follower.

The hardware graph is never reconstructed inside the same OpMode. A construction/configuration
error requires correcting the configuration and restarting the OpMode; it is not a selector retry.

The resource-only `new PhoenixRobot(hardwareMap)` call precedes runtime construction but acquires no
hardware. After the runtime exists, Auto passes the local profile plus only Auto-active roles to
`declareAuto(...)`; no Gamepad is a dormant root dependency. Each long-lived owner captures and
validates its own active slice. The package-private preflight runs before Pedro or Phoenix hardware
effects and rejects an intake or flywheel name that exactly collides with any drive motor name.
Advanced direct declarations with an opaque drive sink must establish exclusive hardware ownership
themselves.

After valid configuration enters SDK/vendor construction, cleanup is necessarily best effort. The
runtime breaks a successfully returned drivetrain if a later construction step fails, but a
Mecanum constructor that throws may return no handle, Pinpoint has no rollback/close operation, and
a failed Follower may already have changed Pedro statics. Those limits are another reason the
OpMode never retries construction in place.

Pedro's completed-Follower adapter constructor remains an advanced custom-host seam, while the
project's package-local native-Follower factory exists only for exclusive generated tuning tools
and `PedroTest`. Phoenix Auto uses neither alternate seam.

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

Phoenix constructs targeting from `PhoenixTargeting.Config` plus the fixed layout and the current
mode's eligibility source. TeleOp and Auto each supply the same singleton shape mapped from their
own START-frozen `PhoenixAlliance`. Eligibility is validated and applied before preview/sticky
selection, so an opposite-alliance tag can still support localization through the complete fixed
layout but cannot become either mode's scoring target.

Passing runtime Config validation proves only that the authored software graph is complete, finite,
and internally coherent. It does not prove motor identity/direction, Pinpoint placement or READY
behavior, follower tuning, field alignment, route clearance, stopping distance, or physical STOP;
those remain Phoenix calibration and on-robot evidence.

## Geometry and routines

`PhoenixPedroPathFactory` is the only owner of Pedro path geometry and route-maturity facts.
`PhoenixPedroAutoRoutineFactory` maps a frozen `PhoenixAutoSpec` to one Task graph over
`PhoenixCapabilities`.

`PhoenixPedroAutoContext` receives a `PhoenixAutoConfig`, retains its own raw snapshot, and exposes
`autoConfig()` as a defensive copy for path/routine collaborators. It has no `PhoenixProfile` or
`profile()` reach-through. `PhoenixAutoConfig.defaults()` is the checked-in owner recipe for the
eight Auto timing and budget fields; its public `copy()` deliberately preserves draft values and
does not validate. The path/task owners validate the finite positive/nonnegative domains they
actually consume in source order before a route or mechanism effect.

The checked-in recipe is deliberately small:

| `PhoenixAutoConfig` field | Current value | Active-owner domain |
|---|---:|---|
| `parkTakeoverElapsedSec` | `25.0` | finite and `> 0` |
| `routeTimeoutSec` | `4.0` | finite and `> 0` |
| `aimHeadingToleranceDeg` | `2.0` | finite, `>= 0`, and finite after radians conversion |
| `aimTimeoutSec` | `1.75` | finite and `> 0` |
| `aimMaxNoGuidanceSec` | `0.75` | finite and `> 0` |
| `waitForTargetSec` | `0.75` | finite and `>= 0` |
| `waitForShotCompleteSec` | `2.5` | finite and `>= 0` |
| `pedroIntegrationTestDistanceIn` | `12.0` | finite and `> 0` |

These values are software-valid starting points, not evidence that a route is clear, an aim timeout
is safe, a shot completes, or the robot stops at the intended physical location.

The path factory builds through `runtime.pathBuilder()`. When a return route depends on the live
start-time pose, it obtains exactly one defensive `runtime.currentPedroPose()` snapshot. It never
receives the raw Follower or polls hardware while interpreting geometry.

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
