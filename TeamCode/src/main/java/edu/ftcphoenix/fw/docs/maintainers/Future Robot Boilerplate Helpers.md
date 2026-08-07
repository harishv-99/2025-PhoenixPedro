# Future Robot Boilerplate Helpers

This note tracks framework ideas that could make future robots easier to write without turning the
framework into a season-specific robot superclass. The framework should carry boring mechanics:
setup UI, cleanup-failure aggregation, telemetry composition, and small integration seams. Robot
code should still own game facts, mechanism vocabulary, retry policy, and strategy choices.

## Implemented foundation: managed robot programs

Repeated starter and Pedro roots proved one complete ordinary lifecycle contract. Phoenix now
provides `FtcRobotOpMode` and its framework-created `RobotProgram`:

```java
public final class MyTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        IntakeMechanism intake = program.output(
                new IntakeMechanism(hardwareMap, profile.intake));
        MyControls controls = new MyControls(program.bindings(), gamepad1, intake);
        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, profile.drive));
    }
}
```

This host owns one clock, the fixed service/binding/Task/output/presenter order, one telemetry
commit, and terminal cleanup. It is an FTC lifecycle owner, not a robot capability superclass:
mechanisms still own Plants, controls still own operator meanings, services still own sensing or
vendor heartbeats, and routines still own strategy. Production Phoenix's selector, retry,
readiness, and handoff migration is deliberately a separate design gate.

## Design boundary

Good framework candidates:

- repeated FTC lifecycle safety patterns
- INIT-time selector screens and locked summaries
- partial-build cleanup mechanics
- retry/error display after multiple callers prove the same policy
- repeated telemetry-frame ownership ceremony, once multiple callers demonstrate it
- route-library adapter seams
- small typed helpers that prevent invalid setup states

Poor framework candidates:

- alliance-specific strategy names
- game-specific route branches
- mechanism-specific capability families
- season scoring rules
- a giant `BaseRobot` class that hides construction order

The principle is: **the framework can own the ceremony, but the robot must own the meaning**.

## Implemented foundation: cleanup actions

Phoenix now provides the small reusable part that repeated callers actually share:

```java
CleanupActions.attemptAll(
        runner::cancelAndClear,
        mechanism::stop,
        drive::stop
);
```

`attemptAll(...)` invokes caller-listed cleanup in order. After a cleanup throws a
`RuntimeException`, it still invokes later actions and then throws the first failure with later
failures suppressed; an `Error` propagates immediately. When another operation already failed,
preserve that primary failure explicitly:

```java
catch (RuntimeException failure) {
    throw CleanupActions.attemptAllAfterFailure(
            failure,
            mechanism::stop,
            drive::stop
    );
}
```

The second method returns the exact supplied failure after attaching cleanup failures. A caller may
instead retain it for telemetry or wrap it as an actionable cause.

The helper deliberately does **not** own resource registration, construction, retry eligibility,
replacement, error presentation, terminal state, or loop order. The caller must first mark itself
terminal or detach owned references when reentrant cleanup requires that guarantee, then list only
eligible cleanup actions in their required safety order. Do not use `CleanupActions` to continue
ordinary robot commands after a failed prerequisite.

### Still deferred: generic retry-safe INIT replacement/retry guard

A `RobotProgram` can clean every sibling registered before a later configuration failure because
registration transfers ownership immediately. It still cannot recover a resource whose own
constructor throws before returning or choose whether a failed selector/Auto construction is safe
to retry. A generic replacement/retry guard would need one shared partial-owner transfer,
error-presentation, retry-eligibility, and replacement policy. Current advanced Auto, tester, and
calibration callers do not share that complete contract.

Keep retry state and error presentation in the real owner for now. Reconsider a generic guard only
after multiple callers independently share the same construction boundary, partial-owner transfer,
cleanup-failure policy, retry eligibility, replacement rules, and telemetry contract.

## Candidate 2: generic Auto/setup wizard

`SelectionMenu`, `MenuNavigator`, `ConfirmationScreen`, and `SummaryScreen` are now general, but a
robot-specific selector still repeats a lot of enum menu boilerplate. A future framework helper could
provide a wizard for common INIT setup flows:

```java
SetupWizard<PhoenixAutoSpec.Builder, PhoenixAutoSpec> wizard = SetupWizard
        .builder("Phoenix Auto Setup", PhoenixAutoSpec.builder())
        .enumStep("Alliance", PhoenixAutoSpec.Alliance.class,
                PhoenixAutoSpec.Builder::alliance,
                PhoenixAutoSpec.Builder::alliance)
        .enumStep("Start Position", PhoenixAutoSpec.StartPosition.class,
                PhoenixAutoSpec.Builder::startPosition,
                PhoenixAutoSpec.Builder::startPosition)
        .enumStep("Partner Plan", PhoenixAutoSpec.PartnerPlan.class,
                PhoenixAutoSpec.Builder::partnerPlan,
                PhoenixAutoSpec.Builder::partnerPlan)
        .enumStep("Strategy", PhoenixAutoStrategyId.class,
                PhoenixAutoSpec.Builder::strategy,
                PhoenixAutoSpec.Builder::strategy)
        .confirmWith(PhoenixAutoSpec.Builder::build);
```

The helper would own the menu plumbing, breadcrumbs, confirmation page, and summary lock. The robot
would still provide the enum values, labels, filtering rules, and final spec builder.

Build this only after two robots or two substantial selectors want the same pattern. Until then, the
current UI primitives are flexible enough.

## Deferred: generic telemetry frame wrapper

Robot code often wants several owners to contribute rows to one Driver Station frame. Today the
smallest sufficient contract is direct and explicit: presenters and renderers add rows without
clearing or committing, optional `DebugSink` dumps are selected at the composition-root call site,
and the owner of the complete frame calls `telemetry.update()` once.

For example:

```java
telemetry.addData("auto.spec", spec.summary());
telemetry.addData("auto.paths", pathLabel);
robotTelemetry.emitAuto(status); // additive; does not commit

if (debugRoute) {
    route.debugDump(debugSink, "route");
}

telemetry.update(); // the complete-frame owner commits once
```

This preserves visible ownership without introducing a second telemetry vocabulary. An ordinary
`RobotProgram` owns its complete INIT and active frames. A custom selector/host still owns a
selection, retry, or error frame that cannot enter the managed program.

A `TelemetryFrame` wrapper is not currently justified. It would duplicate the FTC add-data API and
add a public noun without solving diagnostic volume: selection still belongs at the call site, and
required telemetry must remain independent of optional debug output. Reconsider a helper only after
multiple composition roots need the same additional frame lifecycle, sectioning, or test capability
that direct additive rendering cannot provide.

Do not add a framework topic registry, prefix filter, or automatic whole-robot dump as a substitute.
Those designs hide ownership or still traverse every diagnostic producer when output is disabled.

## Implemented boundary: external route lifecycle roles

`RobotProgram.Service` and the private root Task runner now own the library-neutral parts of a
simple external-route Auto:

- recurring follower heartbeat and adapter stop through one service
- one retained route start/result through the root Task graph
- additive route-status presentation
- Task cancellation on OpMode stop

The framework still does not know Pedro path geometry or strategy labels. Robot code registers the
validated adapter heartbeat as a service and supplies one robot-owned Task sequence. A custom
selector/retry host remains explicit until RUNTIME-02 proves how its extra policy should compose.

## Recommended order

1. Use `FtcRobotOpMode`/`RobotProgram` for ordinary FTC robot programs; do not add a parallel base
   robot or lifecycle builder.
2. Keep the current UI primitives as the stable base.
3. Watch whether another robot or selector repeats Phoenix's menu boilerplate.
4. If yes, build `SetupWizard` before adding any new robot-specific selectors.
5. Use `CleanupActions` only for explicit custom owners outside the managed program, or internally
   for ordered cleanup/error aggregation; keep retry policy in each real owner.
6. Reconsider an INIT replacement/retry guard only after multiple complete runtime owners share the same
   construction, partial-cleanup, retry, replacement, and presentation contract.
7. Keep telemetry composition direct and additive; reconsider a frame helper only after repeated
   callers demonstrate a capability beyond one explicit final commit.

Do not turn the FTC lifecycle host into a generic robot superclass. Add only role capabilities that
remove demonstrated repeated ceremony while leaving construction order and ownership visible in
`configure(program)`.
