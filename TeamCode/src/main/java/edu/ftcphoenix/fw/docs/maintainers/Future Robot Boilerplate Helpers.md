# Future Robot Boilerplate Helpers

This note tracks framework ideas that could make future robots easier to write without turning the
framework into a season-specific robot superclass. The framework should carry boring mechanics:
setup UI, cleanup-failure aggregation, telemetry composition, and small integration seams. Robot
code should still own game facts, mechanism vocabulary, retry policy, and strategy choices.

## Design boundary

Good framework candidates:

- repeated FTC lifecycle safety patterns
- INIT-time selector screens and locked summaries
- partial-build cleanup mechanics
- retry/error display after multiple callers prove the same policy
- telemetry-frame composition
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

### Deferred: generic retry-safe INIT runtime guard

A supplier-shaped `InitRuntimeGuard<T>` cannot clean a resource created before a throwing supplier
returns `T`. Solving that generally would require a resource-registration and ownership-transfer
scope, plus one framework-selected retry/replacement policy. Current Auto, tester, and calibration
callers do not share that complete contract.

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

## Candidate 3: telemetry frame composition

Robot code often wants several owners to contribute rows to one Driver Station frame. Today the
presenter usually owns `telemetry.update()`, which is simple but makes sidecar debug rows order-
dependent.

A future helper could make the frame explicit:

```java
TelemetryFrame frame = TelemetryFrame.begin(telemetry)
        .section("Auto")
        .data("auto.spec", spec.summary())
        .data("auto.paths", pathLabel);

robot.emitAutoTelemetry(frame);
frame.update();
```

Possible benefits:

- one final update per loop by construction
- clear sections for robot, route follower, targeting, and mechanism status
- fewer accidental double updates
- easier unit-style testing of what each presenter contributes

This is a framework-shaped problem, but it should be introduced carefully because it touches a lot of
presenters and OpModes.

## Candidate 4: external route auto host

Phoenix now has Pedro-specific Auto glue in robot code, which is the right first step. A framework
helper might eventually own the library-neutral parts of external-route Auto:

- route follower start/stop lifecycle
- adapter cleanup
- route status telemetry hook
- task cancellation on OpMode stop

The framework should not know Pedro path geometry or strategy labels. At most, it should host an
already-built route adapter and a robot-provided task sequence.

## Recommended order

1. Keep the current UI primitives as the stable base.
2. Watch whether another robot or selector repeats Phoenix's menu boilerplate.
3. If yes, build `SetupWizard` before adding any new robot-specific selectors.
4. Use `CleanupActions` only for ordered cleanup/error aggregation; keep retry and ownership policy
   in each real owner.
5. Reconsider `InitRuntimeGuard` only after multiple complete runtime owners share the same
   construction, partial-cleanup, retry, replacement, and presentation contract.
6. Consider `TelemetryFrame` after a few more presenters need shared-frame composition.

Do not add a generic robot superclass as the first response to boilerplate. Prefer small helpers that
remove one repeated ceremony while leaving construction order and ownership visible in robot code.
