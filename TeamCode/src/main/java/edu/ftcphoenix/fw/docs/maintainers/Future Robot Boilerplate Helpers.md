# Future Robot Boilerplate Helpers

This note tracks framework ideas that could make future robots easier to write without turning the
framework into a season-specific robot superclass. The framework should carry boring mechanics:
setup UI, retry-safe lifecycle guards, telemetry composition, and small integration seams. Robot code
should still own game facts, mechanism vocabulary, and strategy choices.

## Design boundary

Good framework candidates:

- repeated FTC lifecycle safety patterns
- INIT-time selector screens and locked summaries
- partial-build cleanup and retry/error display
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

## Candidate 1: retry-safe INIT runtime guard

Phoenix's Pedro base now has local code that clears partial runtime after a failed INIT build. Future
robots will likely need the same pattern for Road Runner, Pedro, vision-only testers, and calibration
flows.

A framework helper could look conceptually like this:

```java
InitRuntimeGuard<MyRuntime> guard = InitRuntimeGuard.<MyRuntime>builder("Phoenix Pedro Auto")
        .buildWith(() -> buildRuntimeFor(selectedSpec))
        .cleanupWith(runtime -> runtime.close())
        .describe(runtime -> runtime.summaryRows())
        .build();
```

The helper would own:

- clearing the previous error before a new attempt
- stopping an old or partial runtime before retry
- storing the active runtime only after the build succeeds
- preserving a clear error message for telemetry
- exposing a locked summary after success

Robot code would still own what `MyRuntime` contains and how it is built.

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
4. If more OpModes need retry-safe runtime construction, build `InitRuntimeGuard`.
5. Consider `TelemetryFrame` after a few more presenters need shared-frame composition.

Do not add a generic robot superclass as the first response to boilerplate. Prefer small helpers that
remove one repeated ceremony while leaving construction order and ownership visible in robot code.
