# Builder Improvement Backlog

This running list tracks framework builders that should be reviewed against the builder principles in [`../../Framework Principles.md`](<../../Framework Principles.md>). Update this file as builders are improved so maintainers can continue the cleanup one builder at a time.

## Principles to apply

1. Each required conceptual question gets answered explicitly.
2. Mutually-exclusive required choices get their own stage; the stage exposes only the valid answer methods, and each answer advances to the next question.
3. Avoid APIs where a later answer silently replaces an earlier answer for the same required question.
4. Optional tuning only appears after entering a tuning branch.
5. Use `done...()` only for multi-setting tuning branches where several independent knobs may be set before returning.
6. A stage exposes only options that make sense after prior answers.
7. Unit boundaries are obvious from names.
8. Invalid combinations are prevented by types when practical, not merely rejected at build().

## Current status

- [x] `ftc/FtcActuators` position builders
- [x] `actuation/PlantTargets`
- [x] `drive/guidance/DriveGuidance`
- [x] `ftc/FtcActuators` velocity builders
- [x] `actuation/Plants.fromOutputs()` shared Plant grammar
- [x] `spatial/TagSelections`
- [x] `spatial/SpatialQuery` / `spatial/SpatialQuerySpec`
- [x] `actuation/PositionCalibrationTasks`
- [x] `spatial/SpatialSolveSet`

## Notes

### `actuation/PlantTargets`

Completed in the Plant-target consolidation pass. `PlantTargets` is now the single student-facing
place to generate values intended for a Plant target:

1. use `PlantTargets.exact(...)` for simple constants or scalar sources
2. use `PlantTargets.overlay(...)` for base + behavior-layer arbitration
3. use `PlantTargets.equivalentPositionsOf(...)` for one periodic command and
   `PlantTargets.plan(request)` for advanced alternative requests that need Plant context
4. choose an explicit `whenUnavailable()` policy for smart planners
5. use `addIfAvailable(...)` only when an enabled overlay layer should explicitly fall through

The planner factory requires either a fixed `PlantTargetRequest` or a live
`Source<PlantTargetRequest>`, then applies the stricter staged-choice rule. The user must answer
exactly one preference question (`nearestToMeasurement()`, `preferIncreasing()`,
`preferDecreasing()`, or `preferRangeCenter()`), then exactly one unreachable-alternative question
(`rejectUnreachable()` or `clampUnreachableToRange()`). Each answer returns a type that exposes
only the next question, so a later call cannot silently replace an earlier choice. `doneAccept()`
remains because `accept()` is a multi-setting optional tuning branch (`maxObservationAgeSec(...)`,
`minQuality(...)`).

The older target-planner family and scalar overlay helper were removed instead of kept as parallel
paths. Plain `ScalarSource`s still exist as primitive number streams, but Plant target arbitration now
happens in Plant-target space. The normal overlay method is still `add(...)`: an enabled layer is
expected to produce a target. The separate `addIfAvailable(...)` name is reserved for explicit
fall-through layers so target availability does not become a hidden Boolean filter.

### `drive/guidance/DriveGuidance`

Completed in the second builder cleanup pass. `DriveGuidance.plan()` and
`PlantTargets.plan(request)` both answer required behavior questions in order, then enter optional
tuning branches only when needed. Drive Guidance legitimately starts with no argument because its
first stage presents a real translation-versus-facing choice; Plant target planning requires its
fixed or live request at the factory boundary.

1. choose the first requested drive target (`translateTo()` or `faceTo()`), then optionally add the other with `andFaceTo()` / `andTranslateTo()`; target choice methods such as `fieldPointInches(...)`, `point(...)`, and `frameHeading(...)` return directly to the parent stage
2. optionally choose `controlFrames(...)`
3. choose one explicit solve mode in `solveWith()`
4. optionally enter mode-specific solve-policy tuning and general `driveTuning()`
5. build

`build()` is no longer visible before both target and solve-mode questions are answered. `solveWith()` now presents explicit solve-mode choices:

- `localizationOnlyWithDefaults(...)` or `localizationOnly()...doneLocalizationOnly()`
- `aprilTagsOnlyWithDefaults(...)` or `aprilTagsOnly()...doneAprilTagsOnly()`
- `adaptiveWithDefaults(...)` or `adaptive()...doneAdaptive()`

Mode-specific optional policy stays inside the matching branch. Adaptive-only controls such as
`translationTakeover(...)` and `omegaPolicy(...)` are only visible in the adaptive branch. General
drivetrain tuning stays inside `driveTuning()`. The review intentionally kept `doneLocalizationOnly()`,
`doneAprilTagsOnly()`, `doneAdaptive()`, and `doneDriveTuning()` because those branches can set several
independent optional tuning values before returning.

### `ftc/FtcActuators` velocity builders

Completed in the third builder cleanup pass. Motor velocity now follows the same guided shape as
position, but without position-only concepts like periodicity, reference, and homing:

1. choose velocity loop ownership (`deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated()` followed by one direct feedback answer and `regulator(...)`)
2. choose legal velocity target bounds (`bounded(...)` or `unbounded()`)
3. choose plant/native velocity mapping (`nativeUnits()` or `scaleToNative(...)`)
4. answer the required plant-unit completion question exactly once with `velocityTolerance(...)`
5. optionally set guards such as `targetGuards().maxTargetRate(...)`
6. bind an ordinary command with `targetFromNewCommand(initialValue)`, or bind a supplied final
   resolver with `targetFromResolver(finalResolver)`; the resolver may carry a recognized command
   target or none, and separately owned targets or read-only scalars cross this boundary explicitly
   through `PlantTargets.exact(...)`

The old `MotorVelocityControl` value-object API was removed instead of retained as a parallel path.
Velocity uses a zero-preserving mapping only; no `rangeMapsToNative(...)` is exposed for velocity.
As with the rest of the plant API, `bounded(...)`, the required tolerance, and resolved target values remain
in plant units unless a method name explicitly calls out native/controller units. No hidden or
native-unit tolerance shortcut remains.

### `actuation/Plants.fromOutputs()` shared Plant grammar

The one-grammar pass replaced public `MappedPositionPlant`/`MappedVelocityPlant` starts and direct
`Plants` factory overloads with one hardware-neutral gateway. Custom HAL adapters, portable hosts,
and hardware-neutral tests start with `Plants.fromOutputs()`; ordinary FTC mechanisms continue to
start with `FtcActuators.plant(hardwareMap)`. These gateways prove different boundary inputs but
converge on the same periodicity, range, mapping, guard, target, validation, and hidden runtime
engine.

The neutral gateway has six control-path branches: direct power, commanded position,
device-managed position and velocity with explicit feedback, and Phoenix-regulated position and
velocity over raw power. Each branch exposes only the facts it needs. Position answers periodicity
before bounds and mapping; feedback branches require one plant-unit tolerance; command-only
position omits feedback-only reference, calibration, and tolerance stages. Both gateways finish
through the same inline guard branch and either `targetFromNewCommand(initialValue)` or
`targetFromResolver(finalResolver)`, then `build()`.

Concrete mapped runtimes and their mutable assembly objects stay private. This prevents omitted
answers without making implementation classes, direct overloads, reusable stateful guard chains,
or FTC hardware-provider wrappers into competing public construction paths.

### `spatial/SpatialQuery` / `spatial/SpatialQuerySpec`

Completed in the fifth builder cleanup pass. Runtime queries and reusable specs now share the same
staged shape:

1. choose the target relationship (`translateTo(...)` or `faceTo(...)`)
2. optionally add the other channel with `andFaceTo(...)` / `andTranslateTo(...)`
3. optionally supply `controlFrames(...)` and `fixedAprilTagLayout(...)`
4. choose solve lanes with `solveWith(...)`
5. build
