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
3. use `PlantTargets.plan()` for equivalent/candidate target requests that need Plant context
4. choose an explicit `whenUnavailable()` policy for smart planners
5. use `addIfAvailable(...)` only when an enabled overlay layer should explicitly fall through

The planner builder now applies the stricter staged-choice rule: after `request(...)`, the user
must answer exactly one preference question (`nearestToMeasurement()`, `preferIncreasing()`,
`preferDecreasing()`, or `preferRangeCenter()`), then exactly one unreachable-candidate question
(`rejectUnreachable()` or `clampUnreachableToRange()`). Each answer returns a type that exposes
only the next question, so a later call cannot silently replace an earlier choice. `doneAccept()`
remains because `accept()` is a multi-setting optional tuning branch (`maxRequestAgeSec(...)`,
`minQuality(...)`).

The older target-planner family and scalar overlay helper were removed instead of kept as parallel
paths. Plain `ScalarSource`s still exist as primitive number streams, but Plant target arbitration now
happens in Plant-target space. The normal overlay method is still `add(...)`: an enabled layer is
expected to produce a target. The separate `addIfAvailable(...)` name is reserved for explicit
fall-through layers so target availability does not become a hidden Boolean filter.

### `drive/guidance/DriveGuidance`

Completed in the second builder cleanup pass. `DriveGuidance.plan()` now stays parallel with
`PlantTargets.plan()` in spirit: answer required behavior questions in order, then enter optional tuning
branches only when needed.

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
position, but without position-only concepts like topology, reference, and homing:

1. choose velocity loop ownership (`deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated().nativeFeedback(...).regulator(...)`)
2. choose legal velocity target bounds (`bounded(...)` or `unbounded()`)
3. choose plant/native velocity mapping (`nativeUnits()` or `scaleToNative(...)`)
4. optionally set plant-level `velocityTolerance(...)` and `targetGuards().maxTargetRate(...)`
5. bind a target with `targetedBy(...)`, `targetedBy(ScalarSource)`, or `targetedByDefaultWritable(...)`

The old `MotorVelocityControl` value-object API was removed instead of retained as a parallel path.
Velocity uses a zero-preserving mapping only; no `rangeMapsToNative(...)` is exposed for velocity.
As with the rest of the plant API, `bounded(...)`, tolerances, and target sources remain in plant
units unless a method name explicitly calls out native/controller units.

### `spatial/SpatialQuery` / `spatial/SpatialQuerySpec`

Completed in the fifth builder cleanup pass. Runtime queries and reusable specs now share the same
staged shape:

1. choose the target relationship (`translateTo(...)` or `faceTo(...)`)
2. optionally add the other channel with `andFaceTo(...)` / `andTranslateTo(...)`
3. optionally supply `controlFrames(...)` and `fixedAprilTagLayout(...)`
4. choose solve lanes with `solveWith(...)`
5. build
