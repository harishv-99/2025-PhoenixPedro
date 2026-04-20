# Builder Improvement Backlog

This running list tracks framework builders that should be reviewed against the builder principles in [`../../Framework Principles.md`](<../../Framework Principles.md>). Update this file as builders are improved so maintainers can continue the cleanup one builder at a time.

## Principles to apply

1. Each required conceptual question gets answered explicitly.
2. Optional tuning only appears after entering a tuning branch.
3. A stage exposes only options that make sense after prior answers.
4. Unit boundaries are obvious from names.
5. Invalid combinations are prevented by types when practical, not merely rejected at build().

## Current status

- [x] `ftc/FtcActuators` position builders
- [x] `actuation/ScalarSetpointPlanner`
- [x] `drive/guidance/DriveGuidance`
- [x] `ftc/FtcActuators` velocity builders
- [x] `spatial/TagSelections`
- [ ] `spatial/SpatialQuery` / `spatial/SpatialQuerySpec`
- [ ] `actuation/PositionCalibrationTasks` (make timeout choice explicit: failAfter(...) or neverTimeout())
- [ ] `spatial/SpatialSolveSet` (empty build should fail immediately)

## Notes

### `actuation/ScalarSetpointPlanner`

Completed in this pass. The builder now asks required questions in a staged order:

1. choose the request source (`request(...)` or `requestFromSpatial()`)
2. choose the scalar domain (`forPositionPlant(...)` or `explicitDomain()`)
3. optionally enter policy/completion tuning branches
4. build

`requestFromSpatial()` now forces an explicit choice between a facing-derived request and a translation-derived request. That removes the previous hidden precedence where both mappers could be configured and facing silently won.

### `drive/guidance/DriveGuidance`

Completed in the second builder cleanup pass. `DriveGuidance.plan()` now stays parallel with `ScalarSetpoints.plan()`:

1. choose the requested drive target (`translateTo()`, `faceTo()`, or both)
2. optionally choose `controlFrames(...)`
3. choose one explicit solve mode in `solveWith()`
4. optionally enter mode-specific solve-policy tuning and general `driveTuning()`
5. build

`build()` is no longer visible before both target and solve-mode questions are answered. `solveWith()` now presents explicit solve-mode choices:

- `localizationOnlyWithDefaults(...)` or `localizationOnly()...doneLocalizationOnly()`
- `aprilTagsOnlyWithDefaults(...)` or `aprilTagsOnly()...doneAprilTagsOnly()`
- `adaptiveWithDefaults(...)` or `adaptive()...doneAdaptive()`

Mode-specific optional policy stays inside the matching branch. Adaptive-only controls such as `translationTakeover(...)` and `omegaPolicy(...)` are only visible in the adaptive branch. General drivetrain tuning stays inside `driveTuning()`, paralleling `ScalarSetpoints.plan().policy()` / `completion()`.

### `ftc/FtcActuators` velocity builders

Completed in the third builder cleanup pass. Motor velocity now follows the same guided shape as
position, but without position-only concepts like topology, reference, and homing:

1. choose velocity loop ownership (`deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated().nativeFeedback(...).regulator(...)`)
2. choose legal velocity target bounds (`bounded(...)` or `unbounded()`)
3. choose plant/native velocity mapping (`nativeUnits()` or `scaleToNative(...)`)
4. optionally set plant-level `velocityTolerance(...)` and `rateLimit(...)`
5. build

The old `MotorVelocityControl` value-object API was removed instead of retained as a parallel path.
Velocity uses a zero-preserving mapping only; no `rangeMapsToNative(...)` is exposed for velocity.

## Recommended next builder

Next likely target: `spatial/SpatialQuery` / `spatial/SpatialQuerySpec`. They are lower-level than DriveGuidance, but they still expose build() before all conceptual questions are answered.

### `spatial/TagSelections`

Completed in the fourth builder cleanup pass. Tag selection now asks the stateful selection questions explicitly:

1. choose candidate tag IDs with `among(...)`
2. choose the detection freshness window with `freshWithinSec(...)`
3. choose the stateless preview policy with `choose(...)`
4. choose the state mode with `continuous()`, `stickyWhen(...)`, or `stickyUntilReset()`
5. for sticky modes, explicitly choose loss behavior with `holdUntilDisabled()`, `holdUntilReset()`, or `reacquireAfterLossSec(...)`
6. build

The old hidden defaults for freshness, policy, and continuous mode were removed. Sticky-only loss behavior is no longer visible on continuous selectors, and units are explicit in methods that accept seconds.
