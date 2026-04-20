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
- [ ] `drive/guidance/DriveGuidance` (`build()` still appears too early; `resolveWith()` still mixes required answers and optional policy)
- [ ] `ftc/FtcActuators` velocity builders
- [ ] `spatial/TagSelections`
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
