# AprilTag & Localization Follow-Ups

This file is the lightweight backlog for the framework's AprilTag, fixed-tag layout, and localization work.

It exists so season-specific fixes and the next maintainership steps do not get lost between refactors.

---

## Landed in the current pass

- Added a shared multi-tag fixed-field pose solver so AprilTag-only localization and the drive-guidance AprilTag→field-pose bridge use the same math.
- Switched Phoenix TeleOp to use Pinpoint + AprilTag fusion for pose lock and selected-tag localization fallback.
- Moved official-game fixed-tag knowledge into the framework via `FtcGameTagLayout.currentGameFieldFixed()`.
- Removed the need for robot code to carry season-specific "exclude these IDs from localization" logic.
- Updated framework tools that previously converted the entire FTC SDK game library into a field-fixed `TagLayout`.

---

## Near-term follow-ups

1. Surface `FixedTagFieldPoseSolver.Config` through `DriveGuidanceSpec.AprilTags` / the guidance builder so guidance's temporary field-pose bridge and the AprilTag-only localizer can share one explicit tuning source instead of one side using helper defaults.

2. Add a small always-on maintainer/tester view that prints:
   - source FTC library IDs
   - fixed IDs included in the layout
   - excluded IDs
   This will make season bring-up and future manual checks faster.

3. When Phoenix is updated for a new FTC season, extend `FtcGameTagLayout.officialGameFieldFixed(...)` with the new season's fixed-tag policy instead of letting robot code paper over the difference.

---

## Longer-term upgrades worth considering

1. Latency-compensated fusion:
   apply AprilTag corrections at the measurement timestamp and replay odometry forward, rather than only correcting the current fused state.

2. Better per-observation weighting:
   if FTC exposes richer confidence signals (ambiguity, reprojection error, tag area/corner fit), fold them into the fixed-tag pose solver.

3. Hard field plausibility gates:
   reject impossible field poses outside the playable footprint or clearly inconsistent with the field geometry.

4. Role-specific fixed-tag layouts / whitelists:
   allow clean mode-specific subsets without duplicating robot-side filtering logic.

5. Mixed candidate-set design review:
   today selected-tag references only promote through localization when all candidate IDs are field-fixed.
   If a real use case appears, consider a principled runtime policy that allows localization only when the currently selected tag is fixed.
