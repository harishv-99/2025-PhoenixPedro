package edu.ftcphoenix.fw.drive.guidance;

import java.util.Collections;
import java.util.List;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.spatial.SpatialMath2d;

/**
 * Spatial evaluation engine for {@link DriveGuidancePlan}.
 *
 * <p>This class performs the geometry half of guidance: resolve the target, compute translation
 * and omega errors, and report whether localization and/or live AprilTags could solve the
 * requested channel(s) this loop.</p>
 */
final class DriveGuidanceEvaluator {

    private static final TagSelectionResult NO_SELECTION =
            TagSelectionResult.none(Collections.<Integer>emptySet());

    private final DriveGuidanceSpec spec;
    private Pose2d fieldToTranslationFrameAnchor = null;

    /**
     * Creates an evaluator for one immutable guidance spec.
     */
    DriveGuidanceEvaluator(DriveGuidanceSpec spec) {
        this.spec = spec;
    }

    /**
     * Resets runtime state that is captured on enable, such as robot-relative translation anchors
     * and sticky selected-tag references.
     */
    void onEnable() {
        fieldToTranslationFrameAnchor = null;
        resetSelections(spec.translationTarget);
        resetSelections(spec.aimTarget);
    }

    /**
     * Returns the currently latched field-space translation-frame anchor used by robot-relative
     * translation targets, or {@code null} when no anchor has been captured yet.
     */
    Pose2d fieldToTranslationFrameAnchor() {
        return fieldToTranslationFrameAnchor;
    }

    /**
     * Attempts to solve the configured targets from the live AprilTag lane, with optional fallback
     * through a temporary live field pose derived from fixed field tags.
     */
    Solution solveWithAprilTags(LoopClock clock) {
        DriveGuidanceSpec.AprilTags cfg = spec.resolveWith.aprilTags;
        if (cfg == null) {
            return Solution.invalid();
        }

        TagLayout layout = spec.resolveWith.fixedAprilTagLayout;
        LiveFieldPose liveFieldPose = estimateFieldPoseFromAprilTags(clock, cfg, layout);

        TranslationSolve t = solveTranslationWithAprilTags(clock, cfg, layout, liveFieldPose);
        AimSolve o = solveAimWithAprilTags(clock, cfg, layout, liveFieldPose);

        boolean valid = t.canTranslate || o.canOmega;
        return new Solution(
                valid,
                t.canTranslate,
                o.canOmega,
                t.forwardErrorIn,
                t.leftErrorIn,
                o.omegaErrorRad,
                t.hasRangeInches,
                t.rangeInches,
                translationSelectionSnapshot(clock, cfg),
                aimSelectionSnapshot(clock, cfg)
        );
    }

    /**
     * Attempts to solve the configured targets from the localization lane.
     */
    Solution solveWithLocalization(LoopClock clock) {
        DriveGuidanceSpec.Localization cfg = spec.resolveWith.localization;
        if (cfg == null) {
            return Solution.invalid();
        }

        PoseEstimate est = cfg.poseEstimator.getEstimate();
        double estAgeSec = (est != null)
                ? Math.max(est.ageSec, clock.nowSec() - est.timestampSec)
                : Double.POSITIVE_INFINITY;
        boolean valid = est != null
                && est.hasPose
                && estAgeSec <= cfg.maxAgeSec
                && est.quality >= cfg.minQuality;
        if (!valid) {
            return Solution.invalid();
        }

        Pose2d fieldToRobot = est.toPose2d();
        TagLayout layout = spec.resolveWith.fixedAprilTagLayout;

        TranslationSolve t = solveTranslationFromLocalization(clock, fieldToRobot, layout, false, Double.NaN);
        AimSolve o = solveAimFromLocalization(clock, fieldToRobot, layout);

        boolean any = t.canTranslate || o.canOmega;
        return new Solution(
                any,
                t.canTranslate,
                o.canOmega,
                t.forwardErrorIn,
                t.leftErrorIn,
                o.omegaErrorRad,
                t.hasRangeInches,
                t.rangeInches,
                translationSelectionSnapshot(clock, null),
                aimSelectionSnapshot(clock, null)
        );
    }

    /**
     * Solves translation from live tag geometry first, then falls back to a live field-pose bridge
     * when enough fixed tags are visible.
     */
    private TranslationSolve solveTranslationWithAprilTags(LoopClock clock,
                                                           DriveGuidanceSpec.AprilTags cfg,
                                                           TagLayout layout,
                                                           LiveFieldPose liveFieldPose) {
        TranslationSolve direct = solveTranslationDirectFromAprilTags(clock, cfg);
        if (direct.canTranslate) {
            return direct;
        }
        if (liveFieldPose != null) {
            return solveTranslationFromLocalization(clock, liveFieldPose.fieldToRobot, layout, true, liveFieldPose.rangeInches);
        }
        return TranslationSolve.invalid();
    }

    /**
     * Solves omega from live tag geometry first, then falls back to a live field-pose bridge when
     * enough fixed tags are visible.
     */
    private AimSolve solveAimWithAprilTags(LoopClock clock,
                                           DriveGuidanceSpec.AprilTags cfg,
                                           TagLayout layout,
                                           LiveFieldPose liveFieldPose) {
        AimSolve direct = solveAimDirectFromAprilTags(clock, cfg);
        if (direct.canOmega) {
            return direct;
        }
        if (liveFieldPose != null) {
            return solveAimFromLocalization(clock, liveFieldPose.fieldToRobot, layout);
        }
        return AimSolve.invalid();
    }

    /**
     * Solves translation from a field-centric robot pose, promoting semantic references through the
     * fixed AprilTag layout when necessary.
     */
    private TranslationSolve solveTranslationFromLocalization(LoopClock clock,
                                                              Pose2d fieldToRobot,
                                                              TagLayout layout,
                                                              boolean hasRange,
                                                              double rangeInches) {
        Pose2d fieldToTFrame = fieldToRobot.then(spec.controlFrames.robotToTranslationFrame());

        Pose2d fieldToTargetPoint;
        if (spec.translationTarget instanceof DriveGuidanceSpec.RobotRelativePoint) {
            DriveGuidanceSpec.RobotRelativePoint rr = (DriveGuidanceSpec.RobotRelativePoint) spec.translationTarget;
            if (fieldToTranslationFrameAnchor == null) {
                fieldToTranslationFrameAnchor = fieldToTFrame;
            }
            fieldToTargetPoint = fieldToTranslationFrameAnchor.then(new Pose2d(rr.forwardInches, rr.leftInches, 0.0));
        } else {
            fieldToTargetPoint = resolveFieldPointTarget(spec.translationTarget, layout, clock);
        }

        if (fieldToTargetPoint == null) {
            return TranslationSolve.invalid();
        }

        double dxField = fieldToTargetPoint.xInches - fieldToTFrame.xInches;
        double dyField = fieldToTargetPoint.yInches - fieldToTFrame.yInches;

        double h = fieldToRobot.headingRad;
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double forwardErr = dxField * cos + dyField * sin;
        double leftErr = -dxField * sin + dyField * cos;

        return new TranslationSolve(true, forwardErr, leftErr, hasRange, rangeInches);
    }

    /**
     * Solves omega from a field-centric robot pose.
     */
    private AimSolve solveAimFromLocalization(LoopClock clock, Pose2d fieldToRobot, TagLayout layout) {
        if (spec.aimTarget == null) {
            return AimSolve.invalid();
        }

        Pose2d fieldToAimFrame = fieldToRobot.then(spec.controlFrames.robotToAimFrame());

        if (spec.aimTarget instanceof DriveGuidanceSpec.FieldHeading) {
            DriveGuidanceSpec.FieldHeading fh = (DriveGuidanceSpec.FieldHeading) spec.aimTarget;
            return new AimSolve(true, Pose2d.wrapToPi(fh.fieldHeadingRad - fieldToAimFrame.headingRad));
        }

        if (spec.aimTarget instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            DriveGuidanceSpec.ReferenceFrameHeadingTarget rh = (DriveGuidanceSpec.ReferenceFrameHeadingTarget) spec.aimTarget;
            Pose2d fieldToFrame = References.tryResolveFieldFrame(rh.reference, layout, clock);
            if (fieldToFrame == null) {
                return AimSolve.invalid();
            }
            return new AimSolve(true, Pose2d.wrapToPi((fieldToFrame.headingRad + rh.headingOffsetRad) - fieldToAimFrame.headingRad));
        }

        Pose2d fieldToAimPoint = resolveFieldPointTarget(spec.aimTarget, layout, clock);
        if (fieldToAimPoint == null) {
            return AimSolve.invalid();
        }

        Pose2d aimFrameToPoint = fieldToAimFrame.inverse().then(fieldToAimPoint);
        double omegaErr = Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(aimFrameToPoint.xInches, aimFrameToPoint.yInches));
        return new AimSolve(true, omegaErr);
    }

    /**
     * Attempts to solve translation directly from live tag-relative geometry without first deriving
     * a field pose.
     */
    private TranslationSolve solveTranslationDirectFromAprilTags(LoopClock clock, DriveGuidanceSpec.AprilTags cfg) {
        if (!(spec.translationTarget instanceof DriveGuidanceSpec.ReferencePointTarget)) {
            return TranslationSolve.invalid();
        }

        Pose2d robotToTargetPoint = resolveRobotPointDirect(clock,
                ((DriveGuidanceSpec.ReferencePointTarget) spec.translationTarget).reference,
                cfg);
        if (robotToTargetPoint == null) {
            return TranslationSolve.invalid();
        }

        Pose2d robotToTFrame = spec.controlFrames.robotToTranslationFrame();
        double forwardErr = robotToTargetPoint.xInches - robotToTFrame.xInches;
        double leftErr = robotToTargetPoint.yInches - robotToTFrame.yInches;
        double range = Math.hypot(robotToTargetPoint.xInches, robotToTargetPoint.yInches);
        return new TranslationSolve(true, forwardErr, leftErr, Double.isFinite(range), range);
    }

    /**
     * Attempts to solve omega directly from live tag-relative geometry without first deriving a
     * field pose.
     */
    private AimSolve solveAimDirectFromAprilTags(LoopClock clock, DriveGuidanceSpec.AprilTags cfg) {
        if (spec.aimTarget == null) {
            return AimSolve.invalid();
        }

        if (spec.aimTarget instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            DriveGuidanceSpec.ReferenceFrameHeadingTarget rh = (DriveGuidanceSpec.ReferenceFrameHeadingTarget) spec.aimTarget;
            Pose2d robotToFrame = resolveRobotFrameDirect(clock, rh.reference, cfg);
            if (robotToFrame == null) {
                return AimSolve.invalid();
            }
            Pose2d robotToAimFrame = spec.controlFrames.robotToAimFrame();
            double omegaErr = Pose2d.wrapToPi((robotToFrame.headingRad + rh.headingOffsetRad) - robotToAimFrame.headingRad);
            return new AimSolve(true, omegaErr);
        }

        if (!(spec.aimTarget instanceof DriveGuidanceSpec.ReferencePointTarget)) {
            return AimSolve.invalid();
        }

        Pose2d robotToPoint = resolveRobotPointDirect(clock,
                ((DriveGuidanceSpec.ReferencePointTarget) spec.aimTarget).reference,
                cfg);
        if (robotToPoint == null) {
            return AimSolve.invalid();
        }

        Pose2d robotToAimFrame = spec.controlFrames.robotToAimFrame();
        Pose2d aimFrameToPoint = robotToAimFrame.inverse().then(robotToPoint);
        double omegaErr = Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(aimFrameToPoint.xInches, aimFrameToPoint.yInches));
        return new AimSolve(true, omegaErr);
    }

    /**
     * Resolves a translation/aim point target into field coordinates when possible.
     */
    private Pose2d resolveFieldPointTarget(Object target, TagLayout layout, LoopClock clock) {
        if (target == null) {
            return null;
        }
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            DriveGuidanceSpec.FieldPoint fp = (DriveGuidanceSpec.FieldPoint) target;
            return new Pose2d(fp.xInches, fp.yInches, 0.0);
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return References.tryResolveFieldPoint(((DriveGuidanceSpec.ReferencePointTarget) target).reference, layout, clock);
        }
        return null;
    }

    /**
     * Resolves a reference point directly into the robot frame from live AprilTag observations.
     */
    private Pose2d resolveRobotPointDirect(LoopClock clock,
                                           ReferencePoint2d reference,
                                           DriveGuidanceSpec.AprilTags cfg) {
        if (reference instanceof References.FieldPointRef) {
            return null;
        }
        if (reference instanceof References.FramePointRef) {
            References.FramePointRef fp = (References.FramePointRef) reference;
            Pose2d robotToFrame = resolveRobotFrameDirect(clock, fp.frame, cfg);
            return robotToFrame != null ? robotToFrame.then(new Pose2d(fp.forwardInches, fp.leftInches, 0.0)) : null;
        }
        if (reference instanceof References.TagPointRef) {
            References.TagPointRef tp = (References.TagPointRef) reference;
            AprilTagObservation obs = observationForId(clock, cfg, tp.tagId);
            return composeRobotPointFromObservation(obs, cfg, new Pose2d(tp.forwardInches, tp.leftInches, 0.0));
        }
        if (reference instanceof References.SelectedTagPointRef) {
            References.SelectedTagPointRef sp = (References.SelectedTagPointRef) reference;
            TagSelectionResult sel = sp.selection.get(clock);
            if (sel == null || !sel.hasFreshSelectedObservation) {
                return null;
            }
            Pose2d tagToPoint = sp.lookup.tagToPoint(sel.selectedTagId);
            return composeRobotPointFromObservation(sel.selectedObservation, cfg, tagToPoint);
        }
        return null;
    }

    /**
     * Resolves a reference frame directly into the robot frame from live AprilTag observations.
     */
    private Pose2d resolveRobotFrameDirect(LoopClock clock,
                                           ReferenceFrame2d reference,
                                           DriveGuidanceSpec.AprilTags cfg) {
        if (reference instanceof References.FieldFrameRef) {
            return null;
        }
        if (reference instanceof References.TagFrameRef) {
            References.TagFrameRef tf = (References.TagFrameRef) reference;
            AprilTagObservation obs = observationForId(clock, cfg, tf.tagId);
            return composeRobotPointFromObservation(obs, cfg, new Pose2d(tf.forwardInches, tf.leftInches, tf.headingRad));
        }
        if (reference instanceof References.SelectedTagFrameRef) {
            References.SelectedTagFrameRef sf = (References.SelectedTagFrameRef) reference;
            TagSelectionResult sel = sf.selection.get(clock);
            if (sel == null || !sel.hasFreshSelectedObservation) {
                return null;
            }
            Pose2d tagToFrame = sf.lookup.tagToFrame(sel.selectedTagId);
            return composeRobotPointFromObservation(sel.selectedObservation, cfg, tagToFrame);
        }
        return null;
    }

    /**
     * Composes a robot-frame point or frame pose from one camera observation plus a tag-local
     * offset.
     */
    private Pose2d composeRobotPointFromObservation(AprilTagObservation obs,
                                                    DriveGuidanceSpec.AprilTags cfg,
                                                    Pose2d tagToThing) {
        if (obs == null || !obs.hasTarget || tagToThing == null) {
            return null;
        }
        Pose3d robotToTag = CameraMountLogic.robotToTagPose(cfg.cameraMount, obs.cameraToTagPose);
        Pose2d robotToThing = robotToTag.toPose2d().then(tagToThing);
        return new Pose2d(robotToThing.xInches, robotToThing.yInches, robotToThing.headingRad);
    }

    /**
     * Estimates a temporary field-centric robot pose from all fresh fixed-tag observations visible
     * in the current frame.
     *
     * <p>This intentionally reuses the same weighted multi-tag solver as the AprilTag-only
     * localizer so the guidance lane and the localization lane do not diverge.</p>
     */
    private LiveFieldPose estimateFieldPoseFromAprilTags(LoopClock clock,
                                                         DriveGuidanceSpec.AprilTags cfg,
                                                         TagLayout layout) {
        if (cfg == null || layout == null || layout.ids().isEmpty()) {
            return null;
        }

        AprilTagDetections dets = cfg.sensor.get(clock);
        if (dets == null) {
            return null;
        }
        List<AprilTagObservation> observations = dets.freshMatching(layout.ids(), cfg.maxAgeSec);
        if (observations.isEmpty()) {
            return null;
        }

        FixedTagFieldPoseSolver.Result solve = FixedTagFieldPoseSolver.solve(
                observations,
                layout,
                cfg.cameraMount,
                cfg.fieldPoseSolverConfig
        );
        if (!solve.hasPose) {
            return null;
        }

        return new LiveFieldPose(solve.toPose2d(), solve.rangeInches);
    }

    /**
     * Looks up one fresh AprilTag observation by ID, returning a no-target sentinel when absent.
     */
    private AprilTagObservation observationForId(LoopClock clock,
                                                 DriveGuidanceSpec.AprilTags cfg,
                                                 int id) {
        AprilTagDetections dets = cfg.sensor.get(clock);
        return dets != null ? dets.forId(id, cfg.maxAgeSec) : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
    }

    /**
     * Returns the translation reference's current selection snapshot for status/telemetry.
     */
    private TagSelectionResult translationSelectionSnapshot(LoopClock clock, DriveGuidanceSpec.AprilTags cfg) {
        if (!(spec.translationTarget instanceof DriveGuidanceSpec.ReferencePointTarget)) {
            return NO_SELECTION;
        }
        return pointSelectionSnapshot(((DriveGuidanceSpec.ReferencePointTarget) spec.translationTarget).reference, clock, cfg);
    }

    /**
     * Returns the aim reference's current selection snapshot for status/telemetry.
     */
    private TagSelectionResult aimSelectionSnapshot(LoopClock clock, DriveGuidanceSpec.AprilTags cfg) {
        if (spec.aimTarget instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return pointSelectionSnapshot(((DriveGuidanceSpec.ReferencePointTarget) spec.aimTarget).reference, clock, cfg);
        }
        if (spec.aimTarget instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            return frameSelectionSnapshot(((DriveGuidanceSpec.ReferenceFrameHeadingTarget) spec.aimTarget).reference, clock, cfg);
        }
        return NO_SELECTION;
    }

    /**
     * Produces a selection snapshot for a reference point, including fixed-tag and selected-tag
     * cases.
     */
    private TagSelectionResult pointSelectionSnapshot(ReferencePoint2d ref,
                                                      LoopClock clock,
                                                      DriveGuidanceSpec.AprilTags cfg) {
        if (ref instanceof References.FieldPointRef) {
            return NO_SELECTION;
        }
        if (ref instanceof References.FramePointRef) {
            return frameSelectionSnapshot(((References.FramePointRef) ref).frame, clock, cfg);
        }
        if (ref instanceof References.TagPointRef) {
            References.TagPointRef tp = (References.TagPointRef) ref;
            AprilTagObservation obs = cfg != null ? observationForId(clock, cfg, tp.tagId) : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            return fixedTagSelection(tp.tagId, obs);
        }
        if (ref instanceof References.SelectedTagPointRef) {
            TagSelectionResult sel = ((References.SelectedTagPointRef) ref).selection.get(clock);
            return sel != null ? sel : NO_SELECTION;
        }
        return NO_SELECTION;
    }

    /**
     * Produces a selection snapshot for a reference frame, including fixed-tag and selected-tag
     * cases.
     */
    private TagSelectionResult frameSelectionSnapshot(ReferenceFrame2d ref,
                                                      LoopClock clock,
                                                      DriveGuidanceSpec.AprilTags cfg) {
        if (ref instanceof References.FieldFrameRef) {
            return NO_SELECTION;
        }
        if (ref instanceof References.TagFrameRef) {
            References.TagFrameRef tf = (References.TagFrameRef) ref;
            AprilTagObservation obs = cfg != null ? observationForId(clock, cfg, tf.tagId) : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            return fixedTagSelection(tf.tagId, obs);
        }
        if (ref instanceof References.SelectedTagFrameRef) {
            TagSelectionResult sel = ((References.SelectedTagFrameRef) ref).selection.get(clock);
            return sel != null ? sel : NO_SELECTION;
        }
        return NO_SELECTION;
    }

    /**
     * Builds a synthetic selection snapshot for a direct single-tag reference.
     */
    private static TagSelectionResult fixedTagSelection(int tagId, AprilTagObservation obs) {
        boolean fresh = obs != null && obs.hasTarget;
        Set<Integer> visibleIds = fresh ? Collections.singleton(tagId) : Collections.<Integer>emptySet();
        AprilTagObservation shown = fresh ? obs : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        return new TagSelectionResult(
                fresh,
                tagId,
                shown,
                true,
                tagId,
                false,
                fresh,
                shown,
                visibleIds,
                "fixedTagReference",
                "fixed tag reference",
                Double.NaN
        );
    }

    /**
     * Clears any sticky selector state referenced by the supplied target.
     */
    private void resetSelections(Object target) {
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            resetReferenceSelection(((DriveGuidanceSpec.ReferencePointTarget) target).reference);
        } else if (target instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            resetReferenceSelection(((DriveGuidanceSpec.ReferenceFrameHeadingTarget) target).reference);
        }
    }

    /**
     * Resets selection state nested under a point reference.
     */
    private void resetReferenceSelection(ReferencePoint2d ref) {
        if (ref instanceof References.SelectedTagPointRef) {
            ((References.SelectedTagPointRef) ref).selection.reset();
        } else if (ref instanceof References.FramePointRef) {
            resetReferenceSelection(((References.FramePointRef) ref).frame);
        }
    }

    /**
     * Resets selection state nested under a frame reference.
     */
    private void resetReferenceSelection(ReferenceFrame2d ref) {
        if (ref instanceof References.SelectedTagFrameRef) {
            ((References.SelectedTagFrameRef) ref).selection.reset();
        }
    }

    static final class Solution {
        final boolean valid;
        final boolean canTranslate;
        final boolean canOmega;
        final double forwardErrorIn;
        final double leftErrorIn;
        final double omegaErrorRad;
        final boolean hasRangeInches;
        final double rangeInches;
        final TagSelectionResult translationSelection;
        final TagSelectionResult aimSelection;

        Solution(boolean valid,
                 boolean canTranslate,
                 boolean canOmega,
                 double forwardErrorIn,
                 double leftErrorIn,
                 double omegaErrorRad,
                 boolean hasRangeInches,
                 double rangeInches,
                 TagSelectionResult translationSelection,
                 TagSelectionResult aimSelection) {
            this.valid = valid;
            this.canTranslate = canTranslate;
            this.canOmega = canOmega;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.omegaErrorRad = omegaErrorRad;
            this.hasRangeInches = hasRangeInches;
            this.rangeInches = rangeInches;
            this.translationSelection = translationSelection != null ? translationSelection : NO_SELECTION;
            this.aimSelection = aimSelection != null ? aimSelection : NO_SELECTION;
        }

        static Solution invalid() {
            return new Solution(false, false, false, 0.0, 0.0, 0.0, false, Double.NaN, NO_SELECTION, NO_SELECTION);
        }
    }

    private static final class TranslationSolve {
        final boolean canTranslate;
        final double forwardErrorIn;
        final double leftErrorIn;
        final boolean hasRangeInches;
        final double rangeInches;

        TranslationSolve(boolean canTranslate,
                         double forwardErrorIn,
                         double leftErrorIn,
                         boolean hasRangeInches,
                         double rangeInches) {
            this.canTranslate = canTranslate;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.hasRangeInches = hasRangeInches;
            this.rangeInches = rangeInches;
        }

        static TranslationSolve invalid() {
            return new TranslationSolve(false, 0.0, 0.0, false, Double.NaN);
        }
    }

    private static final class AimSolve {
        final boolean canOmega;
        final double omegaErrorRad;

        AimSolve(boolean canOmega, double omegaErrorRad) {
            this.canOmega = canOmega;
            this.omegaErrorRad = omegaErrorRad;
        }

        static AimSolve invalid() {
            return new AimSolve(false, 0.0);
        }
    }

    private static final class LiveFieldPose {
        final Pose2d fieldToRobot;
        final double rangeInches;

        LiveFieldPose(Pose2d fieldToRobot, double rangeInches) {
            this.fieldToRobot = fieldToRobot;
            this.rangeInches = rangeInches;
        }
    }
}
