package edu.ftcphoenix.fw.drive.guidance;

import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.spatial.SpatialMath2d;

/**
 * Spatial evaluation engine for {@link DriveGuidancePlan}.
 *
 * <p>This class performs the “geometry half” of guidance:</p>
 * <ul>
 *   <li>resolve the configured target into either field-relative or robot-relative geometry</li>
 *   <li>compute translation and omega errors in the appropriate control frames</li>
 *   <li>report whether the solve path is valid for translation, omega, or both</li>
 * </ul>
 *
 * <p>It deliberately does <strong>not</strong> apply PID / feedforward tuning. That happens later
 * in {@link DriveGuidanceCore} via {@link DriveGuidanceControllers}.</p>
 *
 * <h2>Two solve lanes</h2>
 *
 * <p>Guidance can solve references in two different ways:</p>
 * <ol>
 *   <li><b>Field-pose lane</b>: use a {@link PoseEstimator} result plus field-fixed references
 *       (or tag-relative references promoted through {@link TagLayout}).</li>
 *   <li><b>AprilTag lane</b>: use live AprilTag observations directly for tag-relative references,
 *       and derive a temporary live field pose for field-fixed references when fixed tag layout
 *       metadata is available.</li>
 * </ol>
 *
 * <p>That split is what lets one semantic reference work across TeleOp assist, autonomous
 * alignment, and localizer fallback without duplicating the target definition.</p>
 */
final class DriveGuidanceEvaluator {

    private final DriveGuidanceSpec spec;

    /**
     * Anchor pose captured when guidance first enables for
     * {@link DriveGuidanceSpec.RobotRelativePoint} translation targets.
     */
    private Pose2d fieldToTranslationFrameAnchor = null;

    DriveGuidanceEvaluator(DriveGuidanceSpec spec) {
        this.spec = spec;
    }

    /**
     * Resets any per-enable state.
     *
     * <p>Today this only clears the captured robot-relative translation anchor.</p>
     */
    void onEnable() {
        fieldToTranslationFrameAnchor = null;
    }

    /**
     * Returns the captured translation anchor for debugging / status output.
     */
    Pose2d fieldToTranslationFrameAnchor() {
        return fieldToTranslationFrameAnchor;
    }

    /**
     * Solves the current guidance target from the live AprilTag lane.
     *
     * <p>Direct tag-relative references are solved from the currently visible tag(s). If the target
     * is field-fixed and fixed tag layout metadata is available, this method may first derive a
     * temporary live field pose from the visible fixed tag and then reuse the normal field-pose
     * solve path.</p>
     */
    Solution solveWithAprilTags(LoopClock clock) {
        DriveGuidanceSpec.AprilTags cfg = spec.feedback.aprilTags;
        if (cfg == null) {
            return Solution.invalid();
        }

        TagLayout layout = spec.feedback.fixedTagLayout;
        LiveFieldPose liveFieldPose = estimateFieldPoseFromAprilTags(cfg, layout);

        TranslationSolve t = solveTranslationWithAprilTags(cfg, layout, liveFieldPose);
        AimSolve o = solveAimWithAprilTags(cfg, layout, liveFieldPose);

        boolean valid = t.canTranslate || o.canOmega;
        return new Solution(
                valid,
                t.canTranslate,
                o.canOmega,
                t.forwardErrorIn,
                t.leftErrorIn,
                o.omegaErrorRad,
                t.hasRangeInches,
                t.rangeInches
        );
    }

    /**
     * Solves the current guidance target from the field-pose lane.
     */
    Solution solveWithFieldPose() {
        DriveGuidanceSpec.FieldPose cfg = spec.feedback.fieldPose;
        if (cfg == null) {
            return Solution.invalid();
        }

        PoseEstimate est = cfg.poseEstimator.getEstimate();
        boolean valid = est != null
                && est.hasPose
                && est.ageSec <= cfg.maxAgeSec
                && est.quality >= cfg.minQuality;

        if (!valid) {
            return Solution.invalid();
        }

        Pose2d fieldToRobot = est.toPose2d();
        TagLayout layout = spec.feedback.fixedTagLayout;

        TranslationSolve t = solveTranslationFromFieldPose(fieldToRobot, layout, false, Double.NaN);
        AimSolve o = solveAimFromFieldPose(fieldToRobot, layout);

        boolean any = t.canTranslate || o.canOmega;
        return new Solution(
                any,
                t.canTranslate,
                o.canOmega,
                t.forwardErrorIn,
                t.leftErrorIn,
                o.omegaErrorRad,
                t.hasRangeInches,
                t.rangeInches
        );
    }

    private TranslationSolve solveTranslationWithAprilTags(DriveGuidanceSpec.AprilTags cfg,
                                                           TagLayout layout,
                                                           LiveFieldPose liveFieldPose) {
        TranslationSolve direct = solveTranslationDirectFromAprilTags(cfg);
        if (direct.canTranslate) {
            return direct;
        }
        if (liveFieldPose != null) {
            return solveTranslationFromFieldPose(liveFieldPose.fieldToRobot, layout, true, liveFieldPose.rangeInches);
        }
        return TranslationSolve.invalid();
    }

    private AimSolve solveAimWithAprilTags(DriveGuidanceSpec.AprilTags cfg,
                                           TagLayout layout,
                                           LiveFieldPose liveFieldPose) {
        AimSolve direct = solveAimDirectFromAprilTags(cfg);
        if (direct.canOmega) {
            return direct;
        }
        if (liveFieldPose != null) {
            return solveAimFromFieldPose(liveFieldPose.fieldToRobot, layout);
        }
        return AimSolve.invalid();
    }

    /**
     * Solves translation after a field pose is already known.
     *
     * <p>This covers true field-fixed targets and the fallback path for single fixed tag-relative
     * references that can be promoted into field coordinates through {@link TagLayout}.</p>
     */
    private TranslationSolve solveTranslationFromFieldPose(Pose2d fieldToRobot,
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
            fieldToTargetPoint = resolveFieldPointTarget(spec.translationTarget, layout);
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
     * Solves aiming after a field pose is already known.
     */
    private AimSolve solveAimFromFieldPose(Pose2d fieldToRobot, TagLayout layout) {
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
            Pose2d fieldToFrame = References.tryResolveFieldFrame(rh.reference, layout);
            if (fieldToFrame == null) {
                return AimSolve.invalid();
            }
            return new AimSolve(true, Pose2d.wrapToPi((fieldToFrame.headingRad + rh.headingOffsetRad) - fieldToAimFrame.headingRad));
        }

        Pose2d fieldToAimPoint = resolveFieldPointTarget(spec.aimTarget, layout);
        if (fieldToAimPoint == null) {
            return AimSolve.invalid();
        }

        Pose2d aimFrameToPoint = fieldToAimFrame.inverse().then(fieldToAimPoint);
        double omegaErr = Pose2d.wrapToPi(
                SpatialMath2d.bearingRadOfVector(aimFrameToPoint.xInches, aimFrameToPoint.yInches));
        return new AimSolve(true, omegaErr);
    }

    /**
     * Solves translation directly from a tag-relative live reference, without going through a
     * localizer.
     */
    private TranslationSolve solveTranslationDirectFromAprilTags(DriveGuidanceSpec.AprilTags cfg) {
        if (spec.translationTarget == null) {
            return TranslationSolve.invalid();
        }

        Pose2d robotToTargetPoint = resolveRobotPointTargetDirect(spec.translationTarget, cfg);
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
     * Solves aiming directly from a tag-relative live reference, without going through a
     * localizer.
     */
    private AimSolve solveAimDirectFromAprilTags(DriveGuidanceSpec.AprilTags cfg) {
        if (spec.aimTarget == null) {
            return AimSolve.invalid();
        }

        if (spec.aimTarget instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            DriveGuidanceSpec.ReferenceFrameHeadingTarget rh = (DriveGuidanceSpec.ReferenceFrameHeadingTarget) spec.aimTarget;
            Pose2d robotToFrame = resolveRobotFrameDirect(rh.reference, cfg);
            if (robotToFrame == null) {
                return AimSolve.invalid();
            }
            Pose2d robotToAimFrame = spec.controlFrames.robotToAimFrame();
            double omegaErr = Pose2d.wrapToPi((robotToFrame.headingRad + rh.headingOffsetRad) - robotToAimFrame.headingRad);
            return new AimSolve(true, omegaErr);
        }

        Pose2d robotToPoint = resolveRobotPointTargetDirect(spec.aimTarget, cfg);
        if (robotToPoint == null) {
            return AimSolve.invalid();
        }

        Pose2d robotToAimFrame = spec.controlFrames.robotToAimFrame();
        Pose2d aimFrameToPoint = robotToAimFrame.inverse().then(robotToPoint);
        double omegaErr = Pose2d.wrapToPi(
                SpatialMath2d.bearingRadOfVector(aimFrameToPoint.xInches, aimFrameToPoint.yInches));
        return new AimSolve(true, omegaErr);
    }

    /**
     * Resolves a target into a field-fixed point whenever possible.
     *
     * <p>This covers:</p>
     * <ul>
     *   <li>field points</li>
     *   <li>reference points / frames already defined in field coordinates</li>
     *   <li>single-tag references that can be promoted through a fixed {@link TagLayout}</li>
     * </ul>
     */
    private Pose2d resolveFieldPointTarget(Object target, TagLayout layout) {
        if (target == null) {
            return null;
        }

        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            DriveGuidanceSpec.FieldPoint fp = (DriveGuidanceSpec.FieldPoint) target;
            return new Pose2d(fp.xInches, fp.yInches, 0.0);
        }

        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return References.tryResolveFieldPoint(((DriveGuidanceSpec.ReferencePointTarget) target).reference, layout);
        }

        if (target instanceof DriveGuidanceSpec.ReferenceFrameOriginTarget) {
            Pose2d frame = References.tryResolveFieldFrame(((DriveGuidanceSpec.ReferenceFrameOriginTarget) target).reference, layout);
            if (frame == null) {
                return null;
            }
            return new Pose2d(frame.xInches, frame.yInches, 0.0);
        }

        if (target instanceof DriveGuidanceSpec.ReferenceFrameOffsetTarget) {
            DriveGuidanceSpec.ReferenceFrameOffsetTarget ro = (DriveGuidanceSpec.ReferenceFrameOffsetTarget) target;
            Pose2d frame = References.tryResolveFieldFrame(ro.reference, layout);
            if (frame == null) {
                return null;
            }
            Pose2d point = frame.then(new Pose2d(ro.forwardInches, ro.leftInches, 0.0));
            return new Pose2d(point.xInches, point.yInches, 0.0);
        }

        return null;
    }

    /**
     * Resolves a target directly into a robot-relative point from live AprilTags.
     */
    private Pose2d resolveRobotPointTargetDirect(Object target, DriveGuidanceSpec.AprilTags cfg) {
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return resolveRobotPointDirect(((DriveGuidanceSpec.ReferencePointTarget) target).reference, cfg);
        }

        if (target instanceof DriveGuidanceSpec.ReferenceFrameOriginTarget) {
            Pose2d frame = resolveRobotFrameDirect(((DriveGuidanceSpec.ReferenceFrameOriginTarget) target).reference, cfg);
            if (frame == null) {
                return null;
            }
            return new Pose2d(frame.xInches, frame.yInches, 0.0);
        }

        if (target instanceof DriveGuidanceSpec.ReferenceFrameOffsetTarget) {
            DriveGuidanceSpec.ReferenceFrameOffsetTarget ro = (DriveGuidanceSpec.ReferenceFrameOffsetTarget) target;
            Pose2d frame = resolveRobotFrameDirect(ro.reference, cfg);
            if (frame == null) {
                return null;
            }
            Pose2d point = frame.then(new Pose2d(ro.forwardInches, ro.leftInches, 0.0));
            return new Pose2d(point.xInches, point.yInches, 0.0);
        }

        return null;
    }

    /**
     * Resolves a tag-relative semantic point directly from the visible tag(s).
     */
    private Pose2d resolveRobotPointDirect(ReferencePoint2d reference, DriveGuidanceSpec.AprilTags cfg) {
        if (!References.isTagRelativePoint(reference)) {
            return null;
        }
        AprilTagObservation obs = bestForIds(cfg, References.tagIds(reference));
        if (!isValidObservation(obs, cfg.maxAgeSec)) {
            return null;
        }
        Pose3d robotToTag = CameraMountLogic.robotToTagPose(cfg.cameraMount, obs.cameraToTagPose);
        Pose2d tagToPoint = References.tagToPoint(reference);
        if (tagToPoint == null) {
            return null;
        }
        Pose2d robotToPoint = robotToTag.toPose2d().then(tagToPoint);
        return new Pose2d(robotToPoint.xInches, robotToPoint.yInches, 0.0);
    }

    /**
     * Resolves a tag-relative semantic frame directly from the visible tag(s).
     */
    private Pose2d resolveRobotFrameDirect(ReferenceFrame2d reference, DriveGuidanceSpec.AprilTags cfg) {
        if (!References.isTagRelativeFrame(reference)) {
            return null;
        }
        AprilTagObservation obs = bestForIds(cfg, References.tagIds(reference));
        if (!isValidObservation(obs, cfg.maxAgeSec)) {
            return null;
        }
        Pose3d robotToTag = CameraMountLogic.robotToTagPose(cfg.cameraMount, obs.cameraToTagPose);
        Pose2d tagToFrame = References.tagToFrame(reference);
        if (tagToFrame == null) {
            return null;
        }
        return robotToTag.toPose2d().then(tagToFrame);
    }

    /**
     * Derives a temporary live field pose from a visible fixed AprilTag.
     *
     * <p>This is the bridge that lets field-fixed references be solved directly from AprilTags
     * without requiring a separate fused localizer.</p>
     */
    private LiveFieldPose estimateFieldPoseFromAprilTags(DriveGuidanceSpec.AprilTags cfg, TagLayout layout) {
        if (cfg == null || layout == null || layout.ids().isEmpty()) {
            return null;
        }

        AprilTagObservation obs = bestForIds(cfg, layout.ids());
        if (!isValidObservation(obs, cfg.maxAgeSec)) {
            return null;
        }

        Pose3d fieldToTag = layout.getFieldToTagPose(obs.id);
        if (fieldToTag == null) {
            return null;
        }

        Pose3d robotToTag = cfg.cameraMount.robotToCameraPose().then(obs.cameraToTagPose);
        Pose3d fieldToRobot = fieldToTag.then(robotToTag.inverse());
        return new LiveFieldPose(fieldToRobot.toPose2d(), obs.cameraRangeInches());
    }

    /**
     * Chooses the best live observation among the supplied candidate IDs.
     */
    private static AprilTagObservation bestForIds(DriveGuidanceSpec.AprilTags cfg, Set<Integer> ids) {
        if (cfg == null || ids == null || ids.isEmpty()) {
            return AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        }
        return cfg.sensor.best(ids, cfg.maxAgeSec);
    }

    /**
     * Freshness / presence filter shared by the direct solve paths.
     */
    private static boolean isValidObservation(AprilTagObservation obs, double maxAgeSec) {
        return obs != null && obs.hasTarget && obs.ageSec <= maxAgeSec;
    }

    /**
     * Geometry-only solve result produced before controller tuning is applied.
     */
    static final class Solution {
        final boolean valid;
        final boolean canTranslate;
        final boolean canOmega;
        final double forwardErrorIn;
        final double leftErrorIn;
        final double omegaErrorRad;
        final boolean hasRangeInches;
        final double rangeInches;

        Solution(boolean valid,
                 boolean canTranslate,
                 boolean canOmega,
                 double forwardErrorIn,
                 double leftErrorIn,
                 double omegaErrorRad,
                 boolean hasRangeInches,
                 double rangeInches) {
            this.valid = valid;
            this.canTranslate = canTranslate;
            this.canOmega = canOmega;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.omegaErrorRad = omegaErrorRad;
            this.hasRangeInches = hasRangeInches;
            this.rangeInches = rangeInches;
        }

        static Solution invalid() {
            return new Solution(false, false, false, 0.0, 0.0, 0.0, false, Double.NaN);
        }
    }

    /**
     * Translation-only portion of a solve result.
     */
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

    /**
     * Aim-only portion of a solve result.
     */
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

    /**
     * Temporary live field pose derived from a visible fixed tag.
     */
    private static final class LiveFieldPose {
        final Pose2d fieldToRobot;
        final double rangeInches;

        LiveFieldPose(Pose2d fieldToRobot, double rangeInches) {
            this.fieldToRobot = fieldToRobot;
            this.rangeInches = rangeInches;
        }
    }
}
