package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.observation.TargetObservation2d;
import edu.ftcphoenix.fw.spatial.SpatialMath2d;

/**
 * Spatial evaluation for {@link DriveGuidancePlan}.
 *
 * <p><b>What this class is:</b> the “spatial math” layer for DriveGuidance. It converts
 * feedback (field pose and/or observation) into <b>errors</b> relative to the plan’s
 * targets. It does <em>not</em> apply tuning or create drive commands.</p>
 *
 * <p><b>What this class is not:</b> a controller. The controller layer lives in
 * {@link DriveGuidanceOverlay} + {@link DriveGuidanceControllers}.</p>
 */
final class DriveGuidanceEvaluator {

    private final DriveGuidanceSpec spec;

    // For “observed tag” targets (tagId = -1), remember the last seen tag ID so field-pose mode
    // can keep working even if the tag temporarily drops out of view.
    private int lastObservedTagId = -1;

    // For robot-relative translation targets, capture the translation-frame pose when the
    // overlay becomes enabled. This allows "move forward N inches" style plans.
    private Pose2d fieldToTranslationFrameAnchor = null;

    DriveGuidanceEvaluator(DriveGuidanceSpec spec) {
        this.spec = spec;
    }

    void onEnable() {
        lastObservedTagId = -1;
        fieldToTranslationFrameAnchor = null;
    }

    int lastObservedTagId() {
        return lastObservedTagId;
    }

    Pose2d fieldToTranslationFrameAnchor() {
        return fieldToTranslationFrameAnchor;
    }

    Solution solveWithObservation(LoopClock clock) {
        DriveGuidanceSpec.Observation cfg = spec.feedback.observation;
        TargetObservation2d obs = cfg.source.sample(clock);

        if (obs == null) {
            return Solution.invalid();
        }

        boolean valid = obs.hasTarget
                && obs.ageSec <= cfg.maxAgeSec
                && obs.quality >= cfg.minQuality;

        if (!valid) {
            return Solution.invalid();
        }

        if (obs.hasTargetId()) {
            lastObservedTagId = obs.targetId;
        }

        // Observation feedback is robot-relative.
        boolean hasPos = obs.hasPosition();
        Pose2d robotToAnchorPose = hasPos
                ? new Pose2d(obs.forwardInches, obs.leftInches, obs.hasOrientation() ? obs.targetHeadingRad : 0.0)
                : null;

        double rangeIn = hasPos ? Math.hypot(obs.forwardInches, obs.leftInches) : Double.NaN;
        boolean hasRange = Double.isFinite(rangeIn);

        // Compute errors.
        double forwardErr = 0.0;
        double leftErr = 0.0;
        double omegaErr = 0.0;
        boolean canTranslate = false;
        boolean canOmega = false;

        // --- Translation ---
        if (spec.translationTarget instanceof DriveGuidanceSpec.TagRelativePoint) {
            DriveGuidanceSpec.TagRelativePoint tp = (DriveGuidanceSpec.TagRelativePoint) spec.translationTarget;

            boolean idMatches = (tp.tagId < 0)
                    || (obs.hasTargetId() && obs.targetId == tp.tagId);

            boolean needsOrientation = !(Math.abs(tp.forwardInches) < 1e-9 && Math.abs(tp.leftInches) < 1e-9);
            boolean orientationOk = !needsOrientation || obs.hasOrientation();

            if (idMatches && hasPos && orientationOk && robotToAnchorPose != null) {
                Pose2d robotToTarget = SpatialMath2d.anchorRelativePointInches(robotToAnchorPose, tp.forwardInches, tp.leftInches);

                Pose2d robotToTFrame = spec.controlFrames.robotToTranslationFrame();
                forwardErr = robotToTarget.xInches - robotToTFrame.xInches;
                leftErr = robotToTarget.yInches - robotToTFrame.yInches;
                canTranslate = true;
            }
        }

        // --- Omega / Aim ---
        if (spec.aimTarget instanceof DriveGuidanceSpec.FieldHeading
                || spec.aimTarget instanceof DriveGuidanceSpec.TagHeading) {
            // Cannot solve absolute field headings from observation-only feedback.
            canOmega = false;
        } else if (spec.aimTarget instanceof DriveGuidanceSpec.TagRelativePoint) {
            DriveGuidanceSpec.TagRelativePoint tp = (DriveGuidanceSpec.TagRelativePoint) spec.aimTarget;

            boolean idMatches = (tp.tagId < 0)
                    || (obs.hasTargetId() && obs.targetId == tp.tagId);

            boolean needsOrientation = !(Math.abs(tp.forwardInches) < 1e-9 && Math.abs(tp.leftInches) < 1e-9);
            boolean orientationOk = !needsOrientation || obs.hasOrientation();

            Pose2d robotToAimFrame = spec.controlFrames.robotToAimFrame();

            // If we have position, compute the true vector from the aim frame origin.
            if (idMatches && hasPos && orientationOk && robotToAnchorPose != null) {
                Pose2d robotToPoint = SpatialMath2d.anchorRelativePointInches(robotToAnchorPose, tp.forwardInches, tp.leftInches);
                Pose2d aimFrameToPoint = robotToAimFrame.inverse().then(robotToPoint);
                omegaErr = Pose2d.wrapToPi(
                        SpatialMath2d.bearingRadOfVector(aimFrameToPoint.xInches, aimFrameToPoint.yInches));
                canOmega = true;
            } else {
                // Bearing-only fallback: only safe when the aim frame origin is the robot origin
                // and we are aiming at the anchor center (forward=0,left=0).
                boolean aimingAtCenter = Math.abs(tp.forwardInches) < 1e-9 && Math.abs(tp.leftInches) < 1e-9;
                boolean aimFrameAtOrigin = Math.abs(robotToAimFrame.xInches) < 1e-9
                        && Math.abs(robotToAimFrame.yInches) < 1e-9;

                if (idMatches && aimingAtCenter && aimFrameAtOrigin) {
                    omegaErr = Pose2d.wrapToPi(obs.bearingRad - robotToAimFrame.headingRad);
                    canOmega = true;
                }
            }
        } else if (spec.aimTarget instanceof DriveGuidanceSpec.FieldPoint) {
            // A field point cannot be resolved from observation-only feedback.
            canOmega = false;
        }

        return new Solution(true, canTranslate, canOmega, forwardErr, leftErr, omegaErr, hasRange, rangeIn);
    }

    Solution solveWithFieldPose() {
        DriveGuidanceSpec.FieldPose cfg = spec.feedback.fieldPose;
        PoseEstimate est = cfg.poseEstimator.getEstimate();

        boolean valid = est != null
                && est.hasPose
                && est.ageSec <= cfg.maxAgeSec
                && est.quality >= cfg.minQuality;

        if (!valid) {
            return Solution.invalid();
        }

        Pose2d fieldToRobot = est.toPose2d();
        TagLayout layout = cfg.tagLayout;

        // Current controlled-frame poses.
        Pose2d fieldToTFrame = fieldToRobot.then(spec.controlFrames.robotToTranslationFrame());

        // Resolve translation target.
        Pose2d fieldToTranslatePoint;
        if (spec.translationTarget instanceof DriveGuidanceSpec.RobotRelativePoint) {
            DriveGuidanceSpec.RobotRelativePoint rr = (DriveGuidanceSpec.RobotRelativePoint) spec.translationTarget;

            // Capture the "starting" translation-frame pose once per enable cycle.
            if (fieldToTranslationFrameAnchor == null) {
                fieldToTranslationFrameAnchor = fieldToTFrame;
            }

            fieldToTranslatePoint = fieldToTranslationFrameAnchor.then(new Pose2d(rr.forwardInches, rr.leftInches, 0.0));
        } else {
            fieldToTranslatePoint = resolveToFieldPoint(spec.translationTarget, layout);
        }

        // Resolve aim target (some targets are handled as non-point targets below).
        Pose2d fieldToAimPoint = ((spec.aimTarget instanceof DriveGuidanceSpec.FieldHeading)
                || (spec.aimTarget instanceof DriveGuidanceSpec.TagHeading))
                ? null
                : resolveToFieldPoint(spec.aimTarget, layout);

        double forwardErr = 0.0;
        double leftErr = 0.0;
        double omegaErr = 0.0;

        boolean canTranslate = false;
        boolean canOmega = false;

        // --- Translation ---
        if (fieldToTranslatePoint != null) {
            // Field error vector from translation-frame origin to target point.
            double dxField = fieldToTranslatePoint.xInches - fieldToTFrame.xInches;
            double dyField = fieldToTranslatePoint.yInches - fieldToTFrame.yInches;

            // Rotate into robot frame.
            double h = fieldToRobot.headingRad;
            double cos = Math.cos(h);
            double sin = Math.sin(h);
            forwardErr = dxField * cos + dyField * sin;
            leftErr = -dxField * sin + dyField * cos;
            canTranslate = true;
        }

        // --- Omega / Aim ---
        if (spec.aimTarget instanceof DriveGuidanceSpec.FieldHeading) {
            DriveGuidanceSpec.FieldHeading fh = (DriveGuidanceSpec.FieldHeading) spec.aimTarget;
            Pose2d fieldToAimFrame = fieldToRobot.then(spec.controlFrames.robotToAimFrame());
            omegaErr = Pose2d.wrapToPi(fh.fieldHeadingRad - fieldToAimFrame.headingRad);
            canOmega = true;
        } else if (spec.aimTarget instanceof DriveGuidanceSpec.TagHeading) {
            if (layout != null) {
                DriveGuidanceSpec.TagHeading th = (DriveGuidanceSpec.TagHeading) spec.aimTarget;
                int tagId = (th.tagId >= 0) ? th.tagId : lastObservedTagId;
                if (tagId >= 0) {
                    TagLayout.TagPose tagPose = layout.get(tagId);
                    if (tagPose != null) {
                        Pose2d fieldToTag = tagPose.fieldToTagPose().toPose2d();
                        Pose2d fieldToAimFrame = fieldToRobot.then(spec.controlFrames.robotToAimFrame());
                        omegaErr = Pose2d.wrapToPi((fieldToTag.headingRad + th.headingOffsetRad) - fieldToAimFrame.headingRad);
                        canOmega = true;
                    }
                }
            }
        } else if (fieldToAimPoint != null) {
            Pose2d fieldToAimFrame = fieldToRobot.then(spec.controlFrames.robotToAimFrame());
            Pose2d aimFrameToPoint = fieldToAimFrame.inverse().then(fieldToAimPoint);
            omegaErr = Pose2d.wrapToPi(
                    SpatialMath2d.bearingRadOfVector(aimFrameToPoint.xInches, aimFrameToPoint.yInches));
            canOmega = true;
        }

        return new Solution(true, canTranslate, canOmega, forwardErr, leftErr, omegaErr, false, Double.NaN);
    }

    /**
     * Resolve a plan target into a field-coordinate point (x,y,heading=0) if possible.
     */
    private Pose2d resolveToFieldPoint(Object target, TagLayout layout) {
        if (target == null) {
            return null;
        }

        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            DriveGuidanceSpec.FieldPoint fp = (DriveGuidanceSpec.FieldPoint) target;
            return new Pose2d(fp.xInches, fp.yInches, 0.0);
        }

        if (target instanceof DriveGuidanceSpec.TagRelativePoint) {
            if (layout == null) {
                return null;
            }

            DriveGuidanceSpec.TagRelativePoint tp = (DriveGuidanceSpec.TagRelativePoint) target;
            int tagId = (tp.tagId >= 0) ? tp.tagId : lastObservedTagId;
            if (tagId < 0) {
                return null;
            }

            TagLayout.TagPose tagPose = layout.get(tagId);
            if (tagPose == null) {
                return null;
            }

            Pose2d fieldToTag = tagPose.fieldToTagPose().toPose2d();
            Pose2d tagToPoint = new Pose2d(tp.forwardInches, tp.leftInches, 0.0);
            return fieldToTag.then(tagToPoint);
        }

        return null;
    }

    /**
     * Small solver result bundle: just errors and capability flags.
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
}
