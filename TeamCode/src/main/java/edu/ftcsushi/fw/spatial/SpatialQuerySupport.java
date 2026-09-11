package edu.ftcsushi.fw.spatial;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionResult;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.CameraMountLogic;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;

final class SpatialQuerySupport {

    static final ReferenceSelectionResult NO_SELECTION =
            ReferenceSelectionResult.none();

    private SpatialQuerySupport() {
        // utility holder
    }

    /** Attaches observed/committed target evidence without discarding the solve pose timestamp. */
    static TranslationSolution targetEvidence(TranslationSolution solution, Object target, LoopClock clock) {
        if (solution == null) return null;
        TargetEvidence evidence = targetEvidence(target, clock);
        return evidence == null ? solution
                : solution.withTargetEvidence(evidence.timestamp, evidence.requiresSightingAge);
    }

    /** Facing counterpart to the translation evidence bridge. */
    static FacingSolution targetEvidence(FacingSolution solution, Object target, LoopClock clock) {
        if (solution == null) return null;
        TargetEvidence evidence = targetEvidence(target, clock);
        return evidence == null ? solution
                : solution.withTargetEvidence(evidence.timestamp, evidence.requiresSightingAge);
    }

    private static TargetEvidence targetEvidence(Object target, LoopClock clock) {
        Object reference = target;
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            reference = ((SpatialTargets.ReferencePointTarget) target).reference;
        } else if (target instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            reference = ((SpatialTargets.ReferenceFrameHeadingTarget) target).reference;
        }
        if (reference instanceof References.FramePointRef) {
            reference = ((References.FramePointRef) reference).frame;
        }
        if (reference instanceof References.ObservedPointRef) {
            return new TargetEvidence(((References.ObservedPointRef) reference).get(clock)
                    .observation().timestamp, true);
        }
        if (reference instanceof References.RememberedPointRef) {
            FieldTargetSelectionResult selected = ((References.RememberedPointRef) reference).get(clock);
            return new TargetEvidence(selected.hasSelection()
                    ? selected.entry().lastSighting().timestamp : LoopTimestamp.unavailable(), true);
        }
        if (reference instanceof References.ApproachFrameRef) {
            ApproachResult2d approach = ((References.ApproachFrameRef) reference).get(clock);
            return new TargetEvidence(approach.observation().timestamp, !approach.isCommitted());
        }
        return null;
    }

    private static final class TargetEvidence {
        final LoopTimestamp timestamp;
        final boolean requiresSightingAge;
        TargetEvidence(LoopTimestamp timestamp, boolean requiresSightingAge) {
            this.timestamp = timestamp;
            this.requiresSightingAge = requiresSightingAge;
        }
    }

    static Pose2d resolveFieldPointTarget(Object target,
                                          edu.ftcsushi.fw.field.TagLayout layout,
                                          LoopClock clock) {
        if (target == null) {
            return null;
        }
        if (target instanceof SpatialTargets.FieldPoint) {
            SpatialTargets.FieldPoint fp = (SpatialTargets.FieldPoint) target;
            return new Pose2d(fp.xInches, fp.yInches, 0.0);
        }
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return References.tryResolveFieldPoint(((SpatialTargets.ReferencePointTarget) target).reference, layout, clock);
        }
        return null;
    }

    static Pose2d resolveFieldFrameHeadingTarget(SpatialTargets.ReferenceFrameHeadingTarget target,
                                                 edu.ftcsushi.fw.field.TagLayout layout,
                                                 LoopClock clock) {
        if (target == null) {
            return null;
        }
        return References.tryResolveFieldFrame(target.reference, layout, clock);
    }

    static Pose2d resolveRobotPointDirect(LoopClock clock,
                                          ReferencePoint2d reference,
                                          AprilTagDetections detections,
                                          CameraMountConfig cameraMount,
                                          double maxAgeSec) {
        if (reference == null) {
            return null;
        }
        if (References.isFieldPoint(reference)) {
            return null;
        }
        if (References.isFramePoint(reference)) {
            ReferenceFrame2d frame = References.framePointBaseFrame(reference);
            Pose2d robotToFrame = resolveRobotFrameDirect(clock, frame, detections, cameraMount, maxAgeSec);
            Pose2d frameOffset = References.framePointOffset(reference);
            return robotToFrame != null && frameOffset != null ? robotToFrame.then(frameOffset) : null;
        }
        if (reference instanceof References.TagPointRef) {
            References.TagPointRef tp = (References.TagPointRef) reference;
            AprilTagObservation obs = observationForId(clock, detections, tp.tagId, maxAgeSec);
            return composeRobotThingFromObservation(obs, cameraMount, new Pose2d(tp.forwardInches, tp.leftInches, 0.0));
        }
        if (reference instanceof References.SelectedTagPointRef) {
            References.SelectedTagPointRef sp = (References.SelectedTagPointRef) reference;
            TagSelectionResult sel = sp.selection.get(clock);
            if (sel == null || !sel.hasSelection) {
                return null;
            }
            Pose2d tagToPoint = sp.lookup.tagToPoint(sel.selectedTagId);
            return composeRobotThingFromObservation(
                    observationForId(clock, detections, sel.selectedTagId, maxAgeSec),
                    cameraMount, tagToPoint);
        }
        return null;
    }

    static Pose2d resolveRobotFrameDirect(LoopClock clock,
                                          ReferenceFrame2d reference,
                                          AprilTagDetections detections,
                                          CameraMountConfig cameraMount,
                                          double maxAgeSec) {
        if (reference == null || References.isFieldFrame(reference)) {
            return null;
        }
        if (reference instanceof References.TagFrameRef) {
            References.TagFrameRef tf = (References.TagFrameRef) reference;
            AprilTagObservation obs = observationForId(clock, detections, tf.tagId, maxAgeSec);
            return composeRobotThingFromObservation(obs, cameraMount, new Pose2d(tf.forwardInches, tf.leftInches, tf.headingRad));
        }
        if (reference instanceof References.SelectedTagFrameRef) {
            References.SelectedTagFrameRef sf = (References.SelectedTagFrameRef) reference;
            TagSelectionResult sel = sf.selection.get(clock);
            if (sel == null || !sel.hasSelection) {
                return null;
            }
            Pose2d tagToFrame = sf.lookup.tagToFrame(sel.selectedTagId);
            return composeRobotThingFromObservation(
                    observationForId(clock, detections, sel.selectedTagId, maxAgeSec),
                    cameraMount, tagToFrame);
        }
        return null;
    }

    static Pose2d composeRobotThingFromObservation(AprilTagObservation obs,
                                                   CameraMountConfig cameraMount,
                                                   Pose2d tagToThing) {
        if (obs == null || !obs.hasTarget || tagToThing == null
                || !SpatialValidation.isFinite(obs.cameraToTagPose)) {
            return null;
        }
        Pose3d robotToTag = CameraMountLogic.robotToTagPose(cameraMount, obs.cameraToTagPose);
        if (!SpatialValidation.isFinite(robotToTag)) return null;
        Pose2d robotToThing = robotToTag.toPose2d().then(tagToThing);
        return SpatialValidation.isFinite(robotToThing) ? robotToThing : null;
    }

    static AprilTagObservation observationForId(LoopClock clock,
                                                AprilTagDetections detections,
                                                int id,
                                                double maxAgeSec) {
        if (detections == null) {
            return AprilTagObservation.noTarget();
        }
        return detections.forId(clock, id, maxAgeSec);
    }

    /** Retains the point's selection authority without substituting a solving camera's evidence. */
    static ReferenceSelectionResult pointSelectionSnapshot(ReferencePoint2d ref, LoopClock clock) {
        if (ref == null || References.isFieldPoint(ref)) {
            return NO_SELECTION;
        }
        if (References.isFramePoint(ref)) {
            return frameSelectionSnapshot(References.framePointBaseFrame(ref), clock);
        }
        if (ref instanceof References.ObservedPointRef) {
            return ReferenceSelectionResult.observedTarget(((References.ObservedPointRef) ref).get(clock));
        }
        if (ref instanceof References.RememberedPointRef) {
            return ReferenceSelectionResult.rememberedTarget(((References.RememberedPointRef) ref).get(clock));
        }
        if (ref instanceof References.TagPointRef) {
            References.TagPointRef tp = (References.TagPointRef) ref;
            return ReferenceSelectionResult.aprilTag(TagSelectionResult.forTagId(tp.tagId));
        }
        if (ref instanceof References.SelectedTagPointRef) {
            TagSelectionResult sel = ((References.SelectedTagPointRef) ref).selection.get(clock);
            return sel != null ? ReferenceSelectionResult.aprilTag(sel) : NO_SELECTION;
        }
        return NO_SELECTION;
    }

    /** Retains selected-tag or computed-approach provenance, including unavailable domain results. */
    static ReferenceSelectionResult frameSelectionSnapshot(ReferenceFrame2d ref, LoopClock clock) {
        if (ref == null || References.isFieldFrame(ref)) {
            return NO_SELECTION;
        }
        if (ref instanceof References.ApproachFrameRef) {
            return ReferenceSelectionResult.approach(((References.ApproachFrameRef) ref).get(clock));
        }
        if (ref instanceof References.TagFrameRef) {
            References.TagFrameRef tf = (References.TagFrameRef) ref;
            return ReferenceSelectionResult.aprilTag(TagSelectionResult.forTagId(tf.tagId));
        }
        if (ref instanceof References.SelectedTagFrameRef) {
            TagSelectionResult sel = ((References.SelectedTagFrameRef) ref).selection.get(clock);
            return sel != null ? ReferenceSelectionResult.aprilTag(sel) : NO_SELECTION;
        }
        return NO_SELECTION;
    }

    /** Extracts selection provenance independently of whether translation geometry was solved. */
    static ReferenceSelectionResult translationSelectionSnapshot(TranslationTarget2d target, LoopClock clock) {
        if (!(target instanceof SpatialTargets.ReferencePointTarget)) {
            return NO_SELECTION;
        }
        return pointSelectionSnapshot(((SpatialTargets.ReferencePointTarget) target).reference,
                clock);
    }

    /** Extracts point/frame selection provenance independently of the facing solution. */
    static ReferenceSelectionResult facingSelectionSnapshot(FacingTarget2d target, LoopClock clock) {
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return pointSelectionSnapshot(((SpatialTargets.ReferencePointTarget) target).reference,
                    clock);
        }
        if (target instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            return frameSelectionSnapshot(((SpatialTargets.ReferenceFrameHeadingTarget) target).reference,
                    clock);
        }
        return NO_SELECTION;
    }

}
