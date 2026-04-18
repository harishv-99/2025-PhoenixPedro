package edu.ftcphoenix.fw.spatial;

import java.util.Collections;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

final class SpatialQuerySupport {

    static final TagSelectionResult NO_SELECTION =
            TagSelectionResult.none(Collections.<Integer>emptySet());

    private SpatialQuerySupport() {
        // utility holder
    }

    static Pose2d resolveFieldPointTarget(Object target,
                                          edu.ftcphoenix.fw.field.TagLayout layout,
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
                                                 edu.ftcphoenix.fw.field.TagLayout layout,
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
            AprilTagObservation obs = observationForId(detections, tp.tagId, maxAgeSec);
            return composeRobotThingFromObservation(obs, cameraMount, new Pose2d(tp.forwardInches, tp.leftInches, 0.0));
        }
        if (reference instanceof References.SelectedTagPointRef) {
            References.SelectedTagPointRef sp = (References.SelectedTagPointRef) reference;
            TagSelectionResult sel = sp.selection.get(clock);
            if (sel == null || !sel.hasFreshSelectedObservation) {
                return null;
            }
            Pose2d tagToPoint = sp.lookup.tagToPoint(sel.selectedTagId);
            return composeRobotThingFromObservation(sel.selectedObservation, cameraMount, tagToPoint);
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
            AprilTagObservation obs = observationForId(detections, tf.tagId, maxAgeSec);
            return composeRobotThingFromObservation(obs, cameraMount, new Pose2d(tf.forwardInches, tf.leftInches, tf.headingRad));
        }
        if (reference instanceof References.SelectedTagFrameRef) {
            References.SelectedTagFrameRef sf = (References.SelectedTagFrameRef) reference;
            TagSelectionResult sel = sf.selection.get(clock);
            if (sel == null || !sel.hasFreshSelectedObservation) {
                return null;
            }
            Pose2d tagToFrame = sf.lookup.tagToFrame(sel.selectedTagId);
            return composeRobotThingFromObservation(sel.selectedObservation, cameraMount, tagToFrame);
        }
        return null;
    }

    static Pose2d composeRobotThingFromObservation(AprilTagObservation obs,
                                                   CameraMountConfig cameraMount,
                                                   Pose2d tagToThing) {
        if (obs == null || !obs.hasTarget || tagToThing == null) {
            return null;
        }
        Pose3d robotToTag = CameraMountLogic.robotToTagPose(cameraMount, obs.cameraToTagPose);
        Pose2d robotToThing = robotToTag.toPose2d().then(tagToThing);
        return new Pose2d(robotToThing.xInches, robotToThing.yInches, robotToThing.headingRad);
    }

    static AprilTagObservation observationForId(AprilTagDetections detections,
                                                int id,
                                                double maxAgeSec) {
        if (detections == null) {
            return AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        }
        return detections.forId(id, maxAgeSec);
    }

    static TagSelectionResult pointSelectionSnapshot(ReferencePoint2d ref,
                                                     LoopClock clock,
                                                     AprilTagDetections detections,
                                                     double maxAgeSec) {
        if (ref == null || References.isFieldPoint(ref)) {
            return NO_SELECTION;
        }
        if (References.isFramePoint(ref)) {
            return frameSelectionSnapshot(References.framePointBaseFrame(ref), clock, detections, maxAgeSec);
        }
        if (ref instanceof References.TagPointRef) {
            References.TagPointRef tp = (References.TagPointRef) ref;
            return fixedTagSelection(tp.tagId, observationForId(detections, tp.tagId, maxAgeSec));
        }
        if (ref instanceof References.SelectedTagPointRef) {
            TagSelectionResult sel = ((References.SelectedTagPointRef) ref).selection.get(clock);
            return sel != null ? sel : NO_SELECTION;
        }
        return NO_SELECTION;
    }

    static TagSelectionResult frameSelectionSnapshot(ReferenceFrame2d ref,
                                                     LoopClock clock,
                                                     AprilTagDetections detections,
                                                     double maxAgeSec) {
        if (ref == null || References.isFieldFrame(ref)) {
            return NO_SELECTION;
        }
        if (ref instanceof References.TagFrameRef) {
            References.TagFrameRef tf = (References.TagFrameRef) ref;
            return fixedTagSelection(tf.tagId, observationForId(detections, tf.tagId, maxAgeSec));
        }
        if (ref instanceof References.SelectedTagFrameRef) {
            TagSelectionResult sel = ((References.SelectedTagFrameRef) ref).selection.get(clock);
            return sel != null ? sel : NO_SELECTION;
        }
        return NO_SELECTION;
    }

    static TagSelectionResult translationSelectionSnapshot(TranslationTarget2d target,
                                                           LoopClock clock,
                                                           AprilTagDetections detections,
                                                           double maxAgeSec) {
        if (!(target instanceof SpatialTargets.ReferencePointTarget)) {
            return NO_SELECTION;
        }
        return pointSelectionSnapshot(((SpatialTargets.ReferencePointTarget) target).reference,
                clock,
                detections,
                maxAgeSec);
    }

    static TagSelectionResult aimSelectionSnapshot(AimTarget2d target,
                                                   LoopClock clock,
                                                   AprilTagDetections detections,
                                                   double maxAgeSec) {
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return pointSelectionSnapshot(((SpatialTargets.ReferencePointTarget) target).reference,
                    clock,
                    detections,
                    maxAgeSec);
        }
        if (target instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            return frameSelectionSnapshot(((SpatialTargets.ReferenceFrameHeadingTarget) target).reference,
                    clock,
                    detections,
                    maxAgeSec);
        }
        return NO_SELECTION;
    }

    static TagSelectionResult fixedTagSelection(int tagId, AprilTagObservation obs) {
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

    static void resetSelections(TranslationTarget2d target) {
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            resetReferenceSelection(((SpatialTargets.ReferencePointTarget) target).reference);
        }
    }

    static void resetSelections(AimTarget2d target) {
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            resetReferenceSelection(((SpatialTargets.ReferencePointTarget) target).reference);
        } else if (target instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            resetReferenceSelection(((SpatialTargets.ReferenceFrameHeadingTarget) target).reference);
        }
    }

    static void resetReferenceSelection(ReferencePoint2d ref) {
        if (ref instanceof References.SelectedTagPointRef) {
            ((References.SelectedTagPointRef) ref).selection.reset();
        } else if (References.isFramePoint(ref)) {
            resetReferenceSelection(References.framePointBaseFrame(ref));
        }
    }

    static void resetReferenceSelection(ReferenceFrame2d ref) {
        if (ref instanceof References.SelectedTagFrameRef) {
            ((References.SelectedTagFrameRef) ref).selection.reset();
        }
    }
}
