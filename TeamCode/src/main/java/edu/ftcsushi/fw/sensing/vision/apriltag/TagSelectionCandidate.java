package edu.ftcsushi.fw.sensing.vision.apriltag;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * One eligible tag's finite geometry at its original evidence time. A candidate is created only
 * by {@link TagSelections}; a policy chooses an instance supplied to that invocation.
 * Distances are inches; camera and robot axes are +X forward, +Y left, +Z up.
 */
public final class TagSelectionCandidate {
    /** Where geometry came from; a field calculation is not a camera observation. */
    public enum EvidenceKind { OBSERVED, FIELD_POSE }

    public final int tagId;
    public final Pose3d cameraToTagPose;
    public final Pose3d robotToTagPose;
    public final EvidenceKind evidenceKind;
    public final LoopTimestamp evidenceTimestamp;
    /** Actual frame observation for OBSERVED; null for FIELD_POSE. */
    public final AprilTagObservation observation;

    TagSelectionCandidate(int tagId, Pose3d cameraToTagPose, Pose3d robotToTagPose,
                          EvidenceKind evidenceKind, LoopTimestamp evidenceTimestamp,
                          AprilTagObservation observation) {
        this.tagId = tagId;
        this.cameraToTagPose = cameraToTagPose;
        this.robotToTagPose = robotToTagPose;
        this.evidenceKind = evidenceKind;
        this.evidenceTimestamp = evidenceTimestamp;
        this.observation = observation;
    }

    /** Signed horizontal bearing from camera forward, radians; not a visibility claim. */
    public double cameraBearingRad() {
        return Math.atan2(cameraToTagPose.yInches, cameraToTagPose.xInches);
    }

    /** Signed horizontal bearing from robot forward, radians. */
    public double robotBearingRad() {
        return Math.atan2(robotToTagPose.yInches, robotToTagPose.xInches);
    }

    /** Three-dimensional camera-origin to tag-center distance, inches. */
    public double cameraRangeInches() {
        return Math.hypot(Math.hypot(cameraToTagPose.xInches, cameraToTagPose.yInches),
                cameraToTagPose.zInches);
    }

    @Override public String toString() {
        return "TagSelectionCandidate{tagId=" + tagId + ", evidenceKind=" + evidenceKind + '}';
    }
}
