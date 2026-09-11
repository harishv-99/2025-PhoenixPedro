package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;

/** Package-local validation for authored spatial values. */
final class SpatialValidation {

    private SpatialValidation() {
        // Utility class.
    }

    static double requireFinite(String fieldName, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(
                    fieldName + " must be finite, got " + value);
        }
        return value;
    }

    static Pose2d requireFinitePose2d(String fieldName, Pose2d pose) {
        Pose2d value = Objects.requireNonNull(pose, fieldName);
        requireFinite(fieldName + ".xInches", value.xInches);
        requireFinite(fieldName + ".yInches", value.yInches);
        requireFinite(fieldName + ".headingRad", value.headingRad);
        return value;
    }

    /** All six components must be finite before runtime 3D geometry is projected to a plane. */
    static boolean isFinite(Pose3d pose) {
        return pose != null && Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches) && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad) && Double.isFinite(pose.rollRad);
    }

    static boolean isFinite(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.xInches)
                && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.headingRad);
    }
}
