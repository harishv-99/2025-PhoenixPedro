package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

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

    static boolean isFinite(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.xInches)
                && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.headingRad);
    }
}
