package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

/**
 * Factory helpers for common spatial-query targets.
 */
public final class SpatialTargets {

    /**
     * Field point target used for translation or point-at aiming.
     */
    public static final class FieldPoint implements TranslationTarget2d, AimTarget2d {
        public final double xInches;
        public final double yInches;

        public FieldPoint(double xInches, double yInches) {
            this.xInches = xInches;
            this.yInches = yInches;
        }

        @Override
        public String toString() {
            return "FieldPoint{xInches=" + xInches + ", yInches=" + yInches + '}';
        }
    }

    /**
     * Absolute field heading target.
     */
    public static final class FieldHeading implements AimTarget2d {
        public final double fieldHeadingRad;

        public FieldHeading(double fieldHeadingRad) {
            this.fieldHeadingRad = fieldHeadingRad;
        }

        @Override
        public String toString() {
            return "FieldHeading{fieldHeadingRad=" + fieldHeadingRad + '}';
        }
    }

    /**
     * Semantic reference point target.
     */
    public static final class ReferencePointTarget implements TranslationTarget2d, AimTarget2d {
        public final ReferencePoint2d reference;

        public ReferencePointTarget(ReferencePoint2d reference) {
            this.reference = Objects.requireNonNull(reference, "reference");
        }

        @Override
        public String toString() {
            return "ReferencePointTarget{reference=" + reference + '}';
        }
    }

    /**
     * Aligns the aim frame to a semantic reference-frame heading.
     */
    public static final class ReferenceFrameHeadingTarget implements AimTarget2d {
        public final ReferenceFrame2d reference;
        public final double headingOffsetRad;

        public ReferenceFrameHeadingTarget(ReferenceFrame2d reference, double headingOffsetRad) {
            this.reference = Objects.requireNonNull(reference, "reference");
            this.headingOffsetRad = headingOffsetRad;
        }

        @Override
        public String toString() {
            return "ReferenceFrameHeadingTarget{reference=" + reference
                    + ", headingOffsetRad=" + headingOffsetRad + '}';
        }
    }

    private SpatialTargets() {
        // utility holder
    }

    /**
     * Creates a field-fixed point target.
     */
    public static FieldPoint fieldPoint(double xInches, double yInches) {
        return new FieldPoint(xInches, yInches);
    }

    /**
     * Creates an absolute field-heading aim target.
     */
    public static FieldHeading fieldHeading(double fieldHeadingRad) {
        return new FieldHeading(fieldHeadingRad);
    }

    /**
     * Creates a semantic reference-point target.
     */
    public static ReferencePointTarget point(ReferencePoint2d reference) {
        return new ReferencePointTarget(reference);
    }

    /**
     * Creates an aim target that matches a semantic frame heading with zero extra offset.
     */
    public static ReferenceFrameHeadingTarget frameHeading(ReferenceFrame2d reference) {
        return new ReferenceFrameHeadingTarget(reference, 0.0);
    }

    /**
     * Creates an aim target that matches a semantic frame heading plus an additional heading offset.
     */
    public static ReferenceFrameHeadingTarget frameHeading(ReferenceFrame2d reference,
                                                           double headingOffsetRad) {
        return new ReferenceFrameHeadingTarget(reference, headingOffsetRad);
    }
}
