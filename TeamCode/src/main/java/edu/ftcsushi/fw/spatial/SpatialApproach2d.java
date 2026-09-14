package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;

/**
 * Immutable coupled position-and-facing question for a rigid tool approaching one point.
 *
 * <p>Construct with {@link #facePoint(ReferencePoint2d, Pose2d, double)}, then pass the description
 * to a spatial query/spec or drive-guidance builder's {@code approach(...)} stage. The description
 * owns no observations, clock, controller, or hardware. Its reference remains borrowed, and the
 * consumer explicitly chooses the evidence that can resolve it.</p>
 *
 * <p>Unlike an authored frame heading or a committed {@link ApproachResult2d}, this describes live
 * geometry rather than a frozen robot-center destination. The tool faces the point while leaving
 * the requested distance along tool +X. A solved approach is not arrival, clearance, visibility,
 * or capture evidence. Its translation distance is remaining approach error, not tool-to-point
 * range; use a separate query at the actual tool origin when range is required.</p>
 */
public final class SpatialApproach2d {
    private final ReferencePoint2d point;
    private final Pose2d robotToToolFrame;
    private final double standOffInches;
    private final Pose2d robotToStandOffFrame;
    private final SpatialControlFrames controlFrames;

    private SpatialApproach2d(ReferencePoint2d point, Pose2d robotToToolFrame,
                              double standOffInches, Pose2d robotToStandOffFrame,
                              SpatialControlFrames controlFrames) {
        this.point = point;
        this.robotToToolFrame = robotToToolFrame;
        this.standOffInches = standOffInches;
        this.robotToStandOffFrame = robotToStandOffFrame;
        this.controlFrames = controlFrames;
    }

    /**
     * Describes a point at {@code (standOffInches, 0)} in a rigid tool's local coordinates.
     *
     * <p>The tool pose is robot-relative, in inches and CCW-positive radians. Facing uses the tool
     * origin; translation uses the point the requested distance along tool +X. This one expansion
     * applies the tool offset once and retains the original target reference for both channels.
     * Nothing is sampled during construction. A positive stand-off avoids requesting an undefined
     * facing direction at the tool origin.</p>
     *
     * @throws NullPointerException if the point or tool pose is null
     * @throws IllegalArgumentException if the rigid pose is non-finite, stand-off is not finite
     *         and positive, or the derived translation-frame pose overflows or loses its separation
     */
    public static SpatialApproach2d facePoint(ReferencePoint2d point, Pose2d robotToToolFrame,
                                             double standOffInches) {
        ReferencePoint2d target = Objects.requireNonNull(point, "point");
        Pose2d tool = SpatialValidation.requireFinitePose2d("robotToToolFrame", robotToToolFrame);
        if (!Double.isFinite(standOffInches) || standOffInches <= 0.0) {
            throw new IllegalArgumentException("standOffInches must be finite and > 0");
        }
        Pose2d translation = tool.then(new Pose2d(standOffInches, 0.0, 0.0));
        SpatialValidation.requireFinitePose2d("derived approach translation frame", translation);
        if (translation.xInches == tool.xInches && translation.yInches == tool.yInches) {
            throw new IllegalArgumentException("standOffInches is too small for robotToToolFrame coordinates; "
                    + "the derived stand-off must remain distinct from the tool origin");
        }
        return new SpatialApproach2d(target, tool, standOffInches, translation,
                SpatialControlFrames.of(translation, tool));
    }

    /** Returns the exact borrowed point shared by the two channels; does not sample its source. */
    public ReferencePoint2d point() { return point; }

    /** Returns the finite rigid robot-to-tool pose, in inches and CCW-positive radians. */
    public Pose2d robotToToolFrame() { return robotToToolFrame; }

    /** Returns the positive desired tool-local forward separation, in inches. */
    public double standOffInches() { return standOffInches; }

    /**
     * Returns the once-computed robot-to-stand-off frame, in inches and CCW-positive radians.
     * Its origin is where the target should appear in robot coordinates at approach completion.
     * Its distance from an observed target is approach error, not tool-to-target range.
     */
    public Pose2d robotToStandOffFrame() { return robotToStandOffFrame; }

    /** The sole precomputed expansion; query/spec construction owns its use. */
    SpatialControlFrames controlFrames() { return controlFrames; }

    @Override
    public String toString() {
        return "SpatialApproach2d{point=" + point + ", robotToToolFrame=" + robotToToolFrame
                + ", standOffInches=" + standOffInches + '}';
    }
}
