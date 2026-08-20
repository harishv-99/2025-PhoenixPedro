package edu.ftcphoenix.robots.examples.reference.autonomous;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.spatial.AxisAlignedBoxRegion2d;
import edu.ftcphoenix.fw.spatial.RobotFrameRectangle2d;
import edu.ftcphoenix.fw.spatial.SpatialMath2d;

/** Robot-owned, controller-free parking facts used before constructing a guidance Task. */
public final class ReferenceParkingPlan {
    private final RobotFrameRectangle2d robotRectangle;
    private final AxisAlignedBoxRegion2d knownClearBox;
    private final double targetXInches;
    private final double targetYInches;
    private final double[] allowedHeadingsRad;

    /** Validates every authored full-box target before any live heading is selected. */
    public ReferenceParkingPlan(RobotFrameRectangle2d robotRectangle,
                                AxisAlignedBoxRegion2d knownClearBox,
                                double targetXInches,
                                double targetYInches,
                                double... allowedHeadingsRad) {
        this.robotRectangle = Objects.requireNonNull(robotRectangle, "robotRectangle");
        this.knownClearBox = Objects.requireNonNull(knownClearBox, "knownClearBox");
        if (!Double.isFinite(targetXInches) || !Double.isFinite(targetYInches)) {
            throw new IllegalArgumentException("parking target translation must be finite");
        }
        Objects.requireNonNull(allowedHeadingsRad, "allowedHeadingsRad");
        if (allowedHeadingsRad.length == 0) {
            throw new IllegalArgumentException("at least one allowed parking heading is required");
        }
        this.targetXInches = targetXInches;
        this.targetYInches = targetYInches;
        this.allowedHeadingsRad = allowedHeadingsRad.clone();
        for (int i = 0; i < this.allowedHeadingsRad.length; i++) {
            Pose2d candidate = new Pose2d(
                    targetXInches, targetYInches, this.allowedHeadingsRad[i]);
            if (!robotRectangle.fullyInside(knownClearBox, candidate)) {
                throw new IllegalArgumentException(
                        "parking target heading at index " + i
                                + " does not place the authored robot rectangle fully inside "
                                + "the known-clear box");
            }
        }
    }

    /** Freezes the nearest reviewed complete pose once from usable start-pose evidence. */
    public Pose2d freezeTargetFrom(Pose2d startPose) {
        Pose2d start = Objects.requireNonNull(startPose, "startPose");
        double heading = SpatialMath2d.nearestHeadingRad(
                start.headingRad, allowedHeadingsRad);
        return new Pose2d(targetXInches, targetYInches, heading);
    }

    /** Returns only the literal any-authored-corner-inside status for manual shared entry. */
    public boolean hasAnyCornerInside(Pose2d currentPose) {
        return robotRectangle.hasAnyCornerInside(
                knownClearBox, Objects.requireNonNull(currentPose, "currentPose"));
    }
}
