package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Builder/factory for common {@link RobotHeading2d} rules.
 *
 * <p>This mirrors the role of {@link RobotZones2d} for position:
 * you choose <b>which robot frame</b> you care about (robot center vs an off-center mechanism)
 * and then define <b>what it should face</b> (a field heading or a field point).</p>
 *
 * <p>All methods here are pure geometry; they do not drive the robot. Combine these rules with
 * {@link HeadingLatch} (hysteresis) or your own controllers as needed.</p>
 */
public final class RobotHeadings2d {

    private RobotHeadings2d() {
    }

    /**
     * Start a heading rule for the robot center (no frame offset).
     */
    public static OnRobotFrame robotCenter() {
        return onRobotFrame(Pose2d.zero());
    }

    /**
     * Start a heading rule for a robot-relative control frame.
     *
     * <p>This is the same concept as {@link edu.ftcphoenix.fw.drive.guidance.ControlFrames#robotToAimFrame()}.
     * If you are aiming with an off-center shooter, pass that robot→shooter transform here.
     * </p>
     *
     * @param robotToFrame robot→frame transform (inches + heading)
     */
    public static OnRobotFrame onRobotFrame(Pose2d robotToFrame) {
        return new OnRobotFrame(robotToFrame);
    }

    /**
     * Builder stage scoped to a particular robot frame.
     */
    public static final class OnRobotFrame {

        private final Pose2d robotToFrame;

        private OnRobotFrame(Pose2d robotToFrame) {
            this.robotToFrame = Objects.requireNonNull(robotToFrame, "robotToFrame");
        }

        /**
         * Heading error to an absolute field heading.
         *
         * <p>The returned error is {@code wrapToPi(targetHeading - frameHeading)}.</p>
         */
        public RobotHeading2d toFieldHeadingRad(final double targetFieldHeadingRad) {
            return fieldToRobot -> {
                if (fieldToRobot == null) {
                    return Double.NaN;
                }
                Pose2d fieldToFrame = fieldToRobot.then(robotToFrame);
                return Pose2d.wrapToPi(targetFieldHeadingRad - fieldToFrame.headingRad);
            };
        }

        /**
         * Heading error to face an absolute field point.
         *
         * <p>For an aim control frame, this answers: “how many radians must the frame rotate so its
         * +X axis points at (x, y)?”</p>
         */
        public RobotHeading2d toFaceFieldPointInches(final double xInches, final double yInches) {
            return fieldToRobot -> {
                if (fieldToRobot == null) {
                    return Double.NaN;
                }

                Pose2d fieldToFrame = fieldToRobot.then(robotToFrame);

                // Vector from frame origin to target in field coordinates.
                double dx = xInches - fieldToFrame.xInches;
                double dy = yInches - fieldToFrame.yInches;

                // If the point is exactly at the origin, the bearing is undefined.
                if (Math.abs(dx) < 1e-9 && Math.abs(dy) < 1e-9) {
                    return Double.NaN;
                }

                double desiredBearing = Math.atan2(dy, dx);
                return Pose2d.wrapToPi(desiredBearing - fieldToFrame.headingRad);
            };
        }

        /**
         * Convenience overload for {@link #toFaceFieldPointInches(double, double)}.
         */
        public RobotHeading2d toFaceFieldPoint(Pose2d fieldToPoint) {
            Objects.requireNonNull(fieldToPoint, "fieldToPoint");
            return toFaceFieldPointInches(fieldToPoint.xInches, fieldToPoint.yInches);
        }
    }
}
