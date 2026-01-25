package edu.ftcphoenix.fw.spatial;

import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Robot footprint modeled as a rectangle.
 *
 * <p>The rectangle is centered at the robot origin. Length is along +X (forward), width along +Y (left/right).</p>
 */
public final class RectangleFootprint2d implements RobotFootprint2d {

    /**
     * Front-to-back length of the robot (inches, non-negative).
     */
    public final double lengthInches;

    /**
     * Left-to-right width of the robot (inches, non-negative).
     */
    public final double widthInches;

    public RectangleFootprint2d(double lengthInches, double widthInches) {
        this.lengthInches = Math.max(0.0, lengthInches);
        this.widthInches = Math.max(0.0, widthInches);
    }

    /**
     * @return the rectangle's 4 corners in the robot frame as (x,y) poses with heading = 0.
     */
    public Pose2d[] cornersRobotFrame() {
        double hx = lengthInches / 2.0;
        double hy = widthInches / 2.0;

        // Order: front-left, front-right, back-right, back-left (CCW around robot).
        return new Pose2d[]{
                new Pose2d(+hx, +hy, 0.0),
                new Pose2d(+hx, -hy, 0.0),
                new Pose2d(-hx, -hy, 0.0),
                new Pose2d(-hx, +hy, 0.0)
        };
    }

    /**
     * Generate boundary probe points around the rectangle.
     *
     * <p>This is intended for <b>approximate</b> overlap checks where you don't want a full polygon intersection test.
     * Probe points are placed equally spaced along each edge.</p>
     *
     * <p>{@code samplesPerEdge} counts points <b>including</b> each edge's endpoints (the corners):</p>
     * <ul>
     *   <li>{@code samplesPerEdge = 2} -> just the 4 corners</li>
     *   <li>{@code samplesPerEdge = 3} -> corners + 1 midpoint per edge</li>
     *   <li>{@code samplesPerEdge = 4} -> corners + 2 interior points per edge</li>
     * </ul>
     */
    public List<Pose2d> boundaryProbePointsRobotFrame(int samplesPerEdge) {
        int s = Math.max(2, samplesPerEdge);
        Pose2d[] c = cornersRobotFrame();

        List<Pose2d> pts = new ArrayList<Pose2d>(4 * s);

        for (int edge = 0; edge < 4; edge++) {
            Pose2d a = c[edge];
            Pose2d b = c[(edge + 1) % 4];

            // For edges after the first, skip t=0 to avoid duplicating the shared corner.
            int startK = (edge == 0) ? 0 : 1;
            for (int k = startK; k < s; k++) {
                double t = (s == 1) ? 0.0 : ((double) k) / (s - 1);
                double x = MathUtil.lerp(a.xInches, b.xInches, t);
                double y = MathUtil.lerp(a.yInches, b.yInches, t);
                pts.add(new Pose2d(x, y, 0.0));
            }
        }

        // Last point duplicates the first corner; remove it.
        if (!pts.isEmpty()) {
            Pose2d last = pts.get(pts.size() - 1);
            Pose2d first = pts.get(0);
            if (Math.abs(last.xInches - first.xInches) < 1e-9 && Math.abs(last.yInches - first.yInches) < 1e-9) {
                pts.remove(pts.size() - 1);
            }
        }

        return pts;
    }

    @Override
    public String toString() {
        return "RectangleFootprint2d{" +
                "lengthInches=" + MathUtil.fmt(lengthInches) +
                ", widthInches=" + MathUtil.fmt(widthInches) +
                '}';
    }
}
