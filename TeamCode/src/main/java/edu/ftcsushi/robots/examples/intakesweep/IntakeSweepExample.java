package edu.ftcsushi.robots.examples.intakesweep;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.spatial.ToolSweep2d;

/**
 * Calculation-only ball-center encounter example for one illustrative straight robot movement.
 *
 * <p>The robot center translates from field (10, 20) to (30, 20) inches while its heading stays
 * zero radians. The fixed intake origin is 6 inches forward and 1 inch left of robot center,
 * facing robot-forward. The window accepts center positions from -1 to +2 inches along intake
 * forward and from -3 to +3 inches along intake left. It is not the physical intake opening.</p>
 *
 * <p>For a simplified 12-inch opening and 4-inch ball, this 6-inch-wide center window leaves one
 * extra inch beside the ball at either side boundary. These are illustrative allowances, not
 * recommended dimensions or capture evidence. Forward/back bounds are already-reduced center
 * bounds too. The framework neither adds the ball's radius nor subtracts an allowance again.</p>
 *
 * <p>The private immutable sweep is built once when this class is initialized. Each query computes
 * immediately without changing it. There is no camera, localization, clock, controller, hardware
 * command, path execution, or STOP responsibility. Inputs must already represent estimated ball
 * centers in the same field coordinates as the movement. The caller retains responsibility for
 * observation meaning, freshness, motion feasibility, whole-robot travel bounds, and capture
 * confirmation. A geometric encounter is not any of those facts.</p>
 */
public final class IntakeSweepExample {
    private static final ToolSweep2d MODELED_MOVE = ToolSweep2d
            .straightFrom(new Pose2d(10.0, 20.0, 0.0))
            .toFieldPoint(30.0, 20.0)
            .throughTool(new Pose2d(6.0, 1.0, 0.0))
            .centerWindowInches(-1.0, 2.0, 6.0);

    private IntakeSweepExample() {
    }

    /**
     * Asks when an estimated stationary ball center enters and leaves the illustrative window.
     *
     * <p>Check {@link ToolSweep2d.Encounter#hasEncounter()} before reading either travel distance.
     * Distances describe progress along the modeled robot-center segment, not measured movement,
     * tool-to-ball range, or seconds. Repeating a point returns the same geometry, not evidence
     * that two objects exist. Ball-edge overlap outside the center window is not an encounter.</p>
     *
     * @param fieldCenterXInches estimated ball-center field X coordinate, in inches
     * @param fieldCenterYInches estimated ball-center field Y coordinate, in inches
     * @return the framework's immutable encounter result, without motion or collection policy
     * @throws IllegalArgumentException if a coordinate or required arithmetic is not finite
     */
    public static ToolSweep2d.Encounter encounterBallCenter(
            double fieldCenterXInches, double fieldCenterYInches) {
        return MODELED_MOVE.encounterFieldCenter(fieldCenterXInches, fieldCenterYInches);
    }
}
