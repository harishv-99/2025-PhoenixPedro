package edu.ftcphoenix.fw.sensing;

import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Source of target bearing information for aiming.
 *
 * <h2>Role</h2>
 * A {@code BearingSource} answers the question:
 * <em>"Is there a target right now, and if so, at what bearing (angle)
 * relative to the robot?"</em>
 *
 * <p>This is intentionally small and generic so it can be used with
 * different sensor backends:
 * <ul>
 *   <li>{@link AprilTagSensor} (AprilTags from a camera).</li>
 *   <li>Other vision targets in the future.</li>
 *   <li>Synthetic sources that always aim at a fixed heading.</li>
 * </ul>
 *
 * <p>It is designed to pair with {@link TagAimController},
 * which converts a {@link BearingSample} into a turn command (omega).
 *
 * <h2>Usage</h2>
 * Typical aim loop:
 * <pre>{@code
 * BearingSource bearing = ...;  // e.g., built from AprilTagSensor
 *
 * // In your TeleOp or Auto loop:
 * BearingSource.BearingSample sample = bearing.sample(clock);
 * double omega = aimController.update(clock, sample);
 * }</pre>
 */
public interface BearingSource {

    /**
     * Sample the current bearing measurement.
     *
     * <p>Implementations may internally cache sensor data and apply filtering
     * or freshness checks. The {@link LoopClock} gives access to the current
     * loop time and delta time, so sources can compute ages if needed.
     *
     * @param clock loop clock for the current iteration
     * @return a snapshot of the current bearing state
     */
    BearingSample sample(LoopClock clock);

    /**
     * Snapshot of bearing information at a point in time.
     *
     * <p>This is a simple immutable carrier object. It answers:
     * <ul>
     *   <li>Is there a target? ({@link #hasTarget})</li>
     *   <li>If so, what is the bearing to it? ({@link #bearingRad})</li>
     * </ul>
     */
    final class BearingSample {

        /**
         * Whether a target is currently available.
         *
         * <p>If {@code false}, the {@link #bearingRad} value should be ignored
         * by consumers.
         */
        public final boolean hasTarget;

        /**
         * Bearing to the target in radians, where 0 means "straight ahead".
         *
         * <p>Positive and negative directions follow the framework's standard
         * convention for robot angular motion (typically + = CCW).
         *
         * <p>If {@link #hasTarget} is {@code false}, this value is undefined
         * and should not be used.
         */
        public final double bearingRad;

        /**
         * Construct a bearing sample.
         *
         * @param hasTarget  whether a target is currently available
         * @param bearingRad bearing to target in radians (ignored if hasTarget is false)
         */
        public BearingSample(boolean hasTarget, double bearingRad) {
            this.hasTarget = hasTarget;
            this.bearingRad = bearingRad;
        }
    }
}
