package edu.ftcphoenix.fw.sensing;

/**
 * Immutable snapshot of a single AprilTag observation in robot-centric
 * coordinates.
 *
 * <p>This value type deliberately decouples robot code from the FTC vision
 * APIs. It contains the information most robots commonly need:
 * <ul>
 *   <li>Selecting a tag by ID from a set of IDs.</li>
 *   <li>Reading the distance to that tag (for shooter control, etc.).</li>
 *   <li>Reading the bearing to that tag (for aiming or alignment).</li>
 * </ul>
 * </p>
 *
 * <h2>Coordinate conventions</h2>
 *
 * <ul>
 *   <li><strong>Bearing:</strong> {@link #bearingRad} is the angle from the
 *       robot's forward direction to the tag, in radians.</li>
 *   <li>0 radians means "directly in front of the robot".</li>
 *   <li>Positive angles are counter-clockwise (tag appears to the left).</li>
 *   <li>Negative angles are clockwise (tag appears to the right).</li>
 *   <li><strong>Range:</strong> {@link #rangeInches} is the straight-line
 *       distance from the robot to the tag center, in <b>inches</b>.</li>
 * </ul>
 *
 * <p>On the FTC side, the AprilTag library exposes an {@code ftcPose} with
 * fields such as {@code range} and {@code bearing}. The FTC adapter in this
 * framework reads those values, verifies that pose metadata is available, and
 * converts them into the normalized units used here (radians for bearing,
 * inches for range).</p>
 *
 * <h2>Freshness and age</h2>
 *
 * <p>The {@link #ageSec} field reports how many seconds have elapsed since the
 * camera frame this observation came from. Callers typically:</p>
 *
 * <ul>
 *   <li>Choose a maximum acceptable age (for example, 0.3&nbsp;s).</li>
 *   <li>Ask an {@link AprilTagSensor} for the "best" observation that is not
 *       older than that age.</li>
 *   <li>Treat observations where {@link #hasTarget} is {@code false} as
 *       "no usable tag available right now".</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <p>To pick a tag from a set of IDs and use its distance and bearing:</p>
 *
 * <pre>{@code
 * AprilTagObservation obs = sensor.best(Set.of(1, 2, 3), 0.5);
 * if (obs.hasTarget) {
 *     // ID-based logic
 *     int id = obs.id;
 *
 *     // Shooter: map range (inches) to flywheel velocity
 *     double rangeIn = obs.rangeInches;
 *
 *     // Aiming: use bearing (radians) with a PID to generate turn command
 *     double bearingRad = obs.bearingRad;
 * }
 * }</pre>
 *
 * <h2>Moving in front of a tag (advanced)</h2>
 *
 * <p>This framework is optimized for simple usage, but it also supports
 * more advanced behaviors such as moving to a specific offset in front of
 * a tag. The helper methods {@link #forwardInches()} and
 * {@link #lateralInches()} decompose the observation into robot-centric
 * X/Y components:</p>
 *
 * <ul>
 *   <li>{@link #forwardInches()} &approx; distance along robot-forward axis.</li>
 *   <li>{@link #lateralInches()} &approx; distance left/right of robot forward.</li>
 * </ul>
 *
 * <p>These can be combined with your drive and PID logic to build "drive to
 * N inches in front of tag" behaviors later, without changing this type.</p>
 */
public final class AprilTagObservation {

    /**
     * Whether this observation represents a valid tag.
     *
     * <p>If {@code hasTarget} is {@code false}, the other fields are undefined
     * and should be ignored.</p>
     */
    public final boolean hasTarget;

    /**
     * AprilTag ID from the field layout.
     *
     * <p>Only meaningful when {@link #hasTarget} is {@code true}. When
     * {@code hasTarget} is {@code false}, this value is set to -1.</p>
     */
    public final int id;

    /**
     * Bearing from robot forward to tag, in radians.
     *
     * <p>Only meaningful when {@link #hasTarget} is {@code true}. See the
     * class-level documentation for the coordinate convention.</p>
     */
    public final double bearingRad;

    /**
     * Straight-line range from robot to tag, in inches.
     *
     * <p>Only meaningful when {@link #hasTarget} is {@code true}. This is
     * directly usable for:
     * <ul>
     *   <li>distance-based shooter control (mapping range to flywheel RPM),</li>
     *   <li>driving to a desired standoff distance from the tag,</li>
     *   <li>coarse positioning relative to a goal or landmark.</li>
     * </ul>
     * The FTC adapter keeps this value in inches to match how many teams
     * measure and tune their robots.</p>
     */
    public final double rangeInches;

    /**
     * Age of this observation in seconds.
     *
     * <p>Specifically, this is the elapsed time between the camera frame that
     * produced this observation and the moment the observation was created.
     * Higher-level code decides what counts as "too old" for a given behavior.</p>
     */
    public final double ageSec;

    private AprilTagObservation(boolean hasTarget,
                                int id,
                                double bearingRad,
                                double rangeInches,
                                double ageSec) {
        this.hasTarget = hasTarget;
        this.id = id;
        this.bearingRad = bearingRad;
        this.rangeInches = rangeInches;
        this.ageSec = ageSec;
    }

    /**
     * Create an observation representing "no tag available".
     *
     * <p>This is typically returned by {@link AprilTagSensor} when there is no
     * visible tag that meets the caller's ID and freshness criteria. The
     * {@link #ageSec} value can still be useful for logging or debugging.</p>
     *
     * @param ageSec how long ago the last camera frame was, in seconds
     * @return an observation with {@link #hasTarget} set to {@code false}
     */
    public static AprilTagObservation noTarget(double ageSec) {
        return new AprilTagObservation(false, -1, 0.0, 0.0, ageSec);
    }

    /**
     * Create an observation representing a single detected tag.
     *
     * @param id          AprilTag ID from the field layout
     * @param bearingRad  bearing from robot forward to the tag, in radians
     * @param rangeInches distance from robot to the tag, in inches
     * @param ageSec      age of the underlying camera frame, in seconds
     * @return an observation with {@link #hasTarget} set to {@code true}
     */
    public static AprilTagObservation target(int id,
                                             double bearingRad,
                                             double rangeInches,
                                             double ageSec) {
        return new AprilTagObservation(true, id, bearingRad, rangeInches, ageSec);
    }

    /**
     * Convenience helper to test whether this observation is considered
     * "fresh enough" for a given behavior.
     *
     * @param maxAgeSec maximum acceptable age in seconds
     * @return {@code true} if this observation has a target and
     * {@link #ageSec} is less than or equal to {@code maxAgeSec}
     */
    public boolean isFresh(double maxAgeSec) {
        return hasTarget && ageSec <= maxAgeSec;
    }

    /**
     * Robot-forward component of the tag position, in inches.
     *
     * <p>This is the distance along the robot's forward axis to the tag
     * (approximately {@code rangeInches * cos(bearingRad)}). If
     * {@link #hasTarget} is {@code false}, this returns 0.</p>
     *
     * <p>Advanced behaviors can use this value to build "drive to N inches in
     * front of the tag" logic without needing to perform their own trigonometry.</p>
     */
    public double forwardInches() {
        if (!hasTarget) return 0.0;
        return rangeInches * Math.cos(bearingRad);
    }

    /**
     * Robot-lateral component of the tag position, in inches.
     *
     * <p>This is the distance left/right from the robot's forward axis to the
     * tag (approximately {@code rangeInches * Math.sin(bearingRad)}). Positive
     * values mean the tag is to the left; negative values mean the tag is to
     * the right. If {@link #hasTarget} is {@code false}, this returns 0.</p>
     *
     * <p>Advanced holonomic drive logic can use this value for strafing to get
     * directly in front of a tag.</p>
     */
    public double lateralInches() {
        if (!hasTarget) return 0.0;
        return rangeInches * Math.sin(bearingRad);
    }

    @Override
    public String toString() {
        if (!hasTarget) {
            return "AprilTagObservation{no target, ageSec=" + ageSec + "}";
        }
        return "AprilTagObservation{"
                + "id=" + id
                + ", bearingRad=" + bearingRad
                + ", rangeInches=" + rangeInches
                + ", ageSec=" + ageSec
                + '}';
    }
}
