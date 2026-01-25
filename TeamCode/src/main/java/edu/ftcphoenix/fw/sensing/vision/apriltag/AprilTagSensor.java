package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Objects;
import java.util.Set;

/**
 * High-level interface for reading AprilTag observations in robot code.
 *
 * <p>This interface is deliberately decoupled from the FTC vision APIs. Robot
 * code talks only to {@code AprilTagSensor} and {@link AprilTagObservation},
 * while the FTC-specific details (VisionPortal, AprilTagProcessor, units,
 * metadata checks, etc.) are handled in an adapter layer.</p>
 *
 * <h2>Key ideas</h2>
 *
 * <ul>
 *   <li>Robot code typically cares about:
 *     <ul>
 *       <li>Which tag (ID) is currently visible.</li>
 *       <li>Whether that tag is in a set of IDs we care about (e.g., goals).</li>
 *       <li>Distance to that tag (for shooter control or standoff distance).</li>
 *       <li>Bearing to that tag (for aiming).</li>
 *     </ul>
 *   </li>
 *   <li>The sensor returns a single {@link AprilTagObservation} representing
 *       the <em>best</em> available tag that matches the caller's criteria.</li>
 *   <li>If no suitable tag is available, the observation has
 *       {@link AprilTagObservation#hasTarget} set to {@code false}.</li>
 * </ul>
 *
 * <h2>Definition of "best"</h2>
 *
 * <p>The concrete implementation (for example, the FTC adapter) is responsible
 * for selecting the best tag. The recommended policy, which the FTC adapter
 * in this framework follows, is:</p>
 *
 * <ol>
 *   <li>Look at the <em>most recent</em> camera frame that has any detections.</li>
 *   <li>Filter detections by the requested ID set, if provided.</li>
 *   <li>Among the remaining detections, choose the one with the smallest
 *       {@link AprilTagObservation#cameraRangeInches()} (closest tag).</li>
 *   <li>Construct an {@link AprilTagObservation} using the chosen detection's
 *       ID, camera-frame pose, and frame age (bearing/range are derived from the pose).</li>
 * </ol>
 *
 * <p>This behavior gives predictable results for common use-cases:
 * selecting a scoring tag from a small set of IDs, aiming at the closest goal,
 * or using the distance to drive shooter velocity.</p>
 *
 * <h2>Usage examples</h2>
 *
 * <h3>Any visible tag</h3>
 *
 * <pre>{@code
 * // Ask for the closest tag of any ID, as long as it is not older than 0.3 s.
 * AprilTagObservation obs = sensor.bestAny(0.3);
 * if (obs.hasTarget) {
 *     telemetry.addData("Tag ID", obs.id);
 *     telemetry.addData("Range (in)", obs.cameraRangeInches());
 * }
 * }</pre>
 *
 * <h3>Tag from a set of IDs (e.g., scoring tags)</h3>
 *
 * <pre>{@code
 * Set<Integer> scoringTags = Set.of(1, 2, 3);
 * AprilTagObservation obs = sensor.best(scoringTags, 0.5);
 * if (obs.hasTarget) {
 *     // Shooter logic: map range (inches) to flywheel RPM.
 *     double distanceInches = obs.cameraRangeInches();
 * }
 * }</pre>
 */
public interface AprilTagSensor {

    /**
     * Get the best observation for any visible tag, subject to a freshness
     * constraint.
     *
     * <p>If there is a suitable tag whose age is less than or equal to
     * {@code maxAgeSec}, this returns an {@link AprilTagObservation} with
     * {@link AprilTagObservation#hasTarget} set to {@code true}. Otherwise it
     * returns an observation with {@code hasTarget} set to {@code false}.</p>
     *
     * @param maxAgeSec maximum acceptable age of the underlying camera frame,
     *                  in seconds (for example, 0.3)
     * @return the best observation for any tag, or a "no target" observation
     */
    AprilTagObservation bestAny(double maxAgeSec);

    /**
     * Get the best observation for tags whose IDs are in {@code idsOfInterest},
     * subject to a freshness constraint.
     *
     * <p>This is the primary entry point for most robot behaviors that care
     * about specific targets, such as scoring goals, backdrops, or field
     * landmarks.</p>
     *
     * <p>If there is a suitable tag whose ID is contained in {@code idsOfInterest}
     * and whose age is less than or equal to {@code maxAgeSec}, this returns an
     * {@link AprilTagObservation} with {@link AprilTagObservation#hasTarget}
     * set to {@code true}. Otherwise it returns an observation with
     * {@code hasTarget} set to {@code false}.</p>
     *
     * @param idsOfInterest set of tag IDs the caller cares about; must not be null
     * @param maxAgeSec     maximum acceptable age of the underlying camera frame,
     *                      in seconds (for example, 0.3)
     * @return the best observation for tags in {@code idsOfInterest}, or a
     * "no target" observation
     */
    AprilTagObservation best(Set<Integer> idsOfInterest, double maxAgeSec);

    // ---------------------------------------------------------------------
    // Convenience helpers
    // ---------------------------------------------------------------------

    /**
     * Convenience overload for the common case of "any tag with this single
     * ID", subject to a freshness constraint.
     *
     * <p>Equivalent to calling {@link #best(Set, double)} with
     * {@code Set.of(id)}.</p>
     *
     * @param id        single tag ID of interest
     * @param maxAgeSec maximum acceptable age in seconds
     * @return the best observation for the given ID, or a "no target" observation
     */
    default AprilTagObservation best(int id, double maxAgeSec) {
        return best(Set.of(id), maxAgeSec);
    }

    /**
     * Convenience method to test whether there is any fresh tag, regardless
     * of ID.
     *
     * @param maxAgeSec maximum acceptable age in seconds
     * @return {@code true} if {@link #bestAny(double)} returns an observation
     * with {@link AprilTagObservation#hasTarget} set to {@code true}
     */
    default boolean hasFreshAny(double maxAgeSec) {
        return bestAny(maxAgeSec).hasTarget;
    }

    /**
     * Convenience method to test whether there is a fresh tag whose ID is in
     * {@code idsOfInterest}.
     *
     * @param idsOfInterest set of tag IDs of interest; must not be null
     * @param maxAgeSec     maximum acceptable age in seconds
     * @return {@code true} if {@link #best(Set, double)} returns an observation
     * with {@link AprilTagObservation#hasTarget} set to {@code true}
     */
    default boolean hasFresh(Set<Integer> idsOfInterest, double maxAgeSec) {
        Objects.requireNonNull(idsOfInterest, "idsOfInterest");
        return best(idsOfInterest, maxAgeSec).hasTarget;
    }

    /**
     * Convenience method to test whether there is a fresh tag with the given
     * single ID.
     *
     * @param id        single tag ID of interest
     * @param maxAgeSec maximum acceptable age in seconds
     * @return {@code true} if {@link #best(int, double)} returns an observation
     * with {@link AprilTagObservation#hasTarget} set to {@code true}
     */
    default boolean hasFresh(int id, double maxAgeSec) {
        return best(id, maxAgeSec).hasTarget;
    }

    // ---------------------------------------------------------------------
    // Resource lifecycle
    // ---------------------------------------------------------------------

    /**
     * Release any underlying resources held by this sensor.
     *
     * <p>Most implementations are lightweight and do not need explicit cleanup.
     * However, FTC vision adapters typically allocate native camera/streaming
     * resources (for example, a {@code VisionPortal}). If those are not released,
     * later code may fail to start a new portal with errors like “multiple vision
     * portals”.</p>
     *
     * <p>This method is:</p>
     * <ul>
     *   <li><b>Optional</b> for callers (safe to ignore if you never create more than one).</li>
     *   <li><b>Safe to call multiple times</b>.</li>
     *   <li><b>Non-throwing</b> by contract (implementations should swallow/guard exceptions).</li>
     * </ul>
     */
    default void close() {
        // Default: no-op.
    }
}
