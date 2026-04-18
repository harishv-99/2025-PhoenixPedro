package edu.ftcphoenix.fw.spatial;

/**
 * Semantic 2D aim target for Phoenix spatial queries.
 *
 * <p>An aim target answers one question: <em>which point or heading should the controlled aim frame
 * align to?</em> The target does not prescribe a drivetrain command, turret command, or plant
 * target. It is simply the semantic thing to be aimed at.</p>
 *
 * <p>Most teams do not implement this interface directly. Instead, use the factory helpers in
 * {@link SpatialTargets}.</p>
 */
public interface AimTarget2d {
    // Marker interface for the public spatial-query API.
}
