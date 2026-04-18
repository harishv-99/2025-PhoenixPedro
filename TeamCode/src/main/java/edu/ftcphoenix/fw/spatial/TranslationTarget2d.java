package edu.ftcphoenix.fw.spatial;

/**
 * Semantic 2D translation target for Phoenix spatial queries.
 *
 * <p>A translation target answers one question: <em>which point should the controlled translation
 * frame move toward?</em> The target itself is immutable and controller-neutral. A consumer such as
 * {@code DriveGuidance}, a turret planner, or a floor-pickup planner decides how to use the solved
 * relationship.</p>
 *
 * <p>Most teams do not implement this interface directly. Instead, use the factory helpers in
 * {@link SpatialTargets}.</p>
 */
public interface TranslationTarget2d {
    // Marker interface for the public spatial-query API.
}
