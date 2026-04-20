package edu.ftcphoenix.fw.spatial;

/**
 * Semantic 2D reference point for Phoenix spatial queries and guidance.
 *
 * <p>A reference point answers only one question: <em>where is the meaningful point?</em> It does
 * <strong>not</strong> say how that point will be solved. The same immutable reference may be
 * translated to, aimed at, checked by a query, or ignored until some other system asks for it.</p>
 *
 * <h2>Common usage</h2>
 *
 * <pre>{@code
 * ReferencePoint2d speakerAim = References.fieldPoint(48.0, 24.0);
 *
 * DriveGuidancePlan aimPlan = DriveGuidance.plan()
 *         .faceTo()
 *             .point(speakerAim)
 *             .doneFaceTo()
 *         .solveWith()
 *             .localizationOnly()
 *             .localization(poseEstimator)
 *             .doneAprilTagsOnly()
 *         .build();
 * }</pre>
 *
 * <p>References can be authored in field coordinates, relative to one fixed AprilTag ID, or
 * relative to a shared {@code TagSelectionSource}. The point object itself remains immutable;
 * selected-tag references resolve through the selector's current state at evaluation time.</p>
 *
 * @see ReferenceFrame2d when the reference also has a meaningful local orientation
 * @see References
 */
public interface ReferencePoint2d {
    // Marker interface for the public spatial-query API.
}
