package edu.ftcphoenix.fw.drive.guidance;

/**
 * Semantic 2D reference point for {@link DriveGuidance}.
 *
 * <p>A reference point answers only one question: <em>where is the meaningful point?</em> It does
 * <strong>not</strong> say what the robot should do there. Guidance may translate to the point,
 * aim at the point, use it as part of a readiness query, or ignore it entirely until some other
 * system asks for it.</p>
 *
 * <h2>Common usage</h2>
 *
 * <pre>{@code
 * ReferencePoint2d speakerAim = References.fieldPoint(48.0, 24.0);
 *
 * DriveGuidancePlan aimPlan = DriveGuidance.plan()
 *         .aimTo()
 *             .referencePoint(speakerAim)
 *             .doneAimTo()
 *         .feedback()
 *             .fieldPose(poseEstimator)
 *             .doneFeedback()
 *         .build();
 * }</pre>
 *
 * <p>Reference points can be authored in field coordinates or relative to a tag through
 * {@link References} factory helpers.</p>
 *
 * @see ReferenceFrame2d when the reference also has a meaningful local orientation
 * @see References
 */
public interface ReferencePoint2d {
    // Marker interface for the public guidance API.
}
