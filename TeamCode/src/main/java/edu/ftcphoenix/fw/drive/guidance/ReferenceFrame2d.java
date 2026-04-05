package edu.ftcphoenix.fw.drive.guidance;

/**
 * Semantic 2D reference frame for {@link DriveGuidance}.
 *
 * <p>A reference frame is an origin point plus an oriented local basis. Guidance can use that
 * information in several different ways:</p>
 * <ul>
 *   <li>drive to the frame origin</li>
 *   <li>drive to an offset expressed in the frame</li>
 *   <li>align the robot to the frame heading</li>
 *   <li>aim at the origin or an offset point while still using the frame heading separately</li>
 * </ul>
 *
 * <h2>Tag-relative heading meaning</h2>
 *
 * <p>When a frame is authored with {@link References#relativeToTagFrame(int, double, double, double)},
 * the supplied {@code headingRad} is the orientation of the reference frame’s {@code +X} axis in
 * the tag’s planar frame:</p>
 * <ul>
 *   <li>{@code 0} → the reference frame’s {@code +X} aligns with the tag’s {@code +X}
 *       (out from the tag face)</li>
 *   <li>{@code +pi/2} → the reference frame’s {@code +X} points tag-left</li>
 *   <li>{@code pi} → the reference frame’s {@code +X} points back toward the tag</li>
 * </ul>
 *
 * <p>This heading describes the <em>reference frame</em>, not automatically the robot’s desired
 * heading. Guidance decides how to use that heading based on whether you call
 * {@code referenceFrameOrigin(...)}, {@code referenceFrameOffsetInches(...)}, or
 * {@code referenceFrameHeading(...)}.</p>
 *
 * <h2>Common usage</h2>
 *
 * <pre>{@code
 * ReferenceFrame2d backdropFace = References.fieldFrame(48.0, 24.0, Math.PI);
 *
 * DriveGuidancePlan scoreAlign = DriveGuidance.plan()
 *         .translateTo()
 *             .referenceFrameOffsetInches(backdropFace, -6.0, 0.0)
 *             .doneTranslateTo()
 *         .aimTo()
 *             .referenceFrameHeading(backdropFace)
 *             .doneAimTo()
 *         .feedback()
 *             .fieldPose(poseEstimator)
 *             .doneFeedback()
 *         .build();
 * }</pre>
 *
 * @see ReferencePoint2d when only a point is meaningful
 * @see References
 */
public interface ReferenceFrame2d {
    // Marker interface for the public guidance API.
}
