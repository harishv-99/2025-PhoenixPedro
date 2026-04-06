package edu.ftcphoenix.fw.field;

import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;

/**
 * Describes the fixed, known placement of AprilTags on a field.
 *
 * <p>This is <strong>field metadata</strong>, not a sensor output. A {@link TagLayout} answers one
 * question: for a given AprilTag ID, where is that tag mounted in the FTC field frame?</p>
 *
 * <p><b>Important:</b> this interface is only for tags that are safe to treat as
 * <em>fixed field landmarks</em>. If a season includes AprilTags that are useful for identification
 * or aiming but whose exact placement can vary from match to match, those tags should remain
 * available through raw detection/selection, but they must be omitted from the {@link TagLayout}
 * used by localization or AprilTag→field-pose solving.</p>
 *
 * <h2>What lives here</h2>
 * <ul>
 *   <li><b>AprilTag ID</b> — the numeric ID reported by the vision pipeline.</li>
 *   <li><b>Field → tag pose</b> — the rigid transform from the FTC field frame to the tag center.</li>
 * </ul>
 *
 * <p>Phoenix keeps tag placements in full 6DOF ({@link Pose3d}) instead of flattening immediately
 * to {@code Pose2d}. That keeps field metadata compatible with camera-mount geometry and with
 * localizers that care about the tag plane not being perfectly vertical.</p>
 *
 * <h2>Coordinate system</h2>
 *
 * <p>The stored poses use the FTC field coordinate system for the current game. Phoenix avoids
 * hard-coding “+X means audience side” style language here because FTC's published X/Y axis
 * directions can vary by season. What is stable across seasons is:</p>
 * <ul>
 *   <li>origin at the center of the field on the floor</li>
 *   <li>{@code +Z} up from the floor</li>
 *   <li>distances in <b>inches</b></li>
 *   <li>angles in <b>radians</b></li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <p>Most robot code does not implement {@link TagLayout} directly. Teams usually use either:</p>
 * <ul>
 *   <li>{@link edu.ftcphoenix.fw.ftc.FtcGameTagLayout} for official FTC game layouts,</li>
 *   <li>{@link SimpleTagLayout} for tests, practice setups, or custom calibration rigs, or</li>
 *   <li>{@link TagLayouts#subset(TagLayout, java.util.Set)} when one behavior should only see a
 *       curated fixed-tag subset without duplicating the underlying metadata.</li>
 * </ul>
 *
 * <p>Example:</p>
 * <pre>{@code
 * TagLayout layout = new SimpleTagLayout()
 *         .add(5, 48.0, 0.0, 18.0, Math.PI, 0.0, 0.0)
 *         .add(6, 48.0, 24.0, 18.0, Math.PI, 0.0, 0.0);
 *
 * Pose3d fieldToTag5 = layout.requireFieldToTagPose(5);
 * }</pre>
 *
 * <p>Callers that only need planar math can project with {@link Pose3d#toPose2d()} at the usage
 * site.</p>
 */
public interface TagLayout {

    /**
     * Returns the fixed field pose ({@code field -> tag}) for {@code id}, or {@code null} if the
     * tag is not present in this layout.
     *
     * <p>Use {@link #requireFieldToTagPose(int)} when absence should be treated as a bug or
     * misconfiguration.</p>
     *
     * @param id AprilTag numeric ID code
     * @return immutable field pose for that tag, or {@code null} when unknown
     */
    Pose3d getFieldToTagPose(int id);

    /**
     * Returns the set of all tag IDs contained in this layout.
     *
     * <p>Implementations should return an immutable set or an effectively immutable view.</p>
     *
     * @return all IDs present in this layout
     */
    Set<Integer> ids();

    /**
     * Convenience helper: {@code true} when {@link #getFieldToTagPose(int)} returns a pose.
     *
     * @param id AprilTag numeric ID code
     * @return whether this layout contains that tag
     */
    default boolean has(int id) {
        return getFieldToTagPose(id) != null;
    }

    /**
     * Returns the fixed field pose ({@code field -> tag}) for {@code id}, throwing if the tag is
     * not present.
     *
     * <p>This is the right helper when a missing tag means the layout or robot code was configured
     * incorrectly and the caller wants an actionable error immediately.</p>
     *
     * @param id AprilTag numeric ID code
     * @return immutable field pose for that tag
     * @throws IllegalArgumentException if the layout does not contain {@code id}
     */
    default Pose3d requireFieldToTagPose(int id) {
        Pose3d pose = getFieldToTagPose(id);
        if (pose == null) {
            throw new IllegalArgumentException("TagLayout does not contain tag id=" + id);
        }
        return pose;
    }
}
