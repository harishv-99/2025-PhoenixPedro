package edu.ftcphoenix.fw.field;

import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;

/**
 * Describes the fixed, known placement of AprilTags on a field.
 *
 * <p>This is <strong>field metadata</strong>, not a sensor output. It is used by localization
 * and other field-aware logic that needs known tag placements.</p>
 *
 * <h2>Field coordinate system (FTC)</h2>
 *
 * <p>Phoenix adopts the <b>FTC Field Coordinate System</b> for all field-centric poses stored in a
 * {@link TagLayout}. Importantly, the exact definition of the X/Y axes depends on the season
 * (diamond vs square vs inverted square). Phoenix therefore avoids hardcoding “+X means …” and
 * “+Y means …” here.</p>
 *
 * <p>Reference (FTC official docs):</p>
 * <pre>
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 * </pre>
 *
 * <p>What is consistent across seasons:</p>
 * <ul>
 *   <li>The origin is at the center of the field on the floor.</li>
 *   <li><b>+Z</b> is upward from the floor.</li>
 *   <li>All distances are in <b>inches</b>.</li>
 *   <li>All angles are in <b>radians</b>.</li>
 *   <li>Rotations use the right-hand rule about the field axes.</li>
 * </ul>
 *
 * <h2>ID mapping</h2>
 *
 * <p>The {@link TagPose#id} stored in this layout is the AprilTag's numeric ID code as reported
 * by the FTC SDK.</p>
 */
public interface TagLayout {

    /**
     * Returns the field placement for a given tag ID, or {@code null} if the tag is not known
     * by this layout.
     *
     * <p>Callers that require a tag to exist should use {@link #require(int)}.</p>
     *
     * @param id AprilTag numeric ID code
     * @return tag placement, or {@code null} if not present
     */
    TagPose get(int id);

    /**
     * Returns the set of all tag IDs contained in this layout.
     *
     * <p>Implementations should return an immutable set.</p>
     *
     * @return all IDs present in this layout
     */
    Set<Integer> ids();

    /**
     * Convenience helper: returns {@code true} if {@link #get(int)} returns a non-null value.
     *
     * @param id AprilTag numeric ID code
     */
    default boolean has(int id) {
        return get(id) != null;
    }

    /**
     * Returns the tag placement for {@code id}, throwing if the tag is not present.
     *
     * @param id AprilTag numeric ID code
     * @return tag placement
     * @throws IllegalArgumentException if the tag is not present in this layout
     */
    default TagPose require(int id) {
        TagPose pose = get(id);
        if (pose == null) {
            throw new IllegalArgumentException("TagLayout does not contain tag id=" + id);
        }
        return pose;
    }

    /**
     * Immutable description of a single AprilTag's known placement on the field (6DOF).
     *
     * <h2>Principled 6DOF representation</h2>
     *
     * <p>This class stores the tag placement as a {@link Pose3d} to keep the representation
     * consistent with other 6DOF concepts in Phoenix (e.g. {@code CameraMountConfig}).</p>
     *
     * <h2>What this represents</h2>
     * <ul>
     *   <li>{@link #id} is the AprilTag numeric ID code (as reported by FTC).</li>
     *   <li>{@link #fieldToTagPose()} is the <b>tag-center pose expressed in the FTC field frame</b>.</li>
     * </ul>
     *
     * <h2>Units</h2>
     * <ul>
     *   <li>Distances are inches</li>
     *   <li>Angles are radians</li>
     * </ul>
     *
     * <h2>Rotation conventions</h2>
     *
     * <p>The pose orientation uses yaw/pitch/roll in radians, using right-hand rule about the
     * <b>field axes</b>. Phoenix uses the same yaw/pitch/roll meaning as {@link Pose3d}:</p>
     *
     * <ul>
     *   <li><b>yawRad</b>: rotation about +Z</li>
     *   <li><b>pitchRad</b>: rotation about +Y</li>
     *   <li><b>rollRad</b>: rotation about +X</li>
     * </ul>
     *
     * <p>Most FTC games mount tags nearly vertical, but real fields and backboards can be slightly
     * tilted. Carrying full 6DOF here allows Phoenix localization to remain correct even when the
     * tag plane is not perfectly vertical.</p>
     */
    final class TagPose {

        /**
         * AprilTag numeric ID code.
         */
        public final int id;

        /**
         * Tag-center pose expressed in the FTC field frame (6DOF).
         *
         * <p>This is “field → tag” as a rigid transform: translation to the tag center plus
         * yaw/pitch/roll orientation in the field axes.</p>
         */
        private final Pose3d fieldToTag;

        private TagPose(int id, Pose3d fieldToTag) {
            if (id < 0) {
                throw new IllegalArgumentException("id must be non-negative");
            }
            this.id = id;
            this.fieldToTag = Objects.requireNonNull(fieldToTag, "fieldToTag");
        }

        /**
         * Creates a new immutable tag placement (6DOF) from a pose object.
         *
         * @param id         AprilTag numeric ID code (non-negative)
         * @param fieldToTag tag-center pose in the FTC field frame (non-null)
         * @return new {@link TagPose}
         */
        public static TagPose ofPose(int id, Pose3d fieldToTag) {
            return new TagPose(id, fieldToTag);
        }

        /**
         * Creates a new immutable tag placement (6DOF) from the six standard values.
         *
         * @param id       AprilTag numeric ID code (non-negative)
         * @param xInches  tag center X in the FTC field frame (inches)
         * @param yInches  tag center Y in the FTC field frame (inches)
         * @param zInches  tag center Z (height above floor) in the FTC field frame (inches)
         * @param yawRad   tag yaw about +Z in the FTC field frame (radians)
         * @param pitchRad tag pitch about +Y in the FTC field frame (radians)
         * @param rollRad  tag roll about +X in the FTC field frame (radians)
         * @return new {@link TagPose}
         */
        public static TagPose of(
                int id,
                double xInches,
                double yInches,
                double zInches,
                double yawRad,
                double pitchRad,
                double rollRad
        ) {
            return new TagPose(id, new Pose3d(xInches, yInches, zInches, yawRad, pitchRad, rollRad));
        }

        /**
         * Returns the tag-center pose expressed in the FTC field frame.
         */
        public Pose3d fieldToTagPose() {
            return fieldToTag;
        }

        /**
         * Tag center X coordinate in the FTC field frame (inches).
         */
        public double xInches() {
            return fieldToTag.xInches;
        }

        /**
         * Tag center Y coordinate in the FTC field frame (inches).
         */
        public double yInches() {
            return fieldToTag.yInches;
        }

        /**
         * Tag center Z coordinate (height above floor) in the FTC field frame (inches).
         */
        public double zInches() {
            return fieldToTag.zInches;
        }

        /**
         * Tag yaw (rotation about +Z) in the FTC field frame (radians).
         */
        public double yawRad() {
            return fieldToTag.yawRad;
        }

        /**
         * Tag pitch (rotation about +Y) in the FTC field frame (radians).
         */
        public double pitchRad() {
            return fieldToTag.pitchRad;
        }

        /**
         * Tag roll (rotation about +X) in the FTC field frame (radians).
         */
        public double rollRad() {
            return fieldToTag.rollRad;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "TagPose{id=" + id + ", fieldToTag=" + fieldToTag + "}";
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean equals(Object o) {
            if (!(o instanceof TagPose)) return false;
            TagPose other = (TagPose) o;
            return id == other.id && fieldToTag.equals(other.fieldToTag);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public int hashCode() {
            return Objects.hash(id, fieldToTag);
        }
    }
}
