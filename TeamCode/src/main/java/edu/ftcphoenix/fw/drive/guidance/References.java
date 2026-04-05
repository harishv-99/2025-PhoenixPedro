package edu.ftcphoenix.fw.drive.guidance;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.TagLayout;

/**
 * Factory helpers for semantic {@link DriveGuidance} references.
 *
 * <p>Phoenix supports two broad authoring styles:</p>
 * <ul>
 *   <li><b>Field-fixed references</b> — define the point/frame directly in field coordinates.</li>
 *   <li><b>Tag-relative references</b> — define the point/frame in an AprilTag's planar frame.</li>
 * </ul>
 *
 * <p>These helpers intentionally describe <em>geometry</em>, not robot behavior. The same
 * reference can later be used to translate, aim, query readiness, or build telemetry.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * ReferencePoint2d speakerAim = References.fieldPoint(48.0, 24.0);
 * ReferenceFrame2d slotFace = References.relativeToTagFrame(5, 6.0, 0.0, Math.PI);
 *
 * DriveGuidancePlan plan = DriveGuidance.plan()
 *         .translateTo()
 *             .referenceFrameOffsetInches(slotFace, -4.0, 0.0)
 *             .doneTranslateTo()
 *         .aimTo()
 *             .referencePoint(speakerAim)
 *             .doneAimTo()
 *         .feedback()
 *             .fieldPose(poseEstimator)
 *             .aprilTags(tagSensor, cameraMount, 0.25)
 *             .fixedTagLayout(tagLayout)
 *             .doneFeedback()
 *         .build();
 * }</pre>
 *
 * <h2>Tag-relative frame headings</h2>
 *
 * <p>For tag-relative frames, {@code headingRad} describes the orientation of the reference
 * frame's <b>+X axis</b> in the tag's planar frame:</p>
 * <ul>
 *   <li>{@code 0} means the reference frame's {@code +X} aligns with the tag's {@code +X}
 *       (out from the tag face)</li>
 *   <li>{@code +pi/2} means the reference frame's {@code +X} points tag-left</li>
 *   <li>{@code pi} means the reference frame's {@code +X} points back toward the tag</li>
 * </ul>
 *
 * <p>If you later tell guidance to align the robot to that frame, the frame heading is the thing
 * being matched. It is <strong>not</strong> implicitly a robot-heading command at creation time.</p>
 */
public final class References {

    private References() {
        // Utility class.
    }

    /**
     * Creates a field-fixed semantic point.
     *
     * <p>Use this when the meaningful location is naturally described in field coordinates, such as
     * a scoring location, intake waypoint, or aim point.</p>
     *
     * @param xInches field X coordinate in inches
     * @param yInches field Y coordinate in inches
     * @return immutable semantic point reference
     */
    public static ReferencePoint2d fieldPoint(double xInches, double yInches) {
        return new FieldPointRef(xInches, yInches);
    }

    /**
     * Creates a field-fixed semantic frame.
     *
     * <p>The frame contains both an origin and a heading. This is useful when you want to express
     * both “go here” and “align to this face” with one shared semantic reference.</p>
     *
     * @param xInches    field X coordinate of the frame origin
     * @param yInches    field Y coordinate of the frame origin
     * @param headingRad heading of the frame's {@code +X} axis in field coordinates
     * @return immutable semantic frame reference
     */
    public static ReferenceFrame2d fieldFrame(double xInches, double yInches, double headingRad) {
        return new FieldFrameRef(xInches, yInches, headingRad);
    }

    /**
     * Creates a point relative to a single tag's planar frame.
     *
     * <p>This is the most common authoring path for “go to / aim at an offset from tag X”.</p>
     *
     * @param tagId         AprilTag ID that defines the local frame
     * @param forwardInches point X in the tag frame (positive out from the tag face)
     * @param leftInches    point Y in the tag frame (positive tag-left)
     * @return immutable semantic point reference
     */
    public static ReferencePoint2d relativeToTagPoint(int tagId, double forwardInches, double leftInches) {
        return relativeToTagsPoint(Collections.singleton(tagId), forwardInches, leftInches);
    }

    /**
     * Creates a frame relative to a single tag's planar frame.
     *
     * @param tagId         AprilTag ID that defines the local frame
     * @param forwardInches frame-origin X in the tag frame
     * @param leftInches    frame-origin Y in the tag frame
     * @param headingRad    heading of the reference frame's {@code +X} axis in the tag frame
     * @return immutable semantic frame reference
     */
    public static ReferenceFrame2d relativeToTagFrame(int tagId,
                                                      double forwardInches,
                                                      double leftInches,
                                                      double headingRad) {
        return relativeToTagsFrame(Collections.singleton(tagId), forwardInches, leftInches, headingRad);
    }

    /**
     * Creates a point resolved from whichever one of {@code tagIds} is currently visible.
     *
     * <p>This is a convenience for use-cases like “aim at the center of any scoring tag”.
     * Because the chosen visible tag may vary at runtime, this form is solved from live AprilTag
     * sensing and is not automatically promoted into a single field-fixed point.</p>
     *
     * @param tagIds        candidate AprilTag IDs; order is preserved only for debug readability
     * @param forwardInches point X in the chosen tag frame
     * @param leftInches    point Y in the chosen tag frame
     * @return immutable semantic point reference
     */
    public static ReferencePoint2d relativeToTagsPoint(Set<Integer> tagIds,
                                                       double forwardInches,
                                                       double leftInches) {
        return new TagPointRef(normalizeTagIds(tagIds), forwardInches, leftInches);
    }

    /**
     * Creates a frame resolved from whichever one of {@code tagIds} is currently visible.
     *
     * <p>Like {@link #relativeToTagsPoint(Set, double, double)}, this is intended for cases where
     * the semantic thing you care about can be authored the same way relative to any visible tag.</p>
     *
     * @param tagIds        candidate AprilTag IDs
     * @param forwardInches frame-origin X in the chosen tag frame
     * @param leftInches    frame-origin Y in the chosen tag frame
     * @param headingRad    heading of the reference frame's {@code +X} axis in the chosen tag frame
     * @return immutable semantic frame reference
     */
    public static ReferenceFrame2d relativeToTagsFrame(Set<Integer> tagIds,
                                                       double forwardInches,
                                                       double leftInches,
                                                       double headingRad) {
        return new TagFrameRef(normalizeTagIds(tagIds), forwardInches, leftInches, headingRad);
    }

    /**
     * @return whether this reference is already field-fixed
     */
    static boolean isFieldPoint(ReferencePoint2d ref) {
        return ref instanceof FieldPointRef;
    }

    /**
     * @return whether this reference frame is already field-fixed
     */
    static boolean isFieldFrame(ReferenceFrame2d ref) {
        return ref instanceof FieldFrameRef;
    }

    /**
     * @return whether this point reference is authored in a tag frame
     */
    static boolean isTagRelativePoint(ReferencePoint2d ref) {
        return ref instanceof TagPointRef;
    }

    /**
     * @return whether this frame reference is authored in a tag frame
     */
    static boolean isTagRelativeFrame(ReferenceFrame2d ref) {
        return ref instanceof TagFrameRef;
    }

    /**
     * Returns the candidate tag IDs for a tag-relative point reference, or an empty set for a
     * field-fixed point.
     */
    static Set<Integer> tagIds(ReferencePoint2d ref) {
        if (ref instanceof TagPointRef) {
            return ((TagPointRef) ref).tagIds;
        }
        return Collections.emptySet();
    }

    /**
     * Returns the candidate tag IDs for a tag-relative frame reference, or an empty set for a
     * field-fixed frame.
     */
    static Set<Integer> tagIds(ReferenceFrame2d ref) {
        if (ref instanceof TagFrameRef) {
            return ((TagFrameRef) ref).tagIds;
        }
        return Collections.emptySet();
    }

    /**
     * Returns {@code true} when this point reference is relative to exactly one tag and that tag is
     * present in {@code layout}. Guidance uses this to determine whether localizer fallback can
     * promote the tag-relative point into a field-fixed point.
     */
    static boolean isSingleFixedTagCandidate(ReferencePoint2d ref, TagLayout layout) {
        Integer id = singleTagIdOrNull(ref);
        return id != null && layout != null && layout.has(id);
    }

    /**
     * Same as {@link #isSingleFixedTagCandidate(ReferencePoint2d, TagLayout)} but for frames.
     */
    static boolean isSingleFixedTagCandidate(ReferenceFrame2d ref, TagLayout layout) {
        Integer id = singleTagIdOrNull(ref);
        return id != null && layout != null && layout.has(id);
    }

    /**
     * Returns the unique tag ID for a single-tag point reference, otherwise {@code null}.
     */
    static Integer singleTagIdOrNull(ReferencePoint2d ref) {
        if (ref instanceof TagPointRef) {
            Set<Integer> tagIds = ((TagPointRef) ref).tagIds;
            return tagIds.size() == 1 ? tagIds.iterator().next() : null;
        }
        return null;
    }

    /**
     * Returns the unique tag ID for a single-tag frame reference, otherwise {@code null}.
     */
    static Integer singleTagIdOrNull(ReferenceFrame2d ref) {
        if (ref instanceof TagFrameRef) {
            Set<Integer> tagIds = ((TagFrameRef) ref).tagIds;
            return tagIds.size() == 1 ? tagIds.iterator().next() : null;
        }
        return null;
    }

    /**
     * Attempts to express a semantic point in field coordinates.
     *
     * <p>This succeeds immediately for field-fixed points. For single-tag-relative points it also
     * succeeds when {@code layout} knows the fixed field pose of that tag, letting guidance fall
     * back through a localizer when the tag is no longer visible.</p>
     *
     * @param ref    semantic point reference
     * @param layout fixed tag layout used to promote fixed tag-relative points
     * @return field-fixed point pose, or {@code null} when no field interpretation is available
     */
    static Pose2d tryResolveFieldPoint(ReferencePoint2d ref, TagLayout layout) {
        if (ref instanceof FieldPointRef) {
            FieldPointRef fp = (FieldPointRef) ref;
            return new Pose2d(fp.xInches, fp.yInches, 0.0);
        }
        if (ref instanceof TagPointRef) {
            Integer tagId = singleTagIdOrNull(ref);
            if (tagId == null || layout == null) {
                return null;
            }
            Pose3d fieldToTag = layout.getFieldToTagPose(tagId);
            if (fieldToTag == null) {
                return null;
            }
            TagPointRef tp = (TagPointRef) ref;
            return fieldToTag.toPose2d().then(new Pose2d(tp.forwardInches, tp.leftInches, 0.0));
        }
        return null;
    }

    /**
     * Attempts to express a semantic frame in field coordinates.
     *
     * <p>This mirrors {@link #tryResolveFieldPoint(ReferencePoint2d, TagLayout)} for frame-shaped
     * references.</p>
     *
     * @param ref    semantic frame reference
     * @param layout fixed tag layout used to promote fixed tag-relative frames
     * @return field-fixed frame pose, or {@code null} when no field interpretation is available
     */
    static Pose2d tryResolveFieldFrame(ReferenceFrame2d ref, TagLayout layout) {
        if (ref instanceof FieldFrameRef) {
            FieldFrameRef ff = (FieldFrameRef) ref;
            return new Pose2d(ff.xInches, ff.yInches, ff.headingRad);
        }
        if (ref instanceof TagFrameRef) {
            Integer tagId = singleTagIdOrNull(ref);
            if (tagId == null || layout == null) {
                return null;
            }
            Pose3d fieldToTag = layout.getFieldToTagPose(tagId);
            if (fieldToTag == null) {
                return null;
            }
            TagFrameRef tf = (TagFrameRef) ref;
            return fieldToTag.toPose2d().then(new Pose2d(tf.forwardInches, tf.leftInches, tf.headingRad));
        }
        return null;
    }

    /**
     * Returns the point transform ({@code tag -> point}) for a tag-relative point reference.
     *
     * @param ref semantic point reference
     * @return tag-relative pose, or {@code null} for field-fixed references
     */
    static Pose2d tagToPoint(ReferencePoint2d ref) {
        if (ref instanceof TagPointRef) {
            TagPointRef tp = (TagPointRef) ref;
            return new Pose2d(tp.forwardInches, tp.leftInches, 0.0);
        }
        return null;
    }

    /**
     * Returns the frame transform ({@code tag -> frame}) for a tag-relative frame reference.
     *
     * @param ref semantic frame reference
     * @return tag-relative frame pose, or {@code null} for field-fixed references
     */
    static Pose2d tagToFrame(ReferenceFrame2d ref) {
        if (ref instanceof TagFrameRef) {
            TagFrameRef tf = (TagFrameRef) ref;
            return new Pose2d(tf.forwardInches, tf.leftInches, tf.headingRad);
        }
        return null;
    }

    private static Set<Integer> normalizeTagIds(Set<Integer> tagIds) {
        Objects.requireNonNull(tagIds, "tagIds");
        if (tagIds.isEmpty()) {
            throw new IllegalArgumentException("tagIds must not be empty");
        }
        LinkedHashSet<Integer> out = new LinkedHashSet<>();
        for (Integer id : tagIds) {
            if (id == null) {
                throw new IllegalArgumentException("tagIds must not contain null");
            }
            if (id < 0) {
                throw new IllegalArgumentException("tagIds must be non-negative");
            }
            out.add(id);
        }
        return Collections.unmodifiableSet(out);
    }

    /**
     * Internal field-fixed point implementation.
     */
    static final class FieldPointRef implements ReferencePoint2d {
        final double xInches;
        final double yInches;

        FieldPointRef(double xInches, double yInches) {
            this.xInches = xInches;
            this.yInches = yInches;
        }

        @Override
        public String toString() {
            return "FieldPointRef{xInches=" + xInches + ", yInches=" + yInches + "}";
        }
    }

    /**
     * Internal field-fixed frame implementation.
     */
    static final class FieldFrameRef implements ReferenceFrame2d {
        final double xInches;
        final double yInches;
        final double headingRad;

        FieldFrameRef(double xInches, double yInches, double headingRad) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRad = headingRad;
        }

        @Override
        public String toString() {
            return "FieldFrameRef{xInches=" + xInches + ", yInches=" + yInches
                    + ", headingRad=" + headingRad + "}";
        }
    }

    /**
     * Internal tag-relative point implementation.
     */
    static final class TagPointRef implements ReferencePoint2d {
        final Set<Integer> tagIds;
        final double forwardInches;
        final double leftInches;

        TagPointRef(Set<Integer> tagIds, double forwardInches, double leftInches) {
            this.tagIds = tagIds;
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }

        @Override
        public String toString() {
            return "TagPointRef{tagIds=" + tagIds + ", forwardInches=" + forwardInches
                    + ", leftInches=" + leftInches + "}";
        }
    }

    /**
     * Internal tag-relative frame implementation.
     */
    static final class TagFrameRef implements ReferenceFrame2d {
        final Set<Integer> tagIds;
        final double forwardInches;
        final double leftInches;
        final double headingRad;

        TagFrameRef(Set<Integer> tagIds, double forwardInches, double leftInches, double headingRad) {
            this.tagIds = tagIds;
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
            this.headingRad = headingRad;
        }

        @Override
        public String toString() {
            return "TagFrameRef{tagIds=" + tagIds + ", forwardInches=" + forwardInches
                    + ", leftInches=" + leftInches + ", headingRad=" + headingRad + "}";
        }
    }
}
