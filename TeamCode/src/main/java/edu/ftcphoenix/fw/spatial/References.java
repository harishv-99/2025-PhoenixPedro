package edu.ftcphoenix.fw.spatial;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;

/**
 * Factory helpers for semantic guidance references.
 *
 * <p>{@link ReferencePoint2d} and {@link ReferenceFrame2d} deliberately describe only
 * <em>geometry</em>, not robot behavior. The same reference can later be used to translate,
 * aim, query readiness, or drive telemetry.</p>
 *
 * <h2>Guiding principles</h2>
 * <ul>
 *   <li><b>One obvious geometry story:</b> references describe field-fixed points / frames,
 *       one-tag-relative points / frames, or points / frames relative to one shared selected-tag
 *       source.</li>
 *   <li><b>No hidden multi-tag magic inside references:</b> if more than one tag may matter,
 *       selection happens explicitly in a {@link TagSelectionSource}. A reference still resolves
 *       through exactly one tag at any instant.</li>
 *   <li><b>Geometry stays reusable:</b> selected-tag references can be solved from live vision,
 *       from localization after a tag has been selected, or adaptively from both.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * ReferenceFrame2d slotFace = References.fieldFrame(48.0, 24.0, Math.PI);
 * ReferencePoint2d settlePoint = References.framePoint(slotFace, -6.0, 0.0);
 *
 * DriveGuidancePlan alignPlan = DriveGuidance.plan()
 *         .translateTo()
 *             .point(settlePoint)
 *             .doneTranslateTo()
 *         .aimTo()
 *             .frameHeading(slotFace)
 *             .doneAimTo()
 *         .resolveWith()
 *             .localization(poseEstimator)
 *             .doneResolveWith()
 *         .build();
 * }</pre>
 */
public final class References {

    /**
     * Immutable per-tag point offset used by selected-tag point references.
     */
    public static final class TagPointOffset {
        public final double forwardInches;
        public final double leftInches;

        /**
         * Creates a per-tag point offset in tag-local forward/left coordinates.
         */
        public TagPointOffset(double forwardInches, double leftInches) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "TagPointOffset{forwardInches=" + forwardInches + ", leftInches=" + leftInches + "}";
        }
    }

    /**
     * Immutable per-tag frame offset used by selected-tag frame references.
     */
    public static final class TagFrameOffset {
        public final double forwardInches;
        public final double leftInches;
        public final double headingRad;

        /**
         * Creates a per-tag frame offset in tag-local forward/left/heading coordinates.
         */
        public TagFrameOffset(double forwardInches, double leftInches, double headingRad) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
            this.headingRad = headingRad;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "TagFrameOffset{forwardInches=" + forwardInches + ", leftInches=" + leftInches
                    + ", headingRad=" + headingRad + "}";
        }
    }

    private References() {
        // Utility class.
    }

    /**
     * Convenience factory for a per-tag point offset.
     */
    public static TagPointOffset pointOffset(double forwardInches, double leftInches) {
        return new TagPointOffset(forwardInches, leftInches);
    }

    /**
     * Convenience factory for a per-tag frame offset.
     */
    public static TagFrameOffset frameOffset(double forwardInches, double leftInches, double headingRad) {
        return new TagFrameOffset(forwardInches, leftInches, headingRad);
    }

    /**
     * Creates a field-fixed semantic point.
     */
    public static ReferencePoint2d fieldPoint(double xInches, double yInches) {
        return new FieldPointRef(xInches, yInches);
    }

    /**
     * Creates a field-fixed semantic frame.
     */
    public static ReferenceFrame2d fieldFrame(double xInches, double yInches, double headingRad) {
        return new FieldFrameRef(xInches, yInches, headingRad);
    }

    /**
     * Creates a point at the origin of a semantic frame.
     */
    public static ReferencePoint2d framePoint(ReferenceFrame2d frame) {
        return framePoint(frame, 0.0, 0.0);
    }

    /**
     * Creates a point expressed as an offset in a semantic frame.
     *
     * <p>Example: “settle 6 inches in front of this frame origin”. This keeps point geometry in
     * {@code References} rather than spreading frame-origin / frame-offset authoring across the
     * drive-guidance builder surface.</p>
     */
    public static ReferencePoint2d framePoint(ReferenceFrame2d frame,
                                              double forwardInches,
                                              double leftInches) {
        return new FramePointRef(Objects.requireNonNull(frame, "frame"), forwardInches, leftInches);
    }

    /**
     * Creates a point relative to a single tag's planar frame.
     */
    public static ReferencePoint2d relativeToTagPoint(int tagId, double forwardInches, double leftInches) {
        return new TagPointRef(normalizeTagId(tagId), forwardInches, leftInches);
    }

    /**
     * Creates a frame relative to a single tag's planar frame.
     */
    public static ReferenceFrame2d relativeToTagFrame(int tagId,
                                                      double forwardInches,
                                                      double leftInches,
                                                      double headingRad) {
        return new TagFrameRef(normalizeTagId(tagId), forwardInches, leftInches, headingRad);
    }

    /**
     * Creates a point relative to the tag currently selected by {@code selection}, using one
     * common offset for every candidate tag.
     *
     * <p>When this reference is later promoted through localization, Phoenix currently requires
     * every candidate ID exposed by {@code selection} to be present in the fixed layout. That
     * keeps the reference's localization capability stable instead of depending on which tag
     * happened to be selected this cycle.</p>
     */
    public static ReferencePoint2d relativeToSelectedTagPoint(TagSelectionSource selection,
                                                              double forwardInches,
                                                              double leftInches) {
        return new SelectedTagPointRef(selection, new FixedPointLookup(forwardInches, leftInches));
    }

    /**
     * Creates a point relative to the tag currently selected by {@code selection}, with a
     * potentially different offset per candidate tag.
     *
     * <p>This is useful when the semantic target is “the thing behind whichever scoring tag we
     * selected”, but the offset differs across tags because the tags sit at different angles or on
     * mirrored sides of the field.</p>
     */
    public static ReferencePoint2d relativeToSelectedTagPoint(TagSelectionSource selection,
                                                              Map<Integer, TagPointOffset> offsetsByTag) {
        return new SelectedTagPointRef(selection, new MapPointLookup(selection, offsetsByTag));
    }

    /**
     * Creates a frame relative to the tag currently selected by {@code selection}, using one
     * common offset / heading for every candidate tag.
     *
     * <p>Like {@link #relativeToSelectedTagPoint(TagSelectionSource, double, double)},
     * localization fallback currently requires every candidate ID exposed by {@code selection} to
     * exist in the fixed layout.</p>
     */
    public static ReferenceFrame2d relativeToSelectedTagFrame(TagSelectionSource selection,
                                                              double forwardInches,
                                                              double leftInches,
                                                              double headingRad) {
        return new SelectedTagFrameRef(selection, new FixedFrameLookup(forwardInches, leftInches, headingRad));
    }

    /**
     * Creates a frame relative to the tag currently selected by {@code selection}, with a
     * potentially different frame for each candidate tag.
     */
    public static ReferenceFrame2d relativeToSelectedTagFrame(TagSelectionSource selection,
                                                              Map<Integer, TagFrameOffset> offsetsByTag) {
        return new SelectedTagFrameRef(selection, new MapFrameLookup(selection, offsetsByTag));
    }

    /**
     * Returns the base frame when {@code ref} is a frame-point reference, else {@code null}.
     */
    public static ReferenceFrame2d framePointBaseFrame(ReferencePoint2d ref) {
        return (ref instanceof FramePointRef) ? ((FramePointRef) ref).frame : null;
    }

    /**
     * Returns the frame-local offset when {@code ref} is a frame-point reference, else {@code null}.
     */
    public static Pose2d framePointOffset(ReferencePoint2d ref) {
        if (!(ref instanceof FramePointRef)) {
            return null;
        }
        FramePointRef fp = (FramePointRef) ref;
        return new Pose2d(fp.forwardInches, fp.leftInches, 0.0);
    }

    public static boolean isFieldPoint(ReferencePoint2d ref) {
        return ref instanceof FieldPointRef;
    }

    public static boolean isFieldFrame(ReferenceFrame2d ref) {
        return ref instanceof FieldFrameRef;
    }

    public static boolean isDirectTagPoint(ReferencePoint2d ref) {
        return ref instanceof TagPointRef;
    }

    public static boolean isDirectTagFrame(ReferenceFrame2d ref) {
        return ref instanceof TagFrameRef;
    }

    public static boolean isSelectedTagPoint(ReferencePoint2d ref) {
        return ref instanceof SelectedTagPointRef;
    }

    public static boolean isSelectedTagFrame(ReferenceFrame2d ref) {
        return ref instanceof SelectedTagFrameRef;
    }

    public static boolean isFramePoint(ReferencePoint2d ref) {
        return ref instanceof FramePointRef;
    }

    public static Set<Integer> candidateTagIds(ReferencePoint2d ref) {
        if (ref instanceof TagPointRef) {
            return Collections.singleton(((TagPointRef) ref).tagId);
        }
        if (ref instanceof SelectedTagPointRef) {
            return ((SelectedTagPointRef) ref).selection.candidateIds();
        }
        if (ref instanceof FramePointRef) {
            return candidateTagIds(((FramePointRef) ref).frame);
        }
        return Collections.emptySet();
    }

    public static Set<Integer> candidateTagIds(ReferenceFrame2d ref) {
        if (ref instanceof TagFrameRef) {
            return Collections.singleton(((TagFrameRef) ref).tagId);
        }
        if (ref instanceof SelectedTagFrameRef) {
            return ((SelectedTagFrameRef) ref).selection.candidateIds();
        }
        return Collections.emptySet();
    }

    public static boolean allCandidateTagsAreFixed(ReferencePoint2d ref, TagLayout layout) {
        return allCandidateTagsAreFixed(candidateTagIds(ref), layout);
    }

    public static boolean allCandidateTagsAreFixed(ReferenceFrame2d ref, TagLayout layout) {
        return allCandidateTagsAreFixed(candidateTagIds(ref), layout);
    }

    public static Set<Integer> missingCandidateTagIds(ReferencePoint2d ref, TagLayout layout) {
        return missingCandidateTagIds(candidateTagIds(ref), layout);
    }

    public static Set<Integer> missingCandidateTagIds(ReferenceFrame2d ref, TagLayout layout) {
        return missingCandidateTagIds(candidateTagIds(ref), layout);
    }

    private static boolean allCandidateTagsAreFixed(Set<Integer> ids, TagLayout layout) {
        if (ids.isEmpty() || layout == null) {
            return false;
        }
        for (Integer id : ids) {
            if (!layout.has(id)) {
                return false;
            }
        }
        return true;
    }

    private static Set<Integer> missingCandidateTagIds(Set<Integer> ids, TagLayout layout) {
        if (ids.isEmpty()) {
            return Collections.emptySet();
        }

        LinkedHashSet<Integer> missing = new LinkedHashSet<Integer>();
        for (Integer id : ids) {
            if (id == null) {
                continue;
            }
            if (layout == null || !layout.has(id)) {
                missing.add(id);
            }
        }
        return missing.isEmpty()
                ? Collections.<Integer>emptySet()
                : Collections.unmodifiableSet(missing);
    }

    public static Pose2d tryResolveFieldPoint(ReferencePoint2d ref, TagLayout layout, LoopClock clock) {
        if (ref instanceof FieldPointRef) {
            FieldPointRef fp = (FieldPointRef) ref;
            return new Pose2d(fp.xInches, fp.yInches, 0.0);
        }
        if (ref instanceof TagPointRef) {
            TagPointRef tp = (TagPointRef) ref;
            if (layout == null) {
                return null;
            }
            Pose3d fieldToTag = layout.getFieldToTagPose(tp.tagId);
            return fieldToTag != null
                    ? fieldToTag.toPose2d().then(new Pose2d(tp.forwardInches, tp.leftInches, 0.0))
                    : null;
        }
        if (ref instanceof SelectedTagPointRef) {
            SelectedTagPointRef sp = (SelectedTagPointRef) ref;
            Integer tagId = currentSelectedTagIdOrNull(sp.selection, clock);
            if (tagId == null || layout == null) {
                return null;
            }
            Pose3d fieldToTag = layout.getFieldToTagPose(tagId);
            Pose2d tagToPoint = sp.lookup.tagToPoint(tagId);
            return (fieldToTag != null && tagToPoint != null)
                    ? fieldToTag.toPose2d().then(tagToPoint)
                    : null;
        }
        if (ref instanceof FramePointRef) {
            FramePointRef fp = (FramePointRef) ref;
            Pose2d fieldToFrame = tryResolveFieldFrame(fp.frame, layout, clock);
            return fieldToFrame != null ? fieldToFrame.then(new Pose2d(fp.forwardInches, fp.leftInches, 0.0)) : null;
        }
        return null;
    }

    public static Pose2d tryResolveFieldFrame(ReferenceFrame2d ref, TagLayout layout, LoopClock clock) {
        if (ref instanceof FieldFrameRef) {
            FieldFrameRef ff = (FieldFrameRef) ref;
            return new Pose2d(ff.xInches, ff.yInches, ff.headingRad);
        }
        if (ref instanceof TagFrameRef) {
            TagFrameRef tf = (TagFrameRef) ref;
            if (layout == null) {
                return null;
            }
            Pose3d fieldToTag = layout.getFieldToTagPose(tf.tagId);
            return fieldToTag != null
                    ? fieldToTag.toPose2d().then(new Pose2d(tf.forwardInches, tf.leftInches, tf.headingRad))
                    : null;
        }
        if (ref instanceof SelectedTagFrameRef) {
            SelectedTagFrameRef sf = (SelectedTagFrameRef) ref;
            Integer tagId = currentSelectedTagIdOrNull(sf.selection, clock);
            if (tagId == null || layout == null) {
                return null;
            }
            Pose3d fieldToTag = layout.getFieldToTagPose(tagId);
            Pose2d tagToFrame = sf.lookup.tagToFrame(tagId);
            return (fieldToTag != null && tagToFrame != null)
                    ? fieldToTag.toPose2d().then(tagToFrame)
                    : null;
        }
        return null;
    }

    static Pose2d directTagToPoint(ReferencePoint2d ref, int tagId) {
        if (ref instanceof TagPointRef) {
            TagPointRef tp = (TagPointRef) ref;
            return tp.tagId == tagId ? new Pose2d(tp.forwardInches, tp.leftInches, 0.0) : null;
        }
        if (ref instanceof SelectedTagPointRef) {
            return ((SelectedTagPointRef) ref).lookup.tagToPoint(tagId);
        }
        if (ref instanceof FramePointRef) {
            FramePointRef fp = (FramePointRef) ref;
            Pose2d tagToFrame = directTagToFrame(fp.frame, tagId);
            return tagToFrame != null ? tagToFrame.then(new Pose2d(fp.forwardInches, fp.leftInches, 0.0)) : null;
        }
        return null;
    }

    static Pose2d directTagToFrame(ReferenceFrame2d ref, int tagId) {
        if (ref instanceof TagFrameRef) {
            TagFrameRef tf = (TagFrameRef) ref;
            return tf.tagId == tagId ? new Pose2d(tf.forwardInches, tf.leftInches, tf.headingRad) : null;
        }
        if (ref instanceof SelectedTagFrameRef) {
            return ((SelectedTagFrameRef) ref).lookup.tagToFrame(tagId);
        }
        return null;
    }

    static Integer singleTagIdOrNull(ReferencePoint2d ref) {
        if (ref instanceof TagPointRef) {
            return ((TagPointRef) ref).tagId;
        }
        if (ref instanceof FramePointRef) {
            return singleTagIdOrNull(((FramePointRef) ref).frame);
        }
        return null;
    }

    static Integer singleTagIdOrNull(ReferenceFrame2d ref) {
        if (ref instanceof TagFrameRef) {
            return ((TagFrameRef) ref).tagId;
        }
        return null;
    }

    static Integer currentSelectedTagIdOrNull(TagSelectionSource selection, LoopClock clock) {
        if (selection == null || clock == null) {
            return null;
        }
        TagSelectionResult sel = selection.get(clock);
        return (sel != null && sel.hasSelection) ? sel.selectedTagId : null;
    }

    private static int normalizeTagId(int tagId) {
        if (tagId < 0) {
            throw new IllegalArgumentException("tagId must be non-negative");
        }
        return tagId;
    }

    private static TagSelectionSource requireSelection(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        if (selection.candidateIds() == null || selection.candidateIds().isEmpty()) {
            throw new IllegalArgumentException("selection must expose at least one candidate tag ID");
        }
        return selection;
    }

    interface PointLookup {
        Pose2d tagToPoint(int tagId);
    }

    interface FrameLookup {
        Pose2d tagToFrame(int tagId);
    }

    private static final class FixedPointLookup implements PointLookup {
        private final double forwardInches;
        private final double leftInches;

        FixedPointLookup(double forwardInches, double leftInches) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Pose2d tagToPoint(int tagId) {
            return new Pose2d(forwardInches, leftInches, 0.0);
        }
    }

    private static final class FixedFrameLookup implements FrameLookup {
        private final double forwardInches;
        private final double leftInches;
        private final double headingRad;

        FixedFrameLookup(double forwardInches, double leftInches, double headingRad) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
            this.headingRad = headingRad;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Pose2d tagToFrame(int tagId) {
            return new Pose2d(forwardInches, leftInches, headingRad);
        }
    }

    private static final class MapPointLookup implements PointLookup {
        private final Map<Integer, TagPointOffset> offsetsByTag;

        MapPointLookup(TagSelectionSource selection, Map<Integer, TagPointOffset> offsetsByTag) {
            requireSelection(selection);
            Objects.requireNonNull(offsetsByTag, "offsetsByTag");
            LinkedHashMap<Integer, TagPointOffset> copy = new LinkedHashMap<Integer, TagPointOffset>();
            for (Map.Entry<Integer, TagPointOffset> e : offsetsByTag.entrySet()) {
                Integer id = e.getKey();
                TagPointOffset offset = e.getValue();
                if (id == null || offset == null) {
                    throw new IllegalArgumentException("offsetsByTag must not contain null keys or values");
                }
                copy.put(normalizeTagId(id), offset);
            }
            for (Integer id : selection.candidateIds()) {
                if (!copy.containsKey(id)) {
                    throw new IllegalArgumentException("offsetsByTag is missing candidate tag " + id);
                }
            }
            this.offsetsByTag = Collections.unmodifiableMap(copy);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Pose2d tagToPoint(int tagId) {
            TagPointOffset offset = offsetsByTag.get(tagId);
            return offset != null ? new Pose2d(offset.forwardInches, offset.leftInches, 0.0) : null;
        }
    }

    private static final class MapFrameLookup implements FrameLookup {
        private final Map<Integer, TagFrameOffset> offsetsByTag;

        MapFrameLookup(TagSelectionSource selection, Map<Integer, TagFrameOffset> offsetsByTag) {
            requireSelection(selection);
            Objects.requireNonNull(offsetsByTag, "offsetsByTag");
            LinkedHashMap<Integer, TagFrameOffset> copy = new LinkedHashMap<Integer, TagFrameOffset>();
            for (Map.Entry<Integer, TagFrameOffset> e : offsetsByTag.entrySet()) {
                Integer id = e.getKey();
                TagFrameOffset offset = e.getValue();
                if (id == null || offset == null) {
                    throw new IllegalArgumentException("offsetsByTag must not contain null keys or values");
                }
                copy.put(normalizeTagId(id), offset);
            }
            for (Integer id : selection.candidateIds()) {
                if (!copy.containsKey(id)) {
                    throw new IllegalArgumentException("offsetsByTag is missing candidate tag " + id);
                }
            }
            this.offsetsByTag = Collections.unmodifiableMap(copy);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Pose2d tagToFrame(int tagId) {
            TagFrameOffset offset = offsetsByTag.get(tagId);
            return offset != null ? new Pose2d(offset.forwardInches, offset.leftInches, offset.headingRad) : null;
        }
    }

    static final class FieldPointRef implements ReferencePoint2d {
        final double xInches;
        final double yInches;

        FieldPointRef(double xInches, double yInches) {
            this.xInches = xInches;
            this.yInches = yInches;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "FieldPointRef{xInches=" + xInches + ", yInches=" + yInches + "}";
        }
    }

    static final class FieldFrameRef implements ReferenceFrame2d {
        final double xInches;
        final double yInches;
        final double headingRad;

        FieldFrameRef(double xInches, double yInches, double headingRad) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRad = headingRad;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "FieldFrameRef{xInches=" + xInches + ", yInches=" + yInches + ", headingRad=" + headingRad + "}";
        }
    }

    static final class TagPointRef implements ReferencePoint2d {
        final int tagId;
        final double forwardInches;
        final double leftInches;

        TagPointRef(int tagId, double forwardInches, double leftInches) {
            this.tagId = tagId;
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "TagPointRef{tagId=" + tagId + ", forwardInches=" + forwardInches + ", leftInches=" + leftInches + "}";
        }
    }

    static final class TagFrameRef implements ReferenceFrame2d {
        final int tagId;
        final double forwardInches;
        final double leftInches;
        final double headingRad;

        TagFrameRef(int tagId, double forwardInches, double leftInches, double headingRad) {
            this.tagId = tagId;
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
            this.headingRad = headingRad;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "TagFrameRef{tagId=" + tagId + ", forwardInches=" + forwardInches
                    + ", leftInches=" + leftInches + ", headingRad=" + headingRad + "}";
        }
    }

    static final class SelectedTagPointRef implements ReferencePoint2d {
        final TagSelectionSource selection;
        final PointLookup lookup;

        SelectedTagPointRef(TagSelectionSource selection, PointLookup lookup) {
            this.selection = requireSelection(selection);
            this.lookup = Objects.requireNonNull(lookup, "lookup");
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "SelectedTagPointRef{candidateIds=" + selection.candidateIds() + "}";
        }
    }

    static final class SelectedTagFrameRef implements ReferenceFrame2d {
        final TagSelectionSource selection;
        final FrameLookup lookup;

        SelectedTagFrameRef(TagSelectionSource selection, FrameLookup lookup) {
            this.selection = requireSelection(selection);
            this.lookup = Objects.requireNonNull(lookup, "lookup");
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "SelectedTagFrameRef{candidateIds=" + selection.candidateIds() + "}";
        }
    }

    static final class FramePointRef implements ReferencePoint2d {
        final ReferenceFrame2d frame;
        final double forwardInches;
        final double leftInches;

        FramePointRef(ReferenceFrame2d frame, double forwardInches, double leftInches) {
            this.frame = frame;
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "FramePointRef{frame=" + frame + ", forwardInches=" + forwardInches
                    + ", leftInches=" + leftInches + "}";
        }
    }
}
