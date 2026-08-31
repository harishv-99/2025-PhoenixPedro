package edu.ftcsushi.fw.field;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import edu.ftcsushi.fw.core.geometry.Pose3d;

/**
 * Factory/helper methods for working with {@link TagLayout} instances.
 *
 * <p>Use {@link #snapshot(TagLayout)} when a long-lived owner retains a layout. Subset helpers
 * intentionally create borrowed views for short-lived authoring and selection; a retaining owner
 * snapshots the completed view at its own construction boundary.</p>
 */
public final class TagLayouts {

    private TagLayouts() {
        // utility holder
    }

    /**
     * Returns a view containing only {@code ids} from {@code base}.
     */
    public static TagLayout subset(TagLayout base, Set<Integer> ids) {
        return new SubsetTagLayout(base, ids);
    }

    /**
     * Returns a described view containing only {@code ids} from {@code base}.
     */
    public static TagLayout subset(TagLayout base, Set<Integer> ids, String description) {
        return new SubsetTagLayout(base, ids, description);
    }

    /**
     * Returns {@code base} unchanged when it already contains exactly {@code ids}; otherwise
     * returns a subset view.
     *
     * <p>This is a small convenience for callers that sometimes request the full layout and do not
     * want to manufacture a wrapper unnecessarily.</p>
     */
    public static TagLayout subsetOrSame(TagLayout base, Set<Integer> ids, String description) {
        Objects.requireNonNull(base, "base");
        Objects.requireNonNull(ids, "ids");
        SubsetTagLayout subset = new SubsetTagLayout(base, ids, description);
        return base.ids().equals(subset.ids()) ? base : subset;
    }

    /**
     * Returns an immutable, validated snapshot of {@code layout}.
     *
     * <p>The source ID set is enumerated once and each declared pose is read once. Every ID must
     * be non-null and non-negative, and every declared pose must be non-null with six finite
     * components. Validation completes before the snapshot is published, so a failure cannot
     * expose a partially copied layout. Passing a snapshot created by this method returns that
     * same object unchanged.</p>
     *
     * @param layout authored or borrowed layout to retain
     * @return immutable validated layout snapshot
     * @throws NullPointerException if {@code layout} is null
     * @throws IllegalArgumentException if the ID set, an ID, or a declared pose is invalid
     */
    public static TagLayout snapshot(TagLayout layout) {
        TagLayout requiredLayout = Objects.requireNonNull(layout, "layout");
        if (requiredLayout instanceof SnapshotTagLayout) {
            return requiredLayout;
        }

        Set<Integer> sourceIds = requiredLayout.ids();
        if (sourceIds == null) {
            throw new IllegalArgumentException("TagLayouts.snapshot layout.ids() must not return null");
        }

        LinkedHashMap<Integer, Pose3d> copiedById = new LinkedHashMap<Integer, Pose3d>();
        for (Integer id : sourceIds) {
            if (id == null) {
                throw new IllegalArgumentException(
                        "TagLayouts.snapshot layout.ids() must not contain null"
                );
            }
            requireNonNegativeTagId(id, "TagLayouts.snapshot layout id");
            Pose3d pose = requiredLayout.getFieldToTagPose(id);
            copiedById.put(
                    id,
                    requireFinitePose(
                            pose,
                            "TagLayouts.snapshot layout pose for id=" + id
                    )
            );
        }
        return new SnapshotTagLayout(copiedById);
    }

    static int requireNonNegativeTagId(int id, String context) {
        if (id < 0) {
            throw new IllegalArgumentException(context + " must be non-negative; received " + id);
        }
        return id;
    }

    static Pose3d requireFinitePose(Pose3d pose, String context) {
        if (pose == null) {
            throw new IllegalArgumentException(context + " must not be null");
        }
        requireFinite(pose.xInches, context + ".xInches");
        requireFinite(pose.yInches, context + ".yInches");
        requireFinite(pose.zInches, context + ".zInches");
        requireFinite(pose.yawRad, context + ".yawRad");
        requireFinite(pose.pitchRad, context + ".pitchRad");
        requireFinite(pose.rollRad, context + ".rollRad");
        return pose;
    }

    private static void requireFinite(double value, String context) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(context + " must be finite; received " + value);
        }
    }

    /** Private immutable representation used to make {@link #snapshot(TagLayout)} idempotent. */
    private static final class SnapshotTagLayout implements TagLayout {
        private final Map<Integer, Pose3d> byId;
        private final Set<Integer> ids;

        SnapshotTagLayout(Map<Integer, Pose3d> validatedById) {
            this.byId = Collections.unmodifiableMap(
                    new LinkedHashMap<Integer, Pose3d>(validatedById)
            );
            this.ids = this.byId.keySet();
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            return byId.get(id);
        }

        @Override
        public Set<Integer> ids() {
            return ids;
        }

        @Override
        public String toString() {
            return "TagLayoutSnapshot{ids=" + ids + '}';
        }
    }
}
