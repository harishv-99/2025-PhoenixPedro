package edu.ftcphoenix.fw.field;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;

/**
 * Lightweight {@link TagLayout} view that exposes only a caller-selected subset of IDs from a
 * backing layout.
 *
 * <p>This is the framework building block for role-specific tag layouts. A robot may want:</p>
 * <ul>
 *   <li>a <b>full fixed-tag layout</b> for global localization, but</li>
 *   <li>a <b>smaller fixed-tag subset</b> for one behavior such as scoring alignment.</li>
 * </ul>
 *
 * <p>Wrapping the layout keeps the filtering policy in the framework rather than forcing each
 * robot to carry ad-hoc {@code if (id == ...)} logic at every call site.</p>
 */
public final class SubsetTagLayout implements TagLayout {

    private final TagLayout base;
    private final Set<Integer> ids;
    private final String description;

    /**
     * Creates a subset view over {@code base}.
     *
     * @param base backing layout containing the original field metadata
     * @param ids  IDs to expose through this view; every ID must already exist in {@code base}
     */
    public SubsetTagLayout(TagLayout base, Set<Integer> ids) {
        this(base, ids, "subset");
    }

    /**
     * Creates a subset view over {@code base} with a human-readable description.
     *
     * @param base        backing layout containing the original field metadata
     * @param ids         IDs to expose through this view; every ID must already exist in {@code base}
     * @param description short label used in debug dumps and {@link #toString()}
     */
    public SubsetTagLayout(TagLayout base, Set<Integer> ids, String description) {
        this.base = Objects.requireNonNull(base, "base");
        Objects.requireNonNull(ids, "ids");

        LinkedHashSet<Integer> keep = new LinkedHashSet<Integer>();
        for (Integer id : ids) {
            if (id == null) {
                throw new IllegalArgumentException("ids must not contain null");
            }
            if (!base.has(id)) {
                throw new IllegalArgumentException(
                        "SubsetTagLayout requested id=" + id + " but base layout does not contain it"
                );
            }
            keep.add(id);
        }

        this.ids = Collections.unmodifiableSet(keep);
        this.description = (description != null && !description.trim().isEmpty())
                ? description.trim()
                : "subset";
    }

    /**
     * Returns the backing layout from which this subset was created.
     */
    public TagLayout base() {
        return base;
    }

    /**
     * Returns the human-readable description attached to this subset.
     */
    public String description() {
        return description;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Pose3d getFieldToTagPose(int id) {
        return ids.contains(id) ? base.getFieldToTagPose(id) : null;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Set<Integer> ids() {
        return ids;
    }

    /**
     * Emits a compact debug dump of the subset and its backing layout size.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagLayout" : prefix;
        dbg.addData(p + ".type", getClass().getSimpleName())
                .addData(p + ".description", description)
                .addData(p + ".ids", ids.toString())
                .addData(p + ".count", ids.size())
                .addData(p + ".baseType", base.getClass().getSimpleName())
                .addData(p + ".baseCount", base.ids().size());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "SubsetTagLayout{" +
                "description='" + description + '\'' +
                ", ids=" + ids +
                ", base=" + base.getClass().getSimpleName() +
                '}';
    }
}
