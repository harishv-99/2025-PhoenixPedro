package edu.ftcphoenix.fw.field;

import java.util.Objects;
import java.util.Set;

/**
 * Factory/helper methods for working with {@link TagLayout} instances.
 *
 * <p>The main use today is creating role-specific views without copying tag metadata or pushing
 * tag-filtering logic down into robot code.</p>
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
        return base.ids().equals(ids) ? base : subset(base, ids, description);
    }
}
