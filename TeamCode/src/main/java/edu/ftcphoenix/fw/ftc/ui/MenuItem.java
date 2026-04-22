package edu.ftcphoenix.fw.ftc.ui;

/**
 * Immutable row model for {@link SelectionMenu}.
 *
 * <p>A menu item has two names on purpose:</p>
 * <ul>
 *   <li>{@link #label} is the human-facing text shown on telemetry.</li>
 *   <li>{@link #id} is the stable identity used to preserve selection when a menu is rebuilt.</li>
 * </ul>
 *
 * <p>For small static menus it is fine to let the label be the id. For menus that are rebuilt from
 * changing data (hardware names, autonomous strategies filtered by match setup, calibration steps),
 * pass a deliberate id so the highlighted choice survives refreshes when that choice still exists.</p>
 *
 * @param <T> value attached to the menu row
 */
public final class MenuItem<T> {

    /** Stable item identity used for selection preservation. */
    public final String id;

    /** Human-facing row label shown in telemetry. */
    public final String label;

    /** Optional one-line explanation shown when this row is selected. */
    public final String help;

    /** Optional compact status label such as {@code OK}, {@code TODO}, {@code WARN}, or {@code DEFAULT}. */
    public final String tag;

    /** Whether this item may be selected/confirmed. Disabled items can still be highlighted for context. */
    public final boolean enabled;

    /** Optional explanation shown when {@link #enabled} is false. */
    public final String disabledReason;

    /** Caller-owned value associated with this row. */
    public final T value;

    /**
     * Create a menu item.
     *
     * @param id stable item identity; if blank, the label is used as the fallback id
     * @param label display label; if blank, a safe generic label is used
     * @param help optional selected-item help text
     * @param tag optional compact status/default tag
     * @param enabled whether the item can be chosen
     * @param disabledReason optional disabled explanation
     * @param value caller-owned value attached to the row
     */
    public MenuItem(String id,
                    String label,
                    String help,
                    String tag,
                    boolean enabled,
                    String disabledReason,
                    T value) {
        String cleanLabel = cleanOrDefault(label, value == null ? "Item" : value.toString());
        this.label = cleanLabel;
        this.id = cleanOrDefault(id, cleanLabel);
        this.help = cleanOrNull(help);
        this.tag = cleanOrNull(tag);
        this.enabled = enabled;
        this.disabledReason = cleanOrNull(disabledReason);
        this.value = value;
    }

    /**
     * Create an enabled item whose label also acts as the stable id.
     */
    public static <T> MenuItem<T> of(String label, T value) {
        return new MenuItem<T>(label, label, null, null, true, null, value);
    }

    /**
     * Create an enabled item with an explicit id, label, and help text.
     */
    public static <T> MenuItem<T> of(String id, String label, String help, T value) {
        return new MenuItem<T>(id, label, help, null, true, null, value);
    }

    /**
     * Create an enabled item with an explicit id, label, help text, and compact tag.
     */
    public static <T> MenuItem<T> tagged(String id, String label, String help, String tag, T value) {
        return new MenuItem<T>(id, label, help, tag, true, null, value);
    }

    /**
     * Return a copy with different help text.
     */
    public MenuItem<T> withHelp(String help) {
        return new MenuItem<T>(id, label, help, tag, enabled, disabledReason, value);
    }

    /**
     * Return a copy with a different compact tag.
     */
    public MenuItem<T> withTag(String tag) {
        return new MenuItem<T>(id, label, help, tag, enabled, disabledReason, value);
    }

    /**
     * Return a disabled copy with a reason shown when the row is highlighted.
     */
    public MenuItem<T> disabled(String reason) {
        return new MenuItem<T>(id, label, help, tag, false, reason, value);
    }

    /**
     * Return an enabled copy.
     */
    public MenuItem<T> enabled() {
        return new MenuItem<T>(id, label, help, tag, true, null, value);
    }

    /**
     * Return a compact debug representation useful when logging menu contents.
     */
    @Override
    public String toString() {
        return "MenuItem{" +
                "id='" + id + '\'' +
                ", label='" + label + '\'' +
                (tag == null ? "" : ", tag='" + tag + '\'') +
                ", enabled=" + enabled +
                '}';
    }

    private static String cleanOrNull(String value) {
        if (value == null) return null;
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private static String cleanOrDefault(String value, String fallback) {
        String clean = cleanOrNull(value);
        if (clean != null) return clean;
        String cleanFallback = cleanOrNull(fallback);
        return cleanFallback == null ? "Item" : cleanFallback;
    }
}
