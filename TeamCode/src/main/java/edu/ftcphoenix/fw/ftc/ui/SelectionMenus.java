package edu.ftcphoenix.fw.ftc.ui;

/**
 * Convenience factories for common {@link SelectionMenu} shapes.
 *
 * <p>The helpers in this class keep repetitive UI construction code out of robot OpModes without
 * making the framework know what the choices mean. For example, an autonomous selector can use an
 * enum-backed menu for alliance or start position while the robot-owned code remains responsible for
 * converting the selected enum values into an autonomous spec.</p>
 */
public final class SelectionMenus {

    private SelectionMenus() {
        // Utility class.
    }

    /**
     * Customizes how enum values appear in an enum-backed menu.
     *
     * @param <E> enum type being displayed
     */
    public interface EnumDisplay<E extends Enum<E>> {
        /**
         * Stable item id. Defaults should usually use {@link Enum#name()}.
         */
        String id(E value);

        /**
         * Human-facing row label.
         */
        String label(E value);

        /**
         * Optional help text for the row.
         */
        String help(E value);

        /**
         * Optional compact tag such as {@code DEFAULT}, {@code OK}, or {@code WARN}.
         */
        String tag(E value);

        /**
         * Whether the row can be chosen.
         */
        boolean enabled(E value);

        /**
         * Optional explanation shown when a disabled row is highlighted.
         */
        String disabledReason(E value);
    }

    /**
     * Minimal enum display that uses enum names as ids and friendly title-cased labels.
     */
    public static final class DefaultEnumDisplay<E extends Enum<E>> implements EnumDisplay<E> {
        /**
         * {@inheritDoc}
         */
        @Override
        public String id(E value) {
            return value.name();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String label(E value) {
            return friendlyEnumName(value.name());
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String help(E value) {
            return null;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String tag(E value) {
            return null;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean enabled(E value) {
            return true;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String disabledReason(E value) {
            return null;
        }
    }

    /**
     * Create a menu containing all values of an enum using default labels.
     *
     * @param title    menu title
     * @param enumType enum class
     * @param <E>      enum type
     * @return new menu populated with one row per enum value
     */
    public static <E extends Enum<E>> SelectionMenu<E> forEnum(String title, Class<E> enumType) {
        return forEnum(title, enumType, new DefaultEnumDisplay<E>());
    }

    /**
     * Create a menu containing all values of an enum using a custom display adapter.
     *
     * @param title    menu title
     * @param enumType enum class
     * @param display  display adapter; when null, the default display is used
     * @param <E>      enum type
     * @return new menu populated with one row per enum value
     */
    public static <E extends Enum<E>> SelectionMenu<E> forEnum(String title,
                                                               Class<E> enumType,
                                                               EnumDisplay<E> display) {
        if (enumType == null) {
            throw new IllegalArgumentException("enumType is required");
        }

        EnumDisplay<E> out = display == null ? new DefaultEnumDisplay<E>() : display;
        SelectionMenu<E> menu = new SelectionMenu<E>().setTitle(title);
        E[] values = enumType.getEnumConstants();
        if (values == null) {
            return menu;
        }

        for (int i = 0; i < values.length; i++) {
            E value = values[i];
            menu.addItem(
                    cleanOrDefault(out.id(value), value.name()),
                    cleanOrDefault(out.label(value), friendlyEnumName(value.name())),
                    cleanOrNull(out.help(value)),
                    cleanOrNull(out.tag(value)),
                    out.enabled(value),
                    cleanOrNull(out.disabledReason(value)),
                    value
            );
        }
        return menu;
    }

    /**
     * Convert an enum constant name such as {@code PARTNER_AWARE_CYCLE} into {@code Partner Aware Cycle}.
     */
    public static String friendlyEnumName(String name) {
        if (name == null || name.trim().isEmpty()) return "Value";
        String[] parts = name.trim().toLowerCase().split("_");
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < parts.length; i++) {
            if (parts[i].isEmpty()) continue;
            if (b.length() > 0) b.append(' ');
            b.append(Character.toUpperCase(parts[i].charAt(0)));
            if (parts[i].length() > 1) {
                b.append(parts[i].substring(1));
            }
        }
        return b.length() == 0 ? "Value" : b.toString();
    }

    private static String cleanOrNull(String value) {
        if (value == null) return null;
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private static String cleanOrDefault(String value, String fallback) {
        String clean = cleanOrNull(value);
        return clean == null ? fallback : clean;
    }
}
