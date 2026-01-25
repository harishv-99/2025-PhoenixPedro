package edu.ftcphoenix.fw.tools.tester.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.input.Button;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Simple scroll-and-select menu helper for TeleOp testers.
 *
 * <p>Opinionated by design so callers don't duplicate logic:
 * <ul>
 *   <li>Maintains a list of items and a selected index.</li>
 *   <li>Wrap-around navigation supported (default on).</li>
 *   <li>Renders the list + selected item + optional help text.</li>
 *   <li>Binds navigation to {@link Bindings} using Phoenix {@link Button} edge detection.</li>
 * </ul>
 *
 * <p><b>Important:</b> Use the {@link #bind(Bindings, Button, Button, Button, BooleanSupplier, Consumer)}
 * overload to gate input handling (e.g., only respond while a menu is active).</p>
 *
 * @param <T> value stored for each item (e.g., a factory, a device name, an enum)
 */
public final class SelectionMenu<T> {

    /**
     * A single menu row: label + optional help + attached value.
     */
    public static final class Item<T> {
        public final String label;
        public final String help;
        public final T value;

        /**
         * Create a menu item.
         *
         * @param label display label shown in the menu
         * @param help  optional help text shown for the selected item (nullable)
         * @param value value associated with this item (for example a factory or device name)
         */
        public Item(String label, String help, T value) {
            this.label = label;
            this.help = help;
            this.value = value;
        }
    }

    private String title = "Menu";
    private String help = null;

    private final List<Item<T>> items = new ArrayList<>();
    private int selected = 0;

    private boolean wrap = true;

    /**
     * Maximum number of menu items to render at once.
     *
     * <p>FTC Driver Station telemetry has limited vertical space. If we render every item,
     * long menus push important instructions (usually shown below the menu) off screen.
     *
     * <p>When {@link #size()} exceeds this value, the menu renders a scroll window centered
     * around the current selection.</p>
     */
    private int maxVisibleItems = 8;

    /**
     * Set the title shown at the top of the menu.
     *
     * @param title title string (nullable; if null, the title line will show {@code "null"})
     * @return this menu for chaining
     */
    public SelectionMenu<T> setTitle(String title) {
        this.title = title;
        return this;
    }

    /**
     * Sets a global help line shown near the top of the menu.
     * (Per-item help is shown near the bottom for the currently selected item.)
     */
    public SelectionMenu<T> setHelp(String help) {
        this.help = help;
        return this;
    }

    /**
     * If true, selection wraps around at ends. Default: true.
     */
    public SelectionMenu<T> setWrap(boolean wrap) {
        this.wrap = wrap;
        return this;
    }

    /**
     * Set how many items the menu renders at one time.
     *
     * <p>Values &lt;= 0 disable the limit (all items will be rendered).
     */
    public SelectionMenu<T> setMaxVisibleItems(int maxVisibleItems) {
        this.maxVisibleItems = maxVisibleItems;
        return this;
    }

    /**
     * Removes all items and resets selection to 0.
     */
    public SelectionMenu<T> clearItems() {
        items.clear();
        selected = 0;
        return this;
    }

    /**
     * Adds an item. {@code help} may be null.
     */
    public SelectionMenu<T> addItem(String label, String help, T value) {
        items.add(new Item<>(label, help, value));
        clampSelected();
        return this;
    }

    /**
     * Convenience overload (no per-item help).
     */
    public SelectionMenu<T> addItem(String label, T value) {
        return addItem(label, null, value);
    }

    /**
     * Return the number of items in the menu.
     *
     * @return item count
     */
    public int size() {
        return items.size();
    }

    /**
     * Return whether the menu has no items.
     *
     * @return {@code true} if empty
     */
    public boolean isEmpty() {
        return items.isEmpty();
    }

    /**
     * Return the currently selected index.
     *
     * @return selected index (0-based); if the menu is empty this will be {@code 0}
     */
    public int selectedIndex() {
        return selected;
    }

    /**
     * Set the selected index.
     *
     * <p>The value is clamped into {@code [0, size-1]} (or to 0 if the menu is empty).</p>
     *
     * @param index desired selected index
     */
    public void setSelectedIndex(int index) {
        selected = index;
        clampSelected();
    }

    /**
     * Return the currently selected item, or {@code null} if the menu is empty.
     *
     * @return selected item or {@code null}
     */
    public Item<T> selectedItemOrNull() {
        if (items.isEmpty()) return null;
        return items.get(selected);
    }

    /**
     * Return the value of the currently selected item, or {@code null} if the menu is empty.
     *
     * @return selected value or {@code null}
     */
    public T selectedValueOrNull() {
        Item<T> item = selectedItemOrNull();
        return item == null ? null : item.value;
    }

    /**
     * Move selection up by one item.
     *
     * <p>If {@link #setWrap(boolean)} is enabled, moving up from index 0 wraps to the end.</p>
     */
    public void up() {
        if (items.isEmpty()) return;
        if (selected > 0) {
            selected--;
        } else if (wrap) {
            selected = items.size() - 1;
        }
    }

    /**
     * Move selection down by one item.
     *
     * <p>If {@link #setWrap(boolean)} is enabled, moving down from the end wraps to 0.</p>
     */
    public void down() {
        if (items.isEmpty()) return;
        if (selected < items.size() - 1) {
            selected++;
        } else if (wrap) {
            selected = 0;
        }
    }

    /**
     * Register navigation bindings (always enabled).
     *
     * <p>If you need this to only respond sometimes (e.g., only while a menu is showing),
     * use the overload that takes an {@code enabled} predicate.</p>
     */
    public void bind(Bindings bindings,
                     Button upButton,
                     Button downButton,
                     Button selectButton,
                     Consumer<Item<T>> onSelect) {
        bind(bindings, upButton, downButton, selectButton, () -> true, onSelect);
    }

    /**
     * Register navigation bindings with an enable predicate.
     *
     * @param enabled if false, the menu ignores all bound inputs
     */
    public void bind(Bindings bindings,
                     Button upButton,
                     Button downButton,
                     Button selectButton,
                     BooleanSupplier enabled,
                     Consumer<Item<T>> onSelect) {

        bindings.onPress(upButton, () -> {
            if (!enabled.getAsBoolean()) return;
            up();
        });

        bindings.onPress(downButton, () -> {
            if (!enabled.getAsBoolean()) return;
            down();
        });

        if (selectButton != null && onSelect != null) {
            bindings.onPress(selectButton, () -> {
                if (!enabled.getAsBoolean()) return;
                Item<T> item = selectedItemOrNull();
                if (item != null) {
                    onSelect.accept(item);
                }
            });
        }
    }

    /**
     * Render the menu to telemetry.
     *
     * <p>This does not call {@code telemetry.update()} so the caller can compose screens.</p>
     */
    public void render(Telemetry telemetry) {
        telemetry.addLine("=== " + title + " ===");
        if (help != null && !help.isEmpty()) telemetry.addLine(help);
        // Keep a stable screen height: for long menus we render a scroll window so
        // callers can reserve the bottom of the telemetry page for instructions.

        final int count = items.size();
        final boolean limit = (maxVisibleItems > 0) && (count > maxVisibleItems);

        int start = 0;
        int end = count;
        if (limit) {
            int half = maxVisibleItems / 2;
            start = selected - half;
            start = MathUtil.clamp(start, 0, count - maxVisibleItems);
            end = start + maxVisibleItems;
            telemetry.addLine(String.format("Items %d-%d of %d", start + 1, end, count));
        } else {
            telemetry.addLine("");
        }

        if (items.isEmpty()) {
            telemetry.addLine("(no items)");
            return;
        }

        for (int i = start; i < end; i++) {
            String prefix = (i == selected) ? ">> " : "   ";
            telemetry.addLine(prefix + items.get(i).label);
        }

        Item<T> sel = items.get(selected);

        telemetry.addLine("");
        telemetry.addLine("Selected: " + sel.label);
        if (sel.help != null && !sel.help.isEmpty()) {
            telemetry.addLine("Info: " + sel.help);
        }
    }

    private void clampSelected() {
        if (items.isEmpty()) {
            selected = 0;
            return;
        }
        selected = MathUtil.clamp(selected, 0, items.size() - 1);
    }
}
