package edu.ftcphoenix.fw.ftc.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Single-screen scroll-and-select menu for FTC telemetry UIs.
 *
 * <p>This class intentionally owns only one screen: row storage, selection index, scrolling, and
 * rendering. It does not own hierarchical navigation; use {@link MenuNavigator} when a UI needs
 * submenus, breadcrumbs, back/home behavior, or a wizard-like flow.</p>
 *
 * <h2>General-use features</h2>
 * <ul>
 *   <li>Stable item ids so selection can be preserved when rows are rebuilt.</li>
 *   <li>Optional status/default tags such as {@code OK}, {@code TODO}, {@code WARN}, or {@code DEFAULT}.</li>
 *   <li>Disabled items that remain visible and explain why they cannot be selected.</li>
 *   <li>Snapshot access for callers such as hardware pickers that need to search existing rows.</li>
 *   <li>Optional direct bindings for simple one-screen menus.</li>
 * </ul>
 *
 * @param <T> value stored for each item, such as a tester factory, hardware name, enum, or auto strategy id
 */
public final class SelectionMenu<T> implements MenuScreen {

    private String title = "Menu";
    private String help = null;
    private String emptyMessage = "(no items)";

    private final List<MenuItem<T>> items = new ArrayList<MenuItem<T>>();
    private int selected = 0;

    private boolean wrap = true;
    private int maxVisibleItems = 8;

    private Consumer<MenuItem<T>> onSelect = null;
    private Runnable onSecondary = null;

    /**
     * Set the title shown at the top of the menu.
     *
     * @param title title string; blank values fall back to {@code Menu}
     * @return this menu for chaining
     */
    public SelectionMenu<T> setTitle(String title) {
        this.title = cleanOrDefault(title, "Menu");
        return this;
    }

    /** {@inheritDoc} */
    @Override
    public String title() {
        return title;
    }

    /**
     * Set a global help line shown near the top of the menu.
     *
     * <p>Per-item help is still shown near the bottom for the currently selected item.</p>
     */
    public SelectionMenu<T> setHelp(String help) {
        this.help = cleanOrNull(help);
        return this;
    }

    /** Set the message shown when the menu has no rows. */
    public SelectionMenu<T> setEmptyMessage(String emptyMessage) {
        this.emptyMessage = cleanOrDefault(emptyMessage, "(no items)");
        return this;
    }

    /** Set whether selection wraps around at the ends. Default: true. */
    public SelectionMenu<T> setWrap(boolean wrap) {
        this.wrap = wrap;
        return this;
    }

    /**
     * Set how many rows the menu renders at one time.
     *
     * <p>Values &lt;= 0 disable the limit and render all rows. Limiting rows helps keep FTC Driver
     * Station telemetry controls and status lines visible on long menus.</p>
     */
    public SelectionMenu<T> setMaxVisibleItems(int maxVisibleItems) {
        this.maxVisibleItems = maxVisibleItems;
        return this;
    }

    /** Set the action invoked by {@link #select()} and navigator-managed select events. */
    public SelectionMenu<T> setOnSelect(Consumer<MenuItem<T>> onSelect) {
        this.onSelect = onSelect;
        return this;
    }

    /** Set the optional action invoked by {@link #secondary()} and navigator-managed secondary events. */
    public SelectionMenu<T> setOnSecondary(Runnable onSecondary) {
        this.onSecondary = onSecondary;
        return this;
    }

    /** Remove all rows and reset selection to index 0. */
    public SelectionMenu<T> clearItems() {
        items.clear();
        selected = 0;
        return this;
    }

    /** Replace rows and reset selection to index 0. */
    public SelectionMenu<T> setItems(List<MenuItem<T>> newItems) {
        items.clear();
        if (newItems != null) {
            for (int i = 0; i < newItems.size(); i++) {
                MenuItem<T> item = newItems.get(i);
                if (item != null) items.add(item);
            }
        }
        selected = 0;
        clampSelected();
        return this;
    }

    /**
     * Replace rows while trying to keep the previously selected item highlighted by stable id.
     *
     * <p>If the old selected id no longer exists, the old index is clamped into the new list.</p>
     */
    public SelectionMenu<T> setItemsPreserveSelectionById(List<MenuItem<T>> newItems) {
        String oldId = selectedIdOrNull();
        int oldIndex = selected;

        setItems(newItems);

        if (oldId != null) {
            int idx = indexOfId(oldId);
            if (idx >= 0) {
                selected = idx;
                return this;
            }
        }

        selected = oldIndex;
        clampSelected();
        return this;
    }

    /** Add a fully specified item. */
    public SelectionMenu<T> addItem(MenuItem<T> item) {
        if (item != null) {
            items.add(item);
            clampSelected();
        }
        return this;
    }

    /** Add an enabled item. The label is also used as the stable id. */
    public SelectionMenu<T> addItem(String label, T value) {
        return addItem(MenuItem.of(label, value));
    }

    /** Add an enabled item. The label is also used as the stable id. */
    public SelectionMenu<T> addItem(String label, String help, T value) {
        return addItem(new MenuItem<T>(label, label, help, null, true, null, value));
    }

    /** Add an item with explicit id, label, help, tag, and enabled state. */
    public SelectionMenu<T> addItem(String id,
                                    String label,
                                    String help,
                                    String tag,
                                    boolean enabled,
                                    String disabledReason,
                                    T value) {
        return addItem(new MenuItem<T>(id, label, help, tag, enabled, disabledReason, value));
    }

    /** Return an immutable snapshot of the current rows. */
    public List<MenuItem<T>> itemsSnapshot() {
        return Collections.unmodifiableList(new ArrayList<MenuItem<T>>(items));
    }

    /** Return the number of rows in the menu. */
    public int size() {
        return items.size();
    }

    /** Return whether the menu has no rows. */
    public boolean isEmpty() {
        return items.isEmpty();
    }

    /** Return the currently selected index, or 0 when the menu is empty. */
    public int selectedIndex() {
        return selected;
    }

    /** Set the selected index, clamped into the valid row range. */
    public void setSelectedIndex(int index) {
        selected = index;
        clampSelected();
    }

    /** Return the stable id of the selected row, or null if the menu is empty. */
    public String selectedIdOrNull() {
        MenuItem<T> item = selectedItemOrNull();
        return item == null ? null : item.id;
    }

    /**
     * Highlight the row with the given stable id when it exists.
     *
     * @return true if the id was found and selected
     */
    public boolean setSelectedId(String id) {
        int idx = indexOfId(id);
        if (idx < 0) return false;
        selected = idx;
        return true;
    }

    /** Return the index of a stable id, or -1 when absent. */
    public int indexOfId(String id) {
        if (id == null) return -1;
        for (int i = 0; i < items.size(); i++) {
            if (id.equals(items.get(i).id)) return i;
        }
        return -1;
    }

    /** Return the currently selected row, or null if the menu is empty. */
    public MenuItem<T> selectedItemOrNull() {
        if (items.isEmpty()) return null;
        return items.get(selected);
    }

    /** Return the value of the currently selected row, or null if the menu is empty. */
    public T selectedValueOrNull() {
        MenuItem<T> item = selectedItemOrNull();
        return item == null ? null : item.value;
    }

    /** Move selection up by one row. */
    @Override
    public void up() {
        if (items.isEmpty()) return;
        if (selected > 0) {
            selected--;
        } else if (wrap) {
            selected = items.size() - 1;
        }
    }

    /** Move selection down by one row. */
    @Override
    public void down() {
        if (items.isEmpty()) return;
        if (selected < items.size() - 1) {
            selected++;
        } else if (wrap) {
            selected = 0;
        }
    }

    /** Confirm the current row using the menu's configured select action. */
    @Override
    public void select() {
        selectCurrent(onSelect);
    }

    /** Invoke the configured secondary action, if any. */
    @Override
    public void secondary() {
        if (onSecondary != null) {
            onSecondary.run();
        }
    }

    /**
     * Confirm the current row using the provided action.
     *
     * @return true when an enabled item was selected and the action was invoked
     */
    public boolean selectCurrent(Consumer<MenuItem<T>> action) {
        MenuItem<T> item = selectedItemOrNull();
        if (item == null || !item.enabled || action == null) {
            return false;
        }
        action.accept(item);
        return true;
    }

    /**
     * Register simple one-screen menu bindings that are always enabled.
     *
     * <p>For nested menus, prefer {@link MenuNavigator#bind(Bindings, UiControls)} so one central
     * dispatcher owns navigation for the active screen.</p>
     */
    public void bind(Bindings bindings,
                     BooleanSource upButton,
                     BooleanSource downButton,
                     BooleanSource selectButton,
                     Consumer<MenuItem<T>> onSelect) {
        bind(bindings, upButton, downButton, selectButton, new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return true;
            }
        }, onSelect);
    }

    /**
     * Register simple one-screen menu bindings with an enable predicate.
     *
     * @param enabled when false, bound inputs are ignored
     */
    public void bind(Bindings bindings,
                     BooleanSource upButton,
                     BooleanSource downButton,
                     BooleanSource selectButton,
                     BooleanSupplier enabled,
                     Consumer<MenuItem<T>> onSelect) {
        if (bindings == null) return;

        final BooleanSupplier gate = enabled == null ? new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return true;
            }
        } : enabled;

        if (upButton != null) {
            bindings.onRise(upButton, new Runnable() {
                @Override
                public void run() {
                    if (!gate.getAsBoolean()) return;
                    up();
                }
            });
        }

        if (downButton != null) {
            bindings.onRise(downButton, new Runnable() {
                @Override
                public void run() {
                    if (!gate.getAsBoolean()) return;
                    down();
                }
            });
        }

        if (selectButton != null && onSelect != null) {
            bindings.onRise(selectButton, new Runnable() {
                @Override
                public void run() {
                    if (!gate.getAsBoolean()) return;
                    selectCurrent(onSelect);
                }
            });
        }
    }

    /**
     * Render the menu to telemetry using an empty render context.
     *
     * <p>This does not call {@code telemetry.update()} so callers can compose screens.</p>
     */
    public void render(Telemetry telemetry) {
        render(telemetry, MenuRenderContext.root());
    }

    /**
     * Render the menu to telemetry.
     *
     * <p>This does not call {@code telemetry.update()} so callers can compose screens.</p>
     */
    @Override
    public void render(Telemetry telemetry, MenuRenderContext context) {
        if (telemetry == null) return;
        MenuRenderContext ctx = (context == null) ? MenuRenderContext.root() : context;

        telemetry.addLine("=== " + title + " ===");

        if (ctx.breadcrumb() != null) {
            telemetry.addLine("Path: " + ctx.breadcrumb());
        }

        String navLine = navLine(ctx);
        if (navLine != null) {
            telemetry.addLine(navLine);
        }

        if (ctx.statusTag() != null || ctx.statusMessage() != null) {
            telemetry.addLine(statusLine(ctx.statusTag(), ctx.statusMessage()));
        }

        if (help != null) {
            telemetry.addLine(help);
        }

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
            telemetry.addLine(emptyMessage);
            renderFooter(telemetry, ctx);
            return;
        }

        for (int i = start; i < end; i++) {
            telemetry.addLine(rowLine(i, items.get(i)));
        }

        MenuItem<T> sel = items.get(selected);
        telemetry.addLine("");
        telemetry.addLine("Selected: " + decoratedLabel(sel));
        if (!sel.enabled) {
            telemetry.addLine("Disabled: " + (sel.disabledReason == null ? "This item cannot be chosen right now." : sel.disabledReason));
        }
        if (sel.help != null) {
            telemetry.addLine("Info: " + sel.help);
        }

        renderFooter(telemetry, ctx);
    }

    private String rowLine(int index, MenuItem<T> item) {
        String prefix = (index == selected) ? ">> " : "   ";
        return prefix + decoratedLabel(item);
    }

    private String decoratedLabel(MenuItem<T> item) {
        String out = item.label;
        if (item.tag != null) {
            out += " [" + item.tag + "]";
        }
        if (!item.enabled) {
            out += " [DISABLED]";
        }
        return out;
    }

    private void renderFooter(Telemetry telemetry, MenuRenderContext ctx) {
        if (ctx.controlsHint() != null) {
            telemetry.addLine("");
            telemetry.addLine("Controls: " + ctx.controlsHint());
        }
        if (ctx.footer() != null) {
            telemetry.addLine(ctx.footer());
        }
    }

    private String navLine(MenuRenderContext ctx) {
        boolean hasStep = ctx.hasStep();
        boolean hasLevel = ctx.level() >= 0;
        if (!hasStep && !hasLevel) return null;

        String out = "";
        if (hasStep) {
            out += "Step " + ctx.stepIndex() + " of " + ctx.stepCount();
        }
        if (hasLevel) {
            if (!out.isEmpty()) out += " | ";
            out += "Level: " + ctx.level();
        }
        return out;
    }

    private String statusLine(String tag, String message) {
        if (tag != null && message != null) {
            return "Status: [" + tag + "] " + message;
        }
        if (tag != null) {
            return "Status: [" + tag + "]";
        }
        return "Status: " + message;
    }

    private void clampSelected() {
        if (items.isEmpty()) {
            selected = 0;
            return;
        }
        selected = MathUtil.clamp(selected, 0, items.size() - 1);
    }

    private static String cleanOrNull(String value) {
        if (value == null) return null;
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private static String cleanOrDefault(String value, String fallback) {
        String clean = cleanOrNull(value);
        if (clean != null) return clean;
        return fallback;
    }
}
