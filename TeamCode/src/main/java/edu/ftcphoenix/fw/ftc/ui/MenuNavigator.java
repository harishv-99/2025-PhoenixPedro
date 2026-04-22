package edu.ftcphoenix.fw.ftc.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Stack-based navigator for FTC telemetry UI screens.
 *
 * <p>{@link SelectionMenu} is intentionally a single screen. This class owns the higher-level job:
 * pushing submenus, popping back to previous screens, returning home, and rendering breadcrumbs/level
 * metadata for the active screen. That split keeps basic menus simple while still supporting richer
 * INIT-phase flows such as:</p>
 * <ul>
 *   <li>tester category menus</li>
 *   <li>hardware picker flows</li>
 *   <li>autonomous setup wizards</li>
 *   <li>calibration walkthroughs with detail screens</li>
 * </ul>
 *
 * <p>The navigator should be bound once. It dispatches inputs to whatever screen is current, which
 * avoids accumulating stale bindings as screens are pushed and popped.</p>
 */
public final class MenuNavigator {

    private final List<MenuScreen> stack = new ArrayList<MenuScreen>();
    private String controlsHint = null;
    private Runnable onRootBack = null;

    /** Create an empty navigator. Call {@link #push(MenuScreen)} before rendering. */
    public MenuNavigator() {
    }

    /** Create a navigator with an initial root screen. */
    public MenuNavigator(MenuScreen root) {
        push(root);
    }

    /** Set the controls hint attached to automatically-created render contexts. */
    public MenuNavigator setControlsHint(String controlsHint) {
        this.controlsHint = cleanOrNull(controlsHint);
        return this;
    }

    /** Set an optional action to run when back is pressed on the root screen and the root does not consume it. */
    public MenuNavigator setOnRootBack(Runnable onRootBack) {
        this.onRootBack = onRootBack;
        return this;
    }

    /** Push a screen on top of the current screen and make it active. */
    public void push(MenuScreen screen) {
        if (screen == null) return;
        MenuScreen cur = currentOrNull();
        if (cur != null) cur.onExit();
        stack.add(screen);
        screen.onEnter();
    }

    /**
     * Pop the current screen and return to the previous screen.
     *
     * @return true if a screen was popped
     */
    public boolean pop() {
        if (stack.size() <= 1) return false;
        MenuScreen removed = stack.remove(stack.size() - 1);
        removed.onExit();
        MenuScreen cur = currentOrNull();
        if (cur != null) cur.onEnter();
        return true;
    }

    /**
     * Return to the root screen.
     *
     * @return true if at least one screen was removed
     */
    public boolean home() {
        if (stack.size() <= 1) return false;
        while (stack.size() > 1) {
            MenuScreen removed = stack.remove(stack.size() - 1);
            removed.onExit();
        }
        MenuScreen cur = currentOrNull();
        if (cur != null) cur.onEnter();
        return true;
    }

    /** Remove all screens. */
    public void clear() {
        while (!stack.isEmpty()) {
            MenuScreen removed = stack.remove(stack.size() - 1);
            removed.onExit();
        }
    }

    /** Return the active screen, or null if the navigator is empty. */
    public MenuScreen currentOrNull() {
        if (stack.isEmpty()) return null;
        return stack.get(stack.size() - 1);
    }

    /**
     * Return the active screen.
     *
     * @throws IllegalStateException if the navigator has no screen
     */
    public MenuScreen current() {
        MenuScreen cur = currentOrNull();
        if (cur == null) {
            throw new IllegalStateException("MenuNavigator has no current screen. Push a root screen first.");
        }
        return cur;
    }

    /** Number of screens on the stack. */
    public int size() {
        return stack.size();
    }

    /** Current nesting depth. Root is depth 0; empty navigators report -1. */
    public int depth() {
        return stack.isEmpty() ? -1 : stack.size() - 1;
    }

    /** Human-facing breadcrumb made from the screen titles in the stack. */
    public String breadcrumb() {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < stack.size(); i++) {
            if (i > 0) b.append(" > ");
            String title = stack.get(i).title();
            b.append(title == null ? "Screen" : title);
        }
        return b.toString();
    }

    /** Register navigator input dispatch with standard controls. */
    public void bind(Bindings bindings, UiControls controls) {
        bind(bindings, controls, new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return true;
            }
        });
    }

    /** Register navigator input dispatch with an enable predicate. */
    public void bind(Bindings bindings, UiControls controls, BooleanSupplier enabled) {
        if (bindings == null || controls == null) return;
        final BooleanSupplier gate = enabled == null ? new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return true;
            }
        } : enabled;

        bindOne(bindings, controls.up, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null) cur.up();
            }
        });

        bindOne(bindings, controls.down, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null) cur.down();
            }
        });

        bindOne(bindings, controls.left, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null) cur.left();
            }
        });

        bindOne(bindings, controls.right, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null) cur.right();
            }
        });

        bindOne(bindings, controls.select, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null) cur.select();
            }
        });

        bindOne(bindings, controls.secondary, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null) cur.secondary();
            }
        });

        bindOne(bindings, controls.back, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null && cur.onBack()) return;
                if (!pop() && onRootBack != null) {
                    onRootBack.run();
                }
            }
        });

        bindOne(bindings, controls.home, gate, new Runnable() {
            @Override
            public void run() {
                MenuScreen cur = currentOrNull();
                if (cur != null && cur.onHome()) return;
                home();
            }
        });

        if (controlsHint == null) {
            controlsHint = controls.hint();
        }
    }

    /**
     * Render the current screen using automatically generated breadcrumb and level metadata.
     *
     * <p>This does not call {@code telemetry.update()}.</p>
     */
    public void render(Telemetry telemetry) {
        MenuScreen cur = currentOrNull();
        if (cur == null) {
            if (telemetry != null) {
                telemetry.addLine("=== Menu ===");
                telemetry.addLine("(no screen)");
            }
            return;
        }

        MenuRenderContext ctx = MenuRenderContext.root()
                .setBreadcrumb(breadcrumb())
                .setLevel(depth())
                .setControlsHint(controlsHint);
        cur.render(telemetry, ctx);
    }

    private void bindOne(Bindings bindings, BooleanSource source, final BooleanSupplier enabled, final Runnable action) {
        if (source == null || action == null) return;
        bindings.onRise(source, new Runnable() {
            @Override
            public void run() {
                if (!enabled.getAsBoolean()) return;
                action.run();
            }
        });
    }

    private static String cleanOrNull(String value) {
        if (value == null) return null;
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }
}
