package edu.ftcphoenix.fw.tools.tester;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.ftc.ui.MenuItem;
import edu.ftcphoenix.fw.ftc.ui.MenuRenderContext;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;

/**
 * Menu-based runner that lets an OpMode select and run any registered {@link TeleOpTester}.
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>Dpad Up/Down</b>: select while the suite menu is visible</li>
 *   <li><b>A</b>: enter the highlighted tester while the suite menu is visible</li>
 *   <li><b>BACK</b>: offer back/cancel to the active tester; if unhandled, return to this menu</li>
 * </ul>
 *
 * <p>The suite supports selection menus during INIT: you can enter a tester during INIT, allow that
 * tester to run its own {@link TeleOpTester#initLoop(double)} (for example, a camera picker), then
 * press Start and continue into RUN without leaving the tester.</p>
 *
 * <p>This class uses the framework-level {@link SelectionMenu} for the list screen, while keeping
 * tester lifecycle ownership here. The separation is intentional: generic UI owns rendering and
 * selection; tester code owns what entering an item means.</p>
 *
 * <p>Every returned child is retained before initialization and is stopped at most once after a
 * runtime failure. Confirmed cleanup returns to this menu and permits a fresh child. Failed cleanup
 * blocks replacement and BACK navigation because the previous hardware ownership is uncertain.</p>
 */
public final class TesterSuite extends BaseTeleOpTester {

    private final SelectionMenu<Supplier<TeleOpTester>> menu =
            new SelectionMenu<Supplier<TeleOpTester>>()
                    .setTitle("Phoenix Tester Menu")
                    .setHelp("Dpad: select | A: enter | BACK: back/exit");

    private TesterChildSession childSession = new TesterChildSession();
    private boolean inMenu = true;
    private boolean opModeStarted = false;
    private String childError = null;
    private String activeTesterName = null;
    private RuntimeException childFailure = null;
    private long lastBackCycle = Long.MIN_VALUE;
    private boolean lastBackConsumed = false;

    // -----------------------------------------------------------------------------------------
    // Menu configuration
    // -----------------------------------------------------------------------------------------

    /** Override the menu title. */
    public TesterSuite setTitle(String title) {
        menu.setTitle(title);
        return this;
    }

    /** Override the menu's global help line. */
    public TesterSuite setHelp(String help) {
        menu.setHelp(help);
        return this;
    }

    /** Set whether menu selection wraps around at the ends. */
    public TesterSuite setWrap(boolean wrap) {
        menu.setWrap(wrap);
        return this;
    }

    /** Limit how many items are rendered at one time. */
    public TesterSuite setMaxVisibleItems(int maxVisibleItems) {
        menu.setMaxVisibleItems(maxVisibleItems);
        return this;
    }

    /** Set the initial selection index for the menu. */
    public TesterSuite setSelectedIndex(int index) {
        menu.setSelectedIndex(index);
        return this;
    }

    /**
     * Register a tester in the menu with no item help.
     *
     * @param name    display name in the menu
     * @param factory factory that returns a fresh inactive tester when selected; acquire owned
     *                hardware during that tester's {@code init}
     * @return this suite for chaining
     */
    public TesterSuite add(String name, Supplier<TeleOpTester> factory) {
        return add(name, null, factory);
    }

    /**
     * Register a tester in the menu with optional one-line help.
     *
     * @param name    display name in the menu
     * @param help    optional help line shown when the item is highlighted
     * @param factory factory that returns a fresh inactive tester when selected; acquire owned
     *                hardware during that tester's {@code init}
     * @return this suite for chaining
     */
    public TesterSuite add(String name, String help, Supplier<TeleOpTester> factory) {
        Objects.requireNonNull(factory, "factory for tester '" + displayName(name) + "'");
        menu.addItem(name, help, factory);
        return this;
    }

    /**
     * Register a tester with a compact item tag such as {@code OK}, {@code TODO}, or {@code WARN}.
     *
     * <p>Tags are rendered by {@link SelectionMenu} instead of being baked into the label, which keeps
     * display names stable and makes status semantics easier to scan. The factory follows the same
     * fresh-inactive-tester contract as the simpler {@code add(...)} overloads.</p>
     */
    public TesterSuite add(String name, String help, String tag, Supplier<TeleOpTester> factory) {
        Objects.requireNonNull(factory, "factory for tester '" + displayName(name) + "'");
        menu.addItem(MenuItem.tagged(name, name, help, tag, factory));
        return this;
    }

    /**
     * Register a fully specified menu item.
     *
     * <p>This advanced hook is useful for robot-specific menu builders that want explicit stable ids,
     * tags, or disabled items while still reusing tester-suite lifecycle handling. An enabled item
     * must contain a non-null factory that returns a fresh inactive tester; a disabled placeholder
     * may omit its value.</p>
     */
    public TesterSuite addItem(MenuItem<Supplier<TeleOpTester>> item) {
        Objects.requireNonNull(item, "item");
        if (item.enabled && item.value == null) {
            throw new IllegalArgumentException(
                    "Enabled tester menu item '" + item.label + "' must provide a factory.");
        }
        menu.addItem(item);
        return this;
    }

    /** {@inheritDoc} */
    @Override
    public String name() {
        return "Tester Suite";
    }

    /**
     * Allow this suite to be nested inside another {@link TesterSuite}.
     *
     * <p>When nested, the parent suite offers BACK to the active tester via
     * {@link TeleOpTester#onBackPressed()}. The default implementation returns {@code false}, which
     * would cause the parent to immediately exit this suite. We implement BACK semantics here so
     * nested suites behave like normal submenus:</p>
     * <ul>
     *   <li>If this suite is showing its own menu, BACK is not consumed and the parent can exit.</li>
     *   <li>If this suite is running a tester, BACK returns to this suite's menu unless the tester consumes it.</li>
     * </ul>
     */
    @Override
    public boolean onBackPressed() {
        long cycle = clock.cycle();
        if (lastBackCycle == cycle) {
            return lastBackConsumed;
        }

        // Publish a consumed result before invoking child code so reentrant BACK cannot recurse
        // into the same navigation transition. The final result is cached below.
        lastBackCycle = cycle;
        lastBackConsumed = true;
        lastBackConsumed = handleBackPressed();
        return lastBackConsumed;
    }

    private boolean handleBackPressed() {
        if (childSession.mustConsumeBackNavigation()) {
            return true;
        }
        if (inMenu) {
            return false;
        }

        TesterChildSession.BackResult back = childSession.backPressed();
        if (back.failure() != null) {
            recordChildFailure(activeTesterName(), "BACK", back.failure());
            activeTesterName = null;
            inMenu = true;
            return true;
        }
        if (back.handled()) {
            return true;
        }

        RuntimeException stopFailure = childSession.stopForReplacement();
        if (stopFailure != null) {
            recordChildFailure(activeTesterName(), "stop after BACK", stopFailure);
        }
        activeTesterName = null;
        inMenu = true;
        return true;
    }

    /** {@inheritDoc} */
    @Override
    protected void onInit() {
        // Menu navigation and selection are only active while the suite menu is visible.
        menu.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                () -> inMenu && childSession.canActivate(),
                item -> enter(item.label, item.value)
        );

        // BACK is treated as navigation:
        //   1) Offer it to the active tester (for multi-step tester UIs).
        //   2) If not handled, stop the tester and return to the menu.
        // We intentionally do not bind B here because many active tester screens use B as a local
        // action (zero, center, reset, abort sample, etc.).
        bindings.onRise(gamepads.p1().back(), () -> onBackPressed());

        inMenu = true;
        opModeStarted = false;
        childSession = new TesterChildSession();
        childError = null;
        activeTesterName = null;
        childFailure = null;
        lastBackCycle = Long.MIN_VALUE;
        lastBackConsumed = false;
    }

    /** {@inheritDoc} */
    @Override
    protected void onInitLoop(double dtSec) {
        if (inMenu) {
            renderMenu();
            return;
        }

        if (childSession.hasActive()) {
            RuntimeException failure = childSession.initLoop(dtSec);
            if (failure != null) {
                recordChildFailure(activeTesterName(), "initLoop", failure);
                activeTesterName = null;
                inMenu = true;
                renderMenu();
            }
            return;
        }

        // Fail-safe: if a tester disappeared unexpectedly, show the menu again.
        inMenu = true;
        renderMenu();
    }

    /** {@inheritDoc} */
    @Override
    protected void onStart() {
        // The FTC root resets the shared clock when RUN begins, so INIT cycle numbers may be
        // reused. Do not let an INIT-phase BACK result suppress a distinct RUN-phase press.
        lastBackCycle = Long.MIN_VALUE;
        lastBackConsumed = false;
        opModeStarted = true;
        if (childSession.hasActive()) {
            RuntimeException failure = childSession.start();
            if (failure != null) {
                recordChildFailure(activeTesterName(), "start", failure);
                activeTesterName = null;
                inMenu = true;
            } else if (!childSession.hasActive()) {
                activeTesterName = null;
                inMenu = true;
            }
        }
    }

    /** {@inheritDoc} */
    @Override
    protected void onLoop(double dtSec) {
        if (inMenu) {
            renderMenu();
            return;
        }

        if (childSession.hasActive()) {
            RuntimeException failure = childSession.loop(dtSec);
            if (failure != null) {
                recordChildFailure(activeTesterName(), "loop", failure);
                activeTesterName = null;
                inMenu = true;
                renderMenu();
            }
            return;
        }

        // Fail-safe: if a tester disappeared unexpectedly, show the menu again.
        inMenu = true;
        renderMenu();
    }

    /** {@inheritDoc} */
    @Override
    protected void onStop() {
        RuntimeException failure = childSession.stopTerminal();
        activeTesterName = null;
        if (failure != null) {
            throw failure;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void enter(String testerName, Supplier<TeleOpTester> factory) {
        if (!childSession.canActivate()) {
            return;
        }

        final TeleOpTester child;
        try {
            child = Objects.requireNonNull(
                    factory.get(),
                    "Tester factory for '" + testerName + "' returned null.");
        } catch (RuntimeException failure) {
            recordChildFailure("'" + testerName + "'", "factory", failure);
            inMenu = true;
            return;
        }

        activeTesterName = testerName;
        childSession.retain(child);
        RuntimeException failure = childSession.init(ctx);
        String phase = "init";
        if (failure == null && opModeStarted && childSession.hasActive()) {
            phase = "start";
            failure = childSession.start();
        }
        if (failure != null) {
            recordChildFailure("'" + testerName + "'", phase, failure);
            activeTesterName = null;
            inMenu = true;
            return;
        }
        if (!childSession.hasActive()) {
            activeTesterName = null;
            inMenu = true;
            return;
        }

        childError = null;
        childFailure = null;
        inMenu = false;
    }

    private void renderMenu() {
        ctx.telemetry.clearAll();
        menu.render(ctx.telemetry, MenuRenderContext.root()
                .setBreadcrumb(menu.title())
                .setLevel(0)
                .setControlsHint("Dpad: select | A: enter | BACK: back/exit"));
        ctx.telemetry.addLine("");
        ctx.telemetry.addLine(opModeStarted
                ? "RUNNING: Enter tester with A."
                : "INIT: Enter a tester with A (it may have its own picker/selection menu).");
        if (childError != null) {
            ctx.telemetry.addLine("");
            ctx.telemetry.addLine("Tester error:");
            ctx.telemetry.addLine(childError);
            if (childSession.cleanupBlocked()) {
                ctx.telemetry.addLine("Cleanup also failed. Retry disabled.");
                ctx.telemetry.addLine("Restart the OpMode and inspect the hardware.");
            }
        }
        ctx.telemetry.update();
    }

    private void recordChildFailure(String testerName, String phase, RuntimeException failure) {
        childFailure = failure;
        childError = testerName + " failed during " + phase + ": " + describe(failure);
        if (failure.getSuppressed().length > 0) {
            childError += " | cleanup: " + describe(failure.getSuppressed()[0]);
        }
    }

    private String activeTesterName() {
        return activeTesterName == null ? "active tester" : "'" + activeTesterName + "'";
    }

    private static String describe(Throwable failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + ((message == null || message.trim().isEmpty()) ? "" : ": " + message);
    }

    private static String displayName(String name) {
        return (name == null || name.trim().isEmpty()) ? "Item" : name.trim();
    }
}
