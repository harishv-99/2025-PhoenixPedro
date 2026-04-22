package edu.ftcphoenix.fw.tools.tester;

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
 */
public final class TesterSuite extends BaseTeleOpTester {

    private final SelectionMenu<Supplier<TeleOpTester>> menu =
            new SelectionMenu<Supplier<TeleOpTester>>()
                    .setTitle("Phoenix Tester Menu")
                    .setHelp("Dpad: select | A: enter | BACK: back/exit");

    private TeleOpTester active = null;
    private boolean inMenu = true;
    private boolean opModeStarted = false;

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
     * @param factory factory that creates a new tester instance when selected
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
     * @param factory factory that creates a new tester instance when selected
     * @return this suite for chaining
     */
    public TesterSuite add(String name, String help, Supplier<TeleOpTester> factory) {
        menu.addItem(name, help, factory);
        return this;
    }

    /**
     * Register a tester with a compact item tag such as {@code OK}, {@code TODO}, or {@code WARN}.
     *
     * <p>Tags are rendered by {@link SelectionMenu} instead of being baked into the label, which keeps
     * display names stable and makes status semantics easier to scan.</p>
     */
    public TesterSuite add(String name, String help, String tag, Supplier<TeleOpTester> factory) {
        menu.addItem(MenuItem.tagged(name, name, help, tag, factory));
        return this;
    }

    /**
     * Register a fully specified menu item.
     *
     * <p>This advanced hook is useful for robot-specific menu builders that want explicit stable ids,
     * tags, or disabled items while still reusing tester-suite lifecycle handling.</p>
     */
    public TesterSuite addItem(MenuItem<Supplier<TeleOpTester>> item) {
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
        if (inMenu) {
            return false;
        }

        if (active != null && active.onBackPressed()) {
            return true;
        }

        stopActive();
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
                () -> inMenu,
                item -> enter(item.value)
        );

        // BACK is treated as navigation:
        //   1) Offer it to the active tester (for multi-step tester UIs).
        //   2) If not handled, stop the tester and return to the menu.
        // We intentionally do not bind B here because many active tester screens use B as a local
        // action (zero, center, reset, abort sample, etc.).
        bindings.onRise(gamepads.p1().back(), () -> {
            if (inMenu) return;

            if (active != null && active.onBackPressed()) {
                return;
            }

            stopActive();
            inMenu = true;
        });

        inMenu = true;
        opModeStarted = false;
        active = null;
    }

    /** {@inheritDoc} */
    @Override
    protected void onInitLoop(double dtSec) {
        if (inMenu) {
            renderMenu();
            return;
        }

        if (active != null) {
            active.initLoop(dtSec);
            return;
        }

        // Fail-safe: if a tester disappeared unexpectedly, show the menu again.
        inMenu = true;
        renderMenu();
    }

    /** {@inheritDoc} */
    @Override
    protected void onStart() {
        opModeStarted = true;
        if (active != null) {
            active.start();
        }
    }

    /** {@inheritDoc} */
    @Override
    protected void onLoop(double dtSec) {
        if (inMenu) {
            renderMenu();
            return;
        }

        if (active != null) {
            active.loop(dtSec);
            return;
        }

        // Fail-safe: if a tester disappeared unexpectedly, show the menu again.
        inMenu = true;
        renderMenu();
    }

    /** {@inheritDoc} */
    @Override
    protected void onStop() {
        stopActive();
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void enter(Supplier<TeleOpTester> factory) {
        stopActive();

        active = factory.get();
        active.init(ctx);

        if (opModeStarted) {
            active.start();
        }

        inMenu = false;
    }

    private void stopActive() {
        if (active == null) return;
        active.stop();
        active = null;
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
        ctx.telemetry.update();
    }
}
