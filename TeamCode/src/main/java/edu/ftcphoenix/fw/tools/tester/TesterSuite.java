package edu.ftcphoenix.fw.tools.tester;

import java.util.function.Supplier;

import edu.ftcphoenix.fw.tools.tester.ui.SelectionMenu;

/**
 * A menu-based "tester runner" that lets you select and run any registered {@link TeleOpTester}.
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>Dpad Up/Down</b>: select (menu only)</li>
 *   <li><b>A</b>: enter selected tester (menu only)</li>
 *   <li><b>BACK</b>: go back (active tester may handle; otherwise returns to menu)</li>
 * </ul>
 *
 * <p>
 * This suite supports selection menus during INIT: you can enter a tester during INIT, allow that
 * tester to run its own {@link TeleOpTester#initLoop(double)} (for example, a camera picker), then
 * press Start and continue into RUN without leaving the tester.
 * </p>
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

    /**
     * Override the menu title.
     */
    public TesterSuite setTitle(String title) {
        menu.setTitle(title);
        return this;
    }

    /**
     * Override the menu's global help line.
     */
    public TesterSuite setHelp(String help) {
        menu.setHelp(help);
        return this;
    }

    /**
     * Set whether menu selection wraps around at the ends.
     */
    public TesterSuite setWrap(boolean wrap) {
        menu.setWrap(wrap);
        return this;
    }

    /**
     * Limit how many items are rendered at one time.
     */
    public TesterSuite setMaxVisibleItems(int maxVisibleItems) {
        menu.setMaxVisibleItems(maxVisibleItems);
        return this;
    }

    /**
     * Set the initial selection index for the menu.
     */
    public TesterSuite setSelectedIndex(int index) {
        menu.setSelectedIndex(index);
        return this;
    }

    /**
     * Register a tester in the menu (no help text).
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
     * @param help    optional help line shown under the item (nullable)
     * @param factory factory that creates a new tester instance when selected
     * @return this suite for chaining
     */
    public TesterSuite add(String name, String help, Supplier<TeleOpTester> factory) {
        menu.addItem(name, help, factory);
        return this;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "Tester Suite";
    }

    /**
     * Allow this suite to be nested inside another {@link TesterSuite}.
     *
     * <p>When nested, the parent suite offers BACK to the active tester via
     * {@link TeleOpTester#onBackPressed()}. The default implementation returns
     * {@code false}, which would cause the parent to immediately exit this suite.
     * We implement BACK semantics here so nested suites behave like a normal "sub-menu":
     * <ul>
     *   <li>If this suite is showing its own menu, BACK is not consumed (parent can exit).</li>
     *   <li>If this suite is running a tester, BACK returns to this suite's menu.</li>
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

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInit() {
        // Menu navigation and selection are ONLY active while inMenu.
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
        bindings.onPress(gamepads.p1().back(), () -> {
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

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInitLoop(double dtSec) {
        if (inMenu) {
            renderMenu();
            return;
        }

        if (active != null) {
            dispatchActiveInitLoop(dtSec);
            return;
        }

        // Fail-safe
        inMenu = true;
        renderMenu();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onStart() {
        opModeStarted = true;
        if (active != null) {
            active.start();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onLoop(double dtSec) {
        if (inMenu) {
            renderMenu();
            return;
        }

        if (active != null) {
            dispatchActiveLoop(dtSec);
            return;
        }

        // Fail-safe
        inMenu = true;
        renderMenu();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onStop() {
        stopActive();
    }

    /**
     * Dispatch INIT-loop callbacks to the active tester without double-updating global button state.
     *
     * <p>Both {@link TesterSuite} and most testers extend {@link BaseTeleOpTester}, which performs
     * {@code gamepads.update()} / {@code bindings.update()} in their {@code initLoop()} and {@code loop()}.
     * When nested (suite calling into a tester), calling {@code active.initLoop()} would cause a second
     * button update in the same OpMode cycle, wiping out edge events (e.g. Dpad/A presses).
     */
    private void dispatchActiveInitLoop(double dtSec) {
        if (active instanceof BaseTeleOpTester) {
            BaseTeleOpTester bt = (BaseTeleOpTester) active;
            // Global buttons were already updated by the suite's BaseTeleOpTester.
            // We still need to update the active tester's Bindings so its actions can fire.
            bt.bindings.update(clock);
            bt.onInitLoop(dtSec);
        } else {
            active.initLoop(dtSec);
        }
    }

    /**
     * Dispatch RUN-loop callbacks to the active tester without double-updating global button state.
     */
    private void dispatchActiveLoop(double dtSec) {
        if (active instanceof BaseTeleOpTester) {
            BaseTeleOpTester bt = (BaseTeleOpTester) active;
            bt.bindings.update(clock);
            bt.onLoop(dtSec);
        } else {
            active.loop(dtSec);
        }
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
        menu.render(ctx.telemetry);
        ctx.telemetry.addLine("");
        ctx.telemetry.addLine(opModeStarted
                ? "RUNNING: Enter tester with A."
                : "INIT: Enter a tester with A (it may have its own picker/selection menu).");
        ctx.telemetry.update();
    }
}
