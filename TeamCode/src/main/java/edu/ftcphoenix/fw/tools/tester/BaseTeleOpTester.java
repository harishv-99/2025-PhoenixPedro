package edu.ftcphoenix.fw.tools.tester;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Convenience base class for Phoenix testers.
 *
 * <p>Provides:</p>
 * <ul>
 *   <li>{@link TesterContext} via {@link #ctx}</li>
 *   <li>{@link Gamepads} wrapper via {@link #gamepads}</li>
 *   <li>{@link Bindings} via {@link #bindings}</li>
 *   <li>A shared per-loop {@link LoopClock} via {@link #clock} (from {@link TesterContext})</li>
 * </ul>
 *
 * <h2>One loop, one heartbeat</h2>
 * <p>Testers must use the <b>shared</b> {@link LoopClock} from {@link TesterContext} so that per-cycle
 * systems (button edge tracking, bindings) can be made idempotent by {@link LoopClock#cycle()} across
 * nested callers (e.g., a menu calling into a tester).</p>
 *
 * <p>This base class therefore does <b>not</b> create or update its own clock. The runner (OpMode or
 * tester suite) advances {@link TesterContext#clock} once per OpMode cycle.</p>
 *
 * <h2>Update order</h2>
 * <p>Each {@code initLoop()} / {@code loop()} call executes in this order:</p>
 * <ol>
 *   <li>{@code gamepads.update(clock)} – advances global button edge state (idempotent by cycle)</li>
 *   <li>{@code bindings.update(clock)} – fires binding actions (idempotent per Bindings instance)</li>
 *   <li>{@code onInitLoop(dtSec)} or {@code onLoop(dtSec)} – tester-specific logic</li>
 * </ol>
 *
 * <p>Because {@code Button.updateAllRegistered(clock)} is idempotent by cycle, it is safe for both a
 * suite and the active tester to call {@code gamepads.update(clock)} in the same cycle: only the first
 * call actually advances button state; subsequent calls are no-ops.</p>
 */
public abstract class BaseTeleOpTester implements TeleOpTester {

    protected TesterContext ctx;
    protected Gamepads gamepads;
    protected final Bindings bindings = new Bindings();

    /**
     * Shared per-loop heartbeat provided by the runner via {@link TesterContext}.
     *
     * <p>This is assigned in {@link #init(TesterContext)} and should be treated as read-only
     * by testers (do not call {@code clock.update(...)} here).</p>
     */
    protected LoopClock clock;

    /**
     * {@inheritDoc}
     */
    @Override
    public final void init(TesterContext ctx) {
        this.ctx = ctx;
        this.clock = ctx.clock;

        // Each tester gets a Gamepads wrapper; button edge tracking is global and idempotent by cycle.
        this.gamepads = Gamepads.create(ctx.gamepad1, ctx.gamepad2);

        onInit();
    }

    /** {@inheritDoc} */
    @Override
    public final void initLoop(double dtSec) {
        // Per-cycle systems first.
        gamepads.update(clock);
        bindings.update(clock);

        onInitLoop(dtSec);
    }

    /** {@inheritDoc} */
    @Override
    public final void start() {
        onStart();
    }

    /** {@inheritDoc} */
    @Override
    public final void loop(double dtSec) {
        // Per-cycle systems first.
        gamepads.update(clock);
        bindings.update(clock);

        onLoop(dtSec);
    }

    /** {@inheritDoc} */
    @Override
    public final void stop() {
        onStop();
    }

    /**
     * Override to set up bindings and internal state.
     *
     * <p>Called once when the tester is entered.</p>
     */
    protected void onInit() {
        // Default: no-op.
    }

    /**
     * Override to implement INIT-phase behavior (selection menus, camera selection, etc).
     *
     * <p>Called every INIT loop while this tester is active.</p>
     */
    protected void onInitLoop(double dtSec) {
        // Default: no-op.
    }

    /**
     * Override to implement RUN-phase tester behavior.
     *
     * <p>Called every RUN loop while this tester is active.</p>
     */
    protected abstract void onLoop(double dtSec);

    /**
     * Optional hook called once when the OpMode transitions from INIT to RUN.
     */
    protected void onStart() {
        // Default: no-op.
    }

    /**
     * Optional hook called once when the tester is stopped (or when returning to menu).
     */
    protected void onStop() {
        // Default: no-op.
    }

    // ---------------------------------------------------------------------------------------------
    // Telemetry helpers
    // ---------------------------------------------------------------------------------------------

    /**
     * Convenience: clear telemetry and draw a consistent header.
     */
    protected final void telemHeader(String title) {
        ctx.telemetry.clearAll();
        ctx.telemetry.addLine("=== " + title + " ===");
    }

    /**
     * Convenience: small control hint line.
     */
    protected final void telemHint(String hint) {
        ctx.telemetry.addLine(hint);
    }

    /**
     * Convenience: always remember to update.
     */
    protected final void telemUpdate() {
        ctx.telemetry.update();
    }
}
