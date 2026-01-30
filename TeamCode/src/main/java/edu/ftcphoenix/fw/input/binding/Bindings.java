package edu.ftcphoenix.fw.input.binding;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Binding manager that maps {@link BooleanSource} inputs to higher-level behavior.
 *
 * <p>Phoenix uses a polled input model: call {@link #update(LoopClock)} once per loop cycle to
 * evaluate bindings and run actions.</p>
 *
 * <h2>Per-cycle idempotency</h2>
 * <p>{@link #update(LoopClock)} is <b>idempotent by {@link LoopClock#cycle()}</b>.
 * If called twice in the same loop cycle, the second call is a no-op. This prevents nested
 * code (e.g., a tester suite calling into a tester) from double-firing actions.</p>
 *
 * <h2>Edges and toggles</h2>
 * <p>Bindings are built on {@link BooleanSource} combinators:</p>
 * <ul>
 *   <li>{@link BooleanSource#risingEdge()} for “rise” actions (false → true)</li>
 *   <li>{@link BooleanSource#fallingEdge()} for “fall” actions (true → false)</li>
 *   <li>{@link BooleanSource#toggled()} for toggle state</li>
 * </ul>
 *
 * <p><b>Important:</b> Like any edge/toggle tracker, these sources must be sampled each loop
 * to avoid missing transitions. {@link Bindings#update(LoopClock)} performs that sampling for
 * the bindings you register here.</p>
 */
public final class Bindings {

    // ---------------------------------------------------------------------------------------------
    // Binding record types
    // ---------------------------------------------------------------------------------------------

    /**
     * Simple (edgeSource, action) pair for rising-edge actions.
     */
    private static final class RiseBinding {
        final BooleanSource onRise;
        final Runnable action;

        RiseBinding(BooleanSource onRise, Runnable action) {
            this.onRise = onRise;
            this.action = action;
        }
    }

    /**
     * Simple (edgeSource, action) pair for falling-edge actions.
     */
    private static final class FallBinding {
        final BooleanSource onFall;
        final Runnable action;

        FallBinding(BooleanSource onFall, Runnable action) {
            this.onFall = onFall;
            this.action = action;
        }
    }

    /**
     * Binding that runs an action every loop while true and optionally once on fall.
     */
    private static final class WhileTrueBinding {
        final BooleanSource signal;
        final Runnable whileTrue;
        final BooleanSource onFall; // falling edge
        final Runnable onFallAction; // may be null

        WhileTrueBinding(BooleanSource signal, Runnable whileTrue, BooleanSource onFall, Runnable onFallAction) {
            this.signal = signal;
            this.whileTrue = whileTrue;
            this.onFall = onFall;
            this.onFallAction = onFallAction;
        }
    }

    /**
     * Binding that reports a toggle state to a consumer.
     *
     * <p>Toggle state is owned by the {@link BooleanSource} created with {@link BooleanSource#toggled()}.
     * If you need the same toggle state in multiple places (e.g., as an enable gate <em>and</em> as a binding),
     * create the toggled source once and share it.</p>
     */
    private static final class ToggleBinding {
        final BooleanSource toggled;
        final Consumer<Boolean> consumer;

        boolean initialized = false;
        boolean lastState = false;

        ToggleBinding(BooleanSource toggled, Consumer<Boolean> consumer) {
            this.toggled = toggled;
            this.consumer = consumer;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Binding storage
    // ---------------------------------------------------------------------------------------------

    private final List<RiseBinding> riseBindings = new ArrayList<>();
    private final List<FallBinding> fallBindings = new ArrayList<>();
    private final List<WhileTrueBinding> whileTrueBindings = new ArrayList<>();
    private final List<ToggleBinding> toggleBindings = new ArrayList<>();

    /**
     * Tracks which loop cycle we last updated for, to prevent double-firing actions.
     */
    private long lastUpdatedCycle = Long.MIN_VALUE;

    /**
     * Register an action to run once whenever the given signal rises (false → true).
     *
     * <p>For a gamepad button, “rise” corresponds to “pressed”.</p>
     *
     * @param signal boolean signal to monitor (non-null)
     * @param action action to run once per rising edge (non-null)
     */
    public void onRise(BooleanSource signal, Runnable action) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(action, "action is required");
        riseBindings.add(new RiseBinding(signal.risingEdge(), action));
    }

    /**
     * Register an action to run once whenever the given signal falls (true → false).
     *
     * <p>For a gamepad button, “fall” corresponds to “released”.</p>
     *
     * @param signal boolean signal to monitor (non-null)
     * @param action action to run once per falling edge (non-null)
     */
    public void onFall(BooleanSource signal, Runnable action) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(action, "action is required");
        fallBindings.add(new FallBinding(signal.fallingEdge(), action));
    }

    /**
     * Common pattern: run one action on rise (false → true) and another on fall (true → false).
     *
     * <p>For a gamepad button, rise = pressed and fall = released.</p>
     */
    public void onRiseAndFall(BooleanSource signal, Runnable onRise, Runnable onFall) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(onRise, "onRise is required");
        Objects.requireNonNull(onFall, "onFall is required");
        onRise(signal, onRise);
        onFall(signal, onFall);
    }

    /**
     * Register an action to run every loop while the signal is true.
     *
     * <p>For a gamepad button, this corresponds to “while held”.</p>
     */
    public void whileTrue(BooleanSource signal, Runnable whileTrue) {
        whileTrue(signal, whileTrue, null);
    }

    /**
     * Register an action to run every loop while true and another action once on fall.
     */
    public void whileTrue(BooleanSource signal, Runnable whileTrue, Runnable onFall) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(whileTrue, "whileTrue is required");
        BooleanSource falling = signal.fallingEdge();
        whileTrueBindings.add(new WhileTrueBinding(signal, whileTrue, falling, onFall));
    }

    /**
     * Register a toggle binding: each time the button is pressed, toggle state flips and
     * the appropriate callback runs.
     */
    public void onToggle(BooleanSource button, Runnable onEnabled, Runnable onDisabled) {
        Objects.requireNonNull(onEnabled, "onEnabled is required");
        Objects.requireNonNull(onDisabled, "onDisabled is required");
        onToggle(button, enabled -> {
            if (enabled) onEnabled.run();
            else onDisabled.run();
        });
    }

    /**
     * Register a toggle binding: each time the button is pressed, toggle state flips and the
     * consumer is invoked with the new state.
     */
    public void onToggle(BooleanSource button, Consumer<Boolean> consumer) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(consumer, "consumer is required");
        toggleBindings.add(new ToggleBinding(button.toggled(), consumer));
    }

    /**
     * Run all registered bindings for the current loop.
     *
     * <p>This method is safe to call multiple times in the same cycle: only the first call does work.</p>
     */
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");

        long cyc = clock.cycle();
        if (cyc == lastUpdatedCycle) {
            return;
        }
        lastUpdatedCycle = cyc;

        // Rise
        for (int i = 0; i < riseBindings.size(); i++) {
            RiseBinding b = riseBindings.get(i);
            if (b.onRise.getAsBoolean(clock)) {
                b.action.run();
            }
        }

        // Fall
        for (int i = 0; i < fallBindings.size(); i++) {
            FallBinding b = fallBindings.get(i);
            if (b.onFall.getAsBoolean(clock)) {
                b.action.run();
            }
        }

        // While true + optional fall callback
        for (int i = 0; i < whileTrueBindings.size(); i++) {
            WhileTrueBinding b = whileTrueBindings.get(i);
            if (b.signal.getAsBoolean(clock)) {
                b.whileTrue.run();
            }
            if (b.onFallAction != null && b.onFall.getAsBoolean(clock)) {
                b.onFallAction.run();
            }
        }

        // Toggle
        for (int i = 0; i < toggleBindings.size(); i++) {
            ToggleBinding b = toggleBindings.get(i);
            boolean cur = b.toggled.getAsBoolean(clock);
            if (!b.initialized) {
                b.initialized = true;
                b.lastState = cur;
                continue;
            }
            if (cur != b.lastState) {
                b.lastState = cur;
                b.consumer.accept(cur);
            }
        }
    }

    /**
     * Emit a compact summary of the current binding configuration.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "bindings" : prefix;
        dbg.addData(p + ".class", "Bindings");
        dbg.addData(p + ".rise", riseBindings.size());
        dbg.addData(p + ".fall", fallBindings.size());
        dbg.addData(p + ".whileTrue", whileTrueBindings.size());
        dbg.addData(p + ".toggle", toggleBindings.size());
    }

    /**
     * Clear all registered bindings.
     *
     * <p>This is mainly useful in testing utilities that rebuild bindings dynamically.</p>
     */
    public void clear() {
        riseBindings.clear();
        fallBindings.clear();
        whileTrueBindings.clear();
        toggleBindings.clear();
        lastUpdatedCycle = Long.MIN_VALUE;
    }
}
