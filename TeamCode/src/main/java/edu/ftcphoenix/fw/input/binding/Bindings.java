package edu.ftcphoenix.fw.input.binding;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Binding manager that maps boolean and scalar sources to higher-level behavior.
 *
 * <p>Phoenix uses a polled input model: call {@link #update(LoopClock)} once per loop cycle to
 * evaluate bindings and run actions.</p>
 *
 * <h2>Per-cycle idempotency</h2>
 * <p>{@link #update(LoopClock)} is <b>idempotent by {@link LoopClock#cycle()}</b>.
 * If called twice in the same loop cycle, the second call is a no-op. This prevents nested
 * code (e.g., a tester suite calling into a tester) from double-firing actions.</p>
 *
 * <h2>When vs. what</h2>
 * <p>The simple edge and level methods name only <em>when</em> an action runs:</p>
 * <ul>
 *   <li>{@link #onRise(BooleanSource, Runnable)} runs once on false → true.</li>
 *   <li>{@link #onFall(BooleanSource, Runnable)} runs once on true → false.</li>
 *   <li>{@link #whileHigh(BooleanSource, Runnable)} and {@link #whileLow(BooleanSource, Runnable)}
 *       run every loop while a signal is in that level.</li>
 * </ul>
 *
 * <p>The stateful convenience methods name both <em>when</em> and <em>what</em>:</p>
 * <ul>
 *   <li>{@link #mirrorOnChange(BooleanSource, Consumer)} mirrors the current signal value to a setter.</li>
 *   <li>{@link #toggleOnRise(BooleanSource, Consumer)} flips an owned toggle state on each rise.</li>
 *   <li>{@link #nudgeOnRise(BooleanSource, BooleanSource, double, DoubleConsumer)} applies signed
 *       step adjustments on rising edges.</li>
 *   <li>{@link #copyEachCycle(ScalarSource, DoubleConsumer)} forwards a scalar value every loop.
 *       This is the standard pairing with frame-valued capability commands.</li>
 * </ul>
 *
 * <h2>Signal vocabulary</h2>
 * <p>Bindings are intentionally written in terms of source semantics, not button names. Boolean
 * bindings can come from a gamepad button, trigger threshold, sensor gate, software mode, or any
 * other {@link BooleanSource}. Continuous copy bindings typically come from a {@link ScalarSource}
 * such as a stick, trigger amount, or shaped manual command source.</p>
 *
 * <p><b>Important:</b> Like any edge/toggle tracker, these sources must be sampled each loop
 * to avoid missing transitions. {@link Bindings#update(LoopClock)} performs that sampling for
 * the bindings you register here.</p>
 */
public final class Bindings {

    private static final class RiseBinding {
        final BooleanSource onRise;
        final Runnable action;

        RiseBinding(BooleanSource onRise, Runnable action) {
            this.onRise = onRise;
            this.action = action;
        }
    }

    private static final class FallBinding {
        final BooleanSource onFall;
        final Runnable action;

        FallBinding(BooleanSource onFall, Runnable action) {
            this.onFall = onFall;
            this.action = action;
        }
    }

    private static final class MirrorOnChangeBinding {
        final BooleanSource signal;
        final Consumer<Boolean> consumer;

        boolean initialized = false;
        boolean lastState = false;

        MirrorOnChangeBinding(BooleanSource signal, Consumer<Boolean> consumer) {
            this.signal = signal;
            this.consumer = consumer;
        }
    }

    private static final class LevelBinding {
        final BooleanSource signal;
        final boolean runWhenHigh;
        final Runnable action;

        LevelBinding(BooleanSource signal, boolean runWhenHigh, Runnable action) {
            this.signal = signal;
            this.runWhenHigh = runWhenHigh;
            this.action = action;
        }
    }

    private static final class ToggleOnRiseBinding {
        final BooleanSource toggled;
        final Consumer<Boolean> consumer;

        boolean initialized = false;
        boolean lastState = false;

        ToggleOnRiseBinding(BooleanSource toggled, Consumer<Boolean> consumer) {
            this.toggled = toggled;
            this.consumer = consumer;
        }
    }

    private static final class NudgeOnRiseBinding {
        final BooleanSource increaseRise;
        final BooleanSource decreaseRise;
        final double step;
        final DoubleConsumer adjuster;

        NudgeOnRiseBinding(BooleanSource increaseRise,
                           BooleanSource decreaseRise,
                           double step,
                           DoubleConsumer adjuster) {
            this.increaseRise = increaseRise;
            this.decreaseRise = decreaseRise;
            this.step = step;
            this.adjuster = adjuster;
        }
    }

    private static final class ScalarCopyBinding {
        final ScalarSource source;
        final DoubleConsumer consumer;

        ScalarCopyBinding(ScalarSource source, DoubleConsumer consumer) {
            this.source = source;
            this.consumer = consumer;
        }
    }

    private final List<RiseBinding> riseBindings = new ArrayList<>();
    private final List<FallBinding> fallBindings = new ArrayList<>();
    private final List<MirrorOnChangeBinding> mirrorOnChangeBindings = new ArrayList<>();
    private final List<LevelBinding> levelBindings = new ArrayList<>();
    private final List<ToggleOnRiseBinding> toggleOnRiseBindings = new ArrayList<>();
    private final List<NudgeOnRiseBinding> nudgeOnRiseBindings = new ArrayList<>();
    private final List<ScalarCopyBinding> scalarCopyBindings = new ArrayList<>();

    private long lastUpdatedCycle = Long.MIN_VALUE;

    /**
     * Register an action to run once whenever the given signal rises (false → true).
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
     * @param signal boolean signal to monitor (non-null)
     * @param action action to run once per falling edge (non-null)
     */
    public void onFall(BooleanSource signal, Runnable action) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(action, "action is required");
        fallBindings.add(new FallBinding(signal.fallingEdge(), action));
    }

    /**
     * Mirror the signal's current value into a boolean consumer on the first update, then again whenever it changes.
     *
     * <p>This is the preferred binding when a signal should directly control, or mirror into, a boolean setter:</p>
     *
     * <pre>{@code
     * bindings.mirrorOnChange(operator.b(), scoring::setShootingEnabled);
     * }</pre>
     *
     * <p>The first sample is treated as a change from an uninitialized state, so the consumer is
     * called once with the current high/low value. Later calls happen only when the value changes.</p>
     *
     * @param signal   boolean signal to mirror (non-null)
     * @param consumer receives the mirrored signal value on first sample and later changes (non-null)
     */
    public void mirrorOnChange(BooleanSource signal, Consumer<Boolean> consumer) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(consumer, "consumer is required");
        mirrorOnChangeBindings.add(new MirrorOnChangeBinding(signal, consumer));
    }

    /**
     * Register an action to run every loop while the signal is high/true.
     *
     * @param signal boolean signal to monitor (non-null)
     * @param action action to run every high loop (non-null)
     */
    public void whileHigh(BooleanSource signal, Runnable action) {
        addLevelBinding(signal, true, action);
    }

    /**
     * Register an action to run every loop while the signal is low/false.
     *
     * @param signal boolean signal to monitor (non-null)
     * @param action action to run every low loop (non-null)
     */
    public void whileLow(BooleanSource signal, Runnable action) {
        addLevelBinding(signal, false, action);
    }

    private void addLevelBinding(BooleanSource signal, boolean runWhenHigh, Runnable action) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(action, "action is required");
        levelBindings.add(new LevelBinding(signal, runWhenHigh, action));
    }

    /**
     * Register a toggle binding: each time the signal rises, toggle state flips and the appropriate
     * callback runs.
     *
     * @param signal     boolean signal to monitor (non-null)
     * @param onEnabled  action to run when the owned toggle state becomes true (non-null)
     * @param onDisabled action to run when the owned toggle state becomes false (non-null)
     */
    public void toggleOnRise(BooleanSource signal, Runnable onEnabled, Runnable onDisabled) {
        Objects.requireNonNull(onEnabled, "onEnabled is required");
        Objects.requireNonNull(onDisabled, "onDisabled is required");
        toggleOnRise(signal, enabled -> {
            if (enabled) onEnabled.run();
            else onDisabled.run();
        });
    }

    /**
     * Register a toggle binding: each time the signal rises, toggle state flips and the consumer is
     * invoked with the new state.
     *
     * <p>The initial toggle state is false. The first sample establishes the edge baseline and does
     * not call the consumer.</p>
     *
     * @param signal   boolean signal to monitor (non-null)
     * @param consumer receives the owned toggle state after each rising-edge flip (non-null)
     */
    public void toggleOnRise(BooleanSource signal, Consumer<Boolean> consumer) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(consumer, "consumer is required");
        toggleOnRiseBindings.add(new ToggleOnRiseBinding(signal.toggled(), consumer));
    }

    /**
     * Apply signed step adjustments when either nudge signal rises.
     *
     * <p>When {@code increaseSignal} rises, {@code adjuster.accept(+step)} is called. When
     * {@code decreaseSignal} rises, {@code adjuster.accept(-step)} is called. If both rise in the
     * same loop, their deltas are combined before the adjuster is called, so equal and opposite
     * nudges cancel out.</p>
     *
     * @param increaseSignal signal that nudges upward on rise (non-null)
     * @param decreaseSignal signal that nudges downward on rise (non-null)
     * @param step           adjustment size; sign is interpreted relative to the two signals
     * @param adjuster       receives the combined signed adjustment for the loop (non-null)
     */
    public void nudgeOnRise(BooleanSource increaseSignal,
                            BooleanSource decreaseSignal,
                            double step,
                            DoubleConsumer adjuster) {
        Objects.requireNonNull(increaseSignal, "increaseSignal is required");
        Objects.requireNonNull(decreaseSignal, "decreaseSignal is required");
        Objects.requireNonNull(adjuster, "adjuster is required");
        if (Double.isNaN(step) || Double.isInfinite(step)) {
            throw new IllegalArgumentException("step must be finite");
        }
        nudgeOnRiseBindings.add(new NudgeOnRiseBinding(
                increaseSignal.risingEdge(),
                decreaseSignal.risingEdge(),
                Math.abs(step),
                adjuster
        ));
    }

    /**
     * Copy the current scalar value into a consumer every loop.
     *
     * <p>This is the preferred binding for continuous non-drive mechanism commands that are modeled
     * as frame-valued capability methods:</p>
     *
     * <pre>{@code
     * bindings.copyEachCycle(operator.leftY().deadbandNormalized(0.08, -1.0, 1.0),
     *         lift::commandManualPower);
     * }</pre>
     *
     * <p>The consumer is called once per loop with the current sampled value. This pairs naturally
     * with helpers such as {@code FrameValue<T>}, where missing writes on later cycles should fall
     * back automatically to a default.</p>
     *
     * @param source   scalar source to sample every loop (non-null)
     * @param consumer consumer that receives the sampled value every loop (non-null)
     */
    public void copyEachCycle(ScalarSource source, DoubleConsumer consumer) {
        Objects.requireNonNull(source, "source is required");
        Objects.requireNonNull(consumer, "consumer is required");
        scalarCopyBindings.add(new ScalarCopyBinding(source, consumer));
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

        for (int i = 0; i < riseBindings.size(); i++) {
            RiseBinding b = riseBindings.get(i);
            if (b.onRise.getAsBoolean(clock)) {
                b.action.run();
            }
        }

        for (int i = 0; i < fallBindings.size(); i++) {
            FallBinding b = fallBindings.get(i);
            if (b.onFall.getAsBoolean(clock)) {
                b.action.run();
            }
        }

        for (int i = 0; i < mirrorOnChangeBindings.size(); i++) {
            MirrorOnChangeBinding b = mirrorOnChangeBindings.get(i);
            boolean cur = b.signal.getAsBoolean(clock);
            if (!b.initialized || cur != b.lastState) {
                b.initialized = true;
                b.lastState = cur;
                b.consumer.accept(cur);
            }
        }

        for (int i = 0; i < levelBindings.size(); i++) {
            LevelBinding b = levelBindings.get(i);
            if (b.signal.getAsBoolean(clock) == b.runWhenHigh) {
                b.action.run();
            }
        }

        for (int i = 0; i < toggleOnRiseBindings.size(); i++) {
            ToggleOnRiseBinding b = toggleOnRiseBindings.get(i);
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

        for (int i = 0; i < nudgeOnRiseBindings.size(); i++) {
            NudgeOnRiseBinding b = nudgeOnRiseBindings.get(i);
            double delta = 0.0;
            if (b.increaseRise.getAsBoolean(clock)) {
                delta += b.step;
            }
            if (b.decreaseRise.getAsBoolean(clock)) {
                delta -= b.step;
            }
            if (delta != 0.0) {
                b.adjuster.accept(delta);
            }
        }

        for (int i = 0; i < scalarCopyBindings.size(); i++) {
            ScalarCopyBinding b = scalarCopyBindings.get(i);
            b.consumer.accept(b.source.getAsDouble(clock));
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
        dbg.addData(p + ".mirrorOnChange", mirrorOnChangeBindings.size());
        dbg.addData(p + ".level", levelBindings.size());
        dbg.addData(p + ".toggleOnRise", toggleOnRiseBindings.size());
        dbg.addData(p + ".nudgeOnRise", nudgeOnRiseBindings.size());
        dbg.addData(p + ".copyEachCycle", scalarCopyBindings.size());
    }

    /**
     * Clear all registered bindings.
     *
     * <p>This is mainly useful in testing utilities that rebuild bindings dynamically.</p>
     */
    public void clear() {
        riseBindings.clear();
        fallBindings.clear();
        mirrorOnChangeBindings.clear();
        levelBindings.clear();
        toggleOnRiseBindings.clear();
        nudgeOnRiseBindings.clear();
        scalarCopyBindings.clear();
        lastUpdatedCycle = Long.MIN_VALUE;
    }
}
