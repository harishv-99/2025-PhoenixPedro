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
 * evaluate bindings and run actions. Declare ordinary always-eligible mappings directly on this
 * object. Use {@link #contextWhen(BooleanSource, ActivationPolicy)} when a related group of mappings
 * should be eligible only while a mode or other condition is active.</p>
 *
 * <h2>Robot controls example</h2>
 * <pre>{@code
 * Bindings bindings = new Bindings();
 *
 * // This safety control is always eligible.
 * bindings.onRise(operator.back(), robot::cancelAutomation);
 *
 * // These controls are eligible only in the robot's endgame mode.
 * BooleanSource endgameMode = BooleanSource.of(() -> mode == RobotMode.ENDGAME);
 * Bindings.ControlContext endgameControls = bindings.contextWhen(
 *         endgameMode,
 *         Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
 * endgameControls.onRise(operator.a(), endgame::deploy);
 * endgameControls.copyEachCycle(
 *         operator.leftY().deadbandNormalized(0.08, -1.0, 1.0),
 *         endgame::commandWinchPower);
 *
 * // The root remains the only heartbeat, even when contexts are present.
 * bindings.update(clock);
 * }</pre>
 *
 * <h2>Per-cycle idempotency</h2>
 * <p>{@link #update(LoopClock)} is <b>idempotent by {@link LoopClock#cycle()}</b>.
 * If called twice in the same loop cycle, the second call is a no-op. This prevents nested
 * code (for example, a tester suite calling into a tester) from double-firing actions.</p>
 *
 * <h2>When vs. what</h2>
 * <p>The simple edge and level methods name only <em>when</em> an action runs:</p>
 * <ul>
 *   <li>{@link #onRise(BooleanSource, Runnable)} runs once on false to true.</li>
 *   <li>{@link #onFall(BooleanSource, Runnable)} runs once on true to false.</li>
 *   <li>{@link #whileHigh(BooleanSource, Runnable)} and {@link #whileLow(BooleanSource, Runnable)}
 *       run every eligible loop while a signal is in that level.</li>
 * </ul>
 *
 * <p>The stateful convenience methods name both <em>when</em> and <em>what</em>:</p>
 * <ul>
 *   <li>{@link #mirrorOnChange(BooleanSource, Consumer)} mirrors a signal into a setter.</li>
 *   <li>{@link #toggleOnRise(BooleanSource, Consumer)} flips owned state on each rise.</li>
 *   <li>{@link #nudgeOnRise(BooleanSource, BooleanSource, double, DoubleConsumer)} applies signed
 *       step adjustments on rising edges.</li>
 *   <li>{@link #copyEachCycle(ScalarSource, DoubleConsumer)} forwards a scalar value every loop.</li>
 * </ul>
 *
 * <p>Bindings are intentionally written in terms of source semantics, not button names. Root edge
 * and toggle registrations must be sampled each loop to avoid missing transitions. This object's
 * single {@code update} heartbeat samples always-eligible mappings and every context activation;
 * contextual input sources intentionally remain unsampled while inactive and begin sampling only
 * under their context's activation policy.</p>
 */
public final class Bindings implements BindingRegistrar {

    /**
     * Select how a context treats controls that are already active when the context becomes active.
     */
    public enum ActivationPolicy {
        /**
         * Require each boolean to be observed false and each scalar to be observed at finite exact
         * zero before that individual input can produce an active effect. Rearming is independent
         * per registered input, and the neutral sample itself is effect-free.
         */
        REARM_AFTER_NEUTRAL,

        /**
         * Establish the current value as an effect-free baseline, then accept it beginning on the
         * following loop. Held levels, mirrors, and finite scalar commands may then become visible,
         * but no button edge or toggle is manufactured by activation.
         */
        ACCEPT_CURRENT
    }

    /**
     * Conditional registration surface owned and updated by one parent {@link Bindings} object.
     *
     * <p>A context has no independent heartbeat. Call only the parent {@link Bindings#update}.
     * Every context activation predicate is sampled before any binding callback in that loop, so a
     * callback that changes a mode affects contexts on the next loop rather than midway through
     * the current controls frame.</p>
     *
     * <p>Contexts may overlap; all eligible mappings run. The framework does not infer priority,
     * consume inputs, arbitrate duplicate setters, cancel tasks, or perform subsystem cleanup.</p>
     */
    public static final class ControlContext implements BindingRegistrar {
        private final Bindings owner;
        private final BooleanSource activation;
        private final ActivationPolicy policy;

        private boolean valid = true;
        private boolean activeSnapshot = false;

        private ControlContext(Bindings owner,
                               BooleanSource activation,
                               ActivationPolicy policy) {
            this.owner = owner;
            this.activation = activation;
            this.policy = policy;
        }

        /** {@inheritDoc} */
        @Override
        public void onRise(BooleanSource signal, Runnable action) {
            requireValid();
            owner.addRiseBinding(this, signal, action);
        }

        /** {@inheritDoc} */
        @Override
        public void onFall(BooleanSource signal, Runnable action) {
            requireValid();
            owner.addFallBinding(this, signal, action);
        }

        /** {@inheritDoc} */
        @Override
        public void mirrorOnChange(BooleanSource signal, Consumer<Boolean> consumer) {
            requireValid();
            owner.addMirrorOnChangeBinding(this, signal, consumer);
        }

        /** {@inheritDoc} */
        @Override
        public void whileHigh(BooleanSource signal, Runnable action) {
            requireValid();
            owner.addLevelBinding(this, signal, true, action);
        }

        /** {@inheritDoc} */
        @Override
        public void whileLow(BooleanSource signal, Runnable action) {
            requireValid();
            owner.addLevelBinding(this, signal, false, action);
        }

        /** {@inheritDoc} */
        @Override
        public void toggleOnRise(BooleanSource signal,
                                 Runnable onEnabled,
                                 Runnable onDisabled) {
            requireValid();
            Objects.requireNonNull(onEnabled, "onEnabled is required");
            Objects.requireNonNull(onDisabled, "onDisabled is required");
            toggleOnRise(signal, enabled -> {
                if (enabled) {
                    onEnabled.run();
                } else {
                    onDisabled.run();
                }
            });
        }

        /** {@inheritDoc} */
        @Override
        public void toggleOnRise(BooleanSource signal, Consumer<Boolean> consumer) {
            requireValid();
            owner.addToggleOnRiseBinding(this, signal, consumer);
        }

        /** {@inheritDoc} */
        @Override
        public void nudgeOnRise(BooleanSource increaseSignal,
                                BooleanSource decreaseSignal,
                                double step,
                                DoubleConsumer adjuster) {
            requireValid();
            owner.addNudgeOnRiseBinding(this, increaseSignal, decreaseSignal, step, adjuster);
        }

        /** {@inheritDoc} */
        @Override
        public void copyEachCycle(ScalarSource source, DoubleConsumer consumer) {
            requireValid();
            owner.addScalarCopyBinding(this, source, consumer);
        }

        private void snapshotActivation(LoopClock clock) {
            activeSnapshot = activation.getAsBoolean(clock);
        }

        private void invalidate() {
            valid = false;
            activeSnapshot = false;
        }

        private void requireValid() {
            if (!valid) {
                throw new IllegalStateException(
                        "control context is no longer valid because its parent Bindings was cleared; "
                                + "create a new context from the rebuilt Bindings");
            }
        }
    }

    /** Tracks one contextual boolean's baseline and independent rearm state. */
    private static final class ContextualBooleanState {
        boolean initialized;
        boolean armed;
        boolean previous;
        boolean current;
        boolean rose;
        boolean fell;

        boolean sample(ControlContext context, BooleanSource signal, LoopClock clock) {
            rose = false;
            fell = false;

            if (!context.activeSnapshot) {
                resetForInactive();
                return false;
            }

            current = signal.getAsBoolean(clock);
            if (!initialized) {
                initialized = true;
                previous = current;
                armed = context.policy == ActivationPolicy.ACCEPT_CURRENT || !current;
                return false;
            }

            if (!armed) {
                previous = current;
                if (!current) {
                    armed = true;
                }
                return false;
            }

            rose = current && !previous;
            fell = !current && previous;
            previous = current;
            return true;
        }

        void resetForInactive() {
            initialized = false;
            armed = false;
            previous = false;
            current = false;
            rose = false;
            fell = false;
        }
    }

    private static final class RiseBinding {
        final ControlContext context;
        final BooleanSource signal;
        final Runnable action;
        final ContextualBooleanState contextualState;

        RiseBinding(ControlContext context, BooleanSource signal, Runnable action) {
            this.context = context;
            this.signal = context == null ? signal.risingEdge() : signal;
            this.action = action;
            this.contextualState = context == null ? null : new ContextualBooleanState();
        }

        void dispatch(LoopClock clock) {
            if (context == null) {
                if (signal.getAsBoolean(clock)) {
                    action.run();
                }
            } else if (contextualState.sample(context, signal, clock) && contextualState.rose) {
                action.run();
            }
        }
    }

    private static final class FallBinding {
        final ControlContext context;
        final BooleanSource signal;
        final Runnable action;
        final ContextualBooleanState contextualState;

        FallBinding(ControlContext context, BooleanSource signal, Runnable action) {
            this.context = context;
            this.signal = context == null ? signal.fallingEdge() : signal;
            this.action = action;
            this.contextualState = context == null ? null : new ContextualBooleanState();
        }

        void dispatch(LoopClock clock) {
            if (context == null) {
                if (signal.getAsBoolean(clock)) {
                    action.run();
                }
            } else if (contextualState.sample(context, signal, clock) && contextualState.fell) {
                action.run();
            }
        }
    }

    private static final class MirrorOnChangeBinding {
        final ControlContext context;
        final BooleanSource signal;
        final Consumer<Boolean> consumer;
        final ContextualBooleanState contextualState;

        boolean outputInitialized;
        boolean lastOutput;

        MirrorOnChangeBinding(ControlContext context,
                              BooleanSource signal,
                              Consumer<Boolean> consumer) {
            this.context = context;
            this.signal = signal;
            this.consumer = consumer;
            this.contextualState = context == null ? null : new ContextualBooleanState();
        }

        void dispatch(LoopClock clock) {
            if (context == null) {
                publish(signal.getAsBoolean(clock));
                return;
            }

            boolean accepted = contextualState.sample(context, signal, clock);
            publish(accepted ? contextualState.current : false);
        }

        private void publish(boolean value) {
            if (!outputInitialized || value != lastOutput) {
                outputInitialized = true;
                lastOutput = value;
                consumer.accept(value);
            }
        }
    }

    private static final class LevelBinding {
        final ControlContext context;
        final BooleanSource signal;
        final boolean runWhenHigh;
        final Runnable action;
        final ContextualBooleanState contextualState;

        LevelBinding(ControlContext context,
                     BooleanSource signal,
                     boolean runWhenHigh,
                     Runnable action) {
            this.context = context;
            this.signal = signal;
            this.runWhenHigh = runWhenHigh;
            this.action = action;
            this.contextualState = context == null ? null : new ContextualBooleanState();
        }

        void dispatch(LoopClock clock) {
            if (context == null) {
                if (signal.getAsBoolean(clock) == runWhenHigh) {
                    action.run();
                }
                return;
            }

            if (contextualState.sample(context, signal, clock)
                    && contextualState.current == runWhenHigh) {
                action.run();
            }
        }
    }

    private static final class ToggleOnRiseBinding {
        final ControlContext context;
        final BooleanSource signal;
        final Consumer<Boolean> consumer;
        final ContextualBooleanState contextualState;

        boolean rootInitialized;
        boolean rootLastState;
        boolean contextualToggleState;

        ToggleOnRiseBinding(ControlContext context,
                            BooleanSource signal,
                            Consumer<Boolean> consumer) {
            this.context = context;
            this.signal = context == null ? signal.toggled() : signal;
            this.consumer = consumer;
            this.contextualState = context == null ? null : new ContextualBooleanState();
        }

        void dispatch(LoopClock clock) {
            if (context == null) {
                boolean current = signal.getAsBoolean(clock);
                if (!rootInitialized) {
                    rootInitialized = true;
                    rootLastState = current;
                    return;
                }
                if (current != rootLastState) {
                    rootLastState = current;
                    consumer.accept(current);
                }
                return;
            }

            if (contextualState.sample(context, signal, clock) && contextualState.rose) {
                contextualToggleState = !contextualToggleState;
                consumer.accept(contextualToggleState);
            }
        }
    }

    private static final class NudgeOnRiseBinding {
        final ControlContext context;
        final BooleanSource increaseSignal;
        final BooleanSource decreaseSignal;
        final double step;
        final DoubleConsumer adjuster;
        final ContextualBooleanState increaseState;
        final ContextualBooleanState decreaseState;

        NudgeOnRiseBinding(ControlContext context,
                           BooleanSource increaseSignal,
                           BooleanSource decreaseSignal,
                           double step,
                           DoubleConsumer adjuster) {
            this.context = context;
            this.increaseSignal = context == null ? increaseSignal.risingEdge() : increaseSignal;
            this.decreaseSignal = context == null ? decreaseSignal.risingEdge() : decreaseSignal;
            this.step = step;
            this.adjuster = adjuster;
            this.increaseState = context == null ? null : new ContextualBooleanState();
            this.decreaseState = context == null ? null : new ContextualBooleanState();
        }

        void dispatch(LoopClock clock) {
            double delta = 0.0;
            if (context == null) {
                if (increaseSignal.getAsBoolean(clock)) {
                    delta += step;
                }
                if (decreaseSignal.getAsBoolean(clock)) {
                    delta -= step;
                }
            } else {
                boolean increaseAccepted = increaseState.sample(context, increaseSignal, clock);
                boolean decreaseAccepted = decreaseState.sample(context, decreaseSignal, clock);
                if (increaseAccepted && increaseState.rose) {
                    delta += step;
                }
                if (decreaseAccepted && decreaseState.rose) {
                    delta -= step;
                }
            }

            if (delta != 0.0) {
                adjuster.accept(delta);
            }
        }
    }

    private static final class ScalarCopyBinding {
        final ControlContext context;
        final ScalarSource source;
        final DoubleConsumer consumer;

        boolean initialized;
        boolean armed;

        ScalarCopyBinding(ControlContext context, ScalarSource source, DoubleConsumer consumer) {
            this.context = context;
            this.source = source;
            this.consumer = consumer;
        }

        void dispatch(LoopClock clock) {
            if (context == null) {
                consumer.accept(source.getAsDouble(clock));
                return;
            }

            if (!context.activeSnapshot) {
                initialized = false;
                armed = false;
                consumer.accept(0.0);
                return;
            }

            double value = source.getAsDouble(clock);
            boolean finite = Double.isFinite(value);
            if (!initialized) {
                initialized = true;
                armed = context.policy == ActivationPolicy.ACCEPT_CURRENT
                        ? finite
                        : finite && value == 0.0;
                consumer.accept(0.0);
                return;
            }

            if (context.policy == ActivationPolicy.ACCEPT_CURRENT) {
                if (finite) {
                    armed = true;
                    consumer.accept(value);
                } else {
                    armed = false;
                    consumer.accept(0.0);
                }
                return;
            }

            if (!finite) {
                armed = false;
                consumer.accept(0.0);
            } else if (!armed) {
                if (value == 0.0) {
                    armed = true;
                }
                consumer.accept(0.0);
            } else {
                consumer.accept(value);
            }
        }
    }

    private final List<ControlContext> contexts = new ArrayList<>();
    private final List<RiseBinding> riseBindings = new ArrayList<>();
    private final List<FallBinding> fallBindings = new ArrayList<>();
    private final List<MirrorOnChangeBinding> mirrorOnChangeBindings = new ArrayList<>();
    private final List<LevelBinding> levelBindings = new ArrayList<>();
    private final List<ToggleOnRiseBinding> toggleOnRiseBindings = new ArrayList<>();
    private final List<NudgeOnRiseBinding> nudgeOnRiseBindings = new ArrayList<>();
    private final List<ScalarCopyBinding> scalarCopyBindings = new ArrayList<>();

    private long lastUpdatedCycle = Long.MIN_VALUE;

    /**
     * Create a conditional control context owned by this root.
     *
     * <p>The activation policy is required because accepting a held button or displaced scalar is
     * motion-relevant behavior. Continue calling only this root's {@link #update(LoopClock)}.</p>
     *
     * @param activation source that makes the context active while true
     * @param policy required held-control activation policy
     * @return a registration-only contextual surface
     * @throws NullPointerException if {@code activation} or {@code policy} is {@code null}
     */
    public ControlContext contextWhen(BooleanSource activation, ActivationPolicy policy) {
        ControlContext context = new ControlContext(
                this,
                Objects.requireNonNull(activation, "activation is required"),
                Objects.requireNonNull(policy, "policy is required"));
        contexts.add(context);
        return context;
    }

    /** {@inheritDoc} */
    @Override
    public void onRise(BooleanSource signal, Runnable action) {
        addRiseBinding(null, signal, action);
    }

    private void addRiseBinding(ControlContext context, BooleanSource signal, Runnable action) {
        riseBindings.add(new RiseBinding(
                context,
                Objects.requireNonNull(signal, "signal is required"),
                Objects.requireNonNull(action, "action is required")));
    }

    /** {@inheritDoc} */
    @Override
    public void onFall(BooleanSource signal, Runnable action) {
        addFallBinding(null, signal, action);
    }

    private void addFallBinding(ControlContext context, BooleanSource signal, Runnable action) {
        fallBindings.add(new FallBinding(
                context,
                Objects.requireNonNull(signal, "signal is required"),
                Objects.requireNonNull(action, "action is required")));
    }

    /** {@inheritDoc} */
    @Override
    public void mirrorOnChange(BooleanSource signal, Consumer<Boolean> consumer) {
        addMirrorOnChangeBinding(null, signal, consumer);
    }

    private void addMirrorOnChangeBinding(ControlContext context,
                                          BooleanSource signal,
                                          Consumer<Boolean> consumer) {
        mirrorOnChangeBindings.add(new MirrorOnChangeBinding(
                context,
                Objects.requireNonNull(signal, "signal is required"),
                Objects.requireNonNull(consumer, "consumer is required")));
    }

    /** {@inheritDoc} */
    @Override
    public void whileHigh(BooleanSource signal, Runnable action) {
        addLevelBinding(null, signal, true, action);
    }

    /** {@inheritDoc} */
    @Override
    public void whileLow(BooleanSource signal, Runnable action) {
        addLevelBinding(null, signal, false, action);
    }

    private void addLevelBinding(ControlContext context,
                                 BooleanSource signal,
                                 boolean runWhenHigh,
                                 Runnable action) {
        levelBindings.add(new LevelBinding(
                context,
                Objects.requireNonNull(signal, "signal is required"),
                runWhenHigh,
                Objects.requireNonNull(action, "action is required")));
    }

    /** {@inheritDoc} */
    @Override
    public void toggleOnRise(BooleanSource signal, Runnable onEnabled, Runnable onDisabled) {
        Objects.requireNonNull(onEnabled, "onEnabled is required");
        Objects.requireNonNull(onDisabled, "onDisabled is required");
        toggleOnRise(signal, enabled -> {
            if (enabled) {
                onEnabled.run();
            } else {
                onDisabled.run();
            }
        });
    }

    /** {@inheritDoc} */
    @Override
    public void toggleOnRise(BooleanSource signal, Consumer<Boolean> consumer) {
        addToggleOnRiseBinding(null, signal, consumer);
    }

    private void addToggleOnRiseBinding(ControlContext context,
                                        BooleanSource signal,
                                        Consumer<Boolean> consumer) {
        toggleOnRiseBindings.add(new ToggleOnRiseBinding(
                context,
                Objects.requireNonNull(signal, "signal is required"),
                Objects.requireNonNull(consumer, "consumer is required")));
    }

    /** {@inheritDoc} */
    @Override
    public void nudgeOnRise(BooleanSource increaseSignal,
                            BooleanSource decreaseSignal,
                            double step,
                            DoubleConsumer adjuster) {
        addNudgeOnRiseBinding(null, increaseSignal, decreaseSignal, step, adjuster);
    }

    private void addNudgeOnRiseBinding(ControlContext context,
                                       BooleanSource increaseSignal,
                                       BooleanSource decreaseSignal,
                                       double step,
                                       DoubleConsumer adjuster) {
        Objects.requireNonNull(increaseSignal, "increaseSignal is required");
        Objects.requireNonNull(decreaseSignal, "decreaseSignal is required");
        Objects.requireNonNull(adjuster, "adjuster is required");
        if (!Double.isFinite(step)) {
            throw new IllegalArgumentException("step must be finite");
        }
        nudgeOnRiseBindings.add(new NudgeOnRiseBinding(
                context,
                increaseSignal,
                decreaseSignal,
                Math.abs(step),
                adjuster));
    }

    /** {@inheritDoc} */
    @Override
    public void copyEachCycle(ScalarSource source, DoubleConsumer consumer) {
        addScalarCopyBinding(null, source, consumer);
    }

    private void addScalarCopyBinding(ControlContext context,
                                      ScalarSource source,
                                      DoubleConsumer consumer) {
        scalarCopyBindings.add(new ScalarCopyBinding(
                context,
                Objects.requireNonNull(source, "source is required"),
                Objects.requireNonNull(consumer, "consumer is required")));
    }

    /**
     * Run all registered root and contextual bindings for the current loop.
     *
     * <p>All context activation predicates are sampled once before any callbacks. This method is
     * safe to call repeatedly in the same cycle; only the first call does work.</p>
     *
     * @param clock current shared loop clock; must not be {@code null}
     * @throws NullPointerException if {@code clock} is {@code null}
     */
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");

        long cycle = clock.cycle();
        if (cycle == lastUpdatedCycle) {
            return;
        }
        lastUpdatedCycle = cycle;

        int contextCount = contexts.size();
        for (int i = 0; i < contextCount; i++) {
            contexts.get(i).snapshotActivation(clock);
        }

        for (int i = 0; i < riseBindings.size(); i++) {
            riseBindings.get(i).dispatch(clock);
        }
        for (int i = 0; i < fallBindings.size(); i++) {
            fallBindings.get(i).dispatch(clock);
        }
        for (int i = 0; i < mirrorOnChangeBindings.size(); i++) {
            mirrorOnChangeBindings.get(i).dispatch(clock);
        }
        for (int i = 0; i < levelBindings.size(); i++) {
            levelBindings.get(i).dispatch(clock);
        }
        for (int i = 0; i < toggleOnRiseBindings.size(); i++) {
            toggleOnRiseBindings.get(i).dispatch(clock);
        }
        for (int i = 0; i < nudgeOnRiseBindings.size(); i++) {
            nudgeOnRiseBindings.get(i).dispatch(clock);
        }
        for (int i = 0; i < scalarCopyBindings.size(); i++) {
            scalarCopyBindings.get(i).dispatch(clock);
        }
    }

    /** Emit a compact summary of the current binding configuration. */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "bindings" : prefix;
        dbg.addData(p + ".class", "Bindings");
        dbg.addData(p + ".contexts", contexts.size());
        dbg.addData(p + ".rise", riseBindings.size());
        dbg.addData(p + ".fall", fallBindings.size());
        dbg.addData(p + ".mirrorOnChange", mirrorOnChangeBindings.size());
        dbg.addData(p + ".level", levelBindings.size());
        dbg.addData(p + ".toggleOnRise", toggleOnRiseBindings.size());
        dbg.addData(p + ".nudgeOnRise", nudgeOnRiseBindings.size());
        dbg.addData(p + ".copyEachCycle", scalarCopyBindings.size());
    }

    /**
     * Clear all registered bindings and invalidate every context created by this root.
     *
     * <p>This is mainly useful in testing utilities that rebuild bindings dynamically. A caller
     * must create new contexts after clearing; declaring through an old context fails fast.</p>
     */
    public void clear() {
        for (int i = 0; i < contexts.size(); i++) {
            contexts.get(i).invalidate();
        }
        contexts.clear();
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
