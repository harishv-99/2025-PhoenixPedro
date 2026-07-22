package edu.ftcphoenix.fw.input.binding;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;

/**
 * Registration-only view of Phoenix input bindings.
 *
 * <p>Reusable control helpers accept this interface so the same helper can declare mappings on
 * either an always-eligible {@link Bindings} root or a conditional
 * {@link Bindings.ControlContext}. The interface deliberately does not expose the root heartbeat,
 * clearing, or context construction.</p>
 */
public interface BindingRegistrar {

    /**
     * Register an action to run once for each accepted false-to-true transition.
     *
     * <p>A root {@link Bindings} registration uses its first sample only to establish an edge
     * baseline, so starting with a held signal does not call {@code action}. A contextual
     * registration is also silent while inactive and on its effect-free activation sample; its
     * later transitions are accepted according to the context's
     * {@link Bindings.ActivationPolicy}.</p>
     *
     * @param signal boolean source to monitor; must not be {@code null}
     * @param action action to run once per accepted rise; must not be {@code null}
     * @throws NullPointerException if either argument is {@code null}
     */
    void onRise(BooleanSource signal, Runnable action);

    /**
     * Register an action to run once for each accepted true-to-false transition.
     *
     * <p>A root registration uses its first sample only to establish an edge baseline. A
     * contextual registration does not turn deactivation or neutral rearming into a fall; under
     * {@link Bindings.ActivationPolicy#ACCEPT_CURRENT}, releasing a high signal after the
     * effect-free activation sample is a real accepted fall.</p>
     *
     * @param signal boolean source to monitor; must not be {@code null}
     * @param action action to run once per accepted fall; must not be {@code null}
     * @throws NullPointerException if either argument is {@code null}
     */
    void onFall(BooleanSource signal, Runnable action);

    /**
     * Mirror the effective boolean value on its first output and whenever that output changes.
     *
     * <p>A root registration publishes the signal's current value on its first update. A context
     * instead publishes effective neutral {@code false} once when first inactive or on an
     * initially-active effect-free activation frame, publishes accepted active changes, and
     * publishes {@code false} once when a previously true mirror deactivates. It does not sample
     * the signal while inactive or publish neutral repeatedly.</p>
     *
     * <p>Use one mirror registration for a persistent state setter; overlapping root/context
     * writers are not automatically arbitrated.</p>
     *
     * @param signal boolean source to mirror; must not be {@code null}
     * @param consumer receives effective state changes; must not be {@code null}
     * @throws NullPointerException if either argument is {@code null}
     */
    void mirrorOnChange(BooleanSource signal, Consumer<Boolean> consumer);

    /**
     * Register an action to run on every eligible update while the signal is true.
     *
     * <p>A root registration may run on its first update. A contextual registration is silent
     * while inactive, on activation, and on a neutral sample used only to rearm; accepted levels
     * begin on a later update according to its activation policy.</p>
     *
     * @param signal boolean source to monitor; must not be {@code null}
     * @param action action to run on each accepted high update; must not be {@code null}
     * @throws NullPointerException if either argument is {@code null}
     */
    void whileHigh(BooleanSource signal, Runnable action);

    /**
     * Register an action to run on every eligible update while the signal is false.
     *
     * <p>A root registration may run on its first update. Context inactivity, deactivation, and a
     * false sample used only to rearm do not invoke this action; accepted low levels begin on a
     * later active update.</p>
     *
     * @param signal boolean source to monitor; must not be {@code null}
     * @param action action to run on each accepted low update; must not be {@code null}
     * @throws NullPointerException if either argument is {@code null}
     */
    void whileLow(BooleanSource signal, Runnable action);

    /**
     * Toggle owned state on each accepted rise and run the corresponding action.
     *
     * <p>The owned toggle starts disabled ({@code false}). Its first signal sample establishes a
     * baseline and calls neither action. Each later accepted rise flips the state and runs exactly
     * one action. Context inactivity does not reset the owned toggle state.</p>
     *
     * @param signal signal whose accepted rises flip the toggle; must not be {@code null}
     * @param onEnabled action run after a flip to enabled; must not be {@code null}
     * @param onDisabled action run after a flip to disabled; must not be {@code null}
     * @throws NullPointerException if any argument is {@code null}
     */
    void toggleOnRise(BooleanSource signal, Runnable onEnabled, Runnable onDisabled);

    /**
     * Toggle owned state on each accepted rise and publish the new state.
     *
     * <p>The state starts {@code false}; the first sample is a silent baseline. The consumer is
     * invoked only after an accepted rise flips the state. A contextual toggle retains its state
     * while inactive but must satisfy its input activation/rearm policy again.</p>
     *
     * @param signal signal whose accepted rises flip the toggle; must not be {@code null}
     * @param consumer receives the new state after each flip; must not be {@code null}
     * @throws NullPointerException if either argument is {@code null}
     */
    void toggleOnRise(BooleanSource signal, Consumer<Boolean> consumer);

    /**
     * Apply a signed adjustment when either nudge signal has an accepted rise.
     *
     * <p>An increase contributes {@code +abs(step)} and a decrease contributes
     * {@code -abs(step)}. Both signals have independent edge/rearm state, so one held input does
     * not block the other. If both rise on the same update, their equal and opposite contributions
     * cancel and the adjuster is not invoked.</p>
     *
     * @param increaseSignal signal that contributes a positive step; must not be {@code null}
     * @param decreaseSignal signal that contributes a negative step; must not be {@code null}
     * @param step finite adjustment magnitude; its sign is ignored
     * @param adjuster receives the nonzero combined adjustment; must not be {@code null}
     * @throws NullPointerException if a signal or {@code adjuster} is {@code null}
     * @throws IllegalArgumentException if {@code step} is non-finite
     */
    void nudgeOnRise(BooleanSource increaseSignal,
                     BooleanSource decreaseSignal,
                     double step,
                     DoubleConsumer adjuster);

    /**
     * Copy a scalar command on every parent {@link Bindings#update}.
     *
     * <p>A root registration samples and forwards the source value directly on every update; it
     * applies no contextual finite-value or fallback policy. A contextual registration does not
     * sample its source while inactive and publishes exact zero while inactive, on its effect-free
     * activation frame, while {@link Bindings.ActivationPolicy#REARM_AFTER_NEUTRAL} is unarmed, or
     * when a sample is non-finite. Under {@code REARM_AFTER_NEUTRAL}, only a finite exact-zero
     * sample rearms the registration and that sample remains effect-free. Under
     * {@link Bindings.ActivationPolicy#ACCEPT_CURRENT}, the first finite activation sample becomes
     * eligible on the following update and any later finite sample resumes immediately after a
     * non-finite sample.</p>
     *
     * <p>Contextual copying is only for a conditioned final command where exact zero is the safe
     * inactive value. Keep exactly one continuous-copy registration for each final command sink
     * across the root and all contexts; nonzero hold/stow or same-sink arbitration belongs in one
     * robot-composed source registered at the root.</p>
     *
     * @param source scalar command source; must not be {@code null}
     * @param consumer receives the root sample or contextual effective command; must not be
     *                 {@code null}
     * @throws NullPointerException if either argument is {@code null}
     */
    void copyEachCycle(ScalarSource source, DoubleConsumer consumer);
}
