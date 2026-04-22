package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Convenience wrapper that binds {@link BooleanSource} events to {@link TaskRunner} enqueues.
 *
 * <p>This is a thin adapter around {@link Bindings} + {@link TaskRunner}. It exists
 * purely to make robot code more readable when most bindings just "enqueue a task".</p>
 *
 * <h2>Usage</h2>
 *
 * <pre>{@code
 * Bindings bindings = new Bindings();
 * TaskRunner runner = new TaskRunner();
 * TaskBindings tb = TaskBindings.of(bindings, runner);
 *
 * tb.onRise(gamepads.p2().y(), shooter::instantStartShooter);
 * tb.toggleOnRise(gamepads.p2().rightBumper(), shooter::instantStartShooter, shooter::instantStopShooter);
 * tb.mirrorOnChange(gamepads.p2().b(), high -> high
 *         ? shooter.instantStartTransfer(Shooter.TransferDirection.FORWARD)
 *         : shooter.instantStopTransfer());
 * }</pre>
 *
 * <p><b>Important:</b> Tasks are generally single-use. For that reason, this API
 * takes {@link Supplier} factories so each signal event can create a fresh {@link Task}.</p>
 */
public final class TaskBindings {

    private final Bindings bindings;
    private final TaskRunner runner;

    private TaskBindings(Bindings bindings, TaskRunner runner) {
        this.bindings = Objects.requireNonNull(bindings, "bindings is required");
        this.runner = Objects.requireNonNull(runner, "runner is required");
    }

    /**
     * Create a {@link TaskBindings} wrapper.
     */
    public static TaskBindings of(Bindings bindings, TaskRunner runner) {
        return new TaskBindings(bindings, runner);
    }

    /**
     * Enqueue a task when the signal rises (false → true).
     */
    public void onRise(BooleanSource signal, Supplier<Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.onRise(signal, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Enqueue a task when the signal falls (true → false).
     */
    public void onFall(BooleanSource signal, Supplier<Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.onFall(signal, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Enqueue a task that mirrors the signal value on the first sample and whenever the signal changes.
     *
     * <p>The factory receives the current high/low value so it can create the appropriate task.</p>
     */
    public void mirrorOnChange(BooleanSource signal, Function<Boolean, Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.mirrorOnChange(signal, high -> runner.enqueue(taskFactory.apply(high)));
    }

    /**
     * Enqueue a task every loop while the signal is high/true.
     *
     * <p>This is best used for <b>instant</b> tasks (set a target, update a mode, etc.).
     * If the returned task takes time, repeatedly enqueuing a new instance each loop will
     * create a backlog in the runner.</p>
     */
    public void whileHigh(BooleanSource signal, Supplier<Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.whileHigh(signal, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Enqueue a task every loop while the signal is low/false.
     *
     * <p>This is best used for <b>instant</b> tasks (set a target, update a mode, etc.).
     * If the returned task takes time, repeatedly enqueuing a new instance each loop will
     * create a backlog in the runner.</p>
     */
    public void whileLow(BooleanSource signal, Supplier<Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.whileLow(signal, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Toggle: when the signal rises, enqueue {@code onEnable} if the toggle is now on,
     * otherwise enqueue {@code onDisable}.
     */
    public void toggleOnRise(BooleanSource signal, Supplier<Task> onEnable, Supplier<Task> onDisable) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(onEnable, "onEnable is required");
        Objects.requireNonNull(onDisable, "onDisable is required");

        bindings.toggleOnRise(signal, isOn -> runner.enqueue(isOn ? onEnable.get() : onDisable.get()));
    }

    /**
     * Enqueue a task when either nudge signal rises.
     *
     * <p>The factory receives the signed adjustment for the loop.</p>
     */
    public void nudgeOnRise(BooleanSource increaseSignal,
                            BooleanSource decreaseSignal,
                            double step,
                            DoubleFunction<Task> taskFactory) {
        Objects.requireNonNull(increaseSignal, "increaseSignal is required");
        Objects.requireNonNull(decreaseSignal, "decreaseSignal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.nudgeOnRise(increaseSignal, decreaseSignal, step,
                delta -> runner.enqueue(taskFactory.apply(delta)));
    }
}
