package edu.ftcphoenix.fw.task;

import java.util.Objects;
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
 * tb.onToggle(gamepads.p2().rightBumper(), shooter::instantStartShooter, shooter::instantStopShooter);
 * tb.onRiseAndFall(gamepads.p2().b(),
 *         () -> shooter.instantStartTransfer(Shooter.TransferDirection.FORWARD),
 *         shooter::instantStopTransfer);
 * }</pre>
 *
 * <p><b>Important:</b> Tasks are generally single-use. For that reason, this API
 * takes {@link Supplier} factories so each button event can create a fresh {@link Task}.</p>
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
     * Common pattern: enqueue one task on rise and another on fall.
     */
    public void onRiseAndFall(BooleanSource signal, Supplier<Task> onRise, Supplier<Task> onFall) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(onRise, "onRise is required");
        Objects.requireNonNull(onFall, "onFall is required");
        bindings.onRiseAndFall(
                signal,
                () -> runner.enqueue(onRise.get()),
                () -> runner.enqueue(onFall.get())
        );
    }

    /**
     * Enqueue a task every loop while the signal is true.
     *
     * <p>This is best used for <b>instant</b> tasks (set a target, update a mode, etc.).
     * If the returned task takes time, repeatedly enqueuing a new instance each loop will
     * create a backlog in the runner.</p>
     */
    public void whileTrue(BooleanSource signal, Supplier<Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.whileTrue(signal, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Toggle: when the button is pressed, enqueue {@code onEnable} if the toggle is now on,
     * otherwise enqueue {@code onDisable}.
     */
    public void onToggle(BooleanSource button, Supplier<Task> onEnable, Supplier<Task> onDisable) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(onEnable, "onEnable is required");
        Objects.requireNonNull(onDisable, "onDisable is required");

        bindings.onToggle(button, isOn -> runner.enqueue(isOn ? onEnable.get() : onDisable.get()));
    }
}