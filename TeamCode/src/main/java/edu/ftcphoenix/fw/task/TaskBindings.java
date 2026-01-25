package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.input.Button;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Convenience wrapper that binds {@link Button} events to {@link TaskRunner} enqueues.
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
 * tb.onPress(gamepads.p2().y(), shooter::instantStartShooter);
 * tb.onToggle(gamepads.p2().rightBumper(), shooter::instantStartShooter, shooter::instantStopShooter);
 * tb.onPressAndRelease(gamepads.p2().b(),
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
     * Enqueue a task on the rising edge of the button.
     */
    public void onPress(Button button, Supplier<Task> taskFactory) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.onPress(button, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Enqueue a task on the falling edge of the button.
     */
    public void onRelease(Button button, Supplier<Task> taskFactory) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.onRelease(button, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Common pattern: run one task once when the button is pressed, and a second
     * task once when it is released.
     */
    public void onPressAndRelease(Button button, Supplier<Task> onPress, Supplier<Task> onRelease) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(onPress, "onPress is required");
        Objects.requireNonNull(onRelease, "onRelease is required");
        bindings.onPressAndRelease(
                button,
                () -> runner.enqueue(onPress.get()),
                () -> runner.enqueue(onRelease.get())
        );
    }

    /**
     * Enqueue a task every loop while the button is held.
     *
     * <p>This is best used for <b>instant</b> tasks (set a target, update a mode, etc.).
     * If the returned task takes time, repeatedly enqueuing a new instance each loop will
     * create a backlog in the runner.</p>
     */
    public void whileHeld(Button button, Supplier<Task> taskFactory) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.whileHeld(button, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Toggle: when the button is pressed, enqueue {@code onEnable} if the toggle is now on,
     * otherwise enqueue {@code onDisable}.
     */
    public void onToggle(Button button, Supplier<Task> onEnable, Supplier<Task> onDisable) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(onEnable, "onEnable is required");
        Objects.requireNonNull(onDisable, "onDisable is required");

        bindings.onToggle(button, isOn -> runner.enqueue(isOn ? onEnable.get() : onDisable.get()));
    }
}
