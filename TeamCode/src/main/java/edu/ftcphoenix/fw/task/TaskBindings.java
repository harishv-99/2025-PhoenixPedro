package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.binding.BindingRegistrar;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Convenience wrapper that binds {@link BooleanSource} events to {@link TaskRunner} enqueues.
 *
 * <p>This is a thin adapter around a {@link BindingRegistrar} and {@link TaskRunner}. It exists
 * purely to make robot code more readable when most bindings just "enqueue a task". Passing a
 * {@link Bindings.ControlContext} applies that context's activation policy before a task factory is
 * invoked; context deactivation does not cancel work already accepted by the shared runner.</p>
 *
 * <p>Each method immediately adds one declaration to the supplied registrar. {@code TaskBindings}
 * creates no separate dispatch phase or priority. When the registrar is a {@link Bindings} root or
 * {@link Bindings.ControlContext}, accepted factories are invoked in the parent's declaration
 * order among ordinary root/context registrations, and their Tasks are enqueued in that order.
 * The runner's FIFO execution is a separate responsibility. If several steps form one dependent
 * behavior, compose one fresh Task graph instead of treating binding order as Task priority or
 * cancellation policy.</p>
 *
 * <p>This adapter intentionally exposes event and change bindings, not level-triggered Task
 * enqueueing. Use {@link Bindings#whileHigh(BooleanSource, Runnable)} or
 * {@link Bindings#whileLow(BooleanSource, Runnable)} for a direct, synchronous, idempotent action
 * on each matching loop. Use {@link OutputTaskRunner#whileHigh} or
 * {@link OutputTaskRunner#whileLow} when held intent should maintain a bounded queue of fresh
 * output Tasks and cancel that queue at the opposite level. A subsystem or service that needs one
 * Task to own one complete held interval should own its private runner, enqueue once on the
 * request edge, and call {@link TaskRunner#cancelAndClear()} on release.</p>
 *
 * <h2>Usage</h2>
 *
 * <pre>{@code
 * protected void configure(RobotProgram program) {
 *     TaskBindings tasks = program.taskBindings();
 *
 *     tasks.onRise(gamepads.p2().y(), shooter::instantStartShooter);
 *     tasks.toggleOnRise(
 *             gamepads.p2().rightBumper(),
 *             shooter::instantStartShooter,
 *             shooter::instantStopShooter);
 * }}</pre>
 *
 * <p>Ordinary FTC robot code gets this adapter from its framework-created
 * {@link edu.ftcphoenix.fw.ftc.RobotProgram}; that program owns the shared binding and runner
 * heartbeats. {@link #of(BindingRegistrar, TaskRunner)} remains the explicit construction seam for
 * framework tools, calibration hosts, tests, and custom lifecycle integrations.</p>
 *
 * <p><b>Important:</b> Tasks are single-use. For that reason, this API takes {@link Supplier}
 * factories; each invocation must return a new Task instance for that signal event.</p>
 */
public final class TaskBindings {

    private final BindingRegistrar bindings;
    private final TaskRunner runner;

    private TaskBindings(BindingRegistrar bindings, TaskRunner runner) {
        this.bindings = Objects.requireNonNull(bindings, "bindings is required");
        this.runner = Objects.requireNonNull(runner, "runner is required");
    }

    /**
     * Create a {@link TaskBindings} wrapper.
     */
    public static TaskBindings of(BindingRegistrar bindings, TaskRunner runner) {
        return new TaskBindings(bindings, runner);
    }

    /**
     * Enqueue a task when the registrar accepts a signal rise (false to true).
     */
    public void onRise(BooleanSource signal, Supplier<Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.onRise(signal, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Enqueue a task when the registrar accepts a signal fall (true to false).
     */
    public void onFall(BooleanSource signal, Supplier<Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.onFall(signal, () -> runner.enqueue(taskFactory.get()));
    }

    /**
     * Enqueue a task that mirrors the registrar's effective value on its first output and whenever
     * that output changes.
     *
     * <p>With root {@link Bindings}, the factory first receives the signal's current high/low value.
     * With a {@link Bindings.ControlContext}, it receives the context's neutral or accepted output:
     * initial inactivity/activation and deactivation may therefore publish {@code false} without
     * sampling or mirroring a currently held high signal.</p>
     */
    public void mirrorOnChange(BooleanSource signal, Function<Boolean, Task> taskFactory) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(taskFactory, "taskFactory is required");
        bindings.mirrorOnChange(signal, high -> runner.enqueue(taskFactory.apply(high)));
    }

    /**
     * Toggle: when the registrar accepts a signal rise, enqueue {@code onEnable} if the toggle is now on,
     * otherwise enqueue {@code onDisable}.
     */
    public void toggleOnRise(BooleanSource signal, Supplier<Task> onEnable, Supplier<Task> onDisable) {
        Objects.requireNonNull(signal, "signal is required");
        Objects.requireNonNull(onEnable, "onEnable is required");
        Objects.requireNonNull(onDisable, "onDisable is required");

        bindings.toggleOnRise(signal, isOn -> runner.enqueue(isOn ? onEnable.get() : onDisable.get()));
    }

    /**
     * Enqueue a task when either nudge signal has an accepted rise.
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
