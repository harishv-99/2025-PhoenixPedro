package edu.ftcphoenix.fw.task;

/**
 * A {@link Task} that produces a scalar output value while it runs.
 *
 * <p>This is Phoenix's principled replacement for "action controllers" that directly write hardware.
 * Instead of writing a plant directly inside a task, an {@code OutputTask} exposes an
 * output value (typically a Plant target such as power, velocity, or position) and lets your
 * subsystem decide how to apply it.
 *
 * <h2>Why this exists</h2>
 * <ul>
 *   <li><b>Reuse across TeleOp and Auto</b>: the same sensor gates and timing logic can drive an
 *       output queue in both modes.</li>
 *   <li><b>Single-writer safety</b>: only one place in your code applies the final target to a Plant
 *       (your subsystem loop). Tasks just propose outputs.</li>
 *   <li><b>Composability</b>: you can combine a continuous "base" target (from Sources) with a
 *       queued "override" target (from an {@link OutputTaskRunner}) using a simple priority rule.</li>
 * </ul>
 *
 * <p>Use {@link OutputTaskRunner} to run {@code OutputTask}s sequentially (FIFO) and expose their
 * current output as a {@link edu.ftcphoenix.fw.core.source.ScalarSource}.</p>
 */
public interface OutputTask extends Task {

    /**
     * The output value for this loop.
     *
     * <p>Contract:
     * <ul>
     *   <li>Must be safe to call multiple times per loop.</li>
     *   <li>Should reflect the most recent {@link #update(edu.ftcphoenix.fw.core.time.LoopClock)} call.</li>
     *   <li>Should return a sensible value even while waiting (for example, 0 power while gated).</li>
     * </ul>
     */
    double getOutput();
}
