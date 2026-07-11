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
 *
 * <p>A positive-duration output window must remain active long enough for the downstream output
 * or Plant phase to observe its run value in at least one runner cycle. A zero-duration run window
 * stays idle; surrounding phases such as a configured cooldown may still remain active.</p>
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
     *   <li>A positive-duration run should expose its run value on the cycle when that interval begins.</li>
     * </ul>
     */
    double getOutput();
}
