package edu.ftcphoenix.fw.core.time;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Minimal loop timing helper.
 *
 * <p>Designed to work with FTC's {@code OpMode.getRuntime()} which returns
 * seconds since the OpMode was initialized.</p>
 *
 * <h2>Responsibilities</h2>
 * <ul>
 *   <li>Track the current loop time (in seconds).</li>
 *   <li>Track the delta time ({@code dtSec}) between successive updates.</li>
 *   <li>Expose a monotonically increasing {@link #cycle()} counter for per-loop identity.</li>
 *   <li>Provide a simple {@link #reset(double)} hook to restart timing.</li>
 * </ul>
 *
 * <h2>Cycle / frame id</h2>
 * <p>{@link #cycle()} increments once per call to {@link #update(double)} after the clock has been
 * started. When robot code follows the intended pattern (calling {@link #update(double)} exactly
 * once per OpMode cycle), {@code cycle()} is a stable per-cycle identity.</p>
 *
 * <p>This is useful for making other per-cycle systems (like input edge tracking and bindings)
 * idempotent: if their update methods are accidentally called twice in the same loop cycle, the
 * second call can be a no-op when it observes the same {@code clock.cycle()}.</p>
 *
 * <h2>Typical usage</h2>
 * <pre>{@code
 * LoopClock clock = new LoopClock();
 *
 * // In init():
 * clock.reset(getRuntime());
 *
 * // In loop():
 * clock.update(getRuntime());
 * long frame = clock.cycle();
 * double dt = clock.dtSec();
 * }</pre>
 */
public final class LoopClock {

    /**
     * Last recorded time (seconds).
     */
    private double lastSec;

    /**
     * Current time (seconds).
     */
    private double nowSec;

    /**
     * Delta time between last and current update (seconds).
     */
    private double dtSec;

    /**
     * Monotonically increasing loop cycle counter.
     *
     * <p>Intended to increment once per OpMode cycle (i.e., once per call to {@link #update(double)}
     * when used correctly).</p>
     */
    private long cycle;

    /**
     * Whether {@link #update(double)} or {@link #reset(double)} has been called at least once.
     */
    private boolean started;

    /**
     * Create a new clock with all times initialized to zero.
     *
     * <p>Call {@link #reset(double)} or {@link #update(double)} before using
     * {@link #dtSec()} or {@link #nowSec()} in a meaningful way.</p>
     */
    public LoopClock() {
        this.lastSec = 0.0;
        this.nowSec = 0.0;
        this.dtSec = 0.0;
        this.cycle = 0L;
        this.started = false;
    }

    /**
     * Reset the clock to a given absolute time (seconds).
     *
     * <p>After reset, {@link #dtSec()} will return 0 until the next update.</p>
     *
     * <p>Reset also clears the cycle counter to 0. The next call to {@link #update(double)}
     * will advance the cycle counter.</p>
     *
     * @param currentTimeSec current absolute time in seconds (e.g., from getRuntime())
     */
    public void reset(double currentTimeSec) {
        this.lastSec = currentTimeSec;
        this.nowSec = currentTimeSec;
        this.dtSec = 0.0;

        this.cycle = 0L;
        this.started = true;
    }

    /**
     * Update the clock with the latest absolute time (seconds).
     *
     * <p>If this is the first call after construction (and {@link #reset(double)}
     * has not been called), the clock will initialize its internal state and
     * {@link #dtSec()} will be 0.0 for this first update.</p>
     *
     * <p>On each update, {@link #cycle()} is incremented.</p>
     *
     * @param currentTimeSec current absolute time in seconds (e.g., from getRuntime())
     */
    public void update(double currentTimeSec) {
        if (!started) {
            // First update after construction: initialize and report dt=0 for this first update.
            reset(currentTimeSec);
            // Still count this as a loop cycle.
            cycle++;
            return;
        }

        this.nowSec = currentTimeSec;
        this.dtSec = nowSec - lastSec;

        // Guard against negative dt due to clock resets.
        if (dtSec < 0.0) {
            dtSec = 0.0;
        }

        this.lastSec = nowSec;
        this.cycle++;
    }

    /**
     * @return current absolute time in seconds at last update.
     */
    public double nowSec() {
        return nowSec;
    }

    /**
     * @return delta time in seconds between the last two updates.
     *
     * <p>Will be 0.0 on the first update after {@link #reset(double)} or after
     * construction.</p>
     */
    public double dtSec() {
        return dtSec;
    }

    /**
     * @return monotonically increasing loop cycle counter.
     *
     * <p>When {@link #update(double)} is called once per OpMode cycle, this value uniquely
     * identifies the current loop cycle and can be used as a per-cycle id (frame id).</p>
     */
    public long cycle() {
        return cycle;
    }

    /**
     * Emit basic timing info for debugging.
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. "clock" or "robot.clock"
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "clock" : prefix;
        dbg.addData(p + ".cycle", cycle)
                .addData(p + ".nowSec", nowSec)
                .addData(p + ".lastSec", lastSec)
                .addData(p + ".dtSec", dtSec)
                .addData(p + ".started", started);
    }
}
