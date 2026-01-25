package edu.ftcphoenix.fw.util;

import edu.ftcphoenix.fw.debug.DebugSink;

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
 *   <li>Provide a simple {@link #reset(double)} hook to restart timing.</li>
 * </ul>
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
     * Whether {@link #update(double)} has been called at least once since reset.
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
        this.started = false;
    }

    /**
     * Reset the clock to a given absolute time (seconds).
     *
     * <p>After reset, {@link #dtSec()} will return 0 until the next update.</p>
     *
     * @param currentTimeSec current absolute time in seconds (e.g., from getRuntime())
     */
    public void reset(double currentTimeSec) {
        this.lastSec = currentTimeSec;
        this.nowSec = currentTimeSec;
        this.dtSec = 0.0;
        this.started = true;
    }

    /**
     * Update the clock with the latest absolute time (seconds).
     *
     * <p>If this is the first call after construction (and {@link #reset(double)}
     * has not been called), the clock will initialize its internal state and
     * {@link #dtSec()} will be 0.0 for this first update.</p>
     *
     * @param currentTimeSec current absolute time in seconds (e.g., from getRuntime())
     */
    public void update(double currentTimeSec) {
        if (!started) {
            // First update after construction: initialize and report dt=0.
            reset(currentTimeSec);
            return;
        }

        this.nowSec = currentTimeSec;
        this.dtSec = nowSec - lastSec;

        // Guard against negative or absurd dt due to clock resets.
        if (dtSec < 0.0) {
            dtSec = 0.0;
        }

        this.lastSec = nowSec;
    }

    /**
     * @return time in seconds at last update.
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
        dbg.addData(p + ".nowSec", nowSec)
                .addData(p + ".lastSec", lastSec)
                .addData(p + ".dtSec", dtSec)
                .addData(p + ".started", started);
    }
}
