package edu.ftcphoenix.fw.core.control;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * A small, reusable hysteresis latch (a.k.a. Schmitt trigger) for turning a noisy
 * continuous signal into a stable boolean.
 *
 * <p>Why this exists: many robot decisions should not "chatter" around a threshold.
 * For example:</p>
 *
 * <ul>
 *   <li>"Enable auto-aim only when the driver stick is idle."</li>
 *   <li>"Consider ourselves in the shooting zone only after we are clearly inside it,
 *       and don't exit until we are clearly outside it."</li>
 *   <li>"Treat a sensor as valid only after its quality is above a threshold, and keep it
 *       valid until it drops below a lower threshold."</li>
 * </ul>
 *
 * <p>This class keeps a boolean state and updates it using two thresholds:
 * an <b>enter</b> threshold and an <b>exit</b> threshold.</p>
 *
 * <h2>Modes</h2>
 *
 * <ul>
 *   <li><b>On when below, off when above</b>: turns ON when {@code value <= enterThreshold},
 *       turns OFF when {@code value >= exitThreshold}. (Example: "stick is idle".)</li>
 *   <li><b>On when above, off when below</b>: turns ON when {@code value >= enterThreshold},
 *       turns OFF when {@code value <= exitThreshold}. (Example: "sensor quality is good".)</li>
 * </ul>
 *
 * <p>If {@code value} is not finite (NaN/inf), {@link #update(double)} leaves the state unchanged.</p>
 */
public final class HysteresisLatch {

    /**
     * Hysteresis mode.
     */
    public enum Mode {
        /**
         * Turn ON when value is <= enter threshold, and OFF when value is >= exit threshold.
         */
        ON_WHEN_BELOW_OFF_WHEN_ABOVE,

        /**
         * Turn ON when value is >= enter threshold, and OFF when value is <= exit threshold.
         */
        ON_WHEN_ABOVE_OFF_WHEN_BELOW
    }

    private final Mode mode;
    private final double enterThreshold;
    private final double exitThreshold;

    private boolean state;

    private HysteresisLatch(Mode mode, double enterThreshold, double exitThreshold, boolean initialState) {
        if (mode == null) {
            throw new IllegalArgumentException("mode is required");
        }
        if (!Double.isFinite(enterThreshold) || !Double.isFinite(exitThreshold)) {
            throw new IllegalArgumentException("thresholds must be finite");
        }

        // Basic sanity checks to catch inverted thresholds early.
        if (mode == Mode.ON_WHEN_BELOW_OFF_WHEN_ABOVE) {
            if (enterThreshold > exitThreshold) {
                throw new IllegalArgumentException("enterThreshold must be <= exitThreshold for ON_WHEN_BELOW_OFF_WHEN_ABOVE");
            }
        } else {
            if (enterThreshold < exitThreshold) {
                throw new IllegalArgumentException("enterThreshold must be >= exitThreshold for ON_WHEN_ABOVE_OFF_WHEN_BELOW");
            }
        }

        this.mode = mode;
        this.enterThreshold = enterThreshold;
        this.exitThreshold = exitThreshold;
        this.state = initialState;
    }

    /**
     * Create a latch that turns ON when {@code value <= enterThreshold} and turns OFF when
     * {@code value >= exitThreshold}.
     *
     * <p>This is commonly used for "idle" / "near zero" checks.</p>
     */
    public static HysteresisLatch onWhenBelowOffWhenAbove(double enterThreshold, double exitThreshold) {
        return new HysteresisLatch(Mode.ON_WHEN_BELOW_OFF_WHEN_ABOVE, enterThreshold, exitThreshold, false);
    }

    /**
     * Create a latch that turns ON when {@code value >= enterThreshold} and turns OFF when
     * {@code value <= exitThreshold}.
     *
     * <p>This is commonly used for "valid when high" checks (quality, voltage, etc.).</p>
     */
    public static HysteresisLatch onWhenAboveOffWhenBelow(double enterThreshold, double exitThreshold) {
        return new HysteresisLatch(Mode.ON_WHEN_ABOVE_OFF_WHEN_BELOW, enterThreshold, exitThreshold, false);
    }

    /**
     * Update the latch state using the provided {@code value}.
     *
     * @param value latest value to evaluate
     * @return the updated state
     */
    public boolean update(double value) {
        if (!Double.isFinite(value)) {
            return state; // no update
        }

        switch (mode) {
            case ON_WHEN_BELOW_OFF_WHEN_ABOVE:
                if (!state) {
                    if (value <= enterThreshold) {
                        state = true;
                    }
                } else {
                    if (value >= exitThreshold) {
                        state = false;
                    }
                }
                break;

            case ON_WHEN_ABOVE_OFF_WHEN_BELOW:
                if (!state) {
                    if (value >= enterThreshold) {
                        state = true;
                    }
                } else {
                    if (value <= exitThreshold) {
                        state = false;
                    }
                }
                break;

            default:
                // unreachable
                break;
        }

        return state;
    }

    /**
     * @return current latched state
     */
    public boolean get() {
        return state;
    }

    /**
     * Set/clear the latch explicitly.
     *
     * <p>This is useful when changing modes (e.g., when a driver releases a button and you want to
     * reset a derived state).</p>
     */
    public void reset(boolean state) {
        this.state = state;
    }

    /**
     * @return latch mode
     */
    public Mode mode() {
        return mode;
    }

    /**
     * @return enter threshold used by this latch
     */
    public double enterThreshold() {
        return enterThreshold;
    }

    /**
     * @return exit threshold used by this latch
     */
    public double exitThreshold() {
        return exitThreshold;
    }

    /**
     * Optional debug helper.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "hys" : prefix;
        dbg.addLine(p + ": HysteresisLatch")
                .addData(p + ".mode", mode.toString())
                .addData(p + ".enter", enterThreshold)
                .addData(p + ".exit", exitThreshold)
                .addData(p + ".state", state);
    }
}
