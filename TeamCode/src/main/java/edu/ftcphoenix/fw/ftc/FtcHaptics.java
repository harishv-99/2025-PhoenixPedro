package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.Gamepad;

import edu.ftcphoenix.fw.haptic.HapticSink;

/**
 * FTC-boundary factory for controller haptic output.
 *
 * <p>The returned {@link HapticSink} keeps FTC SDK types and milliseconds out of robot controls.
 * It submits commands asynchronously through the supplied {@link Gamepad}. The SDK's one-element
 * evicting queue retains only the latest undelivered request. When the FTC event loop forwards
 * that request to the Driver Station, it displaces an effect already playing.</p>
 *
 * <p>FTC does not expose a reliable controller-haptic capability or delivery acknowledgement.
 * Normal return therefore proves only that this adapter queued an SDK request. Physical support,
 * strength, delivery, completion, and stop latency remain FTC runtime, Driver Station, and
 * controller behavior.</p>
 *
 * <p>One logical strength is sent to both FTC rumble channels. The SDK quantizes each channel to
 * 255 nonzero command levels; this adapter rounds a valid positive strength below the first level
 * up to that level so a logical pulse does not become a known zero-power SDK effect. Every
 * positive sub-millisecond duration rounds up to one millisecond, while a duration beyond the
 * SDK's finite integer-millisecond range is rejected before any request is queued.</p>
 *
 * <h2>Typical composition-root usage</h2>
 *
 * <pre>{@code
 * HapticSink driverHaptics = FtcHaptics.gamepad(gamepad1);
 * HapticSink operatorHaptics = FtcHaptics.gamepad(gamepad2);
 *
 * MyTeleOpControls controls = new MyTeleOpControls(
 *         gamepads,
 *         driverHaptics,
 *         operatorHaptics
 * );
 *
 * // During total robot cleanup, attempt both even if one SDK call fails.
 * CleanupActions.attemptAll(
 *         driverHaptics::stop,
 *         operatorHaptics::stop
 * );
 * }</pre>
 *
 * <p>{@code CleanupActions} above is
 * {@link edu.ftcphoenix.fw.core.lifecycle.CleanupActions}; the composition root still chooses
 * which recipients it owns and the safe cleanup order.</p>
 */
public final class FtcHaptics {

    private static final double MILLISECONDS_PER_SECOND = 1000.0;
    private static final double MAX_DURATION_SEC = Integer.MAX_VALUE / MILLISECONDS_PER_SECOND;
    private static final double MIN_NONZERO_FTC_STRENGTH = 1.0 / 255.0;

    private FtcHaptics() {
        // utility class
    }

    /**
     * Create the haptic sink for one FTC gamepad recipient.
     *
     * <p>Construct one sink for each controller that should receive feedback, retain it in the
     * composition root, and pass only the device-neutral sink to robot controls or a dedicated
     * driver-feedback owner. The returned adapter owns no heartbeat, timer, retry, or background
     * thread.</p>
     *
     * @param gamepad FTC gamepad that should receive the haptic requests
     * @return device-neutral command sink for that recipient
     * @throws IllegalArgumentException if {@code gamepad} is {@code null}
     */
    public static HapticSink gamepad(Gamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("gamepad is required");
        }
        return new FtcGamepadHapticSink(gamepad);
    }

    /**
     * Private adapter so {@link #gamepad(Gamepad)} remains the sole public construction path.
     */
    private static final class FtcGamepadHapticSink implements HapticSink {
        private final Gamepad gamepad;

        private FtcGamepadHapticSink(Gamepad gamepad) {
            this.gamepad = gamepad;
        }

        @Override
        public void pulse(double strength, double durationSec) {
            if (!Double.isFinite(strength) || strength <= 0.0 || strength > 1.0) {
                throw new IllegalArgumentException(
                        "strength must be finite and in (0.0, 1.0]; got " + strength);
            }
            if (!Double.isFinite(durationSec) || durationSec <= 0.0) {
                throw new IllegalArgumentException(
                        "durationSec must be finite and > 0.0; got " + durationSec);
            }
            if (durationSec > MAX_DURATION_SEC) {
                throw new IllegalArgumentException(
                        "durationSec must be <= " + MAX_DURATION_SEC
                                + " so FTC milliseconds fit in an int; got " + durationSec);
            }

            double ftcStrength = Math.max(strength, MIN_NONZERO_FTC_STRENGTH);
            int durationMs = (int) Math.ceil(durationSec * MILLISECONDS_PER_SECOND);
            gamepad.rumble(ftcStrength, ftcStrength, durationMs);
        }

        @Override
        public void stop() {
            gamepad.stopRumble();
        }
    }
}
