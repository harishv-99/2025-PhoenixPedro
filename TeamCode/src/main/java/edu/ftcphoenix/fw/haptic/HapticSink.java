package edu.ftcphoenix.fw.haptic;

/**
 * Command-only output for short, fixed-strength controller haptic pulses.
 *
 * <p>A sink represents one recipient selected by the composition root. Robot controls or a
 * dedicated robot-owned driver-feedback owner decides what a cue means, which recipient should
 * feel it, and whether that cue needs an edge, debounce, or repeat cooldown. A supervisor may
 * supply the status or policy that owner consumes; the sink only submits the requested pulse.</p>
 *
 * <p>Requests are non-blocking. A normally returning call means only that the implementation
 * accepted the command locally; it does not prove that a controller supports haptics, received the
 * command, produced the requested strength, or completed the pulse. A later pulse may replace one
 * already playing, as documented by the concrete boundary. FTC composition roots normally obtain
 * this sink with {@code FtcHaptics.gamepad(gamepad1)}.</p>
 *
 * <h2>Typical robot-controls usage</h2>
 *
 * <pre>{@code
 * bindings.onRise(
 *         intakeFull,
 *         () -> driverHaptics.pulse(1.0, 0.50)
 * );
 * }</pre>
 */
public interface HapticSink {

    /**
     * Request one steady haptic pulse.
     *
     * <p>The pulse has one normalized strength for its complete positive duration. Implementations
     * may translate that logical strength to more than one physical vibration motor. Call this on
     * the ordinary OpMode thread and request normal cues from semantic events or edges rather than
     * repeatedly from an unchanged per-loop state.</p>
     *
     * @param strength normalized pulse strength in {@code (0.0, 1.0]}
     * @param durationSec finite positive pulse duration in seconds
     * @throws IllegalArgumentException if either argument is non-finite or outside its documented
     *                                  range, or if the implementation cannot represent the duration
     */
    void pulse(double strength, double durationSec);

    /**
     * Request that this recipient stop its current haptic output.
     *
     * <p>This method is safe to call repeatedly. It is a best-effort command, not physical stop
     * acknowledgement; concrete asynchronous boundaries may deliver the request after this method
     * returns.</p>
     */
    void stop();
}
