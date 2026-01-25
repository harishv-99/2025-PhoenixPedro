package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.util.LoopClock;
import edu.ftcphoenix.fw.debug.DebugSink;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Source of high-level drive commands for a drivetrain.
 *
 * <p>A {@link DriveSource} takes in the current loop timing (via
 * {@link LoopClock}) and produces a {@link DriveSignal} each loop.</p>
 *
 * <p>Typical implementations include:</p>
 * <ul>
 *   <li>{@code GamepadDriveSource} – map gamepad sticks to a drive signal.</li>
 *   <li>A motion planner – follow a trajectory and emit commands.</li>
 *   <li>A closed-loop heading controller – maintain or turn to a target angle.</li>
 * </ul>
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>{@link #get(LoopClock)} is called once per loop by the OpMode.</li>
 *   <li>Implementations may be stateless (pure function of current inputs) or
 *       stateful (e.g., with internal filters, rate limiters, etc.).</li>
 *   <li>The returned {@link DriveSignal} is usually expected to be in the
 *       range [-1, +1] for each component, but callers may clamp if needed.</li>
 * </ul>
 *
 * <p>This interface also provides a few default methods for simple
 * composition, so that higher-level code can build up complex behaviors
 * by <em>wrapping</em> existing sources rather than introducing new
 * concrete classes:</p>
 *
 * <ul>
 *   <li>{@link #scaledWhen(BooleanSupplier, double)} – conditional slow mode
 *       around any drive behavior.</li>
 *   <li>{@link #blendedWith(DriveSource, double)} – blend this source with
 *       another using {@link DriveSignal#lerp(DriveSignal, double)}.</li>
 * </ul>
 */
public interface DriveSource {

    /**
     * Produce a drive signal for the current loop.
     *
     * @param clock loop timing helper; implementations may use this for
     *              dt-based smoothing or rate limiting
     * @return drive command for this loop (never null)
     */
    DriveSignal get(LoopClock clock);

    /**
     * Optional debug hook: emit a compact summary of this source's state.
     *
     * <p>The default implementation only records the implementing class name.
     * Concrete sources are encouraged to override this to include additional
     * details such as last output, configuration parameters, and any internal
     * filter/controller state.</p>
     *
     * <p>Framework classes consistently follow the pattern that if
     * {@code dbg} is {@code null}, the method simply does nothing. This
     * lets callers freely pass either a real sink or a {@code NullDebugSink}
     * (or {@code null}) without having to guard every call.</p>
     *
     * @param dbg    debug sink to write to (may be {@code null})
     * @param prefix key prefix for all entries (may be {@code null} or empty)
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "drive" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName());
    }

    /**
     * Return a new {@link DriveSource} that applies a uniform scale factor
     * to this source's output whenever {@code when} is true.
     *
     * <p>This is a convenient way to implement "slow mode" or fine control
     * that wraps any existing drive behavior (manual drive, tag aim, etc.).</p>
     *
     * <p>Example:</p>
     * <pre>{@code
     * DriveSource manual = GamepadDriveSource.teleOpMecanum(gamepads);
     * DriveSource drive  = manual.scaledWhen(
     *         () -> gamepads.p1().rightBumper().isPressed(),
     *         0.30);
     * }</pre>
     *
     * @param when  condition indicating when the scale should be applied
     * @param scale uniform scale factor to apply to all components
     * @return wrapped {@link DriveSource} that conditionally scales output
     */
    default DriveSource scaledWhen(BooleanSupplier when, double scale) {
        Objects.requireNonNull(when, "when must not be null");
        // If scale is 1.0, no need to wrap.
        if (scale == 1.0) {
            return this;
        }
        // Capture "this" in a local for use inside the lambda.
        DriveSource self = this;
        return clock -> {
            DriveSignal base = self.get(clock);
            if (!when.getAsBoolean()) {
                return base;
            }
            return base.scaled(scale);
        };
    }

    /**
     * Return a new {@link DriveSource} that blends this source with another.
     *
     * <p>The blend is performed using {@link DriveSignal#lerp(DriveSignal, double)}
     * with a fixed {@code alpha}:</p>
     *
     * <ul>
     *   <li>{@code alpha = 0} → pure output of this source.</li>
     *   <li>{@code alpha = 1} → pure output of {@code other}.</li>
     *   <li>Values in-between produce a simple linear mix.</li>
     * </ul>
     *
     * <p>This is useful for "driver-assist" behaviors, where you want to
     * mix manual control with an automatic behavior (e.g., auto-align) at
     * some fixed strength.</p>
     *
     * <p>Example:</p>
     * <pre>{@code
     * DriveSource manual   = GamepadDriveSource.teleOpMecanum(gamepads);
     * DriveSource autoAim  = TagAim.teleOpAim(manual, aimButton, tagSensor, ids);
     *
     * // 40% assist from autoAim, 60% from manual
     * DriveSource mixed = manual.blendedWith(autoAim, 0.4);
     * }</pre>
     *
     * @param other another {@link DriveSource} to blend with
     * @param alpha blend factor in [0, 1] (values outside this range are clamped)
     * @return wrapped {@link DriveSource} that blends the two sources
     */
    default DriveSource blendedWith(DriveSource other, double alpha) {
        Objects.requireNonNull(other, "other DriveSource must not be null");
        DriveSource self = this;
        return clock -> {
            DriveSignal a = self.get(clock);
            DriveSignal b = other.get(clock);
            return a.lerp(b, alpha);
        };
    }
}
