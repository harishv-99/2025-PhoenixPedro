package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A secondary drive behavior that can temporarily take control of one or more
 * drive degrees-of-freedom (DOFs).
 *
 * <p>Overlays are the backbone of “driver assist” in Phoenix. They are used for behaviors like:</p>
 * <ul>
 *   <li><b>Auto-aim</b> that overrides only rotation while the driver keeps translation.</li>
 *   <li><b>Nudge / microdrive</b> that overrides translation while keeping rotation manual.</li>
 *   <li><b>Pose lock</b> that overrides both translation and rotation to resist being bumped.</li>
 * </ul>
 *
 * <h2>How an overlay is applied</h2>
 * <p>An overlay is typically applied using {@link DriveSource#overlayWhen(java.util.function.BooleanSupplier, DriveOverlay, DriveOverlayMask)}.</p>
 * The base {@link DriveSource} continues to run each loop. When enabled, the overlay produces a
 * {@link DriveOverlayOutput} containing:</p>
 * <ul>
 *   <li>a {@link DriveSignal} (robot-centric), and</li>
 *   <li>a {@link DriveOverlayMask} selecting which components should override the base command.</li>
 * </ul>
 *
 * <p>This design keeps the API consistent across very different localization setups:
 * whether the overlay uses odometry, vision observations, or a fused pose estimate, it still
 * outputs the same {@link DriveSignal}/{@link DriveOverlayMask} combination.</p>
 *
 * <h2>Lifecycle hooks</h2>
 * <p>Overlays may optionally implement {@link #onEnable(LoopClock)} and {@link #onDisable(LoopClock)}.
 * This is useful for resetting controller state or capturing a setpoint when the overlay is turned on.</p>
 */
public interface DriveOverlay {

    /**
     * Produce an overlay output for this loop.
     *
     * <p>Implementations should always return a non-null output. If the overlay is not able to
     * provide a safe command (e.g., missing localization), it should typically return
     * {@link DriveOverlayOutput#zero()} or an output with {@link DriveOverlayMask#NONE}.</p>
     */
    DriveOverlayOutput get(LoopClock clock);

    /**
     * Called once when an overlay becomes enabled.
     *
     * <p>The default implementation does nothing.</p>
     */
    default void onEnable(LoopClock clock) {
        // no-op
    }

    /**
     * Called once when an overlay becomes disabled.
     *
     * <p>The default implementation does nothing.</p>
     */
    default void onDisable(LoopClock clock) {
        // no-op
    }

    /**
     * Optional debug hook.
     *
     * <p>The default implementation does nothing. Implementations are encouraged to write compact
     * state that helps tune or diagnose the overlay (e.g. current errors, chosen feedback source).</p>
     */
    default void debugDump(DebugSink dbg, String prefix) {
        // no-op
    }
}
