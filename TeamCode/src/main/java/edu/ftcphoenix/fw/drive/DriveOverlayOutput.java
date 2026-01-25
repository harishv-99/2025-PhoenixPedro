package edu.ftcphoenix.fw.drive;

import java.util.Objects;

/**
 * Output of a {@link DriveOverlay} for a single loop.
 *
 * <p>An overlay produces:</p>
 * <ul>
 *   <li>a {@link DriveSignal}, and</li>
 *   <li>a {@link DriveOverlayMask} telling the framework which components of that signal
 *       should override the underlying (base) drive command.</li>
 * </ul>
 *
 * <p>This explicit mask makes it easy for overlays to “gracefully give up control” on any loop
 * where they do not have enough information (e.g., a vision target is not visible). In that case,
 * the overlay can return {@link DriveOverlayMask#NONE} and the base command passes through unchanged.</p>
 */
public final class DriveOverlayOutput {

    private static final DriveOverlayOutput ZERO = new DriveOverlayOutput(DriveSignal.zero(), DriveOverlayMask.NONE);

    /**
     * Overlay drive command (robot-centric).
     */
    public final DriveSignal signal;

    /**
     * Which components of {@link #signal} should override the base command.
     */
    public final DriveOverlayMask mask;

    /**
     * Create a new output.
     *
     * @param signal overlay drive signal (non-null)
     * @param mask   overlay mask (non-null)
     */
    public DriveOverlayOutput(DriveSignal signal, DriveOverlayMask mask) {
        this.signal = Objects.requireNonNull(signal, "signal");
        this.mask = Objects.requireNonNull(mask, "mask");
    }

    /**
     * Convenience: output with {@link DriveSignal#zero()} and {@link DriveOverlayMask#NONE}.
     */
    public static DriveOverlayOutput zero() {
        return ZERO;
    }

    /**
     * Return a copy of this output with the signal clamped to [-1, +1].
     */
    public DriveOverlayOutput clamped() {
        return new DriveOverlayOutput(signal.clamped(), mask);
    }

    @Override
    public String toString() {
        return "DriveOverlayOutput{" +
                "signal=" + signal +
                ", mask=" + mask +
                '}';
    }
}
