package edu.ftcphoenix.fw.drive;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Small factory helpers for creating common {@link DriveOverlay} implementations.
 */
public final class DriveOverlays {

    private DriveOverlays() {
        // Utility class.
    }

    /**
     * Adapt a plain {@link DriveSource} into a {@link DriveOverlay} with a fixed mask.
     *
     * <p>This is useful when you already have a drive source (e.g., a Gamepad drive source using
     * dpad axes) and you want to use it as an overlay via {@link DriveSource#overlayWhen}.</p>
     */
    public static DriveOverlay fromDriveSource(final DriveSource source, final DriveOverlayMask mask) {
        Objects.requireNonNull(source, "source");
        Objects.requireNonNull(mask, "mask");

        return new DriveOverlay() {
            @Override
            public DriveOverlayOutput get(LoopClock clock) {
                return new DriveOverlayOutput(source.get(clock), mask);
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "driveOverlay" : prefix;
                dbg.addData(p + ".class", "DriveOverlays.fromDriveSource");
                dbg.addData(p + ".mask", mask.toString());
                source.debugDump(dbg, p + ".source");
            }
        };
    }

    /**
     * Create an overlay that outputs a fixed command and mask.
     */
    public static DriveOverlay fixed(final DriveSignal signal, final DriveOverlayMask mask) {
        Objects.requireNonNull(signal, "signal");
        Objects.requireNonNull(mask, "mask");
        return clock -> new DriveOverlayOutput(signal, mask);
    }
}
