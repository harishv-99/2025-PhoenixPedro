package edu.ftcsushi.fw.ftc.vision;

import java.util.Objects;

/**
 * Explicit ownership transfer for a deferred AprilTag diagnostic camera. Consumers borrow
 * {@link #aprilTags()}; only the composing tool or robot retains and closes this handle.
 */
public final class OwnedAprilTagCamera implements AutoCloseable {
    private final AutoCloseable camera;
    private final AprilTagVision aprilTags;
    private boolean closeAttempted;

    /** Advanced adapter seam transferring one camera lifetime and its borrowed tag capability. */
    public OwnedAprilTagCamera(AutoCloseable camera, AprilTagVision aprilTags) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.aprilTags = Objects.requireNonNull(aprilTags, "aprilTags");
    }

    /** Returns the stable borrowed capability without transferring camera ownership. */
    public AprilTagVision aprilTags() { return aprilTags; }

    /** Attempts camera cleanup once; a failure leaves replacement forbidden in this OpMode. */
    @Override public void close() {
        if (closeAttempted) return;
        closeAttempted = true;
        try { camera.close(); }
        catch (RuntimeException failure) { throw failure; }
        catch (Exception failure) { throw new IllegalStateException("Camera cleanup failed", failure); }
    }
}
