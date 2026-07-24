package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Solved planar facing relationship from one spatial solve lane.
 *
 * <p>{@link #facingErrorRad} is the signed rotation needed for the sampled facing frame's +X axis
 * to face the requested target. Positive values are CCW / left turns. The solution also carries
 * quality and an epoch-safe measurement timestamp so consumers can gate stale or low-confidence
 * results consistently.</p>
 */
public final class FacingSolution {

    /**
     * Signed heading error in radians; positive is CCW / left.
     */
    public final double facingErrorRad;

    /**
     * Simple confidence score in [0, 1], interpreted by the lane that produced the result.
     */
    public final double quality;

    /** Epoch-safe timestamp of the underlying measurement/result. */
    public final LoopTimestamp timestamp;

    /**
     * Creates a facing solution.
     *
     * @param facingErrorRad signed heading error in radians; positive is CCW / left
     * @param quality        lane-specific confidence score in [0, 1]
     * @param timestamp      epoch-safe measurement/result timestamp; use
     *                       {@link LoopTimestamp#unavailable()} only when no truthful time exists
     */
    public FacingSolution(double facingErrorRad, double quality, LoopTimestamp timestamp) {
        this.facingErrorRad = facingErrorRad;
        this.quality = quality;
        this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
    }

    @Override
    public String toString() {
        return "FacingSolution{facingErrorRad=" + facingErrorRad
                + ", quality=" + quality
                + ", timestamp=" + timestamp + '}';
    }
}
