package edu.ftcphoenix.fw.spatial;

/**
 * Solved planar facing relationship from one spatial solve lane.
 *
 * <p>{@link #facingErrorRad} is the signed rotation needed for the sampled facing frame's +X axis
 * to face the requested target. Positive values are CCW / left turns. The solution also carries
 * quality and timing metadata so consumers can gate stale or low-confidence results consistently.</p>
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

    /**
     * Age of the underlying measurement/result in seconds at solve time.
     */
    public final double ageSec;

    /**
     * Timestamp of the underlying measurement/result in the LoopClock timebase, or NaN if unknown.
     */
    public final double timestampSec;

    /**
     * Creates a facing solution with no explicit timestamp.
     */
    public FacingSolution(double facingErrorRad, double quality, double ageSec) {
        this(facingErrorRad, quality, ageSec, Double.NaN);
    }

    /**
     * Creates a facing solution.
     *
     * @param facingErrorRad signed heading error in radians; positive is CCW / left
     * @param quality        lane-specific confidence score in [0, 1]
     * @param ageSec         age of the measurement/result at solve time
     * @param timestampSec   measurement/result timestamp in the LoopClock timebase, or NaN
     */
    public FacingSolution(double facingErrorRad, double quality, double ageSec, double timestampSec) {
        this.facingErrorRad = facingErrorRad;
        this.quality = quality;
        this.ageSec = ageSec;
        this.timestampSec = timestampSec;
    }

    @Override
    public String toString() {
        return "FacingSolution{facingErrorRad=" + facingErrorRad
                + ", quality=" + quality
                + ", ageSec=" + ageSec
                + ", timestampSec=" + timestampSec + '}';
    }
}
