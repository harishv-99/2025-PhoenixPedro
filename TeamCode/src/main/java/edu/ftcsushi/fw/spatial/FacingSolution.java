package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopTimestamp;

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
     * Lane-specific score in [0, 1], or NaN when none is supplied; not pickup probability.
     */
    public final double quality;

    /** Oldest required live evidence; a committed goal does not pretend to be a new sighting. */
    public final LoopTimestamp timestamp;
    /** Pose evidence used for a field solve; unavailable for direct observed-point feedback. */
    public final LoopTimestamp robotPoseTimestamp;
    /** Original target sighting, or unavailable for an authored fixed target. */
    public final LoopTimestamp targetObservationTimestamp;
    /** Whether target observation age, rather than a bounded commitment, remains a live constraint. */
    public final boolean liveTarget;

    /**
     * Creates a facing solution.
     *
     * @param facingErrorRad signed heading error in radians; positive is CCW / left
     * @param quality        lane-specific score in [0, 1], or NaN when unknown
     * @param timestamp      epoch-safe measurement/result timestamp; use
     *                       {@link LoopTimestamp#unavailable()} only when no truthful time exists
     */
    public FacingSolution(double facingErrorRad, double quality, LoopTimestamp timestamp) {
        this(facingErrorRad, quality, timestamp, timestamp, LoopTimestamp.unavailable(), false);
    }

    private FacingSolution(double facingErrorRad, double quality, LoopTimestamp timestamp,
                           LoopTimestamp robotPoseTimestamp, LoopTimestamp targetObservationTimestamp,
                           boolean liveTarget) {
        this.facingErrorRad = facingErrorRad;
        this.quality = quality;
        this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
        this.robotPoseTimestamp = robotPoseTimestamp;
        this.targetObservationTimestamp = targetObservationTimestamp;
        this.liveTarget = liveTarget;
    }

    /** Preserves target provenance and uses the older live evidence for ordinary age gates. */
    FacingSolution withTargetEvidence(LoopTimestamp observation, boolean live) {
        LoopTimestamp effective = timestamp;
        double difference = timestamp.secondsSince(observation);
        if (live && Double.isFinite(difference) && difference > 0.0) effective = observation;
        return new FacingSolution(facingErrorRad, quality, effective, robotPoseTimestamp, observation, live);
    }

    /** A direct observation solves in its capture frame without a field-pose estimate. */
    FacingSolution withDirectObservationEvidence(LoopTimestamp observation) {
        return new FacingSolution(facingErrorRad, quality, observation,
                LoopTimestamp.unavailable(), observation, true);
    }

    @Override
    public String toString() {
        return "FacingSolution{facingErrorRad=" + facingErrorRad
                + ", quality=" + quality
                + ", timestamp=" + timestamp + '}';
    }
}
