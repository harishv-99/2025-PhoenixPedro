package edu.ftcphoenix.fw.localization.fusion;

/**
 * Immutable debug/status snapshot shared by odometry + vision localizers.
 *
 * <p>This intentionally captures only the common, high-level behavior that testers and robot code
 * may want regardless of whether the underlying estimator is the lightweight complementary fusion
 * path or a more advanced uncertainty-aware estimator. It is <em>not</em> a full dump of every
 * internal filter variable.</p>
 */
public final class VisionCorrectionStats {

    /**
     * Total number of accepted vision corrections.
     */
    public final int acceptedVisionCount;
    /**
     * Total number of newly-observed vision measurements that were rejected.
     */
    public final int rejectedVisionCount;
    /**
     * Number of duplicate frame timestamps skipped instead of being re-applied.
     */
    public final int skippedDuplicateVisionCount;
    /**
     * Number of older-than-last-evaluated frame timestamps skipped.
     */
    public final int skippedOutOfOrderVisionCount;
    /**
     * Number of accepted corrections that used measurement-time replay.
     */
    public final int replayedVisionCount;
    /**
     * Number of accepted corrections that fell back to a projected-now path.
     */
    public final int projectedVisionCount;
    /**
     * Wall-clock timestamp when vision was last accepted, or {@code NaN} if never.
     */
    public final double lastVisionAcceptedSec;
    /**
     * Measurement timestamp of the most recently accepted vision frame, or {@code NaN} if never.
     */
    public final double lastAcceptedVisionMeasurementTimestampSec;
    /**
     * Measurement timestamp of the most recently evaluated vision frame, or {@code NaN} if never.
     */
    public final double lastEvaluatedVisionTimestampSec;
    /**
     * Whether the most recently accepted correction used measurement-time replay.
     */
    public final boolean lastVisionUsedReplay;

    /**
     * Creates a new immutable status snapshot.
     */
    public VisionCorrectionStats(int acceptedVisionCount,
                                 int rejectedVisionCount,
                                 int skippedDuplicateVisionCount,
                                 int skippedOutOfOrderVisionCount,
                                 int replayedVisionCount,
                                 int projectedVisionCount,
                                 double lastVisionAcceptedSec,
                                 double lastAcceptedVisionMeasurementTimestampSec,
                                 double lastEvaluatedVisionTimestampSec,
                                 boolean lastVisionUsedReplay) {
        this.acceptedVisionCount = acceptedVisionCount;
        this.rejectedVisionCount = rejectedVisionCount;
        this.skippedDuplicateVisionCount = skippedDuplicateVisionCount;
        this.skippedOutOfOrderVisionCount = skippedOutOfOrderVisionCount;
        this.replayedVisionCount = replayedVisionCount;
        this.projectedVisionCount = projectedVisionCount;
        this.lastVisionAcceptedSec = lastVisionAcceptedSec;
        this.lastAcceptedVisionMeasurementTimestampSec = lastAcceptedVisionMeasurementTimestampSec;
        this.lastEvaluatedVisionTimestampSec = lastEvaluatedVisionTimestampSec;
        this.lastVisionUsedReplay = lastVisionUsedReplay;
    }

    /**
     * Convenience snapshot for estimators that have not yet evaluated any vision data.
     */
    public static VisionCorrectionStats none() {
        return new VisionCorrectionStats(0, 0, 0, 0, 0, 0,
                Double.NaN, Double.NaN, Double.NaN, false);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "VisionCorrectionStats{" +
                "acceptedVisionCount=" + acceptedVisionCount +
                ", rejectedVisionCount=" + rejectedVisionCount +
                ", skippedDuplicateVisionCount=" + skippedDuplicateVisionCount +
                ", skippedOutOfOrderVisionCount=" + skippedOutOfOrderVisionCount +
                ", replayedVisionCount=" + replayedVisionCount +
                ", projectedVisionCount=" + projectedVisionCount +
                ", lastVisionAcceptedSec=" + lastVisionAcceptedSec +
                ", lastAcceptedVisionMeasurementTimestampSec=" + lastAcceptedVisionMeasurementTimestampSec +
                ", lastEvaluatedVisionTimestampSec=" + lastEvaluatedVisionTimestampSec +
                ", lastVisionUsedReplay=" + lastVisionUsedReplay +
                '}';
    }
}
