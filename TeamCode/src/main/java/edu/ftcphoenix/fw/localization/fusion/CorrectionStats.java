package edu.ftcphoenix.fw.localization.fusion;

/**
 * Immutable debug/status snapshot shared by predictor + correction localizers.
 *
 * <p>This intentionally captures only the common, high-level behavior that testers and robot code
 * may want regardless of whether the underlying estimator is the lightweight complementary fusion
 * path or a more advanced uncertainty-aware estimator. It is <em>not</em> a full dump of every
 * internal filter variable.</p>
 */
public final class CorrectionStats {

    /**
     * Total number of accepted absolute corrections.
     */
    public final int acceptedCorrectionCount;
    /**
     * Total number of newly observed absolute measurements that were rejected.
     */
    public final int rejectedCorrectionCount;
    /**
     * Number of duplicate frame timestamps skipped instead of being re-applied.
     */
    public final int skippedDuplicateCorrectionCount;
    /**
     * Number of older-than-last-evaluated frame timestamps skipped.
     */
    public final int skippedOutOfOrderCorrectionCount;
    /**
     * Number of accepted corrections that used measurement-time replay.
     */
    public final int replayedCorrectionCount;
    /**
     * Number of accepted corrections that fell back to a projected-now path.
     */
    public final int projectedCorrectionCount;
    /**
     * Wall-clock timestamp when a correction was last accepted, or {@code NaN} if never.
     */
    public final double lastCorrectionAcceptedSec;
    /**
     * Measurement timestamp of the most recently accepted correction, or {@code NaN} if never.
     */
    public final double lastAcceptedCorrectionMeasurementTimestampSec;
    /**
     * Measurement timestamp of the most recently evaluated correction, or {@code NaN} if never.
     */
    public final double lastEvaluatedCorrectionTimestampSec;
    /**
     * Whether the most recently accepted correction used measurement-time replay.
     */
    public final boolean lastCorrectionUsedReplay;

    /**
     * Creates a new immutable status snapshot.
     */
    public CorrectionStats(int acceptedCorrectionCount,
                           int rejectedCorrectionCount,
                           int skippedDuplicateCorrectionCount,
                           int skippedOutOfOrderCorrectionCount,
                           int replayedCorrectionCount,
                           int projectedCorrectionCount,
                           double lastCorrectionAcceptedSec,
                           double lastAcceptedCorrectionMeasurementTimestampSec,
                           double lastEvaluatedCorrectionTimestampSec,
                           boolean lastCorrectionUsedReplay) {
        this.acceptedCorrectionCount = acceptedCorrectionCount;
        this.rejectedCorrectionCount = rejectedCorrectionCount;
        this.skippedDuplicateCorrectionCount = skippedDuplicateCorrectionCount;
        this.skippedOutOfOrderCorrectionCount = skippedOutOfOrderCorrectionCount;
        this.replayedCorrectionCount = replayedCorrectionCount;
        this.projectedCorrectionCount = projectedCorrectionCount;
        this.lastCorrectionAcceptedSec = lastCorrectionAcceptedSec;
        this.lastAcceptedCorrectionMeasurementTimestampSec = lastAcceptedCorrectionMeasurementTimestampSec;
        this.lastEvaluatedCorrectionTimestampSec = lastEvaluatedCorrectionTimestampSec;
        this.lastCorrectionUsedReplay = lastCorrectionUsedReplay;
    }

    /**
     * @return convenience snapshot for estimators that have not yet evaluated any corrections.
     */
    public static CorrectionStats none() {
        return new CorrectionStats(0, 0, 0, 0, 0, 0,
                Double.NaN, Double.NaN, Double.NaN, false);
    }

    @Override
    public String toString() {
        return "CorrectionStats{" +
                "acceptedCorrectionCount=" + acceptedCorrectionCount +
                ", rejectedCorrectionCount=" + rejectedCorrectionCount +
                ", skippedDuplicateCorrectionCount=" + skippedDuplicateCorrectionCount +
                ", skippedOutOfOrderCorrectionCount=" + skippedOutOfOrderCorrectionCount +
                ", replayedCorrectionCount=" + replayedCorrectionCount +
                ", projectedCorrectionCount=" + projectedCorrectionCount +
                ", lastCorrectionAcceptedSec=" + lastCorrectionAcceptedSec +
                ", lastAcceptedCorrectionMeasurementTimestampSec=" + lastAcceptedCorrectionMeasurementTimestampSec +
                ", lastEvaluatedCorrectionTimestampSec=" + lastEvaluatedCorrectionTimestampSec +
                ", lastCorrectionUsedReplay=" + lastCorrectionUsedReplay +
                '}';
    }
}
