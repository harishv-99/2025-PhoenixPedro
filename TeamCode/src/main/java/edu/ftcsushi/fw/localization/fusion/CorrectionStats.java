package edu.ftcsushi.fw.localization.fusion;

import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Immutable debug/status snapshot shared by predictor + correction localizers.
 *
 * <p>This intentionally captures only the common, high-level behavior that testers and robot code
 * may want regardless of whether the underlying estimator is the lightweight complementary fusion
 * path or a more advanced uncertainty-aware estimator. It is <em>not</em> a full dump of every
 * internal filter variable. Accepted corrections partition into replayed and non-replayed counts.
 * Acceptance is not itself proof of a newly incorporated pose: consumers still inspect the
 * estimate's availability, evidence timestamp, and quality.</p>
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
     * Number of accepted corrections that did not use measurement-time replay.
     *
     * <p>This includes direct updates and supported motion-aligned projections; it does not
     * promise that every counted correction was projected to the current loop.</p>
     */
    public final int nonReplayedCorrectionCount;
    /**
     * Loop timestamp when a correction was last accepted, or unavailable before acceptance or
     * after the corresponding lifecycle state is cleared. This is not the pose's evidence time.
     */
    public final LoopTimestamp lastCorrectionAccepted;
    /**
     * Measurement timestamp of the most recently accepted correction, or unavailable before
     * acceptance or after the corresponding lifecycle state is cleared.
     */
    public final LoopTimestamp lastAcceptedCorrectionMeasurementTimestamp;
    /**
     * Time boundary used to skip already-evaluated or pre-rebase corrections.
     *
     * <p>Normally this is the most recently evaluated measurement's capture time. A predictor
     * rebase can instead set it to the observed rebase's loop boundary to exclude older-segment
     * frames. It is unavailable before evaluation or after a temporal clear; it is not itself
     * evidence that a correction was captured or accepted at that time.</p>
     */
    public final LoopTimestamp lastEvaluatedCorrectionTimestamp;
    /**
     * Whether the most recently accepted correction used measurement-time replay.
     *
     * <p>Rejected, duplicate, and out-of-order candidates do not replace this value. It is false
     * before acceptance and after the corresponding lifecycle state is cleared.</p>
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
                           int nonReplayedCorrectionCount,
                           LoopTimestamp lastCorrectionAccepted,
                           LoopTimestamp lastAcceptedCorrectionMeasurementTimestamp,
                           LoopTimestamp lastEvaluatedCorrectionTimestamp,
                           boolean lastCorrectionUsedReplay) {
        this.acceptedCorrectionCount = acceptedCorrectionCount;
        this.rejectedCorrectionCount = rejectedCorrectionCount;
        this.skippedDuplicateCorrectionCount = skippedDuplicateCorrectionCount;
        this.skippedOutOfOrderCorrectionCount = skippedOutOfOrderCorrectionCount;
        this.replayedCorrectionCount = replayedCorrectionCount;
        this.nonReplayedCorrectionCount = nonReplayedCorrectionCount;
        if (lastCorrectionAccepted == null
                || lastAcceptedCorrectionMeasurementTimestamp == null
                || lastEvaluatedCorrectionTimestamp == null) {
            throw new IllegalArgumentException(
                    "CorrectionStats timestamps are required; use LoopTimestamp.unavailable() when absent");
        }
        this.lastCorrectionAccepted = lastCorrectionAccepted;
        this.lastAcceptedCorrectionMeasurementTimestamp = lastAcceptedCorrectionMeasurementTimestamp;
        this.lastEvaluatedCorrectionTimestamp = lastEvaluatedCorrectionTimestamp;
        this.lastCorrectionUsedReplay = lastCorrectionUsedReplay;
    }

    /**
     * @return convenience snapshot for estimators that have not yet evaluated any corrections.
     */
    public static CorrectionStats none() {
        return new CorrectionStats(0, 0, 0, 0, 0, 0,
                LoopTimestamp.unavailable(),
                LoopTimestamp.unavailable(),
                LoopTimestamp.unavailable(),
                false);
    }

    @Override
    public String toString() {
        return "CorrectionStats{" +
                "acceptedCorrectionCount=" + acceptedCorrectionCount +
                ", rejectedCorrectionCount=" + rejectedCorrectionCount +
                ", skippedDuplicateCorrectionCount=" + skippedDuplicateCorrectionCount +
                ", skippedOutOfOrderCorrectionCount=" + skippedOutOfOrderCorrectionCount +
                ", replayedCorrectionCount=" + replayedCorrectionCount +
                ", nonReplayedCorrectionCount=" + nonReplayedCorrectionCount +
                ", lastCorrectionAccepted=" + lastCorrectionAccepted +
                ", lastAcceptedCorrectionMeasurementTimestamp=" + lastAcceptedCorrectionMeasurementTimestamp +
                ", lastEvaluatedCorrectionTimestamp=" + lastEvaluatedCorrectionTimestamp +
                ", lastCorrectionUsedReplay=" + lastCorrectionUsedReplay +
                '}';
    }
}
