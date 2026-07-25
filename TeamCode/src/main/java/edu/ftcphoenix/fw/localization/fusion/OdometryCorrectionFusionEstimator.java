package edu.ftcphoenix.fw.localization.fusion;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;

/**
 * A lightweight fusion localizer:
 *
 * <ul>
 *   <li><b>Predictor</b> provides smooth short-term motion (dead-wheels, Pinpoint, etc.).</li>
 *   <li><b>AprilTag correction</b> provides occasional absolute corrections (when tags are visible).</li>
 * </ul>
 *
 * <p>Design goals for Phoenix:
 * <ul>
 *   <li>Simple, predictable behavior (no Kalman filter tuning to start).</li>
 *   <li>Works well even when tags disappear near the scoring target (predictor carries you).</li>
 *   <li>Allows gentle "snap-back" corrections when tags reappear.</li>
 *   <li>Optionally applies fresh correction measurements at their measurement timestamp and then
 *       replays predictor forward so camera latency hurts less than a naive "compare an old frame
 *       directly against the current pose" scheme.</li>
 * </ul>
 *
 * <p><b>Loop ordering:</b> this estimator calls {@link AbsolutePoseEstimator#update(LoopClock)} on its
 * sources inside its own {@link #update(LoopClock)}. If your correction estimator depends on some other
 * sensor graph being updated first, that dependency must still be updated before calling {@link #update(LoopClock)} on this class. AprilTag sensors and selectors in Phoenix are cycle-idempotent sources, so sharing one instance across consumers remains safe.</p>
 *
 * <p><b>Predictor rebasing after corrections:</b> when this estimator pushes a corrected fused pose
 * back into an underlying predictor localizer that supports {@link PoseResetter}, it also rebases
 * its stored predictor baseline to that corrected pose. Without that rebase, the next predictor
 * delta would accidentally re-apply the reset jump and double-count the correction.</p>
 *
 * <p><b>Duplicate correction frames:</b> camera pipelines often expose the same processed frame across
 * multiple robot loops while its age increases. This estimator only evaluates a given correction
 * measurement timestamp once, so a single stale frame cannot keep "pulling" the fused pose over
 * several loops.</p>
 *
 * <p><b>Reported fused quality:</b> the short-term confidence boost given after an accepted correction
 * now scales with the accepted correction measurement's own quality instead of treating
 * every fresh correction as equally trustworthy. Manual {@link #setPose(Pose2d)} anchors clear
 * that recent-correction hold so resets do not masquerade as fresh camera corrections.</p>
 */
public class OdometryCorrectionFusionEstimator implements CorrectedPoseEstimator {

    private static final double TIMESTAMP_EPS_SEC = 1e-6;

    private enum PredictorMotionRebaseState {
        NONE,
        AWAITING_PREDICTOR_POSE,
        ANCHORED
    }

    /**
     * Configuration for the fusion behavior.
     */
    public static final class Config {
        /**
         * Reject correction measurements older than this (seconds).
         */
        public double maxCorrectionAgeSec = 0.25;

        /**
         * Reject correction measurements with quality below this (0..1).
         */
        public double minCorrectionQuality = 0.05;

        /**
         * How aggressively to correct x/y toward the correction measurement (scaled by correction quality).
         */
        public double correctionPositionGain = 0.25;

        /**
         * How aggressively to correct heading toward the correction measurement (scaled by correction quality).
         */
        public double correctionHeadingGain = 0.35;

        /**
         * Reject corrections that jump more than this distance (inches).
         */
        public double maxCorrectionPositionJumpIn = 24.0;

        /**
         * Reject corrections that jump more than this heading delta (radians).
         */
        public double maxCorrectionHeadingJumpRad = Math.toRadians(60.0);

        /**
         * If true, the estimator may initialize from a fresh correction measurement.
         */
        public boolean enableInitializeFromCorrection = true;

        /**
         * If true, push the fused pose back into the predictor estimator when it supports resets.
         */
        public boolean enablePushCorrectedPoseToPredictor = true;

        /**
         * How long (seconds) a recently-accepted correction measurement should boost the reported quality.
         */
        public double correctionConfidenceHoldSec = 0.75;

        /**
         * If true, accepted correction measurements are corrected at their measurement timestamp and the
         * stored predictor motion since that timestamp is replayed forward to now.
         *
         * <p>This is a lightweight, deterministic form of latency compensation. It intentionally
         * avoids a full state-estimator stack while still fixing the most common AprilTag fusion
         * failure mode: an old camera frame dragging the fused pose backward while the robot keeps
         * moving.</p>
         */
        public boolean enableLatencyCompensation = true;

        /**
         * How much recent predictor history (seconds) to retain for latency compensation.
         *
         * <p>When latency compensation is enabled, this should be at least as large as
         * {@link #maxCorrectionAgeSec} so every still-acceptable correction frame can be replayed through
         * predictor history.</p>
         */
        public double predictorHistorySec = 1.0;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Create a new config instance with Phoenix defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Validates this config and throws an actionable error when it is inconsistent.
         */
        public void validate(String context) {
            String p = (context != null && !context.trim().isEmpty())
                    ? context.trim()
                    : "OdometryCorrectionFusionEstimator.Config";

            if (!Double.isFinite(maxCorrectionAgeSec) || maxCorrectionAgeSec < 0.0) {
                throw new IllegalArgumentException(p + ".maxCorrectionAgeSec must be finite and >= 0");
            }
            if (!Double.isFinite(minCorrectionQuality) || minCorrectionQuality < 0.0 || minCorrectionQuality > 1.0) {
                throw new IllegalArgumentException(p + ".minCorrectionQuality must be finite and within [0, 1]");
            }
            if (!Double.isFinite(correctionPositionGain) || correctionPositionGain < 0.0) {
                throw new IllegalArgumentException(p + ".correctionPositionGain must be finite and >= 0");
            }
            if (!Double.isFinite(correctionHeadingGain) || correctionHeadingGain < 0.0) {
                throw new IllegalArgumentException(p + ".correctionHeadingGain must be finite and >= 0");
            }
            if (!Double.isFinite(maxCorrectionPositionJumpIn) || maxCorrectionPositionJumpIn < 0.0) {
                throw new IllegalArgumentException(p + ".maxCorrectionPositionJumpIn must be finite and >= 0");
            }
            if (!Double.isFinite(maxCorrectionHeadingJumpRad) || maxCorrectionHeadingJumpRad < 0.0) {
                throw new IllegalArgumentException(p + ".maxCorrectionHeadingJumpRad must be finite and >= 0");
            }
            if (!Double.isFinite(correctionConfidenceHoldSec) || correctionConfidenceHoldSec < 0.0) {
                throw new IllegalArgumentException(p + ".correctionConfidenceHoldSec must be finite and >= 0");
            }
            if (!Double.isFinite(predictorHistorySec) || predictorHistorySec < 0.0) {
                throw new IllegalArgumentException(p + ".predictorHistorySec must be finite and >= 0");
            }
            if (enableLatencyCompensation && predictorHistorySec + TIMESTAMP_EPS_SEC < maxCorrectionAgeSec) {
                throw new IllegalArgumentException(
                        p + ".predictorHistorySec must be >= maxCorrectionAgeSec when latency compensation is enabled"
                                + " (increase predictorHistorySec or reduce maxCorrectionAgeSec)"
                );
            }
        }

        /**
         * Returns a validated copy of this config.
         */
        public Config validatedCopy(String context) {
            Config c = copy();
            c.validate(context);
            return c;
        }

        /**
         * Deep copy of this config.
         */
        public Config copy() {
            Config c = new Config();
            c.maxCorrectionAgeSec = this.maxCorrectionAgeSec;
            c.minCorrectionQuality = this.minCorrectionQuality;
            c.correctionPositionGain = this.correctionPositionGain;
            c.correctionHeadingGain = this.correctionHeadingGain;
            c.maxCorrectionPositionJumpIn = this.maxCorrectionPositionJumpIn;
            c.maxCorrectionHeadingJumpRad = this.maxCorrectionHeadingJumpRad;
            c.enableInitializeFromCorrection = this.enableInitializeFromCorrection;
            c.enablePushCorrectedPoseToPredictor = this.enablePushCorrectedPoseToPredictor;
            c.correctionConfidenceHoldSec = this.correctionConfidenceHoldSec;
            c.enableLatencyCompensation = this.enableLatencyCompensation;
            c.predictorHistorySec = this.predictorHistorySec;
            return c;
        }
    }

    private static final class PredictorSample {
        final LoopTimestamp timestamp;
        final Pose3d pose;

        PredictorSample(LoopTimestamp timestamp, Pose3d pose) {
            this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
            this.pose = pose;
        }
    }

    private final MotionPredictor predictor;
    private final AbsolutePoseEstimator correction;
    private final Config cfg;

    private boolean initialized = false;
    private boolean correctionEnabled = true;

    private Pose3d fusedPose = Pose3d.zero();
    private Pose3d lastPredictorPose = Pose3d.zero();

    private PoseEstimate lastEstimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
    private final Deque<PredictorSample> predictorHistory = new ArrayDeque<PredictorSample>();

    // One estimator instance owns one state transition per shared-loop cycle. The attempt is
    // claimed before child updates because those updates and predictor rebases may have effects
    // that cannot be rolled back safely after a failure.
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean updateInProgress = false;
    private RuntimeException lastUpdateFailure = null;

    // Canonical identity of predictor motion already incorporated into fusedPose. This is
    // independent of the loop-cycle guard because a predictor may retain one sample across loops.
    private LoopTimestamp lastCoveredPredictorMotionEndTimestamp = LoopTimestamp.unavailable();

    // A pose anchor can occur after the predictor has observed a changed pose at zero elapsed
    // time. Its next positive MotionDelta then spans both sides of the anchor. Retain the predictor
    // pose at the anchor so that one post-anchor interval can exclude that already-covered motion.
    private PredictorMotionRebaseState predictorMotionRebaseState =
            PredictorMotionRebaseState.NONE;
    private Pose3d predictorMotionRebasePose = Pose3d.zero();

    // Replay base: the most recent moment at which the fused pose was explicitly anchored
    // (initialization, manual reset, or accepted correction). Between replay bases, fused
    // motion is purely predictor-driven, so reconstructing the fused pose at a measurement timestamp
    // is just "base fused pose" composed with the predictor delta from base->measurement.
    private boolean replayBaseValid = false;
    private LoopTimestamp replayBaseTimestamp = LoopTimestamp.unavailable();
    private Pose3d replayBaseFusedPose = Pose3d.zero();
    private Pose3d replayBasePredictorPose = Pose3d.zero();

    // Debug/telemetry helpers.
    private LoopTimestamp lastCorrectionAccepted = LoopTimestamp.unavailable();
    private LoopTimestamp lastAcceptedCorrectionMeasurementTimestamp = LoopTimestamp.unavailable();
    private LoopTimestamp lastEvaluatedCorrectionTimestamp = LoopTimestamp.unavailable();
    private Pose3d lastCorrectionPose = Pose3d.zero();
    private Pose3d lastLatencyCompensatedCorrectionPose = Pose3d.zero();
    private Pose3d lastReplayReferencePose = Pose3d.zero();
    private boolean lastCorrectionUsedReplay = false;
    private int acceptedCorrectionCount = 0;
    private int rejectedCorrectionCount = 0;
    private int skippedDuplicateCorrectionCount = 0;
    private int skippedOutOfOrderCorrectionCount = 0;
    private int replayedCorrectionCount = 0;
    private int projectedCorrectionCount = 0;

    /**
     * Creates a fusion estimator with the default fusion configuration.
     */
    public OdometryCorrectionFusionEstimator(MotionPredictor predictor, AbsolutePoseEstimator correction) {
        this(predictor, correction, Config.defaults());
    }

    /**
     * Creates a fusion estimator that combines predictor with a separate correction estimator.
     */
    public OdometryCorrectionFusionEstimator(MotionPredictor predictor, AbsolutePoseEstimator correction, Config cfg) {
        if (predictor == null) {
            throw new IllegalArgumentException("predictor must not be null");
        }
        if (correction == null) {
            throw new IllegalArgumentException("correction must not be null");
        }
        this.predictor = predictor;
        this.correction = correction;
        Config base = (cfg != null) ? cfg : Config.defaults();
        this.cfg = base.validatedCopy("OdometryCorrectionFusionEstimator.Config");
    }

    /**
     * Enables or disables absolute corrections while leaving predictor integration alive.
     *
     * <p>This is useful during testing when you want to compare pure prediction drift against the
     * corrected/global pose without reconstructing the whole estimator stack.</p>
     */
    @Override
    public void setCorrectionEnabled(boolean enabled) {
        this.correctionEnabled = enabled;
    }

    /**
     * Returns whether absolute corrections are currently enabled.
     */
    @Override
    public boolean isCorrectionEnabled() {
        return correctionEnabled;
    }

    /**
     * The loop timestamp when correction was last accepted, or unavailable if never.
     */
    public LoopTimestamp getLastCorrectionAccepted() {
        return lastCorrectionAccepted;
    }

    /**
     * Measurement timestamp of the most recently accepted correction frame, or unavailable if never.
     */
    public LoopTimestamp getLastAcceptedCorrectionMeasurementTimestamp() {
        return lastAcceptedCorrectionMeasurementTimestamp;
    }

    /**
     * Last accepted correction pose at the frame's own measurement timestamp (planarized).
     */
    public Pose3d getLastCorrectionPose() {
        return lastCorrectionPose;
    }

    /**
     * Returns whether the most recently accepted correction used measurement-time replay instead of
     * a simple now-frame projection fallback.
     */
    public boolean wasLastCorrectionReplay() {
        return lastCorrectionUsedReplay;
    }

    /**
     * Returns the total number of accepted corrections.
     */
    public int getAcceptedCorrectionCount() {
        return acceptedCorrectionCount;
    }

    /**
     * Returns the total number of newly-observed correction measurements that were rejected.
     */
    public int getRejectedCorrectionCount() {
        return rejectedCorrectionCount;
    }

    /**
     * Returns how many duplicate frame timestamps were skipped instead of re-applying the same
     * correction measurement multiple loops in a row.
     */
    public int getSkippedDuplicateCorrectionCount() {
        return skippedDuplicateCorrectionCount;
    }

    /**
     * Returns how many older-than-last-evaluated correction timestamps were skipped.
     */
    public int getSkippedOutOfOrderCorrectionCount() {
        return skippedOutOfOrderCorrectionCount;
    }

    /**
     * Returns how many accepted corrections used measurement-time replay.
     */
    public int getReplayedCorrectionCount() {
        return replayedCorrectionCount;
    }

    /**
     * Returns how many accepted corrections fell back to a simple now-frame projection.
     */
    public int getProjectedCorrectionCount() {
        return projectedCorrectionCount;
    }

    /**
     * Returns aggregate correction counters and timestamps useful for telemetry and tuning.
     */
    @Override
    public CorrectionStats getCorrectionStats() {
        return new CorrectionStats(
                acceptedCorrectionCount,
                rejectedCorrectionCount,
                skippedDuplicateCorrectionCount,
                skippedOutOfOrderCorrectionCount,
                replayedCorrectionCount,
                projectedCorrectionCount,
                lastCorrectionAccepted,
                lastAcceptedCorrectionMeasurementTimestamp,
                lastEvaluatedCorrectionTimestamp,
                lastCorrectionUsedReplay
        );
    }

    /**
     * Advances the corrected/global estimator one loop by propagating the predictor and then
     * conditionally applying the latest absolute correction measurement.
     *
     * <p>The first call in a {@link LoopClock#cycle()} claims that owner's update attempt before
     * any child source is advanced. A successful same-cycle repeat is an exact no-op, a reentrant
     * call fails fast, and a same-cycle repeat after a {@link RuntimeException} rethrows that same
     * failure without retrying effects. The next clock cycle is eligible for a new attempt.</p>
     *
     * <p>Cycle identity and predictor-sample identity are separate safeguards. Across cycles, a
     * usable predictor {@link MotionDelta} is applied only when its current-epoch end timestamp is
     * strictly newer than the predictor motion already consumed or covered by a pose rebase. If a
     * rebase has no truthful predictor-side pose at or after its anchor time, the first later
     * coherent predictor sample establishes that base without replaying its crossing interval.</p>
     */
    @Override
    public void update(LoopClock clock) {
        LoopClock currentClock = Objects.requireNonNull(clock, "clock");
        long cycle = currentClock.cycle();

        if (updateInProgress) {
            throw new IllegalStateException(
                    "OdometryCorrectionFusionEstimator.update(clock) cannot be called "
                            + "reentrantly; one estimator owner may update once per loop cycle"
            );
        }
        if (lastUpdateCycle == cycle) {
            if (lastUpdateFailure != null) {
                throw lastUpdateFailure;
            }
            return;
        }

        lastUpdateCycle = cycle;
        lastUpdateFailure = null;
        updateInProgress = true;
        try {
            updateOnce(currentClock);
        } catch (RuntimeException failure) {
            lastUpdateFailure = failure;
            throw failure;
        } finally {
            updateInProgress = false;
        }
    }

    private void updateOnce(LoopClock clock) {
        final LoopTimestamp nowTimestamp = clock.nowTimestamp();

        invalidateHistoryAcrossReset(nowTimestamp);

        // Update sources.
        predictor.update(clock);
        correction.update(clock);

        final PoseEstimate predictorEst = predictor.getEstimate();
        final MotionDelta predictorDelta = predictor.getLatestMotionDelta();
        final PoseEstimate correctionEst = correction.getEstimate();
        final boolean predictorTimestampCurrent = predictorEst != null
                && predictorEst.timestamp != null
                && Double.isFinite(predictorEst.timestamp.ageSec(clock));
        final Pose3d currentPredictorPose = (predictorEst != null
                && predictorEst.hasPose
                && predictorTimestampCurrent)
                ? planarize(predictorEst.fieldToRobotPose)
                : null;
        final LoopTimestamp currentPredictorTimestamp = predictorTimestampCurrent
                ? predictorEst.timestamp
                : LoopTimestamp.unavailable();

        if (currentPredictorPose != null) {
            recordPredictorSample(currentPredictorTimestamp, currentPredictorPose);
        }

        boolean evaluatedCorrectionThisLoop = false;

        // If we are not initialized yet, pick an initial pose.
        if (!initialized) {
            boolean initializedFromCorrection = false;
            if (correctionEnabled
                    && cfg.enableInitializeFromCorrection
                    && shouldEvaluateCorrectionMeasurement(correctionEst, clock)) {
                evaluatedCorrectionThisLoop = true;
                if (isCorrectionAcceptable(correctionEst, clock)) {
                    Pose3d correctionPose = planarize(correctionEst.fieldToRobotPose);
                    Pose3d initialPose = cfg.enableLatencyCompensation
                            ? projectCorrectionPoseToNow(correctionPose, correctionEst.timestamp, currentPredictorPose)
                            : correctionPose;

                    fusedPose = initialPose;
                    initialized = true;
                    lastCorrectionPose = correctionPose;
                    lastLatencyCompensatedCorrectionPose = initialPose;
                    lastReplayReferencePose = correctionPose;
                    lastCorrectionAccepted = nowTimestamp;
                    lastAcceptedCorrectionMeasurementTimestamp = correctionEst.timestamp;
                    lastCorrectionUsedReplay = false;
                    acceptedCorrectionCount++;
                    projectedCorrectionCount++;

                    boolean pushedToPredictor = pushFusedPoseToPredictor();
                    rebaseAfterPoseChange(
                            nowTimestamp,
                            currentPredictorPose,
                            currentPredictorTimestamp,
                            pushedToPredictor
                    );
                    initializedFromCorrection = true;
                } else {
                    rejectedCorrectionCount++;
                }
            }

            if (!initialized && currentPredictorPose != null) {
                fusedPose = currentPredictorPose;
                initialized = true;
                lastPredictorPose = currentPredictorPose;
                resetPredictorHistory(currentPredictorTimestamp, currentPredictorPose);
                setReplayBase(currentPredictorTimestamp, fusedPose, currentPredictorPose);
                markPredictorMotionCovered(currentPredictorTimestamp);
                rememberPredictorMotionRebase(currentPredictorPose);
            } else if (!initialized && !initializedFromCorrection) {
                // No pose from either source yet.
                lastEstimate = PoseEstimate.noPose(nowTimestamp);
                return;
            }
        } else {
            // Propagate fused pose using the predictor's explicit motion delta.
            establishAwaitingPredictorMotionRebase(
                    currentPredictorTimestamp,
                    currentPredictorPose
            );
            Pose3d predictorMotion = predictorMotionForUpdate(
                    predictorDelta,
                    currentPredictorPose,
                    clock
            );
            if (predictorMotion != null) {
                fusedPose = planarize(fusedPose.then(predictorMotion));
                markPredictorMotionCovered(predictorDelta.endTimestamp);
                clearPredictorMotionRebase();
            }
            if (currentPredictorPose != null) {
                lastPredictorPose = currentPredictorPose;
            }
        }

        // Apply correction if available.
        if (correctionEnabled
                && !evaluatedCorrectionThisLoop
                && shouldEvaluateCorrectionMeasurement(correctionEst, clock)) {
            if (!isCorrectionAcceptable(correctionEst, clock)) {
                rejectedCorrectionCount++;
            } else {
                maybeApplyCorrection(
                        correctionEst,
                        currentPredictorPose,
                        currentPredictorTimestamp,
                        nowTimestamp
                );
            }
        }

        // Compute a simple "confidence" signal.
        double quality = (predictorEst != null && predictorEst.hasPose)
                ? MathUtil.clamp(predictorEst.quality, 0.0, 1.0)
                : 0.0;

        double correctionAgeSec = lastCorrectionAccepted.ageSec(clock);
        if (Double.isFinite(correctionAgeSec)) {
            if (correctionAgeSec < cfg.correctionConfidenceHoldSec) {
                double boost = 1.0 - (correctionAgeSec / cfg.correctionConfidenceHoldSec);
                quality = MathUtil.clamp(Math.max(quality, boost), 0.0, 1.0);
            }
        }

        lastEstimate = new PoseEstimate(fusedPose, true, quality, nowTimestamp);
    }

    /**
     * Returns the most recent corrected/global pose estimate.
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * Manually anchors the corrected/global estimator to a known field pose.
     *
     * <p>Typical usage is a tester button or a known start-pose initialization path.</p>
     */
    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            return;
        }

        final LoopTimestamp nowTimestamp = (lastEstimate != null && lastEstimate.timestamp != null)
                ? lastEstimate.timestamp
                : LoopTimestamp.unavailable();

        fusedPose = new Pose3d(pose.xInches, pose.yInches, 0.0, MathUtil.wrapToPi(pose.headingRad), 0.0, 0.0);
        initialized = true;

        Pose3d currentPredictorPose = null;
        LoopTimestamp currentPredictorTimestamp = LoopTimestamp.unavailable();
        PoseEstimate predictorEst = predictor.getEstimate();
        if (predictorEst != null && predictorEst.hasPose) {
            currentPredictorPose = planarize(predictorEst.fieldToRobotPose);
            if (predictorEst.timestamp != null && predictorEst.timestamp.isAvailable()) {
                currentPredictorTimestamp = predictorEst.timestamp;
            }
        }

        boolean pushedToPredictor = pushFusedPoseToPredictor();
        clearRecentCorrectionState();
        rebaseAfterPoseChange(
                nowTimestamp,
                currentPredictorPose,
                currentPredictorTimestamp,
                pushedToPredictor
        );

        double quality = (predictorEst != null
                && predictorEst.hasPose
                && Double.isFinite(predictorEst.quality))
                ? MathUtil.clamp(predictorEst.quality, 0.0, 1.0)
                : 1.0;
        lastEstimate = new PoseEstimate(fusedPose, true, quality, nowTimestamp);
    }

    private boolean shouldEvaluateCorrectionMeasurement(PoseEstimate correctionEst, LoopClock clock) {
        if (correctionEst == null
                || !correctionEst.hasPose
                || correctionEst.timestamp == null
                || !Double.isFinite(clock.nowTimestamp().secondsSince(correctionEst.timestamp))) {
            return false;
        }
        LoopTimestamp timestamp = correctionEst.timestamp;
        if (!lastEvaluatedCorrectionTimestamp.isAvailable()) {
            lastEvaluatedCorrectionTimestamp = timestamp;
            return true;
        }
        double elapsedSec = timestamp.secondsSince(lastEvaluatedCorrectionTimestamp);
        if (!Double.isFinite(elapsedSec)) {
            lastEvaluatedCorrectionTimestamp = timestamp;
            return true;
        }
        if (elapsedSec > TIMESTAMP_EPS_SEC) {
            lastEvaluatedCorrectionTimestamp = timestamp;
            return true;
        }
        if (Math.abs(elapsedSec) <= TIMESTAMP_EPS_SEC) {
            skippedDuplicateCorrectionCount++;
        } else {
            skippedOutOfOrderCorrectionCount++;
        }
        return false;
    }

    private boolean isCorrectionAcceptable(PoseEstimate correctionEst, LoopClock clock) {
        if (correctionEst == null || !correctionEst.hasPose || correctionEst.fieldToRobotPose == null) {
            return false;
        }
        if (correctionEst.timestamp == null) {
            return false;
        }
        double ageSec = correctionEst.timestamp.ageSec(clock);
        if (!Double.isFinite(ageSec)) {
            return false;
        }

        // Freshness gate.
        if (cfg.maxCorrectionAgeSec > 0.0 && ageSec > cfg.maxCorrectionAgeSec) {
            return false;
        }

        // Quality gate.
        if (!Double.isFinite(correctionEst.quality) || correctionEst.quality < cfg.minCorrectionQuality) {
            return false;
        }

        // NaN gate.
        Pose3d p = correctionEst.fieldToRobotPose;
        return !(Double.isNaN(p.xInches) || Double.isNaN(p.yInches) || Double.isNaN(p.yawRad));
    }

    private void maybeApplyCorrection(PoseEstimate correctionEst,
                                      Pose3d currentPredictorPose,
                                      LoopTimestamp currentPredictorTimestamp,
                                      LoopTimestamp nowTimestamp) {
        Pose3d correctionPoseAtMeasurement = planarize(correctionEst.fieldToRobotPose);
        Pose3d compensatedCorrectionPoseAtNow = cfg.enableLatencyCompensation
                ? projectCorrectionPoseToNow(correctionPoseAtMeasurement, correctionEst.timestamp, currentPredictorPose)
                : correctionPoseAtMeasurement;

        lastCorrectionPose = correctionPoseAtMeasurement;
        lastLatencyCompensatedCorrectionPose = compensatedCorrectionPoseAtNow;

        if (cfg.enableLatencyCompensation
                && replayBaseValid
                && correctionEst.timestamp.secondsSince(replayBaseTimestamp) < -TIMESTAMP_EPS_SEC) {
            // This frame predates the most recent accepted correction/reset base. Re-applying it
            // would drag the estimator back across a newer anchor, so reject it.
            lastReplayReferencePose = replayBaseFusedPose;
            lastCorrectionUsedReplay = false;
            rejectedCorrectionCount++;
            return;
        }

        final double q = MathUtil.clamp(correctionEst.quality, 0.0, 1.0);
        final double posGain = MathUtil.clamp(cfg.correctionPositionGain * q, 0.0, 1.0);
        final double headingGain = MathUtil.clamp(cfg.correctionHeadingGain * q, 0.0, 1.0);

        Pose3d correctedPoseNow = null;
        boolean usedReplay = false;

        if (cfg.enableLatencyCompensation && currentPredictorPose != null) {
            Pose3d predictorAtMeasurement = interpolatePredictorPose(correctionEst.timestamp);
            if (predictorAtMeasurement != null) {
                Pose3d fusedAtMeasurement = reconstructFusedPoseAt(correctionEst.timestamp, predictorAtMeasurement);
                if (fusedAtMeasurement != null) {
                    Pose3d predictorDeltaSinceMeasurement = predictorAtMeasurement.inverse().then(currentPredictorPose);
                    lastReplayReferencePose = fusedAtMeasurement;

                    if (isCorrectionJumpAcceptable(fusedAtMeasurement, correctionPoseAtMeasurement)) {
                        Pose3d correctedAtMeasurement = blendToward(
                                fusedAtMeasurement,
                                correctionPoseAtMeasurement,
                                posGain,
                                headingGain
                        );
                        correctedPoseNow = planarize(correctedAtMeasurement.then(predictorDeltaSinceMeasurement));
                        usedReplay = true;
                    } else {
                        lastCorrectionUsedReplay = true;
                        rejectedCorrectionCount++;
                        return;
                    }
                }
            }
        }

        if (correctedPoseNow == null) {
            lastReplayReferencePose = fusedPose;

            if (!isCorrectionJumpAcceptable(fusedPose, compensatedCorrectionPoseAtNow)) {
                lastCorrectionUsedReplay = false;
                rejectedCorrectionCount++;
                return;
            }

            correctedPoseNow = blendToward(
                    fusedPose,
                    compensatedCorrectionPoseAtNow,
                    posGain,
                    headingGain
            );
        }

        fusedPose = correctedPoseNow;
        lastCorrectionAccepted = nowTimestamp;
        lastAcceptedCorrectionMeasurementTimestamp = correctionEst.timestamp;
        lastCorrectionUsedReplay = usedReplay;
        acceptedCorrectionCount++;
        if (usedReplay) {
            replayedCorrectionCount++;
        } else {
            projectedCorrectionCount++;
        }

        boolean pushedToPredictor = pushFusedPoseToPredictor();
        rebaseAfterPoseChange(
                nowTimestamp,
                currentPredictorPose,
                currentPredictorTimestamp,
                pushedToPredictor
        );
    }

    private boolean isCorrectionJumpAcceptable(Pose3d referencePose, Pose3d targetPose) {
        if (referencePose == null || targetPose == null) {
            return false;
        }
        double dx = targetPose.xInches - referencePose.xInches;
        double dy = targetPose.yInches - referencePose.yInches;
        double dPos = Math.hypot(dx, dy);
        double dHeading = MathUtil.wrapToPi(targetPose.yawRad - referencePose.yawRad);
        return dPos <= cfg.maxCorrectionPositionJumpIn
                && Math.abs(dHeading) <= cfg.maxCorrectionHeadingJumpRad;
    }

    private static Pose3d blendToward(Pose3d from, Pose3d to, double posGain, double headingGain) {
        double dx = to.xInches - from.xInches;
        double dy = to.yInches - from.yInches;
        double dHeading = MathUtil.wrapToPi(to.yawRad - from.yawRad);
        return new Pose3d(
                from.xInches + dx * posGain,
                from.yInches + dy * posGain,
                0.0,
                MathUtil.wrapToPi(from.yawRad + dHeading * headingGain),
                0.0,
                0.0
        );
    }

    private boolean pushFusedPoseToPredictor() {
        if (cfg.enablePushCorrectedPoseToPredictor && predictor instanceof PoseResetter) {
            ((PoseResetter) predictor).setPose(fusedPose.toPose2d());
            return true;
        }
        return false;
    }

    private void clearRecentCorrectionState() {
        lastCorrectionAccepted = LoopTimestamp.unavailable();
        lastAcceptedCorrectionMeasurementTimestamp = LoopTimestamp.unavailable();
        lastCorrectionUsedReplay = false;
    }

    private void rebaseAfterPoseChange(LoopTimestamp anchorTimestamp,
                                       Pose3d currentPredictorPose,
                                       LoopTimestamp currentPredictorTimestamp,
                                       boolean pushedToPredictor) {
        Pose3d basePredictorPose;
        LoopTimestamp baseTimestamp;
        if (pushedToPredictor) {
            lastPredictorPose = fusedPose;
            basePredictorPose = fusedPose;
            baseTimestamp = anchorTimestamp;
        } else if (currentPredictorPose != null
                && timestampAtOrAfter(currentPredictorTimestamp, anchorTimestamp)) {
            lastPredictorPose = currentPredictorPose;
            basePredictorPose = currentPredictorPose;
            baseTimestamp = currentPredictorTimestamp;
        } else {
            lastPredictorPose = currentPredictorPose != null ? currentPredictorPose : fusedPose;
            predictorHistory.clear();
            setReplayBase(
                    LoopTimestamp.unavailable(),
                    fusedPose,
                    Pose3d.zero()
            );
            markPredictorMotionCovered(anchorTimestamp);
            awaitPredictorMotionRebase();
            return;
        }

        resetPredictorHistory(baseTimestamp, basePredictorPose);
        setReplayBase(baseTimestamp, fusedPose, basePredictorPose);
        markPredictorMotionCovered(baseTimestamp);
        rememberPredictorMotionRebase(basePredictorPose);
    }

    private static boolean timestampAtOrAfter(LoopTimestamp candidate,
                                              LoopTimestamp reference) {
        if (candidate == null
                || reference == null
                || !candidate.isAvailable()
                || !reference.isAvailable()) {
            return false;
        }
        double elapsedSec = candidate.secondsSince(reference);
        return Double.isFinite(elapsedSec) && elapsedSec >= 0.0;
    }

    private boolean shouldApplyPredictorMotion(MotionDelta predictorDelta, LoopClock clock) {
        if (predictorDelta == null || !predictorDelta.hasDelta) {
            return false;
        }

        double durationSec = predictorDelta.durationSec();
        if (!Double.isFinite(durationSec) || durationSec <= 0.0) {
            return false;
        }
        if (!Double.isFinite(predictorDelta.endTimestamp.ageSec(clock))) {
            return false;
        }
        if (!lastCoveredPredictorMotionEndTimestamp.isAvailable()) {
            return true;
        }

        double elapsedSec = predictorDelta.endTimestamp.secondsSince(
                lastCoveredPredictorMotionEndTimestamp
        );
        return Double.isFinite(elapsedSec) && elapsedSec > 0.0;
    }

    private Pose3d predictorMotionForUpdate(MotionDelta predictorDelta,
                                            Pose3d currentPredictorPose,
                                            LoopClock clock) {
        if (!shouldApplyPredictorMotion(predictorDelta, clock)) {
            return null;
        }
        if (predictorMotionRebaseState == PredictorMotionRebaseState.NONE) {
            return predictorDelta.deltaPose;
        }
        if (predictorMotionRebaseState == PredictorMotionRebaseState.AWAITING_PREDICTOR_POSE
                || currentPredictorPose == null) {
            // A conforming MotionPredictor publishes a coherent absolute pose with the delta. If
            // it does not, fail closed rather than risk replaying motion from before the anchor.
            return null;
        }
        return predictorMotionRebasePose.inverse().then(currentPredictorPose);
    }

    private void rememberPredictorMotionRebase(Pose3d predictorPose) {
        predictorMotionRebasePose = planarize(Objects.requireNonNull(
                predictorPose,
                "predictorPose"
        ));
        predictorMotionRebaseState = PredictorMotionRebaseState.ANCHORED;
    }

    private void awaitPredictorMotionRebase() {
        predictorMotionRebaseState = PredictorMotionRebaseState.AWAITING_PREDICTOR_POSE;
        predictorMotionRebasePose = Pose3d.zero();
    }

    private void establishAwaitingPredictorMotionRebase(LoopTimestamp predictorTimestamp,
                                                         Pose3d currentPredictorPose) {
        if (predictorMotionRebaseState
                != PredictorMotionRebaseState.AWAITING_PREDICTOR_POSE
                || currentPredictorPose == null) {
            return;
        }
        if (lastCoveredPredictorMotionEndTimestamp.isAvailable()) {
            double elapsedFromCoveredSec = predictorTimestamp.secondsSince(
                    lastCoveredPredictorMotionEndTimestamp
            );
            if (!Double.isFinite(elapsedFromCoveredSec) || elapsedFromCoveredSec < 0.0) {
                return;
            }
        }

        lastPredictorPose = currentPredictorPose;
        resetPredictorHistory(predictorTimestamp, currentPredictorPose);
        setReplayBase(predictorTimestamp, fusedPose, currentPredictorPose);
        markPredictorMotionCovered(predictorTimestamp);
        rememberPredictorMotionRebase(currentPredictorPose);
    }

    private void clearPredictorMotionRebase() {
        predictorMotionRebaseState = PredictorMotionRebaseState.NONE;
        predictorMotionRebasePose = Pose3d.zero();
    }

    private void markPredictorMotionCovered(LoopTimestamp timestamp) {
        lastCoveredPredictorMotionEndTimestamp = timestamp != null && timestamp.isAvailable()
                ? timestamp
                : LoopTimestamp.unavailable();
    }

    private void setReplayBase(LoopTimestamp timestamp,
                               Pose3d fusedPoseAtBase,
                               Pose3d predictorPoseAtBase) {
        replayBaseValid = timestamp != null
                && timestamp.isAvailable()
                && fusedPoseAtBase != null
                && predictorPoseAtBase != null;
        replayBaseTimestamp = replayBaseValid ? timestamp : LoopTimestamp.unavailable();
        replayBaseFusedPose = replayBaseValid ? planarize(fusedPoseAtBase) : Pose3d.zero();
        replayBasePredictorPose = replayBaseValid ? planarize(predictorPoseAtBase) : Pose3d.zero();
    }

    private Pose3d reconstructFusedPoseAt(LoopTimestamp timestamp, Pose3d predictorPoseAtTimestamp) {
        if (!replayBaseValid || predictorPoseAtTimestamp == null) {
            return null;
        }
        if (timestamp == null || !timestamp.isAvailable()) {
            return null;
        }
        double elapsedFromBaseSec = timestamp.secondsSince(replayBaseTimestamp);
        if (!Double.isFinite(elapsedFromBaseSec) || elapsedFromBaseSec < -TIMESTAMP_EPS_SEC) {
            return null;
        }

        Pose3d odomDeltaFromBase = replayBasePredictorPose.inverse().then(predictorPoseAtTimestamp);
        return planarize(replayBaseFusedPose.then(odomDeltaFromBase));
    }

    private static Pose3d planarize(Pose3d pose) {
        if (pose == null) {
            return Pose3d.zero();
        }
        return new Pose3d(
                pose.xInches,
                pose.yInches,
                0.0,
                MathUtil.wrapToPi(pose.yawRad),
                0.0,
                0.0);
    }

    private void recordPredictorSample(LoopTimestamp timestamp, Pose3d predictorPose) {
        if (timestamp == null || !timestamp.isAvailable() || predictorPose == null) {
            return;
        }

        PredictorSample last = predictorHistory.peekLast();
        if (last != null) {
            double elapsedSec = timestamp.secondsSince(last.timestamp);
            if (!Double.isFinite(elapsedSec) || elapsedSec <= 0.0) {
                return;
            }
        }

        predictorHistory.addLast(new PredictorSample(timestamp, planarize(predictorPose)));
        prunePredictorHistory(timestamp);
    }

    private void prunePredictorHistory(LoopTimestamp nowTimestamp) {
        double keepSec = Math.max(0.0, cfg.predictorHistorySec);
        if (keepSec <= 0.0) {
            while (predictorHistory.size() > 1) {
                predictorHistory.removeFirst();
            }
            return;
        }

        while (predictorHistory.size() > 2) {
            PredictorSample[] samples = predictorHistory.toArray(new PredictorSample[0]);
            if (samples.length < 2) {
                break;
            }
            double secondAgeSec = nowTimestamp.secondsSince(samples[1].timestamp);
            if (!Double.isFinite(secondAgeSec) || secondAgeSec < 0.0) {
                predictorHistory.clear();
                return;
            }
            if (secondAgeSec <= keepSec) {
                break;
            }
            predictorHistory.removeFirst();
        }
    }

    private void resetPredictorHistory(LoopTimestamp timestamp, Pose3d pose) {
        predictorHistory.clear();
        if (timestamp != null && timestamp.isAvailable() && pose != null) {
            predictorHistory.addLast(new PredictorSample(timestamp, planarize(pose)));
        }
    }

    private Pose3d projectCorrectionPoseToNow(Pose3d correctionPoseAtMeasurement,
                                              LoopTimestamp measurementTimestamp,
                                              Pose3d currentPredictorPose) {
        if (correctionPoseAtMeasurement == null || currentPredictorPose == null) {
            return correctionPoseAtMeasurement;
        }
        if (measurementTimestamp == null || !measurementTimestamp.isAvailable()) {
            return correctionPoseAtMeasurement;
        }

        Pose3d predictorAtMeasurement = interpolatePredictorPose(measurementTimestamp);
        if (predictorAtMeasurement == null) {
            return correctionPoseAtMeasurement;
        }

        Pose3d predictorDeltaSinceMeasurement = predictorAtMeasurement.inverse().then(currentPredictorPose);
        return planarize(correctionPoseAtMeasurement.then(predictorDeltaSinceMeasurement));
    }

    private Pose3d interpolatePredictorPose(LoopTimestamp timestamp) {
        if (timestamp == null || !timestamp.isAvailable() || predictorHistory.isEmpty()) {
            return null;
        }

        PredictorSample[] samples = predictorHistory.toArray(new PredictorSample[0]);
        if (samples.length == 0) {
            return null;
        }
        double sinceFirstSec = timestamp.secondsSince(samples[0].timestamp);
        double untilLastSec = samples[samples.length - 1].timestamp.secondsSince(timestamp);
        if (!Double.isFinite(sinceFirstSec)
                || !Double.isFinite(untilLastSec)
                || sinceFirstSec < 0.0
                || untilLastSec < 0.0) {
            return null;
        }
        if (samples.length == 1) {
            return samples[0].pose;
        }

        PredictorSample prev = samples[0];
        for (PredictorSample next : samples) {
            double querySinceNextSec = timestamp.secondsSince(next.timestamp);
            if (!Double.isFinite(querySinceNextSec)) {
                predictorHistory.clear();
                return null;
            }
            if (querySinceNextSec > 0.0) {
                prev = next;
                continue;
            }
            if (Math.abs(querySinceNextSec) <= 1e-9 || next == prev) {
                return next.pose;
            }

            double dt = next.timestamp.secondsSince(prev.timestamp);
            if (!Double.isFinite(dt)) {
                predictorHistory.clear();
                return null;
            }
            if (dt <= 1e-9) {
                return next.pose;
            }

            double fromPreviousSec = timestamp.secondsSince(prev.timestamp);
            if (!Double.isFinite(fromPreviousSec)) {
                predictorHistory.clear();
                return null;
            }
            double t = MathUtil.clamp01(fromPreviousSec / dt);
            return interpolatePose(prev.pose, next.pose, t);
        }

        return samples[samples.length - 1].pose;
    }

    private static Pose3d interpolatePose(Pose3d a, Pose3d b, double t) {
        double yawDelta = MathUtil.wrapToPi(b.yawRad - a.yawRad);
        return new Pose3d(
                MathUtil.lerp(a.xInches, b.xInches, t),
                MathUtil.lerp(a.yInches, b.yInches, t),
                0.0,
                MathUtil.wrapToPi(a.yawRad + yawDelta * t),
                0.0,
                0.0
        );
    }

    /** Drop only timestamp-dependent state when the shared clock enters a new reset epoch. */
    private void invalidateHistoryAcrossReset(LoopTimestamp nowTimestamp) {
        PredictorSample lastSample = predictorHistory.peekLast();
        if (lastSample != null) {
            double elapsedSec = nowTimestamp.secondsSince(lastSample.timestamp);
            if (!Double.isFinite(elapsedSec) || elapsedSec < 0.0) {
                predictorHistory.clear();
            }
        }

        if (replayBaseValid) {
            double elapsedSec = nowTimestamp.secondsSince(replayBaseTimestamp);
            if (!Double.isFinite(elapsedSec) || elapsedSec < 0.0) {
                replayBaseValid = false;
                replayBaseTimestamp = LoopTimestamp.unavailable();
                replayBaseFusedPose = Pose3d.zero();
                replayBasePredictorPose = Pose3d.zero();
            }
        }

        if (lastEvaluatedCorrectionTimestamp.isAvailable()
                && !Double.isFinite(nowTimestamp.secondsSince(lastEvaluatedCorrectionTimestamp))) {
            lastEvaluatedCorrectionTimestamp = LoopTimestamp.unavailable();
        }

        if (lastCoveredPredictorMotionEndTimestamp.isAvailable()
                && !Double.isFinite(nowTimestamp.secondsSince(
                lastCoveredPredictorMotionEndTimestamp))) {
            lastCoveredPredictorMotionEndTimestamp = LoopTimestamp.unavailable();
            clearPredictorMotionRebase();
        } else if (!lastCoveredPredictorMotionEndTimestamp.isAvailable()
                && predictorMotionRebaseState == PredictorMotionRebaseState.ANCHORED) {
            // An anchor without a timestamp cannot prove that its predictor pose belongs to this
            // clock epoch. Re-establish from the first coherent predictor sample instead.
            if (initialized) {
                awaitPredictorMotionRebase();
            } else {
                clearPredictorMotionRebase();
            }
        }
    }

    /**
     * Debug helper: emit current fusion state and recent correction gating statistics.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "fusion" : prefix;

        dbg.addLine(p)
                .addData(p + ".initialized", initialized)
                .addData(p + ".correctionEnabled", correctionEnabled)
                .addData(p + ".acceptedCorrectionCount", acceptedCorrectionCount)
                .addData(p + ".rejectedCorrectionCount", rejectedCorrectionCount)
                .addData(p + ".skippedDuplicateCorrectionCount", skippedDuplicateCorrectionCount)
                .addData(p + ".skippedOutOfOrderCorrectionCount", skippedOutOfOrderCorrectionCount)
                .addData(p + ".replayedCorrectionCount", replayedCorrectionCount)
                .addData(p + ".projectedCorrectionCount", projectedCorrectionCount)
                .addData(p + ".lastCorrectionAccepted", lastCorrectionAccepted)
                .addData(p + ".lastAcceptedCorrectionMeasurementTimestamp", lastAcceptedCorrectionMeasurementTimestamp)
                .addData(p + ".lastEvaluatedCorrectionTimestamp", lastEvaluatedCorrectionTimestamp)
                .addData(p + ".cfg.maxCorrectionAgeSec", cfg.maxCorrectionAgeSec)
                .addData(p + ".cfg.minCorrectionQuality", cfg.minCorrectionQuality)
                .addData(p + ".cfg.correctionPositionGain", cfg.correctionPositionGain)
                .addData(p + ".cfg.correctionHeadingGain", cfg.correctionHeadingGain)
                .addData(p + ".cfg.maxCorrectionPositionJumpIn", cfg.maxCorrectionPositionJumpIn)
                .addData(p + ".cfg.maxCorrectionHeadingJumpRad", cfg.maxCorrectionHeadingJumpRad)
                .addData(p + ".cfg.enableInitializeFromCorrection", cfg.enableInitializeFromCorrection)
                .addData(p + ".cfg.enablePushCorrectedPoseToPredictor", cfg.enablePushCorrectedPoseToPredictor)
                .addData(p + ".cfg.correctionConfidenceHoldSec", cfg.correctionConfidenceHoldSec)
                .addData(p + ".cfg.enableLatencyCompensation", cfg.enableLatencyCompensation)
                .addData(p + ".cfg.predictorHistorySec", cfg.predictorHistorySec)
                .addData(p + ".fusedPose", fusedPose)
                .addData(p + ".lastPredictorPose", lastPredictorPose)
                .addData(p + ".lastCorrectionPose", lastCorrectionPose)
                .addData(p + ".lastLatencyCompensatedCorrectionPose", lastLatencyCompensatedCorrectionPose)
                .addData(p + ".lastReplayReferencePose", lastReplayReferencePose)
                .addData(p + ".lastCorrectionUsedReplay", lastCorrectionUsedReplay)
                .addData(p + ".replayBaseValid", replayBaseValid)
                .addData(p + ".replayBaseTimestamp", replayBaseTimestamp)
                .addData(p + ".replayBaseFusedPose", replayBaseFusedPose)
                .addData(p + ".replayBasePredictorPose", replayBasePredictorPose)
                .addData(p + ".predictorHistorySize", predictorHistory.size())
                .addData(p + ".lastUpdateCycle", lastUpdateCycle)
                .addData(p + ".lastUpdateFailed", lastUpdateFailure != null)
                .addData(p + ".lastCoveredPredictorMotionEndTimestamp",
                        lastCoveredPredictorMotionEndTimestamp)
                .addData(p + ".predictorMotionRebaseState", predictorMotionRebaseState)
                .addData(p + ".predictorMotionRebasePose", predictorMotionRebasePose)
                .addData(p + ".lastEstimate", lastEstimate);

        predictor.debugDump(dbg, p + ".predictor");
        correction.debugDump(dbg, p + ".correction");
    }

}
