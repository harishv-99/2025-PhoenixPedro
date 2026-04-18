package edu.ftcphoenix.fw.localization.fusion;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
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
        final double timestampSec;
        final Pose3d pose;

        PredictorSample(double timestampSec, Pose3d pose) {
            this.timestampSec = timestampSec;
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

    private PoseEstimate lastEstimate = PoseEstimate.noPose(0.0);
    private final Deque<PredictorSample> predictorHistory = new ArrayDeque<PredictorSample>();

    // Replay base: the most recent moment at which the fused pose was explicitly anchored
    // (initialization, manual reset, or accepted correction). Between replay bases, fused
    // motion is purely predictor-driven, so reconstructing the fused pose at a measurement timestamp
    // is just "base fused pose" composed with the predictor delta from base->measurement.
    private boolean replayBaseValid = false;
    private double replayBaseTimestampSec = Double.NaN;
    private Pose3d replayBaseFusedPose = Pose3d.zero();
    private Pose3d replayBasePredictorPose = Pose3d.zero();

    // Debug/telemetry helpers.
    private double lastCorrectionAcceptedSec = Double.NaN;
    private double lastAcceptedCorrectionMeasurementTimestampSec = Double.NaN;
    private double lastEvaluatedCorrectionTimestampSec = Double.NaN;
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
     * The timestamp (clock.nowSec) when correction was last accepted, or NaN if never.
     */
    public double getLastCorrectionAcceptedSec() {
        return lastCorrectionAcceptedSec;
    }

    /**
     * Measurement timestamp of the most recently accepted correction frame, or NaN if never.
     */
    public double getLastAcceptedCorrectionMeasurementTimestampSec() {
        return lastAcceptedCorrectionMeasurementTimestampSec;
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
                lastCorrectionAcceptedSec,
                lastAcceptedCorrectionMeasurementTimestampSec,
                lastEvaluatedCorrectionTimestampSec,
                lastCorrectionUsedReplay
        );
    }

    /**
     * Advances the corrected/global estimator one loop by propagating the predictor and then
     * conditionally applying the latest absolute correction measurement.
     */
    @Override
    public void update(LoopClock clock) {
        final double nowSec = clock != null ? clock.nowSec() : 0.0;

        // Update sources.
        predictor.update(clock);
        correction.update(clock);

        final PoseEstimate predictorEst = predictor.getEstimate();
        final MotionDelta predictorDelta = predictor.getLatestMotionDelta();
        final PoseEstimate correctionEst = correction.getEstimate();
        final Pose3d currentPredictorPose = (predictorEst != null && predictorEst.hasPose)
                ? planarize(predictorEst.fieldToRobotPose)
                : null;
        final double currentPredictorTimestampSec = estimateTimestampOr(predictorEst, nowSec);

        if (currentPredictorPose != null) {
            recordPredictorSample(currentPredictorTimestampSec, currentPredictorPose);
        }

        boolean evaluatedCorrectionThisLoop = false;

        // If we are not initialized yet, pick an initial pose.
        if (!initialized) {
            boolean initializedFromCorrection = false;
            if (correctionEnabled && cfg.enableInitializeFromCorrection && shouldEvaluateCorrectionMeasurement(correctionEst)) {
                evaluatedCorrectionThisLoop = true;
                if (isCorrectionAcceptable(correctionEst, nowSec)) {
                    Pose3d correctionPose = planarize(correctionEst.fieldToRobotPose);
                    Pose3d initialPose = cfg.enableLatencyCompensation
                            ? projectCorrectionPoseToNow(correctionPose, correctionEst.timestampSec, currentPredictorPose)
                            : correctionPose;

                    fusedPose = initialPose;
                    initialized = true;
                    lastCorrectionPose = correctionPose;
                    lastLatencyCompensatedCorrectionPose = initialPose;
                    lastReplayReferencePose = correctionPose;
                    lastCorrectionAcceptedSec = nowSec;
                    lastAcceptedCorrectionMeasurementTimestampSec = correctionEst.timestampSec;
                    lastCorrectionUsedReplay = false;
                    acceptedCorrectionCount++;
                    projectedCorrectionCount++;

                    boolean pushedToPredictor = pushFusedPoseToPredictor();
                    rebaseAfterPoseChange(nowSec, currentPredictorPose, pushedToPredictor);
                    initializedFromCorrection = true;
                } else {
                    rejectedCorrectionCount++;
                }
            }

            if (!initialized && currentPredictorPose != null) {
                fusedPose = currentPredictorPose;
                initialized = true;
                lastPredictorPose = currentPredictorPose;
                resetPredictorHistory(nowSec, currentPredictorPose);
                setReplayBase(nowSec, fusedPose, currentPredictorPose);
            } else if (!initialized && !initializedFromCorrection) {
                // No pose from either source yet.
                lastEstimate = PoseEstimate.noPose(nowSec);
                return;
            }
        } else {
            // Propagate fused pose using the predictor's explicit motion delta.
            if (predictorDelta != null && predictorDelta.hasDelta) {
                fusedPose = planarize(fusedPose.then(predictorDelta.deltaPose));
            }
            if (currentPredictorPose != null) {
                lastPredictorPose = currentPredictorPose;
            }
        }

        // Apply correction if available.
        if (correctionEnabled && !evaluatedCorrectionThisLoop && shouldEvaluateCorrectionMeasurement(correctionEst)) {
            if (!isCorrectionAcceptable(correctionEst, nowSec)) {
                rejectedCorrectionCount++;
            } else {
                maybeApplyCorrection(correctionEst, currentPredictorPose, nowSec);
            }
        }

        // Compute a simple "confidence" signal.
        double quality = (predictorEst != null && predictorEst.hasPose)
                ? MathUtil.clamp(predictorEst.quality, 0.0, 1.0)
                : 0.0;

        if (!Double.isNaN(lastCorrectionAcceptedSec)) {
            double age = nowSec - lastCorrectionAcceptedSec;
            if (age >= 0.0 && age < cfg.correctionConfidenceHoldSec) {
                double boost = 1.0 - (age / cfg.correctionConfidenceHoldSec);
                quality = MathUtil.clamp(Math.max(quality, boost), 0.0, 1.0);
            }
        }

        lastEstimate = new PoseEstimate(fusedPose, true, quality, 0.0, nowSec);
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

        final double nowSec = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec))
                ? lastEstimate.timestampSec
                : 0.0;

        fusedPose = new Pose3d(pose.xInches, pose.yInches, 0.0, MathUtil.wrapToPi(pose.headingRad), 0.0, 0.0);
        initialized = true;

        Pose3d currentPredictorPose = null;
        PoseEstimate predictorEst = predictor.getEstimate();
        if (predictorEst != null && predictorEst.hasPose) {
            currentPredictorPose = planarize(predictorEst.fieldToRobotPose);
        }

        boolean pushedToPredictor = pushFusedPoseToPredictor();
        rebaseAfterPoseChange(nowSec, currentPredictorPose, pushedToPredictor);
    }

    private boolean shouldEvaluateCorrectionMeasurement(PoseEstimate correctionEst) {
        if (correctionEst == null || !correctionEst.hasPose) {
            return false;
        }
        double ts = correctionEst.timestampSec;
        if (!Double.isFinite(ts)) {
            return false;
        }
        if (Double.isNaN(lastEvaluatedCorrectionTimestampSec)) {
            lastEvaluatedCorrectionTimestampSec = ts;
            return true;
        }
        if (ts > lastEvaluatedCorrectionTimestampSec + TIMESTAMP_EPS_SEC) {
            lastEvaluatedCorrectionTimestampSec = ts;
            return true;
        }
        if (Math.abs(ts - lastEvaluatedCorrectionTimestampSec) <= TIMESTAMP_EPS_SEC) {
            skippedDuplicateCorrectionCount++;
        } else {
            skippedOutOfOrderCorrectionCount++;
        }
        return false;
    }

    private boolean isCorrectionAcceptable(PoseEstimate correctionEst, double nowSec) {
        if (correctionEst == null || !correctionEst.hasPose || correctionEst.fieldToRobotPose == null) {
            return false;
        }
        if (!Double.isFinite(correctionEst.timestampSec)) {
            return false;
        }
        if (correctionEst.timestampSec > nowSec + TIMESTAMP_EPS_SEC) {
            return false;
        }

        // Freshness gate.
        if (cfg.maxCorrectionAgeSec > 0.0) {
            double age = nowSec - correctionEst.timestampSec;
            if (!Double.isFinite(age) || age < -TIMESTAMP_EPS_SEC || age > cfg.maxCorrectionAgeSec) {
                return false;
            }
        }

        // Quality gate.
        if (!Double.isFinite(correctionEst.quality) || correctionEst.quality < cfg.minCorrectionQuality) {
            return false;
        }

        // NaN gate.
        Pose3d p = correctionEst.fieldToRobotPose;
        return !(Double.isNaN(p.xInches) || Double.isNaN(p.yInches) || Double.isNaN(p.yawRad));
    }

    private void maybeApplyCorrection(PoseEstimate correctionEst, Pose3d currentPredictorPose, double nowSec) {
        Pose3d correctionPoseAtMeasurement = planarize(correctionEst.fieldToRobotPose);
        Pose3d compensatedCorrectionPoseAtNow = cfg.enableLatencyCompensation
                ? projectCorrectionPoseToNow(correctionPoseAtMeasurement, correctionEst.timestampSec, currentPredictorPose)
                : correctionPoseAtMeasurement;

        lastCorrectionPose = correctionPoseAtMeasurement;
        lastLatencyCompensatedCorrectionPose = compensatedCorrectionPoseAtNow;

        if (cfg.enableLatencyCompensation
                && replayBaseValid
                && Double.isFinite(replayBaseTimestampSec)
                && correctionEst.timestampSec + TIMESTAMP_EPS_SEC < replayBaseTimestampSec) {
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
            Pose3d predictorAtMeasurement = interpolatePredictorPose(correctionEst.timestampSec);
            if (predictorAtMeasurement != null) {
                Pose3d fusedAtMeasurement = reconstructFusedPoseAt(correctionEst.timestampSec, predictorAtMeasurement);
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
        lastCorrectionAcceptedSec = nowSec;
        lastAcceptedCorrectionMeasurementTimestampSec = correctionEst.timestampSec;
        lastCorrectionUsedReplay = usedReplay;
        acceptedCorrectionCount++;
        if (usedReplay) {
            replayedCorrectionCount++;
        } else {
            projectedCorrectionCount++;
        }

        boolean pushedToPredictor = pushFusedPoseToPredictor();
        rebaseAfterPoseChange(nowSec, currentPredictorPose, pushedToPredictor);
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

    private void rebaseAfterPoseChange(double nowSec, Pose3d currentPredictorPose, boolean pushedToPredictor) {
        Pose3d basePredictorPose;
        if (pushedToPredictor) {
            lastPredictorPose = fusedPose;
            basePredictorPose = fusedPose;
        } else if (currentPredictorPose != null) {
            lastPredictorPose = currentPredictorPose;
            basePredictorPose = currentPredictorPose;
        } else {
            lastPredictorPose = fusedPose;
            basePredictorPose = fusedPose;
        }

        resetPredictorHistory(nowSec, basePredictorPose);
        setReplayBase(nowSec, fusedPose, basePredictorPose);
    }

    private void setReplayBase(double timestampSec, Pose3d fusedPoseAtBase, Pose3d predictorPoseAtBase) {
        replayBaseValid = Double.isFinite(timestampSec) && fusedPoseAtBase != null && predictorPoseAtBase != null;
        replayBaseTimestampSec = replayBaseValid ? timestampSec : Double.NaN;
        replayBaseFusedPose = replayBaseValid ? planarize(fusedPoseAtBase) : Pose3d.zero();
        replayBasePredictorPose = replayBaseValid ? planarize(predictorPoseAtBase) : Pose3d.zero();
    }

    private Pose3d reconstructFusedPoseAt(double timestampSec, Pose3d predictorPoseAtTimestamp) {
        if (!replayBaseValid || predictorPoseAtTimestamp == null) {
            return null;
        }
        if (!Double.isFinite(timestampSec) || !Double.isFinite(replayBaseTimestampSec)) {
            return null;
        }
        if (timestampSec + TIMESTAMP_EPS_SEC < replayBaseTimestampSec) {
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

    private static double estimateTimestampOr(PoseEstimate est, double fallbackNowSec) {
        if (est != null && Double.isFinite(est.timestampSec)) {
            return est.timestampSec;
        }
        return fallbackNowSec;
    }

    private void recordPredictorSample(double timestampSec, Pose3d predictorPose) {
        if (!Double.isFinite(timestampSec) || predictorPose == null) {
            return;
        }

        PredictorSample last = predictorHistory.peekLast();
        if (last != null && timestampSec <= last.timestampSec) {
            predictorHistory.clear();
        }

        predictorHistory.addLast(new PredictorSample(timestampSec, planarize(predictorPose)));
        prunePredictorHistory(timestampSec);
    }

    private void prunePredictorHistory(double nowSec) {
        double keepSec = Math.max(0.0, cfg.predictorHistorySec);
        if (keepSec <= 0.0) {
            while (predictorHistory.size() > 1) {
                predictorHistory.removeFirst();
            }
            return;
        }

        double minTime = nowSec - keepSec;
        while (predictorHistory.size() > 2) {
            PredictorSample[] samples = predictorHistory.toArray(new PredictorSample[0]);
            if (samples.length < 2 || samples[1].timestampSec >= minTime) {
                break;
            }
            predictorHistory.removeFirst();
        }
    }

    private void resetPredictorHistory(double timestampSec, Pose3d pose) {
        predictorHistory.clear();
        if (Double.isFinite(timestampSec) && pose != null) {
            predictorHistory.addLast(new PredictorSample(timestampSec, planarize(pose)));
        }
    }

    private Pose3d projectCorrectionPoseToNow(Pose3d correctionPoseAtMeasurement,
                                              double measurementTimestampSec,
                                              Pose3d currentPredictorPose) {
        if (correctionPoseAtMeasurement == null || currentPredictorPose == null) {
            return correctionPoseAtMeasurement;
        }
        if (!Double.isFinite(measurementTimestampSec)) {
            return correctionPoseAtMeasurement;
        }

        Pose3d predictorAtMeasurement = interpolatePredictorPose(measurementTimestampSec);
        if (predictorAtMeasurement == null) {
            return correctionPoseAtMeasurement;
        }

        Pose3d predictorDeltaSinceMeasurement = predictorAtMeasurement.inverse().then(currentPredictorPose);
        return planarize(correctionPoseAtMeasurement.then(predictorDeltaSinceMeasurement));
    }

    private Pose3d interpolatePredictorPose(double timestampSec) {
        if (predictorHistory.isEmpty()) {
            return null;
        }

        PredictorSample[] samples = predictorHistory.toArray(new PredictorSample[0]);
        if (samples.length == 0) {
            return null;
        }
        if (timestampSec < samples[0].timestampSec || timestampSec > samples[samples.length - 1].timestampSec) {
            return null;
        }
        if (samples.length == 1) {
            return samples[0].pose;
        }

        PredictorSample prev = samples[0];
        for (PredictorSample next : samples) {
            if (next.timestampSec < timestampSec) {
                prev = next;
                continue;
            }
            if (Math.abs(next.timestampSec - timestampSec) <= 1e-9 || next == prev) {
                return next.pose;
            }

            double dt = next.timestampSec - prev.timestampSec;
            if (dt <= 1e-9) {
                return next.pose;
            }

            double t = MathUtil.clamp01((timestampSec - prev.timestampSec) / dt);
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
                .addData(p + ".lastCorrectionAcceptedSec", lastCorrectionAcceptedSec)
                .addData(p + ".lastAcceptedCorrectionMeasurementTimestampSec", lastAcceptedCorrectionMeasurementTimestampSec)
                .addData(p + ".lastEvaluatedCorrectionTimestampSec", lastEvaluatedCorrectionTimestampSec)
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
                .addData(p + ".replayBaseTimestampSec", replayBaseTimestampSec)
                .addData(p + ".replayBaseFusedPose", replayBaseFusedPose)
                .addData(p + ".replayBasePredictorPose", replayBasePredictorPose)
                .addData(p + ".predictorHistorySize", predictorHistory.size())
                .addData(p + ".lastEstimate", lastEstimate);

        predictor.debugDump(dbg, p + ".predictor");
        correction.debugDump(dbg, p + ".correction");
    }

}
