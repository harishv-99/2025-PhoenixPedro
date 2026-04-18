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
 * Optional covariance-aware global localizer that combines smooth predictor with occasional absolute
 * corrections using a small planar EKF-style model.
 *
 * <p>This estimator exists alongside {@link OdometryCorrectionFusionEstimator}; it does <em>not</em>
 * replace the simpler complementary localizer. Teams that want a lightweight, easier-to-tune stack
 * should still start with {@link OdometryCorrectionFusionEstimator}. Teams that want an optional,
 * uncertainty-aware alternative can choose this class intentionally.</p>
 *
 * <h2>Model shape</h2>
 *
 * <ul>
 *   <li>State: planar {@code field -> robot} pose ({@code x}, {@code y}, {@code heading}).</li>
 *   <li>Prediction: predictor deltas are composed onto the current pose, while covariance grows as a
 *       function of translational / rotational motion.</li>
 *   <li>Measurement: the supplied correction estimator is treated as an absolute pose measurement with a
 *       dynamically sized measurement covariance derived from the correction estimate's quality.</li>
 * </ul>
 *
 * <h2>Calibration and reliability notes</h2>
 *
 * <p>This estimator is more sophisticated than the lightweight fusion localizer, but it is also more
 * sensitive to bad assumptions. It does <b>not</b> rescue poor calibration. Before enabling it on a
 * real robot, you should already have:</p>
 *
 * <ol>
 *   <li>a calibrated {@code robot -> camera} mount,</li>
 *   <li>accurate predictor geometry / Pinpoint pod offsets, and</li>
 *   <li>a fixed-tag layout that excludes non-deterministic tags.</li>
 * </ol>
 *
 * <p>If those inputs are wrong, an EKF can produce a very smooth but still-wrong pose. That is why
 * Phoenix keeps this estimator optional and documents it separately instead of silently replacing the
 * simpler fusion implementation.</p>
 *
 * <h2>Latency compensation</h2>
 *
 * <p>When predictor history is available, accepted correction measurements are applied at their
 * measurement timestamp and the filter is replayed forward through stored predictor motion. If exact
 * replay is unavailable, the estimator falls back to a projected-now update path rather than
 * pretending the delayed measurement was captured at the current loop time.</p>
 */
public final class OdometryCorrectionEkfEstimator implements CorrectedPoseEstimator {

    private static final double TIMESTAMP_EPS_SEC = 1e-6;
    private static final double MIN_VARIANCE = 1e-9;

    /**
     * Configuration for {@link OdometryCorrectionEkfEstimator}.
     *
     * <p>The defaults are intentionally conservative and should be treated as a starting point, not
     * as a substitute for real calibration. In particular, the predictor process-noise terms and the
     * correction measurement-noise terms only make sense once camera extrinsics and predictor geometry are
     * already trustworthy.</p>
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
         * Hard gate on planar innovation magnitude before the EKF update is even attempted.
         *
         * <p>This is a simple reliability guardrail: a wildly contradictory measurement should not be
         * allowed to rely solely on covariance math for rejection.</p>
         */
        public double maxCorrectionPositionInnovationIn = 24.0;

        /**
         * Hard gate on heading innovation magnitude before the EKF update is attempted.
         */
        public double maxCorrectionHeadingInnovationRad = Math.toRadians(60.0);

        /**
         * Maximum allowed Mahalanobis distance squared for a correction innovation.
         *
         * <p>Higher values make the filter more permissive; lower values reject more contradictory
         * measurements. This is a reliability gate, not a scoring function.</p>
         */
        public double maxCorrectionMahalanobisSq = 25.0;

        /**
         * If true, the filter may initialize from a fresh correction measurement.
         */
        public boolean enableInitializeFromCorrection = true;

        /**
         * If true, accepted filtered poses are pushed back into the predictor estimator when it
         * supports {@link PoseResetter}. This keeps predictor and the filtered state aligned.
         */
        public boolean enablePushCorrectedPoseToPredictor = true;

        /**
         * If true, accepted correction measurements are updated at their measurement timestamp and the
         * predictor prediction is replayed forward to the current loop when history is available.
         */
        public boolean enableLatencyCompensation = true;

        /**
         * How much recent predictor history (seconds) to retain for measurement-time replay.
         *
         * <p>When latency compensation is enabled, this should be at least as large as
         * {@link #maxCorrectionAgeSec} so every still-acceptable correction frame can be replayed through
         * the stored predictor history.</p>
         */
        public double predictorHistorySec = 1.0;

        /**
         * Initial planar position standard deviation (inches) when the filter initializes from
         * predictor alone before any absolute correction has been accepted.
         */
        public double initialPositionStdIn = 18.0;

        /**
         * Initial heading standard deviation (radians) when the filter initializes from predictor
         * alone before any absolute correction has been accepted.
         */
        public double initialHeadingStdRad = Math.toRadians(45.0);

        /**
         * Position standard deviation (inches) assigned after an explicit manual pose anchor via
         * {@link #setPose(Pose2d)}.
         */
        public double manualPosePositionStdIn = 2.0;

        /**
         * Heading standard deviation (radians) assigned after an explicit manual pose anchor via
         * {@link #setPose(Pose2d)}.
         */
        public double manualPoseHeadingStdRad = Math.toRadians(6.0);

        /**
         * Base translational process-noise standard deviation added on every predictor predict step.
         */
        public double predictorProcessPositionStdFloorIn = 0.10;
        /**
         * Extra translational process-noise standard deviation per inch of predictor travel.
         */
        public double predictorProcessPositionStdPerIn = 0.025;
        /**
         * Extra translational process-noise standard deviation per radian of predictor heading change.
         */
        public double predictorProcessPositionStdPerRad = 0.50;

        /**
         * Base heading process-noise standard deviation added on every predictor predict step.
         */
        public double predictorProcessHeadingStdFloorRad = Math.toRadians(0.25);
        /**
         * Extra heading process-noise standard deviation per inch of predictor travel.
         */
        public double predictorProcessHeadingStdPerIn = Math.toRadians(0.08);
        /**
         * Extra heading process-noise standard deviation per radian of predictor heading change.
         */
        public double predictorProcessHeadingStdPerRad = 0.05;

        /**
         * Best-case planar position measurement standard deviation (inches) for a high-quality correction pose.
         */
        public double correctionPositionStdFloorIn = 1.25;
        /**
         * Additional planar position measurement standard deviation (inches) added as correction quality falls toward 0.
         */
        public double correctionPositionStdScaleIn = 10.0;
        /**
         * Additional planar position standard deviation (inches/sec) when a delayed frame must be projected to "now" instead of replayed.
         */
        public double projectedCorrectionPositionStdPerSec = 4.0;

        /**
         * Best-case heading measurement standard deviation (radians) for a high-quality correction pose.
         */
        public double correctionHeadingStdFloorRad = Math.toRadians(2.0);
        /**
         * Additional heading measurement standard deviation (radians) added as correction quality falls toward 0.
         */
        public double correctionHeadingStdScaleRad = Math.toRadians(18.0);
        /**
         * Additional heading standard deviation (radians/sec) when a delayed frame must be projected to "now" instead of replayed.
         */
        public double projectedCorrectionHeadingStdPerSec = Math.toRadians(12.0);

        /**
         * Position-uncertainty scale used when turning filter covariance into {@link PoseEstimate#quality}.
         */
        public double qualityPositionStdScaleIn = 24.0;
        /**
         * Heading-uncertainty scale used when turning filter covariance into {@link PoseEstimate#quality}.
         */
        public double qualityHeadingStdScaleRad = Math.toRadians(45.0);

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Returns a fresh config initialized with Phoenix defaults.
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
                    : "OdometryCorrectionEkfEstimator.Config";

            requireFiniteNonNegative(maxCorrectionAgeSec, p + ".maxCorrectionAgeSec");
            requireFiniteInRange(minCorrectionQuality, 0.0, 1.0, p + ".minCorrectionQuality");
            requireFinitePositive(maxCorrectionPositionInnovationIn, p + ".maxCorrectionPositionInnovationIn");
            requireFinitePositive(maxCorrectionHeadingInnovationRad, p + ".maxCorrectionHeadingInnovationRad");
            requireFinitePositive(maxCorrectionMahalanobisSq, p + ".maxCorrectionMahalanobisSq");
            requireFiniteNonNegative(predictorHistorySec, p + ".predictorHistorySec");
            requireFinitePositive(initialPositionStdIn, p + ".initialPositionStdIn");
            requireFinitePositive(initialHeadingStdRad, p + ".initialHeadingStdRad");
            requireFinitePositive(manualPosePositionStdIn, p + ".manualPosePositionStdIn");
            requireFinitePositive(manualPoseHeadingStdRad, p + ".manualPoseHeadingStdRad");
            requireFinitePositive(predictorProcessPositionStdFloorIn, p + ".predictorProcessPositionStdFloorIn");
            requireFiniteNonNegative(predictorProcessPositionStdPerIn, p + ".predictorProcessPositionStdPerIn");
            requireFiniteNonNegative(predictorProcessPositionStdPerRad, p + ".predictorProcessPositionStdPerRad");
            requireFinitePositive(predictorProcessHeadingStdFloorRad, p + ".predictorProcessHeadingStdFloorRad");
            requireFiniteNonNegative(predictorProcessHeadingStdPerIn, p + ".predictorProcessHeadingStdPerIn");
            requireFiniteNonNegative(predictorProcessHeadingStdPerRad, p + ".predictorProcessHeadingStdPerRad");
            requireFinitePositive(correctionPositionStdFloorIn, p + ".correctionPositionStdFloorIn");
            requireFiniteNonNegative(correctionPositionStdScaleIn, p + ".correctionPositionStdScaleIn");
            requireFiniteNonNegative(projectedCorrectionPositionStdPerSec, p + ".projectedCorrectionPositionStdPerSec");
            requireFinitePositive(correctionHeadingStdFloorRad, p + ".correctionHeadingStdFloorRad");
            requireFiniteNonNegative(correctionHeadingStdScaleRad, p + ".correctionHeadingStdScaleRad");
            requireFiniteNonNegative(projectedCorrectionHeadingStdPerSec, p + ".projectedCorrectionHeadingStdPerSec");
            requireFinitePositive(qualityPositionStdScaleIn, p + ".qualityPositionStdScaleIn");
            requireFinitePositive(qualityHeadingStdScaleRad, p + ".qualityHeadingStdScaleRad");

            if (enableLatencyCompensation && predictorHistorySec + TIMESTAMP_EPS_SEC < maxCorrectionAgeSec) {
                throw new IllegalArgumentException(
                        p + ".predictorHistorySec must be >= maxCorrectionAgeSec when latency compensation is enabled"
                                + " (increase predictorHistorySec or reduce maxCorrectionAgeSec)"
                );
            }
        }

        /**
         * Returns a deep copy of this config.
         */
        public Config copy() {
            Config c = new Config();
            c.maxCorrectionAgeSec = this.maxCorrectionAgeSec;
            c.minCorrectionQuality = this.minCorrectionQuality;
            c.maxCorrectionPositionInnovationIn = this.maxCorrectionPositionInnovationIn;
            c.maxCorrectionHeadingInnovationRad = this.maxCorrectionHeadingInnovationRad;
            c.maxCorrectionMahalanobisSq = this.maxCorrectionMahalanobisSq;
            c.enableInitializeFromCorrection = this.enableInitializeFromCorrection;
            c.enablePushCorrectedPoseToPredictor = this.enablePushCorrectedPoseToPredictor;
            c.enableLatencyCompensation = this.enableLatencyCompensation;
            c.predictorHistorySec = this.predictorHistorySec;
            c.initialPositionStdIn = this.initialPositionStdIn;
            c.initialHeadingStdRad = this.initialHeadingStdRad;
            c.manualPosePositionStdIn = this.manualPosePositionStdIn;
            c.manualPoseHeadingStdRad = this.manualPoseHeadingStdRad;
            c.predictorProcessPositionStdFloorIn = this.predictorProcessPositionStdFloorIn;
            c.predictorProcessPositionStdPerIn = this.predictorProcessPositionStdPerIn;
            c.predictorProcessPositionStdPerRad = this.predictorProcessPositionStdPerRad;
            c.predictorProcessHeadingStdFloorRad = this.predictorProcessHeadingStdFloorRad;
            c.predictorProcessHeadingStdPerIn = this.predictorProcessHeadingStdPerIn;
            c.predictorProcessHeadingStdPerRad = this.predictorProcessHeadingStdPerRad;
            c.correctionPositionStdFloorIn = this.correctionPositionStdFloorIn;
            c.correctionPositionStdScaleIn = this.correctionPositionStdScaleIn;
            c.projectedCorrectionPositionStdPerSec = this.projectedCorrectionPositionStdPerSec;
            c.correctionHeadingStdFloorRad = this.correctionHeadingStdFloorRad;
            c.correctionHeadingStdScaleRad = this.correctionHeadingStdScaleRad;
            c.projectedCorrectionHeadingStdPerSec = this.projectedCorrectionHeadingStdPerSec;
            c.qualityPositionStdScaleIn = this.qualityPositionStdScaleIn;
            c.qualityHeadingStdScaleRad = this.qualityHeadingStdScaleRad;
            return c;
        }

        /**
         * Returns a validated copy of this config.
         */
        public Config validatedCopy(String context) {
            Config c = copy();
            c.validate(context);
            return c;
        }

        private static void requireFiniteNonNegative(double value, String name) {
            if (!Double.isFinite(value) || value < 0.0) {
                throw new IllegalArgumentException(name + " must be finite and >= 0");
            }
        }

        private static void requireFinitePositive(double value, String name) {
            if (!Double.isFinite(value) || value <= 0.0) {
                throw new IllegalArgumentException(name + " must be finite and > 0");
            }
        }

        private static void requireFiniteInRange(double value, double lo, double hi, String name) {
            if (!Double.isFinite(value) || value < lo || value > hi) {
                throw new IllegalArgumentException(name + " must be finite and within [" + lo + ", " + hi + "]");
            }
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

    private static final class StateSnapshot {
        final Pose3d pose;
        final double[][] covariance;
        final Pose3d predictorPose;
        final double timestampSec;

        StateSnapshot(Pose3d pose, double[][] covariance, Pose3d predictorPose, double timestampSec) {
            this.pose = pose;
            this.covariance = covariance;
            this.predictorPose = predictorPose;
            this.timestampSec = timestampSec;
        }
    }

    private final MotionPredictor predictor;
    private final AbsolutePoseEstimator correction;
    private final Config cfg;

    private boolean initialized = false;
    private boolean correctionEnabled = true;

    private Pose3d statePose = Pose3d.zero();
    private double[][] stateCovariance = diagonal(1.0, 1.0, 1.0);
    private Pose3d lastPredictorPose = Pose3d.zero();

    private PoseEstimate lastEstimate = PoseEstimate.noPose(0.0);
    private final Deque<PredictorSample> predictorHistory = new ArrayDeque<PredictorSample>();

    private boolean replayBaseValid = false;
    private double replayBaseTimestampSec = Double.NaN;
    private Pose3d replayBasePose = Pose3d.zero();
    private double[][] replayBaseCovariance = diagonal(1.0, 1.0, 1.0);
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

    private double lastInnovationPositionIn = Double.NaN;
    private double lastInnovationHeadingRad = Double.NaN;
    private double lastInnovationMahalanobisSq = Double.NaN;
    private double lastMeasurementPositionStdIn = Double.NaN;
    private double lastMeasurementHeadingStdRad = Double.NaN;

    /**
     * Creates an EKF-style localizer with default configuration.
     */
    public OdometryCorrectionEkfEstimator(MotionPredictor predictor, AbsolutePoseEstimator correction) {
        this(predictor, correction, Config.defaults());
    }

    /**
     * Creates an EKF-style localizer with explicit configuration.
     */
    public OdometryCorrectionEkfEstimator(MotionPredictor predictor, AbsolutePoseEstimator correction, Config cfg) {
        if (predictor == null) {
            throw new IllegalArgumentException("predictor must not be null");
        }
        if (correction == null) {
            throw new IllegalArgumentException("correction must not be null");
        }
        this.predictor = predictor;
        this.correction = correction;
        Config base = (cfg != null) ? cfg : Config.defaults();
        this.cfg = base.validatedCopy("OdometryCorrectionEkfEstimator.Config");
    }

    /**
     * Enables or disables absolute corrections while leaving predictor propagation alive.
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
     * Returns the current planar position standard deviation implied by the filter covariance.
     */
    public double getPositionStdIn() {
        return positionStdIn(stateCovariance);
    }

    /**
     * Returns the current heading standard deviation implied by the filter covariance.
     */
    public double getHeadingStdRad() {
        return headingStdRad(stateCovariance);
    }

    /**
     * Returns the most recent innovation position magnitude (inches), or {@code NaN} if none.
     */
    public double getLastInnovationPositionIn() {
        return lastInnovationPositionIn;
    }

    /**
     * Returns the most recent innovation heading magnitude (radians), or {@code NaN} if none.
     */
    public double getLastInnovationHeadingRad() {
        return lastInnovationHeadingRad;
    }

    /**
     * Returns the most recent innovation Mahalanobis distance squared, or {@code NaN} if none.
     */
    public double getLastInnovationMahalanobisSq() {
        return lastInnovationMahalanobisSq;
    }

    /**
     * Returns the most recent position measurement standard deviation (inches), or {@code NaN} if none.
     */
    public double getLastMeasurementPositionStdIn() {
        return lastMeasurementPositionStdIn;
    }

    /**
     * Returns the most recent heading measurement standard deviation (radians), or {@code NaN} if none.
     */
    public double getLastMeasurementHeadingStdRad() {
        return lastMeasurementHeadingStdRad;
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
     * Returns whether the most recently accepted correction used measurement-time replay.
     */
    public boolean wasLastCorrectionCorrectionReplay() {
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
     * Returns how many duplicate frame timestamps were skipped.
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
     * Returns how many accepted corrections fell back to a simple projected-now path.
     */
    public int getProjectedCorrectionCount() {
        return projectedCorrectionCount;
    }

    /**
     * Returns aggregate correction counters and timestamps useful for EKF telemetry and tuning.
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
     * Advances the EKF one loop by propagating predictor motion and conditionally applying the
     * latest absolute correction measurement.
     */
    @Override
    public void update(LoopClock clock) {
        final double nowSec = clock != null ? clock.nowSec() : 0.0;

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

        if (!initialized) {
            boolean initializedFromCorrection = false;
            if (correctionEnabled && cfg.enableInitializeFromCorrection && shouldEvaluateCorrectionMeasurement(correctionEst)) {
                evaluatedCorrectionThisLoop = true;
                if (isCorrectionAcceptable(correctionEst, nowSec)) {
                    initializeFromCorrection(correctionEst, currentPredictorPose, currentPredictorTimestampSec, nowSec);
                    initializedFromCorrection = initialized;
                } else {
                    rejectedCorrectionCount++;
                }
            }

            if (!initialized && currentPredictorPose != null) {
                statePose = currentPredictorPose;
                stateCovariance = initialPredictorCovariance();
                initialized = true;
                lastPredictorPose = currentPredictorPose;
                resetPredictorHistory(currentPredictorTimestampSec, currentPredictorPose);
                setReplayBase(currentPredictorTimestampSec, statePose, stateCovariance, currentPredictorPose);
            } else if (!initialized && !initializedFromCorrection) {
                lastEstimate = PoseEstimate.noPose(nowSec);
                return;
            }
        } else {
            if (predictorDelta != null && predictorDelta.hasDelta && currentPredictorPose != null) {
                StateSnapshot predicted = predictStep(statePose, stateCovariance, predictorDelta.deltaPose, currentPredictorPose, currentPredictorTimestampSec);
                statePose = predicted.pose;
                stateCovariance = predicted.covariance;
            }
            if (currentPredictorPose != null) {
                lastPredictorPose = currentPredictorPose;
            }
        }

        if (correctionEnabled && !evaluatedCorrectionThisLoop && shouldEvaluateCorrectionMeasurement(correctionEst)) {
            if (!isCorrectionAcceptable(correctionEst, nowSec)) {
                rejectedCorrectionCount++;
            } else {
                maybeApplyCorrection(correctionEst, currentPredictorPose, currentPredictorTimestampSec, nowSec);
            }
        }

        lastEstimate = new PoseEstimate(statePose, true, covarianceQuality(stateCovariance), 0.0, nowSec);
    }

    /**
     * Returns the most recent corrected/global pose estimate produced by the EKF.
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * Manually anchors the EKF to a known field pose and resets its covariance to the configured
     * manual-anchor uncertainty.
     */
    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            return;
        }

        final double nowSec = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec))
                ? lastEstimate.timestampSec
                : 0.0;

        statePose = new Pose3d(pose.xInches, pose.yInches, 0.0, MathUtil.wrapToPi(pose.headingRad), 0.0, 0.0);
        stateCovariance = manualAnchorCovariance();
        initialized = true;

        Pose3d currentPredictorPose = null;
        double currentPredictorTimestampSec = nowSec;
        PoseEstimate predictorEst = predictor.getEstimate();
        if (predictorEst != null && predictorEst.hasPose) {
            currentPredictorPose = planarize(predictorEst.fieldToRobotPose);
            currentPredictorTimestampSec = estimateTimestampOr(predictorEst, nowSec);
        }

        boolean pushedToPredictor = pushFilteredPoseToPredictor();
        clearRecentCorrectionState();
        rebaseAfterPoseChange(currentPredictorTimestampSec, currentPredictorPose, pushedToPredictor);
        lastEstimate = new PoseEstimate(statePose, true, covarianceQuality(stateCovariance), 0.0, nowSec);
    }

    private void initializeFromCorrection(PoseEstimate correctionEst,
                                          Pose3d currentPredictorPose,
                                          double currentPredictorTimestampSec,
                                          double nowSec) {
        Pose3d correctionPoseAtMeasurement = planarize(correctionEst.fieldToRobotPose);
        lastCorrectionPose = correctionPoseAtMeasurement;
        lastReplayReferencePose = correctionPoseAtMeasurement;
        lastInnovationPositionIn = Double.NaN;
        lastInnovationHeadingRad = Double.NaN;
        lastInnovationMahalanobisSq = Double.NaN;

        StateSnapshot currentState = null;
        boolean usedReplay = false;
        if (cfg.enableLatencyCompensation) {
            double[][] measCov = measurementCovariance(correctionEst.quality, false, 0.0);
            currentState = propagateFromArbitraryState(
                    correctionPoseAtMeasurement,
                    measCov,
                    correctionEst.timestampSec,
                    currentPredictorTimestampSec
            );
            usedReplay = currentState != null && currentPredictorTimestampSec > correctionEst.timestampSec + TIMESTAMP_EPS_SEC;
        }

        if (currentState == null) {
            Pose3d projectedCorrectionPoseNow = projectCorrectionPoseToNow(correctionPoseAtMeasurement, correctionEst.timestampSec, currentPredictorPose);
            double projectedAgeSec = Math.max(0.0, nowSec - correctionEst.timestampSec);
            statePose = projectedCorrectionPoseNow;
            stateCovariance = measurementCovariance(correctionEst.quality, true, projectedAgeSec);
            lastLatencyCompensatedCorrectionPose = projectedCorrectionPoseNow;
            lastCorrectionUsedReplay = false;
            projectedCorrectionCount++;
        } else {
            statePose = currentState.pose;
            stateCovariance = currentState.covariance;
            lastLatencyCompensatedCorrectionPose = currentState.pose;
            lastCorrectionUsedReplay = usedReplay;
            if (usedReplay) {
                replayedCorrectionCount++;
            } else {
                projectedCorrectionCount++;
            }
        }

        initialized = true;
        lastCorrectionAcceptedSec = nowSec;
        lastAcceptedCorrectionMeasurementTimestampSec = correctionEst.timestampSec;
        acceptedCorrectionCount++;

        boolean pushedToPredictor = pushFilteredPoseToPredictor();
        rebaseAfterPoseChange(currentPredictorTimestampSec, currentPredictorPose, pushedToPredictor);
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

        if (cfg.maxCorrectionAgeSec > 0.0) {
            double age = nowSec - correctionEst.timestampSec;
            if (!Double.isFinite(age) || age < -TIMESTAMP_EPS_SEC || age > cfg.maxCorrectionAgeSec) {
                return false;
            }
        }

        if (!Double.isFinite(correctionEst.quality) || correctionEst.quality < cfg.minCorrectionQuality) {
            return false;
        }

        Pose3d p = correctionEst.fieldToRobotPose;
        return !(Double.isNaN(p.xInches) || Double.isNaN(p.yInches) || Double.isNaN(p.yawRad));
    }

    private void maybeApplyCorrection(PoseEstimate correctionEst,
                                      Pose3d currentPredictorPose,
                                      double currentPredictorTimestampSec,
                                      double nowSec) {
        Pose3d correctionPoseAtMeasurement = planarize(correctionEst.fieldToRobotPose);
        Pose3d projectedCorrectionPoseAtNow = projectCorrectionPoseToNow(correctionPoseAtMeasurement, correctionEst.timestampSec, currentPredictorPose);

        lastCorrectionPose = correctionPoseAtMeasurement;
        lastLatencyCompensatedCorrectionPose = projectedCorrectionPoseAtNow;

        if (cfg.enableLatencyCompensation
                && replayBaseValid
                && Double.isFinite(replayBaseTimestampSec)
                && correctionEst.timestampSec + TIMESTAMP_EPS_SEC < replayBaseTimestampSec) {
            lastReplayReferencePose = replayBasePose;
            lastCorrectionUsedReplay = false;
            rejectedCorrectionCount++;
            return;
        }

        StateSnapshot correctedNow = null;
        boolean usedReplay = false;

        if (cfg.enableLatencyCompensation) {
            StateSnapshot predictedAtMeasurement = predictFromReplayBaseTo(correctionEst.timestampSec);
            if (predictedAtMeasurement != null) {
                lastReplayReferencePose = predictedAtMeasurement.pose;
                StateSnapshot correctedAtMeasurement = measurementUpdate(
                        predictedAtMeasurement.pose,
                        predictedAtMeasurement.covariance,
                        correctionPoseAtMeasurement,
                        correctionEst.quality,
                        false,
                        0.0,
                        predictedAtMeasurement.predictorPose,
                        correctionEst.timestampSec
                );
                if (correctedAtMeasurement != null) {
                    StateSnapshot replayed = propagateFromArbitraryState(
                            correctedAtMeasurement.pose,
                            correctedAtMeasurement.covariance,
                            correctionEst.timestampSec,
                            currentPredictorTimestampSec
                    );
                    correctedNow = (replayed != null) ? replayed : correctedAtMeasurement;
                    usedReplay = true;
                } else {
                    rejectedCorrectionCount++;
                    lastCorrectionUsedReplay = true;
                    return;
                }
            }
        }

        if (correctedNow == null) {
            lastReplayReferencePose = statePose;
            double projectedAgeSec = Math.max(0.0, nowSec - correctionEst.timestampSec);
            correctedNow = measurementUpdate(
                    statePose,
                    stateCovariance,
                    projectedCorrectionPoseAtNow,
                    correctionEst.quality,
                    true,
                    projectedAgeSec,
                    currentPredictorPose,
                    currentPredictorTimestampSec
            );
            if (correctedNow == null) {
                rejectedCorrectionCount++;
                lastCorrectionUsedReplay = false;
                return;
            }
        }

        statePose = correctedNow.pose;
        stateCovariance = correctedNow.covariance;
        lastCorrectionAcceptedSec = nowSec;
        lastAcceptedCorrectionMeasurementTimestampSec = correctionEst.timestampSec;
        lastCorrectionUsedReplay = usedReplay;
        acceptedCorrectionCount++;
        if (usedReplay) {
            replayedCorrectionCount++;
        } else {
            projectedCorrectionCount++;
        }

        boolean pushedToPredictor = pushFilteredPoseToPredictor();
        rebaseAfterPoseChange(currentPredictorTimestampSec, currentPredictorPose, pushedToPredictor);
    }

    private StateSnapshot predictFromReplayBaseTo(double timestampSec) {
        if (!replayBaseValid) {
            return null;
        }
        return propagateFromArbitraryState(
                replayBasePose,
                replayBaseCovariance,
                replayBaseTimestampSec,
                timestampSec
        );
    }

    private StateSnapshot measurementUpdate(Pose3d priorPose,
                                            double[][] priorCovariance,
                                            Pose3d measurementPose,
                                            double measurementQuality,
                                            boolean projectedMeasurement,
                                            double projectedAgeSec,
                                            Pose3d predictorPose,
                                            double timestampSec) {
        if (priorPose == null || priorCovariance == null || measurementPose == null) {
            return null;
        }

        double[][] R = measurementCovariance(measurementQuality, projectedMeasurement, projectedAgeSec);
        lastMeasurementPositionStdIn = Math.sqrt(Math.max(MIN_VARIANCE, R[0][0]));
        lastMeasurementHeadingStdRad = Math.sqrt(Math.max(MIN_VARIANCE, R[2][2]));

        double dx = measurementPose.xInches - priorPose.xInches;
        double dy = measurementPose.yInches - priorPose.yInches;
        double dHeading = MathUtil.wrapToPi(measurementPose.yawRad - priorPose.yawRad);
        double innovationPosition = Math.hypot(dx, dy);
        double innovationHeading = Math.abs(dHeading);

        lastInnovationPositionIn = innovationPosition;
        lastInnovationHeadingRad = innovationHeading;

        if (innovationPosition > cfg.maxCorrectionPositionInnovationIn
                || innovationHeading > cfg.maxCorrectionHeadingInnovationRad) {
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }

        double[] innovation = new double[]{dx, dy, dHeading};
        double[][] S = add(priorCovariance, R);
        double[][] sInv = invert3x3(S);
        if (sInv == null) {
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }

        double mahaSq = quadraticForm(innovation, sInv);
        lastInnovationMahalanobisSq = mahaSq;
        if (!Double.isFinite(mahaSq) || mahaSq > cfg.maxCorrectionMahalanobisSq) {
            return null;
        }

        double[][] K = mul(priorCovariance, sInv);
        double[] correction = mul(K, innovation);

        Pose3d updatedPose = new Pose3d(
                priorPose.xInches + correction[0],
                priorPose.yInches + correction[1],
                0.0,
                MathUtil.wrapToPi(priorPose.yawRad + correction[2]),
                0.0,
                0.0
        );

        double[][] I = identity();
        double[][] iMinusK = sub(I, K);
        double[][] updatedCovariance = add(
                mul(mul(iMinusK, priorCovariance), transpose(iMinusK)),
                mul(mul(K, R), transpose(K))
        );

        return new StateSnapshot(
                planarize(updatedPose),
                sanitizeCovariance(updatedCovariance),
                predictorPose,
                timestampSec
        );
    }

    private StateSnapshot predictStep(Pose3d priorPose,
                                      double[][] priorCovariance,
                                      Pose3d odomDelta,
                                      Pose3d resultingPredictorPose,
                                      double timestampSec) {
        Pose3d delta = planarize(odomDelta);
        Pose3d predictedPose = planarize(priorPose.then(delta));

        double theta = MathUtil.wrapToPi(priorPose.yawRad);
        double dxLocal = delta.xInches;
        double dyLocal = delta.yInches;
        double dHeading = MathUtil.wrapToPi(delta.yawRad);

        double[][] F = new double[][]{
                {1.0, 0.0, -Math.sin(theta) * dxLocal - Math.cos(theta) * dyLocal},
                {0.0, 1.0, Math.cos(theta) * dxLocal - Math.sin(theta) * dyLocal},
                {0.0, 0.0, 1.0}
        };

        double translation = Math.hypot(dxLocal, dyLocal);
        double sigmaPos = cfg.predictorProcessPositionStdFloorIn
                + cfg.predictorProcessPositionStdPerIn * translation
                + cfg.predictorProcessPositionStdPerRad * Math.abs(dHeading);
        double sigmaHeading = cfg.predictorProcessHeadingStdFloorRad
                + cfg.predictorProcessHeadingStdPerIn * translation
                + cfg.predictorProcessHeadingStdPerRad * Math.abs(dHeading);

        double c = Math.cos(theta);
        double s = Math.sin(theta);
        double[][] G = new double[][]{
                {c, -s, 0.0},
                {s, c, 0.0},
                {0.0, 0.0, 1.0}
        };
        double[][] qLocal = diagonal(sigmaPos * sigmaPos, sigmaPos * sigmaPos, sigmaHeading * sigmaHeading);
        double[][] Q = mul(mul(G, qLocal), transpose(G));

        double[][] predictedCovariance = add(
                mul(mul(F, priorCovariance), transpose(F)),
                Q
        );

        return new StateSnapshot(
                predictedPose,
                sanitizeCovariance(predictedCovariance),
                resultingPredictorPose,
                timestampSec
        );
    }

    private StateSnapshot propagateFromArbitraryState(Pose3d startPose,
                                                      double[][] startCovariance,
                                                      double startTimestampSec,
                                                      double endTimestampSec) {
        if (startPose == null || startCovariance == null) {
            return null;
        }
        if (!Double.isFinite(startTimestampSec) || !Double.isFinite(endTimestampSec)) {
            return null;
        }
        if (endTimestampSec + TIMESTAMP_EPS_SEC < startTimestampSec) {
            return null;
        }
        if (Math.abs(endTimestampSec - startTimestampSec) <= TIMESTAMP_EPS_SEC) {
            Pose3d predictorPose = interpolatePredictorPose(endTimestampSec);
            return new StateSnapshot(planarize(startPose), sanitizeCovariance(copy(startCovariance)), predictorPose, endTimestampSec);
        }

        PredictorSample[] samples = predictorHistory.toArray(new PredictorSample[0]);
        if (samples.length == 0) {
            return null;
        }

        Pose3d startPredictorPose = interpolatePredictorPose(startTimestampSec);
        Pose3d endPredictorPose = interpolatePredictorPose(endTimestampSec);
        if (startPredictorPose == null || endPredictorPose == null) {
            return null;
        }

        Pose3d pose = planarize(startPose);
        double[][] covariance = sanitizeCovariance(copy(startCovariance));
        Pose3d prevPredictorPose = startPredictorPose;
        double prevTimestampSec = startTimestampSec;

        for (PredictorSample sample : samples) {
            if (sample.timestampSec <= startTimestampSec + TIMESTAMP_EPS_SEC) {
                continue;
            }
            if (sample.timestampSec >= endTimestampSec - TIMESTAMP_EPS_SEC) {
                break;
            }
            Pose3d nextPredictorPose = sample.pose;
            StateSnapshot predicted = predictStep(
                    pose,
                    covariance,
                    prevPredictorPose.inverse().then(nextPredictorPose),
                    nextPredictorPose,
                    sample.timestampSec
            );
            pose = predicted.pose;
            covariance = predicted.covariance;
            prevPredictorPose = nextPredictorPose;
            prevTimestampSec = sample.timestampSec;
        }

        if (endTimestampSec > prevTimestampSec + TIMESTAMP_EPS_SEC) {
            StateSnapshot predicted = predictStep(
                    pose,
                    covariance,
                    prevPredictorPose.inverse().then(endPredictorPose),
                    endPredictorPose,
                    endTimestampSec
            );
            pose = predicted.pose;
            covariance = predicted.covariance;
        }

        return new StateSnapshot(pose, covariance, endPredictorPose, endTimestampSec);
    }

    private boolean pushFilteredPoseToPredictor() {
        if (cfg.enablePushCorrectedPoseToPredictor && predictor instanceof PoseResetter) {
            ((PoseResetter) predictor).setPose(statePose.toPose2d());
            return true;
        }
        return false;
    }

    private void clearRecentCorrectionState() {
        lastCorrectionAcceptedSec = Double.NaN;
        lastAcceptedCorrectionMeasurementTimestampSec = Double.NaN;
        lastCorrectionUsedReplay = false;
    }

    private void rebaseAfterPoseChange(double timestampSec, Pose3d currentPredictorPose, boolean pushedToPredictor) {
        Pose3d basePredictorPose;
        if (pushedToPredictor) {
            lastPredictorPose = statePose;
            basePredictorPose = statePose;
        } else if (currentPredictorPose != null) {
            lastPredictorPose = currentPredictorPose;
            basePredictorPose = currentPredictorPose;
        } else {
            lastPredictorPose = statePose;
            basePredictorPose = statePose;
        }

        resetPredictorHistory(timestampSec, basePredictorPose);
        setReplayBase(timestampSec, statePose, stateCovariance, basePredictorPose);
    }

    private void setReplayBase(double timestampSec,
                               Pose3d pose,
                               double[][] covariance,
                               Pose3d predictorPoseAtBase) {
        replayBaseValid = Double.isFinite(timestampSec) && pose != null && covariance != null && predictorPoseAtBase != null;
        replayBaseTimestampSec = replayBaseValid ? timestampSec : Double.NaN;
        replayBasePose = replayBaseValid ? planarize(pose) : Pose3d.zero();
        replayBaseCovariance = replayBaseValid ? sanitizeCovariance(copy(covariance)) : diagonal(1.0, 1.0, 1.0);
        replayBasePredictorPose = replayBaseValid ? planarize(predictorPoseAtBase) : Pose3d.zero();
    }

    private double[][] initialPredictorCovariance() {
        return diagonal(
                cfg.initialPositionStdIn * cfg.initialPositionStdIn,
                cfg.initialPositionStdIn * cfg.initialPositionStdIn,
                cfg.initialHeadingStdRad * cfg.initialHeadingStdRad
        );
    }

    private double[][] manualAnchorCovariance() {
        return diagonal(
                cfg.manualPosePositionStdIn * cfg.manualPosePositionStdIn,
                cfg.manualPosePositionStdIn * cfg.manualPosePositionStdIn,
                cfg.manualPoseHeadingStdRad * cfg.manualPoseHeadingStdRad
        );
    }

    private double[][] measurementCovariance(double measurementQuality,
                                             boolean projectedMeasurement,
                                             double projectedAgeSec) {
        double q = MathUtil.clamp(measurementQuality, 0.0, 1.0);
        double sigmaPos = cfg.correctionPositionStdFloorIn + cfg.correctionPositionStdScaleIn * (1.0 - q);
        double sigmaHeading = cfg.correctionHeadingStdFloorRad + cfg.correctionHeadingStdScaleRad * (1.0 - q);

        if (projectedMeasurement) {
            sigmaPos += cfg.projectedCorrectionPositionStdPerSec * Math.max(0.0, projectedAgeSec);
            sigmaHeading += cfg.projectedCorrectionHeadingStdPerSec * Math.max(0.0, projectedAgeSec);
        }

        sigmaPos = Math.max(1e-3, sigmaPos);
        sigmaHeading = Math.max(1e-6, sigmaHeading);
        return diagonal(sigmaPos * sigmaPos, sigmaPos * sigmaPos, sigmaHeading * sigmaHeading);
    }

    private double covarianceQuality(double[][] covariance) {
        double posStd = positionStdIn(covariance);
        double headingStd = headingStdRad(covariance);
        double posScore = 1.0 - MathUtil.clamp01(posStd / cfg.qualityPositionStdScaleIn);
        double headingScore = 1.0 - MathUtil.clamp01(headingStd / cfg.qualityHeadingStdScaleRad);
        return MathUtil.clamp01(0.70 * posScore + 0.30 * headingScore);
    }

    private static double positionStdIn(double[][] covariance) {
        if (covariance == null || covariance.length < 3 || covariance[0].length < 3) {
            return Double.NaN;
        }
        return Math.sqrt(Math.max(MIN_VARIANCE, 0.5 * (covariance[0][0] + covariance[1][1])));
    }

    private static double headingStdRad(double[][] covariance) {
        if (covariance == null || covariance.length < 3 || covariance[2].length < 3) {
            return Double.NaN;
        }
        return Math.sqrt(Math.max(MIN_VARIANCE, covariance[2][2]));
    }

    private void recordPredictorSample(double timestampSec, Pose3d predictorPose) {
        if (!Double.isFinite(timestampSec) || predictorPose == null) {
            return;
        }

        PredictorSample last = predictorHistory.peekLast();
        if (last != null && timestampSec <= last.timestampSec) {
            predictorHistory.clear();
            predictorHistory.addLast(new PredictorSample(timestampSec, planarize(predictorPose)));
            if (initialized) {
                setReplayBase(timestampSec, statePose, stateCovariance, planarize(predictorPose));
                lastPredictorPose = planarize(predictorPose);
            }
            return;
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
        if (timestampSec < samples[0].timestampSec - TIMESTAMP_EPS_SEC
                || timestampSec > samples[samples.length - 1].timestampSec + TIMESTAMP_EPS_SEC) {
            return null;
        }
        if (samples.length == 1) {
            return samples[0].pose;
        }

        PredictorSample prev = samples[0];
        for (PredictorSample next : samples) {
            if (next.timestampSec < timestampSec - TIMESTAMP_EPS_SEC) {
                prev = next;
                continue;
            }
            if (Math.abs(next.timestampSec - timestampSec) <= TIMESTAMP_EPS_SEC || next == prev) {
                return next.pose;
            }

            double dt = next.timestampSec - prev.timestampSec;
            if (dt <= TIMESTAMP_EPS_SEC) {
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
                0.0
        );
    }

    private static double estimateTimestampOr(PoseEstimate est, double fallbackNowSec) {
        if (est != null && Double.isFinite(est.timestampSec)) {
            return est.timestampSec;
        }
        return fallbackNowSec;
    }

    private static double[][] identity() {
        return diagonal(1.0, 1.0, 1.0);
    }

    private static double[][] diagonal(double a, double b, double c) {
        return new double[][]{
                {a, 0.0, 0.0},
                {0.0, b, 0.0},
                {0.0, 0.0, c}
        };
    }

    private static double[][] copy(double[][] m) {
        return new double[][]{
                {m[0][0], m[0][1], m[0][2]},
                {m[1][0], m[1][1], m[1][2]},
                {m[2][0], m[2][1], m[2][2]}
        };
    }

    private static double[][] sanitizeCovariance(double[][] m) {
        double[][] out = copy(m);
        for (int r = 0; r < 3; r++) {
            for (int c = r + 1; c < 3; c++) {
                double avg = 0.5 * (out[r][c] + out[c][r]);
                out[r][c] = avg;
                out[c][r] = avg;
            }
            out[r][r] = Math.max(MIN_VARIANCE, out[r][r]);
        }
        return out;
    }

    private static double[][] add(double[][] a, double[][] b) {
        double[][] out = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                out[r][c] = a[r][c] + b[r][c];
            }
        }
        return out;
    }

    private static double[][] sub(double[][] a, double[][] b) {
        double[][] out = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                out[r][c] = a[r][c] - b[r][c];
            }
        }
        return out;
    }

    private static double[][] transpose(double[][] m) {
        double[][] out = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                out[r][c] = m[c][r];
            }
        }
        return out;
    }

    private static double[][] mul(double[][] a, double[][] b) {
        double[][] out = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
            }
        }
        return out;
    }

    private static double[] mul(double[][] a, double[] x) {
        return new double[]{
                a[0][0] * x[0] + a[0][1] * x[1] + a[0][2] * x[2],
                a[1][0] * x[0] + a[1][1] * x[1] + a[1][2] * x[2],
                a[2][0] * x[0] + a[2][1] * x[1] + a[2][2] * x[2]
        };
    }

    private static double quadraticForm(double[] x, double[][] m) {
        double[] mx = mul(m, x);
        return x[0] * mx[0] + x[1] * mx[1] + x[2] * mx[2];
    }

    private static double[][] invert3x3(double[][] m) {
        double a = m[0][0], b = m[0][1], c = m[0][2];
        double d = m[1][0], e = m[1][1], f = m[1][2];
        double g = m[2][0], h = m[2][1], i = m[2][2];

        double A = e * i - f * h;
        double B = -(d * i - f * g);
        double C = d * h - e * g;
        double D = -(b * i - c * h);
        double E = a * i - c * g;
        double F = -(a * h - b * g);
        double G = b * f - c * e;
        double H = -(a * f - c * d);
        double I = a * e - b * d;

        double det = a * A + b * B + c * C;
        if (!Double.isFinite(det) || Math.abs(det) <= 1e-12) {
            return null;
        }
        double invDet = 1.0 / det;
        return new double[][]{
                {A * invDet, D * invDet, G * invDet},
                {B * invDet, E * invDet, H * invDet},
                {C * invDet, F * invDet, I * invDet}
        };
    }

    /**
     * Emits the current EKF state, covariance-derived confidence, latest innovation metrics, and
     * correction counters for debugging.
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "ekf" : prefix;

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
                .addData(p + ".lastCorrectionPose", lastCorrectionPose)
                .addData(p + ".lastLatencyCompensatedCorrectionPose", lastLatencyCompensatedCorrectionPose)
                .addData(p + ".lastReplayReferencePose", lastReplayReferencePose)
                .addData(p + ".lastCorrectionUsedReplay", lastCorrectionUsedReplay)
                .addData(p + ".lastInnovationPositionIn", lastInnovationPositionIn)
                .addData(p + ".lastInnovationHeadingRad", lastInnovationHeadingRad)
                .addData(p + ".lastInnovationMahalanobisSq", lastInnovationMahalanobisSq)
                .addData(p + ".lastMeasurementPositionStdIn", lastMeasurementPositionStdIn)
                .addData(p + ".lastMeasurementHeadingStdRad", lastMeasurementHeadingStdRad)
                .addData(p + ".positionStdIn", getPositionStdIn())
                .addData(p + ".headingStdRad", getHeadingStdRad())
                .addData(p + ".replayBaseValid", replayBaseValid)
                .addData(p + ".replayBaseTimestampSec", replayBaseTimestampSec)
                .addData(p + ".replayBasePose", replayBasePose)
                .addData(p + ".replayBasePredictorPose", replayBasePredictorPose)
                .addData(p + ".predictorHistorySize", predictorHistory.size())
                .addData(p + ".statePose", statePose)
                .addData(p + ".stateCovariance.xx", stateCovariance[0][0])
                .addData(p + ".stateCovariance.yy", stateCovariance[1][1])
                .addData(p + ".stateCovariance.hh", stateCovariance[2][2])
                .addData(p + ".cfg.maxCorrectionAgeSec", cfg.maxCorrectionAgeSec)
                .addData(p + ".cfg.minCorrectionQuality", cfg.minCorrectionQuality)
                .addData(p + ".cfg.maxCorrectionPositionInnovationIn", cfg.maxCorrectionPositionInnovationIn)
                .addData(p + ".cfg.maxCorrectionHeadingInnovationRad", cfg.maxCorrectionHeadingInnovationRad)
                .addData(p + ".cfg.maxCorrectionMahalanobisSq", cfg.maxCorrectionMahalanobisSq)
                .addData(p + ".cfg.enableInitializeFromCorrection", cfg.enableInitializeFromCorrection)
                .addData(p + ".cfg.enablePushCorrectedPoseToPredictor", cfg.enablePushCorrectedPoseToPredictor)
                .addData(p + ".cfg.enableLatencyCompensation", cfg.enableLatencyCompensation)
                .addData(p + ".cfg.predictorHistorySec", cfg.predictorHistorySec)
                .addData(p + ".lastEstimate", lastEstimate);

        predictor.debugDump(dbg, p + ".predictor");
        correction.debugDump(dbg, p + ".correction");
    }
}
