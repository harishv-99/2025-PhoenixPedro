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

    private enum PredictorMotionRebaseState {
        NONE,
        AWAITING_PREDICTOR_POSE,
        ANCHORED
    }

    /**
     * Configuration for {@link OdometryCorrectionEkfEstimator}.
     *
     * <p>The defaults are intentionally conservative and should be treated as a starting point, not
     * as a substitute for real calibration. In particular, the predictor process-noise terms and the
     * correction measurement-noise terms only make sense once camera extrinsics and predictor geometry are
     * already trustworthy. Validated capture also rejects deterministic standard-deviation and
     * covariance combinations that cannot remain finite.</p>
     */
    public static final class Config {
        /**
         * Reject correction measurements older than this many seconds.
         *
         * <p>Must be finite and at least zero. Zero is an inclusive current-timestamp-only bound;
         * it does not disable freshness checking.</p>
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
         *
         * <p>Must be finite and within {@code (0, pi]}. {@code pi} disables only this
         * heading-magnitude rejection; the Mahalanobis gate may still reject the observation.</p>
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
         * <p>When latency compensation is enabled, this must be at least as large as
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

        /** Validates this copied draft before an owner retains it. */
        private void validate(String context) {
            String p = (context != null && !context.trim().isEmpty())
                    ? context.trim()
                    : "OdometryCorrectionEkfEstimator.Config";

            requireFiniteNonNegative(maxCorrectionAgeSec, p + ".maxCorrectionAgeSec");
            requireFiniteInRange(minCorrectionQuality, 0.0, 1.0, p + ".minCorrectionQuality");
            requireFinitePositive(maxCorrectionPositionInnovationIn, p + ".maxCorrectionPositionInnovationIn");
            requireFinitePositive(maxCorrectionHeadingInnovationRad, p + ".maxCorrectionHeadingInnovationRad");
            if (maxCorrectionHeadingInnovationRad > Math.PI) {
                throw new IllegalArgumentException(
                        p + ".maxCorrectionHeadingInnovationRad must be finite and within (0, pi], got "
                                + maxCorrectionHeadingInnovationRad);
            }
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

            if (enableLatencyCompensation && predictorHistorySec < maxCorrectionAgeSec) {
                throw new IllegalArgumentException(
                        p + ".predictorHistorySec must be >= maxCorrectionAgeSec when latency compensation is enabled"
                                + " (increase predictorHistorySec or reduce maxCorrectionAgeSec)"
                                + "; got predictorHistorySec=" + predictorHistorySec
                                + ", maxCorrectionAgeSec=" + maxCorrectionAgeSec
                );
            }

            requireFiniteSquare(initialPositionStdIn, p + ".initialPositionStdIn");
            requireFiniteSquare(initialHeadingStdRad, p + ".initialHeadingStdRad");
            requireFiniteSquare(manualPosePositionStdIn, p + ".manualPosePositionStdIn");
            requireFiniteSquare(manualPoseHeadingStdRad, p + ".manualPoseHeadingStdRad");
            requireFiniteSquare(
                    predictorProcessPositionStdFloorIn,
                    p + ".predictorProcessPositionStdFloorIn");
            requireFiniteSquare(
                    predictorProcessHeadingStdFloorRad,
                    p + ".predictorProcessHeadingStdFloorRad");

            requireRepresentableCorrectionSigma(
                    p + ".position correction standard deviation",
                    correctionPositionStdFloorIn,
                    correctionPositionStdScaleIn,
                    projectedCorrectionPositionStdPerSec,
                    maxCorrectionAgeSec);
            requireRepresentableCorrectionSigma(
                    p + ".heading correction standard deviation",
                    correctionHeadingStdFloorRad,
                    correctionHeadingStdScaleRad,
                    projectedCorrectionHeadingStdPerSec,
                    maxCorrectionAgeSec);
            requireRepresentablePerRadSigma(
                    p + ".position per-radian process standard deviation",
                    predictorProcessPositionStdFloorIn,
                    predictorProcessPositionStdPerRad);
            requireRepresentablePerRadSigma(
                    p + ".heading per-radian process standard deviation",
                    predictorProcessHeadingStdFloorRad,
                    predictorProcessHeadingStdPerRad);
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
         * Returns a validated, independently mutable snapshot for one owner.
         *
         * @param context diagnostic field prefix; null or blank uses the canonical Config class
         *                name
         * @return validated defensive copy independent of this authoring draft
         * @throws IllegalArgumentException if any configured value, covariance representation, or
         *                                  cross-field relationship is invalid
         */
        public Config validatedCopy(String context) {
            Config c = copy();
            c.validate(context);
            return c;
        }

        private static void requireFiniteNonNegative(double value, String name) {
            if (!Double.isFinite(value) || value < 0.0) {
                throw new IllegalArgumentException(name + " must be finite and >= 0, got " + value);
            }
        }

        private static void requireFinitePositive(double value, String name) {
            if (!Double.isFinite(value) || value <= 0.0) {
                throw new IllegalArgumentException(name + " must be finite and > 0, got " + value);
            }
        }

        private static void requireFiniteInRange(double value, double lo, double hi, String name) {
            if (!Double.isFinite(value) || value < lo || value > hi) {
                throw new IllegalArgumentException(name + " must be finite and within [" + lo + ", " + hi
                        + "], got " + value);
            }
        }

        private static void requireFiniteSquare(double value, String name) {
            double variance = value * value;
            if (!Double.isFinite(variance)) {
                throw new IllegalArgumentException(
                        name + " must square to a finite variance, got " + value
                                + " (variance=" + variance + ")");
            }
        }

        private static void requireRepresentableCorrectionSigma(String name,
                                                                 double floor,
                                                                 double qualityScale,
                                                                 double projectedPerSec,
                                                                 double maxAgeSec) {
            double sigma = floor + qualityScale + projectedPerSec * maxAgeSec;
            double variance = sigma * sigma;
            if (!Double.isFinite(sigma) || !Double.isFinite(variance)) {
                throw new IllegalArgumentException(
                        name + " and its squared variance must be finite; got floor=" + floor
                                + ", qualityScale=" + qualityScale
                                + ", projectedPerSec=" + projectedPerSec
                                + ", maxCorrectionAgeSec=" + maxAgeSec + ", sigma=" + sigma
                                + ", variance=" + variance);
            }
        }

        private static void requireRepresentablePerRadSigma(String name,
                                                              double floor,
                                                              double perRad) {
            double sigma = floor + perRad * Math.PI;
            double variance = sigma * sigma;
            if (!Double.isFinite(sigma) || !Double.isFinite(variance)) {
                throw new IllegalArgumentException(
                        name + " at pi radians and its squared variance must be finite; got floor="
                                + floor + ", perRad=" + perRad + ", sigma=" + sigma
                                + ", variance=" + variance);
            }
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

    private static final class StateSnapshot {
        final Pose3d pose;
        final double[][] covariance;
        final Pose3d predictorPose;
        final LoopTimestamp timestamp;

        StateSnapshot(Pose3d pose,
                      double[][] covariance,
                      Pose3d predictorPose,
                      LoopTimestamp timestamp) {
            this.pose = pose;
            this.covariance = covariance;
            this.predictorPose = predictorPose;
            this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
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

    private PoseEstimate lastEstimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
    private final Deque<PredictorSample> predictorHistory = new ArrayDeque<PredictorSample>();

    // One estimator instance owns one state transition per shared-loop cycle. Claim the attempt
    // before child updates because source updates, covariance propagation, and predictor rebases
    // may have effects that cannot be rolled back safely after a failure.
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean updateInProgress = false;
    private RuntimeException lastUpdateFailure = null;

    // Canonical identity of predictor motion already incorporated into statePose/stateCovariance.
    // This remains necessary in addition to the cycle guard because a predictor may retain one
    // sample across multiple robot loops.
    private LoopTimestamp lastCoveredPredictorMotionEndTimestamp = LoopTimestamp.unavailable();

    // A pose anchor can occur after the predictor has observed a changed pose at zero elapsed
    // time. Its next positive MotionDelta then spans both sides of the anchor. Retain the predictor
    // pose at the anchor so that one post-anchor interval can exclude that already-covered motion.
    private PredictorMotionRebaseState predictorMotionRebaseState =
            PredictorMotionRebaseState.NONE;
    private Pose3d predictorMotionRebasePose = Pose3d.zero();

    private boolean replayBaseValid = false;
    private LoopTimestamp replayBaseTimestamp = LoopTimestamp.unavailable();
    private Pose3d replayBasePose = Pose3d.zero();
    private double[][] replayBaseCovariance = diagonal(1.0, 1.0, 1.0);
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

    private double lastInnovationPositionIn = Double.NaN;
    private double lastInnovationHeadingRad = Double.NaN;
    private double lastInnovationMahalanobisSq = Double.NaN;
    private double lastMeasurementPositionStdIn = Double.NaN;
    private double lastMeasurementHeadingStdRad = Double.NaN;

    /**
     * Creates an EKF-style localizer with explicit configuration.
     *
     * <p>{@code cfg} is required and is defensively validated and copied. Pass
     * {@link Config#defaults()} explicitly to select the framework baseline.</p>
     *
     * @param predictor  motion predictor to propagate
     * @param correction absolute pose source used for correction
     * @param cfg        non-null estimator policy draft
     * @throws IllegalArgumentException if a dependency or {@code cfg} is null, or if
     *                                  {@code cfg} is invalid
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
        if (cfg == null) {
            throw new IllegalArgumentException(
                    "OdometryCorrectionEkfEstimator.Config must not be null; "
                            + "use Config.defaults() for the framework baseline");
        }
        this.cfg = cfg.validatedCopy("OdometryCorrectionEkfEstimator.Config");
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
                lastCorrectionAccepted,
                lastAcceptedCorrectionMeasurementTimestamp,
                lastEvaluatedCorrectionTimestamp,
                lastCorrectionUsedReplay
        );
    }

    /**
     * Advances the EKF one loop by propagating predictor motion and conditionally applying the
     * latest absolute correction measurement.
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
     *
     * <p>A usable predictor pose needs finite planar x/y/yaw and a current-epoch timestamp.
     * A claimed motion delta additionally needs finite planar components, quality within
     * {@code [0, 1]}, coherent current-epoch timestamps, and a strictly positive duration.
     * Invalid claimed motion is not applied or retained as a replay bridge. A present correction
     * frame advances the evaluation watermark only after its timestamp proves current;
     * unavailable, wrong-epoch, and materially future timestamps remain skipped without changing
     * that watermark or the rejection count. Finite x/y/yaw, quality within {@code [0, 1]},
     * inclusive freshness, and configured innovation gates then determine acceptance. Any
     * non-finite derived pose, covariance, innovation, statistic, or quality fails closed instead
     * of becoming published filter state. If neither a usable current predictor pose nor a
     * correction accepted in this update exists, retained internal state remains available for
     * later recovery but the published estimate is unavailable rather than a freshly timestamped
     * frozen pose.</p>
     */
    @Override
    public void update(LoopClock clock) {
        LoopClock currentClock = Objects.requireNonNull(clock, "clock");
        long cycle = currentClock.cycle();

        if (updateInProgress) {
            throw new IllegalStateException(
                    "OdometryCorrectionEkfEstimator.update(clock) cannot be called reentrantly; "
                            + "one estimator owner may update once per loop cycle"
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
        boolean beforeInitialized = initialized;
        Pose3d beforeStatePose = statePose;
        double[][] beforeStateCovariance = stateCovariance;
        Pose3d beforeLastPredictorPose = lastPredictorPose;
        PoseEstimate beforeLastEstimate = lastEstimate;
        Deque<PredictorSample> beforePredictorHistory =
                new ArrayDeque<PredictorSample>(predictorHistory);
        LoopTimestamp beforeCoveredMotion = lastCoveredPredictorMotionEndTimestamp;
        PredictorMotionRebaseState beforeRebaseState = predictorMotionRebaseState;
        Pose3d beforeRebasePose = predictorMotionRebasePose;
        boolean beforeReplayBaseValid = replayBaseValid;
        LoopTimestamp beforeReplayBaseTimestamp = replayBaseTimestamp;
        Pose3d beforeReplayBasePose = replayBasePose;
        double[][] beforeReplayBaseCovariance = replayBaseCovariance;
        Pose3d beforeReplayBasePredictorPose = replayBasePredictorPose;
        LoopTimestamp beforeCorrectionAccepted = lastCorrectionAccepted;
        LoopTimestamp beforeAcceptedMeasurement = lastAcceptedCorrectionMeasurementTimestamp;
        LoopTimestamp beforeEvaluatedMeasurement = lastEvaluatedCorrectionTimestamp;
        Pose3d beforeCorrectionPose = lastCorrectionPose;
        Pose3d beforeCompensatedCorrectionPose = lastLatencyCompensatedCorrectionPose;
        Pose3d beforeReplayReferencePose = lastReplayReferencePose;
        boolean beforeCorrectionUsedReplay = lastCorrectionUsedReplay;
        int beforeAcceptedCount = acceptedCorrectionCount;
        int beforeRejectedCount = rejectedCorrectionCount;
        int beforeDuplicateCount = skippedDuplicateCorrectionCount;
        int beforeOutOfOrderCount = skippedOutOfOrderCorrectionCount;
        int beforeReplayedCount = replayedCorrectionCount;
        int beforeProjectedCount = projectedCorrectionCount;
        double beforeInnovationPosition = lastInnovationPositionIn;
        double beforeInnovationHeading = lastInnovationHeadingRad;
        double beforeInnovationMahalanobis = lastInnovationMahalanobisSq;
        double beforeMeasurementPosition = lastMeasurementPositionStdIn;
        double beforeMeasurementHeading = lastMeasurementHeadingStdRad;

        try {
            updateOnceMutating(clock);
        } catch (PredictorPosePushFailure pushFailure) {
            initialized = beforeInitialized;
            statePose = beforeStatePose;
            stateCovariance = beforeStateCovariance;
            lastPredictorPose = beforeLastPredictorPose;
            lastEstimate = beforeLastEstimate;
            lastCoveredPredictorMotionEndTimestamp = beforeCoveredMotion;
            predictorHistory.clear();
            predictorHistory.addAll(beforePredictorHistory);
            predictorMotionRebaseState = beforeRebaseState;
            predictorMotionRebasePose = beforeRebasePose;
            replayBaseValid = beforeReplayBaseValid;
            replayBaseTimestamp = beforeReplayBaseTimestamp;
            replayBasePose = beforeReplayBasePose;
            replayBaseCovariance = beforeReplayBaseCovariance;
            replayBasePredictorPose = beforeReplayBasePredictorPose;
            lastCorrectionAccepted = beforeCorrectionAccepted;
            lastAcceptedCorrectionMeasurementTimestamp = beforeAcceptedMeasurement;
            lastEvaluatedCorrectionTimestamp = beforeEvaluatedMeasurement;
            lastCorrectionPose = beforeCorrectionPose;
            lastLatencyCompensatedCorrectionPose = beforeCompensatedCorrectionPose;
            lastReplayReferencePose = beforeReplayReferencePose;
            lastCorrectionUsedReplay = beforeCorrectionUsedReplay;
            acceptedCorrectionCount = beforeAcceptedCount;
            rejectedCorrectionCount = beforeRejectedCount;
            skippedDuplicateCorrectionCount = beforeDuplicateCount;
            skippedOutOfOrderCorrectionCount = beforeOutOfOrderCount;
            replayedCorrectionCount = beforeReplayedCount;
            projectedCorrectionCount = beforeProjectedCount;
            lastInnovationPositionIn = beforeInnovationPosition;
            lastInnovationHeadingRad = beforeInnovationHeading;
            lastInnovationMahalanobisSq = beforeInnovationMahalanobis;
            lastMeasurementPositionStdIn = beforeMeasurementPosition;
            lastMeasurementHeadingStdRad = beforeMeasurementHeading;
            throw pushFailure.original;
        }
    }

    private void updateOnceMutating(LoopClock clock) {
        final LoopTimestamp nowTimestamp = clock.nowTimestamp();
        final int acceptedCorrectionsBeforeUpdate = acceptedCorrectionCount;

        invalidateHistoryAcrossReset(nowTimestamp);

        predictor.update(clock);
        correction.update(clock);

        final PoseEstimate predictorEst = predictor.getEstimate();
        final MotionDelta predictorDelta = predictor.getLatestMotionDelta();
        final PoseEstimate correctionEst = correction.getEstimate();
        final boolean predictorTimestampCurrent = predictorEst != null
                && isTimestampCurrent(predictorEst.timestamp, clock);
        final Pose3d currentPredictorPose = (predictorEst != null
                && predictorEst.hasPose
                && predictorTimestampCurrent
                && isFinitePlanarPose(predictorEst.fieldToRobotPose))
                ? planarize(predictorEst.fieldToRobotPose)
                : null;
        final LoopTimestamp currentPredictorTimestamp = predictorTimestampCurrent
                ? predictorEst.timestamp
                : LoopTimestamp.unavailable();
        final boolean predictorMotionUsable = isUsablePredictorMotion(predictorDelta, clock);
        final boolean predictorClaimedInvalidMotion = predictorDelta != null
                && predictorDelta.hasDelta
                && !predictorMotionUsable;

        if (predictorClaimedInvalidMotion && initialized) {
            if (currentPredictorPose != null) {
                resetPredictorHistory(currentPredictorTimestamp, currentPredictorPose);
                setReplayBase(
                        currentPredictorTimestamp,
                        statePose,
                        stateCovariance,
                        currentPredictorPose);
                rememberPredictorMotionRebase(currentPredictorPose);
            } else {
                predictorHistory.clear();
                setReplayBase(
                        LoopTimestamp.unavailable(),
                        statePose,
                        stateCovariance,
                        Pose3d.zero());
                awaitPredictorMotionRebase();
            }
        } else if (currentPredictorPose != null) {
            recordPredictorSample(currentPredictorTimestamp, currentPredictorPose);
        }

        boolean evaluatedCorrectionThisLoop = false;

        if (!initialized) {
            boolean initializedFromCorrection = false;
            if (correctionEnabled
                    && cfg.enableInitializeFromCorrection
                    && shouldEvaluateCorrectionMeasurement(correctionEst, clock)) {
                evaluatedCorrectionThisLoop = true;
                if (isCorrectionAcceptable(correctionEst, clock)) {
                    boolean accepted = initializeFromCorrection(
                            correctionEst,
                            currentPredictorPose,
                            currentPredictorTimestamp,
                            nowTimestamp,
                            clock
                    );
                    initializedFromCorrection = accepted;
                    if (!accepted) {
                        rejectedCorrectionCount++;
                    }
                } else {
                    rejectedCorrectionCount++;
                }
            }

            if (!initialized && currentPredictorPose != null) {
                statePose = currentPredictorPose;
                stateCovariance = initialPredictorCovariance();
                initialized = true;
                lastPredictorPose = currentPredictorPose;
                resetPredictorHistory(currentPredictorTimestamp, currentPredictorPose);
                setReplayBase(currentPredictorTimestamp, statePose, stateCovariance, currentPredictorPose);
                markPredictorMotionCovered(currentPredictorTimestamp);
                rememberPredictorMotionRebase(currentPredictorPose);
            } else if (!initialized && !initializedFromCorrection) {
                lastEstimate = PoseEstimate.noPose(nowTimestamp);
                return;
            }
        } else {
            establishAwaitingPredictorMotionRebase(
                    currentPredictorTimestamp,
                    currentPredictorPose
            );
            Pose3d predictorMotion = predictorMotionUsable
                    ? predictorMotionForUpdate(
                            predictorDelta,
                            currentPredictorPose,
                            clock
                    )
                    : null;
            if (predictorMotion != null) {
                StateSnapshot predicted = predictStep(
                        statePose,
                        stateCovariance,
                        predictorMotion,
                        currentPredictorPose,
                        currentPredictorTimestamp
                );
                if (isFiniteState(predicted)) {
                    statePose = predicted.pose;
                    stateCovariance = predicted.covariance;
                    markPredictorMotionCovered(predictorDelta.endTimestamp);
                    clearPredictorMotionRebase();
                } else if (currentPredictorPose != null) {
                    resetPredictorHistory(currentPredictorTimestamp, currentPredictorPose);
                    setReplayBase(
                            currentPredictorTimestamp,
                            statePose,
                            stateCovariance,
                            currentPredictorPose);
                    rememberPredictorMotionRebase(currentPredictorPose);
                }
            }
            if (currentPredictorPose != null) {
                lastPredictorPose = currentPredictorPose;
            }
        }

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
                        nowTimestamp,
                        clock
                );
            }
        }

        // Retain the internal state and covariance across a sensor gap for recovery, but do not
        // timestamp that frozen state as fresh evidence. A correction accepted in this update is
        // independently current evidence even when the predictor is unavailable.
        if (currentPredictorPose == null
                && acceptedCorrectionCount == acceptedCorrectionsBeforeUpdate) {
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        double quality = covarianceQuality(stateCovariance);
        if (!isFinitePlanarPose(statePose) || !Double.isFinite(quality)) {
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }
        lastEstimate = new PoseEstimate(statePose, true, quality, nowTimestamp);
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
     *
     * <p>The pose must be non-null with finite x/y/heading. Validation and all derived-state
     * checks precede predictor or local effects. When the configured predictor supports
     * {@link PoseResetter}, its rejection leaves this estimator's complete local pose, covariance,
     * replay, history, diagnostics, and statistics unchanged.</p>
     */
    @Override
    public void setPose(Pose2d pose) {
        requireFiniteAuthoredPose(pose, "OdometryCorrectionEkfEstimator.setPose(pose)");

        final LoopTimestamp nowTimestamp = (lastEstimate != null && lastEstimate.timestamp != null)
                ? lastEstimate.timestamp
                : LoopTimestamp.unavailable();

        Pose3d candidatePose = new Pose3d(
                pose.xInches,
                pose.yInches,
                0.0,
                MathUtil.wrapToPi(pose.headingRad),
                0.0,
                0.0);
        double[][] candidateCovariance = manualAnchorCovariance();

        Pose3d currentPredictorPose = null;
        LoopTimestamp currentPredictorTimestamp = nowTimestamp;
        PoseEstimate predictorEst = predictor.getEstimate();
        if (predictorEst != null
                && predictorEst.hasPose
                && isFinitePlanarPose(predictorEst.fieldToRobotPose)) {
            currentPredictorPose = planarize(predictorEst.fieldToRobotPose);
            if (predictorEst.timestamp != null && predictorEst.timestamp.isAvailable()) {
                currentPredictorTimestamp = predictorEst.timestamp;
            }
        }

        boolean pushedToPredictor;
        try {
            pushedToPredictor = pushPoseToPredictor(candidatePose);
        } catch (PredictorPosePushFailure pushFailure) {
            throw pushFailure.original;
        }

        statePose = candidatePose;
        stateCovariance = candidateCovariance;
        initialized = true;
        clearRecentCorrectionState();
        rebaseAfterPoseChange(
                nowTimestamp,
                currentPredictorPose,
                currentPredictorTimestamp,
                pushedToPredictor
        );
        lastEstimate = new PoseEstimate(
                statePose,
                true,
                covarianceQuality(stateCovariance),
                nowTimestamp
        );
    }

    private boolean initializeFromCorrection(PoseEstimate correctionEst,
                                             Pose3d currentPredictorPose,
                                             LoopTimestamp currentPredictorTimestamp,
                                             LoopTimestamp nowTimestamp,
                                             LoopClock clock) {
        Pose3d correctionPoseAtMeasurement = planarize(correctionEst.fieldToRobotPose);

        StateSnapshot currentState = null;
        boolean usedReplay = false;
        if (cfg.enableLatencyCompensation) {
            double[][] measCov = measurementCovariance(correctionEst.quality, false, 0.0);
            if (isFiniteCovariance(measCov)) {
                currentState = propagateFromArbitraryState(
                        correctionPoseAtMeasurement,
                        measCov,
                        correctionEst.timestamp,
                        currentPredictorTimestamp
                );
            }
            usedReplay = currentState != null
                    && timestampElapsedSec(
                            currentPredictorTimestamp,
                            correctionEst.timestamp) > TIMESTAMP_EPS_SEC;
        }

        Pose3d candidatePose;
        double[][] candidateCovariance;
        boolean projected;
        if (currentState == null) {
            Pose3d projectedCorrectionPoseNow = projectCorrectionPoseToNow(
                    correctionPoseAtMeasurement,
                    correctionEst.timestamp,
                    currentPredictorPose
            );
            double projectedAgeSec = timestampAgeSec(correctionEst.timestamp, clock);
            candidatePose = projectedCorrectionPoseNow;
            candidateCovariance = measurementCovariance(
                    correctionEst.quality,
                    true,
                    projectedAgeSec);
            usedReplay = false;
            projected = true;
        } else {
            candidatePose = currentState.pose;
            candidateCovariance = currentState.covariance;
            projected = !usedReplay;
        }

        if (!isFinitePlanarPose(candidatePose) || !isFiniteCovariance(candidateCovariance)) {
            return false;
        }

        boolean pushedToPredictor = pushPoseToPredictor(candidatePose);

        statePose = candidatePose;
        stateCovariance = candidateCovariance;
        initialized = true;
        lastCorrectionPose = correctionPoseAtMeasurement;
        lastReplayReferencePose = correctionPoseAtMeasurement;
        lastLatencyCompensatedCorrectionPose = candidatePose;
        lastInnovationPositionIn = Double.NaN;
        lastInnovationHeadingRad = Double.NaN;
        lastInnovationMahalanobisSq = Double.NaN;
        lastCorrectionUsedReplay = usedReplay;
        lastCorrectionAccepted = nowTimestamp;
        lastAcceptedCorrectionMeasurementTimestamp = correctionEst.timestamp;
        acceptedCorrectionCount++;
        if (projected) {
            projectedCorrectionCount++;
        } else {
            replayedCorrectionCount++;
        }

        rebaseAfterPoseChange(
                nowTimestamp,
                currentPredictorPose,
                currentPredictorTimestamp,
                pushedToPredictor
        );
        return true;
    }

    private boolean shouldEvaluateCorrectionMeasurement(PoseEstimate correctionEst, LoopClock clock) {
        if (correctionEst == null
                || !correctionEst.hasPose
                || !isTimestampCurrent(correctionEst.timestamp, clock)) {
            return false;
        }
        LoopTimestamp timestamp = correctionEst.timestamp;
        if (!lastEvaluatedCorrectionTimestamp.isAvailable()) {
            lastEvaluatedCorrectionTimestamp = timestamp;
            return true;
        }
        double elapsedSec = timestamp.secondsSince(lastEvaluatedCorrectionTimestamp);
        if (!Double.isFinite(elapsedSec)) {
            return false;
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
        if (correctionEst == null
                || !correctionEst.hasPose
                || !isFinitePlanarPose(correctionEst.fieldToRobotPose)) {
            return false;
        }
        double ageSec = timestampAgeSec(correctionEst.timestamp, clock);
        if (!Double.isFinite(ageSec)) {
            return false;
        }

        if (ageSec > cfg.maxCorrectionAgeSec) {
            return false;
        }

        if (!Double.isFinite(correctionEst.quality)
                || correctionEst.quality < 0.0
                || correctionEst.quality > 1.0
                || correctionEst.quality < cfg.minCorrectionQuality) {
            return false;
        }
        return true;
    }

    private void maybeApplyCorrection(PoseEstimate correctionEst,
                                      Pose3d currentPredictorPose,
                                      LoopTimestamp currentPredictorTimestamp,
                                      LoopTimestamp nowTimestamp,
                                      LoopClock clock) {
        Pose3d correctionPoseAtMeasurement = planarize(correctionEst.fieldToRobotPose);
        Pose3d projectedCorrectionPoseAtNow = projectCorrectionPoseToNow(
                correctionPoseAtMeasurement,
                correctionEst.timestamp,
                currentPredictorPose
        );
        if (!isFinitePlanarPose(correctionPoseAtMeasurement)
                || !isFinitePlanarPose(projectedCorrectionPoseAtNow)) {
            rejectedCorrectionCount++;
            return;
        }

        if (cfg.enableLatencyCompensation
                && replayBaseValid
                && timestampElapsedSec(correctionEst.timestamp, replayBaseTimestamp)
                < -TIMESTAMP_EPS_SEC) {
            lastReplayReferencePose = replayBasePose;
            lastCorrectionUsedReplay = false;
            rejectedCorrectionCount++;
            return;
        }

        double previousInnovationPosition = lastInnovationPositionIn;
        double previousInnovationHeading = lastInnovationHeadingRad;
        double previousInnovationMahalanobis = lastInnovationMahalanobisSq;
        double previousMeasurementPosition = lastMeasurementPositionStdIn;
        double previousMeasurementHeading = lastMeasurementHeadingStdRad;

        StateSnapshot correctedNow = null;
        boolean usedReplay = false;
        Pose3d replayReferencePose = statePose;

        if (cfg.enableLatencyCompensation) {
            StateSnapshot predictedAtMeasurement = predictFromReplayBaseTo(correctionEst.timestamp);
            if (isFiniteState(predictedAtMeasurement)) {
                replayReferencePose = predictedAtMeasurement.pose;
                StateSnapshot correctedAtMeasurement = measurementUpdate(
                        predictedAtMeasurement.pose,
                        predictedAtMeasurement.covariance,
                        correctionPoseAtMeasurement,
                        correctionEst.quality,
                        false,
                        0.0,
                        predictedAtMeasurement.predictorPose,
                        correctionEst.timestamp
                );
                if (correctedAtMeasurement != null) {
                    StateSnapshot replayed = propagateFromArbitraryState(
                            correctedAtMeasurement.pose,
                            correctedAtMeasurement.covariance,
                            correctionEst.timestamp,
                            currentPredictorTimestamp
                    );
                    correctedNow = isFiniteState(replayed) ? replayed : correctedAtMeasurement;
                    usedReplay = true;
                } else {
                    rejectedCorrectionCount++;
                    lastCorrectionUsedReplay = true;
                    return;
                }
            }
        }

        if (correctedNow == null) {
            replayReferencePose = statePose;
            double projectedAgeSec = timestampAgeSec(correctionEst.timestamp, clock);
            correctedNow = measurementUpdate(
                    statePose,
                    stateCovariance,
                    projectedCorrectionPoseAtNow,
                    correctionEst.quality,
                    true,
                    projectedAgeSec,
                    currentPredictorPose,
                    currentPredictorTimestamp
            );
            if (correctedNow == null) {
                rejectedCorrectionCount++;
                lastCorrectionUsedReplay = false;
                return;
            }
        }

        if (!isFiniteState(correctedNow)) {
            rejectedCorrectionCount++;
            return;
        }

        boolean pushedToPredictor;
        try {
            pushedToPredictor = pushPoseToPredictor(correctedNow.pose);
        } catch (RuntimeException rejectedByPredictor) {
            // measurementUpdate computes diagnostics before the configured reset boundary. Restore
            // them so a rejected vendor representation cannot partially publish estimator state.
            lastInnovationPositionIn = previousInnovationPosition;
            lastInnovationHeadingRad = previousInnovationHeading;
            lastInnovationMahalanobisSq = previousInnovationMahalanobis;
            lastMeasurementPositionStdIn = previousMeasurementPosition;
            lastMeasurementHeadingStdRad = previousMeasurementHeading;
            throw rejectedByPredictor;
        }

        statePose = correctedNow.pose;
        stateCovariance = correctedNow.covariance;
        lastCorrectionPose = correctionPoseAtMeasurement;
        lastLatencyCompensatedCorrectionPose = projectedCorrectionPoseAtNow;
        lastReplayReferencePose = replayReferencePose;
        lastCorrectionAccepted = nowTimestamp;
        lastAcceptedCorrectionMeasurementTimestamp = correctionEst.timestamp;
        lastCorrectionUsedReplay = usedReplay;
        acceptedCorrectionCount++;
        if (usedReplay) {
            replayedCorrectionCount++;
        } else {
            projectedCorrectionCount++;
        }

        rebaseAfterPoseChange(
                nowTimestamp,
                currentPredictorPose,
                currentPredictorTimestamp,
                pushedToPredictor
        );
    }

    private StateSnapshot predictFromReplayBaseTo(LoopTimestamp timestamp) {
        if (!replayBaseValid) {
            return null;
        }
        return propagateFromArbitraryState(
                replayBasePose,
                replayBaseCovariance,
                replayBaseTimestamp,
                timestamp
        );
    }

    private StateSnapshot measurementUpdate(Pose3d priorPose,
                                            double[][] priorCovariance,
                                            Pose3d measurementPose,
                                            double measurementQuality,
                                            boolean projectedMeasurement,
                                            double projectedAgeSec,
                                            Pose3d predictorPose,
                                            LoopTimestamp timestamp) {
        if (!isFinitePlanarPose(priorPose)
                || !isFiniteCovariance(priorCovariance)
                || !isFinitePlanarPose(measurementPose)
                || !Double.isFinite(measurementQuality)
                || measurementQuality < 0.0
                || measurementQuality > 1.0
                || (projectedMeasurement
                && (!Double.isFinite(projectedAgeSec) || projectedAgeSec < 0.0))) {
            return null;
        }

        double[][] R = measurementCovariance(measurementQuality, projectedMeasurement, projectedAgeSec);
        if (!isFiniteCovariance(R)) {
            lastMeasurementPositionStdIn = Double.NaN;
            lastMeasurementHeadingStdRad = Double.NaN;
            return null;
        }
        double measurementPositionStd = Math.sqrt(Math.max(MIN_VARIANCE, R[0][0]));
        double measurementHeadingStd = Math.sqrt(Math.max(MIN_VARIANCE, R[2][2]));
        if (!Double.isFinite(measurementPositionStd)
                || !Double.isFinite(measurementHeadingStd)) {
            lastMeasurementPositionStdIn = Double.NaN;
            lastMeasurementHeadingStdRad = Double.NaN;
            return null;
        }
        lastMeasurementPositionStdIn = measurementPositionStd;
        lastMeasurementHeadingStdRad = measurementHeadingStd;

        double dx = measurementPose.xInches - priorPose.xInches;
        double dy = measurementPose.yInches - priorPose.yInches;
        double dHeading = MathUtil.wrapToPi(measurementPose.yawRad - priorPose.yawRad);
        double innovationPosition = Math.hypot(dx, dy);
        double innovationHeading = Math.abs(dHeading);

        if (!Double.isFinite(dx)
                || !Double.isFinite(dy)
                || !Double.isFinite(dHeading)
                || !Double.isFinite(innovationPosition)
                || !Double.isFinite(innovationHeading)) {
            lastInnovationPositionIn = Double.NaN;
            lastInnovationHeadingRad = Double.NaN;
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }

        lastInnovationPositionIn = innovationPosition;
        lastInnovationHeadingRad = innovationHeading;

        if (innovationPosition > cfg.maxCorrectionPositionInnovationIn
                || innovationHeading > cfg.maxCorrectionHeadingInnovationRad) {
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }

        double[] innovation = new double[]{dx, dy, dHeading};
        double[][] S = add(priorCovariance, R);
        if (!isFiniteCovariance(S)) {
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }
        double[][] sInv = invert3x3(S);
        if (!isFiniteCovariance(sInv)) {
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }

        double mahaSq = quadraticForm(innovation, sInv);
        if (!Double.isFinite(mahaSq)) {
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }
        lastInnovationMahalanobisSq = mahaSq;
        if (mahaSq > cfg.maxCorrectionMahalanobisSq) {
            return null;
        }

        double[][] K = mul(priorCovariance, sInv);
        double[] correction = mul(K, innovation);
        if (!isFiniteCovariance(K) || !isFiniteVector(correction)) {
            lastInnovationMahalanobisSq = Double.NaN;
            return null;
        }

        Pose3d updatedPose = new Pose3d(
                priorPose.xInches + correction[0],
                priorPose.yInches + correction[1],
                0.0,
                MathUtil.wrapToPi(priorPose.yawRad + correction[2]),
                0.0,
                0.0
        );
        if (!isFinitePlanarPose(updatedPose)) {
            return null;
        }

        double[][] I = identity();
        double[][] iMinusK = sub(I, K);
        double[][] updatedCovariance = add(
                mul(mul(iMinusK, priorCovariance), transpose(iMinusK)),
                mul(mul(K, R), transpose(K))
        );
        double[][] finiteCovariance = sanitizeCovariance(updatedCovariance);
        if (!isFiniteCovariance(finiteCovariance)) {
            return null;
        }

        return new StateSnapshot(
                planarize(updatedPose),
                finiteCovariance,
                predictorPose,
                timestamp
        );
    }

    private StateSnapshot predictStep(Pose3d priorPose,
                                      double[][] priorCovariance,
                                      Pose3d odomDelta,
                                      Pose3d resultingPredictorPose,
                                      LoopTimestamp timestamp) {
        if (!isFinitePlanarPose(priorPose)
                || !isFiniteCovariance(priorCovariance)
                || !isFinitePlanarPose(odomDelta)
                || (resultingPredictorPose != null
                && !isFinitePlanarPose(resultingPredictorPose))
                || timestamp == null) {
            return null;
        }
        Pose3d delta = planarize(odomDelta);
        Pose3d predictedPose = planarize(priorPose.then(delta));
        if (!isFinitePlanarPose(predictedPose)) {
            return null;
        }

        double theta = MathUtil.wrapToPi(priorPose.yawRad);
        double dxLocal = delta.xInches;
        double dyLocal = delta.yInches;
        double dHeading = MathUtil.wrapToPi(delta.yawRad);

        double[][] F = new double[][]{
                {1.0, 0.0, -Math.sin(theta) * dxLocal - Math.cos(theta) * dyLocal},
                {0.0, 1.0, Math.cos(theta) * dxLocal - Math.sin(theta) * dyLocal},
                {0.0, 0.0, 1.0}
        };
        if (!isFiniteCovariance(F)) {
            return null;
        }

        double translation = Math.hypot(dxLocal, dyLocal);
        double sigmaPos = cfg.predictorProcessPositionStdFloorIn
                + cfg.predictorProcessPositionStdPerIn * translation
                + cfg.predictorProcessPositionStdPerRad * Math.abs(dHeading);
        double sigmaHeading = cfg.predictorProcessHeadingStdFloorRad
                + cfg.predictorProcessHeadingStdPerIn * translation
                + cfg.predictorProcessHeadingStdPerRad * Math.abs(dHeading);
        double sigmaPosVariance = sigmaPos * sigmaPos;
        double sigmaHeadingVariance = sigmaHeading * sigmaHeading;
        if (!Double.isFinite(translation)
                || !Double.isFinite(sigmaPos)
                || !Double.isFinite(sigmaHeading)
                || !Double.isFinite(sigmaPosVariance)
                || !Double.isFinite(sigmaHeadingVariance)) {
            return null;
        }

        double c = Math.cos(theta);
        double s = Math.sin(theta);
        double[][] G = new double[][]{
                {c, -s, 0.0},
                {s, c, 0.0},
                {0.0, 0.0, 1.0}
        };
        double[][] qLocal = diagonal(sigmaPosVariance, sigmaPosVariance, sigmaHeadingVariance);
        double[][] Q = mul(mul(G, qLocal), transpose(G));

        double[][] predictedCovariance = add(
                mul(mul(F, priorCovariance), transpose(F)),
                Q
        );
        double[][] finiteCovariance = sanitizeCovariance(predictedCovariance);
        if (!isFiniteCovariance(finiteCovariance)) {
            return null;
        }

        return new StateSnapshot(
                predictedPose,
                finiteCovariance,
                resultingPredictorPose,
                timestamp
        );
    }

    private StateSnapshot propagateFromArbitraryState(Pose3d startPose,
                                                      double[][] startCovariance,
                                                      LoopTimestamp startTimestamp,
                                                      LoopTimestamp endTimestamp) {
        if (!isFinitePlanarPose(startPose) || !isFiniteCovariance(startCovariance)) {
            return null;
        }
        if (startTimestamp == null
                || endTimestamp == null
                || !startTimestamp.isAvailable()
                || !endTimestamp.isAvailable()) {
            return null;
        }
        double totalDurationSec = timestampElapsedSec(endTimestamp, startTimestamp);
        if (!Double.isFinite(totalDurationSec) || totalDurationSec < -TIMESTAMP_EPS_SEC) {
            return null;
        }
        if (Math.abs(totalDurationSec) <= TIMESTAMP_EPS_SEC) {
            Pose3d predictorPose = interpolatePredictorPose(endTimestamp);
            StateSnapshot stationary = new StateSnapshot(
                    planarize(startPose),
                    sanitizeCovariance(copy(startCovariance)),
                    predictorPose,
                    endTimestamp
            );
            return isFiniteState(stationary) ? stationary : null;
        }

        PredictorSample[] samples = predictorHistory.toArray(new PredictorSample[0]);
        if (samples.length == 0) {
            return null;
        }

        Pose3d startPredictorPose = interpolatePredictorPose(startTimestamp);
        Pose3d endPredictorPose = interpolatePredictorPose(endTimestamp);
        if (startPredictorPose == null || endPredictorPose == null) {
            return null;
        }

        Pose3d pose = planarize(startPose);
        double[][] covariance = sanitizeCovariance(copy(startCovariance));
        if (!isFiniteCovariance(covariance)) {
            return null;
        }
        Pose3d prevPredictorPose = startPredictorPose;
        LoopTimestamp prevTimestamp = startTimestamp;

        for (PredictorSample sample : samples) {
            double sampleFromStartSec = sample.timestamp.secondsSince(startTimestamp);
            double endFromSampleSec = endTimestamp.secondsSince(sample.timestamp);
            if (!Double.isFinite(sampleFromStartSec) || !Double.isFinite(endFromSampleSec)) {
                predictorHistory.clear();
                return null;
            }
            if (sampleFromStartSec <= TIMESTAMP_EPS_SEC) {
                continue;
            }
            if (endFromSampleSec <= TIMESTAMP_EPS_SEC) {
                break;
            }
            Pose3d nextPredictorPose = sample.pose;
            StateSnapshot predicted = predictStep(
                    pose,
                    covariance,
                    prevPredictorPose.inverse().then(nextPredictorPose),
                    nextPredictorPose,
                    sample.timestamp
            );
            if (!isFiniteState(predicted)) {
                return null;
            }
            pose = predicted.pose;
            covariance = predicted.covariance;
            prevPredictorPose = nextPredictorPose;
            prevTimestamp = sample.timestamp;
        }

        double remainingSec = endTimestamp.secondsSince(prevTimestamp);
        if (!Double.isFinite(remainingSec)) {
            predictorHistory.clear();
            return null;
        }
        if (remainingSec > TIMESTAMP_EPS_SEC) {
            StateSnapshot predicted = predictStep(
                    pose,
                    covariance,
                    prevPredictorPose.inverse().then(endPredictorPose),
                    endPredictorPose,
                    endTimestamp
            );
            if (!isFiniteState(predicted)) {
                return null;
            }
            pose = predicted.pose;
            covariance = predicted.covariance;
        }

        StateSnapshot result = new StateSnapshot(pose, covariance, endPredictorPose, endTimestamp);
        return isFiniteState(result) ? result : null;
    }

    private boolean pushPoseToPredictor(Pose3d candidatePose) {
        if (cfg.enablePushCorrectedPoseToPredictor && predictor instanceof PoseResetter) {
            try {
                ((PoseResetter) predictor).setPose(candidatePose.toPose2d());
            } catch (RuntimeException failure) {
                throw new PredictorPosePushFailure(failure);
            }
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
            lastPredictorPose = statePose;
            basePredictorPose = statePose;
            baseTimestamp = anchorTimestamp;
        } else if (currentPredictorPose != null
                && timestampAtOrAfter(currentPredictorTimestamp, anchorTimestamp)) {
            lastPredictorPose = currentPredictorPose;
            basePredictorPose = currentPredictorPose;
            baseTimestamp = currentPredictorTimestamp;
        } else {
            lastPredictorPose = currentPredictorPose != null ? currentPredictorPose : statePose;
            predictorHistory.clear();
            setReplayBase(
                    LoopTimestamp.unavailable(),
                    statePose,
                    stateCovariance,
                    Pose3d.zero()
            );
            markPredictorMotionCovered(anchorTimestamp);
            awaitPredictorMotionRebase();
            return;
        }

        resetPredictorHistory(baseTimestamp, basePredictorPose);
        setReplayBase(baseTimestamp, statePose, stateCovariance, basePredictorPose);
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
        double elapsedSec = timestampElapsedSec(candidate, reference);
        return Double.isFinite(elapsedSec) && elapsedSec >= 0.0;
    }

    private boolean shouldApplyPredictorMotion(MotionDelta predictorDelta, LoopClock clock) {
        if (predictorDelta == null || !predictorDelta.hasDelta) {
            return false;
        }

        double durationSec = timestampElapsedSec(
                predictorDelta.endTimestamp,
                predictorDelta.startTimestamp);
        if (!Double.isFinite(durationSec) || durationSec <= 0.0) {
            return false;
        }
        if (!isTimestampCurrent(predictorDelta.endTimestamp, clock)) {
            return false;
        }
        if (!lastCoveredPredictorMotionEndTimestamp.isAvailable()) {
            return true;
        }

        double elapsedSec = timestampElapsedSec(
                predictorDelta.endTimestamp,
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
            return currentPredictorPose != null ? predictorDelta.deltaPose : null;
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
        setReplayBase(
                predictorTimestamp,
                statePose,
                stateCovariance,
                currentPredictorPose
        );
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
                               Pose3d pose,
                               double[][] covariance,
                               Pose3d predictorPoseAtBase) {
        double[][] sanitized = isFiniteCovariance(covariance)
                ? sanitizeCovariance(copy(covariance))
                : null;
        replayBaseValid = timestamp != null
                && timestamp.isAvailable()
                && isFinitePlanarPose(pose)
                && isFiniteCovariance(sanitized)
                && isFinitePlanarPose(predictorPoseAtBase);
        replayBaseTimestamp = replayBaseValid ? timestamp : LoopTimestamp.unavailable();
        replayBasePose = replayBaseValid ? planarize(pose) : Pose3d.zero();
        replayBaseCovariance = replayBaseValid ? sanitized : diagonal(1.0, 1.0, 1.0);
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
        if (!Double.isFinite(measurementQuality)
                || measurementQuality < 0.0
                || measurementQuality > 1.0
                || (projectedMeasurement
                && (!Double.isFinite(projectedAgeSec) || projectedAgeSec < 0.0))) {
            return null;
        }
        double q = MathUtil.clamp(measurementQuality, 0.0, 1.0);
        double sigmaPos = cfg.correctionPositionStdFloorIn + cfg.correctionPositionStdScaleIn * (1.0 - q);
        double sigmaHeading = cfg.correctionHeadingStdFloorRad + cfg.correctionHeadingStdScaleRad * (1.0 - q);

        if (projectedMeasurement) {
            sigmaPos += cfg.projectedCorrectionPositionStdPerSec * Math.max(0.0, projectedAgeSec);
            sigmaHeading += cfg.projectedCorrectionHeadingStdPerSec * Math.max(0.0, projectedAgeSec);
        }

        sigmaPos = Math.max(1e-3, sigmaPos);
        sigmaHeading = Math.max(1e-6, sigmaHeading);
        double positionVariance = sigmaPos * sigmaPos;
        double headingVariance = sigmaHeading * sigmaHeading;
        if (!Double.isFinite(sigmaPos)
                || !Double.isFinite(sigmaHeading)
                || !Double.isFinite(positionVariance)
                || !Double.isFinite(headingVariance)) {
            return null;
        }
        return diagonal(positionVariance, positionVariance, headingVariance);
    }

    private double covarianceQuality(double[][] covariance) {
        if (!isFiniteCovariance(covariance)) {
            return Double.NaN;
        }
        double posStd = positionStdIn(covariance);
        double headingStd = headingStdRad(covariance);
        if (!Double.isFinite(posStd) || !Double.isFinite(headingStd)) {
            return Double.NaN;
        }
        double posScore = 1.0 - MathUtil.clamp01(posStd / cfg.qualityPositionStdScaleIn);
        double headingScore = 1.0 - MathUtil.clamp01(headingStd / cfg.qualityHeadingStdScaleRad);
        return MathUtil.clamp01(0.70 * posScore + 0.30 * headingScore);
    }

    private static double positionStdIn(double[][] covariance) {
        if (!isFiniteCovariance(covariance)) {
            return Double.NaN;
        }
        double meanVariance = 0.5 * covariance[0][0] + 0.5 * covariance[1][1];
        if (!Double.isFinite(meanVariance)) {
            return Double.NaN;
        }
        return Math.sqrt(Math.max(MIN_VARIANCE, meanVariance));
    }

    private static double headingStdRad(double[][] covariance) {
        if (!isFiniteCovariance(covariance)) {
            return Double.NaN;
        }
        return Math.sqrt(Math.max(MIN_VARIANCE, covariance[2][2]));
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
                || sinceFirstSec < -TIMESTAMP_EPS_SEC
                || untilLastSec < -TIMESTAMP_EPS_SEC) {
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
            if (querySinceNextSec > TIMESTAMP_EPS_SEC) {
                prev = next;
                continue;
            }
            if (Math.abs(querySinceNextSec) <= TIMESTAMP_EPS_SEC || next == prev) {
                return next.pose;
            }

            double dt = next.timestamp.secondsSince(prev.timestamp);
            if (!Double.isFinite(dt) || dt <= TIMESTAMP_EPS_SEC) {
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

    private static boolean isFinitePlanarPose(Pose3d pose) {
        return pose != null
                && Double.isFinite(pose.xInches)
                && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.yawRad);
    }

    private static boolean isFiniteCovariance(double[][] matrix) {
        if (matrix == null || matrix.length < 3) {
            return false;
        }
        for (int row = 0; row < 3; row++) {
            if (matrix[row] == null || matrix[row].length < 3) {
                return false;
            }
            for (int column = 0; column < 3; column++) {
                if (!Double.isFinite(matrix[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    private static boolean isFiniteVector(double[] vector) {
        return vector != null
                && vector.length >= 3
                && Double.isFinite(vector[0])
                && Double.isFinite(vector[1])
                && Double.isFinite(vector[2]);
    }

    private static boolean isFiniteState(StateSnapshot state) {
        return state != null
                && isFinitePlanarPose(state.pose)
                && isFiniteCovariance(state.covariance)
                && (state.predictorPose == null || isFinitePlanarPose(state.predictorPose))
                && state.timestamp != null;
    }

    private static double timestampAgeSec(LoopTimestamp timestamp, LoopClock clock) {
        if (timestamp == null) {
            return Double.NaN;
        }
        try {
            return timestamp.ageSec(clock);
        } catch (IllegalArgumentException differentClock) {
            return Double.NaN;
        }
    }

    private static final class PredictorPosePushFailure extends RuntimeException {
        final RuntimeException original;

        PredictorPosePushFailure(RuntimeException original) {
            super(original);
            this.original = original;
        }
    }

    private static boolean isTimestampCurrent(LoopTimestamp timestamp, LoopClock clock) {
        return Double.isFinite(timestampAgeSec(timestamp, clock));
    }

    private static double timestampElapsedSec(LoopTimestamp later, LoopTimestamp earlier) {
        if (later == null || earlier == null) {
            return Double.NaN;
        }
        try {
            return later.secondsSince(earlier);
        } catch (IllegalArgumentException differentClock) {
            return Double.NaN;
        }
    }

    private static boolean isUsablePredictorMotion(MotionDelta delta, LoopClock clock) {
        if (delta == null || !delta.hasDelta) {
            return false;
        }
        if (!isFinitePlanarPose(delta.deltaPose)
                || !Double.isFinite(delta.quality)
                || delta.quality < 0.0
                || delta.quality > 1.0
                || !isTimestampCurrent(delta.startTimestamp, clock)
                || !isTimestampCurrent(delta.endTimestamp, clock)) {
            return false;
        }
        double durationSec = timestampElapsedSec(delta.endTimestamp, delta.startTimestamp);
        return Double.isFinite(durationSec) && durationSec > 0.0;
    }

    private static void requireFiniteAuthoredPose(Pose2d pose, String context) {
        Objects.requireNonNull(pose, "pose");
        requireFinitePoseComponent(pose.xInches, context + ".xInches");
        requireFinitePoseComponent(pose.yInches, context + ".yInches");
        requireFinitePoseComponent(pose.headingRad, context + ".headingRad");
    }

    private static void requireFinitePoseComponent(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
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
                replayBasePose = Pose3d.zero();
                replayBaseCovariance = diagonal(1.0, 1.0, 1.0);
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
        if (!isFiniteCovariance(m)) {
            return null;
        }
        double[][] out = copy(m);
        for (int r = 0; r < 3; r++) {
            for (int c = r + 1; c < 3; c++) {
                double avg = 0.5 * out[r][c] + 0.5 * out[c][r];
                if (!Double.isFinite(avg)) {
                    return null;
                }
                out[r][c] = avg;
                out[c][r] = avg;
            }
            out[r][r] = Math.max(MIN_VARIANCE, out[r][r]);
        }
        return isFiniteCovariance(out) ? out : null;
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
                .addData(p + ".lastCorrectionAccepted", lastCorrectionAccepted)
                .addData(p + ".lastAcceptedCorrectionMeasurementTimestamp", lastAcceptedCorrectionMeasurementTimestamp)
                .addData(p + ".lastEvaluatedCorrectionTimestamp", lastEvaluatedCorrectionTimestamp)
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
                .addData(p + ".replayBaseTimestamp", replayBaseTimestamp)
                .addData(p + ".replayBasePose", replayBasePose)
                .addData(p + ".replayBasePredictorPose", replayBasePredictorPose)
                .addData(p + ".predictorHistorySize", predictorHistory.size())
                .addData(p + ".lastUpdateCycle", lastUpdateCycle)
                .addData(p + ".lastUpdateFailed", lastUpdateFailure != null)
                .addData(p + ".lastCoveredPredictorMotionEndTimestamp",
                        lastCoveredPredictorMotionEndTimestamp)
                .addData(p + ".predictorMotionRebaseState", predictorMotionRebaseState)
                .addData(p + ".predictorMotionRebasePose", predictorMotionRebasePose)
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
