package edu.ftcphoenix.fw.localization.fusion;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.localization.PoseResetter;

/**
 * Optional covariance-aware global localizer that combines smooth odometry with occasional absolute
 * vision corrections using a small planar EKF-style model.
 *
 * <p>This estimator exists alongside {@link OdometryTagFusionPoseEstimator}; it does <em>not</em>
 * replace the simpler complementary localizer. Teams that want a lightweight, easier-to-tune stack
 * should still start with {@link OdometryTagFusionPoseEstimator}. Teams that want an optional,
 * uncertainty-aware alternative can choose this class intentionally.</p>
 *
 * <h2>Model shape</h2>
 *
 * <ul>
 *   <li>State: planar {@code field -> robot} pose ({@code x}, {@code y}, {@code heading}).</li>
 *   <li>Prediction: odometry deltas are composed onto the current pose, while covariance grows as a
 *       function of translational / rotational motion.</li>
 *   <li>Measurement: the supplied vision estimator is treated as an absolute pose measurement with a
 *       dynamically sized measurement covariance derived from the vision estimate's quality.</li>
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
 *   <li>accurate odometry geometry / Pinpoint pod offsets, and</li>
 *   <li>a fixed-tag layout that excludes non-deterministic tags.</li>
 * </ol>
 *
 * <p>If those inputs are wrong, an EKF can produce a very smooth but still-wrong pose. That is why
 * Phoenix keeps this estimator optional and documents it separately instead of silently replacing the
 * simpler fusion implementation.</p>
 *
 * <h2>Latency compensation</h2>
 *
 * <p>When odometry history is available, accepted vision measurements are applied at their
 * measurement timestamp and the filter is replayed forward through stored odometry motion. If exact
 * replay is unavailable, the estimator falls back to a projected-now update path rather than
 * pretending the delayed measurement was captured at the current loop time.</p>
 */
public final class OdometryTagEkfPoseEstimator implements VisionCorrectionPoseEstimator {

    private static final double TIMESTAMP_EPS_SEC = 1e-6;
    private static final double MIN_VARIANCE = 1e-9;

    /**
     * Configuration for {@link OdometryTagEkfPoseEstimator}.
     *
     * <p>The defaults are intentionally conservative and should be treated as a starting point, not
     * as a substitute for real calibration. In particular, the odometry process-noise terms and the
     * vision measurement-noise terms only make sense once camera extrinsics and odometry geometry are
     * already trustworthy.</p>
     */
    public static final class Config {
        /**
         * Reject vision measurements older than this (seconds).
         */
        public double maxVisionAgeSec = 0.25;
        /**
         * Reject vision measurements with quality below this (0..1).
         */
        public double minVisionQuality = 0.05;

        /**
         * Hard gate on planar innovation magnitude before the EKF update is even attempted.
         *
         * <p>This is a simple reliability guardrail: a wildly contradictory measurement should not be
         * allowed to rely solely on covariance math for rejection.</p>
         */
        public double maxVisionPositionInnovationIn = 24.0;

        /**
         * Hard gate on heading innovation magnitude before the EKF update is attempted.
         */
        public double maxVisionHeadingInnovationRad = Math.toRadians(60.0);

        /**
         * Maximum allowed Mahalanobis distance squared for a vision innovation.
         *
         * <p>Higher values make the filter more permissive; lower values reject more contradictory
         * measurements. This is a reliability gate, not a scoring function.</p>
         */
        public double maxVisionMahalanobisSq = 25.0;

        /**
         * If true, the filter may initialize from a fresh vision measurement.
         */
        public boolean enableInitializeFromVision = true;

        /**
         * If true, accepted filtered poses are pushed back into the odometry estimator when it
         * supports {@link PoseResetter}. This keeps odometry and the filtered state aligned.
         */
        public boolean enablePushFilteredPoseToOdometry = true;

        /**
         * If true, accepted vision measurements are updated at their measurement timestamp and the
         * odometry prediction is replayed forward to the current loop when history is available.
         */
        public boolean enableLatencyCompensation = true;

        /**
         * How much recent odometry history (seconds) to retain for measurement-time replay.
         *
         * <p>When latency compensation is enabled, this should be at least as large as
         * {@link #maxVisionAgeSec} so every still-acceptable vision frame can be replayed through
         * the stored odometry history.</p>
         */
        public double odomHistorySec = 1.0;

        /**
         * Initial planar position standard deviation (inches) when the filter initializes from
         * odometry alone before any absolute vision correction has been accepted.
         */
        public double initialPositionStdIn = 18.0;

        /**
         * Initial heading standard deviation (radians) when the filter initializes from odometry
         * alone before any absolute vision correction has been accepted.
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
         * Base translational process-noise standard deviation added on every odometry predict step.
         */
        public double odomProcessPositionStdFloorIn = 0.10;
        /**
         * Extra translational process-noise standard deviation per inch of odometry travel.
         */
        public double odomProcessPositionStdPerIn = 0.025;
        /**
         * Extra translational process-noise standard deviation per radian of odometry heading change.
         */
        public double odomProcessPositionStdPerRad = 0.50;

        /**
         * Base heading process-noise standard deviation added on every odometry predict step.
         */
        public double odomProcessHeadingStdFloorRad = Math.toRadians(0.25);
        /**
         * Extra heading process-noise standard deviation per inch of odometry travel.
         */
        public double odomProcessHeadingStdPerIn = Math.toRadians(0.08);
        /**
         * Extra heading process-noise standard deviation per radian of odometry heading change.
         */
        public double odomProcessHeadingStdPerRad = 0.05;

        /**
         * Best-case planar position measurement standard deviation (inches) for a high-quality vision pose.
         */
        public double visionPositionStdFloorIn = 1.25;
        /**
         * Additional planar position measurement standard deviation (inches) added as vision quality falls toward 0.
         */
        public double visionPositionStdScaleIn = 10.0;
        /**
         * Additional planar position standard deviation (inches/sec) when a delayed frame must be projected to "now" instead of replayed.
         */
        public double projectedVisionPositionStdPerSec = 4.0;

        /**
         * Best-case heading measurement standard deviation (radians) for a high-quality vision pose.
         */
        public double visionHeadingStdFloorRad = Math.toRadians(2.0);
        /**
         * Additional heading measurement standard deviation (radians) added as vision quality falls toward 0.
         */
        public double visionHeadingStdScaleRad = Math.toRadians(18.0);
        /**
         * Additional heading standard deviation (radians/sec) when a delayed frame must be projected to "now" instead of replayed.
         */
        public double projectedVisionHeadingStdPerSec = Math.toRadians(12.0);

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
                    : "OdometryTagEkfPoseEstimator.Config";

            requireFiniteNonNegative(maxVisionAgeSec, p + ".maxVisionAgeSec");
            requireFiniteInRange(minVisionQuality, 0.0, 1.0, p + ".minVisionQuality");
            requireFinitePositive(maxVisionPositionInnovationIn, p + ".maxVisionPositionInnovationIn");
            requireFinitePositive(maxVisionHeadingInnovationRad, p + ".maxVisionHeadingInnovationRad");
            requireFinitePositive(maxVisionMahalanobisSq, p + ".maxVisionMahalanobisSq");
            requireFiniteNonNegative(odomHistorySec, p + ".odomHistorySec");
            requireFinitePositive(initialPositionStdIn, p + ".initialPositionStdIn");
            requireFinitePositive(initialHeadingStdRad, p + ".initialHeadingStdRad");
            requireFinitePositive(manualPosePositionStdIn, p + ".manualPosePositionStdIn");
            requireFinitePositive(manualPoseHeadingStdRad, p + ".manualPoseHeadingStdRad");
            requireFinitePositive(odomProcessPositionStdFloorIn, p + ".odomProcessPositionStdFloorIn");
            requireFiniteNonNegative(odomProcessPositionStdPerIn, p + ".odomProcessPositionStdPerIn");
            requireFiniteNonNegative(odomProcessPositionStdPerRad, p + ".odomProcessPositionStdPerRad");
            requireFinitePositive(odomProcessHeadingStdFloorRad, p + ".odomProcessHeadingStdFloorRad");
            requireFiniteNonNegative(odomProcessHeadingStdPerIn, p + ".odomProcessHeadingStdPerIn");
            requireFiniteNonNegative(odomProcessHeadingStdPerRad, p + ".odomProcessHeadingStdPerRad");
            requireFinitePositive(visionPositionStdFloorIn, p + ".visionPositionStdFloorIn");
            requireFiniteNonNegative(visionPositionStdScaleIn, p + ".visionPositionStdScaleIn");
            requireFiniteNonNegative(projectedVisionPositionStdPerSec, p + ".projectedVisionPositionStdPerSec");
            requireFinitePositive(visionHeadingStdFloorRad, p + ".visionHeadingStdFloorRad");
            requireFiniteNonNegative(visionHeadingStdScaleRad, p + ".visionHeadingStdScaleRad");
            requireFiniteNonNegative(projectedVisionHeadingStdPerSec, p + ".projectedVisionHeadingStdPerSec");
            requireFinitePositive(qualityPositionStdScaleIn, p + ".qualityPositionStdScaleIn");
            requireFinitePositive(qualityHeadingStdScaleRad, p + ".qualityHeadingStdScaleRad");

            if (enableLatencyCompensation && odomHistorySec + TIMESTAMP_EPS_SEC < maxVisionAgeSec) {
                throw new IllegalArgumentException(
                        p + ".odomHistorySec must be >= maxVisionAgeSec when latency compensation is enabled"
                                + " (increase odomHistorySec or reduce maxVisionAgeSec)"
                );
            }
        }

        /**
         * Returns a deep copy of this config.
         */
        public Config copy() {
            Config c = new Config();
            c.maxVisionAgeSec = this.maxVisionAgeSec;
            c.minVisionQuality = this.minVisionQuality;
            c.maxVisionPositionInnovationIn = this.maxVisionPositionInnovationIn;
            c.maxVisionHeadingInnovationRad = this.maxVisionHeadingInnovationRad;
            c.maxVisionMahalanobisSq = this.maxVisionMahalanobisSq;
            c.enableInitializeFromVision = this.enableInitializeFromVision;
            c.enablePushFilteredPoseToOdometry = this.enablePushFilteredPoseToOdometry;
            c.enableLatencyCompensation = this.enableLatencyCompensation;
            c.odomHistorySec = this.odomHistorySec;
            c.initialPositionStdIn = this.initialPositionStdIn;
            c.initialHeadingStdRad = this.initialHeadingStdRad;
            c.manualPosePositionStdIn = this.manualPosePositionStdIn;
            c.manualPoseHeadingStdRad = this.manualPoseHeadingStdRad;
            c.odomProcessPositionStdFloorIn = this.odomProcessPositionStdFloorIn;
            c.odomProcessPositionStdPerIn = this.odomProcessPositionStdPerIn;
            c.odomProcessPositionStdPerRad = this.odomProcessPositionStdPerRad;
            c.odomProcessHeadingStdFloorRad = this.odomProcessHeadingStdFloorRad;
            c.odomProcessHeadingStdPerIn = this.odomProcessHeadingStdPerIn;
            c.odomProcessHeadingStdPerRad = this.odomProcessHeadingStdPerRad;
            c.visionPositionStdFloorIn = this.visionPositionStdFloorIn;
            c.visionPositionStdScaleIn = this.visionPositionStdScaleIn;
            c.projectedVisionPositionStdPerSec = this.projectedVisionPositionStdPerSec;
            c.visionHeadingStdFloorRad = this.visionHeadingStdFloorRad;
            c.visionHeadingStdScaleRad = this.visionHeadingStdScaleRad;
            c.projectedVisionHeadingStdPerSec = this.projectedVisionHeadingStdPerSec;
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

    private static final class OdomSample {
        final double timestampSec;
        final Pose3d pose;

        OdomSample(double timestampSec, Pose3d pose) {
            this.timestampSec = timestampSec;
            this.pose = pose;
        }
    }

    private static final class StateSnapshot {
        final Pose3d pose;
        final double[][] covariance;
        final Pose3d odomPose;
        final double timestampSec;

        StateSnapshot(Pose3d pose, double[][] covariance, Pose3d odomPose, double timestampSec) {
            this.pose = pose;
            this.covariance = covariance;
            this.odomPose = odomPose;
            this.timestampSec = timestampSec;
        }
    }

    private final PoseEstimator odometry;
    private final PoseEstimator vision;
    private final Config cfg;

    private boolean initialized = false;
    private boolean visionEnabled = true;

    private Pose3d statePose = Pose3d.zero();
    private double[][] stateCovariance = diagonal(1.0, 1.0, 1.0);
    private Pose3d lastOdomPose = Pose3d.zero();

    private PoseEstimate lastEstimate = PoseEstimate.noPose(0.0);
    private final Deque<OdomSample> odomHistory = new ArrayDeque<OdomSample>();

    private boolean replayBaseValid = false;
    private double replayBaseTimestampSec = Double.NaN;
    private Pose3d replayBasePose = Pose3d.zero();
    private double[][] replayBaseCovariance = diagonal(1.0, 1.0, 1.0);
    private Pose3d replayBaseOdomPose = Pose3d.zero();

    // Debug/telemetry helpers.
    private double lastVisionAcceptedSec = Double.NaN;
    private double lastAcceptedVisionMeasurementTimestampSec = Double.NaN;
    private double lastEvaluatedVisionTimestampSec = Double.NaN;
    private Pose3d lastVisionPose = Pose3d.zero();
    private Pose3d lastLatencyCompensatedVisionPose = Pose3d.zero();
    private Pose3d lastReplayReferencePose = Pose3d.zero();
    private boolean lastVisionUsedReplay = false;
    private int acceptedVisionCount = 0;
    private int rejectedVisionCount = 0;
    private int skippedDuplicateVisionCount = 0;
    private int skippedOutOfOrderVisionCount = 0;
    private int replayedVisionCount = 0;
    private int projectedVisionCount = 0;

    private double lastInnovationPositionIn = Double.NaN;
    private double lastInnovationHeadingRad = Double.NaN;
    private double lastInnovationMahalanobisSq = Double.NaN;
    private double lastMeasurementPositionStdIn = Double.NaN;
    private double lastMeasurementHeadingStdRad = Double.NaN;

    /**
     * Creates an EKF-style localizer with default configuration.
     */
    public OdometryTagEkfPoseEstimator(PoseEstimator odometry, PoseEstimator vision) {
        this(odometry, vision, Config.defaults());
    }

    /**
     * Creates an EKF-style localizer with explicit configuration.
     */
    public OdometryTagEkfPoseEstimator(PoseEstimator odometry, PoseEstimator vision, Config cfg) {
        if (odometry == null) {
            throw new IllegalArgumentException("odometry must not be null");
        }
        if (vision == null) {
            throw new IllegalArgumentException("vision must not be null");
        }
        this.odometry = odometry;
        this.vision = vision;
        Config base = (cfg != null) ? cfg : Config.defaults();
        this.cfg = base.validatedCopy("OdometryTagEkfPoseEstimator.Config");
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isVisionEnabled() {
        return visionEnabled;
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
     * The timestamp (clock.nowSec) when vision was last accepted, or NaN if never.
     */
    public double getLastVisionAcceptedSec() {
        return lastVisionAcceptedSec;
    }

    /**
     * Measurement timestamp of the most recently accepted vision frame, or NaN if never.
     */
    public double getLastAcceptedVisionMeasurementTimestampSec() {
        return lastAcceptedVisionMeasurementTimestampSec;
    }

    /**
     * Last accepted vision pose at the frame's own measurement timestamp (planarized).
     */
    public Pose3d getLastVisionPose() {
        return lastVisionPose;
    }

    /**
     * Returns whether the most recently accepted correction used measurement-time replay.
     */
    public boolean wasLastVisionCorrectionReplay() {
        return lastVisionUsedReplay;
    }

    /**
     * Returns the total number of accepted vision corrections.
     */
    public int getAcceptedVisionCount() {
        return acceptedVisionCount;
    }

    /**
     * Returns the total number of newly-observed vision measurements that were rejected.
     */
    public int getRejectedVisionCount() {
        return rejectedVisionCount;
    }

    /**
     * Returns how many duplicate frame timestamps were skipped.
     */
    public int getSkippedDuplicateVisionCount() {
        return skippedDuplicateVisionCount;
    }

    /**
     * Returns how many older-than-last-evaluated vision timestamps were skipped.
     */
    public int getSkippedOutOfOrderVisionCount() {
        return skippedOutOfOrderVisionCount;
    }

    /**
     * Returns how many accepted corrections used measurement-time replay.
     */
    public int getReplayedVisionCount() {
        return replayedVisionCount;
    }

    /**
     * Returns how many accepted corrections fell back to a simple projected-now path.
     */
    public int getProjectedVisionCount() {
        return projectedVisionCount;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public VisionCorrectionStats getVisionCorrectionStats() {
        return new VisionCorrectionStats(
                acceptedVisionCount,
                rejectedVisionCount,
                skippedDuplicateVisionCount,
                skippedOutOfOrderVisionCount,
                replayedVisionCount,
                projectedVisionCount,
                lastVisionAcceptedSec,
                lastAcceptedVisionMeasurementTimestampSec,
                lastEvaluatedVisionTimestampSec,
                lastVisionUsedReplay
        );
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        final double nowSec = clock != null ? clock.nowSec() : 0.0;

        odometry.update(clock);
        vision.update(clock);

        final PoseEstimate odomEst = odometry.getEstimate();
        final PoseEstimate visEst = vision.getEstimate();
        final Pose3d currentOdomPose = (odomEst != null && odomEst.hasPose)
                ? planarize(odomEst.fieldToRobotPose)
                : null;
        final double currentOdomTimestampSec = estimateTimestampOr(odomEst, nowSec);

        if (currentOdomPose != null) {
            recordOdomSample(currentOdomTimestampSec, currentOdomPose);
        }

        boolean evaluatedVisionThisLoop = false;

        if (!initialized) {
            boolean initializedFromVision = false;
            if (visionEnabled && cfg.enableInitializeFromVision && shouldEvaluateVisionMeasurement(visEst)) {
                evaluatedVisionThisLoop = true;
                if (isVisionAcceptable(visEst, nowSec)) {
                    initializeFromVision(visEst, currentOdomPose, currentOdomTimestampSec, nowSec);
                    initializedFromVision = initialized;
                } else {
                    rejectedVisionCount++;
                }
            }

            if (!initialized && currentOdomPose != null) {
                statePose = currentOdomPose;
                stateCovariance = initialOdometryCovariance();
                initialized = true;
                lastOdomPose = statePose;
                resetOdomHistory(currentOdomTimestampSec, statePose);
                setReplayBase(currentOdomTimestampSec, statePose, stateCovariance, statePose);
            } else if (!initialized && !initializedFromVision) {
                lastEstimate = PoseEstimate.noPose(nowSec);
                return;
            }
        } else {
            if (currentOdomPose != null) {
                Pose3d delta = lastOdomPose.inverse().then(currentOdomPose);
                StateSnapshot predicted = predictStep(statePose, stateCovariance, delta, currentOdomPose, currentOdomTimestampSec);
                statePose = predicted.pose;
                stateCovariance = predicted.covariance;
                lastOdomPose = currentOdomPose;
            }
        }

        if (visionEnabled && !evaluatedVisionThisLoop && shouldEvaluateVisionMeasurement(visEst)) {
            if (!isVisionAcceptable(visEst, nowSec)) {
                rejectedVisionCount++;
            } else {
                maybeApplyVisionCorrection(visEst, currentOdomPose, currentOdomTimestampSec, nowSec);
            }
        }

        lastEstimate = new PoseEstimate(statePose, true, covarianceQuality(stateCovariance), 0.0, nowSec);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * {@inheritDoc}
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

        Pose3d currentOdomPose = null;
        double currentOdomTimestampSec = nowSec;
        PoseEstimate odomEst = odometry.getEstimate();
        if (odomEst != null && odomEst.hasPose) {
            currentOdomPose = planarize(odomEst.fieldToRobotPose);
            currentOdomTimestampSec = estimateTimestampOr(odomEst, nowSec);
        }

        boolean pushedToOdometry = pushFilteredPoseToOdometry();
        clearRecentVisionState();
        rebaseAfterPoseChange(currentOdomTimestampSec, currentOdomPose, pushedToOdometry);
        lastEstimate = new PoseEstimate(statePose, true, covarianceQuality(stateCovariance), 0.0, nowSec);
    }

    private void initializeFromVision(PoseEstimate visEst,
                                      Pose3d currentOdomPose,
                                      double currentOdomTimestampSec,
                                      double nowSec) {
        Pose3d visionPoseAtMeasurement = planarize(visEst.fieldToRobotPose);
        lastVisionPose = visionPoseAtMeasurement;
        lastReplayReferencePose = visionPoseAtMeasurement;
        lastInnovationPositionIn = Double.NaN;
        lastInnovationHeadingRad = Double.NaN;
        lastInnovationMahalanobisSq = Double.NaN;

        StateSnapshot currentState = null;
        boolean usedReplay = false;
        if (cfg.enableLatencyCompensation) {
            double[][] measCov = measurementCovariance(visEst.quality, false, 0.0);
            currentState = propagateFromArbitraryState(
                    visionPoseAtMeasurement,
                    measCov,
                    visEst.timestampSec,
                    currentOdomTimestampSec
            );
            usedReplay = currentState != null && currentOdomTimestampSec > visEst.timestampSec + TIMESTAMP_EPS_SEC;
        }

        if (currentState == null) {
            Pose3d projectedVisionPoseNow = projectVisionPoseToNow(visionPoseAtMeasurement, visEst.timestampSec, currentOdomPose);
            double projectedAgeSec = Math.max(0.0, nowSec - visEst.timestampSec);
            statePose = projectedVisionPoseNow;
            stateCovariance = measurementCovariance(visEst.quality, true, projectedAgeSec);
            lastLatencyCompensatedVisionPose = projectedVisionPoseNow;
            lastVisionUsedReplay = false;
            projectedVisionCount++;
        } else {
            statePose = currentState.pose;
            stateCovariance = currentState.covariance;
            lastLatencyCompensatedVisionPose = currentState.pose;
            lastVisionUsedReplay = usedReplay;
            if (usedReplay) {
                replayedVisionCount++;
            } else {
                projectedVisionCount++;
            }
        }

        initialized = true;
        lastVisionAcceptedSec = nowSec;
        lastAcceptedVisionMeasurementTimestampSec = visEst.timestampSec;
        acceptedVisionCount++;

        boolean pushedToOdometry = pushFilteredPoseToOdometry();
        rebaseAfterPoseChange(currentOdomTimestampSec, currentOdomPose, pushedToOdometry);
    }

    private boolean shouldEvaluateVisionMeasurement(PoseEstimate visEst) {
        if (visEst == null || !visEst.hasPose) {
            return false;
        }
        double ts = visEst.timestampSec;
        if (!Double.isFinite(ts)) {
            return false;
        }
        if (Double.isNaN(lastEvaluatedVisionTimestampSec)) {
            lastEvaluatedVisionTimestampSec = ts;
            return true;
        }
        if (ts > lastEvaluatedVisionTimestampSec + TIMESTAMP_EPS_SEC) {
            lastEvaluatedVisionTimestampSec = ts;
            return true;
        }
        if (Math.abs(ts - lastEvaluatedVisionTimestampSec) <= TIMESTAMP_EPS_SEC) {
            skippedDuplicateVisionCount++;
        } else {
            skippedOutOfOrderVisionCount++;
        }
        return false;
    }

    private boolean isVisionAcceptable(PoseEstimate visEst, double nowSec) {
        if (visEst == null || !visEst.hasPose || visEst.fieldToRobotPose == null) {
            return false;
        }
        if (!Double.isFinite(visEst.timestampSec)) {
            return false;
        }
        if (visEst.timestampSec > nowSec + TIMESTAMP_EPS_SEC) {
            return false;
        }

        if (cfg.maxVisionAgeSec > 0.0) {
            double age = nowSec - visEst.timestampSec;
            if (!Double.isFinite(age) || age < -TIMESTAMP_EPS_SEC || age > cfg.maxVisionAgeSec) {
                return false;
            }
        }

        if (!Double.isFinite(visEst.quality) || visEst.quality < cfg.minVisionQuality) {
            return false;
        }

        Pose3d p = visEst.fieldToRobotPose;
        return !(Double.isNaN(p.xInches) || Double.isNaN(p.yInches) || Double.isNaN(p.yawRad));
    }

    private void maybeApplyVisionCorrection(PoseEstimate visEst,
                                            Pose3d currentOdomPose,
                                            double currentOdomTimestampSec,
                                            double nowSec) {
        Pose3d visionPoseAtMeasurement = planarize(visEst.fieldToRobotPose);
        Pose3d projectedVisionPoseAtNow = projectVisionPoseToNow(visionPoseAtMeasurement, visEst.timestampSec, currentOdomPose);

        lastVisionPose = visionPoseAtMeasurement;
        lastLatencyCompensatedVisionPose = projectedVisionPoseAtNow;

        if (cfg.enableLatencyCompensation
                && replayBaseValid
                && Double.isFinite(replayBaseTimestampSec)
                && visEst.timestampSec + TIMESTAMP_EPS_SEC < replayBaseTimestampSec) {
            lastReplayReferencePose = replayBasePose;
            lastVisionUsedReplay = false;
            rejectedVisionCount++;
            return;
        }

        StateSnapshot correctedNow = null;
        boolean usedReplay = false;

        if (cfg.enableLatencyCompensation) {
            StateSnapshot predictedAtMeasurement = predictFromReplayBaseTo(visEst.timestampSec);
            if (predictedAtMeasurement != null) {
                lastReplayReferencePose = predictedAtMeasurement.pose;
                StateSnapshot correctedAtMeasurement = measurementUpdate(
                        predictedAtMeasurement.pose,
                        predictedAtMeasurement.covariance,
                        visionPoseAtMeasurement,
                        visEst.quality,
                        false,
                        0.0,
                        predictedAtMeasurement.odomPose,
                        visEst.timestampSec
                );
                if (correctedAtMeasurement != null) {
                    StateSnapshot replayed = propagateFromArbitraryState(
                            correctedAtMeasurement.pose,
                            correctedAtMeasurement.covariance,
                            visEst.timestampSec,
                            currentOdomTimestampSec
                    );
                    correctedNow = (replayed != null) ? replayed : correctedAtMeasurement;
                    usedReplay = true;
                } else {
                    rejectedVisionCount++;
                    lastVisionUsedReplay = true;
                    return;
                }
            }
        }

        if (correctedNow == null) {
            lastReplayReferencePose = statePose;
            double projectedAgeSec = Math.max(0.0, nowSec - visEst.timestampSec);
            correctedNow = measurementUpdate(
                    statePose,
                    stateCovariance,
                    projectedVisionPoseAtNow,
                    visEst.quality,
                    true,
                    projectedAgeSec,
                    currentOdomPose,
                    currentOdomTimestampSec
            );
            if (correctedNow == null) {
                rejectedVisionCount++;
                lastVisionUsedReplay = false;
                return;
            }
        }

        statePose = correctedNow.pose;
        stateCovariance = correctedNow.covariance;
        lastVisionAcceptedSec = nowSec;
        lastAcceptedVisionMeasurementTimestampSec = visEst.timestampSec;
        lastVisionUsedReplay = usedReplay;
        acceptedVisionCount++;
        if (usedReplay) {
            replayedVisionCount++;
        } else {
            projectedVisionCount++;
        }

        boolean pushedToOdometry = pushFilteredPoseToOdometry();
        rebaseAfterPoseChange(currentOdomTimestampSec, currentOdomPose, pushedToOdometry);
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
                                            Pose3d odomPose,
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

        if (innovationPosition > cfg.maxVisionPositionInnovationIn
                || innovationHeading > cfg.maxVisionHeadingInnovationRad) {
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
        if (!Double.isFinite(mahaSq) || mahaSq > cfg.maxVisionMahalanobisSq) {
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
                odomPose,
                timestampSec
        );
    }

    private StateSnapshot predictStep(Pose3d priorPose,
                                      double[][] priorCovariance,
                                      Pose3d odomDelta,
                                      Pose3d resultingOdomPose,
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
        double sigmaPos = cfg.odomProcessPositionStdFloorIn
                + cfg.odomProcessPositionStdPerIn * translation
                + cfg.odomProcessPositionStdPerRad * Math.abs(dHeading);
        double sigmaHeading = cfg.odomProcessHeadingStdFloorRad
                + cfg.odomProcessHeadingStdPerIn * translation
                + cfg.odomProcessHeadingStdPerRad * Math.abs(dHeading);

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
                resultingOdomPose,
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
            Pose3d odomPose = interpolateOdomPose(endTimestampSec);
            return new StateSnapshot(planarize(startPose), sanitizeCovariance(copy(startCovariance)), odomPose, endTimestampSec);
        }

        OdomSample[] samples = odomHistory.toArray(new OdomSample[0]);
        if (samples.length == 0) {
            return null;
        }

        Pose3d startOdomPose = interpolateOdomPose(startTimestampSec);
        Pose3d endOdomPose = interpolateOdomPose(endTimestampSec);
        if (startOdomPose == null || endOdomPose == null) {
            return null;
        }

        Pose3d pose = planarize(startPose);
        double[][] covariance = sanitizeCovariance(copy(startCovariance));
        Pose3d prevOdomPose = startOdomPose;
        double prevTimestampSec = startTimestampSec;

        for (OdomSample sample : samples) {
            if (sample.timestampSec <= startTimestampSec + TIMESTAMP_EPS_SEC) {
                continue;
            }
            if (sample.timestampSec >= endTimestampSec - TIMESTAMP_EPS_SEC) {
                break;
            }
            Pose3d nextOdomPose = sample.pose;
            StateSnapshot predicted = predictStep(
                    pose,
                    covariance,
                    prevOdomPose.inverse().then(nextOdomPose),
                    nextOdomPose,
                    sample.timestampSec
            );
            pose = predicted.pose;
            covariance = predicted.covariance;
            prevOdomPose = nextOdomPose;
            prevTimestampSec = sample.timestampSec;
        }

        if (endTimestampSec > prevTimestampSec + TIMESTAMP_EPS_SEC) {
            StateSnapshot predicted = predictStep(
                    pose,
                    covariance,
                    prevOdomPose.inverse().then(endOdomPose),
                    endOdomPose,
                    endTimestampSec
            );
            pose = predicted.pose;
            covariance = predicted.covariance;
        }

        return new StateSnapshot(pose, covariance, endOdomPose, endTimestampSec);
    }

    private boolean pushFilteredPoseToOdometry() {
        if (cfg.enablePushFilteredPoseToOdometry && odometry instanceof PoseResetter) {
            ((PoseResetter) odometry).setPose(statePose.toPose2d());
            return true;
        }
        return false;
    }

    private void clearRecentVisionState() {
        lastVisionAcceptedSec = Double.NaN;
        lastAcceptedVisionMeasurementTimestampSec = Double.NaN;
        lastVisionUsedReplay = false;
    }

    private void rebaseAfterPoseChange(double timestampSec, Pose3d currentOdomPose, boolean pushedToOdometry) {
        Pose3d baseOdomPose;
        if (pushedToOdometry) {
            lastOdomPose = statePose;
            baseOdomPose = statePose;
        } else if (currentOdomPose != null) {
            lastOdomPose = currentOdomPose;
            baseOdomPose = currentOdomPose;
        } else {
            lastOdomPose = statePose;
            baseOdomPose = statePose;
        }

        resetOdomHistory(timestampSec, baseOdomPose);
        setReplayBase(timestampSec, statePose, stateCovariance, baseOdomPose);
    }

    private void setReplayBase(double timestampSec,
                               Pose3d pose,
                               double[][] covariance,
                               Pose3d odomPoseAtBase) {
        replayBaseValid = Double.isFinite(timestampSec) && pose != null && covariance != null && odomPoseAtBase != null;
        replayBaseTimestampSec = replayBaseValid ? timestampSec : Double.NaN;
        replayBasePose = replayBaseValid ? planarize(pose) : Pose3d.zero();
        replayBaseCovariance = replayBaseValid ? sanitizeCovariance(copy(covariance)) : diagonal(1.0, 1.0, 1.0);
        replayBaseOdomPose = replayBaseValid ? planarize(odomPoseAtBase) : Pose3d.zero();
    }

    private double[][] initialOdometryCovariance() {
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
        double sigmaPos = cfg.visionPositionStdFloorIn + cfg.visionPositionStdScaleIn * (1.0 - q);
        double sigmaHeading = cfg.visionHeadingStdFloorRad + cfg.visionHeadingStdScaleRad * (1.0 - q);

        if (projectedMeasurement) {
            sigmaPos += cfg.projectedVisionPositionStdPerSec * Math.max(0.0, projectedAgeSec);
            sigmaHeading += cfg.projectedVisionHeadingStdPerSec * Math.max(0.0, projectedAgeSec);
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

    private void recordOdomSample(double timestampSec, Pose3d odomPose) {
        if (!Double.isFinite(timestampSec) || odomPose == null) {
            return;
        }

        OdomSample last = odomHistory.peekLast();
        if (last != null && timestampSec <= last.timestampSec) {
            odomHistory.clear();
            odomHistory.addLast(new OdomSample(timestampSec, planarize(odomPose)));
            if (initialized) {
                setReplayBase(timestampSec, statePose, stateCovariance, planarize(odomPose));
                lastOdomPose = planarize(odomPose);
            }
            return;
        }

        odomHistory.addLast(new OdomSample(timestampSec, planarize(odomPose)));
        pruneOdomHistory(timestampSec);
    }

    private void pruneOdomHistory(double nowSec) {
        double keepSec = Math.max(0.0, cfg.odomHistorySec);
        if (keepSec <= 0.0) {
            while (odomHistory.size() > 1) {
                odomHistory.removeFirst();
            }
            return;
        }

        double minTime = nowSec - keepSec;
        while (odomHistory.size() > 2) {
            OdomSample[] samples = odomHistory.toArray(new OdomSample[0]);
            if (samples.length < 2 || samples[1].timestampSec >= minTime) {
                break;
            }
            odomHistory.removeFirst();
        }
    }

    private void resetOdomHistory(double timestampSec, Pose3d pose) {
        odomHistory.clear();
        if (Double.isFinite(timestampSec) && pose != null) {
            odomHistory.addLast(new OdomSample(timestampSec, planarize(pose)));
        }
    }

    private Pose3d projectVisionPoseToNow(Pose3d visionPoseAtMeasurement,
                                          double measurementTimestampSec,
                                          Pose3d currentOdomPose) {
        if (visionPoseAtMeasurement == null || currentOdomPose == null) {
            return visionPoseAtMeasurement;
        }
        if (!Double.isFinite(measurementTimestampSec)) {
            return visionPoseAtMeasurement;
        }

        Pose3d odomAtMeasurement = interpolateOdomPose(measurementTimestampSec);
        if (odomAtMeasurement == null) {
            return visionPoseAtMeasurement;
        }

        Pose3d odomDeltaSinceMeasurement = odomAtMeasurement.inverse().then(currentOdomPose);
        return planarize(visionPoseAtMeasurement.then(odomDeltaSinceMeasurement));
    }

    private Pose3d interpolateOdomPose(double timestampSec) {
        if (odomHistory.isEmpty()) {
            return null;
        }

        OdomSample[] samples = odomHistory.toArray(new OdomSample[0]);
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

        OdomSample prev = samples[0];
        for (OdomSample next : samples) {
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
     * Debug helper: emit current EKF state, covariance summary, and recent vision-gating details.
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "ekf" : prefix;

        dbg.addLine(p)
                .addData(p + ".initialized", initialized)
                .addData(p + ".visionEnabled", visionEnabled)
                .addData(p + ".acceptedVisionCount", acceptedVisionCount)
                .addData(p + ".rejectedVisionCount", rejectedVisionCount)
                .addData(p + ".skippedDuplicateVisionCount", skippedDuplicateVisionCount)
                .addData(p + ".skippedOutOfOrderVisionCount", skippedOutOfOrderVisionCount)
                .addData(p + ".replayedVisionCount", replayedVisionCount)
                .addData(p + ".projectedVisionCount", projectedVisionCount)
                .addData(p + ".lastVisionAcceptedSec", lastVisionAcceptedSec)
                .addData(p + ".lastAcceptedVisionMeasurementTimestampSec", lastAcceptedVisionMeasurementTimestampSec)
                .addData(p + ".lastEvaluatedVisionTimestampSec", lastEvaluatedVisionTimestampSec)
                .addData(p + ".lastVisionPose", lastVisionPose)
                .addData(p + ".lastLatencyCompensatedVisionPose", lastLatencyCompensatedVisionPose)
                .addData(p + ".lastReplayReferencePose", lastReplayReferencePose)
                .addData(p + ".lastVisionUsedReplay", lastVisionUsedReplay)
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
                .addData(p + ".replayBaseOdomPose", replayBaseOdomPose)
                .addData(p + ".odomHistorySize", odomHistory.size())
                .addData(p + ".statePose", statePose)
                .addData(p + ".stateCovariance.xx", stateCovariance[0][0])
                .addData(p + ".stateCovariance.yy", stateCovariance[1][1])
                .addData(p + ".stateCovariance.hh", stateCovariance[2][2])
                .addData(p + ".cfg.maxVisionAgeSec", cfg.maxVisionAgeSec)
                .addData(p + ".cfg.minVisionQuality", cfg.minVisionQuality)
                .addData(p + ".cfg.maxVisionPositionInnovationIn", cfg.maxVisionPositionInnovationIn)
                .addData(p + ".cfg.maxVisionHeadingInnovationRad", cfg.maxVisionHeadingInnovationRad)
                .addData(p + ".cfg.maxVisionMahalanobisSq", cfg.maxVisionMahalanobisSq)
                .addData(p + ".cfg.enableInitializeFromVision", cfg.enableInitializeFromVision)
                .addData(p + ".cfg.enablePushFilteredPoseToOdometry", cfg.enablePushFilteredPoseToOdometry)
                .addData(p + ".cfg.enableLatencyCompensation", cfg.enableLatencyCompensation)
                .addData(p + ".cfg.odomHistorySec", cfg.odomHistorySec)
                .addData(p + ".lastEstimate", lastEstimate);

        odometry.debugDump(dbg, p + ".odometry");
        vision.debugDump(dbg, p + ".vision");
    }
}
