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
 * A lightweight fusion localizer:
 *
 * <ul>
 *   <li><b>Odometry</b> provides smooth short-term motion (dead-wheels, Pinpoint, etc.).</li>
 *   <li><b>AprilTag vision</b> provides occasional absolute corrections (when tags are visible).</li>
 * </ul>
 *
 * <p>Design goals for Phoenix:
 * <ul>
 *   <li>Simple, predictable behavior (no Kalman filter tuning to start).</li>
 *   <li>Works well even when tags disappear near the scoring target (odometry carries you).</li>
 *   <li>Allows gentle "snap-back" corrections when tags reappear.</li>
 *   <li>Optionally applies fresh vision measurements at their measurement timestamp and then
 *       replays odometry forward so camera latency hurts less than a naive "compare an old frame
 *       directly against the current pose" scheme.</li>
 * </ul>
 *
 * <p><b>Loop ordering:</b> this estimator calls {@link PoseEstimator#update(LoopClock)} on its
 * sources inside its own {@link #update(LoopClock)}. If your vision estimator depends on some other
 * sensor graph being updated first, that dependency must still be updated before calling {@link #update(LoopClock)} on this class. AprilTag sensors and selectors in Phoenix are cycle-idempotent sources, so sharing one instance across consumers remains safe.</p>
 *
 * <p><b>Odometry rebasing after corrections:</b> when this estimator pushes a corrected fused pose
 * back into an underlying odometry localizer that supports {@link PoseResetter}, it also rebases
 * its stored odometry baseline to that corrected pose. Without that rebase, the next odometry
 * delta would accidentally re-apply the reset jump and double-count the correction.</p>
 *
 * <p><b>Duplicate vision frames:</b> camera pipelines often expose the same processed frame across
 * multiple robot loops while its age increases. This estimator only evaluates a given vision
 * measurement timestamp once, so a single stale frame cannot keep "pulling" the fused pose over
 * several loops.</p>
 */
public class OdometryTagFusionPoseEstimator implements PoseEstimator, PoseResetter {

    private static final double TIMESTAMP_EPS_SEC = 1e-6;

    /**
     * Configuration for the fusion behavior.
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
         * How aggressively to correct x/y toward the vision measurement (scaled by vision quality).
         */
        public double visionPositionGain = 0.25;

        /**
         * How aggressively to correct heading toward the vision measurement (scaled by vision quality).
         */
        public double visionHeadingGain = 0.35;

        /**
         * Reject vision corrections that jump more than this distance (inches).
         */
        public double maxVisionPositionJumpIn = 24.0;

        /**
         * Reject vision corrections that jump more than this heading delta (radians).
         */
        public double maxVisionHeadingJumpRad = Math.toRadians(60.0);

        /**
         * If true, the estimator may initialize from a fresh vision measurement.
         */
        public boolean enableInitializeFromVision = true;

        /**
         * If true, push the fused pose back into the odometry estimator when it supports resets.
         */
        public boolean enablePushFusedPoseToOdometry = true;

        /**
         * How long (seconds) a recently-accepted vision measurement should boost the reported quality.
         */
        public double visionConfidenceHoldSec = 0.75;

        /**
         * If true, accepted vision measurements are corrected at their measurement timestamp and the
         * stored odometry motion since that timestamp is replayed forward to now.
         *
         * <p>This is a lightweight, deterministic form of latency compensation. It intentionally
         * avoids a full state-estimator stack while still fixing the most common AprilTag fusion
         * failure mode: an old camera frame dragging the fused pose backward while the robot keeps
         * moving.</p>
         */
        public boolean enableLatencyCompensation = true;

        /**
         * How much recent odometry history (seconds) to retain for latency compensation.
         *
         * <p>When latency compensation is enabled, this should be at least as large as
         * {@link #maxVisionAgeSec} so every still-acceptable vision frame can be replayed through
         * odometry history.</p>
         */
        public double odomHistorySec = 1.0;

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
                    : "OdometryTagFusionPoseEstimator.Config";

            if (!Double.isFinite(maxVisionAgeSec) || maxVisionAgeSec < 0.0) {
                throw new IllegalArgumentException(p + ".maxVisionAgeSec must be finite and >= 0");
            }
            if (!Double.isFinite(minVisionQuality) || minVisionQuality < 0.0 || minVisionQuality > 1.0) {
                throw new IllegalArgumentException(p + ".minVisionQuality must be finite and within [0, 1]");
            }
            if (!Double.isFinite(visionPositionGain) || visionPositionGain < 0.0) {
                throw new IllegalArgumentException(p + ".visionPositionGain must be finite and >= 0");
            }
            if (!Double.isFinite(visionHeadingGain) || visionHeadingGain < 0.0) {
                throw new IllegalArgumentException(p + ".visionHeadingGain must be finite and >= 0");
            }
            if (!Double.isFinite(maxVisionPositionJumpIn) || maxVisionPositionJumpIn < 0.0) {
                throw new IllegalArgumentException(p + ".maxVisionPositionJumpIn must be finite and >= 0");
            }
            if (!Double.isFinite(maxVisionHeadingJumpRad) || maxVisionHeadingJumpRad < 0.0) {
                throw new IllegalArgumentException(p + ".maxVisionHeadingJumpRad must be finite and >= 0");
            }
            if (!Double.isFinite(visionConfidenceHoldSec) || visionConfidenceHoldSec < 0.0) {
                throw new IllegalArgumentException(p + ".visionConfidenceHoldSec must be finite and >= 0");
            }
            if (!Double.isFinite(odomHistorySec) || odomHistorySec < 0.0) {
                throw new IllegalArgumentException(p + ".odomHistorySec must be finite and >= 0");
            }
            if (enableLatencyCompensation && odomHistorySec + TIMESTAMP_EPS_SEC < maxVisionAgeSec) {
                throw new IllegalArgumentException(
                        p + ".odomHistorySec must be >= maxVisionAgeSec when latency compensation is enabled"
                                + " (increase odomHistorySec or reduce maxVisionAgeSec)"
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
            c.maxVisionAgeSec = this.maxVisionAgeSec;
            c.minVisionQuality = this.minVisionQuality;
            c.visionPositionGain = this.visionPositionGain;
            c.visionHeadingGain = this.visionHeadingGain;
            c.maxVisionPositionJumpIn = this.maxVisionPositionJumpIn;
            c.maxVisionHeadingJumpRad = this.maxVisionHeadingJumpRad;
            c.enableInitializeFromVision = this.enableInitializeFromVision;
            c.enablePushFusedPoseToOdometry = this.enablePushFusedPoseToOdometry;
            c.visionConfidenceHoldSec = this.visionConfidenceHoldSec;
            c.enableLatencyCompensation = this.enableLatencyCompensation;
            c.odomHistorySec = this.odomHistorySec;
            return c;
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

    private final PoseEstimator odometry;
    private final PoseEstimator vision;
    private final Config cfg;

    private boolean initialized = false;
    private boolean visionEnabled = true;

    private Pose3d fusedPose = Pose3d.zero();
    private Pose3d lastOdomPose = Pose3d.zero();

    private PoseEstimate lastEstimate = PoseEstimate.noPose(0.0);
    private final Deque<OdomSample> odomHistory = new ArrayDeque<OdomSample>();

    // Replay base: the most recent moment at which the fused pose was explicitly anchored
    // (initialization, manual reset, or accepted vision correction). Between replay bases, fused
    // motion is purely odometry-driven, so reconstructing the fused pose at a measurement timestamp
    // is just "base fused pose" composed with the odometry delta from base->measurement.
    private boolean replayBaseValid = false;
    private double replayBaseTimestampSec = Double.NaN;
    private Pose3d replayBaseFusedPose = Pose3d.zero();
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

    /**
     * Creates a fusion estimator with the default fusion configuration.
     */
    public OdometryTagFusionPoseEstimator(PoseEstimator odometry, PoseEstimator vision) {
        this(odometry, vision, Config.defaults());
    }

    /**
     * Creates a fusion estimator that combines odometry with a separate vision estimator.
     */
    public OdometryTagFusionPoseEstimator(PoseEstimator odometry, PoseEstimator vision, Config cfg) {
        if (odometry == null) {
            throw new IllegalArgumentException("odometry must not be null");
        }
        if (vision == null) {
            throw new IllegalArgumentException("vision must not be null");
        }
        this.odometry = odometry;
        this.vision = vision;
        Config base = (cfg != null) ? cfg : Config.defaults();
        this.cfg = base.validatedCopy("OdometryTagFusionPoseEstimator.Config");
    }

    /**
     * Enables/disables vision corrections (odometry integration still runs).
     */
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }

    /**
     * Returns whether the vision lane is currently enabled for fusion.
     */
    public boolean isVisionEnabled() {
        return visionEnabled;
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
     * Returns whether the most recently accepted correction used measurement-time replay instead of
     * a simple now-frame projection fallback.
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
     * Returns how many duplicate frame timestamps were skipped instead of re-applying the same
     * vision measurement multiple loops in a row.
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
     * Returns how many accepted corrections fell back to a simple now-frame projection.
     */
    public int getProjectedVisionCount() {
        return projectedVisionCount;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        final double nowSec = clock != null ? clock.nowSec() : 0.0;

        // Update sources.
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

        // If we are not initialized yet, pick an initial pose.
        if (!initialized) {
            boolean initializedFromVision = false;
            if (visionEnabled && cfg.enableInitializeFromVision && shouldEvaluateVisionMeasurement(visEst)) {
                evaluatedVisionThisLoop = true;
                if (isVisionAcceptable(visEst, nowSec)) {
                    Pose3d visionPose = planarize(visEst.fieldToRobotPose);
                    Pose3d initialPose = cfg.enableLatencyCompensation
                            ? projectVisionPoseToNow(visionPose, visEst.timestampSec, currentOdomPose)
                            : visionPose;

                    fusedPose = initialPose;
                    initialized = true;
                    lastVisionPose = visionPose;
                    lastLatencyCompensatedVisionPose = initialPose;
                    lastReplayReferencePose = visionPose;
                    lastVisionAcceptedSec = nowSec;
                    lastAcceptedVisionMeasurementTimestampSec = visEst.timestampSec;
                    lastVisionUsedReplay = false;
                    acceptedVisionCount++;
                    projectedVisionCount++;

                    boolean pushedToOdometry = pushFusedPoseToOdometry();
                    rebaseAfterPoseChange(nowSec, currentOdomPose, pushedToOdometry);
                    initializedFromVision = true;
                } else {
                    rejectedVisionCount++;
                }
            }

            if (!initialized && currentOdomPose != null) {
                fusedPose = currentOdomPose;
                initialized = true;
                lastOdomPose = fusedPose;
                resetOdomHistory(nowSec, fusedPose);
                setReplayBase(nowSec, fusedPose, fusedPose);
            } else if (!initialized && !initializedFromVision) {
                // No pose from either source yet.
                lastEstimate = PoseEstimate.noPose(nowSec);
                return;
            }
        } else {
            // Propagate fused pose using odometry delta.
            if (currentOdomPose != null) {
                Pose3d delta = lastOdomPose.inverse().then(currentOdomPose);
                fusedPose = planarize(fusedPose.then(delta));
                lastOdomPose = currentOdomPose;
            }
        }

        // Apply vision correction if available.
        if (visionEnabled && !evaluatedVisionThisLoop && shouldEvaluateVisionMeasurement(visEst)) {
            if (!isVisionAcceptable(visEst, nowSec)) {
                rejectedVisionCount++;
            } else {
                maybeApplyVisionCorrection(visEst, currentOdomPose, nowSec);
            }
        }

        // Compute a simple "confidence" signal.
        double quality = (odomEst != null && odomEst.hasPose)
                ? MathUtil.clamp(odomEst.quality, 0.0, 1.0)
                : 0.0;

        if (!Double.isNaN(lastVisionAcceptedSec)) {
            double age = nowSec - lastVisionAcceptedSec;
            if (age >= 0.0 && age < cfg.visionConfidenceHoldSec) {
                double boost = 1.0 - (age / cfg.visionConfidenceHoldSec);
                quality = MathUtil.clamp(Math.max(quality, boost), 0.0, 1.0);
            }
        }

        lastEstimate = new PoseEstimate(fusedPose, true, quality, 0.0, nowSec);
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

        fusedPose = new Pose3d(pose.xInches, pose.yInches, 0.0, MathUtil.wrapToPi(pose.headingRad), 0.0, 0.0);
        initialized = true;

        Pose3d currentOdomPose = null;
        PoseEstimate odomEst = odometry.getEstimate();
        if (odomEst != null && odomEst.hasPose) {
            currentOdomPose = planarize(odomEst.fieldToRobotPose);
        }

        boolean pushedToOdometry = pushFusedPoseToOdometry();
        rebaseAfterPoseChange(nowSec, currentOdomPose, pushedToOdometry);
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

        // Freshness gate.
        if (cfg.maxVisionAgeSec > 0.0) {
            double age = nowSec - visEst.timestampSec;
            if (!Double.isFinite(age) || age < -TIMESTAMP_EPS_SEC || age > cfg.maxVisionAgeSec) {
                return false;
            }
        }

        // Quality gate.
        if (!Double.isFinite(visEst.quality) || visEst.quality < cfg.minVisionQuality) {
            return false;
        }

        // NaN gate.
        Pose3d p = visEst.fieldToRobotPose;
        return !(Double.isNaN(p.xInches) || Double.isNaN(p.yInches) || Double.isNaN(p.yawRad));
    }

    private void maybeApplyVisionCorrection(PoseEstimate visEst, Pose3d currentOdomPose, double nowSec) {
        Pose3d visionPoseAtMeasurement = planarize(visEst.fieldToRobotPose);
        Pose3d compensatedVisionPoseAtNow = cfg.enableLatencyCompensation
                ? projectVisionPoseToNow(visionPoseAtMeasurement, visEst.timestampSec, currentOdomPose)
                : visionPoseAtMeasurement;

        lastVisionPose = visionPoseAtMeasurement;
        lastLatencyCompensatedVisionPose = compensatedVisionPoseAtNow;

        if (cfg.enableLatencyCompensation
                && replayBaseValid
                && Double.isFinite(replayBaseTimestampSec)
                && visEst.timestampSec + TIMESTAMP_EPS_SEC < replayBaseTimestampSec) {
            // This frame predates the most recent accepted correction/reset base. Re-applying it
            // would drag the estimator back across a newer anchor, so reject it.
            lastReplayReferencePose = replayBaseFusedPose;
            lastVisionUsedReplay = false;
            rejectedVisionCount++;
            return;
        }

        final double q = MathUtil.clamp(visEst.quality, 0.0, 1.0);
        final double posGain = MathUtil.clamp(cfg.visionPositionGain * q, 0.0, 1.0);
        final double headingGain = MathUtil.clamp(cfg.visionHeadingGain * q, 0.0, 1.0);

        Pose3d correctedPoseNow = null;
        boolean usedReplay = false;

        if (cfg.enableLatencyCompensation && currentOdomPose != null) {
            Pose3d odomAtMeasurement = interpolateOdomPose(visEst.timestampSec);
            if (odomAtMeasurement != null) {
                Pose3d fusedAtMeasurement = reconstructFusedPoseAt(visEst.timestampSec, odomAtMeasurement);
                if (fusedAtMeasurement != null) {
                    Pose3d odomDeltaSinceMeasurement = odomAtMeasurement.inverse().then(currentOdomPose);
                    lastReplayReferencePose = fusedAtMeasurement;

                    if (isVisionJumpAcceptable(fusedAtMeasurement, visionPoseAtMeasurement)) {
                        Pose3d correctedAtMeasurement = blendToward(
                                fusedAtMeasurement,
                                visionPoseAtMeasurement,
                                posGain,
                                headingGain
                        );
                        correctedPoseNow = planarize(correctedAtMeasurement.then(odomDeltaSinceMeasurement));
                        usedReplay = true;
                    } else {
                        lastVisionUsedReplay = true;
                        rejectedVisionCount++;
                        return;
                    }
                }
            }
        }

        if (correctedPoseNow == null) {
            lastReplayReferencePose = fusedPose;

            if (!isVisionJumpAcceptable(fusedPose, compensatedVisionPoseAtNow)) {
                lastVisionUsedReplay = false;
                rejectedVisionCount++;
                return;
            }

            correctedPoseNow = blendToward(
                    fusedPose,
                    compensatedVisionPoseAtNow,
                    posGain,
                    headingGain
            );
        }

        fusedPose = correctedPoseNow;
        lastVisionAcceptedSec = nowSec;
        lastAcceptedVisionMeasurementTimestampSec = visEst.timestampSec;
        lastVisionUsedReplay = usedReplay;
        acceptedVisionCount++;
        if (usedReplay) {
            replayedVisionCount++;
        } else {
            projectedVisionCount++;
        }

        boolean pushedToOdometry = pushFusedPoseToOdometry();
        rebaseAfterPoseChange(nowSec, currentOdomPose, pushedToOdometry);
    }

    private boolean isVisionJumpAcceptable(Pose3d referencePose, Pose3d targetPose) {
        if (referencePose == null || targetPose == null) {
            return false;
        }
        double dx = targetPose.xInches - referencePose.xInches;
        double dy = targetPose.yInches - referencePose.yInches;
        double dPos = Math.hypot(dx, dy);
        double dHeading = MathUtil.wrapToPi(targetPose.yawRad - referencePose.yawRad);
        return dPos <= cfg.maxVisionPositionJumpIn
                && Math.abs(dHeading) <= cfg.maxVisionHeadingJumpRad;
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

    private boolean pushFusedPoseToOdometry() {
        if (cfg.enablePushFusedPoseToOdometry && odometry instanceof PoseResetter) {
            ((PoseResetter) odometry).setPose(fusedPose.toPose2d());
            return true;
        }
        return false;
    }

    private void rebaseAfterPoseChange(double nowSec, Pose3d currentOdomPose, boolean pushedToOdometry) {
        Pose3d baseOdomPose;
        if (pushedToOdometry) {
            lastOdomPose = fusedPose;
            baseOdomPose = fusedPose;
        } else if (currentOdomPose != null) {
            lastOdomPose = currentOdomPose;
            baseOdomPose = currentOdomPose;
        } else {
            lastOdomPose = fusedPose;
            baseOdomPose = fusedPose;
        }

        resetOdomHistory(nowSec, baseOdomPose);
        setReplayBase(nowSec, fusedPose, baseOdomPose);
    }

    private void setReplayBase(double timestampSec, Pose3d fusedPoseAtBase, Pose3d odomPoseAtBase) {
        replayBaseValid = Double.isFinite(timestampSec) && fusedPoseAtBase != null && odomPoseAtBase != null;
        replayBaseTimestampSec = replayBaseValid ? timestampSec : Double.NaN;
        replayBaseFusedPose = replayBaseValid ? planarize(fusedPoseAtBase) : Pose3d.zero();
        replayBaseOdomPose = replayBaseValid ? planarize(odomPoseAtBase) : Pose3d.zero();
    }

    private Pose3d reconstructFusedPoseAt(double timestampSec, Pose3d odomPoseAtTimestamp) {
        if (!replayBaseValid || odomPoseAtTimestamp == null) {
            return null;
        }
        if (!Double.isFinite(timestampSec) || !Double.isFinite(replayBaseTimestampSec)) {
            return null;
        }
        if (timestampSec + TIMESTAMP_EPS_SEC < replayBaseTimestampSec) {
            return null;
        }

        Pose3d odomDeltaFromBase = replayBaseOdomPose.inverse().then(odomPoseAtTimestamp);
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

    private void recordOdomSample(double timestampSec, Pose3d odomPose) {
        if (!Double.isFinite(timestampSec) || odomPose == null) {
            return;
        }

        OdomSample last = odomHistory.peekLast();
        if (last != null && timestampSec <= last.timestampSec) {
            odomHistory.clear();
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
        if (timestampSec < samples[0].timestampSec || timestampSec > samples[samples.length - 1].timestampSec) {
            return null;
        }
        if (samples.length == 1) {
            return samples[0].pose;
        }

        OdomSample prev = samples[0];
        for (OdomSample next : samples) {
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
     * Debug helper: emit current fusion state and recent vision gating statistics.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "fusion" : prefix;

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
                .addData(p + ".cfg.maxVisionAgeSec", cfg.maxVisionAgeSec)
                .addData(p + ".cfg.minVisionQuality", cfg.minVisionQuality)
                .addData(p + ".cfg.visionPositionGain", cfg.visionPositionGain)
                .addData(p + ".cfg.visionHeadingGain", cfg.visionHeadingGain)
                .addData(p + ".cfg.maxVisionPositionJumpIn", cfg.maxVisionPositionJumpIn)
                .addData(p + ".cfg.maxVisionHeadingJumpRad", cfg.maxVisionHeadingJumpRad)
                .addData(p + ".cfg.enableInitializeFromVision", cfg.enableInitializeFromVision)
                .addData(p + ".cfg.enablePushFusedPoseToOdometry", cfg.enablePushFusedPoseToOdometry)
                .addData(p + ".cfg.visionConfidenceHoldSec", cfg.visionConfidenceHoldSec)
                .addData(p + ".cfg.enableLatencyCompensation", cfg.enableLatencyCompensation)
                .addData(p + ".cfg.odomHistorySec", cfg.odomHistorySec)
                .addData(p + ".fusedPose", fusedPose)
                .addData(p + ".lastOdomPose", lastOdomPose)
                .addData(p + ".lastVisionPose", lastVisionPose)
                .addData(p + ".lastLatencyCompensatedVisionPose", lastLatencyCompensatedVisionPose)
                .addData(p + ".lastReplayReferencePose", lastReplayReferencePose)
                .addData(p + ".lastVisionUsedReplay", lastVisionUsedReplay)
                .addData(p + ".replayBaseValid", replayBaseValid)
                .addData(p + ".replayBaseTimestampSec", replayBaseTimestampSec)
                .addData(p + ".replayBaseFusedPose", replayBaseFusedPose)
                .addData(p + ".replayBaseOdomPose", replayBaseOdomPose)
                .addData(p + ".odomHistorySize", odomHistory.size())
                .addData(p + ".lastEstimate", lastEstimate);

        odometry.debugDump(dbg, p + ".odometry");
        vision.debugDump(dbg, p + ".vision");
    }

}
