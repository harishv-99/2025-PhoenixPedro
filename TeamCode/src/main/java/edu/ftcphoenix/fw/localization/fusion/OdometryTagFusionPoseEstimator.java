package edu.ftcphoenix.fw.localization.fusion;

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
 * </ul>
 *
 * <p><b>Loop ordering:</b> this estimator calls {@link PoseEstimator#update(LoopClock)} on its
 * sources inside its own {@link #update(LoopClock)}. If your vision estimator depends on some other
 * sensor graph being updated first, that dependency must still be updated before calling {@link #update(LoopClock)} on this class. AprilTag sensors and selectors in Phoenix are cycle-idempotent sources, so sharing one instance across consumers remains safe.</p>
 */
public class OdometryTagFusionPoseEstimator implements PoseEstimator, PoseResetter {

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
            return c;
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

    // Debug/telemetry helpers.
    private double lastVisionAcceptedSec = Double.NaN;
    private Pose3d lastVisionPose = Pose3d.zero();
    private int acceptedVisionCount = 0;
    private int rejectedVisionCount = 0;

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
        this.cfg = cfg != null ? cfg : Config.defaults();
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
     * Last accepted vision pose (planarized).
     */
    public Pose3d getLastVisionPose() {
        return lastVisionPose;
    }

    /**
     * Returns the total number of accepted vision corrections.
     */
    public int getAcceptedVisionCount() {
        return acceptedVisionCount;
    }

    /**
     * Returns the total number of rejected vision corrections.
     */
    public int getRejectedVisionCount() {
        return rejectedVisionCount;
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

        // If we are not initialized yet, pick an initial pose.
        if (!initialized) {
            if (visionEnabled && cfg.enableInitializeFromVision && isVisionAcceptable(visEst, nowSec)) {
                fusedPose = planarize(visEst.fieldToRobotPose);
                initialized = true;

                // Align odometry if possible.
                if (cfg.enablePushFusedPoseToOdometry && odometry instanceof PoseResetter) {
                    ((PoseResetter) odometry).setPose(fusedPose.toPose2d());
                }

                // Establish baseline odom pose for deltas.
                if (odomEst != null && odomEst.hasPose) {
                    lastOdomPose = planarize(odomEst.fieldToRobotPose);
                } else {
                    lastOdomPose = fusedPose;
                }

                lastVisionAcceptedSec = nowSec;
                lastVisionPose = fusedPose;
                acceptedVisionCount++;
            } else if (odomEst != null && odomEst.hasPose) {
                fusedPose = planarize(odomEst.fieldToRobotPose);
                initialized = true;
                lastOdomPose = fusedPose;
            } else {
                // No pose from either source yet.
                lastEstimate = PoseEstimate.noPose(nowSec);
                return;
            }
        } else {
            // Propagate fused pose using odometry delta.
            if (odomEst != null && odomEst.hasPose) {
                Pose3d currOdomPose = planarize(odomEst.fieldToRobotPose);
                Pose3d delta = lastOdomPose.inverse().then(currOdomPose);
                fusedPose = planarize(fusedPose.then(delta));
                lastOdomPose = currOdomPose;
            }
        }

        // Apply vision correction if available.
        if (visionEnabled && isVisionAcceptable(visEst, nowSec)) {
            Pose3d visionPose = planarize(visEst.fieldToRobotPose);

            double dx = visionPose.xInches - fusedPose.xInches;
            double dy = visionPose.yInches - fusedPose.yInches;
            double dPos = Math.hypot(dx, dy);

            double dHeading = MathUtil.wrapToPi(visionPose.yawRad - fusedPose.yawRad);

            boolean jumpOk = dPos <= cfg.maxVisionPositionJumpIn
                    && Math.abs(dHeading) <= cfg.maxVisionHeadingJumpRad;

            if (jumpOk) {
                double q = MathUtil.clamp(visEst.quality, 0.0, 1.0);
                double posGain = MathUtil.clamp(cfg.visionPositionGain * q, 0.0, 1.0);
                double headingGain = MathUtil.clamp(cfg.visionHeadingGain * q, 0.0, 1.0);

                fusedPose = new Pose3d(
                        fusedPose.xInches + dx * posGain,
                        fusedPose.yInches + dy * posGain,
                        0.0,
                        MathUtil.wrapToPi(fusedPose.yawRad + dHeading * headingGain),
                        0.0,
                        0.0);

                // Keep the odometry aligned if possible.
                if (cfg.enablePushFusedPoseToOdometry && odometry instanceof PoseResetter) {
                    ((PoseResetter) odometry).setPose(fusedPose.toPose2d());
                }

                lastVisionAcceptedSec = nowSec;
                lastVisionPose = visionPose;
                acceptedVisionCount++;
            } else {
                rejectedVisionCount++;
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

        fusedPose = new Pose3d(pose.xInches, pose.yInches, 0.0, MathUtil.wrapToPi(pose.headingRad), 0.0, 0.0);
        initialized = true;

        // Baseline odometry delta from its current pose.
        PoseEstimate odomEst = odometry.getEstimate();
        if (odomEst != null && odomEst.hasPose) {
            lastOdomPose = planarize(odomEst.fieldToRobotPose);
        } else {
            lastOdomPose = fusedPose;
        }

        if (cfg.enablePushFusedPoseToOdometry && odometry instanceof PoseResetter) {
            ((PoseResetter) odometry).setPose(pose);
        }
    }

    private boolean isVisionAcceptable(PoseEstimate visEst, double nowSec) {
        if (visEst == null || !visEst.hasPose || visEst.fieldToRobotPose == null) {
            return false;
        }

        // Freshness gate.
        if (cfg.maxVisionAgeSec > 0.0) {
            double age = nowSec - visEst.timestampSec;
            if (age > cfg.maxVisionAgeSec) {
                return false;
            }
        }

        // Quality gate.
        if (visEst.quality < cfg.minVisionQuality) {
            return false;
        }

        // NaN gate.
        Pose3d p = visEst.fieldToRobotPose;
        return !(Double.isNaN(p.xInches) || Double.isNaN(p.yInches) || Double.isNaN(p.yawRad));
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
                .addData(p + ".lastVisionAcceptedSec", lastVisionAcceptedSec)
                .addData(p + ".cfg.maxVisionAgeSec", cfg.maxVisionAgeSec)
                .addData(p + ".cfg.minVisionQuality", cfg.minVisionQuality)
                .addData(p + ".cfg.visionPositionGain", cfg.visionPositionGain)
                .addData(p + ".cfg.visionHeadingGain", cfg.visionHeadingGain)
                .addData(p + ".cfg.maxVisionPositionJumpIn", cfg.maxVisionPositionJumpIn)
                .addData(p + ".cfg.maxVisionHeadingJumpRad", cfg.maxVisionHeadingJumpRad)
                .addData(p + ".cfg.enableInitializeFromVision", cfg.enableInitializeFromVision)
                .addData(p + ".cfg.enablePushFusedPoseToOdometry", cfg.enablePushFusedPoseToOdometry)
                .addData(p + ".cfg.visionConfidenceHoldSec", cfg.visionConfidenceHoldSec)
                .addData(p + ".fusedPose", fusedPose)
                .addData(p + ".lastOdomPose", lastOdomPose)
                .addData(p + ".lastVisionPose", lastVisionPose)
                .addData(p + ".lastEstimate", lastEstimate);

        odometry.debugDump(dbg, p + ".odometry");
        vision.debugDump(dbg, p + ".vision");
    }

}
