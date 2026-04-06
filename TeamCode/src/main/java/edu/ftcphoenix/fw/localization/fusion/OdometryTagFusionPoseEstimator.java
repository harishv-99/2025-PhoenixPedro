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
 *   <li>Optionally projects fresh vision measurements forward through recent odometry so camera
 *       latency hurts less than a naive "compare an old frame directly against the current pose"
 *       scheme.</li>
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

        /**
         * If true, accepted vision measurements are projected forward through recent odometry using
         * their measurement timestamp before the correction is applied.
         *
         * <p>This is a lightweight form of latency compensation. It is intentionally simpler than a
         * full state estimator, but it substantially reduces the tendency for delayed camera frames
         * to "drag" the fused pose backward while the robot is still moving.</p>
         */
        public boolean enableLatencyCompensation = true;

        /**
         * How much recent odometry history (seconds) to retain for latency compensation.
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

    // Debug/telemetry helpers.
    private double lastVisionAcceptedSec = Double.NaN;
    private Pose3d lastVisionPose = Pose3d.zero();
    private Pose3d lastLatencyCompensatedVisionPose = Pose3d.zero();
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
        final Pose3d currentOdomPose = (odomEst != null && odomEst.hasPose)
                ? planarize(odomEst.fieldToRobotPose)
                : null;

        if (currentOdomPose != null) {
            recordOdomSample(nowSec, currentOdomPose);
        }

        // If we are not initialized yet, pick an initial pose.
        if (!initialized) {
            if (visionEnabled && cfg.enableInitializeFromVision && isVisionAcceptable(visEst, nowSec)) {
                fusedPose = planarize(visEst.fieldToRobotPose);
                initialized = true;

                // Align odometry if possible.
                boolean pushedToOdometry = false;
                if (cfg.enablePushFusedPoseToOdometry && odometry instanceof PoseResetter) {
                    ((PoseResetter) odometry).setPose(fusedPose.toPose2d());
                    pushedToOdometry = true;
                }

                // Establish baseline odom pose for deltas.
                if (pushedToOdometry) {
                    // The odometry estimate was just reset to the fused pose, so future deltas
                    // must start from that corrected baseline rather than the pre-reset sample.
                    lastOdomPose = fusedPose;
                } else if (currentOdomPose != null) {
                    lastOdomPose = currentOdomPose;
                } else {
                    lastOdomPose = fusedPose;
                }

                resetOdomHistory(nowSec, lastOdomPose);

                lastVisionAcceptedSec = nowSec;
                lastVisionPose = fusedPose;
                lastLatencyCompensatedVisionPose = fusedPose;
                acceptedVisionCount++;
            } else if (currentOdomPose != null) {
                fusedPose = currentOdomPose;
                initialized = true;
                lastOdomPose = fusedPose;
                resetOdomHistory(nowSec, fusedPose);
            } else {
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
        if (visionEnabled && isVisionAcceptable(visEst, nowSec)) {
            Pose3d visionPose = planarize(visEst.fieldToRobotPose);
            Pose3d visionPoseAtNow = cfg.enableLatencyCompensation
                    ? projectVisionPoseToNow(visionPose, visEst.timestampSec, nowSec, currentOdomPose)
                    : visionPose;

            lastLatencyCompensatedVisionPose = visionPoseAtNow;

            double dx = visionPoseAtNow.xInches - fusedPose.xInches;
            double dy = visionPoseAtNow.yInches - fusedPose.yInches;
            double dPos = Math.hypot(dx, dy);

            double dHeading = MathUtil.wrapToPi(visionPoseAtNow.yawRad - fusedPose.yawRad);

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

                    // The underlying odometry state now matches the corrected fused pose, so the
                    // next odometry delta must be measured relative to that corrected baseline.
                    lastOdomPose = fusedPose;
                }

                resetOdomHistory(nowSec, lastOdomPose);

                lastVisionAcceptedSec = nowSec;
                lastVisionPose = visionPose;
                lastLatencyCompensatedVisionPose = visionPoseAtNow;
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

        final double nowSec = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec))
                ? lastEstimate.timestampSec
                : 0.0;

        fusedPose = new Pose3d(pose.xInches, pose.yInches, 0.0, MathUtil.wrapToPi(pose.headingRad), 0.0, 0.0);
        initialized = true;

        if (cfg.enablePushFusedPoseToOdometry && odometry instanceof PoseResetter) {
            ((PoseResetter) odometry).setPose(pose);

            // We just forced odometry to the fused pose, so future deltas must start from there.
            lastOdomPose = fusedPose;
        } else {
            // Baseline odometry delta from its current pose when we did not push a reset.
            PoseEstimate odomEst = odometry.getEstimate();
            if (odomEst != null && odomEst.hasPose) {
                lastOdomPose = planarize(odomEst.fieldToRobotPose);
            } else {
                lastOdomPose = fusedPose;
            }
        }

        resetOdomHistory(nowSec, lastOdomPose);
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

    private void recordOdomSample(double timestampSec, Pose3d odomPose) {
        if (!Double.isFinite(timestampSec) || odomPose == null) {
            return;
        }

        OdomSample last = odomHistory.peekLast();
        if (last != null && timestampSec <= last.timestampSec) {
            odomHistory.clear();
        }

        odomHistory.addLast(new OdomSample(timestampSec, odomPose));
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
            odomHistory.addLast(new OdomSample(timestampSec, pose));
        }
    }

    private Pose3d projectVisionPoseToNow(Pose3d visionPoseAtMeasurement,
                                          double measurementTimestampSec,
                                          double nowSec,
                                          Pose3d currentOdomPose) {
        if (visionPoseAtMeasurement == null || currentOdomPose == null) {
            return visionPoseAtMeasurement;
        }
        if (!Double.isFinite(measurementTimestampSec) || measurementTimestampSec >= nowSec) {
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
                .addData(p + ".cfg.enableLatencyCompensation", cfg.enableLatencyCompensation)
                .addData(p + ".cfg.odomHistorySec", cfg.odomHistorySec)
                .addData(p + ".fusedPose", fusedPose)
                .addData(p + ".lastOdomPose", lastOdomPose)
                .addData(p + ".lastVisionPose", lastVisionPose)
                .addData(p + ".lastLatencyCompensatedVisionPose", lastLatencyCompensatedVisionPose)
                .addData(p + ".odomHistorySize", odomHistory.size())
                .addData(p + ".lastEstimate", lastEstimate);

        odometry.debugDump(dbg, p + ".odometry");
        vision.debugDump(dbg, p + ".vision");
    }

}
