package edu.ftcsushi.fw.ftc.localization;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.math.MathUtil;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PoseEstimate;

/**
 * Direct absolute field-pose estimator backed by Limelight botpose / MegaTag results.
 *
 * <p>This estimator intentionally sits beside the raw AprilTag path rather than replacing it. A
 * Limelight-backed robot can now choose between:</p>
 * <ul>
 *   <li>{@link edu.ftcsushi.fw.localization.apriltag.AprilTagPoseEstimator}: solve a pose from raw tag observations</li>
 *   <li>{@code LimelightFieldPoseEstimator}: trust the Limelight's own full-field pose estimate</li>
 * </ul>
 *
 * <p>The direct-pose path is convenient, but it should still be treated like an absolute correction
 * source: freshness, tag count, and robot motion all matter. Teams have reported degraded direct
 * pose quality while the robot is moving quickly. This estimator therefore includes lightweight
 * motion-aware quality gating using an optional {@link MotionPredictor}.</p>
 */
public final class LimelightFieldPoseEstimator implements AbsolutePoseEstimator {

    /**
     * Configuration for {@link LimelightFieldPoseEstimator}.
     */
    public static final class Config {

        /**
         * Direct-pose mode to request from the Limelight.
         */
        public enum Mode {
            /**
             * Use the standard botpose / MegaTag 1 result.
             */
            BOTPOSE,
            /**
             * Use the IMU-fused MegaTag 2 result when available.
             */
            BOTPOSE_MT2
        }

        /**
         * Which direct-pose mode to request from the Limelight.
         */
        public Mode mode = Mode.BOTPOSE;

        /** Reject results whose estimated camera-exposure age exceeds this positive number of seconds. */
        public double maxResultAgeSec = 0.25;

        /**
         * Minimum number of visible fiducials required before a direct pose is considered.
         */
        public int minVisibleTags = 1;

        /**
         * Base quality used when only one visible fiducial contributed.
         */
        public double singleTagQuality = 0.55;

        /**
         * Base quality used when multiple visible fiducials contributed.
         */
        public double multiTagQuality = 0.85;

        /**
         * If true, motion from the predictor can reduce direct-pose quality.
         */
        public boolean degradeWhenMoving = true;

        /**
         * Predictor translation speed at which the motion-derived quality term reaches zero.
         */
        public double translationSpeedForZeroQualityInPerSec = 72.0;

        /**
         * Predictor yaw rate at which the motion-derived quality term reaches zero.
         */
        public double yawRateForZeroQualityRadPerSec = Math.toRadians(360.0);

        /**
         * If true, reject direct poses outright when motion exceeds the hard limits below.
         */
        public boolean rejectWhenMovingTooFast = false;

        /**
         * Hard translation-speed reject threshold when {@link #rejectWhenMovingTooFast} is enabled.
         */
        public double maxTranslationSpeedInPerSec = 120.0;

        /**
         * Hard yaw-rate reject threshold when {@link #rejectWhenMovingTooFast} is enabled.
         */
        public double maxYawRateRadPerSec = Math.toRadians(720.0);

        private Config() {
        }

        /**
         * @return new mutable config initialized with framework defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * @return deep copy of this config.
         */
        public Config copy() {
            Config c = new Config();
            c.mode = this.mode;
            c.maxResultAgeSec = this.maxResultAgeSec;
            c.minVisibleTags = this.minVisibleTags;
            c.singleTagQuality = this.singleTagQuality;
            c.multiTagQuality = this.multiTagQuality;
            c.degradeWhenMoving = this.degradeWhenMoving;
            c.translationSpeedForZeroQualityInPerSec = this.translationSpeedForZeroQualityInPerSec;
            c.yawRateForZeroQualityRadPerSec = this.yawRateForZeroQualityRadPerSec;
            c.rejectWhenMovingTooFast = this.rejectWhenMovingTooFast;
            c.maxTranslationSpeedInPerSec = this.maxTranslationSpeedInPerSec;
            c.maxYawRateRadPerSec = this.maxYawRateRadPerSec;
            return c;
        }

        /** Same-package capture seam used by the composite localization owner. */
        Config validatedCopy(String context) {
            Config c = copy();
            String p = (context != null && !context.trim().isEmpty())
                    ? context.trim()
                    : "LimelightFieldPoseEstimator.Config";
            if (c.mode == null) {
                throw new IllegalArgumentException(p + ".mode must not be null, got null");
            }
            requirePositive(c.maxResultAgeSec, p + ".maxResultAgeSec");
            if (c.minVisibleTags < 1) {
                throw new IllegalArgumentException(
                        p + ".minVisibleTags must be >= 1, got " + c.minVisibleTags);
            }
            requireUnitInterval(c.singleTagQuality, p + ".singleTagQuality");
            requireUnitInterval(c.multiTagQuality, p + ".multiTagQuality");
            requirePositive(c.translationSpeedForZeroQualityInPerSec, p + ".translationSpeedForZeroQualityInPerSec");
            requirePositive(c.yawRateForZeroQualityRadPerSec, p + ".yawRateForZeroQualityRadPerSec");
            requirePositive(c.maxTranslationSpeedInPerSec, p + ".maxTranslationSpeedInPerSec");
            requirePositive(c.maxYawRateRadPerSec, p + ".maxYawRateRadPerSec");
            return c;
        }

        private static void requirePositive(double v, String name) {
            if (!Double.isFinite(v) || v <= 0.0) {
                throw new IllegalArgumentException(name + " must be finite and > 0, got " + v);
            }
        }

        private static void requireUnitInterval(double v, String name) {
            if (!Double.isFinite(v) || v < 0.0 || v > 1.0) {
                throw new IllegalArgumentException(
                        name + " must be finite and within [0, 1], got " + v);
            }
        }
    }

    private final FtcLimelightAprilTagVisionLane lane;
    private final MotionPredictor predictor;
    private final Config cfg;

    private PoseEstimate lastEstimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
    private double lastTranslationSpeedInPerSec = 0.0;
    private double lastYawRateRadPerSec = 0.0;
    private int lastVisibleTagCount = 0;
    private double lastBaseQuality = 0.0;
    private double lastMotionScale = 1.0;
    private String lastRejectReason = "none";
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean updateInProgress;
    private RuntimeException lastUpdateFailure;

    /**
     * Creates a direct Limelight field-pose estimator.
     *
     * <p>{@code config} is required and is defensively validated and copied. Pass
     * {@link Config#defaults()} explicitly to select the framework baseline.</p>
     *
     * @param lane      owned Limelight vision lane; this estimator borrows the device owned by that lane
     * @param predictor optional motion predictor used for MegaTag 2 yaw input and motion-aware gating
     * @param config    non-null estimator policy draft
     * @throws NullPointerException if {@code lane} is null
     * @throws IllegalArgumentException if {@code config} is null or invalid
     */
    public LimelightFieldPoseEstimator(FtcLimelightAprilTagVisionLane lane,
                                       MotionPredictor predictor,
                                       Config config) {
        this.lane = Objects.requireNonNull(lane, "lane");
        this.predictor = predictor;
        if (config == null) {
            throw new IllegalArgumentException(
                    "LimelightFieldPoseEstimator.Config must not be null; "
                            + "use Config.defaults() for the framework baseline");
        }
        this.cfg = config.validatedCopy("LimelightFieldPoseEstimator.Config");
    }

    /**
     * Polls the Limelight, evaluates freshness / tag-count / motion gates, and updates the current
     * direct field-pose estimate.
     *
     * <p>Typical usage is to call this once per loop from a localization owner, then inspect
     * {@link #getEstimate()} for the most recent accepted direct pose.</p>
     *
     * <p>The estimator claims the cycle before publishing MT2 yaw or reading a vendor result. A
     * repeated successful call in that cycle is a no-op, a repeated call after failure rethrows
     * the exact first {@link RuntimeException}, and recursive entry fails before another vendor
     * effect. Pipeline changes after the update become visible on the next cycle, preserving one
     * coherent pose-and-diagnostics snapshot for the complete cycle.</p>
     *
     * <p>A direct botpose is usable only when its {@link Pose3D}, {@link Position}, position unit,
     * and {@link YawPitchRollAngles} are non-null and all converted x/y/z/yaw/pitch/roll components
     * are finite. A claimed predictor motion delta must have finite planar components, quality in
     * {@code [0, 1]}, coherent current-epoch timestamps, and positive duration. Invalid motion or
     * predictor yaw publishes no pose for that cycle; non-finite yaw is never sent to Limelight and
     * no invalid motion value is converted into a quality score.</p>
     */
    @Override
    public void update(LoopClock clock) {
        LoopClock requiredClock = Objects.requireNonNull(clock, "clock");
        long cycle = requiredClock.cycle();
        if (updateInProgress) {
            throw new IllegalStateException(
                    "LimelightFieldPoseEstimator.update(clock) was reentered during cycle "
                            + cycle + "; one localization owner may advance only once per cycle"
            );
        }
        if (cycle == lastUpdateCycle) {
            if (lastUpdateFailure != null) {
                throw lastUpdateFailure;
            }
            return;
        }

        // Claim the attempt before MT2 yaw or result access can cause vendor-side effects.
        lastUpdateCycle = cycle;
        updateInProgress = true;
        lastUpdateFailure = null;
        try {
            updateCurrentCycle(requiredClock);
        } catch (RuntimeException failure) {
            lastUpdateFailure = failure;
            throw failure;
        } finally {
            updateInProgress = false;
        }
    }

    /** Perform the one yaw publication, result sample, and pose evaluation for this cycle. */
    private void updateCurrentCycle(LoopClock clock) {
        final LoopTimestamp nowTimestamp = clock.nowTimestamp();
        lastRejectReason = "none";
        lastVisibleTagCount = 0;
        lastBaseQuality = 0.0;
        lastMotionScale = 1.0;
        lastTranslationSpeedInPerSec = 0.0;
        lastYawRateRadPerSec = 0.0;

        if (cfg.mode == Config.Mode.BOTPOSE_MT2) {
            if (!maybePushPredictorYawToLimelight()) {
                lastRejectReason = "predictor reported a non-finite field yaw";
                lastEstimate = PoseEstimate.noPose(nowTimestamp);
                return;
            }
        }

        FtcLimelightVisionLane.ResultSnapshot result = lane.confirmedAprilTagResult(clock);
        if (!result.hasResult()) {
            lastRejectReason = "AprilTag pipeline is not ready";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        LoopTimestamp measurementTimestamp = result.frameTimestamp();
        double ageSec = measurementTimestamp.ageSec(clock);
        if (!Double.isFinite(ageSec)) {
            lastRejectReason = "confirmed result reported invalid frame timing";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        if (!result.isTargetValid()) {
            lastRejectReason = "confirmed pipeline result has no target";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        if (ageSec > cfg.maxResultAgeSec) {
            lastRejectReason = "result age exceeded maxResultAgeSec";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.fiducialResults();
        lastVisibleTagCount = fiducials.size();
        if (lastVisibleTagCount < cfg.minVisibleTags) {
            lastRejectReason = "not enough visible tags for direct pose";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        Pose3D botpose = readBotpose(result, cfg.mode);
        if (botpose == null) {
            lastRejectReason = "direct botpose was unavailable or malformed: require non-null "
                    + "position, position unit, orientation, and finite x/y/z/yaw/pitch/roll";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        Pose3d fieldToRobotPose = sushiFieldPose(botpose);
        if (fieldToRobotPose == null) {
            lastRejectReason = "direct botpose could not convert to finite inches and radians for "
                    + "x/y/z/yaw/pitch/roll";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        lastBaseQuality = lastVisibleTagCount >= 2 ? cfg.multiTagQuality : cfg.singleTagQuality;
        double ageScale = MathUtil.clamp01(1.0 - (ageSec / cfg.maxResultAgeSec));
        double quality = MathUtil.clamp(lastBaseQuality * ageScale, 0.0, 1.0);
        if (!Double.isFinite(ageScale) || !Double.isFinite(quality)) {
            lastRejectReason = "direct-pose age or base quality produced a non-finite quality";
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        if (predictor != null) {
            MotionDelta delta = predictor.getLatestMotionDelta();
            if (delta != null && delta.hasDelta) {
                if (!isUsablePredictorMotion(delta, clock)) {
                    lastRejectReason = "predictor reported an invalid planar motion delta";
                    lastEstimate = PoseEstimate.noPose(nowTimestamp);
                    return;
                }

                double durationSec = delta.durationSec();
                double translationInches = delta.planarTranslationInches();
                double yawDeltaRad = Math.abs(delta.planarYawDeltaRad());
                lastTranslationSpeedInPerSec = translationInches / durationSec;
                lastYawRateRadPerSec = yawDeltaRad / durationSec;
                if (!Double.isFinite(translationInches)
                        || !Double.isFinite(yawDeltaRad)
                        || !Double.isFinite(lastTranslationSpeedInPerSec)
                        || !Double.isFinite(lastYawRateRadPerSec)) {
                    lastTranslationSpeedInPerSec = 0.0;
                    lastYawRateRadPerSec = 0.0;
                    lastRejectReason = "predictor motion produced a non-finite speed or yaw rate";
                    lastEstimate = PoseEstimate.noPose(nowTimestamp);
                    return;
                }

                if (cfg.rejectWhenMovingTooFast
                        && (lastTranslationSpeedInPerSec > cfg.maxTranslationSpeedInPerSec
                        || lastYawRateRadPerSec > cfg.maxYawRateRadPerSec)) {
                    lastRejectReason = "predictor motion exceeded hard limits";
                    lastEstimate = PoseEstimate.noPose(nowTimestamp);
                    return;
                }

                if (cfg.degradeWhenMoving) {
                    double translationScale = 1.0 - Math.min(1.0,
                            lastTranslationSpeedInPerSec / cfg.translationSpeedForZeroQualityInPerSec);
                    double yawScale = 1.0 - Math.min(1.0,
                            lastYawRateRadPerSec / cfg.yawRateForZeroQualityRadPerSec);
                    lastMotionScale = MathUtil.clamp(Math.min(translationScale, yawScale), 0.0, 1.0);
                    quality = MathUtil.clamp(quality * lastMotionScale, 0.0, 1.0);
                    if (!Double.isFinite(translationScale)
                            || !Double.isFinite(yawScale)
                            || !Double.isFinite(lastMotionScale)
                            || !Double.isFinite(quality)) {
                        lastMotionScale = 0.0;
                        lastRejectReason = "predictor motion produced a non-finite quality scale";
                        lastEstimate = PoseEstimate.noPose(nowTimestamp);
                        return;
                    }
                }
            }
        }

        lastEstimate = new PoseEstimate(fieldToRobotPose, true, quality, measurementTimestamp);
    }

    /**
     * Returns the most recent direct Limelight field-pose estimate after all gating in
     * {@link #update(LoopClock)} has been applied.
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * Emits the current direct-pose state, including gating inputs such as tag count, motion scale,
     * and the last reject reason.
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "limelightFieldPose" : prefix;
        dbg.addData(p + ".mode", cfg.mode)
                .addData(p + ".hasPose", lastEstimate.hasPose)
                .addData(p + ".quality", lastEstimate.quality)
                .addData(p + ".timestampAvailable", lastEstimate.timestamp.isAvailable())
                .addData(p + ".fieldToRobotPose", lastEstimate.fieldToRobotPose)
                .addData(p + ".visibleTagCount", lastVisibleTagCount)
                .addData(p + ".baseQuality", lastBaseQuality)
                .addData(p + ".motionScale", lastMotionScale)
                .addData(p + ".translationSpeedInPerSec", lastTranslationSpeedInPerSec)
                .addData(p + ".yawRateRadPerSec", lastYawRateRadPerSec)
                .addData(p + ".rejectReason", lastRejectReason)
                .addData(p + ".cfg.maxResultAgeSec", cfg.maxResultAgeSec)
                .addData(p + ".cfg.minVisibleTags", cfg.minVisibleTags)
                .addData(p + ".cfg.singleTagQuality", cfg.singleTagQuality)
                .addData(p + ".cfg.multiTagQuality", cfg.multiTagQuality)
                .addData(p + ".cfg.degradeWhenMoving", cfg.degradeWhenMoving)
                .addData(p + ".cfg.translationSpeedForZeroQualityInPerSec", cfg.translationSpeedForZeroQualityInPerSec)
                .addData(p + ".cfg.yawRateForZeroQualityRadPerSec", cfg.yawRateForZeroQualityRadPerSec)
                .addData(p + ".cfg.rejectWhenMovingTooFast", cfg.rejectWhenMovingTooFast)
                .addData(p + ".cfg.maxTranslationSpeedInPerSec", cfg.maxTranslationSpeedInPerSec)
                .addData(p + ".cfg.maxYawRateRadPerSec", cfg.maxYawRateRadPerSec);
    }

    private boolean maybePushPredictorYawToLimelight() {
        if (predictor == null) {
            return true;
        }
        PoseEstimate predictorEst = predictor.getEstimate();
        if (predictorEst == null || !predictorEst.hasPose) {
            return true;
        }
        if (predictorEst.fieldToRobotPose == null
                || !Double.isFinite(predictorEst.fieldToRobotPose.yawRad)) {
            return false;
        }
        double wrappedYawRad = MathUtil.wrapToPi(predictorEst.fieldToRobotPose.yawRad);
        if (!Double.isFinite(wrappedYawRad)) {
            return false;
        }
        lane.updateRobotFieldYawRad(wrappedYawRad);
        return true;
    }

    private static Pose3d sushiFieldPose(Pose3D botpose) {
        if (botpose == null) {
            return null;
        }
        Position position = botpose.getPosition();
        YawPitchRollAngles ypr = botpose.getOrientation();
        if (position == null || position.unit == null || ypr == null) {
            return null;
        }
        Position inches = position.toUnit(DistanceUnit.INCH);
        if (inches == null) {
            return null;
        }
        double yawRad = ypr.getYaw(AngleUnit.RADIANS);
        double pitchRad = ypr.getPitch(AngleUnit.RADIANS);
        double rollRad = ypr.getRoll(AngleUnit.RADIANS);
        if (!Double.isFinite(inches.x)
                || !Double.isFinite(inches.y)
                || !Double.isFinite(inches.z)
                || !Double.isFinite(yawRad)
                || !Double.isFinite(pitchRad)
                || !Double.isFinite(rollRad)) {
            return null;
        }
        double wrappedYawRad = MathUtil.wrapToPi(yawRad);
        if (!Double.isFinite(wrappedYawRad)) {
            return null;
        }
        return new Pose3d(
                inches.x,
                inches.y,
                inches.z,
                wrappedYawRad,
                pitchRad,
                rollRad
        );
    }

    private static Pose3D readBotpose(FtcLimelightVisionLane.ResultSnapshot result,
                                      Config.Mode mode) {
        if (result == null || !result.hasResult()) {
            return null;
        }
        if (mode == Config.Mode.BOTPOSE_MT2) {
            Pose3D mt2 = result.botposeMt2();
            if (mt2 != null) {
                return mt2;
            }
        }
        return result.botpose();
    }

    private static boolean isUsablePredictorMotion(MotionDelta delta, LoopClock clock) {
        if (delta == null || !delta.hasDelta || delta.deltaPose == null) {
            return false;
        }
        Pose3d pose = delta.deltaPose;
        if (!Double.isFinite(pose.xInches)
                || !Double.isFinite(pose.yInches)
                || !Double.isFinite(pose.yawRad)
                || !Double.isFinite(delta.quality)
                || delta.quality < 0.0
                || delta.quality > 1.0
                || !isTimestampCurrent(delta.startTimestamp, clock)
                || !isTimestampCurrent(delta.endTimestamp, clock)) {
            return false;
        }
        try {
            double durationSec = delta.durationSec();
            return Double.isFinite(durationSec) && durationSec > 0.0;
        } catch (IllegalArgumentException differentClock) {
            return false;
        }
    }

    private static boolean isTimestampCurrent(LoopTimestamp timestamp, LoopClock clock) {
        if (timestamp == null) {
            return false;
        }
        try {
            return Double.isFinite(timestamp.ageSec(clock));
        } catch (IllegalArgumentException differentClock) {
            return false;
        }
    }
}
