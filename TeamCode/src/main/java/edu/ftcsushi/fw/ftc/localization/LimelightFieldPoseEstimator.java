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
import edu.ftcsushi.fw.ftc.vision.FtcLimelightAprilTagVision;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PoseEstimate;

/**
 * Direct absolute field-pose estimator backed only by Limelight's standard botpose (MegaTag1).
 *
 * <p>This estimator intentionally sits beside the raw AprilTag path rather than replacing it. A
 * Limelight-backed robot can choose between:</p>
 * <ul>
 *   <li>{@link edu.ftcsushi.fw.localization.apriltag.AprilTagPoseEstimator}: solve a pose from raw tag observations</li>
 *   <li>{@code LimelightFieldPoseEstimator}: consume Limelight's standard full-field pose estimate</li>
 * </ul>
 *
 * <p>The direct-pose path is convenient, but it should still be treated like an absolute correction
 * source: freshness, tag count, and robot motion all matter. The optional {@link MotionPredictor}
 * supplies cached motion deltas for configurable quality/gating policy only. This owner does not
 * update that predictor, read its absolute pose, or submit its heading to the camera.</p>
 *
 * <p>MegaTag2 can reuse a supplied predictor heading. Its returned position and heading therefore
 * cannot simply be counted as independent confirmation of that same predictor. This estimator
 * never selects an MT2 result or falls back to it when standard botpose is absent. Advanced raw
 * access remains on {@link FtcLimelightAprilTagVision#confirmedAprilTagResult(LoopClock)}, but it
 * is not a supported full-pose correction recipe. Standard botpose and raw tag solves can still
 * share image, mount, or field-layout errors; this source restriction does not certify statistical
 * independence, physical accuracy, or calibrated confidence.</p>
 */
public final class LimelightFieldPoseEstimator implements AbsolutePoseEstimator {

    /**
     * Configuration for {@link LimelightFieldPoseEstimator}.
     */
    public static final class Config {

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

    private final FtcLimelightAprilTagVision lane;
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
     * @param lane      borrowed Limelight tag capability; camera lifetime stays with its physical owner
     * @param predictor optional borrowed predictor; its owner updates before this estimator, which
     *                  reads cached motion deltas for motion-aware gating only
     * @param config    non-null estimator policy draft
     * @throws NullPointerException if {@code lane} is null
     * @throws IllegalArgumentException if {@code config} is null or invalid
     */
    public LimelightFieldPoseEstimator(FtcLimelightAprilTagVision lane,
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
     * <p>The estimator claims the cycle before reading a vendor result or borrowed motion. A
     * repeated successful call in that cycle is a no-op, a repeated call after failure rethrows
     * the exact first {@link RuntimeException}, and recursive entry fails before another vendor
     * effect. Pipeline changes after the update become visible on the next cycle, preserving one
     * coherent pose-and-diagnostics snapshot for the complete cycle.</p>
     *
     * <p>A direct botpose is usable only when its {@link Pose3D}, {@link Position}, position unit,
     * and {@link YawPitchRollAngles} are non-null and all converted x/y/z/yaw/pitch/roll components
     * are finite. A claimed predictor motion delta must have finite planar components, quality in
     * {@code [0, 1]}, coherent current-epoch timestamps, and positive duration. Invalid motion
     * publishes no pose for that cycle; no invalid motion value is converted into a quality score.
     * Missing or malformed standard botpose also publishes no pose, even if MT2 is available.</p>
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

        // Claim the attempt before result access or borrowed motion can invoke collaborators.
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

    /** Perform the one result sample and standard-pose evaluation for this cycle. */
    private void updateCurrentCycle(LoopClock clock) {
        final LoopTimestamp nowTimestamp = clock.nowTimestamp();
        lastRejectReason = "none";
        lastVisibleTagCount = 0;
        lastBaseQuality = 0.0;
        lastMotionScale = 1.0;
        lastTranslationSpeedInPerSec = 0.0;
        lastYawRateRadPerSec = 0.0;

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

        Pose3D botpose = result.botpose();
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
        dbg.addData(p + ".hasPose", lastEstimate.hasPose)
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
