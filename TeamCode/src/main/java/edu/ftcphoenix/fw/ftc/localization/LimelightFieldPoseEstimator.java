package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Method;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;

/**
 * Direct absolute field-pose estimator backed by Limelight botpose / MegaTag results.
 *
 * <p>This estimator intentionally sits beside the raw AprilTag path rather than replacing it. A
 * Limelight-backed robot can now choose between:</p>
 * <ul>
 *   <li>{@link edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator}: solve a pose from raw tag observations</li>
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

        /**
         * Reject or ignore results older than this many seconds.
         */
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

        /**
         * Validates the config and throws an actionable error when inconsistent.
         */
        public Config validatedCopy(String context) {
            Config c = copy();
            String p = (context != null && !context.trim().isEmpty())
                    ? context.trim()
                    : "LimelightFieldPoseEstimator.Config";
            requireNonNegative(c.maxResultAgeSec, p + ".maxResultAgeSec");
            if (c.minVisibleTags < 1) {
                throw new IllegalArgumentException(p + ".minVisibleTags must be >= 1");
            }
            requireUnitInterval(c.singleTagQuality, p + ".singleTagQuality");
            requireUnitInterval(c.multiTagQuality, p + ".multiTagQuality");
            requirePositive(c.translationSpeedForZeroQualityInPerSec, p + ".translationSpeedForZeroQualityInPerSec");
            requirePositive(c.yawRateForZeroQualityRadPerSec, p + ".yawRateForZeroQualityRadPerSec");
            requirePositive(c.maxTranslationSpeedInPerSec, p + ".maxTranslationSpeedInPerSec");
            requirePositive(c.maxYawRateRadPerSec, p + ".maxYawRateRadPerSec");
            return c;
        }

        private static void requireNonNegative(double v, String name) {
            if (!Double.isFinite(v) || v < 0.0) {
                throw new IllegalArgumentException(name + " must be finite and >= 0");
            }
        }

        private static void requirePositive(double v, String name) {
            if (!Double.isFinite(v) || v <= 0.0) {
                throw new IllegalArgumentException(name + " must be finite and > 0");
            }
        }

        private static void requireUnitInterval(double v, String name) {
            if (!Double.isFinite(v) || v < 0.0 || v > 1.0) {
                throw new IllegalArgumentException(name + " must be within [0, 1]");
            }
        }
    }

    private static final double MILLIS_PER_SECOND = 1000.0;

    private final FtcLimelightAprilTagVisionLane lane;
    private final MotionPredictor predictor;
    private final Config cfg;

    private PoseEstimate lastEstimate = PoseEstimate.noPose(0.0);
    private double lastTranslationSpeedInPerSec = 0.0;
    private double lastYawRateRadPerSec = 0.0;
    private int lastVisibleTagCount = 0;
    private double lastBaseQuality = 0.0;
    private double lastMotionScale = 1.0;
    private String lastRejectReason = "none";

    /**
     * Creates a direct Limelight field-pose estimator.
     *
     * @param lane      owned Limelight vision lane; this estimator borrows the device owned by that lane
     * @param predictor optional motion predictor used for MegaTag 2 yaw input and motion-aware gating
     * @param config    estimator config; defensively copied for the lifetime of this owner
     */
    public LimelightFieldPoseEstimator(FtcLimelightAprilTagVisionLane lane,
                                       MotionPredictor predictor,
                                       Config config) {
        this.lane = Objects.requireNonNull(lane, "lane");
        this.predictor = predictor;
        Config base = (config != null) ? config : Config.defaults();
        this.cfg = base.validatedCopy("LimelightFieldPoseEstimator.Config");
    }

    /**
     * Polls the Limelight, evaluates freshness / tag-count / motion gates, and updates the current
     * direct field-pose estimate.
     *
     * <p>Typical usage is to call this once per loop from a localization owner, then inspect
     * {@link #getEstimate()} for the most recent accepted direct pose.</p>
     */
    @Override
    public void update(LoopClock clock) {
        final double nowSec = clock != null ? clock.nowSec() : 0.0;
        lastRejectReason = "none";
        lastVisibleTagCount = 0;
        lastBaseQuality = 0.0;
        lastMotionScale = 1.0;
        lastTranslationSpeedInPerSec = 0.0;
        lastYawRateRadPerSec = 0.0;

        Limelight3A limelight = lane.limelight();
        if (cfg.mode == Config.Mode.BOTPOSE_MT2) {
            maybePushPredictorYawToLimelight(limelight);
        }

        LLResult result = limelight.getLatestResult();
        double ageSec = ageSecFromResult(result);
        double timestampSec = nowSec - ageSec;

        if (result == null || !result.isValid()) {
            lastRejectReason = "no valid Limelight result";
            lastEstimate = PoseEstimate.noPose(timestampSec);
            return;
        }

        if (cfg.maxResultAgeSec > 0.0 && ageSec > cfg.maxResultAgeSec) {
            lastRejectReason = "result age exceeded maxResultAgeSec";
            lastEstimate = PoseEstimate.noPose(timestampSec);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        lastVisibleTagCount = fiducials != null ? fiducials.size() : 0;
        if (lastVisibleTagCount < cfg.minVisibleTags) {
            lastRejectReason = "not enough visible tags for direct pose";
            lastEstimate = PoseEstimate.noPose(timestampSec);
            return;
        }

        Pose3D botpose = readBotpose(result, cfg.mode);
        if (botpose == null) {
            lastRejectReason = "direct botpose was unavailable";
            lastEstimate = PoseEstimate.noPose(timestampSec);
            return;
        }

        Pose3d fieldToRobotPose = phoenixFieldPose(botpose);
        if (fieldToRobotPose == null) {
            lastRejectReason = "could not decode botpose";
            lastEstimate = PoseEstimate.noPose(timestampSec);
            return;
        }

        lastBaseQuality = lastVisibleTagCount >= 2 ? cfg.multiTagQuality : cfg.singleTagQuality;
        double ageScale = 1.0;
        if (cfg.maxResultAgeSec > 0.0) {
            ageScale = MathUtil.clamp01(1.0 - (ageSec / cfg.maxResultAgeSec));
        }
        double quality = MathUtil.clamp(lastBaseQuality * ageScale, 0.0, 1.0);

        if (predictor != null) {
            MotionDelta delta = predictor.getLatestMotionDelta();
            if (delta != null && delta.hasDelta && delta.durationSec() > 1e-6) {
                lastTranslationSpeedInPerSec = delta.planarTranslationInches() / delta.durationSec();
                lastYawRateRadPerSec = Math.abs(delta.planarYawDeltaRad()) / delta.durationSec();

                if (cfg.rejectWhenMovingTooFast
                        && (lastTranslationSpeedInPerSec > cfg.maxTranslationSpeedInPerSec
                        || lastYawRateRadPerSec > cfg.maxYawRateRadPerSec)) {
                    lastRejectReason = "predictor motion exceeded hard limits";
                    lastEstimate = PoseEstimate.noPose(timestampSec);
                    return;
                }

                if (cfg.degradeWhenMoving) {
                    double translationScale = 1.0 - Math.min(1.0,
                            lastTranslationSpeedInPerSec / cfg.translationSpeedForZeroQualityInPerSec);
                    double yawScale = 1.0 - Math.min(1.0,
                            lastYawRateRadPerSec / cfg.yawRateForZeroQualityRadPerSec);
                    lastMotionScale = MathUtil.clamp(Math.min(translationScale, yawScale), 0.0, 1.0);
                    quality = MathUtil.clamp(quality * lastMotionScale, 0.0, 1.0);
                }
            }
        }

        lastEstimate = new PoseEstimate(fieldToRobotPose, true, quality, ageSec, timestampSec);
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
                .addData(p + ".ageSec", lastEstimate.ageSec)
                .addData(p + ".timestampSec", lastEstimate.timestampSec)
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

    private void maybePushPredictorYawToLimelight(Limelight3A limelight) {
        if (predictor == null) {
            return;
        }
        PoseEstimate predictorEst = predictor.getEstimate();
        if (predictorEst == null || !predictorEst.hasPose) {
            return;
        }
        double yawDeg = Math.toDegrees(predictorEst.fieldToRobotPose.yawRad);
        tryInvokeUpdateRobotOrientation(limelight, yawDeg);
    }

    private static Pose3d phoenixFieldPose(Pose3D botpose) {
        if (botpose == null) {
            return null;
        }
        Position position = botpose.getPosition();
        if (position == null) {
            return null;
        }
        Position inches = position.toUnit(DistanceUnit.INCH);
        YawPitchRollAngles ypr = botpose.getOrientation();
        double yawRad = 0.0;
        double pitchRad = 0.0;
        double rollRad = 0.0;
        if (ypr != null) {
            yawRad = ypr.getYaw(AngleUnit.RADIANS);
            pitchRad = ypr.getPitch(AngleUnit.RADIANS);
            rollRad = ypr.getRoll(AngleUnit.RADIANS);
        }
        return new Pose3d(
                inches.x,
                inches.y,
                inches.z,
                MathUtil.wrapToPi(yawRad),
                pitchRad,
                rollRad
        );
    }

    private static double ageSecFromResult(LLResult result) {
        if (result == null) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(0.0, result.getStaleness()) / MILLIS_PER_SECOND;
    }

    private static Pose3D readBotpose(LLResult result, Config.Mode mode) {
        if (result == null) {
            return null;
        }
        if (mode == Config.Mode.BOTPOSE_MT2) {
            Pose3D mt2 = invokePoseMethod(result, "getBotpose_MT2");
            if (mt2 != null) {
                return mt2;
            }
            mt2 = invokePoseMethod(result, "getBotPose_MT2");
            if (mt2 != null) {
                return mt2;
            }
        }
        Pose3D mt1 = invokePoseMethod(result, "getBotpose");
        if (mt1 != null) {
            return mt1;
        }
        return invokePoseMethod(result, "getBotPose");
    }

    private static Pose3D invokePoseMethod(LLResult result, String methodName) {
        try {
            Method method = result.getClass().getMethod(methodName);
            Object out = method.invoke(result);
            return (out instanceof Pose3D) ? (Pose3D) out : null;
        } catch (Exception ignored) {
            return null;
        }
    }

    private static void tryInvokeUpdateRobotOrientation(Limelight3A limelight, double yawDeg) {
        try {
            Method method = limelight.getClass().getMethod("updateRobotOrientation", double.class);
            method.invoke(limelight, yawDeg);
            return;
        } catch (Exception ignored) {
            // Try alternate overloads below.
        }
        try {
            Method method = limelight.getClass().getMethod("updateRobotOrientation", Double.TYPE, Double.TYPE, Double.TYPE, Double.TYPE, Double.TYPE, Double.TYPE);
            method.invoke(limelight, yawDeg, 0.0, 0.0, 0.0, 0.0, 0.0);
        } catch (Exception ignored) {
            // Best effort. If the SDK version differs, the estimator can still use MT1.
        }
    }
}
