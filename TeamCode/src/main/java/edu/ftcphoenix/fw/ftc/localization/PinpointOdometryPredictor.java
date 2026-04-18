package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;

/**
 * {@link MotionPredictor} wrapper around the goBILDA Pinpoint Odometry Computer.
 *
 * <p>This class is intentionally FTC-hardware-facing (it depends on the FTC SDK). Phoenix robot
 * logic should depend on {@link MotionPredictor} / {@link PoseEstimate} instead of FTC SDK classes.
 * It formalizes Pinpoint's actual role in the localization stack:</p>
 * <ul>
 *   <li>Pinpoint produces a smooth absolute odometry pose.</li>
 *   <li>Pinpoint also produces the incremental motion that global localizers want to replay between
 *       absolute corrections.</li>
 * </ul>
 *
 * <h2>Units</h2>
 * <ul>
 *   <li>All positions are expressed in <b>inches</b>.</li>
 *   <li>Heading is expressed in <b>radians</b>, wrapped to (-pi, pi].</li>
 * </ul>
 *
 * <h2>Pose convention</h2>
 * <p>The output pose follows Phoenix conventions:</p>
 * <ul>
 *   <li><b>+X</b> forward</li>
 *   <li><b>+Y</b> left</li>
 *   <li><b>headingRad</b> is CCW-positive (turn left)</li>
 * </ul>
 *
 * <h2>Pinpoint offsets (naming matters)</h2>
 * <p>The underlying Pinpoint driver is configured via
 * {@link GoBildaPinpointDriver#setOffsets(double, double, DistanceUnit)} using two offsets. The
 * parameter names in the goBILDA API ({@code xOffset}, {@code yOffset}) refer to the pod axes,
 * which can be easy to confuse with Phoenix's coordinate axes.</p>
 *
 * <p>To reduce mixups, Phoenix uses <b>directional names</b> in {@link Config}:</p>
 * <ul>
 *   <li>{@link Config#forwardPodOffsetLeftInches}: how far left (+) / right (-) the forward pod is</li>
 *   <li>{@link Config#strafePodOffsetForwardInches}: how far forward (+) / back (-) the strafe pod is</li>
 * </ul>
 */
public final class PinpointOdometryPredictor implements MotionPredictor, PoseResetter {

    /**
     * Phoenix standard unit for field poses.
     */
    private static final DistanceUnit CONFIG_DISTANCE_UNIT = DistanceUnit.INCH;

    /**
     * Configuration for {@link PinpointOdometryPredictor}.
     */
    public static final class Config {

        /**
         * FTC hardware configuration name of the Pinpoint device (commonly {@code "odo"}).
         */
        public String hardwareMapName = "odo";

        /**
         * Left/right offset of the forward (X) pod from the robot tracking point; +left, -right.
         */
        public double forwardPodOffsetLeftInches = 0.0;

        /**
         * Forward/back offset of the strafe (Y) pod from the robot tracking point; +forward, -back.
         */
        public double strafePodOffsetForwardInches = 0.0;

        /**
         * If true, reset Pinpoint pose + IMU during estimator construction.
         */
        public boolean enableResetOnInit = true;

        /**
         * Time to wait after reset (milliseconds).
         */
        public long resetWaitMs = 300;

        /**
         * Built-in goBILDA odometry pod preset.
         */
        public GoBildaPinpointDriver.GoBildaOdometryPods encoderPods =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        /**
         * Custom encoder resolution in ticks per inch. If non-null, overrides {@link #encoderPods}.
         */
        public Double customEncoderResolutionTicksPerInch = null;

        /**
         * Direction for the forward (X) pod encoder.
         */
        public GoBildaPinpointDriver.EncoderDirection forwardPodDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        /**
         * Direction for the strafe (Y) pod encoder.
         */
        public GoBildaPinpointDriver.EncoderDirection strafePodDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        /**
         * Optional yaw scalar. If null, uses the driver default / factory calibration.
         */
        public Double yawScalar = null;

        /**
         * Quality score used when publishing absolute pose and motion deltas (0..1).
         */
        public double quality = 0.75;

        private Config() {
        }

        /**
         * @return new config instance populated with framework defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Convenience factory for the common case.
         */
        public static Config of(String hardwareMapName,
                                double forwardPodOffsetLeftInches,
                                double strafePodOffsetForwardInches) {
            Config c = defaults();
            c.hardwareMapName = hardwareMapName;
            c.forwardPodOffsetLeftInches = forwardPodOffsetLeftInches;
            c.strafePodOffsetForwardInches = strafePodOffsetForwardInches;
            return c;
        }

        /**
         * Fluent helper: set the hardware map name.
         */
        public Config withHardwareMapName(String hardwareMapName) {
            this.hardwareMapName = hardwareMapName;
            return this;
        }

        /**
         * Fluent helper: set both pod offsets.
         */
        public Config withOffsets(double forwardPodOffsetLeftInches, double strafePodOffsetForwardInches) {
            this.forwardPodOffsetLeftInches = forwardPodOffsetLeftInches;
            this.strafePodOffsetForwardInches = strafePodOffsetForwardInches;
            return this;
        }

        /**
         * Fluent helper: set whether to reset pose + IMU during construction.
         */
        public Config withResetOnInit(boolean enableResetOnInit) {
            this.enableResetOnInit = enableResetOnInit;
            return this;
        }

        /**
         * Fluent helper: set the wait time after a reset (milliseconds).
         */
        public Config withResetWaitMs(long resetWaitMs) {
            this.resetWaitMs = resetWaitMs;
            return this;
        }

        /**
         * Fluent helper: set the built-in goBILDA odometry pod preset.
         */
        public Config withEncoderPods(GoBildaPinpointDriver.GoBildaOdometryPods pods) {
            this.encoderPods = pods;
            return this;
        }

        /**
         * Fluent helper: override encoder resolution in ticks per inch.
         */
        public Config withCustomEncoderResolutionTicksPerInch(Double ticksPerInch) {
            this.customEncoderResolutionTicksPerInch = ticksPerInch;
            return this;
        }

        /**
         * Fluent helper: set direction for the forward (X) pod encoder.
         */
        public Config withForwardPodDirection(GoBildaPinpointDriver.EncoderDirection dir) {
            this.forwardPodDirection = dir;
            return this;
        }

        /**
         * Fluent helper: set direction for the strafe (Y) pod encoder.
         */
        public Config withStrafePodDirection(GoBildaPinpointDriver.EncoderDirection dir) {
            this.strafePodDirection = dir;
            return this;
        }

        /**
         * Fluent helper: set a yaw scalar.
         */
        public Config withYawScalar(Double yawScalar) {
            this.yawScalar = yawScalar;
            return this;
        }

        /**
         * Fluent helper: set the published quality score used by debug/fusion (0..1).
         */
        public Config withQuality(double quality) {
            this.quality = quality;
            return this;
        }

        /**
         * @return deep copy of this config.
         */
        public Config copy() {
            Config c = defaults();
            c.hardwareMapName = this.hardwareMapName;
            c.forwardPodOffsetLeftInches = this.forwardPodOffsetLeftInches;
            c.strafePodOffsetForwardInches = this.strafePodOffsetForwardInches;
            c.enableResetOnInit = this.enableResetOnInit;
            c.resetWaitMs = this.resetWaitMs;
            c.encoderPods = this.encoderPods;
            c.customEncoderResolutionTicksPerInch = this.customEncoderResolutionTicksPerInch;
            c.forwardPodDirection = this.forwardPodDirection;
            c.strafePodDirection = this.strafePodDirection;
            c.yawScalar = this.yawScalar;
            c.quality = this.quality;
            return c;
        }

        /**
         * Emits a compact debug summary of this config.
         */
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "pinpoint" : prefix;
            dbg.addData(p + ".hardwareMapName", hardwareMapName)
                    .addData(p + ".forwardPodOffsetLeftInches", forwardPodOffsetLeftInches)
                    .addData(p + ".strafePodOffsetForwardInches", strafePodOffsetForwardInches)
                    .addData(p + ".enableResetOnInit", enableResetOnInit)
                    .addData(p + ".resetWaitMs", resetWaitMs)
                    .addData(p + ".encoderPods", encoderPods)
                    .addData(p + ".customEncoderResolutionTicksPerInch", customEncoderResolutionTicksPerInch)
                    .addData(p + ".forwardPodDirection", forwardPodDirection)
                    .addData(p + ".strafePodDirection", strafePodDirection)
                    .addData(p + ".yawScalar", yawScalar)
                    .addData(p + ".quality", quality);
        }
    }

    private final GoBildaPinpointDriver odo;
    private final Config cfg;

    private PoseEstimate lastEstimate = PoseEstimate.noPose(0.0);
    private MotionDelta lastMotionDelta = MotionDelta.none(0.0);

    /**
     * Creates a Pinpoint-backed motion predictor.
     *
     * <p>If {@code config} is {@code null}, {@link Config#defaults()} is used.</p>
     */
    public PinpointOdometryPredictor(HardwareMap hardwareMap, Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");

        Config base = (config != null) ? config : Config.defaults();
        this.cfg = base.copy();

        this.odo = hardwareMap.get(GoBildaPinpointDriver.class,
                Objects.requireNonNull(this.cfg.hardwareMapName, "hardwareMapName"));

        odo.setOffsets(
                this.cfg.forwardPodOffsetLeftInches,
                this.cfg.strafePodOffsetForwardInches,
                CONFIG_DISTANCE_UNIT
        );

        if (this.cfg.yawScalar != null) {
            odo.setYawScalar(this.cfg.yawScalar);
        }

        if (this.cfg.customEncoderResolutionTicksPerInch != null) {
            odo.setEncoderResolution(this.cfg.customEncoderResolutionTicksPerInch, CONFIG_DISTANCE_UNIT);
        } else if (this.cfg.encoderPods != null) {
            odo.setEncoderResolution(this.cfg.encoderPods);
        }

        if (this.cfg.forwardPodDirection != null && this.cfg.strafePodDirection != null) {
            odo.setEncoderDirections(this.cfg.forwardPodDirection, this.cfg.strafePodDirection);
        }

        if (this.cfg.enableResetOnInit) {
            odo.resetPosAndIMU();
            if (this.cfg.resetWaitMs > 0) {
                try {
                    Thread.sleep(this.cfg.resetWaitMs);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    /**
     * Exposes the underlying FTC driver in case advanced hardware access is needed.
     */
    public GoBildaPinpointDriver getDriver() {
        return odo;
    }

    /**
     * Polls the Pinpoint, updates the absolute odometry pose, and publishes the latest motion delta.
     *
     * <p>Typical usage is to call this once per loop before reading either {@link #getEstimate()} or
     * {@link #getLatestMotionDelta()}.</p>
     */
    @Override
    public void update(LoopClock clock) {
        odo.update();
        Pose2D pos = odo.getPosition();
        double nowSec = clock != null ? clock.nowSec() : 0.0;

        if (pos == null) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            lastMotionDelta = MotionDelta.none(nowSec);
            return;
        }

        double xIn = pos.getX(DistanceUnit.INCH);
        double yIn = pos.getY(DistanceUnit.INCH);
        double headingRad = pos.getHeading(AngleUnit.RADIANS);

        if (Math.abs(headingRad) > Math.PI * 2.0 + 0.5) {
            headingRad = Math.toRadians(headingRad);
        }
        headingRad = MathUtil.wrapToPi(headingRad);

        Pose3d pose = new Pose3d(xIn, yIn, 0.0, headingRad, 0.0, 0.0);
        PoseEstimate previous = lastEstimate;
        if (previous != null && previous.hasPose) {
            double startSec = Double.isFinite(previous.timestampSec) ? previous.timestampSec : nowSec;
            Pose3d previousPose = previous.fieldToRobotPose;
            lastMotionDelta = new MotionDelta(
                    previousPose.inverse().then(pose),
                    true,
                    cfg.quality,
                    startSec,
                    nowSec
            );
        } else {
            lastMotionDelta = MotionDelta.none(nowSec);
        }

        lastEstimate = new PoseEstimate(pose, true, cfg.quality, 0.0, nowSec);
    }

    /**
     * Returns the most recent absolute odometry pose reported by Pinpoint.
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * Returns the most recent timestamped motion increment computed from successive Pinpoint poses.
     */
    @Override
    public MotionDelta getLatestMotionDelta() {
        return lastMotionDelta;
    }

    /**
     * Resets the Pinpoint IMU and pose back to 0,0,0.
     */
    public void resetPosAndIMU() {
        odo.resetPosAndIMU();
        double ts = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec)) ? lastEstimate.timestampSec : 0.0;
        lastEstimate = PoseEstimate.noPose(ts);
        lastMotionDelta = MotionDelta.none(ts);
    }

    /**
     * Recalibrates the IMU without resetting pose.
     */
    public void recalibrateIMU() {
        odo.recalibrateIMU();
        double ts = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec)) ? lastEstimate.timestampSec : 0.0;
        lastMotionDelta = MotionDelta.none(ts);
    }

    /**
     * Snaps both the underlying Pinpoint device and this predictor's cached estimate to a known field pose.
     */
    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        Pose2D set = new Pose2D(DistanceUnit.INCH, pose.xInches, pose.yInches, AngleUnit.RADIANS, pose.headingRad);
        odo.setPosition(set);
        double ts = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec)) ? lastEstimate.timestampSec : 0.0;
        lastEstimate = new PoseEstimate(new Pose3d(
                pose.xInches,
                pose.yInches,
                0.0,
                MathUtil.wrapToPi(pose.headingRad),
                0.0,
                0.0
        ), true, cfg.quality, 0.0, ts);
        lastMotionDelta = MotionDelta.none(ts);
    }

    /**
     * Emits the current predictor pose, last motion delta, and static config for telemetry/debugging.
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "pinpoint" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName())
                .addData(p + ".driverStatus", odo.getDeviceStatus())
                .addData(p + ".lastEstimate", lastEstimate)
                .addData(p + ".lastMotionDelta", lastMotionDelta);
        cfg.debugDump(dbg, p + ".cfg");
    }
}
