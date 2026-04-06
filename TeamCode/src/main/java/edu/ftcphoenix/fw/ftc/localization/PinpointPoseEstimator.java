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
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.localization.PoseResetter;

/**
 * {@link PoseEstimator} wrapper around the goBILDA Pinpoint Odometry Computer.
 *
 * <p>This class is intentionally FTC-hardware-facing (it depends on the FTC SDK). Phoenix robot
 * logic should depend on {@link PoseEstimator} / {@link PoseEstimate} instead of FTC SDK classes.
 * </p>
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
 * <h2>Pinpoint offsets (naming matters!)</h2>
 * <p>
 * The underlying Pinpoint driver is configured via
 * {@link GoBildaPinpointDriver#setOffsets(double, double, DistanceUnit)} using two offsets.
 * The parameter names in the goBILDA API ({@code xOffset}, {@code yOffset}) refer to the
 * <i>odometry pod axes</i>, which can be easy to confuse with Phoenix's coordinate axes.
 * </p>
 *
 * <p>
 * To reduce mixups, Phoenix uses <b>directional names</b> in {@link Config}:
 * </p>
 * <ul>
 *   <li>{@link Config#forwardPodOffsetLeftInches}: how far left (+) / right (-) the forward (X) pod is</li>
 *   <li>{@link Config#strafePodOffsetForwardInches}: how far forward (+) / back (-) the strafe (Y) pod is</li>
 * </ul>
 */
public final class PinpointPoseEstimator implements PoseEstimator, PoseResetter {

    /**
     * Phoenix standard unit for field poses.
     */
    private static final DistanceUnit CONFIG_DISTANCE_UNIT = DistanceUnit.INCH;

    /**
     * Configuration for {@link PinpointPoseEstimator}.
     *
     * <p>
     * Phoenix convention for <b>single-owner configs</b> is to define them as a nested
     * {@code <Owner>.Config} class (see {@code MecanumDrivebase.Config}, {@code FtcVision.Config}).
     * This keeps the config "near" the code it configures and avoids proliferating top-level
     * config types.
     * </p>
     *
     * <h2>Units</h2>
     * <ul>
     *   <li>All distances are in <b>inches</b>.</li>
     *   <li>All angles are in <b>radians</b> (where applicable).</li>
     * </ul>
     */
    public static final class Config {

        /**
         * FTC hardware configuration name of the Pinpoint device (commonly "odo").
         */
        public String hardwareMapName = "odo";

        // --------------------------------------------------------------------
        // Geometry: pod offsets relative to the robot tracking point
        // --------------------------------------------------------------------

        /**
         * Left/right offset of the <b>forward (X) pod</b> from the robot tracking point.
         *
         * <p>
         * Positive values mean the forward pod is mounted to the <b>left</b> of the tracking point.
         * Negative values mean it is mounted to the <b>right</b>.
         * </p>
         *
         * <p>This maps to the first parameter ({@code xOffset}) of
         * {@link GoBildaPinpointDriver#setOffsets(double, double, DistanceUnit)}.</p>
         */
        public double forwardPodOffsetLeftInches = 0.0;

        /**
         * Forward/back offset of the <b>strafe (Y) pod</b> from the robot tracking point.
         *
         * <p>
         * Positive values mean the strafe pod is mounted <b>forward</b> of the tracking point.
         * Negative values mean it is mounted <b>behind</b> the tracking point.
         * </p>
         *
         * <p>This maps to the second parameter ({@code yOffset}) of
         * {@link GoBildaPinpointDriver#setOffsets(double, double, DistanceUnit)}.</p>
         */
        public double strafePodOffsetForwardInches = 0.0;

        // --------------------------------------------------------------------
        // Hardware behavior
        // --------------------------------------------------------------------

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
         * Custom encoder resolution in <b>ticks per inch</b>. If non-null, overrides {@link #encoderPods}.
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
         * A simple quality score used by fusion / debug displays.
         *
         * <p>Domain is 0..1. Odometry is continuous but drift-prone long-term, so values like
         * 0.6–0.9 are typical.</p>
         */
        public double quality = 0.75;

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
         * Convenience factory for the common case.
         *
         * @param hardwareMapName              FTC hardware configuration name
         * @param forwardPodOffsetLeftInches   left/right offset of forward pod (+left)
         * @param strafePodOffsetForwardInches forward/back offset of strafe pod (+forward)
         */
        public static Config of(
                String hardwareMapName,
                double forwardPodOffsetLeftInches,
                double strafePodOffsetForwardInches
        ) {
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
         * Fluent helper: set both offsets.
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
         *
         * <p>Set to {@code null} to use {@link #encoderPods} instead.</p>
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
         *
         * <p>Set to {@code null} to use the driver default / factory calibration.</p>
         */
        public Config withYawScalar(Double yawScalar) {
            this.yawScalar = yawScalar;
            return this;
        }

        /**
         * Fluent helper: set a quality score used by fusion/debug (0..1).
         */
        public Config withQuality(double quality) {
            this.quality = quality;
            return this;
        }

        /**
         * Create a deep copy of this config.
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
         * Debug-dump this config using the framework {@link DebugSink} pattern.
         */
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
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

    // Start in a "no pose" state until the first successful update.
    private PoseEstimate lastEstimate = PoseEstimate.noPose(0.0);

    /**
     * Create a Pinpoint-backed {@link PoseEstimator}.
     *
     * <p>If {@code config} is {@code null}, {@link Config#defaults()} is used.</p>
     */
    public PinpointPoseEstimator(HardwareMap hardwareMap, Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");

        Config base = (config != null) ? config : Config.defaults();
        this.cfg = base.copy();

        this.odo = hardwareMap.get(GoBildaPinpointDriver.class,
                Objects.requireNonNull(this.cfg.hardwareMapName, "hardwareMapName"));

        // Configure offsets. Phoenix uses inches everywhere, so keep Pinpoint config in inches.
        odo.setOffsets(
                this.cfg.forwardPodOffsetLeftInches,
                this.cfg.strafePodOffsetForwardInches,
                CONFIG_DISTANCE_UNIT
        );

        // Configure yaw scalar if requested.
        if (this.cfg.yawScalar != null) {
            odo.setYawScalar(this.cfg.yawScalar);
        }

        // Configure encoder resolution.
        if (this.cfg.customEncoderResolutionTicksPerInch != null) {
            odo.setEncoderResolution(this.cfg.customEncoderResolutionTicksPerInch, CONFIG_DISTANCE_UNIT);
        } else if (this.cfg.encoderPods != null) {
            odo.setEncoderResolution(this.cfg.encoderPods);
        }

        // Configure encoder direction.
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
     * Exposes the underlying FTC driver in case you need advanced access.
     */
    public GoBildaPinpointDriver getDriver() {
        return odo;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        odo.update();
        Pose2D pos = odo.getPosition();
        double nowSec = clock != null ? clock.nowSec() : 0.0;

        if (pos == null) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        double xIn = pos.getX(DistanceUnit.INCH);
        double yIn = pos.getY(DistanceUnit.INCH);
        double headingRad = pos.getHeading(AngleUnit.RADIANS);

        // Defensive conversion: a known issue in some versions causes unit confusion.
        if (Math.abs(headingRad) > Math.PI * 2.0 + 0.5) {
            headingRad = Math.toRadians(headingRad);
        }

        headingRad = MathUtil.wrapToPi(headingRad);

        Pose3d pose = new Pose3d(xIn, yIn, 0.0, headingRad, 0.0, 0.0);
        lastEstimate = new PoseEstimate(pose, true, cfg.quality, 0.0, nowSec);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * Resets the Pinpoint IMU and pose back to 0,0,0.
     */
    public void resetPosAndIMU() {
        odo.resetPosAndIMU();
    }

    /**
     * Recalibrates the IMU without resetting pose.
     */
    public void recalibrateIMU() {
        odo.recalibrateIMU();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        Pose2D set = new Pose2D(DistanceUnit.INCH, pose.xInches, pose.yInches, AngleUnit.RADIANS, pose.headingRad);
        odo.setPosition(set);
    }
}