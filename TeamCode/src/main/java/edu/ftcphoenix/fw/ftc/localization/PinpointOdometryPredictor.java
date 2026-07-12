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
 *   <li>{@link PinpointKinematicSnapshot} preserves the same physical poll's velocity and unwrapped
 *       heading for passive integration consumers without giving them another hardware update path.</li>
 * </ul>
 *
 * <h2>Units</h2>
 * <ul>
 *   <li>All positions are expressed in <b>inches</b>.</li>
 *   <li>Translational velocity is expressed in <b>inches per second</b>.</li>
 *   <li>Heading is expressed in <b>radians</b>, wrapped to (-pi, pi].</li>
 *   <li>Angular velocity is expressed in <b>radians per second</b>.</li>
 * </ul>
 *
 * <h2>Field-pose convention</h2>
 * <p>The output pose and translational velocity use Phoenix's current FTC season field frame:</p>
 * <ul>
 *   <li>field X/Y meanings come from the current FTC field coordinate convention, not the robot
 *       frame;</li>
 *   <li>heading/yaw is measured about field +Z and is CCW-positive;</li>
 *   <li>the pod-offset names below use robot-relative forward/left directions only for physical
 *       sensor placement.</li>
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
    private PinpointKinematicSnapshot lastKinematicSnapshot =
            PinpointKinematicSnapshot.unavailable(
                    PinpointKinematicSnapshot.NO_CYCLE,
                    0.0,
                    0.0
            );
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean hasPreviousPhysicalHeading;
    private double previousPhysicalHeadingRad;
    private double totalHeadingRad;

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
     * Returns a defensive copy of the hardware/configuration snapshot owned by this predictor.
     *
     * <p>Integration owners may use this to validate lifecycle assumptions such as a controlled
     * INIT reset without acquiring or configuring the raw Pinpoint driver themselves.</p>
     */
    public Config config() {
        return cfg.copy();
    }

    /**
     * Polls Pinpoint once for the supplied cycle and publishes pose, motion, and kinematics.
     *
     * <p>Repeated calls with the same non-null {@link LoopClock#cycle()} are no-ops, so layered
     * estimator/integration code cannot poll the hardware twice in one robot loop. A null clock is
     * retained only for lower-level tools: each null-clock call polls, timestamps the sample at
     * {@code 0.0}, and marks its cycle as {@link PinpointKinematicSnapshot#NO_CYCLE}, so it cannot
     * claim normal same-cycle freshness.</p>
     *
     * @param clock shared robot loop clock, or {@code null} only for lower-level untimed tools
     */
    @Override
    public void update(LoopClock clock) {
        final long cycle;
        final double nowSec;
        if (clock != null) {
            cycle = clock.cycle();
            nowSec = clock.nowSec();
            if (lastUpdateCycle == cycle) {
                return;
            }
            // Record the attempt before touching hardware so a reentrant/failing call cannot poll twice.
            lastUpdateCycle = cycle;
        } else {
            cycle = PinpointKinematicSnapshot.NO_CYCLE;
            nowSec = 0.0;
            // Untimed tooling cannot participate in or poison the normal LoopClock cycle guard.
            lastUpdateCycle = Long.MIN_VALUE;
        }

        odo.update();
        Pose2D pos = odo.getPosition();

        if (pos == null) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            lastMotionDelta = MotionDelta.none(nowSec);
            lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                    cycle,
                    nowSec,
                    totalHeadingRad
            );
            return;
        }

        double xIn = pos.getX(DistanceUnit.INCH);
        double yIn = pos.getY(DistanceUnit.INCH);
        double headingRad = normalizeDriverHeadingRad(pos.getHeading(AngleUnit.RADIANS));

        if (!isFinite(xIn, yIn, headingRad)) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            lastMotionDelta = MotionDelta.none(nowSec);
            lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                    cycle,
                    nowSec,
                    totalHeadingRad
            );
            return;
        }

        if (hasPreviousPhysicalHeading) {
            totalHeadingRad = accumulateUnwrappedHeadingRad(
                    totalHeadingRad,
                    previousPhysicalHeadingRad,
                    headingRad
            );
        }
        previousPhysicalHeadingRad = headingRad;
        hasPreviousPhysicalHeading = true;

        double fieldVelocityXInchesPerSec = odo.getVelX(DistanceUnit.INCH);
        double fieldVelocityYInchesPerSec = odo.getVelY(DistanceUnit.INCH);
        double angularVelocityRadPerSec = odo.getHeadingVelocity(
                AngleUnit.RADIANS.getUnnormalized()
        );
        boolean hasVelocity = isFinite(
                fieldVelocityXInchesPerSec,
                fieldVelocityYInchesPerSec,
                angularVelocityRadPerSec
        );

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
        lastKinematicSnapshot = PinpointKinematicSnapshot.sampled(
                pose.toPose2d(),
                hasVelocity,
                cycle,
                nowSec,
                fieldVelocityXInchesPerSec,
                fieldVelocityYInchesPerSec,
                angularVelocityRadPerSec,
                totalHeadingRad
        );
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
     * Returns the immutable kinematic sample produced by the latest physical Pinpoint poll.
     *
     * <p>A deliberate {@link #setPose(Pose2d)} may rebase the snapshot pose afterward, but preserves
     * the measured velocity, physical total heading, cycle, and poll timestamp. Callers that require
     * current-loop data should also check {@link PinpointKinematicSnapshot#isCurrentFor(LoopClock)}.</p>
     */
    public PinpointKinematicSnapshot getKinematicSnapshot() {
        return lastKinematicSnapshot;
    }

    /**
     * Resets the Pinpoint IMU and pose back to 0,0,0.
     */
    public void resetPosAndIMU() {
        odo.resetPosAndIMU();
        double ts = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec)) ? lastEstimate.timestampSec : 0.0;
        lastEstimate = PoseEstimate.noPose(ts);
        lastMotionDelta = MotionDelta.none(ts);
        hasPreviousPhysicalHeading = false;
        previousPhysicalHeadingRad = 0.0;
        totalHeadingRad = 0.0;
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                lastKinematicSnapshot.cycle,
                ts,
                0.0
        );
    }

    /**
     * Recalibrates the IMU without resetting pose.
     */
    public void recalibrateIMU() {
        odo.recalibrateIMU();
        double ts = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec)) ? lastEstimate.timestampSec : 0.0;
        lastMotionDelta = MotionDelta.none(ts);
        // Establish a new physical-heading baseline after calibration; a calibration jump is not motion.
        hasPreviousPhysicalHeading = false;
        lastKinematicSnapshot = lastKinematicSnapshot.withoutVelocity();
    }

    /**
     * Snaps both the underlying Pinpoint device and this predictor's cached estimate to a known field pose.
     */
    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        requireFinitePose(pose);
        double headingRad = MathUtil.wrapToPi(pose.headingRad);
        Pose2d rebasedPose = new Pose2d(pose.xInches, pose.yInches, headingRad);
        Pose2D set = new Pose2D(
                DistanceUnit.INCH,
                rebasedPose.xInches,
                rebasedPose.yInches,
                AngleUnit.RADIANS,
                rebasedPose.headingRad
        );
        odo.setPosition(set);
        double ts = (lastEstimate != null && Double.isFinite(lastEstimate.timestampSec)) ? lastEstimate.timestampSec : 0.0;
        lastEstimate = new PoseEstimate(new Pose3d(
                rebasedPose.xInches,
                rebasedPose.yInches,
                0.0,
                rebasedPose.headingRad,
                0.0,
                0.0
        ), true, cfg.quality, 0.0, ts);
        lastMotionDelta = MotionDelta.none(ts);
        // A coordinate correction is not physical motion; retain the last measured velocity/turn.
        previousPhysicalHeadingRad = rebasedPose.headingRad;
        hasPreviousPhysicalHeading = true;
        lastKinematicSnapshot = lastKinematicSnapshot.withRebasedPose(rebasedPose);
    }

    /** Accumulate the shortest signed physical turn across a wrapped-heading boundary. */
    static double accumulateUnwrappedHeadingRad(double totalHeadingRad,
                                                double previousHeadingRad,
                                                double currentHeadingRad) {
        return totalHeadingRad + MathUtil.wrapToPi(currentHeadingRad - previousHeadingRad);
    }

    private static double normalizeDriverHeadingRad(double headingRad) {
        if (Math.abs(headingRad) > Math.PI * 2.0 + 0.5) {
            headingRad = Math.toRadians(headingRad);
        }
        return MathUtil.wrapToPi(headingRad);
    }

    private static boolean isFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) {
                return false;
            }
        }
        return true;
    }

    private static void requireFinitePose(Pose2d pose) {
        if (!isFinite(pose.xInches, pose.yInches, pose.headingRad)) {
            throw new IllegalArgumentException("Pinpoint pose must contain finite inches/radians: " + pose);
        }
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
                .addData(p + ".lastMotionDelta", lastMotionDelta)
                .addData(p + ".lastKinematicSnapshot", lastKinematicSnapshot)
                .addData(p + ".lastUpdateCycle", lastUpdateCycle);
        cfg.debugDump(dbg, p + ".cfg");
    }
}
