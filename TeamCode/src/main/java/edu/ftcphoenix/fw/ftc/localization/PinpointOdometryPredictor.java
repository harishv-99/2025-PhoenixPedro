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
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
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

    private PoseEstimate lastEstimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
    private MotionDelta lastMotionDelta = MotionDelta.none(LoopTimestamp.unavailable());
    private PinpointKinematicSnapshot lastKinematicSnapshot =
            PinpointKinematicSnapshot.unavailable(
                    PinpointKinematicSnapshot.NO_CYCLE,
                    LoopTimestamp.unavailable(),
                    0.0
            );
    private final UpdateState updateState = new UpdateState();
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
     * <p>Repeated calls with the same {@link LoopClock#cycle()} are no-ops after a successful
     * update, so layered estimator/integration code cannot poll the hardware twice in one robot
     * loop. The cycle is claimed before touching hardware: reentrant updates fail fast, and if the
     * first attempt throws a {@link RuntimeException}, later calls in that same cycle rethrow that
     * exact failure instead of retrying a partially completed hardware operation. The next cycle
     * may attempt an update normally.</p>
     *
     * <p>A physical poll at the same timestamp as the accepted motion baseline still refreshes the
     * absolute pose and kinematic snapshot, but publishes no motion delta and does not consume the
     * changed pose. The first later positive-time sample therefore includes all motion since the
     * accepted baseline instead of silently losing it.</p>
     *
     * @param clock shared robot loop clock; must not be {@code null}
     * @throws NullPointerException if {@code clock} is {@code null}
     * @throws IllegalStateException if this method is called reentrantly
     */
    @Override
    public void update(LoopClock clock) {
        LoopClock currentClock = Objects.requireNonNull(
                clock,
                "PinpointOdometryPredictor.update(clock) requires the shared LoopClock"
        );
        if (!updateState.beginUpdate(currentClock)) {
            return;
        }

        try {
            updateFromHardware(currentClock.cycle(), currentClock.nowTimestamp());
        } catch (RuntimeException failure) {
            // A failed poll may have consumed only part of a hardware sample. Do not bridge a
            // later motion delta across that unknown interval.
            updateState.invalidateMotionBaseline();
            throw updateState.recordFailure(failure);
        } finally {
            updateState.endUpdate();
        }
    }

    /** Perform the one physical poll for an already-claimed loop cycle. */
    private void updateFromHardware(long cycle, LoopTimestamp timestamp) {
        odo.update();
        Pose2D pos = odo.getPosition();

        if (pos == null) {
            updateState.invalidateMotionBaseline();
            lastEstimate = PoseEstimate.noPose(timestamp);
            lastMotionDelta = MotionDelta.none(timestamp);
            lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                    cycle,
                    timestamp,
                    totalHeadingRad
            );
            return;
        }

        double xIn = pos.getX(DistanceUnit.INCH);
        double yIn = pos.getY(DistanceUnit.INCH);
        double headingRad = normalizeDriverHeadingRad(pos.getHeading(AngleUnit.RADIANS));

        if (!isFinite(xIn, yIn, headingRad)) {
            updateState.invalidateMotionBaseline();
            lastEstimate = PoseEstimate.noPose(timestamp);
            lastMotionDelta = MotionDelta.none(timestamp);
            lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                    cycle,
                    timestamp,
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
        lastMotionDelta = updateState.observeMotion(pose, timestamp, cfg.quality);

        lastEstimate = new PoseEstimate(pose, true, cfg.quality, timestamp);
        lastKinematicSnapshot = PinpointKinematicSnapshot.sampled(
                pose.toPose2d(),
                hasVelocity,
                cycle,
                timestamp,
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
     *
     * <p>Published pose and motion are invalidated before the vendor operation. If that operation
     * throws after acting partially, callers therefore observe unavailable state rather than a
     * replayable pre-reset delta.</p>
     */
    public void resetPosAndIMU() {
        // The vendor call is not transactional. Invalidate both internal and published state
        // first so a partial reset cannot leave pre-reset motion or pose available to a caller.
        LoopTimestamp timestamp = lastTimestamp();
        updateState.invalidateMotionBaseline();
        lastEstimate = PoseEstimate.noPose(timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        hasPreviousPhysicalHeading = false;
        previousPhysicalHeadingRad = 0.0;
        totalHeadingRad = 0.0;
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                lastKinematicSnapshot.cycle,
                timestamp,
                0.0
        );
        odo.resetPosAndIMU();
    }

    /**
     * Recalibrates the IMU without resetting pose.
     *
     * <p>The last pose is restored only after the vendor operation succeeds. A failure leaves pose,
     * velocity, and motion unavailable until a later successful physical poll.</p>
     */
    public void recalibrateIMU() {
        // Preserve the last absolute pose on success, but publish no pose/motion while the
        // nontransactional vendor operation is in progress or if it fails partway through.
        LoopTimestamp timestamp = lastTimestamp();
        PoseEstimate previousEstimate = lastEstimate;
        PinpointKinematicSnapshot previousSnapshot = lastKinematicSnapshot;
        updateState.invalidateMotionBaseline();
        lastEstimate = PoseEstimate.noPose(timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        hasPreviousPhysicalHeading = false;
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                previousSnapshot.cycle,
                timestamp,
                totalHeadingRad
        );

        odo.recalibrateIMU();

        // Establish a new physical-heading baseline after calibration; a calibration jump is not motion.
        lastEstimate = previousEstimate;
        lastKinematicSnapshot = previousSnapshot.withoutVelocity();
    }

    /**
     * Snaps both the underlying Pinpoint device and this predictor's cached estimate to a known field pose.
     *
     * <p>Published pose and motion fail closed before the vendor write. The requested pose becomes
     * visible only after that write succeeds.</p>
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
        // The vendor call is not transactional. Fail closed before it starts so a partial pose
        // write cannot leave either the old motion interval or old coordinate snapshot visible.
        LoopTimestamp timestamp = lastTimestamp();
        PinpointKinematicSnapshot previousSnapshot = lastKinematicSnapshot;
        updateState.invalidateMotionBaseline();
        lastEstimate = PoseEstimate.noPose(timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        hasPreviousPhysicalHeading = false;
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                previousSnapshot.cycle,
                timestamp,
                totalHeadingRad
        );

        odo.setPosition(set);

        lastEstimate = new PoseEstimate(new Pose3d(
                rebasedPose.xInches,
                rebasedPose.yInches,
                0.0,
                rebasedPose.headingRad,
                0.0,
                0.0
        ), true, cfg.quality, timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        updateState.rebaseMotionBaseline(lastEstimate.fieldToRobotPose, timestamp);
        // A coordinate correction is not physical motion; retain the last measured velocity/turn.
        previousPhysicalHeadingRad = rebasedPose.headingRad;
        hasPreviousPhysicalHeading = true;
        lastKinematicSnapshot = previousSnapshot.withRebasedPose(rebasedPose);
    }

    private LoopTimestamp lastTimestamp() {
        return lastEstimate != null && lastEstimate.timestamp != null
                ? lastEstimate.timestamp
                : LoopTimestamp.unavailable();
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
     * Emits the cached predictor pose, last motion delta, and static config for telemetry/debugging.
     *
     * <p>The {@code driverStatus} row is one live, status-only read at this owned hardware boundary;
     * it does not call the driver's pose update or acquire a new pose result.</p>
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
                .addData(p + ".lastUpdateCycle", updateState.lastUpdateCycle());
        cfg.debugDump(dbg, p + ".cfg");
    }

    /**
     * Pure-Java state for cycle-attempt ownership and accepted motion baselines.
     *
     * <p>Package-private visibility keeps the production hardware boundary small while allowing
     * deterministic tests to exercise the timing rules without mocking the FTC I2C driver.</p>
     */
    static final class UpdateState {
        private long lastUpdateCycle = Long.MIN_VALUE;
        private boolean updateInProgress;
        private RuntimeException lastUpdateFailure;

        private boolean hasMotionBaseline;
        private Pose3d motionBaselinePose = Pose3d.zero();
        private LoopTimestamp motionBaselineTimestamp = LoopTimestamp.unavailable();

        /** Claim one cycle before its hardware effects, or report a completed duplicate. */
        boolean beginUpdate(LoopClock clock) {
            LoopClock currentClock = Objects.requireNonNull(
                    clock,
                    "Pinpoint update state requires the shared LoopClock"
            );
            long cycle = currentClock.cycle();
            if (updateInProgress) {
                throw new IllegalStateException(
                        "PinpointOdometryPredictor.update(clock) reentered while cycle "
                                + lastUpdateCycle + " was still in progress; one Pinpoint owner "
                                + "may poll hardware once per cycle"
                );
            }
            if (lastUpdateCycle == cycle) {
                if (lastUpdateFailure != null) {
                    throw lastUpdateFailure;
                }
                return false;
            }

            lastUpdateCycle = cycle;
            lastUpdateFailure = null;
            updateInProgress = true;
            return true;
        }

        /** Retain the first RuntimeException produced by the claimed cycle. */
        RuntimeException recordFailure(RuntimeException failure) {
            RuntimeException requiredFailure = Objects.requireNonNull(failure, "failure");
            if (lastUpdateFailure == null) {
                lastUpdateFailure = requiredFailure;
            }
            return lastUpdateFailure;
        }

        /** Finish the claimed attempt without making the cycle eligible for another poll. */
        void endUpdate() {
            updateInProgress = false;
        }

        /**
         * Publish one motion delta only when positive time has elapsed from the accepted baseline.
         */
        MotionDelta observeMotion(Pose3d pose,
                                  LoopTimestamp timestamp,
                                  double quality) {
            Pose3d currentPose = Objects.requireNonNull(pose, "pose");
            LoopTimestamp currentTimestamp = Objects.requireNonNull(timestamp, "timestamp");

            if (!hasMotionBaseline) {
                rebaseMotionBaseline(currentPose, currentTimestamp);
                return MotionDelta.none(currentTimestamp);
            }

            double elapsedSec = currentTimestamp.secondsSince(motionBaselineTimestamp);
            if (!Double.isFinite(elapsedSec) || elapsedSec < 0.0) {
                rebaseMotionBaseline(currentPose, currentTimestamp);
                return MotionDelta.none(currentTimestamp);
            }
            if (elapsedSec == 0.0) {
                return MotionDelta.none(currentTimestamp);
            }

            MotionDelta result = new MotionDelta(
                    motionBaselinePose.inverse().then(currentPose),
                    true,
                    quality,
                    motionBaselineTimestamp,
                    currentTimestamp
            );
            rebaseMotionBaseline(currentPose, currentTimestamp);
            return result;
        }

        /** Anchor later motion to a deliberate coordinate rebase. */
        void rebaseMotionBaseline(Pose3d pose, LoopTimestamp timestamp) {
            motionBaselinePose = Objects.requireNonNull(pose, "pose");
            motionBaselineTimestamp = Objects.requireNonNull(timestamp, "timestamp");
            hasMotionBaseline = true;
        }

        /** Require the next valid sample to establish a fresh physical-motion baseline. */
        void invalidateMotionBaseline() {
            hasMotionBaseline = false;
            motionBaselinePose = Pose3d.zero();
            motionBaselineTimestamp = LoopTimestamp.unavailable();
        }

        long lastUpdateCycle() {
            return lastUpdateCycle;
        }
    }
}
