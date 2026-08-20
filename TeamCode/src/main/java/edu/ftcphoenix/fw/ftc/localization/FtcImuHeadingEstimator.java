package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.localization.HeadingEstimate;
import edu.ftcphoenix.fw.localization.HeadingEstimator;

/** Managed FTC Control/Expansion Hub IMU heading owner aligned to a fixed field frame at START. */
public final class FtcImuHeadingEstimator implements HeadingEstimator, RobotProgram.Service {

    /** Mutable FTC-boundary configuration, defensively copied at construction. */
    public static final class Config {
        /** FTC hardware-map name. */
        public String hardwareName = "imu";
        /** Physical Hub logo direction on the robot. */
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        /** Physical Hub USB direction on the robot. */
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        private Config() {
        }

        /** Return a new mutable config with standard upright-Hub defaults. */
        public static Config defaults() {
            return new Config();
        }

        /** Return an independent copy. */
        public Config copy() {
            Config copy = new Config();
            copy.hardwareName = hardwareName;
            copy.logoFacingDirection = logoFacingDirection;
            copy.usbFacingDirection = usbFacingDirection;
            return copy;
        }
    }

    interface ImuAccess {
        double yawRad();
    }

    private final ImuAccess imu;
    private final DoubleSupplier initialRobotFieldHeadingSupplier;
    private HeadingEstimate estimate = HeadingEstimate.noHeading(LoopTimestamp.unavailable());
    private boolean started;
    private boolean stopped;
    private double fieldOffsetRad;
    private long attemptedCycle = Long.MIN_VALUE;
    private RuntimeException retainedFailure;
    private boolean updating;

    /**
     * Construct and initialize the configured FTC IMU.
     *
     * @param hardwareMap FTC device registry
     * @param config physical IMU configuration, defensively copied
     * @param initialRobotFieldHeadingSupplier cache-only supplier read once at START after station
     *                                          selection freezes
     */
    public FtcImuHeadingEstimator(HardwareMap hardwareMap,
                                  Config config,
                                  DoubleSupplier initialRobotFieldHeadingSupplier) {
        Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config snapshot = requireValid(config);
        this.initialRobotFieldHeadingSupplier = Objects.requireNonNull(
                initialRobotFieldHeadingSupplier,
                "initialRobotFieldHeadingSupplier is required"
        );
        IMU sdkImu = hardwareMap.get(IMU.class, snapshot.hardwareName);
        sdkImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                snapshot.logoFacingDirection,
                snapshot.usbFacingDirection
        )));
        this.imu = () -> sdkImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    FtcImuHeadingEstimator(ImuAccess imu,
                           DoubleSupplier initialRobotFieldHeadingSupplier) {
        this.imu = Objects.requireNonNull(imu, "imu is required");
        this.initialRobotFieldHeadingSupplier = Objects.requireNonNull(
                initialRobotFieldHeadingSupplier,
                "initialRobotFieldHeadingSupplier is required"
        );
    }

    private static Config requireValid(Config config) {
        if (config == null) {
            throw new IllegalArgumentException("FtcImuHeadingEstimator.Config is required");
        }
        Config snapshot = config.copy();
        if (snapshot.hardwareName == null || snapshot.hardwareName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "FtcImuHeadingEstimator.Config.hardwareName must be non-blank");
        }
        snapshot.hardwareName = snapshot.hardwareName.trim();
        Objects.requireNonNull(
                snapshot.logoFacingDirection,
                "FtcImuHeadingEstimator.Config.logoFacingDirection is required"
        );
        Objects.requireNonNull(
                snapshot.usbFacingDirection,
                "FtcImuHeadingEstimator.Config.usbFacingDirection is required"
        );
        return snapshot;
    }

    /** Align current raw yaw with the station-authored robot field heading. */
    @Override
    public void start(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        if (started || stopped) {
            throw new IllegalStateException(
                    "FtcImuHeadingEstimator may start exactly once on a fresh owner");
        }
        double initialFieldHeadingRad = initialRobotFieldHeadingSupplier.getAsDouble();
        double rawYawRad = imu.yawRad();
        if (!Double.isFinite(initialFieldHeadingRad)) {
            throw new IllegalStateException(
                    "initial robot field heading must be finite, got " + initialFieldHeadingRad);
        }
        if (!Double.isFinite(rawYawRad)) {
            throw new IllegalStateException("FTC IMU yaw at START must be finite, got " + rawYawRad);
        }
        fieldOffsetRad = MathUtil.wrapToPi(initialFieldHeadingRad - rawYawRad);
        started = true;
        estimate = new HeadingEstimate(
                MathUtil.wrapToPi(rawYawRad + fieldOffsetRad),
                true,
                1.0,
                clock.nowTimestamp()
        );
    }

    /** Read and publish one field-aligned yaw snapshot. */
    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        if (!started || stopped) {
            throw new IllegalStateException(
                    "FtcImuHeadingEstimator.update(clock) requires an active started owner");
        }
        long cycle = clock.cycle();
        if (updating) {
            throw new IllegalStateException("FtcImuHeadingEstimator.update(clock) is reentrant");
        }
        if (cycle == attemptedCycle) {
            if (retainedFailure != null) {
                throw retainedFailure;
            }
            return;
        }
        attemptedCycle = cycle;
        updating = true;
        try {
            double rawYawRad = imu.yawRad();
            estimate = Double.isFinite(rawYawRad)
                    ? new HeadingEstimate(
                    MathUtil.wrapToPi(rawYawRad + fieldOffsetRad),
                    true,
                    1.0,
                    clock.nowTimestamp())
                    : HeadingEstimate.noHeading(clock.nowTimestamp());
            retainedFailure = null;
        } catch (RuntimeException failure) {
            retainedFailure = failure;
            throw failure;
        } finally {
            updating = false;
        }
    }

    /** Return the latest cache without another IMU read. */
    @Override
    public HeadingEstimate getHeadingEstimate() {
        return estimate;
    }

    /** Make cached evidence unavailable; FTC's shared hardware registry retains the IMU device. */
    @Override
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        estimate = HeadingEstimate.noHeading(LoopTimestamp.unavailable());
    }
}
