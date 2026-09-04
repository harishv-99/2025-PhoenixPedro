package edu.ftcsushi.robots.examples.basicflywheel;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.ScalarTasks;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/** Owns one device-managed velocity Plant and realizes {@link BasicFlywheel} intent. */
public final class BasicFlywheelMechanism implements BasicFlywheel, RobotProgram.Output {

    /** Data-only motor wiring, legal velocity range, tolerance, and Task timeout. */
    public static final class Config {
        /** FTC Robot Configuration name for the flywheel motor. */
        public String motorName;

        /** Logical direction for positive flywheel velocity. */
        public Direction direction;

        /** Inclusive maximum velocity command, in native encoder ticks per second. */
        public double maximumVelocityTicksPerSec;

        /**
         * Inclusive feedback-completion tolerance, in native encoder ticks per second; finite,
         * nonnegative, and strictly less than {@link #maximumVelocityTicksPerSec}.
         */
        public double velocityToleranceTicksPerSec;

        /** Positive timeout for a feedback-aware velocity Task, in seconds. */
        public double spinUpTimeoutSec;

        private Config() {
            // Begin with defaults() so every required software decision is visible.
        }

        /**
         * Returns a complete software-valid candidate, not reviewed motor identity, safety, or
         * controller tuning.
         */
        public static Config defaults() {
            Config config = new Config();
            config.motorName = "flywheelMotor";
            config.direction = Direction.FORWARD;
            config.maximumVelocityTicksPerSec = 500.0;
            config.velocityToleranceTicksPerSec = 25.0;
            config.spinUpTimeoutSec = 2.0;
            return config;
        }
    }

    private static final double STOPPED_VELOCITY_TICKS_PER_SEC = 0.0;

    private final Plant flywheel;
    private final double maximumVelocityTicksPerSec;
    private final double spinUpTimeoutSec;

    /**
     * Validates a defensive configuration snapshot, then constructs and privately owns the Plant.
     *
     * <p>The ordinary {@code deviceManaged()} path preserves the FTC controller's existing PIDF
     * coefficients. Construction does not submit a velocity command; the first owner
     * {@link #update(LoopClock)} realizes the initial zero request.</p>
     *
     * @param hardwareMap FTC hardware registry containing the configured motor
     * @param config complete data-only flywheel configuration
     */
    public BasicFlywheelMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        maximumVelocityTicksPerSec = c.maximumVelocityTicksPerSec;
        spinUpTimeoutSec = c.spinUpTimeoutSec;
        flywheel = FtcActuators.plant(map)
                .motor(c.motorName, c.direction)
                .velocity()
                .deviceManaged()
                .bounded(STOPPED_VELOCITY_TICKS_PER_SEC, c.maximumVelocityTicksPerSec)
                .nativeUnits()
                .velocityTolerance(c.velocityToleranceTicksPerSec)
                .targetFromNewCommand(STOPPED_VELOCITY_TICKS_PER_SEC)
                .build();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocityTicksPerSec(double velocityTicksPerSec) {
        requireVelocityInRange(velocityTicksPerSec);
        flywheel.commandTarget().set(velocityTicksPerSec);
    }

    /** {@inheritDoc} */
    @Override
    public Task setVelocityTask(double velocityTicksPerSec) {
        requireVelocityInRange(velocityTicksPerSec);
        return ScalarTasks.set(flywheel.commandTarget(), velocityTicksPerSec)
                .untilReachedBy(flywheel)
                .cancelTo(STOPPED_VELOCITY_TICKS_PER_SEC)
                .timeout(spinUpTimeoutSec)
                .build();
    }

    /** {@inheritDoc} */
    @Override
    public Status status() {
        return new Status(flywheel.snapshot());
    }

    /** Applies the current request once and refreshes the Plant's cached feedback evidence. */
    @Override
    public void update(LoopClock clock) {
        flywheel.update(clock);
    }

    /** Terminally stops the one privately owned Plant. */
    @Override
    public void stop() {
        flywheel.stop();
    }

    private void requireVelocityInRange(double velocityTicksPerSec) {
        if (!Double.isFinite(velocityTicksPerSec)
                || velocityTicksPerSec < STOPPED_VELOCITY_TICKS_PER_SEC
                || velocityTicksPerSec > maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "velocityTicksPerSec must be finite and in [0, "
                            + maximumVelocityTicksPerSec + "], got " + velocityTicksPerSec);
        }
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "BasicFlywheelMechanism.Config is required");
        Config c = new Config();
        c.motorName = hardwareName(s.motorName);
        c.direction = Objects.requireNonNull(s.direction, "direction is required");
        c.maximumVelocityTicksPerSec = positive(
                s.maximumVelocityTicksPerSec, "maximumVelocityTicksPerSec");
        c.velocityToleranceTicksPerSec = nonnegative(
                s.velocityToleranceTicksPerSec, "velocityToleranceTicksPerSec");
        if (c.velocityToleranceTicksPerSec >= c.maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "velocityToleranceTicksPerSec must be strictly less than "
                            + "maximumVelocityTicksPerSec so a zero measurement is outside the "
                            + "completion band at the maximum request; got tolerance "
                            + c.velocityToleranceTicksPerSec + " and maximum "
                            + c.maximumVelocityTicksPerSec);
        }
        c.spinUpTimeoutSec = positive(s.spinUpTimeoutSec, "spinUpTimeoutSec");
        return c;
    }

    private static String hardwareName(String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "BasicFlywheelMechanism.Config.motorName must be a non-blank FTC hardware "
                            + "name");
        }
        return value.trim();
    }

    private static double positive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double nonnegative(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(field + " must be finite and >= 0, got " + value);
        }
        return value;
    }
}
