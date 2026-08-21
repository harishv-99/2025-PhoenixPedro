package edu.ftcphoenix.robots.examples.starter.capability.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.task.RunForSecondsTask;
import edu.ftcphoenix.fw.task.Task;

/**
 * Owns the starter intake's semantic request, one final target resolver, Plant, update, and stop.
 * {@link Mode} is mapped forward to configured power and is never inferred from a numeric target.
 */
public final class StarterIntakeMechanism implements StarterIntake, RobotProgram.Output {

    /**
     * Data-only FTC wiring and normalized action-power configuration for the starter intake.
     * Power values realize modes; software does not use numeric uniqueness to identify them. Teams
     * must still validate that chosen values produce the intended physical actions.
     */
    public static final class Config {
        /** FTC Robot Configuration name for the intake motor. */
        public String motorName;

        /** Logical direction that makes the configured action-power signs correct. */
        public Direction direction;

        /** Normalized nonzero power used while collecting. */
        public double collectPower;

        /** Normalized nonzero power used while ejecting. */
        public double ejectPower;

        private Config() {
            // Use defaults() to begin from a complete software-valid example.
        }

        /** Returns a fresh, complete software baseline; it is not a physical-safety claim. */
        public static Config defaults() {
            Config config = new Config();
            config.motorName = "intakeMotor";
            config.direction = Direction.FORWARD;
            config.collectPower = 0.20;
            config.ejectPower = -0.20;
            return config;
        }
    }

    private static final double STOPPED_POWER = 0.0;

    private final Plant plant;
    private final double collectPower;
    private final double ejectPower;
    private Mode requestedMode = Mode.STOPPED;

    /**
     * Constructs and privately owns the ordinary FTC intake Plant from a defensive Config
     * snapshot. Every Config field is validated before hardware lookup.
     *
     * @param hardwareMap FTC hardware registry
     * @param config complete data-only intake configuration
     */
    public StarterIntakeMechanism(HardwareMap hardwareMap, Config config) {
        Config source = Objects.requireNonNull(
                config,
                "StarterIntakeMechanism.Config is required"
        );
        String motorName = source.motorName;
        Direction direction = source.direction;
        double copiedCollectPower = source.collectPower;
        double copiedEjectPower = source.ejectPower;

        requireHardwareName(motorName);
        requireDirection(direction);
        requireActionPowers(copiedCollectPower, copiedEjectPower);

        plant = FtcActuators.plant(
                        Objects.requireNonNull(hardwareMap, "hardwareMap is required")
                )
                .motor(motorName, direction)
                .power()
                .targetFromNewCommand(STOPPED_POWER)
                .build();
        collectPower = copiedCollectPower;
        ejectPower = copiedEjectPower;
    }

    @Override
    public void setMode(Mode mode) {
        Mode requested = Objects.requireNonNull(mode, "mode");
        plant.commandTarget().set(powerFor(requested));
        requestedMode = requested;
    }

    @Override
    public Task collectForSeconds(double durationSec) {
        if (!(durationSec > 0.0) || !Double.isFinite(durationSec)) {
            throw new IllegalArgumentException(
                    "durationSec must be finite and > 0, got " + durationSec);
        }
        return new RunForSecondsTask(
                durationSec,
                () -> setMode(Mode.COLLECT),
                ignoredClock -> setMode(Mode.COLLECT),
                () -> setMode(Mode.STOPPED));
    }

    @Override
    public Status status() {
        return new Status(requestedMode, plant.getAppliedTarget());
    }

    @Override
    public void update(LoopClock clock) {
        plant.update(clock);
    }

    @Override
    public void stop() {
        plant.stop();
    }

    private double powerFor(Mode mode) {
        switch (mode) {
            case COLLECT:
                return collectPower;
            case EJECT:
                return ejectPower;
            case STOPPED:
            default:
                return STOPPED_POWER;
        }
    }

    private static void requireHardwareName(String motorName) {
        if (motorName == null || motorName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "StarterIntakeMechanism.Config.motorName must be a non-blank FTC hardware "
                            + "name, got "
                            + (motorName == null ? "null" : "\"" + motorName + "\""));
        }
    }

    private static void requireDirection(Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException(
                    "StarterIntakeMechanism.Config.direction is required, got null");
        }
    }

    private static void requireActionPowers(double collectPower, double ejectPower) {
        requireActionPower(
                "StarterIntakeMechanism.Config.collectPower",
                collectPower
        );
        requireActionPower(
                "StarterIntakeMechanism.Config.ejectPower",
                ejectPower
        );
    }

    private static void requireActionPower(String fieldName, double value) {
        if (!Double.isFinite(value) || value == 0.0 || value < -1.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    fieldName + " must be finite, nonzero, and in [-1.0, 1.0], got " + value);
        }
    }
}
