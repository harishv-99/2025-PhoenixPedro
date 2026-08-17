package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.ScalarTasks;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.task.Task;

/** Owns the starter intake's one final target resolver, Plant, update, and stop command. */
public final class StarterIntakeMechanism implements StarterIntake, RobotProgram.Output {

    /** Data-only FTC wiring and normalized action-power configuration for the starter intake. */
    public static final class Config {
        /** FTC Robot Configuration name for the intake motor. */
        public String motorName;

        /** Logical direction that makes the configured action-power signs correct. */
        public Direction direction;

        /** Normalized power used while collecting. */
        public double collectPower;

        /** Normalized power used while ejecting. */
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

    /**
     * Package-private hardware-neutral seam for hostile command-target and lifecycle tests.
     * Action powers come only from {@link Config#defaults()}.
     */
    StarterIntakeMechanism(Plant plant) {
        this.plant = requireCommandPlant(Objects.requireNonNull(plant, "plant is required"));
        Config defaults = Config.defaults();
        requireActionPowers(defaults.collectPower, defaults.ejectPower);
        collectPower = defaults.collectPower;
        ejectPower = defaults.ejectPower;
    }

    @Override
    public void setMode(Mode mode) {
        plant.commandTarget().set(powerFor(Objects.requireNonNull(mode, "mode")));
    }

    @Override
    public Task collectForSeconds(double durationSec) {
        if (!(durationSec > 0.0) || !Double.isFinite(durationSec)) {
            throw new IllegalArgumentException(
                    "durationSec must be finite and > 0, got " + durationSec);
        }
        return ScalarTasks.set(plant.commandTarget(), collectPower)
                .forSeconds(durationSec)
                .then(STOPPED_POWER)
                .build();
    }

    @Override
    public Status status() {
        double requestedPower = plant.commandTarget().get();
        return new Status(
                modeFor(requestedPower),
                plant.getAppliedTarget());
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

    private Mode modeFor(double power) {
        if (Double.compare(power, collectPower) == 0) {
            return Mode.COLLECT;
        }
        if (Double.compare(power, ejectPower) == 0) {
            return Mode.EJECT;
        }
        return Mode.STOPPED;
    }

    private static Plant requireCommandPlant(Plant plant) {
        if (!plant.hasCommandTarget()) {
            throw new IllegalArgumentException(
                    "StarterIntakeMechanism requires plant to have a command target");
        }
        Objects.requireNonNull(
                plant.commandTarget(),
                "plant.commandTarget()"
        );
        return plant;
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
        if (Double.compare(collectPower, ejectPower) == 0) {
            throw new IllegalArgumentException(
                    "StarterIntakeMechanism.Config.collectPower and "
                            + "StarterIntakeMechanism.Config.ejectPower must be different, got "
                            + collectPower + " and " + ejectPower);
        }
    }

    private static void requireActionPower(String fieldName, double value) {
        if (!Double.isFinite(value) || value == 0.0 || value < -1.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    fieldName + " must be finite, nonzero, and in [-1.0, 1.0], got " + value);
        }
    }
}
