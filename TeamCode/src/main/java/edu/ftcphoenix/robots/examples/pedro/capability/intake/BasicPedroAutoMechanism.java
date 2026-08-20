package edu.ftcphoenix.robots.examples.pedro.capability.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.ScalarTasks;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.task.Task;

/**
 * Small intake capability used by the basic Pedro Auto reference.
 *
 * <p>The capability creates fresh Tasks while the retained Plant remains the only final actuator
 * owner. Deferred Tasks use the Plant's stable
 * {@link Plant#commandTarget() graph-owned command}; final cleanup terminates the Plant without
 * rewriting that graph. A real robot
 * normally replaces this class with its existing mechanism capability. Ordinary FTC construction
 * receives {@code HardwareMap} plus the mechanism's data-only {@link Config} and privately builds
 * its Plant; the completed-Plant constructor remains only a package-private hardware-neutral
 * test/portable seam.</p>
 */
public final class BasicPedroAutoMechanism implements RobotProgram.Output {

    /** Data-only FTC wiring and command configuration for this example mechanism. */
    public static final class Config {
        /** FTC configuration name of the intake motor. */
        public String motorName;

        /** Physical direction of positive intake motor power. */
        public Direction direction;

        /** Normalized finite, nonzero collection power in {@code [-1, +1]}. */
        public double collectPower;

        private Config() {
            // Use defaults() to begin from a complete software-valid example.
        }

        /**
         * Returns a fresh, complete software baseline; it is not a physical-safety claim.
         */
        public static Config defaults() {
            Config config = new Config();
            config.motorName = "intakeMotor";
            config.direction = Direction.FORWARD;
            config.collectPower = 0.20;
            return config;
        }
    }

    private static final double IDLE_POWER = 0.0;

    private final Plant intakePlant;
    private final double collectPower;
    private boolean stopped;

    /**
     * Construct and privately own the ordinary FTC intake Plant from a defensive configuration
     * snapshot.
     *
     * <p>All data-only configuration is validated before hardware resolution. If construction
     * reaches a completed Plant and a later mechanism invariant fails, that Plant is immediately
     * stopped before the failure is rethrown.</p>
     *
     * @param hardwareMap FTC hardware registry
     * @param config complete data-only mechanism configuration
     */
    public BasicPedroAutoMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        Config source = Objects.requireNonNull(config, "BasicPedroAutoMechanism.Config is required");
        String motorName = source.motorName;
        Direction direction = source.direction;
        double copiedCollectPower = source.collectPower;
        requireHardwareName(motorName);
        requireDirection(direction);
        requireCollectPower(copiedCollectPower);

        Plant builtPlant = FtcActuators.plant(requiredHardwareMap)
                .motor(motorName, direction)
                .power()
                .targetFromNewCommand(IDLE_POWER)
                .build();
        try {
            requireCommandPlant(builtPlant);
        } catch (RuntimeException constructionFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    constructionFailure,
                    builtPlant::stop
            );
        }

        intakePlant = builtPlant;
        collectPower = copiedCollectPower;
    }

    /**
     * Creates the portable example capability around one normalized-power Plant with a command
     * target. This package-private constructor is the explicitly hardware-neutral test/portable
     * seam, not the ordinary FTC mechanism construction path. Its collection power comes only from
     * {@link Config#defaults()} so the completed Plant is its sole peer dependency.
     *
     * @param intakePlant source-driven Plant whose update/stop lifecycle this mechanism owns
     * @throws NullPointerException if the Plant or its claimed command target is null
     * @throws IllegalArgumentException if the Plant has no command target
     */
    BasicPedroAutoMechanism(Plant intakePlant) {
        this.intakePlant = requireCommandPlant(
                Objects.requireNonNull(intakePlant, "intakePlant")
        );
        Config defaults = Config.defaults();
        requireCollectPower(defaults.collectPower);
        collectPower = defaults.collectPower;
    }

    /**
     * Creates a fresh Task that collects for the requested duration and returns to idle.
     *
     * <p>Active cancellation also writes the idle request before the Task becomes terminal.</p>
     */
    public Task collectTask(double durationSec) {
        return ScalarTasks.set(intakePlant.commandTarget(), collectPower)
                .forSeconds(durationSec)
                .then(IDLE_POWER)
                .build();
    }

    /** Creates a fresh write-once Task that restores the safe idle request when started. */
    public Task idleTask() {
        return ScalarTasks.set(intakePlant.commandTarget(), IDLE_POWER).build();
    }

    /** Applies the capability's final source-driven Plant target for this loop. */
    @Override
    public void update(LoopClock clock) {
        if (!stopped) {
            intakePlant.update(clock);
        }
    }

    /**
     * Permanently stops the Plant without rewriting its persistent command.
     */
    @Override
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;

        intakePlant.stop();
    }

    private static Plant requireCommandPlant(Plant plant) {
        if (!plant.hasCommandTarget()) {
            throw new IllegalArgumentException(
                    "BasicPedroAutoMechanism requires intakePlant to have a command target"
            );
        }
        Objects.requireNonNull(
                plant.commandTarget(),
                "intakePlant.commandTarget()"
        );
        return plant;
    }

    private static void requireCollectPower(double collectPower) {
        if (!Double.isFinite(collectPower)
                || collectPower == 0.0
                || collectPower < -1.0
                || collectPower > 1.0) {
            throw new IllegalArgumentException(
                    "BasicPedroAutoMechanism.Config.collectPower must be finite, nonzero, and in "
                            + "[-1.0, 1.0], got " + collectPower
            );
        }
    }

    private static void requireHardwareName(String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "BasicPedroAutoMechanism.Config.motorName must be a non-blank FTC hardware "
                            + "name, got "
                            + (value == null ? "null" : "\"" + value + "\"")
            );
        }
    }

    private static void requireDirection(Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException(
                    "BasicPedroAutoMechanism.Config.direction is required, got null"
            );
        }
    }
}
