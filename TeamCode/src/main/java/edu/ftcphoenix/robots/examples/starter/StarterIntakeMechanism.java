package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.ScalarTasks;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.task.Task;

/** Owns the starter intake's one final target resolver, Plant, update, and stop command. */
final class StarterIntakeMechanism implements StarterIntake, RobotProgram.Output {

    private static final double STOPPED_POWER = 0.0;

    private final Plant plant;
    private final double collectPower;
    private final double ejectPower;

    StarterIntakeMechanism(HardwareMap hardwareMap, StarterProfile.IntakeConfig config) {
        StarterProfile.IntakeConfig snapshot = Objects.requireNonNull(config, "config").copy();
        requireActionPowers(snapshot.collectPower, snapshot.ejectPower);
        collectPower = snapshot.collectPower;
        ejectPower = snapshot.ejectPower;
        plant = FtcActuators.plant(Objects.requireNonNull(hardwareMap, "hardwareMap"))
                .motor(snapshot.motorName, snapshot.direction)
                .power()
                .targetFromNewCommand(STOPPED_POWER)
                .build();
    }

    /** Package-private hardware-neutral seam for focused tests. */
    StarterIntakeMechanism(
            Plant plant,
            double collectPower,
            double ejectPower) {
        this.plant = Objects.requireNonNull(plant, "plant");
        if (!plant.hasCommandTarget()) {
            throw new IllegalArgumentException(
                    "starter intake plant must have a command target");
        }
        Objects.requireNonNull(
                plant.commandTarget(),
                "plant.commandTarget()");
        requireActionPowers(collectPower, ejectPower);
        this.collectPower = collectPower;
        this.ejectPower = ejectPower;
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
        CleanupActions.attemptAll(
                () -> plant.commandTarget().set(STOPPED_POWER),
                plant::stop);
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

    private static void requireActionPowers(double collectPower, double ejectPower) {
        requireActionPower("collectPower", collectPower);
        requireActionPower("ejectPower", ejectPower);
        if (Double.compare(collectPower, ejectPower) == 0) {
            throw new IllegalArgumentException("collectPower and ejectPower must be different");
        }
    }

    private static void requireActionPower(String name, double value) {
        if (!Double.isFinite(value) || value == 0.0 || value < -1.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    name + " must be finite, nonzero, and in [-1.0, 1.0], got " + value);
        }
    }
}
