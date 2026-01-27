package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.core.control.DebounceLatch;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Phoenix robot shooter subsystem.
 *
 * <p>This class wires the shooter-related {@link edu.ftcphoenix.fw.actuation.Plant}s
 * and exposes small, non-blocking {@link Task} helpers for TeleOp and autonomous.
 * Methods are named "instantX" to emphasize that they return tasks that can be
 * scheduled without blocking the main loop.</p>
 */
public class Shooter {

    /**
     * Direction for the transfer (indexer) CR servos.
     */
    public enum TransferDirection {
        FORWARD,
        BACKWARD
    }

    private Plant plantIntake;
    private Plant plantIntakeTransfer;
    private Plant plantShooterTransfer;
    private Plant plantShooter;

    /**
     * Shooter "ready" latch: requires atSetpoint() to be continuously true for
     * {@link RobotConfig.Shooter#readyStableSec} seconds before we report ready.
     */
    private final DebounceLatch readyLatch =
            DebounceLatch.onAfterOffImmediately(RobotConfig.Shooter.readyStableSec);

    private double velocity;
    private boolean isShooterOn;

    // ----------------------------------------------------------------------
    // Calibration table: distance (in) → shooter velocity (native units)
    // ----------------------------------------------------------------------

    /**
     * Shooter velocity table built from sorted (distance, velocity) pairs.
     */
    private static final InterpolatingTable1D SHOOTER_VELOCITY_TABLE =
            InterpolatingTable1D.ofSortedPairs(
                    42, 0,
                    45.8, 1600,
                    47, 1600,
                    49.9, 1550,
                    56.6, 1550,
                    66.5, 1550,
                    76.6, 1600,
                    83.4, 1600,
                    94.5, 1650,
                    102.5, 1625,
                    105.5, 1650,
                    113.3, 1650,
                    117.8, 1685,
                    121, 1700,
                    125, 1700,
                    130.2, 1700,
                    139.4, 1750,
                    150, 1800
            );


    /**
     * Construct the shooter subsystem and wire all associated hardware.
     */
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, Gamepads gamepads) {
        plantIntake = Actuators.plant(hardwareMap)
                .motor(RobotConfig.Shooter.nameMotorIntake,
                    RobotConfig.Shooter.directionMotorIntake)
                .power()
                .build();

        plantIntakeTransfer = Actuators.plant(hardwareMap)
                .crServo(RobotConfig.Shooter.nameCrServoIntakeTransfer,
                        RobotConfig.Shooter.directionServoIntakeTransfer)
                .power()
                .build();

        plantShooterTransfer = Actuators.plant(hardwareMap)
                .crServo(RobotConfig.Shooter.nameCrServoTransferServoLeft,
                        RobotConfig.Shooter.directionServoTransferLeft)
                .andCrServo(RobotConfig.Shooter.nameCrServoTransferServoRight,
                        RobotConfig.Shooter.directionServoTransferRight)
                .power()
                .build();

        plantShooter = Actuators.plant(hardwareMap)
                .motor(RobotConfig.Shooter.nameMotorShooter,
                        RobotConfig.Shooter.directionMotorShooter)
                .velocity(RobotConfig.Shooter.velocityToleranceNative)
                .build();

        isShooterOn = false;
        velocity = RobotConfig.Shooter.velocityMin;
    }

    /**
     * Update the target shooter velocity based on an estimated distance.
     *
     * @param distance distance to target in inches
     * @return a task that applies the new velocity immediately if the shooter is on;
     * otherwise {@link Tasks#noop()}
     */
    public Task instantSetVelocityByDist(double distance) {
        double velForDist = SHOOTER_VELOCITY_TABLE.interpolate(distance);
        this.velocity = MathUtil.clamp(velForDist,
                RobotConfig.Shooter.velocityMin,
                RobotConfig.Shooter.velocityMax);

        if (isShooterOn) {
            return instantStartShooter();
        }

        return Tasks.noop();
    }

    /**
     * Increase the configured shooter velocity by one increment.
     */
    public Task instantIncreaseVelocity() {
        velocity += RobotConfig.Shooter.velocityIncrement;
        velocity = Math.floor(velocity / RobotConfig.Shooter.velocityIncrement) *
                RobotConfig.Shooter.velocityIncrement;
        velocity = MathUtil.clamp(velocity,
                RobotConfig.Shooter.velocityMin,
                RobotConfig.Shooter.velocityMax);

        if (isShooterOn) {
            return instantStartShooter();
        }

        return Tasks.noop();
    }

    /**
     * Decrease the configured shooter velocity by one increment.
     */
    public Task instantDecreaseVelocity() {
        velocity -= RobotConfig.Shooter.velocityIncrement;
        velocity = Math.ceil(velocity / RobotConfig.Shooter.velocityIncrement) *
                RobotConfig.Shooter.velocityIncrement;
        velocity = MathUtil.clamp(velocity,
                RobotConfig.Shooter.velocityMin,
                RobotConfig.Shooter.velocityMax);

        if (isShooterOn) {
            return instantStartShooter();
        }

        return Tasks.noop();
    }

    /**
     * @return current configured shooter velocity (native units)
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * @return whether the shooter motors are currently commanded on.
     */
    public boolean isShooterOn() {
        return isShooterOn;
    }

    /**
     * Start (or re-start) the shooter at the currently configured velocity.
     */
    public Task instantStartShooter() {
        isShooterOn = true;
        readyLatch.reset(false);
        return PlantTasks.setInstant(plantShooter, velocity);
    }

    /**
     * Stop the shooter motor(s).
     */
    public Task instantStopShooter() {
        isShooterOn = false;
        readyLatch.reset(false);
        return PlantTasks.setInstant(plantShooter, 0);
    }

    /**
     * @return true if the shooter is commanded on AND the shooter plant reports {@link Plant#atSetpoint()}.
     */
    public boolean isShooterAtSetpoint() {
        return isShooterOn && plantShooter != null && plantShooter.atSetpoint();
    }

    /**
     * Shooter "ready" check suitable for gating a feeder.
     *
     * <p>Returns true only after the shooter has remained at setpoint continuously for
     * {@link RobotConfig.Shooter#readyStableSec} seconds. When the shooter is turned off,
     * readiness is forced false.</p>
     */
    public boolean isShooterReady(LoopClock clock) {
        return readyLatch.update(clock, isShooterAtSetpoint());
    }

    /**
     * Start the intake (indexer) in the requested direction.
     */
    public Task instantStartIntake(TransferDirection direction) {
        switch (direction) {
            case FORWARD:
                return Tasks.sequence(PlantTasks.setInstant(plantIntake, 0.5),
                        PlantTasks.setInstant(plantIntakeTransfer, 1));
            case BACKWARD:
                return Tasks.sequence(PlantTasks.setInstant(plantIntake, -0.5),
                        PlantTasks.setInstant(plantIntakeTransfer, -1));
        }

        throw new IllegalArgumentException("Unknown direction provided!!!");
    }

    /**
     * Stop the intake (indexer)
     */
    public Task instantStopIntake() {
        return Tasks.sequence(PlantTasks.setInstant(plantIntake, 0),
                PlantTasks.setInstant(plantIntakeTransfer, 0));
    }

    /**
     * Start the transfer (indexer) in the requested direction.
     */
    public Task instantStartTransfer(TransferDirection direction) {
        switch (direction) {
            case FORWARD:
                return PlantTasks.setInstant(plantShooterTransfer, 1);
            case BACKWARD:
                return PlantTasks.setInstant(plantShooterTransfer, -1);
        }

        throw new IllegalArgumentException("Unknown direction provided!!!");
    }

    /**
     * Stop the transfer (indexer).
     */
    public Task instantStopTransfer() {
        return PlantTasks.setInstant(plantShooterTransfer, 0);
    }

    /**
     * Emergency stop for all shooter-related actuators.
     */
    public void stop() {
        plantIntake.stop();
        plantIntakeTransfer.stop();
        plantShooter.stop();
        plantShooterTransfer.stop();
    }


    /**
     * Debug helper: emit shooter state and delegate to each plant.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "shooter" : prefix;

        dbg.addLine(p)
                .addData(p + ".velocity", velocity)
                .addData(p + ".isShooterOn", isShooterOn)
                .addData(p + ".atSetpoint", isShooterAtSetpoint())
                .addData(p + ".ready", readyLatch.get());

        readyLatch.debugDump(dbg, p + ".readyLatch");

        if (plantShooter != null) {
            plantShooter.debugDump(dbg, p + ".plantShooter");
        }
        if (plantIntakeTransfer != null) {
            plantIntakeTransfer.debugDump(dbg, p + ".plantIntakeTransfer");
        }
        if (plantIntake != null) {
            plantIntake.debugDump(dbg, p + ".plantIntake");
        }
        if(plantShooterTransfer != null) {
            plantShooterTransfer.debugDump(dbg, p + ".plantShooterTransfer");
        }
    }

}
