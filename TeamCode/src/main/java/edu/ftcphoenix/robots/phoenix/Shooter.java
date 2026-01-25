package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.math.MathUtil;
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

    private Plant plantPusher;
    private Plant plantTransfer;
    private Plant plantShooter;

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
        plantPusher = Actuators.plant(hardwareMap)
                .servo(RobotConfig.Shooter.nameServoPusher,
                        RobotConfig.Shooter.directionServoPusher)
                .position()
                .build();

        plantTransfer = Actuators.plant(hardwareMap)
                .crServo(RobotConfig.Shooter.nameCrServoTransferLeft,
                        RobotConfig.Shooter.directionServoTransferLeft)
                .andCrServo(RobotConfig.Shooter.nameCrServoTransferRight,
                        RobotConfig.Shooter.directionServoTransferRight)
                .power()
                .build();

        plantShooter = Actuators.plant(hardwareMap)
                .motor(RobotConfig.Shooter.nameMotorShooterLeft,
                        RobotConfig.Shooter.directionMotorShooterLeft)
                .andMotor(RobotConfig.Shooter.nameMotorShooterRight,
                        RobotConfig.Shooter.directionMotorShooterRight)
                .velocity(50)
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
        return PlantTasks.setInstant(plantShooter, velocity);
    }

    /**
     * Stop the shooter motor(s).
     */
    public Task instantStopShooter() {
        isShooterOn = false;
        return PlantTasks.setInstant(plantShooter, 0);
    }

    /**
     * Move the pusher servo to the "back" (retracted) position.
     */
    public Task instantSetPusherBack() {
//        return PlantTasks.holdFor(plantPusher,
//                RobotConfig.Shooter.targetPusherBack,
//                0.5);
        return PlantTasks.setInstant(plantPusher,
                RobotConfig.Shooter.targetPusherBack);
    }

    /**
     * Move the pusher servo to the "front" (extended) position.
     */
    public Task instantSetPusherFront() {
//        return PlantTasks.holdFor(plantPusher,
//                RobotConfig.Shooter.targetPusherFront,
//                0.5);
        return PlantTasks.setInstant(plantPusher,
                RobotConfig.Shooter.targetPusherFront);
    }

    /**
     * Start the transfer (indexer) in the requested direction.
     */
    public Task instantStartTransfer(TransferDirection direction) {
        switch (direction) {
            case FORWARD:
                return PlantTasks.setInstant(plantTransfer, 1);
            case BACKWARD:
                return PlantTasks.setInstant(plantTransfer, -1);
        }

        throw new IllegalArgumentException("Unknown direction provided!!!");
    }

    /**
     * Stop the transfer (indexer).
     */
    public Task instantStopTransfer() {
        return PlantTasks.setInstant(plantTransfer, 0);
    }

    /**
     * Emergency stop for all shooter-related actuators.
     */
    public void stop() {
        plantPusher.stop();
        plantShooter.stop();
        plantTransfer.stop();
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
                .addData(p + ".isShooterOn", isShooterOn);

        if (plantShooter != null) {
            plantShooter.debugDump(dbg, p + ".plantShooter");
        }
        if (plantTransfer != null) {
            plantTransfer.debugDump(dbg, p + ".plantTransfer");
        }
        if (plantPusher != null) {
            plantPusher.debugDump(dbg, p + ".plantPusher");
        }
    }

}
