package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.util.InterpolatingTable1D;
import edu.ftcphoenix.fw.util.MathUtil;

public class Shooter {

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
                    103.8, 1650,
                    112.4, 1700,
                    123.5, 1750,
                    125.2, 1750,
                    132.5, 1750,
                    137.4, 1800,
                    141.4, 1850,
                    148.6, 1850
            );


    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, Gamepads gamepads) {
        plantPusher = Actuators.plant(hardwareMap)
                .servo(RobotConfig.Shooter.nameServoPusher,
                        RobotConfig.Shooter.invertServoPusher)
                .position()
                .build();

        plantTransfer = Actuators.plant(hardwareMap)
                .crServoPair(RobotConfig.Shooter.nameCrServoTransferLeft,
                        RobotConfig.Shooter.invertServoTransferLeft,
                        RobotConfig.Shooter.nameCrServoTransferRight,
                        RobotConfig.Shooter.invertServoTransferRight)
                .power()
                .build();

        plantShooter = Actuators.plant(hardwareMap)
                .motorPair(RobotConfig.Shooter.nameMotorShooterLeft,
                        RobotConfig.Shooter.invertMotorShooterLeft,
                        RobotConfig.Shooter.nameMotorShooterRight,
                        RobotConfig.Shooter.invertMotorShooterRight)
                .velocity(50)
                .build();

        isShooterOn = false;
        velocity = RobotConfig.Shooter.velocityMin;
    }

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

    public double getVelocity() {
        return velocity;
    }

    public Task instantStartShooter() {
        isShooterOn = true;
        return PlantTasks.setInstant(plantShooter, velocity);
    }

    public Task instantStopShooter() {
        isShooterOn = false;
        return PlantTasks.setInstant(plantShooter, 0);
    }

    public Task instantSetPusherBack() {
//        return PlantTasks.holdFor(plantPusher,
//                RobotConfig.Shooter.targetPusherBack,
//                0.5);
        return PlantTasks.setInstant(plantPusher,
                RobotConfig.Shooter.targetPusherBack);
    }

    public Task instantSetPusherFront() {
//        return PlantTasks.holdFor(plantPusher,
//                RobotConfig.Shooter.targetPusherFront,
//                0.5);
        return PlantTasks.setInstant(plantPusher,
                RobotConfig.Shooter.targetPusherFront);
    }

    public Task instantStartTransfer(TransferDirection direction) {
        switch (direction) {
            case FORWARD:
                return PlantTasks.setInstant(plantTransfer, 1);
            case BACKWARD:
                return PlantTasks.setInstant(plantTransfer, -1);
        }

        throw new IllegalArgumentException("Unknown direction provided!!!");
    }

    public Task instantStopTransfer() {
        return PlantTasks.setInstant(plantTransfer, 0);
    }
}
