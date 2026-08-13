package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import edu.ftcphoenix.fw.actuation.Plant;

/**
 * Advanced FTC-boundary factories for narrow motor-controller configuration capabilities.
 *
 * <p>This is not a Plant builder or a motor command facade. Ordinary mechanisms continue to
 * construct their one actuation path through {@link FtcActuators#plant(HardwareMap)}. A dedicated
 * mechanism-owned tuning workflow may use this class to derive a configuration handle from that
 * same Plant while the Plant remains the sole owner of target and hardware actuation.</p>
 */
public final class FtcMotorControllers {

    private static final String FACTORY_OPERATION =
            "FtcMotorControllers.velocityPidf(...)";

    private FtcMotorControllers() {
        // utility class
    }

    /**
     * Capture the device-managed velocity PIDF configuration for one FTC-backed Plant.
     *
     * <p>The Plant must be a single-motor device-managed velocity Plant built by
     * {@link FtcActuators}. The factory derives the already-selected motor from private FTC-boundary
     * metadata; there is no second motor name that could silently select another controller.
     * One Plant may create one handle; a second claim is rejected so controller configuration has
     * one session owner.
     * Construction reads and defensively copies the exact
     * {@link DcMotor.RunMode#RUN_USING_ENCODER} {@link PIDFCoefficients}, including its motor-control
     * algorithm. It performs no controller write and does not change target, power, velocity, run
     * mode, or direction. The captured value is the session restoration point used by
     * {@link FtcMotorVelocityPidf#restoreInitial()}.</p>
     *
     * @param plant single-motor FTC device-managed velocity Plant
     * @return configuration-only velocity PIDF handle bound to that Plant's motor
     * @throws IllegalArgumentException if {@code plant} is null or is not the required
     * single-motor FTC device-managed velocity shape
     * @throws IllegalStateException if the initial PIDF configuration cannot be read or is not a
     * valid restorable controller tuple, or this Plant already supplied a handle; no controller
     * setting is changed
     */
    public static FtcMotorVelocityPidf velocityPidf(Plant plant) {
        if (plant == null) {
            throw new IllegalArgumentException(
                    FACTORY_OPERATION + " requires a Plant");
        }
        if (!(plant instanceof FtcDeviceManagedVelocityPlant)) {
            throw new IllegalArgumentException(
                    FACTORY_OPERATION + " requires a single-motor FTC device-managed velocity "
                            + "Plant built by FtcActuators.plant(...).motor(...).velocity()."
                            + "deviceManagedWithDefaults() or deviceManaged(). Multi-motor, "
                            + "regulated, position, power, and hardware-neutral Plants are not "
                            + "eligible.");
        }

        FtcDeviceManagedVelocityPlant bound = (FtcDeviceManagedVelocityPlant) plant;
        bound.claimVelocityPidf();
        DcMotorEx motor = bound.motor();
        String motorName = bound.motorName();

        try {
            PIDFCoefficients initial = readValidCoefficients(
                    motor,
                    motorName,
                    FACTORY_OPERATION + " initial readback");
            return new FtcMotorVelocityPidf(
                    motor,
                    motorName,
                    initial,
                    bound.targetRange());
        } catch (RuntimeException failure) {
            throw new IllegalStateException(
                    "Failed to create an FTC velocity PIDF handle for motor '" + motorName
                            + "': could not read a valid RUN_USING_ENCODER configuration. No "
                            + "controller configuration was changed.",
                    failure);
        }
    }

    static PIDFCoefficients readValidCoefficients(DcMotorEx motor,
                                                  String motorName,
                                                  String operation) {
        PIDFCoefficients readback = motor.getPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER);
        if (readback == null) {
            throw new IllegalStateException(operation + " for motor '" + motorName
                    + "' returned null PIDFCoefficients");
        }
        if (readback.algorithm == null) {
            throw new IllegalStateException(operation + " for motor '" + motorName
                    + "' returned a null motor-control algorithm");
        }
        double[] checked = FtcControllerConfigurationValidation.requireControllerPidf(
                readback.p,
                readback.i,
                readback.d,
                readback.f,
                operation + " for motor '" + motorName + "'");
        return new PIDFCoefficients(
                checked[0],
                checked[1],
                checked[2],
                checked[3],
                readback.algorithm);
    }

}
