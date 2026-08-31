package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.List;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.PositionPlantTuning;
import edu.ftcsushi.fw.actuation.PositionPlantTunings;

/** Advanced FTC-boundary factories for completed-Plant-derived controller tuning capabilities. */
public final class FtcMotorControllers {

    private FtcMotorControllers() {
        // utility class
    }

    /**
     * Permanently claim the device-managed velocity controller group owned by {@code plant}.
     *
     * <p>Single and grouped FTC motor Plants built by {@link FtcActuators} are eligible. Every
     * ordered member's exact RUN_USING_ENCODER tuple and algorithm is captured before the handle is
     * returned. Claim before the Plant's first {@code update(clock)} or {@code stop()}. This
     * operation performs no controller or actuator write. A second claim, including one after a
     * failed initial read, is rejected so configuration has one session owner.</p>
     */
    public static FtcMotorVelocityControl velocityControl(Plant plant) {
        if (plant == null) {
            throw new IllegalArgumentException(
                    "FtcMotorControllers.velocityControl(...) requires a Plant");
        }
        if (!(plant instanceof FtcDeviceManagedVelocityPlant)) {
            throw new IllegalArgumentException(
                    "FtcMotorControllers.velocityControl(...) requires an FTC device-managed "
                            + "motor velocity Plant built by FtcActuators; regulated, position, "
                            + "power, servo, and hardware-neutral Plants are not eligible");
        }

        FtcDeviceManagedVelocityPlant bound = (FtcDeviceManagedVelocityPlant) plant;
        bound.claimController();
        FtcDeviceManagedVelocityBinding binding = bound.binding();
        List<FtcMotorPidfConfiguration> initial = new ArrayList<>(binding.size());
        try {
            for (int index = 0; index < binding.size(); index++) {
                initial.add(readConfiguration(
                        binding.motors().get(index),
                        binding.names().get(index),
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        "FtcMotorControllers.velocityControl(...) initial readback"));
            }
            return new FtcMotorVelocityControl(bound, initial);
        } catch (RuntimeException failure) {
            throw new IllegalStateException("Failed to create an FTC velocity-control handle: "
                    + "could not capture every ordered RUN_USING_ENCODER configuration. No "
                    + "controller setting was changed, and this Plant claim remains consumed.",
                    failure);
        }
    }

    /**
     * Permanently claim the device-managed position cascade owned by {@code plant}.
     *
     * <p>Only a single-motor FTC device-managed position Plant built by {@link FtcActuators} is
     * eligible. The exact RUN_TO_POSITION and RUN_USING_ENCODER tuples and algorithms are captured
     * without a write. Grouped position Plants are intentionally rejected because the framework
     * has no universal safe reconfiguration state for a coupled position mechanism. Claim before
     * normal referenced realization begins; still-unreferenced calibration-search heartbeats remain
     * eligible under the same-Plant preparation lifecycle.</p>
     */
    public static FtcMotorPositionControl positionControl(PositionPlant plant) {
        if (plant == null) {
            throw new IllegalArgumentException(
                    "FtcMotorControllers.positionControl(...) requires a PositionPlant");
        }
        if (!(plant instanceof FtcDeviceManagedPositionPlant)) {
            throw new IllegalArgumentException(
                    "FtcMotorControllers.positionControl(...) requires a single-motor FTC "
                            + "device-managed position Plant built by FtcActuators; grouped, "
                            + "regulated, servo, and hardware-neutral PositionPlants are not eligible");
        }

        FtcDeviceManagedPositionPlant bound = (FtcDeviceManagedPositionPlant) plant;
        bound.claimController();
        final PositionPlantTuning positionPreparation;
        try {
            positionPreparation = PositionPlantTunings.claim(bound.tuningDelegate());
        } catch (RuntimeException failure) {
            throw new IllegalStateException("Failed to create an FTC position-control handle for "
                    + "motor '" + bound.motorName() + "': the same completed Plant's tuning "
                    + "preparation capability could not be claimed. Claim the controller before "
                    + "the first normal referenced Plant realization; this FTC claim remains "
                    + "consumed.", failure);
        }
        try {
            FtcMotorPositionControl.Configuration initial =
                    FtcMotorPositionControl.readConfiguration(
                            bound.motor(),
                            bound.motorName(),
                            "FtcMotorControllers.positionControl(...) initial readback");
            return new FtcMotorPositionControl(
                    bound,
                    bound.motor(),
                    bound.motorName(),
                    bound.configuredTargetRange(),
                    positionPreparation,
                    initial);
        } catch (RuntimeException failure) {
            throw new IllegalStateException("Failed to create an FTC position-control handle for "
                    + "motor '" + bound.motorName() + "': could not capture both controller mode "
                    + "configurations. No controller setting was changed, and this Plant claim "
                    + "remains consumed.", failure);
        }
    }

    static FtcMotorPidfConfiguration readConfiguration(DcMotorEx motor,
                                                        String motorName,
                                                        DcMotor.RunMode mode,
                                                        String operation) {
        PIDFCoefficients readback = motor.getPIDFCoefficients(mode);
        if (readback == null) {
            throw new IllegalStateException(operation + " for motor '" + motorName
                    + "' and mode " + mode + " returned null PIDFCoefficients");
        }
        if (readback.algorithm == null) {
            throw new IllegalStateException(operation + " for motor '" + motorName
                    + "' and mode " + mode + " returned a null motor-control algorithm");
        }
        double[] checked = FtcControllerConfigurationValidation.requireControllerPidf(
                readback.p,
                readback.i,
                readback.d,
                readback.f,
                operation + " for motor '" + motorName + "' and mode " + mode);
        return new FtcMotorPidfConfiguration(new PIDFCoefficients(
                checked[0], checked[1], checked[2], checked[3], readback.algorithm));
    }
}
