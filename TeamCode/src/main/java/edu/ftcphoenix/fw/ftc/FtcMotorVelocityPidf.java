package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.ScalarRange;

/**
 * Configuration-only access to one FTC motor's device-managed velocity PIDF gains.
 *
 * <p>Obtain this advanced tuning capability from
 * {@link FtcMotorControllers#velocityPidf(edu.ftcphoenix.fw.actuation.Plant)}.
 * It deliberately exposes no motor target, power, velocity, mode, or direction command. A
 * mechanism-owned Plant remains the sole actuation path while a dedicated tuning owner applies
 * complete controller-gain candidates through this handle.</p>
 *
 * <p>The four getters report the most recent complete FTC controller readback: initially the
 * factory's construction-time readback, then the readback after the last successful
 * {@link #setGains(double, double, double, double)} or {@link #restoreInitial()} operation. They
 * may therefore differ slightly from submitted values because the controller owns coefficient
 * representation and quantization.</p>
 */
public final class FtcMotorVelocityPidf {

    private static final String SET_OPERATION =
            "FtcMotorVelocityPidf.setGains(...)";
    private static final String RESTORE_OPERATION =
            "FtcMotorVelocityPidf.restoreInitial()";

    private final DcMotorEx motor;
    private final String motorName;
    private final PIDFCoefficients initial;
    private final ScalarRange plantTargetRange;
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    FtcMotorVelocityPidf(DcMotorEx motor,
                         String motorName,
                         PIDFCoefficients initial,
                         ScalarRange plantTargetRange) {
        this.motor = motor;
        this.motorName = motorName;
        this.initial = new PIDFCoefficients(initial);
        this.plantTargetRange = Objects.requireNonNull(
                plantTargetRange,
                "plantTargetRange");
        cache(initial);
    }

    /**
     * Return the target range of the exact Plant from which this handle was derived.
     *
     * <p>The range is immutable and uses Plant units. Tuning workflows use it to reject a test
     * range that could ask the bound Plant for an illegal target.</p>
     */
    public ScalarRange plantTargetRange() {
        return plantTargetRange;
    }

    /**
     * Apply one complete FTC device-managed velocity PIDF tuple, then read it back.
     *
     * <p>All four values are validated before the SDK velocity-PIDF setter is called. That setter
     * selects the FTC {@code PIDF} motor-control algorithm. Each coefficient must be finite and
     * inside the inclusive, symmetric FTC SDK 11.1 REV public-conversion domain
     * {@code [-Integer.MAX_VALUE / 65536.0, +Integer.MAX_VALUE / 65536.0]}. Negative values are
     * representable; that structural domain is not a tuning recommendation. If the SDK setter or
     * subsequent readback fails or reports another algorithm, the prior successful getter values
     * remain cached, but the controller's actual configuration may be uncertain.</p>
     *
     * @param kP proportional coefficient in FTC controller units
     * @param kI integral coefficient in FTC controller units
     * @param kD derivative coefficient in FTC controller units
     * @param kF feed-forward coefficient in FTC controller units
     * @throws IllegalArgumentException if any coefficient is non-finite or outside the supported
     * controller conversion domain; no SDK setter is called
     * @throws IllegalStateException if the SDK apply or readback fails; controller state may be
     * uncertain
     */
    public synchronized void setGains(double kP, double kI, double kD, double kF) {
        double[] checked = FtcControllerConfigurationValidation.requireControllerPidf(
                kP,
                kI,
                kD,
                kF,
                SET_OPERATION + " for motor '" + motorName + "'");

        try {
            motor.setVelocityPIDFCoefficients(
                    checked[0],
                    checked[1],
                    checked[2],
                    checked[3]);
            PIDFCoefficients readback = FtcMotorControllers.readValidCoefficients(
                    motor,
                    motorName,
                    SET_OPERATION + " readback");
            if (readback.algorithm != MotorControlAlgorithm.PIDF) {
                throw new IllegalStateException(
                        SET_OPERATION + " readback for motor '" + motorName
                                + "' reported algorithm " + readback.algorithm
                                + " instead of PIDF");
            }
            cache(readback);
        } catch (RuntimeException failure) {
            throw uncertainFailure(SET_OPERATION, failure);
        }
    }

    /**
     * Return the proportional coefficient from the latest successful complete readback.
     *
     * @return proportional coefficient in FTC controller units
     */
    public synchronized double getKP() {
        return kP;
    }

    /**
     * Return the integral coefficient from the latest successful complete readback.
     *
     * @return integral coefficient in FTC controller units
     */
    public synchronized double getKI() {
        return kI;
    }

    /**
     * Return the derivative coefficient from the latest successful complete readback.
     *
     * @return derivative coefficient in FTC controller units
     */
    public synchronized double getKD() {
        return kD;
    }

    /**
     * Return the feed-forward coefficient from the latest successful complete readback.
     *
     * @return feed-forward coefficient in FTC controller units
     */
    public synchronized double getKF() {
        return kF;
    }

    /**
     * Restore the exact construction-time {@code RUN_USING_ENCODER} coefficient tuple and motor
     * control algorithm, then read back the resulting configuration.
     *
     * <p>If the SDK setter or subsequent readback fails, the prior successful getter values remain
     * cached, but the controller's actual configuration may be uncertain. The operation is not a
     * rollback guarantee for a failed earlier transport operation.</p>
     *
     * @throws IllegalStateException if the SDK restore or readback fails; controller state may be
     * uncertain
     */
    public synchronized void restoreInitial() {
        try {
            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(initial));
            PIDFCoefficients readback = FtcMotorControllers.readValidCoefficients(
                    motor,
                    motorName,
                    RESTORE_OPERATION + " readback");
            if (!sameConfiguration(initial, readback)) {
                throw new IllegalStateException(
                        RESTORE_OPERATION + " readback for motor '" + motorName
                                + "' did not match the captured initial PIDF tuple and "
                                + "motor-control algorithm");
            }
            cache(readback);
        } catch (RuntimeException failure) {
            throw uncertainFailure(RESTORE_OPERATION, failure);
        }
    }

    private static boolean sameConfiguration(PIDFCoefficients expected,
                                             PIDFCoefficients actual) {
        return expected.p == actual.p
                && expected.i == actual.i
                && expected.d == actual.d
                && expected.f == actual.f
                && expected.algorithm == actual.algorithm;
    }

    private void cache(PIDFCoefficients coefficients) {
        kP = coefficients.p;
        kI = coefficients.i;
        kD = coefficients.d;
        kF = coefficients.f;
    }

    private IllegalStateException uncertainFailure(String operation,
                                                    RuntimeException failure) {
        return new IllegalStateException(
                operation + " failed for motor '" + motorName
                        + "' after an FTC controller operation began. The actual velocity "
                        + "PIDF configuration may be uncertain; stop the mechanism and "
                        + "restore the captured configuration or restart before continuing.",
                failure);
    }
}
