package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Objects;

/** Immutable exact readback of one FTC motor-controller PIDF tuple and algorithm. */
public final class FtcMotorPidfConfiguration {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    private final MotorControlAlgorithm algorithm;

    FtcMotorPidfConfiguration(PIDFCoefficients coefficients) {
        Objects.requireNonNull(coefficients, "coefficients");
        kP = coefficients.p;
        kI = coefficients.i;
        kD = coefficients.d;
        kF = coefficients.f;
        algorithm = Objects.requireNonNull(coefficients.algorithm, "algorithm");
    }

    /** Proportional coefficient in FTC controller units. */
    public double getKP() { return kP; }

    /** Integral coefficient in FTC controller units. */
    public double getKI() { return kI; }

    /** Derivative coefficient in FTC controller units. */
    public double getKD() { return kD; }

    /** Feed-forward coefficient in FTC controller units. */
    public double getKF() { return kF; }

    /** Exact FTC motor-control algorithm reported with this tuple. */
    public MotorControlAlgorithm algorithm() { return algorithm; }

    PIDFCoefficients toSdkCoefficients() {
        return new PIDFCoefficients(kP, kI, kD, kF, algorithm);
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof FtcMotorPidfConfiguration)) return false;
        FtcMotorPidfConfiguration that = (FtcMotorPidfConfiguration) other;
        return sameDouble(kP, that.kP)
                && sameDouble(kI, that.kI)
                && sameDouble(kD, that.kD)
                && sameDouble(kF, that.kF)
                && algorithm == that.algorithm;
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                Double.doubleToLongBits(kP),
                Double.doubleToLongBits(kI),
                Double.doubleToLongBits(kD),
                Double.doubleToLongBits(kF),
                algorithm);
    }

    @Override
    public String toString() {
        return "FtcMotorPidfConfiguration{kP=" + kP + ", kI=" + kI + ", kD=" + kD
                + ", kF=" + kF + ", algorithm=" + algorithm + '}';
    }

    private static boolean sameDouble(double first, double second) {
        return Double.doubleToLongBits(first) == Double.doubleToLongBits(second);
    }
}
