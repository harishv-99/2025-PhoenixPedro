package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.actuation.PositionPlantTuning;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Configuration-only tuning capability for one completed single-motor FTC device-managed position
 * Plant.
 *
 * <p>The tunable cascade is exactly the FTC outer {@link DcMotor.RunMode#RUN_TO_POSITION}
 * proportional coefficient plus the inner {@link DcMotor.RunMode#RUN_USING_ENCODER} velocity PIDF
 * tuple. Plant bounds, reference policy, tolerances, and maximum output power remain fixed parts of
 * the completed Plant and are not duplicated here.</p>
 *
 * <p>FTC writes are not transactional. Any setter or readback failure permanently latches
 * {@link #isTerminallyUncertain()}, and further applies are rejected. Restoration still attempts
 * both captured mode configurations and verifies their exact tuples and algorithms. Stopping the
 * Plant disables further candidate applies while leaving that restoration path available.</p>
 */
public final class FtcMotorPositionControl {

    /** Immutable complete candidate for the supported FTC position cascade. */
    public static final class Candidate {
        private final double outerPositionKP;
        private final double innerVelocityKP;
        private final double innerVelocityKI;
        private final double innerVelocityKD;
        private final double innerVelocityKF;

        private Candidate(double outerPositionKP,
                          double innerVelocityKP,
                          double innerVelocityKI,
                          double innerVelocityKD,
                          double innerVelocityKF) {
            this.outerPositionKP =
                    FtcControllerConfigurationValidation.requireControllerCoefficient(
                            outerPositionKP,
                            "FtcMotorPositionControl.Candidate.of(...)",
                            "outerPositionKP");
            double[] inner = FtcControllerConfigurationValidation.requireControllerPidf(
                    innerVelocityKP,
                    innerVelocityKI,
                    innerVelocityKD,
                    innerVelocityKF,
                    "FtcMotorPositionControl.Candidate.of(...) inner velocity PIDF");
            this.innerVelocityKP = inner[0];
            this.innerVelocityKI = inner[1];
            this.innerVelocityKD = inner[2];
            this.innerVelocityKF = inner[3];
        }

        /** Validate and create one complete FTC position-cascade candidate. */
        public static Candidate of(double outerPositionKP,
                                   double innerVelocityKP,
                                   double innerVelocityKI,
                                   double innerVelocityKD,
                                   double innerVelocityKF) {
            return new Candidate(
                    outerPositionKP,
                    innerVelocityKP,
                    innerVelocityKI,
                    innerVelocityKD,
                    innerVelocityKF);
        }

        public double outerPositionKP() { return outerPositionKP; }
        public double innerVelocityKP() { return innerVelocityKP; }
        public double innerVelocityKI() { return innerVelocityKI; }
        public double innerVelocityKD() { return innerVelocityKD; }
        public double innerVelocityKF() { return innerVelocityKF; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Candidate)) return false;
            Candidate that = (Candidate) other;
            return sameDouble(outerPositionKP, that.outerPositionKP)
                    && sameDouble(innerVelocityKP, that.innerVelocityKP)
                    && sameDouble(innerVelocityKI, that.innerVelocityKI)
                    && sameDouble(innerVelocityKD, that.innerVelocityKD)
                    && sameDouble(innerVelocityKF, that.innerVelocityKF);
        }

        @Override
        public int hashCode() {
            return Objects.hash(
                    Double.doubleToLongBits(outerPositionKP),
                    Double.doubleToLongBits(innerVelocityKP),
                    Double.doubleToLongBits(innerVelocityKI),
                    Double.doubleToLongBits(innerVelocityKD),
                    Double.doubleToLongBits(innerVelocityKF));
        }

        @Override
        public String toString() {
            return "Candidate{outerPositionKP=" + outerPositionKP
                    + ", innerVelocityKP=" + innerVelocityKP
                    + ", innerVelocityKI=" + innerVelocityKI
                    + ", innerVelocityKD=" + innerVelocityKD
                    + ", innerVelocityKF=" + innerVelocityKF + '}';
        }
    }

    /** Exact readback of both FTC controller modes in the supported cascade. */
    public static final class Configuration {
        private final FtcMotorPidfConfiguration outerPosition;
        private final FtcMotorPidfConfiguration innerVelocity;

        private Configuration(FtcMotorPidfConfiguration outerPosition,
                              FtcMotorPidfConfiguration innerVelocity) {
            this.outerPosition = Objects.requireNonNull(outerPosition, "outerPosition");
            this.innerVelocity = Objects.requireNonNull(innerVelocity, "innerVelocity");
        }

        /** Exact RUN_TO_POSITION tuple and algorithm; only its kP is a tuning candidate field. */
        public FtcMotorPidfConfiguration outerPosition() { return outerPosition; }

        /** Exact RUN_USING_ENCODER inner velocity tuple and algorithm. */
        public FtcMotorPidfConfiguration innerVelocity() { return innerVelocity; }

        @Override
        public String toString() {
            return "Configuration{outerPosition=" + outerPosition
                    + ", innerVelocity=" + innerVelocity + '}';
        }
    }

    private final FtcDeviceManagedPositionPlant plant;
    private final DcMotorEx motor;
    private final String motorName;
    private final ScalarRange plantTargetRange;
    private final PositionPlantTuning positionPreparation;
    private final Configuration initial;
    private Configuration readback;
    private boolean terminallyUncertain;

    FtcMotorPositionControl(FtcDeviceManagedPositionPlant plant,
                            DcMotorEx motor,
                            String motorName,
                            ScalarRange plantTargetRange,
                            PositionPlantTuning positionPreparation,
                            Configuration initial) {
        this.plant = Objects.requireNonNull(plant, "plant");
        this.motor = Objects.requireNonNull(motor, "motor");
        this.motorName = Objects.requireNonNull(motorName, "motorName");
        this.plantTargetRange = Objects.requireNonNull(plantTargetRange, "plantTargetRange");
        this.positionPreparation = Objects.requireNonNull(
                positionPreparation, "positionPreparation");
        this.initial = Objects.requireNonNull(initial, "initial");
        readback = initial;
    }

    static Configuration configuration(FtcMotorPidfConfiguration outer,
                                       FtcMotorPidfConfiguration inner) {
        return new Configuration(outer, inner);
    }

    /** Configured Plant-unit physical target range, even before a required reference is established. */
    public ScalarRange plantTargetRange() { return plantTargetRange; }

    /** Configured name of the exact motor already owned by the completed Plant. */
    public String motorName() { return motorName; }

    /** Exact two-mode session restoration baseline. */
    public Configuration initialConfiguration() { return initial; }

    /** Latest complete successful two-mode readback. */
    public synchronized Configuration readbackConfiguration() { return readback; }

    /** Whether this completed Plant has one literal exact graph-owned command target. */
    public boolean hasExactCommandTarget() { return positionPreparation.hasExactCommandTarget(); }

    /** Whether the completed Plant's position coordinate is currently referenced. */
    public boolean isReferenced() { return positionPreparation.isReferenced(); }

    /** Cached human-readable coordinate-reference status from the completed Plant. */
    public String referenceStatus() { return positionPreparation.referenceStatus(); }

    /**
     * Stage one current-position hold without invoking normal Plant output. This is the one-shot
     * preparation used before the first tuning heartbeat for an already-referenced or
     * ASSUME_CURRENT Plant.
     */
    public double prepareHoldAtCurrent(LoopClock clock) {
        return positionPreparation.prepareHoldAtCurrent(Objects.requireNonNull(clock, "clock"));
    }

    /**
     * Sample this same Plant's current position before its normal heartbeat and stage a legal
     * recovery hold inside {@code allowedPhysicalRange}, without writing controller configuration
     * or invoking actuator output.
     */
    public PositionPlantTuning.RecoveryHold prepareRecoveryHoldWithin(
            ScalarRange allowedPhysicalRange,
            LoopClock clock) {
        return positionPreparation.prepareRecoveryHoldWithin(
                Objects.requireNonNull(allowedPhysicalRange, "allowedPhysicalRange"),
                Objects.requireNonNull(clock, "clock"));
    }

    /**
     * Apply the complete supported cascade, outer first and inner second, then read both modes.
     * The owning workflow must establish that the position Plant is settled/holding before calling
     * this method; the handle deliberately owns no target or motion policy.
     */
    public synchronized void apply(Candidate candidate) {
        Candidate checked = Objects.requireNonNull(candidate, "candidate");
        plant.requireControllerConfigurationActive("FtcMotorPositionControl.apply(candidate)");
        if (terminallyUncertain) {
            throw new IllegalStateException("This FTC position controller handle is terminally "
                    + "uncertain after an earlier FTC operation failure; stop the Plant and call "
                    + "restoreInitial() before ending the session");
        }

        try {
            motor.setPositionPIDFCoefficients(checked.outerPositionKP);
            motor.setVelocityPIDFCoefficients(
                    checked.innerVelocityKP,
                    checked.innerVelocityKI,
                    checked.innerVelocityKD,
                    checked.innerVelocityKF);
            Configuration accepted = readConfiguration(
                    motor, motorName, "FtcMotorPositionControl.apply(...) readback");
            if (accepted.outerPosition.algorithm() != MotorControlAlgorithm.PIDF
                    || accepted.innerVelocity.algorithm() != MotorControlAlgorithm.PIDF) {
                throw new IllegalStateException("Position cascade readback for motor '" + motorName
                        + "' must report PIDF for both RUN_TO_POSITION and RUN_USING_ENCODER; got "
                        + accepted);
            }
            readback = accepted;
        } catch (RuntimeException failure) {
            terminallyUncertain = true;
            throw uncertainFailure("apply", failure);
        }
    }

    /** Best-effort restore and exactly verify both captured mode tuples and algorithms. */
    public synchronized void restoreInitial() {
        Runnable restoreOuter = () -> motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_TO_POSITION,
                initial.outerPosition.toSdkCoefficients());
        Runnable restoreInner = () -> motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                initial.innerVelocity.toSdkCoefficients());
        try {
            CleanupActions.attemptAll(restoreOuter, restoreInner);
            Configuration restored = readConfiguration(
                    motor, motorName, "FtcMotorPositionControl.restoreInitial() readback");
            if (!initial.outerPosition.equals(restored.outerPosition)
                    || !initial.innerVelocity.equals(restored.innerVelocity)) {
                throw new IllegalStateException("Position restore readback for motor '" + motorName
                        + "' did not exactly match both captured tuples and algorithms; expected "
                        + initial + ", got " + restored);
            }
            readback = initial;
        } catch (RuntimeException failure) {
            terminallyUncertain = true;
            throw uncertainFailure("restoreInitial", failure);
        }
    }

    /** Whether an FTC operation failure made controller state terminally uncertain. */
    public synchronized boolean isTerminallyUncertain() { return terminallyUncertain; }

    static Configuration readConfiguration(DcMotorEx motor,
                                           String motorName,
                                           String operation) {
        return configuration(
                FtcMotorControllers.readConfiguration(
                        motor, motorName, DcMotor.RunMode.RUN_TO_POSITION, operation),
                FtcMotorControllers.readConfiguration(
                        motor, motorName, DcMotor.RunMode.RUN_USING_ENCODER, operation));
    }

    private IllegalStateException uncertainFailure(String operation, RuntimeException cause) {
        return new IllegalStateException("FtcMotorPositionControl." + operation
                + " failed after an FTC controller operation began for motor '" + motorName
                + "'. The two-mode cascade may be partially changed; stop the Plant and "
                + "best-effort restore both captured configurations.", cause);
    }

    private static boolean sameDouble(double first, double second) {
        return Double.doubleToLongBits(first) == Double.doubleToLongBits(second);
    }
}
