package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Configuration-only tuning capability for one completed FTC device-managed velocity Plant.
 *
 * <p>The handle exposes the exact ordered motor group already owned by the Plant. It never accepts
 * another hardware name and exposes no target, mode, power, or velocity command. One shared
 * {@link Candidate} is applied to every member in declaration order. A single-motor handle may be
 * applied while running. A grouped handle requires cold-zero evidence from the Plant's successful
 * update in the supplied clock cycle before any controller write.</p>
 *
 * <p>An FTC setter or readback failure is non-transactional and permanently latches
 * {@link #isTerminallyUncertain()}. Further applies are rejected. {@link #restoreInitial()} remains
 * available and attempts every member, including after the Plant stops, but successful numeric
 * restoration does not rearm the tuning session or undo physical motion. Stopping the Plant
 * permanently disables evidence and further candidate applies.</p>
 */
public final class FtcMotorVelocityControl {

    /** Immutable shared velocity-PIDF candidate. The FTC setter selects the PIDF algorithm. */
    public static final class Candidate {
        private final double kP;
        private final double kI;
        private final double kD;
        private final double kF;

        private Candidate(double kP, double kI, double kD, double kF) {
            double[] checked = FtcControllerConfigurationValidation.requireControllerPidf(
                    kP, kI, kD, kF, "FtcMotorVelocityControl.Candidate.of(...)");
            this.kP = checked[0];
            this.kI = checked[1];
            this.kD = checked[2];
            this.kF = checked[3];
        }

        /** Validate and create one complete shared velocity-PIDF candidate. */
        public static Candidate of(double kP, double kI, double kD, double kF) {
            return new Candidate(kP, kI, kD, kF);
        }

        public double getKP() { return kP; }
        public double getKI() { return kI; }
        public double getKD() { return kD; }
        public double getKF() { return kF; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Candidate)) return false;
            Candidate that = (Candidate) other;
            return sameDouble(kP, that.kP) && sameDouble(kI, that.kI)
                    && sameDouble(kD, that.kD) && sameDouble(kF, that.kF);
        }

        @Override
        public int hashCode() {
            return Objects.hash(Double.doubleToLongBits(kP), Double.doubleToLongBits(kI),
                    Double.doubleToLongBits(kD), Double.doubleToLongBits(kF));
        }

        @Override
        public String toString() {
            return "Candidate{kP=" + kP + ", kI=" + kI + ", kD=" + kD + ", kF=" + kF + '}';
        }
    }

    /** Exact configuration for one ordered group member. */
    public static final class MemberConfiguration {
        private final String motorName;
        private final FtcMotorPidfConfiguration pidf;

        private MemberConfiguration(String motorName, FtcMotorPidfConfiguration pidf) {
            this.motorName = Objects.requireNonNull(motorName, "motorName");
            this.pidf = Objects.requireNonNull(pidf, "pidf");
        }

        public String motorName() { return motorName; }
        public FtcMotorPidfConfiguration pidf() { return pidf; }

        @Override
        public String toString() {
            return "MemberConfiguration{motorName='" + motorName + "', pidf=" + pidf + '}';
        }
    }

    /** Same-cycle per-member evidence produced from the Plant's own command cache and sensor. */
    public static final class MemberEvidence {
        private final MemberConfiguration initialConfiguration;
        private final MemberConfiguration readbackConfiguration;
        private final double nativeCommandedTarget;
        private final double nativeMeasurement;
        private final double nativeError;
        private final double nativeTolerance;

        private MemberEvidence(MemberConfiguration initialConfiguration,
                               MemberConfiguration readbackConfiguration,
                               double nativeCommandedTarget,
                               double nativeMeasurement,
                               double nativeTolerance) {
            this.initialConfiguration = initialConfiguration;
            this.readbackConfiguration = readbackConfiguration;
            this.nativeCommandedTarget = nativeCommandedTarget;
            this.nativeMeasurement = nativeMeasurement;
            nativeError = nativeCommandedTarget - nativeMeasurement;
            this.nativeTolerance = nativeTolerance;
        }

        public String motorName() { return readbackConfiguration.motorName(); }
        public MemberConfiguration initialConfiguration() { return initialConfiguration; }
        public MemberConfiguration readbackConfiguration() { return readbackConfiguration; }
        public double nativeCommandedTarget() { return nativeCommandedTarget; }
        public double nativeMeasurement() { return nativeMeasurement; }
        public double nativeError() { return nativeError; }
        public double nativeTolerance() { return nativeTolerance; }

        /** Whether the finite native error is inside this member's mapped Plant tolerance. */
        public boolean withinMappedPlantTolerance() {
            return Double.isFinite(nativeError) && Double.isFinite(nativeTolerance)
                    && Math.abs(nativeError) <= nativeTolerance;
        }
    }

    private final FtcDeviceManagedVelocityPlant plant;
    private final FtcDeviceManagedVelocityBinding binding;
    private final List<MemberConfiguration> initial;
    private final Candidate constructionCandidate;
    private List<MemberConfiguration> readback;
    private boolean terminallyUncertain;

    FtcMotorVelocityControl(FtcDeviceManagedVelocityPlant plant,
                            List<FtcMotorPidfConfiguration> initialConfigurations) {
        this.plant = Objects.requireNonNull(plant, "plant");
        binding = plant.binding();
        if (initialConfigurations.size() != binding.size()) {
            throw new IllegalArgumentException("Initial velocity configurations must match members");
        }
        List<MemberConfiguration> captured = new ArrayList<>(binding.size());
        for (int index = 0; index < binding.size(); index++) {
            captured.add(new MemberConfiguration(
                    binding.names().get(index), initialConfigurations.get(index)));
        }
        initial = Collections.unmodifiableList(captured);
        double[] submitted = binding.constructionPidf();
        constructionCandidate = submitted == null
                ? null
                : Candidate.of(submitted[0], submitted[1], submitted[2], submitted[3]);
        readback = initial;
    }

    /** Immutable Plant-unit range configured on the completed Plant. */
    public ScalarRange plantTargetRange() { return plant.targetRange(); }

    /** Number of exact ordered FTC motors controlled by the Plant. */
    public int memberCount() { return binding.size(); }

    /** Exact construction-time session baseline for every member. */
    public synchronized List<MemberConfiguration> initialConfigurations() { return initial; }

    /** Latest complete successful readback for every member. */
    public synchronized List<MemberConfiguration> readbackConfigurations() { return readback; }

    /**
     * Return the one shared tuple explicitly submitted by the completed Plant's builder override.
     * Plain {@code deviceManaged()} construction returns empty rather than inventing a common
     * candidate from possibly divergent per-member controller readbacks.
     */
    public Optional<Candidate> constructionCandidate() {
        return Optional.ofNullable(constructionCandidate);
    }

    /**
     * Return per-member evidence after the Plant updated successfully in this exact clock cycle.
     * No second raw SDK poll occurs: these are the memoized sources consumed by the Plant update.
     */
    public synchronized List<MemberEvidence> evidence(LoopClock clock) {
        plant.requireEvidenceAfterUpdate(clock, "FtcMotorVelocityControl.evidence(clock)");
        return captureEvidence(clock);
    }

    /**
     * Apply one complete shared tuple in declaration order and read every member back.
     *
     * <p>For a group, the supplied clock must identify a successful Plant update with an applied
     * zero target, an exact zero native command for each child, and every child measurement within
     * its mapped Plant tolerance. A failed cold check performs no controller write. One motor does
     * not require cold-zero and therefore supports a hot gain change.</p>
     */
    public synchronized void apply(Candidate candidate, LoopClock clock) {
        Candidate checked = Objects.requireNonNull(candidate, "candidate");
        Objects.requireNonNull(clock, "clock");
        plant.requireControllerConfigurationActive(
                "FtcMotorVelocityControl.apply(candidate, clock)");
        requireApplyAvailable();
        if (binding.size() > 1) {
            requireGroupedColdZero(clock);
        }

        List<MemberConfiguration> next = new ArrayList<>(binding.size());
        boolean operationBegan = false;
        try {
            for (int index = 0; index < binding.size(); index++) {
                operationBegan = true;
                DcMotorEx motor = binding.motors().get(index);
                motor.setVelocityPIDFCoefficients(
                        checked.kP, checked.kI, checked.kD, checked.kF);
                FtcMotorPidfConfiguration accepted = FtcMotorControllers.readConfiguration(
                        motor,
                        binding.names().get(index),
                        com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                        "FtcMotorVelocityControl.apply(...) readback");
                if (accepted.algorithm() != MotorControlAlgorithm.PIDF) {
                    throw new IllegalStateException("Velocity controller readback for motor '"
                            + binding.names().get(index) + "' reported algorithm "
                            + accepted.algorithm() + " instead of PIDF");
                }
                next.add(new MemberConfiguration(binding.names().get(index), accepted));
            }
            readback = Collections.unmodifiableList(next);
        } catch (RuntimeException failure) {
            if (operationBegan) terminallyUncertain = true;
            throw uncertainFailure("apply", failure);
        }
    }

    /**
     * Best-effort restore every member's exact captured tuple and algorithm, then verify all.
     * Every member is attempted even if an earlier member fails.
     */
    public synchronized void restoreInitial() {
        Runnable[] actions = new Runnable[binding.size()];
        for (int index = 0; index < binding.size(); index++) {
            final int memberIndex = index;
            actions[index] = () -> restoreMember(memberIndex);
        }
        try {
            CleanupActions.attemptAll(actions);
            readback = initial;
        } catch (RuntimeException failure) {
            terminallyUncertain = true;
            throw uncertainFailure("restoreInitial", failure);
        }
    }

    /** Whether an FTC operation failure made controller state terminally uncertain. */
    public synchronized boolean isTerminallyUncertain() { return terminallyUncertain; }

    private void requireGroupedColdZero(LoopClock clock) {
        plant.requireEvidenceAfterUpdate(clock,
                "FtcMotorVelocityControl.apply(candidate, clock)");
        if (!Double.isFinite(plant.getAppliedTarget()) || plant.getAppliedTarget() != 0.0) {
            throw new IllegalStateException("Grouped FTC velocity controller changes require the "
                    + "Plant applied target to be zero in the supplied cycle");
        }
        List<MemberEvidence> evidence = captureEvidence(clock);
        for (MemberEvidence member : evidence) {
            if (!Double.isFinite(member.nativeCommandedTarget)
                    || member.nativeCommandedTarget != 0.0) {
                throw new IllegalStateException("Grouped FTC velocity controller changes require "
                        + "motor '" + member.motorName()
                        + "' to have an exact zero native commanded target, got "
                        + member.nativeCommandedTarget);
            }
            if (!member.withinMappedPlantTolerance()) {
                throw new IllegalStateException("Grouped FTC velocity controller changes require "
                        + "motor '" + member.motorName() + "' to be within mapped native zero "
                        + "tolerance " + member.nativeTolerance + ", got measurement "
                        + member.nativeMeasurement);
            }
        }
    }

    private List<MemberEvidence> captureEvidence(LoopClock clock) {
        List<MemberEvidence> result = new ArrayList<>(binding.size());
        for (int index = 0; index < binding.size(); index++) {
            result.add(new MemberEvidence(
                    initial.get(index),
                    readback.get(index),
                    binding.nativeCommandedTarget(index),
                    binding.nativeMeasurement(index, clock),
                    binding.nativeTolerance(index)));
        }
        return Collections.unmodifiableList(result);
    }

    private void restoreMember(int index) {
        DcMotorEx motor = binding.motors().get(index);
        FtcMotorPidfConfiguration expected = initial.get(index).pidf();
        motor.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                expected.toSdkCoefficients());
        FtcMotorPidfConfiguration actual = FtcMotorControllers.readConfiguration(
                motor,
                binding.names().get(index),
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                "FtcMotorVelocityControl.restoreInitial() readback");
        if (!expected.equals(actual)) {
            throw new IllegalStateException("Restore readback for velocity motor '"
                    + binding.names().get(index) + "' did not exactly match the captured tuple "
                    + "and algorithm; expected " + expected + ", got " + actual);
        }
    }

    private void requireApplyAvailable() {
        if (terminallyUncertain) {
            throw new IllegalStateException("This FTC velocity controller handle is terminally "
                    + "uncertain after an earlier FTC operation failure; stop the Plant and call "
                    + "restoreInitial() before ending the session");
        }
    }

    private IllegalStateException uncertainFailure(String operation, RuntimeException cause) {
        return new IllegalStateException("FtcMotorVelocityControl." + operation
                + " failed after an FTC controller operation began. Controller state may be "
                + "partially changed; stop the Plant and best-effort restore every member.", cause);
    }

    private static boolean sameDouble(double first, double second) {
        return Double.doubleToLongBits(first) == Double.doubleToLongBits(second);
    }
}
