package edu.ftcsushi.fw.integrations.panels;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.PositionPlantTuning;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.actuation.StandardControlTuning;
import edu.ftcsushi.fw.actuation.StandardControlTunings;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcMotorControllers;
import edu.ftcsushi.fw.ftc.FtcMotorPidfConfiguration;
import edu.ftcsushi.fw.ftc.FtcMotorPositionControl;
import edu.ftcsushi.fw.ftc.FtcMotorVelocityControl;

/** Bridges typed Plant-derived controller handles into the package-private workflow model. */
final class ControlTuningAdapters {

    private static final String FEEDBACK_KP = "feedback.kP";
    private static final String FEEDBACK_KI = "feedback.kI";
    private static final String FEEDBACK_KD = "feedback.kD";
    private static final String FEEDFORWARD_KS = "feedforward.kS";
    private static final String FEEDFORWARD_KV = "feedforward.kV";
    private static final String FEEDFORWARD_KA = "feedforward.kA";
    private static final String FEEDFORWARD_KG = "feedforward.kG";

    static ControlTuningModel.Session claimVelocity(Plant plant) {
        IllegalArgumentException standardUnsupported = null;
        StandardControlTuning standard;
        try {
            standard = StandardControlTunings.claimVelocity(plant);
        } catch (IllegalArgumentException unsupported) {
            standardUnsupported = unsupported;
            standard = null;
        }
        if (standard != null) {
            if (!standard.hasExactCommandTarget()) {
                throw new IllegalArgumentException(
                        "standard velocity tuning requires one literal exact command target");
            }
            return new StandardSession(standard);
        }

        try {
            return new FtcVelocitySession(FtcMotorControllers.velocityControl(plant));
        } catch (IllegalArgumentException ftcUnsupported) {
            IllegalArgumentException combined = new IllegalArgumentException(
                    "Velocity control tuning supports an exact Sushi standard-regulated Plant "
                            + "or a single/group FTC device-managed motor velocity Plant. The "
                            + "completed Plant matched neither family.");
            combined.addSuppressed(standardUnsupported);
            combined.addSuppressed(ftcUnsupported);
            throw combined;
        }
    }

    static ControlTuningModel.Session claimPosition(PositionPlant plant) {
        IllegalArgumentException standardUnsupported = null;
        StandardControlTuning standard;
        try {
            standard = StandardControlTunings.claimPosition(plant);
        } catch (IllegalArgumentException unsupported) {
            standardUnsupported = unsupported;
            standard = null;
        }
        if (standard != null) {
            requireExactPositionEligibility(plant, standard);
            return new StandardSession(standard);
        }

        FtcMotorPositionControl ftc;
        try {
            ftc = FtcMotorControllers.positionControl(plant);
        } catch (IllegalArgumentException ftcUnsupported) {
            IllegalArgumentException combined = new IllegalArgumentException(
                    "Position control tuning supports an exact Sushi standard-regulated "
                            + "PositionPlant or an exact single-motor FTC device-managed position "
                            + "Plant. Grouped FTC position and equivalent/overlay/planned graphs "
                            + "are not eligible.");
            combined.addSuppressed(standardUnsupported);
            combined.addSuppressed(ftcUnsupported);
            throw combined;
        }
        if (!ftc.hasExactCommandTarget()) {
            throw new IllegalArgumentException(
                    "FTC position tuning requires one literal exact command target");
        }
        return new FtcPositionSession(ftc);
    }

    private static void requireExactPositionEligibility(PositionPlant plant,
                                                        StandardControlTuning tuning) {
        if (!tuning.hasExactCommandTarget()) {
            throw new IllegalArgumentException(
                    "standard position tuning requires one literal exact command target; "
                            + "equivalent-position, overlay, planned, and read-only graphs are "
                            + "not eligible");
        }
        if (plant.periodicity() != PositionPlant.Periodicity.PERIODIC
                || tuning.topology().feedforwardModel()
                != StandardControlTuning.FeedforwardModel.ARM) {
            return;
        }
        double period = plant.period();
        double radiansPerPlantUnit = tuning.topology().radiansPerPlantUnit();
        double gravityPhasePerPeriod = period * radiansPerPlantUnit;
        double turns = gravityPhasePerPeriod / (2.0 * Math.PI);
        double nearestIntegerTurns = Math.rint(turns);
        double tolerance = 1e-9 * Math.max(1.0, Math.abs(gravityPhasePerPeriod));
        if (!Double.isFinite(gravityPhasePerPeriod)
                || !Double.isFinite(turns)
                || Math.abs(gravityPhasePerPeriod
                        - nearestIntegerTurns * 2.0 * Math.PI) > tolerance) {
            throw new IllegalArgumentException(
                    "Periodic ARM feedforward is eligible only when period * "
                            + "radiansPerPlantUnit is an integer multiple of 2*pi; period="
                            + period + ", radiansPerPlantUnit=" + radiansPerPlantUnit
                            + ", product=" + gravityPhasePerPeriod);
        }
    }

    private static final class StandardSession implements ControlTuningModel.Session {
        private final StandardControlTuning tuning;
        private final ControlTuningModel.Parameters initial;

        StandardSession(StandardControlTuning tuning) {
            this.tuning = tuning;
            initial = standardParameters(tuning.initialParameters());
        }

        @Override
        public String topology() {
            StandardControlTuning.Topology topology = tuning.topology();
            return "SUSHI_STANDARD/" + topology.domain() + "/"
                    + topology.setpointModel() + "/" + topology.feedforwardModel();
        }

        @Override
        public ScalarRange plantTargetRange() {
            return tuning.targetRange();
        }

        @Override
        public ControlTuningModel.Parameters initialCandidate() {
            return initial;
        }

        @Override
        public List<ControlTuningModel.Readback> readbacks() {
            LinkedHashMap<String, String> facts = new LinkedHashMap<String, String>();
            facts.put("domain", tuning.domain().toString());
            facts.put("setpointModel", tuning.topology().setpointModel().toString());
            facts.put("feedforwardModel", tuning.topology().feedforwardModel().toString());
            return Collections.singletonList(new ControlTuningModel.Readback(
                    "Sushi standard control",
                    standardParameters(tuning.appliedParameters()),
                    facts));
        }

        @Override
        public void validate(ControlTuningModel.Parameters candidate) {
            standardCandidate(tuning.appliedParameters(), candidate);
        }

        @Override
        public void apply(ControlTuningModel.Parameters candidate, LoopClock clock) {
            tuning.applyAndReseed(standardCandidate(tuning.appliedParameters(), candidate), clock);
        }

        @Override
        public void restoreInitial(LoopClock clock) {
            // A disposable standard controller dies with its terminally stopped Plant. Restoring
            // after Plant.stop() would be both unobservable and rejected by the handle lifecycle.
        }

        @Override
        public ControlTuningModel.ReconfigurationPolicy reconfigurationPolicy() {
            return ControlTuningModel.ReconfigurationPolicy.HOT;
        }

        @Override
        public boolean readyForReconfiguration(LoopClock clock) {
            return true;
        }

        @Override
        public ControlTuningModel.Evidence evidence(LoopClock clock) {
            StandardControlTuning.Evidence evidence = tuning.evidence();
            LinkedHashMap<String, Double> numeric = new LinkedHashMap<String, Double>();
            putIfAvailable(numeric, "goal", evidence.hasGoal(),
                    evidence.hasGoal() ? evidence.goal() : Double.NaN);
            putIfAvailable(numeric, "measurement", evidence.hasMeasurement(),
                    evidence.hasMeasurement() ? evidence.measurement() : Double.NaN);
            putIfAvailable(numeric, "setpointPosition", evidence.hasSetpointPosition(),
                    evidence.hasSetpointPosition() ? evidence.setpointPosition() : Double.NaN);
            putIfAvailable(numeric, "setpointVelocity", evidence.hasSetpointVelocity(),
                    evidence.hasSetpointVelocity() ? evidence.setpointVelocity() : Double.NaN);
            putIfAvailable(numeric, "setpointAcceleration", evidence.hasSetpointAcceleration(),
                    evidence.hasSetpointAcceleration()
                            ? evidence.setpointAcceleration() : Double.NaN);
            LinkedHashMap<String, String> text = new LinkedHashMap<String, String>();
            text.put("completeEvaluation", Boolean.toString(evidence.isAvailable()));
            if (!evidence.isAvailable()) {
                text.put("standardControl", "COMPLETE_EVALUATION_UNAVAILABLE");
                return new ControlTuningModel.Evidence(numeric, text, false, false);
            }
            numeric.put("feedbackError", evidence.feedbackError());
            numeric.put("feedbackIntegral", evidence.feedbackIntegral());
            numeric.put("feedbackOutput", evidence.feedbackOutput());
            numeric.put("feedforwardOutput", evidence.feedforwardOutput());
            putIfAvailable(numeric, "voltageScale", evidence.hasVoltageScale(),
                    evidence.hasVoltageScale() ? evidence.voltageScale() : Double.NaN);
            numeric.put("outputBeforeLimit", evidence.outputBeforeLimit());
            numeric.put("output", evidence.output());
            text.put("setpointSettled", Boolean.toString(evidence.isSetpointSettled()));
            text.put("feedbackLimited", Boolean.toString(evidence.isFeedbackLimited()));
            text.put("integralGrowthBlocked",
                    Boolean.toString(evidence.isIntegralGrowthBlocked()));
            return new ControlTuningModel.Evidence(
                    numeric, text, true, evidence.isOutputLimited());
        }

        @Override
        public double preparePositionHold(LoopClock clock) {
            return tuning.prepareHoldAtCurrent(clock);
        }

        @Override
        public ControlTuningModel.PositionRecoveryHold preparePositionRecoveryHold(
                ScalarRange allowedPhysicalRange,
                LoopClock clock) {
            return recoveryHold(tuning.prepareRecoveryHoldWithin(allowedPhysicalRange, clock));
        }
    }

    private static final class FtcVelocitySession implements ControlTuningModel.Session {
        private final FtcMotorVelocityControl tuning;
        private final ControlTuningModel.Parameters initial;
        private final boolean divergentInitial;
        private boolean requiresInitialApply;
        private boolean restoreRequired;

        FtcVelocitySession(FtcMotorVelocityControl tuning) {
            this.tuning = tuning;
            List<FtcMotorVelocityControl.MemberConfiguration> configurations =
                    tuning.initialConfigurations();
            divergentInitial = hasDivergentVelocityTuples(configurations);
            if (tuning.constructionCandidate().isPresent()) {
                initial = ftcVelocityParameters(tuning.constructionCandidate().get());
                requiresInitialApply = false;
            } else {
                initial = ftcVelocityParameters(configurations.get(0).pidf());
                requiresInitialApply = divergentInitial;
            }
        }

        @Override
        public String topology() {
            return "FTC_DEVICE_VELOCITY/" + (tuning.memberCount() == 1
                    ? "SINGLE" : "GROUP_" + tuning.memberCount())
                    + (divergentInitial ? "/DIVERGENT_INITIAL_READBACKS" : "");
        }

        @Override
        public ScalarRange plantTargetRange() {
            return tuning.plantTargetRange();
        }

        @Override
        public ControlTuningModel.Parameters initialCandidate() {
            return initial;
        }

        @Override
        public boolean requiresInitialApply() {
            return requiresInitialApply;
        }

        @Override
        public List<ControlTuningModel.Readback> readbacks() {
            List<ControlTuningModel.Readback> result = new ArrayList<ControlTuningModel.Readback>();
            for (FtcMotorVelocityControl.MemberConfiguration configuration
                    : tuning.readbackConfigurations()) {
                LinkedHashMap<String, String> facts = new LinkedHashMap<String, String>();
                facts.put("algorithm", configuration.pidf().algorithm().toString());
                facts.put("completeConfiguration", configuration.pidf().toString());
                result.add(new ControlTuningModel.Readback(
                        configuration.motorName(),
                        ftcVelocityParameters(configuration.pidf()),
                        facts));
            }
            return result;
        }

        @Override
        public void validate(ControlTuningModel.Parameters candidate) {
            ftcVelocityCandidate(candidate);
        }

        @Override
        public void apply(ControlTuningModel.Parameters candidate, LoopClock clock) {
            try {
                tuning.apply(ftcVelocityCandidate(candidate), clock);
            } catch (RuntimeException failure) {
                if (tuning.isTerminallyUncertain()) {
                    restoreRequired = true;
                }
                throw failure;
            }
            restoreRequired = true;
            requiresInitialApply = false;
        }

        @Override
        public void restoreInitial(LoopClock clock) {
            if (!restoreRequired) {
                return;
            }
            tuning.restoreInitial();
            restoreRequired = false;
        }

        @Override
        public ControlTuningModel.ReconfigurationPolicy reconfigurationPolicy() {
            return tuning.memberCount() > 1
                    ? ControlTuningModel.ReconfigurationPolicy.ZERO_AND_SETTLE
                    : ControlTuningModel.ReconfigurationPolicy.HOT;
        }

        @Override
        public boolean readyForReconfiguration(LoopClock clock) {
            for (FtcMotorVelocityControl.MemberEvidence member : tuning.evidence(clock)) {
                if (member.nativeCommandedTarget() != 0.0
                        || !member.withinMappedPlantTolerance()) {
                    return false;
                }
            }
            return true;
        }

        @Override
        public boolean experimentAtTarget(boolean plantAtTarget, LoopClock clock) {
            if (!plantAtTarget) {
                return false;
            }
            for (FtcMotorVelocityControl.MemberEvidence member : tuning.evidence(clock)) {
                if (!member.withinMappedPlantTolerance()) {
                    return false;
                }
            }
            return true;
        }

        @Override
        public ControlTuningModel.Evidence evidence(LoopClock clock) {
            LinkedHashMap<String, Double> numeric = new LinkedHashMap<String, Double>();
            LinkedHashMap<String, String> text = new LinkedHashMap<String, String>();
            int index = 1;
            for (FtcMotorVelocityControl.MemberEvidence member : tuning.evidence(clock)) {
                String prefix = "member." + index + ".";
                numeric.put(prefix + "nativeCommandedTarget", member.nativeCommandedTarget());
                numeric.put(prefix + "nativeMeasurement", member.nativeMeasurement());
                numeric.put(prefix + "nativeError", member.nativeError());
                numeric.put(prefix + "nativeTolerance", member.nativeTolerance());
                text.put(prefix + "motorName", member.motorName());
                text.put(prefix + "withinMappedTolerance",
                        Boolean.toString(member.withinMappedPlantTolerance()));
                text.put(prefix + "readbackConfiguration",
                        member.readbackConfiguration().pidf().toString());
                index++;
            }
            if (requiresInitialApply) {
                text.put("initialCandidateSeed",
                        "FIRST_ORDERED_MEMBER_ONLY; first A must apply it to every member");
            } else if (divergentInitial) {
                text.put("initialReadbacks",
                        "DIVERGENT_AFTER_SHARED_CONSTRUCTION_CANDIDATE");
            }
            return new ControlTuningModel.Evidence(numeric, text, false, false);
        }
    }

    private static final class FtcPositionSession implements ControlTuningModel.Session {
        private final FtcMotorPositionControl tuning;
        private final ControlTuningModel.Parameters initial;
        private boolean restoreRequired;

        FtcPositionSession(FtcMotorPositionControl tuning) {
            this.tuning = tuning;
            initial = ftcPositionParameters(tuning.initialConfiguration());
        }

        @Override
        public String topology() {
            return "FTC_DEVICE_POSITION/SINGLE/CASCADE";
        }

        @Override
        public ScalarRange plantTargetRange() {
            return tuning.plantTargetRange();
        }

        @Override
        public ControlTuningModel.Parameters initialCandidate() {
            return initial;
        }

        @Override
        public List<ControlTuningModel.Readback> readbacks() {
            FtcMotorPositionControl.Configuration configuration =
                    tuning.readbackConfiguration();
            LinkedHashMap<String, String> facts = new LinkedHashMap<String, String>();
            facts.put("outerPositionConfiguration", configuration.outerPosition().toString());
            facts.put("innerVelocityConfiguration", configuration.innerVelocity().toString());
            return Collections.singletonList(new ControlTuningModel.Readback(
                    tuning.motorName(), ftcPositionParameters(configuration), facts));
        }

        @Override
        public void validate(ControlTuningModel.Parameters candidate) {
            ftcPositionCandidate(candidate);
        }

        @Override
        public void apply(ControlTuningModel.Parameters candidate, LoopClock clock) {
            try {
                tuning.apply(ftcPositionCandidate(candidate));
            } catch (RuntimeException failure) {
                if (tuning.isTerminallyUncertain()) {
                    restoreRequired = true;
                }
                throw failure;
            }
            restoreRequired = true;
        }

        @Override
        public void restoreInitial(LoopClock clock) {
            if (!restoreRequired) {
                return;
            }
            tuning.restoreInitial();
            restoreRequired = false;
        }

        @Override
        public ControlTuningModel.ReconfigurationPolicy reconfigurationPolicy() {
            return ControlTuningModel.ReconfigurationPolicy.HOT;
        }

        @Override
        public boolean readyForReconfiguration(LoopClock clock) {
            return true;
        }

        @Override
        public ControlTuningModel.Evidence evidence(LoopClock clock) {
            return new ControlTuningModel.Evidence(
                    Collections.<String, Double>emptyMap(),
                    Collections.singletonMap(
                            "controllerEvidence", "FTC_DEVICE_INTERNALS_UNAVAILABLE"),
                    false,
                    false);
        }

        @Override
        public double preparePositionHold(LoopClock clock) {
            return tuning.prepareHoldAtCurrent(clock);
        }

        @Override
        public ControlTuningModel.PositionRecoveryHold preparePositionRecoveryHold(
                ScalarRange allowedPhysicalRange,
                LoopClock clock) {
            return recoveryHold(tuning.prepareRecoveryHoldWithin(allowedPhysicalRange, clock));
        }
    }

    private static ControlTuningModel.PositionRecoveryHold recoveryHold(
            PositionPlantTuning.RecoveryHold hold) {
        return new ControlTuningModel.PositionRecoveryHold(
                hold.measurement(), hold.holdTarget(), hold.wasClamped());
    }

    private static ControlTuningModel.Parameters standardParameters(
            StandardControlTuning.Parameters parameters) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        values.put(FEEDBACK_KP, parameters.kP());
        values.put(FEEDBACK_KI, parameters.kI());
        values.put(FEEDBACK_KD, parameters.kD());
        if (parameters.hasKG()) {
            values.put(FEEDFORWARD_KG, parameters.kG());
        }
        if (parameters.hasKS()) values.put(FEEDFORWARD_KS, parameters.kS());
        if (parameters.hasKV()) values.put(FEEDFORWARD_KV, parameters.kV());
        if (parameters.hasKA()) values.put(FEEDFORWARD_KA, parameters.kA());
        return new ControlTuningModel.Parameters(values);
    }

    private static StandardControlTuning.Parameters standardCandidate(
            StandardControlTuning.Parameters baseline,
            ControlTuningModel.Parameters candidate) {
        StandardControlTuning.Parameters result = baseline.withFeedbackPid(
                candidate.value(FEEDBACK_KP),
                candidate.value(FEEDBACK_KI),
                candidate.value(FEEDBACK_KD));
        switch (baseline.feedforwardModel()) {
            case NONE:
                return result;
            case MOTION:
                if (!baseline.hasKS()) {
                    return result.withMotionFeedforward(candidate.value(FEEDFORWARD_KV));
                }
                return baseline.hasKA()
                        ? result.withMotionFeedforward(
                                candidate.value(FEEDFORWARD_KS),
                                candidate.value(FEEDFORWARD_KV),
                                candidate.value(FEEDFORWARD_KA))
                        : result.withMotionFeedforward(
                                candidate.value(FEEDFORWARD_KS),
                                candidate.value(FEEDFORWARD_KV));
            case LIFT:
                if (!baseline.hasKS()) {
                    return result.withLiftFeedforward(candidate.value(FEEDFORWARD_KG));
                }
                return baseline.hasKA()
                        ? result.withLiftFeedforward(
                                candidate.value(FEEDFORWARD_KG),
                                candidate.value(FEEDFORWARD_KS),
                                candidate.value(FEEDFORWARD_KV),
                                candidate.value(FEEDFORWARD_KA))
                        : result.withLiftFeedforward(
                                candidate.value(FEEDFORWARD_KG),
                                candidate.value(FEEDFORWARD_KS),
                                candidate.value(FEEDFORWARD_KV));
            case ARM:
                if (!baseline.hasKS()) {
                    return result.withArmFeedforward(candidate.value(FEEDFORWARD_KG));
                }
                if (!baseline.hasKA()) {
                    throw new IllegalStateException(
                            "Profiled ARM tuning requires the full kG+kS+kV+kA topology");
                }
                return result.withArmFeedforward(
                        candidate.value(FEEDFORWARD_KG),
                        candidate.value(FEEDFORWARD_KS),
                        candidate.value(FEEDFORWARD_KV),
                        candidate.value(FEEDFORWARD_KA));
            default:
                throw new IllegalStateException(
                        "Unsupported standard feedforward model " + baseline.feedforwardModel());
        }
    }

    private static ControlTuningModel.Parameters ftcVelocityParameters(
            FtcMotorPidfConfiguration configuration) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        values.put("velocity.kP", configuration.getKP());
        values.put("velocity.kI", configuration.getKI());
        values.put("velocity.kD", configuration.getKD());
        values.put("velocity.kF", configuration.getKF());
        return new ControlTuningModel.Parameters(values);
    }

    private static FtcMotorVelocityControl.Candidate ftcVelocityCandidate(
            ControlTuningModel.Parameters candidate) {
        return FtcMotorVelocityControl.Candidate.of(
                candidate.value("velocity.kP"),
                candidate.value("velocity.kI"),
                candidate.value("velocity.kD"),
                candidate.value("velocity.kF"));
    }

    private static ControlTuningModel.Parameters ftcVelocityParameters(
            FtcMotorVelocityControl.Candidate candidate) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        values.put("velocity.kP", candidate.getKP());
        values.put("velocity.kI", candidate.getKI());
        values.put("velocity.kD", candidate.getKD());
        values.put("velocity.kF", candidate.getKF());
        return new ControlTuningModel.Parameters(values);
    }

    private static ControlTuningModel.Parameters ftcPositionParameters(
            FtcMotorPositionControl.Configuration configuration) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        values.put("outerPosition.kP", configuration.outerPosition().getKP());
        values.put("innerVelocity.kP", configuration.innerVelocity().getKP());
        values.put("innerVelocity.kI", configuration.innerVelocity().getKI());
        values.put("innerVelocity.kD", configuration.innerVelocity().getKD());
        values.put("innerVelocity.kF", configuration.innerVelocity().getKF());
        return new ControlTuningModel.Parameters(values);
    }

    private static FtcMotorPositionControl.Candidate ftcPositionCandidate(
            ControlTuningModel.Parameters candidate) {
        return FtcMotorPositionControl.Candidate.of(
                candidate.value("outerPosition.kP"),
                candidate.value("innerVelocity.kP"),
                candidate.value("innerVelocity.kI"),
                candidate.value("innerVelocity.kD"),
                candidate.value("innerVelocity.kF"));
    }

    private static boolean hasDivergentVelocityTuples(
            List<FtcMotorVelocityControl.MemberConfiguration> configurations) {
        FtcMotorPidfConfiguration first = configurations.get(0).pidf();
        for (int index = 1; index < configurations.size(); index++) {
            FtcMotorPidfConfiguration next = configurations.get(index).pidf();
            if (!first.equals(next)) {
                return true;
            }
        }
        return false;
    }

    private static void putIfAvailable(Map<String, Double> destination,
                                       String key,
                                       boolean available,
                                       double value) {
        if (available) destination.put(key, value);
    }

    private ControlTuningAdapters() {
    }
}
