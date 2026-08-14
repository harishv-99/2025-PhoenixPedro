package edu.ftcphoenix.fw.integrations.panels;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.time.LoopClock;

/** Package-local controller-neutral values shared by the Panels experiment workflows. */
final class ControlTuningModel {

    enum ReconfigurationPolicy {
        HOT,
        ZERO_AND_SETTLE
    }

    /** Same-cycle, non-actuating evidence for one position recovery hold. */
    static final class PositionRecoveryHold {
        final double measurement;
        final double holdTarget;
        final boolean clamped;

        PositionRecoveryHold(double measurement, double holdTarget, boolean clamped) {
            if (!Double.isFinite(measurement) || !Double.isFinite(holdTarget)) {
                throw new IllegalArgumentException(
                        "Position recovery evidence must be finite");
            }
            this.measurement = measurement;
            this.holdTarget = holdTarget;
            this.clamped = clamped;
        }
    }

    interface Session {
        String topology();

        ScalarRange plantTargetRange();

        Parameters initialCandidate();

        /** Whether the seeded draft is not yet one tuple proven applied across every member. */
        default boolean requiresInitialApply() {
            return false;
        }

        List<Readback> readbacks();

        /** Validate one complete typed candidate without changing controller or Plant state. */
        default void validate(Parameters candidate) {
            Objects.requireNonNull(candidate, "candidate");
        }

        void apply(Parameters candidate, LoopClock clock);

        void restoreInitial(LoopClock clock);

        ReconfigurationPolicy reconfigurationPolicy();

        /**
         * Whether every implementation-owned controller is ready for a reconfiguration that
         * requires an active-zero transition.
         */
        boolean readyForReconfiguration(LoopClock clock);

        Evidence evidence(LoopClock clock);

        /**
         * Narrows the Plant-level completion fact when an implementation owns additional
         * same-cycle member evidence. Single-controller sessions retain the Plant result.
         */
        default boolean experimentAtTarget(boolean plantAtTarget, LoopClock clock) {
            return plantAtTarget;
        }

        /**
         * Samples position feedback and stages the first graph-owned hold without issuing a normal
         * position output. Velocity sessions do not support this operation.
         */
        default double preparePositionHold(LoopClock clock) {
            throw new IllegalStateException(
                    "The claimed controller does not provide position-hold preparation");
        }

        /**
         * Samples position feedback before normal realization, stages an envelope-safe exact hold,
         * and reseeds implementation-owned state where applicable.
         */
        default PositionRecoveryHold preparePositionRecoveryHold(
                ScalarRange allowedPhysicalRange,
                LoopClock clock) {
            throw new IllegalStateException(
                    "The claimed controller does not provide position recovery-hold preparation");
        }
    }

    static final class Parameters {
        private final Map<String, Double> values;

        Parameters(Map<String, Double> values) {
            Objects.requireNonNull(values, "values");
            LinkedHashMap<String, Double> copy = new LinkedHashMap<String, Double>();
            for (Map.Entry<String, Double> entry : values.entrySet()) {
                String name = requireName(entry.getKey());
                if (copy.containsKey(name)) {
                    throw new IllegalArgumentException("Duplicate controller field: " + name);
                }
                Double boxedValue = Objects.requireNonNull(
                        entry.getValue(), "controller value for " + name);
                copy.put(name, boxedValue);
            }
            if (copy.isEmpty()) {
                throw new IllegalArgumentException(
                        "A tunable controller must expose at least one active parameter");
            }
            this.values = Collections.unmodifiableMap(copy);
        }

        Map<String, Double> values() {
            return values;
        }

        double value(String name) {
            Double value = values.get(name);
            if (value == null) {
                throw new IllegalArgumentException("Controller field is not active: " + name);
            }
            return value;
        }

        String validationError() {
            for (Map.Entry<String, Double> entry : values.entrySet()) {
                if (!Double.isFinite(entry.getValue())) {
                    return entry.getKey() + " must be finite";
                }
            }
            return null;
        }

        boolean sameValues(Parameters other) {
            if (other == null || !values.keySet().equals(other.values.keySet())) {
                return false;
            }
            for (Map.Entry<String, Double> entry : values.entrySet()) {
                if (!sameDouble(entry.getValue(), other.values.get(entry.getKey()))) {
                    return false;
                }
            }
            return true;
        }

        String compact() {
            StringBuilder result = new StringBuilder();
            for (Map.Entry<String, Double> entry : values.entrySet()) {
                if (result.length() > 0) {
                    result.append(", ");
                }
                result.append(entry.getKey()).append('=').append(entry.getValue());
            }
            return result.toString();
        }
    }

    static final class Readback {
        final String owner;
        final Parameters parameters;
        final Map<String, String> fixedFacts;

        Readback(String owner, Parameters parameters) {
            this(owner, parameters, Collections.<String, String>emptyMap());
        }

        Readback(String owner, Parameters parameters, Map<String, String> fixedFacts) {
            this.owner = requireName(owner);
            this.parameters = Objects.requireNonNull(parameters, "parameters");
            this.fixedFacts = Evidence.immutableText(fixedFacts);
        }

        String compact() {
            return owner + "{" + parameters.compact()
                    + (fixedFacts.isEmpty() ? "" : ", " + fixedFacts) + "}";
        }
    }

    static final class Evidence {
        private static final Evidence EMPTY = new Evidence(
                Collections.<String, Double>emptyMap(),
                Collections.<String, String>emptyMap(),
                false,
                false);

        private final Map<String, Double> numeric;
        private final Map<String, String> text;
        final boolean outputLimitedAvailable;
        final boolean outputLimited;

        Evidence(Map<String, Double> numeric,
                 Map<String, String> text,
                 boolean outputLimitedAvailable,
                 boolean outputLimited) {
            this.numeric = immutableNumeric(numeric);
            this.text = immutableText(text);
            this.outputLimitedAvailable = outputLimitedAvailable;
            this.outputLimited = outputLimitedAvailable && outputLimited;
        }

        static Evidence empty() {
            return EMPTY;
        }

        Map<String, Double> numeric() {
            return numeric;
        }

        Map<String, String> text() {
            return text;
        }

        private static Map<String, Double> immutableNumeric(Map<String, Double> source) {
            Objects.requireNonNull(source, "numeric");
            LinkedHashMap<String, Double> copy = new LinkedHashMap<String, Double>();
            for (Map.Entry<String, Double> entry : source.entrySet()) {
                copy.put(requireName(entry.getKey()), Objects.requireNonNull(
                        entry.getValue(), "evidence value for " + entry.getKey()));
            }
            return Collections.unmodifiableMap(copy);
        }

        static Map<String, String> immutableText(Map<String, String> source) {
            Objects.requireNonNull(source, "text");
            LinkedHashMap<String, String> copy = new LinkedHashMap<String, String>();
            for (Map.Entry<String, String> entry : source.entrySet()) {
                copy.put(requireName(entry.getKey()), Objects.requireNonNull(
                        entry.getValue(), "evidence text for " + entry.getKey()));
            }
            return Collections.unmodifiableMap(copy);
        }
    }

    static List<Readback> immutableReadbacks(List<Readback> readbacks) {
        Objects.requireNonNull(readbacks, "readbacks");
        ArrayList<Readback> copy = new ArrayList<Readback>(readbacks.size());
        for (Readback readback : readbacks) {
            copy.add(Objects.requireNonNull(readback, "readback"));
        }
        if (copy.isEmpty()) {
            throw new IllegalArgumentException(
                    "A tuning session must publish at least one controller readback");
        }
        return Collections.unmodifiableList(copy);
    }

    static boolean sameReadbackSchema(Parameters candidate, List<Readback> readbacks) {
        for (Readback readback : readbacks) {
            if (!candidate.values().keySet().equals(readback.parameters.values().keySet())) {
                return false;
            }
        }
        return true;
    }

    static boolean sameDouble(double first, double second) {
        return Double.doubleToLongBits(first) == Double.doubleToLongBits(second);
    }

    static String requireName(String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException("name must be nonblank");
        }
        return value;
    }

    private ControlTuningModel() {
    }
}
