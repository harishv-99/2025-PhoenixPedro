package edu.ftcphoenix.fw.integrations.panels;

import java.util.LinkedHashMap;
import java.util.Map;

/** Deterministic response and post-settlement disturbance metrics. */
final class ControlResponseMetrics {

    static final double SETTLED_DWELL_SEC = 0.20;

    abstract static class Accumulator {
        final double target;
        final double startedAtSec;
        double peakAbsError;
        double firstAtTargetAtSec = Double.NaN;
        double toleranceRunStartedAtSec = Double.NaN;
        double settledAtSec = Double.NaN;
        double previousSampleAtSec = Double.NaN;
        double outputLimitedDurationSec;
        boolean outputLimitingAvailable;
        boolean previousOutputLimitedAvailable;
        boolean previousOutputLimited;
        boolean disturbanceActive;
        boolean finished;
        double disturbanceStartedAtSec = Double.NaN;
        double disturbancePeakAbsError;
        double disturbanceToleranceRunStartedAtSec = Double.NaN;
        double disturbanceRecoverySec = Double.NaN;
        ControlTuningModel.Evidence latestEvidence = ControlTuningModel.Evidence.empty();

        Accumulator(double target, double startedAtSec, double initialMeasurement) {
            this.target = target;
            this.startedAtSec = startedAtSec;
            if (Double.isFinite(initialMeasurement)) {
                peakAbsError = Math.abs(target - initialMeasurement);
            }
        }

        final void updateShared(double nowSec,
                                double measurement,
                                boolean atTarget,
                                ControlTuningModel.Evidence evidence) {
            double sampleDurationSec = Double.isFinite(previousSampleAtSec)
                    && Double.isFinite(nowSec) && nowSec >= previousSampleAtSec
                    ? nowSec - previousSampleAtSec : 0.0;
            if (Double.isFinite(nowSec)) {
                previousSampleAtSec = nowSec;
            }
            if (!Double.isFinite(measurement)) {
                return;
            }
            double absError = Math.abs(target - measurement);
            peakAbsError = Math.max(peakAbsError, absError);
            latestEvidence = evidence == null
                    ? ControlTuningModel.Evidence.empty() : evidence;
            if (latestEvidence.outputLimitedAvailable) {
                outputLimitingAvailable = true;
            }
            if (previousOutputLimitedAvailable
                    && previousOutputLimited && sampleDurationSec > 0.0) {
                outputLimitedDurationSec += sampleDurationSec;
            }
            previousOutputLimitedAvailable = latestEvidence.outputLimitedAvailable;
            previousOutputLimited = latestEvidence.outputLimited;

            if (!Double.isFinite(settledAtSec)) {
                if (atTarget) {
                    if (!Double.isFinite(firstAtTargetAtSec)) {
                        firstAtTargetAtSec = nowSec;
                    }
                    if (!Double.isFinite(toleranceRunStartedAtSec)) {
                        toleranceRunStartedAtSec = nowSec;
                    }
                    if (nowSec - toleranceRunStartedAtSec >= SETTLED_DWELL_SEC) {
                        settledAtSec = toleranceRunStartedAtSec + SETTLED_DWELL_SEC;
                    }
                } else {
                    toleranceRunStartedAtSec = Double.NaN;
                }
                return;
            }

            if (!disturbanceActive && !atTarget) {
                disturbanceActive = true;
                disturbanceStartedAtSec = nowSec;
                disturbancePeakAbsError = absError;
                disturbanceToleranceRunStartedAtSec = Double.NaN;
                disturbanceRecoverySec = Double.NaN;
            }
            if (!disturbanceActive) {
                return;
            }

            disturbancePeakAbsError = Math.max(disturbancePeakAbsError, absError);
            if (!atTarget) {
                disturbanceToleranceRunStartedAtSec = Double.NaN;
                return;
            }
            if (!Double.isFinite(disturbanceToleranceRunStartedAtSec)) {
                disturbanceToleranceRunStartedAtSec = nowSec;
            }
            if (nowSec - disturbanceToleranceRunStartedAtSec >= SETTLED_DWELL_SEC) {
                disturbanceRecoverySec = disturbanceToleranceRunStartedAtSec
                        + SETTLED_DWELL_SEC - disturbanceStartedAtSec;
                disturbanceActive = false;
            }
        }

        final Map<String, Double> sharedSnapshot() {
            LinkedHashMap<String, Double> result = new LinkedHashMap<String, Double>();
            result.put("firstAtTargetSec", elapsed(firstAtTargetAtSec));
            result.put("settlingSec", elapsed(settledAtSec));
            result.put("peakAbsError", peakAbsError);
            result.put("outputLimitedDurationSec",
                    outputLimitingAvailable ? outputLimitedDurationSec : Double.NaN);
            result.put("disturbancePeakAbsError",
                    Double.isFinite(disturbanceStartedAtSec)
                            ? disturbancePeakAbsError : Double.NaN);
            result.put("disturbanceRecoverySec", disturbanceRecoverySec);
            for (Map.Entry<String, Double> entry : latestEvidence.numeric().entrySet()) {
                result.put("controller." + entry.getKey(), entry.getValue());
            }
            return result;
        }

        final void finish(double nowSec) {
            if (finished) {
                return;
            }
            finished = true;
            if (previousOutputLimitedAvailable && previousOutputLimited
                    && Double.isFinite(previousSampleAtSec)
                    && Double.isFinite(nowSec) && nowSec >= previousSampleAtSec) {
                outputLimitedDurationSec += nowSec - previousSampleAtSec;
            }
        }

        final boolean isSettled() {
            return Double.isFinite(settledAtSec);
        }

        final Map<String, String> evidenceSnapshot() {
            LinkedHashMap<String, String> result =
                    new LinkedHashMap<String, String>(latestEvidence.text());
            result.put("outputLimiting",
                    outputLimitingAvailable ? "AVAILABLE" : "UNAVAILABLE");
            result.put("disturbanceState", disturbanceActive ? "RECOVERING" :
                    (Double.isFinite(disturbanceStartedAtSec) ? "RECOVERED" : "NOT_OBSERVED"));
            return result;
        }

        final void retainEvidence(ControlTuningModel.Evidence evidence) {
            latestEvidence = evidence == null
                    ? ControlTuningModel.Evidence.empty() : evidence;
        }

        final Map<String, Double> controllerNumericEvidence() {
            return latestEvidence.numeric();
        }

        private double elapsed(double timestampSec) {
            return Double.isFinite(timestampSec) ? timestampSec - startedAtSec : Double.NaN;
        }
    }

    static final class Velocity extends Accumulator {
        private final double direction;
        private double peakDirectionalDroop;
        private double peakDirectionalOvershoot;

        Velocity(double target, double startedAtSec, double initialMeasurement) {
            super(target, startedAtSec, initialMeasurement);
            direction = Math.signum(target);
        }

        void update(double nowSec,
                    double measurement,
                    boolean atTarget,
                    ControlTuningModel.Evidence evidence) {
            updateShared(nowSec, measurement, atTarget, evidence);
            if (!Double.isFinite(measurement)) {
                return;
            }
            if (isSettled()) {
                peakDirectionalDroop = Math.max(
                        peakDirectionalDroop,
                        direction * (target - measurement));
            }
            peakDirectionalOvershoot = Math.max(
                    peakDirectionalOvershoot,
                    direction * (measurement - target));
        }

        Map<String, Double> snapshot() {
            Map<String, Double> result = sharedSnapshot();
            result.put("directionalDroop", direction == 0.0
                    ? Double.NaN : Math.max(0.0, peakDirectionalDroop));
            result.put("overshoot", direction == 0.0
                    ? Double.NaN : Math.max(0.0, peakDirectionalOvershoot));
            return result;
        }

        Map<String, String> evidence() {
            return evidenceSnapshot();
        }
    }

    static final class Position extends Accumulator {
        private final double startingMeasurement;
        private final double travel;
        private final double direction;
        private double peakOvershoot;
        private double peakMeasuredRate;
        private double previousMeasurement;
        private boolean previousMeasurementAvailable;
        private double previousMeasurementAtSec;
        private double settledMeasurement = Double.NaN;
        private double holdError;
        private double holdDrift;
        private double disturbanceDisplacement;

        Position(double target, double startedAtSec, double initialMeasurement) {
            super(target, startedAtSec, initialMeasurement);
            startingMeasurement = initialMeasurement;
            travel = Double.isFinite(initialMeasurement) ? target - initialMeasurement : Double.NaN;
            direction = Double.isFinite(travel) ? Math.signum(travel) : 0.0;
            previousMeasurement = initialMeasurement;
            previousMeasurementAvailable = Double.isFinite(initialMeasurement);
            previousMeasurementAtSec = startedAtSec;
        }

        void update(double nowSec,
                    double measurement,
                    boolean atTarget,
                    ControlTuningModel.Evidence evidence) {
            boolean wasSettled = Double.isFinite(settledAtSec);
            boolean wasDisturbanceActive = disturbanceActive;
            updateShared(nowSec, measurement, atTarget, evidence);
            if (!wasDisturbanceActive && disturbanceActive) {
                disturbanceDisplacement = 0.0;
            }
            if (!Double.isFinite(measurement)) {
                previousMeasurementAvailable = false;
                return;
            }
            peakOvershoot = Math.max(
                    peakOvershoot,
                    direction * (measurement - target));
            double measurementDurationSec = Double.isFinite(nowSec)
                    && Double.isFinite(previousMeasurementAtSec)
                    ? nowSec - previousMeasurementAtSec : Double.NaN;
            if (previousMeasurementAvailable && Double.isFinite(measurementDurationSec)
                    && measurementDurationSec > 1e-6) {
                double rate = Math.abs(
                        (measurement - previousMeasurement) / measurementDurationSec);
                if (Double.isFinite(rate)) {
                    peakMeasuredRate = Math.max(peakMeasuredRate, rate);
                }
            }
            previousMeasurement = measurement;
            previousMeasurementAvailable = true;
            previousMeasurementAtSec = nowSec;

            if (!wasSettled && Double.isFinite(settledAtSec)) {
                settledMeasurement = measurement;
            }
            if (Double.isFinite(settledAtSec)) {
                holdError = target - measurement;
                if (Double.isFinite(settledMeasurement)) {
                    holdDrift = Math.max(holdDrift, Math.abs(measurement - settledMeasurement));
                    if (disturbanceActive) {
                        disturbanceDisplacement = Math.max(
                                disturbanceDisplacement,
                                Math.abs(measurement - settledMeasurement));
                    }
                }
            }
        }

        Map<String, Double> snapshot() {
            Map<String, Double> result = sharedSnapshot();
            result.put("startPosition", startingMeasurement);
            result.put("travel", travel);
            result.put("direction", direction);
            result.put("overshoot", direction == 0.0
                    ? Double.NaN : Math.max(0.0, peakOvershoot));
            result.put("peakMeasuredRate", peakMeasuredRate);
            result.put("holdError", Double.isFinite(settledAtSec) ? holdError : Double.NaN);
            result.put("holdDrift", Double.isFinite(settledAtSec) ? holdDrift : Double.NaN);
            result.put("disturbanceDisplacement",
                    Double.isFinite(disturbanceStartedAtSec)
                            ? disturbanceDisplacement : Double.NaN);
            return result;
        }

        Map<String, String> evidence() {
            return evidenceSnapshot();
        }
    }

    private ControlResponseMetrics() {
    }
}
