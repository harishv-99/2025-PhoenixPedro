package edu.ftcphoenix.fw.integrations.panels;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicLong;

/** Complete in-memory evidence records for one tuning OpMode lifetime. */
final class ControlExperimentHistory {

    private static final AtomicLong SESSION_COUNTER = new AtomicLong();

    enum Transition {
        TARGET_CHANGE,
        CONTROLLER_CHANGE,
        CONTROLLER_AND_TARGET_CHANGE,
        REPEAT
    }

    static final class Record {
        final String sessionId;
        final long segmentId;
        final String domain;
        final Transition transition;
        final ControlTuningModel.Parameters controllerCandidate;
        final List<ControlTuningModel.Readback> controllerReadbacks;
        final Map<String, Double> experimentRequest;
        final Map<String, Double> metrics;
        final Map<String, String> evidence;
        final double startedAtSec;
        final double endedAtSec;
        final String terminationReason;

        Record(String sessionId,
               long segmentId,
               String domain,
               Transition transition,
               ControlTuningModel.Parameters controllerCandidate,
               List<ControlTuningModel.Readback> controllerReadbacks,
               Map<String, Double> experimentRequest,
               Map<String, Double> metrics,
               Map<String, String> evidence,
               double startedAtSec,
               double endedAtSec,
               String terminationReason) {
            this.sessionId = ControlTuningModel.requireName(sessionId);
            if (segmentId <= 0L) {
                throw new IllegalArgumentException("segmentId must be positive");
            }
            this.segmentId = segmentId;
            this.domain = ControlTuningModel.requireName(domain);
            this.transition = Objects.requireNonNull(transition, "transition");
            this.controllerCandidate = Objects.requireNonNull(
                    controllerCandidate, "controllerCandidate");
            this.controllerReadbacks = ControlTuningModel.immutableReadbacks(controllerReadbacks);
            this.experimentRequest = immutableDoubles(experimentRequest);
            this.metrics = immutableDoubles(metrics);
            this.evidence = immutableStrings(evidence);
            if (!Double.isFinite(startedAtSec) || !Double.isFinite(endedAtSec)
                    || endedAtSec < startedAtSec) {
                throw new IllegalArgumentException(
                        "Record timestamps must be finite and nondecreasing");
            }
            this.startedAtSec = startedAtSec;
            this.endedAtSec = endedAtSec;
            this.terminationReason = ControlTuningModel.requireName(terminationReason);
        }
    }

    private final String sessionId;
    private final List<Record> records = new ArrayList<Record>();

    ControlExperimentHistory() {
        sessionId = Long.toString(System.currentTimeMillis(), 36)
                + "-" + Long.toString(SESSION_COUNTER.incrementAndGet(), 36);
    }

    String sessionId() {
        return sessionId;
    }

    void add(Record record) {
        Objects.requireNonNull(record, "record");
        if (!sessionId.equals(record.sessionId)) {
            throw new IllegalArgumentException("Record belongs to another tuning session");
        }
        if (!records.isEmpty()
                && record.segmentId <= records.get(records.size() - 1).segmentId) {
            throw new IllegalArgumentException(
                    "Record segment IDs must increase within one tuning session");
        }
        records.add(record);
    }

    List<Record> records() {
        return Collections.unmodifiableList(new ArrayList<Record>(records));
    }

    private static Map<String, Double> immutableDoubles(Map<String, Double> source) {
        Objects.requireNonNull(source, "source");
        LinkedHashMap<String, Double> copy = new LinkedHashMap<String, Double>();
        for (Map.Entry<String, Double> entry : source.entrySet()) {
            copy.put(ControlTuningModel.requireName(entry.getKey()),
                    Objects.requireNonNull(entry.getValue(),
                            "numeric value for " + entry.getKey()));
        }
        return Collections.unmodifiableMap(copy);
    }

    private static Map<String, String> immutableStrings(Map<String, String> source) {
        Objects.requireNonNull(source, "source");
        LinkedHashMap<String, String> copy = new LinkedHashMap<String, String>();
        for (Map.Entry<String, String> entry : source.entrySet()) {
            copy.put(ControlTuningModel.requireName(entry.getKey()),
                    Objects.requireNonNull(entry.getValue(),
                            "text value for " + entry.getKey()));
        }
        return Collections.unmodifiableMap(copy);
    }
}
