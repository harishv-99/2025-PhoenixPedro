package edu.ftcsushi.fw.integrations.panels;

import com.google.gson.stream.JsonWriter;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.ResultDownloads;

/**
 * Optional, bounded recording of the actual response-metric calls made by one Panels owner.
 * No method polls a source, changes a controller, writes a file, or advances the borrowed clock.
 * Recoverable serialization and transport failures only make the optional recording unavailable.
 * Java Errors propagate, as in the owning tester lifecycle.
 */
final class ControlExperimentRecording {
    static final String FORMAT = "sushi-control-response";
    static final int VERSION = 1;
    static final int MAX_OPERATIONS = 1024;
    static final int MAX_BYTES = 512 * 1024;
    static final int MAX_BOUNDARY_BYTES = 64 * 1024;
    static final int MAX_OPERATION_BYTES = 64 * 1024;
    static final int MAX_BODY_BYTES = MAX_BYTES - 2 * MAX_BOUNDARY_BYTES;
    static final String UNRECORDED = "UNRECORDED";

    private final ResultDownloads downloads;
    private Capture active;
    private Pending pending;
    private long generation;
    private boolean stopped;
    private String status = "No completed recording";

    ControlExperimentRecording(ResultDownloads downloads) {
        this.downloads = downloads;
    }

    /** Called only after a new segment is accepted; rejected or pending drafts never clear a result. */
    void begin(LoopClock clock, String sessionId, long segmentId, String domain, String topology,
               ScalarRange range, List<ControlTuningModel.Readback> initialReadbacks,
               List<ControlTuningModel.Readback> acceptedReadbacks,
               ControlTuningModel.Parameters candidate, Map<String, Double> request,
               double target, double initialMeasurement) {
        if (stopped) return;
        long startingGeneration = ++generation;
        active = null;
        pending = null;
        status = "Recording current segment";
        try {
            downloads.clear();
            if (stopped || generation != startingGeneration) return;
            active = new Capture(clock, sessionId, segmentId, domain, topology, range,
                    initialReadbacks, acceptedReadbacks, candidate, request, target, initialMeasurement);
        } catch (RuntimeException failure) {
            unavailable("Recording unavailable: start metadata or download service failed");
        }
    }

    /** Preserve an actual update call, including the completion boolean chosen by its real owner. */
    void sample(LoopClock clock, double measurement, boolean atTarget,
                ControlTuningModel.Evidence evidence) {
        operation(clock, "SAMPLE", measurement, atTarget, evidence);
    }

    /** Missing feedback calls retainEvidence, not update(NaN), in both live workflows. */
    void evidenceOnly(LoopClock clock, ControlTuningModel.Evidence evidence) {
        operation(clock, "EVIDENCE_ONLY", Double.NaN, false, evidence);
    }

    private void operation(LoopClock clock, String kind, double measurement, boolean atTarget,
                           ControlTuningModel.Evidence evidence) {
        if (active == null || stopped) return;
        try {
            active.operation(clock, kind, measurement, atTarget, evidence);
            if (active.omitted > 0) status = "Recording incomplete: operation quota reached";
        } catch (RuntimeException failure) {
            unavailable("Recording unavailable: invalid time boundary or encoding failure");
        }
    }

    /**
     * Retain the already-frozen original ending record; do not encode or publish before the
     * owner's pending ordinary zero/hold output has been realized.
     */
    void finish(LoopClock clock, ControlExperimentHistory.Record result) {
        Capture ending = active;
        active = null;
        if (stopped || ending == null) return;
        try {
            ending.checkTime(clock);
            if (!ending.sessionId.equals(result.sessionId) || ending.segmentId != result.segmentId)
                throw new IllegalArgumentException("recording/result identity mismatch");
            pending = new Pending(ending, result, clock.cycle(), clock.nowSec(), generation);
        } catch (RuntimeException failure) {
            unavailable("Recording unavailable: ending evidence crossed a reset or segment boundary");
        }
    }

    /** Called after the normal Plant update, never by STOP/cleanup or a browser request. */
    void publishAfterOutput(LoopClock clock) {
        Pending ready = pending;
        pending = null; // Claim before any encoding/transport callback can reenter the owner.
        if (stopped || ready == null || ready.generation != generation) return;
        try {
            ready.capture.checkTime(clock);
            String text = ready.capture.finishText(ready);
            if (stopped || generation != ready.generation) return;
            boolean published = downloads.publish("control-" + ready.capture.segmentId + ".jsonl", text);
            if (stopped || generation != ready.generation) return;
            status = published
                    ? (ready.capture.omitted == 0 ? "Completed recording available"
                    : "Incomplete recording available: operations omitted")
                    : "Download unavailable from this host";
        } catch (RuntimeException failure) {
            if (!stopped && generation == ready.generation)
                unavailable("Recording unavailable: final encoding or publication failed");
        }
    }

    /** Invalidate before hardware cleanup; a custom transport failure cannot prevent that cleanup. */
    void stop() {
        if (stopped) return;
        stopped = true;
        generation++;
        active = null;
        pending = null;
        status = "Recording ended with tester session";
        try { downloads.clear(); } catch (RuntimeException ignored) { /* Optional transport. */ }
    }

    String status() { return status; }

    /** A cached transport URL is optional and is added to the owner's existing telemetry frame. */
    String url() {
        if (stopped) return null;
        try { return downloads.url(); }
        catch (RuntimeException ignored) { return null; }
    }

    private void unavailable(String message) {
        active = null;
        pending = null;
        status = message;
    }

    private static final class Pending {
        final Capture capture;
        final ControlExperimentHistory.Record result;
        final long cycle, generation;
        final double timeSec;
        Pending(Capture capture, ControlExperimentHistory.Record result, long cycle,
                double timeSec, long generation) {
            this.capture = capture; this.result = result; this.cycle = cycle;
            this.timeSec = timeSec; this.generation = generation;
        }
    }

    /** Retains bounded, already-encoded lines, not an ever-growing graph of live observations. */
    private static final class Capture {
        final String sessionId;
        final long segmentId;
        final LoopTimestamp anchor;
        final List<byte[]> lines = new ArrayList<byte[]>();
        final MessageDigest digest = sha256();
        long attempted, omitted;
        double firstOmittedSec = Double.NaN, lastOmittedSec = Double.NaN;
        double lastTimeSec;
        long lastCycle;
        int bodyBytes, retained;

        Capture(LoopClock clock, String sessionId, long segmentId, String domain, String topology,
                ScalarRange range, List<ControlTuningModel.Readback> initialReadbacks,
                List<ControlTuningModel.Readback> acceptedReadbacks,
                ControlTuningModel.Parameters candidate, Map<String, Double> request,
                double target, double initialMeasurement) {
            this.sessionId = sessionId;
            this.segmentId = segmentId;
            anchor = clock.nowTimestamp();
            lastTimeSec = clock.nowSec();
            lastCycle = clock.cycle();
            append(encode(MAX_BOUNDARY_BYTES, out -> {
                event(out, "START", 0, lastCycle, lastTimeSec);
                out.name("format").value(FORMAT).name("version").value(VERSION);
                out.name("sessionId").value(sessionId).name("segmentId").value(segmentId);
                out.name("domain").value(domain).name("topology").value(topology);
                out.name("clockSpan").value("ONE_RESET_EPOCH");
                out.name("robotConfigurationRevision").value(UNRECORDED);
                out.name("codeRevision").value(UNRECORDED);
                out.name("acquisitionTime").value(UNRECORDED);
                out.name("initialMeasurementObservationTime").value(UNRECORDED);
                out.name("plantPhysicalUnitName").value(UNRECORDED);
                out.name("targetMapping").value(UNRECORDED);
                out.name("nativeEvidenceUnits").value(topology.startsWith("FTC_DEVICE_VELOCITY/")
                        ? "ENCODER_TICKS_PER_SECOND" : topology.startsWith("FTC_DEVICE_POSITION/")
                        ? "ENCODER_TICKS" : UNRECORDED);
                out.name("target"); number(out, target);
                out.name("initialMeasurement"); number(out, initialMeasurement);
                out.name("experimentRange").beginArray();
                number(out, range.minValue); number(out, range.maxValue); out.endArray();
                out.name("controllerCandidate"); numbers(out, candidate.values());
                out.name("experimentRequest"); numbers(out, request);
                out.name("initialReadbacks"); readbacks(out, initialReadbacks);
                out.name("acceptedReadbacks"); readbacks(out, acceptedReadbacks);
                out.endObject();
            }));
        }

        void checkTime(LoopClock clock) {
            if (!Double.isFinite(anchor.ageSec(clock)) || !Double.isFinite(clock.nowSec())
                    || clock.nowSec() < lastTimeSec || clock.cycle() < lastCycle)
                throw new IllegalArgumentException("recording requires its original nondecreasing clock epoch");
        }

        void operation(LoopClock clock, String kind, double measurement, boolean atTarget,
                       ControlTuningModel.Evidence evidence) {
            checkTime(clock);
            lastTimeSec = clock.nowSec(); lastCycle = clock.cycle();
            if (attempted == Long.MAX_VALUE - 2) throw new IllegalStateException("operation count exhausted");
            attempted++;
            if (omitted > 0 || retained == MAX_OPERATIONS) { omit(); return; }
            byte[] encoded;
            try {
                encoded = encode(Math.min(MAX_OPERATION_BYTES, MAX_BODY_BYTES - bodyBytes), out -> {
                    event(out, kind, attempted, lastCycle, lastTimeSec);
                    out.name("acquisitionTime").value(UNRECORDED);
                    if ("SAMPLE".equals(kind)) {
                        out.name("measurement"); number(out, measurement);
                        out.name("atTarget").value(atTarget);
                    }
                    out.name("evidence"); evidence(out, evidence);
                    out.endObject();
                });
            } catch (RecordLimit exceeded) { omit(); return; }
            append(encoded); bodyBytes += encoded.length; retained++;
        }

        private void omit() {
            if (omitted++ == 0) firstOmittedSec = lastTimeSec;
            lastOmittedSec = lastTimeSec;
        }

        private void append(byte[] line) { lines.add(line); digest.update(line); }

        String finishText(Pending end) {
            byte[] finish = encode(MAX_BOUNDARY_BYTES - 256, out -> {
                event(out, "FINISH", attempted + 1, end.cycle, end.timeSec);
                out.name("terminationReason").value(end.result.terminationReason);
                out.name("transition").value(end.result.transition.name());
                out.name("attemptedOperations").value(attempted);
                out.name("retainedOperations").value(retained);
                out.name("omittedOperations").value(omitted);
                out.name("firstOmittedProcessingSec"); number(out, firstOmittedSec);
                out.name("lastOmittedProcessingSec"); number(out, lastOmittedSec);
                out.name("responseMetrics").beginObject();
                for (Map.Entry<String, Double> entry : end.result.metrics.entrySet()) {
                    if (!isFinalPlantFact(entry.getKey())) {
                        out.name(entry.getKey()); number(out, entry.getValue());
                    }
                }
                out.endObject().name("finalPlantFacts").beginObject();
                for (Map.Entry<String, Double> entry : end.result.metrics.entrySet()) {
                    if (isFinalPlantFact(entry.getKey())) {
                        out.name(entry.getKey()); number(out, entry.getValue());
                    }
                }
                out.endObject().name("evidence"); strings(out, end.result.evidence);
                out.endObject();
            });
            append(finish);
            String hash = hex(digest.digest());
            byte[] seal = encode(256, out -> {
                out.beginObject().name("type").value("SEAL");
                out.name("sequence").value(attempted + 2);
                out.name("sha256").value(hash).endObject();
            });
            ByteArrayOutputStream complete = new ByteArrayOutputStream();
            for (byte[] line : lines) complete.write(line, 0, line.length);
            complete.write(seal, 0, seal.length);
            if (complete.size() > MAX_BYTES) throw new RecordLimit();
            return new String(complete.toByteArray(), StandardCharsets.UTF_8);
        }
    }

    static boolean isFinalPlantFact(String key) {
        return "finalRequestedTarget".equals(key) || "finalAppliedTarget".equals(key)
                || "finalMeasurement".equals(key);
    }

    private interface JsonLine { void write(JsonWriter out) throws IOException; }

    /** Bound encoded bytes while Gson writes; never construct an unbounded intermediate JSON tree. */
    private static byte[] encode(int limit, JsonLine line) {
        LimitedBytes bytes = new LimitedBytes(limit);
        try {
            JsonWriter out = new JsonWriter(new OutputStreamWriter(bytes,
                    StandardCharsets.UTF_8.newEncoder().onMalformedInput(CodingErrorAction.REPORT)
                            .onUnmappableCharacter(CodingErrorAction.REPORT)));
            line.write(out);
            out.flush();
            bytes.write('\n');
            return bytes.bytes.toByteArray();
        } catch (IOException invalid) {
            throw new IllegalArgumentException("Cannot encode recording as UTF-8", invalid);
        }
    }

    private static void event(JsonWriter out, String kind, long sequence, long cycle, double seconds)
            throws IOException {
        out.beginObject().name("type").value(kind).name("sequence").value(sequence);
        out.name("cycle").value(cycle).name("processingSec").value(seconds);
    }

    static void number(JsonWriter out, double value) throws IOException {
        if (Double.isFinite(value)) out.value(value);
        else out.value(Double.isNaN(value) ? "NAN" : value > 0 ? "POSITIVE_INFINITY" : "NEGATIVE_INFINITY");
    }

    private static void numbers(JsonWriter out, Map<String, Double> values) throws IOException {
        out.beginObject();
        for (Map.Entry<String, Double> entry : values.entrySet()) {
            out.name(entry.getKey()); number(out, entry.getValue());
        }
        out.endObject();
    }

    private static void strings(JsonWriter out, Map<String, String> values) throws IOException {
        out.beginObject();
        for (Map.Entry<String, String> entry : values.entrySet()) out.name(entry.getKey()).value(entry.getValue());
        out.endObject();
    }

    private static void readbacks(JsonWriter out, List<ControlTuningModel.Readback> values) throws IOException {
        out.beginArray();
        for (ControlTuningModel.Readback value : values) {
            out.beginObject().name("owner").value(value.owner).name("parameters");
            numbers(out, value.parameters.values());
            out.name("fixedFacts"); strings(out, value.fixedFacts); out.endObject();
        }
        out.endArray();
    }

    private static void evidence(JsonWriter out, ControlTuningModel.Evidence evidence) throws IOException {
        out.beginObject().name("numeric"); numbers(out, evidence.numeric());
        out.name("text"); strings(out, evidence.text());
        out.name("outputLimitedAvailable").value(evidence.outputLimitedAvailable);
        out.name("outputLimited").value(evidence.outputLimited).endObject();
    }

    static MessageDigest sha256() {
        try { return MessageDigest.getInstance("SHA-256"); }
        catch (NoSuchAlgorithmException impossible) { throw new IllegalStateException(impossible); }
    }

    static String hex(byte[] bytes) {
        StringBuilder value = new StringBuilder(bytes.length * 2);
        for (byte b : bytes) value.append(Character.forDigit((b >>> 4) & 15, 16))
                .append(Character.forDigit(b & 15, 16));
        return value.toString();
    }

    private static final class RecordLimit extends RuntimeException { }

    private static final class LimitedBytes extends OutputStream {
        final ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        final int limit;
        LimitedBytes(int limit) { this.limit = limit; }
        @Override public void write(int value) {
            if (bytes.size() == limit) throw new RecordLimit();
            bytes.write(value);
        }
        @Override public void write(byte[] values, int offset, int length) {
            if (length > limit - bytes.size()) throw new RecordLimit();
            bytes.write(values, offset, length);
        }
    }
}
