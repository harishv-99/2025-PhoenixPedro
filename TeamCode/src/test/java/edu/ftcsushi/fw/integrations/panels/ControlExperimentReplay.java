package edu.ftcsushi.fw.integrations.panels;

import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonToken;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.StringReader;
import java.nio.ByteBuffer;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Laptop-only strict reader for version-one control-response recordings. It reuses the real
 * package-private metrics; it does not replay controllers, hardware, or physical outcomes.
 * This source is deliberately outside the Android production artifact.
 */
public final class ControlExperimentReplay {
    private ControlExperimentReplay() { }

    /** Run through :TeamCode:replayControlExperiment -Precording=/path/to/control-N.jsonl. */
    public static void main(String[] args) throws IOException {
        if (args.length != 1) throw new IllegalArgumentException("Supply exactly one downloaded recording path");
        try (InputStream input = Files.newInputStream(Paths.get(args[0]))) {
            Report report = replay(input);
            System.out.println("CONTROL_REPLAY status=" + report.status + " session=" + report.sessionId
                    + " segment=" + report.segmentId + " domain=" + report.domain
                    + " retained=" + report.retained + " omitted=" + report.omitted);
            System.out.println("Recorded code identity: UNRECORDED; comparison uses the current implementation.");
            if (!"COMPLETE_MATCH".equals(report.status))
                throw new IllegalStateException("Response-metric replay is " + report.status
                        + "; do not treat incomplete or differing evidence as an exact replay");
        }
    }

    /** Frozen comparison result; incomplete streams never claim matching reconstructed metrics. */
    static final class Report {
        final String status, sessionId, domain;
        final long segmentId, omitted;
        final int retained;
        final Map<String, Double> recordedMetrics, replayedMetrics, finalPlantFacts;
        Report(State state) {
            status = state.omitted > 0 ? "INCOMPLETE" : state.matches ? "COMPLETE_MATCH" : "MISMATCH";
            sessionId = state.sessionId; domain = state.domain; segmentId = state.segmentId;
            retained = state.operations; omitted = state.omitted;
            recordedMetrics = Collections.unmodifiableMap(state.recorded);
            replayedMetrics = state.omitted > 0 ? Collections.<String, Double>emptyMap()
                    : Collections.unmodifiableMap(state.replayed);
            finalPlantFacts = Collections.unmodifiableMap(state.finalFacts);
        }
    }

    /** Decode incrementally with byte/line bounds; chunk boundaries never become metric operations. */
    static Report replay(InputStream input) throws IOException {
        State state = new State();
        ByteArrayOutputStream line = new ByteArrayOutputStream();
        byte[] chunk = new byte[4096];
        int total = 0;
        int count;
        while ((count = input.read(chunk)) != -1) {
            if (count == 0) throw invalid("input made no progress");
            if (count > ControlExperimentRecording.MAX_BYTES - total) throw invalid("recording exceeds 512 KiB");
            total += count;
            for (int i = 0; i < count; i++) {
                if (line.size() == ControlExperimentRecording.MAX_BOUNDARY_BYTES)
                    throw invalid("individual record exceeds 64 KiB");
                line.write(chunk[i]);
                if (chunk[i] == '\n') {
                    state.line(line.toByteArray());
                    line.reset();
                }
            }
        }
        if (line.size() != 0) throw invalid("truncated record: every JSON line must end with LF");
        if (!state.sealed) throw invalid("truncated recording: missing START, FINISH, or SEAL");
        return new Report(state);
    }

    private static final class State {
        final MessageDigest digest = ControlExperimentRecording.sha256();
        ControlResponseMetrics.Accumulator metrics;
        String sessionId, domain;
        long segmentId, lastCycle, attempted, omitted;
        double lastSec;
        int operations, bodyBytes;
        boolean finished, sealed, matches;
        Map<String, Double> recorded, replayed, finalFacts;

        void line(byte[] bytes) throws IOException {
            if (sealed) throw invalid("unexpected record after SEAL");
            String json = StandardCharsets.UTF_8.newDecoder()
                    .onMalformedInput(CodingErrorAction.REPORT)
                    .onUnmappableCharacter(CodingErrorAction.REPORT)
                    .decode(ByteBuffer.wrap(bytes)).toString();
            Map<String, Object> record = parse(json);
            String type = text(record, "type");
            if ("SEAL".equals(type)) {
                if (bytes.length > 256) throw invalid("SEAL exceeds its reserved budget");
                keys(record, "type", "sequence", "sha256");
                if (!finished || integer(record, "sequence") != attempted + 2)
                    throw invalid("SEAL is out of order");
                String expected = ControlExperimentRecording.hex(digest.digest());
                if (!expected.equals(text(record, "sha256"))) throw invalid("SHA-256 integrity mismatch");
                sealed = true;
                return;
            }
            if (finished) throw invalid("expected SEAL after FINISH");
            if (metrics == null) {
                if (!"START".equals(type)) throw invalid("first record must be START");
                start(record);
            } else if ("SAMPLE".equals(type) || "EVIDENCE_ONLY".equals(type)) {
                if (operations == ControlExperimentRecording.MAX_OPERATIONS)
                    throw invalid("too many retained operations");
                bodyBytes += bytes.length;
                if (bodyBytes > ControlExperimentRecording.MAX_BODY_BYTES)
                    throw invalid("operation byte budget exceeded");
                operation(record, type);
            } else if ("FINISH".equals(type)) {
                if (bytes.length > ControlExperimentRecording.MAX_BOUNDARY_BYTES - 256)
                    throw invalid("FINISH exceeds its reserved budget");
                finish(record);
            } else throw invalid("unexpected record type " + type);
            digest.update(bytes);
        }

        private void start(Map<String, Object> r) throws IOException {
            keys(r, "type", "sequence", "cycle", "processingSec", "format", "version", "sessionId",
                    "segmentId", "domain", "topology", "clockSpan", "robotConfigurationRevision",
                    "codeRevision", "acquisitionTime", "initialMeasurementObservationTime",
                    "plantPhysicalUnitName", "targetMapping", "nativeEvidenceUnits", "target",
                    "initialMeasurement", "experimentRange", "controllerCandidate", "experimentRequest",
                    "initialReadbacks", "acceptedReadbacks");
            if (!ControlExperimentRecording.FORMAT.equals(text(r, "format"))
                    || integer(r, "version") != ControlExperimentRecording.VERSION
                    || integer(r, "sequence") != 0) throw invalid("unsupported format/version/start sequence");
            sessionId = text(r, "sessionId"); segmentId = integer(r, "segmentId");
            if (sessionId.trim().isEmpty() || segmentId <= 0) throw invalid("invalid session/segment identity");
            domain = text(r, "domain");
            if (!"VELOCITY".equals(domain) && !"POSITION".equals(domain)) throw invalid("unknown metric domain");
            if (text(r, "topology").trim().isEmpty()) throw invalid("missing topology");
            if (!"ONE_RESET_EPOCH".equals(text(r, "clockSpan"))) throw invalid("unsupported clock span");
            for (String key : Arrays.asList("robotConfigurationRevision", "codeRevision", "acquisitionTime",
                    "initialMeasurementObservationTime", "plantPhysicalUnitName", "targetMapping"))
                unrecorded(r, key);
            String units = text(r, "nativeEvidenceUnits");
            if (!Arrays.asList("UNRECORDED", "ENCODER_TICKS", "ENCODER_TICKS_PER_SECOND").contains(units))
                throw invalid("unknown native evidence units");
            lastCycle = integer(r, "cycle"); lastSec = finite(r, "processingSec");
            if (lastCycle < 0) throw invalid("negative loop cycle");
            double target = finite(r, "target");
            double initial = number(r.get("initialMeasurement"));
            List<?> range = list(r, "experimentRange");
            if (range.size() != 2) throw invalid("experiment range must have two endpoints");
            double min = number(range.get(0)), max = number(range.get(1));
            if (!Double.isFinite(min) || !Double.isFinite(max) || min > max || target < min || target > max)
                throw invalid("target/range is not finite and coherent");
            if (numbers(r, "controllerCandidate").isEmpty()) throw invalid("empty controller candidate");
            numbers(r, "experimentRequest");
            readbacks(r, "initialReadbacks"); readbacks(r, "acceptedReadbacks");
            metrics = "VELOCITY".equals(domain)
                    ? new ControlResponseMetrics.Velocity(target, lastSec, initial)
                    : new ControlResponseMetrics.Position(target, lastSec, initial);
        }

        private void operation(Map<String, Object> r, String type) throws IOException {
            if ("SAMPLE".equals(type)) keys(r, "type", "sequence", "cycle", "processingSec",
                    "acquisitionTime", "measurement", "atTarget", "evidence");
            else keys(r, "type", "sequence", "cycle", "processingSec", "acquisitionTime", "evidence");
            if (integer(r, "sequence") != operations + 1L) throw invalid("duplicate or missing operation sequence");
            time(r); unrecorded(r, "acquisitionTime");
            ControlTuningModel.Evidence evidence = evidence(object(r, "evidence"));
            if ("EVIDENCE_ONLY".equals(type)) metrics.retainEvidence(evidence);
            else {
                double measurement = finite(r, "measurement");
                boolean atTarget = bool(r, "atTarget");
                if (metrics instanceof ControlResponseMetrics.Velocity)
                    ((ControlResponseMetrics.Velocity) metrics).update(lastSec, measurement, atTarget, evidence);
                else ((ControlResponseMetrics.Position) metrics).update(lastSec, measurement, atTarget, evidence);
            }
            operations++;
        }

        private void finish(Map<String, Object> r) throws IOException {
            keys(r, "type", "sequence", "cycle", "processingSec", "terminationReason", "transition",
                    "attemptedOperations", "retainedOperations", "omittedOperations",
                    "firstOmittedProcessingSec", "lastOmittedProcessingSec", "responseMetrics",
                    "finalPlantFacts", "evidence");
            attempted = integer(r, "attemptedOperations"); omitted = integer(r, "omittedOperations");
            if (attempted < operations || attempted > Long.MAX_VALUE - 2
                    || omitted != attempted - operations
                    || integer(r, "retainedOperations") != operations
                    || integer(r, "sequence") != attempted + 1)
                throw invalid("inconsistent omission/sequence counts");
            double firstMissing = number(r.get("firstOmittedProcessingSec"));
            double lastMissing = number(r.get("lastOmittedProcessingSec"));
            double ended = finite(r, "processingSec");
            if (omitted == 0 ? !Double.isNaN(firstMissing) || !Double.isNaN(lastMissing)
                    : !Double.isFinite(firstMissing) || !Double.isFinite(lastMissing)
                    || firstMissing < lastSec || lastMissing < firstMissing || lastMissing > ended)
                throw invalid("invalid omission timing");
            time(r);
            if (text(r, "terminationReason").trim().isEmpty()) throw invalid("missing termination reason");
            try { ControlExperimentHistory.Transition.valueOf(text(r, "transition")); }
            catch (IllegalArgumentException bad) { throw invalid("unknown transition"); }
            recorded = numbers(r, "responseMetrics");
            finalFacts = numbers(r, "finalPlantFacts");
            keys(finalFacts, "finalRequestedTarget", "finalAppliedTarget", "finalMeasurement");
            Map<String, String> finalEvidence = strings(r, "evidence");
            metrics.finish(lastSec);
            replayed = metrics instanceof ControlResponseMetrics.Velocity
                    ? ((ControlResponseMetrics.Velocity) metrics).snapshot()
                    : ((ControlResponseMetrics.Position) metrics).snapshot();
            matches = sameNumbers(recorded, replayed) && finalEvidence.equals(metrics.evidenceSnapshot());
            finished = true;
        }

        private void time(Map<String, Object> r) throws IOException {
            long cycle = integer(r, "cycle"); double now = finite(r, "processingSec");
            if (cycle < lastCycle || now < lastSec || (cycle == lastCycle && now != lastSec))
                throw invalid("processing time/cycle order changed or crossed an unsupported reset");
            lastCycle = cycle; lastSec = now;
        }
    }

    private static boolean sameNumbers(Map<String, Double> a, Map<String, Double> b) {
        if (!a.keySet().equals(b.keySet())) return false;
        for (String key : a.keySet()) if (Double.compare(a.get(key), b.get(key)) != 0) return false;
        return true;
    }

    private static ControlTuningModel.Evidence evidence(Map<String, Object> r) throws IOException {
        keys(r, "numeric", "text", "outputLimitedAvailable", "outputLimited");
        boolean available = bool(r, "outputLimitedAvailable"), limited = bool(r, "outputLimited");
        if (!available && limited) throw invalid("unavailable output limit cannot claim true");
        return new ControlTuningModel.Evidence(numbers(r, "numeric"), strings(r, "text"), available, limited);
    }

    private static void readbacks(Map<String, Object> r, String key) throws IOException {
        List<?> values = list(r, key);
        if (values.isEmpty()) throw invalid("readbacks must not be empty");
        for (Object raw : values) {
            Map<String, Object> value = asObject(raw);
            keys(value, "owner", "parameters", "fixedFacts");
            if (text(value, "owner").trim().isEmpty()) throw invalid("missing readback owner");
            numbers(value, "parameters"); strings(value, "fixedFacts");
        }
    }

    /** Gson streaming parsing plus explicit duplicate-name/depth checks, not lenient object binding. */
    private static Map<String, Object> parse(String text) throws IOException {
        try (JsonReader reader = new JsonReader(new StringReader(text))) {
            reader.setLenient(false);
            Map<String, Object> result = asObject(value(reader, 0));
            if (reader.peek() != JsonToken.END_DOCUMENT) throw invalid("extra JSON after record");
            return result;
        } catch (IllegalStateException | NumberFormatException bad) {
            throw invalid("malformed JSON record");
        }
    }

    private static Object value(JsonReader reader, int depth) throws IOException {
        if (depth > 8) throw invalid("JSON nesting exceeds recording schema");
        switch (reader.peek()) {
            case BEGIN_OBJECT:
                Map<String, Object> object = new LinkedHashMap<String, Object>();
                reader.beginObject();
                while (reader.hasNext()) {
                    String name = reader.nextName();
                    if (name.trim().isEmpty() || object.containsKey(name)) throw invalid("empty/duplicate JSON field");
                    object.put(name, value(reader, depth + 1));
                }
                reader.endObject(); return object;
            case BEGIN_ARRAY:
                List<Object> array = new ArrayList<Object>(); reader.beginArray();
                while (reader.hasNext()) array.add(value(reader, depth + 1));
                reader.endArray(); return array;
            case NUMBER: return new JsonNumber(reader.nextString());
            case STRING: return reader.nextString();
            case BOOLEAN: return reader.nextBoolean();
            default: throw invalid("null or unexpected JSON token");
        }
    }

    private static final class JsonNumber {
        final String value;
        JsonNumber(String value) { this.value = value; }
    }

    private static void keys(Map<String, ?> value, String... expected) throws IOException {
        if (!value.keySet().equals(new HashSet<String>(Arrays.asList(expected))))
            throw invalid("unexpected or missing schema fields");
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> asObject(Object value) throws IOException {
        if (!(value instanceof Map)) throw invalid("expected JSON object");
        return (Map<String, Object>) value;
    }

    private static Map<String, Object> object(Map<String, Object> r, String key) throws IOException {
        return asObject(r.get(key));
    }

    private static List<?> list(Map<String, Object> r, String key) throws IOException {
        Object value = r.get(key);
        if (!(value instanceof List)) throw invalid("expected array: " + key);
        return (List<?>) value;
    }

    private static String text(Map<String, Object> r, String key) throws IOException {
        if (!(r.get(key) instanceof String)) throw invalid("expected text: " + key);
        return (String) r.get(key);
    }

    private static boolean bool(Map<String, Object> r, String key) throws IOException {
        if (!(r.get(key) instanceof Boolean)) throw invalid("expected boolean: " + key);
        return (Boolean) r.get(key);
    }

    private static long integer(Map<String, Object> r, String key) throws IOException {
        if (!(r.get(key) instanceof JsonNumber)) throw invalid("expected integer: " + key);
        try { return Long.parseLong(((JsonNumber) r.get(key)).value); }
        catch (NumberFormatException bad) { throw invalid("invalid integer: " + key); }
    }

    private static double finite(Map<String, Object> r, String key) throws IOException {
        double value = number(r.get(key));
        if (!Double.isFinite(value)) throw invalid("expected finite number: " + key);
        return value;
    }

    private static double number(Object value) throws IOException {
        if (value instanceof JsonNumber) {
            double parsed;
            try { parsed = Double.parseDouble(((JsonNumber) value).value); }
            catch (NumberFormatException bad) { throw invalid("invalid number"); }
            if (!Double.isFinite(parsed)) throw invalid("non-finite numeric literal must use its state token");
            return parsed;
        }
        if ("NAN".equals(value)) return Double.NaN;
        if ("POSITIVE_INFINITY".equals(value)) return Double.POSITIVE_INFINITY;
        if ("NEGATIVE_INFINITY".equals(value)) return Double.NEGATIVE_INFINITY;
        throw invalid("expected number or explicit non-finite state");
    }

    private static Map<String, Double> numbers(Map<String, Object> r, String key) throws IOException {
        Map<String, Double> values = new LinkedHashMap<String, Double>();
        for (Map.Entry<String, Object> entry : object(r, key).entrySet())
            values.put(entry.getKey(), number(entry.getValue()));
        return values;
    }

    private static Map<String, String> strings(Map<String, Object> r, String key) throws IOException {
        Map<String, String> values = new LinkedHashMap<String, String>();
        Map<String, Object> raw = object(r, key);
        for (String name : raw.keySet()) values.put(name, text(raw, name));
        return values;
    }

    private static void unrecorded(Map<String, Object> r, String key) throws IOException {
        if (!ControlExperimentRecording.UNRECORDED.equals(text(r, key)))
            throw invalid("unsupported observation/configuration provenance: " + key);
    }

    private static IOException invalid(String message) { return new IOException(message); }
}
