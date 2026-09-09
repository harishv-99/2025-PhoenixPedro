package edu.ftcsushi.fw.integrations.panels;

import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.ResultDownloads;

import static org.junit.Assert.*;

/** Exact live metric-call recording, strict transport decoding, and deliberately bounded replay. */
public final class ControlExperimentRecordingTest {
    @Test public void bothRealAccumulatorsReplayExactInputsAcrossEveryChunkBoundary() throws Exception {
        for (String domain : Arrays.asList("VELOCITY", "POSITION")) {
            Rig rig = completed(domain);
            assertEquals(0, rig.downloads.publishes);
            rig.publish();
            String text = rig.downloads.text;
            ControlExperimentReplay.Report all = read(text);
            assertEquals("COMPLETE_MATCH", all.status);
            assertEquals(domain, all.domain);
            assertEquals("session-λ-🍣", all.sessionId);
            assertEquals(0.1, all.recordedMetrics.get("firstAtTargetSec"), 1e-12);
            assertEquals(0.3, all.recordedMetrics.get("settlingSec"), 1e-12);
            assertEquals(3.0, all.recordedMetrics.get("disturbancePeakAbsError"), 0.0);
            assertEquals(0.3, all.recordedMetrics.get("disturbanceRecoverySec"), 1e-12);
            for (int[] chunks : new int[][]{{1}, {2, 31, 7, 4096, 3}, {65536}}) {
                ControlExperimentReplay.Report decoded = ControlExperimentReplay.replay(
                        chunked(text.getBytes(StandardCharsets.UTF_8), chunks));
                assertEquals(all.status, decoded.status);
                assertEquals(all.recordedMetrics, decoded.recordedMetrics);
                assertEquals(all.replayedMetrics, decoded.replayedMetrics);
            }
        }
    }

    @Test public void missingMeasurementRetainsEvidenceWithoutInventingAMetricUpdate() throws Exception {
        Rig rig = new Rig("POSITION", 0.0, 10.0, 0.0);
        rig.sample(0.1, 1.0, false, evidence(false, false));
        ControlTuningModel.Evidence missing = new ControlTuningModel.Evidence(
                Collections.singletonMap("unavailableFeedback", Double.NaN),
                Collections.singletonMap("condition", "missing\nfeedback"), true, true);
        rig.only(0.2, missing);
        rig.sample(0.3, 3.0, false, evidence(false, false));
        rig.finish(0.4); rig.publish();
        ControlExperimentReplay.Report result = read(rig.downloads.text);
        assertEquals("COMPLETE_MATCH", result.status);
        assertTrue(rig.downloads.text.contains("EVIDENCE_ONLY"));
        assertFalse(rig.downloads.text.contains("\"measurement\":\"NAN\""));
        assertEquals("retainEvidence leaves the previous actual measurement/rate baseline intact",
                10.0, result.replayedMetrics.get("peakMeasuredRate"), 1e-12);
        assertTrue(Double.isNaN(result.replayedMetrics.get("outputLimitedDurationSec")));
    }

    @Test public void precisionSignedZeroAndUnavailableNumbersSurviveWithoutPresentationRounding() throws Exception {
        Rig rig = new Rig("VELOCITY", 0.0, 0.0, -0.0);
        Map<String, Double> numbers = new LinkedHashMap<String, Double>();
        numbers.put("precise", Math.nextUp(0.1234567890123456));
        numbers.put("minusZero", -0.0);
        numbers.put("absent", Double.NaN);
        numbers.put("positiveOverflow", Double.POSITIVE_INFINITY);
        numbers.put("negativeOverflow", Double.NEGATIVE_INFINITY);
        rig.sample(0.0, 0.0, true, new ControlTuningModel.Evidence(numbers,
                Collections.singletonMap("unicode", "🧪 λ\t\n\"quoted\""), false, false));
        rig.finish(0.2); rig.publish();
        ControlExperimentReplay.Report result = read(rig.downloads.text);
        assertEquals("COMPLETE_MATCH", result.status);
        assertEquals(numbers.get("precise"), result.replayedMetrics.get("controller.precise"));
        assertEquals(Double.doubleToLongBits(-0.0), Double.doubleToLongBits(
                result.replayedMetrics.get("controller.minusZero")));
        assertTrue(Double.isNaN(result.replayedMetrics.get("directionalDroop")));
        assertTrue(Double.isNaN(result.replayedMetrics.get("overshoot")));
        assertTrue(rig.downloads.text.contains("\"acquisitionTime\":\"UNRECORDED\""));
        assertTrue(rig.downloads.text.contains("\"codeRevision\":\"UNRECORDED\""));
    }

    @Test public void outputLimitFlagsAndFinalHeldIntervalAreReplayedNotInferredFromEvidenceMap() throws Exception {
        Rig rig = new Rig("VELOCITY", 5.0, 10.0, 10.0);
        rig.sample(5.0, 10.0, true, evidence(true, true));
        rig.sample(6.0, 10.0, true, evidence(true, false));
        rig.sample(7.0, 10.0, true, evidence(true, true));
        rig.finish(7.75); rig.publish();
        ControlExperimentReplay.Report result = read(rig.downloads.text);
        assertEquals("COMPLETE_MATCH", result.status);
        assertEquals(1.75, result.replayedMetrics.get("outputLimitedDurationSec"), 0.0);
        assertEquals(99.0, result.finalPlantFacts.get("finalMeasurement"), 0.0);
    }

    @Test public void operationQuotaPreservesEndingButMakesExactReplayIncomplete() throws Exception {
        Rig rig = new Rig("VELOCITY", 0, 10, 0);
        for (int i = 0; i < 1030; i++) rig.sample(i / 1000.0, 10, true, evidence(false, false));
        rig.finish(2); rig.publish();
        ControlExperimentReplay.Report result = read(rig.downloads.text);
        assertEquals("INCOMPLETE", result.status);
        assertEquals(1024, result.retained);
        assertEquals(6, result.omitted);
        assertTrue(result.replayedMetrics.isEmpty());
        assertEquals(99.0, result.finalPlantFacts.get("finalMeasurement"), 0.0);
        assertTrue(rig.downloads.text.getBytes(StandardCharsets.UTF_8).length <= 512 * 1024);
        assertTrue(rig.downloads.text.contains("\"firstOmittedProcessingSec\":1.024"));
        assertTrue(rig.downloads.text.contains("\"lastOmittedProcessingSec\":1.029"));
    }

    @Test public void encodedByteQuotaDropsNewOperationsAndReservesFinalOutcome() throws Exception {
        Rig rig = new Rig("POSITION", 0, 10, 0);
        ControlTuningModel.Evidence large = new ControlTuningModel.Evidence(
                Collections.<String, Double>emptyMap(), Collections.singletonMap("wide", repeat('x', 20000)), false, false);
        for (int i = 0; i < 40; i++) rig.sample(i / 10.0, 10, true, large);
        rig.finish(4); rig.publish();
        ControlExperimentReplay.Report result = read(rig.downloads.text);
        assertEquals("INCOMPLETE", result.status);
        assertTrue(result.retained < 40);
        assertEquals(40 - result.retained, result.omitted);
        assertTrue(rig.downloads.text.getBytes(StandardCharsets.UTF_8).length <= 512 * 1024);
        assertTrue(rig.downloads.text.contains("B_PRESSED"));
    }

    @Test public void oversizedHeaderAndFooterBecomeUnavailableWithoutChangingMetrics() {
        Rig header = new Rig("VELOCITY", 0, 10, 0, repeat('x', 65536));
        header.sample(0.1, 10, true, evidence(false, false));
        header.finish(0.4); header.publish();
        assertNull(header.downloads.text);
        assertTrue(header.recording.status().contains("unavailable"));
        Rig footer = new Rig("VELOCITY", 0, 10, 0);
        footer.only(0.1, new ControlTuningModel.Evidence(Collections.<String, Double>emptyMap(),
                Collections.singletonMap("large", repeat('x', 65536)), false, false));
        footer.finish(0.4); footer.publish();
        assertNull(footer.downloads.text);
        assertTrue(footer.recording.status().contains("unavailable"));
        assertEquals(10.0, footer.metrics.peakAbsError, 0.0);
    }

    @Test public void clockResetInvalidatesOpenAndPendingSpansWithoutReinterpretingTheirTimes() {
        Rig open = new Rig("VELOCITY", 0, 10, 0);
        open.sample(0.1, 2, false, evidence(false, false));
        open.clock.reset(0.1);
        open.sample(0.2, 3, false, evidence(false, false));
        open.finish(0.3); open.publish();
        assertNull(open.downloads.text);
        Rig pending = completed("POSITION");
        pending.clock.reset(pending.clock.nowSec());
        pending.publish();
        assertNull(pending.downloads.text);
    }

    @Test public void acceptedReplacementAndStopInvalidatePendingPublication() {
        Rig replaced = completed("VELOCITY");
        replaced.recording.begin(replaced.clock, "replacement", 2, "VELOCITY", "TEST",
                ScalarRange.bounded(-100, 100), readbacks(), readbacks(), candidate(),
                Collections.singletonMap("targetVelocity", 10.0), 10, 0);
        replaced.publish();
        assertEquals(0, replaced.downloads.publishes);
        Rig stopped = completed("VELOCITY");
        stopped.recording.stop(); stopped.publish();
        assertEquals(0, stopped.downloads.publishes);
        stopped.recording.stop();
        assertNull(stopped.recording.url());
    }

    @Test public void optionalTransportRuntimeFailuresDoNotEscape() {
        Rig rig = completed("VELOCITY");
        rig.downloads.fail = new IllegalStateException("transport");
        rig.publish();
        assertTrue(rig.recording.status().contains("unavailable"));
        assertNull(rig.recording.url());
        rig.recording.stop();
    }

    @Test public void javaErrorsPropagateRatherThanBeingReportedAsOptionalTransportFailures() {
        Rig rig = completed("VELOCITY");
        AssertionError failure = new AssertionError("transport or test oracle");
        rig.downloads.fail = failure;
        assertSame(failure, assertThrows(AssertionError.class, rig::publish));
        assertSame(failure, assertThrows(AssertionError.class, rig.recording::url));
        assertSame(failure, assertThrows(AssertionError.class, rig.recording::stop));
        rig.recording.stop(); // The terminal boundary was still claimed before its callback.
    }

    @Test public void strictReaderRejectsMalformedUnknownDuplicateTruncatedAndAlteredRecords() throws Exception {
        Rig rig = completed("VELOCITY"); rig.publish();
        String text = rig.downloads.text;
        String[] lines = text.split("\n");
        List<String> bad = new ArrayList<String>();
        bad.add("");
        bad.add(text.substring(0, text.length() - 1));
        bad.add(text.substring(0, text.lastIndexOf("{\"type\":\"SEAL\"")));
        bad.add(text.replace("\"version\":1", "\"version\":2"));
        bad.add(text.replaceFirst("\\{", "{\"extra\":true,"));
        bad.add(text.replaceFirst("\\{", "{\"type\":\"START\","));
        bad.add(text.replace("\"measurement\":9.8", "\"measurement\":9.7"));
        bad.add(lines[0] + "\n" + lines[1] + "\n" + text.substring(text.indexOf('\n') + 1));
        bad.add(text + "{}\n");
        bad.add(repeat('x', 65536) + "\n");
        for (String malformed : bad) assertThrows(IOException.class, () -> read(malformed));
        assertThrows(IOException.class, () -> ControlExperimentReplay.replay(new ByteArrayInputStream(
                new byte[]{(byte) 0xc3, (byte) 0x28, '\n'})));
        assertThrows(IOException.class, () -> ControlExperimentReplay.replay(new ByteArrayInputStream(
                new byte[ControlExperimentRecording.MAX_BYTES + 1])));
    }

    @Test public void integrityValidButChangedMetricIsReportedAsMismatch() throws Exception {
        Rig rig = completed("VELOCITY"); rig.publish();
        String changed = rig.downloads.text.replace("\"peakAbsError\":10.0", "\"peakAbsError\":11.0");
        assertNotEquals(changed, rig.downloads.text);
        assertEquals("MISMATCH", read(reseal(changed)).status);
    }

    @Test public void identityAndTimeSchemaAreCheckedEvenWhenTheHashIsValid() throws Exception {
        Rig rig = completed("VELOCITY"); rig.publish();
        for (String changed : Arrays.asList(
                rig.downloads.text.replace("\"segmentId\":1", "\"segmentId\":0"),
                rig.downloads.text.replace("\"sequence\":1", "\"sequence\":1.5"),
                rig.downloads.text.replace("\"sequence\":2", "\"sequence\":1"),
                rig.downloads.text.replace("\"processingSec\":2.1", "\"processingSec\":1.0"),
                rig.downloads.text.replace("\"processingSec\":2.1", "\"processingSec\":1e309"),
                rig.downloads.text.replace("\"omittedOperations\":0", "\"omittedOperations\":1"),
                rig.downloads.text.replace("\"acquisitionTime\":\"UNRECORDED\"", "\"acquisitionTime\":2.0"))) {
            assertThrows(IOException.class, () -> read(reseal(changed)));
        }
    }

    @Test public void sealBudgetIsCheckedEvenWhenWhitespaceDoesNotChangeItsValidHash() throws Exception {
        Rig rig = completed("VELOCITY"); rig.publish();
        String text = rig.downloads.text;
        int sealStart = text.lastIndexOf("{\"type\":\"SEAL\"");
        String seal = text.substring(sealStart);
        String prefix = text.substring(0, sealStart);
        int sealBytes = seal.getBytes(StandardCharsets.UTF_8).length;
        assertEquals("COMPLETE_MATCH", read(prefix + repeat(' ', 256 - sealBytes) + seal).status);
        assertThrows(IOException.class, () -> read(prefix + repeat(' ', 257 - sealBytes) + seal));
    }

    @Test public void laptopEntryPointReadsAFileAndRejectsIncompleteOrInvalidInput() throws Exception {
        // These retained build outputs are authored synthetic software fixtures, not robot captures.
        // Gradle runs this test in TeamCode, so the CLI can also check its real task/classpath there.
        Path directory = Files.createDirectories(Paths.get("build", "test-control-recordings"));
        Path completeFile = directory.resolve("complete-position.jsonl");
        Path incompleteFile = directory.resolve("incomplete.jsonl");
        Path malformedFile = directory.resolve("malformed.jsonl");
        Rig complete = completed("POSITION"); complete.publish();
        Files.write(completeFile, complete.downloads.text.getBytes(StandardCharsets.UTF_8));
        ControlExperimentReplay.main(new String[]{completeFile.toString()});
        Rig incomplete = new Rig("VELOCITY", 0, 10, 0);
        for (int i = 0; i < 1025; i++) incomplete.sample(i / 1000.0, 10, true, evidence(false, false));
        incomplete.finish(2); incomplete.publish();
        Files.write(incompleteFile, incomplete.downloads.text.getBytes(StandardCharsets.UTF_8));
        assertThrows(IllegalStateException.class,
                () -> ControlExperimentReplay.main(new String[]{incompleteFile.toString()}));
        Files.write(malformedFile, "{}\n".getBytes(StandardCharsets.UTF_8));
        assertThrows(IOException.class,
                () -> ControlExperimentReplay.main(new String[]{malformedFile.toString()}));
        assertThrows(IllegalArgumentException.class, () -> ControlExperimentReplay.main(new String[0]));
    }

    private static Rig completed(String domain) {
        Rig rig = new Rig(domain, 2, 10, 0);
        rig.sample(2.1, 9.8, true, evidence(true, true));
        rig.sample(2.31, 10, true, evidence(true, false));
        rig.sample(2.4, 7, false, evidence(true, false));
        rig.only(2.45, evidence(false, false));
        rig.sample(2.5, 9.9, true, evidence(true, false));
        rig.sample(2.71, 10, true, evidence(true, false));
        rig.finish(2.8);
        return rig;
    }

    private static final class Rig {
        final LoopClock clock = new LoopClock();
        final Downloads downloads = new Downloads();
        final ControlExperimentRecording recording = new ControlExperimentRecording(downloads);
        final ControlResponseMetrics.Accumulator metrics;
        final String domain, session;
        final double start;
        Rig(String domain, double start, double target, double measurement) {
            this(domain, start, target, measurement, "session-λ-🍣");
        }
        Rig(String domain, double start, double target, double measurement, String session) {
            this.domain = domain; this.start = start; this.session = session;
            clock.reset(start);
            metrics = "VELOCITY".equals(domain) ? new ControlResponseMetrics.Velocity(target, start, measurement)
                    : new ControlResponseMetrics.Position(target, start, measurement);
            recording.begin(clock, session, 1, domain, "TEST", ScalarRange.bounded(-100, 100),
                    readbacks(), readbacks(), candidate(), Collections.singletonMap("target", target),
                    target, measurement);
        }
        void sample(double time, double measurement, boolean atTarget, ControlTuningModel.Evidence evidence) {
            clock.update(time);
            if (metrics instanceof ControlResponseMetrics.Velocity)
                ((ControlResponseMetrics.Velocity) metrics).update(time, measurement, atTarget, evidence);
            else ((ControlResponseMetrics.Position) metrics).update(time, measurement, atTarget, evidence);
            recording.sample(clock, measurement, atTarget, evidence);
        }
        void only(double time, ControlTuningModel.Evidence evidence) {
            clock.update(time); metrics.retainEvidence(evidence); recording.evidenceOnly(clock, evidence);
        }
        void finish(double time) {
            clock.update(time); metrics.finish(time);
            Map<String, Double> result = new LinkedHashMap<String, Double>(metrics instanceof ControlResponseMetrics.Velocity
                    ? ((ControlResponseMetrics.Velocity) metrics).snapshot() : ((ControlResponseMetrics.Position) metrics).snapshot());
            result.put("finalRequestedTarget", 10.0); result.put("finalAppliedTarget", 10.0);
            result.put("finalMeasurement", 99.0);
            recording.finish(clock, new ControlExperimentHistory.Record(session, 1, domain,
                    ControlExperimentHistory.Transition.TARGET_CHANGE, candidate(), readbacks(),
                    Collections.singletonMap("target", 10.0), result, metrics.evidenceSnapshot(), start, time, "B_PRESSED"));
        }
        void publish() { recording.publishAfterOutput(clock); }
    }

    private static final class Downloads implements ResultDownloads {
        String text;
        int publishes;
        Throwable fail;
        @Override public boolean publish(String filename, String text) { check(); publishes++; this.text = text; return true; }
        @Override public String url() { check(); return text == null ? null : "http://example.invalid/result"; }
        @Override public void clear() { check(); text = null; }
        void check() {
            if (fail instanceof RuntimeException) throw (RuntimeException) fail;
            if (fail instanceof Error) throw (Error) fail;
        }
    }

    private static ControlTuningModel.Parameters candidate() {
        return new ControlTuningModel.Parameters(Collections.singletonMap("kP", 1.0));
    }
    private static List<ControlTuningModel.Readback> readbacks() {
        return Collections.singletonList(new ControlTuningModel.Readback("owner", candidate()));
    }
    private static ControlTuningModel.Evidence evidence(boolean available, boolean limited) {
        return new ControlTuningModel.Evidence(Collections.<String, Double>emptyMap(),
                Collections.<String, String>emptyMap(), available, limited);
    }
    private static String repeat(char value, int count) {
        char[] chars = new char[count]; Arrays.fill(chars, value); return new String(chars);
    }
    private static ControlExperimentReplay.Report read(String text) throws IOException {
        return ControlExperimentReplay.replay(new ByteArrayInputStream(text.getBytes(StandardCharsets.UTF_8)));
    }
    private static String reseal(String text) {
        int seal = text.lastIndexOf("{\"type\":\"SEAL\"");
        String payload = text.substring(0, seal);
        String tail = text.substring(seal).replaceFirst("\"sha256\":\"[0-9a-f]+\"",
                "\"sha256\":\"" + ControlExperimentRecording.hex(ControlExperimentRecording.sha256()
                        .digest(payload.getBytes(StandardCharsets.UTF_8))) + "\"");
        return payload + tail;
    }
    private static InputStream chunked(byte[] bytes, int[] sizes) {
        return new ByteArrayInputStream(bytes) {
            int index;
            @Override public synchronized int read(byte[] target, int offset, int length) {
                return super.read(target, offset, Math.min(length, sizes[index++ % sizes.length]));
            }
        };
    }
}
