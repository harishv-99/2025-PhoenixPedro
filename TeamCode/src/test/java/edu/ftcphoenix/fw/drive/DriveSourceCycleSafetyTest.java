package edu.ftcphoenix.fw.drive;

import org.junit.Test;

import java.util.HashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;

/** Tests the per-cycle and reset contracts of {@link DriveSource} composition helpers. */
public final class DriveSourceCycleSafetyTest {

    @Test
    public void overlayWhenCachesOneCompletedEvaluationAndOneLifecycleTransitionPerCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(new DriveSignal(0.1, 0.2, 0.3));
        RecordingBooleanSource enabled = new RecordingBooleanSource(true);
        RecordingOverlay overlay = new RecordingOverlay(
                new DriveSignal(0.8, 0.7, 0.6), DriveOverlayMask.ALL);
        DriveSource composed = base.overlayWhen(enabled, overlay, DriveOverlayMask.OMEGA_ONLY);

        DriveSignal first = composed.get(time.clock());
        DriveSignal repeated = composed.get(time.clock());

        assertSame(first, repeated);
        assertSignal(0.1, 0.2, 0.6, first);
        assertEquals(1, base.getCount);
        assertEquals(1, enabled.getCount);
        assertEquals(1, overlay.enableCount);
        assertEquals(1, overlay.getCount);
        assertEquals(0, overlay.disableCount);

        enabled.value = false;
        time.nextCycle(0.02);
        DriveSignal disabled = composed.get(time.clock());
        assertSame(disabled, composed.get(time.clock()));
        assertSame(base.signal, disabled);
        assertEquals(2, base.getCount);
        assertEquals(2, enabled.getCount);
        assertEquals(1, overlay.getCount);
        assertEquals(1, overlay.disableCount);

        enabled.value = true;
        time.nextCycle(0.02);
        composed.get(time.clock());
        composed.get(time.clock());
        assertEquals(3, base.getCount);
        assertEquals(3, enabled.getCount);
        assertEquals(2, overlay.enableCount);
        assertEquals(2, overlay.getCount);
    }

    @Test
    public void overlayWhenCommitsCycleOnlyAfterACompleteSuccessfulEvaluation() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(new DriveSignal(0.1, 0.2, 0.3));
        RecordingBooleanSource enabled = new RecordingBooleanSource(true);
        RecordingOverlay overlay = new RecordingOverlay(
                new DriveSignal(0.4, 0.5, 0.6), DriveOverlayMask.ALL);
        overlay.failuresRemaining = 1;
        DriveSource composed = base.overlayWhen(enabled, overlay);

        assertThrows(IllegalStateException.class, () -> composed.get(time.clock()));

        DriveSignal recovered = composed.get(time.clock());
        assertSame(recovered, composed.get(time.clock()));
        assertEquals(2, base.getCount);
        assertEquals(2, enabled.getCount);
        assertEquals(1, overlay.enableCount);
        assertEquals(2, overlay.getCount);
        assertEquals(0, overlay.disableCount);
    }

    @Test
    public void overlayWhenResetClearsLocalActivationWithoutSynthesizingDisable() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(new DriveSignal(0.1, 0.2, 0.3));
        RecordingBooleanSource enabled = new RecordingBooleanSource(true);
        RecordingOverlay overlay = new RecordingOverlay(
                new DriveSignal(0.4, 0.5, 0.6), DriveOverlayMask.ALL);
        DriveSource composed = base.overlayWhen(enabled, overlay);
        composed.get(time.clock());

        composed.reset();

        assertEquals(1, base.resetCount);
        assertEquals(1, enabled.resetCount);
        assertEquals(0, overlay.disableCount);
        CapturingDebugSink debug = new CapturingDebugSink();
        composed.debugDump(debug, "drive");
        assertEquals(Boolean.FALSE, debug.values.get("drive.overlay.enabled"));

        // reset invalidates the completed-cycle cache and establishes a fresh activation boundary.
        composed.get(time.clock());
        assertEquals(2, base.getCount);
        assertEquals(2, enabled.getCount);
        assertEquals(2, overlay.enableCount);
        assertEquals(2, overlay.getCount);
        assertEquals(0, overlay.disableCount);
    }

    @Test
    public void sourceAdapterOverlayResetAlsoReachesTheAdaptedSource() {
        RecordingDriveSource base = new RecordingDriveSource(DriveSignal.zero());
        RecordingDriveSource override = new RecordingDriveSource(new DriveSignal(0.3, 0.0, 0.0));
        RecordingBooleanSource enabled = new RecordingBooleanSource(true);
        DriveSource composed = base.overlayWhen(
                enabled, override, DriveOverlayMask.TRANSLATION_ONLY);

        composed.reset();

        assertEquals(1, base.resetCount);
        assertEquals(1, enabled.resetCount);
        assertEquals(1, override.resetCount);
    }

    @Test
    public void pureCompositionResetCascadesAndClearsDiagnosticsWithoutAddingACache() {
        ManualLoopClock time = new ManualLoopClock();

        RecordingDriveSource scaledBase = new RecordingDriveSource(new DriveSignal(0.4, 0.2, 0.1));
        DriveSource scaled = scaledBase.scaled(0.5, 0.25);
        scaled.get(time.clock());
        scaled.get(time.clock());
        assertEquals(2, scaledBase.getCount);
        scaled.reset();
        assertEquals(1, scaledBase.resetCount);
        assertDiagnosticsCleared(scaled, "scaled.lastBase", "scaled.lastOut");

        RecordingDriveSource conditionalBase =
                new RecordingDriveSource(new DriveSignal(0.4, 0.2, 0.1));
        RecordingBooleanSource condition = new RecordingBooleanSource(true);
        DriveSource conditional = conditionalBase.scaledWhen(condition, 0.5, 0.25);
        conditional.get(time.clock());
        conditional.get(time.clock());
        assertEquals(2, conditionalBase.getCount);
        assertEquals(2, condition.getCount);
        conditional.reset();
        assertEquals(1, conditionalBase.resetCount);
        assertEquals(1, condition.resetCount);
        CapturingDebugSink conditionalDebug = new CapturingDebugSink();
        conditional.debugDump(conditionalDebug, "drive");
        assertEquals(Boolean.FALSE, conditionalDebug.values.get("drive.scaledWhen.enabled"));
        assertSame(DriveSignal.zero(), conditionalDebug.values.get("drive.scaledWhen.lastBase"));
        assertSame(DriveSignal.zero(), conditionalDebug.values.get("drive.scaledWhen.lastOut"));

        RecordingDriveSource a = new RecordingDriveSource(new DriveSignal(0.1, 0.2, 0.3));
        RecordingDriveSource b = new RecordingDriveSource(new DriveSignal(0.4, 0.5, 0.6));
        DriveSource blended = a.blendedWith(b, 0.5);
        blended.get(time.clock());
        blended.get(time.clock());
        assertEquals(2, a.getCount);
        assertEquals(2, b.getCount);
        blended.reset();
        assertEquals(1, a.resetCount);
        assertEquals(1, b.resetCount);
        CapturingDebugSink blendedDebug = new CapturingDebugSink();
        blended.debugDump(blendedDebug, "drive");
        assertSame(DriveSignal.zero(), blendedDebug.values.get("drive.blend.lastA"));
        assertSame(DriveSignal.zero(), blendedDebug.values.get("drive.blend.lastB"));
        assertSame(DriveSignal.zero(), blendedDebug.values.get("drive.blend.lastOut"));
    }

    @Test
    public void rateLimitedSourceDoesNotPoisonTheCycleWhenItsUpstreamFails() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(new DriveSignal(0.3, -0.2, 0.1));
        base.failuresRemaining = 1;
        DriveSource limited = base.rateLimited(1.0, 1.0);

        assertThrows(IllegalStateException.class, () -> limited.get(time.clock()));

        DriveSignal recovered = limited.get(time.clock());
        assertSame(recovered, limited.get(time.clock()));
        assertSignal(0.3, -0.2, 0.1, recovered);
        assertEquals(2, base.getCount);
    }

    @Test
    public void clocklessCompatibilityReadsInvalidatePriorOverlayAndLimiterCaches() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource overlayBase = new RecordingDriveSource(DriveSignal.zero());
        RecordingBooleanSource disabled = new RecordingBooleanSource(false);
        DriveSource overlaid = overlayBase.overlayWhen(
                disabled,
                new RecordingOverlay(DriveSignal.zero(), DriveOverlayMask.NONE));

        overlaid.get(time.clock());
        overlaid.get(null);
        overlaid.get(time.clock());
        assertEquals(3, overlayBase.getCount);
        assertEquals(3, disabled.getCount);

        RecordingDriveSource limitedBase = new RecordingDriveSource(DriveSignal.zero());
        DriveSource limited = limitedBase.rateLimited(1.0, 1.0);
        limited.get(time.clock());
        limited.get(null);
        limited.get(time.clock());
        assertEquals(3, limitedBase.getCount);
    }

    private static void assertDiagnosticsCleared(DriveSource source,
                                                 String baseSuffix,
                                                 String outSuffix) {
        CapturingDebugSink debug = new CapturingDebugSink();
        source.debugDump(debug, "drive");
        assertSame(DriveSignal.zero(), debug.values.get("drive." + baseSuffix));
        assertSame(DriveSignal.zero(), debug.values.get("drive." + outSuffix));
    }

    private static void assertSignal(double axial,
                                     double lateral,
                                     double omega,
                                     DriveSignal signal) {
        assertEquals(axial, signal.axial, 0.0);
        assertEquals(lateral, signal.lateral, 0.0);
        assertEquals(omega, signal.omega, 0.0);
    }

    private static final class RecordingDriveSource implements DriveSource {
        private DriveSignal signal;
        private int getCount;
        private int resetCount;
        private int failuresRemaining;

        private RecordingDriveSource(DriveSignal signal) {
            this.signal = signal;
        }

        @Override
        public DriveSignal get(LoopClock clock) {
            getCount++;
            if (failuresRemaining > 0) {
                failuresRemaining--;
                throw new IllegalStateException("test source failure");
            }
            return signal;
        }

        @Override
        public void reset() {
            resetCount++;
        }
    }

    private static final class RecordingBooleanSource implements BooleanSource {
        private boolean value;
        private int getCount;
        private int resetCount;

        private RecordingBooleanSource(boolean value) {
            this.value = value;
        }

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            getCount++;
            return value;
        }

        @Override
        public void reset() {
            resetCount++;
        }
    }

    private static final class RecordingOverlay implements DriveOverlay {
        private final DriveSignal signal;
        private final DriveOverlayMask mask;
        private int getCount;
        private int enableCount;
        private int disableCount;
        private int failuresRemaining;

        private RecordingOverlay(DriveSignal signal, DriveOverlayMask mask) {
            this.signal = signal;
            this.mask = mask;
        }

        @Override
        public DriveOverlayOutput get(LoopClock clock) {
            getCount++;
            if (failuresRemaining > 0) {
                failuresRemaining--;
                throw new IllegalStateException("test overlay failure");
            }
            return new DriveOverlayOutput(signal, mask);
        }

        @Override
        public void onEnable(LoopClock clock) {
            enableCount++;
        }

        @Override
        public void onDisable(LoopClock clock) {
            disableCount++;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new HashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            values.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
