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
import static org.junit.Assert.assertTrue;

/** Tests cycle, lifecycle, ordering, and builder ownership rules for {@link DriveOverlayStack}. */
public final class DriveOverlayStackCycleSafetyTest {

    @Test
    public void completedStackEvaluationIsCachedAndLastEnabledLayerStillWins() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(new DriveSignal(1.0, 2.0, 3.0));
        RecordingBooleanSource firstEnabled = new RecordingBooleanSource(true);
        RecordingBooleanSource secondEnabled = new RecordingBooleanSource(true);
        RecordingOverlay first = new RecordingOverlay(
                new DriveSignal(10.0, 20.0, 30.0), DriveOverlayMask.ALL);
        RecordingOverlay second = new RecordingOverlay(
                new DriveSignal(40.0, 50.0, 60.0), DriveOverlayMask.ALL);
        DriveSource stack = DriveOverlayStack.on(base)
                .add("first", firstEnabled, first, DriveOverlayMask.ALL)
                .add("second", secondEnabled, second, DriveOverlayMask.OMEGA_ONLY)
                .build();

        DriveSignal result = stack.get(time.clock());
        DriveSignal repeated = stack.get(time.clock());

        assertSame(result, repeated);
        assertSignal(10.0, 20.0, 60.0, result);
        assertEquals(1, base.getCount);
        assertEquals(1, firstEnabled.getCount);
        assertEquals(1, secondEnabled.getCount);
        assertEquals(1, first.enableCount);
        assertEquals(1, second.enableCount);
        assertEquals(1, first.getCount);
        assertEquals(1, second.getCount);

        secondEnabled.value = false;
        time.nextCycle(0.02);
        DriveSignal withoutSecond = stack.get(time.clock());
        assertSame(withoutSecond, stack.get(time.clock()));
        assertSignal(10.0, 20.0, 30.0, withoutSecond);
        assertEquals(2, base.getCount);
        assertEquals(2, firstEnabled.getCount);
        assertEquals(2, secondEnabled.getCount);
        assertEquals(1, first.enableCount);
        assertEquals(1, second.enableCount);
        assertEquals(2, first.getCount);
        assertEquals(1, second.getCount);
        assertEquals(1, second.disableCount);
    }

    @Test
    public void stackCommitsCycleOnlyAfterEveryLayerSucceeds() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(DriveSignal.zero());
        RecordingBooleanSource firstEnabled = new RecordingBooleanSource(true);
        RecordingBooleanSource secondEnabled = new RecordingBooleanSource(true);
        RecordingOverlay first = new RecordingOverlay(
                new DriveSignal(0.1, 0.2, 0.3), DriveOverlayMask.ALL);
        RecordingOverlay second = new RecordingOverlay(
                new DriveSignal(0.4, 0.5, 0.6), DriveOverlayMask.ALL);
        second.failuresRemaining = 1;
        DriveSource stack = DriveOverlayStack.on(base)
                .add("first", firstEnabled, first)
                .add("second", secondEnabled, second)
                .build();

        assertThrows(IllegalStateException.class, () -> stack.get(time.clock()));

        DriveSignal recovered = stack.get(time.clock());
        assertSame(recovered, stack.get(time.clock()));
        assertSignal(0.4, 0.5, 0.6, recovered);
        assertEquals(2, base.getCount);
        assertEquals(2, firstEnabled.getCount);
        assertEquals(2, secondEnabled.getCount);
        assertEquals(1, first.enableCount);
        assertEquals(1, second.enableCount);
        assertEquals(2, first.getCount);
        assertEquals(2, second.getCount);
    }

    @Test
    public void resetClearsCacheAndActivationThenCascadesOnlyToStructuralSources() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(DriveSignal.zero());
        RecordingBooleanSource firstEnabled = new RecordingBooleanSource(true);
        RecordingBooleanSource secondEnabled = new RecordingBooleanSource(false);
        RecordingOverlay first = new RecordingOverlay(
                new DriveSignal(0.1, 0.2, 0.3), DriveOverlayMask.ALL);
        RecordingOverlay second = new RecordingOverlay(
                new DriveSignal(0.4, 0.5, 0.6), DriveOverlayMask.ALL);
        DriveSource stack = DriveOverlayStack.on(base)
                .add("first", firstEnabled, first)
                .add("second", secondEnabled, second)
                .build();
        stack.get(time.clock());
        secondEnabled.value = true;

        stack.reset();

        assertEquals(1, base.resetCount);
        assertEquals(1, firstEnabled.resetCount);
        assertEquals(1, secondEnabled.resetCount);
        assertEquals(0, first.disableCount);
        assertEquals(0, second.disableCount);
        CapturingDebugSink debug = new CapturingDebugSink();
        stack.debugDump(debug, "drive");
        assertSame(DriveSignal.zero(), debug.values.get("drive.lastBase"));
        assertSame(DriveSignal.zero(), debug.values.get("drive.lastOut"));
        assertEquals(Boolean.FALSE, debug.values.get("drive.layer[0].enabled"));
        assertEquals(
                DriveOverlayMask.NONE.toString(),
                debug.values.get("drive.layer[0].effectiveMask"));
        assertSame(DriveOverlayOutput.zero(), debug.values.get("drive.layer[0].lastOut"));
        assertEquals(Boolean.FALSE, debug.values.get("drive.layer[1].enabled"));
        assertEquals(
                DriveOverlayMask.NONE.toString(),
                debug.values.get("drive.layer[1].effectiveMask"));
        assertSame(DriveOverlayOutput.zero(), debug.values.get("drive.layer[1].lastOut"));

        // reset permits a fresh evaluation even though LoopClock is still in the same cycle.
        stack.get(time.clock());
        assertEquals(2, base.getCount);
        assertEquals(2, firstEnabled.getCount);
        assertEquals(2, secondEnabled.getCount);
        assertEquals(2, first.enableCount);
        assertEquals(1, second.enableCount);
        assertEquals(2, first.getCount);
        assertEquals(1, second.getCount);
    }

    @Test
    public void builderIsSingleUseForEmptyAndPopulatedStacks() {
        RecordingDriveSource base = new RecordingDriveSource(DriveSignal.zero());
        DriveOverlayStack.Builder empty = DriveOverlayStack.on(base);
        assertSame(base, empty.build());
        IllegalStateException emptySecondBuild =
                assertThrows(IllegalStateException.class, empty::build);
        assertTrue(emptySecondBuild.getMessage().contains("single-use"));
        assertTrue(emptySecondBuild.getMessage().contains("new builder"));

        DriveOverlayStack.Builder populated = DriveOverlayStack.on(base)
                .add(new RecordingBooleanSource(true),
                        new RecordingOverlay(DriveSignal.zero(), DriveOverlayMask.NONE));
        populated.build();

        IllegalStateException secondBuild =
                assertThrows(IllegalStateException.class, populated::build);
        assertTrue(secondBuild.getMessage().contains("already been built"));
        IllegalStateException addAfterBuild = assertThrows(
                IllegalStateException.class,
                () -> populated.add(
                        new RecordingBooleanSource(true),
                        new RecordingOverlay(DriveSignal.zero(), DriveOverlayMask.NONE)));
        assertTrue(addAfterBuild.getMessage().contains("already been built"));
    }

    @Test
    public void builderRejectsDuplicateOverlayIdentityBeforeItCanBeSharedByLayers() {
        RecordingDriveSource base = new RecordingDriveSource(DriveSignal.zero());
        RecordingOverlay shared = new RecordingOverlay(DriveSignal.zero(), DriveOverlayMask.NONE);
        DriveOverlayStack.Builder builder = DriveOverlayStack.on(base)
                .add("first", new RecordingBooleanSource(true), shared);

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> builder.add("second", new RecordingBooleanSource(true), shared));

        assertTrue(failure.getMessage().contains("already owned"));
        assertTrue(failure.getMessage().contains("fresh DriveOverlay"));
        // The rejected call does not poison the builder or add a partial layer.
        builder.build();
    }

    @Test
    public void clocklessCompatibilityReadInvalidatesAPriorStackCycleCache() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingDriveSource base = new RecordingDriveSource(DriveSignal.zero());
        RecordingBooleanSource enabled = new RecordingBooleanSource(false);
        DriveSource stack = DriveOverlayStack.on(base)
                .add(enabled, new RecordingOverlay(DriveSignal.zero(), DriveOverlayMask.NONE))
                .build();

        stack.get(time.clock());
        stack.get(null);
        stack.get(time.clock());

        assertEquals(3, base.getCount);
        assertEquals(3, enabled.getCount);
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
        private final DriveSignal signal;
        private int getCount;
        private int resetCount;

        private RecordingDriveSource(DriveSignal signal) {
            this.signal = signal;
        }

        @Override
        public DriveSignal get(LoopClock clock) {
            getCount++;
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
