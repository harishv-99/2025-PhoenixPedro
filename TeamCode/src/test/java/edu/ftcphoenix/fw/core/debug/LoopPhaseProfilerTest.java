package edu.ftcphoenix.fw.core.debug;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.LongSupplier;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Verifies bounded, diagnostic-only loop phase timing. */
public final class LoopPhaseProfilerTest {

    private static final double EPSILON = 1e-12;
    private static final long MS = 1_000_000L;

    @Test
    public void sequentialBoundariesAttributeOnlyObservedPhaseIntervals() {
        SequenceNanoSource nanos = new SequenceNanoSource(
                1_000 * MS,       // start cycle
                1_002 * MS,       // localization ends
                1_002 * MS + MS / 2, // profiler bookkeeping ends
                1_005 * MS + MS / 2, // tasks end
                1_006 * MS,       // profiler bookkeeping ends
                1_010 * MS);      // cycle observation ends
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();

        profiler.startCycle(clock);
        profiler.finishPhase("localization");
        profiler.finishPhase("tasks");
        profiler.finishCycle(clock);

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        assertTrue(snapshot.isEnabled());
        assertEquals(1L, snapshot.completedCycleCount());
        assertEquals(0L, snapshot.incompleteCycleCount());
        assertEquals(clock.cycle(), snapshot.lastCompletedCycle());
        assertEquals(0.010, snapshot.latestObservedDurationSec(), EPSILON);
        assertEquals(0.010, snapshot.averageObservedDurationSec(), EPSILON);
        assertEquals(0.010, snapshot.maxObservedDurationSec(), EPSILON);
        assertEquals(0.005, snapshot.latestUnattributedDurationSec(), EPSILON);
        assertEquals(0.005, snapshot.averageUnattributedDurationSec(), EPSILON);
        assertEquals(0.005, snapshot.maxUnattributedDurationSec(), EPSILON);

        assertPhase(snapshot, "localization", 1L, clock.cycle(), 0.002, 0.002, 0.002);
        assertPhase(snapshot, "tasks", 1L, clock.cycle(), 0.003, 0.003, 0.003);
        assertEquals(6, nanos.callCount());
    }

    @Test
    public void repeatedNamesAccumulateIntoOneSampleInFirstSeenOrder() {
        SequenceNanoSource nanos = new SequenceNanoSource(
                0,
                2 * MS, 3 * MS,   // A = 2 ms, 1 ms observer gap
                5 * MS, 6 * MS,   // B = 2 ms, 1 ms observer gap
                10 * MS, 11 * MS, // A += 4 ms, 1 ms observer gap
                12 * MS);         // 1 ms trailing unattributed
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();

        profiler.startCycle(clock);
        profiler.finishPhase("A");
        profiler.finishPhase("B");
        profiler.finishPhase("A");
        profiler.finishCycle(clock);

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        assertEquals(Arrays.asList("A", "B"), phaseNames(snapshot));
        assertPhase(snapshot, "A", 1L, clock.cycle(), 0.006, 0.006, 0.006);
        assertPhase(snapshot, "B", 1L, clock.cycle(), 0.002, 0.002, 0.002);
        assertEquals(0.004, snapshot.latestUnattributedDurationSec(), EPSILON);
    }

    @Test
    public void conditionalAbsenceDoesNotInventSamplesAndOnlineStatisticsStayTruthful() {
        SequenceNanoSource nanos = new SequenceNanoSource(
                0,
                2 * MS, 2 * MS,
                5 * MS, 5 * MS,
                5 * MS,
                10 * MS,
                14 * MS, 14 * MS,
                15 * MS);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("A");
        profiler.finishPhase("B");
        profiler.finishCycle(manualClock.clock());

        manualClock.nextCycle(0.020);
        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("A");
        profiler.finishCycle(manualClock.clock());

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        assertEquals(2L, snapshot.completedCycleCount());
        assertEquals(0.005, snapshot.latestObservedDurationSec(), EPSILON);
        assertEquals(0.005, snapshot.averageObservedDurationSec(), EPSILON);
        assertEquals(0.005, snapshot.maxObservedDurationSec(), EPSILON);
        assertEquals(0.001, snapshot.latestUnattributedDurationSec(), EPSILON);
        assertEquals(0.0005, snapshot.averageUnattributedDurationSec(), EPSILON);
        assertEquals(0.001, snapshot.maxUnattributedDurationSec(), EPSILON);
        assertPhase(snapshot, "A", 2L, manualClock.clock().cycle(), 0.004, 0.003, 0.004);
        assertPhase(snapshot, "B", 1L, 1L, 0.003, 0.003, 0.003);
    }

    @Test
    public void unfinishedCycleIsDiscardedAtomicallyAndLaterCycleRecovers() {
        SequenceNanoSource nanos = new SequenceNanoSource(
                0,
                5 * MS, 6 * MS,
                10 * MS,
                12 * MS, 12 * MS,
                13 * MS);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("discarded");
        manualClock.nextCycle(0.020);

        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("recovered");
        profiler.finishCycle(manualClock.clock());

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        assertEquals(1L, snapshot.completedCycleCount());
        assertEquals(1L, snapshot.incompleteCycleCount());
        assertEquals(Arrays.asList("recovered"), phaseNames(snapshot));
        assertPhase(snapshot, "recovered", 1L, 2L, 0.002, 0.002, 0.002);
        assertEquals(0.003, snapshot.latestObservedDurationSec(), EPSILON);
        assertEquals(0.001, snapshot.latestUnattributedDurationSec(), EPSILON);
    }

    @Test
    public void discardedDistinctNamesDoNotConsumeRegistryCapacityOrOrder() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        for (int i = 0; i < LoopPhaseProfiler.MAX_PHASES; i++) {
            profiler.startCycle(manualClock.clock());
            profiler.finishPhase("discarded" + i);
            manualClock.nextCycle(0.020);
        }

        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("kept");
        profiler.finishCycle(manualClock.clock());

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        assertEquals(LoopPhaseProfiler.MAX_PHASES, snapshot.incompleteCycleCount());
        assertEquals(Arrays.asList("kept"), phaseNames(snapshot));
    }

    @Test
    public void orphanAndSameCycleLifecycleCallsFailFastWithoutPoisoningLaterCycles() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.finishPhase("orphan")));
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.finishCycle(manualClock.clock())));

        profiler.startCycle(manualClock.clock());
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.startCycle(manualClock.clock())));

        manualClock.nextCycle(0.020);
        profiler.startCycle(manualClock.clock());
        profiler.finishCycle(manualClock.clock());
        assertEquals(1L, profiler.snapshot().completedCycleCount());

        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.finishCycle(manualClock.clock())));
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.startCycle(manualClock.clock())));
    }

    @Test
    public void discardedAttemptCannotRestartWithinTheSameLoopClockCycle() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        profiler.startCycle(manualClock.clock());
        assertThrows(IllegalArgumentException.class, () -> profiler.finishPhase("  "));
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.startCycle(manualClock.clock())));

        manualClock.nextCycle(0.020);
        profiler.startCycle(manualClock.clock());
        profiler.finishCycle(manualClock.clock());
        assertEquals(1L, profiler.snapshot().completedCycleCount());
        assertEquals(1L, profiler.snapshot().incompleteCycleCount());
    }

    @Test
    public void enabledProfilerRejectsNullClocksAndCycleChangesDuringAnObservation() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        assertThrows(NullPointerException.class, () -> profiler.startCycle(null));
        profiler.startCycle(manualClock.clock());
        manualClock.nextCycle(0.020);
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.finishCycle(manualClock.clock())));

        profiler.reset();
        profiler.startCycle(manualClock.clock());
        assertThrows(NullPointerException.class, () -> profiler.finishCycle(null));
    }

    @Test
    public void aDifferentLoopClockFailsUntilInactiveResetExplicitlyRebinds() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock first = new ManualLoopClock();
        ManualLoopClock second = new ManualLoopClock();

        profiler.startCycle(first.clock());
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.finishCycle(second.clock())));
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.startCycle(second.clock())));

        profiler.reset();
        profiler.startCycle(second.clock());
        profiler.finishCycle(second.clock());
        assertEquals(1L, profiler.snapshot().completedCycleCount());
    }

    @Test
    public void loopClockResetAdvancesCycleAndProfilerResetClearsTheMeasurementWindow() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        manualClock.nextCycle(0.020);
        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("old");
        profiler.finishCycle(manualClock.clock());
        assertEquals(1L, profiler.snapshot().completedCycleCount());

        long cycleBeforeReset = manualClock.clock().cycle();
        manualClock.clock().reset(0.0);
        assertEquals(cycleBeforeReset + 1L, manualClock.clock().cycle());
        profiler.reset();
        LoopPhaseProfiler.Snapshot empty = profiler.snapshot();
        assertEquals(0L, empty.completedCycleCount());
        assertEquals(0L, empty.incompleteCycleCount());
        assertEquals(LoopPhaseProfiler.NO_CYCLE, empty.lastCompletedCycle());
        assertTrue(empty.phases().isEmpty());

        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("new");
        profiler.finishCycle(manualClock.clock());
        assertEquals(Arrays.asList("new"), phaseNames(profiler.snapshot()));
    }

    @Test
    public void resetWhileCycleIsActiveFailsFast() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock clock = new ManualLoopClock();

        profiler.startCycle(clock.clock());
        assertActionable(assertThrows(IllegalStateException.class, profiler::reset));
    }

    @Test
    public void invalidNamesFailFastAndDiscardTheActiveScratchCycle() {
        for (String invalid : new String[] {null, "", "   "}) {
            CountingNanoSource nanos = new CountingNanoSource();
            LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
            ManualLoopClock manualClock = new ManualLoopClock();
            profiler.startCycle(manualClock.clock());

            RuntimeException failure = assertThrows(RuntimeException.class,
                    () -> profiler.finishPhase(invalid));
            assertActionable(failure);

            manualClock.nextCycle(0.020);
            profiler.startCycle(manualClock.clock());
            profiler.finishCycle(manualClock.clock());
            assertTrue(profiler.snapshot().phases().isEmpty());
        }
    }

    @Test
    public void equalNanoReadingsProduceValidZeroDurations() {
        SequenceNanoSource nanos = new SequenceNanoSource(7, 7, 7, 7);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();

        profiler.startCycle(clock);
        profiler.finishPhase("zero");
        profiler.finishCycle(clock);

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        assertEquals(0.0, snapshot.latestObservedDurationSec(), 0.0);
        assertEquals(0.0, snapshot.latestUnattributedDurationSec(), 0.0);
        assertEquals(0.0, phase(snapshot, "zero").latestDurationSec(), 0.0);
    }

    @Test
    public void negativeNanoOriginIsValid() {
        SequenceNanoSource nanos = new SequenceNanoSource(-10, -5);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();

        profiler.startCycle(clock);
        profiler.finishCycle(clock);

        assertEquals(5e-9, profiler.snapshot().latestObservedDurationSec(), EPSILON);
        assertEquals(5e-9, profiler.snapshot().latestUnattributedDurationSec(), EPSILON);
    }

    @Test
    public void signedNanoWrapUsesStandardElapsedSubtraction() {
        SequenceNanoSource nanos = new SequenceNanoSource(Long.MAX_VALUE - 2, Long.MIN_VALUE + 2);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();

        profiler.startCycle(clock);
        profiler.finishCycle(clock);

        assertEquals(5e-9, profiler.snapshot().latestObservedDurationSec(), EPSILON);
    }

    @Test
    public void regressingNanoSourceFailsAndDiscardsScratchState() {
        SequenceNanoSource nanos = new SequenceNanoSource(10, 9, 20, 25);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        profiler.startCycle(manualClock.clock());
        assertActionable(assertThrows(IllegalStateException.class,
                () -> profiler.finishCycle(manualClock.clock())));

        manualClock.nextCycle(0.020);
        profiler.startCycle(manualClock.clock());
        profiler.finishCycle(manualClock.clock());
        assertEquals(1L, profiler.snapshot().completedCycleCount());
        assertEquals(5e-9, profiler.snapshot().latestObservedDurationSec(), EPSILON);
    }

    @Test
    public void profilerReadsDiagnosticTimeOnlyAtDocumentedBoundaries() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();
        CapturingDebugSink debug = new CapturingDebugSink();

        assertEquals(0, nanos.callCount());
        profiler.snapshot();
        profiler.debugDump(debug, "profile");
        assertEquals(0, nanos.callCount());

        profiler.startCycle(clock);
        assertEquals(1, nanos.callCount());
        profiler.finishPhase("work");
        assertEquals(3, nanos.callCount());
        profiler.snapshot();
        assertEquals(3, nanos.callCount());
        profiler.finishCycle(clock);
        assertEquals(4, nanos.callCount());
        profiler.snapshot();
        profiler.debugDump(debug, "profile");
        profiler.reset();
        assertEquals(4, nanos.callCount());
    }

    @Test
    public void disabledProfilerIsSharedInertAndSkipsValidationTimeAndOutput() {
        LongSupplier forbiddenTimeRead = () -> {
            throw new AssertionError("disabled profiler read diagnostic time");
        };
        LoopPhaseProfiler first = LoopPhaseProfiler.createForTest(false, forbiddenTimeRead);
        LoopPhaseProfiler second = LoopPhaseProfiler.createForTest(false, forbiddenTimeRead);
        CapturingDebugSink debug = new CapturingDebugSink();

        assertSame(first, second);

        first.startCycle(null);
        first.finishPhase(null);
        first.finishCycle(null);
        first.reset();
        first.debugDump(debug, "disabled");
        first.debugDump(null, null);

        LoopPhaseProfiler.Snapshot snapshot = first.snapshot();
        assertSame(snapshot, first.snapshot());
        assertSame(snapshot, second.snapshot());
        assertFalse(snapshot.isEnabled());
        assertEquals(0L, snapshot.completedCycleCount());
        assertEquals(0L, snapshot.incompleteCycleCount());
        assertEquals(LoopPhaseProfiler.NO_CYCLE, snapshot.lastCompletedCycle());
        assertEquals(0.0, snapshot.latestObservedDurationSec(), 0.0);
        assertEquals(0.0, snapshot.averageObservedDurationSec(), 0.0);
        assertEquals(0.0, snapshot.maxObservedDurationSec(), 0.0);
        assertEquals(0.0, snapshot.latestUnattributedDurationSec(), 0.0);
        assertEquals(0.0, snapshot.averageUnattributedDurationSec(), 0.0);
        assertEquals(0.0, snapshot.maxUnattributedDurationSec(), 0.0);
        assertTrue(snapshot.phases().isEmpty());
        assertTrue(debug.data.isEmpty());
    }

    @Test
    public void registryAcceptsExactlyThirtyTwoStableNamesAndRejectsTheThirtyThird() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        assertEquals(32, LoopPhaseProfiler.MAX_PHASES);

        for (int i = 0; i < LoopPhaseProfiler.MAX_PHASES; i++) {
            if (i > 0) {
                manualLoopNext(manualClock);
            }
            profiler.startCycle(manualClock.clock());
            profiler.finishPhase("phase" + i);
            profiler.finishCycle(manualClock.clock());
        }

        assertEquals(32, profiler.snapshot().phases().size());
        manualLoopNext(manualClock);
        profiler.startCycle(manualClock.clock());
        RuntimeException failure = assertThrows(RuntimeException.class,
                () -> profiler.finishPhase("phase32"));
        assertActionable(failure);
        assertTrue(failure.getMessage().contains("32"));
        assertEquals(32, profiler.snapshot().phases().size());
    }

    @Test
    public void longRunRetainsConstantPhaseRegistryAndOnlineCountsOnly() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();
        int cycles = 10_000;

        for (int i = 0; i < cycles; i++) {
            if (i > 0) {
                manualLoopNext(manualClock);
            }
            profiler.startCycle(manualClock.clock());
            profiler.finishPhase("stable");
            profiler.finishCycle(manualClock.clock());
        }

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        assertEquals(cycles, snapshot.completedCycleCount());
        assertEquals(1, snapshot.phases().size());
        assertEquals(cycles, phase(snapshot, "stable").sampleCount());
    }

    @Test
    public void snapshotsAreImmutableCopiesAndOlderSnapshotsDoNotDrift() {
        CountingNanoSource nanos = new CountingNanoSource();
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        ManualLoopClock manualClock = new ManualLoopClock();

        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("work");
        profiler.finishCycle(manualClock.clock());
        LoopPhaseProfiler.Snapshot oldSnapshot = profiler.snapshot();

        assertThrows(UnsupportedOperationException.class,
                () -> oldSnapshot.phases().clear());

        manualLoopNext(manualClock);
        profiler.startCycle(manualClock.clock());
        profiler.finishPhase("work");
        profiler.finishPhase("new");
        profiler.finishCycle(manualClock.clock());
        LoopPhaseProfiler.Snapshot newSnapshot = profiler.snapshot();

        assertEquals(1L, oldSnapshot.completedCycleCount());
        assertEquals(1L, phase(oldSnapshot, "work").sampleCount());
        assertEquals(Arrays.asList("work"), phaseNames(oldSnapshot));
        assertEquals(2L, newSnapshot.completedCycleCount());
        assertEquals(2L, phase(newSnapshot, "work").sampleCount());
        assertEquals(Arrays.asList("work", "new"), phaseNames(newSnapshot));
    }

    @Test
    public void debugDumpUsesStableExplicitUnitKeysAndNullableSink() {
        SequenceNanoSource nanos = new SequenceNanoSource(0, 2 * MS, 3 * MS, 5 * MS);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();
        profiler.startCycle(clock);
        profiler.finishPhase("tasks");
        profiler.finishCycle(clock);

        profiler.debugDump(null, "ignored");
        CapturingDebugSink debug = new CapturingDebugSink();
        profiler.debugDump(debug, "loopProfile");

        assertEquals(Boolean.TRUE, debug.data.get("loopProfile.enabled"));
        assertEquals(1L, number(debug, "loopProfile.completedCycleCount").longValue());
        assertEquals(0L, number(debug, "loopProfile.incompleteCycleCount").longValue());
        assertEquals(clock.cycle(), number(debug, "loopProfile.lastCompletedCycle").longValue());
        assertEquals(0.005,
                number(debug, "loopProfile.latestObservedDurationSec").doubleValue(), EPSILON);
        assertEquals(0.005,
                number(debug, "loopProfile.averageObservedDurationSec").doubleValue(), EPSILON);
        assertEquals(0.005,
                number(debug, "loopProfile.maxObservedDurationSec").doubleValue(), EPSILON);
        assertEquals(0.003,
                number(debug, "loopProfile.latestUnattributedDurationSec").doubleValue(), EPSILON);
        assertEquals(0.003,
                number(debug, "loopProfile.averageUnattributedDurationSec").doubleValue(), EPSILON);
        assertEquals(0.003,
                number(debug, "loopProfile.maxUnattributedDurationSec").doubleValue(), EPSILON);
        assertEquals(0.002,
                number(debug, "loopProfile.phase.tasks.latestDurationSec").doubleValue(), EPSILON);
        assertEquals(0.002,
                number(debug, "loopProfile.phase.tasks.averageDurationSec").doubleValue(), EPSILON);
        assertEquals(0.002,
                number(debug, "loopProfile.phase.tasks.maxDurationSec").doubleValue(), EPSILON);
        assertEquals(1L,
                number(debug, "loopProfile.phase.tasks.sampleCount").longValue());
        assertEquals(clock.cycle(),
                number(debug, "loopProfile.phase.tasks.lastObservedCycle").longValue());
        assertFalse(debug.data.containsKey("ignored.enabled"));
    }

    @Test
    public void phoenixShapedCycleMakesTheSimulatedSlowOwnerObvious() {
        SequenceNanoSource nanos = new SequenceNanoSource(
                0,
                1 * MS, 1 * MS,
                2 * MS, 2 * MS,
                3 * MS, 3 * MS,
                4 * MS, 4 * MS,
                24 * MS, 24 * MS,
                26 * MS, 26 * MS,
                29 * MS, 29 * MS,
                30 * MS);
        LoopPhaseProfiler profiler = LoopPhaseProfiler.createForTest(true, nanos);
        LoopClock clock = new ManualLoopClock().clock();

        profiler.startCycle(clock);
        profiler.finishPhase("visionReadiness");
        profiler.finishPhase("localization");
        profiler.finishPhase("targeting");
        profiler.finishPhase("controls");
        profiler.finishPhase("scoring");
        profiler.finishPhase("drive");
        profiler.finishPhase("telemetry");
        profiler.finishCycle(clock);

        LoopPhaseProfiler.Snapshot snapshot = profiler.snapshot();
        double scoringSec = phase(snapshot, "scoring").latestDurationSec();
        assertEquals(0.020, scoringSec, EPSILON);
        for (LoopPhaseProfiler.PhaseTiming phase : snapshot.phases()) {
            if (!phase.name().equals("scoring")) {
                assertTrue("scoring should be visibly slower than " + phase.name(),
                        scoringSec > phase.latestDurationSec());
            }
        }
        assertEquals(0.030, snapshot.latestObservedDurationSec(), EPSILON);
    }

    @Test
    public void publicConstructionSurfaceHasOneFactoryAndNoPublicConstructor() {
        int publicCreateCount = 0;
        for (Method method : LoopPhaseProfiler.class.getDeclaredMethods()) {
            if (method.getName().equals("create") && Modifier.isPublic(method.getModifiers())) {
                publicCreateCount++;
                assertTrue(Modifier.isStatic(method.getModifiers()));
                assertEquals(LoopPhaseProfiler.class, method.getReturnType());
                assertEquals(Arrays.asList(boolean.class),
                        Arrays.asList(method.getParameterTypes()));
            }
            if (method.getName().equals("createForTest")) {
                assertFalse(Modifier.isPublic(method.getModifiers()));
            }
        }
        assertEquals(1, publicCreateCount);
        for (Constructor<?> constructor : LoopPhaseProfiler.class.getDeclaredConstructors()) {
            assertFalse(Modifier.isPublic(constructor.getModifiers()));
        }
    }

    private static void manualLoopNext(ManualLoopClock clock) {
        clock.nextCycle(0.020);
    }

    private static void assertPhase(
            LoopPhaseProfiler.Snapshot snapshot,
            String name,
            long sampleCount,
            long lastObservedCycle,
            double latestSec,
            double averageSec,
            double maxSec) {
        LoopPhaseProfiler.PhaseTiming timing = phase(snapshot, name);
        assertEquals(sampleCount, timing.sampleCount());
        assertEquals(lastObservedCycle, timing.lastObservedCycle());
        assertEquals(latestSec, timing.latestDurationSec(), EPSILON);
        assertEquals(averageSec, timing.averageDurationSec(), EPSILON);
        assertEquals(maxSec, timing.maxDurationSec(), EPSILON);
    }

    private static LoopPhaseProfiler.PhaseTiming phase(
            LoopPhaseProfiler.Snapshot snapshot,
            String name) {
        for (LoopPhaseProfiler.PhaseTiming timing : snapshot.phases()) {
            if (timing.name().equals(name)) {
                return timing;
            }
        }
        throw new AssertionError("Missing phase " + name + " in " + phaseNames(snapshot));
    }

    private static List<String> phaseNames(LoopPhaseProfiler.Snapshot snapshot) {
        List<String> names = new ArrayList<>();
        for (LoopPhaseProfiler.PhaseTiming timing : snapshot.phases()) {
            names.add(timing.name());
        }
        return names;
    }

    private static void assertActionable(RuntimeException failure) {
        assertNotNull(failure.getMessage());
        assertFalse(failure.getMessage().trim().isEmpty());
    }

    private static Number number(CapturingDebugSink debug, String key) {
        Object value = debug.data.get(key);
        assertTrue("Missing numeric debug key " + key + " in " + debug.data,
                value instanceof Number);
        return (Number) value;
    }

    private static final class SequenceNanoSource implements LongSupplier {
        private final long[] values;
        private int index;

        private SequenceNanoSource(long... values) {
            this.values = values.clone();
        }

        @Override
        public long getAsLong() {
            if (index >= values.length) {
                throw new AssertionError("Unexpected diagnostic time read " + (index + 1));
            }
            return values[index++];
        }

        private int callCount() {
            return index;
        }
    }

    private static final class CountingNanoSource implements LongSupplier {
        private long next;
        private int callCount;

        @Override
        public long getAsLong() {
            callCount++;
            return next++;
        }

        private int callCount() {
            return callCount;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> data = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            data.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
