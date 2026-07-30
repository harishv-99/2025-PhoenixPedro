package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused lifecycle and Plant-update ownership coverage for position calibration searches. */
public final class PositionCalibrationTasksTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void ownerPlantPhaseRunsExactlyOnceAcrossStartContinueAndCue() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 7.0);
        ScriptedCondition cue = new ScriptedCondition(events, false, false, true);
        Task search = resumeSearch(plant, cue, 5.0);
        TaskRunner runner = new TaskRunner();
        ManualLoopClock clock = new ManualLoopClock();
        runner.enqueue(search);

        runOwnerCycle(runner, plant, clock.clock());
        runOwnerCycle(runner, plant, clock.nextCycle(0.10));
        runOwnerCycle(runner, plant, clock.nextCycle(0.10));

        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "cue.sample(false)",
                "plant.update.search(-0.2)",
                "cue.sample(false)",
                "plant.update.search(-0.2)",
                "cue.sample(true)",
                "plant.reference.enter(0.0)",
                "plant.reference.exit",
                "plant.end",
                "plant.update.normal(7.0)"
        ), events);
        assertEquals(3, plant.updateCount);
        assertEquals(2, plant.searchWriteCount);
        assertEquals(1, plant.normalWriteCount);
        assertEquals(1, plant.beginCount);
        assertEquals(1, plant.endCount);
        assertEquals(7.0, plant.command.get(), 0.0);
        assertTrue(search.isComplete());
        assertEquals(TaskOutcome.SUCCESS, search.getOutcome());
    }

    @Test
    public void initiallyTrueCueSkipsSearchPulseAndHoldIsVisibleToSameLoopNormalPhase() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 9.0);
        Task search = holdSearch(plant, new ScriptedCondition(events, true), 3.0, 2.0);
        TaskRunner runner = new TaskRunner();
        ManualLoopClock clock = new ManualLoopClock();
        runner.enqueue(search);

        runOwnerCycle(runner, plant, clock.clock());

        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "cue.sample(true)",
                "plant.reference.enter(0.0)",
                "plant.reference.exit",
                "command.set.enter(3.0)",
                "command.set.exit",
                "plant.end",
                "plant.update.normal(3.0)"
        ), events);
        assertEquals(0, plant.searchWriteCount);
        assertEquals(1, plant.updateCount);
        assertEquals(1, plant.normalWriteCount);
        assertEquals(3.0, plant.command.get(), 0.0);
        assertEquals(TaskOutcome.SUCCESS, search.getOutcome());
    }

    @Test
    public void cueWinsAtExactTimeoutBoundary() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 4.0);
        Task search = resumeSearch(plant, new ScriptedCondition(events, false, true), 1.0);
        TaskRunner runner = new TaskRunner();
        ManualLoopClock clock = new ManualLoopClock();
        runner.enqueue(search);

        runOwnerCycle(runner, plant, clock.clock());
        events.clear();
        runOwnerCycle(runner, plant, clock.nextCycle(1.0));

        assertEquals(Arrays.asList(
                "cue.sample(true)",
                "plant.reference.enter(0.0)",
                "plant.reference.exit",
                "plant.end",
                "plant.update.normal(4.0)"
        ), events);
        assertEquals(TaskOutcome.SUCCESS, search.getOutcome());
        assertEquals(1, plant.searchWriteCount);
        assertEquals(2, plant.updateCount);
    }

    @Test
    public void exactTimeoutEndsBeforeOwnerPhaseAndPreservesCommand() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 8.0);
        Task search = resumeSearch(plant, new ScriptedCondition(events, false, false), 1.0);
        TaskRunner runner = new TaskRunner();
        ManualLoopClock clock = new ManualLoopClock();
        runner.enqueue(search);

        runOwnerCycle(runner, plant, clock.clock());
        events.clear();
        runOwnerCycle(runner, plant, clock.nextCycle(1.0));

        assertEquals(Arrays.asList(
                "cue.sample(false)",
                "plant.end",
                "plant.update.normal(8.0)"
        ), events);
        assertEquals(TaskOutcome.TIMEOUT, search.getOutcome());
        assertEquals(8.0, plant.command.get(), 0.0);
        assertEquals(1, plant.searchWriteCount);
        assertEquals(2, plant.updateCount);
    }

    @Test
    public void cancellationEndsBeforeOwnerPhaseAndPreservesCommand() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 6.0);
        Task search = resumeSearch(plant, new ScriptedCondition(events, false), 5.0);
        TaskRunner runner = new TaskRunner();
        ManualLoopClock clock = new ManualLoopClock();
        runner.enqueue(search);

        runOwnerCycle(runner, plant, clock.clock());
        events.clear();
        LoopClock cancelClock = clock.nextCycle(0.10);
        assertTrue(runner.cancelCurrent());
        plant.update(cancelClock);

        assertEquals(Arrays.asList(
                "plant.end",
                "plant.update.normal(6.0)"
        ), events);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(6.0, plant.command.get(), 0.0);
        assertEquals(1, plant.searchWriteCount);
        assertEquals(2, plant.updateCount);
        assertFalse(runner.cancelCurrent());
        assertEquals(1, plant.endCount);
    }

    @Test
    public void freshRetryRetainsCancelledOutcomeAndCanSucceed() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 5.0);
        ManualLoopClock clock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        Task first = resumeSearch(plant, new ScriptedCondition(events, false), 5.0);
        runner.enqueue(first);
        runOwnerCycle(runner, plant, clock.clock());
        assertTrue(runner.cancelCurrent());
        plant.update(clock.nextCycle(0.10));

        RuntimeException restart = expectRuntime(() -> first.start(clock.clock()));
        assertTrue(restart.getMessage().contains("single-use"));

        Task fresh = resumeSearch(plant, new ScriptedCondition(events, true), 5.0);
        runner.enqueue(fresh);
        runOwnerCycle(runner, plant, clock.nextCycle(0.10));

        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, fresh.getOutcome());
        assertEquals(2, plant.beginCount);
        assertEquals(2, plant.endCount);
        assertEquals(3, plant.updateCount);
    }

    @Test
    public void resetAndBeginFailuresDoNotReleaseAnUnacquiredSearch() {
        RuntimeException resetFailure = new IllegalStateException("reset failed");
        List<String> resetEvents = new ArrayList<>();
        LifecyclePlant resetPlant = new LifecyclePlant(resetEvents, 0.0);
        ScriptedCondition resetCue = new ScriptedCondition(resetEvents, false);
        resetCue.resetFailure = resetFailure;
        Task resetTask = resumeSearch(resetPlant, resetCue, 1.0);
        TaskRunner resetRunner = new TaskRunner();
        resetRunner.enqueue(resetTask);

        RuntimeException observedReset = expectRuntime(
                () -> resetRunner.update(new ManualLoopClock().clock()));

        assertSame(resetFailure, observedReset);
        assertEquals(Arrays.asList("cue.reset"), resetEvents);
        assertEquals(0, resetPlant.beginCount);
        assertEquals(0, resetPlant.endCount);
        assertEquals(TaskOutcome.CANCELLED, resetTask.getOutcome());
        assertTrue(resetRunner.isIdle());

        RuntimeException beginFailure = new IllegalStateException("begin failed");
        List<String> beginEvents = new ArrayList<>();
        LifecyclePlant beginPlant = new LifecyclePlant(beginEvents, 0.0);
        beginPlant.beginFailure = beginFailure;
        Task beginTask = resumeSearch(beginPlant, new ScriptedCondition(beginEvents, false), 1.0);
        TaskRunner beginRunner = new TaskRunner();
        beginRunner.enqueue(beginTask);

        RuntimeException observedBegin = expectRuntime(
                () -> beginRunner.update(new ManualLoopClock().clock()));

        assertSame(beginFailure, observedBegin);
        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)"
        ), beginEvents);
        assertEquals(1, beginPlant.beginCount);
        assertEquals(0, beginPlant.endCount);
        assertFalse(beginPlant.searchActive);
        assertEquals(TaskOutcome.CANCELLED, beginTask.getOutcome());
        assertTrue(beginRunner.isIdle());
    }

    @Test
    public void cueFailureReleasesSearchExactlyOnce() {
        RuntimeException cueFailure = new IllegalStateException("cue failed");
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 0.0);
        ScriptedCondition cue = new ScriptedCondition(events, false);
        cue.sampleFailure = cueFailure;
        Task search = resumeSearch(plant, cue, 1.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(search);

        RuntimeException observed = expectRuntime(
                () -> runner.update(new ManualLoopClock().clock()));

        assertSame(cueFailure, observed);
        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "cue.sample.throw",
                "plant.end"
        ), events);
        assertEquals(1, plant.endCount);
        assertFalse(plant.searchActive);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
    }

    @Test
    public void referenceFailureReleasesSearchExactlyOnce() {
        RuntimeException referenceFailure = new IllegalStateException("reference failed");
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 0.0);
        plant.referenceFailure = referenceFailure;
        Task search = resumeSearch(plant, new ScriptedCondition(events, true), 1.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(search);

        RuntimeException observed = expectRuntime(
                () -> runner.update(new ManualLoopClock().clock()));

        assertSame(referenceFailure, observed);
        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "cue.sample(true)",
                "plant.reference.enter(0.0)",
                "plant.end"
        ), events);
        assertEquals(1, plant.endCount);
        assertFalse(plant.searchActive);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
    }

    @Test
    public void holdFailureReleasesSearchExactlyOnceAndPreservesOldCommand() {
        RuntimeException holdFailure = new IllegalStateException("hold failed");
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 11.0);
        plant.command.setFailure = holdFailure;
        plant.command.mutateBeforeSetFailure = true;
        Task search = holdSearch(plant, new ScriptedCondition(events, true), 2.0, 1.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(search);

        RuntimeException observed = expectRuntime(
                () -> runner.update(new ManualLoopClock().clock()));

        assertSame(holdFailure, observed);
        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "cue.sample(true)",
                "plant.reference.enter(0.0)",
                "plant.reference.exit",
                "command.set.enter(2.0)",
                "command.set.enter(11.0)",
                "command.set.exit",
                "plant.end"
        ), events);
        assertEquals(11.0, plant.command.get(), 0.0);
        assertEquals(1, plant.endCount);
        assertFalse(plant.searchActive);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
    }

    @Test
    public void endFailureLeavesSearchTerminalAndCannotStartSequenceContinuation() {
        RuntimeException endFailure = new IllegalStateException("end failed");
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 0.0);
        plant.endFailure = endFailure;
        Task search = resumeSearch(plant, new ScriptedCondition(events, true), 1.0);
        AtomicInteger continuationStarts = new AtomicInteger();
        Task sequence = Tasks.sequence(search, Tasks.runOnce(continuationStarts::incrementAndGet));
        TaskRunner runner = new TaskRunner();
        runner.enqueue(sequence);

        RuntimeException observed = expectRuntime(
                () -> runner.update(new ManualLoopClock().clock()));

        assertSame(endFailure, observed);
        assertEquals(1, plant.endCount);
        assertFalse(plant.searchActive);
        assertEquals(TaskOutcome.SUCCESS, search.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, sequence.getOutcome());
        assertEquals(0, continuationStarts.get());
        assertTrue(runner.isIdle());
        search.cancel();
        assertEquals(1, plant.endCount);
    }

    @Test
    public void cueFailureKeepsPrimaryAndSuppressesThrowingEnd() {
        RuntimeException cueFailure = new IllegalStateException("cue failed");
        RuntimeException endFailure = new IllegalStateException("end failed");
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 0.0);
        plant.endFailure = endFailure;
        ScriptedCondition cue = new ScriptedCondition(events, false);
        cue.sampleFailure = cueFailure;
        Task search = resumeSearch(plant, cue, 1.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(search);

        RuntimeException observed = expectRuntime(
                () -> runner.update(new ManualLoopClock().clock()));

        assertSame(cueFailure, observed);
        assertEquals(1, observed.getSuppressed().length);
        assertSame(endFailure, observed.getSuppressed()[0]);
        assertEquals(1, plant.endCount);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
    }

    @Test
    public void reentrantCancellationDuringBeginReleasesOnlyAfterBeginReturns() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 0.0);
        ScriptedCondition cue = new ScriptedCondition(events, false);
        TaskRunner runner = new TaskRunner();
        Task search = resumeSearch(plant, cue, 1.0);
        plant.onBegin = runner::cancelCurrent;
        runner.enqueue(search);
        ManualLoopClock clock = new ManualLoopClock();

        runner.update(clock.clock());
        plant.update(clock.clock());

        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "plant.end",
                "plant.update.normal(0.0)"
        ), events);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(1, plant.beginCount);
        assertEquals(1, plant.endCount);
        assertEquals(0, plant.searchWriteCount);
        assertFalse(plant.searchActive);
        assertTrue(runner.isIdle());
    }

    @Test
    public void reentrantCancellationDuringReferenceStaysCancelledAndEndsOnce() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 0.0);
        TaskRunner runner = new TaskRunner();
        Task search = resumeSearch(plant, new ScriptedCondition(events, true), 1.0);
        plant.onReference = runner::cancelCurrent;
        runner.enqueue(search);
        ManualLoopClock clock = new ManualLoopClock();

        runner.update(clock.clock());
        plant.update(clock.clock());

        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "cue.sample(true)",
                "plant.reference.enter(0.0)",
                "plant.end",
                "plant.reference.exit",
                "plant.update.normal(0.0)"
        ), events);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(1, plant.endCount);
        assertFalse(plant.searchActive);
        assertTrue(runner.isIdle());
    }

    @Test
    public void reentrantCancellationDuringHoldRestoresCommandAndEndsOnce() {
        List<String> events = new ArrayList<>();
        LifecyclePlant plant = new LifecyclePlant(events, 1.0);
        TaskRunner runner = new TaskRunner();
        Task search = holdSearch(plant, new ScriptedCondition(events, true), 2.0, 1.0);
        plant.command.onSet = runner::cancelCurrent;
        runner.enqueue(search);
        ManualLoopClock clock = new ManualLoopClock();

        runner.update(clock.clock());
        plant.update(clock.clock());

        assertEquals(Arrays.asList(
                "cue.reset",
                "plant.begin.enter(-0.2)",
                "plant.begin.exit",
                "cue.sample(true)",
                "plant.reference.enter(0.0)",
                "plant.reference.exit",
                "command.set.enter(2.0)",
                "plant.end",
                "command.set.exit",
                "command.set.enter(1.0)",
                "command.set.exit",
                "plant.update.normal(1.0)"
        ), events);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(1.0, plant.command.get(), 0.0);
        assertEquals(1, plant.endCount);
        assertFalse(plant.searchActive);
        assertTrue(runner.isIdle());
    }

    @Test
    public void overlappingMappedSearchCannotDisturbOrReleaseFirstOwner() {
        RecordingPositionOutput position = new RecordingPositionOutput();
        RecordingPowerOutput searchOutput = new RecordingPowerOutput();
        ScalarTarget command = ScalarTarget.create(5.0);
        MappedPositionPlant plant = mappedLinearPlant(
                position, searchOutput, clock -> 0.0, command);
        Task first = resumeSearch(plant, BooleanSource.constant(false), 5.0);
        Task second = resumeSearch(plant, BooleanSource.constant(false), 5.0);
        TaskRunner firstRunner = new TaskRunner();
        TaskRunner secondRunner = new TaskRunner();
        ManualLoopClock clock = new ManualLoopClock();
        firstRunner.enqueue(first);
        secondRunner.enqueue(second);

        firstRunner.update(clock.clock());
        RuntimeException overlap = expectRuntime(() -> secondRunner.update(clock.clock()));

        assertTrue(overlap.getMessage().contains("active calibration search"));
        assertEquals(1, position.stopCalls);
        assertEquals(0, searchOutput.setCalls);
        assertEquals(0, searchOutput.stopCalls);
        assertEquals(TaskOutcome.NOT_DONE, first.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, second.getOutcome());

        plant.update(clock.clock());
        assertEquals(1, searchOutput.setCalls);
        assertEquals(-0.2, searchOutput.commanded, 0.0);

        firstRunner.update(clock.nextCycle(0.10));
        plant.update(clock.clock());
        assertEquals(2, searchOutput.setCalls);
        assertEquals(-0.2, searchOutput.commanded, 0.0);

        LoopClock cancelClock = clock.nextCycle(0.10);
        assertTrue(firstRunner.cancelCurrent());
        plant.update(cancelClock);
        assertEquals(1, searchOutput.stopCalls);
        assertEquals(1, position.setCalls);
        assertEquals(5.0, position.commanded, 0.0);
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
    }

    @Test
    public void failedMappedEndClearsOwnershipAndLaterUpdateCannotRefreshSearch() {
        RecordingPositionOutput position = new RecordingPositionOutput();
        RecordingPowerOutput searchOutput = new RecordingPowerOutput();
        RuntimeException stopFailure = new IllegalStateException("search stop failed");
        searchOutput.stopFailure = stopFailure;
        MappedPositionPlant plant = mappedLinearPlant(
                position, searchOutput, clock -> 0.0, ScalarTarget.create(6.0));
        Task search = resumeSearch(plant, BooleanSource.constant(false), 5.0);
        TaskRunner runner = new TaskRunner();
        ManualLoopClock clock = new ManualLoopClock();
        runner.enqueue(search);

        runner.update(clock.clock());
        plant.update(clock.clock());
        assertEquals(1, searchOutput.setCalls);

        LoopClock cancelClock = clock.nextCycle(0.10);
        RuntimeException observed = expectRuntime(runner::cancelCurrent);
        assertSame(stopFailure, observed);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(1, searchOutput.stopCalls);
        search.cancel();
        assertEquals(1, searchOutput.stopCalls);

        plant.update(cancelClock);
        assertEquals(1, searchOutput.setCalls);
        assertEquals(1, position.setCalls);
        assertEquals(6.0, position.commanded, 0.0);

        plant.beginCalibrationSearch(0.4);
        plant.update(clock.nextCycle(0.10));
        assertEquals(2, searchOutput.setCalls);
        assertEquals(0.4, searchOutput.commanded, 0.0);
        assertSame(stopFailure, expectRuntime(plant::endCalibrationSearch));
    }

    @Test
    public void periodicRereferenceUsesCurrentSampleAndSharesItWithOwnerUpdate() {
        RecordingPositionOutput position = new RecordingPositionOutput();
        RecordingPowerOutput searchOutput = new RecordingPowerOutput();
        MutableScalarSource nativeMeasurement = new MutableScalarSource(721.5);
        ScalarTarget command = ScalarTarget.create(1080.0);
        MappedPositionPlant plant = MappedPositionPlant.positionOutput(position, nativeMeasurement)
                .searchPowerOutput(searchOutput)
                .topology(PositionPlant.Topology.PERIODIC, 360.0)
                .range(ScalarRange.unbounded())
                .plantPositionMapsToNative(0.0, 0.0)
                .positionTolerance(0.0)
                .targetedBy(command)
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        plant.update(clock.clock());
        assertEquals(721.5, plant.getMeasurement(), EPSILON);
        assertEquals(1, nativeMeasurement.sampleCount);

        nativeMeasurement.value = 1081.5;
        LoopClock referenceClock = clock.nextCycle(0.10);
        Task search = resumeSearch(plant, BooleanSource.constant(true), 1.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(search);
        runner.update(referenceClock);
        plant.update(referenceClock);

        assertEquals(TaskOutcome.SUCCESS, search.getOutcome());
        assertEquals(1080.0, plant.getMeasurement(), EPSILON);
        assertEquals(2, nativeMeasurement.sampleCount);
        assertEquals(0, searchOutput.setCalls);
        assertEquals(1081.5, position.commanded, EPSILON);
    }

    private static void runOwnerCycle(TaskRunner runner, LifecyclePlant plant, LoopClock clock) {
        int before = plant.updateCount;
        runner.update(clock);
        assertEquals("the Task phase must not update the Plant", before, plant.updateCount);
        plant.update(clock);
        assertEquals("the mechanism owner supplies exactly one Plant phase", before + 1,
                plant.updateCount);
    }

    private static Task resumeSearch(PositionPlant plant, BooleanSource cue, double timeoutSec) {
        return PositionCalibrationTasks.search(plant)
                .withPower(-0.2)
                .until(cue)
                .establishReferenceAt(0.0)
                .resumeTargeting()
                .failAfterSec(timeoutSec)
                .build();
    }

    private static Task holdSearch(PositionPlant plant,
                                   BooleanSource cue,
                                   double holdTarget,
                                   double timeoutSec) {
        return PositionCalibrationTasks.search(plant)
                .withPower(-0.2)
                .until(cue)
                .establishReferenceAt(0.0)
                .holdAfterReference(holdTarget)
                .failAfterSec(timeoutSec)
                .build();
    }

    private static MappedPositionPlant mappedLinearPlant(RecordingPositionOutput position,
                                                         RecordingPowerOutput searchOutput,
                                                         ScalarSource measurement,
                                                         ScalarTarget command) {
        return MappedPositionPlant.positionOutput(position, measurement)
                .searchPowerOutput(searchOutput)
                .positionTolerance(0.0)
                .targetedBy(command)
                .build();
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static final class ScriptedCondition implements BooleanSource {
        private final List<String> events;
        private final boolean[] values;
        private int index;
        private RuntimeException resetFailure;
        private RuntimeException sampleFailure;

        private ScriptedCondition(List<String> events, boolean... values) {
            this.events = events;
            this.values = values;
        }

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            if (sampleFailure != null) {
                events.add("cue.sample.throw");
                throw sampleFailure;
            }
            boolean value = values.length == 0
                    ? false
                    : values[Math.min(index, values.length - 1)];
            index++;
            events.add("cue.sample(" + value + ")");
            return value;
        }

        @Override
        public void reset() {
            events.add("cue.reset");
            index = 0;
            if (resetFailure != null) throw resetFailure;
        }
    }

    private static final class LifecyclePlant implements PositionPlant {
        private final List<String> events;
        private final RecordingTarget command;
        private boolean referenced;
        private boolean searchActive;
        private double searchPower;
        private double appliedTarget;
        private int updateCount;
        private int searchWriteCount;
        private int normalWriteCount;
        private int beginCount;
        private int endCount;
        private RuntimeException beginFailure;
        private RuntimeException referenceFailure;
        private RuntimeException endFailure;
        private Runnable onBegin;
        private Runnable onReference;

        private LifecyclePlant(List<String> events, double initialCommand) {
            this.events = events;
            this.command = new RecordingTarget(events, initialCommand);
            this.appliedTarget = initialCommand;
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (searchActive) {
                searchWriteCount++;
                events.add("plant.update.search(" + searchPower + ")");
            } else {
                normalWriteCount++;
                appliedTarget = command.get();
                events.add("plant.update.normal(" + appliedTarget + ")");
            }
        }

        @Override
        public double getRequestedTarget() {
            return command.get();
        }

        @Override
        public double getAppliedTarget() {
            return appliedTarget;
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return searchActive
                    ? PlantTargetStatus.holdingLast("calibration search active")
                    : PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasCommandTarget() {
            return true;
        }

        @Override
        public ScalarTarget commandTarget() {
            return command;
        }

        @Override
        public Topology topology() {
            return Topology.LINEAR;
        }

        @Override
        public double period() {
            return Double.NaN;
        }

        @Override
        public ScalarRange targetRange() {
            return ScalarRange.unbounded();
        }

        @Override
        public boolean isReferenced() {
            return referenced;
        }

        @Override
        public String referenceStatus() {
            return referenced ? "referenced" : "not referenced";
        }

        @Override
        public void establishReferenceAt(double plantPosition) {
            establishReferenceAt(plantPosition, null);
        }

        @Override
        public void establishReferenceAt(double plantPosition, LoopClock clock) {
            events.add("plant.reference.enter(" + plantPosition + ")");
            if (onReference != null) onReference.run();
            if (referenceFailure != null) throw referenceFailure;
            referenced = true;
            events.add("plant.reference.exit");
        }

        @Override
        public boolean supportsCalibrationSearch() {
            return true;
        }

        @Override
        public void beginCalibrationSearch(double power) {
            beginCount++;
            events.add("plant.begin.enter(" + power + ")");
            if (beginFailure != null) throw beginFailure;
            if (searchActive) throw new IllegalStateException("test search already active");
            if (onBegin != null) onBegin.run();
            searchPower = power;
            searchActive = true;
            events.add("plant.begin.exit");
        }

        @Override
        public void endCalibrationSearch() {
            endCount++;
            searchActive = false;
            events.add("plant.end");
            if (endFailure != null) throw endFailure;
        }

        @Override
        public void stop() {
            searchActive = false;
        }
    }

    private static final class RecordingTarget implements ScalarTarget {
        private final List<String> events;
        private double value;
        private RuntimeException setFailure;
        private boolean mutateBeforeSetFailure;
        private Runnable onSet;

        private RecordingTarget(List<String> events, double value) {
            this.events = events;
            this.value = value;
        }

        @Override
        public void set(double value) {
            events.add("command.set.enter(" + value + ")");
            if (onSet != null) onSet.run();
            RuntimeException failure = setFailure;
            setFailure = null;
            if (failure != null && mutateBeforeSetFailure) this.value = value;
            if (failure != null) throw failure;
            this.value = value;
            events.add("command.set.exit");
        }

        @Override
        public double get() {
            return value;
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double commanded;
        private int setCalls;
        private int stopCalls;

        @Override
        public void setPosition(double position) {
            setCalls++;
            commanded = position;
        }

        @Override
        public double getCommandedPosition() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        private double commanded;
        private int setCalls;
        private int stopCalls;
        private RuntimeException stopFailure;

        @Override
        public void setPower(double power) {
            setCalls++;
            commanded = power;
        }

        @Override
        public double getCommandedPower() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
            if (stopFailure != null) throw stopFailure;
            commanded = 0.0;
        }
    }

    private static final class MutableScalarSource implements ScalarSource {
        private double value;
        private int sampleCount;

        private MutableScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            sampleCount++;
            return value;
        }
    }
}
