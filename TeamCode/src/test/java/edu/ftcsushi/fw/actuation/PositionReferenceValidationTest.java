package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.util.HashMap;
import java.util.Map;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Adversarial finite-value and atomicity coverage for position reference configuration. */
public final class PositionReferenceValidationTest {

    @Test
    public void taskReferenceRejectsNonFiniteBeforeEffectsAndAllowsRetry() {
        for (double invalid : nonFiniteValues()) {
            TaskFixture fixture = new TaskFixture();
            PositionCalibrationTasks.SearchReferenceStep referenceStep =
                    PositionCalibrationTasks.search(fixture.plant)
                            .withPower(0.2)
                            .until(fixture.cue);

            RuntimeException failure = expectRuntime(
                    () -> referenceStep.establishReferenceAt(invalid));

            assertFiniteDiagnostic(failure,
                    "PositionCalibrationTasks.establishReferenceAt(...)",
                    "plantPosition", "plant units", invalid);
            fixture.assertUntouched();

            Task retry = referenceStep.establishReferenceAt(-0.0)
                    .resumeTargeting()
                    .failAfterSec(1.0)
                    .build();
            TaskRunner runner = new TaskRunner();
            runner.enqueue(retry);
            runner.update(fixture.clock.clock());

            assertEquals(TaskOutcome.SUCCESS, retry.getOutcome());
            assertRawEquals(-0.0, fixture.plant.getMeasurement());
            assertEquals(1, fixture.cue.resetCalls);
            assertEquals(1, fixture.cue.sampleCalls);
            assertEquals(1, fixture.source.sampleCalls);
            assertEquals(2, fixture.normalOutput.stopCalls);
            assertEquals(1, fixture.regulator.resetCalls);
            assertEquals(0, fixture.normalOutput.setCalls);
            assertEquals(0, fixture.command.getCalls);
            assertEquals(0, fixture.command.setCalls);
        }
    }

    @Test
    public void taskHoldRejectsNonFiniteBeforeEffectsAndAllowsRetry() {
        for (double invalid : nonFiniteValues()) {
            TaskFixture fixture = new TaskFixture();
            PositionCalibrationTasks.SearchAfterStep afterStep =
                    PositionCalibrationTasks.search(fixture.plant)
                            .withPower(0.2)
                            .until(fixture.cue)
                            .establishReferenceAt(2.0);

            RuntimeException failure = expectRuntime(
                    () -> afterStep.holdAfterReference(invalid));

            assertFiniteDiagnostic(failure,
                    "PositionCalibrationTasks.holdAfterReference(...)",
                    "plantTarget", "plant units", invalid);
            fixture.assertUntouched();

            Task retry = afterStep.holdAfterReference(-0.0)
                    .failAfterSec(1.0)
                    .build();
            TaskRunner runner = new TaskRunner();
            runner.enqueue(retry);
            runner.update(fixture.clock.clock());

            assertEquals(TaskOutcome.SUCCESS, retry.getOutcome());
            assertEquals(2.0, fixture.plant.getMeasurement(), 0.0);
            assertRawEquals(-0.0, fixture.command.value);
            assertEquals(1, fixture.command.getCalls);
            assertEquals(1, fixture.command.setCalls);
            assertEquals(1, fixture.source.sampleCalls);
            assertEquals(2, fixture.normalOutput.stopCalls);
        }
    }

    @Test
    public void retainedTaskAliasesKeepEarlierFiniteReferenceAndHoldAnswers() {
        TaskFixture fixture = new TaskFixture();
        PositionCalibrationTasks.SearchReferenceStep retainedReference =
                PositionCalibrationTasks.search(fixture.plant)
                        .withPower(0.2)
                        .until(fixture.cue);
        PositionCalibrationTasks.SearchAfterStep acceptedReference =
                retainedReference.establishReferenceAt(3.25);

        for (double invalid : nonFiniteValues()) {
            assertFiniteDiagnostic(
                    expectRuntime(() -> retainedReference.establishReferenceAt(invalid)),
                    "PositionCalibrationTasks.establishReferenceAt(...)",
                    "plantPosition", "plant units", invalid);
        }

        PositionCalibrationTasks.SearchTimeoutStep acceptedHold =
                acceptedReference.holdAfterReference(4.5);
        for (double invalid : nonFiniteValues()) {
            assertFiniteDiagnostic(
                    expectRuntime(() -> acceptedReference.holdAfterReference(invalid)),
                    "PositionCalibrationTasks.holdAfterReference(...)",
                    "plantTarget", "plant units", invalid);
        }

        fixture.assertUntouched();

        Task task = acceptedHold.failAfterSec(1.0).build();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);
        runner.update(fixture.clock.clock());

        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertRawEquals(3.25, fixture.plant.getMeasurement());
        assertRawEquals(4.5, fixture.command.value);
    }

    @Test
    public void taskForwardsEveryFiniteReferenceAndHoldBoundaryExactly() {
        for (double value : finiteBoundaries()) {
            TaskFixture fixture = new TaskFixture();
            Task task = PositionCalibrationTasks.search(fixture.plant)
                    .withPower(0.2)
                    .until(fixture.cue)
                    .establishReferenceAt(value)
                    .holdAfterReference(value)
                    .failAfterSec(1.0)
                    .build();
            TaskRunner runner = new TaskRunner();
            runner.enqueue(task);
            runner.update(fixture.clock.clock());

            assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
            assertRawEquals(value, fixture.plant.getMeasurement());
            assertRawEquals(value, fixture.command.value);
        }
    }

    @Test
    public void finiteOutOfRangeHoldsRetainNormalExactAndEquivalentTargetSemantics() {
        RecordingPositionOutput exactOutput = new RecordingPositionOutput();
        RecordingPowerOutput exactSearchOutput = new RecordingPowerOutput();
        MutableScalarSource exactSource = new MutableScalarSource(0.0);
        CountingTarget exactCommand = new CountingTarget(5.0);
        PositionPlant exactPlant = Plants.fromOutputs()
                .deviceManagedPosition(exactOutput, exactSource)
                .searchPowerOutput(exactSearchOutput)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .needsReference("not homed")
                .positionTolerance(0.0)
                .targetFromResolver(PlantTargets.exact(exactCommand))
                .build();
        ManualLoopClock exactClock = new ManualLoopClock();
        Task exactTask = PositionCalibrationTasks.search(exactPlant)
                .withPower(0.2)
                .until(BooleanSource.constant(true))
                .establishReferenceAt(-1.0)
                .holdAfterReference(11.0)
                .failAfterSec(1.0)
                .build();
        TaskRunner exactRunner = new TaskRunner();
        exactRunner.enqueue(exactTask);

        exactRunner.update(exactClock.clock());
        assertEquals(TaskOutcome.SUCCESS, exactTask.getOutcome());
        assertEquals(-1.0, exactPlant.getMeasurement(), 0.0);
        assertEquals(11.0, exactCommand.value, 0.0);
        exactPlant.update(exactClock.clock());

        assertEquals(11.0, exactPlant.getRequestedTarget(), 0.0);
        assertEquals(10.0, exactPlant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE,
                exactPlant.getTargetStatus().kind());
        assertEquals(11.0, exactOutput.commanded, 0.0);

        RecordingPositionOutput equivalentOutput = new RecordingPositionOutput();
        RecordingPowerOutput equivalentSearchOutput = new RecordingPowerOutput();
        MutableScalarSource equivalentSource = new MutableScalarSource(350.0);
        CountingTarget equivalentCommand = new CountingTarget(0.0);
        PlantTargetResolver equivalentResolver = PlantTargets
                .equivalentPositionsOf(equivalentCommand)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        PositionPlant equivalentPlant = Plants.fromOutputs()
                .deviceManagedPosition(equivalentOutput, equivalentSource)
                .searchPowerOutput(equivalentSearchOutput)
                .periodic(360.0)
                .bounded(0.0, 720.0)
                .nativeUnits()
                .needsReference("not indexed")
                .positionTolerance(0.0)
                .targetFromResolver(equivalentResolver)
                .build();
        ManualLoopClock equivalentClock = new ManualLoopClock();
        Task equivalentTask = PositionCalibrationTasks.search(equivalentPlant)
                .withPower(0.2)
                .until(BooleanSource.constant(true))
                .establishReferenceAt(350.0)
                .holdAfterReference(-10.0)
                .failAfterSec(1.0)
                .build();
        TaskRunner equivalentRunner = new TaskRunner();
        equivalentRunner.enqueue(equivalentTask);

        equivalentRunner.update(equivalentClock.clock());
        equivalentPlant.update(equivalentClock.clock());

        assertEquals(TaskOutcome.SUCCESS, equivalentTask.getOutcome());
        assertEquals(-10.0, equivalentCommand.value, 0.0);
        assertEquals(350.0, equivalentPlant.getRequestedTarget(), 0.0);
        assertEquals(350.0, equivalentPlant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED,
                equivalentPlant.getTargetStatus().kind());
        assertEquals(350.0, equivalentOutput.commanded, 0.0);
    }

    @Test
    public void allOutputFactoryShapesRejectNonFiniteStaticReferencesBeforeEffects() {
        RecordingPositionOutput commandedOutput = new RecordingPositionOutput();
        Plants.CommandedPositionReferenceStep commanded = Plants.fromOutputs()
                .commandedPosition(commandedOutput)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(4.0);
        assertStaticReferenceRejections(commanded);
        assertEquals(0, commandedOutput.setCalls);
        assertEquals(0, commandedOutput.stopCalls);
        PositionPlant commandedPlant = commanded
                .plantPositionMapsToNative(2.0, 100.0)
                .targetFromNewCommand(5.0)
                .build();
        commandedPlant.update(new ManualLoopClock().clock());
        assertEquals(112.0, commandedOutput.commanded, 0.0);

        RecordingPositionOutput feedbackOutput = new RecordingPositionOutput();
        MutableScalarSource feedbackSource = new MutableScalarSource(112.0);
        Plants.PositionCoordinateReferenceStep<Plants.TargetStep<PositionPlant>> feedback =
                Plants.fromOutputs()
                .deviceManagedPosition(feedbackOutput, feedbackSource)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(4.0);
        assertStaticReferenceRejections(feedback);
        assertEquals(0, feedbackSource.sampleCalls);
        assertEquals(0, feedbackOutput.setCalls);
        assertEquals(0, feedbackOutput.stopCalls);
        PositionPlant feedbackPlant = feedback
                .plantPositionMapsToNative(2.0, 100.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(5.0)
                .build();
        feedbackPlant.update(new ManualLoopClock().clock());
        assertEquals(5.0, feedbackPlant.getMeasurement(), 0.0);
        assertEquals(112.0, feedbackOutput.commanded, 0.0);

        RecordingPowerOutput regulatedOutput = new RecordingPowerOutput();
        MutableScalarSource regulatedSource = new MutableScalarSource(112.0);
        RecordingRegulator regulator = new RecordingRegulator(0.25);
        Plants.PositionCoordinateReferenceStep<Plants.PositionControlStep> regulated =
                Plants.fromOutputs()
                .regulatedPosition(regulatedOutput, regulatedSource)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(4.0);
        assertStaticReferenceRejections(regulated);
        assertEquals(0, regulatedSource.sampleCalls);
        assertEquals(0, regulator.updateCalls);
        assertEquals(0, regulator.resetCalls);
        assertEquals(0, regulatedOutput.setCalls);
        assertEquals(0, regulatedOutput.stopCalls);
        PositionPlant regulatedPlant = regulated
                .plantPositionMapsToNative(2.0, 100.0)
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(5.0)
                .build();
        regulatedPlant.update(new ManualLoopClock().clock());
        assertEquals(5.0, regulatedPlant.getMeasurement(), 0.0);
        assertEquals(1, regulator.updateCalls);
        assertEquals(0.25, regulatedOutput.commanded, 0.0);
    }

    @Test
    public void outputBuilderSlotsBitPreserveFiniteBoundariesOnNonOverflowingCommands() {
        for (double value : finiteBoundaries()) {
            RecordingPositionOutput plantReferenceOutput = new RecordingPositionOutput();
            PositionPlant plantReference = Plants.fromOutputs()
                    .commandedPosition(plantReferenceOutput)
                    .nonPeriodic()
                    .unbounded()
                    .scaleToNative(1.0)
                    .plantPositionMapsToNative(value, 0.0)
                    .targetFromNewCommand(value)
                    .build();
            plantReference.update(new ManualLoopClock().clock());
            assertDebugReference(plantReference, value, 0.0);
            assertTrue(Double.isFinite(plantReferenceOutput.commanded));

            RecordingPositionOutput nativeReferenceOutput = new RecordingPositionOutput();
            PositionPlant nativeReference = Plants.fromOutputs()
                    .commandedPosition(nativeReferenceOutput)
                    .nonPeriodic()
                    .unbounded()
                    .scaleToNative(1.0)
                    .plantPositionMapsToNative(0.0, value)
                    .targetFromNewCommand(0.0)
                    .build();
            nativeReference.update(new ManualLoopClock().clock());
            assertDebugReference(nativeReference, 0.0, value);
            assertTrue(Double.isFinite(nativeReferenceOutput.commanded));

            RecordingPositionOutput assumeOutput = new RecordingPositionOutput();
            MutableScalarSource assumeSource = new MutableScalarSource(0.0);
            PositionPlant assumedReference = Plants.fromOutputs()
                    .deviceManagedPosition(assumeOutput, assumeSource)
                    .nonPeriodic()
                    .unbounded()
                    .nativeUnits()
                    .assumeCurrentPositionIs(value)
                    .positionTolerance(0.0)
                    .targetFromNewCommand(value)
                    .build();
            assumedReference.update(new ManualLoopClock().clock());
            assertDebugReference(assumedReference, value, 0.0);
            assertTrue(Double.isFinite(assumeOutput.commanded));
        }
    }

    @Test
    public void outputFeedbackFactoriesRejectNonFiniteAssumedReferencesBeforeEffects() {
        RecordingPositionOutput feedbackOutput = new RecordingPositionOutput();
        MutableScalarSource feedbackSource = new MutableScalarSource(200.0);
        Plants.PositionCoordinateReferenceStep<Plants.TargetStep<PositionPlant>> feedback =
                Plants.fromOutputs()
                .deviceManagedPosition(feedbackOutput, feedbackSource)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(5.0);
        assertAssumeReferenceRejections(feedback);
        assertEquals(0, feedbackSource.sampleCalls);
        assertEquals(0, feedbackOutput.setCalls);
        assertEquals(0, feedbackOutput.stopCalls);
        PositionPlant feedbackPlant = feedback.assumeCurrentPositionIs(7.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(8.0)
                .build();
        feedbackPlant.update(new ManualLoopClock().clock());
        assertEquals(7.0, feedbackPlant.getMeasurement(), 0.0);
        assertEquals(205.0, feedbackOutput.commanded, 0.0);

        RecordingPowerOutput regulatedOutput = new RecordingPowerOutput();
        MutableScalarSource regulatedSource = new MutableScalarSource(200.0);
        RecordingRegulator regulator = new RecordingRegulator(0.25);
        Plants.PositionCoordinateReferenceStep<Plants.PositionControlStep> regulated =
                Plants.fromOutputs()
                .regulatedPosition(regulatedOutput, regulatedSource)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(5.0);
        assertAssumeReferenceRejections(regulated);
        assertEquals(0, regulatedSource.sampleCalls);
        assertEquals(0, regulator.updateCalls);
        assertEquals(0, regulator.resetCalls);
        assertEquals(0, regulatedOutput.setCalls);
        assertEquals(0, regulatedOutput.stopCalls);
        PositionPlant regulatedPlant = regulated.assumeCurrentPositionIs(7.0)
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(8.0)
                .build();
        regulatedPlant.update(new ManualLoopClock().clock());
        assertEquals(7.0, regulatedPlant.getMeasurement(), 0.0);
        assertEquals(1, regulator.updateCalls);
        assertEquals(0.25, regulatedOutput.commanded, 0.0);
    }

    @Test
    public void assumeCurrentWaitsForTheFirstFiniteNativeSample() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        MutableScalarSource source = new MutableScalarSource(0.0);
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, source)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(5.0)
                .assumeCurrentPositionIs(7.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(8.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        for (double unavailable : nonFiniteValues()) {
            source.value = unavailable;
            plant.update(source.sampleCalls == 0 ? clock.clock() : clock.nextCycle(0.02));

            assertFalse(plant.isReferenced());
            assertEquals("reference pending first finite native sample", plant.referenceStatus());
            assertTrue(Double.isNaN(plant.getMeasurement()));
            assertEquals(0, output.setCalls);
        }
        assertEquals(3, output.stopCalls);
        assertEquals(3, source.sampleCalls);

        source.value = 200.0;
        plant.update(clock.nextCycle(0.02));

        assertTrue(plant.isReferenced());
        assertEquals("referenced", plant.referenceStatus());
        assertEquals(7.0, plant.getMeasurement(), 0.0);
        assertEquals(205.0, output.commanded, 0.0);
        assertEquals(1, output.setCalls);
        assertEquals(4, source.sampleCalls);
    }

    @Test
    public void retainedOutputAliasesDoNotOverwriteAcceptedReferenceAnswers() {
        RecordingPositionOutput staticOutput = new RecordingPositionOutput();
        MutableScalarSource staticSource = new MutableScalarSource(112.0);
        Plants.PositionCoordinateReferenceStep<Plants.TargetStep<PositionPlant>> retainedStatic =
                Plants.fromOutputs()
                .deviceManagedPosition(staticOutput, staticSource)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(4.0);
        Plants.PositionToleranceStep<Plants.TargetStep<PositionPlant>> acceptedStatic =
                retainedStatic.plantPositionMapsToNative(2.0, 100.0);

        assertFiniteDiagnostic(
                expectRuntime(() -> retainedStatic.plantPositionMapsToNative(Double.NaN, 200.0)),
                "Plants.plantPositionMapsToNative(...)",
                "plantPosition", "plant units", Double.NaN);
        assertFiniteDiagnostic(
                expectRuntime(() -> retainedStatic.plantPositionMapsToNative(
                        3.0, Double.POSITIVE_INFINITY)),
                "Plants.plantPositionMapsToNative(...)",
                "nativePosition", "native units", Double.POSITIVE_INFINITY);
        assertFiniteDiagnostic(
                expectRuntime(() -> retainedStatic.assumeCurrentPositionIs(Double.NaN)),
                "Plants.assumeCurrentPositionIs(...)",
                "plantPosition", "plant units", Double.NaN);

        PositionPlant staticPlant = acceptedStatic.positionTolerance(0.0)
                .targetFromNewCommand(5.0)
                .build();
        staticPlant.update(new ManualLoopClock().clock());
        assertEquals(5.0, staticPlant.getMeasurement(), 0.0);
        assertEquals(112.0, staticOutput.commanded, 0.0);

        RecordingPositionOutput assumeOutput = new RecordingPositionOutput();
        MutableScalarSource assumeSource = new MutableScalarSource(200.0);
        Plants.PositionCoordinateReferenceStep<Plants.TargetStep<PositionPlant>> retainedAssume =
                Plants.fromOutputs()
                .deviceManagedPosition(assumeOutput, assumeSource)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(5.0);
        Plants.PositionToleranceStep<Plants.TargetStep<PositionPlant>> acceptedAssume =
                retainedAssume.assumeCurrentPositionIs(7.0);
        for (double invalid : nonFiniteValues()) {
            assertFiniteDiagnostic(
                    expectRuntime(() -> retainedAssume.assumeCurrentPositionIs(invalid)),
                    "Plants.assumeCurrentPositionIs(...)",
                    "plantPosition", "plant units", invalid);
        }
        assertFiniteDiagnostic(
                expectRuntime(() -> retainedAssume.plantPositionMapsToNative(
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)),
                "Plants.plantPositionMapsToNative(...)",
                "plantPosition", "plant units", Double.NEGATIVE_INFINITY);

        PositionPlant assumePlant = acceptedAssume.positionTolerance(0.0)
                .targetFromNewCommand(8.0)
                .build();
        assumePlant.update(new ManualLoopClock().clock());
        assertEquals(7.0, assumePlant.getMeasurement(), 0.0);
        assertEquals(205.0, assumeOutput.commanded, 0.0);
    }

    @Test
    public void directReferenceOverloadsRejectBeforeSamplingOrChangingState() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        MutableScalarSource source = new MutableScalarSource(40.0);
        source.onSample = () -> fail("invalid reference must be rejected before source sampling");
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, source)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .needsReference("not homed")
                .positionTolerance(0.0)
                .targetFromNewCommand(4.0)
                .build();

        for (double invalid : nonFiniteValues()) {
            assertFiniteDiagnostic(expectRuntime(() -> plant.establishReferenceAt(invalid)),
                    "PositionPlant.establishReferenceAt(...)",
                    "plantPosition", "plant units", invalid);
            assertFiniteDiagnostic(expectRuntime(() -> plant.establishReferenceAt(
                            invalid, new ManualLoopClock().clock())),
                    "PositionPlant.establishReferenceAt(...)",
                    "plantPosition", "plant units", invalid);
        }

        assertEquals(0, source.sampleCalls);
        assertEquals(0, output.setCalls);
        assertEquals(0, output.stopCalls);
        assertFalse(plant.isReferenced());
        assertTrue(Double.isNaN(plant.getMeasurement()));

        source.onSample = null;
        ManualLoopClock clock = new ManualLoopClock();
        plant.establishReferenceAt(-0.0, clock.clock());
        assertTrue(plant.isReferenced());
        assertRawEquals(-0.0, plant.getMeasurement());
        assertEquals(1, source.sampleCalls);
        assertEquals(0, output.setCalls);
        assertEquals(0, output.stopCalls);
    }

    @Test
    public void directReferenceOverloadsForwardFiniteBoundariesExactly() {
        for (double value : finiteBoundaries()) {
            RecordingPositionOutput clockedOutput = new RecordingPositionOutput();
            MutableScalarSource clockedSource = new MutableScalarSource(42.0);
            PositionPlant clocked = unreferencedPlant(clockedOutput, clockedSource);
            clocked.establishReferenceAt(value, new ManualLoopClock().clock());
            assertRawEquals(value, clocked.getMeasurement());

            RecordingPositionOutput cachedOutput = new RecordingPositionOutput();
            MutableScalarSource cachedSource = new MutableScalarSource(42.0);
            PositionPlant cached = unreferencedPlant(cachedOutput, cachedSource);
            ManualLoopClock cachedClock = new ManualLoopClock();
            cached.update(cachedClock.clock());
            cached.establishReferenceAt(value);
            assertRawEquals(value, cached.getMeasurement());
        }
    }

    @Test
    public void invalidDirectReferencePreservesAnActiveSearchAndItsExactPower() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        RecordingPowerOutput searchOutput = new RecordingPowerOutput();
        MutableScalarSource source = new MutableScalarSource(5.0);
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, source)
                .searchPowerOutput(searchOutput)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .plantPositionMapsToNative(0.0, 0.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(5.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        plant.beginCalibrationSearch(0.25);
        PlantTargetStatus status = plant.getTargetStatus();
        PlantTargetResolution resolution = plant.getTargetResolution();
        double requestedTarget = plant.getRequestedTarget();
        double appliedTarget = plant.getAppliedTarget();
        boolean atTarget = plant.atTarget();

        for (double invalid : nonFiniteValues()) {
            int samplesBefore = source.sampleCalls;
            assertFiniteDiagnostic(expectRuntime(() -> plant.establishReferenceAt(invalid)),
                    "PositionPlant.establishReferenceAt(...)",
                    "plantPosition", "plant units", invalid);
            assertFiniteDiagnostic(expectRuntime(() -> plant.establishReferenceAt(
                            invalid, clock.nextCycle(0.02))),
                    "PositionPlant.establishReferenceAt(...)",
                    "plantPosition", "plant units", invalid);
            assertEquals(samplesBefore, source.sampleCalls);
            assertEquals(1, output.setCalls);
            assertEquals(1, output.stopCalls);
            assertEquals(0, searchOutput.setCalls);
            assertEquals(0, searchOutput.stopCalls);
            assertEquals(5.0, plant.getMeasurement(), 0.0);
            assertTargetStateUnchanged(plant, status, resolution,
                    requestedTarget, appliedTarget, atTarget);
        }

        plant.update(clock.nextCycle(0.02));
        assertEquals(1, searchOutput.setCalls);
        assertRawEquals(0.25, searchOutput.commanded);
        plant.endCalibrationSearch();
        assertEquals(1, searchOutput.stopCalls);
    }

    @Test
    public void periodicDerivedReferenceOverflowIsAtomicAndRetainsRawSample() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        MutableScalarSource source = new MutableScalarSource(721.5);
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, source)
                .periodic(360.0)
                .unbounded()
                .nativeUnits()
                .plantPositionMapsToNative(0.0, 0.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(1080.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        PlantTargetStatus status = plant.getTargetStatus();
        PlantTargetResolution resolution = plant.getTargetResolution();
        double requestedTarget = plant.getRequestedTarget();
        double appliedTarget = plant.getAppliedTarget();
        boolean atTarget = plant.atTarget();

        source.value = Double.MAX_VALUE;
        RuntimeException failure = expectRuntime(() -> plant.establishReferenceAt(
                -Double.MAX_VALUE, clock.nextCycle(0.02)));

        assertPeriodicOverflow(failure);
        assertTrue(plant.isReferenced());
        assertEquals(721.5, plant.getMeasurement(), 0.0);
        assertEquals(1, output.setCalls);
        assertEquals(0, output.stopCalls);
        assertEquals(2, source.sampleCalls);
        assertTargetStateUnchanged(plant, status, resolution,
                requestedTarget, appliedTarget, atTarget);
        CapturingDebugSink debug = new CapturingDebugSink();
        plant.debugDump(debug, "plant");
        assertRawEquals(Double.MAX_VALUE,
                ((Number) debug.values.get("plant.lastNativeMeasurement")).doubleValue());

        source.value = 1081.5;
        LoopClock retryClock = clock.nextCycle(0.02);
        plant.establishReferenceAt(0.0, retryClock);
        assertEquals(1080.0, plant.getMeasurement(), 0.0);
        plant.update(retryClock);

        assertEquals(3, source.sampleCalls);
        assertEquals(1081.5, output.commanded, 0.0);
        assertEquals(1080.0, plant.getMeasurement(), 0.0);
    }

    @Test
    public void periodicOverflowThroughTaskRunnerCancelsAndReleasesWithoutHoldWrite() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        RecordingPowerOutput searchOutput = new RecordingPowerOutput();
        MutableScalarSource source = new MutableScalarSource(721.5);
        CountingTarget command = new CountingTarget(1080.0);
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, source)
                .searchPowerOutput(searchOutput)
                .periodic(360.0)
                .unbounded()
                .nativeUnits()
                .plantPositionMapsToNative(0.0, 0.0)
                .positionTolerance(0.0)
                .targetFromResolver(PlantTargets.exact(command))
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        command.resetCounts();
        source.value = Double.MAX_VALUE;
        Task task = PositionCalibrationTasks.search(plant)
                .withPower(0.2)
                .until(BooleanSource.constant(true))
                .establishReferenceAt(-Double.MAX_VALUE)
                .holdAfterReference(0.0)
                .failAfterSec(1.0)
                .build();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        RuntimeException failure = expectRuntime(
                () -> runner.update(clock.nextCycle(0.02)));

        assertPeriodicOverflow(failure);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(runner.isIdle());
        assertTrue(plant.isReferenced());
        assertEquals(721.5, plant.getMeasurement(), 0.0);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0, searchOutput.setCalls);
        assertEquals(1, searchOutput.stopCalls);
        assertEquals(0, command.getCalls);
        assertEquals(0, command.setCalls);
        assertEquals(1080.0, command.value, 0.0);
        assertEquals(2, source.sampleCalls);
        CapturingDebugSink debug = new CapturingDebugSink();
        plant.debugDump(debug, "plant");
        assertRawEquals(Double.MAX_VALUE,
                ((Number) debug.values.get("plant.lastNativeMeasurement")).doubleValue());

        source.value = 722.5;
        plant.update(clock.nextCycle(0.02));
        assertEquals(0, searchOutput.setCalls);
        assertEquals(2, output.setCalls);
        assertEquals(1080.0, output.commanded, 0.0);
        assertEquals(722.5, plant.getMeasurement(), 0.0);
    }

    private static PositionPlant unreferencedPlant(RecordingPositionOutput output,
                                                   MutableScalarSource source) {
        return Plants.fromOutputs()
                .deviceManagedPosition(output, source)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .needsReference("not homed")
                .positionTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static void assertStaticReferenceRejections(
            Plants.CommandedPositionReferenceStep step) {
        for (double invalid : nonFiniteValues()) {
            assertFiniteDiagnostic(expectRuntime(
                            () -> step.plantPositionMapsToNative(invalid, Double.NaN)),
                    "Plants.plantPositionMapsToNative(...)",
                    "plantPosition", "plant units", invalid);
            assertFiniteDiagnostic(expectRuntime(
                            () -> step.plantPositionMapsToNative(invalid, 10.0)),
                    "Plants.plantPositionMapsToNative(...)",
                    "plantPosition", "plant units", invalid);
            assertFiniteDiagnostic(expectRuntime(
                            () -> step.plantPositionMapsToNative(1.0, invalid)),
                    "Plants.plantPositionMapsToNative(...)",
                    "nativePosition", "native units", invalid);
        }
    }

    private static void assertStaticReferenceRejections(
            Plants.PositionCoordinateReferenceStep<?> step) {
        for (double invalid : nonFiniteValues()) {
            assertFiniteDiagnostic(expectRuntime(
                            () -> step.plantPositionMapsToNative(invalid, Double.NaN)),
                    "Plants.plantPositionMapsToNative(...)",
                    "plantPosition", "plant units", invalid);
            assertFiniteDiagnostic(expectRuntime(
                            () -> step.plantPositionMapsToNative(invalid, 10.0)),
                    "Plants.plantPositionMapsToNative(...)",
                    "plantPosition", "plant units", invalid);
            assertFiniteDiagnostic(expectRuntime(
                            () -> step.plantPositionMapsToNative(1.0, invalid)),
                    "Plants.plantPositionMapsToNative(...)",
                    "nativePosition", "native units", invalid);
        }
    }

    private static void assertAssumeReferenceRejections(
            Plants.PositionCoordinateReferenceStep<?> step) {
        for (double invalid : nonFiniteValues()) {
            assertFiniteDiagnostic(expectRuntime(() -> step.assumeCurrentPositionIs(invalid)),
                    "Plants.assumeCurrentPositionIs(...)",
                    "plantPosition", "plant units", invalid);
        }
    }

    private static void assertFiniteDiagnostic(RuntimeException failure,
                                               String operation,
                                               String parameter,
                                               String units,
                                               double value) {
        assertTrue("expected IllegalArgumentException, got " + failure,
                failure instanceof IllegalArgumentException);
        assertEquals(operation + ": " + parameter + " must be finite in " + units
                        + ", got " + Double.toString(value),
                failure.getMessage());
    }

    private static void assertPeriodicOverflow(RuntimeException failure) {
        assertTrue("expected IllegalStateException, got " + failure,
                failure instanceof IllegalStateException);
        String message = String.valueOf(failure.getMessage());
        assertTrue(message.contains("PositionPlant.establishReferenceAt(...)"));
        assertTrue(message.contains("periodic"));
        assertTrue(message.contains("finite"));
        assertTrue(message.contains(Double.toString(-Double.MAX_VALUE)));
        assertTrue(message.contains(Double.toString(Double.MAX_VALUE)));
        assertTrue(message.contains("360.0"));
    }

    private static void assertTargetStateUnchanged(PositionPlant plant,
                                                   PlantTargetStatus expectedStatus,
                                                   PlantTargetResolution expectedResolution,
                                                   double expectedRequestedTarget,
                                                   double expectedAppliedTarget,
                                                   boolean expectedAtTarget) {
        assertRawEquals(expectedRequestedTarget, plant.getRequestedTarget());
        assertRawEquals(expectedAppliedTarget, plant.getAppliedTarget());
        assertEquals(expectedAtTarget, plant.atTarget());

        PlantTargetStatus actualStatus = plant.getTargetStatus();
        assertEquals(expectedStatus.kind(), actualStatus.kind());
        assertEquals(expectedStatus.message(), actualStatus.message());
        assertEquals(expectedStatus.accepted(), actualStatus.accepted());

        PlantTargetResolution actualResolution = plant.getTargetResolution();
        assertEquals(expectedResolution.hasTarget(), actualResolution.hasTarget());
        if (expectedResolution.hasTarget()) {
            assertRawEquals(expectedResolution.target(), actualResolution.target());
        }
        assertEquals(expectedResolution.kind(), actualResolution.kind());
        assertEquals(expectedResolution.satisfiesIntent(), actualResolution.satisfiesIntent());
        assertEquals(expectedResolution.reason(), actualResolution.reason());
    }

    private static void assertDebugReference(PositionPlant plant,
                                             double expectedPlantReference,
                                             double expectedNativeReference) {
        CapturingDebugSink debug = new CapturingDebugSink();
        plant.debugDump(debug, "plant");
        assertRawEquals(expectedPlantReference,
                ((Number) debug.values.get("plant.plantReference")).doubleValue());
        assertRawEquals(expectedNativeReference,
                ((Number) debug.values.get("plant.nativeReference")).doubleValue());
    }

    private static void assertRawEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    private static double[] nonFiniteValues() {
        return new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY};
    }

    private static double[] finiteBoundaries() {
        return new double[]{-Double.MAX_VALUE, -0.0, 0.0, Double.MAX_VALUE};
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

    private static final class TaskFixture {
        private final RecordingPowerOutput normalOutput = new RecordingPowerOutput();
        private final MutableScalarSource source = new MutableScalarSource(100.0);
        private final RecordingRegulator regulator = new RecordingRegulator(0.25);
        private final CountingTarget command = new CountingTarget(5.0);
        private final CountingCue cue = new CountingCue(true);
        private final PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(normalOutput, source)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .needsReference("not homed")
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromResolver(PlantTargets.exact(command))
                .build();
        private final ManualLoopClock clock = new ManualLoopClock();

        private void assertUntouched() {
            assertEquals(0, cue.resetCalls);
            assertEquals(0, cue.sampleCalls);
            assertEquals(0, source.sampleCalls);
            assertEquals(0, normalOutput.setCalls);
            assertEquals(0, normalOutput.stopCalls);
            assertEquals(0, regulator.updateCalls);
            assertEquals(0, regulator.resetCalls);
            assertEquals(0, command.getCalls);
            assertEquals(0, command.setCalls);
            assertFalse(plant.isReferenced());
            assertTrue(Double.isNaN(plant.getMeasurement()));
        }
    }

    private static final class CountingCue implements BooleanSource {
        private final boolean value;
        private int resetCalls;
        private int sampleCalls;

        private CountingCue(boolean value) {
            this.value = value;
        }

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            sampleCalls++;
            return value;
        }

        @Override
        public void reset() {
            resetCalls++;
        }
    }

    private static final class CountingTarget implements ScalarTarget {
        private double value;
        private int getCalls;
        private int setCalls;

        private CountingTarget(double value) {
            this.value = value;
        }

        @Override
        public void set(double value) {
            setCalls++;
            this.value = value;
        }

        @Override
        public double get() {
            getCalls++;
            return value;
        }

        private void resetCounts() {
            getCalls = 0;
            setCalls = 0;
        }
    }

    private static final class MutableScalarSource implements ScalarSource {
        private double value;
        private int sampleCalls;
        private Runnable onSample;

        private MutableScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            sampleCalls++;
            if (onSample != null) onSample.run();
            return value;
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double commanded = Double.NaN;
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
        private double commanded = Double.NaN;
        private int setCalls;
        private int stopCalls;

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
            commanded = 0.0;
        }
    }

    private static final class RecordingRegulator implements ScalarRegulator {
        private final double output;
        private int updateCalls;
        private int resetCalls;

        private RecordingRegulator(double output) {
            this.output = output;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            updateCalls++;
            return output;
        }

        @Override
        public void reset() {
            resetCalls++;
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
