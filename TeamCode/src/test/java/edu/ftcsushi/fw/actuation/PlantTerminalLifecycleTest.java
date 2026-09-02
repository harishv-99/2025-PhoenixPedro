package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that explicit Plant stop is terminal without mutating the target graph. */
public final class PlantTerminalLifecycleTest {

    @Test
    public void plantApiHasOneTerminalStopAndNoBroadReset() {
        try {
            Plant.class.getMethod("reset");
            fail("Plant must not expose a restart-like reset lifecycle");
        } catch (NoSuchMethodException expected) {
            // A new Plant instance owns a new lifecycle.
        }
    }

    @Test
    public void stopBeforeFirstUpdateDoesNotSampleAPlan() {
        CountingRequestSource requests = new CountingRequestSource(0.4);
        CountingPowerOutput output = new CountingPowerOutput();
        PlantTargetResolver plan = PlantTargets.plan(requests)
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().holdLastTarget(0.0);
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(plan)
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        assertFalse(plant.hasCommandTarget());
        plant.stop();
        plant.update(clock.clock());
        plant.update(clock.nextCycle(0.02));

        assertEquals(0, requests.samples);
        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);
    }

    @Test
    public void powerStopPreservesCommandButPreventsLaterMutationFromActuating() {
        CountingPowerOutput output = new CountingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromNewCommand(0.6)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        plant.stop();

        assertEquals(0.6, plant.commandTarget().get(), 0.0);
        assertEquals(0.6, plant.getRequestedTarget(), 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);

        plant.commandTarget().set(-0.4);
        plant.update(clock.nextCycle(0.02));

        assertEquals(-0.4, plant.commandTarget().get(), 0.0);
        assertEquals(0.6, plant.getRequestedTarget(), 0.0);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);
    }

    @Test
    public void terminalStopDoesNotResetTheTargetGuardGraph() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingBooleanSource gate = new CountingBooleanSource(true);
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetGuards()
                .maxTargetRate(1.0)
                .holdLastTargetUnless("ready", gate)
                .doneTargetGuards()
                .targetFromNewCommand(0.5)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        CapturingDebugSink beforeStop = new CapturingDebugSink();
        plant.debugDump(beforeStop, "power");

        plant.stop();
        plant.stop();
        plant.update(clock.nextCycle(0.02));
        CapturingDebugSink afterStop = new CapturingDebugSink();
        plant.debugDump(afterStop, "power");

        assertEquals(1, gate.samples);
        assertEquals(0, gate.resets);
        assertEquals(0.5, number(beforeStop, "power.targetGuards.lastOut"), 0.0);
        assertEquals(0.5, number(afterStop, "power.targetGuards.lastOut"), 0.0);
        assertEquals(beforeStop.data.get("power.targetGuards.lastStatus"),
                afterStop.data.get("power.targetGuards.lastStatus"));
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertStopped(plant);
    }

    @Test
    public void failedStopStillLatchesAndRepeatedStopIsHarmless() {
        RuntimeException stopFailure = new IllegalStateException("power stop failed");
        CountingResolver resolver = new CountingResolver(0.7);
        CountingPowerOutput output = new CountingPowerOutput();
        output.stopFailure = stopFailure;
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(resolver)
                .build();

        assertSame(stopFailure, expectRuntime(plant::stop));

        plant.stop();
        plant.update(new ManualLoopClock().clock());

        assertEquals(1, output.stopCalls);
        assertEquals(0, output.setCalls);
        assertEquals(0, resolver.resolutions);
    }

    @Test
    public void failedVelocityStopStillLatchesAndLeavesLaterUpdatesInert() {
        RuntimeException stopFailure = new IllegalStateException("velocity stop failed");
        CountingVelocityOutput output = new CountingVelocityOutput();
        CountingScalarSource feedback = new CountingScalarSource(0.4);
        output.stopFailure = stopFailure;
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, feedback)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.4)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        assertSame(stopFailure, expectRuntime(plant::stop));
        plant.stop();
        plant.commandTarget().set(0.8);
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, output.stopCalls);
        assertEquals(1, output.setCalls);
        assertEquals(1, feedback.samples);
        assertEquals(0.4, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetResolution().hasTarget());
        assertFalse(plant.atTarget());
        assertFalse("terminal lifecycle invalidates literal velocity arrival",
                plant.atTarget(0.4));
    }

    @Test
    public void failedPositionStopStillLatchesAndLeavesLaterUpdatesInert() {
        RuntimeException stopFailure = new IllegalStateException("position stop failed");
        CountingPositionOutput output = new CountingPositionOutput();
        CountingScalarSource feedback = new CountingScalarSource(4.0);
        output.stopFailure = stopFailure;
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, feedback)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(4.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        assertSame(stopFailure, expectRuntime(plant::stop));
        plant.stop();
        plant.commandTarget().set(8.0);
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, output.stopCalls);
        assertEquals(1, output.setCalls);
        assertEquals(1, feedback.samples);
        assertEquals(4.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetResolution().hasTarget());
        assertFalse(plant.atTarget());
        assertFalse("terminal lifecycle invalidates literal position arrival",
                plant.atTarget(4.0));
    }

    @Test
    public void terminalStopDoesNotResetItsResolverGraph() {
        CountingResolver resolver = new CountingResolver(0.35);
        CountingPowerOutput output = new CountingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(resolver)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        plant.stop();
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, resolver.resolutions);
        assertEquals(0, resolver.resets);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void stopLatchesBeforeAReentrantOutputCleanupCallback() {
        CountingResolver resolver = new CountingResolver(0.7);
        CountingPowerOutput output = new CountingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(resolver)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        output.onStop = () -> plant.update(clock.clock());

        plant.stop();

        assertEquals(1, output.stopCalls);
        assertEquals(0, output.setCalls);
        assertEquals(0, resolver.resolutions);
        assertStopped(plant);
    }

    @Test
    public void reentrantStopDuringOutputWriteCannotBeOverwrittenByTheOuterUpdate() {
        CountingPowerOutput output = new CountingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromNewCommand(0.7)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        output.onSet = plant::stop;

        plant.update(clock.clock());

        assertEquals(1, output.setCalls);
        assertEquals(2, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);

        plant.update(clock.nextCycle(0.02));
        assertEquals(1, output.setCalls);
        assertEquals(2, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
    }

    @Test
    public void stoppedVelocityPlantSubmitsZeroAndNeverSamplesFeedbackAgain() {
        CountingVelocityOutput output = new CountingVelocityOutput();
        CountingScalarSource feedback = new CountingScalarSource(25.0);
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, feedback)
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(40.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        plant.stop();
        plant.commandTarget().set(80.0);
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, feedback.samples);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);
    }

    @Test
    public void stoppedRegulatedVelocityPlantStopsPowerAndControllerOnlyOnce() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(25.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, feedback)
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(40.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        plant.stop();
        plant.stop();
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, feedback.samples);
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);
    }

    @Test
    public void regulatorReentrantStopPreventsTheLaterPowerWriteAndRedundantCleanup() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(0.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, feedback)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(0.5)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        regulator.onUpdate = plant::stop;

        plant.update(clock.clock());

        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);

        plant.stop();
        plant.update(clock.nextCycle(0.02));
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void regulatedVelocityWriteReentrantStopReassertsZeroWithoutResettingAgain() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(0.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, feedback)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(0.5)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        output.onSet = plant::stop;

        plant.update(clock.clock());

        assertEquals(1, feedback.samples);
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(1, output.setCalls);
        assertEquals(2, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);

        plant.update(clock.nextCycle(0.02));
        assertEquals(1, feedback.samples);
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(1, output.setCalls);
        assertEquals(2, output.stopCalls);
    }

    @Test
    public void regulatorFailureAfterReentrantStopPreservesTheSuccessfulTerminalStop() {
        RuntimeException updateFailure = new IllegalStateException("regulator failed after stop");
        RuntimeException forbiddenSecondStopFailure =
                new IllegalStateException("a redundant second stop failed");
        CountingPowerOutput output = new CountingPowerOutput();
        output.stopFailure = forbiddenSecondStopFailure;
        output.stopFailureOnCall = 2;
        CountingScalarSource feedback = new CountingScalarSource(0.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, feedback)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(0.5)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        regulator.onUpdate = plant::stop;
        regulator.updateFailure = updateFailure;

        RuntimeException observed = expectRuntime(() -> plant.update(clock.clock()));

        assertSame(updateFailure, observed);
        assertEquals(0, observed.getSuppressed().length);
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);

        plant.stop();
        plant.update(clock.nextCycle(0.02));
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void internalRegulatedFailureRequiresControllerCleanupBeforePublishingStoppedFacts() {
        RuntimeException updateFailure = new IllegalStateException("regulator update failed");
        RuntimeException resetFailure = new IllegalStateException("regulator reset failed");
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(0.5);
        CountingRegulator regulator = new CountingRegulator(0.25);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, feedback)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(0.5)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        regulator.updateFailure = updateFailure;
        regulator.resetFailure = resetFailure;

        RuntimeException observed = expectRuntime(() -> plant.update(clock.nextCycle(0.02)));

        assertSame(updateFailure, observed);
        assertSuppressedInOrder(observed, resetFailure);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(1, regulator.resets);
        assertEquals(0.5, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetResolution().hasTarget());
        assertFalse(plant.atTarget());

        regulator.updateFailure = null;
        regulator.resetFailure = null;
        plant.commandTarget().set(0.75);
        plant.update(clock.nextCycle(0.02));

        assertEquals(2, output.setCalls);
        assertEquals(0.25, output.commanded, 0.0);
        assertEquals(0.75, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void stoppedDeviceManagedPositionPlantRetainsItsLastAppliedPosition() {
        CountingPositionOutput output = new CountingPositionOutput();
        CountingScalarSource feedback = new CountingScalarSource(12.0);
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, feedback)
                .nonPeriodic()
                .bounded(0.0, 20.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(12.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        plant.stop();
        plant.commandTarget().set(4.0);
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, feedback.samples);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(12.0, output.commanded, 0.0);
        assertEquals(12.0, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);
    }

    @Test
    public void stoppedCommandOnlyPositionPlantUsesItsNaturalHoldAndBecomesInert() {
        CountingPositionOutput output = new CountingPositionOutput();
        PositionPlant plant = Plants.fromOutputs()
                .commandedPosition(output)
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(0.7)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        plant.stop();
        plant.stop();
        plant.commandTarget().set(0.2);
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.7, output.commanded, 0.0);
        assertEquals(0.7, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);
    }

    @Test
    public void stoppedRegulatedPositionPlantStopsPowerAndControllerOnlyOnce() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(3.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, feedback)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(3.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        plant.stop();
        plant.stop();
        plant.update(clock.nextCycle(0.02));

        assertEquals(1, feedback.samples);
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(3.0, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);
    }

    @Test
    public void regulatedPositionWriteReentrantStopReassertsZeroWithoutResettingAgain() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(3.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, feedback)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(3.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        output.onSet = plant::stop;

        plant.update(clock.clock());

        assertEquals(1, feedback.samples);
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(1, output.setCalls);
        assertEquals(2, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(3.0, plant.getAppliedTarget(), 0.0);
        assertStopped(plant);

        plant.update(clock.nextCycle(0.02));
        assertEquals(1, feedback.samples);
        assertEquals(1, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(1, output.setCalls);
        assertEquals(2, output.stopCalls);
    }

    @Test
    public void regulatedPositionSearchWriteReentrantStopReassertsZeroWithoutResettingAgain() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(3.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, feedback)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(3.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.beginCalibrationSearch(0.4);
        int outputStopsBeforeWrite = output.stopCalls;
        int regulatorResetsBeforeWrite = regulator.resets;
        output.onSet = plant::stop;

        plant.update(clock.clock());

        assertEquals(1, feedback.samples);
        assertEquals(0, regulator.updates);
        assertEquals(regulatorResetsBeforeWrite + 1, regulator.resets);
        assertEquals(outputStopsBeforeWrite + 2, output.stopCalls);
        assertEquals(1, output.setCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);

        plant.update(clock.nextCycle(0.02));
        assertEquals(1, feedback.samples);
        assertEquals(regulatorResetsBeforeWrite + 1, regulator.resets);
        assertEquals(outputStopsBeforeWrite + 2, output.stopCalls);
        assertEquals(1, output.setCalls);
    }

    @Test
    public void unreferencedRegulatedOperationalStopYieldsToReentrantTerminalCleanup() {
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(3.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, feedback)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .needsReference("lift not homed")
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(3.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        output.onStop = plant::stop;

        plant.update(clock.clock());

        assertEquals(1, feedback.samples);
        assertEquals(0, regulator.updates);
        assertEquals(1, regulator.resets);
        assertEquals(0, output.setCalls);
        assertEquals(2, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);

        plant.update(clock.nextCycle(0.02));
        assertEquals(1, feedback.samples);
        assertEquals(1, regulator.resets);
        assertEquals(2, output.stopCalls);
    }

    @Test
    public void terminalPositionPlantRejectsReferenceAndSearchBeforeEffects() {
        CountingPositionOutput position = new CountingPositionOutput();
        CountingPowerOutput search = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(5.0);
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(position, feedback)
                .searchPowerOutput(search)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .needsReference("lift not homed")
                .positionTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.stop();
        int positionStops = position.stopCalls;
        int searchStops = search.stopCalls;

        assertStoppedLifecycleFailure(expectRuntime(() -> plant.beginCalibrationSearch(0.2)));
        assertStoppedLifecycleFailure(expectRuntime(() -> plant.establishReferenceAt(0.0)));
        assertStoppedLifecycleFailure(expectRuntime(
                () -> plant.establishReferenceAt(0.0, clock.clock())));
        plant.endCalibrationSearch();
        plant.update(clock.clock());

        assertFalse(plant.isReferenced());
        assertEquals(0, feedback.samples);
        assertEquals(0, position.setCalls);
        assertEquals(positionStops, position.stopCalls);
        assertEquals(0, search.setCalls);
        assertEquals(searchStops, search.stopCalls);
        assertStopped(plant);
    }

    @Test
    public void directPowerWriteFailureAttemptsAllCleanupAndCanRetry() {
        RuntimeException writeFailure = new IllegalStateException("power write failed");
        RuntimeException stopFailure = new IllegalStateException("power cleanup failed");
        CountingPowerOutput output = new CountingPowerOutput();
        CountingBooleanSource gate = new CountingBooleanSource(true);
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetGuards()
                .maxTargetRate(1.0)
                .holdLastTargetUnless("ready", gate)
                .doneTargetGuards()
                .targetFromNewCommand(0.2)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        plant.commandTarget().set(0.6);
        output.setFailure = writeFailure;
        output.stopFailure = stopFailure;

        RuntimeException observed = expectRuntime(() -> plant.update(clock.nextCycle(0.1)));

        assertSame(writeFailure, observed);
        assertSuppressedInOrder(observed, stopFailure);
        assertEquals(1, output.stopCalls);
        assertEquals(0, gate.resets);
        assertEquals(0.2, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());

        output.setFailure = null;
        output.stopFailure = null;
        plant.update(clock.nextCycle(0.1));

        assertEquals(3, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.6, output.commanded, 0.0);
        assertEquals(0.6, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void velocityWriteFailureStopsResetsGuardsAndCanRetry() {
        RuntimeException writeFailure = new IllegalStateException("velocity write failed");
        RuntimeException stopFailure = new IllegalStateException("velocity cleanup failed");
        CountingVelocityOutput output = new CountingVelocityOutput();
        CountingBooleanSource gate = new CountingBooleanSource(true);
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, new CountingScalarSource(0.2))
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetGuards()
                .maxTargetRate(1.0)
                .holdLastTargetUnless("ready", gate)
                .doneTargetGuards()
                .targetFromNewCommand(0.2)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        plant.commandTarget().set(0.6);
        output.setFailure = writeFailure;
        output.stopFailure = stopFailure;

        RuntimeException observed = expectRuntime(() -> plant.update(clock.nextCycle(0.1)));

        assertSame(writeFailure, observed);
        assertSuppressedInOrder(observed, stopFailure);
        assertEquals(1, output.stopCalls);
        assertEquals(0, gate.resets);
        assertEquals(0.2, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());

        output.setFailure = null;
        output.stopFailure = null;
        plant.update(clock.nextCycle(0.1));

        assertEquals(3, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.6, output.commanded, 0.0);
        assertEquals(0.6, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void positionWriteFailureAttemptsAllCleanupAndCanRetry() {
        RuntimeException writeFailure = new IllegalStateException("position write failed");
        RuntimeException stopFailure = new IllegalStateException("position cleanup failed");
        CountingPositionOutput output = new CountingPositionOutput();
        CountingBooleanSource gate = new CountingBooleanSource(true);
        PositionPlant plant = Plants.fromOutputs()
                .commandedPosition(output)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .targetGuards()
                .maxTargetRate(1.0)
                .holdLastTargetUnless("ready", gate)
                .doneTargetGuards()
                .targetFromNewCommand(2.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        plant.commandTarget().set(4.0);
        output.setFailure = writeFailure;
        output.stopFailure = stopFailure;

        RuntimeException observed = expectRuntime(() -> plant.update(clock.nextCycle(0.1)));

        assertSame(writeFailure, observed);
        assertSuppressedInOrder(observed, stopFailure);
        assertEquals(1, output.stopCalls);
        assertEquals(0, gate.resets);
        assertEquals(2.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());

        output.setFailure = null;
        output.stopFailure = null;
        plant.update(clock.nextCycle(0.1));

        assertEquals(3, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(4.0, output.commanded, 0.0);
        assertEquals(4.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void terminalStopDuringFailureCleanupSkipsLaterGuardMutation() {
        RuntimeException writeFailure = new IllegalStateException("power write failed");
        CountingPowerOutput output = new CountingPowerOutput();
        CountingBooleanSource gate = new CountingBooleanSource(true);
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetGuards()
                .holdLastTargetUnless("ready", gate)
                .doneTargetGuards()
                .targetFromNewCommand(0.5)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        output.setFailure = writeFailure;
        output.onStop = plant::stop;

        RuntimeException observed = expectRuntime(() -> plant.update(clock.clock()));

        assertSame(writeFailure, observed);
        assertEquals(0, observed.getSuppressed().length);
        assertEquals(1, gate.samples);
        assertEquals(0, gate.resets);
        assertEquals(2, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);

        plant.stop();
        plant.update(clock.nextCycle(0.02));
        assertEquals(1, gate.samples);
        assertEquals(0, gate.resets);
        assertEquals(2, output.stopCalls);
    }

    @Test
    public void regulatedWriteFailureWithReentrantTerminalStopStopsOutputExactlyTwice() {
        RuntimeException writeFailure = new IllegalStateException("regulated write failed");
        CountingPowerOutput output = new CountingPowerOutput();
        CountingScalarSource feedback = new CountingScalarSource(0.0);
        CountingRegulator regulator = new CountingRegulator(0.5);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, feedback)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(0.5)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        output.setFailure = writeFailure;
        output.onStop = plant::stop;

        RuntimeException observed = expectRuntime(() -> plant.update(clock.clock()));

        assertSame(writeFailure, observed);
        assertEquals(0, observed.getSuppressed().length);
        assertEquals(1, output.setCalls);
        // One internal fail-stop invokes the reentrant public terminal stop. The outer regulated
        // failure handler must consume that completed cleanup rather than issuing a third stop.
        assertEquals(2, output.stopCalls);
        assertEquals(1, regulator.resets);
        assertEquals(0.0, output.commanded, 0.0);
        assertStopped(plant);

        plant.stop();
        plant.update(clock.nextCycle(0.02));
        assertEquals(2, output.stopCalls);
    }

    private static void assertStopped(Plant plant) {
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.getTargetResolution().hasTarget());
        assertFalse(plant.atTarget());
    }

    private static void assertStoppedLifecycleFailure(RuntimeException failure) {
        assertTrue(failure instanceof IllegalStateException);
        assertTrue(String.valueOf(failure.getMessage()).contains("Plant.stop()"));
        assertTrue(String.valueOf(failure.getMessage()).contains("new Plant"));
    }

    private static double number(CapturingDebugSink debug, String key) {
        Object value = debug.data.get(key);
        assertTrue(key + " must contain a number", value instanceof Number);
        return ((Number) value).doubleValue();
    }

    private static void assertSuppressedInOrder(
            RuntimeException primary,
            RuntimeException... expected) {
        Throwable[] suppressed = primary.getSuppressed();
        assertEquals(expected.length, suppressed.length);
        for (int i = 0; i < expected.length; i++) {
            assertSame(expected[i], suppressed[i]);
        }
    }

    private static RuntimeException expectRuntime(Runnable operation) {
        try {
            operation.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static final class CountingResolver implements PlantTargetResolver {
        private final double target;
        private int resolutions;
        private int resets;

        private CountingResolver(double target) {
            this.target = target;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            resolutions++;
            return PlantTargetResolution.exact(target, "counting resolver");
        }

        @Override
        public void reset() {
            resets++;
        }
    }

    private static final class CountingRequestSource implements Source<PlantTargetRequest> {
        private final double target;
        private int samples;

        private CountingRequestSource(double target) {
            this.target = target;
        }

        @Override
        public PlantTargetRequest get(LoopClock clock) {
            samples++;
            return PlantTargetRequest.exact("planned", target);
        }
    }

    private static final class CountingScalarSource implements ScalarSource {
        private final double value;
        private int samples;

        private CountingScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            samples++;
            return value;
        }
    }

    private static final class CountingBooleanSource implements BooleanSource {
        private final boolean value;
        private int samples;
        private int resets;

        private CountingBooleanSource(boolean value) {
            this.value = value;
        }

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            samples++;
            return value;
        }

        @Override
        public void reset() {
            resets++;
        }
    }

    private static final class CountingPowerOutput implements PowerOutput {
        private double commanded = Double.NaN;
        private int setCalls;
        private int stopCalls;
        private RuntimeException setFailure;
        private RuntimeException stopFailure;
        private int stopFailureOnCall;
        private Runnable onSet;
        private Runnable onStop;

        @Override
        public void setPower(double power) {
            setCalls++;
            Runnable callback = onSet;
            onSet = null;
            if (callback != null) callback.run();
            // Commit after the callback so a reentrant stop can be overwritten unless the Plant
            // notices terminality and reasserts its natural stop before update returns.
            commanded = power;
            if (setFailure != null) throw setFailure;
        }

        @Override
        public double getCommandedPower() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
            commanded = 0.0;
            Runnable callback = onStop;
            onStop = null;
            if (callback != null) callback.run();
            if (stopFailure != null
                    && (stopFailureOnCall <= 0 || stopCalls == stopFailureOnCall)) {
                throw stopFailure;
            }
        }
    }

    private static final class CountingVelocityOutput implements VelocityOutput {
        private double commanded = Double.NaN;
        private int setCalls;
        private int stopCalls;
        private RuntimeException setFailure;
        private RuntimeException stopFailure;

        @Override
        public void setVelocity(double velocity) {
            setCalls++;
            commanded = velocity;
            if (setFailure != null) throw setFailure;
        }

        @Override
        public double getCommandedVelocity() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
            commanded = 0.0;
            if (stopFailure != null) throw stopFailure;
        }
    }

    private static final class CountingPositionOutput implements PositionOutput {
        private double commanded = Double.NaN;
        private int setCalls;
        private int stopCalls;
        private RuntimeException setFailure;
        private RuntimeException stopFailure;

        @Override
        public void setPosition(double position) {
            setCalls++;
            commanded = position;
            if (setFailure != null) throw setFailure;
        }

        @Override
        public double getCommandedPosition() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
            if (stopFailure != null) throw stopFailure;
        }
    }

    private static final class CountingRegulator implements ScalarRegulator {
        private final double output;
        private int updates;
        private int resets;
        private RuntimeException updateFailure;
        private RuntimeException resetFailure;
        private Runnable onUpdate;

        private CountingRegulator(double output) {
            this.output = output;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            updates++;
            Runnable callback = onUpdate;
            onUpdate = null;
            if (callback != null) callback.run();
            if (updateFailure != null) throw updateFailure;
            return output;
        }

        @Override
        public void reset() {
            resets++;
            if (resetFailure != null) throw resetFailure;
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
