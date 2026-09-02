package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies atomic semantic/scalar publication and private target-resolution correlation. */
public final class SemanticScalarCommandTest {

    private static final double EPSILON = 1e-12;

    private enum Mode {
        IDLE,
        ACTIVE,
        THROWING,
        NAN,
        POSITIVE_INFINITY,
        NEGATIVE_INFINITY
    }

    @Test
    public void createAndSetMapExactlyOnceAndPublishFreshImmutableRequests() {
        int[] mappings = {0};
        SemanticScalarCommand<Mode> command = SemanticScalarCommand.create(Mode.IDLE, mode -> {
            mappings[0]++;
            return mode == Mode.IDLE ? 0.0 : 0.75;
        });

        SemanticScalarCommand.Request<Mode> initial = command.request();
        SemanticScalarCommand.Request<Mode> repeated = command.set(Mode.IDLE);
        SemanticScalarCommand.Request<Mode> active = command.set(Mode.ACTIVE);

        assertEquals(3, mappings[0]);
        assertNotSame(initial, repeated);
        assertNotSame(repeated, active);
        assertEquals(Mode.IDLE, initial.semantic());
        assertEquals(0.0, initial.commandTarget(), EPSILON);
        assertEquals(Mode.IDLE, repeated.semantic());
        assertEquals(0.0, repeated.commandTarget(), EPSILON);
        assertEquals(Mode.ACTIVE, active.semantic());
        assertEquals(0.75, active.commandTarget(), EPSILON);
        assertSame(active, command.request());
    }

    @Test
    public void invalidOrThrowingMappingsLeaveThePriorRequestUntouched() {
        RuntimeException mapperFailure = new RuntimeException("mapper failed");
        SemanticScalarCommand<Mode> command = SemanticScalarCommand.create(Mode.IDLE, mode -> {
            switch (mode) {
                case THROWING:
                    throw mapperFailure;
                case NAN:
                    return Double.NaN;
                case POSITIVE_INFINITY:
                    return Double.POSITIVE_INFINITY;
                case NEGATIVE_INFINITY:
                    return Double.NEGATIVE_INFINITY;
                case ACTIVE:
                    return 0.75;
                default:
                    return 0.0;
            }
        });
        SemanticScalarCommand.Request<Mode> initial = command.request();

        try {
            command.set(Mode.THROWING);
            fail("Expected mapper failure");
        } catch (RuntimeException actual) {
            assertSame(mapperFailure, actual);
        }
        assertSame(initial, command.request());

        expectIllegalArgument(() -> command.set(Mode.NAN), "finite target");
        assertSame(initial, command.request());
        expectIllegalArgument(() -> command.set(Mode.POSITIVE_INFINITY), "finite target");
        assertSame(initial, command.request());
        expectIllegalArgument(() -> command.set(Mode.NEGATIVE_INFINITY), "finite target");
        assertSame(initial, command.request());

        try {
            command.set(null);
            fail("Expected null semantic request to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("semantic"));
        }
        assertSame(initial, command.request());
    }

    @Test
    public void invalidConstructionPublishesNoPartiallyInitializedCommand() {
        try {
            SemanticScalarCommand.create(Mode.IDLE, null);
            fail("Expected null mapper to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("commandTargetFor"));
        }

        expectIllegalArgument(
                () -> SemanticScalarCommand.create(Mode.IDLE, ignored -> Double.NaN),
                "finite target");
    }

    @Test
    public void exactResolutionAndSnapshotRequireTheExactCurrentRequestIdentity() {
        SemanticScalarCommand<Mode> command =
                SemanticScalarCommand.create(Mode.ACTIVE, ignored -> 0.5);
        PlantTargetResolver resolver = PlantTargets.exact(command);
        ManualLoopClock time = new ManualLoopClock();
        SnapshotPlant plant = new SnapshotPlant();

        SemanticScalarCommand.Request<Mode> firstRequest = command.request();
        PlantTargetResolution firstResolution = resolver.resolve(simpleContext(), time.clock());
        plant.publish(firstResolution, true);
        PlantSnapshot firstPlant = plant.snapshot();
        SemanticScalarSnapshot<Mode, PlantSnapshot> first = command.snapshot(firstPlant);

        assertTrue(firstResolution.satisfiesSemanticCommand(command, firstRequest));
        assertSame(firstRequest, first.request());
        assertSame(firstPlant, first.plant());
        assertTrue(first.currentRequestSelected());
        assertTrue(first.currentRequestAtTarget());

        SemanticScalarCommand.Request<Mode> repeated = command.set(Mode.ACTIVE);
        SemanticScalarSnapshot<Mode, PlantSnapshot> beforeNextHeartbeat =
                command.snapshot(plant.snapshot());

        assertNotSame(firstRequest, repeated);
        assertFalse(firstResolution.satisfiesSemanticCommand(command, repeated));
        assertFalse(beforeNextHeartbeat.currentRequestSelected());
        assertFalse(beforeNextHeartbeat.currentRequestAtTarget());
        assertTrue("An already returned snapshot remains an immutable historical fact",
                first.currentRequestAtTarget());

        PlantTargetResolution repeatedResolution =
                resolver.resolve(simpleContext(), time.nextCycle(0.02));
        plant.publish(repeatedResolution, true);
        SemanticScalarSnapshot<Mode, PlantSnapshot> afterNextHeartbeat =
                command.snapshot(plant.snapshot());

        assertTrue(repeatedResolution.satisfiesSemanticCommand(command, repeated));
        assertTrue(afterNextHeartbeat.currentRequestSelected());
        assertTrue(afterNextHeartbeat.currentRequestAtTarget());
    }

    @Test
    public void openLoopPlantCarriesSemanticSelectionWithoutClaimingArrivalOrNumericOwnership() {
        SemanticScalarCommand<Mode> command =
                SemanticScalarCommand.create(Mode.ACTIVE, ignored -> 0.5);
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(PlantTargets.exact(command))
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertFalse(PlantTargets.isExactCommand(PlantTargets.exact(command)));
        assertFalse(plant.hasCommandTarget());
        plant.update(time.clock());

        SemanticScalarSnapshot<Mode, PlantSnapshot> snapshot =
                command.snapshot(plant.snapshot());
        assertEquals(0.5, output.commanded, EPSILON);
        assertTrue(snapshot.currentRequestSelected());
        assertFalse(snapshot.currentRequestAtTarget());
        try {
            plant.commandTarget();
            fail("Expected semantic Plant to expose no numeric command target");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("no command target"));
        }
    }

    @Test
    public void activeOverlayMasksItsSemanticBaseEvenAtTheSameNumericValue() {
        SemanticScalarCommand<Mode> command =
                SemanticScalarCommand.create(Mode.ACTIVE, ignored -> 0.5);
        final boolean[] override = {true};
        PlantTargetResolver resolver = PlantTargets.overlay(command)
                .add("sameValueOverride", clock -> override[0], 0.5)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetResolution masked = resolver.resolve(simpleContext(), time.clock());
        assertTrue(masked.reportsSemanticCommandResolutionFor(command));
        assertFalse(masked.satisfiesSemanticCommand(command, command.request()));

        override[0] = false;
        PlantTargetResolution base =
                resolver.resolve(simpleContext(), time.nextCycle(0.02));
        assertTrue(base.satisfiesSemanticCommand(command, command.request()));
    }

    @Test
    public void winningLayerCannotLeakItsCommandProvenanceThroughOwnerlessOverlay() {
        SemanticScalarCommand<Mode> semanticLayer =
                SemanticScalarCommand.create(Mode.ACTIVE, ignored -> 0.5);
        ScalarTarget scalarLayer = ScalarTarget.create(0.75);
        PlantTargetResolver semanticOverlay = PlantTargets.overlay(0.0)
                .add("semantic", BooleanSource.constant(true), PlantTargets.exact(semanticLayer))
                .build();
        PlantTargetResolver scalarOverlay = PlantTargets.overlay(0.0)
                .add("scalar", BooleanSource.constant(true), PlantTargets.exact(scalarLayer))
                .build();

        PlantTargetResolution semanticWinner = semanticOverlay.resolve(
                simpleContext(), new ManualLoopClock().clock());
        PlantTargetResolution scalarWinner = scalarOverlay.resolve(
                simpleContext(), new ManualLoopClock().clock());

        assertFalse(semanticWinner.reportsSemanticCommandResolutionFor(semanticLayer));
        assertFalse(semanticWinner.satisfiesSemanticCommand(
                semanticLayer, semanticLayer.request()));
        assertFalse(scalarWinner.reportsCommandResolutionFor(scalarLayer));
        assertFalse(scalarWinner.satisfiesCommand(scalarLayer, 0.75));
    }

    @Test
    public void equivalentPositionPreservesSemanticIdentityButUnavailablePolicyDoesNotSatisfyIt() {
        SemanticScalarCommand<Mode> command =
                SemanticScalarCommand.create(Mode.ACTIVE, ignored -> 20.0);
        PlantTargetResolver equivalent = PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        PlantTargetResolution selected = equivalent.resolve(
                PlantTargetContext.position(
                        true, 350.0, ScalarRange.bounded(0.0, 720.0),
                        PositionPlant.Periodicity.PERIODIC, 360.0,
                        Double.NaN, Double.NaN),
                new ManualLoopClock().clock());

        assertEquals(380.0, selected.target(), EPSILON);
        assertTrue(selected.satisfiesSemanticCommand(command, command.request()));

        PlantTargetResolver fallback = PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable().fallbackTo(7.0);
        PlantTargetResolution unavailable = fallback.resolve(
                simpleContext(), new ManualLoopClock().clock());

        assertEquals(7.0, unavailable.target(), EPSILON);
        assertTrue(unavailable.reportsSemanticCommandResolutionFor(command));
        assertFalse(unavailable.satisfiesSemanticCommand(command, command.request()));
    }

    @Test
    public void publicApiDoesNotExposeScalarSourceTargetRevisionOrSnapshotConstruction()
            throws Exception {
        assertFalse(ScalarSource.class.isAssignableFrom(SemanticScalarCommand.class));
        assertFalse(ScalarTarget.class.isAssignableFrom(SemanticScalarCommand.class));

        Method set = SemanticScalarCommand.class.getDeclaredMethod("set", Object.class);
        Method request = SemanticScalarCommand.class.getDeclaredMethod("request");
        Method snapshot = SemanticScalarCommand.class.getDeclaredMethod(
                "snapshot", PlantSnapshot.class);
        assertSame(SemanticScalarCommand.Request.class, set.getReturnType());
        assertSame(SemanticScalarCommand.Request.class, request.getReturnType());
        assertSame(SemanticScalarSnapshot.class, snapshot.getReturnType());

        for (Constructor<?> constructor : SemanticScalarCommand.Request.class
                .getDeclaredConstructors()) {
            assertFalse(Modifier.isPublic(constructor.getModifiers()));
        }
        for (Constructor<?> constructor : SemanticScalarSnapshot.class.getDeclaredConstructors()) {
            assertFalse(Modifier.isPublic(constructor.getModifiers()));
        }

        assertSame(PlantTargetResolver.class,
                PlantTargets.class.getDeclaredMethod(
                        "exact", SemanticScalarCommand.class).getReturnType());
        assertSame(PlantTargets.OverlayBuilder.class,
                PlantTargets.class.getDeclaredMethod(
                        "overlay", SemanticScalarCommand.class).getReturnType());
        assertSame(PlantTargets.EquivalentPositionPreferenceStage.class,
                PlantTargets.class.getDeclaredMethod(
                        "equivalentPositionsOf", SemanticScalarCommand.class).getReturnType());
    }

    private static PlantTargetContext simpleContext() {
        return PlantTargetContext.simple(
                true, 0.0, ScalarRange.bounded(-1.0, 1.0), Double.NaN, Double.NaN);
    }

    private static void expectIllegalArgument(Runnable call, String messageFragment) {
        try {
            call.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(messageFragment));
        }
    }

    private static final class SnapshotPlant implements Plant {
        private PlantTargetResolution resolution =
                PlantTargetResolution.unavailable("not published");
        private boolean atTarget;

        void publish(PlantTargetResolution nextResolution, boolean nextAtTarget) {
            resolution = nextResolution;
            atTarget = nextAtTarget;
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public double getRequestedTarget() {
            return resolution.hasTarget() ? resolution.target() : Double.NaN;
        }

        @Override
        public double getAppliedTarget() {
            return getRequestedTarget();
        }

        @Override
        public PlantTargetResolution getTargetResolution() {
            return resolution;
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public double getMeasurement() {
            return getRequestedTarget();
        }

        @Override
        public boolean atTarget() {
            return atTarget;
        }

        @Override
        public boolean atTarget(double target) {
            return atTarget && getRequestedTarget() == target;
        }

        @Override
        public void stop() {
            atTarget = false;
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        private double commanded = Double.NaN;

        @Override
        public void setPower(double power) {
            commanded = power;
        }

        @Override
        public double getCommandedPower() {
            return commanded;
        }
    }
}
