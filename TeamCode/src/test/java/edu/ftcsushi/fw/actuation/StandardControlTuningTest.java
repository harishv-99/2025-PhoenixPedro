package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Regression coverage for the Plant-derived standard-control tuning capabilities. */
public final class StandardControlTuningTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void velocityClaimExposesFixedTopologyAndAtomicallyAppliesCompleteGains() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> 2.0)
                .bounded(-20.0, 20.0)
                .nativeUnits()
                .velocityTolerance(0.1)
                .setpointFromAccelerationLimitedProfile(4.0)
                .feedbackFromPid(0.1, 0.2, 0.3)
                .feedbackIntegralLimitedTo(-0.5, 0.5)
                .feedbackOutputLimitedTo(-0.8, 0.8)
                .feedforwardFromLift(0.01, 0.02, 0.03, 0.04)
                .outputPowerLimitedTo(-0.7, 0.6)
                .targetFromNewCommand(2.0)
                .build();

        StandardControlTuning tuning = StandardControlTunings.claimVelocity(plant);
        StandardControlTuning.Topology topology = tuning.topology();
        assertEquals(StandardControlTuning.Domain.VELOCITY, topology.domain());
        assertEquals(StandardControlTuning.SetpointModel.VELOCITY_ACCELERATION_LIMITED,
                topology.setpointModel());
        assertEquals(StandardControlTuning.FeedforwardModel.LIFT,
                topology.feedforwardModel());
        assertFalse(topology.hasMaximumVelocity());
        assertEquals(4.0, topology.maximumAcceleration(), 0.0);
        assertEquals(-0.5, topology.integralMinimum(), 0.0);
        assertEquals(0.8, topology.feedbackMaximum(), 0.0);
        assertEquals(-0.7, topology.outputMinimum(), 0.0);
        assertTrue(topology.hasKS());
        assertTrue(topology.hasKV());
        assertTrue(topology.hasKA());
        assertTrue(topology.hasKG());
        assertTrue(tuning.hasExactCommandTarget());

        assertIllegalState(topology::maximumVelocity, "not configured");
        assertIllegalState(topology::referenceVoltage, "not configured");
        assertSame(tuning.initialParameters(), tuning.appliedParameters());
        assertIllegalState(() -> StandardControlTunings.claimVelocity(plant), "already supplied");

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        StandardControlTuning.Evidence initialEvidence = tuning.evidence();
        assertTrue(initialEvidence.isAvailable());
        assertEquals(2.0, initialEvidence.measurement(), 0.0);
        assertEquals(2.0, initialEvidence.setpointVelocity(), 0.0);
        assertEquals(0.0, initialEvidence.setpointAcceleration(), 0.0);
        assertEquals(0.09, initialEvidence.feedforwardOutput(), EPSILON);
        assertEquals(0.09, initialEvidence.outputBeforeLimit(), EPSILON);
        assertEquals(0.09, initialEvidence.output(), EPSILON);
        assertFalse(initialEvidence.hasVoltageScale());

        StandardControlTuning.Parameters candidate = tuning.appliedParameters()
                .withFeedbackPid(0.4, 0.0, 0.0)
                .withLiftFeedforward(0.11, 0.02, 0.07, 0.03);
        tuning.applyAndReseed(candidate, time.clock());
        assertSame(candidate, tuning.appliedParameters());
        assertFalse("apply reseeds but does not invent a completed evaluation",
                tuning.evidence().isAvailable());
        assertEquals(2.0, tuning.evidence().measurement(), 0.0);

        plant.commandTarget().set(4.0);
        plant.update(time.nextCycle(0.25));
        StandardControlTuning.Evidence applied = tuning.evidence();
        assertEquals(2.0, applied.setpointVelocity(), 0.0);
        assertEquals(0.0, applied.setpointAcceleration(), 0.0);
        assertEquals(0.27, applied.feedforwardOutput(), EPSILON);
        tuning.restoreInitialAndReseed(time.clock());
        assertSame(tuning.initialParameters(), tuning.appliedParameters());
    }

    @Test
    public void candidateFromAnotherTopologyIsRejectedBeforeMutation() {
        Plant first = directVelocityPlant(new RecordingPowerOutput(), 0.0, 0.1);
        Plant second = directVelocityPlant(new RecordingPowerOutput(), 0.0, 0.2);
        StandardControlTuning firstTuning = StandardControlTunings.claimVelocity(first);
        StandardControlTuning secondTuning = StandardControlTunings.claimVelocity(second);
        ManualLoopClock time = new ManualLoopClock();
        first.update(time.clock());
        StandardControlTuning.Parameters before = firstTuning.appliedParameters();

        assertIllegalArgument(
                () -> firstTuning.applyAndReseed(secondTuning.appliedParameters(), time.clock()),
                "different standard-controller instance");
        assertSame(before, firstTuning.appliedParameters());
        assertTrue(firstTuning.evidence().isAvailable());
    }

    @Test
    public void velocityOnlyMotionFeedforwardRetainsItsExactActiveShape() {
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(new RecordingPowerOutput(), clock -> 1.0)
                .bounded(-5.0, 5.0)
                .nativeUnits()
                .velocityTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.0)
                .feedforwardFromMotion(0.2)
                .targetFromNewCommand(1.0)
                .build();
        StandardControlTuning tuning = StandardControlTunings.claimVelocity(plant);

        assertFalse(tuning.topology().hasKS());
        assertTrue(tuning.topology().hasKV());
        assertFalse(tuning.topology().hasKA());
        assertFalse(tuning.topology().hasKG());
        assertEquals(0.2, tuning.appliedParameters().kV(), 0.0);
        assertEquals(0.3,
                tuning.appliedParameters().withMotionFeedforward(0.3).kV(), 0.0);
        assertIllegalState(
                () -> tuning.appliedParameters().withMotionFeedforward(0.0, 0.3),
                "does not match");
    }

    @Test
    public void assumeCurrentPositionPreparationSamplesAndStagesHoldWithoutOutput() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> 123.0)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .scaleToNative(2.0)
                .assumeCurrentPositionIs(7.0)
                .positionTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.4)
                .targetFromNewCommand(1.0)
                .build();
        StandardControlTuning tuning = StandardControlTunings.claimPosition(plant);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(7.0, tuning.prepareHoldAtCurrent(time.clock()), 0.0);
        assertTrue(plant.isReferenced());
        assertEquals(7.0, plant.getMeasurement(), 0.0);
        assertEquals(7.0, plant.commandTarget().get(), 0.0);
        assertEquals(0, output.writes);
        assertFalse(tuning.evidence().isAvailable());
        assertEquals(7.0, tuning.evidence().measurement(), 0.0);
        assertIllegalState(() -> tuning.prepareHoldAtCurrent(time.clock()), "already prepared");

        plant.update(time.clock());
        assertEquals(1, output.writes);
        assertEquals(0.0, output.lastPower, EPSILON);
    }

    @Test
    public void recoveryHoldUsesSameCycleSampleClampsAndReseedsBeforeOutput() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        double[] nativePosition = {5.0};
        int[] samples = {0};
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> {
                    samples[0]++;
                    return nativePosition[0];
                })
                .nonPeriodic()
                .bounded(0.0, 20.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.2)
                .targetFromNewCommand(1.0)
                .build();
        StandardControlTuning tuning = StandardControlTunings.claimPosition(plant);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(5.0, tuning.prepareHoldAtCurrent(time.clock()), 0.0);
        assertEquals(1, samples[0]);
        plant.update(time.clock());
        assertEquals("the Plant reuses the preparation sample in that cycle", 1, samples[0]);
        assertEquals(1, output.writes);

        nativePosition[0] = 12.0;
        time.nextCycle(0.02);
        PositionPlantTuning.RecoveryHold recovery = tuning.prepareRecoveryHoldWithin(
                ScalarRange.bounded(2.0, 8.0), time.clock());
        assertEquals(12.0, recovery.measurement(), 0.0);
        assertEquals(8.0, recovery.holdTarget(), 0.0);
        assertTrue(recovery.wasClamped());
        assertEquals(8.0, plant.commandTarget().get(), 0.0);
        assertEquals(12.0, plant.getMeasurement(), 0.0);
        assertEquals(2, samples[0]);
        assertEquals("recovery preparation itself does not actuate", 1, output.writes);
        assertFalse("standard recovery reseeds instead of retaining a moving setpoint",
                tuning.evidence().isAvailable());

        plant.update(time.clock());
        assertEquals("normal realization consumes the same memoized sample", 2, samples[0]);
        assertEquals(2, output.writes);
        assertIllegalState(() -> tuning.prepareRecoveryHoldWithin(
                ScalarRange.bounded(2.0, 8.0), time.clock()), "before the Plant's normal update");

        nativePosition[0] = 6.0;
        time.nextCycle(0.02);
        PositionPlantTuning.RecoveryHold exact = tuning.prepareRecoveryHoldWithin(
                ScalarRange.bounded(2.0, 8.0), time.clock());
        assertEquals(6.0, exact.measurement(), 0.0);
        assertEquals(6.0, exact.holdTarget(), 0.0);
        assertFalse(exact.wasClamped());
        assertEquals(3, samples[0]);
    }

    @Test
    public void initialHoldPreparationRejectsReentrantUpdateAndPreparationWithoutOutput() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        PositionPlant[] plantRef = new PositionPlant[1];
        PositionPlantTuning[] tuningRef = new PositionPlantTuning[1];
        RuntimeException[] updateFailure = new RuntimeException[1];
        RuntimeException[] preparationFailure = new RuntimeException[1];
        ManualLoopClock time = new ManualLoopClock();
        ScalarTarget target = new ScalarTarget() {
            private boolean callbackAttempted;
            private double value = 1.0;

            @Override
            public void set(double value) {
                this.value = value;
                if (!callbackAttempted && plantRef[0] != null && tuningRef[0] != null) {
                    callbackAttempted = true;
                    try {
                        plantRef[0].update(time.clock());
                    } catch (RuntimeException failure) {
                        updateFailure[0] = failure;
                    }
                    try {
                        tuningRef[0].prepareHoldAtCurrent(time.clock());
                    } catch (RuntimeException failure) {
                        preparationFailure[0] = failure;
                    }
                }
            }

            @Override
            public double get() {
                return value;
            }
        };
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, clock -> 5.0)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .targetFromResolver(PlantTargets.exact(target))
                .build();
        plantRef[0] = plant;
        tuningRef[0] = PositionPlantTunings.claim(plant);

        assertEquals(5.0, tuningRef[0].prepareHoldAtCurrent(time.clock()), 0.0);

        assertTrue(updateFailure[0] instanceof IllegalStateException);
        assertTrue(updateFailure[0].getMessage(),
                updateFailure[0].getMessage().contains("preparation is in progress"));
        assertTrue(preparationFailure[0] instanceof IllegalStateException);
        assertTrue(preparationFailure[0].getMessage(),
                preparationFailure[0].getMessage().contains("already in progress"));
        assertEquals(5.0, plant.commandTarget().get(), 0.0);
        assertEquals("preparation and rejected callbacks do not actuate", 0, output.writes);
        assertIllegalState(
                () -> tuningRef[0].prepareHoldAtCurrent(time.clock()), "already prepared");

        plant.update(time.clock());
        assertEquals("the rejected update did not consume the cycle", 1, output.writes);
        assertEquals(5.0, output.lastPosition, 0.0);
    }

    @Test
    public void recoveryHoldPreparationRejectsReentrantUpdateAndPreparationWithoutOutput() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        double[] nativePosition = {4.0};
        PositionPlant[] plantRef = new PositionPlant[1];
        PositionPlantTuning[] tuningRef = new PositionPlantTuning[1];
        RuntimeException[] updateFailure = new RuntimeException[1];
        RuntimeException[] preparationFailure = new RuntimeException[1];
        boolean[] callbacksEnabled = {false};
        ManualLoopClock time = new ManualLoopClock();
        ScalarRange allowedRange = ScalarRange.bounded(2.0, 8.0);
        ScalarTarget target = new ScalarTarget() {
            private double value = 1.0;

            @Override
            public void set(double value) {
                this.value = value;
                if (callbacksEnabled[0]) {
                    callbacksEnabled[0] = false;
                    try {
                        plantRef[0].update(time.clock());
                    } catch (RuntimeException failure) {
                        updateFailure[0] = failure;
                    }
                    try {
                        tuningRef[0].prepareRecoveryHoldWithin(allowedRange, time.clock());
                    } catch (RuntimeException failure) {
                        preparationFailure[0] = failure;
                    }
                }
            }

            @Override
            public double get() {
                return value;
            }
        };
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, clock -> nativePosition[0])
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .targetFromResolver(PlantTargets.exact(target))
                .build();
        plantRef[0] = plant;
        tuningRef[0] = PositionPlantTunings.claim(plant);
        tuningRef[0].prepareHoldAtCurrent(time.clock());
        plant.update(time.clock());
        assertEquals(1, output.writes);

        nativePosition[0] = 9.0;
        time.nextCycle(0.02);
        callbacksEnabled[0] = true;
        PositionPlantTuning.RecoveryHold hold = tuningRef[0].prepareRecoveryHoldWithin(
                allowedRange, time.clock());

        assertEquals(9.0, hold.measurement(), 0.0);
        assertEquals(8.0, hold.holdTarget(), 0.0);
        assertTrue(updateFailure[0] instanceof IllegalStateException);
        assertTrue(updateFailure[0].getMessage(),
                updateFailure[0].getMessage().contains("preparation is in progress"));
        assertTrue(preparationFailure[0] instanceof IllegalStateException);
        assertTrue(preparationFailure[0].getMessage(),
                preparationFailure[0].getMessage().contains("already in progress"));
        assertEquals("recovery preparation and rejected callbacks do not actuate",
                1, output.writes);

        plant.update(time.clock());
        assertEquals("the rejected update did not consume the recovery cycle", 2, output.writes);
        assertEquals(8.0, output.lastPosition, 0.0);
    }

    @Test
    public void failedTargetCallbackClearsPreparationLatchAndLeavesInitialRetryEligible() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        ScalarTarget target = new ScalarTarget() {
            private boolean failNextSet = true;
            private double value = 1.0;

            @Override
            public void set(double value) {
                if (failNextSet) {
                    failNextSet = false;
                    throw new IllegalStateException("synthetic target write failure");
                }
                this.value = value;
            }

            @Override
            public double get() {
                return value;
            }
        };
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, clock -> 6.0)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .targetFromResolver(PlantTargets.exact(target))
                .build();
        PositionPlantTuning tuning = PositionPlantTunings.claim(plant);
        ManualLoopClock time = new ManualLoopClock();

        assertIllegalState(
                () -> tuning.prepareHoldAtCurrent(time.clock()), "synthetic target write failure");
        assertEquals(1.0, plant.commandTarget().get(), 0.0);
        assertEquals(0, output.writes);

        assertEquals(6.0, tuning.prepareHoldAtCurrent(time.clock()), 0.0);
        assertEquals(6.0, plant.commandTarget().get(), 0.0);
        assertEquals(0, output.writes);
        plant.update(time.clock());
        assertEquals(1, output.writes);
        assertEquals(6.0, output.lastPosition, 0.0);
    }

    @Test
    public void positionPreparationRemainsEligibleAfterSearchButBeforeNormalRealization() {
        RecordingPositionOutput normal = new RecordingPositionOutput();
        RecordingPowerOutput search = new RecordingPowerOutput();
        double[] nativePosition = {12.0};
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(normal, clock -> nativePosition[0])
                .searchPowerOutput(search)
                .nonPeriodic()
                .bounded(-5.0, 20.0)
                .nativeUnits()
                .needsReference("home required")
                .positionTolerance(0.1)
                .targetFromNewCommand(4.0)
                .build();
        PositionPlantTuning tuning = PositionPlantTunings.claim(plant);
        ManualLoopClock time = new ManualLoopClock();

        assertIllegalState(
                () -> tuning.prepareHoldAtCurrent(time.clock()), "needs a physical reference");

        plant.beginCalibrationSearch(-0.2);
        plant.update(time.clock());
        assertEquals(1, search.writes);
        assertEquals(0, normal.writes);

        plant.establishReferenceAt(3.0, time.nextCycle(0.1));
        assertIllegalState(
                () -> tuning.prepareHoldAtCurrent(time.clock()), "calibration search owns output");
        plant.endCalibrationSearch();
        assertEquals(3.0, tuning.prepareHoldAtCurrent(time.clock()), 0.0);
        assertEquals(3.0, plant.commandTarget().get(), 0.0);
        assertEquals("preparation itself does not realize the normal output", 0, normal.writes);

        plant.update(time.clock());
        assertEquals(1, normal.writes);
        assertEquals(12.0, normal.lastPosition, 0.0);
    }

    @Test
    public void stillUnreferencedHeartbeatRemainsPositionPreparationEligible() {
        RecordingPositionOutput normal = new RecordingPositionOutput();
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(normal, clock -> 12.0)
                .nonPeriodic()
                .bounded(-5.0, 20.0)
                .nativeUnits()
                .needsReference("home required")
                .positionTolerance(0.1)
                .targetFromNewCommand(4.0)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        plant.update(time.clock());
        PositionPlantTuning tuning = PositionPlantTunings.claim(plant);
        plant.establishReferenceAt(3.0, time.clock());

        assertEquals(3.0, tuning.prepareHoldAtCurrent(time.clock()), 0.0);
        assertEquals(3.0, plant.commandTarget().get(), 0.0);
        assertEquals(0, normal.writes);
    }

    @Test
    public void pendingAssumeCurrentSourceCannotClaimInsideItsFirstNormalRealization() {
        RecordingPositionOutput normal = new RecordingPositionOutput();
        PositionPlant[] plantRef = new PositionPlant[1];
        RuntimeException[] claimFailure = new RuntimeException[1];
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(normal, clock -> {
                    try {
                        PositionPlantTunings.claim(plantRef[0]);
                    } catch (RuntimeException failure) {
                        claimFailure[0] = failure;
                    }
                    return 12.0;
                })
                .nonPeriodic()
                .bounded(-5.0, 20.0)
                .nativeUnits()
                .assumeCurrentPositionIs(3.0)
                .positionTolerance(0.1)
                .targetFromNewCommand(4.0)
                .build();
        plantRef[0] = plant;

        plant.update(new ManualLoopClock().clock());

        assertTrue(claimFailure[0] instanceof IllegalStateException);
        assertTrue(claimFailure[0].getMessage(),
                claimFailure[0].getMessage().contains("never from inside one"));
        assertEquals(1, normal.writes);
        assertEquals(13.0, normal.lastPosition, 0.0);
        assertIllegalState(() -> PositionPlantTunings.claim(plant), "first normal");
    }

    @Test
    public void nonfinitePendingAssumeSampleLeavesLaterClaimAndPreparationEligible() {
        RecordingPositionOutput normal = new RecordingPositionOutput();
        int[] samples = {0};
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(normal,
                        clock -> samples[0]++ == 0 ? Double.NaN : 12.0)
                .nonPeriodic()
                .bounded(-5.0, 20.0)
                .nativeUnits()
                .assumeCurrentPositionIs(3.0)
                .positionTolerance(0.1)
                .targetFromNewCommand(4.0)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        plant.update(time.clock());
        assertEquals(0, normal.writes);

        PositionPlantTuning tuning = PositionPlantTunings.claim(plant);
        time.nextCycle(0.1);
        assertEquals(3.0, tuning.prepareHoldAtCurrent(time.clock()), 0.0);
        assertEquals(3.0, plant.commandTarget().get(), 0.0);
        assertEquals(0, normal.writes);

        plant.update(time.clock());
        assertEquals(1, normal.writes);
        assertEquals(12.0, normal.lastPosition, 0.0);
        assertEquals(2, samples[0]);
    }

    @Test
    public void reentrantReferenceCannotClaimTuningInsideNormalRealization() {
        RecordingPositionOutput normal = new RecordingPositionOutput();
        PositionPlant[] plantRef = new PositionPlant[1];
        RuntimeException[] claimFailure = new RuntimeException[1];
        PlantTargetResolver reentrantResolver = (context, clock) -> {
            plantRef[0].establishReferenceAt(0.0);
            try {
                PositionPlantTunings.claim(plantRef[0]);
            } catch (RuntimeException failure) {
                claimFailure[0] = failure;
            }
            return PlantTargetResolution.exact(0.0, "reentrant reference test");
        };
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(normal, clock -> 12.0)
                .nonPeriodic()
                .bounded(-5.0, 20.0)
                .nativeUnits()
                .needsReference("home required")
                .positionTolerance(0.1)
                .targetFromResolver(reentrantResolver)
                .build();
        plantRef[0] = plant;

        plant.update(new ManualLoopClock().clock());

        assertTrue(claimFailure[0] instanceof IllegalStateException);
        assertTrue(claimFailure[0].getMessage(),
                claimFailure[0].getMessage().contains("never from inside one"));
        assertEquals(1, normal.writes);
        assertIllegalState(() -> PositionPlantTunings.claim(plant), "first normal");
    }

    @Test
    public void claimedTuningCannotPrepareHoldInsideNormalRealization() {
        RecordingPositionOutput normal = new RecordingPositionOutput();
        PositionPlant[] plantRef = new PositionPlant[1];
        PositionPlantTuning[] tuningRef = new PositionPlantTuning[1];
        RuntimeException[] preparationFailure = new RuntimeException[1];
        ManualLoopClock time = new ManualLoopClock();
        ScalarTarget reentrantTarget = new ScalarTarget() {
            private boolean callbackAttempted;
            private double value;

            @Override
            public void set(double value) {
                this.value = value;
            }

            @Override
            public double get() {
                if (!callbackAttempted && plantRef[0] != null && tuningRef[0] != null) {
                    callbackAttempted = true;
                    plantRef[0].establishReferenceAt(0.0);
                    try {
                        tuningRef[0].prepareHoldAtCurrent(time.clock());
                    } catch (RuntimeException failure) {
                        preparationFailure[0] = failure;
                    }
                }
                return value;
            }
        };
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(normal, clock -> 12.0)
                .nonPeriodic()
                .bounded(-5.0, 20.0)
                .nativeUnits()
                .needsReference("home required")
                .positionTolerance(0.1)
                .targetFromResolver(PlantTargets.exact(reentrantTarget))
                .build();
        plantRef[0] = plant;
        tuningRef[0] = PositionPlantTunings.claim(plant);

        plant.update(time.clock());

        assertTrue(preparationFailure[0] instanceof IllegalStateException);
        assertTrue(preparationFailure[0].getMessage(),
                preparationFailure[0].getMessage().contains("never from inside one"));
        assertEquals(1, normal.writes);
        assertIllegalState(() -> tuningRef[0].prepareHoldAtCurrent(time.clock()), "first normal");
    }

    @Test
    public void positionPreparationRejectsNonExactAndLateNormalRealization() {
        ScalarTarget command = ScalarTarget.create(0.0);
        PositionPlant periodic = Plants.fromOutputs()
                .regulatedPosition(new RecordingPowerOutput(), clock -> 0.0)
                .periodic(360.0)
                .bounded(-720.0, 720.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.1)
                .targetFromResolver(PlantTargets.equivalentPositionsOf(command)
                        .nearestToMeasurement()
                        .whenUnavailable()
                        .holdLastTarget(0.0))
                .build();
        StandardControlTuning periodicTuning = StandardControlTunings.claimPosition(periodic);
        assertFalse(periodicTuning.hasExactCommandTarget());
        assertIllegalState(
                () -> periodicTuning.prepareHoldAtCurrent(new ManualLoopClock().clock()),
                "one exact");

        PositionPlant late = Plants.fromOutputs()
                .deviceManagedPosition(new RecordingPositionOutput(), clock -> 0.0)
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .targetFromNewCommand(0.0)
                .build();
        late.update(new ManualLoopClock().clock());
        assertIllegalState(() -> PositionPlantTunings.claim(late), "first normal");
    }

    @Test
    public void standardClaimRejectsDirectCustomAndLateControllerShapes() {
        Plant direct = Plants.fromOutputs()
                .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.1)
                .targetFromNewCommand(0.0)
                .build();
        assertIllegalState(() -> StandardControlTunings.claimVelocity(direct), "device-managed");

        Plant custom = Plants.fromOutputs()
                .regulatedVelocity(new RecordingPowerOutput(), clock -> 0.0)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.1)
                .controlFromCustomRegulator((goal, measurement, clock) -> 0.0)
                .targetFromNewCommand(0.0)
                .build();
        assertIllegalState(() -> StandardControlTunings.claimVelocity(custom), "custom");

        Plant late = directVelocityPlant(new RecordingPowerOutput(), 0.0, 0.1);
        late.update(new ManualLoopClock().clock());
        assertIllegalState(() -> StandardControlTunings.claimVelocity(late), "first update");
    }

    private static Plant directVelocityPlant(RecordingPowerOutput output,
                                             double measurement,
                                             double kP) {
        return Plants.fromOutputs()
                .regulatedVelocity(output, clock -> measurement)
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .velocityTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(kP)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static void assertIllegalState(Runnable action, String messageFragment) {
        try {
            action.run();
            fail("Expected IllegalStateException");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messageFragment));
        }
    }

    private static void assertIllegalArgument(Runnable action, String messageFragment) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messageFragment));
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        private double lastPower = Double.NaN;
        private int writes;

        @Override
        public void setPower(double power) {
            lastPower = power;
            writes++;
        }

        @Override
        public double getCommandedPower() {
            return lastPower;
        }

        @Override
        public void stop() {
            lastPower = 0.0;
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double lastPosition = Double.NaN;
        private int writes;

        @Override
        public void setPosition(double position) {
            lastPosition = position;
            writes++;
        }

        @Override
        public double getCommandedPosition() {
            return lastPosition;
        }

        @Override
        public void stop() { }
    }

    private static final class RecordingVelocityOutput implements VelocityOutput {
        private double velocity;

        @Override
        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public double getCommandedVelocity() {
            return velocity;
        }
    }
}
