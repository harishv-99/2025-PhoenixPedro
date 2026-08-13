package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Public-builder integration coverage for the package-private standard control runtime. */
public final class StandardControlPlantIntegrationTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void directPositionComposesPidGravityAndFinalOutputPolicyInline() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        int[] feedbackSamples = {0};
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> {
                    feedbackSamples[0]++;
                    return 0.0;
                })
                .nonPeriodic()
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.0)
                .feedforwardFromLift(0.25)
                .outputPowerLimitedTo(0.15)
                .targetFromNewCommand(0.0)
                .build();

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());

        assertEquals(0.15, output.lastPower, EPSILON);
        assertEquals(1, output.writes);
        assertEquals(1, feedbackSamples[0]);
        assertTrue(plant.atTarget());

        plant.update(time.clock());
        assertEquals("one immutable standard-control snapshot writes once per cycle",
                1, output.writes);
        assertEquals("the Plant claims the heartbeat before resampling feedback",
                1, feedbackSamples[0]);
    }

    @Test
    public void failureBeforeControlIsRetainedBeforeFeedbackCanBeResampled() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        RuntimeException feedbackFailure = new IllegalStateException("feedback failed");
        int[] feedbackSamples = {0};
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> {
                    feedbackSamples[0]++;
                    throw feedbackFailure;
                })
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.1)
                .targetFromNewCommand(0.0)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        RuntimeException first = expectRuntime(() -> plant.update(time.clock()));
        RuntimeException repeated = expectRuntime(() -> plant.update(time.clock()));

        assertSame(feedbackFailure, first);
        assertSame(first, repeated);
        assertEquals(1, feedbackSamples[0]);
        assertEquals(0, output.writes);
    }

    @Test
    public void positionCompletionWaitsForProfileEvenWhenMeasurementToleranceIsWide() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> 0.0)
                .nonPeriodic()
                .bounded(0.0, 20.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(20.0)
                .setpointFromTrapezoidalProfile(2.0, 4.0)
                .feedbackFromPid(0.0)
                .targetFromNewCommand(10.0)
                .build();

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        assertFalse("target and measurement tolerance do not bypass a moving setpoint",
                plant.atTarget());

        for (int i = 0; i < 500 && !plant.atTarget(); i++) {
            plant.update(time.nextCycle(0.02));
        }

        assertTrue("completion becomes true after the profiled setpoint reaches the applied goal",
                plant.atTarget());
    }

    @Test
    public void accelerationLimitedVelocityFeedsOneSharedVelocityAccelerationSnapshot() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> 0.0)
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .velocityTolerance(10.0)
                .setpointFromAccelerationLimitedProfile(2.0)
                .feedbackFromPid(0.0)
                .feedforwardFromMotion(0.1, 0.2, 0.3)
                .targetFromNewCommand(4.0)
                .build();

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        assertEquals("the first boundary seeds measured velocity and charges no prior dt",
                0.0, output.lastPower, EPSILON);

        plant.update(time.nextCycle(0.5));
        assertEquals("kS + kV*vSetpoint + kA*aSetpoint",
                0.1 + 0.2 * 1.0 + 0.3 * 2.0,
                output.lastPower,
                EPSILON);
        assertFalse(plant.atTarget());
    }

    @Test
    public void retainedPidAliasCannotReorderOrRepeatLaterControlStages() {
        Plants.PositionDirectPidStep pid = Plants.fromOutputs()
                .regulatedPosition(new RecordingPowerOutput(), clock -> 0.0)
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.1);

        Plants.OutputPowerPolicyStep<PositionPlant> output = pid.feedforwardFromLift(0.2);
        assertIllegalState(() -> pid.feedbackIntegralLimitedTo(-0.1, 0.1),
                "cannot change PID/feedforward");
        assertIllegalState(() -> pid.feedforwardFromArm(0.2, 0.0, 1.0),
                "cannot change PID/feedforward");

        output.outputPowerLimitedTo(0.5);
        assertIllegalState(() -> pid.feedbackOutputLimitedTo(-0.2, 0.2),
                "cannot change PID/feedforward");
    }

    @Test
    public void profiledSetpointsRejectTargetRateGuardsBeforeRetainingThem() {
        BooleanSource allowed = clock -> true;
        Plants.PositionProfiledPidStep position = Plants.fromOutputs()
                .regulatedPosition(new RecordingPowerOutput(), clock -> 0.0)
                .nonPeriodic()
                .bounded(-2.0, 2.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01)
                .setpointFromTrapezoidalProfile(1.0, 2.0)
                .feedbackFromPid(0.1);
        Plants.TargetGuardStep<PositionPlant> positionGuards = position.targetGuards();
        assertIllegalState(() -> positionGuards.maxTargetRate(1.0),
                "setpointFromTrapezoidalProfile", "one motion-shaping owner");
        assertTrue(positionGuards.holdLastTargetUnless("ready", allowed)
                .doneTargetGuards()
                .targetFromNewCommand(0.0) != null);

        Plants.VelocityProfiledPidStep velocity = Plants.fromOutputs()
                .regulatedVelocity(new RecordingPowerOutput(), clock -> 0.0)
                .bounded(-2.0, 2.0)
                .nativeUnits()
                .velocityTolerance(0.01)
                .setpointFromAccelerationLimitedProfile(2.0)
                .feedbackFromPid(0.1);
        Plants.TargetGuardStep<Plant> velocityGuards = velocity.targetGuards();
        assertIllegalState(() -> velocityGuards.maxTargetRates(1.0, 1.0),
                "setpointFromAccelerationLimitedProfile", "one motion-shaping owner");
        assertTrue(velocityGuards.holdLastTargetUnless("ready", allowed)
                .doneTargetGuards()
                .targetFromNewCommand(0.0) != null);
    }

    private static void assertIllegalState(Runnable action, String... messageFragments) {
        try {
            action.run();
            fail("Expected IllegalStateException");
        } catch (IllegalStateException expected) {
            for (String messageFragment : messageFragments) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(messageFragment));
            }
        }
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
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
}
