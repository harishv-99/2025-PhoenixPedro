package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the package-private standard control engine consumed by regulated Plant recipes. */
public final class StandardControlTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void runtimeIsPackagePrivateAndUsesExistingRegulatorSeam() {
        assertFalse(Modifier.isPublic(StandardControl.class.getModifiers()));
        assertTrue(Modifier.isFinal(StandardControl.class.getModifiers()));
        assertTrue(ScalarRegulator.class.isAssignableFrom(StandardControl.class));
        assertEquals(0, StandardControl.class.getConstructors().length);
    }

    @Test
    public void directSetpointsExposeOnlyTruthfulKinematicFacts() {
        ManualLoopClock positionTime = new ManualLoopClock();
        StandardControl position = StandardControl.positionFromAppliedTarget()
                .feedbackFromPid(0.2)
                .build();

        assertEquals(0.6, position.update(5.0, 2.0, positionTime.clock()), EPSILON);
        assertTrue(position.setpointPositionAvailable());
        assertTrue(position.setpointAccelerationAvailable());
        assertEquals(5.0, position.setpointPosition(), 0.0);
        assertEquals(0.0, position.setpointVelocity(), 0.0);
        assertEquals(0.0, position.setpointAcceleration(), 0.0);
        assertTrue(position.setpointSettledAt(5.0));

        ManualLoopClock velocityTime = new ManualLoopClock();
        StandardControl velocity = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.1)
                .build();

        assertEquals(0.3, velocity.update(7.0, 4.0, velocityTime.clock()), EPSILON);
        assertFalse(velocity.setpointPositionAvailable());
        assertFalse(velocity.setpointAccelerationAvailable());
        assertTrue(Double.isNaN(velocity.setpointPosition()));
        assertEquals(7.0, velocity.setpointVelocity(), 0.0);
        assertTrue(Double.isNaN(velocity.setpointAcceleration()));
        assertTrue(velocity.setpointSettledAt(7.0));
    }

    @Test
    public void accelerationLimitedVelocitySeedsWithoutChargingPriorTimeThenRamps() {
        ManualLoopClock time = new ManualLoopClock();
        StandardControl control = StandardControl.velocityFromAccelerationLimitedProfile(2.0)
                .feedbackFromPid(0.0)
                .build();

        control.update(10.0, 3.0, time.clock());
        assertEquals(3.0, control.setpointVelocity(), 0.0);
        assertEquals(0.0, control.setpointAcceleration(), 0.0);
        assertFalse(control.setpointSettledAt(10.0));

        control.update(10.0, 3.0, time.nextCycle(0.5));
        assertEquals(4.0, control.setpointVelocity(), EPSILON);
        assertEquals(2.0, control.setpointAcceleration(), EPSILON);

        control.update(3.5, 3.0, time.nextCycle(0.5));
        assertEquals("The prior goal owns the elapsed interval before boundary replanning",
                5.0, control.setpointVelocity(), EPSILON);
        assertEquals(-2.0, control.setpointAcceleration(), EPSILON);
        control.update(3.5, 3.0, time.nextCycle(0.25));
        assertEquals(4.5, control.setpointVelocity(), EPSILON);
        assertEquals(-2.0, control.setpointAcceleration(), EPSILON);
        control.update(3.5, 3.0, time.nextCycle(0.5));
        assertEquals(3.5, control.setpointVelocity(), EPSILON);
        assertEquals(-2.0, control.setpointAcceleration(), EPSILON);
        assertTrue(control.setpointSettledAt(3.5));
    }

    @Test
    public void continuouslyChangingProfileGoalsStillAdvanceEachElapsedInterval() {
        ManualLoopClock velocityTime = new ManualLoopClock();
        StandardControl velocity = StandardControl.velocityFromAccelerationLimitedProfile(2.0)
                .feedbackFromPid(0.0)
                .build();
        velocity.update(1.0, 0.0, velocityTime.clock());
        velocity.update(2.0, 0.0, velocityTime.nextCycle(0.1));
        assertEquals(0.2, velocity.setpointVelocity(), EPSILON);
        velocity.update(3.0, 0.0, velocityTime.nextCycle(0.1));
        assertEquals("a changing goal must not freeze the retained velocity profile",
                0.4, velocity.setpointVelocity(), EPSILON);

        ManualLoopClock positionTime = new ManualLoopClock();
        StandardControl position = StandardControl.positionFromTrapezoidalProfile(3.0, 2.0)
                .feedbackFromPid(0.0)
                .build();
        position.update(1.0, 0.0, positionTime.clock());
        position.update(2.0, 0.0, positionTime.nextCycle(0.1));
        assertEquals(0.01, position.setpointPosition(), EPSILON);
        position.update(3.0, 0.0, positionTime.nextCycle(0.1));
        assertEquals("a changing goal must not freeze the retained position profile",
                0.04, position.setpointPosition(), EPSILON);
        assertEquals(0.4, position.setpointVelocity(), EPSILON);
    }

    @Test
    public void trapezoidalPositionRespectsLimitsAndEventuallySettles() {
        ManualLoopClock time = new ManualLoopClock();
        StandardControl control = StandardControl.positionFromTrapezoidalProfile(3.0, 2.0)
                .feedbackFromPid(0.0)
                .build();

        control.update(8.0, 0.0, time.clock());
        assertEquals(0.0, control.setpointPosition(), 0.0);
        assertEquals(0.0, control.setpointVelocity(), 0.0);

        double priorVelocity = control.setpointVelocity();
        for (int i = 0; i < 400 && !control.setpointSettledAt(8.0); i++) {
            control.update(8.0, control.setpointPosition(), time.nextCycle(0.02));
            assertTrue(Math.abs(control.setpointVelocity()) <= 3.0 + EPSILON);
            assertTrue(Math.abs(control.setpointVelocity() - priorVelocity) <= 2.0 * 0.02 + EPSILON);
            assertTrue(Math.abs(control.setpointAcceleration()) <= 2.0 + EPSILON);
            priorVelocity = control.setpointVelocity();
        }

        assertTrue(control.setpointSettledAt(8.0));
        assertEquals(8.0, control.setpointPosition(), EPSILON);
        assertEquals(0.0, control.setpointVelocity(), EPSILON);
    }

    @Test
    public void trapezoidalPositionDoesNotOvershootEitherLegalBoundaryGoal() {
        assertProfileStaysOnGoalSide(0.0, 8.0);
        assertProfileStaysOnGoalSide(8.0, 0.0);
    }

    @Test
    public void shortPositionMoveIsTriangularWhileLongMoveReachesCruiseVelocity() {
        StandardControl shortMove = StandardControl.positionFromTrapezoidalProfile(5.0, 4.0)
                .feedbackFromPid(0.0)
                .build();
        ManualLoopClock shortTime = new ManualLoopClock();
        shortMove.update(1.0, 0.0, shortTime.clock());
        double shortPeak = runProfileAndPeak(shortMove, shortTime, 1.0, 300);
        assertTrue(shortPeak < 5.0 - 0.1);
        assertTrue(shortMove.setpointSettledAt(1.0));

        StandardControl longMove = StandardControl.positionFromTrapezoidalProfile(2.0, 4.0)
                .feedbackFromPid(0.0)
                .build();
        ManualLoopClock longTime = new ManualLoopClock();
        longMove.update(20.0, 0.0, longTime.clock());
        double longPeak = runProfileAndPeak(longMove, longTime, 20.0, 1400);
        assertEquals(2.0, longPeak, 1e-6);
        assertTrue(longMove.setpointSettledAt(20.0));
    }

    @Test
    public void motionLiftAndArmFeedforwardUseOneSetpointSnapshot() {
        ManualLoopClock velocityTime = new ManualLoopClock();
        StandardControl velocity = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.0)
                .feedforwardForMotion(0.1, 0.02)
                .build();
        assertEquals(0.16, velocity.update(3.0, 3.0, velocityTime.clock()), EPSILON);
        assertEquals(0.16, velocity.feedforwardOutput(), EPSILON);

        ManualLoopClock zeroTime = new ManualLoopClock();
        StandardControl zeroVelocity = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.0)
                .feedforwardForMotion(0.4, 0.0)
                .build();
        assertEquals("sign(0) is exactly zero", 0.0,
                zeroVelocity.update(0.0, 0.0, zeroTime.clock()), 0.0);

        ManualLoopClock liftTime = new ManualLoopClock();
        StandardControl lift = StandardControl.positionFromAppliedTarget()
                .feedbackFromPid(0.0)
                .feedforwardForLift(-0.23)
                .build();
        assertEquals(-0.23, lift.update(10.0, 10.0, liftTime.clock()), EPSILON);

        ManualLoopClock armTime = new ManualLoopClock();
        StandardControl arm = StandardControl.positionFromAppliedTarget()
                .feedbackFromPid(0.0)
                .feedforwardForArm(0.3, 10.0, Math.PI / 20.0)
                .build();
        assertEquals(0.3, arm.update(10.0, 10.0, armTime.clock()), EPSILON);
        arm.update(20.0, 20.0, armTime.nextCycle(0.02));
        assertEquals(0.0, arm.feedforwardOutput(), EPSILON);
    }

    @Test
    public void accelerationFeedforwardUsesRampAcceleration() {
        ManualLoopClock time = new ManualLoopClock();
        StandardControl control = StandardControl.velocityFromAccelerationLimitedProfile(4.0)
                .feedbackFromPid(0.0)
                .feedforwardForLift(0.1, 0.02, 0.03, 0.04)
                .build();

        control.update(8.0, 0.0, time.clock());
        assertEquals(0.04, control.output(), EPSILON);
        control.update(8.0, 0.0, time.nextCycle(0.5));
        assertEquals(2.0, control.setpointVelocity(), EPSILON);
        assertEquals(4.0, control.setpointAcceleration(), EPSILON);
        assertEquals(0.1 + 0.02 * 2.0 + 0.03 * 4.0 + 0.04,
                control.feedforwardOutput(), EPSILON);
    }

    @Test
    public void voltageCompensationPrecedesFinalOutputPolicy() {
        CountingScalarSource voltage = new CountingScalarSource(10.0);
        ManualLoopClock time = new ManualLoopClock();
        StandardControl control = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.1)
                .voltageCompensatedBy(voltage, 12.0, 9.0, 1.5)
                .outputPowerLimitedTo(0.5)
                .build();

        assertEquals(0.5, control.update(5.0, 0.0, time.clock()), EPSILON);
        assertTrue(control.outputLimited());
        assertEquals(1, voltage.samples);
        control.update(5.0, 0.0, time.clock());
        assertEquals("same-cycle evaluation is memoized", 1, voltage.samples);
        assertEquals(0.5, control.output(), EPSILON);
    }

    @Test
    public void resetDoesNotResetBorrowedVoltageSource() {
        CountingScalarSource voltage = new CountingScalarSource(12.0);
        StandardControl control = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.1)
                .voltageCompensatedBy(voltage, 12.0, 9.0, 1.5)
                .build();
        control.update(1.0, 0.0, new ManualLoopClock().clock());

        control.reset();

        assertEquals("the control recipe borrows but does not own its voltage source lifecycle",
                0, voltage.resets);
    }

    @Test
    public void antiWindupStopsGrowthIntoSaturationButAllowsUnwind() {
        ManualLoopClock time = new ManualLoopClock();
        StandardControl control = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.0, 1.0, 0.0)
                .feedbackIntegralLimitedTo(-2.0, 2.0)
                .outputPowerLimitedTo(0.5)
                .build();

        control.update(1.0, 0.0, time.clock());
        control.update(1.0, 0.0, time.nextCycle(1.0));
        double saturatedIntegral = control.integralContribution();
        assertTrue(control.integralGrowthBlocked());
        assertTrue(saturatedIntegral <= 0.5 + EPSILON);

        control.update(1.0, 0.0, time.nextCycle(1.0));
        assertEquals(saturatedIntegral, control.integralContribution(), EPSILON);
        assertTrue(control.integralGrowthBlocked());

        control.update(-1.0, 0.0, time.nextCycle(0.1));
        assertEquals("changed goal consumes no preceding dt",
                saturatedIntegral, control.integralContribution(), EPSILON);
        control.update(-1.0, 0.0, time.nextCycle(0.25));
        assertTrue(control.integralContribution() < saturatedIntegral);
        assertFalse("integration away from positive saturation is allowed",
                control.integralGrowthBlocked());
    }

    @Test
    public void repeatedSuccessAndFailureAreRetainedPerCycle() {
        CountingScalarSource voltage = new CountingScalarSource(12.0);
        ManualLoopClock time = new ManualLoopClock();
        StandardControl success = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.1)
                .voltageCompensatedBy(voltage, 12.0, 9.0, 1.5)
                .build();

        double first = success.update(2.0, 0.0, time.clock());
        double second = success.update(2.0, 0.0, time.clock());
        assertEquals(first, second, 0.0);
        assertEquals(1, voltage.samples);

        RuntimeException changedSnapshotFailure;
        try {
            success.update(-8.0, 5.0, time.clock());
            fail("Expected changed same-cycle input rejection");
            return;
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("one immutable control snapshot"));
            changedSnapshotFailure = expected;
        }
        try {
            success.update(2.0, 0.0, time.clock());
            fail("Expected retained changed-snapshot failure");
        } catch (RuntimeException expected) {
            assertSame(changedSnapshotFailure, expected);
        }
        assertEquals("rejected input does not resample voltage", 1, voltage.samples);

        StandardControl failure = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(Double.MAX_VALUE)
                .build();
        RuntimeException retained;
        try {
            failure.update(Double.MAX_VALUE, -Double.MAX_VALUE, time.clock());
            fail("Expected overflowing control math to fail");
            return;
        } catch (RuntimeException expected) {
            retained = expected;
        }
        assertFalse("failed first evaluation must not publish a settled setpoint",
                failure.setpointSettled());
        assertTrue("failed first evaluation must not publish setpoint velocity",
                Double.isNaN(failure.setpointVelocity()));
        try {
            failure.update(0.0, 0.0, time.clock());
            fail("Expected same-cycle failure replay");
        } catch (RuntimeException expected) {
            assertSame(retained, expected);
        }
        assertEquals(0.0, failure.update(0.0, 0.0, time.nextCycle(0.02)), 0.0);
    }

    @Test
    public void resetReleasesClockAndReseedStartsWithoutElapsedTime() {
        StandardControl control = StandardControl.velocityFromAccelerationLimitedProfile(2.0)
                .feedbackFromPid(0.0)
                .build();
        ManualLoopClock first = new ManualLoopClock();
        ManualLoopClock second = new ManualLoopClock();

        control.update(4.0, 0.0, first.clock());
        assertDifferentClockRejected(control, second.clock());

        control.reset();
        control.update(4.0, 1.5, second.nextCycle(5.0));
        assertEquals(1.5, control.setpointVelocity(), 0.0);

        second.nextCycle(3.0);
        control.reseed(-2.0, second.clock());
        control.update(4.0, -2.0, second.clock());
        assertEquals(-2.0, control.setpointVelocity(), 0.0);
        assertEquals(0.0, control.setpointAcceleration(), 0.0);
    }

    @Test
    public void typedFeedforwardAndConfigurationRejectInvalidCombinations() {
        assertIllegalState(new Runnable() {
            @Override
            public void run() {
                StandardControl.positionFromAppliedTarget()
                        .feedbackFromPid(0.0)
                        .feedforwardForMotion(0.1, 0.1);
            }
        }, "direct position");
        assertIllegalState(new Runnable() {
            @Override
            public void run() {
                StandardControl.velocityFromAppliedTarget()
                        .feedbackFromPid(0.0)
                        .feedforwardForMotion(0.1, 0.1, 0.1);
            }
        }, "acceleration evidence");
        assertIllegalState(new Runnable() {
            @Override
            public void run() {
                StandardControl.velocityFromAppliedTarget()
                        .feedbackFromPid(0.0)
                        .feedforwardForArm(0.2, 0.0, 1.0);
            }
        }, "position setpoint");
        assertIllegalArgument(new Runnable() {
            @Override
            public void run() {
                StandardControl.positionFromAppliedTarget()
                        .feedbackFromPid(0.0)
                        .feedforwardForArm(0.2, 0.0, 0.0);
            }
        }, "non-zero");
        assertIllegalArgument(new Runnable() {
            @Override
            public void run() {
                StandardControl.velocityFromAppliedTarget()
                        .feedbackFromPid(0.0)
                        .outputPowerLimitedTo(-0.1);
            }
        }, "[0.0, 1.0]");
        assertIllegalArgument(new Runnable() {
            @Override
            public void run() {
                StandardControl.velocityFromAppliedTarget()
                        .feedbackFromPid(0.0)
                        .outputPowerLimitedTo(0.2, 0.8);
            }
        }, "minimum <= 0");
    }

    @Test
    public void diagnosticsExposeRetainedSnapshotWithoutSampling() {
        CountingScalarSource voltage = new CountingScalarSource(12.0);
        StandardControl control = StandardControl.velocityFromAppliedTarget()
                .feedbackFromPid(0.1)
                .voltageCompensatedBy(voltage, 12.0, 9.0, 1.5)
                .build();
        control.update(2.0, 1.0, new ManualLoopClock().clock());

        CapturingDebugSink sink = new CapturingDebugSink();
        control.debugDump(sink, "lift.control");
        control.debugDump(sink, "lift.control");

        assertEquals(1, voltage.samples);
        assertEquals("StandardControl", sink.data.get("lift.control.class"));
        assertEquals("VELOCITY_DIRECT", sink.data.get("lift.control.setpointMode"));
        assertEquals(2.0, (Double) sink.data.get("lift.control.goal"), 0.0);
        assertEquals(0.1, (Double) sink.data.get("lift.control.output.command"), EPSILON);
    }

    private static double runProfileAndPeak(StandardControl control,
                                            ManualLoopClock time,
                                            double goal,
                                            int maximumCycles) {
        double peak = 0.0;
        for (int i = 0; i < maximumCycles && !control.setpointSettledAt(goal); i++) {
            control.update(goal, control.setpointPosition(), time.nextCycle(0.01));
            peak = Math.max(peak, Math.abs(control.setpointVelocity()));
        }
        return peak;
    }

    private static void assertProfileStaysOnGoalSide(double initialPosition, double goal) {
        ManualLoopClock time = new ManualLoopClock();
        StandardControl control = StandardControl.positionFromTrapezoidalProfile(3.0, 2.0)
                .feedbackFromPid(0.0)
                .build();
        control.update(goal, initialPosition, time.clock());

        double direction = Math.signum(goal - initialPosition);
        double priorPosition = initialPosition;
        for (int i = 0; i < 200 && !control.setpointSettledAt(goal); i++) {
            control.update(goal, control.setpointPosition(), time.nextCycle(0.17));
            double position = control.setpointPosition();
            assertTrue("profile position must move toward the boundary goal",
                    direction * (position - priorPosition) >= -EPSILON);
            assertTrue("profile position must not cross the legal boundary goal",
                    direction * (goal - position) >= -EPSILON);
            assertTrue(Math.abs(control.setpointVelocity()) <= 3.0 + EPSILON);
            assertTrue(Math.abs(control.setpointAcceleration()) <= 2.0 + EPSILON);
            priorPosition = position;
        }
        assertTrue(control.setpointSettledAt(goal));
        assertEquals(goal, control.setpointPosition(), EPSILON);
    }

    private static void assertDifferentClockRejected(StandardControl control, LoopClock clock) {
        try {
            control.update(0.0, 0.0, clock);
            fail("Expected different LoopClock identity rejection");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("different clock identity"));
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

    private static void assertIllegalState(Runnable action, String messageFragment) {
        try {
            action.run();
            fail("Expected IllegalStateException");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messageFragment));
        }
    }

    private static final class CountingScalarSource implements ScalarSource {
        private final double value;
        private int samples;
        private int resets;

        private CountingScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            samples++;
            return value;
        }

        @Override
        public void reset() {
            resets++;
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
