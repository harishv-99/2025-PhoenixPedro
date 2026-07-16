package edu.ftcphoenix.fw.core.control;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies scalar-regulator construction, composition, lifecycle, and diagnostics. */
public final class ScalarRegulatorsTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void outputLimitedRejectsNullInnerRegulator() {
        try {
            ScalarRegulators.outputLimited(null, -1.0, 1.0);
            fail("Expected a null inner regulator to be rejected");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("inner"));
        }
    }

    @Test
    public void outputLimitedRejectsEveryNonFiniteAndReversedBound() {
        assertInvalidBound(Double.NaN, 1.0, "minOutput", Double.NaN);
        assertInvalidBound(Double.NEGATIVE_INFINITY, 1.0,
                "minOutput", Double.NEGATIVE_INFINITY);
        assertInvalidBound(Double.POSITIVE_INFINITY, 1.0,
                "minOutput", Double.POSITIVE_INFINITY);
        assertInvalidBound(-1.0, Double.NaN, "maxOutput", Double.NaN);
        assertInvalidBound(-1.0, Double.NEGATIVE_INFINITY,
                "maxOutput", Double.NEGATIVE_INFINITY);
        assertInvalidBound(-1.0, Double.POSITIVE_INFINITY,
                "maxOutput", Double.POSITIVE_INFINITY);
        assertReversedBoundsRejected();
    }

    @Test
    public void outputLimitedAcceptsEqualBounds() {
        LoopClock clock = new ManualLoopClock().clock();
        ScalarRegulator constrained = ScalarRegulators.outputLimited(
                constantRegulator(2.0), 0.25, 0.25);

        assertEquals(0.25, constrained.update(0.0, 0.0, clock), 0.0);
        assertLastOutputLimited(constrained, true);

        ScalarRegulator alreadyAtOnlyAllowedValue = ScalarRegulators.outputLimited(
                constantRegulator(0.25), 0.25, 0.25);
        assertEquals(0.25, alreadyAtOnlyAllowedValue.update(0.0, 0.0, clock), 0.0);
        assertLastOutputLimited(alreadyAtOnlyAllowedValue, false);
    }

    @Test
    public void exactBoundsAreInclusiveAndOneUlpExcursionsAreLimited() {
        MutableRegulator inner = new MutableRegulator();
        ScalarRegulator constrained = ScalarRegulators.outputLimited(inner, -0.5, 0.5);
        LoopClock clock = new ManualLoopClock().clock();

        inner.output = -0.5;
        assertEquals(-0.5, constrained.update(1.0, 2.0, clock), 0.0);
        assertLastOutputLimited(constrained, false);

        inner.output = 0.5;
        assertEquals(0.5, constrained.update(1.0, 2.0, clock), 0.0);
        assertLastOutputLimited(constrained, false);

        inner.output = Math.nextDown(-0.5);
        assertEquals(-0.5, constrained.update(1.0, 2.0, clock), 0.0);
        assertLastOutputLimited(constrained, true);

        inner.output = Math.nextUp(0.5);
        assertEquals(0.5, constrained.update(1.0, 2.0, clock), 0.0);
        assertLastOutputLimited(constrained, true);
    }

    @Test
    public void outputLimitedConstrainsPositiveAndNegativeResults() {
        MutableRegulator inner = new MutableRegulator();
        ScalarRegulator constrained = ScalarRegulators.outputLimited(inner, -0.4, 0.7);
        LoopClock clock = new ManualLoopClock().clock();

        inner.output = 4.0;
        assertEquals(0.7, constrained.update(0.0, 0.0, clock), 0.0);

        inner.output = -4.0;
        assertEquals(-0.4, constrained.update(0.0, 0.0, clock), 0.0);

        inner.output = 0.3;
        assertEquals(0.3, constrained.update(0.0, 0.0, clock), 0.0);
        assertLastOutputLimited(constrained, false);
    }

    @Test
    public void outerLimitAppliesAfterPidLimitAndSetpointFeedforward() {
        Pid pid = Pid.withGains(1.0, 0.0, 0.0)
                .setOutputLimits(-0.65, 0.65);
        ScalarRegulator pidf = ScalarRegulators.pidf(pid, setpoint -> 0.20);
        LoopClock clock = new ManualLoopClock().clock();

        assertEquals(0.85, pidf.update(1.0, 0.0, clock), EPSILON);

        ScalarRegulator completeCommandLimit = ScalarRegulators.outputLimited(pidf, 0.0, 0.65);
        assertEquals(0.65, completeCommandLimit.update(1.0, 0.0, clock), EPSILON);
        assertLastOutputLimited(completeCommandLimit, true);
    }

    @Test
    public void limiterMustBeOutsideVoltageCompensationToBoundCompleteCommand() {
        ScalarRegulator nominal = constantRegulator(0.80);
        ScalarSource tenVolts = clock -> 10.0;
        LoopClock clock = new ManualLoopClock().clock();

        ScalarRegulator correctOrder = ScalarRegulators.outputLimited(
                ScalarRegulators.voltageCompensated(nominal, tenVolts,
                        13.0, 9.0, 1.4),
                0.0,
                0.60);
        assertEquals(0.60, correctOrder.update(0.0, 0.0, clock), EPSILON);

        ScalarRegulator deliberatelyIncorrectOrder = ScalarRegulators.voltageCompensated(
                ScalarRegulators.outputLimited(nominal, 0.0, 0.60),
                tenVolts,
                13.0,
                9.0,
                1.4);
        double incorrectlyOrderedResult = deliberatelyIncorrectOrder.update(0.0, 0.0, clock);
        assertEquals(0.78, incorrectlyOrderedResult, EPSILON);
        assertTrue(incorrectlyOrderedResult > 0.60);
    }

    @Test
    public void outputLimitedRejectsEveryNonFiniteInnerResult() {
        assertNonFiniteResultRejected(Double.NaN);
        assertNonFiniteResultRejected(Double.NEGATIVE_INFINITY);
        assertNonFiniteResultRejected(Double.POSITIVE_INFINITY);
    }

    @Test
    public void repeatedSameCycleUpdatesDelegateExactlyOncePerCall() {
        RecordingRegulator inner = new RecordingRegulator();
        ScalarRegulator constrained = ScalarRegulators.outputLimited(inner, -10.0, 10.0);
        LoopClock sameClock = new ManualLoopClock().clock();

        assertEquals(1.0, constrained.update(2.0, 3.0, sameClock), 0.0);
        assertEquals(2.0, constrained.update(4.0, 5.0, sameClock), 0.0);

        assertEquals(2, inner.updateCount);
        assertEquals(4.0, inner.lastSetpoint, 0.0);
        assertEquals(5.0, inner.lastMeasurement, 0.0);
        assertTrue(inner.lastClock == sameClock);
    }

    @Test
    public void resetDelegatesOnceAndClearsLocalDiagnostics() {
        RecordingRegulator inner = new RecordingRegulator();
        ScalarRegulator constrained = ScalarRegulators.outputLimited(inner, -0.5, 0.5);
        constrained.update(2.0, 3.0, new ManualLoopClock().clock());

        constrained.reset();

        assertEquals(1, inner.resetCount);
        CapturingDebugSink debug = new CapturingDebugSink();
        constrained.debugDump(debug, "regulator");
        assertNaN(debug.data.get("regulator.lastUnconstrainedOutput"));
        assertNaN(debug.data.get("regulator.lastOutput"));
        assertEquals(Boolean.FALSE, debug.data.get("regulator.lastOutputLimited"));
        assertEquals(-0.5, number(debug, "regulator.minOutput"), 0.0);
        assertEquals(0.5, number(debug, "regulator.maxOutput"), 0.0);
    }

    @Test
    public void debugDumpIsNullSafeAndUsesStableNestedKeys() {
        ScalarRegulator inner = new ScalarRegulator() {
            @Override
            public double update(double setpoint, double measurement, LoopClock clock) {
                return 0.8;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                dbg.addData(prefix + ".marker", "inner-debug");
            }
        };
        ScalarRegulator constrained = ScalarRegulators.outputLimited(inner, -0.3, 0.6);
        constrained.update(1200.0, 1100.0, new ManualLoopClock().clock());

        constrained.debugDump(null, "ignored");

        CapturingDebugSink named = new CapturingDebugSink();
        constrained.debugDump(named, "shooter.regulator");
        assertEquals("OutputLimitedScalarRegulator", named.data.get("shooter.regulator.class"));
        assertEquals(-0.3, number(named, "shooter.regulator.minOutput"), 0.0);
        assertEquals(0.6, number(named, "shooter.regulator.maxOutput"), 0.0);
        assertEquals(0.8, number(named, "shooter.regulator.lastUnconstrainedOutput"), 0.0);
        assertEquals(0.6, number(named, "shooter.regulator.lastOutput"), 0.0);
        assertEquals(Boolean.TRUE, named.data.get("shooter.regulator.lastOutputLimited"));
        assertEquals("inner-debug", named.data.get("shooter.regulator.inner.marker"));

        CapturingDebugSink defaultPrefix = new CapturingDebugSink();
        constrained.debugDump(defaultPrefix, null);
        assertEquals("OutputLimitedScalarRegulator",
                defaultPrefix.data.get("outputLimitedRegulator.class"));
        assertEquals("inner-debug",
                defaultPrefix.data.get("outputLimitedRegulator.inner.marker"));
    }

    private static ScalarRegulator constantRegulator(double output) {
        return (setpoint, measurement, clock) -> output;
    }

    private static void assertInvalidBound(double minOutput,
                                           double maxOutput,
                                           String invalidArgument,
                                           double invalidValue) {
        try {
            ScalarRegulators.outputLimited(constantRegulator(0.0), minOutput, maxOutput);
            fail("Expected invalid output bounds to be rejected");
        } catch (IllegalArgumentException expected) {
            String message = expected.getMessage();
            assertNotNull(message);
            assertTrue(message.contains(invalidArgument));
            assertTrue(message.contains(Double.toString(invalidValue)));
        }
    }

    private static void assertReversedBoundsRejected() {
        try {
            ScalarRegulators.outputLimited(constantRegulator(0.0), 1.0, -1.0);
            fail("Expected reversed output bounds to be rejected");
        } catch (IllegalArgumentException expected) {
            String message = expected.getMessage();
            assertNotNull(message);
            assertTrue(message.contains("minOutput"));
            assertTrue(message.contains("maxOutput"));
            assertTrue(message.contains("1.0"));
            assertTrue(message.contains("-1.0"));
            assertTrue(message.contains("<="));
        }
    }

    private static void assertNonFiniteResultRejected(double output) {
        ScalarRegulator constrained = ScalarRegulators.outputLimited(
                constantRegulator(output), -1.0, 1.0);
        try {
            constrained.update(2.0, 1.0, new ManualLoopClock().clock());
            fail("Expected a non-finite inner result to be rejected");
        } catch (IllegalStateException expected) {
            String message = expected.getMessage();
            assertNotNull(message);
            assertTrue(message.contains("non-finite"));
            assertTrue(message.contains(Double.toString(output)));
            assertTrue(message.contains("setpoint"));
            assertTrue(message.contains("measurement"));
            assertTrue(message.contains("feedforward"));
            assertTrue(message.contains("compensation"));
        }

        CapturingDebugSink debug = new CapturingDebugSink();
        constrained.debugDump(debug, "regulator");
        assertEquals(output, number(debug, "regulator.lastUnconstrainedOutput"), 0.0);
        assertNaN(debug.data.get("regulator.lastOutput"));
        assertEquals(Boolean.FALSE, debug.data.get("regulator.lastOutputLimited"));
    }

    private static void assertLastOutputLimited(ScalarRegulator regulator, boolean expected) {
        CapturingDebugSink debug = new CapturingDebugSink();
        regulator.debugDump(debug, "regulator");
        assertEquals(Boolean.valueOf(expected), debug.data.get("regulator.lastOutputLimited"));
    }

    private static double number(CapturingDebugSink debug, String key) {
        Object value = debug.data.get(key);
        assertTrue(key + " must contain a number", value instanceof Number);
        return ((Number) value).doubleValue();
    }

    private static void assertNaN(Object value) {
        assertTrue(value instanceof Number);
        assertTrue(Double.isNaN(((Number) value).doubleValue()));
    }

    private static final class MutableRegulator implements ScalarRegulator {
        private double output;

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            return output;
        }
    }

    private static final class RecordingRegulator implements ScalarRegulator {
        private int updateCount;
        private int resetCount;
        private double lastSetpoint = Double.NaN;
        private double lastMeasurement = Double.NaN;
        private LoopClock lastClock;

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            updateCount++;
            lastSetpoint = setpoint;
            lastMeasurement = measurement;
            lastClock = clock;
            return updateCount;
        }

        @Override
        public void reset() {
            resetCount++;
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
