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

/** Verifies the framework-owned standard linear-PIDF regulator. */
public final class PidfRegulatorTest {

    private static final double EPSILON = 1e-12;
    private static final double[] NON_FINITE_VALUES = {
            Double.NaN,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY
    };

    @Test
    public void factoryRejectsEveryNonFiniteGainPositionWithActionableDiagnostics() {
        String[] gainNames = {"kP", "kI", "kD", "kF"};
        for (int gainIndex = 0; gainIndex < gainNames.length; gainIndex++) {
            for (double invalidValue : NON_FINITE_VALUES) {
                double[] gains = {1.0, 2.0, 3.0, 4.0};
                gains[gainIndex] = invalidValue;

                IllegalArgumentException failure = expectIllegalArgument(
                        "ScalarRegulators.pidf(...) "
                                + gainNames[gainIndex] + "=" + invalidValue,
                        () -> ScalarRegulators.pidf(
                                gains[0], gains[1], gains[2], gains[3]));
                assertGainDiagnostic(failure, "ScalarRegulators.pidf(...)",
                        gainNames[gainIndex], invalidValue);
            }
        }
    }

    @Test
    public void liveGainUpdateRejectsEveryNonFinitePositionBeforeAnyMutation() {
        String[] gainNames = {"kP", "kI", "kD", "kF"};
        for (int gainIndex = 0; gainIndex < gainNames.length; gainIndex++) {
            for (double invalidValue : NON_FINITE_VALUES) {
                PidfRegulator pidf = ScalarRegulators.pidf(1.0, 2.0, 3.0, 4.0);
                double[] candidate = {10.0, 20.0, 30.0, 40.0};
                candidate[gainIndex] = invalidValue;

                IllegalArgumentException failure = expectIllegalArgument(
                        "PidfRegulator.setGains(...) "
                                + gainNames[gainIndex] + "=" + invalidValue,
                        () -> pidf.setGains(
                                candidate[0], candidate[1], candidate[2], candidate[3]));
                assertGainDiagnostic(failure, "PidfRegulator.setGains(...)",
                        gainNames[gainIndex], invalidValue);

                assertEquals(1.0, pidf.getKP(), 0.0);
                assertEquals(2.0, pidf.getKI(), 0.0);
                assertEquals(3.0, pidf.getKD(), 0.0);
                assertEquals(4.0, pidf.getKF(), 0.0);
            }
        }
    }

    @Test
    public void diagnosticsNameEveryInvalidGainInOneCandidate() {
        IllegalArgumentException factoryFailure = expectIllegalArgument(
                "four-invalid-gain PIDF factory candidate",
                () -> ScalarRegulators.pidf(
                        Double.NaN,
                        Double.POSITIVE_INFINITY,
                        Double.NEGATIVE_INFINITY,
                        Double.NaN));
        assertTrue(factoryFailure.getMessage().contains("kP=NaN"));
        assertTrue(factoryFailure.getMessage().contains("kI=Infinity"));
        assertTrue(factoryFailure.getMessage().contains("kD=-Infinity"));
        assertTrue(factoryFailure.getMessage().contains("kF=NaN"));

        PidfRegulator pidf = ScalarRegulators.pidf(1.0, 2.0, 3.0, 4.0);
        IllegalArgumentException liveFailure = expectIllegalArgument(
                "four-invalid-gain PIDF live candidate",
                () -> pidf.setGains(
                        Double.NaN,
                        Double.POSITIVE_INFINITY,
                        Double.NEGATIVE_INFINITY,
                        Double.NaN));
        assertTrue(liveFailure.getMessage().contains("PidfRegulator.setGains(...)"));
        assertTrue(liveFailure.getMessage().contains("kP=NaN"));
        assertTrue(liveFailure.getMessage().contains("kI=Infinity"));
        assertTrue(liveFailure.getMessage().contains("kD=-Infinity"));
        assertTrue(liveFailure.getMessage().contains("kF=NaN"));
        assertEquals(1.0, pidf.getKP(), 0.0);
        assertEquals(2.0, pidf.getKI(), 0.0);
        assertEquals(3.0, pidf.getKD(), 0.0);
        assertEquals(4.0, pidf.getKF(), 0.0);
    }

    @Test
    public void signedFiniteGainsAreAcceptedAtConstructionAndLiveUpdate() {
        PidfRegulator pidf = ScalarRegulators.pidf(-1.0, 2.0, -3.0, 4.0);

        assertEquals(-1.0, pidf.getKP(), 0.0);
        assertEquals(2.0, pidf.getKI(), 0.0);
        assertEquals(-3.0, pidf.getKD(), 0.0);
        assertEquals(4.0, pidf.getKF(), 0.0);

        assertTrue(pidf == pidf.setGains(5.0, -6.0, 7.0, -8.0));
        assertEquals(5.0, pidf.getKP(), 0.0);
        assertEquals(-6.0, pidf.getKI(), 0.0);
        assertEquals(7.0, pidf.getKD(), 0.0);
        assertEquals(-8.0, pidf.getKF(), 0.0);
    }

    @Test
    public void formulaSeparatesErrorDrivenPidFromSetpointDrivenFeedforward() {
        PidfRegulator pidf = ScalarRegulators.pidf(2.0, 0.0, 0.0, 0.1);
        LoopClock clock = new ManualLoopClock().clock();

        assertEquals(5.0, pidf.update(10.0, 8.0, clock), EPSILON);
        assertEquals(6.0, pidf.update(20.0, 18.0, clock), EPSILON);
    }

    @Test
    public void firstZeroAndPositiveDtFollowPidIntegralAndDerivativeSemantics() {
        PidfRegulator zeroThenPositive = ScalarRegulators.pidf(0.0, 2.0, 3.0, 0.0);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(0.0, zeroThenPositive.update(1.0, 0.0, time.clock()), EPSILON);
        assertEquals(0.0, zeroThenPositive.update(2.0, 0.0, time.clock()), EPSILON);
        assertEquals(16.0,
                zeroThenPositive.update(4.0, 0.0, time.nextCycle(0.5)),
                EPSILON);

        PidfRegulator firstPositive = ScalarRegulators.pidf(0.0, 2.0, 3.0, 0.0);
        ManualLoopClock firstPositiveTime = new ManualLoopClock();
        assertEquals(1.0,
                firstPositive.update(1.0, 0.0, firstPositiveTime.nextCycle(0.5)),
                EPSILON);
    }

    @Test
    public void repeatedCallsInOneClockCycleAreNotMemoized() {
        PidfRegulator pidf = ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        ManualLoopClock time = new ManualLoopClock();
        LoopClock sameCycle = time.nextCycle(0.5);

        assertEquals(1.0, pidf.update(2.0, 0.0, sameCycle), EPSILON);
        assertEquals(2.0, pidf.update(2.0, 0.0, sameCycle), EPSILON);
    }

    @Test
    public void validGainUpdatePreservesIntegralAndDerivativeHistory() {
        PidfRegulator pidf = ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        ManualLoopClock time = new ManualLoopClock();
        assertEquals(1.0, pidf.update(1.0, 0.0, time.nextCycle(1.0)), EPSILON);

        pidf.setGains(0.0, 2.0, 2.0, 0.5);

        assertEquals(12.5, pidf.update(3.0, 0.0, time.clock()), EPSILON);
        CapturingDebugSink debug = new CapturingDebugSink();
        pidf.debugDump(debug, "pidf");
        assertEquals(7.0, number(debug, "pidf.pid.integral"), EPSILON);
        assertEquals(4.0, number(debug, "pidf.lastPidOutput")
                - number(debug, "pidf.pid.integral"), EPSILON);
    }

    @Test
    public void integralLimitsRejectNonFiniteReversedAndZeroExcludingRanges() {
        for (double invalidValue : NON_FINITE_VALUES) {
            PidfRegulator invalidMin = ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
            IllegalArgumentException minFailure = expectIllegalArgument(
                    "PidfRegulator.setIntegralLimits(...) min=" + invalidValue,
                    () -> invalidMin.setIntegralLimits(invalidValue, 1.0));
            assertLimitDiagnostic(minFailure, "PidfRegulator.setIntegralLimits(...)",
                    "min", invalidValue, 1.0, "finite");

            PidfRegulator invalidMax = ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
            IllegalArgumentException maxFailure = expectIllegalArgument(
                    "PidfRegulator.setIntegralLimits(...) max=" + invalidValue,
                    () -> invalidMax.setIntegralLimits(-1.0, invalidValue));
            assertLimitDiagnostic(maxFailure, "PidfRegulator.setIntegralLimits(...)",
                    "max", -1.0, invalidValue, "finite");
        }

        PidfRegulator pidf = ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        IllegalArgumentException reversed = expectIllegalArgument(
                "reversed PIDF integral limits",
                () -> pidf.setIntegralLimits(2.0, -1.0));
        assertLimitDiagnostic(reversed, "PidfRegulator.setIntegralLimits(...)",
                "min", 2.0, -1.0, "min <= max");

        IllegalArgumentException positiveOnly = expectIllegalArgument(
                "positive-only PIDF integral limits",
                () -> pidf.setIntegralLimits(0.1, 2.0));
        assertLimitDiagnostic(positiveOnly, "PidfRegulator.setIntegralLimits(...)",
                "min", 0.1, 2.0, "include zero");
        assertTrue(positiveOnly.getMessage().contains("min <= 0 <= max"));

        IllegalArgumentException negativeOnly = expectIllegalArgument(
                "negative-only PIDF integral limits",
                () -> pidf.setIntegralLimits(-2.0, -0.1));
        assertLimitDiagnostic(negativeOnly, "PidfRegulator.setIntegralLimits(...)",
                "max", -2.0, -0.1, "include zero");
        assertTrue(negativeOnly.getMessage().contains("min <= 0 <= max"));
    }

    @Test
    public void integralLimitsAcceptOneSidedAsymmetricSymmetricAndZeroOnlyRanges() {
        PidfRegulator negativeOneSided =
                ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        assertTrue(negativeOneSided
                == negativeOneSided.setIntegralLimits(-2.0, 0.0));

        PidfRegulator positiveOneSided =
                ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        assertTrue(positiveOneSided
                == positiveOneSided.setIntegralLimits(0.0, 2.0));

        PidfRegulator asymmetric =
                ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        assertTrue(asymmetric
                == asymmetric.setIntegralLimits(-1.0, 3.0));

        PidfRegulator symmetric =
                ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        assertTrue(symmetric
                == symmetric.setIntegralLimits(-2.0, 2.0));

        PidfRegulator zeroOnly =
                ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        assertTrue(zeroOnly
                == zeroOnly.setIntegralLimits(0.0, 0.0));
        assertEquals(0.0,
                zeroOnly.update(10.0, 0.0, new ManualLoopClock().nextCycle(1.0)),
                0.0);
    }

    @Test
    public void pidOutputLimitsRejectNonFiniteAndReversedBoundsButAllowEqualBounds() {
        for (double invalidValue : NON_FINITE_VALUES) {
            PidfRegulator invalidMin = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.0);
            IllegalArgumentException minFailure = expectIllegalArgument(
                    "PidfRegulator.setPidOutputLimits(...) min=" + invalidValue,
                    () -> invalidMin.setPidOutputLimits(invalidValue, 1.0));
            assertLimitDiagnostic(minFailure, "PidfRegulator.setPidOutputLimits(...)",
                    "min", invalidValue, 1.0, "finite");

            PidfRegulator invalidMax = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.0);
            IllegalArgumentException maxFailure = expectIllegalArgument(
                    "PidfRegulator.setPidOutputLimits(...) max=" + invalidValue,
                    () -> invalidMax.setPidOutputLimits(-1.0, invalidValue));
            assertLimitDiagnostic(maxFailure, "PidfRegulator.setPidOutputLimits(...)",
                    "max", -1.0, invalidValue, "finite");
        }

        PidfRegulator reversedPidf = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.0);
        IllegalArgumentException reversed = expectIllegalArgument(
                "reversed PIDF PID-output limits",
                () -> reversedPidf.setPidOutputLimits(2.0, -1.0));
        assertLimitDiagnostic(reversed, "PidfRegulator.setPidOutputLimits(...)",
                "min", 2.0, -1.0, "min <= max");

        PidfRegulator equal = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.0)
                .setPidOutputLimits(0.25, 0.25);
        assertEquals(0.25,
                equal.update(-100.0, 0.0, new ManualLoopClock().clock()),
                0.0);

        PidfRegulator positiveOnly = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.0)
                .setPidOutputLimits(2.0, 3.0);
        assertEquals(2.0,
                positiveOnly.update(0.0, 0.0, new ManualLoopClock().clock()),
                0.0);
    }

    @Test
    public void rejectedLimitSettersLeavePriorConfigurationAndIntegralUntouched() {
        ManualLoopClock time = new ManualLoopClock();
        PidfRegulator integralPidf = ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0)
                .setIntegralLimits(-2.0, 2.0);
        assertEquals(1.0,
                integralPidf.update(1.0, 0.0, time.nextCycle(1.0)),
                EPSILON);

        expectIllegalArgument("partly invalid replacement PIDF integral limits",
                () -> integralPidf.setIntegralLimits(0.5, Double.NaN));
        assertEquals(-2.0,
                integralPidf.update(-10.0, 0.0, time.clock()),
                EPSILON);

        PidfRegulator outputPidf = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.0)
                .setPidOutputLimits(-2.0, 2.0);
        expectIllegalArgument("partly invalid replacement PIDF PID-output limits",
                () -> outputPidf.setPidOutputLimits(-0.5, Double.NaN));
        assertEquals(-2.0,
                outputPidf.update(-10.0, 0.0, new ManualLoopClock().clock()),
                EPSILON);
    }

    @Test
    public void narrowingIntegralLimitsImmediatelyReclampsExistingContribution() {
        PidfRegulator pidf = ScalarRegulators.pidf(0.0, 1.0, 0.0, 0.0);
        ManualLoopClock time = new ManualLoopClock();
        assertEquals(10.0, pidf.update(10.0, 0.0, time.nextCycle(1.0)), EPSILON);

        pidf.setIntegralLimits(-1.0, 2.0);

        assertEquals(2.0, pidf.update(0.0, 0.0, time.clock()), EPSILON);
        CapturingDebugSink debug = new CapturingDebugSink();
        pidf.debugDump(debug, "pidf");
        assertEquals(2.0, number(debug, "pidf.pid.integral"), EPSILON);
    }

    @Test
    public void pidContributionLimitAndOuterCompleteOutputLimitRemainDistinct() {
        LoopClock clock = new ManualLoopClock().clock();

        PidfRegulator innerLimited = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.15)
                .setPidOutputLimits(-1.0, 1.0);
        assertEquals(0.5, innerLimited.update(10.0, 12.0, clock), EPSILON);

        PidfRegulator notInnerLimited = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.15);
        assertEquals(-0.5, notInnerLimited.update(10.0, 12.0, clock), EPSILON);

        ScalarRegulator completeLimit = ScalarRegulators.outputLimited(
                notInnerLimited, 0.0, 0.65);
        assertEquals(0.0, completeLimit.update(10.0, 12.0, clock), EPSILON);

        ScalarRegulator completeLimitAroundInnerLimited = ScalarRegulators.outputLimited(
                innerLimited, 0.0, 0.65);
        assertEquals(0.5,
                completeLimitAroundInnerLimited.update(10.0, 12.0, clock),
                EPSILON);
    }

    @Test
    public void pidLimitsDoNotHideNonFiniteMeasurementOrFiniteArithmeticOverflow() {
        LoopClock clock = new ManualLoopClock().clock();
        PidfRegulator limited = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.1)
                .setIntegralLimits(-0.5, 0.5)
                .setPidOutputLimits(-1.0, 1.0);

        assertEquals(Double.POSITIVE_INFINITY,
                limited.update(100.0, Double.NEGATIVE_INFINITY, clock),
                0.0);
        assertEquals(Double.NEGATIVE_INFINITY,
                limited.update(100.0, Double.POSITIVE_INFINITY, clock),
                0.0);
        assertTrue(Double.isNaN(limited.update(100.0, Double.NaN, clock)));

        PidfRegulator overflow =
                ScalarRegulators.pidf(Double.MAX_VALUE, 0.0, 0.0, 0.0)
                        .setPidOutputLimits(-1.0, 1.0);
        assertEquals(Double.POSITIVE_INFINITY,
                overflow.update(2.0, 0.0, clock),
                0.0);
    }

    @Test
    public void resetClearsTransientStateWhilePreservingGainsAndLimits() {
        PidfRegulator pidf = ScalarRegulators.pidf(1.0, 1.0, 1.0, 0.1)
                .setIntegralLimits(-0.5, 0.5)
                .setPidOutputLimits(-1.0, 1.0);
        ManualLoopClock time = new ManualLoopClock();
        assertEquals(1.3, pidf.update(3.0, 0.0, time.nextCycle(0.5)), EPSILON);

        pidf.reset();

        assertEquals(1.0, pidf.getKP(), 0.0);
        assertEquals(1.0, pidf.getKI(), 0.0);
        assertEquals(1.0, pidf.getKD(), 0.0);
        assertEquals(0.1, pidf.getKF(), 0.0);

        CapturingDebugSink debug = new CapturingDebugSink();
        pidf.debugDump(debug, "pidf");
        assertNaN(debug.data.get("pidf.lastSetpoint"));
        assertNaN(debug.data.get("pidf.lastMeasurement"));
        assertNaN(debug.data.get("pidf.lastError"));
        assertNaN(debug.data.get("pidf.lastPidOutput"));
        assertNaN(debug.data.get("pidf.lastFeedforwardOutput"));
        assertNaN(debug.data.get("pidf.lastOutput"));
        assertEquals(0.0, number(debug, "pidf.pid.integral"), 0.0);
        assertEquals(-0.5, number(debug, "pidf.pid.integralMin"), 0.0);
        assertEquals(0.5, number(debug, "pidf.pid.integralMax"), 0.0);
        assertEquals(-1.0, number(debug, "pidf.pid.outputMin"), 0.0);
        assertEquals(1.0, number(debug, "pidf.pid.outputMax"), 0.0);

        assertEquals(2.0, pidf.update(10.0, 0.0, time.clock()), EPSILON);
    }

    @Test
    public void outerDecoratorResetPropagatesAndRetainedPidfHandleUpdatesLiveComposition() {
        PidfRegulator pidf = ScalarRegulators.pidf(0.1, 0.0, 0.0, 0.0);
        RecordingVoltageSource voltage = new RecordingVoltageSource();
        ScalarRegulator composed = ScalarRegulators.outputLimited(
                ScalarRegulators.voltageCompensated(
                        pidf,
                        voltage,
                        13.0,
                        9.0,
                        1.4),
                -10.0,
                10.0);
        LoopClock clock = new ManualLoopClock().clock();

        assertEquals(1.0, composed.update(10.0, 0.0, clock), EPSILON);
        pidf.setGains(0.2, 0.0, 0.0, 0.0);
        assertEquals(2.0, composed.update(10.0, 0.0, clock), EPSILON);

        composed.reset();

        assertEquals(1, voltage.resetCount);
        assertEquals(0.2, pidf.getKP(), 0.0);
        CapturingDebugSink debug = new CapturingDebugSink();
        composed.debugDump(debug, "outer");
        assertNaN(debug.data.get("outer.lastOutput"));
        assertNaN(debug.data.get("outer.inner.lastOutput"));
        assertNaN(debug.data.get("outer.inner.inner.lastOutput"));
    }

    @Test
    public void debugDumpReportsCurrentGainsAndEveryLastControlTerm() {
        PidfRegulator pidf = ScalarRegulators.pidf(2.0, 0.0, 0.0, 0.1);
        assertEquals(5.0,
                pidf.update(10.0, 8.0, new ManualLoopClock().clock()),
                EPSILON);
        pidf.debugDump(null, "ignored");

        CapturingDebugSink named = new CapturingDebugSink();
        pidf.debugDump(named, "shooter.pidf");
        assertEquals("PidfRegulator", named.data.get("shooter.pidf.class"));
        assertEquals(2.0, number(named, "shooter.pidf.kP"), 0.0);
        assertEquals(0.0, number(named, "shooter.pidf.kI"), 0.0);
        assertEquals(0.0, number(named, "shooter.pidf.kD"), 0.0);
        assertEquals(0.1, number(named, "shooter.pidf.kF"), 0.0);
        assertEquals(10.0, number(named, "shooter.pidf.lastSetpoint"), 0.0);
        assertEquals(8.0, number(named, "shooter.pidf.lastMeasurement"), 0.0);
        assertEquals(2.0, number(named, "shooter.pidf.lastError"), 0.0);
        assertEquals(4.0, number(named, "shooter.pidf.lastPidOutput"), 0.0);
        assertEquals(1.0, number(named, "shooter.pidf.lastFeedforwardOutput"), 0.0);
        assertEquals(5.0, number(named, "shooter.pidf.lastOutput"), 0.0);

        CapturingDebugSink defaultPrefix = new CapturingDebugSink();
        pidf.debugDump(defaultPrefix, null);
        assertEquals("PidfRegulator", defaultPrefix.data.get("pidfRegulator.class"));
        assertEquals(5.0, number(defaultPrefix, "pidfRegulator.lastOutput"), 0.0);
    }

    private static IllegalArgumentException expectIllegalArgument(String description,
                                                                  ThrowingRunnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException for " + description);
            return null;
        } catch (IllegalArgumentException expected) {
            assertNotNull(expected.getMessage());
            return expected;
        }
    }

    private static void assertGainDiagnostic(IllegalArgumentException failure,
                                             String api,
                                             String setting,
                                             double value) {
        String message = failure.getMessage();
        assertTrue(message, message.contains(api));
        assertTrue(message, message.contains(setting));
        assertTrue(message, message.contains(Double.toString(value)));
        assertTrue(message, message.contains("finite"));
    }

    private static void assertLimitDiagnostic(IllegalArgumentException failure,
                                              String api,
                                              String setting,
                                              double min,
                                              double max,
                                              String rule) {
        String message = failure.getMessage();
        assertTrue(message, message.contains(api));
        assertTrue(message, message.contains(setting));
        assertTrue(message, message.contains(Double.toString(min)));
        assertTrue(message, message.contains(Double.toString(max)));
        assertTrue(message, message.contains(rule));
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

    private interface ThrowingRunnable {
        void run();
    }

    private static final class RecordingVoltageSource implements ScalarSource {
        private int resetCount;

        @Override
        public double getAsDouble(LoopClock clock) {
            return 13.0;
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
