package edu.ftcphoenix.fw.core.control;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies PID configuration ownership, control math, lifecycle, and diagnostics. */
public final class PidTest {

    private static final double EPSILON = 1e-12;
    private static final double[] NON_FINITE_VALUES = {
            Double.NaN,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY
    };

    @Test
    public void factoryRejectsEveryNonFiniteGainWithActionableDiagnostics() {
        String[] gainNames = {"kP", "kI", "kD"};
        for (int gainIndex = 0; gainIndex < gainNames.length; gainIndex++) {
            for (double invalidValue : NON_FINITE_VALUES) {
                double[] gains = {1.0, 2.0, 3.0};
                gains[gainIndex] = invalidValue;

                IllegalArgumentException failure = expectIllegalArgument(
                        "Pid.withGains(...) " + gainNames[gainIndex] + "=" + invalidValue,
                        () -> Pid.withGains(gains[0], gains[1], gains[2]));
                assertGainDiagnostic(failure, "Pid.withGains(...)",
                        gainNames[gainIndex], invalidValue);
            }
        }
    }

    @Test
    public void liveGainUpdateRejectsEveryNonFinitePositionBeforeAnyMutation() {
        String[] gainNames = {"kP", "kI", "kD"};
        for (int gainIndex = 0; gainIndex < gainNames.length; gainIndex++) {
            for (double invalidValue : NON_FINITE_VALUES) {
                Pid pid = Pid.withGains(1.0, 2.0, 3.0);
                double[] candidate = {10.0, 20.0, 30.0};
                candidate[gainIndex] = invalidValue;

                IllegalArgumentException failure = expectIllegalArgument(
                        "Pid.setGains(...) " + gainNames[gainIndex] + "=" + invalidValue,
                        () -> pid.setGains(candidate[0], candidate[1], candidate[2]));
                assertGainDiagnostic(failure, "Pid.setGains(...)",
                        gainNames[gainIndex], invalidValue);

                assertEquals(1.0, pid.getKP(), 0.0);
                assertEquals(2.0, pid.getKI(), 0.0);
                assertEquals(3.0, pid.getKD(), 0.0);
            }
        }
    }

    @Test
    public void diagnosticsNameEveryInvalidGainInOneCandidate() {
        IllegalArgumentException factoryFailure = expectIllegalArgument(
                "three-invalid-gain PID factory candidate",
                () -> Pid.withGains(
                        Double.NaN,
                        Double.POSITIVE_INFINITY,
                        Double.NEGATIVE_INFINITY));
        assertTrue(factoryFailure.getMessage().contains("kP=NaN"));
        assertTrue(factoryFailure.getMessage().contains("kI=Infinity"));
        assertTrue(factoryFailure.getMessage().contains("kD=-Infinity"));

        Pid pid = Pid.withGains(1.0, 2.0, 3.0);
        IllegalArgumentException liveFailure = expectIllegalArgument(
                "three-invalid-gain PID live candidate",
                () -> pid.setGains(
                        Double.NaN,
                        Double.POSITIVE_INFINITY,
                        Double.NEGATIVE_INFINITY));
        assertTrue(liveFailure.getMessage().contains("Pid.setGains(...)"));
        assertTrue(liveFailure.getMessage().contains("kP=NaN"));
        assertTrue(liveFailure.getMessage().contains("kI=Infinity"));
        assertTrue(liveFailure.getMessage().contains("kD=-Infinity"));
        assertEquals(1.0, pid.getKP(), 0.0);
        assertEquals(2.0, pid.getKI(), 0.0);
        assertEquals(3.0, pid.getKD(), 0.0);
    }

    @Test
    public void signedFiniteGainsAreAcceptedAtConstructionAndLiveUpdate() {
        Pid pid = Pid.withGains(-1.0, 2.0, -3.0);

        assertEquals(-1.0, pid.getKP(), 0.0);
        assertEquals(2.0, pid.getKI(), 0.0);
        assertEquals(-3.0, pid.getKD(), 0.0);

        assertTrue(pid == pid.setGains(4.0, -5.0, 6.0));
        assertEquals(4.0, pid.getKP(), 0.0);
        assertEquals(-5.0, pid.getKI(), 0.0);
        assertEquals(6.0, pid.getKD(), 0.0);
    }

    @Test
    public void integralLimitsRejectEveryNonFiniteBoundWithApiValueAndRule() {
        for (double invalidValue : NON_FINITE_VALUES) {
            Pid invalidMin = Pid.withGains(0.0, 1.0, 0.0);
            IllegalArgumentException minFailure = expectIllegalArgument(
                    "Pid.setIntegralLimits(...) min=" + invalidValue,
                    () -> invalidMin.setIntegralLimits(invalidValue, 1.0));
            assertLimitDiagnostic(minFailure, "Pid.setIntegralLimits(...)",
                    "min", invalidValue, 1.0, "finite");

            Pid invalidMax = Pid.withGains(0.0, 1.0, 0.0);
            IllegalArgumentException maxFailure = expectIllegalArgument(
                    "Pid.setIntegralLimits(...) max=" + invalidValue,
                    () -> invalidMax.setIntegralLimits(-1.0, invalidValue));
            assertLimitDiagnostic(maxFailure, "Pid.setIntegralLimits(...)",
                    "max", -1.0, invalidValue, "finite");
        }
    }

    @Test
    public void integralLimitsRejectReversedAndZeroExcludingRanges() {
        Pid pid = Pid.withGains(0.0, 1.0, 0.0);

        IllegalArgumentException reversed = expectIllegalArgument(
                "reversed integral limits",
                () -> pid.setIntegralLimits(2.0, -1.0));
        assertLimitDiagnostic(reversed, "Pid.setIntegralLimits(...)",
                "min", 2.0, -1.0, "min <= max");

        IllegalArgumentException positiveOnly = expectIllegalArgument(
                "positive-only integral limits",
                () -> pid.setIntegralLimits(0.1, 2.0));
        assertLimitDiagnostic(positiveOnly, "Pid.setIntegralLimits(...)",
                "min", 0.1, 2.0, "include zero");
        assertTrue(positiveOnly.getMessage().contains("min <= 0 <= max"));

        IllegalArgumentException negativeOnly = expectIllegalArgument(
                "negative-only integral limits",
                () -> pid.setIntegralLimits(-2.0, -0.1));
        assertLimitDiagnostic(negativeOnly, "Pid.setIntegralLimits(...)",
                "max", -2.0, -0.1, "include zero");
        assertTrue(negativeOnly.getMessage().contains("min <= 0 <= max"));

        IllegalArgumentException equalNonZero = expectIllegalArgument(
                "equal nonzero integral limits",
                () -> pid.setIntegralLimits(0.5, 0.5));
        assertLimitDiagnostic(equalNonZero, "Pid.setIntegralLimits(...)",
                "min", 0.5, 0.5, "include zero");
    }

    @Test
    public void integralLimitsAcceptOneSidedAsymmetricSymmetricAndZeroOnlyRanges() {
        Pid negativeOneSided = Pid.withGains(0.0, 1.0, 0.0);
        assertTrue(negativeOneSided
                == negativeOneSided.setIntegralLimits(-2.0, 0.0));

        Pid positiveOneSided = Pid.withGains(0.0, 1.0, 0.0);
        assertTrue(positiveOneSided
                == positiveOneSided.setIntegralLimits(0.0, 2.0));

        Pid asymmetric = Pid.withGains(0.0, 1.0, 0.0);
        assertTrue(asymmetric
                == asymmetric.setIntegralLimits(-1.0, 3.0));

        Pid symmetric = Pid.withGains(0.0, 1.0, 0.0);
        assertTrue(symmetric
                == symmetric.setIntegralLimits(-2.0, 2.0));

        Pid zeroOnly = Pid.withGains(0.0, 1.0, 0.0);
        assertTrue(zeroOnly
                == zeroOnly.setIntegralLimits(0.0, 0.0));
        assertEquals(0.0, zeroOnly.update(10.0, 1.0), 0.0);
    }

    @Test
    public void outputLimitsRejectEveryNonFiniteAndReversedBound() {
        for (double invalidValue : NON_FINITE_VALUES) {
            Pid invalidMin = Pid.withGains(1.0, 0.0, 0.0);
            IllegalArgumentException minFailure = expectIllegalArgument(
                    "Pid.setOutputLimits(...) min=" + invalidValue,
                    () -> invalidMin.setOutputLimits(invalidValue, 1.0));
            assertLimitDiagnostic(minFailure, "Pid.setOutputLimits(...)",
                    "min", invalidValue, 1.0, "finite");

            Pid invalidMax = Pid.withGains(1.0, 0.0, 0.0);
            IllegalArgumentException maxFailure = expectIllegalArgument(
                    "Pid.setOutputLimits(...) max=" + invalidValue,
                    () -> invalidMax.setOutputLimits(-1.0, invalidValue));
            assertLimitDiagnostic(maxFailure, "Pid.setOutputLimits(...)",
                    "max", -1.0, invalidValue, "finite");
        }

        Pid reversedPid = Pid.withGains(1.0, 0.0, 0.0);
        IllegalArgumentException reversed = expectIllegalArgument(
                "reversed PID output limits",
                () -> reversedPid.setOutputLimits(2.0, -1.0));
        assertLimitDiagnostic(reversed, "Pid.setOutputLimits(...)",
                "min", 2.0, -1.0, "min <= max");
    }

    @Test
    public void outputLimitsAcceptEqualAndRangesThatExcludeZero() {
        Pid equal = Pid.withGains(1.0, 0.0, 0.0)
                .setOutputLimits(0.25, 0.25);
        assertEquals(0.25, equal.update(-100.0, 0.0), 0.0);

        Pid positiveOnly = Pid.withGains(1.0, 0.0, 0.0)
                .setOutputLimits(2.0, 3.0);
        assertEquals(2.0, positiveOnly.update(0.0, 0.0), 0.0);
    }

    @Test
    public void finiteLimitsDoNotHideNonFiniteInputOrControllerOverflow() {
        Pid proportional = Pid.withGains(1.0, 0.0, 0.0)
                .setOutputLimits(-1.0, 1.0);
        assertEquals(Double.POSITIVE_INFINITY,
                proportional.update(Double.POSITIVE_INFINITY, 0.0),
                0.0);
        assertEquals(Double.NEGATIVE_INFINITY,
                proportional.update(Double.NEGATIVE_INFINITY, 0.0),
                0.0);
        assertTrue(Double.isNaN(proportional.update(Double.NaN, 0.0)));

        Pid proportionalOverflow = Pid.withGains(Double.MAX_VALUE, 0.0, 0.0)
                .setOutputLimits(-1.0, 1.0);
        assertEquals(Double.POSITIVE_INFINITY,
                proportionalOverflow.update(2.0, 0.0),
                0.0);

        Pid integralOverflow = Pid.withGains(0.0, Double.MAX_VALUE, 0.0)
                .setIntegralLimits(-1.0, 1.0)
                .setOutputLimits(-1.0, 1.0);
        assertEquals(Double.POSITIVE_INFINITY,
                integralOverflow.update(2.0, 1.0),
                0.0);
        assertEquals(Double.POSITIVE_INFINITY, integralOverflow.getIntegral(), 0.0);

        Pid derivativeOverflow = Pid.withGains(0.0, 0.0, Double.MAX_VALUE)
                .setOutputLimits(-1.0, 1.0);
        assertEquals(0.0, derivativeOverflow.update(0.0, 1.0), 0.0);
        assertEquals(Double.POSITIVE_INFINITY,
                derivativeOverflow.update(2.0, 1.0),
                0.0);
    }

    @Test
    public void rejectedLimitSettersLeavePriorLimitsAndIntegralUntouched() {
        Pid integralPid = Pid.withGains(0.0, 1.0, 0.0)
                .setIntegralLimits(-2.0, 2.0);
        assertEquals(1.0, integralPid.update(1.0, 1.0), EPSILON);

        expectIllegalArgument("partly invalid replacement integral limits",
                () -> integralPid.setIntegralLimits(0.5, Double.NaN));
        assertEquals(1.0, integralPid.getIntegral(), EPSILON);
        assertEquals(-2.0, integralPid.update(-10.0, 1.0), EPSILON);

        Pid outputPid = Pid.withGains(1.0, 0.0, 0.0)
                .setOutputLimits(-2.0, 2.0);
        expectIllegalArgument("partly invalid replacement output limits",
                () -> outputPid.setOutputLimits(-0.5, Double.NaN));
        assertEquals(-2.0, outputPid.update(-10.0, 0.0), EPSILON);
    }

    @Test
    public void narrowingIntegralLimitsImmediatelyReclampsFiniteContribution() {
        Pid positive = Pid.withGains(0.0, 1.0, 0.0);
        assertEquals(10.0, positive.update(10.0, 1.0), EPSILON);
        positive.setIntegralLimits(-1.0, 2.0);
        assertEquals(2.0, positive.getIntegral(), EPSILON);

        Pid negative = Pid.withGains(0.0, 1.0, 0.0);
        assertEquals(-10.0, negative.update(-10.0, 1.0), EPSILON);
        negative.setIntegralLimits(-2.0, 1.0);
        assertEquals(-2.0, negative.getIntegral(), EPSILON);
    }

    @Test
    public void zeroNegativeAndNaNDtSkipIntegralAndDerivativeButStillUpdateHistory() {
        Pid pid = Pid.withGains(1.0, 2.0, 3.0);

        assertEquals(1.0, pid.update(1.0, 0.0), EPSILON);
        assertEquals(2.0, pid.update(2.0, -1.0), EPSILON);
        assertEquals(3.0, pid.update(3.0, Double.NaN), EPSILON);
        assertEquals(14.0, pid.update(4.0, 0.5), EPSILON);
        assertEquals(4.0, pid.getIntegral(), EPSILON);
    }

    @Test
    public void validGainUpdatePreservesIntegralAndDerivativeHistory() {
        Pid pid = Pid.withGains(0.0, 1.0, 0.0);
        assertEquals(1.0, pid.update(1.0, 1.0), EPSILON);

        pid.setGains(0.0, 2.0, 2.0);

        assertEquals(11.0, pid.update(3.0, 1.0), EPSILON);
        assertEquals(7.0, pid.getIntegral(), EPSILON);
    }

    @Test
    public void resetClearsIntegralAndDerivativeHistoryWithoutChangingConfiguration() {
        Pid pid = Pid.withGains(1.0, 1.0, 1.0)
                .setIntegralLimits(-100.0, 100.0)
                .setOutputLimits(-100.0, 100.0);

        assertEquals(2.0, pid.update(1.0, 1.0), EPSILON);
        assertEquals(9.0, pid.update(3.0, 1.0), EPSILON);

        pid.reset();

        assertEquals(0.0, pid.getIntegral(), 0.0);
        assertEquals(6.0, pid.update(3.0, 1.0), EPSILON);
        assertEquals(1.0, pid.getKP(), 0.0);
        assertEquals(1.0, pid.getKI(), 0.0);
        assertEquals(1.0, pid.getKD(), 0.0);
    }

    @Test
    public void controlMathAndDebugStateRemainTruthful() {
        Pid pid = Pid.withGains(2.0, 3.0, 4.0)
                .setIntegralLimits(-10.0, 10.0)
                .setOutputLimits(-100.0, 100.0);

        assertEquals(3.5, pid.update(1.0, 0.5), EPSILON);
        assertEquals(23.0, pid.update(2.0, 0.25), EPSILON);
        pid.debugDump(null, "ignored");

        CapturingDebugSink debug = new CapturingDebugSink();
        pid.debugDump(debug, "shooter.pid");

        assertEquals(2.0, number(debug, "shooter.pid.kP"), 0.0);
        assertEquals(3.0, number(debug, "shooter.pid.kI"), 0.0);
        assertEquals(4.0, number(debug, "shooter.pid.kD"), 0.0);
        assertEquals(3.0, number(debug, "shooter.pid.integral"), EPSILON);
        assertEquals(2.0, number(debug, "shooter.pid.prevError"), EPSILON);
        assertEquals(Boolean.FALSE, debug.data.get("shooter.pid.firstUpdate"));
        assertEquals(-10.0, number(debug, "shooter.pid.integralMin"), 0.0);
        assertEquals(10.0, number(debug, "shooter.pid.integralMax"), 0.0);
        assertEquals(-100.0, number(debug, "shooter.pid.outputMin"), 0.0);
        assertEquals(100.0, number(debug, "shooter.pid.outputMax"), 0.0);
        assertFalse(debug.lines.isEmpty());
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

    private interface ThrowingRunnable {
        void run();
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> data = new LinkedHashMap<>();
        private final Map<Integer, String> lines = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            data.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            lines.put(lines.size(), text);
            return this;
        }
    }
}
