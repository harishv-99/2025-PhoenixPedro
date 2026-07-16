package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the private normalized-power and fail-stop boundary shared by regulated Plants. */
public final class RegulatedPowerChannelTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void finiteCommandsNormalizeOnceAndPreserveSignedZero() {
        Fixture fixture = new Fixture("test.regulatedVelocity");
        LoopClock clock = new ManualLoopClock().clock();
        double[] rawCommands = {
                -0.4,
                -1.0,
                -0.0,
                0.0,
                1.0,
                Math.nextDown(-1.0),
                Math.nextUp(1.0),
                -4.0,
                4.0
        };
        double[] expectedCommands = {
                -0.4,
                -1.0,
                -0.0,
                0.0,
                1.0,
                -1.0,
                1.0,
                -1.0,
                1.0
        };

        for (int i = 0; i < rawCommands.length; i++) {
            fixture.regulator.result = rawCommands[i];
            fixture.channel.update(17.0, 11.0, clock);

            assertSameBits(rawCommands[i], fixture.channel.regulatorOutput());
            assertSameBits(expectedCommands[i], fixture.channel.normalizedPowerCommand());
            assertSameBits(expectedCommands[i], fixture.output.lastSuccessfulPower);
            assertEquals(i < 5 ? "SUBMITTED" : "SATURATED_AND_SUBMITTED",
                    fixture.channel.status());
            assertFalse(fixture.channel.lastStopSubmitted());
        }

        assertEquals(rawCommands.length, fixture.regulator.updateCount);
        assertEquals(rawCommands.length, fixture.output.setPowerCount);
        assertEquals(0, fixture.output.stopCount);
        assertEquals(0, fixture.regulator.resetCount);
    }

    @Test
    public void repeatedSameCycleUpdatesAreNotMemoized() {
        Fixture fixture = new Fixture("test.regulatedPosition");
        LoopClock sameCycle = new ManualLoopClock().clock();

        fixture.regulator.result = 0.25;
        fixture.channel.update(5.0, 1.0, sameCycle);
        fixture.regulator.result = -0.75;
        fixture.channel.update(4.0, 2.0, sameCycle);

        assertEquals(2, fixture.regulator.updateCount);
        assertEquals(2, fixture.output.setPowerCount);
        assertEquals(Arrays.asList(0.25, -0.75), fixture.output.attemptedPowers);
        assertEquals(-0.75, fixture.channel.normalizedPowerCommand(), EPSILON);
    }

    @Test
    public void nonFiniteResultsAreRejectedAndFailStopped() {
        double[] invalidResults = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };

        for (double invalidResult : invalidResults) {
            Fixture fixture = new Fixture("MappedVelocityPlant regulated-power path");
            fixture.regulator.result = invalidResult;

            RuntimeException failure = expectRuntime(() -> fixture.channel.update(
                    4200.0,
                    2000.0,
                    new ManualLoopClock().clock()));

            assertTrue(failure instanceof IllegalStateException);
            assertContains(failure.getMessage(),
                    "MappedVelocityPlant regulated-power path",
                    "setpoint=4200.0",
                    "measurement=2000.0",
                    String.valueOf(invalidResult),
                    "fix the regulator or control law");
            assertSameNonFinite(invalidResult, fixture.channel.regulatorOutput());
            assertEquals(0.0, fixture.channel.normalizedPowerCommand(), 0.0);
            assertEquals("NON_FINITE_REGULATOR_OUTPUT_STOP_SUBMITTED_RESET_SUCCEEDED",
                    fixture.channel.status());
            assertTrue(fixture.channel.lastStopSubmitted());
            assertEquals(0, fixture.output.setPowerCount);
            assertEquals(1, fixture.output.stopCount);
            assertEquals(1, fixture.regulator.resetCount);
            assertEquals(Arrays.asList(
                            "regulator.update",
                            "output.stop",
                            "regulator.reset"),
                    fixture.events);
        }
    }

    @Test
    public void nonFiniteFailureKeepsSyntheticPrimaryAndSuppressesCleanupInOrder() {
        Fixture fixture = new Fixture("test.nonFinite");
        RuntimeException stopFailure = new IllegalStateException("stop cleanup failed");
        RuntimeException resetFailure = new IllegalArgumentException("reset cleanup failed");
        fixture.regulator.result = Double.NaN;
        fixture.output.stopFailure = stopFailure;
        fixture.regulator.resetFailure = resetFailure;

        RuntimeException primary = expectRuntime(() -> fixture.channel.update(
                3.0,
                2.0,
                new ManualLoopClock().clock()));

        assertTrue(primary instanceof IllegalStateException);
        assertEquals(2, primary.getSuppressed().length);
        assertSame(stopFailure, primary.getSuppressed()[0]);
        assertSame(resetFailure, primary.getSuppressed()[1]);
        assertTrue(Double.isNaN(fixture.channel.normalizedPowerCommand()));
        assertEquals("NON_FINITE_REGULATOR_OUTPUT_STOP_FAILED_RESET_FAILED",
                fixture.channel.status());
        assertFalse(fixture.channel.lastStopSubmitted());
        assertEquals(Arrays.asList(
                        "regulator.update",
                        "output.stop",
                        "regulator.reset"),
                fixture.events);
    }

    @Test
    public void regulatorFailureRemainsPrimaryAndCleanupFailuresAreSuppressedInOrder() {
        Fixture fixture = new Fixture("test.regulatorFailure");
        RuntimeException regulatorFailure = new IllegalStateException("regulator failed");
        RuntimeException stopFailure = new IllegalArgumentException("stop failed");
        RuntimeException resetFailure = new IllegalStateException("reset failed");
        fixture.regulator.updateFailure = regulatorFailure;
        fixture.output.stopFailure = stopFailure;
        fixture.regulator.resetFailure = resetFailure;

        RuntimeException observed = expectRuntime(() -> fixture.channel.update(
                1.0,
                0.0,
                new ManualLoopClock().clock()));

        assertSame(regulatorFailure, observed);
        assertEquals(2, observed.getSuppressed().length);
        assertSame(stopFailure, observed.getSuppressed()[0]);
        assertSame(resetFailure, observed.getSuppressed()[1]);
        assertTrue(Double.isNaN(fixture.channel.regulatorOutput()));
        assertTrue(Double.isNaN(fixture.channel.normalizedPowerCommand()));
        assertEquals("REGULATOR_FAILED_STOP_FAILED_RESET_FAILED", fixture.channel.status());
        assertFalse(fixture.channel.lastStopSubmitted());
        assertEquals(Arrays.asList(
                        "regulator.update",
                        "output.stop",
                        "regulator.reset"),
                fixture.events);
    }

    @Test
    public void outputWriteFailureRetainsRawResultAndSuccessfulFailStopSubmitsZero() {
        Fixture fixture = new Fixture("test.outputFailure");
        RuntimeException outputFailure = new IllegalStateException("write failed");
        fixture.regulator.result = 0.6;
        fixture.output.writeFailure = outputFailure;

        RuntimeException observed = expectRuntime(() -> fixture.channel.update(
                7.0,
                2.0,
                new ManualLoopClock().clock()));

        assertSame(outputFailure, observed);
        assertEquals(0, observed.getSuppressed().length);
        assertEquals(0.6, fixture.channel.regulatorOutput(), 0.0);
        assertEquals(0.0, fixture.channel.normalizedPowerCommand(), 0.0);
        assertEquals("OUTPUT_WRITE_FAILED_STOP_SUBMITTED_RESET_SUCCEEDED",
                fixture.channel.status());
        assertTrue(fixture.channel.lastStopSubmitted());
        assertEquals(Arrays.asList(0.6), fixture.output.attemptedPowers);
        assertEquals(Arrays.asList(
                        "regulator.update",
                        "output.setPower",
                        "output.stop",
                        "regulator.reset"),
                fixture.events);
    }

    @Test
    public void resetDoesNotWriteAndRetainsTheLastSubmittedCommand() {
        Fixture fixture = new Fixture("test.reset");
        fixture.regulator.result = 0.45;
        fixture.channel.update(1.0, 0.0, new ManualLoopClock().clock());
        fixture.events.clear();

        fixture.channel.reset();

        assertEquals(Arrays.asList("regulator.reset"), fixture.events);
        assertEquals(1, fixture.output.setPowerCount);
        assertEquals(0, fixture.output.stopCount);
        assertTrue(Double.isNaN(fixture.channel.regulatorOutput()));
        assertEquals(0.45, fixture.channel.normalizedPowerCommand(), 0.0);
        assertEquals("RESET_WITHOUT_WRITE", fixture.channel.status());
        assertFalse(fixture.channel.lastStopSubmitted());

        fixture.events.clear();
        RuntimeException resetFailure = new IllegalStateException("reset failed");
        fixture.regulator.resetFailure = resetFailure;

        RuntimeException observed = expectRuntime(fixture.channel::reset);

        assertSame(resetFailure, observed);
        assertEquals(Arrays.asList("regulator.reset"), fixture.events);
        assertEquals(1, fixture.output.setPowerCount);
        assertEquals(0, fixture.output.stopCount);
        assertTrue(Double.isNaN(fixture.channel.regulatorOutput()));
        assertEquals(0.45, fixture.channel.normalizedPowerCommand(), 0.0);
        assertEquals("RESET_FAILED_WITHOUT_WRITE", fixture.channel.status());
        assertFalse(fixture.channel.lastStopSubmitted());
    }

    @Test
    public void successfulStopSubmitsZeroEvenWhenRegulatorResetFails() {
        Fixture fixture = updatedFixture("test.stopResetFailure", 0.7);
        RuntimeException resetFailure = new IllegalStateException("reset failed");
        fixture.regulator.resetFailure = resetFailure;

        RuntimeException observed = expectRuntime(fixture.channel::stop);

        assertSame(resetFailure, observed);
        assertEquals(0, observed.getSuppressed().length);
        assertEquals(Arrays.asList("output.stop", "regulator.reset"), fixture.events);
        assertEquals(0.0, fixture.channel.normalizedPowerCommand(), 0.0);
        assertTrue(Double.isNaN(fixture.channel.regulatorOutput()));
        assertEquals("STOP_SUBMITTED_RESET_FAILED", fixture.channel.status());
        assertTrue(fixture.channel.lastStopSubmitted());
    }

    @Test
    public void successfulStopSubmitsZeroThenResetsTheRegulator() {
        Fixture fixture = updatedFixture("test.stop", 0.7);

        fixture.channel.stop();

        assertEquals(Arrays.asList("output.stop", "regulator.reset"), fixture.events);
        assertEquals(0.0, fixture.channel.normalizedPowerCommand(), 0.0);
        assertTrue(Double.isNaN(fixture.channel.regulatorOutput()));
        assertEquals("STOP_SUBMITTED_RESET_SUCCEEDED", fixture.channel.status());
        assertTrue(fixture.channel.lastStopSubmitted());
    }

    @Test
    public void failedStopMakesCommandTruthUnknownAndStillAttemptsReset() {
        Fixture fixture = updatedFixture("test.stopFailure", -0.7);
        RuntimeException stopFailure = new IllegalStateException("stop failed");
        fixture.output.stopFailure = stopFailure;

        RuntimeException observed = expectRuntime(fixture.channel::stop);

        assertSame(stopFailure, observed);
        assertEquals(Arrays.asList("output.stop", "regulator.reset"), fixture.events);
        assertTrue(Double.isNaN(fixture.channel.normalizedPowerCommand()));
        assertTrue(Double.isNaN(fixture.channel.regulatorOutput()));
        assertEquals("STOP_FAILED_RESET_SUCCEEDED", fixture.channel.status());
        assertFalse(fixture.channel.lastStopSubmitted());
    }

    @Test
    public void stopPreservesFirstFailureAndSuppressesLaterResetFailure() {
        Fixture fixture = updatedFixture("test.stopFailures", 0.7);
        RuntimeException stopFailure = new IllegalStateException("stop failed");
        RuntimeException resetFailure = new IllegalArgumentException("reset failed");
        fixture.output.stopFailure = stopFailure;
        fixture.regulator.resetFailure = resetFailure;

        RuntimeException observed = expectRuntime(fixture.channel::stop);

        assertSame(stopFailure, observed);
        assertEquals(1, observed.getSuppressed().length);
        assertSame(resetFailure, observed.getSuppressed()[0]);
        assertEquals(Arrays.asList("output.stop", "regulator.reset"), fixture.events);
        assertTrue(Double.isNaN(fixture.channel.normalizedPowerCommand()));
        assertEquals("STOP_FAILED_RESET_FAILED", fixture.channel.status());
        assertFalse(fixture.channel.lastStopSubmitted());
    }

    @Test
    public void stopDeduplicatesSharedCompanionAndStopsDistinctCompanionBeforeReset() {
        Fixture sharedFixture = updatedFixture("test.sharedStop", 0.7);

        sharedFixture.channel.stop(sharedFixture.output);

        assertEquals(Arrays.asList("output.stop", "regulator.reset"), sharedFixture.events);
        assertEquals(1, sharedFixture.output.stopCount);

        Fixture distinctFixture = updatedFixture("test.distinctStop", 0.7);
        ProbePowerOutput companion = new ProbePowerOutput(distinctFixture.events);

        distinctFixture.channel.stop(companion);

        assertEquals(Arrays.asList("output.stop", "output.stop", "regulator.reset"),
                distinctFixture.events);
        assertEquals(1, distinctFixture.output.stopCount);
        assertEquals(1, companion.stopCount);
        assertTrue(distinctFixture.channel.lastStopSubmitted());
    }

    @Test
    public void companionStopFailureMakesCombinedCommandTruthUnknown() {
        Fixture fixture = updatedFixture("test.companionStopFailure", 0.7);
        ProbePowerOutput companion = new ProbePowerOutput(fixture.events);
        RuntimeException companionFailure = new IllegalStateException("companion stop failed");
        companion.stopFailure = companionFailure;

        RuntimeException observed = expectRuntime(() -> fixture.channel.stop(companion));

        assertSame(companionFailure, observed);
        assertEquals(0, observed.getSuppressed().length);
        assertEquals(Arrays.asList("output.stop", "output.stop", "regulator.reset"),
                fixture.events);
        assertEquals(1, fixture.output.stopCount);
        assertEquals(1, companion.stopCount);
        assertTrue(Double.isNaN(fixture.channel.normalizedPowerCommand()));
        assertEquals("STOP_FAILED_RESET_SUCCEEDED", fixture.channel.status());
        assertFalse(fixture.channel.lastStopSubmitted());
    }

    @Test
    public void errorsEscapeWithoutRuntimeFailStopCleanup() {
        Fixture regulatorErrorFixture = new Fixture("test.regulatorError");
        AssertionError regulatorError = new AssertionError("regulator error");
        regulatorErrorFixture.regulator.updateError = regulatorError;

        Error observedRegulatorError = expectError(() -> regulatorErrorFixture.channel.update(
                1.0,
                0.0,
                new ManualLoopClock().clock()));

        assertSame(regulatorError, observedRegulatorError);
        assertEquals(Arrays.asList("regulator.update"), regulatorErrorFixture.events);
        assertEquals(0, regulatorErrorFixture.output.stopCount);
        assertEquals(0, regulatorErrorFixture.regulator.resetCount);

        Fixture outputErrorFixture = new Fixture("test.outputError");
        AssertionError outputError = new AssertionError("output error");
        outputErrorFixture.regulator.result = 0.3;
        outputErrorFixture.output.writeError = outputError;

        Error observedOutputError = expectError(() -> outputErrorFixture.channel.update(
                1.0,
                0.0,
                new ManualLoopClock().clock()));

        assertSame(outputError, observedOutputError);
        assertEquals(Arrays.asList("regulator.update", "output.setPower"),
                outputErrorFixture.events);
        assertEquals(0, outputErrorFixture.output.stopCount);
        assertEquals(0, outputErrorFixture.regulator.resetCount);
    }

    @Test
    public void debugDumpUsesStableCanonicalKeysAndDelegatesToRegulator() {
        Fixture fixture = new Fixture("test.debug");
        fixture.channel.debugDump(null, null);
        CapturingDebugSink initialDebug = new CapturingDebugSink();
        fixture.channel.debugDump(initialDebug, "shooter.flywheel");

        assertTrue(Double.isNaN(number(initialDebug, "shooter.flywheel.regulatorOutput")));
        assertTrue(Double.isNaN(number(initialDebug,
                "shooter.flywheel.normalizedPowerCommand")));
        assertEquals("NOT_UPDATED",
                initialDebug.data.get("shooter.flywheel.regulatedPowerStatus"));
        assertEquals(0,
                ((Number) initialDebug.data.get(
                        "shooter.flywheel.regulator.probeUpdateCount")).intValue());

        fixture.regulator.result = 1.2;
        fixture.channel.update(3.0, 1.0, new ManualLoopClock().clock());
        CapturingDebugSink debug = new CapturingDebugSink();

        fixture.channel.debugDump(debug, "shooter.flywheel");

        assertEquals(1.2, number(debug, "shooter.flywheel.regulatorOutput"), 0.0);
        assertEquals(1.0, number(debug, "shooter.flywheel.normalizedPowerCommand"), 0.0);
        assertEquals("SATURATED_AND_SUBMITTED",
                debug.data.get("shooter.flywheel.regulatedPowerStatus"));
        assertEquals(fixture.regulator.updateCount,
                ((Number) debug.data.get("shooter.flywheel.regulator.probeUpdateCount")).intValue());
    }

    private static Fixture updatedFixture(String controlPath, double command) {
        Fixture fixture = new Fixture(controlPath);
        fixture.regulator.result = command;
        fixture.channel.update(1.0, 0.0, new ManualLoopClock().clock());
        fixture.events.clear();
        return fixture;
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

    private static Error expectError(Runnable action) {
        try {
            action.run();
            fail("Expected Error");
            return null;
        } catch (Error expected) {
            return expected;
        }
    }

    private static void assertContains(String text, String... fragments) {
        for (String fragment : fragments) {
            assertTrue("Expected <" + text + "> to contain <" + fragment + ">",
                    text.contains(fragment));
        }
    }

    private static void assertSameBits(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    private static void assertSameNonFinite(double expected, double actual) {
        if (Double.isNaN(expected)) {
            assertTrue(Double.isNaN(actual));
        } else {
            assertEquals(expected, actual, 0.0);
        }
    }

    private static double number(CapturingDebugSink debug, String key) {
        Object value = debug.data.get(key);
        assertTrue(key + " must contain a number", value instanceof Number);
        return ((Number) value).doubleValue();
    }

    private static final class Fixture {
        private final List<String> events = new ArrayList<>();
        private final ProbePowerOutput output = new ProbePowerOutput(events);
        private final ProbeRegulator regulator = new ProbeRegulator(events);
        private final RegulatedPowerChannel channel;

        private Fixture(String controlPath) {
            channel = new RegulatedPowerChannel(output, regulator, controlPath);
        }
    }

    private static final class ProbePowerOutput implements PowerOutput {
        private final List<String> events;
        private final List<Double> attemptedPowers = new ArrayList<>();
        private double lastSuccessfulPower = Double.NaN;
        private int setPowerCount;
        private int stopCount;
        private RuntimeException writeFailure;
        private RuntimeException stopFailure;
        private Error writeError;

        private ProbePowerOutput(List<String> events) {
            this.events = events;
        }

        @Override
        public void setPower(double power) {
            events.add("output.setPower");
            setPowerCount++;
            attemptedPowers.add(power);
            if (writeError != null) throw writeError;
            if (writeFailure != null) throw writeFailure;
            lastSuccessfulPower = power;
        }

        @Override
        public double getCommandedPower() {
            return lastSuccessfulPower;
        }

        @Override
        public void stop() {
            events.add("output.stop");
            stopCount++;
            if (stopFailure != null) throw stopFailure;
            lastSuccessfulPower = 0.0;
        }
    }

    private static final class ProbeRegulator implements ScalarRegulator {
        private final List<String> events;
        private double result;
        private int updateCount;
        private int resetCount;
        private RuntimeException updateFailure;
        private RuntimeException resetFailure;
        private Error updateError;

        private ProbeRegulator(List<String> events) {
            this.events = events;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            events.add("regulator.update");
            updateCount++;
            if (updateError != null) throw updateError;
            if (updateFailure != null) throw updateFailure;
            return result;
        }

        @Override
        public void reset() {
            events.add("regulator.reset");
            resetCount++;
            if (resetFailure != null) throw resetFailure;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            dbg.addData(prefix + ".probeUpdateCount", updateCount);
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
