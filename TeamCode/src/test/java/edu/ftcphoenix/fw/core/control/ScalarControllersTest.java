package edu.ftcphoenix.fw.core.control;

import java.util.LinkedHashMap;
import java.util.Map;

import org.junit.Test;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies successful publication and non-rollback attempt handling for scalar controllers. */
public final class ScalarControllersTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void successfulOutputAndDiagnosticsAdvanceOnlyOncePerCycle() {
        ProbeScalarSource setpoint = new ProbeScalarSource(10.0);
        ProbeScalarSource measurement = new ProbeScalarSource(3.0);
        ProbeController controller = new ProbeController();
        controller.outputScale = 2.0;
        ScalarSource output = ScalarControllers.pid(setpoint, measurement, controller);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(14.0, output.getAsDouble(time.clock()), EPSILON);
        measurement.value = 4.0;
        assertEquals(14.0, output.getAsDouble(time.clock()), EPSILON);
        assertEquals(1, setpoint.samples);
        assertEquals(1, measurement.samples);
        assertEquals(1, controller.updates);

        assertEquals(12.0, output.getAsDouble(time.nextCycle(0.02)), EPSILON);
        assertEquals(2, setpoint.samples);
        assertEquals(2, measurement.samples);
        assertEquals(2, controller.updates);
    }

    @Test
    public void failedInputAndControllerAttemptsPreservePublishedDiagnostics() {
        ProbeScalarSource setpoint = new ProbeScalarSource(10.0);
        ProbeScalarSource measurement = new ProbeScalarSource(3.0);
        ProbeController controller = new ProbeController();
        ScalarSource output = ScalarControllers.pid(setpoint, measurement, controller);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(7.0, output.getAsDouble(time.clock()), EPSILON);
        assertDiagnostics(output, 10.0, 3.0, 7.0, 7.0);

        LoopClock nextCycle = time.nextCycle(0.02);
        setpoint.value = 20.0;
        measurement.value = 4.0;
        measurement.sampleFailure = new IllegalStateException("measurement failure");
        captureFailure(() -> output.getAsDouble(nextCycle));
        assertDiagnostics(output, 10.0, 3.0, 7.0, 7.0);

        measurement.sampleFailure = null;
        controller.updateFailure = new IllegalArgumentException("controller failure");
        captureFailure(() -> output.getAsDouble(nextCycle));
        assertDiagnostics(output, 10.0, 3.0, 7.0, 7.0);

        controller.updateFailure = null;
        assertEquals(16.0, output.getAsDouble(time.nextCycle(0.02)), EPSILON);
        assertDiagnostics(output, 20.0, 4.0, 16.0, 16.0);
    }

    @Test
    public void inputFailuresRetryBeforeTheControllerAttempt() {
        ProbeScalarSource setpoint = new ProbeScalarSource(10.0);
        ProbeScalarSource measurement = new ProbeScalarSource(3.0);
        ProbeController controller = new ProbeController();
        ScalarSource output = ScalarControllers.pid(setpoint, measurement, controller);
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException setpointFailure = new IllegalStateException("setpoint failure");
        setpoint.sampleFailure = setpointFailure;

        assertSame(setpointFailure, captureFailure(() -> output.getAsDouble(time.clock())));
        assertEquals(0, measurement.samples);
        assertEquals(0, controller.updates);

        setpoint.sampleFailure = null;
        assertEquals(7.0, output.getAsDouble(time.clock()), EPSILON);
        assertEquals(2, setpoint.samples);
        assertEquals(1, measurement.samples);
        assertEquals(1, controller.updates);

        RuntimeException measurementFailure = new IllegalArgumentException("measurement failure");
        measurement.sampleFailure = measurementFailure;
        LoopClock nextCycle = time.nextCycle(0.02);
        assertSame(measurementFailure, captureFailure(() -> output.getAsDouble(nextCycle)));
        assertEquals(1, controller.updates);

        measurement.sampleFailure = null;
        assertEquals(7.0, output.getAsDouble(nextCycle), EPSILON);
        assertEquals(2, controller.updates);
    }

    @Test
    public void controllerFailureIsRetainedByIdentityWithoutSameCycleReplay() {
        ProbeScalarSource setpoint = new ProbeScalarSource(10.0);
        ProbeScalarSource measurement = new ProbeScalarSource(3.0);
        ProbeController controller = new ProbeController();
        ScalarSource output = ScalarControllers.pid(setpoint, measurement, controller);
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException controllerFailure = new IllegalStateException("controller failure");
        controller.updateFailure = controllerFailure;

        assertSame(controllerFailure, captureFailure(() -> output.getAsDouble(time.clock())));
        controller.updateFailure = null;
        assertSame(controllerFailure, captureFailure(() -> output.getAsDouble(time.clock())));
        assertEquals(1, setpoint.samples);
        assertEquals(1, measurement.samples);
        assertEquals(1, controller.updates);

        assertEquals(7.0, output.getAsDouble(time.nextCycle(0.02)), EPSILON);
        assertEquals(2, setpoint.samples);
        assertEquals(2, measurement.samples);
        assertEquals(2, controller.updates);
    }

    @Test
    public void recursiveSamplingFailsFastAndAValueRetryCanRecover() {
        final ScalarSource[] output = new ScalarSource[1];
        final boolean[] recurse = {true};
        ProbeController controller = new ProbeController();
        ScalarSource setpoint = clock -> recurse[0]
                ? output[0].getAsDouble(clock)
                : 10.0;
        output[0] = ScalarControllers.pid(
                setpoint, ScalarSource.constant(3.0), controller);
        ManualLoopClock time = new ManualLoopClock();

        RuntimeException reentry = captureFailure(() -> output[0].getAsDouble(time.clock()));
        assertTrue(reentry instanceof IllegalStateException);
        assertTrue(reentry.getMessage().contains("reentered"));
        assertEquals(0, controller.updates);

        recurse[0] = false;
        assertEquals(7.0, output[0].getAsDouble(time.clock()), EPSILON);
        assertEquals(1, controller.updates);
    }

    @Test
    public void failedResetPreservesTheRetainedAttemptUntilAllOwnedResetsSucceed() {
        ProbeScalarSource setpoint = new ProbeScalarSource(10.0);
        ProbeScalarSource measurement = new ProbeScalarSource(3.0);
        ProbeController controller = new ProbeController();
        ScalarSource output = ScalarControllers.pid(setpoint, measurement, controller);
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException controllerFailure = new IllegalStateException("controller failure");
        RuntimeException resetFailure = new IllegalArgumentException("measurement reset failure");
        controller.updateFailure = controllerFailure;

        assertSame(controllerFailure, captureFailure(() -> output.getAsDouble(time.clock())));
        measurement.resetFailure = resetFailure;
        assertSame(resetFailure, captureFailure(output::reset));

        controller.updateFailure = null;
        assertSame(controllerFailure, captureFailure(() -> output.getAsDouble(time.clock())));
        assertEquals(1, controller.updates);
        assertEquals(0, controller.resets);

        measurement.resetFailure = null;
        output.reset();
        assertEquals(2, setpoint.resets);
        assertEquals(2, measurement.resets);
        assertEquals(1, controller.resets);
        assertEquals(7.0, output.getAsDouble(time.clock()), EPSILON);
        assertEquals(2, controller.updates);
    }

    @Test
    public void samplingDuringResetFailsFastWithoutClearingTheCommittedCycle() {
        ProbeScalarSource setpoint = new ProbeScalarSource(10.0);
        ProbeScalarSource measurement = new ProbeScalarSource(3.0);
        ProbeController controller = new ProbeController();
        ScalarSource output = ScalarControllers.pid(setpoint, measurement, controller);
        ManualLoopClock time = new ManualLoopClock();
        assertEquals(7.0, output.getAsDouble(time.clock()), EPSILON);
        setpoint.resetAction = () -> output.getAsDouble(time.clock());

        RuntimeException overlap = captureFailure(output::reset);
        assertTrue(overlap instanceof IllegalStateException);
        assertTrue(overlap.getMessage().contains("reset is in progress"));
        assertEquals(7.0, output.getAsDouble(time.clock()), EPSILON);
        assertEquals(1, controller.updates);

        setpoint.resetAction = null;
        output.reset();
        assertEquals(7.0, output.getAsDouble(time.clock()), EPSILON);
        assertEquals(2, controller.updates);
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static void assertDiagnostics(ScalarSource source,
                                          double setpoint,
                                          double measurement,
                                          double error,
                                          double output) {
        CapturingDebugSink debug = new CapturingDebugSink();
        source.debugDump(debug, "pid");
        assertEquals(setpoint, ((Number) debug.data.get("pid.setpoint")).doubleValue(), EPSILON);
        assertEquals(
                measurement,
                ((Number) debug.data.get("pid.measurement")).doubleValue(),
                EPSILON
        );
        assertEquals(error, ((Number) debug.data.get("pid.error")).doubleValue(), EPSILON);
        assertEquals(output, ((Number) debug.data.get("pid.output")).doubleValue(), EPSILON);
    }

    private static final class ProbeScalarSource implements ScalarSource {
        private double value;
        private int samples;
        private int resets;
        private RuntimeException sampleFailure;
        private RuntimeException resetFailure;
        private Runnable resetAction;

        ProbeScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            samples++;
            if (sampleFailure != null) throw sampleFailure;
            return value;
        }

        @Override
        public void reset() {
            resets++;
            if (resetAction != null) resetAction.run();
            if (resetFailure != null) throw resetFailure;
        }
    }

    private static final class ProbeController implements PidController {
        private int updates;
        private int resets;
        private double outputScale = 1.0;
        private RuntimeException updateFailure;

        @Override
        public double update(double error, double dtSec) {
            updates++;
            if (updateFailure != null) throw updateFailure;
            return error * outputScale;
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
