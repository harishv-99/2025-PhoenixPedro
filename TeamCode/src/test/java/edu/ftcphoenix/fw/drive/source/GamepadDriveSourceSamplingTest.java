package edu.ftcphoenix.fw.drive.source;

import org.junit.Test;

import java.util.HashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

/** Verifies one-capture axis sampling and structural reset behavior. */
public final class GamepadDriveSourceSamplingTest {

    @Test
    public void eachAxisIsSampledOnceAndTheSameCaptureFeedsCommandAndDiagnostics() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingScalarSource lateral = new RecordingScalarSource(0.4);
        RecordingScalarSource axial = new RecordingScalarSource(0.5);
        RecordingScalarSource omega = new RecordingScalarSource(-0.8);
        GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
        config.deadband = 0.0;
        config.translateExpo = 1.0;
        config.rotateExpo = 1.0;
        config.translateScale = 0.5;
        config.rotateScale = 0.25;
        GamepadDriveSource source = new GamepadDriveSource(lateral, axial, omega, config);

        DriveSignal signal = source.get(time.clock());

        assertEquals(1, lateral.getCount);
        assertEquals(1, axial.getCount);
        assertEquals(1, omega.getCount);
        assertEquals(0.25, signal.axial, 0.0);
        assertEquals(-0.2, signal.lateral, 0.0);
        assertEquals(0.2, signal.omega, 0.0);

        CapturingDebugSink debug = new CapturingDebugSink();
        source.debugDump(debug, "drive");
        assertEquals(0.4, (Double) debug.values.get("drive.axis.lateral.raw"), 0.0);
        assertEquals(0.5, (Double) debug.values.get("drive.axis.axial.raw"), 0.0);
        assertEquals(-0.8, (Double) debug.values.get("drive.axis.omega.raw"), 0.0);
        assertEquals(signal.axial, (Double) debug.values.get("drive.last.axial"), 0.0);
        assertEquals(signal.lateral, (Double) debug.values.get("drive.last.lateral"), 0.0);
        assertEquals(signal.omega, (Double) debug.values.get("drive.last.omega"), 0.0);

        // This source is a pure mapping, not a behavior cache: another get is another capture.
        source.get(time.clock());
        assertEquals(2, lateral.getCount);
        assertEquals(2, axial.getCount);
        assertEquals(2, omega.getCount);
    }

    @Test
    public void resetClearsDiagnosticsAndCascadesToAllSuppliedAxes() {
        RecordingScalarSource lateral = new RecordingScalarSource(0.4);
        RecordingScalarSource axial = new RecordingScalarSource(0.5);
        RecordingScalarSource omega = new RecordingScalarSource(-0.8);
        GamepadDriveSource source = new GamepadDriveSource(
                lateral, axial, omega, GamepadDriveSource.Config.defaults());
        source.get(new ManualLoopClock().clock());

        source.reset();

        assertEquals(1, lateral.resetCount);
        assertEquals(1, axial.resetCount);
        assertEquals(1, omega.resetCount);
        assertSame(DriveSignal.zero(), source.getLastSignal());
        CapturingDebugSink debug = new CapturingDebugSink();
        source.debugDump(debug, "drive");
        assertEquals(0.0, (Double) debug.values.get("drive.axis.lateral.raw"), 0.0);
        assertEquals(0.0, (Double) debug.values.get("drive.axis.axial.raw"), 0.0);
        assertEquals(0.0, (Double) debug.values.get("drive.axis.omega.raw"), 0.0);
        assertEquals(0.0, (Double) debug.values.get("drive.last.axial"), 0.0);
        assertEquals(0.0, (Double) debug.values.get("drive.last.lateral"), 0.0);
        assertEquals(0.0, (Double) debug.values.get("drive.last.omega"), 0.0);
    }

    private static final class RecordingScalarSource implements ScalarSource {
        private final double value;
        private int getCount;
        private int resetCount;

        private RecordingScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            getCount++;
            return value;
        }

        @Override
        public void reset() {
            resetCount++;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new HashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            values.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
