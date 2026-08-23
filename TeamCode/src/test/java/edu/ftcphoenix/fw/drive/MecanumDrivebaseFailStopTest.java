package edu.ftcphoenix.fw.drive;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.hal.PowerOutput;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Protects SAFE-04's fail-stop fanout and truthful Mecanum diagnostics. */
public final class MecanumDrivebaseFailStopTest {

    private static final double EPSILON = 1.0e-9;
    private static final List<String> ALL_STOPS = Arrays.asList(
            "fl.stop", "fr.stop", "bl.stop", "br.stop");

    @Test
    public void diagnosticsStartUnknown() {
        Fixture fixture = new Fixture();

        assertAllUnknown(fixture.drive);
        assertTrue(fixture.events.isEmpty());
    }

    @Test
    public void successfulDrivePreservesScalingMixingRatiosAndCommitsAfterFanout() {
        MecanumDrivebase.Config config = MecanumDrivebase.Config.defaults();
        config.maxAxial = 0.8;
        config.maxLateral = 0.6;
        config.maxOmega = 0.5;
        Fixture fixture = new Fixture(config);

        fixture.drive.drive(new DriveSignal(1.0, 1.0, -1.0));

        // Scaled command: (0.8, 0.6, -0.5). Raw mix: (0.7, 0.9, 1.9, -0.3).
        // Dividing every wheel by 1.9 preserves the mix ratios.
        assertEquals(Arrays.asList(
                "fl.write", "fr.write", "bl.write", "br.write"), fixture.events);
        assertEquals(0.7 / 1.9, fixture.fl.power, EPSILON);
        assertEquals(0.9 / 1.9, fixture.fr.power, EPSILON);
        assertEquals(1.0, fixture.bl.power, EPSILON);
        assertEquals(-0.3 / 1.9, fixture.br.power, EPSILON);
        assertDiagnostics(
                fixture.drive,
                0.8,
                0.6,
                -0.5,
                0.7 / 1.9,
                0.9 / 1.9,
                1.0,
                -0.3 / 1.9);
    }

    @Test
    public void diagnosticsRemainUnknownUntilEverySuccessfulWheelOperationReturns() {
        Fixture fixture = new Fixture();
        for (ProbeOutput wheel : fixture.wheels()) {
            wheel.writeObserver = () -> assertAllUnknown(fixture.drive);
            wheel.stopObserver = () -> assertAllUnknown(fixture.drive);
        }

        fixture.drive.drive(new DriveSignal(0.2, -0.1, 0.05));
        assertDiagnostics(fixture.drive, 0.2, -0.1, 0.05, 0.25, 0.15, 0.05, 0.35);

        fixture.drive.stop();
        assertAllZero(fixture.drive);
    }

    @Test
    public void everyWheelWriteFailureAbandonsLaterWritesStopsAllAndInvalidatesDiagnostics() {
        String[] wheelNames = {"fl", "fr", "bl", "br"};

        for (int failedIndex = 0; failedIndex < wheelNames.length; failedIndex++) {
            Fixture fixture = new Fixture();
            fixture.drive.drive(new DriveSignal(0.1, 0.0, 0.0));
            fixture.events.clear();

            RuntimeException writeFailure =
                    new IllegalStateException(wheelNames[failedIndex] + " write failed");
            fixture.wheels()[failedIndex].writeFailure = writeFailure;

            RuntimeException thrown = expectRuntimeException(
                    () -> fixture.drive.drive(new DriveSignal(0.2, -0.1, 0.05)));

            assertSame(writeFailure, thrown);
            List<String> expectedEvents = new ArrayList<>();
            for (int writeIndex = 0; writeIndex <= failedIndex; writeIndex++) {
                expectedEvents.add(wheelNames[writeIndex] + ".write");
            }
            expectedEvents.addAll(ALL_STOPS);
            assertEquals(expectedEvents, fixture.events);
            assertAllUnknown(fixture.drive);
        }
    }

    @Test
    public void driveFailureKeepsExactPrimaryAndSuppressesDistinctCleanupFailuresInOrder() {
        Fixture fixture = new Fixture();
        RuntimeException writeFailure = new IllegalStateException("fr write failed");
        RuntimeException firstCleanup = new IllegalArgumentException("fl stop failed");
        RuntimeException laterCleanup = new UnsupportedOperationException("bl stop failed");
        fixture.fr.writeFailure = writeFailure;
        fixture.fl.stopFailure = firstCleanup;
        fixture.fr.stopFailure = writeFailure; // Never suppress a failure onto itself.
        fixture.bl.stopFailure = laterCleanup;

        RuntimeException thrown = expectRuntimeException(
                () -> fixture.drive.drive(new DriveSignal(0.3, 0.0, 0.0)));

        assertSame(writeFailure, thrown);
        assertArrayEquals(
                new Throwable[]{firstCleanup, laterCleanup},
                thrown.getSuppressed());
        assertEquals(Arrays.asList(
                "fl.write", "fr.write",
                "fl.stop", "fr.stop", "bl.stop", "br.stop"),
                fixture.events);
        assertAllUnknown(fixture.drive);
    }

    @Test
    public void middleWheelPostEffectFailureStillAbandonsLaterWritesAndStopsAll() {
        Fixture fixture = new Fixture();
        fixture.drive.drive(new DriveSignal(0.1, 0.0, 0.0));
        fixture.events.clear();
        int blEffectsBefore = fixture.bl.writeEffects;
        int brEffectsBefore = fixture.br.writeEffects;
        RuntimeException writeFailure = new IllegalStateException("bl post-effect failure");
        RuntimeException firstCleanup = new IllegalArgumentException("fl cleanup failure");
        RuntimeException secondCleanup = new UnsupportedOperationException("br cleanup failure");
        fixture.bl.writeFailure = writeFailure;
        fixture.bl.failWriteAfterEffect = true;
        fixture.fl.stopFailure = firstCleanup;
        fixture.br.stopFailure = secondCleanup;

        RuntimeException thrown = expectRuntimeException(
                () -> fixture.drive.drive(new DriveSignal(0.2, -0.1, 0.05)));

        assertSame(writeFailure, thrown);
        assertArrayEquals(
                new Throwable[]{firstCleanup, secondCleanup},
                thrown.getSuppressed());
        assertEquals(Arrays.asList(
                "fl.write", "fr.write", "bl.write",
                "fl.stop", "fr.stop", "bl.stop", "br.stop"),
                fixture.events);
        assertEquals(blEffectsBefore + 1, fixture.bl.writeEffects);
        assertEquals(brEffectsBefore, fixture.br.writeEffects);
        assertAllUnknown(fixture.drive);
    }

    @Test
    public void everyExplicitStopFailureStillAttemptsEveryWheelAndLeavesDiagnosticsUnknown() {
        String[] wheelNames = {"fl", "fr", "bl", "br"};

        for (int failedIndex = 0; failedIndex < wheelNames.length; failedIndex++) {
            Fixture fixture = new Fixture();
            fixture.drive.drive(new DriveSignal(0.25, -0.2, 0.1));
            fixture.events.clear();

            RuntimeException stopFailure =
                    new IllegalStateException(wheelNames[failedIndex] + " stop failed");
            fixture.wheels()[failedIndex].stopFailure = stopFailure;

            RuntimeException thrown = expectRuntimeException(fixture.drive::stop);

            assertSame(stopFailure, thrown);
            assertEquals(ALL_STOPS, fixture.events);
            assertAllUnknown(fixture.drive);
        }
    }

    @Test
    public void explicitStopKeepsFirstFailureAndSuppressesLaterDistinctFailuresInOrder() {
        Fixture fixture = new Fixture();
        RuntimeException first = new IllegalStateException("fl stop failed");
        RuntimeException second = new IllegalArgumentException("bl stop failed");
        RuntimeException third = new UnsupportedOperationException("br stop failed");
        fixture.fl.stopFailure = first;
        fixture.bl.stopFailure = second;
        fixture.br.stopFailure = third;

        RuntimeException thrown = expectRuntimeException(fixture.drive::stop);

        assertSame(first, thrown);
        assertArrayEquals(new Throwable[]{second, third}, thrown.getSuppressed());
        assertEquals(ALL_STOPS, fixture.events);
        assertAllUnknown(fixture.drive);
    }

    @Test
    public void repeatedSuccessfulStopAlwaysFansOutAndPublishesZero() {
        Fixture fixture = new Fixture();

        fixture.drive.stop();
        assertEquals(ALL_STOPS, fixture.events);
        assertAllZero(fixture.drive);

        fixture.events.clear();
        fixture.drive.stop();
        assertEquals(ALL_STOPS, fixture.events);
        assertAllZero(fixture.drive);
    }

    @Test
    public void nullDriveInvalidatesPriorDiagnosticsWithoutTouchingWheels() {
        Fixture fixture = new Fixture();
        fixture.drive.drive(new DriveSignal(0.1, 0.0, 0.0));
        fixture.events.clear();

        RuntimeException thrown = expectRuntimeException(() -> fixture.drive.drive(null));

        assertTrue(thrown instanceof NullPointerException);
        assertTrue(fixture.events.isEmpty());
        assertAllUnknown(fixture.drive);
    }

    @Test
    public void nonFiniteSignalComponentsRejectBeforeWritesAndStopEveryWheel() {
        double[][] invalidSignals = {
                {Double.NaN, 0.0, 0.0},
                {Double.POSITIVE_INFINITY, 0.0, 0.0},
                {Double.NEGATIVE_INFINITY, 0.0, 0.0},
                {0.0, Double.NaN, 0.0},
                {0.0, Double.POSITIVE_INFINITY, 0.0},
                {0.0, Double.NEGATIVE_INFINITY, 0.0},
                {0.0, 0.0, Double.NaN},
                {0.0, 0.0, Double.POSITIVE_INFINITY},
                {0.0, 0.0, Double.NEGATIVE_INFINITY}
        };

        for (double[] signal : invalidSignals) {
            Fixture fixture = new Fixture();
            fixture.drive.drive(new DriveSignal(0.1, 0.1, 0.1));
            fixture.events.clear();

            RuntimeException thrown = expectRuntimeException(() -> fixture.drive.drive(
                    new DriveSignal(signal[0], signal[1], signal[2])));

            assertTrue(thrown instanceof IllegalArgumentException);
            assertTrue(thrown.getMessage(),
                    thrown.getMessage().contains("requires finite DriveSignal components"));
            assertTrue(thrown.getMessage(), thrown.getMessage().contains("axial="));
            assertTrue(thrown.getMessage(), thrown.getMessage().contains("lateral="));
            assertTrue(thrown.getMessage(), thrown.getMessage().contains("omega="));
            assertEquals(ALL_STOPS, fixture.events);
            assertAllUnknown(fixture.drive);
        }
    }

    @Test
    public void nonFiniteDerivedWheelRejectsBeforeWritesAndRetainsCleanupFailures() {
        Fixture fixture = new Fixture();
        RuntimeException firstCleanup = new IllegalStateException("fr stop failed");
        RuntimeException secondCleanup = new IllegalArgumentException("br stop failed");
        fixture.fr.stopFailure = firstCleanup;
        fixture.br.stopFailure = secondCleanup;

        RuntimeException thrown = expectRuntimeException(() -> fixture.drive.drive(
                new DriveSignal(Double.MAX_VALUE, -Double.MAX_VALUE, 0.0)));

        assertTrue(thrown instanceof IllegalArgumentException);
        assertTrue(thrown.getMessage(),
                thrown.getMessage().contains("derived non-finite wheel power before fanout"));
        assertTrue(thrown.getMessage(), thrown.getMessage().contains("fl=Infinity"));
        assertArrayEquals(
                new Throwable[]{firstCleanup, secondCleanup},
                thrown.getSuppressed());
        assertEquals(ALL_STOPS, fixture.events);
        assertAllUnknown(fixture.drive);
    }

    @Test
    public void laterSuccessfulDriveRecoversAfterFailedWrite() {
        Fixture fixture = new Fixture();
        RuntimeException writeFailure = new IllegalStateException("bl write failed");
        fixture.bl.writeFailure = writeFailure;

        assertSame(writeFailure, expectRuntimeException(
                () -> fixture.drive.drive(new DriveSignal(0.4, 0.0, 0.0))));
        assertAllUnknown(fixture.drive);

        fixture.bl.writeFailure = null;
        fixture.events.clear();
        fixture.drive.drive(new DriveSignal(0.2, 0.1, 0.0));

        assertEquals(Arrays.asList(
                "fl.write", "fr.write", "bl.write", "br.write"), fixture.events);
        assertDiagnostics(fixture.drive, 0.2, 0.1, 0.0, 0.1, 0.3, 0.3, 0.1);
    }

    @Test
    public void explicitStopRetriesAllWheelsAfterAutomaticFailStopAndCanRecoverDiagnostics() {
        Fixture fixture = new Fixture();
        RuntimeException writeFailure = new IllegalStateException("fl write failed");
        fixture.fl.writeFailure = writeFailure;

        assertSame(writeFailure, expectRuntimeException(
                () -> fixture.drive.drive(new DriveSignal(0.4, 0.0, 0.0))));
        assertEquals(Arrays.asList(
                "fl.write", "fl.stop", "fr.stop", "bl.stop", "br.stop"),
                fixture.events);

        fixture.fl.writeFailure = null;
        fixture.events.clear();
        fixture.drive.stop();

        assertEquals(ALL_STOPS, fixture.events);
        assertAllZero(fixture.drive);
    }

    @Test
    public void laterSuccessfulStopRecoversAfterStopFailure() {
        Fixture fixture = new Fixture();
        RuntimeException stopFailure = new IllegalStateException("br stop failed");
        fixture.br.stopFailure = stopFailure;

        assertSame(stopFailure, expectRuntimeException(fixture.drive::stop));
        assertAllUnknown(fixture.drive);

        fixture.br.stopFailure = null;
        fixture.events.clear();
        fixture.drive.stop();

        assertEquals(ALL_STOPS, fixture.events);
        assertAllZero(fixture.drive);
    }

    private static RuntimeException expectRuntimeException(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static void assertAllUnknown(MecanumDrivebase drive) {
        assertTrue(Double.isNaN(drive.getLastAxialCmd()));
        assertTrue(Double.isNaN(drive.getLastLateralCmd()));
        assertTrue(Double.isNaN(drive.getLastOmegaCmd()));
        assertTrue(Double.isNaN(drive.getLastFlPower()));
        assertTrue(Double.isNaN(drive.getLastFrPower()));
        assertTrue(Double.isNaN(drive.getLastBlPower()));
        assertTrue(Double.isNaN(drive.getLastBrPower()));
    }

    private static void assertAllZero(MecanumDrivebase drive) {
        assertDiagnostics(drive, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    private static void assertDiagnostics(
            MecanumDrivebase drive,
            double axial,
            double lateral,
            double omega,
            double fl,
            double fr,
            double bl,
            double br) {
        assertEquals(axial, drive.getLastAxialCmd(), EPSILON);
        assertEquals(lateral, drive.getLastLateralCmd(), EPSILON);
        assertEquals(omega, drive.getLastOmegaCmd(), EPSILON);
        assertEquals(fl, drive.getLastFlPower(), EPSILON);
        assertEquals(fr, drive.getLastFrPower(), EPSILON);
        assertEquals(bl, drive.getLastBlPower(), EPSILON);
        assertEquals(br, drive.getLastBrPower(), EPSILON);
    }

    private static final class Fixture {
        private final List<String> events = new ArrayList<>();
        private final ProbeOutput fl = new ProbeOutput("fl", events);
        private final ProbeOutput fr = new ProbeOutput("fr", events);
        private final ProbeOutput bl = new ProbeOutput("bl", events);
        private final ProbeOutput br = new ProbeOutput("br", events);
        private final MecanumDrivebase drive;

        private Fixture() {
            this(MecanumDrivebase.Config.defaults());
        }

        private Fixture(MecanumDrivebase.Config config) {
            drive = new MecanumDrivebase(fl, fr, bl, br, config);
        }

        private ProbeOutput[] wheels() {
            return new ProbeOutput[]{fl, fr, bl, br};
        }
    }

    private static final class ProbeOutput implements PowerOutput {
        private final String name;
        private final List<String> events;
        private double power = Double.NaN;
        private RuntimeException writeFailure;
        private boolean failWriteAfterEffect;
        private RuntimeException stopFailure;
        private int writeEffects;
        private Runnable writeObserver;
        private Runnable stopObserver;

        private ProbeOutput(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void setPower(double power) {
            this.power = Double.NaN;
            events.add(name + ".write");
            if (writeFailure != null && !failWriteAfterEffect) {
                throw writeFailure;
            }
            writeEffects++;
            this.power = power;
            if (writeObserver != null) {
                writeObserver.run();
            }
            if (writeFailure != null) {
                throw writeFailure;
            }
        }

        @Override
        public double getCommandedPower() {
            return power;
        }

        @Override
        public void stop() {
            power = Double.NaN;
            events.add(name + ".stop");
            if (stopFailure != null) {
                throw stopFailure;
            }
            if (stopObserver != null) {
                stopObserver.run();
            }
            power = 0.0;
        }
    }
}
