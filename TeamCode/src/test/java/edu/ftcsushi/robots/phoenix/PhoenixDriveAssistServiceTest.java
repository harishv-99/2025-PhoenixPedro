package edu.ftcsushi.robots.phoenix;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix's calibration-dependent TeleOp assist policy without hardware. */
public final class PhoenixDriveAssistServiceTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void serviceExposesFinalSourceButNoParallelImperativeLifecycle() {
        for (java.lang.reflect.Method method : PhoenixDriveAssistService.class.getDeclaredMethods()) {
            assertFalse("service must not expose an update heartbeat", method.getName().equals("update"));
            assertFalse("service must not expose a reset lifecycle", method.getName().equals("reset"));
        }
    }

    @Test
    public void unavailablePoseAssistsLeaveManualDriveUnchanged() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingAimOverlay aimOverlay = new RecordingAimOverlay();
        PhoenixDriveAssistService service = createService(
                manualClock.clock(), false, aimOverlay);

        DriveSignal output = service.driveSource().get(manualClock.clock());
        DriveSignal repeated = service.driveSource().get(manualClock.clock());

        PhoenixDriveAssistService.Status status = service.status();
        assertFalse(status.poseAssistsAvailable);
        assertTrue(status.autoAimRequested);
        assertFalse(status.shootBraceEligible);
        assertFalse(status.shootBraceEnabled);
        assertEquals(0, aimOverlay.getCalls);
        assertSignalEquals(new DriveSignal(0.40, -0.20, 0.30), output);
        assertSignalEquals(output, repeated);
    }

    @Test
    public void availablePoseAssistsCanBraceAndOverrideOnlyOmega() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingAimOverlay aimOverlay = new RecordingAimOverlay();
        PhoenixDriveAssistService service = createService(
                manualClock.clock(), true, aimOverlay);

        DriveSignal output = service.driveSource().get(manualClock.clock());
        DriveSignal repeated = service.driveSource().get(manualClock.clock());

        PhoenixDriveAssistService.Status status = service.status();
        assertTrue(status.poseAssistsAvailable);
        assertTrue(status.autoAimRequested);
        assertTrue(status.shootBraceEligible);
        assertTrue(status.shootBraceEnabled);
        assertEquals(1, aimOverlay.getCalls);
        assertSignalEquals(new DriveSignal(0.0, 0.0, 0.90), output);
        assertSignalEquals(output, repeated);
    }

    @Test
    public void laterLocalizationLossDropsBraceButPreservesManualRobotCentricDrive() {
        ManualLoopClock time = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(new PoseEstimate(
                new Pose3d(9.0, -4.0, 0.0, 0.5, 0.0, 0.0),
                true,
                1.0,
                time.clock().nowTimestamp()
        ));
        DriveSignal manualSignal = new DriveSignal(0.40, -0.20, 0.30);
        PhoenixDriveAssistService service = new PhoenixDriveAssistService(
                PhoenixDriveAssistService.Config.defaults(),
                clock -> manualSignal,
                ScalarSource.constant(0.0),
                Source.constant(scoringStatus(true)),
                BooleanSource.constant(false),
                true,
                estimator,
                clock -> DriveOverlayOutput.zero()
        );

        DriveSignal braced = service.driveSource().get(time.clock());
        assertSignalEquals(new DriveSignal(0.0, 0.0, 0.30), braced);

        time.nextCycle(0.02);
        estimator.estimate = PoseEstimate.noPose(time.clock().nowTimestamp());
        DriveSignal afterFault = service.driveSource().get(time.clock());

        assertSignalEquals(manualSignal, afterFault);
    }

    @Test
    public void finalDriveReadPublishesOneCurrentScoringSnapshotPerCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingAimOverlay aimOverlay = new RecordingAimOverlay();
        final int[] scoringReads = {0};
        final PhoenixCapabilities.ScoringStatus[] currentScoring = {scoringStatus(true)};
        Source<PhoenixCapabilities.ScoringStatus> scoringSource = Source.of(clock -> {
            scoringReads[0]++;
            return currentScoring[0];
        });
        PhoenixDriveAssistService service = createService(
                manualClock.clock(), true, aimOverlay, scoringSource);

        service.driveSource().get(manualClock.clock());
        service.driveSource().get(manualClock.clock());

        assertEquals(1, scoringReads[0]);
        assertTrue(service.status().shootBraceEnabled);

        currentScoring[0] = scoringStatus(false);
        manualClock.nextCycle(0.02);
        service.driveSource().get(manualClock.clock());

        assertEquals(2, scoringReads[0]);
        assertFalse(service.status().shootBraceEligible);
        assertFalse(service.status().shootBraceEnabled);
    }

    @Test
    public void ownerRetainsItsValidatedScalarSnapshot() {
        ManualLoopClock time = new ManualLoopClock();
        PhoenixDriveAssistService.Config config = PhoenixDriveAssistService.Config.defaults();
        PhoenixDriveAssistService service = createService(
                time.clock(),
                true,
                new RecordingAimOverlay(),
                Source.constant(scoringStatus(true)),
                config
        );

        config.shootBraceEnterTranslateMagnitude = Double.NaN;
        config.shootBraceExitTranslateMagnitude = Double.NaN;
        config.shootBraceTranslateKp = Double.NaN;
        config.shootBraceMaxTranslateCmd = Double.NaN;

        DriveSignal output = service.driveSource().get(time.clock());
        assertTrue(Double.isFinite(output.axial));
        assertTrue(Double.isFinite(output.lateral));
        assertTrue(Double.isFinite(output.omega));
        assertTrue(service.status().shootBraceEnabled);
    }

    @Test
    public void configRejectsEachInvalidDomainAndCrossFieldOrder() {
        PhoenixDriveAssistService.Config enter = PhoenixDriveAssistService.Config.defaults();
        enter.shootBraceEnterTranslateMagnitude = -0.01;
        expectInvalidConfig(enter, "shootBraceEnterTranslateMagnitude");

        PhoenixDriveAssistService.Config exit = PhoenixDriveAssistService.Config.defaults();
        exit.shootBraceExitTranslateMagnitude = Double.NaN;
        expectInvalidConfig(exit, "shootBraceExitTranslateMagnitude");

        PhoenixDriveAssistService.Config ordering = PhoenixDriveAssistService.Config.defaults();
        ordering.shootBraceEnterTranslateMagnitude = 0.8;
        ordering.shootBraceExitTranslateMagnitude = 0.2;
        ordering.shootBraceTranslateKp = Double.NaN;
        expectInvalidConfig(ordering, "shootBraceEnterTranslateMagnitude");

        PhoenixDriveAssistService.Config gain = PhoenixDriveAssistService.Config.defaults();
        gain.shootBraceTranslateKp = -0.01;
        expectInvalidConfig(gain, "shootBraceTranslateKp");

        PhoenixDriveAssistService.Config maximum = PhoenixDriveAssistService.Config.defaults();
        maximum.shootBraceMaxTranslateCmd = Double.POSITIVE_INFINITY;
        expectInvalidConfig(maximum, "shootBraceMaxTranslateCmd");
    }

    @Test
    public void configAcceptsInclusiveDocumentedBoundaries() {
        ManualLoopClock time = new ManualLoopClock();
        PhoenixDriveAssistService.Config config = PhoenixDriveAssistService.Config.defaults();
        config.shootBraceEnterTranslateMagnitude = 0.0;
        config.shootBraceExitTranslateMagnitude = 1.0;
        config.shootBraceTranslateKp = 0.0;
        config.shootBraceMaxTranslateCmd = 0.0;

        PhoenixDriveAssistService service = createService(
                time.clock(),
                true,
                new RecordingAimOverlay(),
                Source.constant(scoringStatus(true)),
                config
        );

        assertTrue(service != null);
    }

    private static PhoenixDriveAssistService createService(
            LoopClock clock,
            boolean poseAssistsAvailable,
            DriveOverlay aimOverlay
    ) {
        return createService(
                clock,
                poseAssistsAvailable,
                aimOverlay,
                Source.constant(scoringStatus(true))
        );
    }

    private static PhoenixDriveAssistService createService(
            LoopClock clock,
            boolean poseAssistsAvailable,
            DriveOverlay aimOverlay,
            Source<PhoenixCapabilities.ScoringStatus> scoringStatusSource
    ) {
        return createService(
                clock,
                poseAssistsAvailable,
                aimOverlay,
                scoringStatusSource,
                PhoenixDriveAssistService.Config.defaults()
        );
    }

    private static PhoenixDriveAssistService createService(
            LoopClock clock,
            boolean poseAssistsAvailable,
            DriveOverlay aimOverlay,
            Source<PhoenixCapabilities.ScoringStatus> scoringStatusSource,
            PhoenixDriveAssistService.Config config
    ) {
        DriveSource manualDrive = new DriveSource() {
            @Override
            public DriveSignal get(LoopClock clock) {
                return new DriveSignal(0.40, -0.20, 0.30);
            }
        };
        AbsolutePoseEstimator poseEstimator = new AbsolutePoseEstimator() {
            private PoseEstimate estimate = new PoseEstimate(
                    new Pose3d(9.0, -4.0, 0.0, 0.5, 0.0, 0.0),
                    true,
                    1.0,
                    clock.nowTimestamp()
            );

            @Override
            public void update(LoopClock clock) {
                // The service consumes the already-updated shared localization snapshot.
            }

            @Override
            public PoseEstimate getEstimate() {
                if (!poseAssistsAvailable) {
                    throw new AssertionError(
                            "Unavailable shoot-brace must not sample localization"
                    );
                }
                return estimate;
            }
        };

        return new PhoenixDriveAssistService(
                config,
                manualDrive,
                ScalarSource.constant(0.0),
                scoringStatusSource,
                BooleanSource.constant(true),
                poseAssistsAvailable,
                poseEstimator,
                aimOverlay
        );
    }

    private static void expectInvalidConfig(
            PhoenixDriveAssistService.Config config,
            String expectedField
    ) {
        try {
            createService(
                    new ManualLoopClock().clock(),
                    true,
                    new RecordingAimOverlay(),
                    Source.constant(scoringStatus(true)),
                    config
            );
            fail("Expected invalid drive-assist config field " + expectedField);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(
                    "PhoenixDriveAssistService.Config." + expectedField
            ));
        }
    }

    private static PhoenixCapabilities.ScoringStatus scoringStatus(boolean shootActive) {
        return new PhoenixCapabilities.ScoringStatus(
                false,
                false,
                shootActive,
                shootActive,
                shootActive,
                0,
                null,
                shootActive,
                false,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                false,
                false,
                0,
                false,
                0.0
        );
    }

    private static void assertSignalEquals(DriveSignal expected, DriveSignal actual) {
        assertEquals(expected.axial, actual.axial, EPSILON);
        assertEquals(expected.lateral, actual.lateral, EPSILON);
        assertEquals(expected.omega, actual.omega, EPSILON);
    }

    private static final class RecordingAimOverlay implements DriveOverlay {
        int getCalls;

        @Override
        public DriveOverlayOutput get(LoopClock clock) {
            getCalls++;
            return new DriveOverlayOutput(
                    new DriveSignal(-1.0, -1.0, 0.90),
                    DriveOverlayMask.ALL
            );
        }
    }

    private static final class MutablePoseEstimator implements AbsolutePoseEstimator {
        private PoseEstimate estimate;

        private MutablePoseEstimator(PoseEstimate estimate) {
            this.estimate = estimate;
        }

        @Override
        public void update(LoopClock clock) {
            // Test controls the immutable published estimate directly.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
