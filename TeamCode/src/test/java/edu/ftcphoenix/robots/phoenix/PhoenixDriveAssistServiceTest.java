package edu.ftcphoenix.robots.phoenix;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

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
    public void finalDriveReadPublishesOneCurrentScoringSnapshotPerCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingAimOverlay aimOverlay = new RecordingAimOverlay();
        final int[] scoringReads = {0};
        final ScoringPath.Status[] currentScoring = {scoringStatus(true)};
        Source<ScoringPath.Status> scoringSource = Source.of(clock -> {
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
            Source<ScoringPath.Status> scoringStatusSource
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
                new PhoenixProfile.DriveAssistConfig(),
                manualDrive,
                ScalarSource.constant(0.0),
                scoringStatusSource,
                BooleanSource.constant(true),
                poseAssistsAvailable,
                poseEstimator,
                aimOverlay
        );
    }

    private static ScoringPath.Status scoringStatus(boolean shootActive) {
        return new ScoringPath.Status(
                false,
                false,
                shootActive,
                shootActive,
                shootActive,
                0,
                null,
                shootActive,
                false,
                null,
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
}
