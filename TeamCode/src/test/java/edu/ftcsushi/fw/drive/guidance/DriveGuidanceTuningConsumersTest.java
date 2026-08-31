package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/** Proves that every public guidance consumer uses its supplied immutable tuning value. */
public final class DriveGuidanceTuningConsumersTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void poseLockUsesSuppliedTranslationTuning() {
        ManualLoopClock time = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator();
        estimator.setPose(Pose3d.zero(), time.clock().nowTimestamp());
        DriveGuidancePlan.Tuning tuning = customTuning();
        DriveOverlay poseLock = DriveGuidance.poseLock(estimator, tuning);

        poseLock.onEnable(time.clock());
        time.nextCycle(0.02);
        estimator.setPose(
                new Pose3d(-2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                time.clock().nowTimestamp()
        );

        DriveOverlayOutput output = poseLock.get(time.clock());
        assertSignal(output.signal, 0.4, 0.0, 0.0);
    }

    @Test
    public void allGoToPoseHelpersUseSuppliedTuningForTheirFirstCommand() {
        ManualLoopClock time = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator();
        estimator.setPose(Pose3d.zero(), time.clock().nowTimestamp());
        DriveGuidancePlan.Tuning tuning = customTuning();
        SimpleTagLayout layout = new SimpleTagLayout().addPose(7, Pose3d.zero());

        assertFirstCommand(
                time,
                sink -> GoToPoseTasks.goToPoseFieldRelative(
                        estimator,
                        sink,
                        new Pose2d(2.0, 0.0, 0.4),
                        tuning,
                        null
                ),
                0.4,
                0.0,
                0.2
        );
        assertFirstCommand(
                time,
                sink -> GoToPoseTasks.goToPoseTagRelative(
                        estimator,
                        sink,
                        layout,
                        7,
                        2.0,
                        0.0,
                        0.4 - Math.PI,
                        tuning,
                        null
                ),
                0.4,
                0.0,
                0.2
        );
        assertFirstCommand(
                time,
                sink -> GoToPoseTasks.holdPositionAndAimFieldHeading(
                        estimator,
                        sink,
                        0.4,
                        tuning,
                        null
                ),
                0.0,
                0.0,
                0.2
        );
        assertFirstCommand(
                time,
                sink -> GoToPoseTasks.aimOnlyFieldHeading(
                        estimator,
                        sink,
                        0.4,
                        tuning,
                        null
                ),
                0.0,
                0.0,
                0.2
        );
    }

    private static DriveGuidancePlan.Tuning customTuning() {
        return DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(0.2)
                .withMaxTranslateCmd(1.0)
                .withAimKp(0.5)
                .withMaxOmegaCmd(1.0)
                .withMinOmegaCmd(0.0)
                .withAimDeadbandRad(0.0);
    }

    private static void assertFirstCommand(ManualLoopClock time,
                                           TaskFactory factory,
                                           double axial,
                                           double lateral,
                                           double omega) {
        RecordingDriveSink sink = new RecordingDriveSink();
        Task task = factory.create(sink);

        task.start(time.clock());
        task.update(time.clock());

        assertNotNull(sink.lastSignal);
        assertSignal(sink.lastSignal, axial, lateral, omega);
    }

    private static void assertSignal(DriveSignal actual,
                                     double axial,
                                     double lateral,
                                     double omega) {
        assertEquals(axial, actual.axial, EPSILON);
        assertEquals(lateral, actual.lateral, EPSILON);
        assertEquals(omega, actual.omega, EPSILON);
    }

    private interface TaskFactory {
        Task create(DriveCommandSink sink);
    }

    private static final class MutablePoseEstimator implements AbsolutePoseEstimator {
        private PoseEstimate estimate;

        void setPose(Pose3d pose, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(pose, true, 1.0, timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            // The test owns the fixed snapshots.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private DriveSignal lastSignal;

        @Override
        public void drive(DriveSignal signal) {
            lastSignal = signal;
        }

        @Override
        public void stop() {
            // Stop calls do not erase the last submitted command under test.
        }
    }
}
