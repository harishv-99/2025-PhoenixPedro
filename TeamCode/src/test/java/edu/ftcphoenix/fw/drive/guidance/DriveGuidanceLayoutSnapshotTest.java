package edu.ftcphoenix.fw.drive.guidance;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.spatial.References;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;

/** Verifies that Drive Guidance retains one shared immutable fixed-layout snapshot. */
public final class DriveGuidanceLayoutSnapshotTest {

    @Test
    public void buildSharesOneSnapshotAcrossResolveAndSpatialSpec() {
        Pose3d originalPose = new Pose3d(12.0, 24.0, 6.0, Math.PI, 0.0, 0.0);
        RecordingMutableLayout authored = new RecordingMutableLayout();
        authored.poses.put(3, originalPose);

        DriveGuidancePlan plan = DriveGuidance.plan()
                .faceTo()
                    .point(References.relativeToTagPoint(3, 1.0, -2.0))
                .solveWith()
                    .localizationOnly()
                    .localization(new NoPoseEstimator())
                    .fixedAprilTagLayout(authored)
                    .doneLocalizationOnly()
                .build();

        TagLayout retained = plan.spec.resolveWith.fixedAprilTagLayout;
        assertNotSame(authored, retained);
        assertSame(retained, plan.spec.spatialQuerySpec.fixedAprilTagLayout);
        assertEquals(1, authored.idsCalls);
        assertEquals(1, authored.poseCalls);

        authored.poses.put(3, Pose3d.zero());
        authored.poses.put(4, Pose3d.zero());

        assertSame(originalPose, retained.requireFieldToTagPose(3));
        assertFalse(retained.has(4));
    }

    private static final class RecordingMutableLayout implements TagLayout {
        private final Map<Integer, Pose3d> poses = new LinkedHashMap<Integer, Pose3d>();
        private int idsCalls;
        private int poseCalls;

        @Override
        public Pose3d getFieldToTagPose(int id) {
            poseCalls++;
            return poses.get(id);
        }

        @Override
        public Set<Integer> ids() {
            idsCalls++;
            return poses.keySet();
        }
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            // No runtime behavior is needed for this construction-retention test.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
