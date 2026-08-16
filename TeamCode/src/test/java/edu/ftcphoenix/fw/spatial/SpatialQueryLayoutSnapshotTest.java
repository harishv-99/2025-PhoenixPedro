package edu.ftcphoenix.fw.spatial;

import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.field.TagLayouts;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;

/** Verifies immutable field-layout retention and the one supported query construction path. */
public final class SpatialQueryLayoutSnapshotTest {

    @Test
    public void specSnapshotsAuthoredLayoutAndRuntimeRetainsSpec() {
        Pose3d originalPose = new Pose3d(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        SimpleTagLayout authored = new SimpleTagLayout().addPose(4, originalPose);

        SpatialQuerySpec spec = SpatialQuerySpec.builder()
                .translateTo(SpatialTargets.fieldPoint(12.0, 18.0))
                .fixedAprilTagLayout(authored)
                .solveWith(SpatialSolveSet.builder().add(new NoopLane()).build())
                .build();
        SpatialQuery runtime = SpatialQuery.from(spec);

        authored.addPose(4, Pose3d.zero());
        authored.addPose(5, Pose3d.zero());

        assertNotSame(authored, spec.fixedAprilTagLayout);
        assertSame(originalPose, spec.fixedAprilTagLayout.requireFieldToTagPose(4));
        assertFalse(spec.fixedAprilTagLayout.has(5));
        assertSame(spec, runtime.spec());
    }

    @Test
    public void specReusesFrameworkSnapshotAndNoDeclaredConstructorIsPublic() {
        TagLayout retained = TagLayouts.snapshot(
                new SimpleTagLayout().addPose(8, Pose3d.zero())
        );
        SpatialQuerySpec spec = SpatialQuerySpec.builder()
                .faceTo(SpatialTargets.fieldHeading(0.0))
                .fixedAprilTagLayout(retained)
                .solveWith(SpatialSolveSet.builder().add(new NoopLane()).build())
                .build();

        assertSame(retained, spec.fixedAprilTagLayout);

        assertNoPublicDeclaredConstructors(SpatialQuerySpec.class);
        assertNoPublicDeclaredConstructors(SpatialQuery.class);
        assertSame(spec, SpatialQuery.from(spec).spec());
    }

    private static void assertNoPublicDeclaredConstructors(Class<?> type) {
        for (Constructor<?> constructor : type.getDeclaredConstructors()) {
            assertFalse(type.getName(), Modifier.isPublic(constructor.getModifiers()));
        }
    }

    private static final class NoopLane implements SpatialSolveLane {
        @Override
        public SpatialLaneResult solve(SpatialSolveRequest request) {
            return SpatialLaneResult.none();
        }
    }
}
