package edu.ftcphoenix.fw.field;

import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused contracts for authored, borrowed, and retained fixed-tag layouts. */
public final class TagLayoutsSnapshotTest {

    @Test
    public void snapshotRejectsNullLayoutAndInvalidIdSets() {
        expectThrows(NullPointerException.class, new Action() {
            @Override
            public void run() {
                TagLayouts.snapshot(null);
            }
        });

        expectIllegalArgument("layout.ids() must not return null", new Action() {
            @Override
            public void run() {
                TagLayouts.snapshot(new RecordingLayout(null, Collections.<Integer, Pose3d>emptyMap()));
            }
        });

        expectIllegalArgument("must not contain null", new Action() {
            @Override
            public void run() {
                TagLayouts.snapshot(new RecordingLayout(
                        new LinkedHashSet<Integer>(Arrays.asList(1, null)),
                        Collections.singletonMap(1, Pose3d.zero())
                ));
            }
        });

        expectIllegalArgument("must be non-negative", new Action() {
            @Override
            public void run() {
                TagLayouts.snapshot(new RecordingLayout(
                        Collections.singleton(-1),
                        Collections.singletonMap(-1, Pose3d.zero())
                ));
            }
        });
    }

    @Test
    public void snapshotRejectsMissingAndEveryNonFinitePoseComponent() {
        expectIllegalArgument("pose for id=7 must not be null", new Action() {
            @Override
            public void run() {
                TagLayouts.snapshot(new RecordingLayout(
                        Collections.singleton(7),
                        Collections.<Integer, Pose3d>emptyMap()
                ));
            }
        });

        final String[] fields = {
                "xInches", "yInches", "zInches", "yawRad", "pitchRad", "rollRad"
        };
        final double[] nonFinite = {
                Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
        };
        for (int nonFiniteIndex = 0; nonFiniteIndex < fields.length; nonFiniteIndex++) {
            for (double invalid : nonFinite) {
                final int index = nonFiniteIndex;
                final double invalidValue = invalid;
                expectIllegalArgument("." + fields[index] + " must be finite", new Action() {
                    @Override
                    public void run() {
                        TagLayouts.snapshot(layoutWithPose(poseWith(index, invalidValue)));
                    }
                });
            }
        }
    }

    @Test
    public void snapshotEnumeratesOnceReadsEachPoseOnceAndIsIdempotent() {
        LinkedHashMap<Integer, Pose3d> poses = new LinkedHashMap<Integer, Pose3d>();
        poses.put(4, new Pose3d(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        poses.put(9, new Pose3d(-1.0, -2.0, -3.0, -4.0, -5.0, -6.0));
        RecordingLayout source = new RecordingLayout(
                new LinkedHashSet<Integer>(poses.keySet()),
                poses
        );

        TagLayout snapshot = TagLayouts.snapshot(source);

        assertEquals(1, source.idsCalls);
        assertEquals(1, source.poseCallsFor(4));
        assertEquals(1, source.poseCallsFor(9));
        assertEquals(new LinkedHashSet<Integer>(Arrays.asList(4, 9)), snapshot.ids());
        assertSame(poses.get(4), snapshot.getFieldToTagPose(4));
        assertSame(snapshot, TagLayouts.snapshot(snapshot));
        assertEquals(1, source.idsCalls);
        assertEquals(1, source.poseCallsFor(4));
        expectThrows(UnsupportedOperationException.class, new Action() {
            @Override
            public void run() {
                snapshot.ids().add(12);
            }
        });
    }

    @Test
    public void emptyAndCustomLayoutsProduceIndependentImmutableSnapshots() {
        RecordingLayout empty = new RecordingLayout(
                Collections.<Integer>emptySet(),
                Collections.<Integer, Pose3d>emptyMap()
        );
        TagLayout emptySnapshot = TagLayouts.snapshot(empty);
        assertTrue(emptySnapshot.ids().isEmpty());
        assertEquals(1, empty.idsCalls);

        SimpleTagLayout authored = new SimpleTagLayout()
                .addPose(1, new Pose3d(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        TagLayout retained = TagLayouts.snapshot(authored);
        Pose3d retainedPose = retained.requireFieldToTagPose(1);

        authored.addPose(1, new Pose3d(10.0, 20.0, 30.0, 40.0, 50.0, 60.0));
        authored.addPose(2, Pose3d.zero());
        authored.remove(1);

        assertNotSame(authored, retained);
        assertEquals(retainedPose, retained.requireFieldToTagPose(1));
        assertFalse(retained.has(2));
        assertEquals(Collections.singleton(1), retained.ids());
    }

    @Test
    public void simpleLayoutRejectsEveryNonFinitePoseSlotAtomically() {
        final SimpleTagLayout layout = new SimpleTagLayout();
        final Pose3d original = new Pose3d(-0.0, 2.0, 3.0, 9.0 * Math.PI, -8.0, 7.0);
        layout.addPose(3, original);

        final String[] fields = {
                "xInches", "yInches", "zInches", "yawRad", "pitchRad", "rollRad"
        };
        final double[] nonFinite = {
                Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
        };
        for (int field = 0; field < fields.length; field++) {
            for (double invalid : nonFinite) {
                final int index = field;
                final double invalidValue = invalid;
                expectIllegalArgument(fields[index] + " must be finite", new Action() {
                    @Override
                    public void run() {
                        layout.addPose(3, poseWith(index, invalidValue));
                    }
                });
                assertSame(original, layout.requireFieldToTagPose(3));

                expectIllegalArgument(fields[index] + " must be finite", new Action() {
                    @Override
                    public void run() {
                        double[] values = poseValuesWith(index, invalidValue);
                        layout.add(3, values[0], values[1], values[2],
                                values[3], values[4], values[5]);
                    }
                });
                assertSame(original, layout.requireFieldToTagPose(3));
            }
        }

        assertEquals(
                Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(layout.requireFieldToTagPose(3).xInches)
        );
        assertEquals(9.0 * Math.PI, layout.requireFieldToTagPose(3).yawRad, 0.0);

        expectIllegalArgument("must be non-negative", new Action() {
            @Override
            public void run() {
                layout.addPose(-2, Pose3d.zero());
            }
        });
        expectIllegalArgument("must not be null", new Action() {
            @Override
            public void run() {
                layout.addPose(4, null);
            }
        });
        assertEquals(1, layout.size());
    }

    @Test
    public void subsetViewsStayLiveSubsetOrSameAvoidsWrapperAndSnapshotFreezesFacts() {
        Pose3d firstPose = new Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        Pose3d secondPose = new Pose3d(4.0, 5.0, 6.0, 0.4, 0.5, 0.6);
        SimpleTagLayout base = new SimpleTagLayout()
                .addPose(1, firstPose)
                .addPose(2, secondPose);
        LinkedHashSet<Integer> allIds = new LinkedHashSet<Integer>(Arrays.asList(1, 2));

        assertSame(base, TagLayouts.subsetOrSame(base, allIds, "all"));
        TagLayout properSubset = TagLayouts.subsetOrSame(
                base,
                Collections.singleton(1),
                "one"
        );
        assertNotSame(base, properSubset);
        assertEquals(Collections.singleton(1), properSubset.ids());

        TagLayout liveView = TagLayouts.subset(base, allIds, "live");
        Pose3d replacement = new Pose3d(10.0, 20.0, 30.0, -0.1, -0.2, -0.3);
        base.addPose(1, replacement);
        assertSame(replacement, liveView.requireFieldToTagPose(1));

        TagLayout snapshot = TagLayouts.snapshot(liveView);
        Pose3d later = new Pose3d(11.0, 21.0, 31.0, -0.4, -0.5, -0.6);
        base.addPose(1, later);
        base.remove(2);
        base.addPose(3, Pose3d.zero());

        assertSame(later, liveView.requireFieldToTagPose(1));
        assertFalse(liveView.has(2));
        assertFalse(liveView.has(3));
        assertSame(replacement, snapshot.requireFieldToTagPose(1));
        assertSame(secondPose, snapshot.requireFieldToTagPose(2));
        assertFalse(snapshot.has(3));
        assertEquals(allIds, snapshot.ids());
    }

    @Test
    public void subsetConstructionStaysBehindFactoriesAndRejectsInvalidMembership() {
        for (Constructor<?> constructor : SubsetTagLayout.class.getDeclaredConstructors()) {
            assertFalse(Modifier.isPublic(constructor.getModifiers()));
        }

        final SimpleTagLayout base = new SimpleTagLayout().addPose(5, Pose3d.zero());
        expectThrows(NullPointerException.class, new Action() {
            @Override
            public void run() {
                TagLayouts.subset(null, Collections.singleton(5));
            }
        });
        expectThrows(NullPointerException.class, new Action() {
            @Override
            public void run() {
                TagLayouts.subset(base, null);
            }
        });
        expectIllegalArgument("must not contain null", new Action() {
            @Override
            public void run() {
                TagLayouts.subset(base, Collections.<Integer>singleton(null));
            }
        });
        expectIllegalArgument("must be non-negative", new Action() {
            @Override
            public void run() {
                TagLayouts.subset(base, Collections.singleton(-1));
            }
        });
        expectIllegalArgument("does not contain", new Action() {
            @Override
            public void run() {
                TagLayouts.subset(base, Collections.singleton(6));
            }
        });
    }

    private static RecordingLayout layoutWithPose(Pose3d pose) {
        return new RecordingLayout(
                Collections.singleton(7),
                Collections.singletonMap(7, pose)
        );
    }

    private static Pose3d poseWith(int field, double value) {
        double[] values = poseValuesWith(field, value);
        return new Pose3d(
                values[0], values[1], values[2], values[3], values[4], values[5]
        );
    }

    private static double[] poseValuesWith(int field, double value) {
        double[] values = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
        values[field] = value;
        return values;
    }

    private static void expectIllegalArgument(String messagePart, Action action) {
        IllegalArgumentException failure = expectThrows(IllegalArgumentException.class, action);
        assertTrue(
                "Expected message to contain '" + messagePart + "' but was '" + failure.getMessage() + "'",
                failure.getMessage() != null && failure.getMessage().contains(messagePart)
        );
    }

    private static <T extends Throwable> T expectThrows(Class<T> type, Action action) {
        try {
            action.run();
            fail("Expected " + type.getSimpleName());
            return null;
        } catch (Throwable failure) {
            if (!type.isInstance(failure)) {
                throw new AssertionError(
                        "Expected " + type.getSimpleName() + " but caught "
                                + failure.getClass().getSimpleName(),
                        failure
                );
            }
            return type.cast(failure);
        }
    }

    private interface Action {
        void run();
    }

    private static final class RecordingLayout implements TagLayout {
        private final Set<Integer> ids;
        private final Map<Integer, Pose3d> poses;
        private final Map<Integer, Integer> poseCalls = new LinkedHashMap<Integer, Integer>();
        private int idsCalls;

        RecordingLayout(Set<Integer> ids, Map<Integer, Pose3d> poses) {
            this.ids = ids;
            this.poses = poses;
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            Integer calls = poseCalls.get(id);
            poseCalls.put(id, calls != null ? calls + 1 : 1);
            return poses.get(id);
        }

        @Override
        public Set<Integer> ids() {
            idsCalls++;
            return ids;
        }

        int poseCallsFor(int id) {
            Integer calls = poseCalls.get(id);
            return calls != null ? calls : 0;
        }
    }
}
