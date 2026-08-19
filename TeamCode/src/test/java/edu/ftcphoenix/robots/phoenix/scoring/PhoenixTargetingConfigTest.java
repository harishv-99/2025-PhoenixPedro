package edu.ftcphoenix.robots.phoenix.scoring;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** API, capture, and active-slice validation locks for {@link PhoenixTargeting}. */
public final class PhoenixTargetingConfigTest {

    @Test
    public void ownerConfigAndRawValueSurfacesStayExact() throws Exception {
        assertEquals(13, PhoenixTargeting.Config.class.getFields().length);
        Constructor<?>[] configConstructors =
                PhoenixTargeting.Config.class.getDeclaredConstructors();
        assertEquals(1, configConstructors.length);
        assertTrue(Modifier.isPrivate(configConstructors[0].getModifiers()));

        Set<String> publicMethods = new LinkedHashSet<String>();
        int publicMethodCount = 0;
        for (Method method : PhoenixTargeting.Config.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethodCount++;
                publicMethods.add(method.getName());
            }
        }
        assertEquals(2, publicMethodCount);
        assertEquals(
                new LinkedHashSet<String>(Arrays.asList("defaults", "scoringTagIdFor")),
                publicMethods
        );

        assertEquals(2, PhoenixTargeting.ScoringTarget.class.getFields().length);
        assertEquals(0, PhoenixTargeting.ScoringTarget.class.getDeclaredMethods().length);
        Constructor<PhoenixTargeting.ScoringTarget> targetConstructor =
                PhoenixTargeting.ScoringTarget.class.getConstructor(
                        String.class,
                        PhoenixTargeting.AimOffset.class
                );
        assertTrue(Modifier.isPublic(targetConstructor.getModifiers()));
        try {
            PhoenixTargeting.ScoringTarget.class.getField("tagId");
            fail("ScoringTarget must not duplicate its map key as tagId");
        } catch (NoSuchFieldException expected) {
            // Exact desired surface.
        }

        assertEquals(2, PhoenixTargeting.AimOffset.class.getFields().length);
        assertEquals(0, PhoenixTargeting.AimOffset.class.getDeclaredMethods().length);
        assertTrue(Modifier.isPublic(
                PhoenixTargeting.AimOffset.class
                        .getConstructor(double.class, double.class)
                        .getModifiers()
        ));

        PhoenixTargeting.ScoringTarget raw =
                new PhoenixTargeting.ScoringTarget(null, null);
        assertNull(raw.label);
        assertNull(raw.aimOffset);
    }

    @Test
    public void defaultsPreserveValuesAndReturnIndependentMutableGraphs() {
        PhoenixTargeting.Config first = PhoenixTargeting.Config.defaults();
        PhoenixTargeting.Config second = PhoenixTargeting.Config.defaults();

        assertEquals(24, first.redAllianceScoringTagId);
        assertEquals(20, first.blueAllianceScoringTagId);
        assertEquals(20, first.scoringTagIdFor(PhoenixAlliance.BLUE));
        assertEquals(24, first.scoringTagIdFor(PhoenixAlliance.RED));
        assertEquals(Arrays.asList(20, 24),
                Arrays.asList(first.scoringTargets.keySet().toArray(new Integer[0])));
        assertEquals("Blue scoring target", first.scoringTargets.get(20).label);
        assertRawDouble(0.0, first.scoringTargets.get(20).aimOffset.forwardInches);
        assertRawDouble(0.0, first.scoringTargets.get(20).aimOffset.leftInches);
        assertEquals("Red scoring target", first.scoringTargets.get(24).label);
        assertRawDouble(0.0, first.scoringTargets.get(24).aimOffset.forwardInches);
        assertRawDouble(0.0, first.scoringTargets.get(24).aimOffset.leftInches);
        assertEquals(0.25, first.aimToleranceDeg, 0.0);
        assertEquals(1.5, first.aimKp, 0.0);
        assertEquals(0.80, first.aimMaxOmegaCmd, 0.0);
        assertEquals(0.50, first.aimReadyToleranceDeg, 0.0);
        assertEquals(0.05, first.aimReadyDebounceSec, 0.0);
        assertEquals(0.05, first.aimMinOmegaCmd, 0.0);
        assertEquals(0.50, first.selectionMaxAgeSec, 0.0);
        assertEquals(0.20, first.selectionReacquireSec, 0.0);
        assertRawDouble(0.0, first.defaultAimOffset.forwardInches);
        assertRawDouble(0.0, first.defaultAimOffset.leftInches);

        assertNotSame(first, second);
        assertNotSame(first.scoringTargets, second.scoringTargets);
        assertNotSame(first.scoringTargets.get(20), second.scoringTargets.get(20));
        assertNotSame(first.scoringTargets.get(20).aimOffset,
                second.scoringTargets.get(20).aimOffset);
        assertNotSame(first.defaultAimOffset, second.defaultAimOffset);
        assertSame("the immutable reviewed calibration table may be shared",
                first.shotVelocityTable, second.shotVelocityTable);

        assertEquals(
                "InterpolatingTable1D{"
                        + "(28.06, 1505.6), (36.52, 1427.4), (42.3, 1424.35), "
                        + "(50.3, 1450.0), (56.5, 1461.0), (62.9, 1538.0), "
                        + "(65.8, 1535.7), (70.0, 1530.0), (74.2, 1575.0), "
                        + "(79.5, 1600.0), (83.4, 1625.0), (93.6, 1700.0), "
                        + "(96.6, 1700.0), (103.2, 1775.0), (104.7, 1800.0), "
                        + "(109.2, 1800.0), (112.0, 1818.0), (115.0, 1825.0), "
                        + "(120.0, 1850.0), (130.0, 1875.0)}",
                first.shotVelocityTable.toString()
        );
    }

    @Test
    public void commonValidationUsesDeclaredOrderAndDefensiveCapture() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixTargeting.Config invalid = PhoenixTargeting.Config.defaults();
        invalid.aimToleranceDeg = Double.NaN;
        invalid.aimKp = Double.NaN;
        assertConstructionFailure(profile, invalid, "aimToleranceDeg");

        PhoenixTargeting.Config crossField = PhoenixTargeting.Config.defaults();
        crossField.aimKp = 0.0;
        crossField.aimMinOmegaCmd = 0.01;
        assertConstructionFailure(profile, crossField, "aimKp");

        PhoenixTargeting.Config captured = PhoenixTargeting.Config.defaults();
        int selectedId = captured.redAllianceScoringTagId;
        CountingEmptyAprilTagSensor sensor = new CountingEmptyAprilTagSensor();
        PhoenixTargeting targeting = targeting(
                profile,
                captured,
                sensor,
                profile.fixedAprilTagLayout,
                Source.constant(Collections.singleton(selectedId))
        );
        captured.scoringTargets.get(selectedId).label = null;
        captured.scoringTargets.get(selectedId).aimOffset.forwardInches = Double.NaN;
        captured.scoringTargets = null;
        captured.defaultAimOffset = null;
        captured.aimKp = Double.NaN;

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        targeting.update(clock);
        assertEquals("selection and guidance both sample the shared sensor", 2,
                sensor.sampleCalls);
        assertEquals("the AprilTag sensor acquires at most once per loop cycle", 1,
                sensor.acquisitions);
    }

    @Test
    public void commonDomainsRejectEveryInvalidFamilyAndAcceptInclusiveBoundaries() {
        PhoenixProfile profile = PhoenixProfile.current();

        PhoenixTargeting.Config config = PhoenixTargeting.Config.defaults();
        config.aimToleranceDeg = Math.nextUp(180.0);
        assertConstructionFailure(profile, config, "aimToleranceDeg");

        config = PhoenixTargeting.Config.defaults();
        config.aimKp = -1.0;
        assertConstructionFailure(profile, config, "aimKp");

        config = PhoenixTargeting.Config.defaults();
        config.aimMaxOmegaCmd = Math.nextUp(1.0);
        assertConstructionFailure(profile, config, "aimMaxOmegaCmd");

        config = PhoenixTargeting.Config.defaults();
        config.aimReadyToleranceDeg = Math.nextDown(config.aimToleranceDeg);
        assertConstructionFailure(profile, config, "aimReadyToleranceDeg");

        config = PhoenixTargeting.Config.defaults();
        config.aimReadyDebounceSec = -1.0;
        assertConstructionFailure(profile, config, "aimReadyDebounceSec");

        config = PhoenixTargeting.Config.defaults();
        config.aimMinOmegaCmd = Math.nextUp(config.aimMaxOmegaCmd);
        assertConstructionFailure(profile, config, "aimMinOmegaCmd");

        config = PhoenixTargeting.Config.defaults();
        config.selectionMaxAgeSec = Double.POSITIVE_INFINITY;
        assertConstructionFailure(profile, config, "selectionMaxAgeSec");

        config = PhoenixTargeting.Config.defaults();
        config.selectionReacquireSec = -1.0;
        assertConstructionFailure(profile, config, "selectionReacquireSec");

        config = PhoenixTargeting.Config.defaults();
        config.defaultAimOffset = null;
        assertConstructionFailure(profile, config, "defaultAimOffset");

        config = PhoenixTargeting.Config.defaults();
        config.defaultAimOffset.leftInches = Double.NaN;
        assertConstructionFailure(profile, config, "defaultAimOffset.leftInches");

        config = PhoenixTargeting.Config.defaults();
        config.shotVelocityTable = null;
        assertConstructionFailure(profile, config, "shotVelocityTable");

        PhoenixTargeting.Config boundaries = PhoenixTargeting.Config.defaults();
        boundaries.aimToleranceDeg = 0.0;
        boundaries.aimKp = 0.0;
        boundaries.aimMaxOmegaCmd = 0.0;
        boundaries.aimReadyToleranceDeg = 0.0;
        boundaries.aimReadyDebounceSec = 0.0;
        boundaries.aimMinOmegaCmd = 0.0;
        boundaries.selectionMaxAgeSec = 0.0;
        boundaries.selectionReacquireSec = 0.0;
        boundaries.defaultAimOffset.forwardInches = Double.MAX_VALUE;
        boundaries.defaultAimOffset.leftInches = -Double.MAX_VALUE;
        targeting(
                profile,
                boundaries,
                new CountingEmptyAprilTagSensor(),
                profile.fixedAprilTagLayout,
                Source.constant(Collections.singleton(boundaries.redAllianceScoringTagId))
        );
    }

    @Test
    public void inactiveAllianceAndUnusedCatalogEvidenceStayDormant() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixTargeting.Config config = PhoenixTargeting.Config.defaults();
        int selectedId = config.redAllianceScoringTagId;
        config.blueAllianceScoringTagId = -1;
        config.scoringTargets.put(-7, null);
        config.scoringTargets.get(20).label = null;
        config.scoringTargets.get(20).aimOffset = null;

        CountingEmptyAprilTagSensor sensor = new CountingEmptyAprilTagSensor();
        PhoenixTargeting targeting = targeting(
                profile,
                config,
                sensor,
                profile.fixedAprilTagLayout,
                Source.constant(Collections.singleton(selectedId))
        );
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        targeting.update(clock);
        assertEquals("selection and guidance both sample the shared sensor", 2,
                sensor.sampleCalls);
        assertEquals("the AprilTag sensor acquires at most once per loop cycle", 1,
                sensor.acquisitions);
    }

    @Test
    public void eligibleMemberShapePrecedesCatalogAndSelectedFactsPrecedeSensorRead() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixTargeting.Config missingCatalog = PhoenixTargeting.Config.defaults();
        missingCatalog.scoringTargets = null;
        assertStartFailure(
                profile,
                missingCatalog,
                Source.constant(Collections.singleton(-1)),
                profile.fixedAprilTagLayout,
                "non-negative"
        );

        Set<Integer> nullMember = new LinkedHashSet<Integer>();
        nullMember.add(null);
        assertStartFailure(
                profile,
                missingCatalog,
                Source.constant(nullMember),
                profile.fixedAprilTagLayout,
                "must not contain null"
        );

        PhoenixTargeting.Config invalidTarget = PhoenixTargeting.Config.defaults();
        int selectedId = invalidTarget.redAllianceScoringTagId;
        invalidTarget.scoringTargets.get(selectedId).label = "   ";
        assertStartFailure(
                profile,
                invalidTarget,
                Source.constant(Collections.singleton(selectedId)),
                profile.fixedAprilTagLayout,
                ".label"
        );

        invalidTarget = PhoenixTargeting.Config.defaults();
        selectedId = invalidTarget.redAllianceScoringTagId;
        invalidTarget.scoringTargets.get(selectedId).aimOffset = null;
        assertStartFailure(
                profile,
                invalidTarget,
                Source.constant(Collections.singleton(selectedId)),
                profile.fixedAprilTagLayout,
                ".aimOffset"
        );

        invalidTarget = PhoenixTargeting.Config.defaults();
        selectedId = invalidTarget.redAllianceScoringTagId;
        invalidTarget.scoringTargets.get(selectedId).aimOffset.leftInches = Double.NaN;
        assertStartFailure(
                profile,
                invalidTarget,
                Source.constant(Collections.singleton(selectedId)),
                profile.fixedAprilTagLayout,
                "aimOffset.leftInches"
        );
    }

    @Test
    public void selectedFieldPoseAndComposedAimPointMustBothRemainFinite() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixTargeting.Config config = PhoenixTargeting.Config.defaults();
        int selectedId = config.redAllianceScoringTagId;
        config.scoringTargets.get(selectedId).aimOffset.forwardInches = Double.MAX_VALUE;
        TagLayout overflowingLayout = oneTagLayout(
                selectedId,
                new Pose3d(Double.MAX_VALUE, 0.0, 0.0, 0.0, 0.0, 0.0)
        );
        assertStartFailure(
                profile,
                config,
                Source.constant(Collections.singleton(selectedId)),
                overflowingLayout,
                "composed fieldToAimPoint"
        );

        config = PhoenixTargeting.Config.defaults();
        selectedId = config.redAllianceScoringTagId;
        TagLayout nonFiniteLayout = oneTagLayout(
                selectedId,
                new Pose3d(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0)
        );
        assertStartFailure(
                profile,
                config,
                Source.constant(Collections.singleton(selectedId)),
                nonFiniteLayout,
                "fixedFieldPose"
        );
    }

    private static void assertConstructionFailure(PhoenixProfile profile,
                                                  PhoenixTargeting.Config config,
                                                  String expectedPath) {
        try {
            targeting(
                    profile,
                    config,
                    new CountingEmptyAprilTagSensor(),
                    profile.fixedAprilTagLayout,
                    Source.constant(Collections.singleton(24))
            );
            fail("expected targeting construction failure for " + expectedPath);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("PhoenixTargeting.Config"));
            assertTrue(expected.getMessage().contains(expectedPath));
        }
    }

    private static void assertStartFailure(PhoenixProfile profile,
                                           PhoenixTargeting.Config config,
                                           Source<Set<Integer>> eligible,
                                           TagLayout layout,
                                           String expectedText) {
        CountingEmptyAprilTagSensor sensor = new CountingEmptyAprilTagSensor();
        PhoenixTargeting targeting = targeting(profile, config, sensor, layout, eligible);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        try {
            targeting.update(clock);
            fail("expected selected targeting validation failure containing " + expectedText);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(expectedText));
        }
        assertEquals("selected validation must complete before the first sensor sample", 0,
                sensor.sampleCalls);
        assertEquals("selected validation must complete before the first sensor acquisition", 0,
                sensor.acquisitions);
        assertFalse(targeting.status().selection.hasSelection);
    }

    private static PhoenixTargeting targeting(PhoenixProfile profile,
                                               PhoenixTargeting.Config config,
                                               AprilTagSensor sensor,
                                               TagLayout layout,
                                               Source<Set<Integer>> eligible) {
        return new PhoenixTargeting(
                config,
                profile.localization.estimation.aprilTags.fieldPoseSolver,
                sensor,
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                layout,
                eligible,
                BooleanSource.constant(true),
                BooleanSource.constant(false)
        );
    }

    private static TagLayout oneTagLayout(final int tagId, final Pose3d pose) {
        return new TagLayout() {
            @Override
            public Pose3d getFieldToTagPose(int id) {
                return id == tagId ? pose : null;
            }

            @Override
            public Set<Integer> ids() {
                return Collections.singleton(tagId);
            }
        };
    }

    private static void assertRawDouble(double expected, double actual) {
        assertEquals(
                Double.doubleToRawLongBits(expected),
                Double.doubleToRawLongBits(actual)
        );
    }

    private static final class CountingEmptyAprilTagSensor implements AprilTagSensor {
        int sampleCalls;
        int acquisitions;
        long lastCycle = Long.MIN_VALUE;
        AprilTagDetections snapshot = AprilTagDetections.none();

        @Override
        public AprilTagDetections get(LoopClock clock) {
            sampleCalls++;
            if (clock.cycle() != lastCycle) {
                acquisitions++;
                lastCycle = clock.cycle();
                snapshot = AprilTagDetections.none();
            }
            return snapshot;
        }
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate noPose = PoseEstimate.noPose(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            // The targeting query consumes this owner's existing snapshot.
        }

        @Override
        public PoseEstimate getEstimate() {
            return noPose;
        }
    }
}
