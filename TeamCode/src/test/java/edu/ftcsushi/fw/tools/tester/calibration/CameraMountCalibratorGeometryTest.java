package edu.ftcsushi.fw.tools.tester.calibration;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.tools.tester.calibration.CameraMountCalibratorEvidenceTest.Fixture;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/**
 * Independent CAL-09 transform and orientation-mean regressions.
 *
 * <p>Observation truth uses elementary rotations of points and unit axes, never the production
 * transform composition/inverse or matrix helpers. The owner fixture keeps real setup, camera
 * ownership, clock and loop behavior, replacing the external camera's observations/readiness.
 * Private averager probes are explicitly calculation-only, not hardware evidence. These tests
 * cannot establish camera intrinsics, tag/robot placement accuracy or physical acceptance.</p>
 */
public final class CameraMountCalibratorGeometryTest {
    private static final double EPS = 1e-9;

    @Test
    public void fullMountRecoveryUsesIndependentCameraAndTagAxes() throws Exception {
        Pose3d mount = degrees(4.0, -2.0, 7.0, 30.0, 20.0, -15.0);
        Pose3d robot = degrees(10.0, 20.0, 0.0, 60.0, 0.0, 0.0);
        Pose3d tag = degrees(8.0, 60.0, 12.0, -20.0, 15.0, 35.0);
        Fixture f = new Fixture(new SimpleTagLayout().addPose(7, tag));
        setField(f.owner, "fieldToRobotPose", robot);
        f.clock.update(0.02);
        f.frameNow(7, observed(robot, mount, tag));
        f.owner.initLoop(f.clock.dtSec());

        Pose3d solved = (Pose3d) field(f.owner, "lastRobotToCameraSample");
        assertMount(mount, solved);
        // This nonsingular case also has one ordinary canonical yaw/pitch/roll spelling.
        assertEquals(mount.yawRad, solved.yawRad, EPS);
        assertEquals(mount.pitchRad, solved.pitchRad, EPS);
        assertEquals(mount.rollRad, solved.rollRad, EPS);
        f.owner.stop();
    }

    @Test
    public void nearVerticalMeanStaysBetweenTheClosePhysicalOrientations() throws Exception {
        Object average = arithmeticAverager();
        Pose3d first = degrees(4.0, -2.0, 7.0, 0.0, 89.0, 0.0);
        Pose3d second = degrees(4.0, -2.0, 7.0, 179.99, 89.0, -179.99);
        add(average, first);
        add(average, second);
        Pose3d mean = mean(average);

        assertNotNull(mean);
        assertEquals(2.00009999496, separationDegrees(first, second), 1e-8);
        assertTrue("the average must not turn two nearby mounts into an opposite orientation",
                separationDegrees(first, mean) < 1.01);
        assertTrue("neither input alone is the equal-weight midpoint",
                separationDegrees(second, mean) < 1.01);
        assertEquals(4.0, mean.xInches, EPS);
        assertEquals(-2.0, mean.yInches, EPS);
        assertEquals(7.0, mean.zInches, EPS);
    }

    @Test
    public void chosenMountIsRecoveredAcrossIndependentRobotPosesAndTagsInSeparateBatches()
            throws Exception {
        Pose3d[] mounts = {
                degrees(4.0, -2.0, 7.0, 30.0, 20.0, -15.0),
                degrees(-3.0, 1.5, 9.0, -145.0, -25.0, 35.0),
                degrees(2.0, 3.0, 6.0, 178.0, 89.0, -177.0)
        };
        Pose3d[] robots = {
                degrees(10.0, 20.0, 0.0, 60.0, 0.0, 0.0),
                degrees(-12.0, 8.0, 0.0, -45.0, 0.0, 0.0),
                degrees(24.0, -18.0, 0.0, 150.0, 0.0, 0.0)
        };
        Pose3d[] tags = {
                degrees(8.0, 60.0, 12.0, -20.0, 15.0, 35.0),
                degrees(20.0, 9.0, 15.0, 170.0, -12.0, 8.0),
                degrees(-20.0, -20.0, 9.0, 80.0, 10.0, -25.0)
        };
        for (int mountIndex = 0; mountIndex < mounts.length; mountIndex++) {
            for (int setup = 0; setup < robots.length; setup++) {
                try {
                    int tagId = 7 + setup;
                    // Every different known setup has a fresh owner/batch. This does not bypass
                    // the user-facing rule that changing a setup clears its previous samples.
                    Fixture f = new Fixture(new SimpleTagLayout().addPose(tagId, tags[setup]));
                    setField(f.owner, "fieldToRobotPose", robots[setup]);
                    Pose3d observation = observed(robots[setup], mounts[mountIndex], tags[setup]);
                    preview(f, tagId, observation);
                    assertMount(mounts[mountIndex],
                            (Pose3d) field(f.owner, "lastRobotToCameraSample"));
                    capture(f, tagId, observation);
                    Object average = field(f.owner, "avg");
                    assertEquals(1, count(average));
                    assertMount(mounts[mountIndex], mean(average));
                    f.owner.stop();
                } catch (AssertionError failure) {
                    throw new AssertionError("mount=" + mountIndex + ", setup=" + setup, failure);
                }
            }
        }
    }

    @Test
    public void sameFixedSetupPerturbedObservationsAverageAboutTheChosenMount() throws Exception {
        Pose3d center = degrees(4.0, -2.0, 7.0, 25.0, 40.0, -35.0);
        Pose3d robot = degrees(10.0, 20.0, 0.0, 60.0, 0.0, 0.0);
        Pose3d tag = degrees(8.0, 60.0, 12.0, -20.0, 15.0, 35.0);
        Pose3d[] perturbed = {
                degrees(5.0, -2.5, 7.25, 19.0, 40.0, -35.0),
                degrees(3.0, -1.5, 6.75, 31.0, 40.0, -35.0),
                degrees(3.0, -2.5, 6.75, 25.0, 40.0, -43.0),
                degrees(5.0, -1.5, 7.25, 25.0, 40.0, -27.0)
        };
        Fixture f = new Fixture(new SimpleTagLayout().addPose(7, tag));
        setField(f.owner, "fieldToRobotPose", robot);
        for (Pose3d sample : perturbed) {
            Pose3d observation = observed(robot, sample, tag);
            preview(f, 7, observation);
            assertMount(sample, (Pose3d) field(f.owner, "lastRobotToCameraSample"));
            capture(f, 7, observation);
        }

        // Equal opposite rotations around two axes have this known common center. The setup
        // never changes: only the synthetic observations model symmetric measurement errors.
        Object average = field(f.owner, "avg");
        assertEquals(4, count(average));
        assertMount(center, mean(average));
        f.owner.stop();
    }

    @Test
    public void exactVerticalMountsRecoverThroughTheActualOwnerSolveAndCapture() throws Exception {
        Pose3d robot = degrees(10.0, 20.0, 0.0, 60.0, 0.0, 0.0);
        Pose3d tag = degrees(8.0, 60.0, 12.0, -20.0, 15.0, 35.0);
        double[][] yawAndRoll = {{12.0, -27.0}, {25.0, -35.0}, {-38.0, 41.0}, {-135.0, 70.0}};
        for (double pitch : new double[]{-90.0, 90.0}) {
            for (double[] pair : yawAndRoll) {
                try {
                    Pose3d mount = degrees(4.0, -2.0, 7.0, pair[0], pitch, pair[1]);
                    Fixture f = new Fixture(new SimpleTagLayout().addPose(7, tag));
                    setField(f.owner, "fieldToRobotPose", robot);
                    Pose3d observation = observed(robot, mount, tag);
                    preview(f, 7, observation);
                    assertMount(mount, (Pose3d) field(f.owner, "lastRobotToCameraSample"));
                    capture(f, 7, observation);
                    Object average = field(f.owner, "avg");
                    assertEquals(1, count(average));
                    assertMount(mount, mean(average));
                    f.owner.stop();
                } catch (AssertionError failure) {
                    throw new AssertionError("owner vertical yaw=" + pair[0] + ", pitch=" + pitch
                            + ", roll=" + pair[1], failure);
                }
            }
        }
    }

    @Test
    public void everySingleRotationIncludesHalfTurnsAndSingularEulerRepresentations()
            throws Exception {
        double[][] orientations = {
                {0.0, 0.0, 0.0}, {180.0, 0.0, 0.0}, {0.0, 180.0, 0.0}, {0.0, 0.0, 180.0},
                {90.0, 0.0, 0.0}, {-90.0, 0.0, 0.0}, {25.0, 40.0, -35.0},
                {12.0, 90.0, -27.0}, {-38.0, -90.0, 41.0},
                {179.99, 89.0, -179.99}, {-170.0, -89.0, 176.0}, {720.0, -360.0, 540.0}
        };
        for (double[] rotation : orientations) {
            Object average = arithmeticAverager();
            Pose3d sample = degrees(4.0, -2.0, 7.0, rotation[0], rotation[1], rotation[2]);
            add(average, sample);
            assertEquals(1, count(average));
            assertMount(sample, mean(average));
        }
    }

    @Test
    public void singleAndRepeatedExactVerticalOrientationsPreserveTheirPhysicalAxes()
            throws Exception {
        for (double pitch : new double[]{-90.0, 90.0}) {
            for (double yaw : new double[]{-170.0, -90.0, -35.0, 0.0, 12.0, 25.0, 90.0, 179.0}) {
                for (double roll : new double[]{-177.0, -90.0, -27.0, 0.0, 41.0, 70.0, 90.0, 172.0}) {
                    Object average = arithmeticAverager();
                    Pose3d expected = degrees(4.0, -2.0, 7.0, yaw, pitch, roll);
                    for (int copies = 1; copies <= 3; copies++) {
                        add(average, expected);
                        assertEquals(copies, count(average));
                        try {
                            assertMount(expected, mean(average));
                        } catch (AssertionError failure) {
                            throw new AssertionError("vertical yaw=" + yaw + ", pitch=" + pitch
                                    + ", roll=" + roll + ", copies=" + copies, failure);
                        }
                    }
                }
            }
        }
    }

    @Test
    public void ordinaryYawWrapHasTheHalfTurnMeanRatherThanZero() throws Exception {
        Object average = arithmeticAverager();
        add(average, degrees(2.0, -4.0, 6.0, 179.0, 0.0, 0.0));
        add(average, degrees(6.0, 2.0, 10.0, -179.0, 0.0, 0.0));

        assertEquals(2, count(average));
        assertMount(degrees(4.0, -1.0, 8.0, 180.0, 0.0, 0.0), mean(average));
    }

    @Test
    public void equivalentEulerSpellingsCannotChangeThePhysicalAverage() throws Exception {
        Pose3d expected = degrees(4.0, -2.0, 7.0, 20.0, 30.0, 40.0);
        Pose3d[] equivalent = {
                expected,
                degrees(4.0, -2.0, 7.0, 380.0, 30.0, 40.0),
                degrees(4.0, -2.0, 7.0, -340.0, 30.0, 400.0),
                degrees(4.0, -2.0, 7.0, 200.0, 150.0, 220.0)
        };
        Object average = arithmeticAverager();
        for (Pose3d spelling : equivalent) {
            assertMount(expected, spelling); // Independently establish equivalence first.
            add(average, spelling);
            assertMount(expected, mean(average));
        }
        assertEquals(equivalent.length, count(average));
    }

    @Test
    public void sampleOrderAndFullTurnSignRepresentationsPreserveTheKnownPairMidpoint()
            throws Exception {
        Pose3d expected = degrees(4.0, -2.0, 7.0, 25.0, 40.0, -35.0);
        Pose3d first = degrees(2.0, -4.0, 6.0, 25.0, 40.0, -45.0);
        Pose3d second = degrees(6.0, 0.0, 8.0, 25.0, 40.0, -25.0);
        Pose3d secondOtherSpelling = degrees(6.0, 0.0, 8.0, 385.0, 40.0, -25.0);
        for (Pose3d[] order : new Pose3d[][]{{first, second}, {second, first},
                {first, secondOtherSpelling}, {secondOtherSpelling, first}}) {
            Object average = arithmeticAverager();
            for (Pose3d sample : order) add(average, sample);
            assertMount(expected, mean(average));
        }
    }

    @Test
    public void opposingRotationsSuppressTheOldMeanWithoutDroppingTheirContribution()
            throws Exception {
        Object average = arithmeticAverager();
        Pose3d identity = degrees(2.0, -4.0, 6.0, 0.0, 0.0, 0.0);
        add(average, identity);
        assertMount(identity, mean(average));
        add(average, degrees(6.0, 2.0, 10.0, 180.0, 0.0, 0.0));
        assertEquals("both finite samples remain accepted", 2, count(average));
        assertNull("an ambiguous batch cannot retain its old printable answer", mean(average));

        add(average, degrees(1.0, -1.0, 8.0, 0.0, 0.0, 0.0));
        assertEquals(3, count(average));
        // Including all three translations proves the disagreeing second sample was not dropped.
        assertMount(degrees(3.0, -1.0, 8.0, 0.0, 0.0, 0.0), mean(average));
    }

    @Test
    public void allFourEqualQuaternionDirectionsAreAmbiguousUntilDistinctEvidenceResolvesThem()
            throws Exception {
        Object average = arithmeticAverager();
        for (Pose3d sample : new Pose3d[]{degrees(4, -2, 7, 0, 0, 0),
                degrees(4, -2, 7, 180, 0, 0), degrees(4, -2, 7, 0, 180, 0),
                degrees(4, -2, 7, 0, 0, 180)}) {
            add(average, sample);
        }
        assertEquals(4, count(average));
        assertNull(mean(average));
        add(average, degrees(4, -2, 7, 0, 0, 0));
        assertEquals(5, count(average));
        assertMount(degrees(4, -2, 7, 0, 0, 0), mean(average));
    }

    @Test
    public void privateInvalidPoseProbesLeaveCountStatisticsAndCachedMeanUnchanged()
            throws Exception {
        Object average = arithmeticAverager();
        Pose3d valid = degrees(4.0, -2.0, 7.0, 25.0, 40.0, -35.0);
        add(average, valid);
        Pose3d originalMean = mean(average);
        // These bypass the camera/readiness boundary deliberately: they probe private numerical
        // defense, not native camera representability or a user-facing configuration path.
        for (int component = 0; component < 6; component++) {
            for (double invalid : new double[]{Double.NaN,
                    Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
                double[] values = {4.0, -2.0, 7.0, 0.2, 0.3, 0.4};
                values[component] = invalid;
                add(average, new Pose3d(values[0], values[1], values[2],
                        values[3], values[4], values[5]));
                assertEquals(1, count(average));
                assertSame("rejected arithmetic cannot replace the cached valid answer",
                        originalMean, mean(average));
            }
        }
        add(average, valid);
        assertEquals(2, count(average));
        assertMount(valid, mean(average));
    }

    @Test
    public void privateExtremeFiniteTranslationsAverageWithoutRawSumOrDeltaOverflow()
            throws Exception {
        Object average = arithmeticAverager();
        double largest = Double.MAX_VALUE;
        Pose3d positive = new Pose3d(largest, -largest, largest, 0.2, 0.3, 0.4);
        Pose3d negative = new Pose3d(-largest, largest, -largest, 0.2, 0.3, 0.4);
        add(average, positive);
        add(average, positive);
        Pose3d repeated = mean(average);
        assertNotNull(repeated);
        assertEquals(largest, repeated.xInches, 0.0);
        assertEquals(-largest, repeated.yInches, 0.0);
        assertEquals(largest, repeated.zInches, 0.0);

        add(average, negative);
        assertEquals(3, count(average));
        Pose3d balanced = mean(average);
        assertNotNull(balanced);
        assertTrue(Double.isFinite(balanced.xInches));
        assertTrue(Double.isFinite(balanced.yInches));
        assertTrue(Double.isFinite(balanced.zInches));
        assertEquals(largest / 3.0, balanced.xInches, largest * 1e-15);
        assertEquals(-largest / 3.0, balanced.yInches, largest * 1e-15);
        assertEquals(largest / 3.0, balanced.zInches, largest * 1e-15);
        assertOrientation(positive, balanced);
        // A finite numerical mean at this magnitude is not physically plausible mount geometry.
    }

    @Test
    public void clearRemovesAmbiguityAndAcceptsAFreshUnrelatedOrientation() throws Exception {
        Object average = arithmeticAverager();
        add(average, degrees(4, -2, 7, 0, 0, 0));
        add(average, degrees(4, -2, 7, 180, 0, 0));
        assertNull(mean(average));
        invoke(average, "clear", new Class<?>[0]);
        assertEquals(0, count(average));
        assertNull(mean(average));

        Pose3d fresh = degrees(-3, 5, 8, -135, -90, 70);
        add(average, fresh);
        assertEquals(1, count(average));
        assertMount(fresh, mean(average));
    }

    @Test
    public void syntheticFiniteTransformOverflowCannotBecomeACapturedMount() throws Exception {
        // Maintainer numerical probe, not a physical field or native detector claim. The finite
        // endpoints overflow when field-to-robot inversion and field-to-tag translation compose.
        Pose3d tag = new Pose3d(Double.MAX_VALUE, 0.0, 0.0, 0.0, 0.0, 0.0);
        Fixture f = new Fixture(new SimpleTagLayout().addPose(7, tag));
        setField(f.owner, "fieldToRobotPose",
                new Pose3d(-Double.MAX_VALUE, 0.0, 0.0, 0.0, 0.0, 0.0));
        preview(f, 7, Pose3d.zero());
        assertNull(field(f.owner, "lastRobotToCameraSample"));
        capture(f, 7, Pose3d.zero());
        assertNull(field(f.owner, "lastRobotToCameraSample"));
        Object average = field(f.owner, "avg");
        assertEquals(0, count(average));
        assertNull(mean(average));
        f.owner.stop();
    }

    /** A released button and a new timestamp create only a preview, not an accepted sample. */
    private static void preview(Fixture f, int tagId, Pose3d observation) {
        f.gamepad.a = false;
        f.clock.update(f.clock.nowSec() + 0.01);
        f.frameNow(tagId, observation);
        f.owner.initLoop(f.clock.dtSec());
    }

    /** The real binding edge requests capture, with a fresh observation in that same owner loop. */
    private static void capture(Fixture f, int tagId, Pose3d observation) {
        f.gamepad.a = true;
        f.clock.update(f.clock.nowSec() + 0.01);
        f.frameNow(tagId, observation);
        f.owner.initLoop(f.clock.dtSec());
    }

    /**
     * Forward measurement construction from known physical points and axes. The camera origin is
     * placed on the robot; the tag's origin and each tag axis are projected onto the camera axes.
     * Only the resulting observation is represented as a Pose3d for the real input boundary.
     */
    private static Pose3d observed(Pose3d robot, Pose3d mount, Pose3d tag) {
        double[] mounted = rotate(translation(mount), robot);
        double[] cameraOrigin = {robot.xInches + mounted[0], robot.yInches + mounted[1],
                robot.zInches + mounted[2]};
        double[][] cameraAxes = axes(mount);
        for (int i = 0; i < 3; i++) cameraAxes[i] = rotate(cameraAxes[i], robot);
        double[] toTag = {tag.xInches - cameraOrigin[0], tag.yInches - cameraOrigin[1],
                tag.zInches - cameraOrigin[2]};
        double[][] tagAxes = axes(tag);
        double[][] projected = new double[3][3];
        for (int axis = 0; axis < 3; axis++) {
            for (int component = 0; component < 3; component++) {
                projected[axis][component] = dot(tagAxes[axis], cameraAxes[component]);
            }
        }
        double yaw = Math.atan2(projected[0][1], projected[0][0]);
        double pitch = Math.atan2(-projected[0][2],
                Math.hypot(projected[0][0], projected[0][1]));
        double roll = Math.atan2(projected[1][2], projected[2][2]);
        return new Pose3d(dot(toTag, cameraAxes[0]), dot(toTag, cameraAxes[1]),
                dot(toTag, cameraAxes[2]), yaw, pitch, roll);
    }

    /** Rotate one vector about X, then Y, then Z; each elementary rotation is explicit. */
    private static double[] rotate(double[] vector, Pose3d orientation) {
        double rollX = vector[0];
        double rollY = Math.cos(orientation.rollRad) * vector[1]
                - Math.sin(orientation.rollRad) * vector[2];
        double rollZ = Math.sin(orientation.rollRad) * vector[1]
                + Math.cos(orientation.rollRad) * vector[2];
        double pitchX = Math.cos(orientation.pitchRad) * rollX
                + Math.sin(orientation.pitchRad) * rollZ;
        double pitchY = rollY;
        double pitchZ = -Math.sin(orientation.pitchRad) * rollX
                + Math.cos(orientation.pitchRad) * rollZ;
        return new double[]{Math.cos(orientation.yawRad) * pitchX
                - Math.sin(orientation.yawRad) * pitchY,
                Math.sin(orientation.yawRad) * pitchX
                        + Math.cos(orientation.yawRad) * pitchY, pitchZ};
    }

    /** Columns are where this orientation places the original forward, left and up unit axes. */
    private static double[][] axes(Pose3d pose) {
        return new double[][]{rotate(new double[]{1.0, 0.0, 0.0}, pose),
                rotate(new double[]{0.0, 1.0, 0.0}, pose),
                rotate(new double[]{0.0, 0.0, 1.0}, pose)};
    }

    private static double[] translation(Pose3d pose) {
        return new double[]{pose.xInches, pose.yInches, pose.zInches};
    }

    private static double dot(double[] first, double[] second) {
        return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
    }

    /** Physical rotation separation, independent of the possibly nonunique Euler spelling. */
    private static double separationDegrees(Pose3d first, Pose3d second) {
        double[][] a = axes(first);
        double[][] b = axes(second);
        double trace = dot(a[0], b[0]) + dot(a[1], b[1]) + dot(a[2], b[2]);
        return Math.toDegrees(Math.acos(Math.max(-1.0, Math.min(1.0, (trace - 1.0) / 2.0))));
    }

    /** Compare known translation and physical axes, never production-composed residuals. */
    private static void assertMount(Pose3d expected, Pose3d actual) {
        assertNotNull(actual);
        assertEquals(expected.xInches, actual.xInches, EPS);
        assertEquals(expected.yInches, actual.yInches, EPS);
        assertEquals(expected.zInches, actual.zInches, EPS);
        assertOrientation(expected, actual);
    }

    /** Compare three independent physical axes rather than subtracting Euler components. */
    private static void assertOrientation(Pose3d expected, Pose3d actual) {
        assertNotNull(actual);
        double[][] expectedAxes = axes(expected);
        double[][] actualAxes = axes(actual);
        for (int axis = 0; axis < 3; axis++) {
            for (int component = 0; component < 3; component++) {
                assertEquals("axis=" + axis + ", component=" + component,
                        expectedAxes[axis][component], actualAxes[axis][component], EPS);
            }
        }
    }

    private static Pose3d degrees(double x, double y, double z,
                                   double yaw, double pitch, double roll) {
        return new Pose3d(x, y, z, Math.toRadians(yaw), Math.toRadians(pitch), Math.toRadians(roll));
    }

    /** Private arithmetic-only probe: construction is real, but there is no device/loop claim. */
    private static Object arithmeticAverager() throws Exception {
        CameraMountCalibrator.Config config = CameraMountCalibrator.Config.defaults();
        config.fixedTagLayout = new SimpleTagLayout();
        CameraMountCalibrator owner = new CameraMountCalibrator(config,
                ignored -> hardwareMap -> null);
        return field(owner, "avg");
    }

    private static void add(Object average, Pose3d pose) throws Exception {
        invoke(average, "add", new Class<?>[]{Pose3d.class}, pose);
    }

    private static Pose3d mean(Object average) throws Exception {
        return (Pose3d) invoke(average, "meanOrNull", new Class<?>[0]);
    }

    private static int count(Object average) throws Exception {
        return ((Number) invoke(average, "count", new Class<?>[0])).intValue();
    }

    private static Object field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }

    private static void setField(Object owner, String name, Object value) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(owner, value);
    }

    private static Object invoke(Object owner, String name, Class<?>[] types, Object... args)
            throws Exception {
        Method method = owner.getClass().getDeclaredMethod(name, types);
        method.setAccessible(true);
        try {
            return method.invoke(owner, args);
        } catch (InvocationTargetException failure) {
            Throwable cause = failure.getCause();
            if (cause instanceof RuntimeException) throw (RuntimeException) cause;
            if (cause instanceof Error) throw (Error) cause;
            throw failure;
        }
    }
}
