package edu.ftcphoenix.fw.drive.source;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the owner-local validation and snapshot contract for gamepad drive configuration. */
public final class GamepadDriveSourceConfigTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void defaultsRemainExactAndFiniteEndpointsHaveTheirDocumentedBehavior() {
        GamepadDriveSource.Config defaults = GamepadDriveSource.Config.defaults();

        assertEquals(0.05, defaults.deadband, EPSILON);
        assertEquals(1.5, defaults.translateExpo, EPSILON);
        assertEquals(1.5, defaults.rotateExpo, EPSILON);
        assertEquals(1.0, defaults.translateScale, EPSILON);
        assertEquals(1.0, defaults.rotateScale, EPSILON);

        GamepadDriveSource.Config identityEndpoints = GamepadDriveSource.Config.defaults();
        identityEndpoints.deadband = 0.0;
        identityEndpoints.translateExpo = 1.0;
        identityEndpoints.rotateExpo = 1.0;
        identityEndpoints.translateScale = 1.0;
        identityEndpoints.rotateScale = 1.0;
        DriveSignal identity = sourceForValues(0.4, 0.5, -0.8, identityEndpoints)
                .get(new ManualLoopClock().clock());
        assertEquals(0.5, identity.axial, EPSILON);
        assertEquals(-0.4, identity.lateral, EPSILON);
        assertEquals(0.8, identity.omega, EPSILON);

        GamepadDriveSource.Config zeroScales = identityEndpoints.copy();
        zeroScales.translateScale = 0.0;
        zeroScales.rotateScale = 0.0;
        DriveSignal zero = sourceForValues(0.4, 0.5, -0.8, zeroScales)
                .get(new ManualLoopClock().clock());
        assertEquals(0.0, zero.axial, EPSILON);
        assertEquals(0.0, zero.lateral, EPSILON);
        assertEquals(0.0, zero.omega, EPSILON);

        GamepadDriveSource.Config fullDeadband = identityEndpoints.copy();
        fullDeadband.deadband = 1.0;
        DriveSignal deadbanded = sourceForValues(1.0, -1.0, 1.0, fullDeadband)
                .get(new ManualLoopClock().clock());
        assertEquals(0.0, deadbanded.axial, EPSILON);
        assertEquals(0.0, deadbanded.lateral, EPSILON);
        assertEquals(0.0, deadbanded.omega, EPSILON);
    }

    @Test
    public void deadbandRejectsNonfiniteAndFiniteOutOfRangeValuesBeforeSampling() {
        assertInvalidValues(
                "deadband",
                "finite and in [0.0, 1.0]",
                new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                        Double.NEGATIVE_INFINITY, -0.01, 1.01},
                new ConfigAnswer() {
                    @Override
                    public void set(GamepadDriveSource.Config config, double value) {
                        config.deadband = value;
                    }
                }
        );
    }

    @Test
    public void exponentsRejectNonfiniteAndFiniteValuesBelowOneBeforeSampling() {
        double[] invalid = new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -1.0, 0.999};
        assertInvalidValues(
                "translateExpo",
                "finite and >= 1.0",
                invalid,
                new ConfigAnswer() {
                    @Override
                    public void set(GamepadDriveSource.Config config, double value) {
                        config.translateExpo = value;
                    }
                }
        );
        assertInvalidValues(
                "rotateExpo",
                "finite and >= 1.0",
                invalid,
                new ConfigAnswer() {
                    @Override
                    public void set(GamepadDriveSource.Config config, double value) {
                        config.rotateExpo = value;
                    }
                }
        );
    }

    @Test
    public void scalesRejectNonfiniteAndFiniteOutOfRangeValuesBeforeSampling() {
        double[] invalid = new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -0.01, 1.01};
        assertInvalidValues(
                "translateScale",
                "finite and in [0.0, 1.0]",
                invalid,
                new ConfigAnswer() {
                    @Override
                    public void set(GamepadDriveSource.Config config, double value) {
                        config.translateScale = value;
                    }
                }
        );
        assertInvalidValues(
                "rotateScale",
                "finite and in [0.0, 1.0]",
                invalid,
                new ConfigAnswer() {
                    @Override
                    public void set(GamepadDriveSource.Config config, double value) {
                        config.rotateScale = value;
                    }
                }
        );
    }

    @Test
    public void rawCopyPreservesInvalidDraftDataAndOwnerRejectsIt() {
        GamepadDriveSource.Config original = GamepadDriveSource.Config.defaults();
        original.deadband = Double.NaN;
        GamepadDriveSource.Config copy = original.copy();

        assertTrue(Double.isNaN(copy.deadband));
        original.deadband = 0.25;
        copy.translateScale = 0.4;
        assertTrue(Double.isNaN(copy.deadband));
        assertEquals(1.0, original.translateScale, EPSILON);
        assertInvalidConstruction(
                copy,
                "GamepadDriveSource.Config.deadband must be finite and in [0.0, 1.0], got NaN."
        );
    }

    @Test
    public void callerMutationCannotChangeTheRetainedConfiguration() {
        GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
        config.deadband = 0.0;
        config.translateExpo = 1.0;
        config.rotateExpo = 1.0;
        config.translateScale = 0.5;
        config.rotateScale = 0.25;
        GamepadDriveSource source = new GamepadDriveSource(
                ScalarSource.of(() -> 0.4),
                ScalarSource.of(() -> 0.5),
                ScalarSource.of(() -> -0.8),
                config
        );

        config.deadband = Double.NaN;
        config.translateExpo = Double.NaN;
        config.rotateExpo = Double.NaN;
        config.translateScale = Double.NaN;
        config.rotateScale = Double.NaN;

        DriveSignal signal = source.get(new ManualLoopClock().clock());

        assertEquals(0.25, signal.axial, EPSILON);
        assertEquals(-0.2, signal.lateral, EPSILON);
        assertEquals(0.2, signal.omega, EPSILON);

        CapturingDebugSink debug = new CapturingDebugSink();
        source.debugDump(debug, "drive");
        assertEquals(0.0, (Double) debug.values.get("drive.cfg.deadband"), EPSILON);
        assertEquals(1.0, (Double) debug.values.get("drive.cfg.translateExpo"), EPSILON);
        assertEquals(1.0, (Double) debug.values.get("drive.cfg.rotateExpo"), EPSILON);
        assertEquals(0.5, (Double) debug.values.get("drive.cfg.translateScale"), EPSILON);
        assertEquals(0.25, (Double) debug.values.get("drive.cfg.rotateScale"), EPSILON);
    }

    @Test
    public void publicApiRemainsTheSingleMutableConfigAndSourceConstructionPath()
            throws Exception {
        Constructor<?>[] configConstructors = GamepadDriveSource.Config.class
                .getDeclaredConstructors();
        assertEquals(1, configConstructors.length);
        assertTrue(Modifier.isPrivate(configConstructors[0].getModifiers()));
        assertEquals(0, configConstructors[0].getParameterTypes().length);
        assertEquals(0, GamepadDriveSource.Config.class.getConstructors().length);

        Set<String> expectedFields = new HashSet<String>(Arrays.asList(
                "deadband",
                "translateExpo",
                "rotateExpo",
                "translateScale",
                "rotateScale"
        ));
        int publicFields = 0;
        for (Field field : GamepadDriveSource.Config.class.getDeclaredFields()) {
            if (!Modifier.isPublic(field.getModifiers())) {
                continue;
            }
            publicFields++;
            assertTrue("unexpected public Config field: " + field.getName(),
                    expectedFields.remove(field.getName()));
            assertEquals(double.class, field.getType());
            assertFalse(Modifier.isStatic(field.getModifiers()));
            assertFalse(Modifier.isFinal(field.getModifiers()));
        }
        assertEquals(5, publicFields);
        assertTrue(expectedFields.isEmpty());

        Method defaults = GamepadDriveSource.Config.class.getDeclaredMethod("defaults");
        assertTrue(Modifier.isPublic(defaults.getModifiers()));
        assertTrue(Modifier.isStatic(defaults.getModifiers()));
        assertEquals(GamepadDriveSource.Config.class, defaults.getReturnType());

        Method copy = GamepadDriveSource.Config.class.getDeclaredMethod("copy");
        assertTrue(Modifier.isPublic(copy.getModifiers()));
        assertFalse(Modifier.isStatic(copy.getModifiers()));
        assertEquals(GamepadDriveSource.Config.class, copy.getReturnType());

        int publicConfigMethods = 0;
        for (Method method : GamepadDriveSource.Config.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                publicConfigMethods++;
            }
        }
        assertEquals("Config keeps exactly defaults() and copy()", 2, publicConfigMethods);

        Constructor<GamepadDriveSource> sourceConstructor = GamepadDriveSource.class.getConstructor(
                ScalarSource.class,
                ScalarSource.class,
                ScalarSource.class,
                GamepadDriveSource.Config.class
        );
        assertTrue(Modifier.isPublic(sourceConstructor.getModifiers()));
        assertEquals(1, GamepadDriveSource.class.getConstructors().length);
        assertEquals(1, GamepadDriveSource.class.getDeclaredConstructors().length);
    }

    private static void assertInvalidValues(String field,
                                            String domain,
                                            double[] invalidValues,
                                            ConfigAnswer answer) {
        for (double value : invalidValues) {
            GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
            answer.set(config, value);
            assertInvalidConstruction(
                    config,
                    "GamepadDriveSource.Config." + field + " must be " + domain
                            + ", got " + value + "."
            );
        }
    }

    private static void assertInvalidConstruction(GamepadDriveSource.Config config,
                                                  String expectedMessage) {
        RecordingScalarSource axis = new RecordingScalarSource(0.5);

        try {
            newSource(axis, config);
            fail("Expected invalid GamepadDriveSource.Config to be rejected");
        } catch (IllegalArgumentException expected) {
            assertEquals(expectedMessage, expected.getMessage());
        }

        assertEquals("construction must not sample any supplied axis", 0, axis.getCount);
    }

    private static GamepadDriveSource newSource(ScalarSource axis,
                                                 GamepadDriveSource.Config config) {
        return new GamepadDriveSource(axis, axis, axis, config);
    }

    private static GamepadDriveSource sourceForValues(double lateral,
                                                       double axial,
                                                       double omega,
                                                       GamepadDriveSource.Config config) {
        return new GamepadDriveSource(
                ScalarSource.of(() -> lateral),
                ScalarSource.of(() -> axial),
                ScalarSource.of(() -> omega),
                config
        );
    }

    private interface ConfigAnswer {
        void set(GamepadDriveSource.Config config, double value);
    }

    private static final class RecordingScalarSource implements ScalarSource {
        private final double value;
        private int getCount;

        private RecordingScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            getCount++;
            return value;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<String, Object>();

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
