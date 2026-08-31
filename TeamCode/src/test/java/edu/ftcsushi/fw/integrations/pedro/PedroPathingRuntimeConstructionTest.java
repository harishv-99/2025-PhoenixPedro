package edu.ftcsushi.fw.integrations.pedro;

import com.pedropathing.VectorCalculator;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public final class PedroPathingRuntimeConstructionTest {

    @Test
    public void invalidConfigHasNoConstructionHardwareOrVendorGlobalEffects() {
        PedroPathingRuntime.Config invalid = PedroPathingRuntime.Config.defaults();
        invalid.followerConstants.mass = Double.NaN;
        RecordingConstructionFactory factory = new RecordingConstructionFactory();
        CountingHardwareMap hardwareMap = new CountingHardwareMap();
        Map<String, Object> vectorGlobalsBefore = vectorGlobals();
        PathConstraints globalBefore = PathConstraints.defaultConstraints;
        double[] globalValuesBefore = pathValues(globalBefore);

        assertTopLevelNull(
                () -> PedroPathingRuntime.create(
                        null,
                        PedroPathingRuntime.Config.defaults()
                ),
                "requires a non-null HardwareMap"
        );
        assertTopLevelNull(
                () -> PedroPathingRuntime.create(
                        hardwareMap,
                        null
                ),
                PedroPathingRuntime.Config.class.getCanonicalName(),
                "must not be null"
        );

        assertInvalidMass(() -> PedroPathingRuntime.create(
                hardwareMap,
                invalid
        ));
        assertInvalidMass(() -> PedroPathingRuntime.createForTest(
                hardwareMap,
                invalid,
                factory
        ));

        assertEquals(0, hardwareMap.lookupCount);
        assertTrue(factory.events.isEmpty());
        assertEquals(vectorGlobalsBefore, vectorGlobals());
        assertSame(globalBefore, PathConstraints.defaultConstraints);
        assertPathValues(globalValuesBefore, PathConstraints.defaultConstraints);
    }

    @Test
    public void validGraphUsesExactOrderOwnedSnapshotsAndNoPedroGlobal() {
        PathConstraints originalGlobal = PathConstraints.defaultConstraints;
        PathConstraints poisonedGlobal = new PathConstraints(
                0.21,
                22.0,
                23.0,
                0.24,
                25.0,
                2.6,
                27,
                2.8
        );
        PathConstraints.defaultConstraints = poisonedGlobal;
        try {
            PedroPathingRuntime.Config draft = distinctiveValidConfig();
            Vector authoredVector = draft.mecanumConstants.frontLeftVector;
            double authoredTheta = authoredVector.getTheta();
            RecordingConstructionFactory factory = new RecordingConstructionFactory();

            PedroPathingRuntime runtime = PedroPathingRuntime.createForTest(
                    new HardwareMap(null, null),
                    draft,
                    factory
            );

            assertEquals(0, factory.events.indexOf("mecanum.create"));
            assertEquals(1, factory.events.indexOf("pinpoint.create"));
            assertTrue(factory.events.indexOf("pinpoint.resetPosAndIMU") > 1);
            assertTrue(factory.events.indexOf("follower.create")
                    > factory.events.indexOf("pinpoint.resetPosAndIMU"));
            assertEquals(1, factory.drivetrain.breakCount);
            assertSame(poisonedGlobal, PathConstraints.defaultConstraints);
            assertPathValues(
                    new double[] {0.21, 22.0, 23.0, 0.24, 25.0, 2.6, 27.0, 2.8},
                    PathConstraints.defaultConstraints
            );

            assertNotSame(draft.predictor, factory.predictorConfig);
            assertNotSame(draft.followerConstants, factory.followerConstants);
            assertNotSame(draft.mecanumConstants, factory.mecanumConstants);
            assertNotSame(draft.mecanumConstants.frontLeftVector,
                    factory.mecanumConstants.frontLeftVector);
            assertNotSame(draft.pathConstraints, factory.followerPathConstraints);
            assertEquals("runtime-odo", factory.predictorConfig.hardwareMapName);
            assertEquals(0.42,
                    factory.followerConstants.coefficientsTranslationalPIDF.P,
                    0.0);
            assertEquals("runtime-left-front",
                    factory.mecanumConstants.leftFrontMotorName);
            assertEquals(authoredTheta,
                    factory.mecanumConstants.frontLeftVector.getTheta(),
                    0.0);
            assertEquals(1.25,
                    factory.followerPathConstraints.getBrakingStrength(),
                    0.0);

            draft.predictor.hardwareMapName = "mutated-odo";
            draft.followerConstants.coefficientsTranslationalPIDF.P = 9.0;
            draft.mecanumConstants.leftFrontMotorName = "mutated-left-front";
            draft.mecanumConstants.frontLeftVector.rotateVector(0.75);
            draft.pathConstraints.setBrakingStrength(3.0);

            assertEquals("runtime-odo", factory.predictorConfig.hardwareMapName);
            assertEquals(0.42,
                    factory.followerConstants.coefficientsTranslationalPIDF.P,
                    0.0);
            assertEquals("runtime-left-front",
                    factory.mecanumConstants.leftFrontMotorName);
            assertEquals(authoredTheta,
                    factory.mecanumConstants.frontLeftVector.getTheta(),
                    0.0);
            assertEquals(1.25,
                    factory.followerPathConstraints.getBrakingStrength(),
                    0.0);

            factory.followerPathConstraints.setBrakingStrength(4.0);
            PathChain path = runtime.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(0.0, 0.0, 0.0),
                            new Pose(12.0, 3.0, 0.25)
                    ))
                    .build();
            assertEquals(1.25, path.getPath(0).getBrakingStrength(), 0.0);
            assertEquals(0.6, path.getPath(0).getBrakingStartMultiplier(), 0.0);
        } finally {
            PathConstraints.defaultConstraints = originalGlobal;
        }
    }

    @Test
    public void laterFailureStopsReturnedDrivetrainAndPreservesPrimaryFailure() {
        RuntimeException primary = new RuntimeException("pinpoint boom");
        RecordingConstructionFactory factory = new RecordingConstructionFactory();
        factory.predictorFailure = primary;

        IllegalStateException actual = expectConstructionFailure(factory);

        assertSame(primary, actual.getCause());
        assertEquals(1, factory.drivetrain.breakCount);
        assertEquals("mecanum.create", factory.events.get(0));
        assertEquals("pinpoint.create", factory.events.get(1));
        assertEquals("drivetrain.breakFollowing", factory.events.get(2));
        assertEquals(0, primary.getSuppressed().length);
    }

    @Test
    public void laterCleanupFailureIsSuppressedOnOriginalConstructionCause() {
        RuntimeException primary = new RuntimeException("follower graph failed");
        RuntimeException cleanup = new RuntimeException("stop failed");
        RecordingConstructionFactory factory = new RecordingConstructionFactory();
        factory.followerBreakFailure = primary;
        factory.cleanupFailure = cleanup;

        IllegalStateException actual = expectConstructionFailure(factory);

        assertSame(primary, actual.getCause());
        assertEquals(2, factory.drivetrain.breakCount);
        assertTrue(factory.events.indexOf("pinpoint.resetPosAndIMU")
                < factory.events.indexOf("follower.create"));
        assertEquals(1, primary.getSuppressed().length);
        assertSame(cleanup, primary.getSuppressed()[0]);
    }

    private static void assertTopLevelNull(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected required top-level collaborator failure");
        } catch (NullPointerException expected) {
            for (String fragment : fragments) {
                assertTrue(
                        "Expected message containing '" + fragment + "' but got: "
                                + expected.getMessage(),
                        expected.getMessage() != null
                                && expected.getMessage().contains(fragment)
                );
            }
        }
    }

    private static void assertInvalidMass(Runnable action) {
        try {
            action.run();
            fail("Expected invalid configuration to fail before construction");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("followerConstants.mass"));
            assertTrue(expected.getMessage().contains("NaN"));
        }
    }

    private static IllegalStateException expectConstructionFailure(
            RecordingConstructionFactory factory) {
        try {
            PedroPathingRuntime.createForTest(
                    new HardwareMap(null, null),
                    PedroPathingRuntime.Config.defaults(),
                    factory
            );
            fail("Expected construction failure");
            return null;
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static PedroPathingRuntime.Config distinctiveValidConfig() {
        PedroPathingRuntime.Config c = PedroPathingRuntime.Config.defaults();
        c.predictor.hardwareMapName = "runtime-odo";
        c.followerConstants.coefficientsTranslationalPIDF.P = 0.42;
        c.followerConstants.forwardZeroPowerAcceleration = -18.0;
        c.followerConstants.lateralZeroPowerAcceleration = -15.0;
        c.mecanumConstants.leftFrontMotorName = "runtime-left-front";
        c.mecanumConstants.leftRearMotorName = "runtime-left-rear";
        c.mecanumConstants.rightFrontMotorName = "runtime-right-front";
        c.mecanumConstants.rightRearMotorName = "runtime-right-rear";
        c.mecanumConstants.xVelocity = 80.0;
        c.mecanumConstants.yVelocity = 65.0;
        c.mecanumConstants.frontLeftVector = new Vector(1.0, 0.8);
        c.pathConstraints = new PathConstraints(
                0.91,
                42.0,
                1.3,
                0.8,
                90.0,
                1.25,
                17,
                0.6
        );
        return c;
    }

    private static Map<String, Object> vectorGlobals() {
        Map<String, Object> values = new LinkedHashMap<String, Object>();
        try {
            for (Field field : VectorCalculator.class.getDeclaredFields()) {
                if (java.lang.reflect.Modifier.isPublic(field.getModifiers())
                        && java.lang.reflect.Modifier.isStatic(field.getModifiers())) {
                    values.put(field.getName(), field.get(null));
                }
            }
            return values;
        } catch (IllegalAccessException failure) {
            throw new AssertionError(failure);
        }
    }

    private static double[] pathValues(PathConstraints constraints) {
        return new double[] {
                constraints.getTValueConstraint(),
                constraints.getVelocityConstraint(),
                constraints.getTranslationalConstraint(),
                constraints.getHeadingConstraint(),
                constraints.getTimeoutConstraint(),
                constraints.getBrakingStrength(),
                constraints.getBEZIER_CURVE_SEARCH_LIMIT(),
                constraints.getBrakingStart()
        };
    }

    private static void assertPathValues(double[] expected, PathConstraints actual) {
        double[] actualValues = pathValues(actual);
        assertEquals(expected.length, actualValues.length);
        for (int index = 0; index < expected.length; index++) {
            assertEquals("path value " + index, expected[index], actualValues[index], 0.0);
        }
    }

    private static PinpointOdometryPredictor reflectivePredictor(
            PinpointOdometryPredictor.Config config,
            List<String> events) {
        try {
            Class<?> lookupType = Class.forName(
                    PinpointOdometryPredictor.class.getName() + "$PinpointDeviceLookup"
            );
            Class<?> deviceType = Class.forName(
                    PinpointOdometryPredictor.class.getName() + "$PinpointDevice"
            );
            Object device = Proxy.newProxyInstance(
                    deviceType.getClassLoader(),
                    new Class<?>[] {deviceType},
                    (proxy, method, arguments) -> {
                        if (method.getDeclaringClass() == Object.class) {
                            return objectMethod(proxy, method, arguments);
                        }
                        events.add("pinpoint." + method.getName());
                        return defaultValue(method.getReturnType());
                    }
            );
            Object lookup = Proxy.newProxyInstance(
                    lookupType.getClassLoader(),
                    new Class<?>[] {lookupType},
                    (proxy, method, arguments) -> {
                        if (method.getDeclaringClass() == Object.class) {
                            return objectMethod(proxy, method, arguments);
                        }
                        events.add("pinpoint.lookup");
                        return device;
                    }
            );
            Constructor<PinpointOdometryPredictor> constructor =
                    PinpointOdometryPredictor.class.getDeclaredConstructor(
                            lookupType,
                            PinpointOdometryPredictor.Config.class
                    );
            constructor.setAccessible(true);
            return constructor.newInstance(lookup, config);
        } catch (InvocationTargetException failure) {
            Throwable cause = failure.getCause();
            if (cause instanceof RuntimeException) {
                throw (RuntimeException) cause;
            }
            throw new AssertionError(cause);
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError(failure);
        }
    }

    private static Object objectMethod(Object proxy, Method method, Object[] arguments) {
        if (method.getName().equals("toString")) {
            return "PinpointTestProxy";
        }
        if (method.getName().equals("hashCode")) {
            return System.identityHashCode(proxy);
        }
        if (method.getName().equals("equals")) {
            return proxy == arguments[0];
        }
        throw new AssertionError("Unexpected Object method " + method);
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive() || type == Void.TYPE) {
            return null;
        }
        if (type == Boolean.TYPE) {
            return false;
        }
        if (type == Character.TYPE) {
            return '\0';
        }
        if (type == Byte.TYPE) {
            return (byte) 0;
        }
        if (type == Short.TYPE) {
            return (short) 0;
        }
        if (type == Integer.TYPE) {
            return 0;
        }
        if (type == Long.TYPE) {
            return 0L;
        }
        if (type == Float.TYPE) {
            return 0.0f;
        }
        if (type == Double.TYPE) {
            return 0.0;
        }
        throw new AssertionError("Unexpected primitive " + type);
    }

    private static final class RecordingConstructionFactory
            implements PedroPathingRuntime.ConstructionFactory {
        final List<String> events = new ArrayList<String>();
        final RecordingDrivetrain drivetrain = new RecordingDrivetrain(events);
        PinpointOdometryPredictor.Config predictorConfig;
        FollowerConstants followerConstants;
        MecanumConstants mecanumConstants;
        PathConstraints followerPathConstraints;
        RuntimeException predictorFailure;
        RuntimeException followerBreakFailure;
        RuntimeException cleanupFailure;

        @Override
        public Drivetrain createDrivetrain(HardwareMap hardwareMap,
                                           MecanumConstants constants) {
            events.add("mecanum.create");
            mecanumConstants = constants;
            drivetrain.xVelocity = constants.xVelocity;
            drivetrain.yVelocity = constants.yVelocity;
            drivetrain.firstBreakFailure = followerBreakFailure;
            drivetrain.cleanupFailure = cleanupFailure;
            return drivetrain;
        }

        @Override
        public PinpointOdometryPredictor createPredictor(
                HardwareMap hardwareMap,
                PinpointOdometryPredictor.Config config) {
            events.add("pinpoint.create");
            predictorConfig = config;
            if (predictorFailure != null) {
                throw predictorFailure;
            }
            return reflectivePredictor(config, events);
        }

        @Override
        public Follower createFollower(FollowerConstants constants,
                                       PedroPathingPassiveLocalizer localizer,
                                       Drivetrain drivetrain,
                                       PathConstraints constraints) {
            events.add("follower.create");
            followerConstants = constants;
            followerPathConstraints = constraints;
            return new Follower(constants, localizer, drivetrain, constraints);
        }
    }

    private static final class CountingHardwareMap extends HardwareMap {
        int lookupCount;

        CountingHardwareMap() {
            super(null, null);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCount++;
            throw new AssertionError("Unexpected hardware lookup for " + name);
        }
    }

    private static final class RecordingDrivetrain extends Drivetrain {
        private final List<String> events;
        double xVelocity;
        double yVelocity;
        int breakCount;
        RuntimeException firstBreakFailure;
        RuntimeException cleanupFailure;

        RecordingDrivetrain(List<String> events) {
            this.events = events;
        }

        @Override
        public double[] calculateDrive(Vector correctivePower,
                                       Vector headingPower,
                                       Vector pathingPower,
                                       double robotHeading) {
            return new double[] {0.0, 0.0, 0.0, 0.0};
        }

        @Override
        public void updateConstants() {
        }

        @Override
        public void breakFollowing() {
            breakCount++;
            events.add("drivetrain.breakFollowing");
            if (breakCount == 1 && firstBreakFailure != null) {
                throw firstBreakFailure;
            }
            if (cleanupFailure != null) {
                throw cleanupFailure;
            }
        }

        @Override
        public void runDrive(double[] drivePowers) {
        }

        @Override
        public void startTeleopDrive() {
        }

        @Override
        public void startTeleopDrive(boolean brakeMode) {
        }

        @Override
        public double xVelocity() {
            return xVelocity;
        }

        @Override
        public double yVelocity() {
            return yVelocity;
        }

        @Override
        public void setXVelocity(double xMovement) {
            xVelocity = xMovement;
        }

        @Override
        public void setYVelocity(double yMovement) {
            yVelocity = yMovement;
        }

        @Override
        public double getVoltage() {
            return 12.0;
        }

        @Override
        public String debugString() {
            return "RecordingDrivetrain";
        }
    }
}
