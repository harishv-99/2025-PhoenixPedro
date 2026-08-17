package edu.ftcphoenix.robots.examples.pedro;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Locks the Basic Pedro profile, Config, host, capability, and root API contracts. */
public final class BasicPedroProfileAndApiTest {

    private static final String[] FOLLOWER_MUTABLE_OBJECT_FIELDS = {
            "coefficientsTranslationalPIDF",
            "integralTranslational",
            "coefficientsHeadingPIDF",
            "coefficientsDrivePIDF",
            "coefficientsSecondaryTranslationalPIDF",
            "integralSecondaryTranslational",
            "coefficientsSecondaryHeadingPIDF",
            "coefficientsSecondaryDrivePIDF",
            "predictiveBrakingCoefficients"
    };

    @Test
    public void profileAndIntakeConfigExposeOnlyTheApprovedAuthoringSurface()
            throws Exception {
        assertTrue(Modifier.isPublic(BasicPedroProfile.class.getModifiers()));
        assertTrue(Modifier.isFinal(BasicPedroProfile.class.getModifiers()));
        Constructor<?>[] profileConstructors = BasicPedroProfile.class.getDeclaredConstructors();
        assertEquals(1, profileConstructors.length);
        assertEquals(0, profileConstructors[0].getParameterTypes().length);
        assertTrue(Modifier.isPrivate(profileConstructors[0].getModifiers()));
        assertEquals(
                fields(
                        "pedro", PedroPathingRuntime.Config.class,
                        "intake", BasicPedroAutoMechanism.Config.class,
                        "allowRobotMotion", boolean.class),
                publicDeclaredFields(BasicPedroProfile.class));
        assertEquals(
                setOf(methodKey("current", BasicPedroProfile.class)),
                publicDeclaredMethods(BasicPedroProfile.class));
        Method current = BasicPedroProfile.class.getDeclaredMethod("current");
        assertTrue(Modifier.isStatic(current.getModifiers()));

        Class<?> configType = BasicPedroAutoMechanism.Config.class;
        assertTrue(Modifier.isPublic(configType.getModifiers()));
        assertTrue(Modifier.isStatic(configType.getModifiers()));
        assertTrue(Modifier.isFinal(configType.getModifiers()));
        Constructor<?>[] configConstructors = configType.getDeclaredConstructors();
        assertEquals(1, configConstructors.length);
        assertEquals(0, configConstructors[0].getParameterTypes().length);
        assertTrue(Modifier.isPrivate(configConstructors[0].getModifiers()));
        assertEquals(
                fields(
                        "motorName", String.class,
                        "direction", Direction.class,
                        "collectPower", double.class),
                publicDeclaredFields(configType));
        assertEquals(
                setOf(methodKey("defaults", BasicPedroAutoMechanism.Config.class)),
                publicDeclaredMethods(configType));
        Method defaults = configType.getDeclaredMethod("defaults");
        assertTrue(Modifier.isStatic(defaults.getModifiers()));
    }

    @Test
    public void exampleMechanismAndRobotExposeOnlyTheApprovedConstructionPaths()
            throws Exception {
        assertTrue(Modifier.isPublic(BasicPedroAutoExample.class.getModifiers()));
        assertTrue(Modifier.isFinal(BasicPedroAutoExample.class.getModifiers()));
        assertTrue(publicDeclaredFields(BasicPedroAutoExample.class).isEmpty());
        assertEquals(2, BasicPedroAutoExample.class.getDeclaredConstructors().length);
        Constructor<?> exampleProduction = BasicPedroAutoExample.class.getDeclaredConstructor();
        assertTrue(Modifier.isPublic(exampleProduction.getModifiers()));
        Constructor<?> exampleTest = BasicPedroAutoExample.class.getDeclaredConstructor(
                Function.class);
        assertPackagePrivate(exampleTest.getModifiers());
        assertTrue(publicDeclaredMethods(BasicPedroAutoExample.class).isEmpty());

        assertTrue(Modifier.isPublic(BasicPedroAutoMechanism.class.getModifiers()));
        assertTrue(Modifier.isFinal(BasicPedroAutoMechanism.class.getModifiers()));
        assertTrue(publicDeclaredFields(BasicPedroAutoMechanism.class).isEmpty());
        assertEquals(2, BasicPedroAutoMechanism.class.getDeclaredConstructors().length);
        Constructor<?> ordinaryMechanism = BasicPedroAutoMechanism.class.getDeclaredConstructor(
                HardwareMap.class,
                BasicPedroAutoMechanism.Config.class);
        assertTrue(Modifier.isPublic(ordinaryMechanism.getModifiers()));
        Constructor<?> neutralMechanism = BasicPedroAutoMechanism.class.getDeclaredConstructor(
                Plant.class);
        assertPackagePrivate(neutralMechanism.getModifiers());
        assertEquals(
                setOf(
                        methodKey("collectTask", Task.class, double.class),
                        methodKey("idleTask", Task.class),
                        methodKey("update", void.class, LoopClock.class),
                        methodKey("stop", void.class)),
                publicDeclaredMethods(BasicPedroAutoMechanism.class));

        assertTrue(Modifier.isPublic(BasicPedroAutoRobot.class.getModifiers()));
        assertTrue(Modifier.isFinal(BasicPedroAutoRobot.class.getModifiers()));
        assertTrue(publicDeclaredFields(BasicPedroAutoRobot.class).isEmpty());
        assertEquals(2, BasicPedroAutoRobot.class.getDeclaredConstructors().length);
        Constructor<?> ordinaryRobot = BasicPedroAutoRobot.class.getDeclaredConstructor(
                RobotProgram.class,
                HardwareMap.class,
                BasicPedroProfile.class);
        assertTrue(Modifier.isPublic(ordinaryRobot.getModifiers()));
        Constructor<?> componentRobot = BasicPedroAutoRobot.class.getDeclaredConstructor(
                RobotProgram.class,
                AbsolutePoseEstimator.class,
                DriveCommandSink.class,
                Runnable.class,
                Supplier.class,
                Task.class);
        assertPackagePrivate(componentRobot.getModifiers());
        assertEquals(
                setOf(
                        methodKey("pedroStartPose", Pose.class),
                        methodKey("isRootComplete", boolean.class),
                        methodKey("rootOutcome", TaskOutcome.class)),
                publicDeclaredMethods(BasicPedroAutoRobot.class));
        Method routeStatus = BasicPedroAutoRobot.class.getDeclaredMethod("latestRouteStatus");
        assertPackagePrivate(routeStatus.getModifiers());
        assertEquals(RouteStatus.class, routeStatus.getReturnType());
    }

    @Test
    public void currentReturnsFreshCompleteOwnerLocalGraphs() throws Exception {
        BasicPedroProfile first = BasicPedroProfile.current();
        BasicPedroProfile second = BasicPedroProfile.current();

        assertNotSame(first, second);
        assertNotSame(first.pedro, second.pedro);
        assertNotSame(first.pedro.predictor, second.pedro.predictor);
        assertNotSame(first.pedro.followerConstants, second.pedro.followerConstants);
        assertNotSame(first.pedro.mecanumConstants, second.pedro.mecanumConstants);
        assertNotSame(first.pedro.mecanumConstants.frontLeftVector,
                second.pedro.mecanumConstants.frontLeftVector);
        assertNotSame(first.pedro.pathConstraints, second.pedro.pathConstraints);
        assertNotSame(first.intake, second.intake);
        assertSame(first.pedro.fieldTransform, second.pedro.fieldTransform);
        for (String fieldName : FOLLOWER_MUTABLE_OBJECT_FIELDS) {
            assertNotSame(
                    fieldName,
                    FollowerConstants.class.getField(fieldName).get(first.pedro.followerConstants),
                    FollowerConstants.class.getField(fieldName).get(second.pedro.followerConstants));
        }

        assertCurrentValues(first);
        assertCurrentValues(second);

        first.pedro.predictor.hardwareMapName = "changed odo";
        first.pedro.followerConstants.coefficientsDrivePIDF.T = 0.123456;
        first.pedro.mecanumConstants.frontLeftVector.setTheta(1.2345);
        first.pedro.pathConstraints.setTimeoutConstraint(54321.0);
        first.intake.motorName = "changed intake";
        first.intake.direction = Direction.REVERSE;
        first.intake.collectPower = -0.75;
        first.allowRobotMotion = true;

        assertCurrentValues(second);
    }

    @Test
    public void currentIgnoresAndDoesNotMutatePedroGlobalPathDefaults() {
        PathConstraints original = PathConstraints.defaultConstraints;
        PathConstraints poison = new PathConstraints(
                0.21,
                22.0,
                23.0,
                0.24,
                25.0,
                2.6,
                27,
                2.8);
        PathConstraints.defaultConstraints = poison;
        try {
            BasicPedroProfile profile = BasicPedroProfile.current();

            assertNotSame(poison, profile.pedro.pathConstraints);
            assertPathValues(
                    profile.pedro.pathConstraints,
                    0.995,
                    0.1,
                    0.1,
                    0.007,
                    100.0,
                    1.0,
                    10,
                    1.0);
            assertSame(poison, PathConstraints.defaultConstraints);
        } finally {
            PathConstraints.defaultConstraints = original;
        }
    }

    @Test
    public void intakeDefaultsAreFreshCompleteAndMutationIsolated() {
        BasicPedroAutoMechanism.Config first = BasicPedroAutoMechanism.Config.defaults();
        BasicPedroAutoMechanism.Config second = BasicPedroAutoMechanism.Config.defaults();

        assertNotSame(first, second);
        assertIntakeValues(first);
        assertIntakeValues(second);

        first.motorName = "changed";
        first.direction = Direction.REVERSE;
        first.collectPower = -0.9;

        assertIntakeValues(second);
    }

    @Test
    public void runtimeOwnerSnapshotCannotBeRetunedThroughTheBasicProfileDraft() {
        BasicPedroProfile profile = BasicPedroProfile.current();
        PedroPathingRuntime.Config owned = profile.pedro.validatedCopy(
                "BasicPedroProfileAndApiTest.profile.pedro");

        profile.pedro.predictor.hardwareMapName = "changed odo";
        profile.pedro.followerConstants.coefficientsDrivePIDF.T = 0.123456;
        profile.pedro.mecanumConstants.leftFrontMotorName = "changed drive";
        profile.pedro.mecanumConstants.frontLeftVector.setTheta(1.2345);
        profile.pedro.pathConstraints.setTimeoutConstraint(54321.0);

        assertEquals("odo", owned.predictor.hardwareMapName);
        assertEquals(
                new FollowerConstants().coefficientsDrivePIDF.T,
                owned.followerConstants.coefficientsDrivePIDF.T,
                0.0);
        assertEquals("frontLeftMotor", owned.mecanumConstants.leftFrontMotorName);
        assertEquals(
                new MecanumConstants().frontLeftVector.getTheta(),
                owned.mecanumConstants.frontLeftVector.getTheta(),
                0.0);
        assertEquals(100.0, owned.pathConstraints.getTimeoutConstraint(), 0.0);
    }

    private static void assertCurrentValues(BasicPedroProfile profile) throws Exception {
        assertFalse(profile.allowRobotMotion);
        assertIntakeValues(profile.intake);

        PinpointOdometryPredictor.Config predictor = profile.pedro.predictor;
        assertEquals("odo", predictor.hardwareMapName);
        assertEquals(0.0, predictor.forwardPodOffsetLeftInches, 0.0);
        assertEquals(0.0, predictor.strafePodOffsetForwardInches, 0.0);
        assertEquals(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                predictor.forwardPodDirection);
        assertEquals(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                predictor.strafePodDirection);
        assertNull(predictor.yawScalar);
        assertEquals(0.75, predictor.quality, 0.0);
        AtomicReference<GoBildaPinpointDriver.GoBildaOdometryPods> selectedPod =
                new AtomicReference<GoBildaPinpointDriver.GoBildaOdometryPods>();
        AtomicReference<Double> customTicksPerInch = new AtomicReference<Double>();
        predictor.encoderResolution.applyTo(selectedPod::set, customTicksPerInch::set);
        assertEquals(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                selectedPod.get());
        assertNull(customTicksPerInch.get());

        FollowerConstants followerDefaults = new FollowerConstants();
        assertPublicPrimitiveFieldsEqual(followerDefaults, profile.pedro.followerConstants);
        for (String fieldName : FOLLOWER_MUTABLE_OBJECT_FIELDS) {
            assertPublicPrimitiveFieldsEqual(
                    FollowerConstants.class.getField(fieldName).get(followerDefaults),
                    FollowerConstants.class.getField(fieldName).get(
                            profile.pedro.followerConstants));
        }

        MecanumConstants expectedMecanum = new MecanumConstants();
        expectedMecanum.leftFrontMotorName = "frontLeftMotor";
        expectedMecanum.leftRearMotorName = "backLeftMotor";
        expectedMecanum.rightFrontMotorName = "frontRightMotor";
        expectedMecanum.rightRearMotorName = "backRightMotor";
        expectedMecanum.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        expectedMecanum.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        expectedMecanum.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        expectedMecanum.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        expectedMecanum.maxPower = 0.25;
        expectedMecanum.useBrakeModeInTeleOp = true;
        MecanumConstants actualMecanum = profile.pedro.mecanumConstants;
        assertPublicPrimitiveFieldsEqual(expectedMecanum, actualMecanum);
        assertEquals(expectedMecanum.leftFrontMotorName, actualMecanum.leftFrontMotorName);
        assertEquals(expectedMecanum.leftRearMotorName, actualMecanum.leftRearMotorName);
        assertEquals(expectedMecanum.rightFrontMotorName, actualMecanum.rightFrontMotorName);
        assertEquals(expectedMecanum.rightRearMotorName, actualMecanum.rightRearMotorName);
        assertSame(expectedMecanum.leftFrontMotorDirection,
                actualMecanum.leftFrontMotorDirection);
        assertSame(expectedMecanum.leftRearMotorDirection,
                actualMecanum.leftRearMotorDirection);
        assertSame(expectedMecanum.rightFrontMotorDirection,
                actualMecanum.rightFrontMotorDirection);
        assertSame(expectedMecanum.rightRearMotorDirection,
                actualMecanum.rightRearMotorDirection);
        assertEquals(expectedMecanum.frontLeftVector.getMagnitude(),
                actualMecanum.frontLeftVector.getMagnitude(), 0.0);
        assertEquals(expectedMecanum.frontLeftVector.getTheta(),
                actualMecanum.frontLeftVector.getTheta(), 0.0);

        assertPathValues(
                profile.pedro.pathConstraints,
                0.995,
                0.1,
                0.1,
                0.007,
                100.0,
                1.0,
                10,
                1.0);
        assertSame(PedroFieldTransform.decodeInvertedFtc(), profile.pedro.fieldTransform);
    }

    private static void assertIntakeValues(BasicPedroAutoMechanism.Config config) {
        assertEquals("intakeMotor", config.motorName);
        assertEquals(Direction.FORWARD, config.direction);
        assertEquals(0.20, config.collectPower, 0.0);
    }

    private static void assertPathValues(PathConstraints value,
                                         double t,
                                         double velocity,
                                         double translation,
                                         double heading,
                                         double timeout,
                                         double brakingStrength,
                                         int searchLimit,
                                         double brakingStart) {
        assertEquals(t, value.getTValueConstraint(), 0.0);
        assertEquals(velocity, value.getVelocityConstraint(), 0.0);
        assertEquals(translation, value.getTranslationalConstraint(), 0.0);
        assertEquals(heading, value.getHeadingConstraint(), 0.0);
        assertEquals(timeout, value.getTimeoutConstraint(), 0.0);
        assertEquals(brakingStrength, value.getBrakingStrength(), 0.0);
        assertEquals(searchLimit, value.getBEZIER_CURVE_SEARCH_LIMIT());
        assertEquals(brakingStart, value.getBrakingStart(), 0.0);
    }

    private static void assertPublicPrimitiveFieldsEqual(Object expected, Object actual)
            throws Exception {
        assertEquals(expected.getClass(), actual.getClass());
        for (Field field : expected.getClass().getFields()) {
            if (!Modifier.isStatic(field.getModifiers()) && field.getType().isPrimitive()) {
                assertEquals(field.getName(), field.get(expected), field.get(actual));
            }
        }
    }

    private static Map<String, String> publicDeclaredFields(Class<?> type) {
        Map<String, String> fields = new LinkedHashMap<String, String>();
        for (Field field : type.getDeclaredFields()) {
            if (Modifier.isPublic(field.getModifiers())) {
                fields.put(
                        field.getName(),
                        fieldKey(field.getType(), Modifier.isStatic(field.getModifiers())));
            }
        }
        return fields;
    }

    private static Map<String, String> fields(Object... nameTypePairs) {
        Map<String, String> result = new LinkedHashMap<String, String>();
        for (int index = 0; index < nameTypePairs.length; index += 2) {
            result.put(
                    (String) nameTypePairs[index],
                    fieldKey((Class<?>) nameTypePairs[index + 1], false));
        }
        return result;
    }

    private static String fieldKey(Class<?> type, boolean isStatic) {
        return type.getName() + (isStatic ? ":static" : ":instance");
    }

    private static Set<String> publicDeclaredMethods(Class<?> type) {
        Set<String> methods = new LinkedHashSet<String>();
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                methods.add(methodKey(
                        method.getName(),
                        method.getReturnType(),
                        method.getParameterTypes()));
            }
        }
        return methods;
    }

    private static Set<String> setOf(String... values) {
        return new LinkedHashSet<String>(Arrays.asList(values));
    }

    private static String methodKey(String name,
                                    Class<?> returnType,
                                    Class<?>... parameterTypes) {
        return name + Arrays.toString(parameterTypes) + "->" + returnType.getName();
    }

    private static void assertPackagePrivate(int modifiers) {
        assertFalse(Modifier.isPublic(modifiers));
        assertFalse(Modifier.isProtected(modifiers));
        assertFalse(Modifier.isPrivate(modifiers));
    }
}
