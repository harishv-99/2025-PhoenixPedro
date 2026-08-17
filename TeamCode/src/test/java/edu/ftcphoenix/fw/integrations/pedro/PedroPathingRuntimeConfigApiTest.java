package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
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

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.localization.MotionPredictor;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Version locks the deliberately narrow Phoenix owner and pinned Pedro 2.1.2 field closure. */
public final class PedroPathingRuntimeConfigApiTest {

    @Test
    public void configAndRuntimeExposeOnlyTheApprovedProductionSurface() throws Exception {
        Class<PedroPathingRuntime.Config> configType = PedroPathingRuntime.Config.class;
        assertTrue(Modifier.isPublic(configType.getModifiers()));
        assertTrue(Modifier.isStatic(configType.getModifiers()));
        assertTrue(Modifier.isFinal(configType.getModifiers()));
        assertEquals(
                fields(
                        "predictor", PinpointOdometryPredictor.Config.class,
                        "followerConstants", FollowerConstants.class,
                        "mecanumConstants", MecanumConstants.class,
                        "pathConstraints", PathConstraints.class,
                        "fieldTransform", PedroFieldTransform.class
                ),
                publicInstanceFields(configType)
        );
        Constructor<?>[] configConstructors = configType.getDeclaredConstructors();
        assertEquals(1, configConstructors.length);
        assertEquals(0, configConstructors[0].getParameterTypes().length);
        assertTrue(Modifier.isPrivate(configConstructors[0].getModifiers()));
        assertEquals(
                setOf(
                        methodKey("defaults", PedroPathingRuntime.Config.class),
                        methodKey("copy", PedroPathingRuntime.Config.class),
                        methodKey(
                                "validatedCopy",
                                PedroPathingRuntime.Config.class,
                                String.class
                        )
                ),
                publicDeclaredMethods(configType)
        );
        assertPublicStatic(configType, "defaults");
        assertPublicInstance(configType, "copy");
        assertPublicInstance(configType, "validatedCopy", String.class);

        assertEquals(0, publicConstructors(PedroPathingRuntime.class));
        assertEquals(
                setOf(
                        methodKey(
                                "create",
                                PedroPathingRuntime.class,
                                HardwareMap.class,
                                PedroPathingRuntime.Config.class
                        ),
                        methodKey("motionPredictor", MotionPredictor.class),
                        methodKey("driveAdapter", PedroPathingDriveAdapter.class),
                        methodKey("pathBuilder", PathBuilder.class),
                        methodKey("currentPedroPose", Pose.class),
                        methodKey("setStartingPose", void.class, Pose.class)
                ),
                publicDeclaredMethods(PedroPathingRuntime.class)
        );
        assertPublicStatic(
                PedroPathingRuntime.class,
                "create",
                HardwareMap.class,
                PedroPathingRuntime.Config.class
        );
        assertPublicInstance(PedroPathingRuntime.class, "motionPredictor");
        assertPublicInstance(PedroPathingRuntime.class, "driveAdapter");
        assertPublicInstance(PedroPathingRuntime.class, "pathBuilder");
        assertPublicInstance(PedroPathingRuntime.class, "currentPedroPose");
        assertPublicInstance(PedroPathingRuntime.class, "setStartingPose", Pose.class);
        assertNoDeclaredMethod(PedroPathingRuntime.class, "follower");
        assertNoDeclaredMethod(
                PedroPathingRuntime.class,
                "create",
                HardwareMap.class,
                PinpointOdometryPredictor.class,
                FollowerConstants.class,
                MecanumConstants.class,
                PathConstraints.class,
                PedroFieldTransform.class
        );
    }

    @Test
    public void adapterRetainsOnlyTheCompletedFollowerAdvancedConstructor() throws Exception {
        Constructor<?>[] constructors = PedroPathingDriveAdapter.class.getConstructors();
        assertEquals(1, constructors.length);
        assertArrayEquals(
                new Class<?>[]{Follower.class},
                constructors[0].getParameterTypes()
        );
        assertEquals(
                setOf(
                        methodKey("getLatestRouteStatus", RouteStatus.class),
                        methodKey("update", void.class, LoopClock.class),
                        methodKey("follow", RouteExecution.class, PathChain.class),
                        methodKey(
                                "follow",
                                RouteExecution.class,
                                PathChain.class,
                                boolean.class
                        ),
                        methodKey("drive", void.class, DriveSignal.class),
                        methodKey("stop", void.class)
                ),
                publicDeclaredMethods(PedroPathingDriveAdapter.class)
        );
        assertNoDeclaredMethod(PedroPathingDriveAdapter.class, "follower");
    }

    @Test
    public void pinnedFollowerAndMecanumFieldClosuresAreExact() {
        assertEquals(
                fields(
                        "coefficientsTranslationalPIDF", PIDFCoefficients.class,
                        "integralTranslational", PIDFCoefficients.class,
                        "coefficientsHeadingPIDF", PIDFCoefficients.class,
                        "coefficientsDrivePIDF", FilteredPIDFCoefficients.class,
                        "coefficientsSecondaryTranslationalPIDF", PIDFCoefficients.class,
                        "integralSecondaryTranslational", PIDFCoefficients.class,
                        "headingPIDFSwitch", double.class,
                        "coefficientsSecondaryHeadingPIDF", PIDFCoefficients.class,
                        "drivePIDFSwitch", double.class,
                        "coefficientsSecondaryDrivePIDF", FilteredPIDFCoefficients.class,
                        "predictiveBrakingCoefficients", PredictiveBrakingCoefficients.class,
                        "usePredictiveBraking", boolean.class,
                        "holdPointTranslationalScaling", double.class,
                        "holdPointHeadingScaling", double.class,
                        "BEZIER_CURVE_SEARCH_LIMIT", int.class,
                        "useSecondaryTranslationalPIDF", boolean.class,
                        "useSecondaryHeadingPIDF", boolean.class,
                        "useSecondaryDrivePIDF", boolean.class,
                        "translationalPIDFSwitch", double.class,
                        "turnHeadingErrorThreshold", double.class,
                        "centripetalScaling", double.class,
                        "automaticHoldEnd", boolean.class,
                        "mass", double.class,
                        "forwardZeroPowerAcceleration", double.class,
                        "lateralZeroPowerAcceleration", double.class,
                        "driveKalmanFilterModelCovariance", double.class,
                        "driveKalmanFilterDataCovariance", double.class,
                        "stuckVelocity", double.class,
                        "stuckTValueLow", double.class,
                        "stuckTValueHigh", double.class,
                        "stuckTimeout", double.class
                ),
                publicInstanceFields(FollowerConstants.class)
        );
        assertEquals(
                fields(
                        "xVelocity", double.class,
                        "yVelocity", double.class,
                        "frontLeftVector", Vector.class,
                        "maxPower", double.class,
                        "leftFrontMotorName", String.class,
                        "leftRearMotorName", String.class,
                        "rightFrontMotorName", String.class,
                        "rightRearMotorName", String.class,
                        "leftFrontMotorDirection", DcMotorSimple.Direction.class,
                        "leftRearMotorDirection", DcMotorSimple.Direction.class,
                        "rightFrontMotorDirection", DcMotorSimple.Direction.class,
                        "rightRearMotorDirection", DcMotorSimple.Direction.class,
                        "motorCachingThreshold", double.class,
                        "useBrakeModeInTeleOp", boolean.class,
                        "useVoltageCompensation", boolean.class,
                        "nominalVoltage", double.class,
                        "staticFrictionCoefficient", double.class
                ),
                publicInstanceFields(MecanumConstants.class)
        );
    }

    @Test
    public void pinnedCoefficientConstraintAndGlobalClosuresAreExact() {
        assertEquals(
                fields(
                        "P", double.class,
                        "I", double.class,
                        "D", double.class,
                        "F", double.class
                ),
                publicInstanceFields(PIDFCoefficients.class)
        );
        assertEquals(
                fields(
                        "P", double.class,
                        "I", double.class,
                        "D", double.class,
                        "F", double.class,
                        "T", double.class
                ),
                publicInstanceFields(FilteredPIDFCoefficients.class)
        );
        assertEquals(
                fields(
                        "kLinearBraking", double.class,
                        "kQuadraticFriction", double.class,
                        "P", double.class,
                        "maximumBrakingPower", double.class
                ),
                publicInstanceFields(PredictiveBrakingCoefficients.class)
        );
        assertEquals(
                fields(
                        "velocityConstraint", double.class,
                        "translationalConstraint", double.class,
                        "headingConstraint", double.class,
                        "tValueConstraint", double.class,
                        "timeoutConstraint", double.class,
                        "brakingStrength", double.class,
                        "brakingStart", double.class,
                        "BEZIER_CURVE_SEARCH_LIMIT", int.class
                ),
                declaredInstanceFields(PathConstraints.class)
        );
        assertEquals(
                fields(
                        "drivePIDFSwitch", double.class,
                        "headingPIDFSwitch", double.class,
                        "translationalPIDFSwitch", double.class,
                        "useSecondaryDrivePID", boolean.class,
                        "useSecondaryHeadingPID", boolean.class,
                        "useSecondaryTranslationalPID", boolean.class
                ),
                publicStaticFields(VectorCalculator.class)
        );
    }

    private static Map<String, Class<?>> publicInstanceFields(Class<?> type) {
        return selectedFields(type.getFields(), Modifier.PUBLIC, Modifier.STATIC);
    }

    private static Map<String, Class<?>> publicStaticFields(Class<?> type) {
        Map<String, Class<?>> result = new LinkedHashMap<String, Class<?>>();
        for (Field field : type.getDeclaredFields()) {
            int modifiers = field.getModifiers();
            if (Modifier.isPublic(modifiers) && Modifier.isStatic(modifiers)) {
                result.put(field.getName(), field.getType());
            }
        }
        return result;
    }

    private static Map<String, Class<?>> declaredInstanceFields(Class<?> type) {
        return selectedFields(type.getDeclaredFields(), 0, Modifier.STATIC);
    }

    private static Map<String, Class<?>> selectedFields(Field[] fields,
                                                         int requiredModifiers,
                                                         int rejectedModifiers) {
        Map<String, Class<?>> result = new LinkedHashMap<String, Class<?>>();
        for (Field field : fields) {
            int modifiers = field.getModifiers();
            if ((modifiers & requiredModifiers) == requiredModifiers
                    && (modifiers & rejectedModifiers) == 0) {
                result.put(field.getName(), field.getType());
            }
        }
        return result;
    }

    private static Map<String, Class<?>> fields(Object... entries) {
        Map<String, Class<?>> result = new LinkedHashMap<String, Class<?>>();
        for (int i = 0; i < entries.length; i += 2) {
            result.put((String) entries[i], (Class<?>) entries[i + 1]);
        }
        return result;
    }

    private static int publicConstructors(Class<?> type) {
        int count = 0;
        for (Constructor<?> constructor : type.getDeclaredConstructors()) {
            if (Modifier.isPublic(constructor.getModifiers())) {
                count++;
            }
        }
        return count;
    }

    private static Set<String> publicDeclaredMethods(Class<?> type) {
        Set<String> result = new LinkedHashSet<String>();
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                result.add(methodKey(
                        method.getName(),
                        method.getReturnType(),
                        method.getParameterTypes()
                ));
            }
        }
        return result;
    }

    private static String methodKey(String name,
                                    Class<?> returnType,
                                    Class<?>... parameterTypes) {
        return name + Arrays.toString(parameterTypes) + "->" + returnType.getName();
    }

    private static Set<String> setOf(String... values) {
        return new LinkedHashSet<String>(Arrays.asList(values));
    }

    private static void assertPublicStatic(Class<?> type,
                                           String name,
                                           Class<?>... parameterTypes)
            throws NoSuchMethodException {
        Method method = type.getDeclaredMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertTrue(Modifier.isStatic(method.getModifiers()));
    }

    private static void assertPublicInstance(Class<?> type,
                                             String name,
                                             Class<?>... parameterTypes)
            throws NoSuchMethodException {
        Method method = type.getDeclaredMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertFalse(Modifier.isStatic(method.getModifiers()));
    }

    private static void assertNoDeclaredMethod(Class<?> type,
                                               String name,
                                               Class<?>... parameterTypes) {
        try {
            type.getDeclaredMethod(name, parameterTypes);
            fail("Expected " + type.getSimpleName() + "." + name + " to be absent");
        } catch (NoSuchMethodException expected) {
            assertFalse(expected.getMessage() == null && name.isEmpty());
        }
    }
}
