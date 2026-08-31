package edu.ftcsushi.fw.ftc.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.stream.Collectors;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down CONFIG-04's single Pinpoint configuration and runtime API grammar. */
public final class PinpointOdometryPredictorConfigApiTest {

    @Test
    public void configHasEightFieldsAndOnlyDefaultsCopyAndValidatedCopy() throws Exception {
        Class<PinpointOdometryPredictor.Config> type = PinpointOdometryPredictor.Config.class;

        assertTrue(Modifier.isFinal(type.getModifiers()));
        Constructor<?>[] constructors = type.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        Set<String> publicFields = Arrays.stream(type.getDeclaredFields())
                .filter(field -> Modifier.isPublic(field.getModifiers()))
                .map(Field::getName)
                .collect(Collectors.toSet());
        assertEquals(8, publicFields.size());
        assertEquals(setOf(
                "hardwareMapName",
                "forwardPodOffsetLeftInches",
                "strafePodOffsetForwardInches",
                "encoderResolution",
                "forwardPodDirection",
                "strafePodDirection",
                "yawScalar",
                "quality"
        ), publicFields);
        assertEquals(
                PinpointOdometryPredictor.EncoderResolution.class,
                type.getDeclaredField("encoderResolution").getType()
        );

        List<Method> publicMethodList = Arrays.stream(type.getDeclaredMethods())
                .filter(method -> Modifier.isPublic(method.getModifiers()))
                .collect(Collectors.toList());
        assertEquals(3, publicMethodList.size());
        Set<String> publicMethods = publicMethodList.stream()
                .map(Method::getName)
                .collect(Collectors.toSet());
        assertEquals(setOf("defaults", "copy", "validatedCopy"), publicMethods);

        assertAbsent(type, "of", String.class, double.class, double.class);
        assertAbsent(type, "withHardwareMapName", String.class);
        assertAbsent(type, "withOffsets", double.class, double.class);
        assertAbsent(type, "withResetOnInit", boolean.class);
        assertAbsent(type, "withResetWaitMs", long.class);
        assertAbsent(type, "withEncoderPods", GoBildaPinpointDriver.GoBildaOdometryPods.class);
        assertAbsent(type, "withCustomEncoderResolutionTicksPerInch", Double.class);
        assertAbsent(type, "withForwardPodDirection",
                GoBildaPinpointDriver.EncoderDirection.class);
        assertAbsent(type, "withStrafePodDirection",
                GoBildaPinpointDriver.EncoderDirection.class);
        assertAbsent(type, "withYawScalar", Double.class);
        assertAbsent(type, "withQuality", double.class);
        assertAbsent(type, "debugDump", DebugSink.class, String.class);
    }

    @Test
    public void encoderResolutionIsImmutableTaggedVisitor() throws Exception {
        Class<PinpointOdometryPredictor.EncoderResolution> type =
                PinpointOdometryPredictor.EncoderResolution.class;

        assertTrue(Modifier.isFinal(type.getModifiers()));
        for (Field field : type.getDeclaredFields()) {
            assertTrue(field.getName(), Modifier.isPrivate(field.getModifiers()));
            assertTrue(field.getName(), Modifier.isFinal(field.getModifiers()));
        }
        for (Constructor<?> constructor : type.getDeclaredConstructors()) {
            assertTrue(Modifier.isPrivate(constructor.getModifiers()));
        }

        Method preset = type.getDeclaredMethod(
                "forGoBildaPod",
                GoBildaPinpointDriver.GoBildaOdometryPods.class
        );
        Method custom = type.getDeclaredMethod("ticksPerInch", double.class);
        Method apply = type.getDeclaredMethod("applyTo", Consumer.class, DoubleConsumer.class);
        Method toString = type.getDeclaredMethod("toString");
        assertTrue(Modifier.isPublic(preset.getModifiers()));
        assertTrue(Modifier.isStatic(preset.getModifiers()));
        assertTrue(Modifier.isPublic(custom.getModifiers()));
        assertTrue(Modifier.isStatic(custom.getModifiers()));
        assertTrue(Modifier.isPublic(apply.getModifiers()));
        assertFalse(Modifier.isStatic(apply.getModifiers()));
        assertTrue(Modifier.isPublic(toString.getModifiers()));
        assertEquals(4L, Arrays.stream(type.getDeclaredMethods())
                .filter(method -> Modifier.isPublic(method.getModifiers()))
                .count());
    }

    @Test
    public void predictorKeepsOnePublicConstructorAndTenPublicOperations() throws Exception {
        long publicConstructors = Arrays.stream(PinpointOdometryPredictor.class.getDeclaredConstructors())
                .filter(constructor -> Modifier.isPublic(constructor.getModifiers()))
                .count();
        assertEquals(1L, publicConstructors);
        Constructor<PinpointOdometryPredictor> constructor =
                PinpointOdometryPredictor.class.getDeclaredConstructor(
                        HardwareMap.class,
                        PinpointOdometryPredictor.Config.class
                );
        assertTrue(Modifier.isPublic(constructor.getModifiers()));

        List<Method> publicMethodList = Arrays.stream(
                        PinpointOdometryPredictor.class.getDeclaredMethods())
                .filter(method -> Modifier.isPublic(method.getModifiers()))
                .filter(method -> !method.isSynthetic())
                .collect(Collectors.toList());
        assertEquals(10, publicMethodList.size());
        Set<String> publicMethods = publicMethodList.stream()
                .map(Method::getName)
                .collect(Collectors.toSet());
        assertEquals(setOf(
                "update",
                "getEstimate",
                "trajectorySegmentId",
                "getLatestMotionDelta",
                "getKinematicSnapshot",
                "lastDeviceStatus",
                "setPose",
                "resetPosAndIMU",
                "recalibrateIMU",
                "debugDump"
        ), publicMethods);

        assertAbsent(PinpointOdometryPredictor.class, "getDriver");
        assertAbsent(PinpointOdometryPredictor.class, "config");
        PinpointOdometryPredictor.class.getDeclaredMethod("update", LoopClock.class);
        PinpointOdometryPredictor.class.getDeclaredMethod("setPose", Pose2d.class);
    }

    private static Set<String> setOf(String... values) {
        return Arrays.stream(values).collect(Collectors.toSet());
    }

    private static void assertAbsent(Class<?> owner, String name, Class<?>... parameters) {
        try {
            owner.getDeclaredMethod(name, parameters);
            fail(owner.getSimpleName() + "." + name + " must be absent");
        } catch (NoSuchMethodException expected) {
            // Expected removal.
        }
    }
}
