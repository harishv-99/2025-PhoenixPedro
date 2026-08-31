package edu.ftcsushi.robots.phoenix;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

/** Locks Phoenix's cohesive Auto policy values, public surface, and raw-copy semantics. */
public final class PhoenixAutoConfigTest {

    private static final List<String> FIELD_NAMES = Arrays.asList(
            "parkTakeoverElapsedSec",
            "routeTimeoutSec",
            "aimHeadingToleranceDeg",
            "aimTimeoutSec",
            "aimMaxNoGuidanceSec",
            "waitForTargetSec",
            "waitForShotCompleteSec",
            "pedroIntegrationTestDistanceIn"
    );

    @Test
    public void publicSurfaceIsDefaultsPlusRawCopyOnly() throws Exception {
        assertTrue(Modifier.isPublic(PhoenixAutoConfig.class.getModifiers()));
        assertTrue(Modifier.isFinal(PhoenixAutoConfig.class.getModifiers()));

        Constructor<?>[] constructors = PhoenixAutoConfig.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));
        assertEquals(0, constructors[0].getParameterTypes().length);

        List<String> publicMethods = new ArrayList<>();
        for (Method method : PhoenixAutoConfig.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethods.add(method.getName());
            }
        }
        publicMethods.sort(String::compareTo);
        assertEquals(Arrays.asList("copy", "defaults"), publicMethods);

        Method defaults = PhoenixAutoConfig.class.getMethod("defaults");
        assertTrue(Modifier.isStatic(defaults.getModifiers()));
        assertEquals(PhoenixAutoConfig.class, defaults.getReturnType());
        Method copy = PhoenixAutoConfig.class.getMethod("copy");
        assertFalse(Modifier.isStatic(copy.getModifiers()));
        assertEquals(PhoenixAutoConfig.class, copy.getReturnType());
    }

    @Test
    public void fieldsAreExactlyEightPublicMutableDoublesInSourceOrder() {
        List<String> fields = new ArrayList<>();
        for (Field field : PhoenixAutoConfig.class.getDeclaredFields()) {
            assertTrue(Modifier.isPublic(field.getModifiers()));
            assertFalse(Modifier.isStatic(field.getModifiers()));
            assertFalse(Modifier.isFinal(field.getModifiers()));
            assertEquals(Double.TYPE, field.getType());
            fields.add(field.getName());
        }
        assertEquals(FIELD_NAMES, fields);
    }

    @Test
    public void defaultsPreserveEveryCheckedInRawValueAndAreFresh() {
        PhoenixAutoConfig first = PhoenixAutoConfig.defaults();
        PhoenixAutoConfig second = PhoenixAutoConfig.defaults();

        assertNotSame(first, second);
        assertRawEquals(25.0, first.parkTakeoverElapsedSec);
        assertRawEquals(4.0, first.routeTimeoutSec);
        assertRawEquals(2.0, first.aimHeadingToleranceDeg);
        assertRawEquals(1.75, first.aimTimeoutSec);
        assertRawEquals(0.75, first.aimMaxNoGuidanceSec);
        assertRawEquals(0.75, first.waitForTargetSec);
        assertRawEquals(2.5, first.waitForShotCompleteSec);
        assertRawEquals(12.0, first.pedroIntegrationTestDistanceIn);

        first.parkTakeoverElapsedSec = 19.0;
        assertRawEquals(25.0, second.parkTakeoverElapsedSec);
    }

    @Test
    public void copyIsIndependentAndPreservesRawInvalidEvidenceWithoutValidation() {
        PhoenixAutoConfig source = PhoenixAutoConfig.defaults();
        source.parkTakeoverElapsedSec = -0.0;
        source.routeTimeoutSec = Double.POSITIVE_INFINITY;
        source.aimHeadingToleranceDeg = Double.longBitsToDouble(0x7ff8000000000042L);
        source.aimTimeoutSec = -1.0;
        source.aimMaxNoGuidanceSec = Double.NEGATIVE_INFINITY;
        source.waitForTargetSec = -2.0;
        source.waitForShotCompleteSec = Double.longBitsToDouble(0x7ff8000000000043L);
        source.pedroIntegrationTestDistanceIn = 0.0;

        PhoenixAutoConfig copy = source.copy();

        assertNotSame(source, copy);
        assertRawEquals(source.parkTakeoverElapsedSec, copy.parkTakeoverElapsedSec);
        assertRawEquals(source.routeTimeoutSec, copy.routeTimeoutSec);
        assertRawEquals(source.aimHeadingToleranceDeg, copy.aimHeadingToleranceDeg);
        assertRawEquals(source.aimTimeoutSec, copy.aimTimeoutSec);
        assertRawEquals(source.aimMaxNoGuidanceSec, copy.aimMaxNoGuidanceSec);
        assertRawEquals(source.waitForTargetSec, copy.waitForTargetSec);
        assertRawEquals(source.waitForShotCompleteSec, copy.waitForShotCompleteSec);
        assertRawEquals(
                source.pedroIntegrationTestDistanceIn,
                copy.pedroIntegrationTestDistanceIn
        );

        source.routeTimeoutSec = 3.0;
        copy.waitForTargetSec = 1.0;
        assertRawEquals(Double.POSITIVE_INFINITY, copy.routeTimeoutSec);
        assertRawEquals(-2.0, source.waitForTargetSec);
    }

    private static void assertRawEquals(double expected, double actual) {
        assertEquals(
                Double.doubleToRawLongBits(expected),
                Double.doubleToRawLongBits(actual)
        );
    }
}
