package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the complete valid, invalid, and numeric contract of {@link ScalarRange}. */
public final class ScalarRangeTest {

    @Test
    public void exposesExactlyOnePublicFactoryForEachSupportedRangeShape() {
        Set<String> expected = new HashSet<>(Arrays.asList(
                "bounded(double,double)",
                "boundedFrom(double)",
                "boundedTo(double)",
                "invalid(String)",
                "unbounded()"));
        Set<String> actual = new HashSet<>();

        for (Method method : ScalarRange.class.getDeclaredMethods()) {
            int modifiers = method.getModifiers();
            if (Modifier.isPublic(modifiers) && Modifier.isStatic(modifiers)) {
                actual.add(signature(method));
                assertSame(ScalarRange.class, method.getReturnType());
            }
            assertFalse("Removed public range shortcut remains: " + method,
                    Modifier.isPublic(modifiers)
                            && (method.getName().equals("minOnly")
                            || method.getName().equals("maxOnly")
                            || method.getName().equals("unboundedSource")));
        }

        assertEquals(expected, actual);
        for (Constructor<?> constructor : ScalarRange.class.getDeclaredConstructors()) {
            assertFalse(Modifier.isPublic(constructor.getModifiers()));
        }
        assertNoPublicMethod(PositionPlant.class, "targetRangeSource");
    }

    @Test
    public void canonicalFactoriesExposeTheirExactShapeFieldsReasonsAndText() {
        ScalarRange bounded = ScalarRange.bounded(-2.0, 3.0);
        assertRange(bounded, true, -2.0, 3.0, "bounded", false,
                "ScalarRange{valid=true, minValue=-2.0, maxValue=3.0, reason='bounded'}");

        ScalarRange boundedFrom = ScalarRange.boundedFrom(4.0);
        assertRange(boundedFrom, true, 4.0, Double.POSITIVE_INFINITY,
                "boundedFrom", false,
                "ScalarRange{valid=true, minValue=4.0, maxValue=Infinity, "
                        + "reason='boundedFrom'}");

        ScalarRange boundedTo = ScalarRange.boundedTo(-4.0);
        assertRange(boundedTo, true, Double.NEGATIVE_INFINITY, -4.0,
                "boundedTo", false,
                "ScalarRange{valid=true, minValue=-Infinity, maxValue=-4.0, "
                        + "reason='boundedTo'}");

        ScalarRange unbounded = ScalarRange.unbounded();
        assertRange(unbounded, true, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
                "unbounded", true,
                "ScalarRange{valid=true, minValue=-Infinity, maxValue=Infinity, "
                        + "reason='unbounded'}");
    }

    @Test
    public void everyBoundedFactoryRejectsEveryNonFiniteSuppliedEndpoint() {
        double[] nonFinite = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };

        for (double value : nonFinite) {
            assertNonFiniteRejected(
                    () -> ScalarRange.bounded(value, 1.0),
                    "bounded", "minValue", value);
            assertNonFiniteRejected(
                    () -> ScalarRange.bounded(-1.0, value),
                    "bounded", "maxValue", value);
            assertNonFiniteRejected(
                    () -> ScalarRange.boundedFrom(value),
                    "boundedFrom", "minValue", value);
            assertNonFiniteRejected(
                    () -> ScalarRange.boundedTo(value),
                    "boundedTo", "maxValue", value);
        }
    }

    @Test
    public void finiteExtremesSingletonsAndBothSignedZeroOrdersAreValidAndPreserved() {
        ScalarRange extremes = ScalarRange.bounded(-Double.MAX_VALUE, Double.MAX_VALUE);
        assertEquals(-Double.MAX_VALUE, extremes.minValue, 0.0);
        assertEquals(Double.MAX_VALUE, extremes.maxValue, 0.0);
        assertTrue(extremes.contains(-Double.MAX_VALUE));
        assertTrue(extremes.contains(Double.MAX_VALUE));

        ScalarRange lowerExtreme = ScalarRange.boundedFrom(-Double.MAX_VALUE);
        ScalarRange upperExtreme = ScalarRange.boundedTo(Double.MAX_VALUE);
        assertEquals(-Double.MAX_VALUE, lowerExtreme.minValue, 0.0);
        assertEquals(Double.MAX_VALUE, upperExtreme.maxValue, 0.0);

        ScalarRange singleton = ScalarRange.bounded(Double.MIN_VALUE, Double.MIN_VALUE);
        assertTrue(singleton.valid);
        assertTrue(singleton.contains(Double.MIN_VALUE));

        ScalarRange negativeThenPositive = ScalarRange.bounded(-0.0, +0.0);
        assertRawBits(-0.0, negativeThenPositive.minValue);
        assertRawBits(+0.0, negativeThenPositive.maxValue);
        assertTrue(negativeThenPositive.contains(-0.0));
        assertTrue(negativeThenPositive.contains(+0.0));

        ScalarRange positiveThenNegative = ScalarRange.bounded(+0.0, -0.0);
        assertRawBits(+0.0, positiveThenNegative.minValue);
        assertRawBits(-0.0, positiveThenNegative.maxValue);
        assertTrue(positiveThenNegative.contains(-0.0));
        assertTrue(positiveThenNegative.contains(+0.0));

        assertRawBits(-0.0, ScalarRange.boundedFrom(-0.0).minValue);
        assertRawBits(+0.0, ScalarRange.boundedFrom(+0.0).minValue);
        assertRawBits(-0.0, ScalarRange.boundedTo(-0.0).maxValue);
        assertRawBits(+0.0, ScalarRange.boundedTo(+0.0).maxValue);
    }

    @Test
    public void reversedFiniteBoundsFailWithBothEndpointValues() {
        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> ScalarRange.bounded(2.0, 1.0));

        assertContains(failure, "ScalarRange.bounded", "minValue", "2.0", "maxValue", "1.0");
    }

    @Test
    public void containmentIsInclusiveForFiniteValuesAcrossAllCanonicalShapes() {
        ScalarRange bounded = ScalarRange.bounded(-2.0, 3.0);
        assertTrue(bounded.contains(-2.0));
        assertTrue(bounded.contains(0.5));
        assertTrue(bounded.contains(3.0));
        assertFalse(bounded.contains(Math.nextDown(-2.0)));
        assertFalse(bounded.contains(Math.nextUp(3.0)));

        ScalarRange boundedFrom = ScalarRange.boundedFrom(4.0);
        assertTrue(boundedFrom.contains(4.0));
        assertTrue(boundedFrom.contains(Double.MAX_VALUE));
        assertFalse(boundedFrom.contains(Math.nextDown(4.0)));

        ScalarRange boundedTo = ScalarRange.boundedTo(-4.0);
        assertTrue(boundedTo.contains(-4.0));
        assertTrue(boundedTo.contains(-Double.MAX_VALUE));
        assertFalse(boundedTo.contains(Math.nextUp(-4.0)));

        ScalarRange unbounded = ScalarRange.unbounded();
        assertTrue(unbounded.contains(-Double.MAX_VALUE));
        assertTrue(unbounded.contains(0.0));
        assertTrue(unbounded.contains(Double.MAX_VALUE));
    }

    @Test
    public void everyCanonicalShapeRejectsAndRefusesToClampEveryNonFiniteValue() {
        ScalarRange[] ranges = {
                ScalarRange.bounded(-2.0, 3.0),
                ScalarRange.boundedFrom(4.0),
                ScalarRange.boundedTo(-4.0),
                ScalarRange.unbounded()
        };
        double[] nonFinite = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };

        for (ScalarRange range : ranges) {
            for (double value : nonFinite) {
                assertFalse(range.toString(), range.contains(value));
                assertTrue(range.toString(), Double.isNaN(range.clamp(value)));
            }
        }
    }

    @Test
    public void finiteClampHonorsClosedHalfBoundedAndUnboundedShapes() {
        ScalarRange bounded = ScalarRange.bounded(-2.0, 3.0);
        assertEquals(-2.0, bounded.clamp(-100.0), 0.0);
        assertEquals(-2.0, bounded.clamp(-2.0), 0.0);
        assertEquals(0.5, bounded.clamp(0.5), 0.0);
        assertEquals(3.0, bounded.clamp(3.0), 0.0);
        assertEquals(3.0, bounded.clamp(100.0), 0.0);

        ScalarRange boundedFrom = ScalarRange.boundedFrom(4.0);
        assertEquals(4.0, boundedFrom.clamp(-100.0), 0.0);
        assertEquals(4.0, boundedFrom.clamp(4.0), 0.0);
        assertEquals(Double.MAX_VALUE, boundedFrom.clamp(Double.MAX_VALUE), 0.0);

        ScalarRange boundedTo = ScalarRange.boundedTo(-4.0);
        assertEquals(-Double.MAX_VALUE, boundedTo.clamp(-Double.MAX_VALUE), 0.0);
        assertEquals(-4.0, boundedTo.clamp(-4.0), 0.0);
        assertEquals(-4.0, boundedTo.clamp(100.0), 0.0);

        ScalarRange unbounded = ScalarRange.unbounded();
        assertEquals(-Double.MAX_VALUE, unbounded.clamp(-Double.MAX_VALUE), 0.0);
        assertRawBits(-0.0, unbounded.clamp(-0.0));
        assertEquals(Double.MAX_VALUE, unbounded.clamp(Double.MAX_VALUE), 0.0);
    }

    @Test
    public void invalidRangeRetainsRuntimeReasonAndFailsEveryNumericOperationClosed() {
        ScalarRange invalid = ScalarRange.invalid("not homed");

        assertFalse(invalid.valid);
        assertTrue(Double.isNaN(invalid.minValue));
        assertTrue(Double.isNaN(invalid.maxValue));
        assertEquals("not homed", invalid.reason);
        assertEquals("ScalarRange{valid=false, minValue=NaN, maxValue=NaN, "
                + "reason='not homed'}", invalid.toString());
        assertFalse(invalid.isUnbounded());
        assertFalse(invalid.contains(0.0));
        assertFalse(invalid.contains(Double.NaN));
        assertTrue(Double.isNaN(invalid.clamp(0.0)));
        assertTrue(Double.isNaN(invalid.clamp(Double.POSITIVE_INFINITY)));
        assertTrue(Double.isNaN(invalid.center()));

        assertEquals("invalid range", ScalarRange.invalid(null).reason);
        assertEquals("", ScalarRange.invalid("").reason);
    }

    @Test
    public void centerIsOverflowSafeForEveryFiniteBoundPattern() {
        assertEquals(0.0,
                ScalarRange.bounded(-Double.MAX_VALUE, Double.MAX_VALUE).center(), 0.0);
        assertEquals(0.75 * Double.MAX_VALUE,
                ScalarRange.bounded(0.5 * Double.MAX_VALUE, Double.MAX_VALUE).center(), 0.0);
        assertEquals(-0.75 * Double.MAX_VALUE,
                ScalarRange.bounded(-Double.MAX_VALUE, -0.5 * Double.MAX_VALUE).center(), 0.0);
        assertEquals(Double.MAX_VALUE,
                ScalarRange.bounded(Double.MAX_VALUE, Double.MAX_VALUE).center(), 0.0);
        assertEquals(0.0, ScalarRange.bounded(-0.0, -0.0).center(), 0.0);

        assertTrue(Double.isNaN(ScalarRange.boundedFrom(0.0).center()));
        assertTrue(Double.isNaN(ScalarRange.boundedTo(0.0).center()));
        assertTrue(Double.isNaN(ScalarRange.unbounded().center()));
        assertTrue(Double.isNaN(ScalarRange.invalid("unavailable").center()));
    }

    @Test
    public void targetContextsRequireAnExplicitRangeAndRetainTheProvidedValue() {
        Throwable simpleFailure = assertThrows(
                NullPointerException.class,
                () -> PlantTargetContext.simple(
                        false, Double.NaN, null, Double.NaN, Double.NaN));
        assertContains(simpleFailure, "targetRange");

        Throwable positionFailure = assertThrows(
                NullPointerException.class,
                () -> PlantTargetContext.position(
                        false,
                        Double.NaN,
                        null,
                        PositionPlant.Periodicity.NON_PERIODIC,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN));
        assertContains(positionFailure, "targetRange");

        ScalarRange explicit = ScalarRange.unbounded();
        PlantTargetContext simple = PlantTargetContext.simple(
                false, Double.NaN, explicit, Double.NaN, Double.NaN);
        PlantTargetContext position = PlantTargetContext.position(
                false,
                Double.NaN,
                explicit,
                PositionPlant.Periodicity.NON_PERIODIC,
                Double.NaN,
                Double.NaN,
                Double.NaN);
        assertSame(explicit, simple.targetRange());
        assertSame(explicit, position.targetRange());
    }

    private static void assertRange(ScalarRange range,
                                    boolean valid,
                                    double minValue,
                                    double maxValue,
                                    String reason,
                                    boolean unbounded,
                                    String text) {
        assertEquals(valid, range.valid);
        assertEquals(minValue, range.minValue, 0.0);
        assertEquals(maxValue, range.maxValue, 0.0);
        assertEquals(reason, range.reason);
        assertEquals(unbounded, range.isUnbounded());
        assertEquals(text, range.toString());
    }

    private static void assertNonFiniteRejected(Runnable action,
                                                String factory,
                                                String argument,
                                                double value) {
        Throwable failure = assertThrows(IllegalArgumentException.class, action);
        assertContains(failure,
                "ScalarRange." + factory,
                "finite",
                argument,
                Double.toString(value));
    }

    private static void assertRawBits(double expected, double actual) {
        assertEquals(Long.toHexString(Double.doubleToRawLongBits(expected)),
                Long.toHexString(Double.doubleToRawLongBits(actual)));
    }

    private static String signature(Method method) {
        StringBuilder result = new StringBuilder(method.getName()).append('(');
        Class<?>[] parameterTypes = method.getParameterTypes();
        for (int i = 0; i < parameterTypes.length; i++) {
            if (i > 0) result.append(',');
            result.append(parameterTypes[i].getSimpleName());
        }
        return result.append(')').toString();
    }

    private static void assertNoPublicMethod(Class<?> type, String name) {
        for (Method method : type.getMethods()) {
            if (method.getName().equals(name)) {
                fail(type.getSimpleName() + " must not expose removed method " + name);
            }
        }
    }

    private static <T extends Throwable> T assertThrows(Class<T> type, Runnable action) {
        try {
            action.run();
            fail("Expected " + type.getSimpleName());
            throw new AssertionError("unreachable");
        } catch (Throwable failure) {
            if (!type.isInstance(failure)) {
                throw new AssertionError("Unexpected exception", failure);
            }
            return type.cast(failure);
        }
    }

    private static void assertContains(Throwable failure, String... fragments) {
        String message = failure.getMessage();
        assertTrue("Expected exception message", message != null);
        for (String fragment : fragments) {
            assertTrue(message, message.contains(fragment));
        }
    }
}
