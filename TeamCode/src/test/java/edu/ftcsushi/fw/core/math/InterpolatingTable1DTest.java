package edu.ftcsushi.fw.core.math;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

import edu.ftcsushi.fw.core.debug.DebugSink;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the finite authored-table and finite interpolation contract. */
public final class InterpolatingTable1DTest {

    @Test
    public void factoryRejectsNullEmptyAndMismatchedInputs() {
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable1D.ofSorted(null, new double[]{1.0})),
                "xs", "required");
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable1D.ofSorted(new double[]{1.0}, null)),
                "values", "required");

        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofSorted(new double[]{1.0}, new double[]{1.0, 2.0})),
                "same length");
        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofSorted(new double[0], new double[0])),
                "at least one point");
    }

    @Test
    public void factoryRejectsEveryNonFiniteAuthoredSampleWithItsIndex() {
        double[] invalidValues = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};

        for (int component = 0; component < 2; component++) {
            for (int authoredIndex = 0; authoredIndex < 3; authoredIndex++) {
                for (double invalidValue : invalidValues) {
                    double[] xs = {10.0, 20.0, 30.0};
                    double[] ys = {100.0, 200.0, 300.0};
                    if (component == 0) {
                        xs[authoredIndex] = invalidValue;
                    } else {
                        ys[authoredIndex] = invalidValue;
                    }

                    IllegalArgumentException failure = assertThrows(
                            IllegalArgumentException.class,
                            () -> InterpolatingTable1D.ofSorted(xs, ys));
                    assertContains(failure,
                            component == 0 ? "x" : "value",
                            "authored index " + authoredIndex,
                            Double.toString(invalidValue),
                            "finite");
                }
            }
        }
    }

    @Test
    public void factoryReportsConflictingAdjacentAuthoredSamples() {
        assertSortedConflict(
                () -> InterpolatingTable1D.ofSorted(
                        new double[]{10.0, 30.0, 20.0}, new double[]{1.0, 2.0, 3.0}),
                "index 2", "20.0", "index 1", "30.0");
        assertSortedConflict(
                () -> InterpolatingTable1D.ofSorted(
                        new double[]{10.0, 20.0, 20.0}, new double[]{1.0, 2.0, 3.0}),
                "index 2", "20.0", "index 1", "20.0");
        assertSortedConflict(
                () -> InterpolatingTable1D.ofSorted(
                        new double[]{-0.0, 0.0}, new double[]{1.0, 2.0}),
                "index 1", "0.0", "index 0", "-0.0");
    }

    @Test
    public void factoryCapturesBothArraysWithoutMutatingThem() {
        double[] sortedXs = {10.0, 20.0, 30.0};
        double[] sortedYs = {100.0, 200.0, 300.0};
        InterpolatingTable1D sorted = InterpolatingTable1D.ofSorted(sortedXs, sortedYs);
        assertTrue(Arrays.equals(new double[]{10.0, 20.0, 30.0}, sortedXs));
        assertTrue(Arrays.equals(new double[]{100.0, 200.0, 300.0}, sortedYs));
        Arrays.fill(sortedXs, -1.0);
        Arrays.fill(sortedYs, -1.0);
        assertEquals(150.0, sorted.interpolate(15.0), 0.0);

        assertEquals(250.0, sorted.interpolate(25.0), 0.0);
    }

    @Test
    public void finiteQueriesClampMatchAndInterpolateNonMonotonicYValues() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSorted(
                new double[]{10.0, 20.0, 40.0}, new double[]{100.0, 0.0, 80.0});

        assertEquals(100.0, table.interpolate(-Double.MAX_VALUE), 0.0);
        assertEquals(100.0, table.interpolate(10.0), 0.0);
        assertEquals(50.0, table.interpolate(15.0), 0.0);
        assertEquals(0.0, table.interpolate(20.0), 0.0);
        assertEquals(40.0, table.interpolate(30.0), 0.0);
        assertEquals(80.0, table.interpolate(40.0), 0.0);
        assertEquals(80.0, table.interpolate(Double.MAX_VALUE), 0.0);
        assertEquals(table.interpolate(30.0), table.applyAsDouble(30.0), 0.0);
    }

    @Test
    public void eitherSignedZeroQueryPreservesNegativeZeroAtAnInteriorZeroNode() {
        for (double authoredZero : new double[]{-0.0, 0.0}) {
            InterpolatingTable1D table = InterpolatingTable1D.ofSorted(
                    new double[]{-1.0, authoredZero, 1.0}, new double[]{1.0, -0.0, 1.0});
            for (double queryZero : new double[]{-0.0, 0.0}) {
                assertEquals(Double.doubleToRawLongBits(-0.0),
                        Double.doubleToRawLongBits(table.interpolate(queryZero)));
                assertEquals(Double.doubleToRawLongBits(-0.0),
                        Double.doubleToRawLongBits(table.applyAsDouble(queryZero)));
            }
        }
    }

    @Test
    public void nonFiniteQueriesAreUnavailableForOneAndManyPointTables() {
        InterpolatingTable1D onePoint = InterpolatingTable1D.ofSorted(
                new double[]{-0.0}, new double[]{-0.0});
        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(onePoint.interpolate(-Double.MAX_VALUE)));
        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(onePoint.interpolate(Double.MAX_VALUE)));

        InterpolatingTable1D manyPoints = InterpolatingTable1D.ofSorted(
                new double[]{10.0, 20.0}, new double[]{100.0, 200.0});
        double[] unavailableQueries = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double query : unavailableQueries) {
            assertTrue(Double.isNaN(onePoint.interpolate(query)));
            assertTrue(Double.isNaN(onePoint.applyAsDouble(query)));
            assertTrue(Double.isNaN(manyPoints.interpolate(query)));
            assertTrue(Double.isNaN(manyPoints.applyAsDouble(query)));
        }
    }

    @Test
    public void extremeFiniteXSpanUsesStableFractions() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSorted(
                new double[]{-Double.MAX_VALUE, Double.MAX_VALUE}, new double[]{0.0, 100.0});

        assertEquals(25.0, table.interpolate(-Double.MAX_VALUE / 2.0), 0.0);
        assertEquals(50.0, table.interpolate(0.0), 0.0);
        assertEquals(75.0, table.interpolate(Double.MAX_VALUE / 2.0), 0.0);
    }

    @Test
    public void oppositeExtremeFiniteYValuesRemainFiniteAndConvex() {
        InterpolatingTable1D increasing = InterpolatingTable1D.ofSorted(
                new double[]{0.0, 1.0}, new double[]{-Double.MAX_VALUE, Double.MAX_VALUE});
        InterpolatingTable1D decreasing = InterpolatingTable1D.ofSorted(
                new double[]{0.0, 1.0}, new double[]{Double.MAX_VALUE, -Double.MAX_VALUE});

        assertEquals(0.0, increasing.interpolate(0.5), 0.0);
        assertEquals(0.0, decreasing.interpolate(0.5), 0.0);

        double[] nearEndpointQueries = {
                Double.MIN_VALUE,
                Math.nextDown(1.0)
        };
        for (double query : nearEndpointQueries) {
            double increasingValue = increasing.interpolate(query);
            double decreasingValue = decreasing.interpolate(query);
            assertTrue(Double.isFinite(increasingValue));
            assertTrue(Double.isFinite(decreasingValue));
            assertTrue(increasingValue >= -Double.MAX_VALUE);
            assertTrue(increasingValue <= Double.MAX_VALUE);
            assertTrue(decreasingValue >= -Double.MAX_VALUE);
            assertTrue(decreasingValue <= Double.MAX_VALUE);
        }
    }

    @Test
    public void sameSignNearMaximumYUsesTheNearerEndpointCalculation() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSorted(
                new double[]{0.0, 1.0}, new double[]{Double.MAX_VALUE / 2.0, Double.MAX_VALUE});

        double result = table.interpolate(Math.nextDown(1.0));

        assertTrue(Double.isFinite(result));
        assertEquals(Double.MAX_VALUE, result, 0.0);
    }

    @Test
    public void combinedExtremeFiniteXAndYValuesRemainLinearAndFinite() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSorted(
                new double[]{-Double.MAX_VALUE, Double.MAX_VALUE},
                new double[]{-Double.MAX_VALUE, Double.MAX_VALUE});

        assertEquals(-Double.MAX_VALUE / 2.0,
                table.interpolate(-Double.MAX_VALUE / 2.0),
                Math.ulp(Double.MAX_VALUE / 2.0));
        assertEquals(0.0, table.interpolate(0.0), 0.0);
        assertEquals(Double.MAX_VALUE / 2.0,
                table.interpolate(Double.MAX_VALUE / 2.0),
                Math.ulp(Double.MAX_VALUE / 2.0));
        assertTrue(Double.isFinite(table.interpolate(-Double.MAX_VALUE / 2.0)));
        assertTrue(Double.isFinite(table.interpolate(Double.MAX_VALUE / 2.0)));
    }

    @Test
    public void finiteSubnormalSamplesAndIntervalsRemainValid() {
        InterpolatingTable1D subnormal = InterpolatingTable1D.ofSorted(
                new double[]{0.0, 2.0 * Double.MIN_VALUE}, new double[]{0.0, 2.0});
        assertEquals(1.0, subnormal.interpolate(Double.MIN_VALUE), 0.0);

        InterpolatingTable1D finiteExtrema = InterpolatingTable1D.ofSorted(
                new double[]{-Double.MAX_VALUE, 0.0, Double.MAX_VALUE},
                new double[]{-Double.MIN_VALUE, 0.0, Double.MIN_VALUE});
        assertEquals(0.0, finiteExtrema.interpolate(0.0), 0.0);
        assertTrue(Double.isFinite(finiteExtrema.interpolate(-1.0)));
        assertTrue(Double.isFinite(finiteExtrema.interpolate(1.0)));
    }

    @Test
    public void publicSurfaceHasOneSortedFactoryOneQueryAndTheFunctionalAdapter() throws Exception {
        assertTrue(DoubleUnaryOperator.class.isAssignableFrom(InterpolatingTable1D.class));
        assertEquals(0, InterpolatingTable1D.class.getConstructors().length);
        assertEquals(0, InterpolatingTable1D.class.getDeclaredClasses().length);

        Set<String> factories = new HashSet<>();
        int factoryCount = 0;
        for (Method method : InterpolatingTable1D.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())
                    && Modifier.isStatic(method.getModifiers())
                    && method.getReturnType() == InterpolatingTable1D.class) {
                factories.add(method.getName());
                factoryCount++;
            }
        }
        assertEquals(1, factoryCount);
        assertEquals(new HashSet<>(Arrays.asList("ofSorted")), factories);

        Method sortedFactory = InterpolatingTable1D.class.getDeclaredMethod(
                "ofSorted", double[].class, double[].class);
        assertPublicStaticTableFactory(sortedFactory);
        assertFalse(sortedFactory.isVarArgs());

        Constructor<?>[] constructors = InterpolatingTable1D.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));
        assertTrue(Arrays.equals(
                new Class<?>[]{double[].class, double[].class},
                constructors[0].getParameterTypes()));

        assertEquals(double.class,
                InterpolatingTable1D.class.getDeclaredMethod(
                        "interpolate", double.class).getReturnType());
        assertEquals(double.class,
                InterpolatingTable1D.class.getDeclaredMethod(
                        "applyAsDouble", double.class).getReturnType());

        assertNoDeclaredMethod("builder");
        assertNoDeclaredMethod("ofUnsorted");
        assertNoDeclaredMethod("ofSortedPairs");
        assertNoDeclaredMethod("size");
        assertNoDeclaredMethod("xs");
        assertNoDeclaredMethod("ys");
    }

    @Test
    public void diagnosticsRetainStableKeysAndHumanReadablePairs() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSorted(
                new double[]{10.0, 30.0}, new double[]{100.0, 300.0});

        table.debugDump(null, "ignored");

        CapturingDebugSink named = new CapturingDebugSink();
        table.debugDump(named, "lookup.shooter");
        assertEquals(new HashSet<>(Arrays.asList(
                "lookup.shooter.size",
                "lookup.shooter.xMin",
                "lookup.shooter.xMax")), named.values.keySet());
        assertEquals(2, number(named, "lookup.shooter.size").intValue());
        assertEquals(10.0, number(named, "lookup.shooter.xMin").doubleValue(), 0.0);
        assertEquals(30.0, number(named, "lookup.shooter.xMax").doubleValue(), 0.0);

        CapturingDebugSink defaultPrefix = new CapturingDebugSink();
        table.debugDump(defaultPrefix, null);
        assertNotNull(defaultPrefix.values.get("table.size"));
        assertNotNull(defaultPrefix.values.get("table.xMin"));
        assertNotNull(defaultPrefix.values.get("table.xMax"));

        String description = table.toString();
        assertTrue(description.contains("InterpolatingTable1D"));
        assertTrue(description.contains("(10.0, 100.0)"));
        assertTrue(description.contains("(30.0, 300.0)"));
        assertFalse(description.isEmpty());
    }

    private static void assertSortedConflict(ThrowingRunnable action, String... fragments) {
        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                action::run);
        assertContains(failure, "strictly increasing", "reorder", fragments);
    }

    private static void assertContains(Throwable failure, String... fragments) {
        assertNotNull(failure.getMessage());
        for (String fragment : fragments) {
            assertTrue("Expected message to contain '" + fragment + "' but was: "
                            + failure.getMessage(),
                    failure.getMessage().contains(fragment));
        }
    }

    private static void assertContains(Throwable failure,
                                       String first,
                                       String second,
                                       String[] remaining) {
        assertContains(failure, first, second);
        assertContains(failure, remaining);
    }

    private static void assertNoDeclaredMethod(String name) {
        for (Method method : InterpolatingTable1D.class.getDeclaredMethods()) {
            if (method.getName().equals(name)) {
                fail("InterpolatingTable1D must not declare " + name);
            }
        }
    }

    private static void assertPublicStaticTableFactory(Method method) {
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertTrue(Modifier.isStatic(method.getModifiers()));
        assertEquals(InterpolatingTable1D.class, method.getReturnType());
    }

    private static Number number(CapturingDebugSink sink, String key) {
        Object value = sink.values.get(key);
        assertTrue("Expected numeric debug value for " + key, value instanceof Number);
        return (Number) value;
    }

    private interface ThrowingRunnable {
        void run();
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<>();

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
