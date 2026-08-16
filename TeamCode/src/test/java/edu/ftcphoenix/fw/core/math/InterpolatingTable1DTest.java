package edu.ftcphoenix.fw.core.math;

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

import edu.ftcphoenix.fw.core.debug.DebugSink;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the finite authored-table and finite interpolation contract. */
public final class InterpolatingTable1DTest {

    @Test
    public void factoriesRejectNullEmptyAndIncompleteInputs() {
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable1D.ofSorted(null, new double[]{1.0})),
                "xs", "required");
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable1D.ofSorted(new double[]{1.0}, null)),
                "ys", "required");
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable1D.ofUnsorted(null, new double[]{1.0})),
                "xs", "required");
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable1D.ofUnsorted(new double[]{1.0}, null)),
                "ys", "required");
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable1D.ofSortedPairs((double[]) null)),
                "xsAndYs", "required");

        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofSorted(new double[]{1.0}, new double[]{1.0, 2.0})),
                "same length");
        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofUnsorted(new double[]{1.0}, new double[0])),
                "same length");
        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofSorted(new double[0], new double[0])),
                "at least one point");
        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofUnsorted(new double[0], new double[0])),
                "at least one point");
        assertContains(assertThrows(IllegalArgumentException.class,
                InterpolatingTable1D::ofSortedPairs),
                "even number", "x0, y0");
        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofSortedPairs(1.0, 2.0, 3.0)),
                "even number", "x0, y0");
    }

    @Test
    public void everyFactoryRejectsEveryNonFiniteAuthoredSampleWithItsOriginalIndex() {
        double[] invalidValues = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};

        for (NamedFactory factory : FACTORIES) {
            for (int component = 0; component < 2; component++) {
                for (int authoredIndex = 0; authoredIndex < 3; authoredIndex++) {
                    for (double invalidValue : invalidValues) {
                        double[] xs = factory.xs.clone();
                        double[] ys = {100.0, 200.0, 300.0};
                        if (component == 0) {
                            xs[authoredIndex] = invalidValue;
                        } else {
                            ys[authoredIndex] = invalidValue;
                        }

                        IllegalArgumentException failure = assertThrows(
                                factory.name + " must reject non-finite authored samples",
                                IllegalArgumentException.class,
                                () -> factory.factory.create(xs, ys));
                        assertContains(failure,
                                component == 0 ? "x" : "y",
                                "authored index " + authoredIndex,
                                Double.toString(invalidValue),
                                "finite");
                    }
                }
            }
        }
    }

    @Test
    public void sortedFactoriesReportConflictingAdjacentAuthoredSamples() {
        assertSortedConflict(
                () -> InterpolatingTable1D.ofSorted(
                        new double[]{10.0, 30.0, 20.0}, new double[]{1.0, 2.0, 3.0}),
                "index 2", "20.0", "index 1", "30.0");
        assertSortedConflict(
                () -> InterpolatingTable1D.ofSortedPairs(
                        10.0, 1.0, 30.0, 2.0, 20.0, 3.0),
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
    public void unsortedFactoryReportsDuplicateOriginalAuthoredIndices() {
        IllegalArgumentException duplicate = assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofUnsorted(
                        new double[]{30.0, 10.0, 30.0}, new double[]{3.0, 1.0, 4.0}));
        assertContains(duplicate, "duplicate", "authored indices", "0 (30.0)", "2 (30.0)",
                "unique");

        IllegalArgumentException signedZero = assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable1D.ofUnsorted(
                        new double[]{10.0, 0.0, -0.0}, new double[]{3.0, 1.0, 2.0}));
        assertContains(signedZero, "duplicate", "1 (0.0)", "2 (-0.0)", "unique");
    }

    @Test
    public void everyFactoryCapturesItsInputsAndUnsortedFactoryPreservesPairs() {
        double[] sortedXs = {10.0, 20.0, 30.0};
        double[] sortedYs = {100.0, 200.0, 300.0};
        InterpolatingTable1D sorted = InterpolatingTable1D.ofSorted(sortedXs, sortedYs);
        Arrays.fill(sortedXs, -1.0);
        Arrays.fill(sortedYs, -1.0);
        assertEquals(150.0, sorted.interpolate(15.0), 0.0);

        double[] unsortedXs = {30.0, 10.0, 20.0};
        double[] unsortedYs = {300.0, 100.0, 200.0};
        InterpolatingTable1D unsorted = InterpolatingTable1D.ofUnsorted(unsortedXs, unsortedYs);
        Arrays.fill(unsortedXs, -1.0);
        Arrays.fill(unsortedYs, -1.0);
        assertEquals(100.0, unsorted.interpolate(10.0), 0.0);
        assertEquals(150.0, unsorted.interpolate(15.0), 0.0);
        assertEquals(300.0, unsorted.interpolate(30.0), 0.0);

        double[] pairs = {10.0, 100.0, 20.0, 200.0, 30.0, 300.0};
        InterpolatingTable1D paired = InterpolatingTable1D.ofSortedPairs(pairs);
        Arrays.fill(pairs, -1.0);
        assertEquals(250.0, paired.interpolate(25.0), 0.0);
    }

    @Test
    public void finiteQueriesClampMatchAndInterpolateNonMonotonicYValues() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSortedPairs(
                10.0, 100.0,
                20.0, 0.0,
                40.0, 80.0);

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
    public void nonFiniteQueriesAreUnavailableForOneAndManyPointTables() {
        InterpolatingTable1D onePoint = InterpolatingTable1D.ofSortedPairs(-0.0, -0.0);
        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(onePoint.interpolate(-Double.MAX_VALUE)));
        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(onePoint.interpolate(Double.MAX_VALUE)));

        InterpolatingTable1D manyPoints = InterpolatingTable1D.ofSortedPairs(
                10.0, 100.0, 20.0, 200.0);
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
        InterpolatingTable1D table = InterpolatingTable1D.ofSortedPairs(
                -Double.MAX_VALUE, 0.0,
                Double.MAX_VALUE, 100.0);

        assertEquals(25.0, table.interpolate(-Double.MAX_VALUE / 2.0), 0.0);
        assertEquals(50.0, table.interpolate(0.0), 0.0);
        assertEquals(75.0, table.interpolate(Double.MAX_VALUE / 2.0), 0.0);
    }

    @Test
    public void oppositeExtremeFiniteYValuesRemainFiniteAndConvex() {
        InterpolatingTable1D increasing = InterpolatingTable1D.ofSortedPairs(
                0.0, -Double.MAX_VALUE,
                1.0, Double.MAX_VALUE);
        InterpolatingTable1D decreasing = InterpolatingTable1D.ofSortedPairs(
                0.0, Double.MAX_VALUE,
                1.0, -Double.MAX_VALUE);

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
        InterpolatingTable1D table = InterpolatingTable1D.ofSortedPairs(
                0.0, Double.MAX_VALUE / 2.0,
                1.0, Double.MAX_VALUE);

        double result = table.interpolate(Math.nextDown(1.0));

        assertTrue(Double.isFinite(result));
        assertEquals(Double.MAX_VALUE, result, 0.0);
    }

    @Test
    public void combinedExtremeFiniteXAndYValuesRemainLinearAndFinite() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSortedPairs(
                -Double.MAX_VALUE, -Double.MAX_VALUE,
                Double.MAX_VALUE, Double.MAX_VALUE);

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
        InterpolatingTable1D subnormal = InterpolatingTable1D.ofSortedPairs(
                0.0, 0.0,
                2.0 * Double.MIN_VALUE, 2.0);
        assertEquals(1.0, subnormal.interpolate(Double.MIN_VALUE), 0.0);

        InterpolatingTable1D finiteExtrema = InterpolatingTable1D.ofSortedPairs(
                -Double.MAX_VALUE, -Double.MIN_VALUE,
                0.0, 0.0,
                Double.MAX_VALUE, Double.MIN_VALUE);
        assertEquals(0.0, finiteExtrema.interpolate(0.0), 0.0);
        assertTrue(Double.isFinite(finiteExtrema.interpolate(-1.0)));
        assertTrue(Double.isFinite(finiteExtrema.interpolate(1.0)));
    }

    @Test
    public void publicSurfaceHasThreeFactoriesOneQueryAndTheFunctionalAdapter() throws Exception {
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
        assertEquals(3, factoryCount);
        assertEquals(new HashSet<>(Arrays.asList(
                "ofSorted", "ofUnsorted", "ofSortedPairs")), factories);

        Method sortedFactory = InterpolatingTable1D.class.getDeclaredMethod(
                "ofSorted", double[].class, double[].class);
        Method unsortedFactory = InterpolatingTable1D.class.getDeclaredMethod(
                "ofUnsorted", double[].class, double[].class);
        Method pairsFactory = InterpolatingTable1D.class.getDeclaredMethod(
                "ofSortedPairs", double[].class);
        assertPublicStaticTableFactory(sortedFactory);
        assertPublicStaticTableFactory(unsortedFactory);
        assertPublicStaticTableFactory(pairsFactory);
        assertFalse(sortedFactory.isVarArgs());
        assertFalse(unsortedFactory.isVarArgs());
        assertTrue(pairsFactory.isVarArgs());

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
        assertNoDeclaredMethod("size");
        assertNoDeclaredMethod("xs");
        assertNoDeclaredMethod("ys");
    }

    @Test
    public void diagnosticsRetainStableKeysAndHumanReadablePairs() {
        InterpolatingTable1D table = InterpolatingTable1D.ofSortedPairs(
                10.0, 100.0,
                30.0, 300.0);

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

    private static double[] pairs(double[] xs, double[] ys) {
        double[] pairs = new double[xs.length * 2];
        for (int i = 0; i < xs.length; i++) {
            pairs[2 * i] = xs[i];
            pairs[2 * i + 1] = ys[i];
        }
        return pairs;
    }

    private interface Factory {
        InterpolatingTable1D create(double[] xs, double[] ys);
    }

    private interface ThrowingRunnable {
        void run();
    }

    private static final NamedFactory[] FACTORIES = {
            new NamedFactory("ofSorted", new double[]{10.0, 20.0, 30.0},
                    InterpolatingTable1D::ofSorted),
            new NamedFactory("ofUnsorted", new double[]{30.0, 10.0, 20.0},
                    InterpolatingTable1D::ofUnsorted),
            new NamedFactory("ofSortedPairs", new double[]{10.0, 20.0, 30.0},
                    (xs, ys) -> InterpolatingTable1D.ofSortedPairs(pairs(xs, ys)))
    };

    private static final class NamedFactory {
        private final String name;
        private final double[] xs;
        private final Factory factory;

        private NamedFactory(String name, double[] xs, Factory factory) {
            this.name = name;
            this.xs = xs;
            this.factory = factory;
        }
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
