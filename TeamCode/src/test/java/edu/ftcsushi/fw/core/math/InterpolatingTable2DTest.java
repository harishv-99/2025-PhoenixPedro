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
import java.util.function.DoubleBinaryOperator;

import edu.ftcsushi.fw.core.debug.DebugSink;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Proves finite rectangular-grid mathematics, not the accuracy of a robot's calibration. */
public final class InterpolatingTable2DTest {

    @Test
    public void factoryRejectsNullAxesGridAndRowsWithTheirNames() {
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable2D.ofSorted(
                        null, new double[]{1.0}, new double[][]{{2.0}})), "xs", "required");
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable2D.ofSorted(
                        new double[]{1.0}, null, new double[][]{{2.0}})), "ys", "required");
        assertContains(assertThrows(NullPointerException.class,
                () -> InterpolatingTable2D.ofSorted(
                        new double[]{1.0}, new double[]{1.0}, null)), "values", "required");
        for (int row = 0; row < 2; row++) {
            double[][] values = {{1.0}, {2.0}};
            values[row] = null;
            assertContains(assertThrows(NullPointerException.class,
                    () -> InterpolatingTable2D.ofSorted(
                            new double[]{1.0, 2.0}, new double[]{1.0}, values)),
                    "values[" + row + "]", "required");
        }
    }

    @Test
    public void factoryRejectsEmptyAxesAndMissingOrExtraRows() {
        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable2D.ofSorted(
                        new double[0], new double[]{1.0}, new double[0][])), "x", "at least one");
        assertContains(assertThrows(IllegalArgumentException.class,
                () -> InterpolatingTable2D.ofSorted(
                        new double[]{1.0}, new double[0], new double[][]{new double[0]})),
                "y", "at least one");
        for (double[][] invalid : new double[][][]{
                new double[0][], {{1.0}}, {{1.0}, {2.0}, {3.0}}
        }) {
            assertContains(assertThrows(IllegalArgumentException.class,
                    () -> InterpolatingTable2D.ofSorted(
                            new double[]{1.0, 2.0}, new double[]{1.0}, invalid)),
                    "values", "length");
        }
    }

    @Test
    public void factoryRejectsEveryRaggedRowIncludingEmptyAndExtraColumns() {
        for (int row = 0; row < 3; row++) {
            for (int columnCount : new int[]{0, 1, 3}) {
                double[][] values = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};
                values[row] = new double[columnCount];
                assertContains(assertThrows(IllegalArgumentException.class,
                        () -> InterpolatingTable2D.ofSorted(
                                new double[]{1.0, 2.0, 3.0}, new double[]{4.0, 5.0}, values)),
                        "values[" + row + "]", "length");
            }
        }
    }

    @Test
    public void factoryRejectsEachNonFiniteAxisValueAtItsAuthoredIndex() {
        for (int axis = 0; axis < 2; axis++) {
            for (int index = 0; index < 3; index++) {
                for (double invalid : unavailableValues()) {
                    double[] xs = {1.0, 2.0, 3.0};
                    double[] ys = {4.0, 5.0, 6.0};
                    (axis == 0 ? xs : ys)[index] = invalid;
                    assertContains(assertThrows(IllegalArgumentException.class,
                            () -> InterpolatingTable2D.ofSorted(xs, ys,
                                    new double[][]{{1.0, 2.0, 3.0},
                                            {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}})),
                            axis == 0 ? "x" : "y", "index " + index,
                            Double.toString(invalid), "finite");
                }
            }
        }
    }

    @Test
    public void factoryRejectsEachNonFiniteGridValueAtItsRowAndColumn() {
        for (int row = 0; row < 2; row++) {
            for (int column = 0; column < 3; column++) {
                for (double invalid : unavailableValues()) {
                    double[][] values = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
                    values[row][column] = invalid;
                    assertContains(assertThrows(IllegalArgumentException.class,
                            () -> InterpolatingTable2D.ofSorted(
                                    new double[]{1.0, 2.0}, new double[]{4.0, 5.0, 6.0}, values)),
                            "values[" + row + "][" + column + "]",
                            Double.toString(invalid), "finite");
                }
            }
        }
    }

    @Test
    public void bothAxesRejectDuplicatesDescendingValuesAndBothSignedZeroOrders() {
        for (int axis = 0; axis < 2; axis++) {
            for (double[] invalid : new double[][]{
                    {10.0, 30.0, 20.0}, {10.0, 20.0, 20.0},
                    {-0.0, 0.0, 10.0}, {0.0, -0.0, 10.0}
            }) {
                double[] xs = axis == 0 ? invalid : new double[]{1.0, 2.0, 3.0};
                double[] ys = axis == 1 ? invalid : new double[]{4.0, 5.0, 6.0};
                int laterIndex = invalid[2] == 10.0 ? 1 : 2;
                assertContains(assertThrows(IllegalArgumentException.class,
                        () -> InterpolatingTable2D.ofSorted(xs, ys,
                                new double[][]{{1.0, 2.0, 3.0},
                                        {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}})),
                        axis == 0 ? "x" : "y", "strictly increasing", "reorder",
                        "index " + laterIndex, "index " + (laterIndex - 1),
                        Double.toString(invalid[laterIndex]),
                        Double.toString(invalid[laterIndex - 1]));
            }
        }
    }

    @Test
    public void constructionDoesNotMutateInputsAndCapturesBothAxesOuterArrayAndEveryRow() {
        double[] xs = {10.0, 30.0};
        double[] ys = {-5.0, 5.0, 20.0};
        double[] firstRow = {5.0, -10.0, 30.0};
        double[] secondRow = {105.0, 50.0, -20.0};
        double[][] values = {firstRow, secondRow};
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(xs, ys, values);
        assertArrayEquals(new double[]{10.0, 30.0}, xs, 0.0);
        assertArrayEquals(new double[]{-5.0, 5.0, 20.0}, ys, 0.0);
        assertArrayEquals(new double[]{5.0, -10.0, 30.0}, firstRow, 0.0);
        assertArrayEquals(new double[]{105.0, 50.0, -20.0}, secondRow, 0.0);

        Arrays.fill(xs, Double.NaN);
        Arrays.fill(ys, Double.NaN);
        Arrays.fill(firstRow, Double.NaN);
        Arrays.fill(secondRow, Double.NaN);
        values[0] = null;
        values[1] = new double[0];
        assertEquals(11.25, table.interpolate(15.0, 12.5), 0.0);
        assertEquals(5.0, table.interpolate(10.0, -5.0), 0.0);
        assertEquals(-20.0, table.interpolate(30.0, 20.0), 0.0);
    }

    @Test
    public void asymmetricRectangularGridReproducesAnIndependentCrossTermEquation() {
        // Authored values of 7 + 2*x - 3*y + 5*x*y; unequal axis counts catch transposition.
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{-2.0, 1.0, 5.0}, new double[]{-3.0, 0.0, 2.0, 7.0},
                new double[][]{{42.0, 3.0, -23.0, -88.0},
                        {3.0, 9.0, 13.0, 23.0}, {-49.0, 17.0, 61.0, 171.0}});
        for (double x : new double[]{-2.0, -0.5, 1.0, 3.0, 5.0}) {
            for (double y : new double[]{-3.0, -1.5, 0.0, 1.0, 2.0, 4.5, 7.0}) {
                double expected = 7.0 + 2.0 * x - 3.0 * y + 5.0 * x * y;
                assertEquals(expected, table.interpolate(x, y), 1e-12);
                assertEquals(expected, table.applyAsDouble(x, y), 1e-12);
            }
        }
    }

    @Test
    public void finiteQueriesClampEachAxisIndependentlyWithoutExtrapolation() {
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{10.0, 30.0}, new double[]{-5.0, 5.0, 20.0},
                new double[][]{{5.0, -10.0, 30.0}, {105.0, 50.0, -20.0}});
        double low = -Double.MAX_VALUE;
        double high = Double.MAX_VALUE;
        assertEquals(5.0, table.interpolate(low, low), 0.0);
        assertEquals(30.0, table.interpolate(low, high), 0.0);
        assertEquals(105.0, table.interpolate(high, low), 0.0);
        assertEquals(-20.0, table.interpolate(high, high), 0.0);
        assertEquals(-2.5, table.interpolate(low, 0.0), 0.0);
        assertEquals(77.5, table.interpolate(high, 0.0), 0.0);
        assertEquals(30.0, table.interpolate(15.0, low), 0.0);
        assertEquals(17.5, table.interpolate(15.0, high), 0.0);
        assertEquals(11.25, table.interpolate(15.0, 12.5), 0.0);
    }

    @Test
    public void everyExactNodePreservesStoredBitsIncludingSignedZeroAndExtrema() {
        double[] xs = {-1.0, -0.0, 1.0};
        double[] ys = {2.0, 4.0, 8.0};
        double[][] values = {{-0.0, Double.MIN_VALUE, Double.MAX_VALUE},
                {-Double.MIN_VALUE, -0.0, 0.0}, {-Double.MAX_VALUE, 1.0, -0.0}};
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(xs, ys, values);
        for (int row = 0; row < xs.length; row++) {
            for (int column = 0; column < ys.length; column++) {
                assertBits(values[row][column], table.interpolate(xs[row], ys[column]));
                assertBits(values[row][column], table.applyAsDouble(xs[row], ys[column]));
            }
        }
        assertBits(-0.0, table.interpolate(0.0, 4.0));
        assertBits(-0.0, table.interpolate(-Double.MAX_VALUE, -Double.MAX_VALUE));
        assertBits(-0.0, table.interpolate(Double.MAX_VALUE, Double.MAX_VALUE));
    }

    @Test
    public void eitherSignedZeroQueryOnEitherAxisPreservesInteriorNodeOutputBits() {
        for (double authoredXZero : new double[]{-0.0, 0.0}) {
            for (double authoredYZero : new double[]{-0.0, 0.0}) {
                InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                        new double[]{-1.0, authoredXZero, 1.0},
                        new double[]{-2.0, authoredYZero, 2.0},
                        new double[][]{{1.0, -0.0, 3.0},
                                {-0.0, -0.0, -0.0}, {6.0, -0.0, 8.0}});
                for (double x : new double[]{-1.0, -0.0, 0.0, 1.0}) {
                    for (double y : new double[]{-2.0, -0.0, 0.0, 2.0}) {
                        if (x == 0.0 || y == 0.0) {
                            assertBits(-0.0, table.interpolate(x, y));
                            assertBits(-0.0, table.applyAsDouble(x, y));
                        }
                    }
                }
            }
        }
    }

    @Test
    public void exactRowsAndColumnsReduceToTheirOwnOneDimensionalTable() {
        double[] xs = {-10.0, 0.0, 30.0};
        double[] ys = {-2.0, 5.0, 9.0};
        double[][] values = {{-Double.MAX_VALUE, -0.0, Double.MAX_VALUE},
                {20.0, 10.0, -10.0}, {Double.MIN_VALUE, -50.0, 100.0}};
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(xs, ys, values);
        for (int row = 0; row < xs.length; row++) {
            InterpolatingTable1D line = InterpolatingTable1D.ofSorted(ys, values[row]);
            for (double y : new double[]{-Double.MAX_VALUE, -2.0, 1.5, 5.0, 7.0, 9.0,
                    Double.MAX_VALUE}) {
                assertBits(line.interpolate(y), table.interpolate(xs[row], y));
            }
        }
        for (int column = 0; column < ys.length; column++) {
            InterpolatingTable1D line = InterpolatingTable1D.ofSorted(xs,
                    new double[]{values[0][column], values[1][column], values[2][column]});
            for (double x : new double[]{-Double.MAX_VALUE, -10.0, -5.0, 0.0, 15.0, 30.0,
                    Double.MAX_VALUE}) {
                assertBits(line.interpolate(x), table.interpolate(x, ys[column]));
            }
        }
    }

    @Test
    public void singletonAxesMatchOneDimensionalTablesAndIgnoreFiniteUnusedCoordinates() {
        double[] axis = {-5.0, 0.0, 20.0};
        double[] values = {10.0, -0.0, 30.0};
        InterpolatingTable1D line = InterpolatingTable1D.ofSorted(axis, values);
        InterpolatingTable2D singleX = InterpolatingTable2D.ofSorted(
                new double[]{3.0}, axis, new double[][]{values});
        InterpolatingTable2D singleY = InterpolatingTable2D.ofSorted(
                axis, new double[]{7.0}, new double[][]{{10.0}, {-0.0}, {30.0}});
        for (double query : new double[]{-Double.MAX_VALUE, -5.0, -2.5, -0.0, 0.0, 10.0,
                20.0, Double.MAX_VALUE}) {
            for (double unused : new double[]{-Double.MAX_VALUE, 0.0, Double.MAX_VALUE}) {
                assertBits(line.interpolate(query), singleX.interpolate(unused, query));
                assertBits(line.interpolate(query), singleY.interpolate(query, unused));
            }
        }
        InterpolatingTable2D singlePoint = InterpolatingTable2D.ofSorted(
                new double[]{-0.0}, new double[]{-0.0}, new double[][]{{-0.0}});
        for (double x : new double[]{-Double.MAX_VALUE, 0.0, Double.MAX_VALUE}) {
            for (double y : new double[]{-Double.MAX_VALUE, 0.0, Double.MAX_VALUE}) {
                assertBits(-0.0, singlePoint.interpolate(x, y));
            }
        }
    }

    @Test
    public void eitherNonFiniteQueryIsUnavailableBeforeClampingOrSingletonHandling() {
        InterpolatingTable2D[] tables = {
                InterpolatingTable2D.ofSorted(new double[]{0.0}, new double[]{0.0},
                        new double[][]{{1.0}}),
                InterpolatingTable2D.ofSorted(new double[]{0.0}, new double[]{0.0, 1.0},
                        new double[][]{{1.0, 2.0}}),
                InterpolatingTable2D.ofSorted(new double[]{0.0, 1.0}, new double[]{0.0},
                        new double[][]{{1.0}, {2.0}}),
                InterpolatingTable2D.ofSorted(new double[]{0.0, 1.0}, new double[]{0.0, 1.0},
                        new double[][]{{1.0, 2.0}, {3.0, 4.0}})
        };
        for (InterpolatingTable2D table : tables) {
            for (double invalid : unavailableValues()) {
                for (double other : new double[]{-Double.MAX_VALUE, 0.0, Double.MAX_VALUE}) {
                    assertTrue(Double.isNaN(table.interpolate(invalid, other)));
                    assertTrue(Double.isNaN(table.interpolate(other, invalid)));
                    assertTrue(Double.isNaN(table.applyAsDouble(invalid, other)));
                    assertTrue(Double.isNaN(table.applyAsDouble(other, invalid)));
                }
                for (double otherInvalid : unavailableValues()) {
                    assertTrue(Double.isNaN(table.interpolate(invalid, otherInvalid)));
                    assertTrue(Double.isNaN(table.applyAsDouble(invalid, otherInvalid)));
                }
            }
        }
    }

    @Test
    public void bothOppositeSignExtremeAxisSpansRetainCorrectFractions() {
        double max = Double.MAX_VALUE;
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{-max, max}, new double[]{-max, max},
                new double[][]{{0.0, 40.0}, {60.0, 100.0}});
        double[] coordinates = {-max / 2.0, 0.0, max / 2.0};
        double[][] expected = {{25.0, 35.0, 45.0}, {40.0, 50.0, 60.0}, {55.0, 65.0, 75.0}};
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                assertEquals(expected[row][column],
                        table.interpolate(coordinates[row], coordinates[column]), 0.0);
            }
        }
    }

    @Test
    public void oppositeExtremeValuesStayFiniteAndConvexThroughBothBlends() {
        double max = Double.MAX_VALUE;
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{0.0, 1.0}, new double[]{0.0, 1.0},
                new double[][]{{-max, max}, {max, -max}});
        assertEquals(0.0, table.interpolate(0.5, 0.5), 0.0);
        assertEquals(-max / 4.0, table.interpolate(0.25, 0.25), Math.ulp(max / 4.0));
        assertEquals(max / 4.0, table.interpolate(0.25, 0.75), Math.ulp(max / 4.0));
        for (double x : new double[]{Double.MIN_VALUE, 0.25, 0.5, 0.75, Math.nextDown(1.0)}) {
            for (double y : new double[]{Double.MIN_VALUE, 0.25, 0.5, 0.75, Math.nextDown(1.0)}) {
                double result = table.interpolate(x, y);
                assertTrue(Double.isFinite(result));
                assertTrue(result >= -max && result <= max);
            }
        }
    }

    @Test
    public void sameSignExtremeValuesStayInsideTheirFiniteCornerRange() {
        double max = Double.MAX_VALUE;
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{0.0, 1.0}, new double[]{0.0, 1.0},
                new double[][]{{max / 2.0, max}, {max, max / 2.0}});
        for (double x : new double[]{Double.MIN_VALUE, 0.5, Math.nextDown(1.0)}) {
            for (double y : new double[]{Double.MIN_VALUE, 0.5, Math.nextDown(1.0)}) {
                double result = table.interpolate(x, y);
                assertTrue(Double.isFinite(result));
                assertTrue(result >= max / 2.0 && result <= max);
            }
        }
    }

    @Test
    public void combinedExtremeAxesAndValuesRemainFiniteAndLinear() {
        double max = Double.MAX_VALUE;
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{-max, max}, new double[]{-max, max},
                new double[][]{{-max, -max / 2.0}, {max / 2.0, max}});
        assertEquals(-max / 2.0, table.interpolate(-max / 2.0, -max / 2.0), Math.ulp(max));
        assertEquals(0.0, table.interpolate(0.0, 0.0), 0.0);
        assertEquals(max / 2.0, table.interpolate(max / 2.0, max / 2.0), Math.ulp(max));
    }

    @Test
    public void subnormalAxesAndValuesAreValidWithoutArtificialMinimumSpacing() {
        double tiny = Double.MIN_VALUE;
        InterpolatingTable2D tinyAxes = InterpolatingTable2D.ofSorted(
                new double[]{0.0, 2.0 * tiny}, new double[]{0.0, 4.0 * tiny},
                new double[][]{{0.0, 8.0}, {4.0, 12.0}});
        assertEquals(4.0, tinyAxes.interpolate(tiny, tiny), 0.0);
        assertEquals(6.0, tinyAxes.interpolate(tiny, 2.0 * tiny), 0.0);

        InterpolatingTable2D tinyValues = InterpolatingTable2D.ofSorted(
                new double[]{0.0, 1.0}, new double[]{0.0, 1.0},
                new double[][]{{-2.0 * tiny, 0.0}, {0.0, 2.0 * tiny}});
        assertEquals(0.0, tinyValues.interpolate(0.5, 0.5), 0.0);
        assertBits(-2.0 * tiny, tinyValues.interpolate(0.0, 0.0));
        assertBits(2.0 * tiny, tinyValues.interpolate(1.0, 1.0));
    }

    @Test
    public void documentedEvaluationOrderBlendsSecondAxisBeforeFirstAxis() {
        // Floating-point rounding distinguishes the specified order from the reverse order.
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{0.0, 1.0}, new double[]{0.0, 1.0},
                new double[][]{{1e16, 1.0}, {1.0, 1.0}});
        assertBits(3_750_000_000_000_000.0, table.interpolate(0.25, 0.5));
        assertBits(3_750_000_000_000_000.0, table.applyAsDouble(0.25, 0.5));
    }

    @Test
    public void publicSurfaceHasOnlyOneSortedFactoryParallelQueriesAndDiagnostics() throws Exception {
        assertTrue(Modifier.isFinal(InterpolatingTable2D.class.getModifiers()));
        assertTrue(DoubleBinaryOperator.class.isAssignableFrom(InterpolatingTable2D.class));
        assertEquals(0, InterpolatingTable2D.class.getConstructors().length);
        assertEquals(0, InterpolatingTable2D.class.getFields().length);
        assertEquals(0, InterpolatingTable2D.class.getDeclaredClasses().length);
        Set<String> publicMethods = new HashSet<>();
        int publicMethodCount = 0;
        for (Method method : InterpolatingTable2D.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethods.add(method.getName());
                publicMethodCount++;
            }
        }
        assertEquals(5, publicMethodCount);
        assertEquals(new HashSet<>(Arrays.asList(
                "ofSorted", "interpolate", "applyAsDouble", "debugDump", "toString")),
                publicMethods);
        Method factory = InterpolatingTable2D.class.getDeclaredMethod(
                "ofSorted", double[].class, double[].class, double[][].class);
        assertTrue(Modifier.isPublic(factory.getModifiers()));
        assertTrue(Modifier.isStatic(factory.getModifiers()));
        assertFalse(factory.isVarArgs());
        assertEquals(InterpolatingTable2D.class, factory.getReturnType());
        Constructor<?>[] constructors = InterpolatingTable2D.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));
        assertEquals(double.class, InterpolatingTable2D.class.getDeclaredMethod(
                "interpolate", double.class, double.class).getReturnType());
        assertEquals(double.class, InterpolatingTable2D.class.getDeclaredMethod(
                "applyAsDouble", double.class, double.class).getReturnType());
        assertEquals(void.class, InterpolatingTable2D.class.getDeclaredMethod(
                "debugDump", DebugSink.class, String.class).getReturnType());
    }

    @Test
    public void diagnosticsReportSampleCountAndBothRangesWithoutChangingQueryResults() {
        InterpolatingTable2D table = InterpolatingTable2D.ofSorted(
                new double[]{10.0, 30.0}, new double[]{-5.0, 5.0, 20.0},
                new double[][]{{5.0, -10.0, 30.0}, {105.0, 50.0, -20.0}});
        double before = table.interpolate(15.0, 12.5);
        table.debugDump(null, "ignored");
        CapturingDebugSink named = new CapturingDebugSink();
        table.debugDump(named, "lookup.shooter");
        assertEquals(new HashSet<>(Arrays.asList("lookup.shooter.size", "lookup.shooter.xMin",
                "lookup.shooter.xMax", "lookup.shooter.yMin", "lookup.shooter.yMax")),
                named.values.keySet());
        assertEquals(Long.valueOf(6L), named.values.get("lookup.shooter.size"));
        assertEquals(10.0, number(named, "lookup.shooter.xMin"), 0.0);
        assertEquals(30.0, number(named, "lookup.shooter.xMax"), 0.0);
        assertEquals(-5.0, number(named, "lookup.shooter.yMin"), 0.0);
        assertEquals(20.0, number(named, "lookup.shooter.yMax"), 0.0);
        for (String prefix : new String[]{null, ""}) {
            CapturingDebugSink defaults = new CapturingDebugSink();
            table.debugDump(defaults, prefix);
            assertEquals(new HashSet<>(Arrays.asList(
                    "table.size", "table.xMin", "table.xMax", "table.yMin", "table.yMax")),
                    defaults.values.keySet());
            assertEquals(Long.valueOf(6L), defaults.values.get("table.size"));
        }
        String description = table.toString();
        assertTrue(description.contains("InterpolatingTable2D"));
        assertTrue(description.contains("[10.0, 30.0]"));
        assertTrue(description.contains("[-5.0, 5.0, 20.0]"));
        assertTrue(description.contains("[5.0, -10.0, 30.0]"));
        assertTrue(description.contains("[105.0, 50.0, -20.0]"));
        assertBits(before, table.interpolate(15.0, 12.5));
    }

    /** All non-finite inputs are missing runtime evidence, never endpoint requests. */
    private static double[] unavailableValues() {
        return new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
    }

    private static void assertBits(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    private static void assertContains(Throwable failure, String... fragments) {
        assertNotNull(failure.getMessage());
        for (String fragment : fragments) {
            assertTrue("Expected message to contain '" + fragment + "' but was: "
                    + failure.getMessage(), failure.getMessage().contains(fragment));
        }
    }

    private static double number(CapturingDebugSink sink, String key) {
        Object value = sink.values.get(key);
        assertTrue("Expected numeric debug value for " + key, value instanceof Number);
        return ((Number) value).doubleValue();
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
