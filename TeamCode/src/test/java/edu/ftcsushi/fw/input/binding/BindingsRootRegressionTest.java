package edu.ftcsushi.fw.input.binding;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Protects the ordinary root-binding behavior while contextual controls are added. */
public final class BindingsRootRegressionTest {

    @Test
    public void rootBindingsPreserveSemanticsAndUseGlobalDeclarationOrder() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] button = {false};
        boolean[] other = {false};
        double[] scalar = {0.25};
        List<String> calls = new ArrayList<>();

        // Intentionally scramble every binding kind. The declarations, not their kinds, define
        // the order of samples and callbacks.
        bindings.copyEachCycle(ScalarSource.of(() -> scalar[0]),
                value -> calls.add("scalar-" + value));
        bindings.whileLow(BooleanSource.of(() -> button[0]), () -> calls.add("low"));
        bindings.onFall(BooleanSource.of(() -> button[0]), () -> calls.add("fall"));
        bindings.nudgeOnRise(
                BooleanSource.of(() -> button[0]),
                BooleanSource.of(() -> other[0]),
                0.5,
                value -> calls.add("nudge-" + value));
        bindings.mirrorOnChange(BooleanSource.of(() -> button[0]),
                value -> calls.add("mirror-" + value));
        bindings.onRise(BooleanSource.of(() -> button[0]), () -> calls.add("rise-1"));
        bindings.toggleOnRise(BooleanSource.of(() -> button[0]),
                value -> calls.add("toggle-" + value));
        bindings.whileHigh(BooleanSource.of(() -> button[0]), () -> calls.add("high"));
        bindings.onRise(BooleanSource.of(() -> button[0]), () -> calls.add("rise-2"));

        update(bindings, manualClock);
        assertEquals(Arrays.asList("scalar-0.25", "low", "mirror-false"), calls);

        calls.clear();
        button[0] = true;
        scalar[0] = Double.NaN;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(
                "scalar-NaN", "nudge-0.5", "mirror-true", "rise-1", "toggle-true",
                "high", "rise-2"), calls);

        calls.clear();
        button[0] = false;
        other[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList("scalar-NaN", "low", "fall", "nudge--0.5", "mirror-false"),
                calls);

        calls.clear();
        other[0] = false;
        update(bindings, manualClock);
        button[0] = true;
        update(bindings, manualClock);
        assertTrue(calls.contains("toggle-false"));
    }

    @Test
    public void ordinarySourcesAreSampledAtTheirDeclarationPositions() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        List<String> trace = new ArrayList<>();

        bindings.copyEachCycle(
                ScalarSource.of(() -> {
                    trace.add("scalar-sample");
                    return 0.5;
                }),
                value -> trace.add("scalar-callback"));
        bindings.whileHigh(
                BooleanSource.of(() -> {
                    trace.add("level-sample");
                    return true;
                }),
                () -> trace.add("level-callback"));
        bindings.mirrorOnChange(
                BooleanSource.of(() -> {
                    trace.add("mirror-sample");
                    return false;
                }),
                value -> trace.add("mirror-callback"));

        update(bindings, manualClock);

        assertEquals(Arrays.asList(
                "scalar-sample", "scalar-callback",
                "level-sample", "level-callback",
                "mirror-sample", "mirror-callback"), trace);
    }

    @Test
    public void structuralMutationDuringUpdateFailsBeforeChangingGraph() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        List<String> failures = new ArrayList<>();
        int[] visits = {0};

        bindings.whileHigh(BooleanSource.constant(true), () -> {
            visits[0]++;
            failures.add(captureIllegalState(() -> bindings.onRise(
                    BooleanSource.constant(false), () -> { })));
        });
        bindings.whileHigh(BooleanSource.constant(true), () -> {
            visits[0]++;
            failures.add(captureIllegalState(() -> bindings.contextWhen(
                    BooleanSource.constant(true), Bindings.ActivationPolicy.ACCEPT_CURRENT)));
        });
        bindings.whileHigh(BooleanSource.constant(true), () -> {
            visits[0]++;
            failures.add(captureIllegalState(bindings::clear));
        });

        update(bindings, manualClock);

        assertEquals(3, visits[0]);
        assertEquals(3, failures.size());
        assertTrue(failures.get(0).contains("cannot register a binding"));
        assertTrue(failures.get(1).contains("cannot create a control context"));
        assertTrue(failures.get(2).contains("cannot clear bindings"));
        for (String failure : failures) {
            assertTrue(failure.contains("before or after update"));
        }

        CapturingDebugSink debug = new CapturingDebugSink();
        bindings.debugDump(debug, "test");
        assertDebugCount(debug, "test.contexts", 0);
        assertDebugCount(debug, "test.rise", 0);
        assertDebugCount(debug, "test.level", 3);

        update(bindings, manualClock);
        assertEquals("caught failures must leave the original traversal intact", 6, visits[0]);
    }

    @Test
    public void structuralMutationGuardIsReleasedWhenACallbackThrows() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();

        bindings.whileHigh(BooleanSource.constant(true), () -> {
            throw new RuntimeException("callback failure");
        });

        manualClock.nextCycle(0.02);
        try {
            bindings.update(manualClock.clock());
            fail("expected callback failure");
        } catch (RuntimeException expected) {
            assertEquals("callback failure", expected.getMessage());
        }

        // API-04 owns only the structural guard. INPUT-02 separately owns failure retention.
        bindings.clear();
        bindings.onRise(BooleanSource.constant(false), () -> { });

        CapturingDebugSink debug = new CapturingDebugSink();
        bindings.debugDump(debug, "test");
        assertDebugCount(debug, "test.rise", 1);
        assertDebugCount(debug, "test.level", 0);
    }

    @Test
    public void debugCountsReflectAllRegistrarsAndResetOnClear() {
        Bindings bindings = new Bindings();
        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.constant(true), Bindings.ActivationPolicy.ACCEPT_CURRENT);

        bindings.onRise(BooleanSource.constant(false), () -> { });
        context.onRise(BooleanSource.constant(false), () -> { });
        bindings.onFall(BooleanSource.constant(false), () -> { });
        bindings.mirrorOnChange(BooleanSource.constant(false), ignored -> { });
        bindings.whileHigh(BooleanSource.constant(false), () -> { });
        bindings.whileLow(BooleanSource.constant(false), () -> { });
        bindings.toggleOnRise(BooleanSource.constant(false), ignored -> { });
        bindings.nudgeOnRise(
                BooleanSource.constant(false), BooleanSource.constant(false), 1.0, ignored -> { });
        bindings.copyEachCycle(ScalarSource.constant(0.0), ignored -> { });

        CapturingDebugSink beforeClear = new CapturingDebugSink();
        bindings.debugDump(beforeClear, "test");
        assertDebugCount(beforeClear, "test.contexts", 1);
        assertDebugCount(beforeClear, "test.rise", 2);
        assertDebugCount(beforeClear, "test.fall", 1);
        assertDebugCount(beforeClear, "test.mirrorOnChange", 1);
        assertDebugCount(beforeClear, "test.level", 2);
        assertDebugCount(beforeClear, "test.toggleOnRise", 1);
        assertDebugCount(beforeClear, "test.nudgeOnRise", 1);
        assertDebugCount(beforeClear, "test.copyEachCycle", 1);

        bindings.clear();

        CapturingDebugSink afterClear = new CapturingDebugSink();
        bindings.debugDump(afterClear, "test");
        assertDebugCount(afterClear, "test.contexts", 0);
        assertDebugCount(afterClear, "test.rise", 0);
        assertDebugCount(afterClear, "test.fall", 0);
        assertDebugCount(afterClear, "test.mirrorOnChange", 0);
        assertDebugCount(afterClear, "test.level", 0);
        assertDebugCount(afterClear, "test.toggleOnRise", 0);
        assertDebugCount(afterClear, "test.nudgeOnRise", 0);
        assertDebugCount(afterClear, "test.copyEachCycle", 0);
    }

    @Test
    public void repeatedRootUpdateInSameCycleIsANoop() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        int[] levelCalls = {0};
        int[] scalarCalls = {0};

        bindings.whileHigh(BooleanSource.constant(true), () -> levelCalls[0]++);
        bindings.copyEachCycle(ScalarSource.constant(1.0), value -> scalarCalls[0]++);

        bindings.update(manualClock.clock());
        bindings.update(manualClock.clock());

        assertEquals(1, levelCalls[0]);
        assertEquals(1, scalarCalls[0]);
    }

    private static void update(Bindings bindings, ManualLoopClock manualClock) {
        manualClock.nextCycle(0.02);
        bindings.update(manualClock.clock());
    }

    private static String captureIllegalState(Runnable mutation) {
        try {
            mutation.run();
            fail("expected structural mutation during Bindings.update to fail");
            return "";
        } catch (IllegalStateException expected) {
            return expected.getMessage();
        }
    }

    private static void assertDebugCount(CapturingDebugSink debug, String key, int expected) {
        assertEquals(expected, ((Number) debug.data.get(key)).intValue());
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> data = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            data.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
