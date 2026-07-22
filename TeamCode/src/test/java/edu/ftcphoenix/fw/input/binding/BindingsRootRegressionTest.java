package edu.ftcphoenix.fw.input.binding;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Protects the ordinary root-binding behavior while contextual controls are added. */
public final class BindingsRootRegressionTest {

    @Test
    public void rootBindingsPreserveEdgesLevelsStatefulHelpersAndPhaseOrder() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] button = {false};
        boolean[] other = {false};
        double[] scalar = {0.25};
        List<String> calls = new ArrayList<>();

        bindings.onRise(BooleanSource.of(() -> button[0]), () -> calls.add("rise-1"));
        bindings.onRise(BooleanSource.of(() -> button[0]), () -> calls.add("rise-2"));
        bindings.onFall(BooleanSource.of(() -> button[0]), () -> calls.add("fall"));
        bindings.mirrorOnChange(BooleanSource.of(() -> button[0]),
                value -> calls.add("mirror-" + value));
        bindings.whileHigh(BooleanSource.of(() -> button[0]), () -> calls.add("high"));
        bindings.whileLow(BooleanSource.of(() -> button[0]), () -> calls.add("low"));
        bindings.toggleOnRise(BooleanSource.of(() -> button[0]),
                value -> calls.add("toggle-" + value));
        bindings.nudgeOnRise(
                BooleanSource.of(() -> button[0]),
                BooleanSource.of(() -> other[0]),
                0.5,
                value -> calls.add("nudge-" + value));
        bindings.copyEachCycle(ScalarSource.of(() -> scalar[0]),
                value -> calls.add("scalar-" + value));

        update(bindings, manualClock);
        assertEquals(Arrays.asList("mirror-false", "low", "scalar-0.25"), calls);

        calls.clear();
        button[0] = true;
        scalar[0] = Double.NaN;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(
                "rise-1", "rise-2", "mirror-true", "high", "toggle-true",
                "nudge-0.5", "scalar-NaN"), calls);

        calls.clear();
        button[0] = false;
        other[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList("fall", "mirror-false", "low", "nudge--0.5", "scalar-NaN"),
                calls);

        calls.clear();
        other[0] = false;
        update(bindings, manualClock);
        button[0] = true;
        update(bindings, manualClock);
        assertTrue(calls.contains("toggle-false"));
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
}
