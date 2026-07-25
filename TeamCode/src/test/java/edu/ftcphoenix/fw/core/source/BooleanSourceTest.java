package edu.ftcphoenix.fw.core.source;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies eager, deterministic Boolean-source composition and stateful child behavior. */
public final class BooleanSourceTest {

    @Test
    public void andAndOrPreserveBooleanTruthTables() {
        LoopClock clock = new ManualLoopClock().clock();

        for (boolean left : new boolean[]{false, true}) {
            for (boolean right : new boolean[]{false, true}) {
                assertEquals(
                        left && right,
                        BooleanSource.constant(left)
                                .and(BooleanSource.constant(right))
                                .getAsBoolean(clock)
                );
                assertEquals(
                        left || right,
                        BooleanSource.constant(left)
                                .or(BooleanSource.constant(right))
                                .getAsBoolean(clock)
                );
            }
        }
    }

    @Test
    public void decisiveLeftValueStillSamplesRightOperand() {
        LoopClock clock = new ManualLoopClock().clock();
        ProbeBooleanSource andLeft = new ProbeBooleanSource("andLeft", false);
        ProbeBooleanSource andRight = new ProbeBooleanSource("andRight", true);
        ProbeBooleanSource orLeft = new ProbeBooleanSource("orLeft", true);
        ProbeBooleanSource orRight = new ProbeBooleanSource("orRight", false);

        assertFalse(andLeft.and(andRight).getAsBoolean(clock));
        assertTrue(orLeft.or(orRight).getAsBoolean(clock));

        assertEquals(1, andLeft.sampleCount);
        assertEquals(1, andRight.sampleCount);
        assertEquals(1, orLeft.sampleCount);
        assertEquals(1, orRight.sampleCount);
    }

    @Test
    public void threeOperandGraphsSampleLeftToRightEvenWhenFirstValueIsDecisive() {
        LoopClock clock = new ManualLoopClock().clock();
        List<String> events = new ArrayList<>();
        ProbeBooleanSource first = new ProbeBooleanSource("first", false, events);
        ProbeBooleanSource second = new ProbeBooleanSource("second", true, events);
        ProbeBooleanSource third = new ProbeBooleanSource("third", true, events);

        assertFalse(first.and(second).and(third).getAsBoolean(clock));
        assertEquals(Arrays.asList("sample:first", "sample:second", "sample:third"), events);

        events.clear();
        first.value = true;
        second.value = false;
        third.value = false;
        assertTrue(first.or(second).or(third).getAsBoolean(clock));
        assertEquals(Arrays.asList("sample:first", "sample:second", "sample:third"), events);
    }

    @Test
    public void debouncedRightOperandAccumulatesReadinessWhileAndIsMasked() {
        ManualLoopClock time = new ManualLoopClock();
        ProbeBooleanSource enabled = new ProbeBooleanSource("enabled", false);
        ProbeBooleanSource rawReady = new ProbeBooleanSource("rawReady", false);
        BooleanSource ready = rawReady.debouncedOn(0.10);
        BooleanSource allowed = enabled.and(ready);

        assertFalse(allowed.getAsBoolean(time.clock()));

        rawReady.value = true;
        time.nextCycle(0.06);
        assertFalse(allowed.getAsBoolean(time.clock()));
        time.nextCycle(0.06);
        assertFalse(allowed.getAsBoolean(time.clock()));

        enabled.value = true;
        time.nextCycle(0.01);
        assertTrue(allowed.getAsBoolean(time.clock()));
        assertEquals(4, rawReady.sampleCount);
    }

    @Test
    public void maskedRisingEdgeIsConsumedInsteadOfReplayedWhenAndReopens() {
        ManualLoopClock time = new ManualLoopClock();
        ProbeBooleanSource enabled = new ProbeBooleanSource("enabled", true);
        ProbeBooleanSource button = new ProbeBooleanSource("button", false);
        BooleanSource gatedEdge = enabled.and(button.risingEdge());

        assertFalse(gatedEdge.getAsBoolean(time.clock()));

        enabled.value = false;
        button.value = true;
        time.nextCycle(0.02);
        assertFalse(gatedEdge.getAsBoolean(time.clock()));

        enabled.value = true;
        time.nextCycle(0.02);
        assertFalse(gatedEdge.getAsBoolean(time.clock()));
        assertEquals(3, button.sampleCount);
    }

    @Test
    public void toggledRightOperandRetainsPulseObservedWhileAndIsMasked() {
        ManualLoopClock time = new ManualLoopClock();
        ProbeBooleanSource enabled = new ProbeBooleanSource("enabled", true);
        ProbeBooleanSource button = new ProbeBooleanSource("button", false);
        BooleanSource gatedToggle = enabled.and(button.toggled());

        assertFalse(gatedToggle.getAsBoolean(time.clock()));

        enabled.value = false;
        button.value = true;
        time.nextCycle(0.02);
        assertFalse(gatedToggle.getAsBoolean(time.clock()));

        button.value = false;
        time.nextCycle(0.02);
        assertFalse(gatedToggle.getAsBoolean(time.clock()));

        enabled.value = true;
        time.nextCycle(0.02);
        assertTrue(gatedToggle.getAsBoolean(time.clock()));
        assertEquals(4, button.sampleCount);
    }

    @Test
    public void repeatedCompositeReadsDoNotAdvanceStatefulRightOperandTwicePerCycle() {
        ManualLoopClock time = new ManualLoopClock();
        ProbeBooleanSource button = new ProbeBooleanSource("button", false);
        BooleanSource toggled = BooleanSource.constant(true).and(button.toggled());

        assertFalse(toggled.getAsBoolean(time.clock()));
        assertFalse(toggled.getAsBoolean(time.clock()));
        assertEquals(1, button.sampleCount);

        button.value = true;
        time.nextCycle(0.02);
        assertTrue(toggled.getAsBoolean(time.clock()));
        assertTrue(toggled.getAsBoolean(time.clock()));
        assertEquals(2, button.sampleCount);
    }

    @Test
    public void pureLogicalCompositesResampleBothOperandsWithinOneCycle() {
        LoopClock clock = new ManualLoopClock().clock();
        ProbeBooleanSource andLeft = new ProbeBooleanSource("andLeft", true);
        ProbeBooleanSource andRight = new ProbeBooleanSource("andRight", true);
        BooleanSource and = andLeft.and(andRight);

        assertTrue(and.getAsBoolean(clock));
        andLeft.value = false;
        andRight.value = false;
        assertFalse(and.getAsBoolean(clock));
        assertEquals(2, andLeft.sampleCount);
        assertEquals(2, andRight.sampleCount);

        ProbeBooleanSource orLeft = new ProbeBooleanSource("orLeft", false);
        ProbeBooleanSource orRight = new ProbeBooleanSource("orRight", false);
        BooleanSource or = orLeft.or(orRight);

        assertFalse(or.getAsBoolean(clock));
        orLeft.value = true;
        orRight.value = true;
        assertTrue(or.getAsBoolean(clock));
        assertEquals(2, orLeft.sampleCount);
        assertEquals(2, orRight.sampleCount);
    }

    @Test
    public void rawRightFailureIsNotCachedByPureCompositeWithinOneCycle() {
        LoopClock clock = new ManualLoopClock().clock();
        RuntimeException expectedFailure = new IllegalStateException("first right sample failed");
        ProbeBooleanSource left = new ProbeBooleanSource("left", false);
        final int[] rightSamples = {0};
        BooleanSource right = new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock ignored) {
                rightSamples[0]++;
                if (rightSamples[0] == 1) {
                    throw expectedFailure;
                }
                return true;
            }
        };
        BooleanSource composite = left.and(right);

        assertSame(expectedFailure, captureFailure(() -> composite.getAsBoolean(clock)));
        assertFalse(composite.getAsBoolean(clock));
        assertEquals(2, left.sampleCount);
        assertEquals(2, rightSamples[0]);
    }

    @Test
    public void logicalCompositionResetPropagatesLeftToRight() {
        List<String> events = new ArrayList<>();
        ProbeBooleanSource andLeft = new ProbeBooleanSource("andLeft", false, events);
        ProbeBooleanSource andRight = new ProbeBooleanSource("andRight", true, events);

        andLeft.and(andRight).reset();

        assertEquals(Arrays.asList("reset:andLeft", "reset:andRight"), events);
        assertEquals(1, andLeft.resetCount);
        assertEquals(1, andRight.resetCount);

        events.clear();
        ProbeBooleanSource orLeft = new ProbeBooleanSource("orLeft", true, events);
        ProbeBooleanSource orRight = new ProbeBooleanSource("orRight", false, events);

        orLeft.or(orRight).reset();

        assertEquals(Arrays.asList("reset:orLeft", "reset:orRight"), events);
        assertEquals(1, orLeft.resetCount);
        assertEquals(1, orRight.resetCount);
    }

    @Test
    public void rightFailureIsVisibleEvenWhenLeftValueDeterminesResult() {
        LoopClock clock = new ManualLoopClock().clock();
        List<String> events = new ArrayList<>();
        RuntimeException andFailure = new IllegalStateException("and right failed");
        ProbeBooleanSource andLeft = new ProbeBooleanSource("andLeft", false, events);
        BooleanSource andRight = throwingSource("andRight", andFailure, events);

        assertSame(andFailure, captureFailure(() -> andLeft.and(andRight).getAsBoolean(clock)));
        assertEquals(Arrays.asList("sample:andLeft", "sample:andRight"), events);

        events.clear();
        RuntimeException orFailure = new IllegalArgumentException("or right failed");
        ProbeBooleanSource orLeft = new ProbeBooleanSource("orLeft", true, events);
        BooleanSource orRight = throwingSource("orRight", orFailure, events);

        assertSame(orFailure, captureFailure(() -> orLeft.or(orRight).getAsBoolean(clock)));
        assertEquals(Arrays.asList("sample:orLeft", "sample:orRight"), events);
    }

    @Test
    public void leftFailureStopsBeforeRightOperand() {
        LoopClock clock = new ManualLoopClock().clock();
        List<String> events = new ArrayList<>();
        RuntimeException andFailure = new IllegalStateException("and left failed");
        BooleanSource andLeft = throwingSource("andLeft", andFailure, events);
        ProbeBooleanSource andRight = new ProbeBooleanSource("andRight", true, events);

        assertSame(andFailure, captureFailure(() -> andLeft.and(andRight).getAsBoolean(clock)));
        assertEquals(Arrays.asList("sample:andLeft"), events);
        assertEquals(0, andRight.sampleCount);

        events.clear();
        RuntimeException orFailure = new IllegalArgumentException("or left failed");
        BooleanSource orLeft = throwingSource("orLeft", orFailure, events);
        ProbeBooleanSource orRight = new ProbeBooleanSource("orRight", false, events);

        assertSame(orFailure, captureFailure(() -> orLeft.or(orRight).getAsBoolean(clock)));
        assertEquals(Arrays.asList("sample:orLeft"), events);
        assertEquals(0, orRight.sampleCount);
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected source sampling to fail");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static BooleanSource throwingSource(
            String name,
            RuntimeException failure,
            List<String> events
    ) {
        return new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                events.add("sample:" + name);
                throw failure;
            }
        };
    }

    private static final class ProbeBooleanSource implements BooleanSource {
        private final String name;
        private final List<String> events;
        private boolean value;
        private int sampleCount;
        private int resetCount;

        private ProbeBooleanSource(String name, boolean value) {
            this(name, value, null);
        }

        private ProbeBooleanSource(String name, boolean value, List<String> events) {
            this.name = name;
            this.value = value;
            this.events = events;
        }

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            sampleCount++;
            if (events != null) {
                events.add("sample:" + name);
            }
            return value;
        }

        @Override
        public void reset() {
            resetCount++;
            if (events != null) {
                events.add("reset:" + name);
            }
        }
    }
}
