package edu.ftcphoenix.fw.input.binding;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the approved activation, rearm, overlap, and neutral-output contracts. */
public final class BindingsControlContextTest {

    @Test
    public void rearmPolicyRequiresNeutralWithoutManufacturingEdgesOrLevels() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {false};
        boolean[] button = {true};
        int[] rises = {0};
        int[] falls = {0};
        int[] highs = {0};
        int[] lows = {0};
        List<Boolean> toggles = new ArrayList<>();

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        context.onRise(BooleanSource.of(() -> button[0]), () -> rises[0]++);
        context.onFall(BooleanSource.of(() -> button[0]), () -> falls[0]++);
        context.whileHigh(BooleanSource.of(() -> button[0]), () -> highs[0]++);
        context.whileLow(BooleanSource.of(() -> button[0]), () -> lows[0]++);
        context.toggleOnRise(BooleanSource.of(() -> button[0]), toggles::add);

        update(bindings, manualClock); // Inactive.
        active[0] = true;
        update(bindings, manualClock); // Held activation frame.
        update(bindings, manualClock); // Still held and unarmed.
        assertCounts(rises, falls, highs, lows, 0, 0, 0, 0);
        assertTrue(toggles.isEmpty());

        button[0] = false;
        update(bindings, manualClock); // Neutral rearms, with no low/fall effect.
        assertCounts(rises, falls, highs, lows, 0, 0, 0, 0);

        update(bindings, manualClock);
        assertCounts(rises, falls, highs, lows, 0, 0, 0, 1);

        button[0] = true;
        update(bindings, manualClock);
        assertCounts(rises, falls, highs, lows, 1, 0, 1, 1);
        assertEquals(Arrays.asList(true), toggles);

        button[0] = false;
        update(bindings, manualClock);
        assertCounts(rises, falls, highs, lows, 1, 1, 1, 2);

        active[0] = false;
        update(bindings, manualClock);
        active[0] = true;
        button[0] = false;
        update(bindings, manualClock); // Reactivation frame rearms, toggle state persists.
        button[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(true, false), toggles);
    }

    @Test
    public void acceptCurrentUsesEffectFreeBaselineThenExposesHeldLevelsAndRealFall() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {false};
        boolean[] button = {true};
        int[] rises = {0};
        int[] falls = {0};
        int[] highs = {0};
        int[] lows = {0};
        List<Boolean> mirrors = new ArrayList<>();

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.ACCEPT_CURRENT);
        context.onRise(BooleanSource.of(() -> button[0]), () -> rises[0]++);
        context.onFall(BooleanSource.of(() -> button[0]), () -> falls[0]++);
        context.whileHigh(BooleanSource.of(() -> button[0]), () -> highs[0]++);
        context.whileLow(BooleanSource.of(() -> button[0]), () -> lows[0]++);
        context.mirrorOnChange(BooleanSource.of(() -> button[0]), mirrors::add);

        update(bindings, manualClock);
        assertEquals(Arrays.asList(false), mirrors);

        active[0] = true;
        update(bindings, manualClock); // Effect-free activation; false neutral is already published.
        assertCounts(rises, falls, highs, lows, 0, 0, 0, 0);
        assertEquals(Arrays.asList(false), mirrors);

        update(bindings, manualClock);
        assertCounts(rises, falls, highs, lows, 0, 0, 1, 0);
        assertEquals(Arrays.asList(false, true), mirrors);

        button[0] = false;
        update(bindings, manualClock);
        assertCounts(rises, falls, highs, lows, 0, 1, 1, 1);
        assertEquals(Arrays.asList(false, true, false), mirrors);

        button[0] = true;
        update(bindings, manualClock);
        assertCounts(rises, falls, highs, lows, 1, 1, 2, 1);
    }

    @Test
    public void deactivationNeutralizesMirrorOnceWithoutCreatingFallOrLowCallbacks() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {true};
        boolean[] button = {true};
        List<Boolean> mirrors = new ArrayList<>();
        int[] falls = {0};
        int[] lows = {0};

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.ACCEPT_CURRENT);
        context.mirrorOnChange(BooleanSource.of(() -> button[0]), mirrors::add);
        context.onFall(BooleanSource.of(() -> button[0]), () -> falls[0]++);
        context.whileLow(BooleanSource.of(() -> button[0]), () -> lows[0]++);

        update(bindings, manualClock); // Activation publishes only effective neutral.
        update(bindings, manualClock); // Current held value becomes visible.
        active[0] = false;
        update(bindings, manualClock); // Deactivation publishes neutral false.
        update(bindings, manualClock); // No duplicate neutral callback.

        assertEquals(Arrays.asList(false, true, false), mirrors);
        assertEquals(0, falls[0]);
        assertEquals(0, lows[0]);
    }

    @Test
    public void rearmMirrorStaysNeutralWhenReactivatedHeldUntilFreshReleaseAndPress() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {true};
        boolean[] button = {false};
        List<Boolean> mirrors = new ArrayList<>();

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        context.mirrorOnChange(BooleanSource.of(() -> button[0]), mirrors::add);

        update(bindings, manualClock); // Neutral activation frame rearms and publishes false.
        button[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(false, true), mirrors);

        active[0] = false;
        update(bindings, manualClock); // Deactivation neutralizes the true mirror.
        active[0] = true;
        update(bindings, manualClock); // Held reactivation remains neutral and unarmed.
        update(bindings, manualClock);
        assertEquals(Arrays.asList(false, true, false), mirrors);

        button[0] = false;
        update(bindings, manualClock); // Release rearms without a mirror callback.
        button[0] = true;
        update(bindings, manualClock); // A fresh press can become visible again.
        assertEquals(Arrays.asList(false, true, false, true), mirrors);
    }

    @Test
    public void nudgeInputsRearmIndependently() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {true};
        boolean[] increase = {true};
        boolean[] decrease = {false};
        List<Double> deltas = new ArrayList<>();

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        context.nudgeOnRise(
                BooleanSource.of(() -> increase[0]),
                BooleanSource.of(() -> decrease[0]),
                0.25,
                deltas::add);

        update(bindings, manualClock); // Increase is held; decrease independently rearms.
        update(bindings, manualClock);
        decrease[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(-0.25), deltas);

        decrease[0] = false;
        increase[0] = false;
        update(bindings, manualClock); // Increase finally rearms.
        increase[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(-0.25, 0.25), deltas);
    }

    @Test
    public void acceptCurrentToggleAndNudgeUseBaselinesAndRetainToggleAcrossReactivation() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {false};
        boolean[] toggleSignal = {true};
        boolean[] increase = {true};
        boolean[] decrease = {false};
        List<Boolean> toggles = new ArrayList<>();
        List<Double> deltas = new ArrayList<>();

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.ACCEPT_CURRENT);
        context.toggleOnRise(BooleanSource.of(() -> toggleSignal[0]), toggles::add);
        context.nudgeOnRise(
                BooleanSource.of(() -> increase[0]),
                BooleanSource.of(() -> decrease[0]),
                0.5,
                deltas::add);

        update(bindings, manualClock);
        active[0] = true;
        update(bindings, manualClock); // Held inputs establish effect-free baselines.
        update(bindings, manualClock);
        assertTrue(toggles.isEmpty());
        assertTrue(deltas.isEmpty());

        toggleSignal[0] = false;
        increase[0] = false;
        update(bindings, manualClock);
        toggleSignal[0] = true;
        increase[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(true), toggles);
        assertEquals(Arrays.asList(0.5), deltas);

        increase[0] = false;
        decrease[0] = false;
        update(bindings, manualClock);
        increase[0] = true;
        decrease[0] = true;
        update(bindings, manualClock); // Simultaneous nudges cancel without a callback.
        assertEquals(Arrays.asList(0.5), deltas);

        active[0] = false;
        update(bindings, manualClock);
        active[0] = true;
        update(bindings, manualClock); // Held toggle does not flip during reactivation.
        assertEquals(Arrays.asList(true), toggles);
        toggleSignal[0] = false;
        update(bindings, manualClock);
        toggleSignal[0] = true;
        update(bindings, manualClock);
        assertEquals(Arrays.asList(true, false), toggles);
    }

    @Test
    public void scalarRearmPolicyUsesZeroNeutralAndNonFiniteDisarms() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {false};
        double[] input = {0.7};
        int[] samples = {0};
        List<Double> outputs = new ArrayList<>();
        ScalarSource source = clock -> {
            samples[0]++;
            return input[0];
        };

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        context.copyEachCycle(source, outputs::add);

        update(bindings, manualClock);
        update(bindings, manualClock);
        assertEquals(0, samples[0]);
        assertEquals(Arrays.asList(0.0, 0.0), outputs);

        active[0] = true;
        update(bindings, manualClock); // Displaced activation frame.
        update(bindings, manualClock); // Still displaced and unarmed.
        input[0] = 0.0;
        update(bindings, manualClock); // Neutral rearms without passing a live command.
        input[0] = 0.4;
        update(bindings, manualClock);
        assertEquals(0.4, outputs.get(outputs.size() - 1), 0.0);

        input[0] = Double.NaN;
        update(bindings, manualClock);
        assertEquals(0.0, outputs.get(outputs.size() - 1), 0.0);
        input[0] = 0.6;
        update(bindings, manualClock);
        assertEquals(0.0, outputs.get(outputs.size() - 1), 0.0);
        input[0] = -0.0;
        update(bindings, manualClock);
        input[0] = -0.3;
        update(bindings, manualClock);
        assertEquals(-0.3, outputs.get(outputs.size() - 1), 0.0);

        int outputCount = outputs.size();
        int sampleCount = samples[0];
        bindings.update(manualClock.clock());
        assertEquals(outputCount, outputs.size());
        assertEquals(sampleCount, samples[0]);

        active[0] = false;
        update(bindings, manualClock);
        assertEquals(sampleCount, samples[0]);
        assertEquals(0.0, outputs.get(outputs.size() - 1), 0.0);

        input[0] = 0.9;
        active[0] = true;
        update(bindings, manualClock); // Reactivation is effect-free while held nonzero.
        update(bindings, manualClock); // REARM continues publishing zero while displaced.
        assertEquals(0.0, outputs.get(outputs.size() - 1), 0.0);
        input[0] = 0.0;
        update(bindings, manualClock); // Fresh neutral rearms without passing another command.
        input[0] = 0.2;
        update(bindings, manualClock);
        assertEquals(0.2, outputs.get(outputs.size() - 1), 0.0);
    }

    @Test
    public void scalarAcceptCurrentResumesAtNextFiniteSample() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {true};
        double[] input = {0.8};
        List<Double> outputs = new ArrayList<>();

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.ACCEPT_CURRENT);
        context.copyEachCycle(ScalarSource.of(() -> input[0]), outputs::add);

        update(bindings, manualClock);
        assertEquals(Arrays.asList(0.0), outputs);
        update(bindings, manualClock);
        assertEquals(0.8, outputs.get(outputs.size() - 1), 0.0);

        input[0] = Double.POSITIVE_INFINITY;
        update(bindings, manualClock);
        assertEquals(0.0, outputs.get(outputs.size() - 1), 0.0);
        input[0] = 0.35;
        update(bindings, manualClock);
        assertEquals(0.35, outputs.get(outputs.size() - 1), 0.0);

        active[0] = false;
        update(bindings, manualClock);
        input[0] = 0.55;
        active[0] = true;
        update(bindings, manualClock); // Reactivation frame remains zero despite held finite input.
        assertEquals(0.0, outputs.get(outputs.size() - 1), 0.0);
        update(bindings, manualClock); // ACCEPT_CURRENT exposes it beginning next frame.
        assertEquals(0.55, outputs.get(outputs.size() - 1), 0.0);
    }

    @Test
    public void activationIsSnapshottedBeforeCallbacksAndSampledOncePerCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {false};
        boolean[] modeButton = {false};
        boolean[] disableButton = {false};
        boolean[] contextualButton = {false};
        int[] activationSamples = {0};
        int[] contextualRises = {0};
        int[] contextualHighs = {0};

        BooleanSource activation = clock -> {
            activationSamples[0]++;
            return active[0];
        };
        Bindings.ControlContext context = bindings.contextWhen(
                activation,
                Bindings.ActivationPolicy.ACCEPT_CURRENT);
        bindings.onRise(BooleanSource.of(() -> modeButton[0]), () -> active[0] = true);
        bindings.onRise(BooleanSource.of(() -> disableButton[0]), () -> active[0] = false);
        context.onRise(BooleanSource.of(() -> contextualButton[0]), () -> contextualRises[0]++);
        context.whileHigh(BooleanSource.of(() -> contextualButton[0]), () -> contextualHighs[0]++);

        update(bindings, manualClock);
        modeButton[0] = true;
        contextualButton[0] = true;
        update(bindings, manualClock); // Root callback changes mode after the false snapshot.
        assertEquals(0, contextualRises[0]);
        assertEquals(0, contextualHighs[0]);

        update(bindings, manualClock); // Context activation frame.
        update(bindings, manualClock); // Held level is now accepted, but no manufactured rise.
        assertEquals(0, contextualRises[0]);
        assertEquals(1, contextualHighs[0]);
        assertEquals(4, activationSamples[0]);

        disableButton[0] = true;
        update(bindings, manualClock); // True snapshot remains in force after callback disables mode.
        assertEquals(2, contextualHighs[0]);
        update(bindings, manualClock); // The false mode becomes visible on this next snapshot.
        assertEquals(2, contextualHighs[0]);
        assertEquals(6, activationSamples[0]);

        bindings.update(manualClock.clock());
        assertEquals(6, activationSamples[0]);
    }

    @Test
    public void rootEmergencyActionRemainsEligibleWhenContextDeactivates() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] active = {true};
        boolean[] contextualButton = {false};
        boolean[] abortButton = {false};
        int[] contextualStarts = {0};
        int[] aborts = {0};

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        context.onRise(
                BooleanSource.of(() -> contextualButton[0]),
                () -> contextualStarts[0]++);
        bindings.onRise(BooleanSource.of(() -> abortButton[0]), () -> aborts[0]++);

        update(bindings, manualClock); // Arm the contextual control from neutral.
        contextualButton[0] = true;
        update(bindings, manualClock);
        assertEquals(1, contextualStarts[0]);

        active[0] = false;
        abortButton[0] = true;
        update(bindings, manualClock);

        assertEquals("context deactivation must not suppress a root emergency action", 1, aborts[0]);
        assertEquals(1, contextualStarts[0]);
    }

    @Test
    public void overlappingContextsAllRunWithoutHiddenWinner() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        boolean[] button = {false};
        int[] first = {0};
        int[] second = {0};

        Bindings.ControlContext firstContext = bindings.contextWhen(
                BooleanSource.constant(true), Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        Bindings.ControlContext secondContext = bindings.contextWhen(
                BooleanSource.constant(true), Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        firstContext.onRise(BooleanSource.of(() -> button[0]), () -> first[0]++);
        secondContext.onRise(BooleanSource.of(() -> button[0]), () -> second[0]++);

        update(bindings, manualClock); // Both independently rearm from false.
        button[0] = true;
        update(bindings, manualClock);
        assertEquals(1, first[0]);
        assertEquals(1, second[0]);
    }

    @Test
    public void clearInvalidatesOldContextsButRootCanBeRebuilt() {
        Bindings bindings = new Bindings();
        Bindings.ControlContext oldContext = bindings.contextWhen(
                BooleanSource.constant(true), Bindings.ActivationPolicy.ACCEPT_CURRENT);
        BindingRegistrar registrar = oldContext;
        assertSame(oldContext, registrar);

        bindings.clear();

        try {
            oldContext.onRise(BooleanSource.constant(false), () -> { });
            fail("expected invalidated context declaration to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("Bindings was cleared"));
        }

        int[] calls = {0};
        bindings.whileHigh(BooleanSource.constant(true), () -> calls[0]++);
        Bindings.ControlContext replacement = bindings.contextWhen(
                BooleanSource.constant(true), Bindings.ActivationPolicy.ACCEPT_CURRENT);
        assertFalse(replacement == oldContext);

        ManualLoopClock manualClock = new ManualLoopClock();
        update(bindings, manualClock);
        assertEquals(1, calls[0]);
    }

    private static void assertCounts(int[] rises,
                                     int[] falls,
                                     int[] highs,
                                     int[] lows,
                                     int expectedRises,
                                     int expectedFalls,
                                     int expectedHighs,
                                     int expectedLows) {
        assertEquals(expectedRises, rises[0]);
        assertEquals(expectedFalls, falls[0]);
        assertEquals(expectedHighs, highs[0]);
        assertEquals(expectedLows, lows[0]);
    }

    private static void update(Bindings bindings, ManualLoopClock manualClock) {
        manualClock.nextCycle(0.02);
        bindings.update(manualClock.clock());
    }
}
