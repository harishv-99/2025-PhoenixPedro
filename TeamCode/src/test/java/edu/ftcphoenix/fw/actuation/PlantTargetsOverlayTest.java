package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies priority, laziness, and entry semantics for Plant target overlays. */
public final class PlantTargetsOverlayTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void samplesEveryGateOnceBeforeResolvingTheSelectedTarget() {
        List<String> events = new ArrayList<>();
        ProbeTargetSource base = target("base", 0.0, events);
        ProbeBooleanSource lowerGate = gate("lowerGate", true, events);
        ProbeTargetSource lower = target("lowerTarget", 1.0, events);
        ProbeBooleanSource higherGate = gate("higherGate", true, events);
        ProbeTargetSource higher = target("higherTarget", 2.0, events);
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("lower", lowerGate, lower)
                .add("higher", higherGate, higher)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetPlan first = overlay.resolve(context(0.0), time.clock());

        assertEquals(2.0, first.target(), EPSILON);
        assertEquals(Arrays.asList("lowerGate", "higherGate", "higherTarget"), events);
        assertEquals(1, lowerGate.samples);
        assertEquals(1, higherGate.samples);
        assertEquals(0, base.resolutions);
        assertEquals(0, lower.resolutions);
        assertEquals(1, higher.resolutions);

        overlay.resolve(context(5.0), time.clock());

        assertEquals("memoized gates must not resample in one cycle", 1, lowerGate.samples);
        assertEquals("memoized gates must not resample in one cycle", 1, higherGate.samples);
    }

    @Test
    public void higherValidLayerWinsWithoutResolvingLowerRequiredUnavailableLayer() {
        ProbeTargetSource base = target(0.0);
        ProbeTargetSource lowerUnavailable = unavailable("lower missing");
        ProbeTargetSource higher = target(2.0);
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("lowerRequired", gate(true), lowerUnavailable)
                .add("higher", gate(true), higher)
                .build();

        PlantTargetPlan plan = overlay.resolve(context(0.0), new ManualLoopClock().clock());

        assertTrue(plan.hasTarget());
        assertEquals(2.0, plan.target(), EPSILON);
        assertEquals(0, base.resolutions);
        assertEquals(0, lowerUnavailable.resolutions);
        assertEquals(1, higher.resolutions);
    }

    @Test
    public void selectedRequiredUnavailableLayerBlocksLowerLayerAndBase() {
        ProbeTargetSource base = target(0.0);
        ProbeTargetSource lower = target(1.0);
        ProbeTargetSource higherUnavailable = unavailable("camera has no target");
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("lower", gate(true), lower)
                .add("visionRequired", gate(true), higherUnavailable)
                .build();

        PlantTargetPlan plan = overlay.resolve(context(0.0), new ManualLoopClock().clock());

        assertFalse(plan.hasTarget());
        assertTrue(plan.reason().contains("visionRequired"));
        assertTrue(plan.reason().contains("camera has no target"));
        assertEquals(0, base.resolutions);
        assertEquals(0, lower.resolutions);
        assertEquals(1, higherUnavailable.resolutions);
    }

    @Test
    public void addIfAvailableFallsThroughUntilAValidLowerLayerWins() {
        ProbeTargetSource base = target(0.0);
        ProbeTargetSource lower = target(1.0);
        ProbeTargetSource middleUnavailable = unavailable("middle missing");
        ProbeTargetSource higherUnavailable = unavailable("higher missing");
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("lower", gate(true), lower)
                .addIfAvailable("middleOptional", gate(true), middleUnavailable)
                .addIfAvailable("higherOptional", gate(true), higherUnavailable)
                .build();

        PlantTargetPlan plan = overlay.resolve(context(0.0), new ManualLoopClock().clock());

        assertTrue(plan.hasTarget());
        assertEquals(1.0, plan.target(), EPSILON);
        assertEquals(0, base.resolutions);
        assertEquals(1, lower.resolutions);
        assertEquals(1, middleUnavailable.resolutions);
        assertEquals(1, higherUnavailable.resolutions);
    }

    @Test
    public void availableOptionalLayerWinsAfterHigherOptionalFallsThrough() {
        List<String> events = new ArrayList<>();
        ProbeTargetSource base = target("baseTarget", 0.0, events);
        ProbeTargetSource lower = target("lowerTarget", 1.0, events);
        ProbeTargetSource middle = target("middleTarget", 2.0, events);
        ProbeTargetSource higherUnavailable = new ProbeTargetSource("higherTarget",
                PlantTargetPlan.unavailable("higher missing"), events);
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("lower", gate("lowerGate", true, events), lower)
                .addIfAvailable("middleOptional", gate("middleGate", true, events), middle)
                .addIfAvailable("higherOptional", gate("higherGate", true, events), higherUnavailable)
                .build();

        PlantTargetPlan plan = overlay.resolve(context(0.0), new ManualLoopClock().clock());

        assertEquals(2.0, plan.target(), EPSILON);
        assertEquals(Arrays.asList("lowerGate", "middleGate", "higherGate",
                "higherTarget", "middleTarget"), events);
        assertEquals(0, base.resolutions);
        assertEquals(0, lower.resolutions);
        assertEquals(1, middle.resolutions);
        assertEquals(1, higherUnavailable.resolutions);
    }

    @Test
    public void resolvesFallThroughTargetsHighToLowBeforeBase() {
        List<String> events = new ArrayList<>();
        ProbeTargetSource base = target("baseTarget", 0.5, events);
        ProbeTargetSource lowerUnavailable = new ProbeTargetSource("lowerTarget",
                PlantTargetPlan.unavailable("lower missing"), events);
        ProbeTargetSource higherUnavailable = new ProbeTargetSource("higherTarget",
                PlantTargetPlan.unavailable("higher missing"), events);
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .addIfAvailable("lowerOptional", gate("lowerGate", true, events), lowerUnavailable)
                .addIfAvailable("higherOptional", gate("higherGate", true, events), higherUnavailable)
                .build();

        PlantTargetPlan plan = overlay.resolve(context(0.0), new ManualLoopClock().clock());

        assertEquals(0.5, plan.target(), EPSILON);
        assertEquals(Arrays.asList("lowerGate", "higherGate", "higherTarget",
                "lowerTarget", "baseTarget"), events);
    }

    @Test
    public void resolvesBaseOnlyAfterEveryEnabledOptionalLayerFallsThrough() {
        ProbeTargetSource base = target(0.5);
        ProbeTargetSource disabled = target(1.0);
        ProbeTargetSource optionalUnavailable = unavailable("optional missing");
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("disabled", gate(false), disabled)
                .addIfAvailable("optional", gate(true), optionalUnavailable)
                .build();

        PlantTargetPlan plan = overlay.resolve(context(0.0), new ManualLoopClock().clock());

        assertTrue(plan.hasTarget());
        assertEquals(0.5, plan.target(), EPSILON);
        assertEquals(1, base.resolutions);
        assertEquals(0, disabled.resolutions);
        assertEquals(1, optionalUnavailable.resolutions);
    }

    @Test
    public void returnsUnavailableBaseWhenNoLayerWinsOrBlocks() {
        ProbeTargetSource baseUnavailable = unavailable("base missing");
        ProbeTargetSource disabled = target(1.0);
        PlantTargetSource overlay = PlantTargets.overlay(baseUnavailable)
                .add("disabled", gate(false), disabled)
                .build();

        PlantTargetPlan plan = overlay.resolve(context(0.0), new ManualLoopClock().clock());

        assertFalse(plan.hasTarget());
        assertTrue(plan.reason().contains("base missing"));
        assertEquals(1, baseUnavailable.resolutions);
        assertEquals(0, disabled.resolutions);
    }

    @Test
    public void selectedTargetFailurePropagatesWithoutFallingThrough() {
        RuntimeException failure = new RuntimeException("selected target failed");
        ProbeTargetSource base = target(0.0);
        ProbeTargetSource lower = target(1.0);
        PlantTargetSource throwing = (context, clock) -> {
            throw failure;
        };
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("lower", gate(true), lower)
                .addIfAvailable("higherOptional", gate(true), throwing)
                .build();

        try {
            overlay.resolve(context(0.0), new ManualLoopClock().clock());
            fail("Expected the selected target failure");
        } catch (RuntimeException actual) {
            assertSame(failure, actual);
        }

        assertEquals(0, base.resolutions);
        assertEquals(0, lower.resolutions);

        CapturingDebugSink debug = new CapturingDebugSink();
        overlay.debugDump(debug, "overlay");
        assertEquals("higherOptional target failed", debug.data.get("overlay.winner"));
        assertTrue((Boolean) debug.data.get("overlay.layer1.targetSampled"));
        assertEquals("TARGET_FAILED", String.valueOf(debug.data.get("overlay.layer1.resolutionState")));
    }

    @Test
    public void activationGateFailurePropagatesBeforeResolvingAnyTarget() {
        RuntimeException failure = new RuntimeException("activation failed");
        ProbeTargetSource base = target(0.0);
        ProbeTargetSource target = target(1.0);
        BooleanSource throwingGate = clock -> {
            throw failure;
        };
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("throwing", throwingGate, target)
                .build();

        try {
            overlay.resolve(context(0.0), new ManualLoopClock().clock());
            fail("Expected the activation gate failure");
        } catch (RuntimeException actual) {
            assertSame(failure, actual);
        }

        assertEquals(0, base.resolutions);
        assertEquals(0, target.resolutions);

        CapturingDebugSink debug = new CapturingDebugSink();
        overlay.debugDump(debug, "overlay");
        assertEquals("throwing activation failed", debug.data.get("overlay.winner"));
        assertFalse((Boolean) debug.data.get("overlay.layer0.targetSampled"));
        assertEquals("GATE_FAILED", String.valueOf(debug.data.get("overlay.layer0.resolutionState")));
    }

    @Test
    public void freezesAllGateValuesBeforeTargetResolutionSideEffects() {
        ProbeBooleanSource lowerGate = gate(true);
        ProbeBooleanSource higherGate = gate(false);
        ProbeTargetSource lower = target(1.0);
        lower.onResolve = () -> higherGate.value = true;
        ProbeTargetSource higher = target(2.0);
        PlantTargetSource overlay = PlantTargets.overlay(0.0)
                .add("lower", lowerGate, lower)
                .add("higher", higherGate, higher)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetPlan first = overlay.resolve(context(0.0), time.clock());

        assertEquals(1.0, first.target(), EPSILON);
        assertEquals(1, lower.resolutions);
        assertEquals(0, higher.resolutions);

        PlantTargetPlan next = overlay.resolve(context(0.0), time.nextCycle(0.02));

        assertEquals(2.0, next.target(), EPSILON);
        assertEquals(1, lower.resolutions);
        assertEquals(1, higher.resolutions);
    }

    @Test
    public void resetPropagatesToBaseGatesAndTargets() {
        ProbeTargetSource base = target(0.0);
        ProbeBooleanSource firstGate = gate(true);
        ProbeTargetSource firstTarget = target(1.0);
        ProbeBooleanSource secondGate = gate(false);
        ProbeTargetSource secondTarget = target(2.0);
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("first", firstGate, firstTarget)
                .addIfAvailable("second", secondGate, secondTarget)
                .build();

        overlay.reset();

        assertEquals(1, base.resets);
        assertEquals(1, firstGate.resets);
        assertEquals(1, firstTarget.resets);
        assertEquals(1, secondGate.resets);
        assertEquals(1, secondTarget.resets);
    }

    @Test
    public void resetClearsGateMemoizationAndOverlayDiagnostics() {
        ProbeTargetSource base = target(0.0);
        ProbeBooleanSource gate = gate(true);
        ProbeTargetSource selected = target(1.0);
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("selected", gate, selected)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(1.0, resolveTarget(overlay, context(0.0), time.clock()), EPSILON);
        gate.value = false;
        assertEquals(1.0, resolveTarget(overlay, context(0.0), time.clock()), EPSILON);
        assertEquals(1, gate.samples);

        overlay.reset();
        assertEquals(0.0, resolveTarget(overlay, context(0.0), time.clock()), EPSILON);

        assertEquals(2, gate.samples);
        CapturingDebugSink debug = new CapturingDebugSink();
        overlay.debugDump(debug, "overlay");
        assertEquals("base", debug.data.get("overlay.winner"));
        assertTrue((Boolean) debug.data.get("overlay.baseSampled"));
        assertFalse((Boolean) debug.data.get("overlay.layer0.enabled"));
        assertFalse((Boolean) debug.data.get("overlay.layer0.targetSampled"));
        assertFalse((Boolean) debug.data.get("overlay.layer0.fellThrough"));
        assertEquals("DISABLED", String.valueOf(debug.data.get("overlay.layer0.resolutionState")));
    }

    @Test
    public void standaloneMeasuredHoldRetainsContinuousEntryAndRecapturesAfterGapOrReset() {
        PlantTargetSource hold = PlantTargets.holdMeasuredTargetOnEntry(-1.0);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(10.0, resolveTarget(hold, context(10.0), time.clock()), EPSILON);
        assertEquals(10.0, resolveTarget(hold, context(20.0), time.clock()), EPSILON);
        assertEquals(10.0, resolveTarget(hold, context(30.0), time.nextCycle(0.02)), EPSILON);

        time.nextCycle(0.02); // The source is not selected in this cycle.
        assertEquals(40.0, resolveTarget(hold, context(40.0), time.nextCycle(0.02)), EPSILON);

        hold.reset();
        assertEquals(50.0, resolveTarget(hold, context(50.0), time.clock()), EPSILON);
    }

    @Test
    public void standaloneMeasuredHoldUsesFallbackForAnEntryWithoutFeedback() {
        PlantTargetSource hold = PlantTargets.holdMeasuredTargetOnEntry(-5.0);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(-5.0, resolveTarget(hold, contextWithoutFeedback(), time.clock()), EPSILON);
        assertEquals(-5.0, resolveTarget(hold, context(10.0), time.nextCycle(0.02)), EPSILON);

        time.nextCycle(0.02); // A sampling gap ends the fallback entry.
        assertEquals(20.0, resolveTarget(hold, context(20.0), time.nextCycle(0.02)), EPSILON);
    }

    @Test
    public void measuredHoldRequiresLoopClockForEntrySemantics() {
        PlantTargetSource hold = PlantTargets.holdMeasuredTargetOnEntry(-1.0);

        try {
            hold.resolve(context(10.0), null);
            fail("Expected a missing clock failure");
        } catch (NullPointerException expected) {
            assertEquals("clock", expected.getMessage());
        }
    }

    @Test
    public void shadowedMeasuredBaseCapturesOnlyWhenSelectedAndRecapturesOnReentry() {
        PlantTargetSource measuredBase = PlantTargets.holdMeasuredTargetOnEntry(-1.0);
        ProbeBooleanSource overrideEnabled = gate(true);
        PlantTargetSource overlay = PlantTargets.overlay(measuredBase)
                .add("override", overrideEnabled, 0.75)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(0.75, resolveTarget(overlay, context(10.0), time.clock()), EPSILON);

        overrideEnabled.value = false;
        assertEquals(20.0, resolveTarget(overlay, context(20.0), time.nextCycle(0.02)), EPSILON);

        overrideEnabled.value = true;
        assertEquals(0.75, resolveTarget(overlay, context(30.0), time.nextCycle(0.02)), EPSILON);

        overrideEnabled.value = false;
        assertEquals(40.0, resolveTarget(overlay, context(40.0), time.nextCycle(0.02)), EPSILON);
    }

    @Test
    public void plannerMeasuredHoldRecapturesUnavailableMeasurementAfterSamplingGap() {
        PlantTargetSource planner = PlantTargets.plan()
                .request(Source.constant(PlantTargetRequest.none("no request")))
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().holdMeasuredTargetOnEntry(-1.0);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(11.0, resolveTarget(planner, context(11.0), time.clock()), EPSILON);
        assertEquals(11.0, resolveTarget(planner, context(12.0), time.clock()), EPSILON);
        assertEquals(11.0, resolveTarget(planner, context(13.0), time.nextCycle(0.02)), EPSILON);

        time.nextCycle(0.02); // The planner is shadowed for one full cycle.
        assertEquals(14.0, resolveTarget(planner, context(14.0), time.nextCycle(0.02)), EPSILON);
    }

    @Test
    public void plannerMeasuredHoldResetStartsANewEntryWithinTheSameCycle() {
        PlantTargetSource planner = PlantTargets.plan()
                .request(Source.constant(PlantTargetRequest.none("no request")))
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().holdMeasuredTargetOnEntry(-5.0);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(-5.0, resolveTarget(planner, contextWithoutFeedback(), time.clock()), EPSILON);
        assertEquals(-5.0, resolveTarget(planner, context(8.0), time.clock()), EPSILON);

        planner.reset();
        assertEquals(9.0, resolveTarget(planner, context(9.0), time.clock()), EPSILON);
    }

    @Test
    public void plannerHoldLastKeepsItsCandidateAcrossSamplingGap() {
        final PlantTargetRequest[] request = {PlantTargetRequest.exact("first", 7.0)};
        PlantTargetSource planner = PlantTargets.plan()
                .request(clock -> request[0])
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().holdLastTarget(-1.0);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(7.0, resolveTarget(planner, context(0.0), time.clock()), EPSILON);
        time.nextCycle(0.02); // The planner is not sampled in this cycle.
        request[0] = PlantTargetRequest.none("request ended");

        PlantTargetPlan held = planner.resolve(context(0.0), time.nextCycle(0.02));

        assertEquals(7.0, held.target(), EPSILON);
        assertEquals(PlantTargetPlan.Kind.HOLD_LAST_TARGET, held.kind());
    }

    @Test
    public void diagnosticsDistinguishDisabledShadowedSelectedAndUnavailableOutcomes() {
        ProbeBooleanSource lowerGate = gate(false);
        ProbeTargetSource lowerUnavailable = unavailable("lower missing");
        ProbeBooleanSource optionalGate = gate(true);
        ProbeTargetSource optionalUnavailable = unavailable("optional missing");
        ProbeBooleanSource higherGate = gate(true);
        ProbeTargetSource higher = target(3.0);
        PlantTargetSource overlay = PlantTargets.overlay(target(0.0))
                .add("lower", lowerGate, lowerUnavailable)
                .addIfAvailable("optional", optionalGate, optionalUnavailable)
                .add("higher", higherGate, higher)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        overlay.resolve(context(0.0), time.clock());
        CapturingDebugSink selectedDebug = new CapturingDebugSink();
        overlay.debugDump(selectedDebug, "overlay");

        assertEquals("higher", selectedDebug.data.get("overlay.winner"));
        assertFalse((Boolean) selectedDebug.data.get("overlay.baseSampled"));
        assertFalse((Boolean) selectedDebug.data.get("overlay.layer0.enabled"));
        assertFalse((Boolean) selectedDebug.data.get("overlay.layer0.targetSampled"));
        assertEquals("DISABLED", String.valueOf(selectedDebug.data.get("overlay.layer0.resolutionState")));
        assertTrue(plan(selectedDebug, "overlay.layer0.plan").reason().contains("disabled"));
        assertTrue((Boolean) selectedDebug.data.get("overlay.layer1.enabled"));
        assertFalse((Boolean) selectedDebug.data.get("overlay.layer1.targetSampled"));
        assertEquals("SHADOWED", String.valueOf(selectedDebug.data.get("overlay.layer1.resolutionState")));
        assertTrue(plan(selectedDebug, "overlay.layer1.plan").reason().contains("shadowed"));
        assertTrue((Boolean) selectedDebug.data.get("overlay.layer2.targetSampled"));
        assertEquals("SELECTED", String.valueOf(selectedDebug.data.get("overlay.layer2.resolutionState")));
        assertEquals(3.0, plan(selectedDebug, "overlay.layer2.plan").target(), EPSILON);

        lowerGate.value = true;
        higherGate.value = false;
        overlay.resolve(context(0.0), time.nextCycle(0.02));
        CapturingDebugSink unavailableDebug = new CapturingDebugSink();
        overlay.debugDump(unavailableDebug, "overlay");

        assertEquals("lower unavailable", unavailableDebug.data.get("overlay.winner"));
        assertFalse((Boolean) unavailableDebug.data.get("overlay.baseSampled"));
        assertTrue((Boolean) unavailableDebug.data.get("overlay.layer1.fellThrough"));
        assertTrue((Boolean) unavailableDebug.data.get("overlay.layer1.targetSampled"));
        assertEquals("FELL_THROUGH", String.valueOf(unavailableDebug.data.get("overlay.layer1.resolutionState")));
        assertTrue(plan(unavailableDebug, "overlay.layer1.plan").reason().contains("optional missing"));
        assertTrue((Boolean) unavailableDebug.data.get("overlay.layer0.targetSampled"));
        assertEquals("REQUIRED_UNAVAILABLE", String.valueOf(unavailableDebug.data.get("overlay.layer0.resolutionState")));
        assertTrue(plan(unavailableDebug, "overlay.layer0.plan").reason().contains("lower missing"));
        assertFalse((Boolean) unavailableDebug.data.get("overlay.layer2.targetSampled"));
        assertEquals("DISABLED", String.valueOf(unavailableDebug.data.get("overlay.layer2.resolutionState")));
        assertTrue(plan(unavailableDebug, "overlay.layer2.plan").reason().contains("disabled"));

        lowerGate.value = false;
        overlay.resolve(context(0.0), time.nextCycle(0.02));
        CapturingDebugSink baseDebug = new CapturingDebugSink();
        overlay.debugDump(baseDebug, "overlay");

        assertEquals("base", baseDebug.data.get("overlay.winner"));
        assertTrue((Boolean) baseDebug.data.get("overlay.baseSampled"));
        assertEquals("FELL_THROUGH", String.valueOf(baseDebug.data.get("overlay.layer1.resolutionState")));
    }

    @Test
    public void powerPlantAppliesOnlyTheSelectedOverlayTarget() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        ProbeTargetSource base = target(0.1);
        ProbeTargetSource lower = target(0.4);
        ProbeTargetSource higher = target(0.8);
        PlantTargetSource overlay = PlantTargets.overlay(base)
                .add("lower", gate(true), lower)
                .add("higher", gate(true), higher)
                .build();
        Plant plant = Plants.power(output, overlay);

        plant.update(new ManualLoopClock().clock());

        assertEquals(0.8, plant.getRequestedTarget(), EPSILON);
        assertEquals(0.8, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.8, output.commanded, EPSILON);
        assertEquals(0, base.resolutions);
        assertEquals(0, lower.resolutions);
        assertEquals(1, higher.resolutions);
    }

    private static PlantTargetContext context(double measurement) {
        return PlantTargetContext.simple(true, measurement, ScalarRange.unbounded(),
                Double.NaN, Double.NaN);
    }

    private static PlantTargetContext contextWithoutFeedback() {
        return PlantTargetContext.simple(false, Double.NaN, ScalarRange.unbounded(),
                Double.NaN, Double.NaN);
    }

    private static double resolveTarget(PlantTargetSource source,
                                        PlantTargetContext context,
                                        LoopClock clock) {
        PlantTargetPlan plan = source.resolve(context, clock);
        assertTrue(plan.hasTarget());
        return plan.target();
    }

    private static PlantTargetPlan plan(CapturingDebugSink debug, String key) {
        Object value = debug.data.get(key);
        assertTrue(key + " must contain a PlantTargetPlan", value instanceof PlantTargetPlan);
        return (PlantTargetPlan) value;
    }

    private static ProbeBooleanSource gate(boolean value) {
        return new ProbeBooleanSource("", value, null);
    }

    private static ProbeBooleanSource gate(String name, boolean value, List<String> events) {
        return new ProbeBooleanSource(name, value, events);
    }

    private static ProbeTargetSource target(double value) {
        return new ProbeTargetSource("", PlantTargetPlan.exact(value, "test target"), null);
    }

    private static ProbeTargetSource target(String name, double value, List<String> events) {
        return new ProbeTargetSource(name, PlantTargetPlan.exact(value, "test target"), events);
    }

    private static ProbeTargetSource unavailable(String reason) {
        return new ProbeTargetSource("", PlantTargetPlan.unavailable(reason), null);
    }

    private static final class ProbeBooleanSource implements BooleanSource {
        private final String name;
        private final List<String> events;
        private boolean value;
        private int samples;
        private int resets;

        private ProbeBooleanSource(String name, boolean value, List<String> events) {
            this.name = name;
            this.value = value;
            this.events = events;
        }

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            samples++;
            if (events != null) events.add(name);
            return value;
        }

        @Override
        public void reset() {
            resets++;
        }
    }

    private static final class ProbeTargetSource implements PlantTargetSource {
        private final String name;
        private final List<String> events;
        private PlantTargetPlan plan;
        private Runnable onResolve;
        private int resolutions;
        private int resets;

        private ProbeTargetSource(String name, PlantTargetPlan plan, List<String> events) {
            this.name = name;
            this.plan = plan;
            this.events = events;
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            resolutions++;
            if (events != null) events.add(name);
            if (onResolve != null) onResolve.run();
            return plan;
        }

        @Override
        public void reset() {
            resets++;
        }
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

    private static final class RecordingPowerOutput implements PowerOutput {
        private double commanded = Double.NaN;

        @Override
        public void setPower(double power) {
            commanded = power;
        }

        @Override
        public double getCommandedPower() {
            return commanded;
        }
    }
}
