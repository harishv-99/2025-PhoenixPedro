package edu.ftcphoenix.fw.actuation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Factory and builders for Plant-aware target resolvers.
 *
 * <p>{@link PlantTargetResolver} is the final graph invoked by a {@link Plant}. In ordinary FTC
 * robot code, the mechanism/subsystem constructor receives {@code HardwareMap} and a data-only
 * config, snapshots that config, and builds this graph together with its privately owned Plant.
 * The composition root constructs the mechanism rather than prebuilding the Plant. For an ordinary
 * writable command with no independent owner, Plant construction uses
 * {@code targetFromNewCommand(initialValue)}; the completed Plant exposes that stable request through
 * {@link Plant#commandTarget()}. Keep a named {@link ScalarTarget} when it is
 * shared, target-only policy owns it, or it usefully identifies the stable base of a composed
 * graph. If an explicitly labeled hardware-neutral test, custom-adapter, portable-host, or
 * advanced-assembly seam injects a completed Plant, it passes the Plant alone instead of both
 * objects as peer dependencies. A read-only/planned realization requires no command. Use
 * {@code PlantTargets} explicitly for read-only values,
 * overlays, periodic equivalence, or advanced planning. Simple values are lifted with
 * {@link #exact(double)} or
 * {@link #exact(ScalarSource)}. Behavior arbitration uses {@link #overlay(PlantTargetResolver)}:
 * every layer's activation gate is sampled once, then target producers are resolved lazily from
 * highest to lowest priority. Layers added with {@code add(...)} must produce a target when enabled,
 * while {@code addIfAvailable(...)} is the explicit opt-in for enabled layers that may fall through.
 * A normal periodic command uses {@link #equivalentPositionsOf(ScalarTarget)}. Advanced
 * multi-alternative planning uses {@link #plan(PlantTargetRequest)} or
 * {@link #plan(Source)}; periodic work is bounded by the explicit request-alternative count rather
 * than the number of equivalent positions in a Plant's range.</p>
 *
 * <h2>Exact target inside a mechanism constructor</h2>
 * <pre>{@code
 * LiftConfig cfg = config.copy();
 * this.lift = FtcActuators.plant(hardwareMap)
 *     .motor(cfg.motorName, cfg.direction)
 *     .position()
 *     .deviceManagedWithDefaults()
 *     .nonPeriodic()
 *     .bounded(0.0, 4200.0)
 *     .nativeUnits()
 *     .alreadyReferenced()
 *     .positionTolerance(20.0)
 *     .targetFromNewCommand(0.0)
 *     .build();
 * // Later, in a semantic mechanism method:
 * lift.commandTarget().set(1200.0);
 * }</pre>
 *
 * <h2>Typical overlay</h2>
 * <pre>{@code
 * PlantTargetResolver feederTarget = PlantTargets.overlay(0.0)
 *     .add("feedPulse", feedPulse.activeSource(), feedPulse)
 *     .add("eject", ejectRequested, -1.0)
 *     .build();
 * }</pre>
 *
 * <h2>Typical periodic command</h2>
 * <pre>{@code
 * ScalarTarget turretCommand = ScalarTarget.create(0.0);
 * PlantTargetResolver turretTarget = PlantTargets.equivalentPositionsOf(turretCommand)
 *     .nearestToMeasurement()
 *     .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
 * }</pre>
 */
public final class PlantTargets {

    private PlantTargets() {
    }

    /**
     * How a planner chooses among multiple reachable request alternatives.
     */
    private enum CandidatePreference {
        /**
         * Choose the reachable alternative closest to the current measurement.
         * Within one periodic family, an exact midpoint tie chooses the lower target.
         */
        NEAREST_TO_MEASUREMENT,
        /**
         * Choose the closest alternative at or above the current measurement, falling back to the
         * closest alternative below only when no increasing alternative is reachable.
         */
        PREFER_INCREASING,
        /**
         * Choose the closest alternative at or below the current measurement, falling back to the
         * closest alternative above only when no decreasing alternative is reachable.
         */
        PREFER_DECREASING,
        /**
         * Choose alternatives closest to the center of a finite legal range, with lower periodic
         * targets winning exact midpoint ties. When the range has no finite center, fall back to
         * nearest-to-measurement selection.
         */
        PREFER_RANGE_CENTER
    }

    /**
     * How a planner handles a finite but unreachable request alternative.
     */
    private enum UnreachablePolicy {
        /**
         * Reject unreachable alternatives; unavailable policy must then produce the target.
         */
        REJECT,
        /**
         * Clamp unreachable alternatives into the Plant's legal range.
         */
        CLAMP_TO_RANGE
    }

    /**
     * What an overlay should do when an enabled layer cannot produce a target.
     */
    private enum LayerUnavailablePolicy {
        /**
         * Stop resolution and report the enabled layer as unavailable.
         */
        REPORT_UNAVAILABLE,
        /**
         * Continue to lower-priority resolution and record that this layer explicitly fell through.
         */
        FALL_THROUGH
    }

    /**
     * Convert a finite constant into an exact Plant target resolver.
     */
    public static PlantTargetResolver exact(double value) {
        if (!Double.isFinite(value))
            throw new IllegalArgumentException("Plant target value must be finite, got " + value);
        return new ExactPlantTargetResolver(ScalarSource.constant(value), "constant " + value);
    }

    /**
     * Convert a scalar source into an exact Plant target resolver.
     *
     * <p>The scalar is sampled during plant update. If it returns NaN or infinity, the target is
     * reported unavailable so the surrounding overlay or plant telemetry can explain the failure.
     * When the supplied object also implements {@link ScalarTarget}, that same object becomes the
     * graph's stable command target before sampling is memoized. A transformed or otherwise wrapped
     * scalar source is read-only unless that wrapper itself implements {@code ScalarTarget}.</p>
     */
    public static PlantTargetResolver exact(ScalarSource source) {
        ScalarSource actualSource = Objects.requireNonNull(source, "source");
        ScalarTarget commandTarget = actualSource instanceof ScalarTarget
                ? (ScalarTarget) actualSource
                : null;
        return new ExactPlantTargetResolver(actualSource, commandTarget, "exact scalar source");
    }

    /**
     * Return the stable command target carried by a framework-created target graph, if any.
     *
     * <p>This package-private query deliberately exposes no public graph-inspection API. Framework
     * Plants use it once during construction so task command ownership is derived from the final
     * graph rather than supplied as a second, potentially disconnected answer.</p>
     */
    static ScalarTarget commandTargetOf(PlantTargetResolver resolver) {
        return resolver instanceof CommandTargetOwner
                ? ((CommandTargetOwner) resolver).commandTarget()
                : null;
    }

    /**
     * Return the previous requested target when available, otherwise {@code initialTarget}.
     */
    public static PlantTargetResolver holdLastTarget(double initialTarget) {
        return new HoldLastTargetResolver(initialTarget);
    }

    /**
     * Latch the current measurement while this resolver is continuously invoked.
     *
     * <p>An entry is the first resolution after construction/reset or after a loop-cycle sampling
     * gap. Repeated resolution in the same cycle and resolution in consecutive cycles retain the
     * same capture. Resolution after one or more unobserved cycles captures the current measurement
     * again.</p>
     */
    public static PlantTargetResolver holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement) {
        return new HoldMeasuredTargetResolver(fallbackIfNoMeasurement);
    }

    /**
     * Start a plant-target overlay with a constant base target.
     */
    public static OverlayBuilder overlay(double baseTarget) {
        return overlay(exact(baseTarget));
    }

    /**
     * Start a plant-target overlay with a scalar base target.
     */
    public static OverlayBuilder overlay(ScalarSource baseTarget) {
        return overlay(exact(baseTarget));
    }

    /**
     * Start a Plant-target overlay with a total base resolver.
     */
    public static OverlayBuilder overlay(PlantTargetResolver baseTarget) {
        return new OverlayBuilder(Objects.requireNonNull(baseTarget, "baseTarget"));
    }

    /**
     * Interpret one graph-owned command as an equivalent-position family in the consuming Plant's
     * periodic coordinate.
     *
     * <p>This is the normal periodic-mechanism path. Robot code and {@link ScalarTasks} keep writing
     * the same logical command value; this final resolver chooses the legal physical representative
     * each loop. Omitting this transform preserves exact, unwrapped target semantics.</p>
     */
    public static EquivalentPositionPreferenceStage equivalentPositionsOf(
            ScalarTarget commandTarget) {
        return equivalentPositionsOf(exact(Objects.requireNonNull(commandTarget, "commandTarget")));
    }

    /**
     * Interpret the final winner of a logical target graph as an equivalent-position family.
     *
     * <p>Compose exact targets and overlays first, then apply this transform so every winning layer
     * receives the same periodic interpretation. If the graph carries a command target, that
     * command identity remains available to feedback-aware {@link ScalarTasks}.</p>
     */
    public static EquivalentPositionPreferenceStage equivalentPositionsOf(
            PlantTargetResolver finalLogicalTarget) {
        return new EquivalentPositionPreferenceBuilder(
                Objects.requireNonNull(finalLogicalTarget, "finalLogicalTarget"));
    }

    /**
     * Starts an advanced target resolver for one fixed immutable request.
     *
     * <p>The next stage asks how reachable alternatives should be preferred. Use
     * {@link #plan(Source)} when the request changes from cycle to cycle.</p>
     */
    public static PlanPreferenceStage plan(PlantTargetRequest request) {
        return plan(Source.constant(Objects.requireNonNull(request, "request")));
    }

    /**
     * Starts an advanced target resolver for requests supplied each cycle.
     *
     * <p>The staged builder asks required questions in order: alternative preference,
     * unreachable-alternative policy, then unavailable-target policy. Optional observation
     * age/quality tuning is available after the required motion-semantics answers. Each answer
     * returns an immutable snapshot, so retaining and branching from an earlier stage cannot
     * silently overwrite another completed branch.</p>
     */
    public static PlanPreferenceStage plan(Source<PlantTargetRequest> requestSource) {
        return new PlannerPreferenceAnswer(Objects.requireNonNull(requestSource, "requestSource"));
    }

    /** Private metadata seam implemented only by framework graphs with a stable command base. */
    private interface CommandTargetOwner {
        ScalarTarget commandTarget();
    }

    private static final class ExactPlantTargetResolver
            implements PlantTargetResolver, CommandTargetOwner {
        private final ScalarSource source;
        private final ScalarTarget commandTarget;
        private final String reason;

        ExactPlantTargetResolver(ScalarSource source, String reason) {
            this(source, null, reason);
        }

        ExactPlantTargetResolver(ScalarSource source,
                                 ScalarTarget commandTarget,
                                 String reason) {
            this.source = Objects.requireNonNull(source, "source").memoized();
            this.commandTarget = commandTarget;
            this.reason = reason;
        }

        @Override
        public ScalarTarget commandTarget() {
            return commandTarget;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            double v = source.getAsDouble(clock);
            PlantTargetResolution resolution = Double.isFinite(v)
                    ? PlantTargetResolution.exact(v, reason)
                    : PlantTargetResolution.unavailable(
                            "exact scalar source returned non-finite target: " + v);
            if (commandTarget == null) return resolution;
            return Double.isFinite(v)
                    ? resolution.withSelectedCommand(commandTarget, v)
                    : resolution.withoutSelectedCommand(commandTarget);
        }

        @Override
        public void reset() {
            source.reset();
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "exactPlantTarget" : prefix;
            dbg.addData(p + ".class", "ExactPlantTargetResolver")
                    .addData(p + ".reason", reason);
            source.debugDump(dbg, p + ".scalar");
        }
    }

    private static final class HoldLastTargetResolver implements PlantTargetResolver {
        private final double initialTarget;

        HoldLastTargetResolver(double initialTarget) {
            if (!Double.isFinite(initialTarget))
                throw new IllegalArgumentException("initialTarget must be finite");
            this.initialTarget = initialTarget;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            double target = Double.isFinite(context.previousRequestedTarget()) ? context.previousRequestedTarget() : initialTarget;
            return PlantTargetResolution.holdLast(target, "holding previous requested target");
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "holdLastPlantTarget" : prefix;
            dbg.addData(p + ".class", "HoldLastTargetResolver")
                    .addData(p + ".initialTarget", initialTarget);
        }
    }

    private static final class HoldMeasuredTargetResolver implements PlantTargetResolver {
        private final double fallback;
        private boolean latched;
        private double latchedTarget;
        private long lastResolvedCycle = Long.MIN_VALUE;

        HoldMeasuredTargetResolver(double fallback) {
            if (!Double.isFinite(fallback))
                throw new IllegalArgumentException("fallbackIfNoMeasurement must be finite");
            this.fallback = fallback;
            this.latchedTarget = fallback;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            long cycle = Objects.requireNonNull(clock, "clock").cycle();
            if (!latched || hasSamplingGap(lastResolvedCycle, cycle)) {
                latchedTarget = context.feedbackAvailable() ? context.measurement() : fallback;
                latched = true;
            }
            lastResolvedCycle = cycle;
            return PlantTargetResolution.holdMeasured(
                    latchedTarget, "holding measured target captured on entry");
        }

        @Override
        public void reset() {
            latched = false;
            latchedTarget = fallback;
            lastResolvedCycle = Long.MIN_VALUE;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "holdMeasuredPlantTarget" : prefix;
            dbg.addData(p + ".class", "HoldMeasuredTargetResolver")
                    .addData(p + ".fallback", fallback)
                    .addData(p + ".latched", latched)
                    .addData(p + ".latchedTarget", latchedTarget)
                    .addData(p + ".lastResolvedCycle", lastResolvedCycle);
        }
    }

    /**
     * Builder for base-plus-priority plant target overlays.
     *
     * <p>The base target should be total. Every activation gate is sampled once in insertion order,
     * then enabled target producers are resolved lazily in reverse insertion order, so later layers
     * have higher priority. Layers added with
     * {@link #add(String, BooleanSource, PlantTargetResolver)} must produce a target when enabled;
     * unavailable active layers report an unavailable resolution instead of silently falling
     * through. Use {@link #addIfAvailable(String, BooleanSource, PlantTargetResolver)} only when an
     * enabled-but-unavailable layer should explicitly continue to the next lower priority.</p>
     *
     * <p>If the base graph carries a command target, the completed overlay carries that same
     * command identity. Conditional layer targets never become or redirect the command target,
     * even when a layer itself is a {@link ScalarTarget}.</p>
     */
    public static final class OverlayBuilder {
        private final PlantTargetResolver base;
        private final List<Layer> layers = new ArrayList<Layer>();

        private OverlayBuilder(PlantTargetResolver base) {
            this.base = base;
        }

        /**
         * Add an enabled layer with a constant target.
         */
        public OverlayBuilder add(String name, BooleanSource enabled, double target) {
            return add(name, enabled, exact(target));
        }

        /**
         * Add an enabled layer with an exact scalar target.
         */
        public OverlayBuilder add(String name, BooleanSource enabled, ScalarSource target) {
            return add(name, enabled, exact(target));
        }

        /**
         * Add an enabled layer that must produce a target when its Boolean is high.
         *
         * <p>This is the normal overlay behavior. The Boolean means “this behavior is requested.”
         * If the target resolver cannot produce a target, the overlay reports that failure instead of
         * silently falling through to a lower-priority behavior.</p>
         */
        public OverlayBuilder add(String name, BooleanSource enabled, PlantTargetResolver target) {
            layers.add(new Layer(cleanName(name), Objects.requireNonNull(enabled, "enabled"),
                    Objects.requireNonNull(target, "target"), LayerUnavailablePolicy.REPORT_UNAVAILABLE));
            return this;
        }

        /**
         * Add a fall-through layer with a constant target.
         */
        public OverlayBuilder addIfAvailable(String name, BooleanSource enabled, double target) {
            return addIfAvailable(name, enabled, exact(target));
        }

        /**
         * Add a fall-through layer with an exact scalar target.
         */
        public OverlayBuilder addIfAvailable(String name, BooleanSource enabled, ScalarSource target) {
            return addIfAvailable(name, enabled, exact(target));
        }

        /**
         * Add an enabled layer that may explicitly fall through when unavailable.
         *
         * <p>Use this only when “requested but no valid target” should let lower-priority behavior
         * continue. Debug output still records that the layer was enabled, unavailable, and fell
         * through.
         * For most behavior layers, prefer
         * {@link #add(String, BooleanSource, PlantTargetResolver)} so
         * missing targets are visible as failures.</p>
         */
        public OverlayBuilder addIfAvailable(
                String name, BooleanSource enabled, PlantTargetResolver target) {
            layers.add(new Layer(cleanName(name), Objects.requireNonNull(enabled, "enabled"),
                    Objects.requireNonNull(target, "target"), LayerUnavailablePolicy.FALL_THROUGH));
            return this;
        }

        /**
         * Build the overlay resolver.
         */
        public PlantTargetResolver build() {
            return new OverlayTargetResolver(base, layers.toArray(new Layer[0]));
        }
    }

    private static final class Layer {
        final String name;
        final BooleanSource enabled;
        final PlantTargetResolver target;
        final LayerUnavailablePolicy unavailablePolicy;

        Layer(String name,
              BooleanSource enabled,
              PlantTargetResolver target,
              LayerUnavailablePolicy unavailablePolicy) {
            this.name = name;
            this.enabled = enabled.memoized();
            this.target = target;
            this.unavailablePolicy = unavailablePolicy;
        }
    }

    private enum LayerResolutionState {
        NOT_SAMPLED,
        GATE_FAILED,
        DISABLED,
        ENABLED_NOT_REACHED,
        SHADOWED,
        SELECTED,
        FELL_THROUGH,
        TARGET_FAILED,
        REQUIRED_UNAVAILABLE
    }

    private static final class LayerRuntimeState {
        private static final PlantTargetResolution NOT_SAMPLED_RESOLUTION =
                PlantTargetResolution.unavailable("not sampled");
        private static final PlantTargetResolution GATE_NOT_SAMPLED_RESOLUTION =
                PlantTargetResolution.unavailable("activation gate not sampled");
        private static final PlantTargetResolution DISABLED_RESOLUTION =
                PlantTargetResolution.unavailable("layer disabled");
        private static final PlantTargetResolution ENABLED_NOT_REACHED_RESOLUTION =
                PlantTargetResolution.unavailable("enabled layer target not reached");
        private static final PlantTargetResolution SHADOWED_RESOLUTION =
                PlantTargetResolution.unavailable(
                        "enabled layer target shadowed by higher-priority layer");
        private static final PlantTargetResolution GATE_FAILED_RESOLUTION =
                PlantTargetResolution.unavailable("activation gate sampling failed");
        private static final PlantTargetResolution TARGET_FAILED_RESOLUTION =
                PlantTargetResolution.unavailable("target resolution failed");

        boolean enabled;
        boolean targetSampled;
        boolean fellThrough;
        LayerResolutionState resolutionState = LayerResolutionState.NOT_SAMPLED;
        PlantTargetResolution resolution = NOT_SAMPLED_RESOLUTION;

        void clearForGateSampling() {
            enabled = false;
            targetSampled = false;
            fellThrough = false;
            resolutionState = LayerResolutionState.NOT_SAMPLED;
            resolution = GATE_NOT_SAMPLED_RESOLUTION;
        }

        void reset() {
            enabled = false;
            targetSampled = false;
            fellThrough = false;
            resolutionState = LayerResolutionState.NOT_SAMPLED;
            resolution = NOT_SAMPLED_RESOLUTION;
        }
    }

    private static final class OverlayTargetResolver
            implements PlantTargetResolver, CommandTargetOwner {
        private static final PlantTargetResolution INCOMPLETE_RESOLUTION =
                PlantTargetResolution.unavailable("overlay resolution did not complete");

        private final PlantTargetResolver base;
        private final ScalarTarget commandTarget;
        private final Layer[] layers;
        private final LayerRuntimeState[] layerStates;
        private PlantTargetResolution lastResolution =
                PlantTargetResolution.unavailable("not sampled");
        private String lastWinner = "not sampled";
        private boolean lastBaseSampled;

        OverlayTargetResolver(PlantTargetResolver base, Layer[] layers) {
            this.base = base;
            this.commandTarget = commandTargetOf(base);
            this.layers = layers;
            this.layerStates = new LayerRuntimeState[layers.length];
            for (int i = 0; i < layerStates.length; i++) {
                layerStates[i] = new LayerRuntimeState();
            }
        }

        @Override
        public ScalarTarget commandTarget() {
            return commandTarget;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            lastResolution = INCOMPLETE_RESOLUTION;
            lastWinner = "not resolved";
            lastBaseSampled = false;

            for (LayerRuntimeState state : layerStates) {
                state.clearForGateSampling();
            }

            // Activation sources are arbitration state, not target-producing branches. Sample every
            // gate once before resolving a target so queues, edges, and debounce state stay current,
            // and so target-side effects cannot change this loop's priority snapshot.
            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];
                LayerRuntimeState state = layerStates[i];
                try {
                    state.enabled = layer.enabled.getAsBoolean(clock);
                } catch (RuntimeException failure) {
                    state.resolutionState = LayerResolutionState.GATE_FAILED;
                    state.resolution = LayerRuntimeState.GATE_FAILED_RESOLUTION;
                    lastWinner = layer.name + " activation failed";
                    lastResolution = state.resolution;
                    throw failure;
                }
                state.resolutionState = state.enabled
                        ? LayerResolutionState.ENABLED_NOT_REACHED
                        : LayerResolutionState.DISABLED;
                state.resolution = state.enabled
                        ? LayerRuntimeState.ENABLED_NOT_REACHED_RESOLUTION
                        : LayerRuntimeState.DISABLED_RESOLUTION;
            }

            PlantTargetResolution winner = null;
            int decisiveLayerIndex = -1;
            for (int i = layers.length - 1; i >= 0; i--) {
                Layer layer = layers[i];
                LayerRuntimeState state = layerStates[i];
                if (!state.enabled) continue;

                state.targetSampled = true;
                PlantTargetResolution resolution;
                try {
                    resolution = layer.target.resolve(context, clock);
                    if (resolution == null) {
                        throw new NullPointerException(
                                "Plant target layer '" + layer.name
                                        + "' returned null resolution");
                    }
                } catch (RuntimeException failure) {
                    state.resolutionState = LayerResolutionState.TARGET_FAILED;
                    state.resolution = LayerRuntimeState.TARGET_FAILED_RESOLUTION;
                    lastWinner = layer.name + " target failed";
                    lastResolution = state.resolution;
                    throw failure;
                }
                state.resolution = resolution;
                if (!resolution.hasTarget()) {
                    if (layer.unavailablePolicy == LayerUnavailablePolicy.FALL_THROUGH) {
                        state.fellThrough = true;
                        state.resolutionState = LayerResolutionState.FELL_THROUGH;
                        continue;
                    }
                    state.resolutionState = LayerResolutionState.REQUIRED_UNAVAILABLE;
                    lastWinner = layer.name + " unavailable";
                    winner = PlantTargetResolution.unavailable(
                            "enabled plant target layer '" + layer.name
                                    + "' produced no target: " + resolution.reason());
                    decisiveLayerIndex = i;
                    break;
                }
                state.resolutionState = LayerResolutionState.SELECTED;
                lastWinner = layer.name;
                winner = resolution;
                decisiveLayerIndex = i;
                break;
            }

            if (decisiveLayerIndex >= 0) {
                for (int i = decisiveLayerIndex - 1; i >= 0; i--) {
                    LayerRuntimeState state = layerStates[i];
                    if (state.enabled
                            && state.resolutionState == LayerResolutionState.ENABLED_NOT_REACHED) {
                        state.resolutionState = LayerResolutionState.SHADOWED;
                        state.resolution = LayerRuntimeState.SHADOWED_RESOLUTION;
                    }
                }
            } else {
                lastBaseSampled = true;
                lastWinner = "base";
                winner = base.resolve(context, clock);
            }

            if (decisiveLayerIndex >= 0 && commandTarget != null) {
                winner = winner.withoutSelectedCommand(commandTarget);
            }
            lastResolution = winner;
            return winner;
        }

        @Override
        public void reset() {
            base.reset();
            for (Layer layer : layers) {
                layer.enabled.reset();
                layer.target.reset();
            }
            for (LayerRuntimeState state : layerStates) {
                state.reset();
            }
            lastResolution = PlantTargetResolution.unavailable("not sampled");
            lastWinner = "not sampled";
            lastBaseSampled = false;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plantTargetOverlay" : prefix;
            dbg.addData(p + ".class", "PlantTargetOverlay")
                    .addData(p + ".winner", lastWinner)
                    .addData(p + ".resolution", lastResolution)
                    .addData(p + ".baseSampled", lastBaseSampled)
                    .addData(p + ".layers", layers.length);
            base.debugDump(dbg, p + ".base");
            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];
                LayerRuntimeState state = layerStates[i];
                String lp = p + ".layer" + i;
                dbg.addData(lp + ".name", layer.name)
                        .addData(lp + ".enabled", state.enabled)
                        .addData(lp + ".targetSampled", state.targetSampled)
                        .addData(lp + ".resolutionState", state.resolutionState)
                        .addData(lp + ".unavailablePolicy", layer.unavailablePolicy)
                        .addData(lp + ".fellThrough", state.fellThrough)
                        .addData(lp + ".resolution", state.resolution);
                layer.target.debugDump(dbg, lp + ".target");
            }
        }
    }

    /** Required choice for selecting one legal equivalent physical position. */
    public interface EquivalentPositionPreferenceStage {
        /** Choose the legal equivalent nearest to the current measurement. */
        EquivalentPositionReadyStage nearestToMeasurement();

        /** Prefer the closest legal equivalent at or above the current measurement. */
        EquivalentPositionReadyStage preferIncreasing();

        /** Prefer the closest legal equivalent at or below the current measurement. */
        EquivalentPositionReadyStage preferDecreasing();

        /** Prefer the legal equivalent nearest the center of a finite target range. */
        EquivalentPositionReadyStage preferRangeCenter();
    }

    /** Equivalent-position stage after the required selection preference. */
    public interface EquivalentPositionReadyStage {
        /** Choose what the final resolver should report when no legal equivalent is available. */
        UnavailableTargetBranch whenUnavailable();
    }

    /**
     * Required planner question: how should reachable request alternatives be preferred?
     *
     * <p>Each method answers the preference question and advances to the next required question. The
     * returned type intentionally hides the other preference methods so a later call cannot silently
     * replace an earlier answer.</p>
     */
    public interface PlanPreferenceStage {
        /**
         * Choose the reachable alternative nearest to the current measurement. Within one periodic
         * family, an exact midpoint tie chooses the lower target. An exact tie between distinct
         * request alternatives retains their declared order.
         */
        PlanUnreachableStage nearestToMeasurement();

        /**
         * Choose the closest reachable alternative at or above the current measurement. Fall back
         * to the closest alternative below only when no increasing alternative is reachable.
         */
        PlanUnreachableStage preferIncreasing();

        /**
         * Choose the closest reachable alternative at or below the current measurement. Fall back
         * to the closest alternative above only when no decreasing alternative is reachable.
         */
        PlanUnreachableStage preferDecreasing();

        /**
         * Choose alternatives closest to a finite legal range center, with lower periodic targets
         * winning exact midpoint ties. Fall back to nearest-to-measurement selection when the range
         * has no finite center.
         */
        PlanUnreachableStage preferRangeCenter();
    }

    /**
     * Required planner question: what should happen to finite alternatives outside the legal range?
     */
    public interface PlanUnreachableStage {
        /**
         * Reject unreachable alternatives and let the unavailable policy produce the target.
         */
        PlanReadyStage rejectUnreachable();

        /**
         * Clamp unreachable alternatives into the legal range before scoring them.
         */
        PlanReadyStage clampUnreachableToRange();
    }

    /**
     * Main planner stage after required request, preference, and unreachable-policy answers.
     */
    public interface PlanReadyStage {
        /**
         * Enter optional request-acceptance tuning.
         */
        PlanAcceptBranch accept();

        /**
         * Enter the required unavailable-target policy branch.
         */
        UnavailableTargetBranch whenUnavailable();
    }

    /**
     * Observed-alternative age/quality acceptance tuning branch.
     */
    public interface PlanAcceptBranch {
        /**
         * Ignore observed alternatives older than {@code ageSec} at planner resolution.
         *
         * <p>Timeless alternatives created with the short factories are not observations and are not
         * affected by this limit.</p>
         *
         * @param ageSec finite maximum observation age in seconds, inclusive
         * @throws IllegalArgumentException if {@code ageSec} is negative or non-finite
         */
        PlanAcceptBranch maxObservationAgeSec(double ageSec);

        /**
         * Ignore alternatives below the supplied inclusive quality threshold.
         *
         * <p>Timeless alternatives have implicit quality {@code 1.0}.</p>
         *
         * @param quality finite minimum quality in {@code [0, 1]}
         * @throws IllegalArgumentException if {@code quality} is outside {@code [0, 1]} or
         *                                  non-finite
         */
        PlanAcceptBranch minQuality(double quality);

        /**
         * Return to the main planner builder after optional acceptance tuning.
         */
        PlanReadyStage doneAccept();
    }

    /** Required branch that makes a context-aware target resolver total. */
    public interface UnavailableTargetBranch {
        /**
         * Produce a fixed physical Plant-unit fallback when no target can be selected.
         */
        PlantTargetResolver fallbackTo(double target);

        /**
         * Hold the last successfully resolved physical target; use {@code initialTarget} first.
         */
        PlantTargetResolver holdLastTarget(double initialTarget);

        /**
         * Latch current measurement on each continuously sampled unavailable entry.
         *
         * <p>A sampling gap ends the current unavailable entry. If this resolver is selected
         * again and remains unavailable, it captures the then-current measurement.</p>
         */
        PlantTargetResolver holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement);

        /**
         * Explicitly report unavailability when no target can be selected.
         */
        PlantTargetResolver reportUnavailable();
    }

    private enum UnavailableKind {FALLBACK, HOLD_LAST, HOLD_MEASURED, REJECT}

    private static final class EquivalentPositionPreferenceBuilder
            implements EquivalentPositionPreferenceStage {
        private final PlantTargetResolver logicalTarget;

        EquivalentPositionPreferenceBuilder(PlantTargetResolver logicalTarget) {
            this.logicalTarget = logicalTarget;
        }

        @Override
        public EquivalentPositionReadyStage nearestToMeasurement() {
            return new EquivalentPositionAnswer(
                    logicalTarget, CandidatePreference.NEAREST_TO_MEASUREMENT);
        }

        @Override
        public EquivalentPositionReadyStage preferIncreasing() {
            return new EquivalentPositionAnswer(
                    logicalTarget, CandidatePreference.PREFER_INCREASING);
        }

        @Override
        public EquivalentPositionReadyStage preferDecreasing() {
            return new EquivalentPositionAnswer(
                    logicalTarget, CandidatePreference.PREFER_DECREASING);
        }

        @Override
        public EquivalentPositionReadyStage preferRangeCenter() {
            return new EquivalentPositionAnswer(
                    logicalTarget, CandidatePreference.PREFER_RANGE_CENTER);
        }
    }

    /** Immutable equivalent-position answer snapshot for one selected preference. */
    private static final class EquivalentPositionAnswer
            implements EquivalentPositionReadyStage, UnavailableTargetBranch {
        private final PlantTargetResolver logicalTarget;
        private final CandidatePreference preference;

        EquivalentPositionAnswer(PlantTargetResolver logicalTarget,
                                 CandidatePreference preference) {
            this.logicalTarget = logicalTarget;
            this.preference = preference;
        }

        @Override
        public UnavailableTargetBranch whenUnavailable() {
            return this;
        }

        @Override
        public PlantTargetResolver fallbackTo(double target) {
            return build(UnavailableKind.FALLBACK, target);
        }

        @Override
        public PlantTargetResolver holdLastTarget(double initialTarget) {
            return build(UnavailableKind.HOLD_LAST, initialTarget);
        }

        @Override
        public PlantTargetResolver holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement) {
            return build(UnavailableKind.HOLD_MEASURED, fallbackIfNoMeasurement);
        }

        @Override
        public PlantTargetResolver reportUnavailable() {
            return build(UnavailableKind.REJECT, Double.NaN);
        }

        private PlantTargetResolver build(UnavailableKind unavailableKind,
                                          double unavailableValue) {
            return new EquivalentPositionsTargetResolver(logicalTarget, preference,
                    unavailableKind, unavailableValue);
        }
    }

    /** Immutable request-source snapshot before the planner preference answer. */
    private static final class PlannerPreferenceAnswer implements PlanPreferenceStage {
        private final Source<PlantTargetRequest> requestSource;

        PlannerPreferenceAnswer(Source<PlantTargetRequest> requestSource) {
            this.requestSource = requestSource;
        }

        @Override
        public PlanUnreachableStage nearestToMeasurement() {
            return next(CandidatePreference.NEAREST_TO_MEASUREMENT);
        }

        @Override
        public PlanUnreachableStage preferIncreasing() {
            return next(CandidatePreference.PREFER_INCREASING);
        }

        @Override
        public PlanUnreachableStage preferDecreasing() {
            return next(CandidatePreference.PREFER_DECREASING);
        }

        @Override
        public PlanUnreachableStage preferRangeCenter() {
            return next(CandidatePreference.PREFER_RANGE_CENTER);
        }

        private PlanUnreachableStage next(CandidatePreference preference) {
            return new PlannerUnreachableAnswer(requestSource, preference);
        }
    }

    /** Immutable planner snapshot before the unreachable-alternative answer. */
    private static final class PlannerUnreachableAnswer implements PlanUnreachableStage {
        private final Source<PlantTargetRequest> requestSource;
        private final CandidatePreference preference;

        PlannerUnreachableAnswer(Source<PlantTargetRequest> requestSource,
                                 CandidatePreference preference) {
            this.requestSource = requestSource;
            this.preference = preference;
        }

        @Override
        public PlanReadyStage rejectUnreachable() {
            return next(UnreachablePolicy.REJECT);
        }

        @Override
        public PlanReadyStage clampUnreachableToRange() {
            return next(UnreachablePolicy.CLAMP_TO_RANGE);
        }

        private PlanReadyStage next(UnreachablePolicy unreachablePolicy) {
            return new PlannerReadyAnswer(requestSource, preference, unreachablePolicy,
                    Double.POSITIVE_INFINITY, 0.0);
        }
    }

    /** Immutable planner snapshot after required motion-policy answers. */
    private static final class PlannerReadyAnswer implements PlanReadyStage {
        private final Source<PlantTargetRequest> requestSource;
        private final CandidatePreference preference;
        private final UnreachablePolicy unreachablePolicy;
        private final double maxObservationAgeSec;
        private final double minQuality;

        PlannerReadyAnswer(Source<PlantTargetRequest> requestSource,
                           CandidatePreference preference,
                           UnreachablePolicy unreachablePolicy,
                           double maxObservationAgeSec,
                           double minQuality) {
            this.requestSource = requestSource;
            this.preference = preference;
            this.unreachablePolicy = unreachablePolicy;
            this.maxObservationAgeSec = maxObservationAgeSec;
            this.minQuality = minQuality;
        }

        @Override
        public PlanAcceptBranch accept() {
            return new PlannerAcceptAnswer(requestSource, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality);
        }

        @Override
        public UnavailableTargetBranch whenUnavailable() {
            return new PlannerUnavailableAnswer(requestSource, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality);
        }
    }

    /** Immutable optional acceptance-tuning branch. */
    private static final class PlannerAcceptAnswer implements PlanAcceptBranch {
        private final Source<PlantTargetRequest> requestSource;
        private final CandidatePreference preference;
        private final UnreachablePolicy unreachablePolicy;
        private final double maxObservationAgeSec;
        private final double minQuality;

        PlannerAcceptAnswer(Source<PlantTargetRequest> requestSource,
                            CandidatePreference preference,
                            UnreachablePolicy unreachablePolicy,
                            double maxObservationAgeSec,
                            double minQuality) {
            this.requestSource = requestSource;
            this.preference = preference;
            this.unreachablePolicy = unreachablePolicy;
            this.maxObservationAgeSec = maxObservationAgeSec;
            this.minQuality = minQuality;
        }

        @Override
        public PlanAcceptBranch maxObservationAgeSec(double ageSec) {
            if (ageSec < 0.0 || !Double.isFinite(ageSec)) {
                throw new IllegalArgumentException(
                        "maxObservationAgeSec must be finite and >= 0");
            }
            return new PlannerAcceptAnswer(requestSource, preference, unreachablePolicy,
                    ageSec, minQuality);
        }

        @Override
        public PlanAcceptBranch minQuality(double quality) {
            if (!Double.isFinite(quality) || quality < 0.0 || quality > 1.0) {
                throw new IllegalArgumentException("minQuality must be finite and in [0, 1]");
            }
            return new PlannerAcceptAnswer(requestSource, preference, unreachablePolicy,
                    maxObservationAgeSec, quality);
        }

        @Override
        public PlanReadyStage doneAccept() {
            return new PlannerReadyAnswer(requestSource, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality);
        }
    }

    /** Immutable final planner snapshot that builds one selected unavailable policy. */
    private static final class PlannerUnavailableAnswer implements UnavailableTargetBranch {
        private final Source<PlantTargetRequest> requestSource;
        private final CandidatePreference preference;
        private final UnreachablePolicy unreachablePolicy;
        private final double maxObservationAgeSec;
        private final double minQuality;

        PlannerUnavailableAnswer(Source<PlantTargetRequest> requestSource,
                                 CandidatePreference preference,
                                 UnreachablePolicy unreachablePolicy,
                                 double maxObservationAgeSec,
                                 double minQuality) {
            this.requestSource = requestSource;
            this.preference = preference;
            this.unreachablePolicy = unreachablePolicy;
            this.maxObservationAgeSec = maxObservationAgeSec;
            this.minQuality = minQuality;
        }

        @Override
        public PlantTargetResolver fallbackTo(double target) {
            return build(UnavailableKind.FALLBACK, target);
        }

        @Override
        public PlantTargetResolver holdLastTarget(double initialTarget) {
            return build(UnavailableKind.HOLD_LAST, initialTarget);
        }

        @Override
        public PlantTargetResolver holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement) {
            return build(UnavailableKind.HOLD_MEASURED, fallbackIfNoMeasurement);
        }

        @Override
        public PlantTargetResolver reportUnavailable() {
            return build(UnavailableKind.REJECT, Double.NaN);
        }

        private PlantTargetResolver build(UnavailableKind unavailableKind,
                                          double unavailableValue) {
            return new PlannerTargetResolver(requestSource, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality, unavailableKind, unavailableValue);
        }
    }

    /** Complete staged publication for one context-aware target resolution. */
    private static final class ResolverCandidate {
        final PlantTargetResolution resolution;
        final double lastTarget;
        final boolean unavailableActive;
        final double heldMeasuredTarget;

        ResolverCandidate(PlantTargetResolution resolution,
                          double lastTarget,
                          boolean unavailableActive,
                          double heldMeasuredTarget) {
            this.resolution = Objects.requireNonNull(resolution, "resolution");
            this.lastTarget = lastTarget;
            this.unavailableActive = unavailableActive;
            this.heldMeasuredTarget = heldMeasuredTarget;
        }
    }

    private static final class EquivalentPositionsTargetResolver
            implements PlantTargetResolver, CommandTargetOwner {
        private final PlantTargetResolver logicalTarget;
        private final ScalarTarget commandTarget;
        private final CandidatePreference preference;
        private final UnavailableKind unavailableKind;
        private final double unavailableValue;

        private PlantTargetResolution lastResolution =
                PlantTargetResolution.unavailable("not sampled");
        private long lastCycle = Long.MIN_VALUE;
        private double lastTarget = Double.NaN;
        private boolean unavailableActive;
        private double heldMeasuredTarget = Double.NaN;
        private boolean resolving;
        private boolean resetting;

        EquivalentPositionsTargetResolver(PlantTargetResolver logicalTarget,
                                          CandidatePreference preference,
                                          UnavailableKind unavailableKind,
                                          double unavailableValue) {
            this.logicalTarget = Objects.requireNonNull(logicalTarget, "logicalTarget");
            this.commandTarget = commandTargetOf(logicalTarget);
            this.preference = Objects.requireNonNull(preference, "preference");
            this.unavailableKind = Objects.requireNonNull(unavailableKind, "unavailableKind");
            this.unavailableValue = unavailableValue;
            if ((unavailableKind == UnavailableKind.FALLBACK
                    || unavailableKind == UnavailableKind.HOLD_LAST
                    || unavailableKind == UnavailableKind.HOLD_MEASURED)
                    && !Double.isFinite(unavailableValue)) {
                throw new IllegalArgumentException(
                        "Unavailable policy fallback/initial value must be finite");
            }
        }

        @Override
        public ScalarTarget commandTarget() {
            return commandTarget;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            Objects.requireNonNull(context, "context");
            long cycle = Objects.requireNonNull(clock, "clock").cycle();
            if (resolving) {
                throw new IllegalStateException(
                        "Equivalent-position target resolution reentered; check the logical "
                                + "target graph for a cycle");
            }
            if (resetting) {
                throw new IllegalStateException(
                        "Equivalent-position target cannot resolve while reset is in progress");
            }
            if (cycle == lastCycle) return lastResolution;

            resolving = true;
            try {
                double candidateLastTarget = lastTarget;
                boolean candidateUnavailableActive = unavailableActive;
                double candidateHeldMeasuredTarget = heldMeasuredTarget;
                if (unavailableKind == UnavailableKind.HOLD_MEASURED
                        && hasSamplingGap(lastCycle, cycle)) {
                    candidateUnavailableActive = false;
                    candidateHeldMeasuredTarget = Double.NaN;
                }

                PlantTargetResolution logicalResolution = logicalTarget.resolve(context, clock);
                if (logicalResolution == null) {
                    throw new NullPointerException(
                            "Equivalent-position logical target returned null resolution");
                }

                ResolverCandidate candidate;
                if (!logicalResolution.hasTarget()) {
                    candidate = unavailableCandidate(context,
                            "logical target unavailable: " + logicalResolution.reason(),
                            candidateLastTarget,
                            candidateUnavailableActive,
                            candidateHeldMeasuredTarget);
                } else if (context.periodicity() != PositionPlant.Periodicity.PERIODIC) {
                    candidate = unavailableCandidate(context,
                            "equivalent positions require a periodic Plant coordinate",
                            candidateLastTarget,
                            candidateUnavailableActive,
                            candidateHeldMeasuredTarget);
                } else if (!PeriodicTargetSelector.isUsableRange(context.targetRange())) {
                    candidate = unavailableCandidate(context,
                            "plant target range is unavailable",
                            candidateLastTarget,
                            candidateUnavailableActive,
                            candidateHeldMeasuredTarget);
                } else {
                    double measurement = selectionMeasurement(context);
                    if (!Double.isFinite(measurement)) {
                        candidate = unavailableCandidate(context,
                                "equivalent-position selection has no measurement or prior applied target",
                                candidateLastTarget,
                                candidateUnavailableActive,
                                candidateHeldMeasuredTarget);
                    } else {
                        double logicalValue = logicalResolution.target();
                        double physicalTarget = PeriodicTargetSelector.select(
                                logicalValue,
                                context.period(),
                                preference,
                                measurement,
                                context.targetRange());
                        if (Double.isFinite(physicalTarget)) {
                            PlantTargetResolution resolved =
                                    logicalResolution.withEquivalentTarget(physicalTarget,
                                            "selected equivalent position for "
                                                    + logicalResolution.reason());
                            if (commandTarget != null
                                    && !resolved.reportsCommandResolutionFor(commandTarget)) {
                                resolved = resolved.withoutSelectedCommand(commandTarget);
                            }
                            candidate = new ResolverCandidate(
                                    resolved, physicalTarget, false, Double.NaN);
                        } else {
                            candidate = unavailableCandidate(context,
                                    "logical target has no legal equivalent in the Plant range",
                                    candidateLastTarget,
                                    candidateUnavailableActive,
                                    candidateHeldMeasuredTarget);
                        }
                    }
                }

                return commit(candidate, cycle);
            } finally {
                resolving = false;
            }
        }

        private double selectionMeasurement(PlantTargetContext context) {
            if (context.feedbackAvailable()) return context.measurement();
            if (Double.isFinite(context.previousAppliedTarget())) {
                return context.previousAppliedTarget();
            }
            if (preference == CandidatePreference.PREFER_RANGE_CENTER) {
                return context.targetRange().center();
            }
            return Double.NaN;
        }

        private ResolverCandidate unavailableCandidate(PlantTargetContext context,
                                                       String reason,
                                                       double candidateLastTarget,
                                                       boolean candidateUnavailableActive,
                                                       double candidateHeldMeasuredTarget) {
            PlantTargetResolution resolution;
            if (unavailableKind == UnavailableKind.REJECT) {
                resolution = PlantTargetResolution.unavailable(reason);
                candidateUnavailableActive = true;
            } else if (unavailableKind == UnavailableKind.FALLBACK) {
                resolution = PlantTargetResolution.fallback(unavailableValue, reason);
                candidateUnavailableActive = true;
                candidateLastTarget = unavailableValue;
            } else if (unavailableKind == UnavailableKind.HOLD_LAST) {
                double target = Double.isFinite(candidateLastTarget)
                        ? candidateLastTarget
                        : unavailableValue;
                resolution = PlantTargetResolution.holdLast(target, reason);
                candidateUnavailableActive = true;
                candidateLastTarget = target;
            } else {
                if (!candidateUnavailableActive
                        || !Double.isFinite(candidateHeldMeasuredTarget)) {
                    candidateHeldMeasuredTarget = context.feedbackAvailable()
                            ? context.measurement()
                            : unavailableValue;
                }
                resolution = PlantTargetResolution.holdMeasured(
                        candidateHeldMeasuredTarget, reason);
                candidateUnavailableActive = true;
                candidateLastTarget = candidateHeldMeasuredTarget;
            }
            PlantTargetResolution resolved = commandTarget != null
                    ? resolution.withoutSelectedCommand(commandTarget)
                    : resolution;
            return new ResolverCandidate(resolved,
                    candidateLastTarget,
                    candidateUnavailableActive,
                    candidateHeldMeasuredTarget);
        }

        private PlantTargetResolution commit(ResolverCandidate candidate, long cycle) {
            lastTarget = candidate.lastTarget;
            unavailableActive = candidate.unavailableActive;
            heldMeasuredTarget = candidate.heldMeasuredTarget;
            lastResolution = candidate.resolution;
            lastCycle = cycle;
            return candidate.resolution;
        }

        @Override
        public void reset() {
            if (resolving) {
                throw new IllegalStateException(
                        "Equivalent-position target cannot reset while resolution is in progress");
            }
            if (resetting) {
                throw new IllegalStateException("Equivalent-position target reset reentered");
            }

            resetting = true;
            try {
                logicalTarget.reset();
                lastResolution = PlantTargetResolution.unavailable("not sampled");
                lastCycle = Long.MIN_VALUE;
                lastTarget = Double.NaN;
                unavailableActive = false;
                heldMeasuredTarget = Double.NaN;
            } finally {
                resetting = false;
            }
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty())
                    ? "equivalentPositionTarget"
                    : prefix;
            dbg.addData(p + ".class", "EquivalentPositionsTargetResolver")
                    .addData(p + ".preference", preference)
                    .addData(p + ".unavailablePolicy", unavailableKind)
                    .addData(p + ".lastResolution", lastResolution);
            logicalTarget.debugDump(dbg, p + ".logical");
        }
    }

    private static final class PlannerTargetResolver implements PlantTargetResolver {
        private final Source<PlantTargetRequest> requestSource;
        private final CandidatePreference preference;
        private final UnreachablePolicy unreachablePolicy;
        private final double maxObservationAgeSec;
        private final double minQuality;
        private final UnavailableKind unavailableKind;
        private final double unavailableValue;

        private PlantTargetResolution lastResolution =
                PlantTargetResolution.unavailable("not sampled");
        private long lastCycle = Long.MIN_VALUE;
        private double lastTarget = Double.NaN;
        private boolean unavailableActive;
        private double heldMeasuredTarget = Double.NaN;
        private boolean resolving;
        private boolean resetting;

        PlannerTargetResolver(Source<PlantTargetRequest> requestSource,
                              CandidatePreference preference,
                              UnreachablePolicy unreachablePolicy,
                              double maxObservationAgeSec,
                              double minQuality,
                              UnavailableKind unavailableKind,
                              double unavailableValue) {
            this.requestSource = Objects.requireNonNull(requestSource, "requestSource");
            this.preference = Objects.requireNonNull(preference, "preference");
            this.unreachablePolicy = Objects.requireNonNull(unreachablePolicy, "unreachablePolicy");
            this.maxObservationAgeSec = maxObservationAgeSec;
            this.minQuality = minQuality;
            this.unavailableKind = Objects.requireNonNull(unavailableKind, "unavailableKind");
            this.unavailableValue = unavailableValue;
            if ((unavailableKind == UnavailableKind.FALLBACK
                    || unavailableKind == UnavailableKind.HOLD_LAST
                    || unavailableKind == UnavailableKind.HOLD_MEASURED)
                    && !Double.isFinite(unavailableValue)) {
                throw new IllegalArgumentException("Unavailable policy fallback/initial value must be finite");
            }
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            Objects.requireNonNull(context, "context");
            long cycle = Objects.requireNonNull(clock, "clock").cycle();
            if (resolving) {
                throw new IllegalStateException(
                        "Plant target planner resolution reentered; check the request source "
                                + "graph for a cycle");
            }
            if (resetting) {
                throw new IllegalStateException(
                        "Plant target planner cannot resolve while reset is in progress");
            }
            if (cycle == lastCycle) return lastResolution;

            resolving = true;
            try {
                double candidateLastTarget = lastTarget;
                boolean candidateUnavailableActive = unavailableActive;
                double candidateHeldMeasuredTarget = heldMeasuredTarget;
                if (unavailableKind == UnavailableKind.HOLD_MEASURED
                        && hasSamplingGap(lastCycle, cycle)) {
                    candidateUnavailableActive = false;
                    candidateHeldMeasuredTarget = Double.NaN;
                }

                PlantTargetRequest request = requestSource.get(clock);
                ResolverCandidate candidate;
                if (request == null || !request.hasAlternatives()) {
                    candidate = unavailableCandidate(context,
                            request != null ? request.reason() : "missing plant target request",
                            candidateLastTarget,
                            candidateUnavailableActive,
                            candidateHeldMeasuredTarget);
                } else {
                    AlternativeSearch search = chooseBest(request, context, clock);
                    AlternativeChoice best = search.choice;
                    if (best == null) {
                        candidate = unavailableCandidate(context,
                                search.rejectionReason,
                                candidateLastTarget,
                                candidateUnavailableActive,
                                candidateHeldMeasuredTarget);
                    } else {
                        PlantTargetResolution resolution = PlantTargetResolution.planned(
                                best.target,
                                best.alternative,
                                best.selectedAgeSec,
                                best.clamped,
                                best.clamped
                                        ? "candidate clamped to range"
                                        : "selected plant target candidate");
                        candidate = new ResolverCandidate(
                                resolution, best.target, false, Double.NaN);
                    }
                }

                return commit(candidate, cycle);
            } finally {
                resolving = false;
            }
        }

        private ResolverCandidate unavailableCandidate(PlantTargetContext context,
                                                       String reason,
                                                       double candidateLastTarget,
                                                       boolean candidateUnavailableActive,
                                                       double candidateHeldMeasuredTarget) {
            PlantTargetResolution resolution;
            if (unavailableKind == UnavailableKind.REJECT) {
                resolution = PlantTargetResolution.unavailable(reason);
                candidateUnavailableActive = true;
            } else if (unavailableKind == UnavailableKind.FALLBACK) {
                resolution = PlantTargetResolution.fallback(unavailableValue, reason);
                candidateUnavailableActive = true;
                candidateLastTarget = unavailableValue;
            } else if (unavailableKind == UnavailableKind.HOLD_LAST) {
                double target = Double.isFinite(candidateLastTarget)
                        ? candidateLastTarget
                        : unavailableValue;
                resolution = PlantTargetResolution.holdLast(target, reason);
                candidateUnavailableActive = true;
                candidateLastTarget = target;
            } else {
                if (!candidateUnavailableActive
                        || !Double.isFinite(candidateHeldMeasuredTarget)) {
                    candidateHeldMeasuredTarget = context.feedbackAvailable()
                            ? context.measurement()
                            : unavailableValue;
                }
                resolution = PlantTargetResolution.holdMeasured(
                        candidateHeldMeasuredTarget, reason);
                candidateUnavailableActive = true;
                candidateLastTarget = candidateHeldMeasuredTarget;
            }
            return new ResolverCandidate(resolution,
                    candidateLastTarget,
                    candidateUnavailableActive,
                    candidateHeldMeasuredTarget);
        }

        private PlantTargetResolution commit(ResolverCandidate candidate, long cycle) {
            lastTarget = candidate.lastTarget;
            unavailableActive = candidate.unavailableActive;
            heldMeasuredTarget = candidate.heldMeasuredTarget;
            lastResolution = candidate.resolution;
            lastCycle = cycle;
            return candidate.resolution;
        }

        private AlternativeSearch chooseBest(PlantTargetRequest request,
                                             PlantTargetContext context,
                                             LoopClock clock) {
            ScalarRange range = context.targetRange();
            if (!PeriodicTargetSelector.isUsableRange(range))
                return AlternativeSearch.rejected("plant target range is unavailable");
            double measurement = context.feedbackAvailable() ? context.measurement() : context.previousAppliedTarget();
            if (!Double.isFinite(measurement)) measurement = 0.0;

            AlternativeChoice best = null;
            String firstRejection = null;
            for (PlantTargetRequest.Alternative alternative : request.alternatives()) {
                AlternativeAcceptance acceptance = alternativeAcceptance(alternative, clock);
                if (!acceptance.accepted) {
                    if (firstRejection == null) firstRejection = acceptance.reason;
                    continue;
                }
                AlternativeChoice choice = chooseForAlternative(alternative, acceptance.ageSec,
                        measurement, range, context);
                if (choice == null && firstRejection == null) {
                    firstRejection = "candidate '" + alternative.id()
                            + "' did not produce a reachable target";
                }
                if (choice != null && (best == null
                        || PeriodicTargetSelector.compare(preference, choice.target, best.target,
                        measurement, range) < 0)) {
                    best = choice;
                }
            }
            return best != null
                    ? AlternativeSearch.selected(best)
                    : AlternativeSearch.rejected(firstRejection != null
                    ? firstRejection
                    : "no plant target candidate passed observation gates or range");
        }

        private AlternativeAcceptance alternativeAcceptance(
                PlantTargetRequest.Alternative alternative,
                LoopClock clock) {
            if (!alternative.observed()) return AlternativeAcceptance.accepted(Double.NaN);

            if (!Double.isFinite(alternative.quality())
                    || alternative.quality() < 0.0
                    || alternative.quality() > 1.0) {
                return AlternativeAcceptance.rejected("candidate '" + alternative.id()
                        + "' has invalid observed quality " + alternative.quality()
                        + "; expected a finite value in [0, 1]");
            }
            if (!alternative.timestamp().isAvailable()) {
                return AlternativeAcceptance.rejected("candidate '" + alternative.id()
                        + "' has no available observation timestamp");
            }

            double ageSec = alternative.timestamp().ageSec(clock);
            if (!Double.isFinite(ageSec)) {
                return AlternativeAcceptance.rejected("candidate '" + alternative.id()
                        + "' observation timestamp is not valid in the current LoopClock epoch "
                        + "or is materially later than the current loop time");
            }
            if (alternative.quality() < minQuality) {
                return AlternativeAcceptance.rejected("candidate '" + alternative.id()
                        + "' observed quality " + alternative.quality()
                        + " is below minimum " + minQuality);
            }
            if (Double.isFinite(maxObservationAgeSec) && ageSec > maxObservationAgeSec) {
                return AlternativeAcceptance.rejected("candidate '" + alternative.id()
                        + "' observation age " + ageSec
                        + " exceeds maximum " + maxObservationAgeSec + " seconds");
            }
            return AlternativeAcceptance.accepted(ageSec);
        }

        private AlternativeChoice chooseForAlternative(
                PlantTargetRequest.Alternative alternative,
                double selectedAgeSec,
                double measurement,
                ScalarRange range,
                PlantTargetContext context) {
            double base = alternative.relative()
                    ? measurement + alternative.value()
                    : alternative.value();
            if (!Double.isFinite(base)) return null;
            if (!alternative.periodic()) {
                if (range.contains(base))
                    return new AlternativeChoice(alternative, base, selectedAgeSec, false);
                return clampedChoice(alternative, base, selectedAgeSec, range);
            }

            if (alternative.usesPlantPeriod()
                    && context.periodicity() != PositionPlant.Periodicity.PERIODIC) return null;
            double period = alternative.usesPlantPeriod()
                    ? context.period()
                    : alternative.period();
            if (!(period > 0.0) || !Double.isFinite(period)) return null;

            double selected = PeriodicTargetSelector.select(
                    base, period, preference, measurement, range);
            if (Double.isFinite(selected)) {
                return new AlternativeChoice(alternative, selected, selectedAgeSec, false);
            }
            return clampedChoice(alternative, base, selectedAgeSec, range);
        }

        private AlternativeChoice clampedChoice(
                PlantTargetRequest.Alternative alternative,
                double base,
                double selectedAgeSec,
                ScalarRange range) {
            if (unreachablePolicy != UnreachablePolicy.CLAMP_TO_RANGE
                    || !Double.isFinite(base)) {
                return null;
            }
            double clamped = range.clamp(base);
            return Double.isFinite(clamped) && range.contains(clamped)
                    ? new AlternativeChoice(alternative, clamped, selectedAgeSec, true)
                    : null;
        }

        @Override
        public void reset() {
            if (resolving) {
                throw new IllegalStateException(
                        "Plant target planner cannot reset while resolution is in progress");
            }
            if (resetting) {
                throw new IllegalStateException("Plant target planner reset reentered");
            }

            resetting = true;
            try {
                requestSource.reset();
                lastResolution = PlantTargetResolution.unavailable("not sampled");
                lastCycle = Long.MIN_VALUE;
                lastTarget = Double.NaN;
                unavailableActive = false;
                heldMeasuredTarget = Double.NaN;
            } finally {
                resetting = false;
            }
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plantTargetPlanner" : prefix;
            dbg.addData(p + ".class", "PlannerTargetResolver")
                    .addData(p + ".preference", preference)
                    .addData(p + ".unreachablePolicy", unreachablePolicy)
                    .addData(p + ".maxObservationAgeSec", maxObservationAgeSec)
                    .addData(p + ".minQuality", minQuality)
                    .addData(p + ".unavailablePolicy", unavailableKind)
                    .addData(p + ".lastResolution", lastResolution);
            requestSource.debugDump(dbg, p + ".request");
        }
    }

    /** Shared constant-work selector for one family {@code base + k * period}. */
    private static final class PeriodicTargetSelector {
        private static final double FIRST_INDEX_WITHOUT_UNIT_NEIGHBORS = 0x1.0p53;

        private PeriodicTargetSelector() {
        }

        static double select(double base,
                             double period,
                             CandidatePreference preference,
                             double measurement,
                             ScalarRange range) {
            if (!Double.isFinite(base)
                    || !(period > 0.0)
                    || !Double.isFinite(period)
                    || !Double.isFinite(measurement)
                    || preference == null
                    || !isUsableRange(range)) {
                return Double.NaN;
            }

            double best = Double.NaN;
            double reference = preferenceReference(preference, measurement, range);
            best = considerAnchor(base, period, preference, reference, measurement, range, best);
            if (Double.isFinite(range.minValue)) {
                best = considerAnchor(base, period, preference, range.minValue,
                        measurement, range, best);
            }
            if (Double.isFinite(range.maxValue)) {
                best = considerAnchor(base, period, preference, range.maxValue,
                        measurement, range, best);
            }
            return best;
        }

        /**
         * Examines floor-1, floor, ceil, and ceil+1 around one relevant coordinate. Three anchors
         * at most keep work independent of the number of equivalent positions in the range.
         */
        private static double considerAnchor(double base,
                                             double period,
                                             CandidatePreference preference,
                                             double anchor,
                                             double measurement,
                                             ScalarRange range,
                                             double best) {
            double index = periodicIndex(anchor, base, period);
            if (Double.isFinite(index)
                    && Math.abs(index) < FIRST_INDEX_WITHOUT_UNIT_NEIGHBORS) {
                double floor = Math.floor(index);
                double ceil = Math.ceil(index);
                best = considerIndex(base, period, preference, floor - 1.0,
                        measurement, range, best);
                best = considerIndex(base, period, preference, floor,
                        measurement, range, best);
                best = considerIndex(base, period, preference, ceil,
                        measurement, range, best);
                return considerIndex(base, period, preference, ceil + 1.0,
                        measurement, range, best);
            }

            double phase = periodicPhase(base, anchor, period);
            if (!Double.isFinite(phase)) return best;

            double down;
            double up;
            if (phase < 0.0) {
                down = phase;
                up = finiteSum(phase, period);
            } else if (phase > 0.0) {
                down = finiteSum(phase, -period);
                up = phase;
            } else {
                down = 0.0;
                up = 0.0;
            }

            double floorTarget = finiteSum(anchor, down);
            double ceilTarget = finiteSum(anchor, up);
            double belowOuter = finiteSum(floorTarget, -period);
            double aboveOuter = finiteSum(ceilTarget, period);
            double lower = Double.NaN;
            double upper = Double.NaN;
            lower = closerAtOrBelow(anchor, lower, floorTarget);
            lower = closerAtOrBelow(anchor, lower, ceilTarget);
            upper = closerAtOrAbove(anchor, upper, floorTarget);
            upper = closerAtOrAbove(anchor, upper, ceilTarget);
            if (Double.isFinite(index)) {
                double directTarget = periodicTarget(base, Math.rint(index), period);
                lower = closerAtOrBelow(anchor, lower, directTarget);
                upper = closerAtOrAbove(anchor, upper, directTarget);
            }

            best = considerTarget(preference, belowOuter, measurement, range, best);
            best = considerTarget(preference, lower, measurement, range, best);
            best = considerTarget(preference, upper, measurement, range, best);
            return considerTarget(preference, aboveOuter, measurement, range, best);
        }

        private static double considerIndex(double base,
                                            double period,
                                            CandidatePreference preference,
                                            double index,
                                            double measurement,
                                            ScalarRange range,
                                            double best) {
            if (!Double.isFinite(index)) return best;
            return considerTarget(preference, periodicTarget(base, index, period),
                    measurement, range, best);
        }

        private static double considerTarget(CandidatePreference preference,
                                              double target,
                                              double measurement,
                                              ScalarRange range,
                                              double best) {
            if (!Double.isFinite(target) || !range.contains(target)) return best;
            int comparison = !Double.isFinite(best)
                    ? -1
                    : compare(preference, target, best, measurement, range);
            if (comparison < 0 || (comparison == 0 && target < best)) return target;
            return best;
        }

        /** Returns a negative value when {@code left} is preferred over {@code right}. */
        static int compare(CandidatePreference preference,
                           double left,
                           double right,
                           double measurement,
                           ScalarRange range) {
            if (preference == CandidatePreference.PREFER_INCREASING) {
                return compareDirectional(left, right, measurement, true);
            }
            if (preference == CandidatePreference.PREFER_DECREASING) {
                return compareDirectional(left, right, measurement, false);
            }
            return compareDistance(left, right,
                    preferenceReference(preference, measurement, range));
        }

        private static double preferenceReference(CandidatePreference preference,
                                                  double measurement,
                                                  ScalarRange range) {
            if (preference == CandidatePreference.PREFER_RANGE_CENTER) {
                double center = range.center();
                if (Double.isFinite(center)) return center;
            }
            return measurement;
        }

        private static int compareDirectional(double left,
                                              double right,
                                              double measurement,
                                              boolean increasing) {
            boolean leftPreferred = increasing ? left >= measurement : left <= measurement;
            boolean rightPreferred = increasing ? right >= measurement : right <= measurement;
            if (leftPreferred != rightPreferred) return leftPreferred ? -1 : 1;
            return compareDistance(left, right, measurement);
        }

        /** Compares finite values by distance without letting subtraction overflow reverse order. */
        private static int compareDistance(double left, double right, double reference) {
            if (left == right) return 0;
            if (left == reference) return -1;
            if (right == reference) return 1;

            if (left <= reference && right <= reference) return left > right ? -1 : 1;
            if (left >= reference && right >= reference) return left < right ? -1 : 1;

            double lower = Math.min(left, right);
            double upper = Math.max(left, right);
            double midpoint = safeMidpoint(lower, upper);
            int distanceComparison;
            if (reference < midpoint) {
                distanceComparison = -1;
            } else if (reference > midpoint) {
                distanceComparison = 1;
            } else {
                distanceComparison = Double.compare(reference - lower, upper - reference);
            }
            return left == lower ? distanceComparison : -distanceComparison;
        }

        private static double finiteSum(double left, double right) {
            double sum = left + right;
            return Double.isFinite(sum) ? sum : Double.NaN;
        }

        private static double periodicTarget(double base, double index, double period) {
            double target = base + index * period;
            if (Double.isFinite(target)) return target;
            double regrouped = period * (index + base / period);
            return Double.isFinite(regrouped) ? regrouped : Double.NaN;
        }

        private static double periodicIndex(double anchor, double base, double period) {
            double index = (anchor - base) / period;
            if (Double.isFinite(index)) return index;
            double regrouped = anchor / period - base / period;
            return Double.isFinite(regrouped) ? regrouped : Double.NaN;
        }

        private static double closerAtOrBelow(double anchor, double current, double candidate) {
            if (!Double.isFinite(candidate) || candidate > anchor) return current;
            return !Double.isFinite(current) || candidate > current ? candidate : current;
        }

        private static double closerAtOrAbove(double anchor, double current, double candidate) {
            if (!Double.isFinite(candidate) || candidate < anchor) return current;
            return !Double.isFinite(current) || candidate < current ? candidate : current;
        }

        private static double safeMidpoint(double lower, double upper) {
            if (lower < 0.0 && upper >= 0.0) return (lower + upper) / 2.0;
            return lower + (upper - lower) / 2.0;
        }

        private static double periodicPhase(double base, double anchor, double period) {
            double baseRemainder = Math.IEEEremainder(base, period);
            double anchorRemainder = Math.IEEEremainder(anchor, period);
            if (!Double.isFinite(baseRemainder) || !Double.isFinite(anchorRemainder)) {
                return Double.NaN;
            }

            double rawDifference = baseRemainder - anchorRemainder;
            double halfPeriod = period / 2.0;
            if (baseRemainder >= 0.0
                    && anchorRemainder < 0.0
                    && rawDifference >= halfPeriod) {
                return (baseRemainder - period) - anchorRemainder;
            }
            if (baseRemainder < 0.0
                    && anchorRemainder >= 0.0
                    && rawDifference <= -halfPeriod) {
                return (baseRemainder + period) - anchorRemainder;
            }
            return rawDifference;
        }

        static boolean isUsableRange(ScalarRange range) {
            return range != null
                    && range.valid
                    && !Double.isNaN(range.minValue)
                    && !Double.isNaN(range.maxValue)
                    && range.minValue != Double.POSITIVE_INFINITY
                    && range.maxValue != Double.NEGATIVE_INFINITY
                    && range.minValue <= range.maxValue;
        }
    }

    private static final class AlternativeChoice {
        final PlantTargetRequest.Alternative alternative;
        final double target;
        final double selectedAgeSec;
        final boolean clamped;

        AlternativeChoice(PlantTargetRequest.Alternative alternative,
                          double target,
                          double selectedAgeSec,
                          boolean clamped) {
            this.alternative = alternative;
            this.target = target;
            this.selectedAgeSec = selectedAgeSec;
            this.clamped = clamped;
        }
    }

    /** Result of checking one request alternative's live observation metadata. */
    private static final class AlternativeAcceptance {
        final boolean accepted;
        final double ageSec;
        final String reason;

        private AlternativeAcceptance(boolean accepted, double ageSec, String reason) {
            this.accepted = accepted;
            this.ageSec = ageSec;
            this.reason = reason;
        }

        static AlternativeAcceptance accepted(double ageSec) {
            return new AlternativeAcceptance(true, ageSec, "");
        }

        static AlternativeAcceptance rejected(String reason) {
            return new AlternativeAcceptance(false, Double.NaN, reason);
        }
    }

    /** Alternative search outcome retaining the first actionable rejection when none wins. */
    private static final class AlternativeSearch {
        final AlternativeChoice choice;
        final String rejectionReason;

        private AlternativeSearch(AlternativeChoice choice, String rejectionReason) {
            this.choice = choice;
            this.rejectionReason = rejectionReason;
        }

        static AlternativeSearch selected(AlternativeChoice choice) {
            return new AlternativeSearch(choice, "");
        }

        static AlternativeSearch rejected(String reason) {
            return new AlternativeSearch(null, reason);
        }
    }

    private static String cleanName(String name) {
        return name == null || name.trim().isEmpty() ? "layer" : name.trim();
    }

    private static boolean hasSamplingGap(long lastResolvedCycle, long cycle) {
        if (lastResolvedCycle == Long.MIN_VALUE) return false;
        return cycle != lastResolvedCycle && cycle != lastResolvedCycle + 1L;
    }
}
