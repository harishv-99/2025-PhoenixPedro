package edu.ftcphoenix.fw.actuation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Factory and builders for plant target sources.
 *
 * <p>{@code PlantTargets} is the central place to generate values for {@link Plant}s. It keeps one
 * student-facing rule: <b>anything intended to become a Plant target should be expressed as a
 * {@link PlantTargetSource}</b>. Simple values are lifted with {@link #exact(double)} or
 * {@link #exact(ScalarSource)}. Behavior arbitration uses {@link #overlay(PlantTargetSource)}:
 * every layer's activation gate is sampled once, then target producers are resolved lazily from
 * highest to lowest priority. Layers added with {@code add(...)} must produce a target when enabled,
 * while {@code addIfAvailable(...)} is the explicit opt-in for enabled layers that may fall through.
 * Periodic/equivalent target selection uses {@link #plan()}.</p>
 *
 * <h2>Typical exact target</h2>
 * <pre>{@code
 * ScalarTarget liftCommand = ScalarTarget.held(0.0);
 * PositionPlant lift = FtcActuators.plant(hardwareMap)
 *     .motor("lift", Direction.FORWARD)
 *     .position()
 *     ...
 *     .targetedBy(liftCommand)   // builder lifts it to a PlantTargetSource
 *     .build();
 * }</pre>
 *
 * <h2>Typical overlay</h2>
 * <pre>{@code
 * PlantTargetSource feederTarget = PlantTargets.overlay(0.0)
 *     .add("feedPulse", feedPulse.activeSource(), feedPulse)
 *     .add("eject", ejectRequested, -1.0)
 *     .build();
 * }</pre>
 *
 * <h2>Typical smart planner</h2>
 * <pre>{@code
 * PlantTargetSource turretTarget = PlantTargets.plan()
 *     .request(clock -> PlantTargetRequest.equivalentPosition("faceGoal", desiredAngleDeg.get()))
 *     .nearestToMeasurement()
 *     .rejectUnreachable()
 *     .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
 * }</pre>
 */
public final class PlantTargets {

    private PlantTargets() {
    }

    /**
     * How a planner chooses among multiple reachable candidates.
     */
    public enum CandidatePreference {
        /**
         * Choose the reachable candidate closest to the current measurement.
         */
        NEAREST_TO_MEASUREMENT,
        /**
         * Prefer candidates that move upward/increasing from the current measurement.
         */
        PREFER_INCREASING,
        /**
         * Prefer candidates that move downward/decreasing from the current measurement.
         */
        PREFER_DECREASING,
        /**
         * Prefer candidates closest to the center of the legal range.
         */
        PREFER_RANGE_CENTER
    }

    /**
     * How a planner handles a finite but unreachable candidate.
     */
    public enum UnreachablePolicy {
        /**
         * Reject unreachable candidates; unavailable policy must then produce the target.
         */
        REJECT,
        /**
         * Clamp unreachable candidates into the plant's legal range.
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
     * Convert a finite constant into an exact plant target source.
     */
    public static PlantTargetSource exact(double value) {
        if (!Double.isFinite(value))
            throw new IllegalArgumentException("Plant target value must be finite, got " + value);
        return new ExactPlantTargetSource(ScalarSource.constant(value), "constant " + value);
    }

    /**
     * Convert a scalar source into an exact plant target source.
     *
     * <p>The scalar is sampled during plant update. If it returns NaN or infinity, the target is
     * reported unavailable so the surrounding overlay or plant telemetry can explain the failure.</p>
     */
    public static PlantTargetSource exact(ScalarSource source) {
        return new ExactPlantTargetSource(Objects.requireNonNull(source, "source"), "exact scalar source");
    }

    /**
     * Alias for {@link #exact(ScalarSource)} that reads naturally at builder call sites.
     */
    public static PlantTargetSource fromScalar(ScalarSource source) {
        return exact(source);
    }

    /**
     * Return the previous requested target when available, otherwise {@code initialTarget}.
     */
    public static PlantTargetSource holdLastTarget(double initialTarget) {
        return new HoldLastTargetSource(initialTarget);
    }

    /**
     * Latch the current measurement while this source is continuously resolved.
     *
     * <p>An entry is the first resolution after construction/reset or after a loop-cycle sampling
     * gap. Repeated resolution in the same cycle and resolution in consecutive cycles retain the
     * same capture. Resolution after one or more unobserved cycles captures the current measurement
     * again.</p>
     */
    public static PlantTargetSource holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement) {
        return new HoldMeasuredTargetSource(fallbackIfNoMeasurement);
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
     * Start a plant-target overlay with a total base source.
     */
    public static OverlayBuilder overlay(PlantTargetSource baseTarget) {
        return new OverlayBuilder(Objects.requireNonNull(baseTarget, "baseTarget"));
    }

    /**
     * Start a smart target planner.
     *
     * <p>The staged builder asks required questions in order: request source, candidate
     * preference, unreachable-candidate policy, then unavailable-target policy. Optional
     * observation-age/quality tuning is available after the required preference and unreachable
     * answers. This shape avoids hidden defaults for choices that affect motion semantics.</p>
     */
    public static PlanRequestStage plan() {
        return new PlannerBuilder();
    }

    private static final class ExactPlantTargetSource implements PlantTargetSource {
        private final ScalarSource source;
        private final String reason;

        ExactPlantTargetSource(ScalarSource source, String reason) {
            this.source = Objects.requireNonNull(source, "source").memoized();
            this.reason = reason;
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            double v = source.getAsDouble(clock);
            return Double.isFinite(v)
                    ? PlantTargetPlan.exact(v, reason)
                    : PlantTargetPlan.unavailable("exact scalar source returned non-finite target: " + v);
        }

        @Override
        public void reset() {
            source.reset();
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "exactPlantTarget" : prefix;
            dbg.addData(p + ".class", "ExactPlantTargetSource")
                    .addData(p + ".reason", reason);
            source.debugDump(dbg, p + ".scalar");
        }
    }

    private static final class HoldLastTargetSource implements PlantTargetSource {
        private final double initialTarget;

        HoldLastTargetSource(double initialTarget) {
            if (!Double.isFinite(initialTarget))
                throw new IllegalArgumentException("initialTarget must be finite");
            this.initialTarget = initialTarget;
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            double target = Double.isFinite(context.previousRequestedTarget()) ? context.previousRequestedTarget() : initialTarget;
            return PlantTargetPlan.holdLast(target, "holding previous requested target");
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "holdLastPlantTarget" : prefix;
            dbg.addData(p + ".class", "HoldLastTargetSource")
                    .addData(p + ".initialTarget", initialTarget);
        }
    }

    private static final class HoldMeasuredTargetSource implements PlantTargetSource {
        private final double fallback;
        private boolean latched;
        private double latchedTarget;
        private long lastResolvedCycle = Long.MIN_VALUE;

        HoldMeasuredTargetSource(double fallback) {
            if (!Double.isFinite(fallback))
                throw new IllegalArgumentException("fallbackIfNoMeasurement must be finite");
            this.fallback = fallback;
            this.latchedTarget = fallback;
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            long cycle = Objects.requireNonNull(clock, "clock").cycle();
            if (!latched || hasSamplingGap(lastResolvedCycle, cycle)) {
                latchedTarget = context.feedbackAvailable() ? context.measurement() : fallback;
                latched = true;
            }
            lastResolvedCycle = cycle;
            return PlantTargetPlan.holdMeasured(latchedTarget, "holding measured target captured on entry");
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
            dbg.addData(p + ".class", "HoldMeasuredTargetSource")
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
     * {@link #add(String, BooleanSource, PlantTargetSource)} must produce a target when enabled;
     * unavailable active layers report an unavailable plan instead of silently falling through.
     * Use {@link #addIfAvailable(String, BooleanSource, PlantTargetSource)} only when an
     * enabled-but-unavailable layer should explicitly continue to the next lower priority.</p>
     */
    public static final class OverlayBuilder {
        private final PlantTargetSource base;
        private final List<Layer> layers = new ArrayList<Layer>();

        private OverlayBuilder(PlantTargetSource base) {
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
         * If the target source cannot produce a target, the overlay reports that failure instead of
         * silently falling through to a lower-priority behavior.</p>
         */
        public OverlayBuilder add(String name, BooleanSource enabled, PlantTargetSource target) {
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
         * For most behavior layers, prefer {@link #add(String, BooleanSource, PlantTargetSource)} so
         * missing targets are visible as failures.</p>
         */
        public OverlayBuilder addIfAvailable(String name, BooleanSource enabled, PlantTargetSource target) {
            layers.add(new Layer(cleanName(name), Objects.requireNonNull(enabled, "enabled"),
                    Objects.requireNonNull(target, "target"), LayerUnavailablePolicy.FALL_THROUGH));
            return this;
        }

        /**
         * Build the overlay source.
         */
        public PlantTargetSource build() {
            return new OverlayTargetSource(base, layers.toArray(new Layer[0]));
        }
    }

    private static final class Layer {
        final String name;
        final BooleanSource enabled;
        final PlantTargetSource target;
        final LayerUnavailablePolicy unavailablePolicy;

        Layer(String name,
              BooleanSource enabled,
              PlantTargetSource target,
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
        private static final PlantTargetPlan NOT_SAMPLED_PLAN =
                PlantTargetPlan.unavailable("not sampled");
        private static final PlantTargetPlan GATE_NOT_SAMPLED_PLAN =
                PlantTargetPlan.unavailable("activation gate not sampled");
        private static final PlantTargetPlan DISABLED_PLAN =
                PlantTargetPlan.unavailable("layer disabled");
        private static final PlantTargetPlan ENABLED_NOT_REACHED_PLAN =
                PlantTargetPlan.unavailable("enabled layer target not reached");
        private static final PlantTargetPlan SHADOWED_PLAN =
                PlantTargetPlan.unavailable("enabled layer target shadowed by higher-priority layer");
        private static final PlantTargetPlan GATE_FAILED_PLAN =
                PlantTargetPlan.unavailable("activation gate sampling failed");
        private static final PlantTargetPlan TARGET_FAILED_PLAN =
                PlantTargetPlan.unavailable("target resolution failed");

        boolean enabled;
        boolean targetSampled;
        boolean fellThrough;
        LayerResolutionState resolutionState = LayerResolutionState.NOT_SAMPLED;
        PlantTargetPlan plan = NOT_SAMPLED_PLAN;

        void clearForGateSampling() {
            enabled = false;
            targetSampled = false;
            fellThrough = false;
            resolutionState = LayerResolutionState.NOT_SAMPLED;
            plan = GATE_NOT_SAMPLED_PLAN;
        }

        void reset() {
            enabled = false;
            targetSampled = false;
            fellThrough = false;
            resolutionState = LayerResolutionState.NOT_SAMPLED;
            plan = NOT_SAMPLED_PLAN;
        }
    }

    private static final class OverlayTargetSource implements PlantTargetSource {
        private static final PlantTargetPlan INCOMPLETE_PLAN =
                PlantTargetPlan.unavailable("overlay resolution did not complete");

        private final PlantTargetSource base;
        private final Layer[] layers;
        private final LayerRuntimeState[] layerStates;
        private PlantTargetPlan lastPlan = PlantTargetPlan.unavailable("not sampled");
        private String lastWinner = "not sampled";
        private boolean lastBaseSampled;

        OverlayTargetSource(PlantTargetSource base, Layer[] layers) {
            this.base = base;
            this.layers = layers;
            this.layerStates = new LayerRuntimeState[layers.length];
            for (int i = 0; i < layerStates.length; i++) {
                layerStates[i] = new LayerRuntimeState();
            }
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            lastPlan = INCOMPLETE_PLAN;
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
                    state.plan = LayerRuntimeState.GATE_FAILED_PLAN;
                    lastWinner = layer.name + " activation failed";
                    lastPlan = state.plan;
                    throw failure;
                }
                state.resolutionState = state.enabled
                        ? LayerResolutionState.ENABLED_NOT_REACHED
                        : LayerResolutionState.DISABLED;
                state.plan = state.enabled
                        ? LayerRuntimeState.ENABLED_NOT_REACHED_PLAN
                        : LayerRuntimeState.DISABLED_PLAN;
            }

            PlantTargetPlan winner = null;
            int decisiveLayerIndex = -1;
            for (int i = layers.length - 1; i >= 0; i--) {
                Layer layer = layers[i];
                LayerRuntimeState state = layerStates[i];
                if (!state.enabled) continue;

                state.targetSampled = true;
                PlantTargetPlan plan;
                try {
                    plan = layer.target.resolve(context, clock);
                    if (plan == null) {
                        throw new NullPointerException(
                                "Plant target layer '" + layer.name + "' returned null plan");
                    }
                } catch (RuntimeException failure) {
                    state.resolutionState = LayerResolutionState.TARGET_FAILED;
                    state.plan = LayerRuntimeState.TARGET_FAILED_PLAN;
                    lastWinner = layer.name + " target failed";
                    lastPlan = state.plan;
                    throw failure;
                }
                state.plan = plan;
                if (!plan.hasTarget()) {
                    if (layer.unavailablePolicy == LayerUnavailablePolicy.FALL_THROUGH) {
                        state.fellThrough = true;
                        state.resolutionState = LayerResolutionState.FELL_THROUGH;
                        continue;
                    }
                    state.resolutionState = LayerResolutionState.REQUIRED_UNAVAILABLE;
                    lastWinner = layer.name + " unavailable";
                    winner = PlantTargetPlan.unavailable("enabled plant target layer '" + layer.name + "' produced no target: " + plan.reason());
                    decisiveLayerIndex = i;
                    break;
                }
                state.resolutionState = LayerResolutionState.SELECTED;
                lastWinner = layer.name;
                winner = plan;
                decisiveLayerIndex = i;
                break;
            }

            if (decisiveLayerIndex >= 0) {
                for (int i = decisiveLayerIndex - 1; i >= 0; i--) {
                    LayerRuntimeState state = layerStates[i];
                    if (state.enabled
                            && state.resolutionState == LayerResolutionState.ENABLED_NOT_REACHED) {
                        state.resolutionState = LayerResolutionState.SHADOWED;
                        state.plan = LayerRuntimeState.SHADOWED_PLAN;
                    }
                }
            } else {
                lastBaseSampled = true;
                lastWinner = "base";
                winner = base.resolve(context, clock);
            }

            lastPlan = winner;
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
            lastPlan = PlantTargetPlan.unavailable("not sampled");
            lastWinner = "not sampled";
            lastBaseSampled = false;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plantTargetOverlay" : prefix;
            dbg.addData(p + ".class", "PlantTargetOverlay")
                    .addData(p + ".winner", lastWinner)
                    .addData(p + ".plan", lastPlan)
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
                        .addData(lp + ".plan", state.plan);
                layer.target.debugDump(dbg, lp + ".target");
            }
        }
    }

    /**
     * First required planner question: where do target requests come from?
     */
    public interface PlanRequestStage {
        /**
         * Supply a source of plant target requests.
         *
         * <p>The next stage asks how reachable candidates should be preferred. There is no generic
         * {@code select()} entry method because this is a single required choice: pick one answer,
         * then continue to the unreachable-candidate policy question.</p>
         */
        PlanPreferenceStage request(Source<PlantTargetRequest> request);
    }

    /**
     * Required planner question: how should reachable candidates be preferred?
     *
     * <p>Each method answers the preference question and advances to the next required question. The
     * returned type intentionally hides the other preference methods so a later call cannot silently
     * replace an earlier answer.</p>
     */
    public interface PlanPreferenceStage {
        /**
         * Choose the reachable candidate nearest to the current measurement.
         */
        PlanUnreachableStage nearestToMeasurement();

        /**
         * Prefer candidates that move upward/increasing from the current measurement.
         */
        PlanUnreachableStage preferIncreasing();

        /**
         * Prefer candidates that move downward/decreasing from the current measurement.
         */
        PlanUnreachableStage preferDecreasing();

        /**
         * Prefer candidates closest to the legal range center.
         */
        PlanUnreachableStage preferRangeCenter();
    }

    /**
     * Required planner question: what should happen to finite candidates outside the legal range?
     */
    public interface PlanUnreachableStage {
        /**
         * Reject unreachable candidates and let the unavailable policy produce the target.
         */
        PlanReadyStage rejectUnreachable();

        /**
         * Clamp unreachable candidates into the legal range before scoring them.
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
        PlanUnavailableBranch whenUnavailable();
    }

    /**
     * Observed-candidate age/quality acceptance tuning branch.
     */
    public interface PlanAcceptBranch {
        /**
         * Ignore observed candidates older than {@code ageSec} at planner resolution.
         *
         * <p>Timeless candidates created with the short factories are not observations and are not
         * affected by this limit.</p>
         *
         * @param ageSec finite maximum observation age in seconds, inclusive
         * @throws IllegalArgumentException if {@code ageSec} is negative or non-finite
         */
        PlanAcceptBranch maxObservationAgeSec(double ageSec);

        /**
         * Ignore candidates below the supplied inclusive quality threshold.
         *
         * <p>Timeless candidates have implicit quality {@code 1.0}.</p>
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

    /**
     * Required branch that makes a planner total.
     */
    public interface PlanUnavailableBranch {
        /**
         * Produce a fixed fallback target when no candidate can be selected.
         */
        PlantTargetSource fallbackTo(double target);

        /**
         * Hold the last planned target; use {@code initialTarget} before any successful plan.
         */
        PlantTargetSource holdLastTarget(double initialTarget);

        /**
         * Latch current measurement on each continuously sampled unavailable entry.
         *
         * <p>A sampling gap ends the current unavailable entry. If this planner is selected again
         * and remains unavailable, it captures the then-current measurement.</p>
         */
        PlantTargetSource holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement);

        /**
         * Explicitly report unavailability when no target can be selected.
         */
        PlantTargetSource reportUnavailable();
    }

    private enum UnavailableKind {FALLBACK, HOLD_LAST, HOLD_MEASURED, REJECT}

    private static final class PlannerBuilder implements PlanRequestStage, PlanPreferenceStage,
            PlanUnreachableStage, PlanReadyStage, PlanAcceptBranch, PlanUnavailableBranch {
        private Source<PlantTargetRequest> request;
        private CandidatePreference preference;
        private UnreachablePolicy unreachablePolicy;
        private double maxObservationAgeSec = Double.POSITIVE_INFINITY;
        private double minQuality = 0.0;

        @Override
        public PlanPreferenceStage request(Source<PlantTargetRequest> request) {
            this.request = Objects.requireNonNull(request, "request");
            return this;
        }

        @Override
        public PlanUnreachableStage nearestToMeasurement() {
            preference = CandidatePreference.NEAREST_TO_MEASUREMENT;
            return this;
        }

        @Override
        public PlanUnreachableStage preferIncreasing() {
            preference = CandidatePreference.PREFER_INCREASING;
            return this;
        }

        @Override
        public PlanUnreachableStage preferDecreasing() {
            preference = CandidatePreference.PREFER_DECREASING;
            return this;
        }

        @Override
        public PlanUnreachableStage preferRangeCenter() {
            preference = CandidatePreference.PREFER_RANGE_CENTER;
            return this;
        }

        @Override
        public PlanReadyStage rejectUnreachable() {
            unreachablePolicy = UnreachablePolicy.REJECT;
            return this;
        }

        @Override
        public PlanReadyStage clampUnreachableToRange() {
            unreachablePolicy = UnreachablePolicy.CLAMP_TO_RANGE;
            return this;
        }

        @Override
        public PlanAcceptBranch accept() {
            return this;
        }

        @Override
        public PlanUnavailableBranch whenUnavailable() {
            return this;
        }

        @Override
        public PlanAcceptBranch maxObservationAgeSec(double ageSec) {
            if (ageSec < 0.0 || !Double.isFinite(ageSec))
                throw new IllegalArgumentException("maxObservationAgeSec must be finite and >= 0");
            maxObservationAgeSec = ageSec;
            return this;
        }

        @Override
        public PlanAcceptBranch minQuality(double quality) {
            if (!Double.isFinite(quality) || quality < 0.0 || quality > 1.0)
                throw new IllegalArgumentException("minQuality must be finite and in [0, 1]");
            minQuality = quality;
            return this;
        }

        @Override
        public PlanReadyStage doneAccept() {
            return this;
        }

        @Override
        public PlantTargetSource fallbackTo(double target) {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality, UnavailableKind.FALLBACK, target);
        }

        @Override
        public PlantTargetSource holdLastTarget(double initialTarget) {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality, UnavailableKind.HOLD_LAST, initialTarget);
        }

        @Override
        public PlantTargetSource holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement) {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality, UnavailableKind.HOLD_MEASURED, fallbackIfNoMeasurement);
        }

        @Override
        public PlantTargetSource reportUnavailable() {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxObservationAgeSec, minQuality, UnavailableKind.REJECT, Double.NaN);
        }

        private void validateRequest() {
            if (request == null)
                throw new IllegalStateException("PlantTargets.plan() requires request(...)");
            if (preference == null)
                throw new IllegalStateException("PlantTargets.plan() requires a preference choice such as nearestToMeasurement() before whenUnavailable()");
            if (unreachablePolicy == null)
                throw new IllegalStateException("PlantTargets.plan() requires rejectUnreachable() or clampUnreachableToRange() before whenUnavailable()");
        }
    }

    private static final class PlannerTargetSource implements PlantTargetSource {
        private final Source<PlantTargetRequest> requestSource;
        private final CandidatePreference preference;
        private final UnreachablePolicy unreachablePolicy;
        private final double maxObservationAgeSec;
        private final double minQuality;
        private final UnavailableKind unavailableKind;
        private final double unavailableValue;

        private PlantTargetPlan lastPlan = PlantTargetPlan.unavailable("not sampled");
        private long lastCycle = Long.MIN_VALUE;
        private double lastTarget = Double.NaN;
        private boolean unavailableActive;
        private double heldMeasuredTarget = Double.NaN;

        PlannerTargetSource(Source<PlantTargetRequest> requestSource,
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
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            long cycle = Objects.requireNonNull(clock, "clock").cycle();
            if (cycle == lastCycle) return lastPlan;
            if (unavailableKind == UnavailableKind.HOLD_MEASURED
                    && hasSamplingGap(lastCycle, cycle)) {
                unavailableActive = false;
                heldMeasuredTarget = Double.NaN;
            }
            lastCycle = cycle;

            PlantTargetRequest request = requestSource.get(clock);
            if (request == null || !request.hasCandidates()) {
                lastPlan = unavailable(context, request != null ? request.reason() : "missing plant target request");
                return lastPlan;
            }

            CandidateSearch search = chooseBest(request, context, clock);
            CandidateChoice best = search.choice;
            if (best == null) {
                lastPlan = unavailable(context, search.rejectionReason);
                return lastPlan;
            }

            unavailableActive = false;
            heldMeasuredTarget = Double.NaN;
            lastTarget = best.target;
            lastPlan = PlantTargetPlan.planned(best.target, best.candidate, best.selectedAgeSec,
                    best.clamped,
                    best.clamped ? "candidate clamped to range" : "selected plant target candidate");
            return lastPlan;
        }

        private PlantTargetPlan unavailable(PlantTargetContext context, String reason) {
            if (unavailableKind == UnavailableKind.REJECT) {
                unavailableActive = true;
                return PlantTargetPlan.unavailable(reason);
            }
            if (unavailableKind == UnavailableKind.FALLBACK) {
                unavailableActive = true;
                lastTarget = unavailableValue;
                return PlantTargetPlan.fallback(unavailableValue, reason);
            }
            if (unavailableKind == UnavailableKind.HOLD_LAST) {
                unavailableActive = true;
                double target = Double.isFinite(lastTarget) ? lastTarget : unavailableValue;
                return PlantTargetPlan.holdLast(target, reason);
            }
            if (!unavailableActive || !Double.isFinite(heldMeasuredTarget)) {
                heldMeasuredTarget = context.feedbackAvailable() ? context.measurement() : unavailableValue;
            }
            unavailableActive = true;
            lastTarget = heldMeasuredTarget;
            return PlantTargetPlan.holdMeasured(heldMeasuredTarget, reason);
        }

        private CandidateSearch chooseBest(PlantTargetRequest request,
                                           PlantTargetContext context,
                                           LoopClock clock) {
            ScalarRange range = context.targetRange();
            if (range == null || !range.valid)
                return CandidateSearch.rejected("plant target range is unavailable");
            double measurement = context.feedbackAvailable() ? context.measurement() : context.previousAppliedTarget();
            if (!Double.isFinite(measurement)) measurement = 0.0;

            CandidateChoice best = null;
            String firstRejection = null;
            for (PlantTargetCandidate candidate : request.candidates()) {
                CandidateAcceptance acceptance = candidateAcceptance(candidate, clock);
                if (!acceptance.accepted) {
                    if (firstRejection == null) firstRejection = acceptance.reason;
                    continue;
                }
                CandidateChoice choice = chooseForCandidate(candidate, acceptance.ageSec,
                        measurement, range, context);
                if (choice == null && firstRejection == null) {
                    firstRejection = "candidate '" + candidate.id
                            + "' did not produce a reachable target";
                }
                if (choice != null && (best == null || score(choice, measurement, range) < score(best, measurement, range))) {
                    best = choice;
                }
            }
            return best != null
                    ? CandidateSearch.selected(best)
                    : CandidateSearch.rejected(firstRejection != null
                    ? firstRejection
                    : "no plant target candidate passed observation gates or range");
        }

        private CandidateAcceptance candidateAcceptance(PlantTargetCandidate candidate,
                                                        LoopClock clock) {
            if (!candidate.isObserved()) return CandidateAcceptance.accepted(Double.NaN);

            if (!Double.isFinite(candidate.quality)
                    || candidate.quality < 0.0
                    || candidate.quality > 1.0) {
                return CandidateAcceptance.rejected("candidate '" + candidate.id
                        + "' has invalid observed quality " + candidate.quality
                        + "; expected a finite value in [0, 1]");
            }
            if (!Double.isFinite(candidate.timestampSec)) {
                return CandidateAcceptance.rejected("candidate '" + candidate.id
                        + "' has invalid observation timestamp " + candidate.timestampSec
                        + "; expected a finite LoopClock time");
            }

            double nowSec = clock.nowSec();
            if (!Double.isFinite(nowSec)) {
                return CandidateAcceptance.rejected("planner LoopClock time is non-finite: " + nowSec);
            }
            if (candidate.timestampSec - nowSec > FUTURE_OBSERVATION_TOLERANCE_SEC) {
                return CandidateAcceptance.rejected("candidate '" + candidate.id
                        + "' observation timestamp " + candidate.timestampSec
                        + " is later than LoopClock time " + nowSec);
            }

            double ageSec = Math.max(0.0, nowSec - candidate.timestampSec);
            if (!Double.isFinite(ageSec)) {
                return CandidateAcceptance.rejected("candidate '" + candidate.id
                        + "' produced non-finite observation age");
            }
            if (candidate.quality < minQuality) {
                return CandidateAcceptance.rejected("candidate '" + candidate.id
                        + "' observed quality " + candidate.quality
                        + " is below minimum " + minQuality);
            }
            if (Double.isFinite(maxObservationAgeSec) && ageSec > maxObservationAgeSec) {
                return CandidateAcceptance.rejected("candidate '" + candidate.id
                        + "' observation age " + ageSec
                        + " exceeds maximum " + maxObservationAgeSec + " seconds");
            }
            return CandidateAcceptance.accepted(ageSec);
        }

        private CandidateChoice chooseForCandidate(PlantTargetCandidate c,
                                                   double selectedAgeSec,
                                                   double measurement,
                                                   ScalarRange range,
                                                   PlantTargetContext context) {
            double base = c.relative ? measurement + c.value : c.value;
            if (!c.periodic) {
                if (range.contains(base))
                    return new CandidateChoice(c, base, selectedAgeSec, false);
                if (unreachablePolicy == UnreachablePolicy.CLAMP_TO_RANGE && range.valid)
                    return new CandidateChoice(c, range.clamp(base), selectedAgeSec, true);
                return null;
            }

            double period = c.usesPlantPeriod ? context.period() : c.period;
            if (!(period > 0.0) || !Double.isFinite(period)) return null;

            if (range.isUnbounded()) {
                double k;
                if (preference == CandidatePreference.PREFER_INCREASING) {
                    k = Math.ceil((measurement - base) / period);
                } else if (preference == CandidatePreference.PREFER_DECREASING) {
                    k = Math.floor((measurement - base) / period);
                } else {
                    k = Math.rint((measurement - base) / period);
                }
                return new CandidateChoice(c, base + k * period, selectedAgeSec, false);
            }

            double minK = Double.isFinite(range.minValue)
                    ? Math.ceil((range.minValue - base) / period)
                    : Math.rint((measurement - base) / period) - 8;
            double maxK = Double.isFinite(range.maxValue)
                    ? Math.floor((range.maxValue - base) / period)
                    : Math.rint((measurement - base) / period) + 8;
            CandidateChoice best = null;
            for (long k = (long) minK; k <= (long) maxK; k++) {
                double v = base + k * period;
                if (!range.contains(v)) continue;
                CandidateChoice choice = new CandidateChoice(c, v, selectedAgeSec, false);
                if (best == null || score(choice, measurement, range) < score(best, measurement, range)) {
                    best = choice;
                }
            }
            if (best != null) return best;
            if (unreachablePolicy == UnreachablePolicy.CLAMP_TO_RANGE && range.valid)
                return new CandidateChoice(c, range.clamp(base), selectedAgeSec, true);
            return null;
        }

        private double score(CandidateChoice choice, double measurement, ScalarRange range) {
            if (preference == CandidatePreference.PREFER_INCREASING) {
                double delta = choice.target - measurement;
                return delta >= 0.0 ? delta : Math.abs(delta) + 1.0e9;
            }
            if (preference == CandidatePreference.PREFER_DECREASING) {
                double delta = measurement - choice.target;
                return delta >= 0.0 ? delta : Math.abs(delta) + 1.0e9;
            }
            if (preference == CandidatePreference.PREFER_RANGE_CENTER) {
                double center = range.center();
                if (Double.isFinite(center)) return Math.abs(choice.target - center);
            }
            return Math.abs(choice.target - measurement);
        }

        @Override
        public void reset() {
            requestSource.reset();
            lastPlan = PlantTargetPlan.unavailable("not sampled");
            lastCycle = Long.MIN_VALUE;
            lastTarget = Double.NaN;
            unavailableActive = false;
            heldMeasuredTarget = Double.NaN;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plantTargetPlanner" : prefix;
            dbg.addData(p + ".class", "PlantTargetPlanner")
                    .addData(p + ".preference", preference)
                    .addData(p + ".unreachablePolicy", unreachablePolicy)
                    .addData(p + ".maxObservationAgeSec", maxObservationAgeSec)
                    .addData(p + ".minQuality", minQuality)
                    .addData(p + ".unavailablePolicy", unavailableKind)
                    .addData(p + ".lastPlan", lastPlan);
            requestSource.debugDump(dbg, p + ".request");
        }
    }

    private static final class CandidateChoice {
        final PlantTargetCandidate candidate;
        final double target;
        final double selectedAgeSec;
        final boolean clamped;

        CandidateChoice(PlantTargetCandidate candidate,
                        double target,
                        double selectedAgeSec,
                        boolean clamped) {
            this.candidate = candidate;
            this.target = target;
            this.selectedAgeSec = selectedAgeSec;
            this.clamped = clamped;
        }
    }

    /** Result of checking one candidate's live observation metadata. */
    private static final class CandidateAcceptance {
        final boolean accepted;
        final double ageSec;
        final String reason;

        private CandidateAcceptance(boolean accepted, double ageSec, String reason) {
            this.accepted = accepted;
            this.ageSec = ageSec;
            this.reason = reason;
        }

        static CandidateAcceptance accepted(double ageSec) {
            return new CandidateAcceptance(true, ageSec, "");
        }

        static CandidateAcceptance rejected(String reason) {
            return new CandidateAcceptance(false, Double.NaN, reason);
        }
    }

    /** Candidate search outcome retaining an actionable first rejection when no candidate wins. */
    private static final class CandidateSearch {
        final CandidateChoice choice;
        final String rejectionReason;

        private CandidateSearch(CandidateChoice choice, String rejectionReason) {
            this.choice = choice;
            this.rejectionReason = rejectionReason;
        }

        static CandidateSearch selected(CandidateChoice choice) {
            return new CandidateSearch(choice, "");
        }

        static CandidateSearch rejected(String reason) {
            return new CandidateSearch(null, reason);
        }
    }

    private static final double FUTURE_OBSERVATION_TOLERANCE_SEC = 1.0e-6;

    private static String cleanName(String name) {
        return name == null || name.trim().isEmpty() ? "layer" : name.trim();
    }

    private static boolean hasSamplingGap(long lastResolvedCycle, long cycle) {
        if (lastResolvedCycle == Long.MIN_VALUE) return false;
        return cycle != lastResolvedCycle && cycle != lastResolvedCycle + 1L;
    }
}
