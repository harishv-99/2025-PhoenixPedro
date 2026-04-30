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
 * {@link #exact(ScalarSource)}. Behavior arbitration uses {@link #overlay(PlantTargetSource)}.
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
 *     .select().nearestToMeasurement().doneSelect()
 *     .unreachable().reject().doneUnreachable()
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
     * Latch the current measurement the first time this source is entered, then keep returning it.
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

        HoldMeasuredTargetSource(double fallback) {
            if (!Double.isFinite(fallback))
                throw new IllegalArgumentException("fallbackIfNoMeasurement must be finite");
            this.fallback = fallback;
            this.latchedTarget = fallback;
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            if (!latched) {
                latchedTarget = context.feedbackAvailable() ? context.measurement() : fallback;
                latched = true;
            }
            return PlantTargetPlan.holdMeasured(latchedTarget, "holding measured target captured on entry");
        }

        @Override
        public void reset() {
            latched = false;
            latchedTarget = fallback;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "holdMeasuredPlantTarget" : prefix;
            dbg.addData(p + ".class", "HoldMeasuredTargetSource")
                    .addData(p + ".fallback", fallback)
                    .addData(p + ".latched", latched)
                    .addData(p + ".latchedTarget", latchedTarget);
        }
    }

    /**
     * Builder for base-plus-priority plant target overlays.
     *
     * <p>The base target should be total. Layers are evaluated in insertion order; each enabled
     * layer that produces a target replaces the current winner, so later enabled layers have higher
     * priority. If an enabled layer is unavailable, the overlay returns an unavailable plan for that
     * layer instead of silently falling through. Give smart layer sources their own
     * {@code whenUnavailable()} policy when an active behavior should hold, fallback, or explicitly
     * report unavailability.</p>
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
         * Add an enabled layer with a plant-aware target source.
         */
        public OverlayBuilder add(String name, BooleanSource enabled, PlantTargetSource target) {
            layers.add(new Layer(cleanName(name), Objects.requireNonNull(enabled, "enabled"), Objects.requireNonNull(target, "target")));
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
        boolean lastEnabled;
        PlantTargetPlan lastPlan = PlantTargetPlan.unavailable("not sampled");

        Layer(String name, BooleanSource enabled, PlantTargetSource target) {
            this.name = name;
            this.enabled = enabled.memoized();
            this.target = target;
        }
    }

    private static final class OverlayTargetSource implements PlantTargetSource {
        private final PlantTargetSource base;
        private final Layer[] layers;
        private PlantTargetPlan lastPlan = PlantTargetPlan.unavailable("not sampled");
        private String lastWinner = "base";

        OverlayTargetSource(PlantTargetSource base, Layer[] layers) {
            this.base = base;
            this.layers = layers;
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            PlantTargetPlan winner = base.resolve(context, clock);
            lastWinner = "base";
            for (Layer layer : layers) {
                boolean enabled = layer.enabled.getAsBoolean(clock);
                layer.lastEnabled = enabled;
                if (!enabled) {
                    layer.lastPlan = PlantTargetPlan.unavailable("layer disabled");
                    continue;
                }
                PlantTargetPlan plan = layer.target.resolve(context, clock);
                layer.lastPlan = plan;
                lastWinner = layer.name;
                if (!plan.hasTarget()) {
                    winner = PlantTargetPlan.unavailable("enabled plant target layer '" + layer.name + "' produced no target: " + plan.reason());
                    break;
                }
                winner = plan;
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
                layer.lastEnabled = false;
                layer.lastPlan = PlantTargetPlan.unavailable("not sampled");
            }
            lastPlan = PlantTargetPlan.unavailable("not sampled");
            lastWinner = "base";
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plantTargetOverlay" : prefix;
            dbg.addData(p + ".class", "PlantTargetOverlay")
                    .addData(p + ".winner", lastWinner)
                    .addData(p + ".plan", lastPlan)
                    .addData(p + ".layers", layers.length);
            base.debugDump(dbg, p + ".base");
            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];
                String lp = p + ".layer" + i;
                dbg.addData(lp + ".name", layer.name)
                        .addData(lp + ".enabled", layer.lastEnabled)
                        .addData(lp + ".plan", layer.lastPlan);
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
         */
        PlanOptionalStage request(Source<PlantTargetRequest> request);
    }

    /**
     * Main planner builder stage after the request source has been supplied.
     */
    public interface PlanOptionalStage {
        /**
         * Enter candidate-selection tuning.
         */
        PlanSelectBranch select();

        /**
         * Enter request-acceptance tuning.
         */
        PlanAcceptBranch accept();

        /**
         * Enter unreachable-candidate policy tuning.
         */
        PlanUnreachableBranch unreachable();

        /**
         * Enter the required unavailable-target policy branch.
         */
        PlanUnavailableBranch whenUnavailable();
    }

    /**
     * Candidate-selection branch.
     */
    public interface PlanSelectBranch {
        /**
         * Choose the reachable candidate nearest to the current measurement.
         */
        PlanSelectBranch nearestToMeasurement();

        /**
         * Prefer increasing moves when possible.
         */
        PlanSelectBranch preferIncreasing();

        /**
         * Prefer decreasing moves when possible.
         */
        PlanSelectBranch preferDecreasing();

        /**
         * Prefer candidates closest to the legal range center.
         */
        PlanSelectBranch preferRangeCenter();

        /**
         * Return to the main planner builder.
         */
        PlanOptionalStage doneSelect();
    }

    /**
     * Request age/quality acceptance branch.
     */
    public interface PlanAcceptBranch {
        /**
         * Ignore candidates older than {@code ageSec}.
         */
        PlanAcceptBranch maxRequestAgeSec(double ageSec);

        /**
         * Ignore candidates below the supplied quality threshold.
         */
        PlanAcceptBranch minQuality(double quality);

        /**
         * Return to the main planner builder.
         */
        PlanOptionalStage doneAccept();
    }

    /**
     * Unreachable-candidate branch.
     */
    public interface PlanUnreachableBranch {
        /**
         * Reject unreachable candidates and let unavailable policy handle the layer.
         */
        PlanUnreachableBranch reject();

        /**
         * Clamp unreachable candidates into the legal range.
         */
        PlanUnreachableBranch clampToRange();

        /**
         * Return to the main planner builder.
         */
        PlanOptionalStage doneUnreachable();
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
         * Latch current measurement on entry; use fallback when no measurement is available.
         */
        PlantTargetSource holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement);

        /**
         * Explicitly report unavailability when no target can be selected.
         */
        PlantTargetSource reportUnavailable();
    }

    private enum UnavailableKind {FALLBACK, HOLD_LAST, HOLD_MEASURED, REJECT}

    private static final class PlannerBuilder implements PlanRequestStage, PlanOptionalStage,
            PlanSelectBranch, PlanAcceptBranch, PlanUnreachableBranch, PlanUnavailableBranch {
        private Source<PlantTargetRequest> request;
        private CandidatePreference preference = CandidatePreference.NEAREST_TO_MEASUREMENT;
        private UnreachablePolicy unreachablePolicy = UnreachablePolicy.REJECT;
        private double maxRequestAgeSec = Double.POSITIVE_INFINITY;
        private double minQuality = 0.0;

        @Override
        public PlanOptionalStage request(Source<PlantTargetRequest> request) {
            this.request = Objects.requireNonNull(request, "request");
            return this;
        }

        @Override
        public PlanSelectBranch select() {
            return this;
        }

        @Override
        public PlanAcceptBranch accept() {
            return this;
        }

        @Override
        public PlanUnreachableBranch unreachable() {
            return this;
        }

        @Override
        public PlanUnavailableBranch whenUnavailable() {
            return this;
        }

        @Override
        public PlanSelectBranch nearestToMeasurement() {
            preference = CandidatePreference.NEAREST_TO_MEASUREMENT;
            return this;
        }

        @Override
        public PlanSelectBranch preferIncreasing() {
            preference = CandidatePreference.PREFER_INCREASING;
            return this;
        }

        @Override
        public PlanSelectBranch preferDecreasing() {
            preference = CandidatePreference.PREFER_DECREASING;
            return this;
        }

        @Override
        public PlanSelectBranch preferRangeCenter() {
            preference = CandidatePreference.PREFER_RANGE_CENTER;
            return this;
        }

        @Override
        public PlanOptionalStage doneSelect() {
            return this;
        }

        @Override
        public PlanAcceptBranch maxRequestAgeSec(double ageSec) {
            if (ageSec < 0.0 || !Double.isFinite(ageSec))
                throw new IllegalArgumentException("maxRequestAgeSec must be finite and >= 0");
            maxRequestAgeSec = ageSec;
            return this;
        }

        @Override
        public PlanAcceptBranch minQuality(double quality) {
            minQuality = quality;
            return this;
        }

        @Override
        public PlanOptionalStage doneAccept() {
            return this;
        }

        @Override
        public PlanUnreachableBranch reject() {
            unreachablePolicy = UnreachablePolicy.REJECT;
            return this;
        }

        @Override
        public PlanUnreachableBranch clampToRange() {
            unreachablePolicy = UnreachablePolicy.CLAMP_TO_RANGE;
            return this;
        }

        @Override
        public PlanOptionalStage doneUnreachable() {
            return this;
        }

        @Override
        public PlantTargetSource fallbackTo(double target) {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxRequestAgeSec, minQuality, UnavailableKind.FALLBACK, target);
        }

        @Override
        public PlantTargetSource holdLastTarget(double initialTarget) {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxRequestAgeSec, minQuality, UnavailableKind.HOLD_LAST, initialTarget);
        }

        @Override
        public PlantTargetSource holdMeasuredTargetOnEntry(double fallbackIfNoMeasurement) {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxRequestAgeSec, minQuality, UnavailableKind.HOLD_MEASURED, fallbackIfNoMeasurement);
        }

        @Override
        public PlantTargetSource reportUnavailable() {
            validateRequest();
            return new PlannerTargetSource(request, preference, unreachablePolicy,
                    maxRequestAgeSec, minQuality, UnavailableKind.REJECT, Double.NaN);
        }

        private void validateRequest() {
            if (request == null)
                throw new IllegalStateException("PlantTargets.plan() requires request(...) before whenUnavailable()");
        }
    }

    private static final class PlannerTargetSource implements PlantTargetSource {
        private final Source<PlantTargetRequest> requestSource;
        private final CandidatePreference preference;
        private final UnreachablePolicy unreachablePolicy;
        private final double maxRequestAgeSec;
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
                            double maxRequestAgeSec,
                            double minQuality,
                            UnavailableKind unavailableKind,
                            double unavailableValue) {
            this.requestSource = Objects.requireNonNull(requestSource, "requestSource");
            this.preference = Objects.requireNonNull(preference, "preference");
            this.unreachablePolicy = Objects.requireNonNull(unreachablePolicy, "unreachablePolicy");
            this.maxRequestAgeSec = maxRequestAgeSec;
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
            if (clock != null && clock.cycle() == lastCycle) return lastPlan;
            if (clock != null) lastCycle = clock.cycle();

            PlantTargetRequest request = requestSource.get(clock);
            if (request == null || !request.hasCandidates()) {
                lastPlan = unavailable(context, request != null ? request.reason() : "missing plant target request");
                return lastPlan;
            }

            CandidateChoice best = chooseBest(request, context);
            if (best == null) {
                lastPlan = unavailable(context, "no plant target candidate passed gates or range");
                return lastPlan;
            }

            unavailableActive = false;
            heldMeasuredTarget = Double.NaN;
            lastTarget = best.target;
            lastPlan = PlantTargetPlan.planned(best.target, best.candidate, best.clamped,
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

        private CandidateChoice chooseBest(PlantTargetRequest request, PlantTargetContext context) {
            ScalarRange range = context.targetRange();
            if (range == null || !range.valid) return null;
            double measurement = context.feedbackAvailable() ? context.measurement() : context.previousAppliedTarget();
            if (!Double.isFinite(measurement)) measurement = 0.0;

            CandidateChoice best = null;
            for (PlantTargetCandidate candidate : request.candidates()) {
                if (!candidatePassesGate(candidate)) continue;
                CandidateChoice choice = chooseForCandidate(candidate, measurement, range, context);
                if (choice != null && (best == null || score(choice, measurement, range) < score(best, measurement, range))) {
                    best = choice;
                }
            }
            return best;
        }

        private boolean candidatePassesGate(PlantTargetCandidate candidate) {
            return candidate.quality >= minQuality
                    && (!Double.isFinite(maxRequestAgeSec) || candidate.ageSec <= maxRequestAgeSec);
        }

        private CandidateChoice chooseForCandidate(PlantTargetCandidate c,
                                                   double measurement,
                                                   ScalarRange range,
                                                   PlantTargetContext context) {
            double base = c.relative ? measurement + c.value : c.value;
            if (!c.periodic) {
                if (range.contains(base)) return new CandidateChoice(c, base, false);
                if (unreachablePolicy == UnreachablePolicy.CLAMP_TO_RANGE && range.valid)
                    return new CandidateChoice(c, range.clamp(base), true);
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
                return new CandidateChoice(c, base + k * period, false);
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
                CandidateChoice choice = new CandidateChoice(c, v, false);
                if (best == null || score(choice, measurement, range) < score(best, measurement, range)) {
                    best = choice;
                }
            }
            if (best != null) return best;
            if (unreachablePolicy == UnreachablePolicy.CLAMP_TO_RANGE && range.valid)
                return new CandidateChoice(c, range.clamp(base), true);
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
                    .addData(p + ".maxRequestAgeSec", maxRequestAgeSec)
                    .addData(p + ".minQuality", minQuality)
                    .addData(p + ".unavailablePolicy", unavailableKind)
                    .addData(p + ".lastPlan", lastPlan);
            requestSource.debugDump(dbg, p + ".request");
        }
    }

    private static final class CandidateChoice {
        final PlantTargetCandidate candidate;
        final double target;
        final boolean clamped;

        CandidateChoice(PlantTargetCandidate candidate, double target, boolean clamped) {
            this.candidate = candidate;
            this.target = target;
            this.clamped = clamped;
        }
    }

    private static String cleanName(String name) {
        return name == null || name.trim().isEmpty() ? "layer" : name.trim();
    }
}
