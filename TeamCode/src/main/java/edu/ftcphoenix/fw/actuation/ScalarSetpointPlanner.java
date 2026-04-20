package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.spatial.FacingTarget2d;
import edu.ftcphoenix.fw.spatial.SpatialControlFrames;
import edu.ftcphoenix.fw.spatial.SpatialFacingSelection;
import edu.ftcphoenix.fw.spatial.SpatialQuery;
import edu.ftcphoenix.fw.spatial.SpatialQueryResult;
import edu.ftcphoenix.fw.spatial.SpatialQuerySelectors;
import edu.ftcphoenix.fw.spatial.SpatialSolutionGate;
import edu.ftcphoenix.fw.spatial.SpatialSolveSet;
import edu.ftcphoenix.fw.spatial.SpatialTranslationSelection;
import edu.ftcphoenix.fw.spatial.TranslationTarget2d;

/**
 * Runtime planner that turns scalar requests into feasible plant-domain setpoints.
 *
 * <p>The planner is unit-agnostic. Requests, measurements, range limits, tolerances, and output
 * setpoints must all be in the same caller-facing coordinate used by the downstream {@link Plant}.
 * For {@link PositionPlant}s created by the guided FTC builder, that means plant units, not
 * hardware-native units unless the builder explicitly chose native units.</p>
 */
public final class ScalarSetpointPlanner implements Source<ScalarSetpointStatus> {

    /**
     * How to choose between feasible candidates.
     */
    public enum CandidatePreference {NEAREST_TO_MEASUREMENT, PREFER_INCREASING, PREFER_DECREASING, PREFER_RANGE_CENTER}

    /**
     * What to do when a request is unavailable.
     */
    public enum LossPolicy {NO_SETPOINT, HOLD_CURRENT, HOLD_LAST_SETPOINT}

    /**
     * What to do when a request exists but no satisfying value lies inside the legal range.
     */
    public enum UnreachablePolicy {NO_SETPOINT, CLAMP_TO_RANGE}

    private final Source<ScalarSetpointRequest> requestSource;
    private final ScalarSource measurementSource;
    private final Source<ScalarRange> rangeSource;
    private final Source<Double> plantPeriodSource;
    private final Config config;

    private long lastCycle = Long.MIN_VALUE;
    private ScalarSetpointStatus lastStatus = null;
    private double lastSetpoint = Double.NaN;

    /**
     * Creates a planner from explicit sources and config.
     */
    public ScalarSetpointPlanner(Source<ScalarSetpointRequest> requestSource,
                                 ScalarSource measurementSource,
                                 Source<ScalarRange> rangeSource,
                                 Config config) {
        this(requestSource, measurementSource, rangeSource, null, config);
    }

    private ScalarSetpointPlanner(Source<ScalarSetpointRequest> requestSource,
                                  ScalarSource measurementSource,
                                  Source<ScalarRange> rangeSource,
                                  Source<Double> plantPeriodSource,
                                  Config config) {
        this.requestSource = Objects.requireNonNull(requestSource, "requestSource");
        this.measurementSource = Objects.requireNonNull(measurementSource, "measurementSource");
        this.rangeSource = rangeSource != null ? rangeSource : ScalarRange.unboundedSource();
        this.plantPeriodSource = plantPeriodSource;
        this.config = config != null ? config.copy() : Config.defaults();
    }

    /**
     * Starts a convenience builder for a scalar setpoint planner.
     */
    public static RequestStage builder() {
        return new RootBuilder();
    }

    /**
     * Samples once per loop and returns the planner status.
     */
    @Override
    public ScalarSetpointStatus get(LoopClock clock) {
        if (clock == null) {
            return lastStatus;
        }
        if (lastStatus != null && lastCycle == clock.cycle()) {
            return lastStatus;
        }
        lastClockForPeriod = clock;
        double measurement = measurementSource.getAsDouble(clock);
        ScalarRange range = rangeSource.get(clock);
        if (range == null || !range.valid) {
            lastStatus = ScalarSetpointStatus.blocked(measurement,
                    range != null ? range.reason : "missing scalar range");
            lastCycle = clock.cycle();
            return lastStatus;
        }

        ScalarSetpointRequest request = requestSource.get(clock);
        if (request == null || !request.hasCandidates()) {
            lastStatus = lossStatus(measurement, request != null ? request.reason() : "missing request");
            lastCycle = clock.cycle();
            return lastStatus;
        }

        CandidateChoice best = chooseBest(request, measurement, range);
        if (best == null) {
            lastStatus = lossStatus(measurement, "no scalar candidate passed gates or range");
        } else {
            lastSetpoint = best.setpoint;
            lastStatus = ScalarSetpointStatus.of(
                    best.setpoint,
                    measurement,
                    best.reachable,
                    best.clamped,
                    best.candidate,
                    config.atSetpointTolerance,
                    config.requestSatisfiedTolerance
            );
        }
        lastCycle = clock.cycle();
        return lastStatus;
    }

    private ScalarSetpointStatus lossStatus(double measurement, String reason) {
        if (config.lossPolicy == LossPolicy.HOLD_CURRENT && Double.isFinite(measurement)) {
            lastSetpoint = measurement;
            return ScalarSetpointStatus.of(measurement, measurement, true, false,
                    ScalarSetpointCandidate.exact("hold-current", measurement),
                    config.atSetpointTolerance, config.requestSatisfiedTolerance);
        }
        if (config.lossPolicy == LossPolicy.HOLD_LAST_SETPOINT && Double.isFinite(lastSetpoint)) {
            return ScalarSetpointStatus.of(lastSetpoint, measurement, true, false,
                    ScalarSetpointCandidate.exact("hold-last-setpoint", lastSetpoint),
                    config.atSetpointTolerance, config.requestSatisfiedTolerance);
        }
        return ScalarSetpointStatus.blocked(measurement, reason);
    }

    private CandidateChoice chooseBest(ScalarSetpointRequest request, double measurement, ScalarRange range) {
        CandidateChoice best = null;
        for (ScalarSetpointCandidate candidate : request.candidates()) {
            if (!candidatePassesGate(candidate)) {
                continue;
            }
            CandidateChoice choice = chooseForCandidate(candidate, measurement, range);
            if (choice != null && (best == null || score(choice, measurement, range) < score(best, measurement, range))) {
                best = choice;
            }
        }
        return best;
    }

    private boolean candidatePassesGate(ScalarSetpointCandidate candidate) {
        return candidate.quality >= config.minRequestQuality
                && (!Double.isFinite(config.maxRequestAgeSec) || candidate.ageSec <= config.maxRequestAgeSec);
    }

    private CandidateChoice chooseForCandidate(ScalarSetpointCandidate c, double measurement, ScalarRange range) {
        double base = c.relative ? measurement + c.value : c.value;
        if (!c.periodic) {
            if (range.contains(base)) {
                return new CandidateChoice(c, base, true, false);
            }
            if (config.unreachablePolicy == UnreachablePolicy.CLAMP_TO_RANGE && range.valid) {
                return new CandidateChoice(c, range.clamp(base), false, true);
            }
            return null;
        }

        double period = c.usesPlantPeriod ? currentPlantPeriod() : c.period;
        if (!(period > 0.0) || !Double.isFinite(period)) {
            return null;
        }

        if (range.isUnbounded()) {
            double k;
            if (config.candidatePreference == CandidatePreference.PREFER_INCREASING) {
                k = Math.ceil((measurement - base) / period);
            } else if (config.candidatePreference == CandidatePreference.PREFER_DECREASING) {
                k = Math.floor((measurement - base) / period);
            } else {
                k = Math.rint((measurement - base) / period);
            }
            return new CandidateChoice(c, base + k * period, true, false);
        }

        double minK = Double.isFinite(range.minValue) ? Math.ceil((range.minValue - base) / period) : Math.rint((measurement - base) / period) - 8;
        double maxK = Double.isFinite(range.maxValue) ? Math.floor((range.maxValue - base) / period) : Math.rint((measurement - base) / period) + 8;
        CandidateChoice best = null;
        for (long k = (long) minK; k <= (long) maxK; k++) {
            double v = base + k * period;
            if (!range.contains(v)) {
                continue;
            }
            CandidateChoice choice = new CandidateChoice(c, v, true, false);
            if (best == null || score(choice, measurement, range) < score(best, measurement, range)) {
                best = choice;
            }
        }
        if (best != null) {
            return best;
        }
        if (config.unreachablePolicy == UnreachablePolicy.CLAMP_TO_RANGE && range.valid) {
            return new CandidateChoice(c, range.clamp(base), false, true);
        }
        return null;
    }

    private double currentPlantPeriod() {
        if (plantPeriodSource == null || lastClockForPeriod == null) {
            return Double.NaN;
        }
        Double period = plantPeriodSource.get(lastClockForPeriod);
        return period != null ? period : Double.NaN;
    }

    private LoopClock lastClockForPeriod;

    private double score(CandidateChoice choice, double measurement, ScalarRange range) {
        if (config.candidatePreference == CandidatePreference.PREFER_INCREASING) {
            double delta = choice.setpoint - measurement;
            return delta >= 0.0 ? delta : Math.abs(delta) + 1.0e9;
        }
        if (config.candidatePreference == CandidatePreference.PREFER_DECREASING) {
            double delta = measurement - choice.setpoint;
            return delta >= 0.0 ? delta : Math.abs(delta) + 1.0e9;
        }
        if (config.candidatePreference == CandidatePreference.PREFER_RANGE_CENTER) {
            double center = range.center();
            if (Double.isFinite(center)) {
                return Math.abs(choice.setpoint - center);
            }
        }
        return Math.abs(choice.setpoint - measurement);
    }

    /**
     * Clears planner cache and child sources.
     */
    @Override
    public void reset() {
        requestSource.reset();
        measurementSource.reset();
        rangeSource.reset();
        if (plantPeriodSource != null) {
            plantPeriodSource.reset();
        }
        lastCycle = Long.MIN_VALUE;
        lastStatus = null;
        lastSetpoint = Double.NaN;
    }

    /**
     * Emits planner status and config for debugging.
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "scalarSetpointPlanner" : prefix;
        dbg.addData(p + ".status", lastStatus)
                .addData(p + ".config", config);
    }

    private static final class CandidateChoice {
        final ScalarSetpointCandidate candidate;
        final double setpoint;
        final boolean reachable;
        final boolean clamped;

        CandidateChoice(ScalarSetpointCandidate candidate, double setpoint, boolean reachable, boolean clamped) {
            this.candidate = candidate;
            this.setpoint = setpoint;
            this.reachable = reachable;
            this.clamped = clamped;
        }
    }

    /**
     * Configuration for scalar request gating, range handling, and status tolerances.
     */
    public static final class Config {
        public CandidatePreference candidatePreference = CandidatePreference.NEAREST_TO_MEASUREMENT;
        public LossPolicy lossPolicy = LossPolicy.NO_SETPOINT;
        public UnreachablePolicy unreachablePolicy = UnreachablePolicy.CLAMP_TO_RANGE;
        public double atSetpointTolerance = 0.0;
        public double requestSatisfiedTolerance = 0.0;
        public double maxRequestAgeSec = Double.POSITIVE_INFINITY;
        public double minRequestQuality = 0.0;

        public static Config defaults() {
            return new Config();
        }

        public Config copy() {
            Config c = new Config();
            c.candidatePreference = candidatePreference;
            c.lossPolicy = lossPolicy;
            c.unreachablePolicy = unreachablePolicy;
            c.atSetpointTolerance = atSetpointTolerance;
            c.requestSatisfiedTolerance = requestSatisfiedTolerance;
            c.maxRequestAgeSec = maxRequestAgeSec;
            c.minRequestQuality = minRequestQuality;
            return c;
        }

        public Config withCandidatePreference(CandidatePreference p) {
            Config c = copy();
            c.candidatePreference = p;
            return c;
        }

        public Config withLossPolicy(LossPolicy p) {
            Config c = copy();
            c.lossPolicy = p;
            return c;
        }

        public Config withUnreachablePolicy(UnreachablePolicy p) {
            Config c = copy();
            c.unreachablePolicy = p;
            return c;
        }

        public Config withAtSetpointTolerance(double tol) {
            Config c = copy();
            c.atSetpointTolerance = tol;
            return c;
        }

        public Config withRequestSatisfiedTolerance(double tol) {
            Config c = copy();
            c.requestSatisfiedTolerance = tol;
            return c;
        }

        public Config withMaxRequestAgeSec(double age) {
            Config c = copy();
            c.maxRequestAgeSec = age;
            return c;
        }

        public Config withMinRequestQuality(double q) {
            Config c = copy();
            c.minRequestQuality = q;
            return c;
        }

        @Override
        public String toString() {
            return "Config{candidatePreference=" + candidatePreference
                    + ", lossPolicy=" + lossPolicy
                    + ", unreachablePolicy=" + unreachablePolicy
                    + ", atSetpointTolerance=" + atSetpointTolerance
                    + ", requestSatisfiedTolerance=" + requestSatisfiedTolerance
                    + ", maxRequestAgeSec=" + maxRequestAgeSec
                    + ", minRequestQuality=" + minRequestQuality + '}';
        }
    }

    /**
     * Maps selected spatial facing into a plant-domain scalar request.
     */
    public interface FacingRequestMapper {
        ScalarSetpointRequest map(SpatialFacingSelection facing, LoopClock clock);
    }

    /**
     * Maps selected spatial translation into a plant-domain scalar request.
     */
    public interface TranslationRequestMapper {
        ScalarSetpointRequest map(SpatialTranslationSelection translation, LoopClock clock);
    }

    /**
     * First builder stage: choose the scalar request source.
     *
     * <p>This required conceptual question is explicit on purpose. Choose either an existing
     * scalar request source or derive one from a spatial query. The next stage asks what scalar
     * domain the planner should command.</p>
     */
    public interface RequestStage {
        /**
         * Supplies an explicit scalar request source.
         */
        DomainStage request(Source<ScalarSetpointRequest> request);

        /**
         * Begins deriving the request from a spatial query.
         */
        SpatialRequestChoice requestFromSpatial();
    }

    /**
     * Second builder stage: choose the scalar domain the planner commands.
     */
    public interface DomainStage {
        /**
         * Uses a {@link PositionPlant} as the measurement/range/period source for this planner.
         *
         * <p>Planner requests remain in the plant's caller-facing position units. If a request uses
         * {@link ScalarSetpointRequest#equivalentPosition(String, double)}, the planner expands
         * equivalent candidates with the plant's declared period. Linear plants do not provide a
         * period, so equivalent-position requests will be blocked rather than silently treated as
         * exact targets.</p>
         */
        OptionalTuningStage forPositionPlant(PositionPlant plant);

        /**
         * Begins explicitly supplying measurement/range/topology instead of reusing a
         * {@link PositionPlant}'s domain.
         */
        ExplicitDomainMeasurementStage explicitDomain();
    }

    /**
     * Explicit-domain stage: provide the current scalar measurement in plant units.
     */
    public interface ExplicitDomainMeasurementStage {
        /**
         * Supplies the current measurement in the same caller-facing units as the planner output.
         */
        ExplicitDomainTopologyStage measurement(ScalarSource measurement);
    }

    /**
     * Explicit-domain stage: choose whether the scalar domain is linear or periodic.
     */
    public interface ExplicitDomainTopologyStage {
        /**
         * Declares a linear scalar domain with no plant-owned period.
         */
        ExplicitDomainRangeStage linear();

        /**
         * Declares a periodic scalar domain with a fixed period in plant units.
         */
        ExplicitDomainRangeStage periodic(double period);

        /**
         * Declares a periodic scalar domain with a dynamic period in plant units.
         */
        ExplicitDomainRangeStage periodic(Source<Double> periodSource);
    }

    /**
     * Explicit-domain stage: answer the legal-range question.
     */
    public interface ExplicitDomainRangeStage {
        /**
         * Supplies a dynamic legal range in plant units.
         */
        OptionalTuningStage range(Source<ScalarRange> range);

        /**
         * Supplies a fixed legal range in plant units.
         */
        OptionalTuningStage range(ScalarRange range);

        /**
         * Declares that the scalar domain is unbounded.
         */
        OptionalTuningStage unboundedRange();
    }

    /**
     * Optional-tuning stage. Build immediately or enter a tuning branch.
     */
    public interface OptionalTuningStage extends BuildStage {
        /**
         * Enters request-selection and loss-policy tuning.
         */
        PolicyBranch policy();

        /**
         * Enters status/completion tuning.
         */
        CompletionBranch completion();
    }

    /**
     * Terminal stage that can build the runtime planner.
     */
    public interface BuildStage {
        /**
         * Builds the runtime planner.
         */
        ScalarSetpointPlanner build();
    }

    /**
     * Optional branch for request gating, candidate choice, and loss behavior.
     */
    public interface PolicyBranch {
        /**
         * Sets candidate preference.
         */
        PolicyBranch candidatePreference(CandidatePreference p);

        /**
         * Sets loss policy.
         */
        PolicyBranch lossPolicy(LossPolicy p);

        /**
         * Sets unreachable target policy.
         */
        PolicyBranch unreachablePolicy(UnreachablePolicy p);

        /**
         * Sets maximum accepted request age.
         */
        PolicyBranch maxRequestAgeSec(double age);

        /**
         * Sets minimum accepted request quality.
         */
        PolicyBranch minRequestQuality(double q);

        /**
         * Returns to the main builder after policy tuning.
         */
        OptionalTuningStage donePolicy();
    }

    /**
     * Optional branch for plant-unit status and completion tolerances.
     */
    public interface CompletionBranch {
        /**
         * Sets at-setpoint tolerance in plant units.
         */
        CompletionBranch atSetpointTolerance(double tol);

        /**
         * Sets original-request satisfied tolerance in plant units.
         */
        CompletionBranch requestSatisfiedTolerance(double tol);

        /**
         * Returns to the main builder after completion tuning.
         */
        OptionalTuningStage doneCompletion();
    }

    /**
     * First nested spatial-request stage: choose whether spatial output should come from facing or
     * translation.
     */
    public interface SpatialRequestChoice {
        /**
         * Derives a scalar request from a spatial facing solution.
         */
        SpatialFacingRequestBuilder faceTo(FacingTarget2d target);

        /**
         * Derives a scalar request from a spatial translation solution.
         */
        SpatialTranslationRequestBuilder translateTo(TranslationTarget2d target);
    }

    /**
     * Shared options available after the spatial mapper has been chosen.
     */
    public interface SpatialMappedRequestBuilder {
        /**
         * Supplies controlled frames.
         */
        SpatialMappedRequestBuilder controlFrames(SpatialControlFrames frames);

        /**
         * Supplies solve lanes.
         */
        SpatialMappedRequestBuilder solveWith(SpatialSolveSet lanes);

        /**
         * Supplies fixed tag layout for tag-relative field fallback.
         */
        SpatialMappedRequestBuilder fixedAprilTagLayout(TagLayout layout);

        /**
         * Supplies selection gate shared with drive guidance.
         */
        SpatialMappedRequestBuilder selectWith(SpatialSolutionGate gate);

        /**
         * Finishes the nested request source and returns to the parent planner builder.
         */
        DomainStage doneRequest();
    }

    /**
     * Nested builder for a facing-derived spatial request.
     */
    public interface SpatialFacingRequestBuilder {
        /**
         * Supplies controlled frames.
         */
        SpatialFacingRequestBuilder controlFrames(SpatialControlFrames frames);

        /**
         * Supplies solve lanes.
         */
        SpatialFacingRequestBuilder solveWith(SpatialSolveSet lanes);

        /**
         * Supplies fixed tag layout for tag-relative field fallback.
         */
        SpatialFacingRequestBuilder fixedAprilTagLayout(TagLayout layout);

        /**
         * Supplies selection gate shared with drive guidance.
         */
        SpatialFacingRequestBuilder selectWith(SpatialSolutionGate gate);

        /**
         * Maps the selected spatial facing result into a scalar request in plant units.
         */
        SpatialMappedRequestBuilder mapToRequest(FacingRequestMapper mapper);
    }

    /**
     * Nested builder for a translation-derived spatial request.
     */
    public interface SpatialTranslationRequestBuilder {
        /**
         * Supplies controlled frames.
         */
        SpatialTranslationRequestBuilder controlFrames(SpatialControlFrames frames);

        /**
         * Supplies solve lanes.
         */
        SpatialTranslationRequestBuilder solveWith(SpatialSolveSet lanes);

        /**
         * Supplies fixed tag layout for tag-relative field fallback.
         */
        SpatialTranslationRequestBuilder fixedAprilTagLayout(TagLayout layout);

        /**
         * Supplies selection gate shared with drive guidance.
         */
        SpatialTranslationRequestBuilder selectWith(SpatialSolutionGate gate);

        /**
         * Maps the selected spatial translation result into a scalar request in plant units.
         */
        SpatialMappedRequestBuilder mapToRequest(TranslationRequestMapper mapper);
    }

    private static final class RootBuilder implements RequestStage, DomainStage, OptionalTuningStage, PolicyBranch, CompletionBranch {
        private Source<ScalarSetpointRequest> request;
        private ScalarSource measurement;
        private Source<ScalarRange> range = ScalarRange.unboundedSource();
        private Source<Double> plantPeriod;
        private Config config = Config.defaults();

        @Override
        public DomainStage request(Source<ScalarSetpointRequest> request) {
            this.request = request;
            return this;
        }

        @Override
        public SpatialRequestChoice requestFromSpatial() {
            return new SpatialChoiceBuilder(this);
        }

        @Override
        public OptionalTuningStage forPositionPlant(PositionPlant plant) {
            Objects.requireNonNull(plant, "plant");
            this.measurement = plant.positionSource();
            this.range = plant.targetRangeSource();
            this.plantPeriod = clock -> plant.topology() == PositionPlant.Topology.PERIODIC
                    ? plant.period()
                    : Double.NaN;
            return this;
        }

        @Override
        public ExplicitDomainMeasurementStage explicitDomain() {
            return new ExplicitDomainBuilder(this);
        }

        @Override
        public PolicyBranch policy() {
            return this;
        }

        @Override
        public CompletionBranch completion() {
            return this;
        }

        @Override
        public PolicyBranch candidatePreference(CandidatePreference p) {
            this.config = config.withCandidatePreference(p);
            return this;
        }

        @Override
        public PolicyBranch lossPolicy(LossPolicy p) {
            this.config = config.withLossPolicy(p);
            return this;
        }

        @Override
        public PolicyBranch unreachablePolicy(UnreachablePolicy p) {
            this.config = config.withUnreachablePolicy(p);
            return this;
        }

        @Override
        public PolicyBranch maxRequestAgeSec(double age) {
            this.config = config.withMaxRequestAgeSec(age);
            return this;
        }

        @Override
        public PolicyBranch minRequestQuality(double q) {
            this.config = config.withMinRequestQuality(q);
            return this;
        }

        @Override
        public OptionalTuningStage donePolicy() {
            return this;
        }

        @Override
        public CompletionBranch atSetpointTolerance(double tol) {
            this.config = config.withAtSetpointTolerance(tol);
            return this;
        }

        @Override
        public CompletionBranch requestSatisfiedTolerance(double tol) {
            this.config = config.withRequestSatisfiedTolerance(tol);
            return this;
        }

        @Override
        public OptionalTuningStage doneCompletion() {
            return this;
        }

        @Override
        public ScalarSetpointPlanner build() {
            if (request == null)
                throw new IllegalStateException("ScalarSetpointPlanner requires request(...) or requestFromSpatial()...");
            if (measurement == null)
                throw new IllegalStateException("ScalarSetpointPlanner requires forPositionPlant(...) or explicitDomain()...");
            return new ScalarSetpointPlanner(request, measurement, range, plantPeriod, config);
        }
    }

    private static final class ExplicitDomainBuilder implements ExplicitDomainMeasurementStage, ExplicitDomainTopologyStage, ExplicitDomainRangeStage {
        private final RootBuilder parent;

        ExplicitDomainBuilder(RootBuilder parent) {
            this.parent = parent;
        }

        @Override
        public ExplicitDomainTopologyStage measurement(ScalarSource measurement) {
            parent.measurement = Objects.requireNonNull(measurement, "measurement");
            return this;
        }

        @Override
        public ExplicitDomainRangeStage linear() {
            parent.plantPeriod = null;
            return this;
        }

        @Override
        public ExplicitDomainRangeStage periodic(double period) {
            if (!(period > 0.0) || !Double.isFinite(period)) {
                throw new IllegalArgumentException("period must be finite and > 0");
            }
            parent.plantPeriod = clock -> period;
            return this;
        }

        @Override
        public ExplicitDomainRangeStage periodic(Source<Double> periodSource) {
            parent.plantPeriod = Objects.requireNonNull(periodSource, "periodSource");
            return this;
        }

        @Override
        public OptionalTuningStage range(Source<ScalarRange> range) {
            parent.range = Objects.requireNonNull(range, "range");
            return parent;
        }

        @Override
        public OptionalTuningStage range(ScalarRange range) {
            parent.range = clock -> range;
            return parent;
        }

        @Override
        public OptionalTuningStage unboundedRange() {
            parent.range = ScalarRange.unboundedSource();
            return parent;
        }
    }

    private static final class SpatialChoiceBuilder implements SpatialRequestChoice {
        private final RootBuilder parent;

        SpatialChoiceBuilder(RootBuilder parent) {
            this.parent = parent;
        }

        @Override
        public SpatialFacingRequestBuilder faceTo(FacingTarget2d target) {
            return new FacingSpatialRequestBuilder(parent, Objects.requireNonNull(target, "target"));
        }

        @Override
        public SpatialTranslationRequestBuilder translateTo(TranslationTarget2d target) {
            return new TranslationSpatialRequestBuilder(parent, Objects.requireNonNull(target, "target"));
        }
    }

    private static abstract class SpatialRequestState {
        final RootBuilder parent;
        SpatialControlFrames controlFrames = SpatialControlFrames.robotCenter();
        SpatialSolveSet solveSet;
        TagLayout fixedAprilTagLayout;
        SpatialSolutionGate gate = SpatialSolutionGate.defaults();

        SpatialRequestState(RootBuilder parent) {
            this.parent = parent;
        }

        void validateBase(String requestLabel) {
            if (solveSet == null) {
                throw new IllegalStateException(requestLabel + " requires solveWith(...)");
            }
        }
    }

    private static final class FacingSpatialRequestBuilder extends SpatialRequestState implements SpatialFacingRequestBuilder, SpatialMappedRequestBuilder {
        private final FacingTarget2d facingTarget;
        private FacingRequestMapper mapper;

        FacingSpatialRequestBuilder(RootBuilder parent, FacingTarget2d facingTarget) {
            super(parent);
            this.facingTarget = facingTarget;
        }

        @Override
        public FacingSpatialRequestBuilder controlFrames(SpatialControlFrames frames) {
            this.controlFrames = frames;
            return this;
        }

        @Override
        public FacingSpatialRequestBuilder solveWith(SpatialSolveSet lanes) {
            this.solveSet = lanes;
            return this;
        }

        @Override
        public FacingSpatialRequestBuilder fixedAprilTagLayout(TagLayout layout) {
            this.fixedAprilTagLayout = layout;
            return this;
        }

        @Override
        public FacingSpatialRequestBuilder selectWith(SpatialSolutionGate gate) {
            this.gate = gate;
            return this;
        }

        @Override
        public FacingSpatialRequestBuilder mapToRequest(FacingRequestMapper mapper) {
            this.mapper = Objects.requireNonNull(mapper, "mapper");
            return this;
        }

        @Override
        public DomainStage doneRequest() {
            validateBase("requestFromSpatial().faceTo(...)");
            if (mapper == null) {
                throw new IllegalStateException("requestFromSpatial().faceTo(...) requires mapToRequest(...)");
            }
            final SpatialQuery query = SpatialQuery.builder()
                    .faceTo(facingTarget)
                    .controlFrames(controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(fixedAprilTagLayout)
                    .build();
            parent.request(clock -> {
                SpatialQueryResult result = query.get(clock);
                SpatialFacingSelection facing = SpatialQuerySelectors.firstValidFacing(result, gate);
                return facing != null
                        ? mapper.map(facing, clock)
                        : ScalarSetpointRequest.none("no spatial facing solution passed selection gate");
            });
            return parent;
        }
    }

    private static final class TranslationSpatialRequestBuilder extends SpatialRequestState implements SpatialTranslationRequestBuilder, SpatialMappedRequestBuilder {
        private final TranslationTarget2d translationTarget;
        private TranslationRequestMapper mapper;

        TranslationSpatialRequestBuilder(RootBuilder parent, TranslationTarget2d translationTarget) {
            super(parent);
            this.translationTarget = translationTarget;
        }

        @Override
        public TranslationSpatialRequestBuilder controlFrames(SpatialControlFrames frames) {
            this.controlFrames = frames;
            return this;
        }

        @Override
        public TranslationSpatialRequestBuilder solveWith(SpatialSolveSet lanes) {
            this.solveSet = lanes;
            return this;
        }

        @Override
        public TranslationSpatialRequestBuilder fixedAprilTagLayout(TagLayout layout) {
            this.fixedAprilTagLayout = layout;
            return this;
        }

        @Override
        public TranslationSpatialRequestBuilder selectWith(SpatialSolutionGate gate) {
            this.gate = gate;
            return this;
        }

        @Override
        public TranslationSpatialRequestBuilder mapToRequest(TranslationRequestMapper mapper) {
            this.mapper = Objects.requireNonNull(mapper, "mapper");
            return this;
        }

        @Override
        public DomainStage doneRequest() {
            validateBase("requestFromSpatial().translateTo(...)");
            if (mapper == null) {
                throw new IllegalStateException("requestFromSpatial().translateTo(...) requires mapToRequest(...)");
            }
            final SpatialQuery query = SpatialQuery.builder()
                    .translateTo(translationTarget)
                    .controlFrames(controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(fixedAprilTagLayout)
                    .build();
            parent.request(clock -> {
                SpatialQueryResult result = query.get(clock);
                SpatialTranslationSelection translation = SpatialQuerySelectors.firstValidTranslation(result, gate);
                return translation != null
                        ? mapper.map(translation, clock)
                        : ScalarSetpointRequest.none("no spatial translation solution passed selection gate");
            });
            return parent;
        }
    }
}
