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
 * setpoints must all be in the same native coordinate used by the downstream {@link Plant}.</p>
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
        this.requestSource = Objects.requireNonNull(requestSource, "requestSource");
        this.measurementSource = Objects.requireNonNull(measurementSource, "measurementSource");
        this.rangeSource = rangeSource != null ? rangeSource : ScalarRange.unboundedSource();
        this.config = config != null ? config.copy() : Config.defaults();
    }

    /**
     * Starts a convenience builder for a scalar setpoint planner.
     */
    public static Builder builder() {
        return new Builder();
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

        if (range.isUnbounded()) {
            double k;
            if (config.candidatePreference == CandidatePreference.PREFER_INCREASING) {
                k = Math.ceil((measurement - base) / c.period);
            } else if (config.candidatePreference == CandidatePreference.PREFER_DECREASING) {
                k = Math.floor((measurement - base) / c.period);
            } else {
                k = Math.rint((measurement - base) / c.period);
            }
            return new CandidateChoice(c, base + k * c.period, true, false);
        }

        double minK = Double.isFinite(range.minValue) ? Math.ceil((range.minValue - base) / c.period) : Math.rint((measurement - base) / c.period) - 8;
        double maxK = Double.isFinite(range.maxValue) ? Math.floor((range.maxValue - base) / c.period) : Math.rint((measurement - base) / c.period) + 8;
        CandidateChoice best = null;
        for (long k = (long) minK; k <= (long) maxK; k++) {
            double v = base + k * c.period;
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
     * Builder for common scalar-setpoint planner setup.
     */
    public static final class Builder {
        private Source<ScalarSetpointRequest> request;
        private ScalarSource measurement;
        private Source<ScalarRange> range = ScalarRange.unboundedSource();
        private Config config = Config.defaults();

        /**
         * Supplies an explicit scalar request source.
         */
        public Builder request(Source<ScalarSetpointRequest> request) {
            this.request = request;
            return this;
        }

        /**
         * Supplies the current measurement in the same native units as the plant target.
         */
        public Builder measurement(ScalarSource measurement) {
            this.measurement = measurement;
            return this;
        }

        /**
         * Supplies a dynamic legal range source.
         */
        public Builder range(Source<ScalarRange> range) {
            this.range = range;
            return this;
        }

        /**
         * Supplies a fixed legal range.
         */
        public Builder range(ScalarRange range) {
            this.range = clock -> range;
            return this;
        }

        /**
         * Sets candidate preference.
         */
        public Builder candidatePreference(CandidatePreference p) {
            this.config = config.withCandidatePreference(p);
            return this;
        }

        /**
         * Sets loss policy.
         */
        public Builder lossPolicy(LossPolicy p) {
            this.config = config.withLossPolicy(p);
            return this;
        }

        /**
         * Sets unreachable target policy.
         */
        public Builder unreachablePolicy(UnreachablePolicy p) {
            this.config = config.withUnreachablePolicy(p);
            return this;
        }

        /**
         * Sets at-setpoint tolerance in native plant units.
         */
        public Builder atSetpointTolerance(double tol) {
            this.config = config.withAtSetpointTolerance(tol);
            return this;
        }

        /**
         * Sets original-request satisfied tolerance in native plant units.
         */
        public Builder requestSatisfiedTolerance(double tol) {
            this.config = config.withRequestSatisfiedTolerance(tol);
            return this;
        }

        /**
         * Sets maximum accepted request age.
         */
        public Builder maxRequestAgeSec(double age) {
            this.config = config.withMaxRequestAgeSec(age);
            return this;
        }

        /**
         * Sets minimum accepted request quality.
         */
        public Builder minRequestQuality(double q) {
            this.config = config.withMinRequestQuality(q);
            return this;
        }

        /**
         * Begins building a scalar request from a spatial query.
         */
        public SpatialRequestBuilder requestFromSpatial() {
            return new SpatialRequestBuilder(this);
        }

        /**
         * Builds the runtime planner.
         */
        public ScalarSetpointPlanner build() {
            if (request == null)
                throw new IllegalStateException("ScalarSetpointPlanner requires request(...)");
            if (measurement == null)
                throw new IllegalStateException("ScalarSetpointPlanner requires measurement(...)");
            return new ScalarSetpointPlanner(request, measurement, range, config);
        }
    }

    /**
     * Maps selected spatial facing into a native-unit scalar request.
     */
    public interface FacingRequestMapper {
        ScalarSetpointRequest map(SpatialFacingSelection facing, LoopClock clock);
    }

    /**
     * Maps selected spatial translation into a native-unit scalar request.
     */
    public interface TranslationRequestMapper {
        ScalarSetpointRequest map(SpatialTranslationSelection translation, LoopClock clock);
    }

    /**
     * Nested builder for the common SpatialQuery -> ScalarSetpointRequest path.
     */
    public static final class SpatialRequestBuilder {
        private final Builder parent;
        private TranslationTarget2d translationTarget;
        private FacingTarget2d facingTarget;
        private SpatialControlFrames controlFrames = SpatialControlFrames.robotCenter();
        private SpatialSolveSet solveSet;
        private TagLayout fixedAprilTagLayout;
        private SpatialSolutionGate gate = SpatialSolutionGate.defaults();
        private FacingRequestMapper facingMapper;
        private TranslationRequestMapper translationMapper;

        SpatialRequestBuilder(Builder parent) {
            this.parent = parent;
        }

        /**
         * Requests a translation relationship.
         */
        public SpatialRequestBuilder translateTo(TranslationTarget2d target) {
            this.translationTarget = target;
            return this;
        }

        /**
         * Requests a facing relationship.
         */
        public SpatialRequestBuilder faceTo(FacingTarget2d target) {
            this.facingTarget = target;
            return this;
        }

        /**
         * Supplies controlled frames.
         */
        public SpatialRequestBuilder controlFrames(SpatialControlFrames frames) {
            this.controlFrames = frames;
            return this;
        }

        /**
         * Supplies solve lanes.
         */
        public SpatialRequestBuilder solveWith(SpatialSolveSet lanes) {
            this.solveSet = lanes;
            return this;
        }

        /**
         * Supplies fixed tag layout for tag-relative field fallback.
         */
        public SpatialRequestBuilder fixedAprilTagLayout(TagLayout layout) {
            this.fixedAprilTagLayout = layout;
            return this;
        }

        /**
         * Supplies selection gate shared with drive guidance.
         */
        public SpatialRequestBuilder selectWith(SpatialSolutionGate gate) {
            this.gate = gate;
            return this;
        }

        /**
         * Maps selected facing result to a native-unit scalar request.
         */
        public SpatialRequestBuilder mapFacingToRequest(FacingRequestMapper mapper) {
            this.facingMapper = mapper;
            return this;
        }

        /**
         * Maps selected translation result to a native-unit scalar request.
         */
        public SpatialRequestBuilder mapTranslationToRequest(TranslationRequestMapper mapper) {
            this.translationMapper = mapper;
            return this;
        }

        /**
         * Finishes the nested request source and returns to the parent planner builder.
         */
        public Builder doneRequest() {
            if (solveSet == null)
                throw new IllegalStateException("requestFromSpatial() requires solveWith(...)");
            final SpatialQuery query = SpatialQuery.builder()
                    .translateTo(translationTarget)
                    .faceTo(facingTarget)
                    .controlFrames(controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(fixedAprilTagLayout)
                    .build();
            if (facingMapper == null && translationMapper == null) {
                throw new IllegalStateException("requestFromSpatial() requires mapFacingToRequest(...) or mapTranslationToRequest(...)");
            }
            parent.request(clock -> {
                SpatialQueryResult result = query.get(clock);
                if (facingMapper != null) {
                    SpatialFacingSelection facing = SpatialQuerySelectors.firstValidFacing(result, gate);
                    if (facing != null) return facingMapper.map(facing, clock);
                }
                if (translationMapper != null) {
                    SpatialTranslationSelection translation = SpatialQuerySelectors.firstValidTranslation(result, gate);
                    if (translation != null) return translationMapper.map(translation, clock);
                }
                return ScalarSetpointRequest.none("no spatial solution passed selection gate");
            });
            return parent;
        }
    }
}
