package edu.ftcsushi.fw.localization;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Optional bounded history of one authoritative planar pose trajectory.
 *
 * <p>The history borrows one {@link PoseTrajectoryEstimator}. It never updates or resets that
 * estimator. The existing localization lifecycle owner calls {@link #recordCurrent(LoopClock)}
 * immediately after the estimator's final update, then gives consumers the stable read-only
 * {@link #lookupSource()} projection.</p>
 *
 * <p>Samples preserve the estimator's as-published trajectory. Historical entries are never
 * rewritten after a later correction. A bracket is interpolated only when its time, translation,
 * yaw, and trajectory-continuity boundaries satisfy the configured limits. Lookup never
 * extrapolates, clamps, chooses a nearest sample, or substitutes the estimator's current pose.</p>
 *
 * <p>This owner is planar by design: it retains field x/y and yaw, in inches and radians. It does
 * not claim to interpolate z, pitch, or roll.</p>
 */
public final class PlanarPoseHistory {

    /** Mutable, data-only authoring configuration. One owner retains a validated copy. */
    public static final class Config {

        /**
         * Maximum sample age eligible for lookup and retained after a successful record heartbeat.
         * Zero permits only samples at the current time.
         */
        public double retentionSec = 0.50;

        /** Hard retained-sample bound, inclusive of exact endpoints. */
        public int maxSamples = 128;

        /** Maximum time spanned by one interpolated bracket, in seconds. */
        public double maxInterpolationGapSec = 0.10;

        /** Maximum planar translation spanned by one interpolated bracket, in inches. */
        public double maxInterpolationTranslationInches = 12.0;

        /** Maximum shortest-path yaw spanned by one interpolated bracket, in radians. */
        public double maxInterpolationYawRad = Math.PI / 2.0;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /** Returns a fresh mutable draft with Sushi defaults. */
        public static Config defaults() {
            return new Config();
        }

        /** Returns an independently mutable copy of this draft. */
        public Config copy() {
            Config copy = new Config();
            copy.retentionSec = retentionSec;
            copy.maxSamples = maxSamples;
            copy.maxInterpolationGapSec = maxInterpolationGapSec;
            copy.maxInterpolationTranslationInches = maxInterpolationTranslationInches;
            copy.maxInterpolationYawRad = maxInterpolationYawRad;
            return copy;
        }

        /**
         * Returns a validated, independently mutable snapshot for one history owner.
         *
         * @param context diagnostic field prefix; null or blank uses the canonical Config name
         * @return validated defensive copy
         */
        public Config validatedCopy(String context) {
            Config copy = copy();
            copy.validate(context);
            return copy;
        }

        private void validate(String context) {
            String owner = context == null || context.trim().isEmpty()
                    ? Config.class.getCanonicalName()
                    : context.trim();
            requireFiniteNonNegative(owner, "retentionSec", retentionSec);
            if (maxSamples < 2 || maxSamples > 4096) {
                throw new IllegalArgumentException(
                        owner + ".maxSamples must be within [2, 4096], got " + maxSamples
                );
            }
            requireFiniteNonNegative(
                    owner,
                    "maxInterpolationGapSec",
                    maxInterpolationGapSec
            );
            requireFiniteNonNegative(
                    owner,
                    "maxInterpolationTranslationInches",
                    maxInterpolationTranslationInches
            );
            if (!Double.isFinite(maxInterpolationYawRad)
                    || maxInterpolationYawRad < 0.0
                    || maxInterpolationYawRad > Math.PI) {
                throw new IllegalArgumentException(
                        owner + ".maxInterpolationYawRad must be finite and within [0, pi], got "
                                + maxInterpolationYawRad
                );
            }
        }

        private static void requireFiniteNonNegative(String owner,
                                                     String field,
                                                     double value) {
            if (!Double.isFinite(value) || value < 0.0) {
                throw new IllegalArgumentException(
                        owner + "." + field + " must be finite and >= 0, got " + value
                );
            }
        }
    }

    /** Immutable result of one exact-time pose-history query. */
    public static final class Lookup {

        /** How an available pose was resolved, or whether no truthful answer exists. */
        public enum Kind {
            EXACT,
            INTERPOLATED,
            UNAVAILABLE
        }

        /** Typed reason for an unavailable lookup. */
        public enum UnavailableReason {
            EMPTY,
            QUERY_TIMESTAMP_UNAVAILABLE,
            QUERY_TIMESTAMP_NOT_CURRENT,
            BEFORE_FIRST,
            EVICTED,
            DISCONTINUITY,
            INTERPOLATION_TIME_GAP,
            INTERPOLATION_TRANSLATION_GAP,
            INTERPOLATION_YAW_GAP,
            AFTER_LATEST
        }

        private final Kind kind;
        private final Pose2d fieldToRobotPose;
        private final double quality;
        private final LoopTimestamp timestamp;
        private final UnavailableReason unavailableReason;

        private Lookup(Kind kind,
                       Pose2d fieldToRobotPose,
                       double quality,
                       LoopTimestamp timestamp,
                       UnavailableReason unavailableReason) {
            this.kind = Objects.requireNonNull(kind, "kind");
            this.fieldToRobotPose = fieldToRobotPose;
            this.quality = quality;
            this.timestamp = timestamp;
            this.unavailableReason = unavailableReason;
        }

        private static Lookup exact(Sample sample, LoopTimestamp requestedTimestamp) {
            return new Lookup(
                    Kind.EXACT,
                    sample.fieldToRobotPose,
                    sample.quality,
                    requestedTimestamp,
                    null
            );
        }

        private static Lookup interpolated(Pose2d pose,
                                           double quality,
                                           LoopTimestamp timestamp) {
            return new Lookup(Kind.INTERPOLATED, pose, quality, timestamp, null);
        }

        private static Lookup unavailable(UnavailableReason reason,
                                          LoopTimestamp requestedTimestamp) {
            return new Lookup(
                    Kind.UNAVAILABLE,
                    null,
                    Double.NaN,
                    Objects.requireNonNull(requestedTimestamp, "requestedTimestamp"),
                    Objects.requireNonNull(reason, "reason")
            );
        }

        /** Returns whether this result is exact, interpolated, or unavailable. */
        public Kind kind() {
            return kind;
        }

        /** Returns whether a truthful exact or interpolated pose is available. */
        public boolean isAvailable() {
            return kind != Kind.UNAVAILABLE;
        }

        /**
         * Returns the available planar field-to-robot pose.
         *
         * @throws IllegalStateException when this lookup is unavailable
         */
        public Pose2d fieldToRobotPose() {
            requireAvailable("fieldToRobotPose");
            return fieldToRobotPose;
        }

        /**
         * Returns the exact sample quality or the minimum quality of interpolated endpoints.
         *
         * @throws IllegalStateException when this lookup is unavailable
         */
        public double quality() {
            requireAvailable("quality");
            return quality;
        }

        /**
         * Returns the requested timestamp for every available or unavailable result.
         *
         * <p>For an exact result this is equal in time to the retained sample timestamp. Keeping
         * the request on unavailable results lets diagnostics correlate a typed failure with the
         * delayed observation that asked for it.</p>
         */
        public LoopTimestamp timestamp() {
            return timestamp;
        }

        /**
         * Returns the typed reason why no pose is available.
         *
         * @throws IllegalStateException when this lookup is available
         */
        public UnavailableReason unavailableReason() {
            if (isAvailable()) {
                throw new IllegalStateException(
                        "PlanarPoseHistory.Lookup.unavailableReason() requires an unavailable "
                                + "lookup; kind is " + kind
                );
            }
            return unavailableReason;
        }

        private void requireAvailable(String field) {
            if (!isAvailable()) {
                throw new IllegalStateException(
                        "PlanarPoseHistory.Lookup." + field + "() requires an available lookup; "
                                + "unavailable reason is " + unavailableReason
                );
            }
        }

        @Override
        public String toString() {
            if (!isAvailable()) {
                return "Lookup{kind=UNAVAILABLE, reason=" + unavailableReason + '}';
            }
            return "Lookup{kind=" + kind
                    + ", fieldToRobotPose=" + fieldToRobotPose
                    + ", quality=" + quality
                    + ", timestamp=" + timestamp
                    + '}';
        }
    }

    private static final class Sample {
        final Pose2d fieldToRobotPose;
        final double quality;
        final LoopTimestamp timestamp;
        final long historySegmentId;

        Sample(Pose2d fieldToRobotPose,
               double quality,
               LoopTimestamp timestamp,
               long historySegmentId) {
            this.fieldToRobotPose = fieldToRobotPose;
            this.quality = quality;
            this.timestamp = timestamp;
            this.historySegmentId = historySegmentId;
        }
    }

    private enum RecordAction {
        GAP,
        DUPLICATE,
        APPEND
    }

    private static final class RecordDecision {
        final RecordAction action;
        final Sample sample;
        final int removals;
        final boolean evicted;
        final boolean discontinuityClear;
        final long historySegmentId;
        final long sourceSegmentId;
        final boolean hasSourceSegment;
        final boolean gapSinceLastSample;

        RecordDecision(RecordAction action,
                       Sample sample,
                       int removals,
                       boolean evicted,
                       boolean discontinuityClear,
                       long historySegmentId,
                       long sourceSegmentId,
                       boolean hasSourceSegment,
                       boolean gapSinceLastSample) {
            this.action = action;
            this.sample = sample;
            this.removals = removals;
            this.evicted = evicted;
            this.discontinuityClear = discontinuityClear;
            this.historySegmentId = historySegmentId;
            this.sourceSegmentId = sourceSegmentId;
            this.hasSourceSegment = hasSourceSegment;
            this.gapSinceLastSample = gapSinceLastSample;
        }
    }

    private final PoseTrajectoryEstimator estimator;
    private final Config config;
    private final Deque<Sample> samples = new ArrayDeque<>();
    private final TimeAwareSource<Lookup> lookupSource;

    private LoopClock ownerClock;
    private LoopClock lastRecordClock;
    private long lastRecordCycle = Long.MIN_VALUE;
    private RuntimeException lastRecordFailure;
    private boolean recordInProgress;
    private boolean resetInProgress;
    private boolean hasEvicted;
    private boolean hasDiscardedDiscontinuity;
    private long historySegmentId;
    private long lastSourceSegmentId;
    private boolean hasSourceSegment;
    private boolean gapSinceLastSample;

    /**
     * Creates one bounded history over the exact authoritative trajectory estimator supplied.
     *
     * <p>Construction snapshots and validates {@code config} without reading or updating the
     * estimator.</p>
     */
    public PlanarPoseHistory(PoseTrajectoryEstimator estimator, Config config) {
        this.estimator = Objects.requireNonNull(estimator, "estimator");
        this.config = Objects.requireNonNull(config, "config").validatedCopy(
                PlanarPoseHistory.class.getCanonicalName() + ".Config"
        );
        this.lookupSource = new TimeAwareSource<Lookup>() {
            @Override
            public Lookup getAt(LoopClock clock, LoopTimestamp timestamp) {
                return lookup(clock, timestamp);
            }

            /** This borrowed read-only projection does not own the concrete history. */
            @Override
            public void reset() {
                // Deliberately no-op. Only PlanarPoseHistory.this.reset() clears retained samples.
            }

            @Override
            public String toString() {
                return "PlanarPoseHistory.lookupSource";
            }
        };
    }

    /**
     * Records the estimator's already-published current sample at most once in this loop cycle.
     *
     * <p>The first attempt claims the cycle before reading the borrowed estimator. A successful
     * same-cycle repeat is a no-op. A failed attempt is retained and rethrown by identity for the
     * rest of that cycle; the next cycle may try again. Reentrant recording fails before publishing
     * any partial history mutation.</p>
     */
    public void recordCurrent(LoopClock clock) {
        LoopClock requiredClock = Objects.requireNonNull(clock, "clock");
        if (resetInProgress) {
            throw new IllegalStateException(
                    "PlanarPoseHistory.recordCurrent(clock) cannot overlap reset()"
            );
        }
        if (ownerClock != null && ownerClock != requiredClock) {
            throw new IllegalArgumentException(
                    "PlanarPoseHistory requires one stable LoopClock instance; call reset() at a "
                            + "lifecycle boundary before recording with another clock"
            );
        }
        long cycle = requiredClock.cycle();
        if (recordInProgress) {
            IllegalStateException reentry = new IllegalStateException(
                    "PlanarPoseHistory.recordCurrent(clock) was reentered during cycle " + cycle
                            + "; one localization owner must record at most once per cycle"
            );
            lastRecordFailure = reentry;
            throw reentry;
        }
        if (requiredClock == lastRecordClock && cycle == lastRecordCycle) {
            if (lastRecordFailure != null) {
                throw lastRecordFailure;
            }
            return;
        }

        // Claim this exact clock/cycle before clock readiness or borrowed-estimator reads.
        lastRecordClock = requiredClock;
        lastRecordCycle = cycle;
        lastRecordFailure = null;
        recordInProgress = true;
        try {
            requiredClock.nowTimestamp();
            RecordDecision decision = prepareRecord(requiredClock);
            if (lastRecordFailure != null) {
                throw lastRecordFailure;
            }
            commit(decision);
            ownerClock = requiredClock;
        } catch (RuntimeException failure) {
            lastRecordFailure = failure;
            throw failure;
        } finally {
            recordInProgress = false;
        }
    }

    /**
     * Internal query implementation used by the stable public projection and focused tests.
     *
     * <p>A timestamp from another clock is an actionable caller error. An unavailable, prior-epoch,
     * or materially future timestamp instead returns a typed unavailable lookup.</p>
     */
    Lookup lookup(LoopClock clock, LoopTimestamp timestamp) {
        LoopClock requiredClock = Objects.requireNonNull(clock, "clock");
        LoopTimestamp requested = Objects.requireNonNull(timestamp, "timestamp");
        if (ownerClock != null && ownerClock != requiredClock) {
            throw new IllegalArgumentException(
                    "PlanarPoseHistory is bound to a different LoopClock; pass the same stable "
                            + "clock used by recordCurrent(clock), or reset the concrete history "
                            + "before a lifecycle restart"
            );
        }
        if (!requested.isAvailable()) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.QUERY_TIMESTAMP_UNAVAILABLE,
                    requested
            );
        }

        final double requestedAgeSec = requested.ageSec(requiredClock);
        if (!Double.isFinite(requestedAgeSec)) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.QUERY_TIMESTAMP_NOT_CURRENT,
                    requested
            );
        }
        if (samples.isEmpty()) {
            return Lookup.unavailable(emptyReason(), requested);
        }

        Sample storedFirst = samples.peekFirst();
        Sample last = samples.peekLast();
        double storedFirstAgeSec = storedFirst.timestamp.ageSec(requiredClock);
        double lastAgeSec = last.timestamp.ageSec(requiredClock);
        if (!Double.isFinite(storedFirstAgeSec) || !Double.isFinite(lastAgeSec)) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.DISCONTINUITY,
                    requested
            );
        }
        Sample first = null;
        for (Sample sample : samples) {
            double sampleAgeSec = sample.timestamp.ageSec(requiredClock);
            if (!Double.isFinite(sampleAgeSec)) {
                return Lookup.unavailable(
                        Lookup.UnavailableReason.DISCONTINUITY,
                        requested
                );
            }
            if (sampleAgeSec <= config.retentionSec) {
                first = sample;
                break;
            }
        }
        if (first == null) {
            return Lookup.unavailable(Lookup.UnavailableReason.EVICTED, requested);
        }
        boolean skippedExpiredPrefix = first != storedFirst;
        double sinceFirst = requested.secondsSince(first.timestamp);
        double sinceLast = requested.secondsSince(last.timestamp);
        if (!Double.isFinite(sinceFirst) || !Double.isFinite(sinceLast)) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.QUERY_TIMESTAMP_NOT_CURRENT,
                    requested
            );
        }
        if (sinceFirst < 0.0) {
            return Lookup.unavailable(
                    skippedExpiredPrefix
                            ? Lookup.UnavailableReason.EVICTED
                            : beforeFirstReason(),
                    requested
            );
        }
        if (sinceLast > 0.0) {
            return Lookup.unavailable(Lookup.UnavailableReason.AFTER_LATEST, requested);
        }

        Sample exact = null;
        Sample left = null;
        Sample right = null;
        boolean reachedEligibleWindow = false;
        for (Sample sample : samples) {
            if (!reachedEligibleWindow) {
                reachedEligibleWindow = sample == first;
                if (!reachedEligibleWindow) {
                    continue;
                }
            }
            double sinceSample = requested.secondsSince(sample.timestamp);
            if (!Double.isFinite(sinceSample)) {
                return Lookup.unavailable(
                        Lookup.UnavailableReason.QUERY_TIMESTAMP_NOT_CURRENT,
                        requested
                );
            }
            if (sinceSample == 0.0) {
                if (exact != null) {
                    return Lookup.unavailable(
                            Lookup.UnavailableReason.DISCONTINUITY,
                            requested
                    );
                }
                exact = sample;
            } else if (sinceSample > 0.0) {
                left = sample;
            } else {
                right = sample;
                break;
            }
        }
        if (exact != null) {
            return Lookup.exact(exact, requested);
        }
        if (left == null) {
            return Lookup.unavailable(
                    skippedExpiredPrefix
                            ? Lookup.UnavailableReason.EVICTED
                            : beforeFirstReason(),
                    requested
            );
        }
        if (right == null) {
            return Lookup.unavailable(Lookup.UnavailableReason.AFTER_LATEST, requested);
        }
        if (left.historySegmentId != right.historySegmentId) {
            return Lookup.unavailable(Lookup.UnavailableReason.DISCONTINUITY, requested);
        }

        double intervalSec = right.timestamp.secondsSince(left.timestamp);
        if (!Double.isFinite(intervalSec) || intervalSec <= 0.0) {
            return Lookup.unavailable(Lookup.UnavailableReason.DISCONTINUITY, requested);
        }
        if (intervalSec > config.maxInterpolationGapSec) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.INTERPOLATION_TIME_GAP,
                    requested
            );
        }
        double translationInches = left.fieldToRobotPose.distanceTo(right.fieldToRobotPose);
        if (translationInches > config.maxInterpolationTranslationInches) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.INTERPOLATION_TRANSLATION_GAP,
                    requested
            );
        }
        // Wrap each finite endpoint before subtraction so opposite extreme finite headings cannot
        // overflow to infinity and then leak a NaN interpolation result.
        double leftYawRad = Pose2d.wrapToPi(left.fieldToRobotPose.headingRad);
        double rightYawRad = Pose2d.wrapToPi(right.fieldToRobotPose.headingRad);
        double yawDeltaRad = Pose2d.wrapToPi(rightYawRad - leftYawRad);
        if (!Double.isFinite(yawDeltaRad)
                || Math.abs(yawDeltaRad) > config.maxInterpolationYawRad) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.INTERPOLATION_YAW_GAP,
                    requested
            );
        }

        double fromLeftSec = requested.secondsSince(left.timestamp);
        double alpha = fromLeftSec / intervalSec;
        double interpolatedYawRad = Pose2d.wrapToPi(leftYawRad + alpha * yawDeltaRad);
        if (!Double.isFinite(interpolatedYawRad)) {
            return Lookup.unavailable(
                    Lookup.UnavailableReason.INTERPOLATION_YAW_GAP,
                    requested
            );
        }
        Pose2d pose = new Pose2d(
                left.fieldToRobotPose.xInches
                        + alpha * (right.fieldToRobotPose.xInches
                        - left.fieldToRobotPose.xInches),
                left.fieldToRobotPose.yInches
                        + alpha * (right.fieldToRobotPose.yInches
                        - left.fieldToRobotPose.yInches),
                interpolatedYawRad
        );
        return Lookup.interpolated(
                pose,
                Math.min(left.quality, right.quality),
                requested
        );
    }

    /**
     * Returns one stable read-only projection for consumers that need a {@link TimeAwareSource}.
     *
     * <p>Calling {@link TimeAwareSource#reset()} on the projection is deliberately a no-op. The
     * projection borrows this concrete owner and cannot clear its history.</p>
     */
    public TimeAwareSource<Lookup> lookupSource() {
        return lookupSource;
    }

    /**
     * Clears retained samples and releases the bound clock for a lifecycle restart.
     *
     * <p>This operation never resets the borrowed estimator. It is immediate and idempotent.</p>
     */
    public void reset() {
        if (recordInProgress) {
            IllegalStateException overlap = new IllegalStateException(
                    "PlanarPoseHistory.reset() cannot overlap recordCurrent(clock)"
            );
            lastRecordFailure = overlap;
            throw overlap;
        }
        if (resetInProgress) {
            throw new IllegalStateException("PlanarPoseHistory.reset() was reentered");
        }
        resetInProgress = true;
        try {
            samples.clear();
            ownerClock = null;
            lastRecordClock = null;
            lastRecordCycle = Long.MIN_VALUE;
            lastRecordFailure = null;
            hasEvicted = false;
            hasDiscardedDiscontinuity = false;
            historySegmentId = 0L;
            lastSourceSegmentId = 0L;
            hasSourceSegment = false;
            gapSinceLastSample = false;
        } finally {
            resetInProgress = false;
        }
    }

    private RecordDecision prepareRecord(LoopClock clock) {
        long segmentBefore = estimator.trajectorySegmentId();
        PoseEstimate estimate = Objects.requireNonNull(
                estimator.getEstimate(),
                "PoseTrajectoryEstimator.getEstimate() returned null"
        );
        long segmentAfter = estimator.trajectorySegmentId();
        if (segmentBefore != segmentAfter) {
            throw new IllegalStateException(
                    "PoseTrajectoryEstimator changed trajectorySegmentId while its cached estimate "
                            + "was being read; publish one coherent trajectory observation"
            );
        }

        int removals = expiredPrefixCount(clock);
        boolean clearForEpochOrRegression = removals == samples.size() && !samples.isEmpty()
                && !oldestTimestampIsCurrent(clock);
        boolean usable = isUsableCurrentEstimate(estimate, clock);
        if (!usable) {
            return new RecordDecision(
                    RecordAction.GAP,
                    null,
                    removals,
                    removals > 0 && !clearForEpochOrRegression,
                    clearForEpochOrRegression,
                    historySegmentId,
                    lastSourceSegmentId,
                    hasSourceSegment,
                    true
            );
        }

        Pose2d pose = estimate.toPose2d();
        Sample newest = samples.peekLast();
        double elapsedSec = newest == null
                ? Double.NaN
                : estimate.timestamp.secondsSince(newest.timestamp);
        boolean timeRestart = newest != null
                && (!Double.isFinite(elapsedSec) || elapsedSec < 0.0);
        if (timeRestart) {
            removals = samples.size();
            clearForEpochOrRegression = true;
        }

        if (newest != null
                && elapsedSec == 0.0
                && !gapSinceLastSample
                && segmentBefore == lastSourceSegmentId
                && newest.historySegmentId == historySegmentId
                && samePose(newest.fieldToRobotPose, pose)
                && Double.compare(newest.quality, estimate.quality) == 0) {
            return new RecordDecision(
                    RecordAction.DUPLICATE,
                    null,
                    removals,
                    removals > 0 && !clearForEpochOrRegression,
                    clearForEpochOrRegression,
                    historySegmentId,
                    segmentBefore,
                    true,
                    false
            );
        }

        boolean startsNewSegment = clearForEpochOrRegression
                || gapSinceLastSample
                || (hasSourceSegment && segmentBefore != lastSourceSegmentId)
                || (newest != null && elapsedSec == 0.0);
        long candidateHistorySegment = startsNewSegment
                ? nextHistorySegmentId()
                : historySegmentId;
        Sample sample = new Sample(
                pose,
                estimate.quality,
                estimate.timestamp,
                candidateHistorySegment
        );

        int retainedBeforeAppend = samples.size() - removals;
        int countOverflow = Math.max(0, retainedBeforeAppend + 1 - config.maxSamples);
        removals += countOverflow;
        return new RecordDecision(
                RecordAction.APPEND,
                sample,
                removals,
                removals > 0 && !clearForEpochOrRegression,
                clearForEpochOrRegression,
                candidateHistorySegment,
                segmentBefore,
                true,
                false
        );
    }

    private int expiredPrefixCount(LoopClock clock) {
        int removals = 0;
        for (Sample sample : samples) {
            double ageSec = sample.timestamp.ageSec(clock);
            if (!Double.isFinite(ageSec)) {
                return samples.size();
            }
            if (ageSec <= config.retentionSec) {
                break;
            }
            removals++;
        }
        return removals;
    }

    private boolean oldestTimestampIsCurrent(LoopClock clock) {
        Sample oldest = samples.peekFirst();
        return oldest == null || Double.isFinite(oldest.timestamp.ageSec(clock));
    }

    private static boolean isUsableCurrentEstimate(PoseEstimate estimate, LoopClock clock) {
        if (!estimate.hasPose
                || estimate.fieldToRobotPose == null
                || estimate.timestamp == null
                || !Double.isFinite(estimate.fieldToRobotPose.xInches)
                || !Double.isFinite(estimate.fieldToRobotPose.yInches)
                || !Double.isFinite(estimate.fieldToRobotPose.yawRad)
                || !Double.isFinite(estimate.quality)
                || estimate.quality < 0.0
                || estimate.quality > 1.0) {
            return false;
        }
        double ageSec = estimate.timestamp.ageSec(clock);
        return Double.isFinite(ageSec) && ageSec == 0.0;
    }

    private void commit(RecordDecision decision) {
        for (int i = 0; i < decision.removals; i++) {
            samples.removeFirst();
        }
        if (decision.action == RecordAction.APPEND) {
            samples.addLast(decision.sample);
        }
        if (decision.discontinuityClear) {
            hasEvicted = false;
            hasDiscardedDiscontinuity = true;
        }
        hasEvicted |= decision.evicted;
        historySegmentId = decision.historySegmentId;
        lastSourceSegmentId = decision.sourceSegmentId;
        hasSourceSegment = decision.hasSourceSegment;
        gapSinceLastSample = decision.gapSinceLastSample;
    }

    private long nextHistorySegmentId() {
        if (historySegmentId == Long.MAX_VALUE) {
            throw new IllegalStateException(
                    "PlanarPoseHistory trajectory-segment counter overflowed; reset the history "
                            + "before recording another discontinuity"
            );
        }
        return historySegmentId + 1L;
    }

    private Lookup.UnavailableReason emptyReason() {
        if (hasEvicted) {
            return Lookup.UnavailableReason.EVICTED;
        }
        if (hasDiscardedDiscontinuity) {
            return Lookup.UnavailableReason.DISCONTINUITY;
        }
        return Lookup.UnavailableReason.EMPTY;
    }

    private Lookup.UnavailableReason beforeFirstReason() {
        if (hasEvicted) {
            return Lookup.UnavailableReason.EVICTED;
        }
        if (hasDiscardedDiscontinuity) {
            return Lookup.UnavailableReason.DISCONTINUITY;
        }
        return Lookup.UnavailableReason.BEFORE_FIRST;
    }

    private static boolean samePose(Pose2d first, Pose2d second) {
        return Double.compare(first.xInches, second.xInches) == 0
                && Double.compare(first.yInches, second.yInches) == 0
                && Double.compare(first.headingRad, second.headingRad) == 0;
    }
}
