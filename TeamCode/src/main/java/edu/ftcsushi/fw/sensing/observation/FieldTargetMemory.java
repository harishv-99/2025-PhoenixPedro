package edu.ftcsushi.fw.sensing.observation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Bounded recent field-location hypotheses from one fixed-camera, anonymous-object producer.
 *
 * <p>The owning service calls {@link #update(LoopClock)} after camera/localization/history work.
 * Reads never advance memory. A retained location means "last seen here", not current visibility,
 * physical identity, capture confirmation, or permission to move. Field coordinates and original
 * capture-time pose provenance come from the borrowed input; this owner never polls localization.
 * It neither predicts movement nor removes a location merely because an image omits it.</p>
 *
 * <p>Explicitly {@link #reset(LoopClock) reset} before changing field coordinates, history,
 * camera configuration, or pipeline. Generic projected frames cannot reveal these transitions.
 * Reset fences cached captures at or before its boundary. Construct a new owner for a new source,
 * coordinate system, or camera graph. No borrowed source is reset or stopped.</p>
 */
public final class FieldTargetMemory {
    /** First required scalar answer: inclusive maximum original sighting age. */
    public interface RetentionStep {
        /** Requires finite, non-negative seconds; zero permits current-time sightings only. */
        MatchingStep retainingForSec(double retentionSec);
    }

    /** Second required scalar answer: inclusive field-position association distance. */
    public interface MatchingStep {
        /** Requires finite, non-negative inches; zero matches only exact coordinates. */
        CapacityStep matchingWithinInches(double matchRadiusInches);
    }

    /** Final answer bounds retained locations and pairwise matching work. */
    public interface CapacityStep {
        /** Constructs an independent owner; capacity must be within [1, MAX_OBSERVATIONS]. */
        FieldTargetMemory maxEntries(int capacity);
    }

    /**
     * Passive borrowed view with the owner's immutable usable-age bound. Its inherited reset is
     * intentionally inert. {@code get(clock)} returns the last snapshot, or rethrows a retained
     * failed advance until recovery/STOP; it never polls, expires, matches, or resets memory.
     */
    public interface View extends Source<Snapshot> {
        /** Inclusive finite, non-negative maximum sighting age, in seconds. */
        double retentionSec();
    }

    /** What happened to this update's input, separately from retained locations. */
    public enum InputDecision {
        NOT_UPDATED, UNAVAILABLE, INVALID_TIMESTAMP, RESET_FENCE, REPEATED_OR_OUT_OF_ORDER,
        CAPTURE_EXPIRED, OBSERVED_EMPTY, PROCESSED, RESET, CLOCK_RESET, STOPPED
    }

    /** Owner/generation/entry-lifetime identity, never a detector target ID or physical identity. */
    public static final class Key {
        private final Generation generation;
        private final long sequence;
        private boolean live = true;

        private Key(Generation generation, long sequence) {
            this.generation = generation;
            this.sequence = sequence;
        }

        /** Deterministic birth order within this owner generation; not globally unique. */
        public long sequence() { return sequence; }

        private boolean eligible() { return live && generation.eligible; }
    }

    /**
     * Immutable last sighting. A refresh creates another Entry with the same Key; old entries
     * never move or receive the new capture time. The private lifetime witness can only revoke
     * current eligibility, or temporarily suspend it after a failed owner update.
     */
    public static final class Entry {
        private final Key key;
        private final TargetObservation2d lastSighting;
        private final double retentionSec;

        private Entry(Key key, TargetObservation2d lastSighting, double retentionSec) {
            this.key = key;
            this.lastSighting = lastSighting;
            this.retentionSec = retentionSec;
        }

        /** Owner-minted location hypothesis key. */
        public Key key() { return key; }
        /** Exact admitted observation, including original timestamp and capture-time field lookup. */
        public TargetObservation2d lastSighting() { return lastSighting; }
        /**
         * Whether this original sighting remains within retention and its owner/key is eligible.
         * Does not advance memory. Reset, STOP, expiry, and eviction permanently revoke old keys.
         * A clock reset also makes old evidence unusable immediately, before the next heartbeat.
         */
        public boolean isUsable(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            return key.eligible() && lastSighting.timestamp.isFresh(clock, retentionSec);
        }
    }

    /** Immutable publication with mixed-age entries, never a fabricated common capture frame. */
    public static final class Snapshot {
        private final List<Entry> entries;
        private final LoopTimestamp timestamp;
        private final InputDecision inputDecision;
        private final String inputReason;
        private final int inputCandidateCount;
        private final int eligibleCandidateCount;
        private final int ambiguousCandidateCount;
        private final int refreshedCount;
        private final int createdCount;
        private final int expiredCount;
        private final int evictedCount;

        private Snapshot(List<Entry> entries, LoopTimestamp timestamp, InputDecision inputDecision,
                         String inputReason, int inputCandidateCount, int eligibleCandidateCount,
                         int ambiguousCandidateCount, int refreshedCount, int createdCount,
                         int expiredCount, int evictedCount) {
            this.entries = Collections.unmodifiableList(new ArrayList<>(entries));
            this.timestamp = timestamp;
            this.inputDecision = inputDecision;
            this.inputReason = inputReason;
            this.inputCandidateCount = inputCandidateCount;
            this.eligibleCandidateCount = eligibleCandidateCount;
            this.ambiguousCandidateCount = ambiguousCandidateCount;
            this.refreshedCount = refreshedCount;
            this.createdCount = createdCount;
            this.expiredCount = expiredCount;
            this.evictedCount = evictedCount;
        }

        /** Immutable bounded location hypotheses; list size is not a verified ball count. */
        public List<Entry> entries() { return entries; }
        /** Publication/reset time, or unavailable before update and after clock-independent STOP. */
        public LoopTimestamp timestamp() { return timestamp; }
        /** Input decision or lifecycle publication, distinct from entry eligibility. */
        public InputDecision inputDecision() { return inputDecision; }
        /** Input/lifecycle explanation, including the first ineligible-member reason when present. */
        public String inputReason() { return inputReason; }
        /** Whether this publication is the terminal stopped/empty value. */
        public boolean stopped() { return inputDecision == InputDecision.STOPPED; }
        /** Members in a newly considered, current and retention-eligible frame; otherwise zero. */
        public int inputCandidateCount() { return inputCandidateCount; }
        /** Members with eligible anonymous capture-time field geometry, including ambiguous ones. */
        public int eligibleCandidateCount() { return eligibleCandidateCount; }
        /** Considered members rejected for identity, position, or capture-time field evidence. */
        public int ineligibleCandidateCount() { return inputCandidateCount - eligibleCandidateCount; }
        /** Members blocked by duplicate geometry or a non-unique existing-location association. */
        public int ambiguousCandidateCount() { return ambiguousCandidateCount; }
        /** Existing keys refreshed by this publication. */
        public int refreshedCount() { return refreshedCount; }
        /** New keys created before this publication's possible capacity eviction. */
        public int createdCount() { return createdCount; }
        /** Keys retired because their last sighting exceeded retention. */
        public int expiredCount() { return expiredCount; }
        /** Keys retired to enforce capacity, oldest original sighting first. */
        public int evictedCount() { return evictedCount; }
    }

    private static final class Generation {
        boolean eligible = true;
    }

    private final Source<TargetObservations2d> fieldObjects;
    private final double retentionSec;
    private final double matchRadiusInches;
    private final int capacity;
    private final View view;
    private Snapshot snapshot = empty(LoopTimestamp.unavailable(), InputDecision.NOT_UPDATED,
            "memory has not been updated");
    private Generation generation = new Generation();
    private LoopClock ownerClock;
    private LoopTimestamp generationTimestamp = LoopTimestamp.unavailable();
    private LoopTimestamp fence = LoopTimestamp.unavailable();
    private LoopTimestamp watermark = LoopTimestamp.unavailable();
    private long nextSequence;
    private long claimedCycle = Long.MIN_VALUE;
    private RuntimeException cycleFailure;
    private RuntimeException viewFailure;
    private boolean updating;
    private boolean stopped;

    private FieldTargetMemory(Source<TargetObservations2d> fieldObjects, double retentionSec,
                              double matchRadiusInches, int capacity) {
        this.fieldObjects = fieldObjects;
        this.retentionSec = retentionSec;
        this.matchRadiusInches = matchRadiusInches;
        this.capacity = capacity;
        view = new View() {
            @Override public double retentionSec() { return FieldTargetMemory.this.retentionSec; }
            @Override public Snapshot get(LoopClock clock) {
                requireClock(clock, false);
                if (!stopped && viewFailure != null) throw viewFailure;
                return snapshot;
            }
        };
    }

    /**
     * Starts the sole construction path without sampling the borrowed projected-frame source.
     * Every returned stage is an independent immutable answer and may be safely reused.
     */
    public static RetentionStep fromFieldObjects(Source<TargetObservations2d> fieldObjects) {
        Objects.requireNonNull(fieldObjects, "fieldObjects");
        return retention -> {
            requireFiniteNonNegative("retentionSec", retention);
            return radius -> {
                requireFiniteNonNegative("matchRadiusInches", radius);
                return maximum -> {
                    if (maximum < 1 || maximum > TargetObservations2d.MAX_OBSERVATIONS) {
                        throw new IllegalArgumentException("maxEntries must be within [1, "
                                + TargetObservations2d.MAX_OBSERVATIONS + "], got " + maximum);
                    }
                    return new FieldTargetMemory(fieldObjects, retention, radius, maximum);
                };
            };
        };
    }

    /** Returns the last immutable publication without polling or advancing expiry. */
    public Snapshot snapshot() { return snapshot; }
    /** Returns one stable passive borrowed view, without sampling its input. */
    public View source() { return view; }

    /**
     * Advances once per shared clock cycle. New captures are consumed once, even if projection
     * is unavailable; equal/out-of-order frames cannot renew a sighting. Only proximity pairs
     * with degree one on both sides refresh. Exact duplicate candidates remain ambiguity blockers.
     *
     * <p>Claims the cycle before polling. A RuntimeException preserves the previous publication,
     * suspends entry eligibility, and is rethrown on repeated updates and view reads; a later cycle
     * can recover. Reentrant update/reset fails closed even if the input catches the exception.
     * Reset never releases a claimed cycle. STOP during input polling prevents later publication.</p>
     *
     * @throws IllegalStateException after STOP or on reentry
     * @throws IllegalArgumentException when a different owner clock is supplied
     */
    public void update(LoopClock clock) {
        requireActive();
        if (updating) throw fail(new IllegalStateException("FieldTargetMemory update/reset reentry"));
        requireClock(clock, true);
        if (claimedCycle == clock.cycle()) {
            if (cycleFailure != null) throw cycleFailure;
            return;
        }
        claimedCycle = clock.cycle();
        cycleFailure = null;
        updating = true;
        try {
            LoopTimestamp now = clock.nowTimestamp();
            if (!generationTimestamp.isAvailable()) {
                generationTimestamp = now;
            } else if (!Double.isFinite(now.secondsSince(generationTimestamp))) {
                invalidate(now, InputDecision.CLOCK_RESET, "clock epoch changed; awaiting newer capture");
            }
            TargetObservations2d frame = Objects.requireNonNull(fieldObjects.get(clock),
                    "fieldObjects returned null");
            if (stopped) return;
            if (cycleFailure != null) throw cycleFailure;
            if (clock.cycle() != claimedCycle
                    || !Double.isFinite(clock.nowTimestamp().secondsSince(now))) {
                throw new IllegalStateException("FieldTargetMemory input advanced/reset the shared LoopClock");
            }
            publishNext(clock, now, frame);
            viewFailure = null;
            generation.eligible = true;
        } catch (RuntimeException failure) {
            throw fail(failure);
        } finally {
            updating = false;
        }
    }

    /**
     * Immediately revokes keys and empties memory, fencing captures at/before the current time.
     * Bind before first update if needed. Does not poll/reset borrowed input or release this
     * cycle's successful/failed polling guard. A failed view stays failed until a later success.
     *
     * @throws IllegalStateException after STOP or on update/reset reentry
     */
    public void reset(LoopClock clock) {
        requireActive();
        if (updating) throw fail(new IllegalStateException("FieldTargetMemory update/reset reentry"));
        requireClock(clock, true);
        invalidate(clock.nowTimestamp(), InputDecision.RESET, "memory reset; awaiting newer capture");
    }

    /** Terminal, clock-independent, idempotent revocation; never stops a borrowed source. */
    public void stop() {
        if (stopped) return;
        stopped = true;
        generation.eligible = false;
        snapshot = empty(LoopTimestamp.unavailable(), InputDecision.STOPPED, "memory stopped");
        viewFailure = null;
    }

    /** Prepares all matching and eviction decisions before retiring keys or committing state. */
    private void publishNext(LoopClock clock, LoopTimestamp now, TargetObservations2d frame) {
        List<Entry> retained = new ArrayList<>();
        List<Key> retiring = new ArrayList<>();
        for (Entry entry : snapshot.entries) {
            if (entry.lastSighting.timestamp.isFresh(clock, retentionSec)) retained.add(entry);
            else retiring.add(entry.key);
        }
        int expired = retiring.size();
        LoopTimestamp nextWatermark = watermark;
        long preparedSequence = nextSequence;
        InputDecision decision = inputDecision(clock, now, frame);
        String reason = inputReason(decision, frame);
        int inputCount = 0;
        int eligibleCount = 0;
        int ambiguous = 0;
        int refreshed = 0;
        int created = 0;
        if (decision == InputDecision.PROCESSED || decision == InputDecision.OBSERVED_EMPTY
                || decision == InputDecision.CAPTURE_EXPIRED) {
            nextWatermark = frame.timestamp();
        }
        if (decision == InputDecision.PROCESSED) {
            List<TargetObservation2d> candidates = new ArrayList<>();
            String firstIneligible = null;
            inputCount = frame.observations().size();
            for (TargetObservation2d observation : frame.observations()) {
                String ineligible = ineligibleReason(observation);
                if (ineligible == null) candidates.add(observation);
                else if (firstIneligible == null) firstIneligible = ineligible;
            }
            eligibleCount = candidates.size();
            if (firstIneligible != null) {
                reason += "; ineligible=" + (inputCount - eligibleCount) + "; first: " + firstIneligible;
            }
            candidates.sort(FieldTargetMemory::compareCoordinates);
            int[] oldDegrees = new int[retained.size()];
            int[] newDegrees = new int[candidates.size()];
            int[] soleOldNeighbor = new int[candidates.size()];
            boolean[] duplicate = new boolean[candidates.size()];
            for (int i = 0; i < candidates.size(); i++) {
                TargetObservation2d candidate = candidates.get(i);
                for (int j = 0; j < i; j++) {
                    if (compareCoordinates(candidate, candidates.get(j)) == 0) {
                        duplicate[i] = true;
                        duplicate[j] = true;
                    }
                }
                for (int j = 0; j < retained.size(); j++) {
                    TargetObservation2d old = retained.get(j).lastSighting;
                    if (Math.hypot(candidate.fieldXInches - old.fieldXInches,
                            candidate.fieldYInches - old.fieldYInches) <= matchRadiusInches) {
                        oldDegrees[j]++;
                        newDegrees[i]++;
                        soleOldNeighbor[i] = j;
                    }
                }
            }
            // Only the pre-update retained set defines neighbors; births are not compared again.
            List<Entry> next = new ArrayList<>(retained);
            for (int i = 0; i < candidates.size(); i++) {
                TargetObservation2d candidate = candidates.get(i);
                if (duplicate[i]) {
                    ambiguous++;
                } else if (newDegrees[i] == 0) {
                    if (preparedSequence == Long.MAX_VALUE) {
                        throw new IllegalStateException("FieldTargetMemory key sequence exhausted; create a new owner");
                    }
                    next.add(new Entry(new Key(generation, preparedSequence++), candidate, retentionSec));
                    created++;
                } else if (newDegrees[i] == 1 && oldDegrees[soleOldNeighbor[i]] == 1) {
                    int index = soleOldNeighbor[i];
                    next.set(index, new Entry(retained.get(index).key, candidate, retentionSec));
                    refreshed++;
                } else {
                    ambiguous++;
                }
            }
            retained = next;
        }
        // Newest sightings win; deterministic birth order breaks equal-time eviction ties.
        retained.sort((a, b) -> {
            int ageOrder = Double.compare(a.lastSighting.timestamp.ageSec(clock),
                    b.lastSighting.timestamp.ageSec(clock));
            return ageOrder != 0 ? ageOrder : Long.compare(b.key.sequence, a.key.sequence);
        });
        int evicted = Math.max(0, retained.size() - capacity);
        for (int i = retained.size() - 1; i >= capacity; i--) retiring.add(retained.remove(i).key);
        retained.sort(Comparator.comparingLong(entry -> entry.key.sequence));
        Snapshot prepared = new Snapshot(retained, now, decision, reason, inputCount, eligibleCount,
                ambiguous, refreshed, created, expired, evicted);
        for (Key key : retiring) key.live = false;
        snapshot = prepared;
        watermark = nextWatermark;
        nextSequence = preparedSequence;
    }

    /** Rejects invalid/duplicate captures without moving the considered-capture watermark. */
    private InputDecision inputDecision(LoopClock clock, LoopTimestamp now, TargetObservations2d frame) {
        if (!frame.isAvailable()) return InputDecision.UNAVAILABLE;
        LoopTimestamp capture = frame.timestamp();
        try {
            double age = capture.ageSec(clock);
            double fromNow = capture.secondsSince(now);
            if (!Double.isFinite(age) || !Double.isFinite(fromNow) || fromNow > 0.0) {
                return InputDecision.INVALID_TIMESTAMP;
            }
            if (fence.isAvailable() && !(capture.secondsSince(fence) > 0.0)) {
                return InputDecision.RESET_FENCE;
            }
            if (watermark.isAvailable() && !(capture.secondsSince(watermark) > 0.0)) {
                return InputDecision.REPEATED_OR_OUT_OF_ORDER;
            }
            if (age > retentionSec) return InputDecision.CAPTURE_EXPIRED;
            return frame.observations().isEmpty() ? InputDecision.OBSERVED_EMPTY : InputDecision.PROCESSED;
        } catch (IllegalArgumentException foreignClock) {
            return InputDecision.INVALID_TIMESTAMP;
        }
    }

    /** Gives input rejection a diagnostic rather than fabricating current image evidence. */
    private static String inputReason(InputDecision decision, TargetObservations2d frame) {
        switch (decision) {
            case UNAVAILABLE: return frame.reason();
            case INVALID_TIMESTAMP: return "capture timestamp is future, foreign-clock, or no longer current";
            case RESET_FENCE: return "capture is at or before the reset fence";
            case REPEATED_OR_OUT_OF_ORDER: return "capture is not newer than the considered capture";
            case CAPTURE_EXPIRED: return "new capture is already older than retention";
            case OBSERVED_EMPTY: return "observed empty frame";
            default: return "processed field observations";
        }
    }

    /** Returns the reason a member cannot become a field location, or null when eligible. */
    private static String ineligibleReason(TargetObservation2d observation) {
        if (observation.targetId != -1) return "identified targets belong on the tag-selection path";
        if (!observation.hasPosition()) return "observation has no robot-relative position";
        if (!observation.hasFieldPosition() || observation.fieldLookup() == null
                || !observation.fieldLookup().isAvailable()
                || observation.fieldLookup().timestamp() != observation.timestamp) {
            return observation.fieldProjectionReason();
        }
        return null;
    }

    /** Treats signed zero as the same coordinate, for both duplicates and canonical key order. */
    private static int compareCoordinates(TargetObservation2d a, TargetObservation2d b) {
        int x = a.fieldXInches == b.fieldXInches ? 0 : Double.compare(a.fieldXInches, b.fieldXInches);
        if (x != 0) return x;
        return a.fieldYInches == b.fieldYInches ? 0 : Double.compare(a.fieldYInches, b.fieldYInches);
    }

    /** Invalidates one whole generation without resetting borrowed collaborators or cycle guards. */
    private void invalidate(LoopTimestamp now, InputDecision decision, String reason) {
        generation.eligible = false;
        generation = new Generation();
        generation.eligible = viewFailure == null;
        generationTimestamp = now;
        fence = now;
        watermark = LoopTimestamp.unavailable();
        nextSequence = 0;
        snapshot = empty(now, decision, reason);
    }

    /** Retains the first failure, including one caught by a reentrant input callback. */
    private RuntimeException fail(RuntimeException failure) {
        if (cycleFailure == null) cycleFailure = failure;
        if (!stopped) {
            viewFailure = cycleFailure;
            generation.eligible = false;
        }
        return cycleFailure;
    }

    /** Binds only at advancing lifecycle calls; passive reads do not establish an owner clock. */
    private void requireClock(LoopClock clock, boolean bind) {
        Objects.requireNonNull(clock, "clock");
        if (ownerClock != null && ownerClock != clock) {
            throw new IllegalArgumentException("FieldTargetMemory requires one stable LoopClock");
        }
        if (bind && ownerClock == null) ownerClock = clock;
    }

    private void requireActive() {
        if (stopped) throw new IllegalStateException("FieldTargetMemory is stopped; construct a new owner");
    }

    private static void requireFiniteNonNegative(String name, double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and >= 0, got " + value);
        }
    }

    private static Snapshot empty(LoopTimestamp timestamp, InputDecision decision, String reason) {
        return new Snapshot(Collections.emptyList(), timestamp, decision, reason, 0, 0, 0, 0, 0, 0, 0);
    }
}
