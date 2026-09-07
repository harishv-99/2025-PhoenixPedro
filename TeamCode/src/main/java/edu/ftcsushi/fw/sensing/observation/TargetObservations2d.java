package edu.ftcsushi.fw.sensing.observation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.IdentityHashMap;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Immutable bounded observations from one processed frame, including observed-empty frames.
 * Unavailable means no usable frame; empty means a frame with no published candidates.
 * Ordering is frame-local, never a persistent identity or tracking promise.
 */
public final class TargetObservations2d {
    /** Software allocation/ranking bound, not a physical-scene object-count guarantee. */
    public static final int MAX_OBSERVATIONS = 256;
    private final boolean available;
    private final LoopTimestamp timestamp;
    private final List<TargetObservation2d> observations;
    private final String reason;

    private TargetObservations2d(boolean available, LoopTimestamp timestamp,
                                 List<TargetObservation2d> observations, String reason) {
        this.available = available;
        this.timestamp = timestamp;
        this.observations = observations;
        this.reason = reason;
    }

    /**
     * Copies one processed frame. Every member must retain this exact timestamp object.
     * Freshness is checked at consumption. Rejects unavailable timestamps, excessive counts,
     * no-target members, and observations from another frame.
     */
    public static TargetObservations2d fromFrame(LoopTimestamp timestamp,
                                                List<TargetObservation2d> observations) {
        Objects.requireNonNull(timestamp, "timestamp");
        Objects.requireNonNull(observations, "observations");
        if (!timestamp.isAvailable()) throw new IllegalArgumentException("frame timestamp must be available");
        if (observations.size() > MAX_OBSERVATIONS) {
            throw new IllegalArgumentException("frame exceeds MAX_OBSERVATIONS=" + MAX_OBSERVATIONS);
        }
        ArrayList<TargetObservation2d> copy = new ArrayList<>(observations.size());
        IdentityHashMap<TargetObservation2d, Boolean> identities = new IdentityHashMap<>();
        for (TargetObservation2d observation : observations) {
            TargetObservation2d value = Objects.requireNonNull(observation, "observation member");
            if (!value.hasTarget || value.timestamp != timestamp) {
                throw new IllegalArgumentException("every observation must retain the exact frame timestamp");
            }
            if (identities.put(value, Boolean.TRUE) != null) {
                throw new IllegalArgumentException("one observation instance cannot appear twice in a frame");
            }
            copy.add(value);
        }
        return new TargetObservations2d(true, timestamp, Collections.unmodifiableList(copy),
                copy.isEmpty() ? "observed empty frame" : "available");
    }

    /** Returns no frame with a reason, rather than fabricating an empty capture. */
    public static TargetObservations2d unavailable(String reason) {
        Objects.requireNonNull(reason, "reason");
        if (reason.trim().isEmpty()) throw new IllegalArgumentException("reason must be nonblank");
        return new TargetObservations2d(false, LoopTimestamp.unavailable(), Collections.emptyList(), reason);
    }

    /** Whether a processed frame exists; does not imply freshness or a visible candidate. */
    public boolean isAvailable() { return available; }
    /** Tests original capture freshness, including for legitimately empty frames. */
    public boolean isFresh(LoopClock clock, double maxAgeSec) {
        return timestamp.isFresh(clock, maxAgeSec) && available;
    }
    /** Original capture timestamp, or unavailable when no frame exists. */
    public LoopTimestamp timestamp() { return timestamp; }
    /** Copied unmodifiable candidates from the one frame. */
    public List<TargetObservation2d> observations() { return observations; }
    /** Frame availability explanation. */
    public String reason() { return reason; }
}
