package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Immutable snapshot of the AprilTag detections produced from one processed camera frame.
 *
 * <p>All observations in {@link #observations} are understood to come from the same underlying
 * frame. This is important for multi-tag reasoning: selection policies, telemetry, and
 * localization can all operate on a coherent per-loop snapshot instead of issuing independent
 * sensor reads.</p>
 *
 * <h2>Why Phoenix returns all detections</h2>
 * <ul>
 *   <li><b>Selection should be explicit:</b> the framework no longer hides a global “best tag”
 *       policy inside the sensor boundary.</li>
 *   <li><b>Localization may use more than one tag:</b> a pose estimator can combine several fixed
 *       tags from the same frame to reduce noise.</li>
 *   <li><b>Telemetry should be honest:</b> drivers can see which tags are visible, which tag a
 *       selector preview would choose, and which tag is currently latched for aiming.</li>
 * </ul>
 */
public final class AprilTagDetections {

    /**
     * Age of the underlying camera frame, in seconds, when this snapshot was created.
     *
     * <p>When the implementation cannot determine a meaningful frame age, it may use 0 for
     * “captured just now” or {@link Double#POSITIVE_INFINITY} for “unknown / unusably stale”.</p>
     */
    public final double ageSec;

    /**
     * Timestamp of the underlying camera frame in the same timebase as LoopClock, or NaN when unknown.
     *
     * <p>When unknown, callers can compute an approximate timestamp as clock.nowSec() - ageSec.</p>
     */
    public final double timestampSec;

    /**
     * Immutable observations from the frame, expressed in Phoenix framing.
     */
    public final List<AprilTagObservation> observations;

    private AprilTagDetections(double ageSec, double timestampSec, List<AprilTagObservation> observations) {
        this.ageSec = ageSec;
        this.timestampSec = timestampSec;
        this.observations = observations;
    }

    /**
     * Returns an empty detections snapshot with unknown / unusably stale age.
     */
    public static AprilTagDetections none() {
        return none(Double.POSITIVE_INFINITY);
    }

    /**
     * Returns an empty detections snapshot with a caller-provided age.
     */
    public static AprilTagDetections none(double ageSec) {
        return new AprilTagDetections(ageSec, Double.NaN, Collections.<AprilTagObservation>emptyList());
    }

    /**
     * Creates a detections snapshot.
     *
     * @param ageSec       age of the underlying frame in seconds
     * @param observations detections from that frame; may be empty but must not be null
     * @return immutable detections snapshot
     */
    public static AprilTagDetections of(double ageSec, List<AprilTagObservation> observations) {
        return of(ageSec, Double.NaN, observations);
    }

    /**
     * Creates a detections snapshot with an explicit frame timestamp.
     * <p>
     * ageSec       age of the underlying frame in seconds
     * timestampSec timestamp of the frame in the LoopClock timebase, or NaN if unknown
     * observations detections from that frame; may be empty but must not be null
     * immutable detections snapshot
     */
    public static AprilTagDetections of(double ageSec, double timestampSec, List<AprilTagObservation> observations) {
        Objects.requireNonNull(observations, "observations");
        ArrayList<AprilTagObservation> copy = new ArrayList<AprilTagObservation>(observations.size());
        for (AprilTagObservation obs : observations) {
            copy.add(Objects.requireNonNull(obs, "observations must not contain null"));
        }
        return new AprilTagDetections(ageSec, timestampSec, Collections.unmodifiableList(copy));
    }

    /**
     * Returns the best available frame timestamp for this snapshot.
     *
     * <p>If timestampSec is known, it is returned directly. Otherwise this uses
     * clock.nowSec() - ageSec when age is finite, or NaN when neither value is usable.</p>
     *
     * @param clock current loop clock
     * @return timestamp in the LoopClock timebase, or NaN if unknown
     */
    public double frameTimestampSec(LoopClock clock) {
        if (Double.isFinite(timestampSec)) {
            return timestampSec;
        }
        if (clock != null && Double.isFinite(ageSec)) {
            return clock.nowSec() - ageSec;
        }
        return Double.NaN;
    }

    /**
     * Returns {@code true} when this frame age is within {@code maxAgeSec}.
     */
    public boolean isFresh(double maxAgeSec) {
        return Double.isFinite(ageSec) && ageSec <= maxAgeSec;
    }

    /**
     * Returns the first fresh observation for {@code id}, or {@link AprilTagObservation#noTarget(double)}.
     */
    public AprilTagObservation forId(int id, double maxAgeSec) {
        if (maxAgeSec < 0.0 || !isFresh(maxAgeSec)) {
            return AprilTagObservation.noTarget(ageSec);
        }
        for (AprilTagObservation obs : observations) {
            if (obs != null && obs.hasTarget && obs.id == id && obs.ageSec <= maxAgeSec) {
                return obs;
            }
        }
        return AprilTagObservation.noTarget(ageSec);
    }

    /**
     * Returns all fresh observations, preserving frame order.
     */
    public List<AprilTagObservation> freshObservations(double maxAgeSec) {
        if (maxAgeSec < 0.0 || !isFresh(maxAgeSec) || observations.isEmpty()) {
            return Collections.emptyList();
        }
        ArrayList<AprilTagObservation> out = new ArrayList<AprilTagObservation>(observations.size());
        for (AprilTagObservation obs : observations) {
            if (obs != null && obs.hasTarget && obs.ageSec <= maxAgeSec) {
                out.add(obs);
            }
        }
        return Collections.unmodifiableList(out);
    }

    /**
     * Returns all fresh observations whose IDs are contained in {@code idsOfInterest}.
     */
    public List<AprilTagObservation> freshMatching(Set<Integer> idsOfInterest, double maxAgeSec) {
        Objects.requireNonNull(idsOfInterest, "idsOfInterest");
        if (idsOfInterest.isEmpty()) {
            return Collections.emptyList();
        }
        if (maxAgeSec < 0.0 || !isFresh(maxAgeSec) || observations.isEmpty()) {
            return Collections.emptyList();
        }
        ArrayList<AprilTagObservation> out = new ArrayList<AprilTagObservation>();
        for (AprilTagObservation obs : observations) {
            if (obs != null && obs.hasTarget && obs.ageSec <= maxAgeSec && idsOfInterest.contains(obs.id)) {
                out.add(obs);
            }
        }
        return Collections.unmodifiableList(out);
    }

    /**
     * Returns the set of fresh visible IDs.
     */
    public Set<Integer> visibleIds(double maxAgeSec) {
        if (maxAgeSec < 0.0 || !isFresh(maxAgeSec) || observations.isEmpty()) {
            return Collections.emptySet();
        }
        LinkedHashSet<Integer> ids = new LinkedHashSet<Integer>();
        for (AprilTagObservation obs : observations) {
            if (obs != null && obs.hasTarget && obs.ageSec <= maxAgeSec) {
                ids.add(obs.id);
            }
        }
        return Collections.unmodifiableSet(ids);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "AprilTagDetections{ageSec=" + ageSec + ", observations=" + observations + "}";
    }
}
