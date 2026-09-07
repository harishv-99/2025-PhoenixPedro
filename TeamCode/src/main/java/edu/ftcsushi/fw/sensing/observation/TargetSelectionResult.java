package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Immutable geometric choice from one observed frame, not a tracked physical-object identity.
 * A retained result remains tied to the original camera capture and configured freshness bound.
 */
public final class TargetSelectionResult {
    private final TargetObservations2d frame;
    private final TargetObservation2d observation;
    private final double maxAgeSec;
    private final double metricValue;
    private final String reason;

    private TargetSelectionResult(TargetObservations2d frame, TargetObservation2d observation,
                                   double maxAgeSec, double metricValue, String reason) {
        this.frame = Objects.requireNonNull(frame, "frame");
        this.observation = Objects.requireNonNull(observation, "observation");
        if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0) {
            throw new IllegalArgumentException("maxAgeSec must be finite and >= 0");
        }
        this.maxAgeSec = maxAgeSec;
        this.metricValue = metricValue;
        this.reason = Objects.requireNonNull(reason, "reason");
        if (reason.trim().isEmpty()) throw new IllegalArgumentException("reason must be nonblank");
    }

    /**
     * Creates a selection of an actual member of the retained frame. A finite metric is ranking
     * evidence under the named policy, not confidence. Freshness is checked through isUsable.
     */
    public static TargetSelectionResult selected(TargetObservations2d frame,
            TargetObservation2d observation, double maxAgeSec, double metricValue, String reason) {
        Objects.requireNonNull(frame, "frame");
        Objects.requireNonNull(observation, "observation");
        if (!frame.isAvailable() || !observation.hasPosition()
                || !frame.observations().contains(observation)) {
            throw new IllegalArgumentException("selected observation must be a positioned member of the frame");
        }
        if (!Double.isFinite(metricValue)) throw new IllegalArgumentException("metricValue must be finite");
        return new TargetSelectionResult(frame, observation, maxAgeSec, metricValue, reason);
    }

    /** Retains the actual frame/reason when no candidate qualifies; does not invent a selection. */
    public static TargetSelectionResult none(TargetObservations2d frame, double maxAgeSec,
                                             String reason) {
        return new TargetSelectionResult(frame, TargetObservation2d.none(), maxAgeSec, Double.NaN, reason);
    }

    /** Whether this snapshot selected a candidate; not a claim it remains fresh now. */
    public boolean hasSelection() { return observation.hasTarget; }
    /** Whether this retained selection is still usable at the supplied shared-clock time. */
    public boolean isUsable(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        return hasSelection() && observation.isFresh(clock, maxAgeSec);
    }
    /** Selected frame member, or no-target if no candidate qualified. */
    public TargetObservation2d observation() { return observation; }
    /** Exact input frame, preserving unselected candidates and original capture time. */
    public TargetObservations2d frame() { return frame; }
    /** Inclusive observation-age policy retained by this result, in seconds. */
    public double maxAgeSec() { return maxAgeSec; }
    /** Policy metric (smaller wins), or NaN when nothing was selected. */
    public double metricValue() { return metricValue; }
    /** Policy/absence reason, without implying tracking or measured confidence. */
    public String reason() { return reason; }
}
