package edu.ftcphoenix.fw.actuation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Request describing plant-unit targets that would satisfy a mechanism goal.
 *
 * <p>The request is semantic enough to allow equivalent choices, but still scalar: it does not own
 * hardware, does not command a Plant, and does not report physical completion. A
 * {@link PlantTargets#plan()} source resolves the request using the consuming Plant's context.</p>
 */
public final class PlantTargetRequest {

    private final boolean valid;
    private final String reason;
    private final List<PlantTargetCandidate> candidates;

    private PlantTargetRequest(boolean valid, String reason, List<PlantTargetCandidate> candidates) {
        this.valid = valid;
        this.reason = reason != null ? reason : "";
        this.candidates = Collections.unmodifiableList(new ArrayList<PlantTargetCandidate>(candidates));
    }

    /**
     * Returns a request with no valid target candidates.
     */
    public static PlantTargetRequest none(String reason) {
        return new PlantTargetRequest(false, reason != null ? reason : "no request", Collections.emptyList());
    }

    /**
     * Creates a request satisfied by exactly one plant-unit value.
     */
    public static PlantTargetRequest exact(String id, double value) {
        return oneOf(PlantTargetCandidate.exact(id, value));
    }

    /**
     * Creates an exact request with quality/timing metadata.
     */
    public static PlantTargetRequest exact(String id, double value, double quality, double ageSec, double timestampSec) {
        return oneOf(PlantTargetCandidate.exact(id, value, quality, ageSec, timestampSec));
    }

    /**
     * Creates a request satisfied by {@code value + k * plant.period()}.
     */
    public static PlantTargetRequest equivalentPosition(String id, double value) {
        return oneOf(PlantTargetCandidate.equivalentPosition(id, value));
    }

    /**
     * Creates an equivalent-position request with quality/timing metadata.
     */
    public static PlantTargetRequest equivalentPosition(String id, double value, double quality, double ageSec, double timestampSec) {
        return oneOf(PlantTargetCandidate.equivalentPosition(id, value, quality, ageSec, timestampSec));
    }

    /**
     * Creates a request satisfied by {@code value + k*period}.
     */
    public static PlantTargetRequest periodic(String id, double value, double period) {
        return oneOf(PlantTargetCandidate.periodic(id, value, period));
    }

    /**
     * Creates a periodic request with quality/timing metadata.
     */
    public static PlantTargetRequest periodic(String id, double value, double period, double quality, double ageSec, double timestampSec) {
        return oneOf(PlantTargetCandidate.periodic(id, value, period, quality, ageSec, timestampSec));
    }

    /**
     * Creates a relative request: target = current measurement + delta.
     */
    public static PlantTargetRequest relative(String id, double delta) {
        return oneOf(PlantTargetCandidate.relative(id, delta));
    }

    /**
     * Creates a relative periodic-equivalent request using the consuming plant's period.
     */
    public static PlantTargetRequest relativeEquivalentPosition(String id, double delta) {
        return oneOf(PlantTargetCandidate.relativeEquivalentPosition(id, delta));
    }

    /**
     * Creates a relative periodic-equivalent request using the consuming plant's period.
     */
    public static PlantTargetRequest relativeEquivalentPosition(String id, double delta, double quality, double ageSec, double timestampSec) {
        return oneOf(PlantTargetCandidate.relativeEquivalentPosition(id, delta, quality, ageSec, timestampSec));
    }

    /**
     * Creates a relative periodic request: target = current measurement + delta + k*period.
     */
    public static PlantTargetRequest relativePeriodic(String id, double delta, double period, double quality, double ageSec, double timestampSec) {
        return oneOf(PlantTargetCandidate.relativePeriodic(id, delta, period, quality, ageSec, timestampSec));
    }

    /**
     * Creates a relative periodic request: target = current measurement + delta + k*period.
     */
    public static PlantTargetRequest relativePeriodic(String id, double delta, double period) {
        return oneOf(PlantTargetCandidate.relativePeriodic(id, delta, period));
    }

    /**
     * Creates a request satisfied by any one of the supplied candidates.
     */
    public static PlantTargetRequest oneOf(PlantTargetCandidate... candidates) {
        return oneOf(Arrays.asList(candidates));
    }

    /**
     * Creates a request satisfied by any one of the supplied candidates.
     */
    public static PlantTargetRequest oneOf(List<PlantTargetCandidate> candidates) {
        Objects.requireNonNull(candidates, "candidates");
        if (candidates.isEmpty()) return none("no plant target candidates");
        ArrayList<PlantTargetCandidate> copy = new ArrayList<PlantTargetCandidate>();
        for (PlantTargetCandidate c : candidates)
            copy.add(Objects.requireNonNull(c, "candidates must not contain null"));
        return new PlantTargetRequest(true, "", copy);
    }

    /**
     * True when this request contains at least one usable candidate.
     */
    public boolean hasCandidates() {
        return valid && !candidates.isEmpty();
    }

    /**
     * Immutable candidate list.
     */
    public List<PlantTargetCandidate> candidates() {
        return candidates;
    }

    /**
     * Explanation when {@link #hasCandidates()} is false.
     */
    public String reason() {
        return reason;
    }

    @Override
    public String toString() {
        return hasCandidates() ? "PlantTargetRequest" + candidates : "PlantTargetRequest.none{" + reason + "}";
    }
}
