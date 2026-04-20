package edu.ftcphoenix.fw.actuation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Request describing the scalar target values that would satisfy a mechanism goal.
 *
 * <p>The request is in the same caller-facing units used by the downstream {@link Plant}. For a
 * {@link PositionPlant}, that means plant units: inches, degrees, logical servo position, ticks, or
 * whatever coordinate the position builder declared. The request does not know about field geometry,
 * robot semantics, or hardware-native conversion.</p>
 */
public final class ScalarSetpointRequest {

    private final boolean valid;
    private final String reason;
    private final List<ScalarSetpointCandidate> candidates;

    private ScalarSetpointRequest(boolean valid,
                                  String reason,
                                  List<ScalarSetpointCandidate> candidates) {
        this.valid = valid;
        this.reason = reason != null ? reason : "";
        this.candidates = Collections.unmodifiableList(new ArrayList<ScalarSetpointCandidate>(candidates));
    }

    /**
     * Returns a request with no valid setpoint.
     */
    public static ScalarSetpointRequest none(String reason) {
        return new ScalarSetpointRequest(false, reason != null ? reason : "no request", Collections.<ScalarSetpointCandidate>emptyList());
    }

    /**
     * Creates a request satisfied by exactly one value in the downstream Plant's public units.
     */
    public static ScalarSetpointRequest exact(String id, double value) {
        return oneOf(ScalarSetpointCandidate.exact(id, value));
    }

    /**
     * Creates an exact request with quality/timing metadata.
     */
    public static ScalarSetpointRequest exact(String id, double value, double quality, double ageSec, double timestampSec) {
        return oneOf(ScalarSetpointCandidate.exact(id, value, quality, ageSec, timestampSec));
    }


    /**
     * Creates a request satisfied by {@code value + k * plant.period()}.
     *
     * <p>Use this with {@link ScalarSetpointPlanner.DomainStage#forPositionPlant(PositionPlant)} when a
     * goal accepts periodic-equivalent positions but the period belongs to the plant/domain rather
     * than the request itself.</p>
     */
    public static ScalarSetpointRequest equivalentPosition(String id, double value) {
        return oneOf(ScalarSetpointCandidate.equivalentPosition(id, value));
    }

    /**
     * Creates an equivalent-position request with quality/timing metadata.
     */
    public static ScalarSetpointRequest equivalentPosition(String id, double value, double quality, double ageSec, double timestampSec) {
        return oneOf(ScalarSetpointCandidate.equivalentPosition(id, value, quality, ageSec, timestampSec));
    }

    /**
     * Creates a request satisfied by {@code value + k*period}.
     *
     * <p>This explicit-period form is useful for standalone scalar planners. Prefer
     * {@link #equivalentPosition(String, double)} when the planner is bound to a
     * {@link PositionPlant} that declares its own period.</p>
     */
    public static ScalarSetpointRequest periodic(String id, double value, double period) {
        return oneOf(ScalarSetpointCandidate.periodic(id, value, period));
    }

    /**
     * Creates a periodic request with quality/timing metadata.
     */
    public static ScalarSetpointRequest periodic(String id, double value, double period, double quality, double ageSec) {
        return oneOf(ScalarSetpointCandidate.periodic(id, value, period, quality, ageSec, Double.NaN));
    }

    /**
     * Creates a periodic request with quality/timing metadata.
     */
    public static ScalarSetpointRequest periodic(String id, double value, double period, double quality, double ageSec, double timestampSec) {
        return oneOf(ScalarSetpointCandidate.periodic(id, value, period, quality, ageSec, timestampSec));
    }

    /**
     * Creates a relative request: target = current measurement + delta.
     */
    public static ScalarSetpointRequest relative(String id, double delta) {
        return oneOf(ScalarSetpointCandidate.relative(id, delta));
    }

    /**
     * Creates a relative periodic-equivalent request using the downstream plant's period.
     */
    public static ScalarSetpointRequest relativeEquivalentPosition(String id, double delta, double quality, double ageSec, double timestampSec) {
        return oneOf(ScalarSetpointCandidate.relativeEquivalentPosition(id, delta, quality, ageSec, timestampSec));
    }

    /**
     * Creates a relative periodic request: target = current measurement + delta + k*period.
     */
    public static ScalarSetpointRequest relativePeriodic(String id, double delta, double period, double quality, double ageSec, double timestampSec) {
        return oneOf(ScalarSetpointCandidate.relativePeriodic(id, delta, period, quality, ageSec, timestampSec));
    }

    /**
     * Creates a request satisfied by any one of the supplied candidates.
     */
    public static ScalarSetpointRequest oneOf(ScalarSetpointCandidate... candidates) {
        return oneOf(Arrays.asList(candidates));
    }

    /**
     * Creates a request satisfied by any one of the supplied candidates.
     */
    public static ScalarSetpointRequest oneOf(List<ScalarSetpointCandidate> candidates) {
        Objects.requireNonNull(candidates, "candidates");
        if (candidates.isEmpty()) {
            return none("no scalar candidates");
        }
        ArrayList<ScalarSetpointCandidate> copy = new ArrayList<ScalarSetpointCandidate>();
        for (ScalarSetpointCandidate c : candidates) {
            copy.add(Objects.requireNonNull(c, "candidates must not contain null"));
        }
        return new ScalarSetpointRequest(true, "", copy);
    }

    /**
     * Returns true when this request contains at least one candidate.
     */
    public boolean hasCandidates() {
        return valid && !candidates.isEmpty();
    }

    /**
     * Returns the immutable candidate list.
     */
    public List<ScalarSetpointCandidate> candidates() {
        return candidates;
    }

    /**
     * Returns why this request is invalid when {@link #hasCandidates()} is false.
     */
    public String reason() {
        return reason;
    }

    @Override
    public String toString() {
        return hasCandidates() ? "ScalarSetpointRequest" + candidates : "ScalarSetpointRequest.none{" + reason + "}";
    }
}
