package edu.ftcsushi.fw.actuation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Immutable request describing plant-unit alternatives that would satisfy one mechanism intent.
 *
 * <p>The request may contain one or several ordered alternatives, but remains scalar: it does not
 * own hardware, command a Plant, or report physical completion. A
 * {@link PlantTargets#plan(PlantTargetRequest)} resolver uses the consuming Plant's context to
 * select one alternative. Robot code creates alternatives with the twelve named factories below
 * and combines them only through {@link #oneOf(PlantTargetRequest...)} or
 * {@link #oneOf(List)}; there is no separate public candidate type.</p>
 */
public final class PlantTargetRequest {

    private static final String EMPTY_REASON = "no plant target alternatives";
    private static final String NO_REQUEST_REASON = "no request";

    private final String reason;
    private final List<Alternative> alternatives;

    private PlantTargetRequest(String reason, List<Alternative> alternatives) {
        ArrayList<Alternative> copy = new ArrayList<Alternative>(alternatives);
        this.alternatives = Collections.unmodifiableList(copy);
        this.reason = copy.isEmpty() ? cleanReason(reason, EMPTY_REASON) : "";
    }

    /**
     * Returns a request with no usable target alternatives.
     */
    public static PlantTargetRequest none(String reason) {
        return new PlantTargetRequest(reason != null ? reason : NO_REQUEST_REASON,
                Collections.<Alternative>emptyList());
    }

    /**
     * Creates a request satisfied by exactly one plant-unit value.
     */
    public static PlantTargetRequest exact(String id, double value) {
        return single(Alternative.exact(id, value));
    }

    /**
     * Creates an exact request derived from an observation.
     *
     * @param quality observation quality for resolver acceptance
     * @param timestamp stable observation timestamp from the consuming loop clock
     */
    public static PlantTargetRequest observedExact(String id,
                                                   double value,
                                                   double quality,
                                                   LoopTimestamp timestamp) {
        return single(Alternative.observedExact(id, value, quality, timestamp));
    }

    /**
     * Creates a request satisfied by {@code value + k * plant.period()}.
     */
    public static PlantTargetRequest equivalentPosition(String id, double value) {
        return single(Alternative.equivalentPosition(id, value));
    }

    /**
     * Creates an equivalent-position request derived from an observation.
     *
     * @param quality observation quality for resolver acceptance
     * @param timestamp stable observation timestamp from the consuming loop clock
     */
    public static PlantTargetRequest observedEquivalentPosition(String id,
                                                                double value,
                                                                double quality,
                                                                LoopTimestamp timestamp) {
        return single(Alternative.observedEquivalentPosition(id, value, quality, timestamp));
    }

    /**
     * Creates a request satisfied by {@code value + k * period}.
     */
    public static PlantTargetRequest periodic(String id, double value, double period) {
        return single(Alternative.periodic(id, value, period));
    }

    /**
     * Creates a periodic request derived from an observation.
     *
     * @param quality observation quality for resolver acceptance
     * @param timestamp stable observation timestamp from the consuming loop clock
     */
    public static PlantTargetRequest observedPeriodic(String id,
                                                      double value,
                                                      double period,
                                                      double quality,
                                                      LoopTimestamp timestamp) {
        return single(Alternative.observedPeriodic(id, value, period, quality, timestamp));
    }

    /**
     * Creates a relative request: target = current measurement + delta.
     */
    public static PlantTargetRequest relative(String id, double delta) {
        return single(Alternative.relative(id, delta));
    }

    /**
     * Creates a relative exact request derived from an observation.
     *
     * @param quality observation quality for resolver acceptance
     * @param timestamp stable observation timestamp from the consuming loop clock
     */
    public static PlantTargetRequest observedRelative(String id,
                                                      double delta,
                                                      double quality,
                                                      LoopTimestamp timestamp) {
        return single(Alternative.observedRelative(id, delta, quality, timestamp));
    }

    /**
     * Creates a relative periodic-equivalent request using the consuming Plant's period.
     */
    public static PlantTargetRequest relativeEquivalentPosition(String id, double delta) {
        return single(Alternative.relativeEquivalentPosition(id, delta));
    }

    /**
     * Creates a relative periodic-equivalent request derived from an observation and using the
     * consuming Plant's period.
     *
     * @param quality observation quality for resolver acceptance
     * @param timestamp stable observation timestamp from the consuming loop clock
     */
    public static PlantTargetRequest observedRelativeEquivalentPosition(String id,
                                                                        double delta,
                                                                        double quality,
                                                                        LoopTimestamp timestamp) {
        return single(Alternative.observedRelativeEquivalentPosition(
                id, delta, quality, timestamp));
    }

    /**
     * Creates a relative periodic request: target = current measurement + delta + k * period.
     */
    public static PlantTargetRequest relativePeriodic(String id, double delta, double period) {
        return single(Alternative.relativePeriodic(id, delta, period));
    }

    /**
     * Creates a relative periodic request derived from an observation:
     * target = current measurement + delta + k * period.
     *
     * @param quality observation quality for resolver acceptance
     * @param timestamp stable observation timestamp from the consuming loop clock
     */
    public static PlantTargetRequest observedRelativePeriodic(String id,
                                                              double delta,
                                                              double period,
                                                              double quality,
                                                              LoopTimestamp timestamp) {
        return single(Alternative.observedRelativePeriodic(
                id, delta, period, quality, timestamp));
    }

    /**
     * Creates a request satisfied by any supplied request alternative, in declaration order.
     *
     * <p>Nested requests are flattened. Unavailable requests contribute no alternatives. If every
     * supplied request is unavailable, the result retains the first useful unavailable reason.</p>
     *
     * @throws NullPointerException if the array or one of its entries is null
     */
    public static PlantTargetRequest oneOf(PlantTargetRequest... alternatives) {
        Objects.requireNonNull(alternatives, "alternatives");
        return combine(Arrays.asList(alternatives));
    }

    /**
     * Creates a request satisfied by any request in an ordered dynamic list.
     *
     * <p>Nested requests are flattened through the same implementation used by the varargs form.
     * The result defensively copies every internal alternative, so later list changes cannot alter
     * this request. An empty list produces an unavailable request.</p>
     *
     * @throws NullPointerException if the list or one of its entries is null
     */
    public static PlantTargetRequest oneOf(List<PlantTargetRequest> alternatives) {
        return combine(Objects.requireNonNull(alternatives, "alternatives"));
    }

    private static PlantTargetRequest single(Alternative alternative) {
        return new PlantTargetRequest("", Collections.singletonList(alternative));
    }

    private static PlantTargetRequest combine(List<PlantTargetRequest> requests) {
        ArrayList<Alternative> flattened = new ArrayList<Alternative>();
        String firstReason = null;
        for (PlantTargetRequest request : requests) {
            PlantTargetRequest actual = Objects.requireNonNull(request,
                    "alternatives must not contain null");
            if (actual.hasAlternatives()) {
                flattened.addAll(actual.alternatives);
            } else if (firstReason == null && isActionableReason(actual.reason)) {
                firstReason = actual.reason;
            }
        }
        return flattened.isEmpty()
                ? none(firstReason != null ? firstReason : EMPTY_REASON)
                : new PlantTargetRequest("", flattened);
    }

    /**
     * Returns true when this request contains at least one usable target alternative.
     */
    public boolean hasAlternatives() {
        return !alternatives.isEmpty();
    }

    /** Package-private immutable alternatives consumed only by the framework planner. */
    List<Alternative> alternatives() {
        return alternatives;
    }

    /**
     * Returns the explanation when {@link #hasAlternatives()} is false.
     */
    public String reason() {
        return reason;
    }

    @Override
    public String toString() {
        return hasAlternatives()
                ? "PlantTargetRequest" + alternatives
                : "PlantTargetRequest.none{" + reason + "}";
    }

    private static String cleanReason(String text, String fallback) {
        return text == null || text.trim().isEmpty() ? fallback : text.trim();
    }

    private static boolean isActionableReason(String reason) {
        return !reason.isEmpty()
                && !EMPTY_REASON.equals(reason)
                && !NO_REQUEST_REASON.equals(reason);
    }

    /** Internal immutable representation owned by the request rather than exposed as an API noun. */
    static final class Alternative {
        private final String id;
        private final double value;
        private final boolean periodic;
        private final double period;
        private final boolean usesPlantPeriod;
        private final boolean relative;
        private final double quality;
        private final LoopTimestamp timestamp;
        private final boolean observed;

        private Alternative(String id,
                            double value,
                            boolean periodic,
                            double period,
                            boolean usesPlantPeriod,
                            boolean relative,
                            double quality,
                            LoopTimestamp timestamp,
                            boolean observed) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("Plant target value must be finite");
            }
            if (periodic && !usesPlantPeriod && (!(period > 0.0) || !Double.isFinite(period))) {
                throw new IllegalArgumentException(
                        "Periodic plant target alternatives require finite period > 0");
            }
            this.id = (id == null || id.trim().isEmpty()) ? "candidate" : id.trim();
            this.value = value;
            this.periodic = periodic;
            this.period = period;
            this.usesPlantPeriod = usesPlantPeriod;
            this.relative = relative;
            this.quality = quality;
            this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
            this.observed = observed;
        }

        private static Alternative exact(String id, double value) {
            return new Alternative(id, value, false, Double.NaN, false, false,
                    1.0, LoopTimestamp.unavailable(), false);
        }

        private static Alternative observedExact(String id,
                                                 double value,
                                                 double quality,
                                                 LoopTimestamp timestamp) {
            return new Alternative(id, value, false, Double.NaN, false, false,
                    quality, timestamp, true);
        }

        private static Alternative equivalentPosition(String id, double value) {
            return new Alternative(id, value, true, Double.NaN, true, false,
                    1.0, LoopTimestamp.unavailable(), false);
        }

        private static Alternative observedEquivalentPosition(String id,
                                                               double value,
                                                               double quality,
                                                               LoopTimestamp timestamp) {
            return new Alternative(id, value, true, Double.NaN, true, false,
                    quality, timestamp, true);
        }

        private static Alternative periodic(String id, double value, double period) {
            return new Alternative(id, value, true, period, false, false,
                    1.0, LoopTimestamp.unavailable(), false);
        }

        private static Alternative observedPeriodic(String id,
                                                    double value,
                                                    double period,
                                                    double quality,
                                                    LoopTimestamp timestamp) {
            return new Alternative(id, value, true, period, false, false,
                    quality, timestamp, true);
        }

        private static Alternative relative(String id, double delta) {
            return new Alternative(id, delta, false, Double.NaN, false, true,
                    1.0, LoopTimestamp.unavailable(), false);
        }

        private static Alternative observedRelative(String id,
                                                    double delta,
                                                    double quality,
                                                    LoopTimestamp timestamp) {
            return new Alternative(id, delta, false, Double.NaN, false, true,
                    quality, timestamp, true);
        }

        private static Alternative relativeEquivalentPosition(String id, double delta) {
            return new Alternative(id, delta, true, Double.NaN, true, true,
                    1.0, LoopTimestamp.unavailable(), false);
        }

        private static Alternative observedRelativeEquivalentPosition(String id,
                                                                      double delta,
                                                                      double quality,
                                                                      LoopTimestamp timestamp) {
            return new Alternative(id, delta, true, Double.NaN, true, true,
                    quality, timestamp, true);
        }

        private static Alternative relativePeriodic(String id, double delta, double period) {
            return new Alternative(id, delta, true, period, false, true,
                    1.0, LoopTimestamp.unavailable(), false);
        }

        private static Alternative observedRelativePeriodic(String id,
                                                            double delta,
                                                            double period,
                                                            double quality,
                                                            LoopTimestamp timestamp) {
            return new Alternative(id, delta, true, period, false, true,
                    quality, timestamp, true);
        }

        String id() {
            return id;
        }

        double value() {
            return value;
        }

        boolean periodic() {
            return periodic;
        }

        double period() {
            return period;
        }

        boolean usesPlantPeriod() {
            return usesPlantPeriod;
        }

        boolean relative() {
            return relative;
        }

        double quality() {
            return quality;
        }

        LoopTimestamp timestamp() {
            return timestamp;
        }

        boolean observed() {
            return observed;
        }

        @Override
        public String toString() {
            return "{id='" + id + "', value=" + value
                    + ", periodic=" + periodic + ", period=" + period
                    + ", usesPlantPeriod=" + usesPlantPeriod
                    + ", relative=" + relative + ", quality=" + quality
                    + ", observed=" + observed + ", timestamp=" + timestamp + '}';
        }
    }
}
