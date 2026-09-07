package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;
import java.util.function.ToDoubleFunction;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Continuous geometric selection from located observations. Choose capture freshness, then a
 * policy. The policy answer constructs the source directly; there is no identity-latching mode.
 * A new frame may choose another candidate. Freezing a destination or associating physical objects
 * is an explicit robot behavior, never inferred from frame order or proximity here.
 *
 * <p>Every result publishes at most once per successful loop cycle; failure can retry. Reset clears
 * only selection-local cache, never borrowed observation or bearing sources. Cost callbacks must
 * be side-effect-free. Equal costs use geometric/identity/value ordering, not candidate list order;
 * entirely indistinguishable values remain indistinguishable, not tracked.</p>
 */
public final class TargetSelections {
    private TargetSelections() { }

    /** Starts one selector over a borrowed source. Construction does not sample it. */
    public static FreshnessStep from(Source<TargetObservations2d> observations) {
        Source<TargetObservations2d> required = Objects.requireNonNull(observations, "observations");
        return maxAgeSec -> new Policies(required, nonnegative("maxAgeSec", maxAgeSec));
    }

    /** Required choice of how old the original camera observation may be. */
    public interface FreshnessStep {
        /** Requires a finite non-negative inclusive capture-age bound in seconds. */
        PolicyStep freshWithinSec(double maxAgeSec);
    }

    /** Each complete policy answer constructs a fresh, continuously evaluated selection source. */
    public interface PolicyStep {
        /** Chooses minimum planar distance from robot origin at capture, not camera 3D range. */
        Source<TargetSelectionResult> nearestToRobot();
        /** Chooses minimum planar distance from an explicit fixed robot-at-capture control origin. */
        Source<TargetSelectionResult> nearestToControlFrame(Pose2d robotToControlFrame);
        /** Chooses the nearest available field point within an inclusive finite non-negative radius. */
        Source<TargetSelectionResult> nearFieldPoint(double fieldXInches, double fieldYInches,
                                                    double maxDistanceInches);
        /**
         * Chooses minimum angular difference from an explicit robot-frame bearing intent. The
         * intent must describe the same capture frame as the observation; this does not compensate
         * current driver input for robot motion. Non-finite intent yields no selection.
         */
        Source<TargetSelectionResult> nearestBearingRad(ScalarSource robotBearingIntent);
        /**
         * Chooses the candidate with most other positioned candidates within an inclusive radius.
         * Counts one frame's observations, not proven physical objects or accumulated sightings.
         */
        Source<TargetSelectionResult> mostNeighborsWithinInches(double radiusInches);
        /**
         * Chooses the lowest finite custom cost. Non-finite cost excludes a candidate. The supplied
         * function is a pure value operation; thrown failures remain eligible for same-cycle retry.
         */
        Source<TargetSelectionResult> lowestCost(ToDoubleFunction<TargetObservation2d> cost);
    }

    private interface MetricFactory {
        ToDoubleFunction<TargetObservation2d> prepare(TargetObservations2d frame, LoopClock clock);
    }

    private static final class Policies implements PolicyStep {
        private final Source<TargetObservations2d> source;
        private final double maxAgeSec;

        Policies(Source<TargetObservations2d> source, double maxAgeSec) {
            this.source = source;
            this.maxAgeSec = maxAgeSec;
        }

        @Override
        public Source<TargetSelectionResult> nearestToRobot() {
            return select("nearest robot-at-capture origin", (frame, clock) -> observation ->
                    Math.hypot(observation.forwardInches, observation.leftInches));
        }

        @Override
        public Source<TargetSelectionResult> nearestToControlFrame(Pose2d origin) {
            Objects.requireNonNull(origin, "robotToControlFrame");
            finite("robotToControlFrame.xInches", origin.xInches);
            finite("robotToControlFrame.yInches", origin.yInches);
            finite("robotToControlFrame.headingRad", origin.headingRad);
            return select("nearest fixed control origin at capture", (frame, clock) -> observation ->
                    Math.hypot(observation.forwardInches - origin.xInches,
                            observation.leftInches - origin.yInches));
        }

        @Override
        public Source<TargetSelectionResult> nearFieldPoint(double x, double y, double radius) {
            finite("fieldXInches", x);
            finite("fieldYInches", y);
            nonnegative("maxDistanceInches", radius);
            return select("nearest field point within radius", (frame, clock) -> observation -> {
                if (!observation.hasFieldPosition()) return Double.NaN;
                double distance = Math.hypot(observation.fieldXInches - x, observation.fieldYInches - y);
                return distance <= radius ? distance : Double.NaN;
            });
        }

        @Override
        public Source<TargetSelectionResult> nearestBearingRad(ScalarSource bearingIntent) {
            Objects.requireNonNull(bearingIntent, "robotBearingIntent");
            return select("nearest robot-frame bearing", (frame, clock) -> {
                double angle = bearingIntent.getAsDouble(clock);
                double wrapped = Math.atan2(Math.sin(angle), Math.cos(angle));
                return observation -> {
                    double difference = observation.bearingRad - wrapped;
                    return Math.abs(Math.atan2(Math.sin(difference), Math.cos(difference)));
                };
            });
        }

        @Override
        public Source<TargetSelectionResult> mostNeighborsWithinInches(double radius) {
            nonnegative("radiusInches", radius);
            return select("most neighboring observations in one frame", (frame, clock) -> observation -> {
                int neighbors = 0;
                for (TargetObservation2d other : frame.observations()) {
                    if (other != observation && other.hasPosition()
                            && Math.hypot(other.forwardInches - observation.forwardInches,
                            other.leftInches - observation.leftInches) <= radius) neighbors++;
                }
                return -neighbors;
            });
        }

        @Override
        public Source<TargetSelectionResult> lowestCost(ToDoubleFunction<TargetObservation2d> cost) {
            Objects.requireNonNull(cost, "cost");
            return select("lowest custom cost", (frame, clock) -> cost);
        }

        private Source<TargetSelectionResult> select(String policy, MetricFactory factory) {
            Source<TargetSelectionResult> calculated = Source.of(clock -> {
                TargetObservations2d frame = Objects.requireNonNull(source.get(clock), "observations frame");
                if (!frame.isAvailable()) return TargetSelectionResult.none(frame, maxAgeSec, frame.reason());
                if (!frame.isFresh(clock, maxAgeSec)) {
                    return TargetSelectionResult.none(frame, maxAgeSec, "observation frame stale, reset, or future");
                }
                if (frame.observations().isEmpty()) {
                    return TargetSelectionResult.none(frame, maxAgeSec, "observed empty frame");
                }
                ToDoubleFunction<TargetObservation2d> cost = factory.prepare(frame, clock);
                TargetObservation2d best = null;
                double bestCost = Double.POSITIVE_INFINITY;
                for (TargetObservation2d candidate : frame.observations()) {
                    if (!candidate.hasPosition()) continue;
                    double value = cost.applyAsDouble(candidate);
                    if (!Double.isFinite(value)) continue;
                    if (best == null || value < bestCost
                            || (value == bestCost && compare(candidate, best) < 0)) {
                        best = candidate;
                        bestCost = value;
                    }
                }
                return best == null
                        ? TargetSelectionResult.none(frame, maxAgeSec, "no positioned candidate satisfies " + policy)
                        : TargetSelectionResult.selected(frame, best, maxAgeSec, bestCost, policy);
            });
            return calculated.memoized();
        }
    }

    /** Total ordering of observable geometric values, independent of original list order. */
    private static int compare(TargetObservation2d a, TargetObservation2d b) {
        double[] av = {a.forwardInches, a.leftInches, a.fieldXInches, a.fieldYInches,
                a.bearingRad, a.targetHeadingRad, a.targetId, a.quality};
        double[] bv = {b.forwardInches, b.leftInches, b.fieldXInches, b.fieldYInches,
                b.bearingRad, b.targetHeadingRad, b.targetId, b.quality};
        for (int index = 0; index < av.length; index++) {
            int compared = Double.compare(av[index], bv[index]);
            if (compared != 0) return compared;
        }
        return 0;
    }

    private static double finite(String name, double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
        return value;
    }

    private static double nonnegative(String name, double value) {
        finite(name, value);
        if (value < 0) throw new IllegalArgumentException(name + " must be >= 0");
        return value;
    }
}
