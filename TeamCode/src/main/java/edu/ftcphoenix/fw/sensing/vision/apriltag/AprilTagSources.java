package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Convenience helpers for deriving smaller source-graph values from raw AprilTag detections.
 *
 * <p>Use these when you want direct visibility checks or a specific tag observation without
 * introducing a {@link TagSelectionSource}. The helpers remain explicit: you still choose the ID
 * set and freshness window at the call site.</p>
 */
public final class AprilTagSources {

    private AprilTagSources() {
        // Utility class.
    }

    /**
     * Returns a source that looks up one specific fresh tag observation by ID.
     */
    public static Source<AprilTagObservation> observationForId(Source<AprilTagDetections> detections,
                                                               int id,
                                                               double maxAgeSec) {
        Objects.requireNonNull(detections, "detections");
        return new Source<AprilTagObservation>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public AprilTagObservation get(LoopClock clock) {
                AprilTagDetections dets = Objects.requireNonNull(detections.get(clock), "detections returned null");
                return dets.forId(id, maxAgeSec);
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                detections.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "aprilTag.obs" : prefix;
                dbg.addData(p + ".id", id)
                        .addData(p + ".maxAgeSec", maxAgeSec);
                detections.debugDump(dbg, p + ".detections");
            }
        };
    }

    /**
     * Returns a boolean source that is true when the given ID is freshly visible.
     */
    public static BooleanSource hasFresh(Source<AprilTagDetections> detections, int id, double maxAgeSec) {
        return observationForId(detections, id, maxAgeSec).mapToBoolean(new java.util.function.Predicate<AprilTagObservation>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean test(AprilTagObservation obs) {
                return obs != null && obs.hasTarget;
            }
        });
    }

    /**
     * Returns a boolean source that is true when any ID in {@code idsOfInterest} is freshly visible.
     */
    public static BooleanSource hasFreshAny(Source<AprilTagDetections> detections,
                                            Set<Integer> idsOfInterest,
                                            double maxAgeSec) {
        Objects.requireNonNull(detections, "detections");
        Objects.requireNonNull(idsOfInterest, "idsOfInterest");
        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                AprilTagDetections dets = Objects.requireNonNull(detections.get(clock), "detections returned null");
                return !dets.freshMatching(idsOfInterest, maxAgeSec).isEmpty();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                detections.reset();
            }
        };
    }

    /**
     * Returns a source exposing the set of freshly visible IDs.
     */
    public static Source<Set<Integer>> visibleIds(Source<AprilTagDetections> detections, double maxAgeSec) {
        Objects.requireNonNull(detections, "detections");
        return new Source<Set<Integer>>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public Set<Integer> get(LoopClock clock) {
                AprilTagDetections dets = Objects.requireNonNull(detections.get(clock), "detections returned null");
                return dets.visibleIds(maxAgeSec);
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                detections.reset();
            }
        };
    }
}
