package edu.ftcsushi.fw.sensing.vision.apriltag;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.ToDoubleFunction;

/** Source-independent tag ranking. Equal finite scores always choose the lower numeric tag ID. */
public final class TagSelectionPolicies {
    private TagSelectionPolicies() { }

    /** Chooses the smallest three-dimensional camera-origin to tag-center range, inches. */
    public static TagSelectionPolicy closestRange() {
        return ranking("closestRange", "closest camera-to-tag range", TagSelectionCandidate::cameraRangeInches);
    }

    /** Chooses the smallest absolute horizontal camera bearing, without an angle bound. */
    public static TagSelectionPolicy smallestAbsCameraBearing() {
        return bearing(true, Math.PI);
    }

    /** Chooses within an inclusive finite camera bearing bound, radians in [0, pi]. */
    public static TagSelectionPolicy smallestAbsCameraBearing(double maxAbsBearingRad) {
        requireBearingBound(maxAbsBearingRad);
        return bearing(true, maxAbsBearingRad);
    }

    /** Chooses the smallest absolute robot bearing using the source's one camera mount answer. */
    public static TagSelectionPolicy smallestAbsRobotBearing() {
        return bearing(false, Math.PI);
    }

    /** Chooses within an inclusive finite robot bearing bound, radians in [0, pi]. */
    public static TagSelectionPolicy smallestAbsRobotBearing(double maxAbsBearingRad) {
        requireBearingBound(maxAbsBearingRad);
        return bearing(false, maxAbsBearingRad);
    }

    /** Chooses the first eligible candidate in this defensively copied ID priority list. */
    public static TagSelectionPolicy priorityOrder(List<Integer> orderedIds) {
        Objects.requireNonNull(orderedIds, "orderedIds");
        Map<Integer, Integer> rank = new HashMap<>();
        for (int i = 0; i < orderedIds.size(); i++) {
            Integer id = orderedIds.get(i);
            if (id == null || id < 0) {
                throw new IllegalArgumentException("orderedIds must contain non-negative IDs");
            }
            if (!rank.containsKey(id)) rank.put(id, i);
        }
        return ranking("priorityOrder", "first eligible tag in configured priority order",
                candidate -> rank.containsKey(candidate.tagId) ? rank.get(candidate.tagId) : Double.NaN);
    }

    private static TagSelectionPolicy bearing(boolean camera, double bound) {
        return ranking(camera ? "smallestAbsCameraBearing" : "smallestAbsRobotBearing",
                camera ? "closest to camera centerline" : "closest to robot forward axis", candidate -> {
                    double bearing = Math.abs(camera ? candidate.cameraBearingRad() : candidate.robotBearingRad());
                    return bearing <= bound ? bearing : Double.NaN;
                });
    }

    private static void requireBearingBound(double bound) {
        if (!Double.isFinite(bound) || bound < 0 || bound > Math.PI) {
            throw new IllegalArgumentException("maxAbsBearingRad must be finite and in [0, pi]");
        }
    }

    private static TagSelectionPolicy ranking(String name, String reason,
                                              ToDoubleFunction<TagSelectionCandidate> metric) {
        return candidates -> {
            TagSelectionCandidate best = null;
            double bestMetric = Double.POSITIVE_INFINITY;
            for (TagSelectionCandidate candidate : candidates) {
                double score = metric.applyAsDouble(candidate);
                if (Double.isFinite(score) && (score < bestMetric
                        || (score == bestMetric && best != null && candidate.tagId < best.tagId))) {
                    best = candidate;
                    bestMetric = score;
                }
            }
            return best == null ? null : new TagSelectionChoice(best, name, reason, bestMetric);
        };
    }
}
