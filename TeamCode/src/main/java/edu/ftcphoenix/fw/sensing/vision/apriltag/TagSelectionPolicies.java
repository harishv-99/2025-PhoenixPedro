package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;

/**
 * Common {@link TagSelectionPolicy} factories.
 */
public final class TagSelectionPolicies {

    private TagSelectionPolicies() {
        // Utility class.
    }

    /**
     * Chooses the visible tag with the smallest 3D line-of-sight range.
     */
    public static TagSelectionPolicy closestRange() {
        return new TagSelectionPolicy() {
            /**
             * {@inheritDoc}
             */
            @Override
            public TagSelectionChoice choose(List<AprilTagObservation> candidates) {
                AprilTagObservation best = null;
                double bestMetric = Double.POSITIVE_INFINITY;
                for (AprilTagObservation obs : candidates) {
                    if (obs == null || !obs.hasTarget) continue;
                    double metric = obs.cameraRangeInches();
                    if (metric < bestMetric) {
                        bestMetric = metric;
                        best = obs;
                    }
                }
                return best != null ? new TagSelectionChoice(best, "closestRange", "closest line-of-sight range", bestMetric) : null;
            }
        };
    }

    /**
     * Chooses the visible tag whose camera-frame bearing is closest to the image center.
     */
    public static TagSelectionPolicy smallestAbsCameraBearing() {
        return new TagSelectionPolicy() {
            /**
             * {@inheritDoc}
             */
            @Override
            public TagSelectionChoice choose(List<AprilTagObservation> candidates) {
                AprilTagObservation best = null;
                double bestMetric = Double.POSITIVE_INFINITY;
                for (AprilTagObservation obs : candidates) {
                    if (obs == null || !obs.hasTarget) continue;
                    double metric = Math.abs(obs.cameraBearingRad());
                    if (metric < bestMetric) {
                        bestMetric = metric;
                        best = obs;
                    }
                }
                return best != null ? new TagSelectionChoice(best, "smallestAbsCameraBearing", "closest to camera centerline", bestMetric) : null;
            }
        };
    }

    /**
     * Chooses the visible tag whose robot-frame bearing is closest to straight ahead.
     *
     * <p>This is often the most intuitive policy for driver aim assist because it accounts for
     * camera offset and answers “which tag is the robot most directly facing?”</p>
     */
    public static TagSelectionPolicy smallestAbsRobotBearing(CameraMountConfig mount) {
        Objects.requireNonNull(mount, "mount");
        return new TagSelectionPolicy() {
            /**
             * {@inheritDoc}
             */
            @Override
            public TagSelectionChoice choose(List<AprilTagObservation> candidates) {
                AprilTagObservation best = null;
                double bestMetric = Double.POSITIVE_INFINITY;
                for (AprilTagObservation obs : candidates) {
                    if (obs == null || !obs.hasTarget) continue;
                    double metric = Math.abs(CameraMountLogic.robotBearingRad(obs, mount));
                    if (metric < bestMetric) {
                        bestMetric = metric;
                        best = obs;
                    }
                }
                return best != null ? new TagSelectionChoice(best, "smallestAbsRobotBearing", "closest to robot forward axis", bestMetric) : null;
            }
        };
    }

    /**
     * Chooses the first visible tag according to the supplied priority order.
     */
    public static TagSelectionPolicy priorityOrder(List<Integer> orderedIds) {
        Objects.requireNonNull(orderedIds, "orderedIds");
        final Map<Integer, Integer> rank = new HashMap<Integer, Integer>();
        for (int i = 0; i < orderedIds.size(); i++) {
            Integer id = orderedIds.get(i);
            if (id != null && !rank.containsKey(id)) {
                rank.put(id, i);
            }
        }
        return new TagSelectionPolicy() {
            /**
             * {@inheritDoc}
             */
            @Override
            public TagSelectionChoice choose(List<AprilTagObservation> candidates) {
                AprilTagObservation best = null;
                double bestMetric = Double.POSITIVE_INFINITY;
                for (AprilTagObservation obs : candidates) {
                    if (obs == null || !obs.hasTarget) continue;
                    Integer r = rank.get(obs.id);
                    if (r == null) continue;
                    double metric = r.doubleValue();
                    if (metric < bestMetric) {
                        bestMetric = metric;
                        best = obs;
                    }
                }
                return best != null ? new TagSelectionChoice(best, "priorityOrder", "first visible tag in configured priority order", bestMetric) : null;
            }
        };
    }
}
