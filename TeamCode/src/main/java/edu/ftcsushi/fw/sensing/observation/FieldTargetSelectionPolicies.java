package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;

/**
 * Factory-only geometric policies for {@link TargetSelections#fromRecentFieldLocations}.
 *
 * <p>Unlike visible-frame rules, these compare field coordinates from differently aged sightings.
 * They do not compare each image's different robot-at-capture origin. Factory calls validate
 * configuration without reading memory, localization, or hardware. Cluster, capacity, approach,
 * and collection decisions are not part of these single-location ranking rules.</p>
 */
public final class FieldTargetSelectionPolicies {
    private FieldTargetSelectionPolicies() { }

    /**
     * Chooses the nearest remembered field location inside an inclusive finite non-negative
     * radius. All arguments are in field inches; no current localization is needed to rank.
     * This radius limits selection, not the memory owner's independent association radius.
     */
    public static FieldTargetSelectionPolicy nearFieldPoint(double fieldXInches,
            double fieldYInches, double maxDistanceInches) {
        finite("fieldXInches", fieldXInches);
        finite("fieldYInches", fieldYInches);
        nonnegative("maxDistanceInches", maxDistanceInches);
        return new FieldTargetSelectionPolicy("nearest remembered field point within radius",
                clock -> new FieldTargetSelectionPolicy.Ranking(entry -> {
                    TargetObservation2d sighting = entry.lastSighting();
                    double distance = Math.hypot(sighting.fieldXInches - fieldXInches,
                            sighting.fieldYInches - fieldYInches);
                    return distance <= maxDistanceInches ? distance : Double.NaN;
                }, null, null));
    }

    /**
     * Chooses the minimum planar distance from the robot's already-published field position.
     * Reads {@code localization.getEstimate()} once per successful calculation with eligible
     * entries; never updates or resets the estimator. A failed value calculation may retry.
     *
     * <p>The pose must be available, have finite geometry and a finite quality in
     * {@code [minPoseQuality, 1]}, and pass the inclusive {@code poseAgeSec} bound. Missing,
     * stale, future, foreign-clock, or invalid pose evidence gives no selection, not a fallback
     * to the old robot-at-capture position. The exact pose is retained as ranking evidence;
     * downstream field solving still uses its own explicitly selected localization evidence.</p>
     *
     * @param localization borrowed already-updated absolute pose owner
     * @param poseAgeSec finite non-negative maximum pose-evidence age, seconds
     * @param minPoseQuality finite minimum producer-specific quality in {@code [0, 1]}
     */
    public static FieldTargetSelectionPolicy nearestToRobot(AbsolutePoseEstimator localization,
            double poseAgeSec, double minPoseQuality) {
        Objects.requireNonNull(localization, "localization");
        nonnegative("poseAgeSec", poseAgeSec);
        if (!Double.isFinite(minPoseQuality) || minPoseQuality < 0 || minPoseQuality > 1) {
            throw new IllegalArgumentException("minPoseQuality must be finite and in [0, 1]");
        }
        return new FieldTargetSelectionPolicy("nearest current robot field position", clock -> {
            PoseEstimate pose = localization.getEstimate();
            if (pose == null || !pose.hasPose || !finitePose(pose.fieldToRobotPose)
                    || !freshPose(pose, clock, poseAgeSec)
                    || !Double.isFinite(pose.quality) || pose.quality < minPoseQuality
                    || pose.quality > 1) {
                return new FieldTargetSelectionPolicy.Ranking(entry -> Double.NaN, pose,
                        "ranking pose unavailable, stale, reset, future, or below required quality");
            }
            return new FieldTargetSelectionPolicy.Ranking(entry -> Math.hypot(
                    entry.lastSighting().fieldXInches - pose.fieldToRobotPose.xInches,
                    entry.lastSighting().fieldYInches - pose.fieldToRobotPose.yInches), pose, null);
        });
    }

    /** Rejects wrong-clock pose evidence as ineligible without retrying or touching its owner. */
    private static boolean freshPose(PoseEstimate pose, LoopClock clock, double maxAgeSec) {
        try {
            return pose.timestamp.isFresh(clock, maxAgeSec);
        } catch (IllegalArgumentException foreignClock) {
            return false; // maxAgeSec was validated before this value calculation.
        }
    }

    /** Validates complete pose geometry before accepting it as ranking evidence. */
    private static boolean finitePose(Pose3d pose) {
        return Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches) && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad) && Double.isFinite(pose.rollRad);
    }

    /** Rejects non-finite coordinate and bound configuration at construction. */
    private static void finite(String name, double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
    }

    /** Validates inclusive ages and radii; zero is a meaningful bound. */
    private static void nonnegative(String name, double value) {
        finite(name, value);
        if (value < 0) throw new IllegalArgumentException(name + " must be >= 0");
    }
}
