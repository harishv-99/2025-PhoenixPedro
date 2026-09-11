package edu.ftcsushi.fw.spatial;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Immutable ordered spatial algorithms, created through one staged builder.
 *
 * <p>An absolute-pose lane reads an already updated localizer. A relative-AprilTag lane interprets
 * actual camera observations without estimating a field pose. Observed points retain their own
 * capture evidence. Ordering exposes alternative geometry for explicit inspection or priority;
 * it does not fuse poses, update localization, or command hardware.</p>
 */
public final class SpatialSolveSet {
    private final List<SpatialSolveLane> lanes;

    private SpatialSolveSet(List<SpatialSolveLane> lanes) {
        this.lanes = lanes;
    }

    /** Starts a builder that requires at least one algorithm before build is available. */
    public static FirstLaneStep builder() { return new Builder(); }

    /** Returns the immutable ordered algorithms. */
    public List<SpatialSolveLane> lanes() { return lanes; }

    /** Returns the number of algorithms. */
    public int size() { return lanes.size(); }

    /** Returns the algorithm at the requested zero-based index. */
    public SpatialSolveLane lane(int index) { return lanes.get(index); }

    /** Select the first algorithm; all supplied sources and estimators remain borrowed. */
    public interface FirstLaneStep {
        /** Adds a custom algorithm; its lifecycle remains with its supplier. */
        MoreLanesStep add(SpatialSolveLane lane);

        /** Reads field pose with a 0.50-second age limit and minimum producer quality 0.10. */
        MoreLanesStep absolutePose(AbsolutePoseEstimator estimator);

        /** Reads field pose with explicit finite age (seconds) and quality in [0, 1]. */
        MoreLanesStep absolutePose(AbsolutePoseEstimator estimator, double maxAgeSec, double minQuality);

        /** Interprets direct tag-relative geometry with a fixed mount and 0.50-second age limit. */
        MoreLanesStep relativeAprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount);

        /** Interprets direct tag-relative geometry with a fixed mount and finite age limit. */
        MoreLanesStep relativeAprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount,
                                       double maxAgeSec);

        /**
         * Interprets direct tag-relative geometry using mount history at the exposure timestamp.
         * The supplied source must provide that history; no current-mount fallback is supplied.
         */
        MoreLanesStep relativeAprilTags(AprilTagSensor sensor,
                                       TimeAwareSource<CameraMountConfig> cameraMount,
                                       double maxAgeSec);

        /** Interprets selected observed points in the robot frame at their capture time. */
        MoreLanesStep observedPoints();
    }

    /** Nonempty stage: more algorithms may be added, or the immutable set may be built. */
    public interface MoreLanesStep extends FirstLaneStep {
        /** Copies the declared algorithms into an immutable nonempty ordered set. */
        SpatialSolveSet build();
    }

    private static final class Builder implements FirstLaneStep, MoreLanesStep {
        private final ArrayList<SpatialSolveLane> lanes = new ArrayList<>();

        @Override public MoreLanesStep add(SpatialSolveLane lane) {
            lanes.add(Objects.requireNonNull(lane, "lane"));
            return this;
        }

        @Override public MoreLanesStep absolutePose(AbsolutePoseEstimator estimator) {
            return add(new AbsolutePoseSpatialSolveLane(estimator));
        }

        @Override public MoreLanesStep absolutePose(AbsolutePoseEstimator estimator,
                                                  double maxAgeSec, double minQuality) {
            return add(new AbsolutePoseSpatialSolveLane(estimator, maxAgeSec, minQuality));
        }

        @Override public MoreLanesStep relativeAprilTags(AprilTagSensor sensor,
                                                       CameraMountConfig cameraMount) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount));
        }

        @Override public MoreLanesStep relativeAprilTags(AprilTagSensor sensor,
                                                       CameraMountConfig cameraMount,
                                                       double maxAgeSec) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec));
        }

        @Override public MoreLanesStep relativeAprilTags(AprilTagSensor sensor,
                                                       TimeAwareSource<CameraMountConfig> cameraMount,
                                                       double maxAgeSec) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec));
        }

        @Override public MoreLanesStep observedPoints() {
            return add(new ObservedTargetSpatialSolveLane());
        }

        @Override public SpatialSolveSet build() {
            if (lanes.isEmpty()) throw new IllegalStateException("SpatialSolveSet requires a solve lane");
            return new SpatialSolveSet(Collections.unmodifiableList(new ArrayList<>(lanes)));
        }
    }

    @Override public String toString() { return "SpatialSolveSet{lanes=" + lanes + '}'; }
}
