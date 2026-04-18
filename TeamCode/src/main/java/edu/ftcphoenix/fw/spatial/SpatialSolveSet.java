package edu.ftcphoenix.fw.spatial;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Immutable ordered collection of {@link SpatialSolveLane}s.
 *
 * <p>The order is significant. A consumer may choose to interpret the first lane as a primary
 * solve source and the second lane as a secondary source, or it may simply inspect all results in
 * order for telemetry and decision-making.</p>
 */
public final class SpatialSolveSet {

    private final List<SpatialSolveLane> lanes;

    private SpatialSolveSet(List<SpatialSolveLane> lanes) {
        this.lanes = lanes;
    }

    /**
     * Starts building a solve set.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Returns the immutable ordered lane list.
     */
    public List<SpatialSolveLane> lanes() {
        return lanes;
    }

    /**
     * Returns how many solve lanes are present.
     */
    public int size() {
        return lanes.size();
    }

    /**
     * Returns one solve lane by its ordered index.
     */
    public SpatialSolveLane lane(int index) {
        return lanes.get(index);
    }

    /**
     * Builder for immutable solve sets.
     */
    public static final class Builder {
        private final ArrayList<SpatialSolveLane> lanes = new ArrayList<SpatialSolveLane>();

        /**
         * Adds one explicit solve lane in evaluation order.
         */
        public Builder add(SpatialSolveLane lane) {
            lanes.add(Objects.requireNonNull(lane, "lane"));
            return this;
        }

        /**
         * Adds an absolute-pose-backed solve lane with default gating.
         */
        public Builder absolutePose(AbsolutePoseEstimator estimator) {
            return add(new AbsolutePoseSpatialSolveLane(estimator));
        }

        /**
         * Adds an absolute-pose-backed solve lane with explicit freshness and quality gates.
         */
        public Builder absolutePose(AbsolutePoseEstimator estimator,
                                    double maxAgeSec,
                                    double minQuality) {
            return add(new AbsolutePoseSpatialSolveLane(estimator, maxAgeSec, minQuality));
        }

        /**
         * Adds a live-AprilTag solve lane with default freshness and field-pose bridge settings.
         */
        public Builder aprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount));
        }

        /**
         * Adds a live-AprilTag solve lane with an explicit maximum tag age.
         */
        public Builder aprilTags(AprilTagSensor sensor,
                                 CameraMountConfig cameraMount,
                                 double maxAgeSec) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec));
        }

        /**
         * Adds a live-AprilTag solve lane with explicit freshness and fixed-tag field-pose bridge tuning.
         */
        public Builder aprilTags(AprilTagSensor sensor,
                                 CameraMountConfig cameraMount,
                                 double maxAgeSec,
                                 FixedTagFieldPoseSolver.Config fieldPoseSolverConfig) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec, fieldPoseSolverConfig));
        }

        /**
         * Builds the immutable solve set.
         */
        public SpatialSolveSet build() {
            return new SpatialSolveSet(Collections.unmodifiableList(new ArrayList<SpatialSolveLane>(lanes)));
        }
    }

    @Override
    public String toString() {
        return "SpatialSolveSet{lanes=" + lanes + '}';
    }
}
