package edu.ftcphoenix.fw.spatial;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.source.TimeAwareSource;
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
 *
 * <p>The builder is staged so an empty solve set cannot be built accidentally. Start with
 * {@link #builder()}, add at least one lane using {@link FirstLaneStep#add(SpatialSolveLane)},
 * {@link FirstLaneStep#absolutePose(AbsolutePoseEstimator)}, or one of the AprilTag helpers, and
 * then continue adding lanes or call {@link MoreLanesStep#build()}.</p>
 */
public final class SpatialSolveSet {

    private final List<SpatialSolveLane> lanes;

    private SpatialSolveSet(List<SpatialSolveLane> lanes) {
        this.lanes = lanes;
    }

    /**
     * Starts a staged solve-set builder.
     *
     * <p>The returned stage intentionally does not expose {@code build()}; a solve set with no
     * lanes is not meaningful, so the first conceptual question is which solve lane should be
     * evaluated first.</p>
     */
    public static FirstLaneStep builder() {
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
     * First solve-lane stage. Pick the first lane to evaluate.
     */
    public interface FirstLaneStep {
        /**
         * Adds one explicit solve lane in evaluation order.
         */
        MoreLanesStep add(SpatialSolveLane lane);

        /**
         * Adds an absolute-pose-backed solve lane with default gating.
         */
        MoreLanesStep absolutePose(AbsolutePoseEstimator estimator);

        /**
         * Adds an absolute-pose-backed solve lane with explicit freshness and quality gates.
         *
         * @param maxAgeSec  maximum accepted estimator age, in seconds
         * @param minQuality minimum accepted estimator quality in {@code [0, 1]}
         */
        MoreLanesStep absolutePose(AbsolutePoseEstimator estimator,
                                   double maxAgeSec,
                                   double minQuality);

        /**
         * Adds a live-AprilTag solve lane with default freshness and field-pose bridge settings.
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount);

        /**
         * Adds a live-AprilTag solve lane with an explicit maximum tag age, in seconds.
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                CameraMountConfig cameraMount,
                                double maxAgeSec);

        /**
         * Adds a live-AprilTag solve lane with a dynamic timestamp-aware robot->camera mount.
         *
         * <p>Use this for turret-mounted cameras or other sensors whose mount changes over time.</p>
         *
         * @param maxAgeSec maximum accepted tag age, in seconds
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                TimeAwareSource<CameraMountConfig> cameraMount,
                                double maxAgeSec);

        /**
         * Adds a live-AprilTag solve lane with explicit freshness and fixed-tag field-pose bridge tuning.
         *
         * @param maxAgeSec maximum accepted tag age, in seconds
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                CameraMountConfig cameraMount,
                                double maxAgeSec,
                                FixedTagFieldPoseSolver.Config fieldPoseSolverConfig);

        /**
         * Adds a live-AprilTag solve lane with a dynamic camera mount and explicit field-pose bridge tuning.
         *
         * @param maxAgeSec maximum accepted tag age, in seconds
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                TimeAwareSource<CameraMountConfig> cameraMount,
                                double maxAgeSec,
                                FixedTagFieldPoseSolver.Config fieldPoseSolverConfig);
    }

    /**
     * Non-empty solve-set stage. Additional lanes may be added, or the set can be built.
     */
    public interface MoreLanesStep {
        /**
         * Adds one explicit solve lane in evaluation order.
         */
        MoreLanesStep add(SpatialSolveLane lane);

        /**
         * Adds an absolute-pose-backed solve lane with default gating.
         */
        MoreLanesStep absolutePose(AbsolutePoseEstimator estimator);

        /**
         * Adds an absolute-pose-backed solve lane with explicit freshness and quality gates.
         *
         * @param maxAgeSec  maximum accepted estimator age, in seconds
         * @param minQuality minimum accepted estimator quality in {@code [0, 1]}
         */
        MoreLanesStep absolutePose(AbsolutePoseEstimator estimator,
                                   double maxAgeSec,
                                   double minQuality);

        /**
         * Adds a live-AprilTag solve lane with default freshness and field-pose bridge settings.
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount);

        /**
         * Adds a live-AprilTag solve lane with an explicit maximum tag age, in seconds.
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                CameraMountConfig cameraMount,
                                double maxAgeSec);

        /**
         * Adds a live-AprilTag solve lane with a dynamic timestamp-aware robot->camera mount.
         *
         * <p>Use this for turret-mounted cameras or other sensors whose mount changes over time.</p>
         *
         * @param maxAgeSec maximum accepted tag age, in seconds
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                TimeAwareSource<CameraMountConfig> cameraMount,
                                double maxAgeSec);

        /**
         * Adds a live-AprilTag solve lane with explicit freshness and fixed-tag field-pose bridge tuning.
         *
         * @param maxAgeSec maximum accepted tag age, in seconds
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                CameraMountConfig cameraMount,
                                double maxAgeSec,
                                FixedTagFieldPoseSolver.Config fieldPoseSolverConfig);

        /**
         * Adds a live-AprilTag solve lane with a dynamic camera mount and explicit field-pose bridge tuning.
         *
         * @param maxAgeSec maximum accepted tag age, in seconds
         */
        MoreLanesStep aprilTags(AprilTagSensor sensor,
                                TimeAwareSource<CameraMountConfig> cameraMount,
                                double maxAgeSec,
                                FixedTagFieldPoseSolver.Config fieldPoseSolverConfig);

        /**
         * Builds the immutable non-empty solve set.
         */
        SpatialSolveSet build();
    }

    private static final class Builder implements FirstLaneStep, MoreLanesStep {
        private final ArrayList<SpatialSolveLane> lanes = new ArrayList<SpatialSolveLane>();

        @Override
        public MoreLanesStep add(SpatialSolveLane lane) {
            lanes.add(Objects.requireNonNull(lane, "lane"));
            return this;
        }

        @Override
        public MoreLanesStep absolutePose(AbsolutePoseEstimator estimator) {
            return add(new AbsolutePoseSpatialSolveLane(estimator));
        }

        @Override
        public MoreLanesStep absolutePose(AbsolutePoseEstimator estimator,
                                          double maxAgeSec,
                                          double minQuality) {
            return add(new AbsolutePoseSpatialSolveLane(estimator, maxAgeSec, minQuality));
        }

        @Override
        public MoreLanesStep aprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount));
        }

        @Override
        public MoreLanesStep aprilTags(AprilTagSensor sensor,
                                       CameraMountConfig cameraMount,
                                       double maxAgeSec) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec));
        }

        @Override
        public MoreLanesStep aprilTags(AprilTagSensor sensor,
                                       TimeAwareSource<CameraMountConfig> cameraMount,
                                       double maxAgeSec) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec));
        }

        @Override
        public MoreLanesStep aprilTags(AprilTagSensor sensor,
                                       CameraMountConfig cameraMount,
                                       double maxAgeSec,
                                       FixedTagFieldPoseSolver.Config fieldPoseSolverConfig) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec, fieldPoseSolverConfig));
        }

        @Override
        public MoreLanesStep aprilTags(AprilTagSensor sensor,
                                       TimeAwareSource<CameraMountConfig> cameraMount,
                                       double maxAgeSec,
                                       FixedTagFieldPoseSolver.Config fieldPoseSolverConfig) {
            return add(new AprilTagSpatialSolveLane(sensor, cameraMount, maxAgeSec, fieldPoseSolverConfig));
        }

        @Override
        public SpatialSolveSet build() {
            if (lanes.isEmpty()) {
                throw new IllegalStateException("SpatialSolveSet requires at least one solve lane");
            }
            return new SpatialSolveSet(Collections.unmodifiableList(new ArrayList<SpatialSolveLane>(lanes)));
        }
    }

    @Override
    public String toString() {
        return "SpatialSolveSet{lanes=" + lanes + '}';
    }
}
