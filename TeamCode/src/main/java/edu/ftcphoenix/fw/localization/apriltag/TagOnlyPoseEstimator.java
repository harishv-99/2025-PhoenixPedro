package edu.ftcphoenix.fw.localization.apriltag;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * {@link PoseEstimator} that derives a field-centric robot pose estimate from one or more fresh
 * AprilTag detections and a known {@link TagLayout}.
 *
 * <p>This estimator intentionally sits on the <em>raw detections</em> boundary rather than a
 * selected-tag helper. That keeps the layering clean:</p>
 * <ul>
 *   <li><b>selection</b> decides which tag a behavior should aim relative to,</li>
 *   <li><b>localization</b> may still use <em>all</em> visible fixed tags to reduce noise.</li>
 * </ul>
 *
 * <p>Internally the estimator delegates the actual multi-tag solve to
 * {@link FixedTagFieldPoseSolver}. That shared solver applies weighting, optional SDK-pose use,
 * and outlier rejection so localization and guidance's temporary AprilTag field-pose bridge stay
 * behaviorally aligned.</p>
 */
public final class TagOnlyPoseEstimator implements PoseEstimator {

    /** Configuration parameters for {@link TagOnlyPoseEstimator}. */
    public static final class Config extends FixedTagFieldPoseSolver.Config {
        /** Camera mount extrinsics in the robot frame. */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        private Config() {
            // Defaults assigned in field initializers and base class fields.
        }

        /**
         * Returns a fresh config initialized with framework defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Sets the camera mount extrinsics used to convert camera observations into robot-frame
         * poses.
         */
        public Config withCameraMount(CameraMountConfig mount) {
            this.cameraMount = mount;
            return this;
        }

        /**
         * Returns a shallow copy of this config.
         */
        @Override
        public Config copy() {
            Config c = new Config();
            c.maxAbsBearingRad = this.maxAbsBearingRad;
            c.preferObservationFieldPose = this.preferObservationFieldPose;
            c.observationFieldPoseMaxDeltaInches = this.observationFieldPoseMaxDeltaInches;
            c.observationFieldPoseMaxDeltaHeadingRad = this.observationFieldPoseMaxDeltaHeadingRad;
            c.rangeSoftnessInches = this.rangeSoftnessInches;
            c.minObservationWeight = this.minObservationWeight;
            c.outlierPositionGateInches = this.outlierPositionGateInches;
            c.outlierHeadingGateRad = this.outlierHeadingGateRad;
            c.consistencyPositionScaleInches = this.consistencyPositionScaleInches;
            c.consistencyHeadingScaleRad = this.consistencyHeadingScaleRad;
            c.cameraMount = this.cameraMount;
            return c;
        }
    }

    private final AprilTagSensor tags;
    private final TagLayout layout;
    private final Config cfg;

    private PoseEstimate lastEstimate;
    private AprilTagDetections lastDetections = AprilTagDetections.none();
    private FixedTagFieldPoseSolver.Result lastSolve = FixedTagFieldPoseSolver.Result.none();

    /**
     * Creates an AprilTag-only pose estimator that may use multiple visible fixed tags from the
     * same frame.
     */
    public TagOnlyPoseEstimator(AprilTagSensor tags, TagLayout layout, Config cfg) {
        this.tags = Objects.requireNonNull(tags, "tags");
        this.layout = Objects.requireNonNull(layout, "layout");
        this.cfg = (cfg != null) ? cfg : Config.defaults();
        if (this.cfg.cameraMount == null) {
            this.cfg.cameraMount = CameraMountConfig.identity();
        }
        this.lastEstimate = PoseEstimate.noPose(0.0);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        final double nowSec = clock.nowSec();
        lastDetections = tags.get(clock);
        lastSolve = FixedTagFieldPoseSolver.Result.none();

        if (lastDetections == null || !lastDetections.isFresh(Double.POSITIVE_INFINITY)) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        lastSolve = FixedTagFieldPoseSolver.solve(
                lastDetections.observations,
                layout,
                cfg.cameraMount,
                cfg
        );

        if (!lastSolve.hasPose) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        double ageSec = lastDetections.ageSec;
        double timestampSec = nowSec - ageSec;
        Pose3d fieldToRobotPose = lastSolve.fieldToRobotPose;
        lastEstimate = new PoseEstimate(fieldToRobotPose, true, lastSolve.quality, ageSec, timestampSec);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagPose" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName())
                .addData(p + ".detections.ageSec", lastDetections.ageSec)
                .addData(p + ".detections.count", lastDetections.observations.size())
                .addData(p + ".solve.candidates", lastSolve.candidateCount)
                .addData(p + ".solve.accepted", lastSolve.acceptedCount)
                .addData(p + ".solve.totalWeight", lastSolve.totalWeight)
                .addData(p + ".solve.rangeInches", lastSolve.rangeInches)
                .addData(p + ".hasPose", lastEstimate.hasPose)
                .addData(p + ".quality", lastEstimate.quality)
                .addData(p + ".timestampSec", lastEstimate.timestampSec);

        if (lastEstimate.hasPose && lastEstimate.fieldToRobotPose != null) {
            Pose3d est = lastEstimate.fieldToRobotPose;
            dbg.addData(p + ".fieldToRobotPose.xInches", est.xInches)
                    .addData(p + ".fieldToRobotPose.yInches", est.yInches)
                    .addData(p + ".fieldToRobotPose.zInches", est.zInches)
                    .addData(p + ".fieldToRobotPose.yawRad", est.yawRad)
                    .addData(p + ".fieldToRobotPose.pitchRad", est.pitchRad)
                    .addData(p + ".fieldToRobotPose.rollRad", est.rollRad);
        }

        for (int i = 0; i < lastSolve.acceptedContributions.size(); i++) {
            FixedTagFieldPoseSolver.Contribution c = lastSolve.acceptedContributions.get(i);
            String q = p + ".accepted[" + i + "]";
            dbg.addData(q + ".id", c.observation.id)
                    .addData(q + ".ageSec", c.observation.ageSec)
                    .addData(q + ".weight", c.weight)
                    .addData(q + ".usedObservationFieldPose", c.usedObservationFieldPose)
                    .addData(q + ".cameraBearingRad", c.observation.cameraBearingRad())
                    .addData(q + ".cameraRangeInches", c.observation.cameraRangeInches())
                    .addData(q + ".fieldToRobotPose", c.fieldToRobotPose);
        }
    }
}
