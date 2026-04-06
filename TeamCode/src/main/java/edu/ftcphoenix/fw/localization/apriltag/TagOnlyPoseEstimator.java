package edu.ftcphoenix.fw.localization.apriltag;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
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
 * <p>The implementation is deliberately simple and predictable for early-season use:
 * each valid fixed-tag observation is converted into a field-centric robot pose estimate using the
 * rigid-transform chain {@code fieldToTag ∘ (robotToCamera ∘ cameraToTag)^(-1)}. The resulting
 * planar poses are then averaged across the current camera frame.</p>
 */
public final class TagOnlyPoseEstimator implements PoseEstimator {

    /** Configuration parameters for {@link TagOnlyPoseEstimator}. */
    public static final class Config {
        /** Optional maximum absolute bearing (radians) for an observation to be trusted. */
        public double maxAbsBearingRad = 0.0;

        /** Camera mount extrinsics in the robot frame. */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        private Config() {
            // Defaults assigned in field initializers.
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
        public Config copy() {
            Config c = new Config();
            c.maxAbsBearingRad = this.maxAbsBearingRad;
            c.cameraMount = this.cameraMount;
            return c;
        }
    }

    private final AprilTagSensor tags;
    private final TagLayout layout;
    private final Config cfg;

    private PoseEstimate lastEstimate;
    private AprilTagDetections lastDetections = AprilTagDetections.none();
    private final List<AprilTagObservation> lastAccepted = new ArrayList<AprilTagObservation>();

    /**
     * Creates a simple AprilTag-only pose estimator that may use multiple visible fixed tags from
     * the same frame.
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
        lastAccepted.clear();
        lastDetections = tags.get(clock);

        if (lastDetections == null || !lastDetections.isFresh(Double.POSITIVE_INFINITY)) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumSinYaw = 0.0;
        double sumCosYaw = 0.0;
        int count = 0;

        for (AprilTagObservation obs : lastDetections.observations) {
            if (obs == null || !obs.hasTarget || !layout.has(obs.id)) {
                continue;
            }

            double bearingRad = obs.cameraBearingRad();
            if (cfg.maxAbsBearingRad > 0.0 && Math.abs(bearingRad) > cfg.maxAbsBearingRad) {
                continue;
            }

            Pose3d fieldToTagPose = layout.requireFieldToTagPose(obs.id);
            Pose3d robotToCameraPose = cfg.cameraMount.robotToCameraPose();
            Pose3d robotToTagPose = robotToCameraPose.then(obs.cameraToTagPose);
            Pose3d fieldToRobot6DofPose = fieldToTagPose.then(robotToTagPose.inverse());

            double yawRad = Pose2d.wrapToPi(fieldToRobot6DofPose.yawRad);
            sumX += fieldToRobot6DofPose.xInches;
            sumY += fieldToRobot6DofPose.yInches;
            sumSinYaw += Math.sin(yawRad);
            sumCosYaw += Math.cos(yawRad);
            lastAccepted.add(obs);
            count++;
        }

        if (count <= 0) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        double avgX = sumX / count;
        double avgY = sumY / count;
        double avgYaw = Math.atan2(sumSinYaw / count, sumCosYaw / count);
        Pose3d fieldToRobotPose = new Pose3d(avgX, avgY, 0.0, avgYaw, 0.0, 0.0);

        double ageSec = lastDetections.ageSec;
        double timestampSec = nowSec - ageSec;
        double quality = Math.min(1.0, count / 3.0);
        lastEstimate = new PoseEstimate(fieldToRobotPose, true, quality, ageSec, timestampSec);
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
                .addData(p + ".accepted.count", lastAccepted.size())
                .addData(p + ".hasPose", lastEstimate.hasPose)
                .addData(p + ".quality", lastEstimate.quality)
                .addData(p + ".timestampSec", lastEstimate.timestampSec);

        if (lastEstimate.hasPose && lastEstimate.fieldToRobotPose != null) {
            Pose3d est = lastEstimate.fieldToRobotPose;
            dbg.addData(p + ".fieldToRobotPose.xInches", est.xInches)
                    .addData(p + ".fieldToRobotPose.yInches", est.yInches)
                    .addData(p + ".fieldToRobotPose.yawRad", est.yawRad);
        }

        for (int i = 0; i < lastAccepted.size(); i++) {
            AprilTagObservation obs = lastAccepted.get(i);
            String q = p + ".accepted[" + i + "]";
            dbg.addData(q + ".id", obs.id)
                    .addData(q + ".ageSec", obs.ageSec)
                    .addData(q + ".cameraBearingRad", obs.cameraBearingRad())
                    .addData(q + ".cameraRangeInches", obs.cameraRangeInches());
        }
    }
}
