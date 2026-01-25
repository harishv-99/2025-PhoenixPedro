package edu.ftcphoenix.fw.localization.apriltag;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.field.TagLayout.TagPose;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;

/**
 * {@link PoseEstimator} that derives a field-centric robot pose estimate from:
 * <ul>
 *   <li>a tracked {@link TagTarget} (best AprilTag observation for a chosen ID set), and</li>
 *   <li>a known {@link TagLayout} (tag placements in the FTC field frame).</li>
 * </ul>
 *
 * <h2>Why this uses {@link TagTarget}</h2>
 * <p>
 * {@code TagTarget} is the framework’s single “best tag selection” helper. By depending on
 * {@code TagTarget}, this estimator shares the same observation as other features (aiming,
 * shooter decisions, telemetry) and avoids duplicated sensor queries with mismatched filters.
 * </p>
 *
 * <h3>Loop ordering requirement</h3>
 * <p>
 * {@link TagTarget} is intended to be updated exactly once per control loop. Therefore,
 * robot code should call:
 * </p>
 * <ol>
 *   <li>{@code tagTarget.update()}</li>
 *   <li>{@code poseEstimator.update(clock)}</li>
 * </ol>
 *
 * <h2>Planar approximation</h2>
 * <p>
 * {@link AprilTagObservation} contains {@link AprilTagObservation#cameraToTagPose} (a 6DOF pose in the
 * Phoenix camera frame). This estimator uses the full rigid-transform chain to compute a field-centric
 * robot pose, but it intentionally outputs a <b>planar approximation</b> for drivetrain use:
 * </p>
 * <ul>
 *   <li>It computes {@code fieldToRobotPose} using the transform chain:
 *       {@code fieldToTagPose ∘ (robotToCameraPose ∘ cameraToTagPose)^(-1)}.</li>
 *   <li>It then projects the result into the floor plane by setting {@code z=0} and {@code pitch/roll=0}.</li>
 * </ul>
 *
 * <p>
 * This keeps the implementation simple and robust for early-season use. Teams that need higher accuracy
 * can replace this with a true solve (multi-tag fusion, full 6DOF, covariance weighting, etc.).
 * </p>
 *
 * <h2>Camera mount</h2>
 * <p>
 * The camera mount is provided as {@link CameraMountConfig} and is expressed in the robot frame:
 * </p>
 * <ul>
 *   <li>Origin: robot center of rotation on the floor plane (z=0)</li>
 *   <li><b>+X</b> forward, <b>+Y</b> left, <b>+Z</b> up</li>
 * </ul>
 */
public final class TagOnlyPoseEstimator implements PoseEstimator {

    /**
     * Configuration parameters for {@link TagOnlyPoseEstimator}.
     */
    public static final class Config {

        /**
         * Optional maximum absolute bearing (radians) for an observation to be trusted for pose estimation.
         *
         * <ul>
         *   <li>If {@code maxAbsBearingRad <= 0}, bearing is not used as a filter.</li>
         *   <li>If positive, observations where {@code |bearingRad| > maxAbsBearingRad} are ignored.</li>
         * </ul>
         */
        public double maxAbsBearingRad = 0.0;

        /**
         * Camera mount extrinsics in the robot frame.
         */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Create a new configuration instance with Phoenix defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Convenience helper: set the camera mount extrinsics.
         *
         * <p>
         * This uses the framework-wide config naming convention: optional fluent helpers are
         * named {@code withX(...)} / {@code withoutX()} (never {@code useX(...)}).
         * </p>
         */
        public Config withCameraMount(CameraMountConfig mount) {
            this.cameraMount = mount;
            return this;
        }

        /**
         * Deep copy of this configuration.
         */
        public Config copy() {
            Config c = new Config();
            c.maxAbsBearingRad = this.maxAbsBearingRad;
            c.cameraMount = this.cameraMount;
            return c;
        }
    }

    private final TagTarget target;
    private final TagLayout layout;
    private final Config cfg;

    // Last estimate we produced; always non-null.
    private PoseEstimate lastEstimate;

    // Last observation we examined (for debug).
    private AprilTagObservation lastObs = AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);

    /**
     * Creates a new {@code TagOnlyPoseEstimator}.
     *
     * @param target tracked tag target (must be updated once per loop before this estimator update)
     * @param layout field tag layout providing tag poses
     * @param cfg    configuration parameters (may be {@code null} to use {@link Config#defaults()})
     */
    public TagOnlyPoseEstimator(TagTarget target, TagLayout layout, Config cfg) {
        this.target = Objects.requireNonNull(target, "target");
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

        // Default to "no pose" this loop.
        PoseEstimate estimate = PoseEstimate.noPose(nowSec);

        // IMPORTANT: TagTarget.update() should already have been called this loop.
        AprilTagObservation obs = target.last();
        lastObs = obs;

        if (obs.hasTarget && layout.has(obs.id)) {

            // Optionally enforce a maximum bearing to ensure we are roughly in front of the tag.
            double bearingRad = obs.cameraBearingRad();
            if (cfg.maxAbsBearingRad > 0.0 && Math.abs(bearingRad) > cfg.maxAbsBearingRad) {
                this.lastEstimate = estimate;
                return;
            }

            // Known placement for this tag in the field frame.
            TagPose tag = layout.require(obs.id);

            // Compute pose using the full transform chain:
            //   fieldToRobotPose = fieldToTagPose ∘ (robotToCameraPose ∘ cameraToTagPose)^(-1)
            // This captures lateral offsets correctly and sets us up for odometry/tag fusion later.
            final Pose3d fieldToTagPose = tag.fieldToTagPose();
            final Pose3d robotToCameraPose = cfg.cameraMount.robotToCameraPose();
            final Pose3d cameraToTagPose = obs.cameraToTagPose;

            final Pose3d robotToTagPose = robotToCameraPose.then(cameraToTagPose);
            final Pose3d fieldToRobot6DofPose = fieldToTagPose.then(robotToTagPose.inverse());

            // Planar projection for drivetrain use: assume robot center is on the floor plane.
            final double fieldRobotYawRad = Pose2d.wrapToPi(fieldToRobot6DofPose.yawRad);
            final Pose3d fieldToRobotPose = new Pose3d(
                    fieldToRobot6DofPose.xInches,
                    fieldToRobot6DofPose.yInches,
                    0.0,
                    fieldRobotYawRad,
                    0.0,
                    0.0
            );

            // obs.ageSec is "how long since the frame was captured" at observation creation time.
            final double ageSec = obs.ageSec;
            final double timestampSec = nowSec - ageSec;

            // Simple estimator: treat quality as 1.0 when we have a valid tag.
            final double quality = 1.0;

            estimate = new PoseEstimate(fieldToRobotPose, true, quality, ageSec, timestampSec);
        }

        this.lastEstimate = estimate;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * Debug-dump this estimator's config and last computed state.
     *
     * <p>This method also delegates to {@link TagTarget#debugDump(DebugSink, String)} so the
     * tracked ID set and freshness window are visible in telemetry.</p>
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        final String p = (prefix == null || prefix.isEmpty()) ? "pose.tagOnly" : prefix;

        dbg.addData(p + ".cfg.maxAbsBearingRad", cfg.maxAbsBearingRad);

        CameraMountConfig cam = (cfg.cameraMount != null) ? cfg.cameraMount : CameraMountConfig.identity();
        dbg.addData(p + ".cfg.cameraMount.xInches", cam.xInches())
                .addData(p + ".cfg.cameraMount.yInches", cam.yInches())
                .addData(p + ".cfg.cameraMount.zInches", cam.zInches())
                .addData(p + ".cfg.cameraMount.yawRad", cam.yawRad())
                .addData(p + ".cfg.cameraMount.pitchRad", cam.pitchRad())
                .addData(p + ".cfg.cameraMount.rollRad", cam.rollRad());

        // TagTarget config + observation.
        target.debugDump(dbg, p + ".target");

        // Last estimate.
        dbg.addData(p + ".lastEstimate.hasPose", lastEstimate.hasPose);
        if (lastEstimate.fieldToRobotPose != null) {
            dbg.addData(p + ".lastEstimate.fieldToRobotPose.xInches", lastEstimate.fieldToRobotPose.xInches)
                    .addData(p + ".lastEstimate.fieldToRobotPose.yInches", lastEstimate.fieldToRobotPose.yInches)
                    .addData(p + ".lastEstimate.fieldToRobotPose.zInches", lastEstimate.fieldToRobotPose.zInches)
                    .addData(p + ".lastEstimate.fieldToRobotPose.yawRad", lastEstimate.fieldToRobotPose.yawRad)
                    .addData(p + ".lastEstimate.fieldToRobotPose.pitchRad", lastEstimate.fieldToRobotPose.pitchRad)
                    .addData(p + ".lastEstimate.fieldToRobotPose.rollRad", lastEstimate.fieldToRobotPose.rollRad);
        }
        dbg.addData(p + ".lastEstimate.quality", lastEstimate.quality)
                .addData(p + ".lastEstimate.ageSec", lastEstimate.ageSec)
                .addData(p + ".lastEstimate.timestampSec", lastEstimate.timestampSec);

        // Last raw obs seen by this estimator (for quick checking without expanding target dump).
        dbg.addData(p + ".lastObs.hasTarget", lastObs.hasTarget)
                .addData(p + ".lastObs.id", lastObs.id)
                .addData(p + ".lastObs.cameraBearingRad", lastObs.cameraBearingRad())
                .addData(p + ".lastObs.cameraRangeInches", lastObs.cameraRangeInches())
                .addData(p + ".lastObs.ageSec", lastObs.ageSec);

        // Optional: compare our estimate to an SDK-provided fieldToRobotPose (if the sensor supplies one).
        if (lastObs.hasFieldToRobotPose()) {
            Pose3d obsFieldToRobotPose = lastObs.fieldToRobotPose;
            dbg.addData(p + ".lastObs.fieldToRobotPose.xInches", obsFieldToRobotPose.xInches)
                    .addData(p + ".lastObs.fieldToRobotPose.yInches", obsFieldToRobotPose.yInches)
                    .addData(p + ".lastObs.fieldToRobotPose.zInches", obsFieldToRobotPose.zInches)
                    .addData(p + ".lastObs.fieldToRobotPose.yawRad", obsFieldToRobotPose.yawRad)
                    .addData(p + ".lastObs.fieldToRobotPose.pitchRad", obsFieldToRobotPose.pitchRad)
                    .addData(p + ".lastObs.fieldToRobotPose.rollRad", obsFieldToRobotPose.rollRad);

            if (lastEstimate.hasPose && lastEstimate.fieldToRobotPose != null) {
                Pose3d est = lastEstimate.fieldToRobotPose;

                // Compare against a planar-projected SDK pose. This avoids "expected" non-zero deltas
                // if the SDK reports a non-zero Z / pitch / roll but we intentionally publish a
                // drivetrain-friendly planar estimate.
                Pose3d obsPlanar = new Pose3d(
                        obsFieldToRobotPose.xInches,
                        obsFieldToRobotPose.yInches,
                        0.0,
                        Pose2d.wrapToPi(obsFieldToRobotPose.yawRad),
                        0.0,
                        0.0
                );

                double dx = est.xInches - obsPlanar.xInches;
                double dy = est.yInches - obsPlanar.yInches;
                double dYaw = MathUtil.wrapToPi(est.yawRad - obsPlanar.yawRad);
                double dXY = Math.hypot(dx, dy);

                dbg.addData(p + ".deltaToSdkPlanarPose.dxInches", dx)
                        .addData(p + ".deltaToSdkPlanarPose.dyInches", dy)
                        .addData(p + ".deltaToSdkPlanarPose.dXYInches", dXY)
                        .addData(p + ".deltaToSdkPlanarPose.dYawRad", dYaw);

                // Also expose raw 6DOF deltas (useful while validating frame conventions / extrinsics).
                double dz = est.zInches - obsFieldToRobotPose.zInches;
                double dPitch = MathUtil.wrapToPi(est.pitchRad - obsFieldToRobotPose.pitchRad);
                double dRoll = MathUtil.wrapToPi(est.rollRad - obsFieldToRobotPose.rollRad);
                double dXYZ = Math.sqrt(dx * dx + dy * dy + dz * dz);

                dbg.addData(p + ".deltaToSdkRawPose.dzInches", dz)
                        .addData(p + ".deltaToSdkRawPose.dXYZInches", dXYZ)
                        .addData(p + ".deltaToSdkRawPose.dPitchRad", dPitch)
                        .addData(p + ".deltaToSdkRawPose.dRollRad", dRoll);
            }
        }
    }
}
