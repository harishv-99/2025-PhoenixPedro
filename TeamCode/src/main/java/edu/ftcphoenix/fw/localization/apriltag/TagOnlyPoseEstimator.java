package edu.ftcphoenix.fw.localization.apriltag;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.spatial.Region2d;

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
 * behaviorally aligned. This estimator then turns the solve into a {@link PoseEstimate} and may
 * age the reported quality down as the detections frame approaches the configured freshness limit.</p>
 */
public final class TagOnlyPoseEstimator implements PoseEstimator {

    /** Configuration parameters for {@link TagOnlyPoseEstimator}. */
    public static final class Config extends FixedTagFieldPoseSolver.Config {
        /**
         * Maximum age (seconds) of the underlying detections frame accepted by this estimator.
         *
         * <p>This is the AprilTag-localization counterpart to guidance's live-tag freshness gate:
         * stale frames should not continue to produce a "global" robot pose just because the
         * detections object is still non-null. Values must be finite and >= 0.</p>
         */
        public double maxDetectionAgeSec = 0.50;

        /** Camera mount extrinsics in the robot frame. */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        /**
         * Quality scale applied when a detections frame has reached {@link #maxDetectionAgeSec}.
         *
         * <p>The final AprilTag-localizer quality linearly interpolates from {@code 1.0} for a
         * fresh frame to this value at the maximum accepted age. Set to {@code 1.0} to disable
         * freshness-based quality decay.</p>
         */
        public double qualityScaleAtMaxDetectionAge = 0.25;

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
         * Sets the maximum accepted detections-frame age in seconds.
         *
         * <p>The value must be finite and >= 0.</p>
         */
        public Config withMaxDetectionAgeSec(double maxDetectionAgeSec) {
            this.maxDetectionAgeSec = maxDetectionAgeSec;
            return this;
        }

        /**
         * Sets an optional field-region plausibility gate for AprilTag global pose solves.
         */
        public Config withPlausibleFieldRegion(Region2d region) {
            this.plausibleFieldRegion = region;
            return this;
        }

        /**
         * Sets how far outside the plausible field region a solve may drift before it is rejected.
         */
        public Config withMaxOutsidePlausibleFieldRegionInches(double maxOutsideInches) {
            this.maxOutsidePlausibleFieldRegionInches = maxOutsideInches;
            return this;
        }

        /**
         * Sets the quality scale applied when a detections frame reaches the maximum accepted age.
         *
         * <p>The value must lie in {@code [0, 1]}. Use {@code 1.0} to disable freshness-based
         * quality decay.</p>
         */
        public Config withQualityScaleAtMaxDetectionAge(double qualityScaleAtMaxDetectionAge) {
            this.qualityScaleAtMaxDetectionAge = qualityScaleAtMaxDetectionAge;
            return this;
        }

        /**
         * Returns a pure {@link FixedTagFieldPoseSolver.Config} snapshot of the shared solver
         * settings contained in this config.
         *
         * <p>Use this when one robot wants AprilTag-only localization and Drive Guidance's
         * temporary AprilTag field-pose bridge to share the same weighting / plausibility policy
         * without leaking TagOnly-specific settings such as {@link #cameraMount} or
         * {@link #maxDetectionAgeSec} into the guidance API.</p>
         */
        public FixedTagFieldPoseSolver.Config toSolverConfig() {
            return FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                    this,
                    "TagOnlyPoseEstimator.Config.toSolverConfig"
            );
        }

        /**
         * Backward-friendly alias for {@link #toSolverConfig()}.
         */
        public FixedTagFieldPoseSolver.Config solverConfig() {
            return toSolverConfig();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void validate(String context) {
            super.validate(context);
            String p = (context != null && !context.trim().isEmpty())
                    ? context.trim()
                    : "TagOnlyPoseEstimator.Config";
            if (!Double.isFinite(maxDetectionAgeSec) || maxDetectionAgeSec < 0.0) {
                throw new IllegalArgumentException(p + ".maxDetectionAgeSec must be finite and >= 0");
            }
            if (!Double.isFinite(qualityScaleAtMaxDetectionAge)
                    || qualityScaleAtMaxDetectionAge < 0.0
                    || qualityScaleAtMaxDetectionAge > 1.0) {
                throw new IllegalArgumentException(
                        p + ".qualityScaleAtMaxDetectionAge must be finite and within [0, 1]"
                );
            }
        }

        /**
         * Returns a shallow validated copy of this config.
         */
        @Override
        public Config validatedCopy(String context) {
            Config c = copy();
            c.validate(context);
            return c;
        }

        /**
         * Returns a shallow copy of this config.
         */
        @Override
        public Config copy() {
            Config c = new Config();
            copyBaseFieldsInto(c);
            c.maxDetectionAgeSec = this.maxDetectionAgeSec;
            c.cameraMount = this.cameraMount;
            c.qualityScaleAtMaxDetectionAge = this.qualityScaleAtMaxDetectionAge;
            return c;
        }
    }

    private final AprilTagSensor tags;
    private final TagLayout layout;
    private final Config cfg;

    private PoseEstimate lastEstimate;
    private AprilTagDetections lastDetections = AprilTagDetections.none();
    private FixedTagFieldPoseSolver.Result lastSolve = FixedTagFieldPoseSolver.Result.none();
    private double lastFreshnessQualityScale = 0.0;

    /**
     * Creates an AprilTag-only pose estimator that may use multiple visible fixed tags from the
     * same frame.
     */
    public TagOnlyPoseEstimator(AprilTagSensor tags, TagLayout layout, Config cfg) {
        this.tags = Objects.requireNonNull(tags, "tags");
        this.layout = Objects.requireNonNull(layout, "layout");
        Config base = (cfg != null) ? cfg : Config.defaults();
        this.cfg = base.validatedCopy("TagOnlyPoseEstimator.Config");
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
        lastFreshnessQualityScale = 0.0;

        if (lastDetections == null || !lastDetections.isFresh(cfg.maxDetectionAgeSec)) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        AprilTagDetections freshDetections = AprilTagDetections.of(
                lastDetections.ageSec,
                lastDetections.freshMatching(layout.ids(), cfg.maxDetectionAgeSec)
        );

        if (freshDetections.observations.isEmpty()) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        lastSolve = FixedTagFieldPoseSolver.solve(
                freshDetections.observations,
                layout,
                cfg.cameraMount,
                cfg
        );

        if (!lastSolve.hasPose) {
            lastEstimate = PoseEstimate.noPose(nowSec);
            return;
        }

        double ageSec = freshDetections.ageSec;
        double timestampSec = nowSec - ageSec;
        Pose3d fieldToRobotPose = lastSolve.fieldToRobotPose;
        lastFreshnessQualityScale = freshnessQualityScale(ageSec);
        double finalQuality = MathUtil.clamp01(lastSolve.quality * lastFreshnessQualityScale);
        lastEstimate = new PoseEstimate(fieldToRobotPose, true, finalQuality, ageSec, timestampSec);
    }

    private double freshnessQualityScale(double ageSec) {
        if (!Double.isFinite(ageSec)) {
            return 0.0;
        }
        if (cfg.maxDetectionAgeSec <= 1e-9) {
            return 1.0;
        }
        double t = MathUtil.clamp01(ageSec / cfg.maxDetectionAgeSec);
        return MathUtil.lerp(1.0, cfg.qualityScaleAtMaxDetectionAge, t);
    }

    /**
     * Returns the immutable shared-solver result from the most recent update.
     */
    public FixedTagFieldPoseSolver.Result getLastSolveResult() {
        return lastSolve;
    }

    /**
     * Returns the freshness-based quality scale applied on the most recent update.
     */
    public double getLastFreshnessQualityScale() {
        return lastFreshnessQualityScale;
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
                .addData(p + ".cfg.maxDetectionAgeSec", cfg.maxDetectionAgeSec)
                .addData(p + ".solve.candidates", lastSolve.candidateCount)
                .addData(p + ".solve.accepted", lastSolve.acceptedCount)
                .addData(p + ".solve.acceptedFraction", lastSolve.acceptedFraction)
                .addData(p + ".solve.acceptedWeightFraction", lastSolve.acceptedWeightFraction)
                .addData(p + ".solve.totalWeight", lastSolve.totalWeight)
                .addData(p + ".solve.rangeInches", lastSolve.rangeInches)
                .addData(p + ".cfg.qualityScaleAtMaxDetectionAge", cfg.qualityScaleAtMaxDetectionAge)
                .addData(p + ".freshnessQualityScale", lastFreshnessQualityScale)
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
