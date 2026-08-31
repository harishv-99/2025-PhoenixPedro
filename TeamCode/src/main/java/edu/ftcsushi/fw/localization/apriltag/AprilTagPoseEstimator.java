package edu.ftcsushi.fw.localization.apriltag;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.field.TagLayouts;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * {@link AbsolutePoseEstimator} that derives a field-centric robot pose estimate from one or more fresh
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
public final class AprilTagPoseEstimator implements AbsolutePoseEstimator {

    /**
     * Configuration parameters for {@link AprilTagPoseEstimator}.
     */
    public static final class Config {
        /** Multi-tag weighting, consensus, and field-plausibility policy. */
        public FixedTagFieldPoseSolver.Config fieldPoseSolver =
                FixedTagFieldPoseSolver.Config.defaults();

        /**
         * Maximum age (seconds) of the underlying detections frame accepted by this estimator.
         *
         * <p>This is the AprilTag-localization counterpart to guidance's live-tag freshness gate:
         * stale frames should not continue to produce a "global" robot pose just because the
         * detections object is still non-null. Values must be finite and >= 0.</p>
         */
        public double maxDetectionAgeSec = 0.50;

        /**
         * Camera mount extrinsics in the robot frame.
         */
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
         * Returns a raw authoring copy of this config without validating it.
         *
         * <p>A non-null nested solver config is copied independently. A null nested draft stays
         * null so an inactive profile branch can be copied without inventing defaults; the active
         * estimator owner rejects it.</p>
         */
        public Config copy() {
            Config c = new Config();
            c.fieldPoseSolver = fieldPoseSolver != null ? fieldPoseSolver.copy() : null;
            c.maxDetectionAgeSec = this.maxDetectionAgeSec;
            c.cameraMount = this.cameraMount;
            return c;
        }
    }

    private final AprilTagSensor tags;
    private final TagLayout layout;
    private final Config cfg;
    private final FixedTagFieldPoseSolver fieldPoseSolver;

    private PoseEstimate lastEstimate;
    private AprilTagDetections lastDetections = AprilTagDetections.none();
    private FixedTagFieldPoseSolver.Result lastSolve = FixedTagFieldPoseSolver.Result.none();
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean updateInProgress;
    private RuntimeException lastUpdateFailure;

    /**
     * Creates an AprilTag-only pose estimator that may use multiple visible fixed tags from the
     * same frame.
     *
     * <p>The estimator validates and snapshots {@code layout} during construction. Later mutation
     * of an authored or borrowed source layout cannot change this estimator's trusted field
     * facts.</p>
     *
     * @param tags shared AprilTag observation source
     * @param layout fixed field-tag facts to validate and snapshot
     * @param cfg complete estimator setup; non-null
     * @throws NullPointerException if {@code cfg}, its nested solver config or mount, {@code tags},
     *                              or {@code layout} is null
     * @throws IllegalArgumentException if the layout or estimator configuration is invalid
     */
    public AprilTagPoseEstimator(AprilTagSensor tags, TagLayout layout, Config cfg) {
        Config snapshot = captureConfig(cfg);
        FixedTagFieldPoseSolver solver = new FixedTagFieldPoseSolver(
                Objects.requireNonNull(
                        snapshot.fieldPoseSolver,
                        "AprilTagPoseEstimator.Config.fieldPoseSolver"
                )
        );
        Objects.requireNonNull(snapshot.cameraMount, "AprilTagPoseEstimator.Config.cameraMount");

        this.cfg = snapshot;
        this.fieldPoseSolver = solver;
        this.tags = Objects.requireNonNull(tags, "tags");
        this.layout = TagLayouts.snapshot(Objects.requireNonNull(layout, "layout"));
        this.lastEstimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
    }

    /** Capture and validate estimator-owned fields before touching sensor or layout collaborators. */
    private static Config captureConfig(Config config) {
        Config snapshot = Objects.requireNonNull(config, "cfg").copy();
        if (!Double.isFinite(snapshot.maxDetectionAgeSec) || snapshot.maxDetectionAgeSec < 0.0) {
            throw new IllegalArgumentException(
                    "AprilTagPoseEstimator.Config.maxDetectionAgeSec must be finite and >= 0, got "
                            + snapshot.maxDetectionAgeSec
            );
        }
        return snapshot;
    }

    /**
     * Samples detections and publishes at most one coherent solve snapshot per loop cycle.
     *
     * <p>The estimator claims the cycle before it samples or solves. A repeated successful call in
     * that cycle is a no-op, a repeated call after failure rethrows the exact first
     * {@link RuntimeException}, and recursive entry fails fast before another source sample. A
     * changed processor/source state after this update is therefore observed on the next cycle.</p>
     */
    @Override
    public void update(LoopClock clock) {
        LoopClock requiredClock = Objects.requireNonNull(clock, "clock");
        long cycle = requiredClock.cycle();
        if (updateInProgress) {
            throw new IllegalStateException(
                    "AprilTagPoseEstimator.update(clock) was reentered during cycle "
                            + cycle + "; one localization owner may advance only once per cycle"
            );
        }
        if (cycle == lastUpdateCycle) {
            if (lastUpdateFailure != null) {
                throw lastUpdateFailure;
            }
            return;
        }

        // Claim the attempt before sampling the sensor or solving so reentry cannot repeat either.
        lastUpdateCycle = cycle;
        updateInProgress = true;
        lastUpdateFailure = null;
        try {
            updateCurrentCycle(requiredClock);
        } catch (RuntimeException failure) {
            lastUpdateFailure = failure;
            throw failure;
        } finally {
            updateInProgress = false;
        }
    }

    /** Perform the one sensor sample and field solve owned by the claimed loop cycle. */
    private void updateCurrentCycle(LoopClock clock) {
        final LoopTimestamp nowTimestamp = clock.nowTimestamp();
        lastDetections = tags.get(clock);
        lastSolve = FixedTagFieldPoseSolver.Result.none();

        if (lastDetections == null || !lastDetections.isFresh(clock, cfg.maxDetectionAgeSec)) {
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        AprilTagDetections freshDetections = AprilTagDetections.fromFrame(
                lastDetections.frameTimestamp(),
                lastDetections.freshMatching(clock, layout.ids(), cfg.maxDetectionAgeSec)
        );

        if (freshDetections.observations.isEmpty()) {
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        lastSolve = fieldPoseSolver.solve(
                freshDetections.observations,
                layout,
                cfg.cameraMount
        );

        if (!lastSolve.hasPose) {
            lastEstimate = PoseEstimate.noPose(nowTimestamp);
            return;
        }

        Pose3d fieldToRobotPose = lastSolve.fieldToRobotPose;
        lastEstimate = new PoseEstimate(
                fieldToRobotPose,
                true,
                lastSolve.quality,
                freshDetections.frameTimestamp()
        );
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
                .addData(p + ".detections.timestampAvailable", lastDetections.frameTimestamp().isAvailable())
                .addData(p + ".detections.count", lastDetections.observations.size())
                .addData(p + ".cfg.maxDetectionAgeSec", cfg.maxDetectionAgeSec)
                .addData(p + ".solve.candidates", lastSolve.candidateCount)
                .addData(p + ".solve.accepted", lastSolve.acceptedCount)
                .addData(p + ".solve.acceptedFraction", lastSolve.acceptedFraction)
                .addData(p + ".solve.acceptedWeightFraction", lastSolve.acceptedWeightFraction)
                .addData(p + ".solve.totalWeight", lastSolve.totalWeight)
                .addData(p + ".solve.rangeInches", lastSolve.rangeInches)
                .addData(p + ".hasPose", lastEstimate.hasPose)
                .addData(p + ".quality", lastEstimate.quality)
                .addData(p + ".timestampAvailable", lastEstimate.timestamp.isAvailable());

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
                    .addData(q + ".timestampAvailable", c.observation.frameTimestamp().isAvailable())
                    .addData(q + ".weight", c.weight)
                    .addData(q + ".usedObservationFieldPose", c.usedObservationFieldPose)
                    .addData(q + ".cameraBearingRad", c.observation.cameraBearingRad())
                    .addData(q + ".cameraRangeInches", c.observation.cameraRangeInches())
                    .addData(q + ".fieldToRobotPose", c.fieldToRobotPose);
        }
    }
}
