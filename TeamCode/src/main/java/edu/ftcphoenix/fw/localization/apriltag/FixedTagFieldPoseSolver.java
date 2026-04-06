package edu.ftcphoenix.fw.localization.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;

/**
 * Shared helper that estimates a field-centric robot pose from one camera frame containing one or
 * more fixed AprilTag observations.
 *
 * <p>This solver is intentionally lower-level than a full {@link edu.ftcphoenix.fw.localization.PoseEstimator}.
 * It takes one coherent set of raw observations plus fixed field metadata and returns a best-effort
 * field pose solve. Both {@link TagOnlyPoseEstimator} and drive-guidance's temporary
 * "live field pose" bridge reuse this implementation so they do not diverge over time.</p>
 *
 * <p>Design goals:</p>
 * <ul>
 *   <li>Use all visible fixed tags from one frame when that helps.</li>
 *   <li>Prefer closer / more centered tags rather than equally trusting every detection.</li>
 *   <li>Reject obvious outliers before averaging.</li>
 *   <li>Allow FTC SDK {@code robotPose} observations when available, but fall back to Phoenix's
 *       explicit geometry chain when they disagree or are unavailable.</li>
 * </ul>
 */
public final class FixedTagFieldPoseSolver {

    /**
     * Configuration for the multi-tag field-pose solve.
     */
    public static class Config {
        /**
         * Optional maximum absolute camera bearing (radians). Zero disables this gate.
         */
        public double maxAbsBearingRad = 0.0;

        /**
         * Prefer the FTC SDK's per-detection {@code fieldToRobotPose} when present and roughly
         * consistent with Phoenix's explicit geometry solve.
         */
        public boolean preferObservationFieldPose = true;

        /**
         * Maximum allowed position disagreement between the SDK-provided field pose and the
         * geometry-derived pose before the SDK pose is ignored.
         */
        public double observationFieldPoseMaxDeltaInches = 8.0;

        /**
         * Maximum allowed heading disagreement between the SDK-provided field pose and the
         * geometry-derived pose before the SDK pose is ignored.
         */
        public double observationFieldPoseMaxDeltaHeadingRad = Math.toRadians(12.0);

        /**
         * Distance weighting soft scale (inches).
         *
         * <p>Weights are computed as {@code 1 / (1 + (range / rangeSoftnessInches)^2)}, so closer
         * tags contribute more without making far tags instantly worthless.</p>
         */
        public double rangeSoftnessInches = 36.0;

        /**
         * Ignore observations whose final weight falls below this threshold.
         */
        public double minObservationWeight = 0.05;

        /**
         * Position gate (inches) used when rejecting outlier tag solves against the consensus seed.
         */
        public double outlierPositionGateInches = 18.0;

        /**
         * Heading gate (radians) used when rejecting outlier tag solves against the consensus seed.
         */
        public double outlierHeadingGateRad = Math.toRadians(25.0);

        /**
         * Position residual scale (inches) used when turning solve consistency into a 0..1 quality
         * score.
         */
        public double consistencyPositionScaleInches = 6.0;

        /**
         * Heading residual scale (radians) used when turning solve consistency into a 0..1 quality
         * score.
         */
        public double consistencyHeadingScaleRad = Math.toRadians(8.0);

        /**
         * Creates a fresh config with Phoenix defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Returns a shallow copy of this config.
         */
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
            return c;
        }
    }

    /**
     * One accepted contribution used in the final solve.
     */
    public static final class Contribution {
        public final AprilTagObservation observation;
        public final Pose3d fieldToRobotPose;
        public final double weight;
        public final boolean usedObservationFieldPose;

        private Contribution(AprilTagObservation observation,
                             Pose3d fieldToRobotPose,
                             double weight,
                             boolean usedObservationFieldPose) {
            this.observation = observation;
            this.fieldToRobotPose = fieldToRobotPose;
            this.weight = weight;
            this.usedObservationFieldPose = usedObservationFieldPose;
        }
    }

    /**
     * Immutable result from one multi-tag solve attempt.
     */
    public static final class Result {
        public final boolean hasPose;
        public final Pose3d fieldToRobotPose;
        public final double rangeInches;
        public final double quality;
        public final int candidateCount;
        public final int acceptedCount;
        public final double totalWeight;
        public final List<Contribution> acceptedContributions;

        private Result(boolean hasPose,
                       Pose3d fieldToRobotPose,
                       double rangeInches,
                       double quality,
                       int candidateCount,
                       int acceptedCount,
                       double totalWeight,
                       List<Contribution> acceptedContributions) {
            this.hasPose = hasPose;
            this.fieldToRobotPose = fieldToRobotPose;
            this.rangeInches = rangeInches;
            this.quality = quality;
            this.candidateCount = candidateCount;
            this.acceptedCount = acceptedCount;
            this.totalWeight = totalWeight;
            this.acceptedContributions = acceptedContributions;
        }

        /**
         * Returns a no-pose result.
         */
        public static Result none() {
            return new Result(
                    false,
                    Pose3d.zero(),
                    Double.NaN,
                    0.0,
                    0,
                    0,
                    0.0,
                    Collections.<Contribution>emptyList()
            );
        }

        /**
         * Convenience planar projection of {@link #fieldToRobotPose}.
         */
        public Pose2d toPose2d() {
            return fieldToRobotPose.toPose2d();
        }
    }

    private static final class Candidate {
        final AprilTagObservation observation;
        final Pose3d fieldToRobotPose;
        final Pose2d fieldToRobot2d;
        final double weight;
        final boolean usedObservationFieldPose;

        Candidate(AprilTagObservation observation,
                  Pose3d fieldToRobotPose,
                  double weight,
                  boolean usedObservationFieldPose) {
            this.observation = observation;
            this.fieldToRobotPose = fieldToRobotPose;
            this.fieldToRobot2d = fieldToRobotPose.toPose2d();
            this.weight = weight;
            this.usedObservationFieldPose = usedObservationFieldPose;
        }
    }

    private FixedTagFieldPoseSolver() {
        // utility holder
    }

    /**
     * Solves for {@code field -> robot} using one coherent frame of raw AprilTag observations.
     */
    public static Result solve(List<AprilTagObservation> observations,
                               TagLayout layout,
                               CameraMountConfig cameraMount,
                               Config cfg) {
        Objects.requireNonNull(observations, "observations");
        Objects.requireNonNull(layout, "layout");
        CameraMountConfig mount = (cameraMount != null) ? cameraMount : CameraMountConfig.identity();
        Config solveCfg = (cfg != null) ? cfg : Config.defaults();

        if (observations.isEmpty() || layout.ids().isEmpty()) {
            return Result.none();
        }

        ArrayList<Candidate> candidates = new ArrayList<Candidate>(observations.size());
        for (AprilTagObservation obs : observations) {
            Candidate c = candidateFor(obs, layout, mount, solveCfg);
            if (c != null) {
                candidates.add(c);
            }
        }

        if (candidates.isEmpty()) {
            return Result.none();
        }

        Candidate seed = chooseConsensusSeed(candidates, solveCfg);
        if (seed == null) {
            return Result.none();
        }

        ArrayList<Candidate> accepted = new ArrayList<Candidate>(candidates.size());
        for (Candidate c : candidates) {
            if (withinOutlierGate(c, seed, solveCfg)) {
                accepted.add(c);
            }
        }
        if (accepted.isEmpty()) {
            accepted.add(seed);
        }

        return combineAccepted(candidates.size(), accepted, solveCfg);
    }

    private static Candidate candidateFor(AprilTagObservation obs,
                                          TagLayout layout,
                                          CameraMountConfig cameraMount,
                                          Config cfg) {
        if (obs == null || !obs.hasTarget || !layout.has(obs.id)) {
            return null;
        }

        double bearingRad = obs.cameraBearingRad();
        if (cfg.maxAbsBearingRad > 0.0 && Math.abs(bearingRad) > cfg.maxAbsBearingRad) {
            return null;
        }

        Pose3d geometryPose = geometryPoseFor(obs, layout, cameraMount);
        Pose3d observationFieldPose = obs.hasFieldToRobotPose() && isFinitePose(obs.fieldToRobotPose)
                ? obs.fieldToRobotPose
                : null;

        Pose3d chosenPose = geometryPose;
        boolean usedObservationFieldPose = false;

        if (cfg.preferObservationFieldPose && observationFieldPose != null) {
            if (geometryPose == null) {
                chosenPose = observationFieldPose;
                usedObservationFieldPose = true;
            } else if (posesAgree(geometryPose, observationFieldPose, cfg)) {
                chosenPose = observationFieldPose;
                usedObservationFieldPose = true;
            }
        }

        if (chosenPose == null || !isFinitePose(chosenPose)) {
            return null;
        }

        double weight = observationWeight(obs, cfg);
        if (!Double.isFinite(weight) || weight < cfg.minObservationWeight) {
            return null;
        }

        return new Candidate(obs, chosenPose, weight, usedObservationFieldPose);
    }

    private static Pose3d geometryPoseFor(AprilTagObservation obs,
                                          TagLayout layout,
                                          CameraMountConfig cameraMount) {
        if (obs == null || !obs.hasTarget || obs.cameraToTagPose == null || !layout.has(obs.id)) {
            return null;
        }
        Pose3d fieldToTagPose = layout.requireFieldToTagPose(obs.id);
        Pose3d robotToCameraPose = cameraMount.robotToCameraPose();
        Pose3d robotToTagPose = robotToCameraPose.then(obs.cameraToTagPose);
        Pose3d fieldToRobotPose = fieldToTagPose.then(robotToTagPose.inverse());
        return isFinitePose(fieldToRobotPose) ? fieldToRobotPose : null;
    }

    private static boolean posesAgree(Pose3d a, Pose3d b, Config cfg) {
        if (a == null || b == null) {
            return false;
        }
        double posErr = a.toPose2d().distanceTo(b.toPose2d());
        double headingErr = Math.abs(Pose2d.wrapToPi(b.yawRad - a.yawRad));
        return posErr <= cfg.observationFieldPoseMaxDeltaInches
                && headingErr <= cfg.observationFieldPoseMaxDeltaHeadingRad;
    }

    private static double observationWeight(AprilTagObservation obs, Config cfg) {
        double rangeSoft = cfg.rangeSoftnessInches;
        if (!Double.isFinite(rangeSoft) || rangeSoft <= 1e-6) {
            rangeSoft = 36.0;
        }

        double range = Math.max(0.0, obs.cameraRangeInches());
        double rangeRatio = range / rangeSoft;
        double rangeWeight = 1.0 / (1.0 + rangeRatio * rangeRatio);

        double absBearing = Math.abs(obs.cameraBearingRad());
        double cosBearing = Math.cos(Math.min(absBearing, Math.PI / 2.0));
        if (cosBearing < 0.0) {
            cosBearing = 0.0;
        }
        double bearingWeight = cosBearing * cosBearing;

        return MathUtil.clamp01(rangeWeight * bearingWeight);
    }

    private static Candidate chooseConsensusSeed(List<Candidate> candidates, Config cfg) {
        if (candidates == null || candidates.isEmpty()) {
            return null;
        }
        if (candidates.size() == 1) {
            return candidates.get(0);
        }

        Candidate best = null;
        double bestScore = Double.POSITIVE_INFINITY;

        double posGate = positiveOr(cfg.outlierPositionGateInches, 18.0);
        double headingGate = positiveOr(cfg.outlierHeadingGateRad, Math.toRadians(25.0));

        for (Candidate seed : candidates) {
            double score = 0.0;
            for (Candidate other : candidates) {
                double posErr = seed.fieldToRobot2d.distanceTo(other.fieldToRobot2d);
                double headingErr = Math.abs(Pose2d.wrapToPi(other.fieldToRobot2d.headingRad - seed.fieldToRobot2d.headingRad));
                score += other.weight * ((posErr / posGate) + (headingErr / headingGate));
            }
            if (score < bestScore - 1e-9
                    || (Math.abs(score - bestScore) <= 1e-9 && best != null && seed.weight > best.weight)
                    || best == null) {
                bestScore = score;
                best = seed;
            }
        }

        return best;
    }

    private static boolean withinOutlierGate(Candidate candidate, Candidate seed, Config cfg) {
        double posGate = positiveOr(cfg.outlierPositionGateInches, 18.0);
        double headingGate = positiveOr(cfg.outlierHeadingGateRad, Math.toRadians(25.0));

        double posErr = candidate.fieldToRobot2d.distanceTo(seed.fieldToRobot2d);
        double headingErr = Math.abs(Pose2d.wrapToPi(candidate.fieldToRobot2d.headingRad - seed.fieldToRobot2d.headingRad));
        return posErr <= posGate && headingErr <= headingGate;
    }

    private static Result combineAccepted(int candidateCount,
                                          List<Candidate> accepted,
                                          Config cfg) {
        if (accepted == null || accepted.isEmpty()) {
            return Result.none();
        }

        double sumW = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        double sumSinYaw = 0.0;
        double sumCosYaw = 0.0;
        double sumSinPitch = 0.0;
        double sumCosPitch = 0.0;
        double sumSinRoll = 0.0;
        double sumCosRoll = 0.0;
        double sumRange = 0.0;

        for (Candidate c : accepted) {
            double w = c.weight;
            sumW += w;
            sumX += w * c.fieldToRobotPose.xInches;
            sumY += w * c.fieldToRobotPose.yInches;
            sumZ += w * c.fieldToRobotPose.zInches;
            sumSinYaw += w * Math.sin(c.fieldToRobotPose.yawRad);
            sumCosYaw += w * Math.cos(c.fieldToRobotPose.yawRad);
            sumSinPitch += w * Math.sin(c.fieldToRobotPose.pitchRad);
            sumCosPitch += w * Math.cos(c.fieldToRobotPose.pitchRad);
            sumSinRoll += w * Math.sin(c.fieldToRobotPose.rollRad);
            sumCosRoll += w * Math.cos(c.fieldToRobotPose.rollRad);
            sumRange += w * c.observation.cameraRangeInches();
        }

        if (!Double.isFinite(sumW) || sumW <= 1e-9) {
            return Result.none();
        }

        Pose3d solvedPose = new Pose3d(
                sumX / sumW,
                sumY / sumW,
                sumZ / sumW,
                Math.atan2(sumSinYaw, sumCosYaw),
                Math.atan2(sumSinPitch, sumCosPitch),
                Math.atan2(sumSinRoll, sumCosRoll)
        );

        Pose2d solved2d = solvedPose.toPose2d();
        double residualPos = 0.0;
        double residualHeading = 0.0;
        for (Candidate c : accepted) {
            double w = c.weight / sumW;
            residualPos += w * c.fieldToRobot2d.distanceTo(solved2d);
            residualHeading += w * Math.abs(Pose2d.wrapToPi(c.fieldToRobot2d.headingRad - solved2d.headingRad));
        }

        double avgWeight = MathUtil.clamp01(sumW / accepted.size());
        double countScore = MathUtil.clamp01(((double) accepted.size()) / 3.0);
        double posConsistency = 1.0 - MathUtil.clamp01(residualPos / positiveOr(cfg.consistencyPositionScaleInches, 6.0));
        double headingConsistency = 1.0 - MathUtil.clamp01(residualHeading / positiveOr(cfg.consistencyHeadingScaleRad, Math.toRadians(8.0)));
        double consistency = 0.5 * (posConsistency + headingConsistency);
        double quality = MathUtil.clamp01(
                (0.35 + 0.65 * avgWeight)
                        * (0.5 + 0.5 * countScore)
                        * (0.5 + 0.5 * consistency)
        );

        ArrayList<Contribution> contributions = new ArrayList<Contribution>(accepted.size());
        for (Candidate c : accepted) {
            contributions.add(new Contribution(
                    c.observation,
                    c.fieldToRobotPose,
                    c.weight,
                    c.usedObservationFieldPose
            ));
        }

        return new Result(
                true,
                solvedPose,
                sumRange / sumW,
                quality,
                candidateCount,
                accepted.size(),
                sumW,
                Collections.unmodifiableList(contributions)
        );
    }

    private static boolean isFinitePose(Pose3d pose) {
        return pose != null
                && Double.isFinite(pose.xInches)
                && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches)
                && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad)
                && Double.isFinite(pose.rollRad);
    }

    private static double positiveOr(double value, double fallback) {
        return (Double.isFinite(value) && value > 1e-9) ? value : fallback;
    }
}
