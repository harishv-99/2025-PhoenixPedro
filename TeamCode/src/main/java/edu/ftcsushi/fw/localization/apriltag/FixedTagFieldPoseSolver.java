package edu.ftcsushi.fw.localization.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.math.MathUtil;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.spatial.Region2d;

/**
 * Configured solver that estimates a field-centric robot pose from one camera frame containing one
 * or more fixed AprilTag observations.
 *
 * <p>This solver is intentionally lower-level than a full {@link edu.ftcsushi.fw.localization.AbsolutePoseEstimator}.
 * It takes one coherent set of raw observations plus fixed field metadata and returns a best-effort
 * field pose solve. Both {@link AprilTagPoseEstimator} and drive-guidance's temporary
 * "live field pose" bridge reuse this implementation so they do not diverge over time.</p>
 *
 * <p>Design goals:</p>
 * <ul>
 *   <li>Use all visible fixed tags from one frame when that helps.</li>
 *   <li>Prefer closer / more centered tags rather than equally trusting every detection.</li>
 *   <li>Reject obvious outliers before averaging.</li>
 *   <li>Allow FTC SDK {@code robotPose} observations when available, but fall back to Sushi's
 *       explicit geometry chain when they disagree or are unavailable.</li>
 * </ul>
 */
public final class FixedTagFieldPoseSolver {

    /**
     * Configuration for the multi-tag field-pose solve.
     */
    public static final class Config {
        /**
         * Optional maximum absolute camera bearing in radians, in {@code [0, pi]}. Zero disables
         * this gate.
         */
        public double maxAbsBearingRad = 0.0;

        /**
         * Prefer the FTC SDK's per-detection {@code fieldToRobotPose} when present and roughly
         * consistent with Sushi's explicit geometry solve.
         */
        public boolean preferObservationFieldPose = true;

        /**
         * Maximum allowed position disagreement between the SDK-provided field pose and the
         * geometry-derived pose before the SDK pose is ignored. Must be finite and {@code >= 0}.
         */
        public double observationFieldPoseMaxDeltaInches = 8.0;

        /**
         * Maximum allowed heading disagreement between the SDK-provided field pose and the
         * geometry-derived pose before the SDK pose is ignored. Must be finite and in
         * {@code [0, pi]}.
         */
        public double observationFieldPoseMaxDeltaHeadingRad = Math.toRadians(12.0);

        /**
         * Distance weighting soft scale (inches).
         *
         * <p>Weights are computed as {@code 1 / (1 + (range / rangeSoftnessInches)^2)}, so closer
         * tags contribute more without making far tags instantly worthless. Must be finite and
         * {@code > 0}; every accepted positive value is used exactly.</p>
         */
        public double rangeSoftnessInches = 36.0;

        /**
         * Ignore observations whose final weight falls below this finite threshold in
         * {@code [0, 1]}.
         */
        public double minObservationWeight = 0.05;

        /**
         * Finite positive position gate (inches) used when rejecting outlier tag solves against
         * the consensus seed.
         */
        public double outlierPositionGateInches = 18.0;

        /**
         * Heading gate (radians) used when rejecting outlier tag solves against the consensus
         * seed. Must be finite and in {@code (0, pi]}.
         */
        public double outlierHeadingGateRad = Math.toRadians(25.0);

        /**
         * Position residual scale (inches) used when turning solve consistency into a 0..1 quality
         * score. Must be finite and {@code > 0}.
         */
        public double consistencyPositionScaleInches = 6.0;

        /**
         * Heading residual scale (radians) used when turning solve consistency into a 0..1 quality
         * score. Must be finite and {@code > 0}.
         */
        public double consistencyHeadingScaleRad = Math.toRadians(8.0);

        /**
         * Optional region describing where the robot is plausibly allowed to be on the field.
         *
         * <p>When non-null, obviously impossible AprilTag field solves may be rejected before they
         * reach the localizer or guidance lane. This is a lightweight reliability gate rather than
         * a full collision / rules model.</p>
         */
        public Region2d plausibleFieldRegion = null;

        /**
         * How far outside {@link #plausibleFieldRegion} a pose may drift before it is rejected.
         *
         * <p>Zero means the pose must lie inside the region (or exactly on its boundary). A small
         * positive tolerance is often useful because AprilTag measurements have some noise and the
         * robot center may briefly solve a hair outside the legal floor footprint. The value must
         * be finite and {@code >= 0}; every accepted positive value is used exactly.</p>
         */
        public double maxOutsidePlausibleFieldRegionInches = 0.0;

        private Config() {
            // Defaults are assigned in the field initializers above.
        }

        /**
         * Creates a fresh config with Sushi defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Returns a raw authoring copy of this config.
         *
         * <p>The optional {@link #plausibleFieldRegion} is retained by reference. A supplied
         * region is therefore a stable, side-effect-free policy collaborator rather than live
         * tuning state. Validation occurs when a {@link FixedTagFieldPoseSolver} owns the copy.</p>
         */
        public Config copy() {
            Config c = new Config();
            c.maxAbsBearingRad = maxAbsBearingRad;
            c.preferObservationFieldPose = preferObservationFieldPose;
            c.observationFieldPoseMaxDeltaInches = observationFieldPoseMaxDeltaInches;
            c.observationFieldPoseMaxDeltaHeadingRad = observationFieldPoseMaxDeltaHeadingRad;
            c.rangeSoftnessInches = rangeSoftnessInches;
            c.minObservationWeight = minObservationWeight;
            c.outlierPositionGateInches = outlierPositionGateInches;
            c.outlierHeadingGateRad = outlierHeadingGateRad;
            c.consistencyPositionScaleInches = consistencyPositionScaleInches;
            c.consistencyHeadingScaleRad = consistencyHeadingScaleRad;
            c.plausibleFieldRegion = plausibleFieldRegion;
            c.maxOutsidePlausibleFieldRegionInches = maxOutsidePlausibleFieldRegionInches;
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
        public final double acceptedFraction;
        public final double acceptedWeightFraction;
        public final double totalWeight;
        public final List<Contribution> acceptedContributions;

        private Result(boolean hasPose,
                       Pose3d fieldToRobotPose,
                       double rangeInches,
                       double quality,
                       int candidateCount,
                       int acceptedCount,
                       double acceptedFraction,
                       double acceptedWeightFraction,
                       double totalWeight,
                       List<Contribution> acceptedContributions) {
            this.hasPose = hasPose;
            this.fieldToRobotPose = fieldToRobotPose;
            this.rangeInches = rangeInches;
            this.quality = quality;
            this.candidateCount = candidateCount;
            this.acceptedCount = acceptedCount;
            this.acceptedFraction = acceptedFraction;
            this.acceptedWeightFraction = acceptedWeightFraction;
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
                    0.0,
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

    private final Config config;

    /**
     * Creates one immutable field-pose policy owner from a complete authoring config.
     *
     * <p>The config is copied and validated once. Later mutation of the supplied draft cannot
     * change this solver. The optional region is retained as the documented stable policy
     * collaborator.</p>
     *
     * @param config complete solver configuration; non-null
     * @throws NullPointerException if {@code config} is null
     * @throws IllegalArgumentException if a numeric field is non-finite or outside its documented
     *                                  domain
     */
    public FixedTagFieldPoseSolver(Config config) {
        Config snapshot = Objects.requireNonNull(config, "config").copy();
        validate(snapshot);
        this.config = snapshot;
    }

    /**
     * Solves for {@code field -> robot} using one coherent frame of raw AprilTag observations.
     *
     * <p>Runtime evidence that is missing, inconsistent, outside configured gates, or unable to
     * produce finite published pose/range/quality aggregates returns {@link Result#none()}.</p>
     *
     * @param observations one coherent frame of observations; non-null
     * @param layout trusted fixed field-tag facts; non-null
     * @param cameraMount finite robot-to-camera extrinsics; non-null
     * @return a finite bounded pose result when solvable; otherwise {@link Result#none()}
     * @throws NullPointerException if any argument is null
     */
    public Result solve(List<AprilTagObservation> observations,
                        TagLayout layout,
                        CameraMountConfig cameraMount) {
        Objects.requireNonNull(observations, "observations");
        Objects.requireNonNull(layout, "layout");
        CameraMountConfig mount = Objects.requireNonNull(cameraMount, "cameraMount");

        if (observations.isEmpty() || layout.ids().isEmpty()) {
            return Result.none();
        }

        ArrayList<Candidate> candidates = new ArrayList<Candidate>(observations.size());
        double totalCandidateWeight = 0.0;
        for (AprilTagObservation obs : observations) {
            Candidate c = candidateFor(obs, layout, mount, config);
            if (c != null) {
                candidates.add(c);
                totalCandidateWeight += c.weight;
            }
        }

        if (candidates.isEmpty()) {
            return Result.none();
        }

        Candidate seed = chooseConsensusSeed(candidates, config);
        if (seed == null) {
            return Result.none();
        }

        ArrayList<Candidate> accepted = new ArrayList<Candidate>(candidates.size());
        for (Candidate c : candidates) {
            if (withinOutlierGate(c, seed, config)) {
                accepted.add(c);
            }
        }
        if (accepted.isEmpty()) {
            accepted.add(seed);
        }

        return combineAccepted(candidates.size(), totalCandidateWeight, accepted, config);
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

        if (!isPosePlausible(chosenPose, cfg)) {
            if (usedObservationFieldPose && geometryPose != null && isFinitePose(geometryPose)
                    && isPosePlausible(geometryPose, cfg)) {
                chosenPose = geometryPose;
                usedObservationFieldPose = false;
            } else {
                return null;
            }
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
        double range = Math.max(0.0, obs.cameraRangeInches());
        double rangeRatio = range / cfg.rangeSoftnessInches;
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

        double posGate = cfg.outlierPositionGateInches;
        double headingGate = cfg.outlierHeadingGateRad;

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
        double posGate = cfg.outlierPositionGateInches;
        double headingGate = cfg.outlierHeadingGateRad;

        double posErr = candidate.fieldToRobot2d.distanceTo(seed.fieldToRobot2d);
        double headingErr = Math.abs(Pose2d.wrapToPi(candidate.fieldToRobot2d.headingRad - seed.fieldToRobot2d.headingRad));
        return posErr <= posGate && headingErr <= headingGate;
    }

    private static Result combineAccepted(int candidateCount,
                                          double totalCandidateWeight,
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

        if (!isFinitePose(solvedPose) || !isPosePlausible(solvedPose, cfg)) {
            return Result.none();
        }

        Pose2d solved2d = solvedPose.toPose2d();
        double residualPos = 0.0;
        double residualHeading = 0.0;
        for (Candidate c : accepted) {
            double w = c.weight / sumW;
            residualPos += w * c.fieldToRobot2d.distanceTo(solved2d);
            residualHeading += w * Math.abs(Pose2d.wrapToPi(c.fieldToRobot2d.headingRad - solved2d.headingRad));
        }

        if (!Double.isFinite(residualPos) || !Double.isFinite(residualHeading)
                || !Double.isFinite(totalCandidateWeight) || totalCandidateWeight <= 0.0) {
            return Result.none();
        }

        double avgWeight = MathUtil.clamp01(sumW / accepted.size());
        double countScore = MathUtil.clamp01(((double) accepted.size()) / 3.0);
        double acceptedFraction = MathUtil.clamp01(((double) accepted.size()) / Math.max(1.0, candidateCount));
        double acceptedWeightFraction = MathUtil.clamp01(sumW / totalCandidateWeight);
        double posConsistency = 1.0 - MathUtil.clamp01(residualPos / cfg.consistencyPositionScaleInches);
        double headingConsistency = 1.0 - MathUtil.clamp01(residualHeading / cfg.consistencyHeadingScaleRad);
        double consistency = 0.5 * (posConsistency + headingConsistency);
        double quality = MathUtil.clamp01(
                (0.35 + 0.65 * avgWeight)
                        * (0.5 + 0.5 * countScore)
                        * (0.40 + 0.60 * acceptedFraction)
                        * (0.30 + 0.70 * acceptedWeightFraction)
                        * (0.5 + 0.5 * consistency)
                        * plausibleRegionQualityScale(solved2d, cfg)
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

        double rangeInches = sumRange / sumW;
        if (!isFiniteSuccessfulResult(
                solvedPose,
                rangeInches,
                quality,
                candidateCount,
                accepted.size(),
                acceptedFraction,
                acceptedWeightFraction,
                sumW
        )) {
            return Result.none();
        }

        return new Result(
                true,
                solvedPose,
                rangeInches,
                quality,
                candidateCount,
                accepted.size(),
                acceptedFraction,
                acceptedWeightFraction,
                sumW,
                Collections.unmodifiableList(contributions)
        );
    }

    private static boolean isPosePlausible(Pose3d pose, Config cfg) {
        if (pose == null || cfg == null || cfg.plausibleFieldRegion == null) {
            return true;
        }
        Pose2d p = pose.toPose2d();
        double signedDistance = cfg.plausibleFieldRegion.signedDistanceInches(p);
        return Double.isFinite(signedDistance)
                && signedDistance >= -cfg.maxOutsidePlausibleFieldRegionInches;
    }

    private static double plausibleRegionQualityScale(Pose2d pose, Config cfg) {
        if (pose == null || cfg == null || cfg.plausibleFieldRegion == null) {
            return 1.0;
        }
        double signedDistance = cfg.plausibleFieldRegion.signedDistanceInches(pose);
        if (!Double.isFinite(signedDistance)) {
            return Double.NaN;
        }
        if (signedDistance >= 0.0) {
            return 1.0;
        }
        double maxOutside = cfg.maxOutsidePlausibleFieldRegionInches;
        if (maxOutside == 0.0) {
            return 0.0;
        }
        return MathUtil.clamp01(1.0 - ((-signedDistance) / maxOutside));
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

    private static boolean isFiniteSuccessfulResult(Pose3d pose,
                                                    double rangeInches,
                                                    double quality,
                                                    int candidateCount,
                                                    int acceptedCount,
                                                    double acceptedFraction,
                                                    double acceptedWeightFraction,
                                                    double totalWeight) {
        return isFinitePose(pose)
                && Double.isFinite(rangeInches)
                && rangeInches >= 0.0
                && isFiniteFraction(quality)
                && candidateCount > 0
                && acceptedCount > 0
                && acceptedCount <= candidateCount
                && isFiniteFraction(acceptedFraction)
                && isFiniteFraction(acceptedWeightFraction)
                && Double.isFinite(totalWeight)
                && totalWeight > 0.0;
    }

    private static boolean isFiniteFraction(double value) {
        return Double.isFinite(value) && value >= 0.0 && value <= 1.0;
    }

    private static void validate(Config config) {
        String p = "FixedTagFieldPoseSolver.Config";
        requireFiniteInRange(config.maxAbsBearingRad, 0.0, Math.PI, p + ".maxAbsBearingRad");
        requireFiniteNonNegative(
                config.observationFieldPoseMaxDeltaInches,
                p + ".observationFieldPoseMaxDeltaInches"
        );
        requireFiniteInRange(
                config.observationFieldPoseMaxDeltaHeadingRad,
                0.0,
                Math.PI,
                p + ".observationFieldPoseMaxDeltaHeadingRad"
        );
        requireFinitePositive(config.rangeSoftnessInches, p + ".rangeSoftnessInches");
        requireFiniteInRange(config.minObservationWeight, 0.0, 1.0, p + ".minObservationWeight");
        requireFinitePositive(config.outlierPositionGateInches, p + ".outlierPositionGateInches");
        requireFiniteInRange(
                config.outlierHeadingGateRad,
                Math.nextUp(0.0),
                Math.PI,
                p + ".outlierHeadingGateRad",
                "finite and in (0, " + Math.PI + "]"
        );
        requireFinitePositive(
                config.consistencyPositionScaleInches,
                p + ".consistencyPositionScaleInches"
        );
        requireFinitePositive(
                config.consistencyHeadingScaleRad,
                p + ".consistencyHeadingScaleRad"
        );
        requireFiniteNonNegative(
                config.maxOutsidePlausibleFieldRegionInches,
                p + ".maxOutsidePlausibleFieldRegionInches"
        );
    }

    private static void requireFinitePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and > 0, got " + value);
        }
    }

    private static void requireFiniteNonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and >= 0, got " + value);
        }
    }

    private static void requireFiniteInRange(double value, double min, double max, String name) {
        if (!Double.isFinite(value) || value < min || value > max) {
            throw new IllegalArgumentException(
                    name + " must be finite and in [" + min + ", " + max + "], got " + value);
        }
    }

    private static void requireFiniteInRange(double value,
                                             double min,
                                             double max,
                                             String name,
                                             String domain) {
        if (!Double.isFinite(value) || value < min || value > max) {
            throw new IllegalArgumentException(name + " must be " + domain + ", got " + value);
        }
    }

    /**
     * Returns a compact description of the immutable policy snapshot owned by this solver.
     */
    @Override
    public String toString() {
        return "FixedTagFieldPoseSolver{maxAbsBearingRad=" + config.maxAbsBearingRad
                + ", preferObservationFieldPose=" + config.preferObservationFieldPose
                + ", observationFieldPoseMaxDeltaInches="
                + config.observationFieldPoseMaxDeltaInches
                + ", observationFieldPoseMaxDeltaHeadingRad="
                + config.observationFieldPoseMaxDeltaHeadingRad
                + ", rangeSoftnessInches=" + config.rangeSoftnessInches
                + ", minObservationWeight=" + config.minObservationWeight
                + ", outlierPositionGateInches=" + config.outlierPositionGateInches
                + ", outlierHeadingGateRad=" + config.outlierHeadingGateRad
                + ", consistencyPositionScaleInches="
                + config.consistencyPositionScaleInches
                + ", consistencyHeadingScaleRad=" + config.consistencyHeadingScaleRad
                + ", plausibleFieldRegion=" + config.plausibleFieldRegion
                + ", maxOutsidePlausibleFieldRegionInches="
                + config.maxOutsidePlausibleFieldRegionInches + '}';
    }
}
