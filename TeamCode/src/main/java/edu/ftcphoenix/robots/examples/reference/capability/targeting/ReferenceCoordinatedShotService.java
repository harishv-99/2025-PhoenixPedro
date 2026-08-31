package edu.ftcphoenix.robots.examples.reference.capability.targeting;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.PlantTargetRequest;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.spatial.SpatialControlFrames;
import edu.ftcphoenix.fw.spatial.SpatialQuery;
import edu.ftcphoenix.fw.spatial.SpatialQuerySelectors;
import edu.ftcphoenix.fw.spatial.SpatialSolutionGate;
import edu.ftcphoenix.fw.spatial.SpatialSolveSet;
import edu.ftcphoenix.fw.spatial.SpatialTargets;
import edu.ftcphoenix.fw.spatial.SpatialTranslationSelection;

/**
 * Optional Reference case study that publishes one coherent turret, flywheel, and hood intent.
 *
 * <p>This is robot-owned example policy, not framework projectile physics. The configured model is
 * deliberately illustrative and uncalibrated. An adopting robot must measure its own flight time,
 * lookup tables, mechanism response, and safe operating domains.</p>
 *
 * <p>Declare the localization owner before this {@link RobotProgram.Service}. This service borrows
 * the supplied {@link MotionPredictor}: it reads only the predictor's already-published pose and
 * motion snapshots and never updates, resets, or stops it. One private translation-only
 * {@link SpatialQuery} supplies target geometry, and one memoized calculation publishes an
 * immutable {@link Solution} at most once per loop cycle.</p>
 */
public final class ReferenceCoordinatedShotService implements RobotProgram.Service {

    /** Mutable data-only configuration; construction validates and snapshots every field. */
    public static final class Config {
        public double targetFieldXInches;
        public double targetFieldYInches;

        public double spatialMaxAgeSec;
        public double spatialMinQuality;
        public double motionMaxAgeSec;
        public double motionMinQuality;
        public double motionMaxIntervalSec;

        public double illustrativeFlightTimeSec;
        public double minimumModelDistanceInches;
        public double maximumModelDistanceInches;
        public InterpolatingTable1D flywheelVelocityTicksPerSecByDistance;
        public InterpolatingTable1D hoodPositionByDistance;

        public double minimumFlywheelVelocityTicksPerSec;
        public double maximumFlywheelVelocityTicksPerSec;
        public double minimumHoodPosition;
        public double maximumHoodPosition;
        public double unavailableFlywheelVelocityTicksPerSec;
        public double unavailableHoodPosition;

        private Config() {
        }

        /**
         * Returns compiling software values only; none is a calibrated or physically safe claim.
         */
        public static Config defaults() {
            Config config = new Config();
            config.targetFieldXInches = 72.0;
            config.targetFieldYInches = 72.0;
            config.spatialMaxAgeSec = 0.10;
            config.spatialMinQuality = 0.50;
            config.motionMaxAgeSec = 0.10;
            config.motionMinQuality = 0.50;
            config.motionMaxIntervalSec = 0.10;
            config.illustrativeFlightTimeSec = 0.20;
            config.minimumModelDistanceInches = 24.0;
            config.maximumModelDistanceInches = 120.0;
            config.flywheelVelocityTicksPerSecByDistance =
                    InterpolatingTable1D.ofSortedPairs(
                            24.0, 1200.0,
                            72.0, 1800.0,
                            120.0, 2400.0
                    );
            config.hoodPositionByDistance = InterpolatingTable1D.ofSortedPairs(
                    24.0, 0.25,
                    72.0, 0.50,
                    120.0, 0.75
            );
            config.minimumFlywheelVelocityTicksPerSec = 0.0;
            config.maximumFlywheelVelocityTicksPerSec = 3000.0;
            config.minimumHoodPosition = 0.0;
            config.maximumHoodPosition = 1.0;
            config.unavailableFlywheelVelocityTicksPerSec = 0.0;
            config.unavailableHoodPosition = 0.25;
            return config;
        }
    }

    /** Truthful calculation mode for the published snapshot. */
    public enum Mode {
        /** Co-temporal target geometry and motion both contributed to the result. */
        MOVING_COMPENSATED,
        /** Target geometry was usable, but motion was deliberately excluded. */
        STATIONARY_FALLBACK,
        /** No target request or modeled active intent is available. */
        UNAVAILABLE
    }

    /** Bounded explanation for the selected {@link Mode}. */
    public enum Reason {
        /** No degradation or failure applies. */
        NONE,
        /** The owner has not reached its managed START boundary. */
        NOT_STARTED,
        /** The terminal owner has been stopped. */
        STOPPED,
        /** No translation observation passed the configured spatial gates. */
        SPATIAL_UNAVAILABLE,
        /** Selected spatial geometry or quality was not finite and within its declared domain. */
        SPATIAL_INVALID,
        /** The predictor published no usable positive-duration motion delta. */
        MOTION_UNAVAILABLE,
        /** The motion delta's end observation exceeded its configured age. */
        MOTION_STALE,
        /** A clock reset invalidated the delta's timestamp interval. */
        MOTION_RESET_INVALIDATED,
        /** The motion delta and target geometry did not end at the exact same observation time. */
        MOTION_TIMESTAMP_MISMATCH,
        /** Motion interval, planar geometry, or quality failed its configured contract. */
        MOTION_INVALID,
        /** The effective target distance was outside the authored table domain. */
        MODEL_DISTANCE_OUT_OF_DOMAIN,
        /** An interpolated flywheel or hood intent was outside its software domain. */
        MODEL_OUTPUT_INVALID
    }

    /** Immutable coordinated result from one complete successful calculation. */
    public static final class Solution {
        public final Mode mode;
        public final Reason reason;
        public final LoopTimestamp observationTimestamp;
        public final double effectiveForwardInches;
        public final double effectiveLeftInches;
        public final double quality;
        public final PlantTargetRequest turretRequest;
        public final double flywheelVelocityTicksPerSec;
        public final double hoodPosition;

        private Solution(Mode mode,
                         Reason reason,
                         LoopTimestamp observationTimestamp,
                         double effectiveForwardInches,
                         double effectiveLeftInches,
                         double quality,
                         PlantTargetRequest turretRequest,
                         double flywheelVelocityTicksPerSec,
                         double hoodPosition) {
            this.mode = Objects.requireNonNull(mode, "mode");
            this.reason = Objects.requireNonNull(reason, "reason");
            this.observationTimestamp = Objects.requireNonNull(
                    observationTimestamp, "observationTimestamp");
            this.effectiveForwardInches = effectiveForwardInches;
            this.effectiveLeftInches = effectiveLeftInches;
            this.quality = quality;
            this.turretRequest = Objects.requireNonNull(turretRequest, "turretRequest");
            this.flywheelVelocityTicksPerSec = flywheelVelocityTicksPerSec;
            this.hoodPosition = hoodPosition;
        }
    }

    private enum Lifecycle {
        NEW,
        ACTIVE,
        TERMINAL
    }

    private static final String TURRET_REQUEST_ID = "reference-coordinated-shot";

    private final MotionPredictor predictor;
    private final double motionMaxAgeSec;
    private final double motionMinQuality;
    private final double motionMaxIntervalSec;
    private final double illustrativeFlightTimeSec;
    private final double minimumModelDistanceInches;
    private final double maximumModelDistanceInches;
    private final InterpolatingTable1D flywheelVelocityTicksPerSecByDistance;
    private final InterpolatingTable1D hoodPositionByDistance;
    private final double minimumFlywheelVelocityTicksPerSec;
    private final double maximumFlywheelVelocityTicksPerSec;
    private final double minimumHoodPosition;
    private final double maximumHoodPosition;
    private final double unavailableFlywheelVelocityTicksPerSec;
    private final double unavailableHoodPosition;
    private final SpatialSolutionGate spatialGate;
    private final SpatialQuery targetQuery;
    private final Source<Solution> calculation;

    private Lifecycle lifecycle = Lifecycle.NEW;
    private Solution solution;

    /**
     * Constructs the calculation owner without sampling or mutating the borrowed predictor.
     */
    public ReferenceCoordinatedShotService(MotionPredictor predictor, Config config) {
        this.predictor = Objects.requireNonNull(predictor, "predictor is required");
        Config required = Objects.requireNonNull(config, "config is required");

        double targetX = requireFinite(required.targetFieldXInches, "targetFieldXInches");
        double targetY = requireFinite(required.targetFieldYInches, "targetFieldYInches");
        double spatialAge = requireNonNegative(
                required.spatialMaxAgeSec, "spatialMaxAgeSec");
        double spatialQuality = requireQuality(
                required.spatialMinQuality, "spatialMinQuality");
        motionMaxAgeSec = requireNonNegative(required.motionMaxAgeSec, "motionMaxAgeSec");
        motionMinQuality = requireQuality(required.motionMinQuality, "motionMinQuality");
        motionMaxIntervalSec = requireNonNegative(
                required.motionMaxIntervalSec, "motionMaxIntervalSec");
        illustrativeFlightTimeSec = requireNonNegative(
                required.illustrativeFlightTimeSec, "illustrativeFlightTimeSec");
        minimumModelDistanceInches = requireNonNegative(
                required.minimumModelDistanceInches, "minimumModelDistanceInches");
        maximumModelDistanceInches = requireOrderedMaximum(
                required.maximumModelDistanceInches,
                minimumModelDistanceInches,
                "maximumModelDistanceInches");
        flywheelVelocityTicksPerSecByDistance = Objects.requireNonNull(
                required.flywheelVelocityTicksPerSecByDistance,
                "flywheelVelocityTicksPerSecByDistance is required");
        hoodPositionByDistance = Objects.requireNonNull(
                required.hoodPositionByDistance,
                "hoodPositionByDistance is required");

        minimumFlywheelVelocityTicksPerSec = requireFinite(
                required.minimumFlywheelVelocityTicksPerSec,
                "minimumFlywheelVelocityTicksPerSec");
        maximumFlywheelVelocityTicksPerSec = requireOrderedMaximum(
                required.maximumFlywheelVelocityTicksPerSec,
                minimumFlywheelVelocityTicksPerSec,
                "maximumFlywheelVelocityTicksPerSec");
        minimumHoodPosition = requireFinite(
                required.minimumHoodPosition, "minimumHoodPosition");
        maximumHoodPosition = requireOrderedMaximum(
                required.maximumHoodPosition, minimumHoodPosition, "maximumHoodPosition");
        unavailableFlywheelVelocityTicksPerSec = requireInside(
                required.unavailableFlywheelVelocityTicksPerSec,
                minimumFlywheelVelocityTicksPerSec,
                maximumFlywheelVelocityTicksPerSec,
                "unavailableFlywheelVelocityTicksPerSec");
        unavailableHoodPosition = requireInside(
                required.unavailableHoodPosition,
                minimumHoodPosition,
                maximumHoodPosition,
                "unavailableHoodPosition");

        spatialGate = SpatialSolutionGate.builder()
                .maxAgeSec(spatialAge)
                .minQuality(spatialQuality)
                .build();
        SpatialSolveSet solveSet = SpatialSolveSet.builder()
                .absolutePose(this.predictor, spatialAge, spatialQuality)
                .build();
        targetQuery = SpatialQuery.builder()
                .translateTo(SpatialTargets.fieldPoint(targetX, targetY))
                .controlFrames(SpatialControlFrames.robotCenter())
                .solveWith(solveSet)
                .build();
        calculation = Source.of(this::calculate).memoized();
        solution = unavailable(Reason.NOT_STARTED, LoopTimestamp.unavailable());
    }

    /** Returns the latest immutable snapshot without sampling localization or recalculating. */
    public Solution solution() {
        return solution;
    }

    @Override
    public void start(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        if (lifecycle != Lifecycle.NEW) {
            throw new IllegalStateException(
                    "ReferenceCoordinatedShotService may start exactly once");
        }
        resetOwnedCalculation();
        solution = unavailable(Reason.NOT_STARTED, LoopTimestamp.unavailable());
        lifecycle = Lifecycle.ACTIVE;
        update(clock);
    }

    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        if (lifecycle != Lifecycle.ACTIVE) {
            throw new IllegalStateException(
                    "ReferenceCoordinatedShotService update requires an active started owner");
        }
        Solution next = calculation.get(clock);
        solution = next;
    }

    @Override
    public void stop() {
        if (lifecycle == Lifecycle.TERMINAL) {
            return;
        }
        lifecycle = Lifecycle.TERMINAL;
        solution = unavailable(Reason.STOPPED, LoopTimestamp.unavailable());
        resetOwnedCalculation();
    }

    /** Package-private synchronization seam for the sibling turret planner. */
    double plannerObservationMaxAgeSec() {
        return spatialGate.maxAgeSec;
    }

    /** Package-private synchronization seam for the sibling turret planner. */
    double plannerMinimumObservationQuality() {
        // A moving request already carries min(spatial, motion) quality. Use the lower accepted
        // input threshold so the turret cannot reject a solution that this service accepted for
        // the coordinated flywheel/hood intents.
        return Math.min(spatialGate.minQuality, motionMinQuality);
    }

    private Solution calculate(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        SpatialTranslationSelection selection = SpatialQuerySelectors.firstValidTranslation(
                targetQuery.get(clock), spatialGate);
        if (selection == null) {
            return unavailable(Reason.SPATIAL_UNAVAILABLE, LoopTimestamp.unavailable());
        }

        LoopTimestamp timestamp = selection.timestamp();
        double observedForward = selection.solution.robotForwardInches();
        double observedLeft = selection.solution.robotLeftInches();
        double spatialQuality = selection.quality();
        if (!Double.isFinite(observedForward)
                || !Double.isFinite(observedLeft)
                || !isUnitQuality(spatialQuality)) {
            return unavailable(Reason.SPATIAL_INVALID, timestamp);
        }

        MotionAssessment motion = assessMotion(predictor.getLatestMotionDelta(), timestamp, clock);
        double effectiveForward = observedForward;
        double effectiveLeft = observedLeft;
        double quality = spatialQuality;
        Mode mode = Mode.STATIONARY_FALLBACK;
        Reason reason = motion.reason;
        if (motion.usable) {
            effectiveForward -= motion.forwardInchesPerSec * illustrativeFlightTimeSec;
            effectiveLeft -= motion.leftInchesPerSec * illustrativeFlightTimeSec;
            quality = Math.min(spatialQuality, motion.quality);
            mode = Mode.MOVING_COMPENSATED;
            reason = Reason.NONE;
        }

        if (!Double.isFinite(effectiveForward) || !Double.isFinite(effectiveLeft)) {
            return unavailable(Reason.MODEL_OUTPUT_INVALID, timestamp);
        }
        double distance = Math.hypot(effectiveForward, effectiveLeft);
        if (!Double.isFinite(distance)
                || distance < minimumModelDistanceInches
                || distance > maximumModelDistanceInches) {
            return unavailable(Reason.MODEL_DISTANCE_OUT_OF_DOMAIN, timestamp);
        }

        double flywheel = flywheelVelocityTicksPerSecByDistance.applyAsDouble(distance);
        double hood = hoodPositionByDistance.applyAsDouble(distance);
        if (!inside(flywheel,
                minimumFlywheelVelocityTicksPerSec,
                maximumFlywheelVelocityTicksPerSec)
                || !inside(hood, minimumHoodPosition, maximumHoodPosition)) {
            return unavailable(Reason.MODEL_OUTPUT_INVALID, timestamp);
        }

        double turretAngleRad = Math.atan2(effectiveLeft, effectiveForward);
        PlantTargetRequest turretRequest = PlantTargetRequest.observedEquivalentPosition(
                TURRET_REQUEST_ID, turretAngleRad, quality, timestamp);
        return new Solution(
                mode,
                reason,
                timestamp,
                effectiveForward,
                effectiveLeft,
                quality,
                turretRequest,
                flywheel,
                hood
        );
    }

    private MotionAssessment assessMotion(MotionDelta delta,
                                              LoopTimestamp observationTimestamp,
                                              LoopClock clock) {
        if (delta == null || !delta.hasDelta) {
            if (delta != null) {
                double startAgeSec = delta.startTimestamp.ageSec(clock);
                double endAgeSec = delta.endTimestamp.ageSec(clock);
                if ((delta.startTimestamp.isAvailable() && !Double.isFinite(startAgeSec))
                        || (delta.endTimestamp.isAvailable() && !Double.isFinite(endAgeSec))) {
                    return MotionAssessment.unusable(Reason.MOTION_RESET_INVALIDATED);
                }
            }
            return MotionAssessment.unusable(Reason.MOTION_UNAVAILABLE);
        }

        double durationSec = delta.durationSec();
        double endAgeSec = delta.endTimestamp.ageSec(clock);
        if (!Double.isFinite(durationSec) || !Double.isFinite(endAgeSec)) {
            return MotionAssessment.unusable(Reason.MOTION_RESET_INVALIDATED);
        }
        if (endAgeSec > motionMaxAgeSec) {
            return MotionAssessment.unusable(Reason.MOTION_STALE);
        }

        double timestampDifferenceSec = delta.endTimestamp.secondsSince(observationTimestamp);
        if (!Double.isFinite(timestampDifferenceSec)) {
            return MotionAssessment.unusable(Reason.MOTION_RESET_INVALIDATED);
        }
        if (timestampDifferenceSec != 0.0) {
            return MotionAssessment.unusable(Reason.MOTION_TIMESTAMP_MISMATCH);
        }
        if (!(durationSec > 0.0)
                || durationSec > motionMaxIntervalSec
                || !isUnitQuality(delta.quality)
                || delta.quality < motionMinQuality
                || !Double.isFinite(delta.deltaPose.xInches)
                || !Double.isFinite(delta.deltaPose.yInches)
                || !Double.isFinite(delta.deltaPose.yawRad)) {
            return MotionAssessment.unusable(Reason.MOTION_INVALID);
        }

        // MotionDelta translation is expressed in the previous robot frame. Rotate by the
        // negative relative yaw so the velocity matches the current robot-frame target vector.
        double deltaYawRad = delta.planarYawDeltaRad();
        double cos = Math.cos(deltaYawRad);
        double sin = Math.sin(deltaYawRad);
        double currentForwardDisplacement =
                cos * delta.deltaPose.xInches + sin * delta.deltaPose.yInches;
        double currentLeftDisplacement =
                -sin * delta.deltaPose.xInches + cos * delta.deltaPose.yInches;
        double forwardVelocity = currentForwardDisplacement / durationSec;
        double leftVelocity = currentLeftDisplacement / durationSec;
        if (!Double.isFinite(forwardVelocity) || !Double.isFinite(leftVelocity)) {
            return MotionAssessment.unusable(Reason.MOTION_INVALID);
        }
        return MotionAssessment.usable(forwardVelocity, leftVelocity, delta.quality);
    }

    private Solution unavailable(Reason reason, LoopTimestamp timestamp) {
        return new Solution(
                Mode.UNAVAILABLE,
                reason,
                timestamp,
                0.0,
                0.0,
                0.0,
                PlantTargetRequest.none("coordinated shot unavailable: " + reason.name()),
                unavailableFlywheelVelocityTicksPerSec,
                unavailableHoodPosition
        );
    }

    private void resetOwnedCalculation() {
        targetQuery.reset();
        calculation.reset();
    }

    private static double requireFinite(double value, String field) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(field + " must be finite, got " + value);
        }
        return value;
    }

    private static double requireNonNegative(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(field + " must be finite and >= 0, got " + value);
        }
        return value;
    }

    private static double requireQuality(double value, String field) {
        if (!isUnitQuality(value)) {
            throw new IllegalArgumentException(field + " must be finite in [0, 1], got " + value);
        }
        return value;
    }

    private static double requireOrderedMaximum(double value, double minimum, String field) {
        if (!Double.isFinite(value) || value < minimum) {
            throw new IllegalArgumentException(
                    field + " must be finite and >= its minimum, got " + value);
        }
        return value;
    }

    private static double requireInside(double value, double minimum, double maximum, String field) {
        if (!inside(value, minimum, maximum)) {
            throw new IllegalArgumentException(
                    field + " must be finite in [" + minimum + ", " + maximum + "], got " + value);
        }
        return value;
    }

    private static boolean isUnitQuality(double value) {
        return Double.isFinite(value) && value >= 0.0 && value <= 1.0;
    }

    private static boolean inside(double value, double minimum, double maximum) {
        return Double.isFinite(value) && value >= minimum && value <= maximum;
    }

    private static final class MotionAssessment {
        private final boolean usable;
        private final Reason reason;
        private final double forwardInchesPerSec;
        private final double leftInchesPerSec;
        private final double quality;

        private MotionAssessment(boolean usable,
                                 Reason reason,
                                 double forwardInchesPerSec,
                                 double leftInchesPerSec,
                                 double quality) {
            this.usable = usable;
            this.reason = reason;
            this.forwardInchesPerSec = forwardInchesPerSec;
            this.leftInchesPerSec = leftInchesPerSec;
            this.quality = quality;
        }

        private static MotionAssessment usable(double forward, double left, double quality) {
            return new MotionAssessment(true, Reason.NONE, forward, left, quality);
        }

        private static MotionAssessment unusable(Reason reason) {
            return new MotionAssessment(false, reason, 0.0, 0.0, 0.0);
        }
    }
}
