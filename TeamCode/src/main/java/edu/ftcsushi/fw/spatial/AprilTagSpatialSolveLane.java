package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.source.TimeAwareSources;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

/** Direct tag-relative geometry only; built through {@link SpatialSolveSet#builder()}. */
final class AprilTagSpatialSolveLane implements SpatialSolveLane {
    static final double DEFAULT_MAX_AGE_SEC = 0.50;
    private final AprilTagSensor sensor;
    private final TimeAwareSource<CameraMountConfig> cameraMount;
    private final double maxAgeSec;

    AprilTagSpatialSolveLane(AprilTagSensor sensor, CameraMountConfig cameraMount) {
        this(sensor, cameraMount, DEFAULT_MAX_AGE_SEC);
    }

    AprilTagSpatialSolveLane(AprilTagSensor sensor, CameraMountConfig cameraMount, double maxAgeSec) {
        this(sensor, TimeAwareSources.fixed(Objects.requireNonNull(cameraMount, "cameraMount")), maxAgeSec);
    }

    AprilTagSpatialSolveLane(AprilTagSensor sensor,
                            TimeAwareSource<CameraMountConfig> cameraMount, double maxAgeSec) {
        if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0.0) {
            throw new IllegalArgumentException("maxAgeSec must be finite and >= 0, got " + maxAgeSec);
        }
        this.sensor = Objects.requireNonNull(sensor, "sensor");
        this.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
        this.maxAgeSec = maxAgeSec;
    }

    @Override public SpatialLaneResult solve(SpatialSolveRequest request) {
        AprilTagDetections detections = sensor.get(request.clock);
        TranslationSolution translation = null;
        FacingSolution facing = null;
        if (detections != null && detections.isFresh(request.clock, maxAgeSec)
                && !detections.observations.isEmpty()) {
            LoopTimestamp timestamp = detections.frameTimestamp();
            CameraMountConfig mount = Objects.requireNonNull(cameraMount.getAt(request.clock, timestamp),
                    "cameraMount.getAt(clock, timestamp) returned null");
            translation = solveTranslation(request, detections, mount, timestamp);
            facing = solveFacing(request, detections, mount, timestamp);
        }
        return SpatialLaneResult.of(translation, facing,
                SpatialQuerySupport.translationSelectionSnapshot(request.translationTarget,
                        request.clock, detections, maxAgeSec),
                SpatialQuerySupport.facingSelectionSnapshot(request.facingTarget,
                        request.clock, detections, maxAgeSec));
    }

    /** A relative lane never interprets a field-only target through a temporary robot field pose. */
    private TranslationSolution solveTranslation(SpatialSolveRequest request,
            AprilTagDetections detections, CameraMountConfig mount, LoopTimestamp timestamp) {
        if (!(request.translationTarget instanceof SpatialTargets.ReferencePointTarget)) return null;
        Pose2d point = SpatialQuerySupport.resolveRobotPointDirect(request.clock,
                ((SpatialTargets.ReferencePointTarget) request.translationTarget).reference,
                detections, mount, maxAgeSec);
        Pose2d frame = request.robotToTranslationFrameAt(timestamp);
        if (!SpatialValidation.isFinite(point) || !SpatialValidation.isFinite(frame)) return null;
        double range = Math.hypot(point.xInches, point.yInches);
        if (!Double.isFinite(range)) return null;
        TranslationSolution solution = SpatialSolveMath.translationFromRobotPoint(frame, point,
                true, range, Double.NaN, timestamp).withDirectObservationEvidence(timestamp);
        return SpatialValidation.isFinite(solution.translationFrameToTargetPoint)
                && Double.isFinite(solution.frameDistanceInches()) ? solution : null;
    }

    private FacingSolution solveFacing(SpatialSolveRequest request, AprilTagDetections detections,
            CameraMountConfig mount, LoopTimestamp timestamp) {
        Pose2d frame = request.robotToFacingFrameAt(timestamp);
        if (!SpatialValidation.isFinite(frame)) return null;
        FacingSolution solution = null;
        if (request.facingTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            SpatialTargets.ReferenceFrameHeadingTarget target =
                    (SpatialTargets.ReferenceFrameHeadingTarget) request.facingTarget;
            Pose2d targetFrame = SpatialQuerySupport.resolveRobotFrameDirect(request.clock,
                    target.reference, detections, mount, maxAgeSec);
            if (SpatialValidation.isFinite(targetFrame)) {
                solution = SpatialSolveMath.facingFromRobotHeading(frame,
                        SpatialSolveMath.wrappedHeadingSumRad(targetFrame.headingRad, target.headingOffsetRad),
                        Double.NaN, timestamp);
            }
        } else if (request.facingTarget instanceof SpatialTargets.ReferencePointTarget) {
            Pose2d point = SpatialQuerySupport.resolveRobotPointDirect(request.clock,
                    ((SpatialTargets.ReferencePointTarget) request.facingTarget).reference,
                    detections, mount, maxAgeSec);
            if (SpatialValidation.isFinite(point)
                    && SpatialValidation.isFinite(frame.inverse().then(point))) {
                solution = SpatialSolveMath.facingFromRobotPoint(frame, point, Double.NaN, timestamp);
            }
        }
        return solution != null && Double.isFinite(solution.facingErrorRad)
                ? solution.withDirectObservationEvidence(timestamp) : null;
    }

    @Override public String toString() {
        return "RelativeAprilTags{sensor=" + sensor + ", cameraMount=" + cameraMount
                + ", maxAgeSec=" + maxAgeSec + '}';
    }
}
