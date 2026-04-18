package edu.ftcphoenix.fw.spatial;

import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Spatial solve lane backed by live AprilTag observations.
 *
 * <p>This lane first attempts direct robot-relative solves from the live tag observations. When a
 * target instead needs a trusted field pose (for example a field-fixed point or heading), the lane
 * may bridge through a temporary fixed-tag field-pose solve using the supplied
 * {@link FixedTagFieldPoseSolver.Config} and the query's fixed AprilTag layout.</p>
 */
public final class AprilTagSpatialSolveLane implements SpatialSolveLane {

    public static final double DEFAULT_MAX_AGE_SEC = 0.50;

    private final AprilTagSensor sensor;
    private final CameraMountConfig cameraMount;
    private final double maxAgeSec;
    private final FixedTagFieldPoseSolver.Config fieldPoseSolverConfig;

    /**
     * Creates a live-AprilTag solve lane with default freshness and field-pose bridge tuning.
     */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor, CameraMountConfig cameraMount) {
        this(sensor, cameraMount, DEFAULT_MAX_AGE_SEC, FixedTagFieldPoseSolver.Config.defaults());
    }

    /**
     * Creates a live-AprilTag solve lane with an explicit maximum observation age.
     */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor,
                                    CameraMountConfig cameraMount,
                                    double maxAgeSec) {
        this(sensor, cameraMount, maxAgeSec, FixedTagFieldPoseSolver.Config.defaults());
    }

    /**
     * Creates a live-AprilTag solve lane with explicit observation freshness and fixed-tag field-pose bridge tuning.
     */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor,
                                    CameraMountConfig cameraMount,
                                    double maxAgeSec,
                                    FixedTagFieldPoseSolver.Config fieldPoseSolverConfig) {
        this.sensor = Objects.requireNonNull(sensor, "sensor");
        this.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
        this.maxAgeSec = maxAgeSec;
        this.fieldPoseSolverConfig = fieldPoseSolverConfig != null
                ? FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                fieldPoseSolverConfig,
                "AprilTagSpatialSolveLane.fieldPoseSolverConfig"
        )
                : FixedTagFieldPoseSolver.Config.defaults();
    }

    @Override
    public SpatialLaneResult solve(SpatialSolveRequest request) {
        AprilTagDetections detections = sensor.get(request.clock);
        LiveFieldPose liveFieldPose = estimateFieldPoseFromAprilTags(detections, request.fixedAprilTagLayout);

        TranslationSolution translation = solveTranslation(request, detections, liveFieldPose);
        AimSolution aim = solveAim(request, detections, liveFieldPose);

        return SpatialLaneResult.of(
                translation,
                aim,
                SpatialQuerySupport.translationSelectionSnapshot(request.translationTarget, request.clock, detections, maxAgeSec),
                SpatialQuerySupport.aimSelectionSnapshot(request.aimTarget, request.clock, detections, maxAgeSec)
        );
    }

    private TranslationSolution solveTranslation(SpatialSolveRequest request,
                                                 AprilTagDetections detections,
                                                 LiveFieldPose liveFieldPose) {
        if (!(request.translationTarget instanceof SpatialTargets.ReferencePointTarget)) {
            if (liveFieldPose != null) {
                Pose2d fieldToTargetPoint = SpatialQuerySupport.resolveFieldPointTarget(
                        request.translationTarget,
                        request.fixedAprilTagLayout,
                        request.clock
                );
                if (fieldToTargetPoint != null) {
                    return SpatialSolveMath.translationFromFieldPose(
                            liveFieldPose.fieldToRobot,
                            request.robotToTranslationFrame,
                            fieldToTargetPoint,
                            true,
                            liveFieldPose.rangeInches,
                            liveFieldPose.quality,
                            liveFieldPose.ageSec
                    );
                }
            }
            return null;
        }

        Pose2d robotToTargetPoint = SpatialQuerySupport.resolveRobotPointDirect(
                request.clock,
                ((SpatialTargets.ReferencePointTarget) request.translationTarget).reference,
                detections,
                cameraMount,
                maxAgeSec
        );
        if (robotToTargetPoint != null) {
            double range = Math.hypot(robotToTargetPoint.xInches, robotToTargetPoint.yInches);
            double ageSec = detections != null ? detections.ageSec : Double.POSITIVE_INFINITY;
            return SpatialSolveMath.translationFromRobotPoint(
                    request.robotToTranslationFrame,
                    robotToTargetPoint,
                    Double.isFinite(range),
                    range,
                    1.0,
                    ageSec
            );
        }

        if (liveFieldPose != null) {
            Pose2d fieldToTargetPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.translationTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToTargetPoint != null) {
                return SpatialSolveMath.translationFromFieldPose(
                        liveFieldPose.fieldToRobot,
                        request.robotToTranslationFrame,
                        fieldToTargetPoint,
                        true,
                        liveFieldPose.rangeInches,
                        liveFieldPose.quality,
                        liveFieldPose.ageSec
                );
            }
        }
        return null;
    }

    private AimSolution solveAim(SpatialSolveRequest request,
                                 AprilTagDetections detections,
                                 LiveFieldPose liveFieldPose) {
        if (request.aimTarget == null) {
            return null;
        }

        if (request.aimTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            SpatialTargets.ReferenceFrameHeadingTarget target = (SpatialTargets.ReferenceFrameHeadingTarget) request.aimTarget;
            Pose2d robotToFrame = SpatialQuerySupport.resolveRobotFrameDirect(
                    request.clock,
                    target.reference,
                    detections,
                    cameraMount,
                    maxAgeSec
            );
            if (robotToFrame != null) {
                double ageSec = detections != null ? detections.ageSec : Double.POSITIVE_INFINITY;
                return SpatialSolveMath.aimFromRobotHeading(
                        request.robotToAimFrame,
                        robotToFrame.headingRad + target.headingOffsetRad,
                        1.0,
                        ageSec
                );
            }
        } else if (request.aimTarget instanceof SpatialTargets.ReferencePointTarget) {
            Pose2d robotToPoint = SpatialQuerySupport.resolveRobotPointDirect(
                    request.clock,
                    ((SpatialTargets.ReferencePointTarget) request.aimTarget).reference,
                    detections,
                    cameraMount,
                    maxAgeSec
            );
            if (robotToPoint != null) {
                double ageSec = detections != null ? detections.ageSec : Double.POSITIVE_INFINITY;
                return SpatialSolveMath.aimFromRobotPoint(request.robotToAimFrame, robotToPoint, 1.0, ageSec);
            }
        }

        if (liveFieldPose != null) {
            if (request.aimTarget instanceof SpatialTargets.FieldHeading) {
                return SpatialSolveMath.aimFromFieldHeading(
                        liveFieldPose.fieldToRobot,
                        request.robotToAimFrame,
                        ((SpatialTargets.FieldHeading) request.aimTarget).fieldHeadingRad,
                        liveFieldPose.quality,
                        liveFieldPose.ageSec
                );
            }
            if (request.aimTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
                SpatialTargets.ReferenceFrameHeadingTarget target = (SpatialTargets.ReferenceFrameHeadingTarget) request.aimTarget;
                Pose2d fieldToFrame = SpatialQuerySupport.resolveFieldFrameHeadingTarget(
                        target,
                        request.fixedAprilTagLayout,
                        request.clock
                );
                if (fieldToFrame != null) {
                    return SpatialSolveMath.aimFromFieldHeading(
                            liveFieldPose.fieldToRobot,
                            request.robotToAimFrame,
                            fieldToFrame.headingRad + target.headingOffsetRad,
                            liveFieldPose.quality,
                            liveFieldPose.ageSec
                    );
                }
            }

            Pose2d fieldToAimPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.aimTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToAimPoint != null) {
                return SpatialSolveMath.aimFromFieldPoint(
                        liveFieldPose.fieldToRobot,
                        request.robotToAimFrame,
                        fieldToAimPoint,
                        liveFieldPose.quality,
                        liveFieldPose.ageSec
                );
            }
        }

        return null;
    }

    private LiveFieldPose estimateFieldPoseFromAprilTags(AprilTagDetections detections, TagLayout layout) {
        if (detections == null || layout == null || layout.ids().isEmpty()) {
            return null;
        }
        List<AprilTagObservation> observations = detections.freshMatching(layout.ids(), maxAgeSec);
        if (observations.isEmpty()) {
            return null;
        }

        FixedTagFieldPoseSolver.Result solve = FixedTagFieldPoseSolver.solve(
                observations,
                layout,
                cameraMount,
                fieldPoseSolverConfig
        );
        if (!solve.hasPose) {
            return null;
        }
        return new LiveFieldPose(solve.toPose2d(), solve.rangeInches, solve.quality, detections.ageSec);
    }

    private static final class LiveFieldPose {
        final Pose2d fieldToRobot;
        final double rangeInches;
        final double quality;
        final double ageSec;

        LiveFieldPose(Pose2d fieldToRobot, double rangeInches, double quality, double ageSec) {
            this.fieldToRobot = fieldToRobot;
            this.rangeInches = rangeInches;
            this.quality = quality;
            this.ageSec = ageSec;
        }
    }

    @Override
    public String toString() {
        return "AprilTagSpatialSolveLane{sensor=" + sensor
                + ", cameraMount=" + cameraMount
                + ", maxAgeSec=" + maxAgeSec
                + ", fieldPoseSolverConfig=" + fieldPoseSolverConfig + '}';
    }
}
