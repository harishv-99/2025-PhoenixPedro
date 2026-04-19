package edu.ftcphoenix.fw.spatial;

import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.TimeAwareSource;
import edu.ftcphoenix.fw.core.source.TimeAwareSources;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Spatial solve lane backed by live AprilTag observations.
 *
 * <p>This lane can use a fixed camera mount or a timestamp-aware dynamic camera mount. Dynamic
 * mounts matter for turret-mounted cameras and other moving sensor rigs: the tag observation may be
 * older than the current loop, so the camera pose used for geometry should match the camera-frame
 * timestamp whenever possible.</p>
 */
public final class AprilTagSpatialSolveLane implements SpatialSolveLane {

    public static final double DEFAULT_MAX_AGE_SEC = 0.50;

    private final AprilTagSensor sensor;
    private final TimeAwareSource<CameraMountConfig> cameraMount;
    private final double maxAgeSec;
    private final FixedTagFieldPoseSolver.Config fieldPoseSolverConfig;

    /**
     * Creates a live-AprilTag lane for a fixed camera mount.
     */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor, CameraMountConfig cameraMount) {
        this(sensor, TimeAwareSources.fixed(cameraMount), DEFAULT_MAX_AGE_SEC, FixedTagFieldPoseSolver.Config.defaults());
    }

    /** Creates a live-AprilTag lane for a fixed camera mount and explicit maximum observation age. */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor,
                                    CameraMountConfig cameraMount,
                                    double maxAgeSec) {
        this(sensor, TimeAwareSources.fixed(cameraMount), maxAgeSec, FixedTagFieldPoseSolver.Config.defaults());
    }

    /**
     * Creates a live-AprilTag lane for a fixed camera mount with explicit field-pose bridge tuning.
     */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor,
                                    CameraMountConfig cameraMount,
                                    double maxAgeSec,
                                    FixedTagFieldPoseSolver.Config fieldPoseSolverConfig) {
        this(sensor, TimeAwareSources.fixed(cameraMount), maxAgeSec, fieldPoseSolverConfig);
    }

    /**
     * Creates a live-AprilTag lane for a dynamic timestamp-aware camera mount.
     */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor,
                                    TimeAwareSource<CameraMountConfig> cameraMount,
                                    double maxAgeSec) {
        this(sensor, cameraMount, maxAgeSec, FixedTagFieldPoseSolver.Config.defaults());
    }

    /**
     * Creates a live-AprilTag lane.
     *
     * @param sensor                 tag sensor source
     * @param cameraMount            robot->camera mount provider; may be fixed or timestamp-aware
     * @param maxAgeSec              maximum acceptable age for tag observations
     * @param fieldPoseSolverConfig  config used only when this lane bridges live tags into a field pose
     */
    public AprilTagSpatialSolveLane(AprilTagSensor sensor,
                                    TimeAwareSource<CameraMountConfig> cameraMount,
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
        double timestampSec = detections != null ? detections.frameTimestampSec(request.clock) : Double.NaN;
        CameraMountConfig mountAtFrame = cameraMount.getAt(request.clock, timestampSec);

        LiveFieldPose liveFieldPose = estimateFieldPoseFromAprilTags(
                detections,
                request.fixedAprilTagLayout,
                mountAtFrame,
                timestampSec
        );

        TranslationSolution translation = solveTranslation(request, detections, mountAtFrame, timestampSec, liveFieldPose);
        FacingSolution facing = solveFacing(request, detections, mountAtFrame, timestampSec, liveFieldPose);

        return SpatialLaneResult.of(
                translation,
                facing,
                SpatialQuerySupport.translationSelectionSnapshot(request.translationTarget, request.clock, detections, maxAgeSec),
                SpatialQuerySupport.facingSelectionSnapshot(request.facingTarget, request.clock, detections, maxAgeSec)
        );
    }

    private TranslationSolution solveTranslation(SpatialSolveRequest request,
                                                 AprilTagDetections detections,
                                                 CameraMountConfig mountAtFrame,
                                                 double timestampSec,
                                                 LiveFieldPose liveFieldPose) {
        Pose2d frame = request.robotToTranslationFrameAt(timestampSec);
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
                            frame,
                            fieldToTargetPoint,
                            true,
                            liveFieldPose.rangeInches,
                            liveFieldPose.quality,
                            liveFieldPose.ageSec,
                            liveFieldPose.timestampSec
                    );
                }
            }
            return null;
        }

        Pose2d robotToTargetPoint = SpatialQuerySupport.resolveRobotPointDirect(
                request.clock,
                ((SpatialTargets.ReferencePointTarget) request.translationTarget).reference,
                detections,
                mountAtFrame,
                maxAgeSec
        );
        if (robotToTargetPoint != null) {
            double range = Math.hypot(robotToTargetPoint.xInches, robotToTargetPoint.yInches);
            double ageSec = detections != null ? detections.ageSec : Double.POSITIVE_INFINITY;
            return SpatialSolveMath.translationFromRobotPoint(
                    frame,
                    robotToTargetPoint,
                    Double.isFinite(range),
                    range,
                    1.0,
                    ageSec,
                    timestampSec
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
                        frame,
                        fieldToTargetPoint,
                        true,
                        liveFieldPose.rangeInches,
                        liveFieldPose.quality,
                        liveFieldPose.ageSec,
                        liveFieldPose.timestampSec
                );
            }
        }
        return null;
    }

    private FacingSolution solveFacing(SpatialSolveRequest request,
                                       AprilTagDetections detections,
                                       CameraMountConfig mountAtFrame,
                                       double timestampSec,
                                       LiveFieldPose liveFieldPose) {
        if (request.facingTarget == null) {
            return null;
        }
        Pose2d facingFrame = request.robotToFacingFrameAt(timestampSec);

        if (request.facingTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            SpatialTargets.ReferenceFrameHeadingTarget target = (SpatialTargets.ReferenceFrameHeadingTarget) request.facingTarget;
            Pose2d robotToFrame = SpatialQuerySupport.resolveRobotFrameDirect(
                    request.clock,
                    target.reference,
                    detections,
                    mountAtFrame,
                    maxAgeSec
            );
            if (robotToFrame != null) {
                double ageSec = detections != null ? detections.ageSec : Double.POSITIVE_INFINITY;
                return SpatialSolveMath.facingFromRobotHeading(
                        facingFrame,
                        robotToFrame.headingRad + target.headingOffsetRad,
                        1.0,
                        ageSec,
                        timestampSec
                );
            }
        } else if (request.facingTarget instanceof SpatialTargets.ReferencePointTarget) {
            Pose2d robotToPoint = SpatialQuerySupport.resolveRobotPointDirect(
                    request.clock,
                    ((SpatialTargets.ReferencePointTarget) request.facingTarget).reference,
                    detections,
                    mountAtFrame,
                    maxAgeSec
            );
            if (robotToPoint != null) {
                double ageSec = detections != null ? detections.ageSec : Double.POSITIVE_INFINITY;
                return SpatialSolveMath.facingFromRobotPoint(facingFrame, robotToPoint, 1.0, ageSec, timestampSec);
            }
        }

        if (liveFieldPose != null) {
            if (request.facingTarget instanceof SpatialTargets.FieldHeading) {
                return SpatialSolveMath.facingFromFieldHeading(
                        liveFieldPose.fieldToRobot,
                        facingFrame,
                        ((SpatialTargets.FieldHeading) request.facingTarget).fieldHeadingRad,
                        liveFieldPose.quality,
                        liveFieldPose.ageSec,
                        liveFieldPose.timestampSec
                );
            }
            if (request.facingTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
                SpatialTargets.ReferenceFrameHeadingTarget target = (SpatialTargets.ReferenceFrameHeadingTarget) request.facingTarget;
                Pose2d fieldToFrame = SpatialQuerySupport.resolveFieldFrameHeadingTarget(
                        target,
                        request.fixedAprilTagLayout,
                        request.clock
                );
                if (fieldToFrame != null) {
                    return SpatialSolveMath.facingFromFieldHeading(
                            liveFieldPose.fieldToRobot,
                            facingFrame,
                            fieldToFrame.headingRad + target.headingOffsetRad,
                            liveFieldPose.quality,
                            liveFieldPose.ageSec,
                            liveFieldPose.timestampSec
                    );
                }
            }

            Pose2d fieldToFacingPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.facingTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToFacingPoint != null) {
                return SpatialSolveMath.facingFromFieldPoint(
                        liveFieldPose.fieldToRobot,
                        facingFrame,
                        fieldToFacingPoint,
                        liveFieldPose.quality,
                        liveFieldPose.ageSec,
                        liveFieldPose.timestampSec
                );
            }
        }

        return null;
    }

    private LiveFieldPose estimateFieldPoseFromAprilTags(AprilTagDetections detections,
                                                         TagLayout layout,
                                                         CameraMountConfig mountAtFrame,
                                                         double timestampSec) {
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
                mountAtFrame,
                fieldPoseSolverConfig
        );
        if (!solve.hasPose) {
            return null;
        }
        return new LiveFieldPose(solve.toPose2d(), solve.rangeInches, solve.quality, detections.ageSec, timestampSec);
    }

    private static final class LiveFieldPose {
        final Pose2d fieldToRobot;
        final double rangeInches;
        final double quality;
        final double ageSec;
        final double timestampSec;

        LiveFieldPose(Pose2d fieldToRobot, double rangeInches, double quality, double ageSec, double timestampSec) {
            this.fieldToRobot = fieldToRobot;
            this.rangeInches = rangeInches;
            this.quality = quality;
            this.ageSec = ageSec;
            this.timestampSec = timestampSec;
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
