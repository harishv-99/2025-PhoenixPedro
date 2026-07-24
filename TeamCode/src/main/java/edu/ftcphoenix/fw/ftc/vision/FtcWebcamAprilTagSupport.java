package edu.ftcphoenix.fw.ftc.vision;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Mat3;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.geometry.Vec3;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.FtcFrames;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/** Package-private FTC AprilTag processor construction and result conversion support. */
final class FtcWebcamAprilTagSupport {

    private static final double NANOS_PER_SECOND = 1_000_000_000.0;
    private static final Mat3 FTC_ROBOT_FROM_PHOENIX_ROBOT = new Mat3(
            0, -1, 0,
            1, 0, 0,
            0, 0, 1
    );
    private static final Mat3 PHOENIX_CAMERA_FROM_FTC_OPTICAL_CAMERA = new Mat3(
            0, 0, 1,
            -1, 0, 0,
            0, -1, 0
    );

    private FtcWebcamAprilTagSupport() {
        // Static support only.
    }

    static AprilTagProcessor createProcessor(
            CameraMountConfig cameraMount,
            AprilTagLibrary tagLibrary
    ) {
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary != null
                        ? tagLibrary
                        : AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS);
        applyCameraMount(builder, cameraMount);
        return builder.build();
    }

    private static void applyCameraMount(
            AprilTagProcessor.Builder builder,
            CameraMountConfig mount
    ) {
        Objects.requireNonNull(builder, "builder");
        SdkCameraPose cameraPose = toSdkCameraPose(mount);

        Position position = new Position(
                DistanceUnit.INCH,
                cameraPose.rightInches,
                cameraPose.forwardInches,
                cameraPose.upInches,
                0
        );
        YawPitchRollAngles orientation = new YawPitchRollAngles(
                AngleUnit.RADIANS,
                cameraPose.yawRad,
                cameraPose.pitchRad,
                cameraPose.rollRad,
                0
        );
        builder.setCameraPose(position, orientation);
    }

    /** Converts Phoenix mount semantics into the FTC SDK's robot/optical-camera convention. */
    static SdkCameraPose toSdkCameraPose(CameraMountConfig mount) {
        Pose3d robotToCameraPose = Objects.requireNonNull(mount, "cameraMount")
                .robotToCameraPose();

        Vec3 ftcTranslation = FTC_ROBOT_FROM_PHOENIX_ROBOT
                .mul(robotToCameraPose.translation());
        Mat3 sdkRotation = FTC_ROBOT_FROM_PHOENIX_ROBOT
                .mul(robotToCameraPose.rotation())
                .mul(PHOENIX_CAMERA_FROM_FTC_OPTICAL_CAMERA);
        GeneralMatrixF rotationMatrix = new GeneralMatrixF(3, 3, new float[]{
                (float) sdkRotation.m00, (float) sdkRotation.m01, (float) sdkRotation.m02,
                (float) sdkRotation.m10, (float) sdkRotation.m11, (float) sdkRotation.m12,
                (float) sdkRotation.m20, (float) sdkRotation.m21, (float) sdkRotation.m22
        });
        Orientation sdkOrientation = Orientation.getOrientation(
                rotationMatrix,
                AxesReference.INTRINSIC,
                AxesOrder.ZXZ,
                AngleUnit.RADIANS
        );
        return new SdkCameraPose(
                ftcTranslation.x,
                ftcTranslation.y,
                ftcTranslation.z,
                sdkOrientation.firstAngle,
                sdkOrientation.secondAngle,
                sdkOrientation.thirdAngle
        );
    }

    /** Package-private immutable test seam for the SDK camera-pose conversion. */
    static final class SdkCameraPose {
        final double rightInches;
        final double forwardInches;
        final double upInches;
        final double yawRad;
        final double pitchRad;
        final double rollRad;

        SdkCameraPose(
                double rightInches,
                double forwardInches,
                double upInches,
                double yawRad,
                double pitchRad,
                double rollRad
        ) {
            this.rightInches = rightInches;
            this.forwardInches = forwardInches;
            this.upInches = upInches;
            this.yawRad = yawRad;
            this.pitchRad = pitchRad;
            this.rollRad = rollRad;
        }
    }

    static final class PortalAprilTagSensor implements AprilTagSensor {
        private final FtcWebcamVisionPortalLane owner;
        private final AprilTagProcessor processor;

        private long lastCycle = Long.MIN_VALUE;
        private long lastDataGeneration = Long.MIN_VALUE;
        private AprilTagDetections lastDetections = AprilTagDetections.none();

        PortalAprilTagSensor(
                FtcWebcamVisionPortalLane owner,
                AprilTagProcessor processor
        ) {
            this.owner = Objects.requireNonNull(owner, "owner");
            this.processor = Objects.requireNonNull(processor, "processor");
            if (!owner.ownsProcessor(processor)) {
                throw new IllegalArgumentException(
                        "AprilTag processor must be registered with its webcam owner");
            }
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            long cycle = clock.cycle();
            long dataGeneration = owner.processorDataGeneration(processor);

            // Check readiness before returning a cached same-cycle value. A disable or stream stop
            // must make old detections unavailable immediately, even within the same loop cycle.
            if (!owner.processorReadiness(processor).isReady()) {
                lastCycle = cycle;
                lastDataGeneration = dataGeneration;
                lastDetections = AprilTagDetections.none();
                return lastDetections;
            }
            if (cycle == lastCycle && dataGeneration == lastDataGeneration) {
                return lastDetections;
            }

            AprilTagDetections next =
                    readDetections(clock, owner.acceptProcessorFramesAfterNanos(processor));
            lastCycle = cycle;
            lastDataGeneration = dataGeneration;
            lastDetections = next;
            return lastDetections;
        }

        @Override
        public void reset() {
            lastCycle = Long.MIN_VALUE;
            lastDataGeneration = Long.MIN_VALUE;
            lastDetections = AprilTagDetections.none();
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "webcamVision.tags" : prefix;
            dbg.addData(p + ".detections.count", lastDetections.observations.size());
            dbg.addData(
                    p + ".detections.visibleIds",
                    visibleIds(lastDetections).toString()
            );
            dbg.addData(
                    p + ".detections.hasFrameTimestamp",
                    lastDetections.frameTimestamp().isAvailable()
            );
            dbg.addData(p + ".dataGeneration", lastDataGeneration);
        }

        private AprilTagDetections readDetections(LoopClock clock, long acceptFramesAfterNanos) {
            List<AprilTagDetection> detections = processor.getDetections();
            if (detections == null || detections.isEmpty()) {
                return AprilTagDetections.none();
            }

            long nowNanos = owner.monotonicNowNanos();
            ArrayList<PendingObservation> pending =
                    new ArrayList<PendingObservation>(detections.size());
            double snapshotAgeSec = Double.NEGATIVE_INFINITY;

            for (AprilTagDetection detection : detections) {
                if (detection == null) {
                    continue;
                }
                long frameTime = detection.frameAcquisitionNanoTime;
                if (frameTime == 0L
                        || frameTime <= acceptFramesAfterNanos
                        || frameTime > nowNanos) {
                    // A retained pre-enable/pre-resume result is not evidence from this generation.
                    // A future timestamp is not a trustworthy current frame either.
                    continue;
                }
                if (detection.rawPose == null && detection.ftcPose == null) {
                    continue;
                }

                Pose3d cameraToTagPose = detection.rawPose != null
                        ? cameraToTagFromRawPose(detection.rawPose)
                        : null;
                if (cameraToTagPose == null && detection.ftcPose != null) {
                    Pose3d ftcCameraToTag = new Pose3d(
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z,
                            detection.ftcPose.yaw,
                            detection.ftcPose.roll,
                            detection.ftcPose.pitch
                    );
                    cameraToTagPose = FtcFrames.toPhoenixFromFtcDetectionFrame(ftcCameraToTag);
                }
                if (cameraToTagPose == null) {
                    continue;
                }

                Pose3d fieldToRobotPose = null;
                if (detection.robotPose != null) {
                    Position position = detection.robotPose.getPosition();
                    YawPitchRollAngles orientation = detection.robotPose.getOrientation();
                    fieldToRobotPose = new Pose3d(
                            position.x,
                            position.y,
                            position.z,
                            orientation.getYaw(AngleUnit.RADIANS),
                            orientation.getPitch(AngleUnit.RADIANS),
                            orientation.getRoll(AngleUnit.RADIANS)
                    );
                }

                pending.add(new PendingObservation(
                        detection.id,
                        cameraToTagPose,
                        fieldToRobotPose
                ));
                double ageSec = (nowNanos - frameTime) / NANOS_PER_SECOND;
                // This value type represents one coherent frame. Until the camera-identity work in
                // VISION-02 can reject mixed SDK lists, use the oldest accepted capture so no older
                // geometry is made fresher by a newer list member.
                if (ageSec > snapshotAgeSec) {
                    snapshotAgeSec = ageSec;
                }
            }

            if (pending.isEmpty()) {
                return AprilTagDetections.none();
            }
            if (!Double.isFinite(snapshotAgeSec)) {
                snapshotAgeSec = 0.0;
            }
            LoopTimestamp frameTimestamp = clock.timestampSecondsAgo(snapshotAgeSec);
            ArrayList<AprilTagObservation> observations =
                    new ArrayList<AprilTagObservation>(pending.size());
            for (PendingObservation observation : pending) {
                observations.add(observation.fieldToRobotPose != null
                        ? AprilTagObservation.target(
                                observation.id,
                                observation.cameraToTagPose,
                                observation.fieldToRobotPose,
                                frameTimestamp
                        )
                        : AprilTagObservation.target(
                                observation.id,
                                observation.cameraToTagPose,
                                frameTimestamp
                        ));
            }
            return AprilTagDetections.of(frameTimestamp, observations);
        }

        private static LinkedHashSet<Integer> visibleIds(AprilTagDetections detections) {
            LinkedHashSet<Integer> ids = new LinkedHashSet<Integer>();
            for (AprilTagObservation observation : detections.observations) {
                ids.add(observation.id);
            }
            return ids;
        }
    }

    /** Geometry captured before one frame timestamp is attached to every observation. */
    private static final class PendingObservation {
        final int id;
        final Pose3d cameraToTagPose;
        final Pose3d fieldToRobotPose;

        PendingObservation(int id, Pose3d cameraToTagPose, Pose3d fieldToRobotPose) {
            this.id = id;
            this.cameraToTagPose = cameraToTagPose;
            this.fieldToRobotPose = fieldToRobotPose;
        }
    }

    private static Pose3d cameraToTagFromRawPose(AprilTagPoseRaw rawPose) {
        if (rawPose == null || rawPose.R == null) {
            return null;
        }

        // FTC raw camera (+X right, +Y down, +Z forward) -> Phoenix (+X forward, +Y left, +Z up).
        double xInches = rawPose.z;
        double yInches = -rawPose.x;
        double zInches = -rawPose.y;

        // Change the camera basis only, preserving the FTC tag-frame convention used by layouts.
        Mat3 rawRotation = mat3FromMatrixF(rawPose.R);
        Mat3 cameraRotation =
                FtcFrames.phoenixFromAprilTagRawCameraFrame().mul(rawRotation);
        Mat3.YawPitchRoll orientation = Mat3.toYawPitchRoll(cameraRotation);
        return new Pose3d(
                xInches,
                yInches,
                zInches,
                orientation.yawRad,
                orientation.pitchRad,
                orientation.rollRad
        );
    }

    private static Mat3 mat3FromMatrixF(MatrixF matrix) {
        return new Mat3(
                matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2),
                matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2),
                matrix.get(2, 0), matrix.get(2, 1), matrix.get(2, 2)
        );
    }
}
