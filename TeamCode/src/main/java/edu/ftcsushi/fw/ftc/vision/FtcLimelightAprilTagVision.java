package edu.ftcsushi.fw.ftc.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane.ResultSnapshot;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.FtcFrames;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Borrowed Limelight AprilTag capability. The camera owner retains polling, pipeline selection,
 * and shutdown; this view exposes confirmed tag evidence and the narrow MT2 orientation write.
 */
public final class FtcLimelightAprilTagVision implements AprilTagVision {
    private final FtcLimelightVisionLane owner;
    private final FtcLimelightVisionLane.AprilTagConfig cfg;
    private final CameraMountConfig mount;
    private final AprilTagSensor tagSensor;

    FtcLimelightAprilTagVision(FtcLimelightVisionLane owner,
                              FtcLimelightVisionLane.AprilTagConfig config,
                              CameraMountConfig mount) {
        this.owner = Objects.requireNonNull(owner, "owner");
        this.cfg = config.copy();
        this.mount = Objects.requireNonNull(mount, "mount");
        this.tagSensor = new LimelightAprilTagSensor();
    }

    /** Publishes finite field yaw through the existing owner for MegaTag2 estimation. */
    public boolean updateRobotFieldYawRad(double fieldYawRad) {
        return owner.updateRobotFieldYawRad(fieldYawRad);
    }

    /** {@inheritDoc} */
    @Override
    public CameraMountConfig cameraMountConfig() {
        return mount;
    }

    /** {@inheritDoc} */
    @Override
    public AprilTagSensor tagSensor() {
        return tagSensor;
    }

    /**
     * Returns readiness for the configured AprilTag purpose, not merely for any selected pipeline.
     */
    @Override
    public VisionReadiness readiness(LoopClock clock) {
        synchronized (owner) {
            Objects.requireNonNull(clock, "clock");
            if (owner.requestedPipelineIndex() != cfg.pipelineIndex) {
                return VisionReadiness.notReady("AprilTag vision requires Limelight pipeline "
                        + cfg.pipelineIndex + "; currently requested pipeline is "
                        + owner.requestedPipelineIndex());
            }
            return owner.pipelineReadiness(clock);
        }
    }

    /**
     * Returns a confirmed result only while the configured AprilTag pipeline is selected and ready.
     *
     * @param clock shared loop clock
     * @return confirmed AprilTag-pipeline result, or an unavailable snapshot
     */
    public ResultSnapshot confirmedAprilTagResult(LoopClock clock) {
        synchronized (owner) {
            if (!readiness(clock).isReady()) {
                return confirmedUnavailableResult();
            }
            return owner.confirmedPipelineResult(clock);
        }
    }

    /** Adds specialization diagnostics without reading the device again. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        synchronized (owner) {
            String p = prefix == null || prefix.isEmpty() ? "limelightVision" : prefix;
            dbg.addData(p + ".aprilTagPipeline", cfg.pipelineIndex)
                    .addData(p + ".aprilTagPipelineRequested",
                            owner.requestedPipelineIndex() == cfg.pipelineIndex);
            mount.debugDump(dbg, p + ".cameraMount");
        }
    }

    private static ResultSnapshot confirmedUnavailableResult() {
        return FtcLimelightVisionLane.unavailableResult();
    }

    static Pose3d sushiCameraToTagPose(Pose3D limelightPose) {
        if (!FtcLimelightVisionLane.isUsableSdkPose(limelightPose)) {
            return null;
        }
        Position position = limelightPose.getPosition();
        Position inches = position.toUnit(DistanceUnit.INCH);

        YawPitchRollAngles ypr = limelightPose.getOrientation();
        double yawRad = 0.0;
        double pitchRad = 0.0;
        double rollRad = 0.0;
        if (ypr != null) {
            yawRad = ypr.getYaw(AngleUnit.RADIANS);
            pitchRad = ypr.getPitch(AngleUnit.RADIANS);
            rollRad = ypr.getRoll(AngleUnit.RADIANS);
        }

        Pose3d limelightCameraPose = new Pose3d(
                inches.x,
                inches.y,
                inches.z,
                yawRad,
                pitchRad,
                rollRad
        );
        return FtcFrames.toSushiFromFtcLocalizationCameraAxes(limelightCameraPose);
    }

    private final class LimelightAprilTagSensor implements AprilTagSensor {

        private long lastCycle = Long.MIN_VALUE;
        private long lastPipelineGeneration = Long.MIN_VALUE;
        private AprilTagDetections lastDetections = AprilTagDetections.none();

        @Override
        public AprilTagDetections get(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            long cycle = clock.cycle();
            long generation = owner.pipelineGeneration();

            // A close or pipeline transition must invalidate cached detections immediately, even
            // when robot policy changes modes more than once within one OpMode cycle.
            if (!readiness(clock).isReady()) {
                lastCycle = cycle;
                lastPipelineGeneration = generation;
                lastDetections = AprilTagDetections.none();
                return lastDetections;
            }
            if (cycle == lastCycle && generation == lastPipelineGeneration) {
                return lastDetections;
            }
            AprilTagDetections next = readDetections(clock);
            lastCycle = cycle;
            lastPipelineGeneration = generation;
            lastDetections = next;
            return lastDetections;
        }

        @Override
        public void reset() {
            lastCycle = Long.MIN_VALUE;
            lastPipelineGeneration = Long.MIN_VALUE;
            lastDetections = AprilTagDetections.none();
        }

        private AprilTagDetections readDetections(LoopClock clock) {
            ResultSnapshot result = confirmedAprilTagResult(clock);
            if (!result.hasResult()) {
                return AprilTagDetections.none();
            }

            LoopTimestamp frameTimestamp = result.frameTimestamp();
            List<LLResultTypes.FiducialResult> fiducials = result.fiducialResults();
            if (!result.isTargetValid() || fiducials.isEmpty()) {
                return AprilTagDetections.fromFrame(
                        frameTimestamp,
                        Collections.<AprilTagObservation>emptyList()
                );
            }

            ArrayList<AprilTagObservation> out =
                    new ArrayList<AprilTagObservation>(fiducials.size());
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                Pose3d cameraToTagPose = sushiCameraToTagPose(
                        fiducial.getTargetPoseCameraSpace());
                if (cameraToTagPose != null) {
                    out.add(AprilTagObservation.target(
                            fiducial.getFiducialId(),
                            cameraToTagPose
                    ));
                }
            }
            return out.isEmpty()
                    ? AprilTagDetections.fromFrame(
                            frameTimestamp,
                            Collections.<AprilTagObservation>emptyList()
                    )
                    : AprilTagDetections.fromFrame(frameTimestamp, out);
        }
    }
}
