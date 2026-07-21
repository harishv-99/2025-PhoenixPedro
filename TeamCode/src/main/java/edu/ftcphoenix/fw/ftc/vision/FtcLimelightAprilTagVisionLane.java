package edu.ftcphoenix.fw.ftc.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcFrames;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * AprilTag-specialized form of {@link FtcLimelightVisionLane}.
 *
 * <p>The inherited owner holds the one physical Limelight. This specialization adds the configured
 * AprilTag pipeline meaning, camera extrinsics, and conversion to the framework's shared
 * {@link AprilTagSensor}. Switching the owner to another pipeline intentionally makes this lane's
 * AprilTag view not ready until the configured tag pipeline is selected and confirmed again.</p>
 */
public final class FtcLimelightAprilTagVisionLane extends FtcLimelightVisionLane
        implements AprilTagVisionLane {

    /** Configuration for the Limelight AprilTag specialization. */
    public static final class Config {

        /** FTC hardware-map name for the Limelight device. */
        public String hardwareName = "limelight";

        /** Limelight pipeline configured to produce AprilTag results. */
        public int pipelineIndex = 0;

        /**
         * Requested Control Hub polling rate in Hertz.
         *
         * <p>Keep the 100 Hz default unless real-device testing justifies another value; very low
         * rates can make the FTC SDK's short-window connection signal appear intermittent.</p>
         */
        public int pollRateHz = 100;

        /** Maximum age of a result that may confirm the AprilTag pipeline. */
        public double maxResultAgeSec = 0.25;

        /** Camera extrinsics expressed in the Phoenix robot frame. */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        private Config() {
            // Defaults assigned above.
        }

        /** @return a new mutable config initialized with framework defaults. */
        public static Config defaults() {
            return new Config();
        }

        /** @return an independent copy of this config. */
        public Config copy() {
            Config c = new Config();
            c.hardwareName = this.hardwareName;
            c.pipelineIndex = this.pipelineIndex;
            c.pollRateHz = this.pollRateHz;
            c.maxResultAgeSec = this.maxResultAgeSec;
            c.cameraMount = this.cameraMount;
            return c;
        }
    }

    private static final class Prepared {
        final Config tagConfig;
        final FtcLimelightVisionLane.Config ownerConfig;

        Prepared(Config tagConfig, FtcLimelightVisionLane.Config ownerConfig) {
            this.tagConfig = tagConfig;
            this.ownerConfig = ownerConfig;
        }
    }

    private final Config cfg;
    private final AprilTagSensor tagSensor;

    /**
     * Creates and starts one Limelight owner specialized for a configured AprilTag pipeline.
     *
     * @param hardwareMap FTC hardware map used for device lookup
     * @param config lane config; defensively copied and validated before lookup
     */
    public FtcLimelightAprilTagVisionLane(HardwareMap hardwareMap, Config config) {
        this(hardwareMap, prepare(config));
    }

    private FtcLimelightAprilTagVisionLane(HardwareMap hardwareMap, Prepared prepared) {
        super(hardwareMap, prepared.ownerConfig);
        this.cfg = prepared.tagConfig;
        this.tagSensor = new LimelightAprilTagSensor();
    }

    FtcLimelightAprilTagVisionLane(Config config, DeviceFactory deviceFactory) {
        this(prepare(config), deviceFactory);
    }

    private FtcLimelightAprilTagVisionLane(Prepared prepared, DeviceFactory deviceFactory) {
        super(prepared.ownerConfig, deviceFactory);
        this.cfg = prepared.tagConfig;
        this.tagSensor = new LimelightAprilTagSensor();
    }

    /** @return independent copy of the AprilTag-specialized config. */
    public Config config() {
        return cfg.copy();
    }

    /** {@inheritDoc} */
    @Override
    public CameraMountConfig cameraMountConfig() {
        return cfg.cameraMount;
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
    public synchronized VisionReadiness readiness(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (requestedPipelineIndex() != cfg.pipelineIndex) {
            return VisionReadiness.notReady("AprilTag vision requires Limelight pipeline "
                    + cfg.pipelineIndex + "; currently requested pipeline is "
                    + requestedPipelineIndex());
        }
        return pipelineReadiness(clock);
    }

    /**
     * Returns a confirmed result only while the configured AprilTag pipeline is selected and ready.
     *
     * @param clock shared loop clock
     * @return confirmed AprilTag-pipeline result, or an unavailable snapshot
     */
    public synchronized ResultSnapshot confirmedAprilTagResult(LoopClock clock) {
        if (!readiness(clock).isReady()) {
            return confirmedUnavailableResult();
        }
        return confirmedPipelineResult(clock);
    }

    /** Adds specialization diagnostics without reading the device again. */
    @Override
    public synchronized void debugDump(DebugSink dbg, String prefix) {
        super.debugDump(dbg, prefix);
        if (dbg == null) {
            return;
        }
        String p = prefix == null || prefix.isEmpty() ? "limelightVision" : prefix;
        dbg.addData(p + ".aprilTagPipeline", cfg.pipelineIndex)
                .addData(p + ".aprilTagPipelineRequested",
                        requestedPipelineIndex() == cfg.pipelineIndex);
        cfg.cameraMount.debugDump(dbg, p + ".cameraMount");
    }

    private static Prepared prepare(Config config) {
        Config copied = Objects.requireNonNull(config, "config").copy();
        if (copied.hardwareName == null || copied.hardwareName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "FtcLimelightAprilTagVisionLane.Config.hardwareName must not be blank");
        }
        copied.hardwareName = copied.hardwareName.trim();
        if (copied.cameraMount == null) {
            throw new IllegalArgumentException(
                    "FtcLimelightAprilTagVisionLane.Config.cameraMount must not be null");
        }

        FtcLimelightVisionLane.Config owner = FtcLimelightVisionLane.Config.defaults();
        owner.hardwareName = copied.hardwareName;
        owner.pipelineIndex = copied.pipelineIndex;
        owner.pollRateHz = copied.pollRateHz;
        owner.maxResultAgeSec = copied.maxResultAgeSec;
        return new Prepared(copied, owner);
    }

    private static ResultSnapshot confirmedUnavailableResult() {
        // The generic owner is deliberately queried only after tag readiness succeeds. An
        // impossible clock cannot manufacture a result, so use a tiny unopened state by asking
        // the shared singleton through this package-private helper on the base class.
        return unavailableResult();
    }

    static Pose3d phoenixCameraToTagPose(Pose3D limelightPose) {
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
        return FtcFrames.toPhoenixFromFtcLocalizationCameraAxes(limelightCameraPose);
    }

    private final class LimelightAprilTagSensor implements AprilTagSensor {

        private long lastCycle = Long.MIN_VALUE;
        private long lastPipelineGeneration = Long.MIN_VALUE;
        private AprilTagDetections lastDetections = AprilTagDetections.none();

        @Override
        public AprilTagDetections get(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            long cycle = clock.cycle();
            long generation = pipelineGeneration();

            // A close or pipeline transition must invalidate cached detections immediately, even
            // when robot policy changes modes more than once within one OpMode cycle.
            if (!readiness(clock).isReady()) {
                lastCycle = cycle;
                lastPipelineGeneration = generation;
                lastDetections = AprilTagDetections.none(Double.POSITIVE_INFINITY);
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
            if (!result.hasResult() || !result.isTargetValid()) {
                return AprilTagDetections.none();
            }

            double ageSec = result.ageSec();
            List<LLResultTypes.FiducialResult> fiducials = result.fiducialResults();
            if (fiducials.isEmpty()) {
                return AprilTagDetections.none(ageSec);
            }

            ArrayList<AprilTagObservation> out =
                    new ArrayList<AprilTagObservation>(fiducials.size());
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                Pose3d cameraToTagPose = phoenixCameraToTagPose(
                        fiducial.getTargetPoseCameraSpace());
                if (cameraToTagPose != null) {
                    out.add(AprilTagObservation.target(
                            fiducial.getFiducialId(),
                            cameraToTagPose,
                            ageSec
                    ));
                }
            }
            return out.isEmpty()
                    ? AprilTagDetections.none(ageSec)
                    : AprilTagDetections.of(ageSec, out);
        }
    }
}
