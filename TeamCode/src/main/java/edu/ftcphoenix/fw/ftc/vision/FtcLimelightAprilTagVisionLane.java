package edu.ftcphoenix.fw.ftc.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
 * FTC-boundary lane owner for a Limelight-backed AprilTag vision rig.
 *
 * <p>This lane owns the Limelight device lifecycle, startup pipeline selection, poll-rate setup,
 * and adaptation from Limelight fiducial results into the framework's shared {@link AprilTagSensor}
 * seam. It deliberately stops there. Field-pose solving, tag selection, targeting, and odometry
 * fusion remain separate concerns that consume the shared {@link AprilTagVisionLane} output just as
 * they do for webcam-backed lanes.</p>
 *
 * <p>In other words, this lane answers: "how do we own and read AprilTag detections from a
 * Limelight?" It does not answer: "how should the robot use those detections?"</p>
 */
public final class FtcLimelightAprilTagVisionLane implements AprilTagVisionLane {

    /**
     * Configuration for a Limelight-backed FTC AprilTag vision lane.
     *
     * <p>The config groups the stable hardware-rig concerns that belong at the FTC boundary:
     * Limelight identity, startup AprilTag pipeline selection, poll rate, and the camera mount
     * expressed in robot coordinates. It intentionally does not try to own higher-level localization
     * policy such as trust in direct botpose estimates or field-map upload. Those remain separate
     * choices above this lane.</p>
     */
    public static final class Config {

        /**
         * Preferred FTC hardware-map name for the Limelight device.
         */
        public String hardwareName = "limelight";

        /**
         * Pipeline index to switch to when the lane starts.
         *
         * <p>This should normally point at an AprilTag pipeline configured in the Limelight web UI.
         * The pipeline switch is fire-and-forget, so callers should still expect a short settling
         * period before meaningful detections appear.</p>
         */
        public int pipelineIndex = 0;

        /**
         * Requested Control Hub polling rate in Hertz.
         *
         * <p>Limelight clamps this internally to its supported range. The framework default keeps
         * the result feed responsive without hard-coding any robot-specific assumptions.</p>
         */
        public int pollRateHz = 100;

        /**
         * Camera extrinsics expressed in the robot frame.
         */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Creates a config populated with framework defaults.
         *
         * @return new mutable config instance
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Creates a deep copy of this config.
         *
         * @return copied config whose fields can be edited independently
         */
        public Config copy() {
            Config c = new Config();
            c.hardwareName = this.hardwareName;
            c.pipelineIndex = this.pipelineIndex;
            c.pollRateHz = this.pollRateHz;
            c.cameraMount = this.cameraMount;
            return c;
        }
    }

    private static final double MILLIS_PER_SECOND = 1000.0;

    private final Config cfg;
    private final Limelight3A limelight;
    private final AprilTagSensor tagSensor;
    private boolean closed = false;

    /**
     * Creates the Limelight AprilTag vision lane from one FTC hardware map and one config snapshot.
     *
     * @param hardwareMap FTC hardware map used to acquire the Limelight device
     * @param config      lane config; defensively copied for the lifetime of this owner
     */
    public FtcLimelightAprilTagVisionLane(HardwareMap hardwareMap, Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.limelight = hardwareMap.get(Limelight3A.class, this.cfg.hardwareName);
        this.limelight.setPollRateHz(this.cfg.pollRateHz);
        this.limelight.pipelineSwitch(this.cfg.pipelineIndex);
        this.limelight.start();
        this.tagSensor = new LimelightAprilTagSensor(this.limelight);
    }

    /**
     * Returns a defensive copy of the lane config owned by this vision rig.
     *
     * @return copied config snapshot describing device identity and startup settings
     */
    public Config config() {
        return cfg.copy();
    }

    /**
     * Returns the configured FTC hardware-map name for the Limelight.
     *
     * @return configured Limelight hardware name
     */
    public String hardwareName() {
        return cfg.hardwareName;
    }

    /**
     * Returns the raw FTC Limelight device owned by this lane.
     *
     * <p>This is intentionally a concrete-lane convenience, not part of the shared
     * {@link AprilTagVisionLane} seam. Most callers should consume {@link #tagSensor()} instead.
     * Direct Limelight access is mainly useful for backend-specific diagnostics or future features
     * such as direct botpose estimation.</p>
     *
     * @return owned FTC Limelight device
     */
    public Limelight3A limelight() {
        return limelight;
    }

    /**
     * Returns the fixed camera-mount configuration used by this vision rig.
     *
     * @return camera extrinsics for the active robot profile
     */
    @Override
    public CameraMountConfig cameraMountConfig() {
        return cfg.cameraMount;
    }

    /**
     * Returns the shared AprilTag sensor resource owned by this lane.
     *
     * @return shared AprilTag sensor
     */
    @Override
    public AprilTagSensor tagSensor() {
        return tagSensor;
    }

    /**
     * Releases the shared vision resources owned by this lane.
     *
     * <p>This method is idempotent so it is safe to call from repeated shutdown paths.</p>
     */
    @Override
    public void close() {
        if (closed) {
            return;
        }
        closed = true;
        tagSensor.close();
    }

    /**
     * Returns whether this vision lane has already released its Limelight resource.
     *
     * @return {@code true} after {@link #close()} has been called
     */
    public boolean isClosed() {
        return closed;
    }

    /**
     * Dumps the lane's live debug state.
     *
     * @param dbg    debug sink to write to; ignored when {@code null}
     * @param prefix key prefix for all entries; may be {@code null} or empty
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }

        String p = (prefix == null || prefix.isEmpty()) ? "visionLane" : prefix;
        dbg.addData(p + ".backend", "limelight");
        dbg.addData(p + ".hardwareName", cfg.hardwareName);
        dbg.addData(p + ".pipelineIndex.requested", cfg.pipelineIndex);
        dbg.addData(p + ".pollRateHz", cfg.pollRateHz);
        dbg.addData(p + ".connected", limelight.isConnected());
        dbg.addData(p + ".running", limelight.isRunning());
        dbg.addData(p + ".timeSinceLastUpdateMs", limelight.getTimeSinceLastUpdate());
        dbg.addData(p + ".closed", closed);

        LLResult latest = limelight.getLatestResult();
        if (latest != null) {
            dbg.addData(p + ".latest.valid", latest.isValid());
            dbg.addData(p + ".latest.pipelineIndex", latest.getPipelineIndex());
            dbg.addData(p + ".latest.pipelineType", latest.getPipelineType());
            dbg.addData(p + ".latest.stalenessMs", latest.getStaleness());
            List<LLResultTypes.FiducialResult> fiducials = latest.getFiducialResults();
            dbg.addData(p + ".latest.fiducialCount", fiducials != null ? fiducials.size() : 0);
        }

        cfg.cameraMount.debugDump(dbg, p + ".cameraMount");
    }

    private static Pose3d phoenixCameraToTagPose(Pose3D limelightPose) {
        if (limelightPose == null) {
            return null;
        }

        Position position = limelightPose.getPosition();
        if (position == null) {
            return null;
        }
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

    private static double ageSecFromResult(LLResult result) {
        if (result == null) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(0.0, result.getStaleness()) / MILLIS_PER_SECOND;
    }

    /**
     * Internal implementation of {@link AprilTagSensor} backed by one FTC {@link Limelight3A}.
     *
     * <p>The sensor is memoized by {@link LoopClock#cycle()} so all consumers in one Phoenix loop
     * observe the same fiducial snapshot.</p>
     */
    static final class LimelightAprilTagSensor implements AprilTagSensor {

        private final Limelight3A limelight;
        private boolean closed = false;

        private long lastCycle = Long.MIN_VALUE;
        private AprilTagDetections lastDetections = AprilTagDetections.none();

        LimelightAprilTagSensor(Limelight3A limelight) {
            this.limelight = Objects.requireNonNull(limelight, "limelight");
        }

        /**
         * Releases the underlying Limelight device.
         */
        @Override
        public void close() {
            if (closed) {
                return;
            }
            closed = true;
            try {
                limelight.stop();
            } catch (Exception ignored) {
                // Non-throwing by contract.
            }
            try {
                limelight.close();
            } catch (Exception ignored) {
                // Non-throwing by contract.
            }
        }

        /**
         * Returns the raw detections snapshot for the current Phoenix loop.
         *
         * <p>This method is idempotent by {@link LoopClock#cycle()} so guidance, localization,
         * telemetry, and tag selection can all share the same sensor instance without accidentally
         * reading different Limelight frames.</p>
         */
        @Override
        public AprilTagDetections get(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            long cyc = clock.cycle();
            if (cyc == lastCycle) {
                return lastDetections;
            }
            lastCycle = cyc;
            lastDetections = readDetections();
            return lastDetections;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void reset() {
            lastCycle = Long.MIN_VALUE;
            lastDetections = AprilTagDetections.none();
        }

        private AprilTagDetections readDetections() {
            LLResult result = limelight.getLatestResult();
            double ageSec = ageSecFromResult(result);
            if (result == null) {
                return AprilTagDetections.none(ageSec);
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                return AprilTagDetections.none(ageSec);
            }

            ArrayList<AprilTagObservation> out = new ArrayList<AprilTagObservation>(fiducials.size());
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial == null) {
                    continue;
                }

                Pose3D targetPoseCameraSpace = fiducial.getTargetPoseCameraSpace();
                Pose3d cameraToTagPose = phoenixCameraToTagPose(targetPoseCameraSpace);
                if (cameraToTagPose == null) {
                    continue;
                }

                out.add(AprilTagObservation.target(fiducial.getFiducialId(), cameraToTagPose, ageSec));
            }

            if (out.isEmpty()) {
                return AprilTagDetections.none(ageSec);
            }
            return AprilTagDetections.of(ageSec, out);
        }
    }
}
