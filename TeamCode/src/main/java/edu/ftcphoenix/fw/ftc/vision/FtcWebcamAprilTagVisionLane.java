package edu.ftcphoenix.fw.ftc.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Webcam portal owner specialized with one FTC AprilTag processor.
 *
 * <p>The inherited {@link FtcWebcamVisionPortalLane} owns the one physical webcam, portal,
 * complete processor set, and lifecycle. This specialization adds the backend-neutral
 * {@link AprilTagVisionLane} view without creating a second owner or borrowing a raw portal.</p>
 */
public final class FtcWebcamAprilTagVisionLane
        extends FtcWebcamVisionPortalLane
        implements AprilTagVisionLane {

    /** Stable AprilTag and camera-rig configuration. */
    public static final class Config {

        /** Preferred FTC hardware-map name for the AprilTag webcam. */
        public String webcamName = "Webcam 1";

        /** Camera streaming resolution used by the FTC vision portal. */
        public Size cameraResolution =
                FtcWebcamVisionPortalLane.Config.defaults().cameraResolution;

        /** Camera extrinsics expressed in the robot frame. */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        /** Optional detector-library override; {@code null} selects the current FTC game. */
        public AprilTagLibrary tagLibrary = null;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /** Creates a config populated with framework defaults. */
        public static Config defaults() {
            return new Config();
        }

        /** Creates an independently editable config copy. */
        public Config copy() {
            Config copy = new Config();
            copy.webcamName = webcamName;
            copy.cameraResolution = cameraResolution;
            copy.cameraMount = cameraMount;
            copy.tagLibrary = tagLibrary;
            return copy;
        }

        @Override
        public String toString() {
            return "FtcWebcamAprilTagVisionLane.Config{"
                    + "webcamName='" + webcamName + '\''
                    + ", cameraResolution=" + cameraResolution
                    + ", cameraMount=" + cameraMount
                    + ", tagLibrary=" + (tagLibrary == null ? "currentGame" : "custom")
                    + '}';
        }
    }

    private static final class Prepared {
        final Config aprilTagConfig;
        final FtcWebcamVisionPortalLane.Config portalConfig;
        final AprilTagProcessor aprilTagProcessor;

        Prepared(
                Config aprilTagConfig,
                FtcWebcamVisionPortalLane.Config portalConfig,
                AprilTagProcessor aprilTagProcessor
        ) {
            this.aprilTagConfig = aprilTagConfig;
            this.portalConfig = portalConfig;
            this.aprilTagProcessor = aprilTagProcessor;
        }
    }

    private final Config cfg;
    private final AprilTagProcessor aprilTagProcessor;
    private final FtcWebcamAprilTagSupport.PortalAprilTagSensor tagSensor;

    /** Opens a webcam owner containing only its fresh framework AprilTag processor. */
    public FtcWebcamAprilTagVisionLane(HardwareMap hardwareMap, Config config) {
        this(hardwareMap, prepare(config), new VisionProcessor[0]);
    }

    /**
     * Opens one webcam owner containing the framework AprilTag processor and a complete additional
     * processor set.
     *
     * <p>Every additional processor must be a fresh instance that has never been attached to a
     * VisionPortal. Keep references to those typed processors in the robot-owned vision capability
     * so it can enable modes and publish their immutable results.</p>
     *
     * @param hardwareMap FTC hardware map containing the configured webcam
     * @param config AprilTag and camera-rig configuration
     * @param additionalProcessors all other processors that this portal will ever use
     */
    public FtcWebcamAprilTagVisionLane(
            HardwareMap hardwareMap,
            Config config,
            VisionProcessor... additionalProcessors
    ) {
        this(hardwareMap, prepare(config), additionalProcessors);
    }

    private FtcWebcamAprilTagVisionLane(
            HardwareMap hardwareMap,
            Prepared prepared,
            VisionProcessor[] additionalProcessors
    ) {
        super(
                hardwareMap,
                prepared.portalConfig,
                completeProcessorSet(prepared.aprilTagProcessor, additionalProcessors)
        );
        this.cfg = prepared.aprilTagConfig;
        this.aprilTagProcessor = prepared.aprilTagProcessor;
        this.tagSensor = new FtcWebcamAprilTagSupport.PortalAprilTagSensor(
                this,
                prepared.aprilTagProcessor
        );
    }

    /** Package-private constructor for focused portal ownership/lifecycle tests. */
    FtcWebcamAprilTagVisionLane(
            Config config,
            AprilTagProcessor aprilTagProcessor,
            PortalFactory portalFactory,
            NanoClock nanoClock,
            ResolutionReader resolutionReader,
            VisionProcessor... additionalProcessors
    ) {
        this(prepareWithProcessor(config, aprilTagProcessor), portalFactory, nanoClock,
                resolutionReader, additionalProcessors);
    }

    private FtcWebcamAprilTagVisionLane(
            Prepared prepared,
            PortalFactory portalFactory,
            NanoClock nanoClock,
            ResolutionReader resolutionReader,
            VisionProcessor[] additionalProcessors
    ) {
        super(
                prepared.portalConfig,
                portalFactory,
                nanoClock,
                resolutionReader,
                completeProcessorSet(prepared.aprilTagProcessor, additionalProcessors)
        );
        this.cfg = prepared.aprilTagConfig;
        this.aprilTagProcessor = prepared.aprilTagProcessor;
        this.tagSensor = new FtcWebcamAprilTagSupport.PortalAprilTagSensor(
                this,
                prepared.aprilTagProcessor
        );
    }

    /** Returns a defensive copy of the AprilTag camera-rig configuration. */
    public Config config() {
        return cfg.copy();
    }

    @Override
    public CameraMountConfig cameraMountConfig() {
        return cfg.cameraMount;
    }

    @Override
    public AprilTagSensor tagSensor() {
        return tagSensor;
    }

    /**
     * Enables or disables this owner's built-in AprilTag processor.
     *
     * <p>This specialized operation lets a robot-owned semantic vision capability switch between
     * AprilTag and other registered processor modes without exposing the SDK processor instance.
     * Re-enabling also advances the inherited processor-data generation, so retained pre-enable
     * detections are rejected until a newer frame arrives.</p>
     *
     * @param enabled whether the built-in AprilTag processor should run
     */
    public void setAprilTagProcessorEnabled(boolean enabled) {
        setProcessorEnabled(aprilTagProcessor, enabled);
    }

    /**
     * Returns whether this owner's built-in AprilTag processor is currently enabled.
     *
     * @return current processor enablement while this owner remains usable
     */
    public boolean isAprilTagProcessorEnabled() {
        return isProcessorEnabled(aprilTagProcessor);
    }

    /**
     * Returns AprilTag processor readiness, independent of whether any tag is visible.
     */
    @Override
    public VisionReadiness readiness(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        return processorReadiness(aprilTagProcessor);
    }

    @Override
    protected void appendDebug(DebugSink dbg, String prefix) {
        dbg.addData(
                prefix + ".aprilTag.tagLibrary",
                cfg.tagLibrary == null ? "currentGame" : cfg.tagLibrary.getClass().getSimpleName()
        );
        cfg.cameraMount.debugDump(dbg, prefix + ".cameraMount");
        tagSensor.debugDump(dbg, prefix + ".aprilTag");
    }

    private static Prepared prepare(Config config) {
        Config checked = validateAndCopy(config);
        AprilTagProcessor processor = FtcWebcamAprilTagSupport.createProcessor(
                checked.cameraMount,
                checked.tagLibrary
        );
        return prepared(checked, processor);
    }

    private static Prepared prepareWithProcessor(
            Config config,
            AprilTagProcessor aprilTagProcessor
    ) {
        return prepared(
                validateAndCopy(config),
                Objects.requireNonNull(aprilTagProcessor, "aprilTagProcessor")
        );
    }

    private static Prepared prepared(Config checked, AprilTagProcessor processor) {
        FtcWebcamVisionPortalLane.Config portalConfig =
                FtcWebcamVisionPortalLane.Config.defaults();
        portalConfig.webcamName = checked.webcamName;
        portalConfig.cameraResolution = checked.cameraResolution;
        return new Prepared(checked, portalConfig, processor);
    }

    private static Config validateAndCopy(Config config) {
        Config checked = Objects.requireNonNull(config, "config").copy();
        checked.webcamName = Objects.requireNonNull(checked.webcamName, "webcamName").trim();
        if (checked.webcamName.isEmpty()) {
            throw new IllegalArgumentException("webcamName must not be blank");
        }
        Objects.requireNonNull(checked.cameraResolution, "cameraResolution");
        Objects.requireNonNull(checked.cameraMount, "cameraMount");
        return checked;
    }

    private static VisionProcessor[] completeProcessorSet(
            AprilTagProcessor aprilTagProcessor,
            VisionProcessor[] additionalProcessors
    ) {
        Objects.requireNonNull(additionalProcessors, "additionalProcessors");
        VisionProcessor[] complete = new VisionProcessor[additionalProcessors.length + 1];
        complete[0] = Objects.requireNonNull(aprilTagProcessor, "aprilTagProcessor");
        System.arraycopy(
                additionalProcessors,
                0,
                complete,
                1,
                additionalProcessors.length
        );
        return complete;
    }
}
