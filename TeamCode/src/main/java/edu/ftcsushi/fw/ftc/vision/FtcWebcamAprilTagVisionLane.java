package edu.ftcsushi.fw.ftc.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

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

    /** Package-private built-in processor seam for preflight-order tests. */
    interface AprilTagProcessorFactory {
        AprilTagProcessor create(CameraMountConfig cameraMount, AprilTagLibrary tagLibrary);
    }

    private static final AprilTagProcessorFactory APRIL_TAG_PROCESSOR_FACTORY =
            new AprilTagProcessorFactory() {
                @Override
                public AprilTagProcessor create(
                        CameraMountConfig cameraMount,
                        AprilTagLibrary tagLibrary
                ) {
                    return FtcWebcamAprilTagSupport.createProcessor(cameraMount, tagLibrary);
                }
            };

    /** Mutable data-only authoring config for the AprilTag webcam owner. */
    public static final class Config {

        /** Preferred FTC hardware-map name for the AprilTag webcam. */
        public String webcamName = "Webcam 1";

        /** Camera streaming resolution used by the FTC vision portal. */
        public Size cameraResolution =
                FtcWebcamVisionPortalLane.Config.defaults().cameraResolution;

        /** Camera extrinsics expressed in the robot frame. */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        /**
         * Optional detector-library override; {@code null} selects the current FTC game.
         *
         * <p>The FTC SDK library and its metadata contain mutable values. {@link #copy()} retains
         * this borrowed authoring reference deliberately and performs no validation, so copying an
         * inactive profile branch cannot fail. A factory or lane that activates this config
         * validates and deeply snapshots the library before retaining it.</p>
         */
        public AprilTagLibrary tagLibrary = null;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /** Creates a config populated with framework defaults. */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Creates a raw, independently editable top-level authoring copy without validating it.
         *
         * <p>The optional SDK {@link #tagLibrary} remains a borrowed reference until an active
         * factory or lane captures it. All other leaves are immutable.</p>
         */
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

    /** Package-private immutable snapshot retained by a deferred factory or active lane. */
    static final class ActiveConfig {
        private final String webcamName;
        private final Size cameraResolution;
        private final CameraMountConfig cameraMount;
        private final FtcAprilTagLibrarySnapshot tagLibrary;

        private ActiveConfig(
                String webcamName,
                Size cameraResolution,
                CameraMountConfig cameraMount,
                FtcAprilTagLibrarySnapshot tagLibrary
        ) {
            this.webcamName = webcamName;
            this.cameraResolution = cameraResolution;
            this.cameraMount = cameraMount;
            this.tagLibrary = tagLibrary;
        }

        String webcamName() {
            return webcamName;
        }

        Size cameraResolution() {
            return cameraResolution;
        }

        CameraMountConfig cameraMount() {
            return cameraMount;
        }

        AprilTagLibrary freshTagLibrary() {
            return tagLibrary.freshLibrary();
        }

        String tagLibraryProvenance() {
            return tagLibrary.provenance();
        }

        ActiveConfig recaptured(ResolutionReader resolutionReader) {
            validateResolution(cameraResolution, resolutionReader);
            return new ActiveConfig(
                    requireWebcamName(webcamName),
                    cameraResolution,
                    Objects.requireNonNull(
                            cameraMount,
                            "FtcWebcamAprilTagVisionLane.Config.cameraMount"
                    ),
                    tagLibrary.recaptured()
            );
        }
    }

    private static final class Prepared {
        final HardwareMap hardwareMap;
        final ActiveConfig aprilTagConfig;
        final FtcWebcamVisionPortalLane.Config portalConfig;
        final AprilTagProcessor aprilTagProcessor;
        final VisionProcessor[] completeProcessors;

        Prepared(
                HardwareMap hardwareMap,
                ActiveConfig aprilTagConfig,
                FtcWebcamVisionPortalLane.Config portalConfig,
                AprilTagProcessor aprilTagProcessor,
                VisionProcessor[] completeProcessors
        ) {
            this.hardwareMap = hardwareMap;
            this.aprilTagConfig = aprilTagConfig;
            this.portalConfig = portalConfig;
            this.aprilTagProcessor = aprilTagProcessor;
            this.completeProcessors = completeProcessors;
        }
    }

    private static final ResolutionReader ANDROID_RESOLUTION_READER = new ResolutionReader() {
        @Override
        public int width(Size size) {
            return size.getWidth();
        }

        @Override
        public int height(Size size) {
            return size.getHeight();
        }
    };

    private final ActiveConfig cfg;
    private final AprilTagProcessor aprilTagProcessor;
    private final FtcWebcamAprilTagSupport.PortalAprilTagSensor tagSensor;

    /**
     * Opens one webcam owner containing the framework AprilTag processor and a complete additional
     * processor set.
     *
     * <p>Every additional processor must be a fresh instance that has never been attached to a
     * VisionPortal. Keep references to those typed processors in the robot-owned vision capability
     * so it can enable modes and publish their immutable results.</p>
     *
     * <p>The complete config, custom/current-game tag library, and additional-processor set are
     * validated and captured before the built-in processor is created or the webcam is looked up.
     * The processor receives its own canonical private tag-library snapshot.</p>
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
        this(prepare(hardwareMap, config, additionalProcessors));
    }

    /** Package-private owner construction from a factory's already captured snapshot. */
    FtcWebcamAprilTagVisionLane(HardwareMap hardwareMap, ActiveConfig factoryConfig) {
        this(prepare(hardwareMap, factoryConfig, new VisionProcessor[0]));
    }

    /** Package-private constructor for processor/preflight-order tests. */
    FtcWebcamAprilTagVisionLane(
            Config config,
            AprilTagProcessorFactory processorFactory,
            PortalFactory portalFactory,
            NanoClock nanoClock,
            ResolutionReader resolutionReader,
            VisionProcessor... additionalProcessors
    ) {
        this(prepareWithProcessorFactory(
                        config,
                        processorFactory,
                        resolutionReader,
                        additionalProcessors
                ),
                portalFactory,
                nanoClock,
                resolutionReader);
    }

    private FtcWebcamAprilTagVisionLane(Prepared prepared) {
        super(
                prepared.hardwareMap,
                prepared.portalConfig,
                prepared.completeProcessors
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
        this(prepareWithProcessor(
                        config,
                        aprilTagProcessor,
                        resolutionReader,
                        additionalProcessors
                ),
                portalFactory,
                nanoClock,
                resolutionReader);
    }

    private FtcWebcamAprilTagVisionLane(
            Prepared prepared,
            PortalFactory portalFactory,
            NanoClock nanoClock,
            ResolutionReader resolutionReader
    ) {
        super(
                prepared.portalConfig,
                portalFactory,
                nanoClock,
                resolutionReader,
                prepared.completeProcessors
        );
        this.cfg = prepared.aprilTagConfig;
        this.aprilTagProcessor = prepared.aprilTagProcessor;
        this.tagSensor = new FtcWebcamAprilTagSupport.PortalAprilTagSensor(
                this,
                prepared.aprilTagProcessor
        );
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
                cfg.tagLibraryProvenance()
        );
        cfg.cameraMount.debugDump(dbg, prefix + ".cameraMount");
        tagSensor.debugDump(dbg, prefix + ".aprilTag");
    }

    static ActiveConfig captureActiveConfig(Config config) {
        return captureActiveConfig(config, ANDROID_RESOLUTION_READER);
    }

    private static Prepared prepare(
            HardwareMap hardwareMap,
            Config config,
            VisionProcessor[] additionalProcessors
    ) {
        HardwareMap checkedMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        ActiveConfig checked = captureActiveConfig(config, ANDROID_RESOLUTION_READER);
        VisionProcessor[] checkedAdditional = validateAdditionalProcessors(additionalProcessors);
        AprilTagProcessor processor = APRIL_TAG_PROCESSOR_FACTORY.create(
                checked.cameraMount,
                checked.tagLibrary.freshLibrary()
        );
        return prepared(checkedMap, checked, processor, checkedAdditional);
    }

    private static Prepared prepare(
            HardwareMap hardwareMap,
            ActiveConfig factoryConfig,
            VisionProcessor[] additionalProcessors
    ) {
        HardwareMap checkedMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        ActiveConfig checked = Objects.requireNonNull(factoryConfig, "factoryConfig")
                .recaptured(ANDROID_RESOLUTION_READER);
        VisionProcessor[] checkedAdditional = validateAdditionalProcessors(additionalProcessors);
        AprilTagProcessor processor = APRIL_TAG_PROCESSOR_FACTORY.create(
                checked.cameraMount,
                checked.tagLibrary.freshLibrary()
        );
        return prepared(checkedMap, checked, processor, checkedAdditional);
    }

    private static Prepared prepareWithProcessor(
            Config config,
            AprilTagProcessor aprilTagProcessor,
            ResolutionReader resolutionReader,
            VisionProcessor[] additionalProcessors
    ) {
        ActiveConfig checked = captureActiveConfig(config, resolutionReader);
        VisionProcessor[] checkedAdditional = validateAdditionalProcessors(additionalProcessors);
        return prepared(
                null,
                checked,
                Objects.requireNonNull(aprilTagProcessor, "aprilTagProcessor"),
                checkedAdditional
        );
    }

    private static Prepared prepareWithProcessorFactory(
            Config config,
            AprilTagProcessorFactory processorFactory,
            ResolutionReader resolutionReader,
            VisionProcessor[] additionalProcessors
    ) {
        ActiveConfig checked = captureActiveConfig(config, resolutionReader);
        VisionProcessor[] checkedAdditional = validateAdditionalProcessors(additionalProcessors);
        AprilTagProcessorFactory checkedFactory = Objects.requireNonNull(
                processorFactory,
                "processorFactory"
        );
        AprilTagProcessor processor = Objects.requireNonNull(
                checkedFactory.create(checked.cameraMount, checked.tagLibrary.freshLibrary()),
                "processorFactory returned null"
        );
        return prepared(null, checked, processor, checkedAdditional);
    }

    private static Prepared prepared(
            HardwareMap hardwareMap,
            ActiveConfig checked,
            AprilTagProcessor processor,
            VisionProcessor[] additionalProcessors
    ) {
        FtcWebcamVisionPortalLane.Config portalConfig =
                FtcWebcamVisionPortalLane.Config.defaults();
        portalConfig.webcamName = checked.webcamName;
        portalConfig.cameraResolution = checked.cameraResolution;
        return new Prepared(
                hardwareMap,
                checked,
                portalConfig,
                processor,
                completeProcessorSet(processor, additionalProcessors)
        );
    }

    static ActiveConfig captureActiveConfig(
            Config config,
            ResolutionReader resolutionReader
    ) {
        Config checked = Objects.requireNonNull(
                config,
                "FtcWebcamAprilTagVisionLane.Config"
        ).copy();
        String webcamName = requireWebcamName(checked.webcamName);
        Size resolution = Objects.requireNonNull(
                checked.cameraResolution,
                "FtcWebcamAprilTagVisionLane.Config.cameraResolution"
        );
        validateResolution(resolution, resolutionReader);
        CameraMountConfig mount = Objects.requireNonNull(
                checked.cameraMount,
                "FtcWebcamAprilTagVisionLane.Config.cameraMount"
        );
        FtcAprilTagLibrarySnapshot library = FtcAprilTagLibrarySnapshot.capture(
                checked.tagLibrary,
                "FtcWebcamAprilTagVisionLane.Config.tagLibrary"
        );
        return new ActiveConfig(webcamName, resolution, mount, library);
    }

    private static VisionProcessor[] validateAdditionalProcessors(
            VisionProcessor[] additionalProcessors
    ) {
        Objects.requireNonNull(additionalProcessors, "additionalProcessors");
        VisionProcessor[] copy = additionalProcessors.clone();
        List<VisionProcessor> seen = new ArrayList<VisionProcessor>(copy.length);
        for (int index = 0; index < copy.length; index++) {
            VisionProcessor processor = Objects.requireNonNull(
                    copy[index],
                    "additionalProcessors[" + index + "]"
            );
            if (seen.contains(processor)) {
                throw new IllegalArgumentException(
                        "additionalProcessors must be distinct under equals(); duplicate at "
                                + "additionalProcessors[" + index + "]");
            }
            seen.add(processor);
        }
        return copy;
    }

    private static String requireWebcamName(String webcamName) {
        String checked = Objects.requireNonNull(
                webcamName,
                "FtcWebcamAprilTagVisionLane.Config.webcamName"
        ).trim();
        if (checked.isEmpty()) {
            throw new IllegalArgumentException(
                    "FtcWebcamAprilTagVisionLane.Config.webcamName must not be blank");
        }
        return checked;
    }

    private static void validateResolution(Size resolution, ResolutionReader resolutionReader) {
        ResolutionReader checkedReader = Objects.requireNonNull(
                resolutionReader,
                "resolutionReader"
        );
        int width = checkedReader.width(resolution);
        int height = checkedReader.height(resolution);
        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException(
                    "FtcWebcamAprilTagVisionLane.Config.cameraResolution must have positive "
                            + "width and height, got " + width + "x" + height);
        }
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
