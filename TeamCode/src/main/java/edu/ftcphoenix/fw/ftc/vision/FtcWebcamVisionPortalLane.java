package edu.ftcphoenix.fw.ftc.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;

/**
 * FTC-boundary owner for one webcam and one complete, fixed set of VisionPortal processors.
 *
 * <p>The processor set is supplied at construction because the FTC SDK attaches processors only
 * while a {@link VisionPortal} is built. Robot code may enable and disable those processors later,
 * but cannot borrow the portal or add another processor after construction. A retry therefore
 * means closing this owner successfully and then constructing a new owner with fresh processor
 * instances. If close fails, hardware ownership is uncertain: do not open a replacement in the
 * same OpMode; stop and restart the OpMode first.</p>
 *
 * <p>This class owns webcam plumbing and lifecycle only. A robot-owned vision capability should
 * assign season meaning to the processors and expose typed immutable results to strategy code.</p>
 */
public class FtcWebcamVisionPortalLane implements AutoCloseable {

    private static final Size DEFAULT_RESOLUTION = new Size(640, 480);
    private static final String DEFAULT_CAMERA_MONITOR_VIEW_ID_NAME = "cameraMonitorViewId";

    /** Stable configuration for one webcam VisionPortal owner. */
    public static final class Config {

        /** FTC hardware-map name of the webcam. */
        public String webcamName = "Webcam 1";

        /** Requested camera stream resolution. */
        public Size cameraResolution = DEFAULT_RESOLUTION;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /** Returns a new config populated with Phoenix defaults. */
        public static Config defaults() {
            return new Config();
        }

        /** Returns an independently editable config copy. */
        public Config copy() {
            Config copy = new Config();
            copy.webcamName = webcamName;
            copy.cameraResolution = cameraResolution;
            return copy;
        }

        @Override
        public String toString() {
            return "FtcWebcamVisionPortalLane.Config{"
                    + "webcamName='" + webcamName + '\''
                    + ", cameraResolution=" + cameraResolution
                    + '}';
        }
    }

    /** Package-private device seam used by focused lifecycle tests. */
    interface PortalDevice {
        VisionPortal.CameraState cameraState();

        void setProcessorEnabled(VisionProcessor processor, boolean enabled);

        boolean isProcessorEnabled(VisionProcessor processor);

        void stopStreaming();

        void resumeStreaming();

        float framesPerSecond();

        <T extends CameraControl> T cameraControl(Class<T> controlType);

        void close();
    }

    /** Package-private creation seam; production construction still owns one real portal. */
    interface PortalFactory {
        PortalDevice open(Config config, VisionProcessor[] processors);
    }

    /** Package-private monotonic-time seam for exact processor-generation tests. */
    interface NanoClock {
        long nowNanos();
    }

    /** Package-private Android-value seam so config checks remain pure-JVM testable. */
    interface ResolutionReader {
        int width(Size size);

        int height(Size size);
    }

    private static final NanoClock SYSTEM_NANO_CLOCK = new NanoClock() {
        @Override
        public long nowNanos() {
            return System.nanoTime();
        }
    };

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

    private static final class ProcessorState {
        long dataGeneration;
        long acceptFramesAfterNanos;

        ProcessorState(long acceptFramesAfterNanos) {
            this.acceptFramesAfterNanos = acceptFramesAfterNanos;
        }
    }

    private final Config cfg;
    private final PortalDevice portal;
    private final NanoClock nanoClock;
    private final VisionProcessor[] processors;
    private final IdentityHashMap<VisionProcessor, ProcessorState> processorStates =
            new IdentityHashMap<VisionProcessor, ProcessorState>();

    private boolean streamingRequested = true;
    private boolean closeAttempted;
    private boolean closeSucceeded;
    private String closeFailureReason = "";

    /**
     * Opens one webcam portal with its complete processor set.
     *
     * @param hardwareMap FTC hardware map containing the configured webcam
     * @param config stable webcam settings; defensively copied
     * @param processors complete set of fresh processors to attach to this portal
     */
    public FtcWebcamVisionPortalLane(
            HardwareMap hardwareMap,
            Config config,
            VisionProcessor... processors
    ) {
        this(config, new SdkPortalFactory(hardwareMap), SYSTEM_NANO_CLOCK,
                ANDROID_RESOLUTION_READER, processors);
    }

    /** Package-private constructor for deterministic ownership/lifecycle tests. */
    FtcWebcamVisionPortalLane(
            Config config,
            PortalFactory portalFactory,
            VisionProcessor... processors
    ) {
        this(config, portalFactory, SYSTEM_NANO_CLOCK, ANDROID_RESOLUTION_READER, processors);
    }

    /** Package-private constructor with deterministic monotonic time for freshness tests. */
    FtcWebcamVisionPortalLane(
            Config config,
            PortalFactory portalFactory,
            NanoClock nanoClock,
            VisionProcessor... processors
    ) {
        this(config, portalFactory, nanoClock, ANDROID_RESOLUTION_READER, processors);
    }

    /** Package-private constructor with deterministic Android-value and time seams. */
    FtcWebcamVisionPortalLane(
            Config config,
            PortalFactory portalFactory,
            NanoClock nanoClock,
            ResolutionReader resolutionReader,
            VisionProcessor... processors
    ) {
        this.cfg = validateAndCopy(config, resolutionReader);
        this.processors = validateAndCopyProcessors(processors);
        PortalFactory checkedFactory = Objects.requireNonNull(portalFactory, "portalFactory");
        this.nanoClock = Objects.requireNonNull(nanoClock, "nanoClock");

        // Every caller-owned value is validated before the factory is allowed to resolve hardware.
        PortalDevice opened = Objects.requireNonNull(
                checkedFactory.open(this.cfg.copy(), this.processors.clone()),
                "portalFactory returned null"
        );

        try {
            // Verify that every declared processor really belongs to the completed portal before
            // publishing this owner. This catches a malformed factory or an SDK attachment failure
            // while this constructor still owns the only cleanup path.
            for (VisionProcessor processor : this.processors) {
                if (!opened.isProcessorEnabled(processor)) {
                    throw new IllegalStateException(
                            "VisionPortal processor was not enabled after construction: "
                                    + processor.getClass().getSimpleName());
                }
            }
        } catch (RuntimeException failure) {
            throw CleanupActions.attemptAllAfterFailure(failure, opened::close);
        }
        this.portal = opened;

        // A processor can retain its previous result until a newer frame arrives. Only typed
        // processor adapters can inspect result timestamps, so record the freshness boundary here.
        long constructionCompletedNanos = this.nanoClock.nowNanos();
        for (VisionProcessor processor : this.processors) {
            processorStates.put(processor, new ProcessorState(constructionCompletedNanos));
        }
    }

    /** Returns a defensive copy of the portal configuration. */
    public final Config portalConfig() {
        return cfg.copy();
    }

    /** Returns the normalized FTC hardware-map name of the owned webcam. */
    public final String webcamName() {
        return cfg.webcamName;
    }

    /** Returns whether this owner was constructed with the processor instance. */
    public final boolean ownsProcessor(VisionProcessor processor) {
        return processor != null && processorStates.containsKey(processor);
    }

    /**
     * Returns webcam-stream readiness, independent of any particular processor or visible target.
     */
    public final VisionReadiness readiness() {
        if (closeAttempted) {
            return VisionReadiness.notReady(closeFailureReason.isEmpty()
                    ? "webcam vision owner is closed; construct a fresh owner to use it again"
                    : "webcam close failed; do not open a replacement in this OpMode; stop and restart it: "
                    + closeFailureReason);
        }
        if (!streamingRequested) {
            return VisionReadiness.notReady(
                    "webcam streaming is stopped; call resumeStreaming() before using vision");
        }

        VisionPortal.CameraState state;
        try {
            state = portal.cameraState();
        } catch (RuntimeException ex) {
            return VisionReadiness.notReady("could not read webcam state: " + describe(ex));
        }

        if (state == VisionPortal.CameraState.STREAMING) {
            return VisionReadiness.ready();
        }
        if (state == VisionPortal.CameraState.ERROR) {
            return VisionReadiness.notReady(
                    "webcam entered ERROR; check its hardware configuration and USB connection");
        }
        if (state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            return VisionReadiness.notReady(
                    "webcam device is closed; close this owner successfully before constructing a fresh one");
        }
        return VisionReadiness.notReady("webcam is not streaming yet (state=" + state + ")");
    }

    /**
     * Returns readiness for one registered processor.
     *
     * <p>This is processor execution readiness, not target visibility or proof that the processor
     * has published a post-enable result. Typed processor adapters own that result-level check.</p>
     *
     * @param processor registered processor whose readiness is required
     */
    public final VisionReadiness processorReadiness(VisionProcessor processor) {
        requireOwnedProcessor(processor);
        VisionReadiness stream = readiness();
        if (!stream.isReady()) {
            return stream;
        }
        try {
            if (!portal.isProcessorEnabled(processor)) {
                return VisionReadiness.notReady(
                        "required webcam processor is disabled; enable it before using its results");
            }
        } catch (RuntimeException ex) {
            return VisionReadiness.notReady("could not read webcam processor state: " + describe(ex));
        }
        return VisionReadiness.ready();
    }

    /** Enables or disables one processor that was registered at construction. */
    public final void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        ProcessorState processorState = requireOwnedProcessor(processor);
        requireUsable();
        boolean wasEnabled = portal.isProcessorEnabled(processor);
        if (wasEnabled == enabled) {
            return;
        }
        portal.setProcessorEnabled(processor, enabled);
        invalidateProcessorData(processorState, enabled ? nanoClock.nowNanos() : Long.MAX_VALUE);
    }

    /** Returns whether one registered processor is currently enabled. */
    public final boolean isProcessorEnabled(VisionProcessor processor) {
        requireOwnedProcessor(processor);
        requireUsable();
        return portal.isProcessorEnabled(processor);
    }

    /** Requests an asynchronous stream stop once; repeated requests are no-ops. */
    public final void stopStreaming() {
        requireUsable();
        if (!streamingRequested) {
            return;
        }
        portal.stopStreaming();
        streamingRequested = false;
        invalidateAllProcessorData(Long.MAX_VALUE);
    }

    /** Requests an asynchronous stream resume once; repeated requests are no-ops. */
    public final void resumeStreaming() {
        requireUsable();
        if (streamingRequested) {
            return;
        }
        portal.resumeStreaming();
        streamingRequested = true;
        invalidateAllProcessorData(nanoClock.nowNanos());
    }

    /** Returns the current FTC camera state while this owner remains usable. */
    public final VisionPortal.CameraState cameraState() {
        requireUsable();
        return portal.cameraState();
    }

    /** Returns the current portal frame rate. */
    public final float framesPerSecond() {
        requireUsable();
        return portal.framesPerSecond();
    }

    /**
     * Returns one supported webcam control without exposing the mutable VisionPortal itself.
     */
    public final <T extends CameraControl> T cameraControl(Class<T> controlType) {
        requireUsable();
        return portal.cameraControl(Objects.requireNonNull(controlType, "controlType"));
    }

    /**
     * Permanently releases the portal. This is idempotent. A close failure is recorded for
     * diagnostics and rethrown on the first call; later calls remain no-ops. Only a successful
     * close permits construction of a replacement owner in the same OpMode. After a close failure,
     * stop and restart the OpMode before acquiring the webcam again.
     */
    @Override
    public final void close() {
        if (closeAttempted) {
            return;
        }
        closeAttempted = true;
        streamingRequested = false;
        invalidateAllProcessorData(Long.MAX_VALUE);
        try {
            portal.close();
            closeSucceeded = true;
        } catch (RuntimeException ex) {
            closeFailureReason = describe(ex);
            throw ex;
        }
    }

    /** Returns whether a close was requested, regardless of the SDK close result. */
    public final boolean isCloseAttempted() {
        return closeAttempted;
    }

    /** Returns whether the portal close call completed successfully. */
    public final boolean isClosed() {
        return closeSucceeded;
    }

    /** Returns an empty string after a successful close, otherwise the recorded failure reason. */
    public final String closeFailureReason() {
        return closeFailureReason;
    }

    /** Emits live ownership, stream, processor, and close diagnostics. */
    public final void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "webcamVision" : prefix;
        dbg.addData(p + ".backend", "webcam");
        dbg.addData(p + ".webcamName", cfg.webcamName);
        dbg.addData(p + ".cameraResolution", cfg.cameraResolution.toString());
        dbg.addData(p + ".streamingRequested", streamingRequested);
        dbg.addData(p + ".cameraState", safeCameraState());
        dbg.addData(p + ".fps", safeFramesPerSecond());
        dbg.addData(p + ".processorCount", processors.length);
        for (int i = 0; i < processors.length; i++) {
            VisionProcessor processor = processors[i];
            String pp = p + ".processors[" + i + "]";
            dbg.addData(pp + ".type", processor.getClass().getSimpleName());
            dbg.addData(pp + ".enabled", safeProcessorEnabled(processor));
            dbg.addData(pp + ".dataGeneration", processorStates.get(processor).dataGeneration);
        }
        dbg.addData(p + ".closeAttempted", closeAttempted);
        dbg.addData(p + ".closeSucceeded", closeSucceeded);
        dbg.addData(p + ".closeFailure", closeFailureReason);
        appendDebug(dbg, p);
    }

    /** Hook for a typed specialization to append diagnostics under the same owner prefix. */
    protected void appendDebug(DebugSink dbg, String prefix) {
        // Base owner has no processor-specific result state.
    }

    final long processorDataGeneration(VisionProcessor processor) {
        return requireOwnedProcessor(processor).dataGeneration;
    }

    final long acceptProcessorFramesAfterNanos(VisionProcessor processor) {
        return requireOwnedProcessor(processor).acceptFramesAfterNanos;
    }

    final long monotonicNowNanos() {
        return nanoClock.nowNanos();
    }

    private ProcessorState requireOwnedProcessor(VisionProcessor processor) {
        Objects.requireNonNull(processor, "processor");
        ProcessorState state = processorStates.get(processor);
        if (state == null) {
            throw new IllegalArgumentException(
                    "Processor is not registered with this webcam owner; include it when constructing the owner");
        }
        return state;
    }

    private void requireUsable() {
        if (closeAttempted) {
            if (closeSucceeded) {
                throw new IllegalStateException(
                        "Webcam vision owner is closed; construct a fresh owner to use it again");
            }
            if (!closeFailureReason.isEmpty()) {
                throw new IllegalStateException(
                        "Webcam close failed; do not open a replacement in this OpMode; stop and restart it");
            }
            throw new IllegalStateException(
                    "Webcam close is in progress; wait for confirmed cleanup before replacing the owner");
        }
    }

    private void invalidateAllProcessorData(long acceptFramesAfterNanos) {
        for (ProcessorState state : processorStates.values()) {
            invalidateProcessorData(state, acceptFramesAfterNanos);
        }
    }

    private static void invalidateProcessorData(ProcessorState state, long acceptFramesAfterNanos) {
        state.dataGeneration++;
        state.acceptFramesAfterNanos = acceptFramesAfterNanos;
    }

    private String safeCameraState() {
        if (closeAttempted) {
            return closeSucceeded
                    ? VisionPortal.CameraState.CAMERA_DEVICE_CLOSED.name()
                    : "UNUSABLE_AFTER_CLOSE_FAILURE";
        }
        try {
            VisionPortal.CameraState state = portal.cameraState();
            return state != null ? state.name() : "null";
        } catch (RuntimeException ex) {
            return "ERROR(" + describe(ex) + ")";
        }
    }

    private String safeFramesPerSecond() {
        if (closeAttempted) {
            return "unavailable";
        }
        try {
            return Float.toString(portal.framesPerSecond());
        } catch (RuntimeException ex) {
            return "ERROR(" + describe(ex) + ")";
        }
    }

    private String safeProcessorEnabled(VisionProcessor processor) {
        if (closeAttempted) {
            return "unavailable";
        }
        try {
            return Boolean.toString(portal.isProcessorEnabled(processor));
        } catch (RuntimeException ex) {
            return "ERROR(" + describe(ex) + ")";
        }
    }

    private static Config validateAndCopy(Config config, ResolutionReader resolutionReader) {
        Config copy = Objects.requireNonNull(config, "config").copy();
        copy.webcamName = requireWebcamName(copy.webcamName);
        Objects.requireNonNull(copy.cameraResolution, "cameraResolution");
        ResolutionReader checkedReader = Objects.requireNonNull(
                resolutionReader, "resolutionReader");
        int width = checkedReader.width(copy.cameraResolution);
        int height = checkedReader.height(copy.cameraResolution);
        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException(
                    "cameraResolution must have positive width and height, got "
                            + width + "x" + height);
        }
        return copy;
    }

    private static VisionProcessor[] validateAndCopyProcessors(VisionProcessor[] processors) {
        Objects.requireNonNull(processors, "processors");
        if (processors.length == 0) {
            throw new IllegalArgumentException(
                    "At least one VisionProcessor is required when constructing webcam vision");
        }
        VisionProcessor[] copy = processors.clone();
        // VisionPortal.Builder uses List.contains(), so distinct instances that compare equal are
        // duplicates too. Match that rule before the factory can attach any processor globally.
        List<VisionProcessor> seen = new ArrayList<VisionProcessor>(copy.length);
        for (int i = 0; i < copy.length; i++) {
            VisionProcessor processor = Objects.requireNonNull(
                    copy[i], "processors[" + i + "]");
            if (seen.contains(processor)) {
                throw new IllegalArgumentException(
                        "VisionProcessors must be distinct under equals(); duplicate at processors["
                                + i + "]");
            }
            seen.add(processor);
        }
        return copy;
    }

    private static String requireWebcamName(String webcamName) {
        String checked = Objects.requireNonNull(webcamName, "webcamName").trim();
        if (checked.isEmpty()) {
            throw new IllegalArgumentException("webcamName must not be blank");
        }
        return checked;
    }

    private static String describe(RuntimeException ex) {
        String message = ex.getMessage();
        return (message == null || message.trim().isEmpty())
                ? ex.getClass().getSimpleName()
                : message.trim();
    }

    private static final class SdkPortalFactory implements PortalFactory {
        private final HardwareMap hardwareMap;

        SdkPortalFactory(HardwareMap hardwareMap) {
            this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        }

        @Override
        public PortalDevice open(Config config, VisionProcessor[] processors) {
            // Config and the complete processor set have already been checked before this lookup.
            WebcamName webcam = hardwareMap.get(WebcamName.class, config.webcamName);
            VisionPortal.Builder builder = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setCameraResolution(config.cameraResolution);
            for (VisionProcessor processor : processors) {
                builder.addProcessor(processor);
            }
            applyDefaultLiveViewContainerId(builder, hardwareMap);
            return new SdkPortalDevice(builder.build());
        }
    }

    private static final class SdkPortalDevice implements PortalDevice {
        private final VisionPortal portal;

        SdkPortalDevice(VisionPortal portal) {
            this.portal = Objects.requireNonNull(portal, "portal");
        }

        @Override
        public VisionPortal.CameraState cameraState() {
            return portal.getCameraState();
        }

        @Override
        public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
            portal.setProcessorEnabled(processor, enabled);
        }

        @Override
        public boolean isProcessorEnabled(VisionProcessor processor) {
            return portal.getProcessorEnabled(processor);
        }

        @Override
        public void stopStreaming() {
            portal.stopStreaming();
        }

        @Override
        public void resumeStreaming() {
            portal.resumeStreaming();
        }

        @Override
        public float framesPerSecond() {
            return portal.getFps();
        }

        @Override
        public <T extends CameraControl> T cameraControl(Class<T> controlType) {
            return portal.getCameraControl(controlType);
        }

        @Override
        public void close() {
            portal.close();
        }
    }

    private static void applyDefaultLiveViewContainerId(
            VisionPortal.Builder builder,
            HardwareMap hardwareMap
    ) {
        int containerId = resolveDefaultLiveViewContainerId(hardwareMap);
        if (containerId != 0) {
            builder.setLiveViewContainerId(containerId);
        }
    }

    private static int resolveDefaultLiveViewContainerId(HardwareMap hardwareMap) {
        if (hardwareMap.appContext == null) {
            return 0;
        }
        try {
            int id = hardwareMap.appContext.getResources().getIdentifier(
                    DEFAULT_CAMERA_MONITOR_VIEW_ID_NAME,
                    "id",
                    hardwareMap.appContext.getPackageName()
            );
            if (id != 0) {
                return id;
            }
            String[] packages = new String[]{
                    "com.qualcomm.ftcrobotcontroller",
                    "org.firstinspires.ftc.robotcontroller"
            };
            for (String packageName : packages) {
                id = hardwareMap.appContext.getResources().getIdentifier(
                        DEFAULT_CAMERA_MONITOR_VIEW_ID_NAME,
                        "id",
                        packageName
                );
                if (id != 0) {
                    return id;
                }
            }
        } catch (RuntimeException ignored) {
            // Live view is optional; camera processing still works without a resolved container.
        }
        return 0;
    }
}
