package edu.ftcphoenix.fw.ftc.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * FTC-boundary owner for one Limelight and its selected onboard pipeline.
 *
 * <p>The Limelight runs only one onboard pipeline at a time. This owner therefore exposes that
 * real lifecycle directly: robot-owned vision code requests a pipeline, waits until a newer result
 * confirms that pipeline, and then interprets the cycle-stable {@link ResultSnapshot}. Pipeline
 * meanings remain outside the framework.</p>
 *
 * <p>Construction starts polling. There is intentionally no public start, pause, retry, or raw
 * device escape hatch. If setup must be retried, close this owner and construct a fresh one only
 * after that close returns successfully. A close failure leaves hardware ownership uncertain, so
 * stop and restart the OpMode before acquiring the Limelight again.</p>
 */
public class FtcLimelightVisionLane implements AutoCloseable {

    /** Configuration for one Limelight owner. */
    public static final class Config {

        /** FTC hardware-map name for the Limelight. */
        public String hardwareName = "limelight";

        /** Pipeline requested during construction; Limelight supports indices 0 through 9. */
        public int pipelineIndex = 0;

        /**
         * Control Hub polling rate in Hertz.
         *
         * <p>The FTC SDK's connection signal uses a short fixed observation window, so unusually
         * low rates may make readiness alternate between connected and disconnected. Keep the
         * 100 Hz default unless testing the real device demonstrates a reason to change it.</p>
         */
        public int pollRateHz = 100;

        /** Maximum age of a result that may confirm the requested pipeline. */
        public double maxResultAgeSec = 0.25;

        private Config() {
            // Defaults assigned above.
        }

        /** @return a new mutable config with framework defaults. */
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
            return c;
        }

        private Config validatedCopy(String context) {
            Config c = copy();
            String p = context != null && !context.trim().isEmpty()
                    ? context.trim()
                    : "FtcLimelightVisionLane.Config";
            c.hardwareName = requireNonBlank(c.hardwareName, p + ".hardwareName");
            requirePipelineIndex(c.pipelineIndex, p + ".pipelineIndex");
            if (c.pollRateHz < 1 || c.pollRateHz > 250) {
                throw new IllegalArgumentException(p + ".pollRateHz must be within [1, 250]");
            }
            if (!Double.isFinite(c.maxResultAgeSec) || c.maxResultAgeSec <= 0.0) {
                throw new IllegalArgumentException(p + ".maxResultAgeSec must be finite and > 0");
            }
            return c;
        }
    }

    /**
     * Controlled, cycle-stable view of one confirmed Limelight pipeline result.
     *
     * <p>The list structures cannot be mutated, but their elements are FTC SDK result values and
     * may contain vendor-owned mutable structures such as corner lists. These values belong only
     * inside the FTC-boundary realization: project the fields the robot needs immediately into a
     * robot-owned immutable snapshot. This keeps custom adapters capable without exposing the
     * mutable camera owner or pretending vendor results are framework value objects.</p>
     */
    public static final class ResultSnapshot {

        private static final ResultSnapshot NONE = new ResultSnapshot();

        private final boolean hasResult;
        private final long controlHubTimestampMillis;
        private final double ageSec;
        private final int pipelineIndex;
        private final String pipelineType;
        private final boolean targetValid;
        private final List<LLResultTypes.BarcodeResult> barcodeResults;
        private final List<LLResultTypes.ClassifierResult> classifierResults;
        private final List<LLResultTypes.DetectorResult> detectorResults;
        private final List<LLResultTypes.FiducialResult> fiducialResults;
        private final List<LLResultTypes.ColorResult> colorResults;
        private final Pose3D botpose;
        private final Pose3D botposeMt2;

        private ResultSnapshot() {
            this.hasResult = false;
            this.controlHubTimestampMillis = Long.MIN_VALUE;
            this.ageSec = Double.POSITIVE_INFINITY;
            this.pipelineIndex = -1;
            this.pipelineType = "";
            this.targetValid = false;
            this.barcodeResults = Collections.emptyList();
            this.classifierResults = Collections.emptyList();
            this.detectorResults = Collections.emptyList();
            this.fiducialResults = Collections.emptyList();
            this.colorResults = Collections.emptyList();
            this.botpose = null;
            this.botposeMt2 = null;
        }

        private ResultSnapshot(DeviceResult result) {
            this.hasResult = true;
            this.controlHubTimestampMillis = result.controlHubTimestampMillis;
            this.ageSec = result.ageSec;
            this.pipelineIndex = result.pipelineIndex;
            this.pipelineType = result.pipelineType;
            this.targetValid = result.targetValid;
            this.barcodeResults = result.barcodeResults;
            this.classifierResults = result.classifierResults;
            this.detectorResults = result.detectorResults;
            this.fiducialResults = result.fiducialResults;
            this.colorResults = result.colorResults;
            this.botpose = copyPose(result.botpose);
            this.botposeMt2 = copyPose(result.botposeMt2);
        }

        /** @return whether a confirmed result is present. */
        public boolean hasResult() {
            return hasResult;
        }

        /** @return Control Hub wall-clock timestamp supplied by the SDK, in milliseconds. */
        public long controlHubTimestampMillis() {
            return controlHubTimestampMillis;
        }

        /** @return result age in seconds when this snapshot was sampled. */
        public double ageSec() {
            return ageSec;
        }

        /** @return pipeline index reported by this result, or -1 when unavailable. */
        public int pipelineIndex() {
            return pipelineIndex;
        }

        /** @return pipeline type reported by Limelight, or an empty string when unavailable. */
        public String pipelineType() {
            return pipelineType;
        }

        /**
         * @return whether the active pipeline reported a target
         *
         * <p>This is target availability, not pipeline readiness.</p>
         */
        public boolean isTargetValid() {
            return targetValid;
        }

        /** @return unmodifiable outer list of FTC SDK barcode results. */
        public List<LLResultTypes.BarcodeResult> barcodeResults() {
            return barcodeResults;
        }

        /** @return unmodifiable outer list of FTC SDK classifier results. */
        public List<LLResultTypes.ClassifierResult> classifierResults() {
            return classifierResults;
        }

        /** @return unmodifiable outer list of FTC SDK detector results. */
        public List<LLResultTypes.DetectorResult> detectorResults() {
            return detectorResults;
        }

        /** @return unmodifiable outer list of FTC SDK fiducial results. */
        public List<LLResultTypes.FiducialResult> fiducialResults() {
            return fiducialResults;
        }

        /** @return unmodifiable outer list of FTC SDK color-pipeline results. */
        public List<LLResultTypes.ColorResult> colorResults() {
            return colorResults;
        }

        /**
         * @return standard Limelight field pose, or {@code null} when unavailable
         *
         * <p>The FTC SDK represents an absent pose array as an exact all-zero pose. This owner
         * fails closed by treating that sentinel as unavailable.</p>
         */
        public Pose3D botpose() {
            return copyPose(botpose);
        }

        /** @return MegaTag 2 field pose, or {@code null} for the SDK's absent/all-zero sentinel. */
        public Pose3D botposeMt2() {
            return copyPose(botposeMt2);
        }
    }

    /* Package-private device seam. Tests provide metadata directly and never construct LLResult. */
    interface DeviceFactory {
        Device open(String hardwareName);
    }

    interface Device {
        void setPollRateHz(int pollRateHz);

        boolean pipelineSwitch(int pipelineIndex);

        void start();

        void stop();

        boolean isRunning();

        boolean isConnected();

        DeviceResult latestResult();

        boolean updateRobotOrientationDegrees(double fieldYawDegrees);

        long nowMillis();

        void close();
    }

    static final class DeviceResult {
        final long controlHubTimestampMillis;
        final double ageSec;
        final int pipelineIndex;
        final String pipelineType;
        final boolean targetValid;
        final List<LLResultTypes.BarcodeResult> barcodeResults;
        final List<LLResultTypes.ClassifierResult> classifierResults;
        final List<LLResultTypes.DetectorResult> detectorResults;
        final List<LLResultTypes.FiducialResult> fiducialResults;
        final List<LLResultTypes.ColorResult> colorResults;
        final Pose3D botpose;
        final Pose3D botposeMt2;

        DeviceResult(long controlHubTimestampMillis,
                     double ageSec,
                     int pipelineIndex,
                     String pipelineType,
                     boolean targetValid,
                     List<LLResultTypes.BarcodeResult> barcodeResults,
                     List<LLResultTypes.ClassifierResult> classifierResults,
                     List<LLResultTypes.DetectorResult> detectorResults,
                     List<LLResultTypes.FiducialResult> fiducialResults,
                     List<LLResultTypes.ColorResult> colorResults,
                     Pose3D botpose,
                     Pose3D botposeMt2) {
            this.controlHubTimestampMillis = controlHubTimestampMillis;
            this.ageSec = ageSec;
            this.pipelineIndex = pipelineIndex;
            this.pipelineType = pipelineType != null ? pipelineType : "";
            this.targetValid = targetValid;
            this.barcodeResults = immutableCopy(barcodeResults);
            this.classifierResults = immutableCopy(classifierResults);
            this.detectorResults = immutableCopy(detectorResults);
            this.fiducialResults = immutableCopy(fiducialResults);
            this.colorResults = immutableCopy(colorResults);
            this.botpose = copyPose(botpose);
            this.botposeMt2 = copyPose(botposeMt2);
        }

        static DeviceResult metadata(long controlHubTimestampMillis,
                                     double ageSec,
                                     int pipelineIndex,
                                     boolean targetValid) {
            return new DeviceResult(
                    controlHubTimestampMillis,
                    ageSec,
                    pipelineIndex,
                    "",
                    targetValid,
                    null,
                    null,
                    null,
                    null,
                    null,
                    null,
                    null
            );
        }
    }

    private static final class DeviceSample {
        final boolean running;
        final boolean connected;
        final DeviceResult result;

        DeviceSample(boolean running, boolean connected, DeviceResult result) {
            this.running = running;
            this.connected = connected;
            this.result = result;
        }
    }

    private final Config cfg;
    private final Device device;

    private int requestedPipelineIndex;
    private long pipelineGeneration = 1L;
    private boolean pipelineRequestAccepted;
    private long requestCompletionBaselineMillis;
    private String terminalFailureReason = "";
    private boolean closeAttempted;
    private boolean closeSucceeded;
    private String closeFailureReason = "";

    private long sampledCycle = Long.MIN_VALUE;
    private long sampledGeneration = Long.MIN_VALUE;
    private DeviceSample lastSample;
    private VisionReadiness lastReadiness = VisionReadiness.notReady("Limelight has not produced a confirmed pipeline result yet");

    /**
     * Opens and starts one configured Limelight.
     *
     * @param hardwareMap FTC hardware map used for the device lookup
     * @param config owner config; defensively copied and validated before lookup
     */
    public FtcLimelightVisionLane(final HardwareMap hardwareMap, Config config) {
        this(config, new DeviceFactory() {
            @Override
            public Device open(String hardwareName) {
                HardwareMap requiredMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
                return new SdkDevice(requiredMap.get(Limelight3A.class, hardwareName));
            }
        });
    }

    FtcLimelightVisionLane(Config config, DeviceFactory deviceFactory) {
        Config base = Objects.requireNonNull(config, "config")
                .validatedCopy("FtcLimelightVisionLane.Config");
        DeviceFactory requiredFactory = Objects.requireNonNull(deviceFactory, "deviceFactory");

        Device opened = requiredFactory.open(base.hardwareName);
        if (opened == null) {
            throw new IllegalStateException("Limelight device factory returned null for '"
                    + base.hardwareName + "'");
        }

        boolean accepted;
        long baseline;
        try {
            opened.setPollRateHz(base.pollRateHz);
            accepted = opened.pipelineSwitch(base.pipelineIndex);
            baseline = opened.nowMillis();
            opened.start();
        } catch (RuntimeException e) {
            throw CleanupActions.attemptAllAfterFailure(e, opened::stop, opened::close);
        }

        this.cfg = base;
        this.device = opened;
        this.requestedPipelineIndex = base.pipelineIndex;
        this.pipelineRequestAccepted = accepted;
        this.requestCompletionBaselineMillis = baseline;
        if (!accepted) {
            this.lastReadiness = rejectedReadiness(base.pipelineIndex);
        }
    }

    /** @return an independent snapshot of the generic owner config. */
    public final Config visionConfig() {
        return cfg.copy();
    }

    /** @return configured FTC hardware-map name. */
    public final String hardwareName() {
        return cfg.hardwareName;
    }

    /** @return currently requested pipeline index. */
    public final synchronized int requestedPipelineIndex() {
        return requestedPipelineIndex;
    }

    /** @return generation of the current pipeline request; construction begins at generation 1. */
    public final synchronized long pipelineGeneration() {
        return pipelineGeneration;
    }

    /** @return whether Limelight accepted the current pipeline request. */
    public final synchronized boolean wasPipelineRequestAccepted() {
        return pipelineRequestAccepted;
    }

    /**
     * Requests a different Limelight pipeline.
     *
     * <p>Requesting the already-requested index is always a no-op, including after rejection. This
     * prevents an every-loop setter from repeatedly issuing synchronous HTTP requests. When the
     * same setup request must be retried, close this owner and reconstruct it only after that close
     * returns successfully; restart the OpMode instead if close fails.</p>
     *
     * @param pipelineIndex requested Limelight pipeline in [0, 9]
     * @return whether Limelight accepted the new request
     */
    public final synchronized boolean requestPipeline(int pipelineIndex) {
        requirePipelineIndex(pipelineIndex, "pipelineIndex");
        requireUsable();
        if (pipelineIndex == requestedPipelineIndex) {
            return pipelineRequestAccepted;
        }

        requestedPipelineIndex = pipelineIndex;
        pipelineGeneration++;
        pipelineRequestAccepted = false;
        invalidateSample();

        boolean accepted;
        try {
            accepted = device.pipelineSwitch(pipelineIndex);
            requestCompletionBaselineMillis = device.nowMillis();
        } catch (RuntimeException transitionFailure) {
            terminalFailureReason = "Limelight pipeline " + pipelineIndex
                    + " transition failed (" + describe(transitionFailure)
                    + "); close this owner before replacement";
            lastReadiness = VisionReadiness.notReady(terminalFailureReason);
            throw transitionFailure;
        }
        pipelineRequestAccepted = accepted;
        lastReadiness = accepted
                ? VisionReadiness.notReady("Waiting for Limelight pipeline " + pipelineIndex
                + " to produce a post-request result")
                : rejectedReadiness(pipelineIndex);
        return accepted;
    }

    /**
     * Returns whether the requested pipeline is operational this loop.
     *
     * <p>Readiness requires a running, connected device and a fresh result whose timestamp is
     * strictly newer than completion of the request and whose pipeline index matches. Target
     * validity is deliberately not part of readiness.</p>
     */
    public final synchronized VisionReadiness pipelineReadiness(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (closeAttempted) {
            if (closeSucceeded) {
                lastReadiness = VisionReadiness.notReady(
                        "Limelight vision owner is closed; construct a fresh owner to use it again");
            } else if (!closeFailureReason.isEmpty()) {
                lastReadiness = VisionReadiness.notReady(
                        "Limelight close failed; do not open a replacement in this OpMode; "
                                + "stop and restart it: " + closeFailureReason);
            } else {
                lastReadiness = VisionReadiness.notReady(
                        "Limelight close is in progress; wait for confirmed cleanup before replacement");
            }
            return lastReadiness;
        }
        if (!terminalFailureReason.isEmpty()) {
            lastReadiness = VisionReadiness.notReady(terminalFailureReason);
            return lastReadiness;
        }
        if (!pipelineRequestAccepted) {
            lastReadiness = rejectedReadiness(requestedPipelineIndex);
            return lastReadiness;
        }

        DeviceSample sample = sample(clock);
        lastReadiness = evaluatePipelineReadiness(sample);
        return lastReadiness;
    }

    /**
     * Returns the current cycle's confirmed pipeline result, or an unavailable value while the
     * requested pipeline is not ready.
     */
    public final synchronized ResultSnapshot confirmedPipelineResult(LoopClock clock) {
        if (!pipelineReadiness(clock).isReady()) {
            return ResultSnapshot.NONE;
        }
        DeviceSample sample = sample(clock);
        return new ResultSnapshot(sample.result);
    }

    static ResultSnapshot unavailableResult() {
        return ResultSnapshot.NONE;
    }

    /**
     * Sends Phoenix field-frame yaw to Limelight for algorithms such as MegaTag 2.
     *
     * @param fieldYawRad Phoenix field-frame yaw, counter-clockwise positive, in radians
     * @return whether Limelight accepted the orientation update
     */
    public final synchronized boolean updateRobotFieldYawRad(double fieldYawRad) {
        requireUsable();
        if (!Double.isFinite(fieldYawRad)) {
            throw new IllegalArgumentException("fieldYawRad must be finite");
        }
        return device.updateRobotOrientationDegrees(Math.toDegrees(fieldYawRad));
    }

    /** @return whether a close was requested, regardless of the device cleanup result. */
    public final synchronized boolean isCloseAttempted() {
        return closeAttempted;
    }

    /** @return whether both Limelight cleanup calls completed successfully. */
    public final synchronized boolean isClosed() {
        return closeSucceeded;
    }

    /** @return an empty string before/since successful close, otherwise the first cleanup failure. */
    public final synchronized String closeFailureReason() {
        return closeFailureReason;
    }

    /**
     * Stops polling and closes the device exactly once. Repeated calls are no-ops. The first
     * lifecycle failure is rethrown after both cleanup steps have been attempted; later failures
     * are suppressed on it. A failed attempt remains distinguishable from a successful close, and
     * requires an OpMode restart before another owner acquires this Limelight.
     */
    @Override
    public final synchronized void close() {
        if (closeAttempted) {
            return;
        }
        closeAttempted = true;
        lastReadiness = VisionReadiness.notReady(
                "Limelight close is in progress; wait for confirmed cleanup before replacement");

        try {
            CleanupActions.attemptAll(device::stop, device::close);
            closeSucceeded = true;
            closeFailureReason = "";
            lastReadiness = VisionReadiness.notReady(
                    "Limelight vision owner is closed; construct a fresh owner to use it again");
        } catch (RuntimeException cleanupFailure) {
            closeFailureReason = describe(cleanupFailure);
            lastReadiness = VisionReadiness.notReady(
                    "Limelight close failed; do not open a replacement in this OpMode; "
                            + "stop and restart it: " + closeFailureReason);
            throw cleanupFailure;
        }
    }

    /** Emits cached diagnostics without causing another device or result read. */
    public synchronized void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = prefix == null || prefix.isEmpty() ? "limelightVision" : prefix;
        dbg.addData(p + ".hardwareName", cfg.hardwareName)
                .addData(p + ".pollRateHz", cfg.pollRateHz)
                .addData(p + ".maxResultAgeSec", cfg.maxResultAgeSec)
                .addData(p + ".pipeline.requested", requestedPipelineIndex)
                .addData(p + ".pipeline.generation", pipelineGeneration)
                .addData(p + ".pipeline.requestAccepted", pipelineRequestAccepted)
                .addData(p + ".pipeline.requestBaselineMillis", requestCompletionBaselineMillis)
                .addData(p + ".terminalFailure", terminalFailureReason)
                .addData(p + ".readiness.ready", lastReadiness.isReady())
                .addData(p + ".readiness.reason", lastReadiness.reason())
                .addData(p + ".closeAttempted", closeAttempted)
                .addData(p + ".closeSucceeded", closeSucceeded)
                .addData(p + ".closeFailure", closeFailureReason);

        DeviceSample sample = lastSample;
        if (sample != null) {
            dbg.addData(p + ".sample.running", sample.running)
                    .addData(p + ".sample.connected", sample.connected)
                    .addData(p + ".sample.hasResult", sample.result != null);
            if (sample.result != null) {
                dbg.addData(p + ".sample.pipeline", sample.result.pipelineIndex)
                        .addData(p + ".sample.pipelineType", sample.result.pipelineType)
                        .addData(p + ".sample.ageSec", sample.result.ageSec)
                        .addData(p + ".sample.targetValid", sample.result.targetValid)
                        .addData(p + ".sample.fiducialCount", sample.result.fiducialResults.size())
                        .addData(p + ".sample.detectorCount", sample.result.detectorResults.size())
                        .addData(p + ".sample.classifierCount", sample.result.classifierResults.size());
            }
        }
    }

    private DeviceSample sample(LoopClock clock) {
        long cycle = clock.cycle();
        if (lastSample != null && sampledCycle == cycle && sampledGeneration == pipelineGeneration) {
            return lastSample;
        }
        boolean running = device.isRunning();
        boolean connected = device.isConnected();
        // The FTC SDK may synthesize an apparently fresh pipeline-zero result before a real device
        // result exists. Do not even admit that value into the sample while polling is unavailable.
        DeviceResult result = running && connected ? device.latestResult() : null;
        lastSample = new DeviceSample(running, connected, result);
        sampledCycle = cycle;
        sampledGeneration = pipelineGeneration;
        return lastSample;
    }

    private VisionReadiness evaluatePipelineReadiness(DeviceSample sample) {
        if (!sample.running) {
            return VisionReadiness.notReady(
                    "Limelight polling is not running; close this owner and replace it only after close succeeds");
        }
        if (!sample.connected) {
            return VisionReadiness.notReady("Limelight is disconnected; check USB, power, and hardware configuration");
        }
        if (sample.result == null) {
            return VisionReadiness.notReady("Limelight has not produced a result yet");
        }
        if (sample.result.controlHubTimestampMillis <= requestCompletionBaselineMillis) {
            return VisionReadiness.notReady("Waiting for a result newer than pipeline request generation "
                    + pipelineGeneration);
        }
        if (sample.result.pipelineIndex != requestedPipelineIndex) {
            return VisionReadiness.notReady("Waiting for Limelight pipeline " + requestedPipelineIndex
                    + "; latest result reports pipeline " + sample.result.pipelineIndex);
        }
        if (!Double.isFinite(sample.result.ageSec) || sample.result.ageSec < 0.0) {
            return VisionReadiness.notReady("Limelight result reported an invalid age");
        }
        if (sample.result.ageSec > cfg.maxResultAgeSec) {
            return VisionReadiness.notReady("Limelight result is stale (age " + sample.result.ageSec
                    + " sec; max " + cfg.maxResultAgeSec + " sec)");
        }
        return VisionReadiness.ready();
    }

    private void requireUsable() {
        if (closeAttempted) {
            if (closeSucceeded) {
                throw new IllegalStateException(
                        "Limelight vision owner is closed; construct a fresh owner to use it again");
            }
            if (!closeFailureReason.isEmpty()) {
                throw new IllegalStateException(
                        "Limelight close failed; do not open a replacement in this OpMode; "
                                + "stop and restart it");
            }
            throw new IllegalStateException(
                    "Limelight close is in progress; wait for confirmed cleanup before replacement");
        }
        if (!terminalFailureReason.isEmpty()) {
            throw new IllegalStateException(terminalFailureReason);
        }
    }

    private void invalidateSample() {
        sampledCycle = Long.MIN_VALUE;
        sampledGeneration = Long.MIN_VALUE;
        lastSample = null;
    }

    private static VisionReadiness rejectedReadiness(int pipelineIndex) {
        return VisionReadiness.notReady("Limelight rejected pipeline " + pipelineIndex
                + "; this owner must be closed before replacement");
    }

    private static String requireNonBlank(String value, String name) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(name + " must not be blank");
        }
        return value.trim();
    }

    private static String describe(RuntimeException ex) {
        String message = ex.getMessage();
        return (message == null || message.trim().isEmpty())
                ? ex.getClass().getSimpleName()
                : message.trim();
    }

    private static void requirePipelineIndex(int pipelineIndex, String name) {
        if (pipelineIndex < 0 || pipelineIndex > 9) {
            throw new IllegalArgumentException(name + " must be within [0, 9]");
        }
    }

    private static <T> List<T> immutableCopy(List<T> source) {
        if (source == null || source.isEmpty()) {
            return Collections.emptyList();
        }
        ArrayList<T> copy = new ArrayList<T>(source.size());
        for (T item : source) {
            copy.add(Objects.requireNonNull(item, "Limelight result lists must not contain null"));
        }
        return Collections.unmodifiableList(copy);
    }

    private static Pose3D copyPose(Pose3D source) {
        if (!isUsableSdkPose(source)) {
            return null;
        }
        Position position = source.getPosition();
        YawPitchRollAngles orientation = source.getOrientation();
        return new Pose3D(
                new Position(
                        position.unit,
                        position.x,
                        position.y,
                        position.z,
                        position.acquisitionTime
                ),
                new YawPitchRollAngles(
                        AngleUnit.RADIANS,
                        orientation.getYaw(AngleUnit.RADIANS),
                        orientation.getPitch(AngleUnit.RADIANS),
                        orientation.getRoll(AngleUnit.RADIANS),
                        orientation.getAcquisitionTime()
                )
        );
    }

    /** Returns false for malformed poses and the FTC SDK's exact all-zero absent-pose sentinel. */
    static boolean isUsableSdkPose(Pose3D source) {
        if (source == null) {
            return false;
        }
        Position position = source.getPosition();
        YawPitchRollAngles orientation = source.getOrientation();
        if (position == null || orientation == null) {
            return false;
        }
        double yawRad = orientation.getYaw(AngleUnit.RADIANS);
        double pitchRad = orientation.getPitch(AngleUnit.RADIANS);
        double rollRad = orientation.getRoll(AngleUnit.RADIANS);
        if (!Double.isFinite(position.x)
                || !Double.isFinite(position.y)
                || !Double.isFinite(position.z)
                || !Double.isFinite(yawRad)
                || !Double.isFinite(pitchRad)
                || !Double.isFinite(rollRad)) {
            return false;
        }
        return position.x != 0.0
                || position.y != 0.0
                || position.z != 0.0
                || yawRad != 0.0
                || pitchRad != 0.0
                || rollRad != 0.0;
    }

    private static final class SdkDevice implements Device {
        private static final double MILLIS_PER_SECOND = 1000.0;

        private final Limelight3A limelight;

        SdkDevice(Limelight3A limelight) {
            this.limelight = Objects.requireNonNull(limelight, "limelight");
        }

        @Override
        public void setPollRateHz(int pollRateHz) {
            limelight.setPollRateHz(pollRateHz);
        }

        @Override
        public boolean pipelineSwitch(int pipelineIndex) {
            return limelight.pipelineSwitch(pipelineIndex);
        }

        @Override
        public void start() {
            limelight.start();
        }

        @Override
        public void stop() {
            limelight.stop();
        }

        @Override
        public boolean isRunning() {
            return limelight.isRunning();
        }

        @Override
        public boolean isConnected() {
            return limelight.isConnected();
        }

        @Override
        public DeviceResult latestResult() {
            LLResult result = limelight.getLatestResult();
            if (result == null) {
                return null;
            }
            return new DeviceResult(
                    result.getControlHubTimeStamp(),
                    result.getStaleness() / MILLIS_PER_SECOND,
                    result.getPipelineIndex(),
                    result.getPipelineType(),
                    result.isValid(),
                    result.getBarcodeResults(),
                    result.getClassifierResults(),
                    result.getDetectorResults(),
                    result.getFiducialResults(),
                    result.getColorResults(),
                    result.getBotpose(),
                    result.getBotpose_MT2()
            );
        }

        @Override
        public boolean updateRobotOrientationDegrees(double fieldYawDegrees) {
            return limelight.updateRobotOrientation(fieldYawDegrees);
        }

        @Override
        public long nowMillis() {
            return System.currentTimeMillis();
        }

        @Override
        public void close() {
            limelight.close();
        }
    }
}
