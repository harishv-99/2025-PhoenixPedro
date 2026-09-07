package edu.ftcsushi.fw.ftc.vision;

import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Vec3;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.FloorTargetModel;
import edu.ftcsushi.fw.sensing.vision.FloorTargetProjection;

/**
 * Configuration vocabulary for the floor-object capability of an existing FTC camera owner.
 *
 * <p>Set the owner's {@code Config.floorObjects} to a configuration, then read that owner's
 * {@code floorObjects()} source. This class is not a second camera factory or lifecycle owner.
 * Both backends produce robot-relative positions at capture time, not current-robot or field
 * positions. The camera mount is declared once on the camera owner. Field conversion uses the
 * separate timestamp-aware observation/history path.</p>
 *
 * <p>Webcam positions model the calibrated box-center ray; Limelight positions model its configured
 * per-candidate targeting point using crosshair-independent angles. The selected reference point's
 * physical height must agree with {@link Config#targetModel}. Neither point is automatically a
 * ball center, and a color detection does not prove object identity, clearance, or capture.</p>
 */
public final class FtcFloorObjectVision {

    /**
     * Mutable data-only settings, defensively copied by the physical camera owner. The complete
     * draft is validated when enabled, including backend-specific fields unused by the selected
     * camera. This one coherent configuration is not a bag of silently accepted invalid values.
     */
    public static final class Config {
        /**
         * Reference-point height above the robot's horizontal Z=0 floor. The software baseline
         * models a point on the floor, not a ball center; configure the adopted target explicitly.
         */
        public FloorTargetModel targetModel = FloorTargetModel.atHeightInches(0.0);

        /** Maximum accepted exposure age in seconds; stale data is unavailable, not refreshed. */
        public double maxFrameAgeSec = 0.25;

        /**
         * Maximum matching candidates per frame, within [1, 64]. An excess rejects the whole frame
         * instead of silently trimming an ambiguous scene to an apparently unique target.
         */
        public int maxCandidates = 16;

        /** Limelight-only color pipeline index, within [0, 9]; reads never request a switch. */
        public int limelightPipelineIndex = 1;

        /** Webcam-only minimum Y channel, [0, 255]; defaults match the SDK yellow color range. */
        public int minY = 32;
        /** Webcam-only maximum Y channel, [0, 255]. */
        public int maxY = 255;
        /** Webcam-only minimum Cr channel, [0, 255]. */
        public int minCr = 128;
        /** Webcam-only maximum Cr channel, [0, 255]. */
        public int maxCr = 170;
        /** Webcam-only minimum Cb channel, [0, 255]. */
        public int minCb = 0;
        /** Webcam-only maximum Cb channel, [0, 255]. */
        public int maxCb = 120;
        /** Webcam-only minimum contour area in square pixels; default zero adds no area filter. */
        public double minContourAreaPixels = 0.0;
        /** Webcam-only odd blur kernel width in pixels, or zero to disable; default disabled. */
        public int blurSizePixels = 0;

        private Config() {
            // Software defaults above are not measured physical facts.
        }

        /** Returns fresh software defaults, not camera calibration or a robot safety review. */
        public static Config defaults() {
            return new Config();
        }

        /** Returns an independent authoring copy; immutable target-model data may be shared. */
        public Config copy() {
            Config copy = new Config();
            copy.targetModel = targetModel;
            copy.maxFrameAgeSec = maxFrameAgeSec;
            copy.maxCandidates = maxCandidates;
            copy.limelightPipelineIndex = limelightPipelineIndex;
            copy.minY = minY;
            copy.maxY = maxY;
            copy.minCr = minCr;
            copy.maxCr = maxCr;
            copy.minCb = minCb;
            copy.maxCb = maxCb;
            copy.minContourAreaPixels = minContourAreaPixels;
            copy.blurSizePixels = blurSizePixels;
            return copy;
        }

        Config validatedCopy(String context) {
            Config copy = copy();
            String prefix = context == null || context.trim().isEmpty()
                    ? "FtcFloorObjectVision.Config" : context.trim();
            if (!prefix.endsWith(".floorObjects")) prefix += ".floorObjects";
            Objects.requireNonNull(copy.targetModel, prefix + ".targetModel");
            if (!Double.isFinite(copy.maxFrameAgeSec) || copy.maxFrameAgeSec <= 0.0) {
                throw new IllegalArgumentException(prefix + ".maxFrameAgeSec must be finite and > 0");
            }
            if (copy.maxCandidates < 1 || copy.maxCandidates > LimelightColorFrame.MAX_CANDIDATES) {
                throw new IllegalArgumentException(prefix + ".maxCandidates must be within [1, 64]");
            }
            if (copy.limelightPipelineIndex < 0 || copy.limelightPipelineIndex > 9) {
                throw new IllegalArgumentException(prefix + ".limelightPipelineIndex must be within [0, 9]");
            }
            validateChannel(copy.minY, copy.maxY, prefix + ".Y");
            validateChannel(copy.minCr, copy.maxCr, prefix + ".Cr");
            validateChannel(copy.minCb, copy.maxCb, prefix + ".Cb");
            if (!Double.isFinite(copy.minContourAreaPixels) || copy.minContourAreaPixels < 0.0) {
                throw new IllegalArgumentException(prefix + ".minContourAreaPixels must be finite and >= 0");
            }
            if (copy.blurSizePixels < 0 || copy.blurSizePixels > 99
                    || (copy.blurSizePixels != 0 && copy.blurSizePixels % 2 == 0)) {
                throw new IllegalArgumentException(prefix + ".blurSizePixels must be zero or an odd integer within [1, 99]");
            }
            return copy;
        }

        private static void validateChannel(int minimum, int maximum, String context) {
            if (minimum < 0 || maximum > 255 || minimum > maximum) {
                throw new IllegalArgumentException(context + " threshold must satisfy 0 <= min <= max <= 255");
            }
        }

        @Override
        public String toString() {
            return "FtcFloorObjectVision.Config{targetModel=" + targetModel
                    + ", maxFrameAgeSec=" + maxFrameAgeSec + ", maxCandidates=" + maxCandidates
                    + ", limelightPipelineIndex=" + limelightPipelineIndex + '}';
        }
    }

    private FtcFloorObjectVision() {
        // The existing physical owners provide the public capability construction path.
    }

    /** Prepares one fresh, native-allocation-free processor for the owning portal's fixed set. */
    static PreparedWebcam prepareWebcam(Config config, CameraMountConfig cameraMount) {
        return new PreparedWebcam(config, cameraMount);
    }

    /** A private owner-construction bundle, not a second public processor or camera factory. */
    static final class PreparedWebcam {
        private final Config config;
        private final CameraMountConfig cameraMount;
        private final FtcColorBlobProcessor processor;
        private FtcWebcamVisionLane boundOwner;
        private Source<TargetObservations2d> source;

        private PreparedWebcam(Config config, CameraMountConfig cameraMount) {
            this.config = Objects.requireNonNull(config, "config")
                    .validatedCopy("FtcFloorObjectVision");
            this.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            this.processor = new FtcColorBlobProcessor(this.config);
        }

        VisionProcessor processor() {
            return processor;
        }

        void terminalize() {
            processor.terminalize();
        }

        /** Binds exactly once to the physical owner that registered this private processor. */
        Source<TargetObservations2d> bind(FtcWebcamVisionLane owner) {
            Objects.requireNonNull(owner, "owner");
            if (!owner.ownsProcessor(processor)) {
                throw new IllegalArgumentException("webcam floor-object processor must belong to this camera owner");
            }
            if (boundOwner != null && boundOwner != owner) {
                throw new IllegalArgumentException("webcam floor-object processor cannot be shared by camera owners");
            }
            if (source == null) {
                boundOwner = owner;
                source = new WebcamSource(owner, processor, config, cameraMount);
            }
            return source;
        }
    }

    /** Borrows the owner's confirmed immutable parse; it never polls or switches the device. */
    static Source<TargetObservations2d> bindLimelight(FtcLimelightVisionLane owner,
                                                    Config config, CameraMountConfig cameraMount) {
        return new LimelightSource(Objects.requireNonNull(owner, "owner"),
                Objects.requireNonNull(config, "config").validatedCopy("FtcFloorObjectVision"),
                Objects.requireNonNull(cameraMount, "cameraMount"));
    }

    /** Pure location interpretation shared by the two acquisition edges. */
    private static TargetObservations2d project(List<Vec3> rays, LoopTimestamp timestamp,
                                               Config config, CameraMountConfig mount) {
        if (rays.size() > config.maxCandidates) {
            return TargetObservations2d.unavailable("floor-object candidate count exceeds configured maxCandidates");
        }
        List<TargetObservation2d> located = new ArrayList<>(rays.size());
        for (Vec3 ray : rays) {
            FloorTargetProjection.Result result = FloorTargetProjection.projectRay(
                    ray, mount, config.targetModel, timestamp);
            if (!result.isAvailable()) {
                return TargetObservations2d.unavailable("floor-object location unavailable: " + result.reason());
            }
            located.add(result.observation());
        }
        return TargetObservations2d.fromFrame(timestamp, located);
    }

    /** Per-start camera generation and immutable SDK capture identity for timestamp anchoring. */
    private static final class WebcamIdentity {
        final long generation;
        final long captureNanos;

        WebcamIdentity(long generation, long captureNanos) {
            this.generation = generation;
            this.captureNanos = captureNanos;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof WebcamIdentity)) {
                return false;
            }
            WebcamIdentity that = (WebcamIdentity) other;
            return generation == that.generation && captureNanos == that.captureNanos;
        }

        @Override
        public int hashCode() {
            return Long.valueOf(generation).hashCode() * 31 + Long.valueOf(captureNanos).hashCode();
        }
    }

    /** Camera-owner-borrowed source; reset clears its value cache, not the camera or capture age. */
    private static final class WebcamSource implements Source<TargetObservations2d> {
        private final FtcWebcamVisionLane owner;
        private final FtcColorBlobProcessor processor;
        private final Config config;
        private final CameraMountConfig mount;
        private final FtcFrameTimestampAnchor timestampAnchor = new FtcFrameTimestampAnchor();
        private LoopClock ownerClock;
        private long cachedCycle = Long.MIN_VALUE;
        private long cachedGeneration = Long.MIN_VALUE;
        private long greatestCaptureNanos = Long.MIN_VALUE;
        private TargetObservations2d cached = TargetObservations2d.unavailable("webcam color has not been sampled");

        WebcamSource(FtcWebcamVisionLane owner, FtcColorBlobProcessor processor,
                     Config config, CameraMountConfig mount) {
            this.owner = owner;
            this.processor = processor;
            this.config = config;
            this.mount = mount;
        }

        @Override
        public TargetObservations2d get(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            if (ownerClock != null && ownerClock != clock) {
                throw new IllegalArgumentException("webcam floorObjects requires one stable LoopClock instance");
            }
            ownerClock = clock;
            VisionReadiness readiness = owner.processorReadiness(processor);
            if (!readiness.isReady()) {
                return TargetObservations2d.unavailable(readiness.reason());
            }
            if (processor.isTerminal()) {
                return TargetObservations2d.unavailable(processor.latest().reason);
            }
            long generation = owner.processorDataGeneration(processor);
            if (clock.cycle() == cachedCycle && generation == cachedGeneration) {
                return cached;
            }
            FtcColorBlobProcessor.Frame frame = processor.latest();
            TargetObservations2d next;
            long capture = frame.captureTimeNanos;
            long now = owner.monotonicNowNanos();
            if (capture <= 0L || capture > now || capture < greatestCaptureNanos
                    || capture <= owner.acceptProcessorFramesAfterNanos(processor)) {
                next = TargetObservations2d.unavailable("webcam floorObjects is waiting for a valid post-enable frame");
            } else {
                LoopTimestamp timestamp = timestampAnchor.anchor(clock,
                        new WebcamIdentity(generation, capture), (now - capture) * 1e-9);
                if (!timestamp.isFresh(clock, config.maxFrameAgeSec)) {
                    next = TargetObservations2d.unavailable("webcam floor-object frame is stale or invalid after clock reset");
                } else if (!frame.isAvailable()) {
                    next = TargetObservations2d.unavailable(frame.reason);
                } else {
                    next = project(frame.rays, timestamp, config, mount);
                }
                greatestCaptureNanos = capture;
            }
            cached = next;
            cachedCycle = clock.cycle();
            cachedGeneration = generation;
            return next;
        }

        @Override
        public void reset() {
            cachedCycle = Long.MIN_VALUE;
            cachedGeneration = Long.MIN_VALUE;
        }
    }

    /** Confirmed-snapshot consumer with no raw LLResult access and no competing device heartbeat. */
    private static final class LimelightSource implements Source<TargetObservations2d> {
        private final FtcLimelightVisionLane owner;
        private final Config config;
        private final CameraMountConfig mount;
        private LoopClock ownerClock;
        private long cachedCycle = Long.MIN_VALUE;
        private LoopTimestamp cachedTimestamp = LoopTimestamp.unavailable();
        private TargetObservations2d cached = TargetObservations2d.unavailable("Limelight color has not been sampled");

        LimelightSource(FtcLimelightVisionLane owner, Config config, CameraMountConfig mount) {
            this.owner = owner;
            this.config = config;
            this.mount = mount;
        }

        @Override
        public TargetObservations2d get(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            if (ownerClock != null && ownerClock != clock) {
                throw new IllegalArgumentException("Limelight floorObjects requires one stable LoopClock instance");
            }
            ownerClock = clock;
            FtcLimelightVisionLane.ResultSnapshot snapshot = owner.confirmedPipelineResult(clock);
            if (!snapshot.hasResult() || snapshot.pipelineIndex() != config.limelightPipelineIndex) {
                return TargetObservations2d.unavailable("Limelight floorObjects is waiting for the configured color pipeline");
            }
            LoopTimestamp timestamp = snapshot.frameTimestamp();
            if (!timestamp.isFresh(clock, config.maxFrameAgeSec)) {
                return TargetObservations2d.unavailable("Limelight floor-object frame is stale or invalid after clock reset");
            }
            if (cachedCycle == clock.cycle() && cachedTimestamp == timestamp) {
                return cached;
            }
            LimelightColorFrame frame = snapshot.colorFrame();
            TargetObservations2d next = frame != null && frame.isAvailable()
                    ? project(frame.rays(), timestamp, config, mount)
                    : TargetObservations2d.unavailable(frame == null
                            ? "Limelight color snapshot is unavailable" : frame.reason());
            cached = next;
            cachedCycle = clock.cycle();
            cachedTimestamp = timestamp;
            return next;
        }

        @Override
        public void reset() {
            cachedCycle = Long.MIN_VALUE;
        }
    }
}
