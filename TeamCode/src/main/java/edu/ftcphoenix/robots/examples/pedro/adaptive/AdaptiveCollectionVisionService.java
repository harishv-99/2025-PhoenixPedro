package edu.ftcphoenix.robots.examples.pedro.adaptive;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * Optional example service that selects one field-Y collection band from Limelight detections.
 *
 * <p>This is robot-owned example policy, not a generic Phoenix object-detection abstraction. The
 * service is the sole owner of one {@link FtcLimelightVisionLane}. It immediately copies the two
 * principal-pixel detector angles needed by this example, projects those immutable values to the
 * field floor, and atomically publishes one cached {@link Decision}.</p>
 *
 * <p>The projection deliberately supports only a stationary observation interval. The adopting
 * composition root must ensure that the robot and rigidly mounted camera remain stationary from
 * camera exposure until this service samples the current pose. Software checks that the pose is
 * available, finite, and timestamped at the current clock time; it does not measure or prove
 * stationarity. A moving robot needs a separately owned pose history at the frame timestamp.</p>
 *
 * <p>Declare the current-pose owner before this service in {@link RobotProgram}. The managed host
 * then updates localization before this service, and this service before Tasks that freeze its
 * cached decision.</p>
 */
public final class AdaptiveCollectionVisionService implements RobotProgram.Service {

    /** Mutable data-only configuration; defaults are software-valid, not physically calibrated. */
    public static final class Config {
        /** Generic owner configuration for the one Limelight used by this example. */
        public FtcLimelightVisionLane.Config limelight;

        /** Calibrated robot-to-camera transform; identity is only a software placeholder. */
        public CameraMountConfig cameraMount;

        /** Inclusive maximum age of a camera exposure eligible for selection, in seconds. */
        public double maxFrameAgeSec;

        /** Inclusive minimum collection-box X in the current Phoenix field frame, in inches. */
        public double minCollectionFieldXInches;

        /** Inclusive maximum collection-box X in the current Phoenix field frame, in inches. */
        public double maxCollectionFieldXInches;

        /** Inclusive minimum collection-box Y in the current Phoenix field frame, in inches. */
        public double minCollectionFieldYInches;

        /** Inclusive maximum collection-box Y in the current Phoenix field frame, in inches. */
        public double maxCollectionFieldYInches;

        /** Width of every candidate collection band along field Y, in inches. */
        public double bandWidthInches;

        private Config() {
        }

        /**
         * Returns a software-valid authoring baseline.
         *
         * <p>The identity mount is not a calibrated physical camera placement and normally cannot
         * produce a positive floor intersection. Adopters must replace it and review the field box
         * for their season and route.</p>
         */
        public static Config defaults() {
            Config config = new Config();
            config.limelight = FtcLimelightVisionLane.Config.defaults();
            config.cameraMount = CameraMountConfig.identity();
            config.maxFrameAgeSec = 0.25;
            config.minCollectionFieldXInches = 0.0;
            config.maxCollectionFieldXInches = 72.0;
            config.minCollectionFieldYInches = -72.0;
            config.maxCollectionFieldYInches = 72.0;
            config.bandWidthInches = 18.0;
            return config;
        }
    }

    /** Exact typed reason that the latest complete observation did not select a band. */
    public enum UnavailableReason {
        /** No active-lifecycle observation has been published, or the service has stopped. */
        NOT_OBSERVED,
        /** The Limelight pipeline has no confirmed result or no exposure timestamp. */
        FRAME_UNAVAILABLE,
        /** The exposure timestamp belongs to a reset epoch or is materially in the future. */
        FRAME_RESET_OR_FUTURE,
        /** The exposure is older than {@link Config#maxFrameAgeSec}. */
        FRAME_STALE,
        /** The current-pose owner did not publish a finite usable field pose. */
        POSE_UNAVAILABLE,
        /** The pose timestamp is reset-stale, future, or older than the current clock time. */
        POSE_NOT_CURRENT,
        /** The confirmed result contained no detector results. */
        ZERO_DETECTIONS,
        /** No finite detector ray intersected the floor in front of the camera. */
        NO_PROJECTABLE_FLOOR_INTERSECTIONS,
        /** Floor intersections existed, but none were inside the configured collection box. */
        NO_POINTS_IN_COLLECTION_BOX
    }

    /** Immutable result from one complete service calculation. */
    public static final class Decision {
        private final LoopTimestamp frameTimestamp;
        private final double frameAgeSec;
        private final int detectionCount;
        private final int projectablePointCount;
        private final int inBoxPointCount;
        private final boolean hasSelection;
        private final double selectedBandStartYInches;
        private final double selectedBandEndYInches;
        private final int selectedBandPointCount;
        private final UnavailableReason unavailableReason;

        private Decision(LoopTimestamp frameTimestamp,
                         double frameAgeSec,
                         int detectionCount,
                         int projectablePointCount,
                         int inBoxPointCount,
                         boolean hasSelection,
                         double selectedBandStartYInches,
                         double selectedBandEndYInches,
                         int selectedBandPointCount,
                         UnavailableReason unavailableReason) {
            this.frameTimestamp = Objects.requireNonNull(frameTimestamp, "frameTimestamp");
            this.frameAgeSec = frameAgeSec;
            this.detectionCount = detectionCount;
            this.projectablePointCount = projectablePointCount;
            this.inBoxPointCount = inBoxPointCount;
            this.hasSelection = hasSelection;
            this.selectedBandStartYInches = selectedBandStartYInches;
            this.selectedBandEndYInches = selectedBandEndYInches;
            this.selectedBandPointCount = selectedBandPointCount;
            this.unavailableReason = unavailableReason;
        }

        /** Returns the exact Limelight exposure timestamp retained by this decision. */
        public LoopTimestamp frameTimestamp() {
            return frameTimestamp;
        }

        /** Returns frame age when calculated, or NaN before a usable timestamp was available. */
        public double frameAgeSec() {
            return frameAgeSec;
        }

        /** Returns the number of detector results copied from the confirmed frame. */
        public int detectionCount() {
            return detectionCount;
        }

        /** Returns how many copied rays produced a finite positive floor intersection. */
        public int projectablePointCount() {
            return projectablePointCount;
        }

        /** Returns how many projected points were inside the inclusive collection box. */
        public int inBoxPointCount() {
            return inBoxPointCount;
        }

        /** Returns whether this decision selected a collection band. */
        public boolean hasSelection() {
            return hasSelection;
        }

        /** Returns the selected band's inclusive lower field-Y boundary, in inches. */
        public double selectedBandStartYInches() {
            requireSelection();
            return selectedBandStartYInches;
        }

        /** Returns the selected band's inclusive upper field-Y boundary, in inches. */
        public double selectedBandEndYInches() {
            requireSelection();
            return selectedBandEndYInches;
        }

        /** Returns the selected band's field-Y center used as the route target, in inches. */
        public double selectedBandCenterYInches() {
            requireSelection();
            return selectedBandStartYInches
                    + (selectedBandEndYInches - selectedBandStartYInches) * 0.5;
        }

        /** Returns how many in-box points were contained by the selected inclusive band. */
        public int selectedBandPointCount() {
            requireSelection();
            return selectedBandPointCount;
        }

        /** Returns the typed unavailable reason; selected decisions reject this accessor. */
        public UnavailableReason unavailableReason() {
            if (hasSelection) {
                throw new IllegalStateException(
                        "Adaptive collection decision selected a band; no unavailable reason exists");
            }
            return unavailableReason;
        }

        private void requireSelection() {
            if (!hasSelection) {
                throw new IllegalStateException(
                        "Adaptive collection decision has no selected band; reason="
                                + unavailableReason);
            }
        }

        static Decision selectedForHardwareNeutralTest(double bandCenterYInches) {
            requireFinite("bandCenterYInches", bandCenterYInches);
            return selected(
                    LoopTimestamp.unavailable(),
                    Double.NaN,
                    1,
                    1,
                    1,
                    bandCenterYInches,
                    bandCenterYInches,
                    1
            );
        }

        static Decision unavailableForHardwareNeutralTest(UnavailableReason reason) {
            return unavailable(
                    LoopTimestamp.unavailable(),
                    Double.NaN,
                    0,
                    0,
                    0,
                    Objects.requireNonNull(reason, "reason")
            );
        }

        private static Decision selected(LoopTimestamp timestamp,
                                         double frameAgeSec,
                                         int detectionCount,
                                         int projectablePointCount,
                                         int inBoxPointCount,
                                         double bandStartYInches,
                                         double bandEndYInches,
                                         int bandPointCount) {
            return new Decision(
                    timestamp,
                    frameAgeSec,
                    detectionCount,
                    projectablePointCount,
                    inBoxPointCount,
                    true,
                    bandStartYInches,
                    bandEndYInches,
                    bandPointCount,
                    null
            );
        }

        private static Decision unavailable(LoopTimestamp timestamp,
                                            double frameAgeSec,
                                            int detectionCount,
                                            int projectablePointCount,
                                            int inBoxPointCount,
                                            UnavailableReason reason) {
            return new Decision(
                    timestamp,
                    frameAgeSec,
                    detectionCount,
                    projectablePointCount,
                    inBoxPointCount,
                    false,
                    Double.NaN,
                    Double.NaN,
                    0,
                    Objects.requireNonNull(reason, "reason")
            );
        }
    }

    /** Immutable principal-pixel detector angles after the FTC SDK boundary. */
    static final class DetectorAngles {
        final double horizontalRightDeg;
        final double verticalDownDeg;

        DetectorAngles(double horizontalRightDeg, double verticalDownDeg) {
            this.horizontalRightDeg = horizontalRightDeg;
            this.verticalDownDeg = verticalDownDeg;
        }
    }

    /** Immutable camera-frame value used by the hardware-neutral service calculation. */
    static final class CopiedFrame {
        final boolean hasResult;
        final LoopTimestamp frameTimestamp;
        final List<DetectorAngles> detectorAngles;

        private CopiedFrame(boolean hasResult,
                            LoopTimestamp frameTimestamp,
                            List<DetectorAngles> detectorAngles) {
            this.hasResult = hasResult;
            this.frameTimestamp = Objects.requireNonNull(frameTimestamp, "frameTimestamp");
            List<DetectorAngles> source = Objects.requireNonNull(
                    detectorAngles, "detectorAngles");
            ArrayList<DetectorAngles> copied = new ArrayList<DetectorAngles>(source.size());
            for (DetectorAngles angles : source) {
                copied.add(Objects.requireNonNull(angles, "detectorAngles member"));
            }
            this.detectorAngles = Collections.unmodifiableList(copied);
        }

        static CopiedFrame observed(LoopTimestamp timestamp, List<DetectorAngles> angles) {
            return new CopiedFrame(true, timestamp, angles);
        }

        static CopiedFrame unavailable() {
            return new CopiedFrame(
                    false,
                    LoopTimestamp.unavailable(),
                    Collections.<DetectorAngles>emptyList()
            );
        }
    }

    /** Package-private hardware-neutral seam for focused service and orchestration tests. */
    interface FrameOwner {
        CopiedFrame confirmedFrame(LoopClock clock);

        void close();
    }

    private static final Decision NOT_OBSERVED = Decision.unavailable(
            LoopTimestamp.unavailable(),
            Double.NaN,
            0,
            0,
            0,
            UnavailableReason.NOT_OBSERVED
    );

    private final Config cfg;
    private final AbsolutePoseEstimator currentFieldPose;
    private final FrameOwner frameOwner;
    private Decision decision = NOT_OBSERVED;

    /**
     * Constructs and becomes the sole owner of one configured Limelight.
     *
     * <p>All configuration and collaborators are validated before the hardware owner is opened.
     * Construction starts Limelight polling but performs no result or pose read.</p>
     *
     * @param hardwareMap FTC hardware map used to open the configured Limelight
     * @param currentFieldPose current Phoenix-field pose cache, updated earlier in service order
     * @param config mutable authoring config; defensively copied before retention
     */
    public AdaptiveCollectionVisionService(HardwareMap hardwareMap,
                                           AbsolutePoseEstimator currentFieldPose,
                                           Config config) {
        HardwareMap requiredMap = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        AbsolutePoseEstimator requiredPose = Objects.requireNonNull(
                currentFieldPose, "currentFieldPose is required");
        Config snapshot = validatedCopy(config);

        this.cfg = snapshot;
        this.currentFieldPose = requiredPose;
        this.frameOwner = new LimelightFrameOwner(
                new FtcLimelightVisionLane(requiredMap, snapshot.limelight));
    }

    /** Explicit hardware-neutral seam; production callers use the sole public constructor. */
    AdaptiveCollectionVisionService(AbsolutePoseEstimator currentFieldPose,
                                    Config config,
                                    FrameOwner frameOwner) {
        this.cfg = validatedCopy(config);
        this.currentFieldPose = Objects.requireNonNull(
                currentFieldPose, "currentFieldPose is required");
        this.frameOwner = Objects.requireNonNull(frameOwner, "frameOwner is required");
    }

    /** Returns the most recently atomically published immutable decision without sampling. */
    public Decision decision() {
        return decision;
    }

    @Override
    public void start(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        decision = NOT_OBSERVED;
        update(clock);
    }

    @Override
    public void update(LoopClock clock) {
        LoopClock requiredClock = Objects.requireNonNull(clock, "clock is required");
        CopiedFrame frame = Objects.requireNonNull(
                frameOwner.confirmedFrame(requiredClock),
                "frameOwner.confirmedFrame(clock) returned null");
        PoseEstimate pose = currentFieldPose.getEstimate();
        Decision calculated = calculateDecision(requiredClock, frame, pose, cfg);
        decision = calculated;
    }

    @Override
    public void stop() {
        decision = NOT_OBSERVED;
        frameOwner.close();
    }

    private static Decision calculateDecision(LoopClock clock,
                                              CopiedFrame frame,
                                              PoseEstimate pose,
                                              Config config) {
        LoopTimestamp frameTimestamp = frame.frameTimestamp;
        if (!frame.hasResult || !frameTimestamp.isAvailable()) {
            return unavailable(frame, Double.NaN, 0, 0,
                    UnavailableReason.FRAME_UNAVAILABLE);
        }

        double frameAgeSec = frameTimestamp.ageSec(clock);
        if (!Double.isFinite(frameAgeSec)) {
            return unavailable(frame, frameAgeSec, 0, 0,
                    UnavailableReason.FRAME_RESET_OR_FUTURE);
        }
        if (frameAgeSec > config.maxFrameAgeSec) {
            return unavailable(frame, frameAgeSec, 0, 0, UnavailableReason.FRAME_STALE);
        }
        if (frame.detectorAngles.isEmpty()) {
            return unavailable(frame, frameAgeSec, 0, 0,
                    UnavailableReason.ZERO_DETECTIONS);
        }
        if (!hasFinitePose(pose)) {
            return unavailable(frame, frameAgeSec, 0, 0, UnavailableReason.POSE_UNAVAILABLE);
        }

        double poseAgeSec = pose.timestamp.ageSec(clock);
        if (!Double.isFinite(poseAgeSec) || poseAgeSec != 0.0) {
            return unavailable(frame, frameAgeSec, 0, 0, UnavailableReason.POSE_NOT_CURRENT);
        }

        AdaptiveCollectionProjection.Result projection = AdaptiveCollectionProjection.select(
                frame.detectorAngles,
                pose.fieldToRobotPose,
                config.cameraMount.robotToCameraPose(),
                config.minCollectionFieldXInches,
                config.maxCollectionFieldXInches,
                config.minCollectionFieldYInches,
                config.maxCollectionFieldYInches,
                config.bandWidthInches
        );
        if (projection.projectablePointCount == 0) {
            return unavailable(frame, frameAgeSec, 0, 0,
                    UnavailableReason.NO_PROJECTABLE_FLOOR_INTERSECTIONS);
        }
        if (projection.inBoxPointCount == 0) {
            return unavailable(frame, frameAgeSec, projection.projectablePointCount, 0,
                    UnavailableReason.NO_POINTS_IN_COLLECTION_BOX);
        }
        return Decision.selected(
                frameTimestamp,
                frameAgeSec,
                frame.detectorAngles.size(),
                projection.projectablePointCount,
                projection.inBoxPointCount,
                projection.bandStartYInches,
                projection.bandEndYInches,
                projection.bandPointCount
        );
    }

    private static Decision unavailable(CopiedFrame frame, double frameAgeSec,
                                        int projectableCount, int inBoxCount,
                                        UnavailableReason reason) {
        return Decision.unavailable(frame.frameTimestamp, frameAgeSec,
                frame.detectorAngles.size(), projectableCount, inBoxCount, reason);
    }

    private static boolean hasFinitePose(PoseEstimate estimate) {
        if (estimate == null || !estimate.hasPose || estimate.fieldToRobotPose == null
                || estimate.timestamp == null) {
            return false;
        }
        Pose3d pose = estimate.fieldToRobotPose;
        return Double.isFinite(pose.xInches)
                && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches)
                && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad)
                && Double.isFinite(pose.rollRad);
    }

    private static Config validatedCopy(Config config) {
        Config source = Objects.requireNonNull(config, "config is required");
        if (source.limelight == null) {
            throw new IllegalArgumentException("config.limelight must not be null");
        }
        if (source.cameraMount == null) {
            throw new IllegalArgumentException("config.cameraMount must not be null");
        }

        Config copy = new Config();
        copy.limelight = source.limelight.copy();
        copy.cameraMount = source.cameraMount;
        copy.maxFrameAgeSec = requirePositive(
                "config.maxFrameAgeSec", source.maxFrameAgeSec);
        copy.minCollectionFieldXInches = requireFinite(
                "config.minCollectionFieldXInches", source.minCollectionFieldXInches);
        copy.maxCollectionFieldXInches = requireFinite(
                "config.maxCollectionFieldXInches", source.maxCollectionFieldXInches);
        copy.minCollectionFieldYInches = requireFinite(
                "config.minCollectionFieldYInches", source.minCollectionFieldYInches);
        copy.maxCollectionFieldYInches = requireFinite(
                "config.maxCollectionFieldYInches", source.maxCollectionFieldYInches);
        if (copy.minCollectionFieldXInches > copy.maxCollectionFieldXInches) {
            throw new IllegalArgumentException(
                    "config collection-field X bounds must satisfy min <= max");
        }
        if (copy.minCollectionFieldYInches >= copy.maxCollectionFieldYInches) {
            throw new IllegalArgumentException(
                    "config collection-field Y bounds must satisfy min < max");
        }

        copy.bandWidthInches = requirePositive(
                "config.bandWidthInches", source.bandWidthInches);
        double ySpanInches = copy.maxCollectionFieldYInches
                - copy.minCollectionFieldYInches;
        if (!Double.isFinite(ySpanInches) || copy.bandWidthInches > ySpanInches) {
            throw new IllegalArgumentException(
                    "config.bandWidthInches must be no larger than the finite collection-field "
                            + "Y span " + ySpanInches + ", got " + copy.bandWidthInches);
        }
        return copy;
    }

    private static double requirePositive(String field, double value) {
        requireFinite(field, value);
        if (value <= 0.0) {
            throw new IllegalArgumentException(field + " must be > 0, got " + value);
        }
        return value;
    }

    private static double requireFinite(String field, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(field + " must be finite, got " + value);
        }
        return value;
    }

    /** Converts mutable SDK detector objects immediately into the two values this example needs. */
    static List<DetectorAngles> copyDetectorAngles(
            List<LLResultTypes.DetectorResult> detectorResults) {
        List<LLResultTypes.DetectorResult> source = Objects.requireNonNull(
                detectorResults, "detectorResults");
        ArrayList<DetectorAngles> copied = new ArrayList<DetectorAngles>(source.size());
        for (LLResultTypes.DetectorResult detector : source) {
            LLResultTypes.DetectorResult required = Objects.requireNonNull(
                    detector, "detectorResults member");
            copied.add(new DetectorAngles(
                    required.getTargetXDegreesNoCrosshair(),
                    required.getTargetYDegreesNoCrosshair()
            ));
        }
        return Collections.unmodifiableList(copied);
    }

    /** Production adapter that keeps the mutable FTC SDK result inside the service boundary. */
    private static final class LimelightFrameOwner implements FrameOwner {
        private final FtcLimelightVisionLane lane;

        LimelightFrameOwner(FtcLimelightVisionLane lane) {
            this.lane = Objects.requireNonNull(lane, "lane");
        }

        @Override
        public CopiedFrame confirmedFrame(LoopClock clock) {
            FtcLimelightVisionLane.ResultSnapshot result =
                    lane.confirmedPipelineResult(clock);
            if (!result.hasResult()) {
                return CopiedFrame.unavailable();
            }
            return CopiedFrame.observed(
                    result.frameTimestamp(),
                    copyDetectorAngles(result.detectorResults())
            );
        }

        @Override
        public void close() {
            lane.close();
        }
    }

}
