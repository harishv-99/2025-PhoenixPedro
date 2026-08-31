package edu.ftcsushi.robots.examples.pedro.adaptive;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;

/**
 * Optional example service that selects one field-Y collection band from Limelight detections.
 *
 * <p>This is robot-owned example policy, not a generic Sushi object-detection abstraction. The
 * service is the sole owner of one {@link FtcLimelightVisionLane}. It immediately copies the two
 * principal-pixel detector angles needed by this example, projects those immutable values to the
 * field floor, and atomically publishes one cached {@link Decision}.</p>
 *
 * <p>The service queries an adopting localization owner's read-only planar pose history at the
 * exact camera exposure timestamp. Exact and safely interpolated lookups both support a moving
 * robot; typed unavailable history outcomes remain explicit fallback evidence. The service neither
 * advances nor resets localization or its history.</p>
 *
 * <p>Declare the authoritative localization owner before this service in {@link RobotProgram}.
 * That owner must update localization and record the final current pose before this service runs;
 * this service then publishes before Tasks freeze its cached decision.</p>
 */
public final class AdaptiveCollectionVisionService implements RobotProgram.Service {

    /** Mutable data-only configuration; defaults are software-valid, not physically calibrated. */
    public static final class Config {
        /** Generic owner configuration for the one Limelight used by this example. */
        public FtcLimelightVisionLane.Config limelight;

        /**
         * Calibrated fixed robot-to-camera transform; identity is only a software placeholder.
         *
         * <p>This example does not model an articulated or otherwise moving camera mount. Such a
         * mount needs its own timestamp-aware transform at the detector-frame timestamp.</p>
         */
        public CameraMountConfig cameraMount;

        /** Inclusive maximum age of a camera exposure eligible for selection, in seconds. */
        public double maxFrameAgeSec;

        /** Inclusive minimum collection-box X in the current Sushi field frame, in inches. */
        public double minCollectionFieldXInches;

        /** Inclusive maximum collection-box X in the current Sushi field frame, in inches. */
        public double maxCollectionFieldXInches;

        /** Inclusive minimum collection-box Y in the current Sushi field frame, in inches. */
        public double minCollectionFieldYInches;

        /** Inclusive maximum collection-box Y in the current Sushi field frame, in inches. */
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
        /** The pose history could not answer at the frame timestamp; inspect the retained lookup. */
        POSE_HISTORY_UNAVAILABLE,
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
        private final PlanarPoseHistory.Lookup poseLookup;
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
                         PlanarPoseHistory.Lookup poseLookup,
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
            this.poseLookup = poseLookup;
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

        /**
         * Returns whether this decision retained the exact history lookup used for projection or
         * fallback diagnosis.
         *
         * <p>Frame-level rejection can happen before a history query is meaningful. Once queried,
         * both available and unavailable lookup results are retained.</p>
         */
        public boolean hasPoseLookup() {
            return poseLookup != null;
        }

        /**
         * Returns the exact immutable pose-history lookup used by this decision.
         *
         * @throws IllegalStateException if frame-level rejection occurred before history lookup
         */
        public PlanarPoseHistory.Lookup poseLookup() {
            if (poseLookup == null) {
                throw new IllegalStateException(
                        "Adaptive collection decision did not reach the pose-history lookup");
            }
            return poseLookup;
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
            return new Decision(
                    LoopTimestamp.unavailable(),
                    Double.NaN,
                    1,
                    1,
                    1,
                    null,
                    true,
                    bandCenterYInches,
                    bandCenterYInches,
                    1,
                    null
            );
        }

        static Decision unavailableForHardwareNeutralTest(UnavailableReason reason) {
            return unavailable(
                    LoopTimestamp.unavailable(),
                    Double.NaN,
                    0,
                    0,
                    0,
                    null,
                    Objects.requireNonNull(reason, "reason")
            );
        }

        private static Decision selected(LoopTimestamp timestamp,
                                         double frameAgeSec,
                                         int detectionCount,
                                         int projectablePointCount,
                                         int inBoxPointCount,
                                         PlanarPoseHistory.Lookup poseLookup,
                                         double bandStartYInches,
                                         double bandEndYInches,
                                         int bandPointCount) {
            return new Decision(
                    timestamp,
                    frameAgeSec,
                    detectionCount,
                    projectablePointCount,
                    inBoxPointCount,
                    Objects.requireNonNull(poseLookup, "poseLookup"),
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
                                            PlanarPoseHistory.Lookup poseLookup,
                                            UnavailableReason reason) {
            return new Decision(
                    timestamp,
                    frameAgeSec,
                    detectionCount,
                    projectablePointCount,
                    inBoxPointCount,
                    poseLookup,
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
            null,
            UnavailableReason.NOT_OBSERVED
    );

    private final Config cfg;
    private final TimeAwareSource<PlanarPoseHistory.Lookup> fieldPoseHistory;
    private final FrameOwner frameOwner;
    private Decision decision = NOT_OBSERVED;

    /**
     * Constructs and becomes the sole owner of one configured Limelight.
     *
     * <p>All configuration and collaborators are validated before the hardware owner is opened.
     * Construction starts Limelight polling but performs no result or pose read.</p>
     *
     * @param hardwareMap FTC hardware map used to open the configured Limelight
     * @param fieldPoseHistory stable read-only history query recorded earlier in service order
     * @param config mutable authoring config; defensively copied before retention
     */
    public AdaptiveCollectionVisionService(HardwareMap hardwareMap,
                                           TimeAwareSource<PlanarPoseHistory.Lookup> fieldPoseHistory,
                                           Config config) {
        HardwareMap requiredMap = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        TimeAwareSource<PlanarPoseHistory.Lookup> requiredHistory = Objects.requireNonNull(
                fieldPoseHistory, "fieldPoseHistory is required");
        Config snapshot = validatedCopy(config);

        this.cfg = snapshot;
        this.fieldPoseHistory = requiredHistory;
        this.frameOwner = new LimelightFrameOwner(
                new FtcLimelightVisionLane(requiredMap, snapshot.limelight));
    }

    /** Explicit hardware-neutral seam; production callers use the sole public constructor. */
    AdaptiveCollectionVisionService(
            TimeAwareSource<PlanarPoseHistory.Lookup> fieldPoseHistory,
            Config config,
            FrameOwner frameOwner) {
        this.cfg = validatedCopy(config);
        this.fieldPoseHistory = Objects.requireNonNull(
                fieldPoseHistory, "fieldPoseHistory is required");
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
        Decision calculated = calculateDecision(
                requiredClock, frame, fieldPoseHistory, cfg);
        decision = calculated;
    }

    @Override
    public void stop() {
        decision = NOT_OBSERVED;
        frameOwner.close();
    }

    private static Decision calculateDecision(LoopClock clock,
                                              CopiedFrame frame,
                                              TimeAwareSource<PlanarPoseHistory.Lookup>
                                                      fieldPoseHistory,
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

        PlanarPoseHistory.Lookup poseLookup = Objects.requireNonNull(
                fieldPoseHistory.getAt(clock, frameTimestamp),
                "fieldPoseHistory.getAt(clock, frameTimestamp) returned null");
        requireLookupTimestamp(poseLookup, frameTimestamp);
        if (!poseLookup.isAvailable()) {
            return unavailable(frame, frameAgeSec, 0, 0, poseLookup,
                    UnavailableReason.POSE_HISTORY_UNAVAILABLE);
        }
        Pose2d planarFieldToRobotPose = poseLookup.fieldToRobotPose();
        Pose3d fieldToRobotPose = new Pose3d(
                planarFieldToRobotPose.xInches,
                planarFieldToRobotPose.yInches,
                0.0,
                planarFieldToRobotPose.headingRad,
                0.0,
                0.0
        );

        AdaptiveCollectionProjection.Result projection = AdaptiveCollectionProjection.select(
                frame.detectorAngles,
                fieldToRobotPose,
                config.cameraMount.robotToCameraPose(),
                config.minCollectionFieldXInches,
                config.maxCollectionFieldXInches,
                config.minCollectionFieldYInches,
                config.maxCollectionFieldYInches,
                config.bandWidthInches
        );
        if (projection.projectablePointCount == 0) {
            return unavailable(frame, frameAgeSec, 0, 0, poseLookup,
                    UnavailableReason.NO_PROJECTABLE_FLOOR_INTERSECTIONS);
        }
        if (projection.inBoxPointCount == 0) {
            return unavailable(frame, frameAgeSec, projection.projectablePointCount, 0, poseLookup,
                    UnavailableReason.NO_POINTS_IN_COLLECTION_BOX);
        }
        return Decision.selected(
                frameTimestamp,
                frameAgeSec,
                frame.detectorAngles.size(),
                projection.projectablePointCount,
                projection.inBoxPointCount,
                poseLookup,
                projection.bandStartYInches,
                projection.bandEndYInches,
                projection.bandPointCount
        );
    }

    private static void requireLookupTimestamp(PlanarPoseHistory.Lookup lookup,
                                               LoopTimestamp requestedTimestamp) {
        LoopTimestamp actualTimestamp = Objects.requireNonNull(
                lookup.timestamp(),
                "fieldPoseHistory returned a Lookup with a null timestamp"
        );
        final double offsetSec;
        try {
            offsetSec = actualTimestamp.secondsSince(requestedTimestamp);
        } catch (IllegalArgumentException wrongClock) {
            throw new IllegalStateException(
                    "fieldPoseHistory must return a Lookup for the exact requested frame "
                            + "timestamp from the shared LoopClock",
                    wrongClock
            );
        }
        if (!Double.isFinite(offsetSec) || offsetSec != 0.0) {
            throw new IllegalStateException(
                    "fieldPoseHistory returned a Lookup for a different timestamp; pass the "
                            + "stable PlanarPoseHistory.lookupSource() projection and preserve "
                            + "the requested frame timestamp"
            );
        }
    }

    private static Decision unavailable(CopiedFrame frame, double frameAgeSec,
                                        int projectableCount, int inBoxCount,
                                        UnavailableReason reason) {
        return Decision.unavailable(frame.frameTimestamp, frameAgeSec,
                frame.detectorAngles.size(), projectableCount, inBoxCount, null, reason);
    }

    private static Decision unavailable(CopiedFrame frame, double frameAgeSec,
                                        int projectableCount, int inBoxCount,
                                        PlanarPoseHistory.Lookup poseLookup,
                                        UnavailableReason reason) {
        return Decision.unavailable(frame.frameTimestamp, frameAgeSec,
                frame.detectorAngles.size(), projectableCount, inBoxCount, poseLookup, reason);
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
