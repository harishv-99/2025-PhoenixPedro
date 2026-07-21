package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.Objects;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcphoenix.fw.ftc.vision.FtcWebcamVisionPortalLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;

/**
 * Compiling ownership sketch for season-specific vision beyond the shared AprilTag seam.
 *
 * <p>Auto and TeleOp consume only {@link RobotVision}, {@link Mode}, and the immutable timestamped
 * {@link Snapshot}. The two realization classes below contain the different FTC-boundary details:
 * a webcam toggles processors that were all supplied at construction, while a Limelight requests
 * one onboard pipeline only when the semantic mode changes.</p>
 */
public final class CustomVisionOwnershipExample {

    private CustomVisionOwnershipExample() {
        // Example namespace only.
    }

    /** Meanings chosen by this example robot, not by either camera backend. */
    public enum Mode {
        DRIVER_VIEW,
        AIMING
    }

    /** Small immutable, timestamped result consumed by robot strategy. */
    public static final class Snapshot {
        private static final Snapshot UNAVAILABLE =
                new Snapshot(false, 0, Double.NaN);

        public final boolean hasTarget;
        public final int candidateCount;
        /** Approximate observation time in the current OpMode's {@link LoopClock} timebase. */
        public final double observedAtSec;

        private Snapshot(boolean hasTarget, int candidateCount, double observedAtSec) {
            this.hasTarget = hasTarget;
            this.candidateCount = candidateCount;
            this.observedAtSec = observedAtSec;
        }

        /** Returns one accepted observation, including a ready frame that saw no target. */
        public static Snapshot observed(
                boolean hasTarget,
                int candidateCount,
                double observedAtSec
        ) {
            if (candidateCount < 0) {
                throw new IllegalArgumentException("candidateCount must be >= 0");
            }
            if (!Double.isFinite(observedAtSec)) {
                throw new IllegalArgumentException("observedAtSec must be finite");
            }
            return new Snapshot(hasTarget, candidateCount, observedAtSec);
        }

        /** Returns the intentional no-observation value used while this capability is not ready. */
        public static Snapshot unavailable() {
            return UNAVAILABLE;
        }
    }

    /** The one robot-owned surface shared by Auto, TeleOp, and strategy. */
    public interface RobotVision extends AutoCloseable {
        void select(Mode mode);

        VisionReadiness readiness(LoopClock clock);

        Snapshot snapshot(LoopClock clock);

        @Override
        void close();
    }

    /** Webcam realization: FTC processor types stop inside this class. */
    public static final class WebcamRobotVision implements RobotVision {

        /**
         * Robot-owned typed processor contract. Its implementation discards pre-generation
         * results and reports readiness only after publishing a newer immutable snapshot that is
         * still within the robot's chosen freshness bound. Both result methods evaluate against
         * the supplied {@link LoopClock}; the snapshot method converts the stored frame age into
         * that clock's timebase. A fresh frame that saw no target is still a ready observation.
         */
        public interface AimingProcessor extends VisionProcessor {
            void beginResultGeneration();

            VisionReadiness resultReadiness(LoopClock clock);

            Snapshot latestSnapshot(LoopClock clock);
        }

        private final VisionProcessor driverViewProcessor;
        private final AimingProcessor aimingProcessor;
        private final FtcWebcamVisionPortalLane owner;
        private Mode mode;

        public WebcamRobotVision(
                HardwareMap hardwareMap,
                String webcamName,
                VisionProcessor driverViewProcessor,
                AimingProcessor aimingProcessor
        ) {
            this.driverViewProcessor = Objects.requireNonNull(
                    driverViewProcessor, "driverViewProcessor");
            this.aimingProcessor = Objects.requireNonNull(aimingProcessor, "aimingProcessor");

            FtcWebcamVisionPortalLane.Config config =
                    FtcWebcamVisionPortalLane.Config.defaults();
            config.webcamName = webcamName;
            owner = new FtcWebcamVisionPortalLane(
                    hardwareMap,
                    config,
                    this.driverViewProcessor,
                    this.aimingProcessor
            );

            // Both processors were declared before the portal was built; mode changes only toggle.
            // select(...) owns failure cleanup, including this initial transition.
            select(Mode.DRIVER_VIEW);
        }

        @Override
        public void select(Mode mode) {
            Mode requested = Objects.requireNonNull(mode, "mode");
            if (requested == this.mode) {
                return;
            }

            try {
                if (requested == Mode.AIMING) {
                    aimingProcessor.beginResultGeneration();
                    owner.setProcessorEnabled(aimingProcessor, true);
                    owner.setProcessorEnabled(driverViewProcessor, false);
                } else {
                    owner.setProcessorEnabled(driverViewProcessor, true);
                    owner.setProcessorEnabled(aimingProcessor, false);
                }
                this.mode = requested;
            } catch (RuntimeException transitionFailure) {
                // A partial processor transition has uncertain semantics. Make this capability
                // terminal and preserve cleanup failures on the original exception.
                throw CleanupActions.attemptAllAfterFailure(transitionFailure, owner::close);
            }
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            if (mode == Mode.AIMING) {
                VisionReadiness processor = owner.processorReadiness(aimingProcessor);
                return processor.isReady()
                        ? Objects.requireNonNull(
                                aimingProcessor.resultReadiness(clock),
                                "aimingProcessor.resultReadiness(clock)")
                        : processor;
            }
            return owner.processorReadiness(driverViewProcessor);
        }

        @Override
        public Snapshot snapshot(LoopClock clock) {
            return mode == Mode.AIMING && readiness(clock).isReady()
                    ? Objects.requireNonNull(
                            aimingProcessor.latestSnapshot(clock),
                            "aimingProcessor.latestSnapshot(clock)")
                    : Snapshot.unavailable();
        }

        @Override
        public void close() {
            owner.close();
        }
    }

    /** Limelight realization: pipeline result types stop inside this class. */
    public static final class LimelightRobotVision implements RobotVision {

        private final int driverViewPipeline;
        private final int aimingPipeline;
        private final FtcLimelightVisionLane owner;
        private Mode mode = Mode.DRIVER_VIEW;

        public LimelightRobotVision(
                HardwareMap hardwareMap,
                String hardwareName,
                int driverViewPipeline,
                int aimingPipeline
        ) {
            this.driverViewPipeline = requirePipelineIndex(
                    driverViewPipeline, "driverViewPipeline");
            this.aimingPipeline = requirePipelineIndex(aimingPipeline, "aimingPipeline");

            FtcLimelightVisionLane.Config config = FtcLimelightVisionLane.Config.defaults();
            config.hardwareName = hardwareName;
            config.pipelineIndex = this.driverViewPipeline;
            FtcLimelightVisionLane opened = new FtcLimelightVisionLane(hardwareMap, config);
            if (!opened.wasPipelineRequestAccepted()) {
                RuntimeException rejection = new IllegalStateException(
                        "Limelight rejected initial pipeline " + this.driverViewPipeline
                                + "; this RobotVision owner is terminal");
                throw CleanupActions.attemptAllAfterFailure(rejection, opened::close);
            }
            owner = opened;
        }

        @Override
        public void select(Mode mode) {
            Mode requested = Objects.requireNonNull(mode, "mode");
            if (requested == this.mode) {
                return;
            }
            // Record the semantic request before the vendor call. If that synchronous call fails,
            // this capability and the owner still agree about which transition was attempted.
            this.mode = requested;
            int pipeline = requested == Mode.AIMING ? aimingPipeline : driverViewPipeline;
            try {
                if (!owner.requestPipeline(pipeline)) {
                    throw new IllegalStateException(
                            "Limelight rejected pipeline " + pipeline
                                    + "; this RobotVision owner is terminal");
                }
            } catch (RuntimeException transitionFailure) {
                // A rejected or failed request cannot be retried on this generation. Close the
                // physical owner now so callers cannot accidentally continue using uncertain state.
                throw CleanupActions.attemptAllAfterFailure(transitionFailure, owner::close);
            }
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            return owner.pipelineReadiness(clock);
        }

        @Override
        public Snapshot snapshot(LoopClock clock) {
            if (mode != Mode.AIMING || !readiness(clock).isReady()) {
                return Snapshot.unavailable();
            }
            FtcLimelightVisionLane.ResultSnapshot result =
                    owner.confirmedPipelineResult(clock);
            int candidates = result.detectorResults().size()
                    + result.classifierResults().size()
                    + result.fiducialResults().size()
                    + result.colorResults().size()
                    + result.barcodeResults().size();
            return Snapshot.observed(
                    result.isTargetValid(),
                    candidates,
                    clock.nowSec() - result.ageSec()
            );
        }

        @Override
        public void close() {
            owner.close();
        }

        private static int requirePipelineIndex(int pipelineIndex, String name) {
            if (pipelineIndex < 0 || pipelineIndex > 9) {
                throw new IllegalArgumentException(name + " must be within [0, 9]");
            }
            return pipelineIndex;
        }
    }
}
