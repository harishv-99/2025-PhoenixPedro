package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.localization.PinpointKinematicSnapshot;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;

/**
 * Passive Pedro Localizer view over Phoenix's one Pinpoint hardware owner.
 *
 * <p>This type deliberately has no hardware update operation. The Phoenix localization lane polls
 * its predictor first; immediately before the owned Pedro heartbeat, the adapter supplies the same
 * {@link LoopClock}. Pedro then consumes only the predictor's immutable same-cycle snapshot.</p>
 */
final class PedroPathingPassiveLocalizer implements Localizer {

    /** Narrow read/rebase seam that makes a second hardware poll impossible in this class. */
    interface PredictorAccess {
        Sample currentSnapshot();

        void setPose(Pose2d phoenixFieldToRobotPose);
    }

    /** Internal immutable copy of the predictor values required by Pedro. */
    static final class Sample {
        final Pose2d phoenixFieldToRobotPose;
        final boolean hasPose;
        final boolean hasVelocity;
        final long cycle;
        final double timestampSec;
        final double phoenixFieldVelocityXInchesPerSec;
        final double phoenixFieldVelocityYInchesPerSec;
        final double angularVelocityRadPerSec;
        final double totalHeadingRad;

        Sample(Pose2d phoenixFieldToRobotPose,
               boolean hasPose,
               boolean hasVelocity,
               long cycle,
               double timestampSec,
               double phoenixFieldVelocityXInchesPerSec,
               double phoenixFieldVelocityYInchesPerSec,
               double angularVelocityRadPerSec,
               double totalHeadingRad) {
            this.phoenixFieldToRobotPose = phoenixFieldToRobotPose;
            this.hasPose = hasPose;
            this.hasVelocity = hasVelocity;
            this.cycle = cycle;
            this.timestampSec = timestampSec;
            this.phoenixFieldVelocityXInchesPerSec = phoenixFieldVelocityXInchesPerSec;
            this.phoenixFieldVelocityYInchesPerSec = phoenixFieldVelocityYInchesPerSec;
            this.angularVelocityRadPerSec = angularVelocityRadPerSec;
            this.totalHeadingRad = totalHeadingRad;
        }

        static Sample unavailable() {
            return new Sample(
                    Pose2d.zero(),
                    false,
                    false,
                    PinpointKinematicSnapshot.NO_CYCLE,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            );
        }

        static Sample from(PinpointKinematicSnapshot snapshot) {
            PinpointKinematicSnapshot value = Objects.requireNonNull(
                    snapshot,
                    "PinpointOdometryPredictor returned a null kinematic snapshot"
            );
            return new Sample(
                    value.fieldToRobotPose,
                    value.hasPose,
                    value.hasVelocity,
                    value.cycle,
                    value.timestampSec,
                    value.fieldVelocityXInchesPerSec,
                    value.fieldVelocityYInchesPerSec,
                    value.angularVelocityRadPerSec,
                    value.totalHeadingRad
            );
        }
    }

    private static final double POSE_SYNC_TOLERANCE = 1e-9;

    private final PredictorAccess predictorAccess;
    private final PedroFieldTransform fieldTransform;

    private Sample currentSample = Sample.unavailable();
    private LoopClock preparedClock;
    private int resetImuCallCount;
    private boolean constructionComplete;
    private boolean startPoseAssigned;
    private boolean heartbeatCompleted;

    PedroPathingPassiveLocalizer(PinpointOdometryPredictor motionPredictor,
                                 PedroFieldTransform fieldTransform) {
        this(new PredictorSnapshotAccess(motionPredictor), fieldTransform);
    }

    PedroPathingPassiveLocalizer(PredictorAccess predictorAccess,
                                 PedroFieldTransform fieldTransform) {
        this.predictorAccess = Objects.requireNonNull(predictorAccess, "predictorAccess");
        this.fieldTransform = Objects.requireNonNull(fieldTransform, "fieldTransform");
    }

    /** Mark the pinned Follower/PoseTracker construction reset as fully consumed. */
    void completeFollowerConstruction() {
        if (constructionComplete) {
            throw new IllegalStateException("Pedro Follower construction was already completed");
        }
        if (resetImuCallCount != 1) {
            throw new IllegalStateException(
                    "Pedro 2.1.2 Follower construction must call Localizer.resetIMU() exactly once; "
                            + "observed " + resetImuCallCount
            );
        }
        constructionComplete = true;
    }

    /** Validate a starting-pose request before Pedro mutates its PoseTracker state. */
    void requireStartingPoseAllowed(Pose pedroStartPose) {
        fieldTransform.pedroToPhoenixFieldPose(pedroStartPose);
        if (!constructionComplete) {
            throw new IllegalStateException(
                    "Set the Pedro starting pose only after the production Follower is constructed"
            );
        }
        if (heartbeatCompleted) {
            throw new IllegalStateException(
                    "Pedro starting pose must be set before the first heartbeat; apply later pose "
                            + "corrections through the Phoenix localization owner"
            );
        }
    }

    /** Bind the shared Phoenix cycle that the next pinned Follower update must consume. */
    void prepareForHeartbeat(LoopClock clock) {
        if (!constructionComplete) {
            throw new IllegalStateException("Pedro Follower construction is not complete");
        }
        if (!startPoseAssigned) {
            throw new IllegalStateException(
                    "Set PedroPathingRuntime.setStartingPose(pedroStartPose) before the first "
                            + "Pedro heartbeat"
            );
        }
        preparedClock = Objects.requireNonNull(
                clock,
                "Pedro passive localization requires the shared LoopClock"
        );
    }

    @Override
    public Pose getPose() {
        if (!currentSample.hasPose || currentSample.phoenixFieldToRobotPose == null) {
            return nanPedroPose();
        }
        return fieldTransform.phoenixFieldToPedroPose(currentSample.phoenixFieldToRobotPose);
    }

    @Override
    public Pose getVelocity() {
        if (!currentSample.hasVelocity) {
            return nanPedroPose();
        }
        return fieldTransform.phoenixFieldVelocityToPedro(
                currentSample.phoenixFieldVelocityXInchesPerSec,
                currentSample.phoenixFieldVelocityYInchesPerSec,
                currentSample.angularVelocityRadPerSec
        );
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    /**
     * Establishes one coherent start through the Phoenix predictor.
     *
     * <p>{@link PinpointOdometryPredictor#setPose(Pose2d)} preserves the physical velocity and
     * accumulated heading in its cached sample, so this coordinate rebase cannot look like motion.</p>
     */
    @Override
    public void setStartPose(Pose pedroStartPose) {
        requireStartingPoseAllowed(pedroStartPose);
        Pose2d requestedPhoenixPose = fieldTransform.pedroToPhoenixFieldPose(pedroStartPose);
        predictorAccess.setPose(requestedPhoenixPose);

        Sample rebased = requirePoseSample(
                predictorAccess.currentSnapshot(),
                "Pinpoint predictor did not publish the requested starting pose"
        );
        Pose2d published = rebased.phoenixFieldToRobotPose;
        if (Math.abs(published.xInches - requestedPhoenixPose.xInches) > POSE_SYNC_TOLERANCE
                || Math.abs(published.yInches - requestedPhoenixPose.yInches) > POSE_SYNC_TOLERANCE
                || Math.abs(Pose2d.wrapToPi(
                        published.headingRad - requestedPhoenixPose.headingRad
                )) > POSE_SYNC_TOLERANCE) {
            throw new IllegalStateException(
                    "Pinpoint predictor starting pose does not match the requested Pedro pose; "
                            + "requested Phoenix pose " + requestedPhoenixPose
                            + ", published " + published
            );
        }

        currentSample = rebased;
        startPoseAssigned = true;
    }

    /**
     * Rejects raw Pedro pose mutation because it bypasses Phoenix correction history and timing.
     */
    @Override
    public void setPose(Pose ignored) {
        throw new IllegalStateException(
                "Raw Pedro pose resets are unsupported in the shared Phoenix runtime; apply the "
                        + "correction through the Phoenix localization owner"
        );
    }

    /** Consume one already-polled, exact-cycle Pinpoint sample; never poll hardware here. */
    @Override
    public void update() {
        LoopClock expectedClock = preparedClock;
        preparedClock = null;
        if (expectedClock == null) {
            throw new IllegalStateException(
                    "Pedro passive Localizer.update() must run through the owned "
                            + "PedroPathingDriveAdapter heartbeat"
            );
        }

        Sample next = Objects.requireNonNull(
                predictorAccess.currentSnapshot(),
                "Pinpoint predictor returned a null kinematic snapshot"
        );
        if (next.cycle == PinpointKinematicSnapshot.NO_CYCLE
                || next.cycle != expectedClock.cycle()) {
            throw new IllegalStateException(
                    "Pedro requires a current Pinpoint snapshot for Phoenix cycle "
                            + expectedClock.cycle() + ", but found cycle " + next.cycle
                            + ". Update Phoenix localization with the shared LoopClock before the "
                            + "Pedro drive heartbeat"
            );
        }
        if (!next.hasPose) {
            throw new IllegalStateException(
                    "Pinpoint pose is unavailable for Phoenix cycle " + expectedClock.cycle()
                            + "; Pedro drive output was stopped"
            );
        }
        if (!next.hasVelocity) {
            throw new IllegalStateException(
                    "Pinpoint physical velocity is unavailable for Phoenix cycle "
                            + expectedClock.cycle()
                            + "; wait for a valid post-reset sample before driving Pedro"
            );
        }
        requireFiniteSample(next);

        currentSample = next;
        heartbeatCompleted = true;
    }

    @Override
    public double getTotalHeading() {
        return currentSample.totalHeadingRad;
    }

    /** Tuning uses the separately named native-localizer tool runtime. */
    @Override
    public double getForwardMultiplier() {
        return Double.NaN;
    }

    /** Tuning uses the separately named native-localizer tool runtime. */
    @Override
    public double getLateralMultiplier() {
        return Double.NaN;
    }

    /** Tuning uses the separately named native-localizer tool runtime. */
    @Override
    public double getTurningMultiplier() {
        return Double.NaN;
    }

    /**
     * Suppresses only the one duplicate reset hard-coded by pinned Pedro 2.1.2 construction.
     */
    @Override
    public void resetIMU() {
        if (!constructionComplete && resetImuCallCount == 0) {
            resetImuCallCount = 1;
            return;
        }
        throw new IllegalStateException(
                "Raw Pedro resetIMU() is unsupported after the controlled Phoenix Pinpoint INIT "
                        + "reset; use the Phoenix localization owner for a coordinated reset"
        );
    }

    /** Phoenix's Pinpoint owner is the IMU boundary; Pedro must not reset against a second view. */
    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    @Override
    public boolean isNAN() {
        Pose pose = getPose();
        return !Double.isFinite(pose.getX())
                || !Double.isFinite(pose.getY())
                || !Double.isFinite(pose.getHeading());
    }

    private static Sample requirePoseSample(Sample sample, String message) {
        if (sample == null || !sample.hasPose || sample.phoenixFieldToRobotPose == null) {
            throw new IllegalStateException(message);
        }
        requireFinitePose(sample.phoenixFieldToRobotPose, "Pinpoint pose");
        requireFinite(sample.timestampSec, "Pinpoint sample timestampSec");
        requireFinite(sample.totalHeadingRad, "Pinpoint totalHeadingRad");
        return sample;
    }

    private static void requireFiniteSample(Sample sample) {
        requirePoseSample(sample, "Pinpoint pose is unavailable");
        requireFinite(sample.phoenixFieldVelocityXInchesPerSec,
                "Pinpoint fieldVelocityXInchesPerSec");
        requireFinite(sample.phoenixFieldVelocityYInchesPerSec,
                "Pinpoint fieldVelocityYInchesPerSec");
        requireFinite(sample.angularVelocityRadPerSec, "Pinpoint angularVelocityRadPerSec");
    }

    private static void requireFinitePose(Pose2d pose, String name) {
        requireFinite(pose.xInches, name + ".xInches");
        requireFinite(pose.yInches, name + ".yInches");
        requireFinite(pose.headingRad, name + ".headingRad");
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalStateException(name + " must be finite, got " + value);
        }
    }

    private static Pose nanPedroPose() {
        return new Pose(
                Double.NaN,
                Double.NaN,
                Double.NaN,
                PedroCoordinates.INSTANCE
        );
    }

    /** Production read/rebase view; it exposes no Pinpoint polling method. */
    private static final class PredictorSnapshotAccess implements PredictorAccess {
        private final PinpointOdometryPredictor predictor;

        private PredictorSnapshotAccess(PinpointOdometryPredictor predictor) {
            this.predictor = Objects.requireNonNull(predictor, "motionPredictor");
        }

        @Override
        public Sample currentSnapshot() {
            return Sample.from(predictor.getKinematicSnapshot());
        }

        @Override
        public void setPose(Pose2d phoenixFieldToRobotPose) {
            predictor.setPose(phoenixFieldToRobotPose);
        }
    }
}
