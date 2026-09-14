package edu.ftcsushi.fw.drive.guidance;

import java.util.Objects;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.sensing.observation.OccupancyObservation;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionSource;
import edu.ftcsushi.fw.spatial.References;
import edu.ftcsushi.fw.spatial.SpatialApproach2d;
import edu.ftcsushi.fw.task.AbstractTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/**
 * One source-driven, camera-only pickup with a verified, bounded final intake.
 *
 * <p>Construct once through {@link #cameraOnly(TargetSelectionSource)}, request a fresh Task for
 * each attempt, and connect {@link #driveSource()} to the robot's one final drive writer. The
 * Task runner alone advances attempts. A robot capability owns {@link #stop()}; this core owner
 * does not update cameras, localization, hardware, borrowed sources, or a second Task runner.</p>
 *
 * <p>Guide follows live ranking. Close alignment commands zero, waits the configured settling
 * interval, and requires two distinct subsequent qualifying captures before final intake.
 * Commanded zero and repeated image geometry do not prove physical rest or object identity.
 * The final command has a time and normalized-command bound, not a distance or clearance bound.
 * Only independent, fresh empty-to-occupied feedback can establish successful capture.</p>
 *
 * <p>The robot grants this attempt temporary authority over its intake request. The enabling
 * setter runs once, then the persistent request is released once on ending; this owner cannot
 * detect competing mechanism writers or supersession through a plain setter. Keep the rigid
 * camera/tool setup fixed for this owner's lifetime; stop and replace the owner before changing
 * that setup. Normal cancellation returns the explicit idle source; exceptions and STOP select
 * zero. The continuously advanced shared Task runner remains required for all elapsed bounds.</p>
 */
public final class GuidedApproach {
    /** Cached behavioral phase, not a measured physical state. */
    public enum Phase { IDLE, GUIDE, VERIFY, FINAL_INTAKE, DONE, STOPPED }

    /**
     * Immutable last accepted attempt/owner evidence. Refused pre-admission Tasks retain their own
     * CANCELLED outcome without replacing this status. Historical verification is not visibility.
     */
    public static final class Status {
        /** Current requested phase. */
        public final Phase phase;
        /** NOT_DONE while active, during unsettled cleanup, or after an exceptional failure. */
        public final TaskOutcome outcome;
        /** Last decision or ending explanation. */
        public final String reason;
        /** True for a lifecycle failure, which must not be interpreted as ordinary cancellation. */
        public final boolean hasFailure;
        /** Exact last sampled selection, including its original whole frame. */
        public final TargetSelectionResult selection;
        /** Exact last accepted post-settle verification frame, or unavailable before one passes. */
        public final TargetObservations2d verificationFrame;
        /** Accepted distinct post-settle captures, from zero through two. */
        public final int verifiedCaptureCount;
        /** Last sampled occupancy evidence; availability and age still retain their original meaning. */
        public final OccupancyObservation occupancy;
        /** Whether this attempt currently claims the intake request; not proof of applied output. */
        public final boolean commandedIntake;

        private Status(Phase phase, TaskOutcome outcome, String reason, boolean hasFailure,
                       TargetSelectionResult selection, TargetObservations2d verificationFrame,
                       int verifiedCaptureCount, OccupancyObservation occupancy,
                       boolean commandedIntake) {
            this.phase = phase;
            this.outcome = outcome;
            this.reason = reason;
            this.hasFailure = hasFailure;
            this.selection = selection;
            this.verificationFrame = verificationFrame;
            this.verifiedCaptureCount = verifiedCaptureCount;
            this.occupancy = occupancy;
            this.commandedIntake = commandedIntake;
        }
    }

    /** Required rigid tool geometry; inches and counter-clockwise radians. */
    public interface ToolStep {
        /** Answer the tool frame and positive tool-to-target stand-off once. */
        TuningStep throughTool(Pose2d robotToTool, double standOffInches);
    }
    /** Required reviewed controller tuning. */
    public interface TuningStep {
        /** Retain the immutable existing guidance tuning value. */
        VerificationStep driveTuning(DriveGuidancePlan.Tuning tuning);
    }
    /** Required close-window and zero-command verification bounds. */
    public interface VerificationStep {
        /**
         * Configure nonnegative settling seconds and tolerances, and a positive timeout greater
         * than settling. Position tolerance must be less than stand-off; heading tolerance is
         * below pi/2. Two distinct strictly post-settle captures are always required.
         */
        FinalIntakeStep verifyWithZeroCommand(double settleSec, double positionToleranceInches,
                                              double headingToleranceRad, double verificationTimeoutSec);
    }
    /** Required mechanism setter and bounded final straight command. */
    public interface FinalIntakeStep {
        /**
         * Borrow a request setter (true collects, false releases), choose a normalized command in
         * (0,1], and choose positive finite maximum seconds. Direction is the configured tool +X.
         */
        CaptureStep finalIntake(Consumer<Boolean> requestIntake, double finalCommand, double maxFinalSec);
    }
    /** Required independent timestamped occupancy evidence. */
    public interface CaptureStep {
        /** Borrow accepted occupancy observations and choose their inclusive nonnegative age bound. */
        IdleStep captureFeedback(Source<OccupancyObservation> occupancy, double maxCaptureAgeSec);
    }
    /** Required ordinary terminal/idle drive intent. */
    public interface IdleStep {
        /** Borrow manual TeleOp intent or an explicitly zero Auto source, without resetting it. */
        LimitStep idleFrom(DriveSource idle);
    }
    /** Required positive whole-attempt elapsed bound. */
    public interface LimitStep {
        /** Finish construction; the whole bound must contain the configured final/verification bounds. */
        GuidedApproach withinSec(double maxAttemptSec);
    }

    /**
     * Begin the sole construction path without sampling selection. Selection supplies its original
     * observation-age rule; remembered field selections are intentionally not accepted here.
     * Retaining a stage and completing it repeatedly constructs independent owners.
     */
    public static ToolStep cameraOnly(TargetSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        return (tool, standOff) -> {
            // The shared description validates and expands geometry; no second offset equation.
            SpatialApproach2d geometry = SpatialApproach2d.facePoint(
                    References.selectedTargetPoint(selection), tool, standOff);
            return tuning -> {
                Objects.requireNonNull(tuning, "tuning");
                return (settle, positionTol, headingTol, verifyTimeout) -> {
                    nonnegative("settleSec", settle);
                    nonnegative("positionToleranceInches", positionTol);
                    nonnegative("headingToleranceRad", headingTol);
                    positive("verificationTimeoutSec", verifyTimeout);
                    if (!(positionTol < standOff)) throw new IllegalArgumentException(
                            "positionToleranceInches must be less than standOffInches");
                    if (!(headingTol < Math.PI / 2.0)) throw new IllegalArgumentException(
                            "headingToleranceRad must be below pi/2");
                    if (!(verifyTimeout > settle)) throw new IllegalArgumentException(
                            "verificationTimeoutSec must be greater than settleSec");
                    return (request, command, finalSec) -> {
                        Objects.requireNonNull(request, "requestIntake");
                        positive("finalCommand", command);
                        if (command > 1.0) throw new IllegalArgumentException("finalCommand must be <= 1");
                        positive("maxFinalSec", finalSec);
                        return (feedback, age) -> {
                            Objects.requireNonNull(feedback, "occupancy");
                            nonnegative("maxCaptureAgeSec", age);
                            return idle -> {
                                Objects.requireNonNull(idle, "idle");
                                return attemptSec -> {
                                    positive("maxAttemptSec", attemptSec);
                                    if (finalSec > attemptSec || verifyTimeout > attemptSec) {
                                        throw new IllegalArgumentException(
                                                "maxFinalSec and verificationTimeoutSec must fit maxAttemptSec");
                                    }
                                    return new GuidedApproach(selection, geometry, tuning,
                                            settle, positionTol, headingTol, verifyTimeout, request,
                                            command, finalSec, feedback, age, idle, attemptSec);
                                };
                            };
                        };
                    };
                };
            };
        };
    }

    private final TargetSelectionSource selection;
    private final SpatialApproach2d geometry;
    private final Pose2d robotToTool;
    private final double standOff;
    private final DriveGuidancePlan.Tuning tuning;
    private final double settleSec;
    private final double positionTolerance;
    private final double headingTolerance;
    private final double verificationTimeout;
    private final Consumer<Boolean> requestIntake;
    private final DriveSignal finalSignal;
    private final double maxFinalSec;
    private final Source<OccupancyObservation> occupancy;
    private final double maxCaptureAgeSec;
    private final DriveSource idle;
    private final double maxAttemptSec;
    private final DriveSource driveSource = this::readDrive;
    private LoopClock ownerClock;
    private Attempt active;
    private Attempt admitting;
    private Attempt reported;
    private boolean stopped;
    private RuntimeException ownerFailure;
    private boolean failingOwner;
    private boolean readingDrive;
    private long idleCycle = Long.MIN_VALUE;
    private DriveSignal idleValue = DriveSignal.zero();
    private DriveSignal intent = DriveSignal.zero();
    private long intentCycle = Long.MIN_VALUE;
    private Status status = new Status(Phase.IDLE, TaskOutcome.NOT_DONE, "no pickup requested", false,
            TargetSelectionResult.none(TargetObservations2d.unavailable("not sampled"), 0, "not sampled"),
            TargetObservations2d.unavailable("no verified capture"), 0,
            OccupancyObservation.unavailable(), false);

    /** Retain already validated immutable geometry and bounds; never sample borrowed collaborators. */
    private GuidedApproach(TargetSelectionSource selection, SpatialApproach2d geometry,
                           DriveGuidancePlan.Tuning tuning,
                           double settleSec, double positionTolerance, double headingTolerance,
                           double verificationTimeout, Consumer<Boolean> requestIntake,
                           double command, double maxFinalSec, Source<OccupancyObservation> occupancy,
                           double maxCaptureAgeSec, DriveSource idle, double maxAttemptSec) {
        this.selection = selection;
        this.geometry = geometry;
        this.robotToTool = geometry.robotToToolFrame();
        this.standOff = geometry.standOffInches();
        this.tuning = tuning;
        this.settleSec = settleSec;
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
        this.verificationTimeout = verificationTimeout;
        this.requestIntake = requestIntake;
        this.finalSignal = new DriveSignal(command * Math.cos(robotToTool.headingRad),
                command * Math.sin(robotToTool.headingRad), 0.0);
        this.maxFinalSec = maxFinalSec;
        this.occupancy = occupancy;
        this.maxCaptureAgeSec = maxCaptureAgeSec;
        this.idle = idle;
        this.maxAttemptSec = maxAttemptSec;
    }

    /**
     * Create fresh single-use work. Permission is sampled at admission and each active cycle;
     * TeleOp controls must bind it to the original gesture, not a later press of the same button.
     * Construction, cancellation before start, and rejected overlap do not acquire intake intent.
     * An overlapping/stopped request completes CANCELLED without replacing the existing owner's
     * reported attempt. An exceptional owner failure remains exceptional on subsequent starts.
     */
    public Task createPickupTask(BooleanSource permission) {
        return new Attempt(Objects.requireNonNull(permission, "permission"));
    }

    /** Cancel this owner's current/admitting attempt only; never clear an unrelated runner queue. */
    public void cancelPickup() {
        if (active != null) active.cancel();
        else if (admitting != null) admitting.cancel();
    }

    /**
     * Stable final drive source. Sampling does not advance phases. An active attempt without a
     * command published in this cycle produces zero; cancellation is visible in the same cycle.
     * Its inherited reset is inert and never resets borrowed collaborators.
     */
    public DriveSource driveSource() { return driveSource; }

    /**
     * Cached last accepted attempt plus owner STOP/failure; no input or pending outcome sampling.
     * A refused queued request reports through that Task, not by overwriting a newer accepted attempt.
     */
    public Status status() {
        boolean failed = ownerFailure != null || (reported != null && reported.failed());
        boolean pending = reported != null && reported.isComplete() && !reported.settled();
        if (!failed && !pending) return status;
        return new Status(status.phase, TaskOutcome.NOT_DONE, status.reason, failed, status.selection,
                status.verificationFrame, status.verifiedCaptureCount, status.occupancy, status.commandedIntake);
    }

    /** Terminal, idempotent stop, including before START. Physical sink/Plant stop remains their owner. */
    public void stop() {
        if (stopped) return;
        stopped = true;
        withdraw();
        try { cancelPickup(); }
        finally {
            status = new Status(Phase.STOPPED, status.outcome, "pickup owner stopped", ownerFailure != null,
                    status.selection, status.verificationFrame, status.verifiedCaptureCount,
                    status.occupancy, false);
        }
    }

    /** Select current-cycle intent or one cached idle read, with synchronous ending/reentry checks. */
    private DriveSignal readDrive(LoopClock clock) {
        if (stopped || ownerFailure != null) return DriveSignal.zero();
        try {
            requireClock(clock);
            if (readingDrive) throw new IllegalStateException("GuidedApproach drive sampling is reentrant");
            readingDrive = true;
            if (active != null) return active.liveForDrive(clock) && intentCycle == clock.cycle()
                    ? intent : DriveSignal.zero();
            if (admitting != null) return DriveSignal.zero();
            if (idleCycle != clock.cycle()) {
                DriveSignal candidate = Objects.requireNonNull(idle.get(clock), "idle drive signal");
                if (ownerFailure != null) throw ownerFailure;
                if (stopped) return DriveSignal.zero();
                if (active != null) return active.liveForDrive(clock) && intentCycle == clock.cycle()
                        ? intent : DriveSignal.zero();
                if (!finite(candidate.axial) || !finite(candidate.lateral) || !finite(candidate.omega)) {
                    throw new IllegalStateException("GuidedApproach idle drive signal must be finite");
                }
                idleValue = candidate;
                idleCycle = clock.cycle();
            }
            return idleValue;
        } catch (RuntimeException failure) {
            failOwner(failure);
            throw failure;
        } finally {
            readingDrive = false;
        }
    }

    /** Capture one stable clock identity without advancing or resetting it. */
    private void requireClock(LoopClock clock) {
        Objects.requireNonNull(clock, "GuidedApproach requires one stable LoopClock");
        if (ownerClock != null && ownerClock != clock) throw new IllegalArgumentException(
                "GuidedApproach requires one stable LoopClock");
        ownerClock = clock;
    }

    /** Make any previously selected command immediately ineligible, including within the same cycle. */
    private void withdraw() { intent = DriveSignal.zero(); intentCycle = Long.MIN_VALUE; }

    /** Latch terminal owner failure and immediately best-effort end any externally active request. */
    private void failOwner(RuntimeException failure) {
        if (ownerFailure == null) ownerFailure = failure;
        withdraw();
        if (failingOwner) return;
        failingOwner = true;
        try {
            Attempt attempt = active != null ? active : admitting;
            if (attempt != null && !attempt.isComplete()) {
                try { attempt.failExternally(ownerFailure); }
                catch (RuntimeException retained) {
                    if (retained != ownerFailure) ownerFailure.addSuppressed(retained);
                }
            } else if (attempt == null) {
                status = new Status(Phase.STOPPED, TaskOutcome.NOT_DONE,
                        "pickup owner failed: " + ownerFailure.getMessage(), true, status.selection,
                        status.verificationFrame, status.verifiedCaptureCount, status.occupancy,
                        status.commandedIntake);
            }
        } finally {
            failingOwner = false;
        }
    }

    /** Domain phases need retained image and sensor evidence beyond generic Task composition. */
    private final class Attempt extends AbstractTask {
        private final BooleanSource permission;
        private final LoopTimestamp createdAt;
        private boolean claimed;
        private boolean intakeClaimed;
        private Phase phase = Phase.IDLE;
        private TaskOutcome ending = TaskOutcome.NOT_DONE;
        private String reason = "not started";
        private LoopTimestamp startedAt = LoopTimestamp.unavailable();
        private LoopTimestamp phaseStartedAt = LoopTimestamp.unavailable();
        private LoopTimestamp lastFrame = LoopTimestamp.unavailable();
        private LoopTimestamp lastOccupancy = LoopTimestamp.unavailable();
        private boolean lastOccupied;
        private int verified;
        private LoopTimestamp firstVerifiedAt = LoopTimestamp.unavailable();
        private double firstVerifiedMaxAgeSec;
        private TargetObservations2d verifiedFrame = TargetObservations2d.unavailable("no verified capture");
        private TargetSelectionResult selected = TargetSelectionResult.none(
                TargetObservations2d.unavailable("attempt selection not sampled"), 0,
                "attempt selection not sampled");
        private OccupancyObservation feedback = OccupancyObservation.unavailable();
        private DriveGuidanceQuery query;
        private long processedCycle = Long.MIN_VALUE;

        /** Capture one fresh Task identity and any already-known clock epoch, without reading permission. */
        Attempt(BooleanSource permission) {
            super("GuidedApproach camera-only pickup");
            this.permission = permission;
            this.createdAt = ownerClock == null ? LoopTimestamp.unavailable() : ownerClock.nowTimestamp();
        }

        /** Acquire admission before callbacks, then check initial evidence and publish the first guide step. */
        @Override protected void onStart(LoopClock clock) {
            requireClock(clock);
            if (ownerFailure != null) throw ownerFailure;
            if (stopped || active != null || admitting != null
                    || (createdAt.isAvailable() && !finite(createdAt.ageSec(clock)))) {
                ending = TaskOutcome.CANCELLED;
                reason = stopped ? "pickup owner stopped" : active != null || admitting != null
                        ? "another pickup attempt owns admission or active work"
                        : "queued pickup belongs to an earlier clock epoch";
                phase = Phase.DONE;
                complete(TaskOutcome.CANCELLED);
                return;
            }
            admitting = this;
            boolean permitted;
            try {
                permitted = permission.getAsBoolean(clock);
                checkFailure();
                if (ownerFailure != null) throw ownerFailure;
            } finally {
                if (admitting == this) admitting = null;
            }
            if (!isActive()) return;
            if (!permitted || stopped) {
                ending = TaskOutcome.CANCELLED;
                reason = stopped ? "pickup owner stopped during admission" : "pickup permission refused at admission";
                phase = Phase.DONE;
                complete(TaskOutcome.CANCELLED);
                return;
            }
            active = this;
            reported = this;
            claimed = true;
            startedAt = clock.nowTimestamp();
            phaseStartedAt = startedAt;
            phase = Phase.GUIDE;
            withdraw();
            // Each attempt owns a fresh runtime/reference cache; no borrowed reset is needed.
            SpatialApproach2d approach = SpatialApproach2d.facePoint(
                    References.selectedTargetPoint(Source.of(ignored -> selected)), robotToTool, standOff);
            query = DriveGuidance.plan().approach(approach).solveWith()
                    .observedPoints(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT)
                    .driveTuning().use(tuning).doneDriveTuning().build().query();
            processedCycle = clock.cycle();
            if (!readOccupancy(clock)) return;
            guide(clock);
        }

        /** Check permission/deadlines before sensors, then advance exactly this attempt's phase. */
        @Override protected void onUpdate(LoopClock clock) {
            requireClock(clock);
            if (!live()) return;
            if (processedCycle == clock.cycle()) return;
            processedCycle = clock.cycle();
            withdraw();
            boolean permitted = permission.getAsBoolean(clock);
            if (!live()) return;
            if (!permitted) { finish(TaskOutcome.CANCELLED, "pickup permission released"); return; }
            double elapsed = startedAt.ageSec(clock);
            if (!finite(elapsed)) { finish(TaskOutcome.CANCELLED, "clock reset invalidated pickup"); return; }
            if (elapsed >= maxAttemptSec) { finish(TaskOutcome.TIMEOUT, "whole pickup deadline reached"); return; }
            if (phase == Phase.FINAL_INTAKE && phaseStartedAt.ageSec(clock) >= maxFinalSec) {
                finish(TaskOutcome.TIMEOUT, "final intake deadline reached without capture"); return;
            }
            if (phase == Phase.VERIFY && phaseStartedAt.ageSec(clock) >= verificationTimeout) {
                finish(TaskOutcome.TIMEOUT, "zero-command verification deadline reached"); return;
            }
            if (!readOccupancy(clock)) return;
            if (phase == Phase.GUIDE) guide(clock);
            else if (phase == Phase.VERIFY) verify(clock);
            else if (phase == Phase.FINAL_INTAKE) {
                publishIntent(clock, finalSignal);
                publish("bounded final intake; camera visibility is not required");
            }
        }

        /** Validate continuous empty evidence or a strictly new final-phase occupied transition. */
        private boolean readOccupancy(LoopClock clock) {
            OccupancyObservation value = Objects.requireNonNull(occupancy.get(clock), "occupancy observation");
            if (!live()) return false;
            feedback = value;
            if (!value.available || !value.timestamp.isFresh(clock, maxCaptureAgeSec)) {
                finish(TaskOutcome.CANCELLED, "occupancy evidence unavailable, stale, future or reset"); return false;
            }
            if (lastOccupancy.isAvailable()) {
                double step = value.timestamp.secondsSince(lastOccupancy);
                if (!finite(step) || step < 0 || (step == 0 && value.occupied != lastOccupied)) {
                    finish(TaskOutcome.CANCELLED, "occupancy timestamp regressed or changed at the same time"); return false;
                }
            }
            if (value.occupied) {
                if (phase == Phase.FINAL_INTAKE && !lastOccupied
                        && value.timestamp.secondsSince(lastOccupancy) > 0
                        && value.timestamp.secondsSince(phaseStartedAt) > 0) {
                    finish(TaskOutcome.SUCCESS, "new occupancy transition confirmed configured capture");
                } else finish(TaskOutcome.CANCELLED, "occupied before an eligible final-intake capture transition");
                return false;
            }
            lastOccupancy = value.timestamp;
            lastOccupied = value.occupied;
            return true;
        }

        /** Read one borrowed selection and preserve its full frame, capture age and monotonicity. */
        private boolean readSelection(LoopClock clock) {
            TargetSelectionResult value = Objects.requireNonNull(selection.get(clock), "selected target");
            if (!live()) return false;
            selected = value;
            if (!value.isUsable(clock) || !value.frame().isFresh(clock, value.maxAgeSec())) {
                finish(TaskOutcome.CANCELLED, "visible selection unavailable, stale, empty or reset: " + value.reason());
                return false;
            }
            LoopTimestamp stamp = value.frame().timestamp();
            if (lastFrame.isAvailable()) {
                double step = stamp.secondsSince(lastFrame);
                if (!finite(step) || step < 0) {
                    finish(TaskOutcome.CANCELLED, "camera capture timestamp regressed or reset"); return false;
                }
            }
            return true;
        }

        /** Evaluate the shared approach against the already checked selected snapshot, never another poll. */
        private void guide(LoopClock clock) {
            if (!readSelection(clock)) return;
            lastFrame = selected.frame().timestamp();
            DriveGuidanceStatus guided = query.get(clock);
            if (!live()) return;
            if (!guided.hasTranslationError || !guided.hasOmegaError
                    || !finite(guided.translationErrorMagInches()) || !finite(guided.omegaErrorRad)
                    || !finite(guided.signal.axial) || !finite(guided.signal.lateral)
                    || !finite(guided.signal.omega)) {
                finish(TaskOutcome.CANCELLED, "camera-only approach geometry unavailable"); return;
            }
            if (guided.translationWithin(positionTolerance) && guided.omegaWithin(headingTolerance)) {
                phase = Phase.VERIFY;
                phaseStartedAt = clock.nowTimestamp();
                publishIntent(clock, DriveSignal.zero());
                publish("zero commanded; waiting for settling and two distinct fresh captures");
            } else {
                publishIntent(clock, guided.signal);
                publish("guiding from current visible ranking; no physical identity is retained");
            }
        }

        /** Require unique aligned full-frame evidence, then count two fresh strictly post-settle captures. */
        private void verify(LoopClock clock) {
            publishIntent(clock, DriveSignal.zero());
            if (verified > 0 && !firstVerifiedAt.isFresh(clock, firstVerifiedMaxAgeSec)) {
                finish(TaskOutcome.CANCELLED, "first verification capture expired before handoff"); return;
            }
            if (!readSelection(clock)) return;
            TargetObservations2d frame = selected.frame();
            double sinceLast = frame.timestamp().secondsSince(lastFrame);
            double afterEntry = frame.timestamp().secondsSince(phaseStartedAt);
            int close = 0;
            for (TargetObservation2d candidate : frame.observations()) {
                if (!candidate.hasPosition()) {
                    finish(TaskOutcome.CANCELLED, "incomplete frame cannot establish unique close alignment"); return;
                }
                if (insideWindow(candidate)) close++;
            }
            if (close != 1 || !insideWindow(selected.observation())) {
                finish(TaskOutcome.CANCELLED, close > 1 ? "ambiguous close candidates during verification"
                        : "selected target left the close/aligned verification window"); return;
            }
            if (sinceLast == 0) {
                publish("zero commanded; waiting for a distinct strictly post-settle capture");
                return;
            }
            lastFrame = frame.timestamp();
            if (!(afterEntry > settleSec)) {
                publish("zero commanded; qualifying capture is not strictly after settling");
                return;
            }
            if (verified == 0) {
                firstVerifiedAt = frame.timestamp();
                firstVerifiedMaxAgeSec = selected.maxAgeSec();
            }
            verified++;
            verifiedFrame = frame;
            if (verified < 2) {
                publish("one qualifying post-settle capture; waiting for the second");
                return;
            }
            phase = Phase.FINAL_INTAKE;
            phaseStartedAt = clock.nowTimestamp();
            intakeClaimed = true; // Claim ending before a setter can mutate then cancel/throw.
            publish("two captures verified; requesting one bounded final intake");
            requestIntake.accept(true);
            if (!live()) return;
            publishIntent(clock, finalSignal);
            publish("bounded final intake; verified geometry is historical, not continued visibility");
        }

        /** Use the same fixed frame transforms as the shared spatial solve for every frame candidate. */
        private boolean insideWindow(TargetObservation2d candidate) {
            // Consume the shared expansion rather than applying the tool offset a second time.
            Pose2d robotPoint = new Pose2d(candidate.forwardInches, candidate.leftInches, 0);
            Pose2d remaining = geometry.robotToStandOffFrame().inverse().then(robotPoint);
            Pose2d toolPoint = robotToTool.inverse().then(robotPoint);
            double rangeError = Math.hypot(remaining.xInches, remaining.yInches);
            double bearing = Pose2d.wrapToPi(Math.atan2(toolPoint.yInches, toolPoint.xInches));
            return finite(rangeError) && finite(bearing) && rangeError <= positionTolerance
                    && Math.abs(bearing) <= headingTolerance;
        }

        /** Recheck ownership/failure after a borrowed callback before producing another effect. */
        private boolean live() {
            checkFailure();
            if (ownerFailure != null) throw ownerFailure;
            return isActive() && !stopped && active == this;
        }

        /** Test cached lifecycle eligibility only, without advancing phase or polling inputs. */
        private boolean liveForDrive(LoopClock clock) {
            return isActive() && finite(startedAt.ageSec(clock));
        }

        /** Outside-world failures join the same guarded ending without consuming an update cycle. */
        private void failExternally(RuntimeException failure) {
            observe(() -> { throw failure; });
        }

        /** Associate intent with exactly the cycle in which this Task produced it. */
        private void publishIntent(LoopClock clock, DriveSignal command) {
            if (!live()) return;
            intent = command;
            intentCycle = clock.cycle();
        }

        /** Choose a truthful normal ending before invoking the shared all-ending lifecycle. */
        private void finish(TaskOutcome outcome, String explanation) {
            if (!isActive()) return;
            ending = outcome;
            reason = explanation;
            complete(outcome);
        }

        /** Intentional cancellation never invents a capture or a continuation. */
        @Override protected void onCancel() {
            ending = TaskOutcome.CANCELLED;
            reason = "pickup cancelled";
        }

        /** Lifecycle failure is exceptional and latches zero for the owner, not ordinary idle fallback. */
        @Override protected void onFailure(RuntimeException failure) {
            ending = TaskOutcome.NOT_DONE;
            reason = "pickup lifecycle failed: " + failure.getClass().getSimpleName();
            failOwner(failure);
        }

        /** Withdraw before the once-only release; keep ownership until synchronous cleanup returns. */
        @Override protected void onFinish() {
            if (admitting == this) admitting = null;
            if (!claimed) return;
            withdraw();
            phase = Phase.DONE;
            try {
                if (intakeClaimed) {
                    intakeClaimed = false;
                    publish(reason);
                    requestIntake.accept(false);
                }
                checkFailure();
                if (ownerFailure != null) throw ownerFailure;
            } catch (RuntimeException failure) {
                ending = TaskOutcome.NOT_DONE;
                reason = "pickup ending failed; stop the owning robot";
                failOwner(failure);
                throw failure;
            } finally {
                if (active == this) active = null;
                publish(reason);
            }
        }

        /** Publish cached accepted-attempt evidence without consuming a pending Task outcome. */
        private void publish(String explanation) {
            reason = explanation;
            if (reported != this) return;
            status = new Status(stopped ? Phase.STOPPED : phase, ending, reason, ownerFailure != null,
                    selected, verifiedFrame, verified, feedback, intakeClaimed);
        }

        private boolean failed() { return hasFailure(); }
        private boolean settled() { return isEndingSettled(); }

        /** Report refusal/phase facts for this exact Task without replacing the owner's accepted status. */
        @Override protected void debugState(DebugSink sink, String prefix) {
            sink.addData(prefix + ".phase", phase)
                    .addData(prefix + ".reason", reason)
                    .addData(prefix + ".verifiedCaptureCount", verified)
                    .addData(prefix + ".commandedIntake", intakeClaimed);
        }
    }

    private static boolean finite(double value) { return Double.isFinite(value); }
    /** Validate one explicitly nonnegative finite stage answer at construction. */
    private static void nonnegative(String name, double value) {
        if (!finite(value) || value < 0) throw new IllegalArgumentException(name + " must be finite and >= 0");
    }
    /** Validate one explicitly positive finite stage answer at construction. */
    private static void positive(String name, double value) {
        if (!finite(value) || value <= 0) throw new IllegalArgumentException(name + " must be finite and > 0");
    }
}
