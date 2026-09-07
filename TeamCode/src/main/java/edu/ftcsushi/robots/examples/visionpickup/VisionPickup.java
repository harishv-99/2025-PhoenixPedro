package edu.ftcsushi.robots.examples.visionpickup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.spatial.ApproachResult2d;
import edu.ftcsushi.fw.spatial.AxisAlignedBoxRegion2d;
import edu.ftcsushi.fw.spatial.ReferenceFrame2d;
import edu.ftcsushi.fw.spatial.References;
import edu.ftcsushi.fw.spatial.RobotFrameRectangle2d;
import edu.ftcsushi.fw.spatial.SpatialControlFrames;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/**
 * Independent hardware-neutral robot policy for one supervised pickup attempt.
 *
 * <p>This is not a ready-to-run FTC OpMode. Defaults disable automatic motion. An adopting robot
 * must supply measured geometry, reviewed command/travel/contact limits, current localization, and
 * mechanism-owned capture feedback. A downstream program wires {@link #driveSource()} to its one
 * drive sink; the intake callback changes the mechanism's request, never hardware directly.</p>
 *
 * <p>A fresh Task freezes one resting-target staging goal, reobserves a finite neighborhood before
 * final intake, then permits expected vision occlusion only during a bounded straight maneuver.
 * A unique nearby observation is geometric association, not proof of physical identity. Arrival,
 * commanded contact, and new capture feedback are distinct. There is no route safety certificate,
 * obstacle avoidance, force sensing, retry, retreat, or automatic multi-object collection.</p>
 */
public final class VisionPickup implements RobotProgram.Service {

    /** Named field-box sides whose contact may be explicitly permitted by one template. */
    public enum Contact {
        NONE(false, false, false, false),
        MIN_X(true, false, false, false), MAX_X(false, true, false, false),
        MIN_Y(false, false, true, false), MAX_Y(false, false, false, true),
        MIN_X_MIN_Y(true, false, true, false), MIN_X_MAX_Y(true, false, false, true),
        MAX_X_MIN_Y(false, true, true, false), MAX_X_MAX_Y(false, true, false, true);

        private final boolean minX;
        private final boolean maxX;
        private final boolean minY;
        private final boolean maxY;

        Contact(boolean minX, boolean maxX, boolean minY, boolean maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }
    }

    /** One authored finite open-floor, wall, or adjacent-corner approach option. */
    public static final class Template {
        public final String name;
        public final AxisAlignedBoxRegion2d targetRegion;
        public final double robotFieldHeadingRad;
        public final Contact contact;

        /**
         * Creates an immutable template. The region locates eligible target reference points,
         * not traversable space; heading/contact permission belong to this robot's strategy.
         */
        public Template(String name, AxisAlignedBoxRegion2d targetRegion,
                        double robotFieldHeadingRad, Contact contact) {
            this.name = Objects.requireNonNull(name, "template name").trim();
            if (this.name.isEmpty()) throw new IllegalArgumentException("template name must be nonblank");
            this.targetRegion = Objects.requireNonNull(targetRegion, "targetRegion");
            finite("robotFieldHeadingRad", robotFieldHeadingRad);
            this.robotFieldHeadingRad = robotFieldHeadingRad;
            this.contact = Objects.requireNonNull(contact, "contact");
        }
    }

    /** Robot-owned measurements and limits; an incomplete/default graph remains motion-disabled. */
    public static final class Config {
        /** Explicit robot adoption decision; defaults false and never changed by the example. */
        public boolean enableMotion;
        /** Explicit permission for the named wall/corner contact templates, independently of motion. */
        public boolean allowWallContact;
        /** Explicitly allow an unconfirmed UNKNOWN finish when capture feedback is unavailable. */
        public boolean allowUnconfirmedCapture;
        /** Measured rigid robot-to-intake-mouth frame, inches and CCW-positive radians. */
        public Pose2d robotToIntake;
        /** Authored conservative robot/mechanism rectangle for this fixed collection configuration. */
        public RobotFrameRectangle2d robotEnvelope;
        /** Authored field-wall coordinates; does not describe interior obstacles. */
        public AxisAlignedBoxRegion2d fieldInterior;
        /** Finite authored options; first eligible geometrically admissible template wins. */
        public List<Template> templates = Collections.emptyList();
        /** Reviewed staging control gains/caps; absent by default rather than guessed for a robot. */
        public DriveGuidancePlan.Tuning guidanceTuning;
        /** Intake-mouth stand-off from the frozen target at staging, inches. */
        public double stagingStandOffInches;
        /** Required body margin from every wall during staging, inches. */
        public double stagingWallMarginInches;
        /**
         * Maximum nominal final-goal extension beyond specifically permitted wall planes, inches.
         * This bounds a command geometry model, not real wall penetration or contact force.
         */
        public double contactCommandExtensionInches;
        /** Maximum normalized magnitude of the straight final translation, within (0,1]. */
        public double finalTranslateCommand;
        /** Maximum accumulated observed translation over the complete attempt, inches. */
        public double maxAttemptTravelInches;
        /** Maximum accumulated observed translation during final intake, inches. */
        public double maxFinalTravelInches;
        /** Hard whole-attempt elapsed limit in seconds. */
        public double maxAttemptSec;
        /** Hard final-intake elapsed limit in seconds, starting only on entering that phase. */
        public double maxFinalSec;
        /** Maximum wait for a new frame after arrival at staging, seconds. */
        public double recheckTimeoutSec;
        /** Radius of the frozen target's final recheck neighborhood, inches. */
        public double recheckRadiusInches;
        /** Maximum lateral deviation during the final straight maneuver, inches. */
        public double finalCorridorHalfWidthInches;
        /** Translation arrival tolerance at staging, inches. */
        public double arrivalToleranceInches;
        /** Heading arrival/deviation tolerance, radians. */
        public double headingToleranceRad;
        /** Maximum age of target captures used for admission/recheck, seconds. */
        public double maxObservationAgeSec = 0.20;
        /** Maximum age of the current published robot pose, seconds. */
        public double maxPoseAgeSec = 0.10;
        /** Maximum age of a capture-feedback sample, seconds. */
        public double maxCaptureAgeSec = 0.10;

        private Config() { }

        /** Returns a valid software baseline with no physical motion enabled or measurements guessed. */
        public static Config defaults() { return new Config(); }

        private Config snapshot() {
            Config c = new Config();
            c.enableMotion = enableMotion;
            c.allowWallContact = allowWallContact;
            c.allowUnconfirmedCapture = allowUnconfirmedCapture;
            c.robotToIntake = robotToIntake;
            c.robotEnvelope = robotEnvelope;
            c.fieldInterior = fieldInterior;
            c.templates = Collections.unmodifiableList(new ArrayList<>(
                    Objects.requireNonNull(templates, "templates")));
            c.guidanceTuning = guidanceTuning;
            c.stagingStandOffInches = stagingStandOffInches;
            c.stagingWallMarginInches = stagingWallMarginInches;
            c.contactCommandExtensionInches = contactCommandExtensionInches;
            c.finalTranslateCommand = finalTranslateCommand;
            c.maxAttemptTravelInches = maxAttemptTravelInches;
            c.maxFinalTravelInches = maxFinalTravelInches;
            c.maxAttemptSec = maxAttemptSec;
            c.maxFinalSec = maxFinalSec;
            c.recheckTimeoutSec = recheckTimeoutSec;
            c.recheckRadiusInches = recheckRadiusInches;
            c.finalCorridorHalfWidthInches = finalCorridorHalfWidthInches;
            c.arrivalToleranceInches = arrivalToleranceInches;
            c.headingToleranceRad = headingToleranceRad;
            c.maxObservationAgeSec = maxObservationAgeSec;
            c.maxPoseAgeSec = maxPoseAgeSec;
            c.maxCaptureAgeSec = maxCaptureAgeSec;
            positive("maxObservationAgeSec", c.maxObservationAgeSec);
            positive("maxPoseAgeSec", c.maxPoseAgeSec);
            positive("maxCaptureAgeSec", c.maxCaptureAgeSec);
            if (!c.enableMotion) return c;
            requirePose("robotToIntake", c.robotToIntake);
            Objects.requireNonNull(c.robotEnvelope, "configure robotEnvelope before enabling motion");
            Objects.requireNonNull(c.fieldInterior, "configure fieldInterior before enabling motion");
            Objects.requireNonNull(c.guidanceTuning, "configure guidanceTuning before enabling motion");
            if (c.templates.isEmpty() || c.templates.size() > 16) {
                throw new IllegalArgumentException("configure between 1 and 16 finite pickup templates");
            }
            for (Template template : c.templates) Objects.requireNonNull(template, "template");
            positive("stagingStandOffInches", c.stagingStandOffInches);
            positive("stagingWallMarginInches", c.stagingWallMarginInches);
            finite("contactCommandExtensionInches", c.contactCommandExtensionInches);
            if (c.contactCommandExtensionInches < 0) throw new IllegalArgumentException("contact extension must be >= 0");
            positive("finalTranslateCommand", c.finalTranslateCommand);
            if (c.finalTranslateCommand > 1) throw new IllegalArgumentException("finalTranslateCommand must be <= 1");
            positive("maxAttemptTravelInches", c.maxAttemptTravelInches);
            positive("maxFinalTravelInches", c.maxFinalTravelInches);
            positive("maxAttemptSec", c.maxAttemptSec);
            positive("maxFinalSec", c.maxFinalSec);
            positive("recheckTimeoutSec", c.recheckTimeoutSec);
            positive("recheckRadiusInches", c.recheckRadiusInches);
            positive("finalCorridorHalfWidthInches", c.finalCorridorHalfWidthInches);
            positive("arrivalToleranceInches", c.arrivalToleranceInches);
            positive("headingToleranceRad", c.headingToleranceRad);
            if (c.maxFinalTravelInches > c.maxAttemptTravelInches || c.maxFinalSec > c.maxAttemptSec
                    || c.headingToleranceRad >= Math.PI / 2) {
                throw new IllegalArgumentException("final bounds must fit attempt bounds; heading tolerance must be < pi/2");
            }
            interiorWithMargin(c); // Reject a margin that consumes the authored field.
            return c;
        }
    }

    /** Mechanism-owned sensor evidence; unavailable is not an empty intake. */
    public static final class CaptureFeedback {
        public final boolean available;
        public final boolean occupied;
        public final LoopTimestamp timestamp;

        private CaptureFeedback(boolean available, boolean occupied, LoopTimestamp timestamp) {
            this.available = available;
            this.occupied = occupied;
            this.timestamp = Objects.requireNonNull(timestamp, "capture timestamp");
        }

        /** Records the mechanism sensor's accepted occupied/empty observation at its original time. */
        public static CaptureFeedback observed(boolean occupied, LoopTimestamp timestamp) {
            if (!Objects.requireNonNull(timestamp, "timestamp").isAvailable()) {
                throw new IllegalArgumentException("observed capture feedback requires a timestamp");
            }
            return new CaptureFeedback(true, occupied, timestamp);
        }

        /** No usable sensor sample; never means empty or captured. */
        public static CaptureFeedback unavailable() {
            return new CaptureFeedback(false, false, LoopTimestamp.unavailable());
        }
    }

    /** Arrival and final-intake phases remain visible independently of the Task's outcome. */
    public enum Phase { IDLE, STAGING, RECHECK, FINAL_INTAKE, DONE }

    /** Immutable robot status; approach intent is not a safe path or measured contact. */
    public static final class Status {
        public final Phase phase;
        public final TaskOutcome outcome;
        public final String reason;
        public final ApproachResult2d approach;
        public final String templateName;
        public final Contact permittedContact;
        public final double observedTravelInches;

        private Status(Phase phase, TaskOutcome outcome, String reason, ApproachResult2d approach,
                       Template template, double travel) {
            this.phase = phase;
            this.outcome = outcome;
            this.reason = reason;
            this.approach = approach;
            this.templateName = template == null ? "" : template.name;
            this.permittedContact = template == null ? Contact.NONE : template.contact;
            this.observedTravelInches = travel;
        }
    }

    private final Config config;
    private final Source<TargetSelectionResult> selection;
    private final PoseTrajectoryEstimator localization;
    private final Source<CaptureFeedback> captureFeedback;
    private final Consumer<Boolean> requestIntake;
    private final DriveSource manualDrive;
    private final DriveGuidanceQuery aimQuery;
    private final DriveSource finalDrive = this::readDriveIntent;
    private boolean stopped;
    private boolean aimEnabled;
    private boolean aimAvailable;
    private double aimOmega;
    private Attempt active;
    private DriveSignal pickupIntent = DriveSignal.zero();
    private Status status = new Status(Phase.IDLE, TaskOutcome.NOT_DONE, "no pickup requested",
            ApproachResult2d.unavailable("no target committed"), null, 0.0);
    private LoopClock ownerClock;
    private long serviceCycle = Long.MIN_VALUE;
    private RuntimeException serviceFailure;
    private boolean updatingService;
    private boolean terminalCleanup;
    private boolean trajectoryKnown;
    private long observedSegment;
    private LoopTimestamp trajectoryChangedAt = LoopTimestamp.unavailable();

    /**
     * Creates a hardware-neutral policy owner; does not sample dependencies or request motion.
     * Localization is borrowed and must be updated by its actual owner before this service.
     * The selection must consume the live {@code ObservationSources.inField(...)} history path,
     * not a permanently retained pre-rebase field snapshot; cold-start code cannot reconstruct
     * the history of a trajectory change that happened before this owner existed.
     */
    public VisionPickup(Config config, Source<TargetSelectionResult> selection,
                        PoseTrajectoryEstimator localization, Source<CaptureFeedback> captureFeedback,
                        Consumer<Boolean> requestIntake, DriveSource manualDrive) {
        this.config = Objects.requireNonNull(config, "config").snapshot();
        this.selection = Objects.requireNonNull(selection, "selection");
        this.localization = Objects.requireNonNull(localization, "localization");
        this.captureFeedback = Objects.requireNonNull(captureFeedback, "captureFeedback");
        this.requestIntake = Objects.requireNonNull(requestIntake, "requestIntake");
        this.manualDrive = Objects.requireNonNull(manualDrive, "manualDrive");
        aimQuery = this.config.enableMotion ? DriveGuidance.plan()
                .faceTo().point(References.observedPoint(selection))
                .controlFrames(SpatialControlFrames.robotCenter().withFacingFrame(this.config.robotToIntake))
                .solveWith().localizationOnly().localization(localization)
                .maxAgeSec(this.config.maxPoseAgeSec).minQuality(0.0)
                .onLoss(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT).doneLocalizationOnly()
                .driveTuning().use(this.config.guidanceTuning).doneDriveTuning().build().query() : null;
    }

    /** The one downstream drive source; Tasks only change its private selected intent. */
    public DriveSource driveSource() { return finalDrive; }

    /** Enables heading-only assistance; it never takes the driver's translation channels. */
    public void setAimEnabled(boolean enabled) { if (!stopped) aimEnabled = enabled; }

    /** Captured status only: does not resample vision, sensors, localization, or guidance. */
    public Status status() { return status; }

    /**
     * Builds one fresh attempt shared by TeleOp and Auto. The permission is checked at start and
     * on active cycles, so a queued TeleOp request released before start cannot execute later.
     * An Auto client deliberately supplies a constant true source and retains normal cancellation.
     */
    public Task createPickupTask(BooleanSource permission) {
        return new Attempt(Objects.requireNonNull(permission, "permission"));
    }

    /** Cancels only this capability's active transient attempt, not unrelated program work. */
    public void cancelPickup() { if (active != null) active.cancel(); }

    /** Computes cached heading assistance once per cycle; never updates the borrowed localizer. */
    @Override
    public void update(LoopClock clock) {
        requireClock(clock);
        if (stopped) return;
        if (serviceCycle == clock.cycle()) {
            if (updatingService) throw new IllegalStateException("VisionPickup service update is reentrant");
            if (serviceFailure != null) throw serviceFailure;
            return;
        }
        serviceCycle = clock.cycle();
        serviceFailure = null;
        aimOmega = 0.0;
        aimAvailable = false;
        updatingService = true;
        try {
            observeTrajectoryBoundary(clock);
            if (stopped) return;
            if (active != null && localization.trajectorySegmentId() != active.segment) {
                active.finish(TaskOutcome.CANCELLED, "localization trajectory changed");
            }
            if (config.enableMotion && active == null && aimEnabled) {
                PoseEstimate pose = currentPose(clock);
                if (stopped || active != null) return;
                if (pose != null && config.robotEnvelope.fullyInside(interiorWithMargin(config), pose.toPose2d())) {
                    DriveGuidanceStatus aim = aimQuery.get(clock);
                    if (stopped || active != null) return;
                    if (aim.hasOmegaError) {
                        aimOmega = aim.signal.omega;
                        aimAvailable = true;
                    }
                }
            }
        } catch (RuntimeException failure) {
            serviceFailure = failure;
            try { cancelPickup(); } catch (RuntimeException cleanup) { failure.addSuppressed(cleanup); }
            throw failure;
        } finally {
            updatingService = false;
        }
    }

    /** Terminal, idempotent stop; subsequent drive reads are zero and requests cannot restart it. */
    @Override
    public void stop() {
        if (stopped) return;
        stopped = true;
        aimEnabled = false;
        aimOmega = 0.0;
        aimAvailable = false;
        pickupIntent = DriveSignal.zero();
        cancelPickup();
    }

    private DriveSignal readDriveIntent(LoopClock clock) {
        requireClock(clock);
        if (stopped) return DriveSignal.zero();
        if (active != null) return pickupIntent;
        DriveSignal manual = Objects.requireNonNull(manualDrive.get(clock), "manual drive signal");
        if (stopped) return DriveSignal.zero();
        if (active != null) return pickupIntent;
        return aimEnabled && aimAvailable && config.enableMotion
                ? new DriveSignal(manual.axial, manual.lateral, aimOmega) : manual;
    }

    private void observeTrajectoryBoundary(LoopClock clock) {
        long current = localization.trajectorySegmentId();
        if (trajectoryKnown && current != observedSegment) trajectoryChangedAt = clock.nowTimestamp();
        observedSegment = current;
        trajectoryKnown = true;
    }

    private boolean afterKnownTrajectoryChange(LoopTimestamp timestamp) {
        return !trajectoryChangedAt.isAvailable() || timestamp.secondsSince(trajectoryChangedAt) > 0.0;
    }

    private void requireClock(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (ownerClock != null && ownerClock != clock) {
            throw new IllegalArgumentException("VisionPickup requires one stable LoopClock");
        }
        ownerClock = clock;
    }

    private PoseEstimate currentPose(LoopClock clock) {
        long before = localization.trajectorySegmentId();
        PoseEstimate pose = Objects.requireNonNull(localization.getEstimate(), "localization estimate");
        if (before != localization.trajectorySegmentId()) {
            throw new IllegalStateException("localization changed trajectory while its snapshot was read");
        }
        return pose.hasPose && pose.timestamp.isFresh(clock, config.maxPoseAgeSec)
                && finitePose(pose.toPose2d()) ? pose : null;
    }

    /** New robot policy state is needed for capture evidence, recheck association, and contact bounds. */
    private final class Attempt implements Task {
        private final BooleanSource permission;
        private boolean started;
        private boolean complete;
        private boolean updating;
        private boolean intakeClaimed;
        private boolean canConfirm;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        private Phase phase = Phase.IDLE;
        private long lastCycle = Long.MIN_VALUE;
        private long segment;
        private LoopTimestamp startedAt = LoopTimestamp.unavailable();
        private LoopTimestamp phaseStartedAt = LoopTimestamp.unavailable();
        private LoopTimestamp lastCaptureTime = LoopTimestamp.unavailable();
        private boolean lastOccupied;
        private double travel;
        private double finalTravel;
        private Pose2d lastPose;
        private Pose2d finalStartPose;
        private Template template;
        private ApproachResult2d approach = ApproachResult2d.unavailable("no target committed");
        private DriveGuidanceQuery stagingQuery;

        Attempt(BooleanSource permission) { this.permission = permission; }

        @Override
        public void start(LoopClock clock) {
            if (started) throw new IllegalStateException("pickup Tasks are single-use; create a fresh Task");
            started = true;
            requireClock(clock);
            if (active != null || terminalCleanup) {
                complete = true;
                outcome = TaskOutcome.CANCELLED;
                return; // Another attempt retains its own drive/intake requests.
            }
            active = this;
            startedAt = clock.nowTimestamp();
            try {
                if (stopped || !config.enableMotion) { finish(TaskOutcome.CANCELLED, "pickup is physically unconfigured or stopped"); return; }
                observeTrajectoryBoundary(clock);
                if (!isLive()) return;
                segment = localization.trajectorySegmentId();
                if (!isLive()) return;
                boolean permitted = permission.getAsBoolean(clock);
                if (!isLive()) return;
                if (!permitted) { finish(TaskOutcome.CANCELLED, "pickup permission released"); return; }
                PoseEstimate pose = currentPose(clock);
                if (!isLive()) return;
                if (pose == null) { finish(TaskOutcome.CANCELLED, "current localization unavailable"); return; }
                TargetSelectionResult selected = Objects.requireNonNull(selection.get(clock), "selection");
                if (!isLive()) return;
                if (!selected.isUsable(clock) || !selected.observation().hasFieldPosition()
                        || !selected.observation().isFresh(clock, config.maxObservationAgeSec)
                        || !afterKnownTrajectoryChange(selected.observation().timestamp)) {
                    finish(TaskOutcome.CANCELLED, "fresh field target unavailable"); return;
                }
                CaptureFeedback feedback = Objects.requireNonNull(captureFeedback.get(clock), "capture feedback");
                if (!isLive()) return;
                canConfirm = feedback.available && feedback.timestamp.isFresh(clock, config.maxCaptureAgeSec);
                if (canConfirm && feedback.occupied) { finish(TaskOutcome.CANCELLED, "intake already occupied before attempt"); return; }
                if (!canConfirm && !config.allowUnconfirmedCapture) { finish(TaskOutcome.CANCELLED, "capture feedback unavailable"); return; }
                if (canConfirm) lastCaptureTime = feedback.timestamp;
                lastOccupied = false;
                lastPose = pose.toPose2d();
                if (!config.robotEnvelope.fullyInside(interiorWithMargin(config), lastPose)) {
                    finish(TaskOutcome.CANCELLED, "starting envelope violates staging wall margin"); return;
                }
                chooseTemplate(selected.observation(), clock);
                if (template == null) { finish(TaskOutcome.CANCELLED, "no authored admissible approach template"); return; }
                ReferenceFrame2d goal = References.approachFrame(Source.of(ignored -> approach));
                stagingQuery = DriveGuidance.plan().translateTo().point(References.framePoint(goal))
                        .andFaceTo().frameHeading(goal).solveWith().localizationOnly()
                        .localization(localization).maxAgeSec(config.maxPoseAgeSec).minQuality(0.0)
                        .onLoss(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT).doneLocalizationOnly()
                        .driveTuning().use(config.guidanceTuning).doneDriveTuning().build().query();
                phase = Phase.STAGING;
                phaseStartedAt = clock.nowTimestamp();
                publish("approaching one frozen resting-target goal; path safety is not established");
                update(clock);
            } catch (RuntimeException failure) {
                failWithCleanup(failure);
                throw failure;
            }
        }

        private void chooseTemplate(TargetObservation2d target, LoopClock clock) {
            for (Template candidate : config.templates) {
                if (!candidate.targetRegion.contains(target.fieldXInches, target.fieldYInches)
                        || (candidate.contact != Contact.NONE && !config.allowWallContact)) continue;
                ApproachResult2d proposed = ApproachResult2d.forTarget(target, config.robotToIntake,
                        config.stagingStandOffInches, candidate.robotFieldHeadingRad,
                        config.maxObservationAgeSec);
                if (!proposed.isUsable(clock)) continue;
                Pose2d staging = proposed.fieldToRobotGoalPose();
                Pose2d nominalEnd = staging.then(new Pose2d(
                        config.maxFinalTravelInches * Math.cos(config.robotToIntake.headingRad),
                        config.maxFinalTravelInches * Math.sin(config.robotToIntake.headingRad), 0));
                if (!config.robotEnvelope.fullyInside(interiorWithMargin(config), staging)
                        || !config.robotEnvelope.fullyInside(finalBounds(config, candidate.contact), nominalEnd)) continue;
                template = candidate;
                approach = proposed.committedFor(clock, config.maxAttemptSec);
                return;
            }
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) throw new IllegalStateException("start pickup Task before update");
            requireClock(clock);
            if (complete) return;
            if (updating) throw new IllegalStateException("pickup Task update is reentrant");
            if (lastCycle == clock.cycle()) return;
            lastCycle = clock.cycle();
            updating = true;
            try {
                boolean permitted = permission.getAsBoolean(clock);
                if (!isLive()) return;
                if (!permitted) { finish(TaskOutcome.CANCELLED, "pickup permission released"); return; }
                double elapsed = startedAt.ageSec(clock);
                if (Double.isFinite(elapsed) && elapsed >= config.maxAttemptSec) {
                    finish(TaskOutcome.TIMEOUT, "whole pickup attempt timed out"); return;
                }
                long currentSegment = localization.trajectorySegmentId();
                if (!isLive()) return;
                if (!Double.isFinite(elapsed) || currentSegment != segment
                        || !approach.isUsable(clock)) {
                    finish(TaskOutcome.CANCELLED, "committed target expired or localization trajectory changed"); return;
                }
                PoseEstimate estimate = currentPose(clock);
                if (!isLive()) return;
                if (estimate == null) { finish(TaskOutcome.CANCELLED, "current localization unavailable"); return; }
                currentSegment = localization.trajectorySegmentId();
                if (!isLive()) return;
                if (currentSegment != segment) {
                    finish(TaskOutcome.CANCELLED, "localization trajectory changed"); return;
                }
                Pose2d pose = estimate.toPose2d();
                double step = Math.hypot(pose.xInches - lastPose.xInches, pose.yInches - lastPose.yInches);
                travel += step;
                if (phase == Phase.FINAL_INTAKE) finalTravel += step;
                lastPose = pose;
                if (!Double.isFinite(travel) || travel >= config.maxAttemptTravelInches) {
                    finish(TaskOutcome.CANCELLED, "attempt travel limit reached"); return;
                }
                AxisAlignedBoxRegion2d allowed = phase == Phase.FINAL_INTAKE
                        ? finalBounds(config, template.contact) : interiorWithMargin(config);
                if (!config.robotEnvelope.fullyInside(allowed, pose)) {
                    finish(TaskOutcome.CANCELLED, "observed envelope left the configured region"); return;
                }
                // Stop/permission, coordinate validity, command-region and elapsed/travel limits
                // win over a sensor transition first sampled at or beyond an exhausted bound.
                if (phase == Phase.FINAL_INTAKE
                        && (!finalMotionWithinBounds(pose) || finalBoundReached(clock))) return;
                if (!observeCapture(clock)) return;
                if (!isLive()) return;
                if (phase == Phase.STAGING) stage(clock);
                else if (phase == Phase.RECHECK) recheck(clock, pose);
                else if (phase == Phase.FINAL_INTAKE) finalIntake(clock, pose);
            } catch (RuntimeException failure) {
                failWithCleanup(failure);
                throw failure;
            } finally {
                updating = false;
            }
        }

        private void stage(LoopClock clock) {
            DriveGuidanceStatus guidance = stagingQuery.get(clock);
            if (!isLive()) return;
            if (!guidance.hasTranslationError || !guidance.hasOmegaError) {
                finish(TaskOutcome.CANCELLED, "staging guidance unavailable"); return;
            }
            pickupIntent = guidance.signal;
            if (guidance.translationWithin(config.arrivalToleranceInches)
                    && guidance.omegaWithin(config.headingToleranceRad)) {
                pickupIntent = DriveSignal.zero();
                phase = Phase.RECHECK;
                phaseStartedAt = clock.nowTimestamp();
                publish("approach arrived; waiting for a new unambiguous observation, not capture");
            } else publish("staging; computed guidance does not prove path clearance");
        }

        private void recheck(LoopClock clock, Pose2d pose) {
            if (phaseStartedAt.ageSec(clock) >= config.recheckTimeoutSec) {
                finish(TaskOutcome.TIMEOUT, "no qualifying fresh recheck before deadline"); return;
            }
            TargetSelectionResult reobserved = Objects.requireNonNull(selection.get(clock), "selection");
            if (!isLive()) return;
            TargetObservations2d frame = reobserved.frame();
            if (!frame.isFresh(clock, Math.min(config.maxObservationAgeSec, reobserved.maxAgeSec()))
                    || !(frame.timestamp().secondsSince(phaseStartedAt) > 0.0)
                    || !afterKnownTrajectoryChange(frame.timestamp())) return;
            int nearby = 0;
            TargetObservation2d soleNearby = null;
            TargetObservation2d frozen = approach.observation();
            for (TargetObservation2d candidate : frame.observations()) {
                if (!candidate.hasFieldPosition()) {
                    finish(TaskOutcome.CANCELLED, "recheck candidate has no field position"); return;
                }
                if (Math.hypot(candidate.fieldXInches - frozen.fieldXInches,
                        candidate.fieldYInches - frozen.fieldYInches) <= config.recheckRadiusInches) {
                    nearby++;
                    soleNearby = candidate;
                }
            }
            if (nearby != 1) {
                finish(TaskOutcome.CANCELLED, nearby == 0 ? "target lost at final recheck"
                        : "ambiguous candidates at final recheck"); return;
            }
            if (!template.targetRegion.contains(soleNearby.fieldXInches, soleNearby.fieldYInches)) {
                finish(TaskOutcome.CANCELLED, "unique nearby candidate left the template region"); return;
            }
            Pose2d stagingGoal = approach.fieldToRobotGoalPose();
            if (Math.hypot(pose.xInches - stagingGoal.xInches, pose.yInches - stagingGoal.yInches)
                    > config.arrivalToleranceInches
                    || Math.abs(Pose2d.wrapToPi(pose.headingRad - template.robotFieldHeadingRad))
                    > config.headingToleranceRad) {
                finish(TaskOutcome.CANCELLED, "robot left staging tolerance during recheck"); return;
            }
            Pose2d actualNominalEnd = pose.then(new Pose2d(
                    config.maxFinalTravelInches * Math.cos(config.robotToIntake.headingRad),
                    config.maxFinalTravelInches * Math.sin(config.robotToIntake.headingRad), 0.0));
            if (!config.robotEnvelope.fullyInside(finalBounds(config, template.contact), actualNominalEnd)) {
                finish(TaskOutcome.CANCELLED, "final command endpoint violates configured contact bounds"); return;
            }
            phase = Phase.FINAL_INTAKE;
            phaseStartedAt = clock.nowTimestamp();
            finalStartPose = pose;
            finalTravel = 0;
            intakeClaimed = true; // Claim cleanup before a request callback can mutate then throw.
            requestIntake.accept(true);
            if (!isLive()) return;
            pickupIntent = finalSignal();
            publish("bounded final intake; camera occlusion allowed, contact/capture not inferred");
        }

        private void finalIntake(LoopClock clock, Pose2d pose) {
            requestIntake.accept(true);
            if (!isLive()) return;
            pickupIntent = finalSignal();
            publish("bounded final intake; vision is not reacquired and no retry is started");
        }

        private boolean finalMotionWithinBounds(Pose2d pose) {
            double headingError = Math.abs(Pose2d.wrapToPi(pose.headingRad - template.robotFieldHeadingRad));
            double heading = finalStartPose.headingRad + config.robotToIntake.headingRad;
            double dx = pose.xInches - finalStartPose.xInches;
            double dy = pose.yInches - finalStartPose.yInches;
            double lateral = -Math.sin(heading) * dx + Math.cos(heading) * dy;
            if (headingError > config.headingToleranceRad
                    || Math.abs(lateral) > config.finalCorridorHalfWidthInches) {
                finish(TaskOutcome.CANCELLED, "final heading/corridor bound exceeded"); return false;
            }
            return true;
        }

        private DriveSignal finalSignal() {
            return new DriveSignal(config.finalTranslateCommand * Math.cos(config.robotToIntake.headingRad),
                    config.finalTranslateCommand * Math.sin(config.robotToIntake.headingRad), 0.0);
        }

        private boolean finalBoundReached(LoopClock clock) {
            boolean distance = finalTravel >= config.maxFinalTravelInches;
            boolean time = phaseStartedAt.ageSec(clock) >= config.maxFinalSec;
            if (!distance && !time) return false;
            finish(!canConfirm ? TaskOutcome.UNKNOWN
                            : distance ? TaskOutcome.CANCELLED : TaskOutcome.TIMEOUT,
                    !canConfirm ? "bounded intake ended unconfirmed; feedback unavailable"
                            : distance ? "final travel limit reached without capture confirmation"
                            : "final intake timed out without capture confirmation");
            return true;
        }

        private boolean observeCapture(LoopClock clock) {
            CaptureFeedback feedback = Objects.requireNonNull(captureFeedback.get(clock), "capture feedback");
            if (!isLive()) return false;
            if (!feedback.available || !feedback.timestamp.isFresh(clock, config.maxCaptureAgeSec)) {
                canConfirm = false;
                if (!config.allowUnconfirmedCapture) {
                    finish(TaskOutcome.CANCELLED, "capture feedback became unavailable"); return false;
                }
                return true;
            }
            if (!canConfirm) return true; // Missing initial/continuous evidence cannot invent a transition.
            double sinceLast = feedback.timestamp.secondsSince(lastCaptureTime);
            if (!Double.isFinite(sinceLast) || sinceLast < 0.0
                    || (sinceLast == 0.0 && feedback.occupied != lastOccupied)) {
                finish(TaskOutcome.CANCELLED, "capture feedback regressed or changed at the same timestamp"); return false;
            }
            if (feedback.occupied && !lastOccupied) {
                if (phase == Phase.FINAL_INTAKE && feedback.timestamp.secondsSince(startedAt) > 0.0
                        && feedback.timestamp.secondsSince(phaseStartedAt) > 0.0) {
                    finish(TaskOutcome.SUCCESS, "new capture confirmed by configured intake feedback");
                } else finish(TaskOutcome.CANCELLED, "intake became occupied before final intake");
                return false;
            }
            lastCaptureTime = feedback.timestamp;
            lastOccupied = feedback.occupied;
            return true;
        }

        private void failWithCleanup(RuntimeException failure) {
            try { finish(TaskOutcome.CANCELLED, "pickup failed: " + failure.getClass().getSimpleName()); }
            catch (RuntimeException cleanup) { if (cleanup != failure) failure.addSuppressed(cleanup); }
        }

        private boolean isLive() {
            return !complete && !stopped && active == this;
        }

        private void finish(TaskOutcome ending, String reason) {
            if (complete) return;
            complete = true;
            outcome = ending;
            phase = Phase.DONE;
            if (active == this) {
                pickupIntent = DriveSignal.zero();
                aimOmega = 0;
                aimAvailable = false;
                active = null;
                publish(reason);
            }
            if (intakeClaimed) {
                intakeClaimed = false;
                terminalCleanup = true;
                try { requestIntake.accept(false); }
                catch (RuntimeException failure) {
                    stopped = true;
                    aimEnabled = false;
                    aimAvailable = false;
                    pickupIntent = DriveSignal.zero();
                    outcome = TaskOutcome.CANCELLED;
                    publish("intake cleanup failed; stop the owning robot");
                    throw failure;
                } finally {
                    terminalCleanup = false;
                }
            }
        }

        private void publish(String reason) {
            status = new Status(phase, outcome, reason, approach, template, travel);
        }

        @Override public void cancel() { if (started && !complete) finish(TaskOutcome.CANCELLED, "pickup cancelled"); }
        @Override public boolean isComplete() { return complete; }
        @Override public TaskOutcome getOutcome() { return outcome; }
    }

    private static AxisAlignedBoxRegion2d interiorWithMargin(Config c) {
        double minX = c.fieldInterior.minXInches + c.stagingWallMarginInches;
        double maxX = c.fieldInterior.maxXInches - c.stagingWallMarginInches;
        double minY = c.fieldInterior.minYInches + c.stagingWallMarginInches;
        double maxY = c.fieldInterior.maxYInches - c.stagingWallMarginInches;
        if (!(minX < maxX && minY < maxY)) throw new IllegalArgumentException("staging wall margin consumes field interior");
        return new AxisAlignedBoxRegion2d(minX, maxX, minY, maxY);
    }

    private static AxisAlignedBoxRegion2d finalBounds(Config c, Contact contact) {
        double extension = c.allowWallContact ? c.contactCommandExtensionInches : 0;
        return new AxisAlignedBoxRegion2d(
                c.fieldInterior.minXInches + (contact.minX ? -extension : c.stagingWallMarginInches),
                c.fieldInterior.maxXInches + (contact.maxX ? extension : -c.stagingWallMarginInches),
                c.fieldInterior.minYInches + (contact.minY ? -extension : c.stagingWallMarginInches),
                c.fieldInterior.maxYInches + (contact.maxY ? extension : -c.stagingWallMarginInches));
    }

    private static void finite(String name, double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
    }
    private static void positive(String name, double value) {
        if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException(name + " must be finite and > 0");
    }
    private static boolean finitePose(Pose2d pose) {
        return pose != null && Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.headingRad);
    }
    private static void requirePose(String name, Pose2d pose) {
        if (!finitePose(pose)) throw new IllegalArgumentException(name + " requires three finite pose values");
    }
}
