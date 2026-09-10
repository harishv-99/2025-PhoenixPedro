package edu.ftcsushi.robots.examples.reference.capability.launcher;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.ScalarTasks;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.AbstractTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheelMechanism;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheels;
import edu.ftcsushi.robots.examples.reference.capability.inventory.ReferenceInventoryStatusService;

/**
 * Owns one software-only feed policy, paired wheels, inventory, and exact release/transfer Plants.
 *
 * <p>Register only this owner as an output. Its private inventory child is sampled after the Plants,
 * once per output cycle; Tasks consume the preceding complete publication. Neither Tasks nor status
 * accessors poll hardware. This fixture has no internal queue and no physical recovery routine.</p>
 */
public final class ReferenceLauncherMechanism implements ReferenceLauncher, RobotProgram.Output {
    /** Data-only illustrative wiring and policy; all retained values are defensively copied. */
    public static final class Config {
        public ReferenceFlywheelMechanism.Config flywheels;
        public ReferenceInventoryStatusService.Config inventory;
        public String transferName;
        public Direction transferDirection;
        public String releaseServoName;
        public Direction releaseServoDirection;
        public double feedVelocityTicksPerSec;
        /** Bounds all prerequisite waiting, including staging and paired settling. */
        public double spinUpTimeoutSec;
        /** Minimum span of advancing ready samples, not continuous physical dwell. */
        public double readySettlingSec;
        /** Inclusive maximum sample age and gap between eligible observations, in seconds. */
        public double evidenceMaxAgeSec;
        /** Total bound from release request through transfer and departure confirmation. */
        public double departureTimeoutSec;
        public double transferPower;
        public double transferDurationSec;
        /** Native FTC Servo endpoint for normalized retracted request 0; requires physical review. */
        public double releaseRetractedNativePosition;
        /** Native FTC Servo endpoint for normalized extended request 1; requires physical review. */
        public double releaseExtendedNativePosition;
        public double releaseDurationSec;

        private Config() { }

        /** Returns a compiling software fixture, never permission to operate an assembled shooter. */
        public static Config defaults() {
            Config c = new Config();
            c.flywheels = ReferenceFlywheelMechanism.Config.defaults();
            c.inventory = ReferenceInventoryStatusService.Config.defaults();
            c.transferName = "transfer";
            c.transferDirection = Direction.FORWARD;
            c.releaseServoName = "release";
            c.releaseServoDirection = Direction.FORWARD;
            c.feedVelocityTicksPerSec = 3000.0;
            c.spinUpTimeoutSec = 2.0;
            c.readySettlingSec = 0.10;
            c.evidenceMaxAgeSec = 0.10;
            c.departureTimeoutSec = 0.50;
            c.transferPower = 0.25;
            c.transferDurationSec = 0.20;
            c.releaseRetractedNativePosition = 0.25;
            c.releaseExtendedNativePosition = 0.60;
            c.releaseDurationSec = 0.15;
            return c;
        }
    }

    private final Config config;
    private final ReferenceFlywheelMechanism flywheels;
    private final ReferenceInventoryStatusService inventory;
    private final Plant transfer;
    private final Plant release;
    private FeedTask active;
    private long generation;
    private long epochGeneration;
    private LoopClock ownerClock;
    private LoopTimestamp epochAnchor = LoopTimestamp.unavailable();
    private boolean inventoryStarted;
    private boolean stopped;
    private boolean updating;
    private long attemptedOutputCycle = -1;
    private RuntimeException outputFailure;
    private LoopTimestamp sampledAt = LoopTimestamp.unavailable();
    private long sampleCycle = -1;
    private boolean transferActive;
    private boolean idlePublished;
    private long idleRequestId = -1;
    private boolean recoveryRequired;
    private long recoveryAfterCycle = -1;
    private Phase lastPhase = Phase.IDLE;
    private Reason lastReason = Reason.NONE;
    private Status lastStatus;

    /**
     * Constructs one complete privately owned graph. Validation precedes actuator construction;
     * partial actuator construction failure best-effort stops every already acquired owner.
     */
    public ReferenceLauncherMechanism(HardwareMap hardwareMap, Config source) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        config = copyAndValidate(source);
        inventory = new ReferenceInventoryStatusService(map, config.inventory);
        ReferenceFlywheelMechanism builtFlywheels = null;
        Plant builtTransfer = null;
        Plant builtRelease = null;
        try {
            builtFlywheels = new ReferenceFlywheelMechanism(map, config.flywheels);
            builtTransfer = FtcActuators.plant(map)
                    .crServo(config.transferName, config.transferDirection)
                    .power()
                    .targetFromNewCommand(0.0)
                    .build();
            builtRelease = FtcActuators.plant(map)
                    .servo(config.releaseServoName, config.releaseServoDirection)
                    .position()
                    .nonPeriodic()
                    .bounded(0.0, 1.0)
                    .rangeMapsToNative(config.releaseRetractedNativePosition,
                            config.releaseExtendedNativePosition)
                    .targetFromNewCommand(0.0)
                    .build();
        } catch (RuntimeException failure) {
            ReferenceFlywheelMechanism f = builtFlywheels;
            Plant t = builtTransfer;
            Plant r = builtRelease;
            throw CleanupActions.attemptAllAfterFailure(failure,
                    () -> { if (r != null) r.stop(); },
                    () -> { if (t != null) t.stop(); },
                    () -> { if (f != null) f.stop(); }, inventory::stop);
        }
        flywheels = builtFlywheels;
        transfer = builtTransfer;
        release = builtRelease;
        publishStatus();
    }

    /** {@inheritDoc} */
    @Override public ReferenceFlywheels flywheels() { return flywheels; }

    /** {@inheritDoc} */
    @Override public Task feedOne() { return new FeedTask(generation); }

    /** {@inheritDoc} */
    @Override public Status status() { return lastStatus; }

    /** {@inheritDoc} */
    @Override public void abortFeedAttempts() {
        if (stopped) return;
        invalidate(Reason.ABORTED);
    }

    /** {@inheritDoc} */
    @Override public RecoveryResult acknowledgeRecovery() {
        if (stopped) return RecoveryResult.STOPPED;
        if (active != null) return RecoveryResult.ATTEMPT_ACTIVE;
        if (!recoveryRequired) return RecoveryResult.NOT_REQUIRED;
        if (ownerClock == null || !sampledAt.isFresh(ownerClock, config.evidenceMaxAgeSec)
                || sampleCycle <= recoveryAfterCycle || !idlePublished
                || flywheels.requestId() != idleRequestId) {
            return RecoveryResult.WAITING_FOR_IDLE;
        }
        ReferenceInventoryStatusService.Status observed = inventory.status();
        if (!observed.observed || !observed.sampledAt.isFresh(ownerClock, config.evidenceMaxAgeSec)
                || observed.sampleCycle <= recoveryAfterCycle
                || observed.orderIssue != ReferenceInventoryStatusService.OrderIssue.NONE) {
            return RecoveryResult.INVENTORY_UNAVAILABLE;
        }
        recoveryRequired = false;
        generation++; // Acknowledgement does not revive old pending work.
        publishStatus();
        return RecoveryResult.ACKNOWLEDGED;
    }

    /**
     * Realizes requests, then publishes one complete software observation. Duplicate successful
     * calls are inert; a failed effect is retained and never retried. Reentrant update is rejected.
     * STOP during an output prevents subsequent ordinary effects.
     */
    @Override public void update(LoopClock clock) {
        if (outputFailure != null) throw outputFailure;
        if (stopped) return;
        Objects.requireNonNull(clock, "Reference launcher requires its shared LoopClock");
        if (updating) {
            outputFailure = new IllegalStateException("Reference launcher output cannot reenter update");
            throw outputFailure;
        }
        if (ownerClock != null && ownerClock != clock) {
            throw new IllegalArgumentException("Reference launcher requires one stable LoopClock");
        }
        if (attemptedOutputCycle == clock.cycle()) return;
        attemptedOutputCycle = clock.cycle();
        updating = true;
        try {
            ensureEpoch(clock);
            flywheels.update(clock);
            checkOutputCycle(clock);
            if (stopped) return;
            transfer.update(clock);
            checkOutputCycle(clock);
            if (stopped) return;
            release.update(clock);
            checkOutputCycle(clock);
            if (stopped) return;
            if (!inventoryStarted) {
                inventoryStarted = true;
                inventory.start(clock);
            } else {
                inventory.update(clock);
            }
            checkOutputCycle(clock);
            if (stopped) return;
            sampledAt = clock.nowTimestamp();
            sampleCycle = clock.cycle();
            transferActive = transfer.snapshot().appliedTarget() != 0.0;
            idlePublished = !transferActive && release.snapshot().appliedTarget() == 0.0
                    && flywheels.status().appliedVelocityTicksPerSec() == 0.0
                    && flywheels.status().requestedVelocityTicksPerSec() == 0.0;
            idleRequestId = flywheels.status().requestId();
            if (active != null && active.feeding && active.releaseRealizedCycle < 0
                    && release.snapshot().appliedTarget() == 1.0) {
                active.releaseRealizedCycle = sampleCycle;
            }
            publishStatus();
        } catch (RuntimeException failure) {
            if (outputFailure == null) outputFailure = failure;
            FeedTask failedAttempt = active;
            throw CleanupActions.attemptAllAfterFailure(outputFailure,
                    () -> { if (failedAttempt != null) failedAttempt.failFromOutput(outputFailure); },
                    this::stop);
        } finally {
            updating = false;
        }
    }

    /** Fail closed even if a hardware callback caught a forbidden recursive call or clock change. */
    private void checkOutputCycle(LoopClock clock) {
        if (outputFailure != null) throw outputFailure;
        if (clock.cycle() != attemptedOutputCycle) {
            throw new IllegalStateException("Do not advance the LoopClock during launcher output");
        }
    }

    /** Terminally stops every owned resource even if cancellation or one stop fails. */
    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        generation++;
        FeedTask ending = active;
        try {
            CleanupActions.attemptAll(
                    () -> { if (ending != null) ending.abort(Reason.STOPPED); },
                    release::stop, transfer::stop, flywheels::stop, inventory::stop);
        } finally {
            sampledAt = LoopTimestamp.unavailable();
            sampleCycle = -1;
            transferActive = false;
            idlePublished = false;
            publishStatus();
        }
    }

    /** Reinitialize only a changed epoch; initial START never invalidates a just-started Auto Task. */
    private void ensureEpoch(LoopClock clock) {
        if (ownerClock != null && ownerClock != clock) {
            throw new IllegalArgumentException("Reference launcher requires one stable LoopClock");
        }
        ownerClock = clock;
        if (epochAnchor.isAvailable() && !Double.isFinite(epochAnchor.ageSec(clock))) {
            invalidate(Reason.CLOCK_RESET);
            inventoryStarted = false;
            sampledAt = LoopTimestamp.unavailable();
            sampleCycle = -1;
            idlePublished = false;
        }
        epochAnchor = clock.nowTimestamp();
    }

    /** Invalidate old task construction before cancelling the current owner and requesting idle. */
    private void invalidate(Reason reason) {
        if (reason == Reason.CLOCK_RESET) {
            epochGeneration++;
        } else {
            generation++;
        }
        FeedTask ending = active;
        try {
            if (ending != null) {
                ending.abort(reason); // Its onFinish already attempts every idle request once.
            } else {
                requestIdle();
            }
        } finally {
            publishStatus();
        }
    }

    /** These are persistent requests; only the ordinary Plant output phase realizes them. */
    private void requestIdle() {
        if (stopped) return;
        CleanupActions.attemptAll(
                () -> transfer.commandTarget().set(0.0),
                () -> release.commandTarget().set(0.0),
                () -> flywheels.setVelocityTicksPerSec(0.0));
    }

    /** Policy changes reuse the existing observation time; they cannot refresh evidence. */
    private void publishStatus() {
        lastStatus = new Status(flywheels.status(), inventory.status(),
                active == null ? lastPhase : active.phase,
                active == null ? lastReason : active.reason,
                active != null, recoveryRequired, transferActive, sampledAt, sampleCycle);
    }

    /** Composite sampled-evidence decisions, with lifecycle/failure mechanics inherited once. */
    private final class FeedTask extends AbstractTask {
        private final long createdGeneration;
        private final long createdEpochGeneration;
        private final LoopTimestamp createdAt;
        private Task phaseTask;
        private Phase phase = Phase.SETTLING;
        private Reason reason = Reason.NONE;
        private LoopTimestamp startedAt = LoopTimestamp.unavailable();
        private LoopTimestamp feedStartedAt = LoopTimestamp.unavailable();
        private LoopTimestamp settledSince = LoopTimestamp.unavailable();
        private LoopTimestamp previousReadyAt = LoopTimestamp.unavailable();
        private LoopTimestamp previousFeedSampleAt = LoopTimestamp.unavailable();
        private long requestId = -1;
        private long previousSampleCycle = -1;
        private long releaseRealizedCycle = -1;
        private long armedOccupiedCycle = -1;
        private boolean departureObserved;
        private boolean feeding;

        private FeedTask(long createdGeneration) {
            super("referenceFeedOne");
            this.createdGeneration = createdGeneration;
            createdEpochGeneration = epochGeneration;
            createdAt = ownerClock == null ? LoopTimestamp.unavailable() : ownerClock.nowTimestamp();
        }

        /** Claim only after all non-effectful admission checks; construction itself reserves nothing. */
        @Override protected void onStart(LoopClock clock) {
            if (stopped) { end(Reason.STOPPED, TaskOutcome.CANCELLED); return; }
            ensureEpoch(clock);
            // Reset invalidates old observations, not tasks already constructed in the new epoch.
            // Before initial START there is no clock capture; only a later observed reset rejects it.
            boolean currentCreation = createdAt.isAvailable()
                    ? Double.isFinite(createdAt.ageSec(clock))
                    : createdEpochGeneration == epochGeneration;
            if (createdGeneration != generation || !currentCreation) {
                end(Reason.INVALIDATED, TaskOutcome.CANCELLED); return;
            }
            if (active != null) { end(Reason.BUSY, TaskOutcome.CANCELLED); return; }
            if (recoveryRequired) { end(Reason.RECOVERY_REQUIRED, TaskOutcome.CANCELLED); return; }
            active = this;
            startedAt = clock.nowTimestamp();
            flywheels.setVelocityTicksPerSec(config.feedVelocityTicksPerSec);
            requestId = flywheels.requestId();
            publishStatus();
        }

        /** Consume cached facts only; all phase timing uses the shared clock's own start boundary. */
        @Override protected void onUpdate(LoopClock clock) {
            if (ownerClock != clock) {
                throw new IllegalArgumentException("Reference feed requires its owner's LoopClock");
            }
            if (!Double.isFinite(startedAt.ageSec(clock))) {
                end(Reason.CLOCK_RESET, TaskOutcome.CANCELLED); return;
            }
            if (createdGeneration != generation || active != this || stopped) {
                end(Reason.INVALIDATED, TaskOutcome.CANCELLED); return;
            }
            if (!feeding) {
                settle(clock);
            } else {
                feed(clock);
            }
        }

        /** Require advancing observations spanning the dwell, resetting on any broken prerequisite. */
        private void settle(LoopClock clock) {
            ReferenceFlywheels.Status wheels = flywheels.status();
            ReferenceInventoryStatusService.Status objects = inventory.status();
            long currentRequest = flywheels.requestId();
            if (currentRequest != requestId) {
                requestId = currentRequest;
                resetSettling();
            }
            boolean eligible = fresh(clock) && wheels.requestId() == requestId
                    && wheels.requestedVelocityTicksPerSec() == config.feedVelocityTicksPerSec
                    && wheels.ready() && objects.firstPositionOccupied
                    && objects.orderIssue == ReferenceInventoryStatusService.OrderIssue.NONE
                    && wheels.sampledAt().secondsSince(startedAt) >= 0.0;
            if (!eligible) {
                resetSettling();
            } else if (sampleCycle > previousSampleCycle) {
                double gap = wheels.sampledAt().secondsSince(previousReadyAt);
                if (!settledSince.isAvailable() || !Double.isFinite(gap)
                        || gap > config.evidenceMaxAgeSec) {
                    settledSince = wheels.sampledAt();
                }
                previousReadyAt = wheels.sampledAt();
                previousSampleCycle = sampleCycle;
                if (wheels.sampledAt().secondsSince(settledSince) >= config.readySettlingSec
                        && startedAt.ageSec(clock) <= config.spinUpTimeoutSec) {
                    feeding = true; // Uncertain cancellation from this point requires acknowledgement.
                    feedStartedAt = clock.nowTimestamp();
                    previousFeedSampleAt = wheels.sampledAt();
                    phase = Phase.RELEASING;
                    phaseTask = ScalarTasks.set(release.commandTarget(), 1.0)
                            .forSeconds(config.releaseDurationSec).leaveThere().build();
                    phaseTask.start(clock);
                    publishStatus();
                    return;
                }
            }
            if (startedAt.ageSec(clock) >= config.spinUpTimeoutSec) {
                end(Reason.PREREQUISITE_TIMEOUT, TaskOutcome.TIMEOUT);
            }
        }

        /** Discard all previously accumulated ready span without inventing an observation. */
        private void resetSettling() {
            settledSince = LoopTimestamp.unavailable();
            previousReadyAt = LoopTimestamp.unavailable();
            previousSampleCycle = -1;
        }

        /** Confirm a post-realization edge and preserve exact timed-phase outcomes. */
        private void feed(LoopClock clock) {
            double elapsed = feedStartedAt.ageSec(clock);
            if (elapsed > config.departureTimeoutSec) {
                end(Reason.DEPARTURE_TIMEOUT, TaskOutcome.TIMEOUT); return;
            }
            if (flywheels.requestId() != requestId) {
                end(Reason.REQUEST_CHANGED, TaskOutcome.CANCELLED); return;
            }
            if (!fresh(clock)) {
                end(Reason.EVIDENCE_LOST, TaskOutcome.CANCELLED); return;
            }
            double observationGap = sampledAt.secondsSince(previousFeedSampleAt);
            if (!Double.isFinite(observationGap) || observationGap > config.evidenceMaxAgeSec) {
                end(Reason.EVIDENCE_LOST, TaskOutcome.CANCELLED); return;
            }
            previousFeedSampleAt = sampledAt;
            if (!flywheels.status().ready()) {
                end(Reason.WHEEL_SPEED_LOST, TaskOutcome.CANCELLED); return;
            }
            ReferenceInventoryStatusService.Status objects = inventory.status();
            if (releaseRealizedCycle >= 0 && objects.sampleCycle >= releaseRealizedCycle) {
                if (armedOccupiedCycle < 0 && objects.firstPositionOccupied) {
                    armedOccupiedCycle = objects.sampleCycle;
                } else if (armedOccupiedCycle >= 0 && objects.sampleCycle > armedOccupiedCycle
                        && !objects.firstPositionOccupied) {
                    departureObserved = true;
                }
            }
            if (phaseTask != null) {
                phaseTask.update(clock);
                if (!isActive()) return;
                if (phaseTask.isComplete()) {
                    TaskOutcome phaseOutcome = phaseTask.getOutcome();
                    if (phaseOutcome != TaskOutcome.SUCCESS) {
                        end(Reason.FAILED, phaseOutcome); return;
                    }
                    if (phase == Phase.RELEASING) {
                        release.commandTarget().set(0.0);
                        phase = Phase.TRANSFERRING;
                        phaseTask = ScalarTasks.set(transfer.commandTarget(), config.transferPower)
                                .forSeconds(config.transferDurationSec).leaveThere().build();
                        phaseTask.start(clock);
                    } else {
                        transfer.commandTarget().set(0.0);
                        phase = Phase.CONFIRMING;
                        phaseTask = null;
                    }
                    publishStatus();
                }
            }
            if (phase == Phase.CONFIRMING && departureObserved) {
                end(Reason.DEPARTURE_OBSERVED, TaskOutcome.SUCCESS);
            } else if (elapsed >= config.departureTimeoutSec) {
                end(Reason.DEPARTURE_TIMEOUT, TaskOutcome.TIMEOUT);
            }
        }

        /** Both child observations must belong to the owner's same complete output publication. */
        private boolean fresh(LoopClock clock) {
            ReferenceFlywheels.Status wheels = flywheels.status();
            ReferenceInventoryStatusService.Status objects = inventory.status();
            return sampledAt.isFresh(clock, config.evidenceMaxAgeSec)
                    && objects.observed && objects.sampleCycle == sampleCycle
                    && wheels.sampleCycle() == sampleCycle
                    && objects.sampledAt.isFresh(clock, config.evidenceMaxAgeSec)
                    && wheels.sampledAt().isFresh(clock, config.evidenceMaxAgeSec);
        }

        /** Select a truthful reason before shared terminal cleanup. */
        private void end(Reason endingReason, TaskOutcome outcome) {
            reason = endingReason;
            complete(outcome);
        }

        /** A named direct abort uses the same guarded cancellation/cleanup path. */
        private void abort(Reason endingReason) {
            if (!isComplete()) {
                if (reason != Reason.FAILED) reason = endingReason;
                cancel();
            }
        }

        /** Preserve the output owner's exception in the active attempt; never release a continuation. */
        private void failFromOutput(RuntimeException failure) {
            observe(() -> { throw failure; });
        }

        /** Cancellation chooses a reason; finalization owns all resource/request cleanup. */
        @Override protected void onCancel() {
            if (reason == Reason.NONE) reason = Reason.CANCELLED;
        }

        /** Retain exceptional failure rather than presenting it as normal cancellation. */
        @Override protected void onFailure(RuntimeException failure) {
            reason = Reason.FAILED;
        }

        /** Cancel owned phase, restore requests once, and freeze policy even if cleanup fails. */
        @Override protected void onFinish() {
            if (active != this) return; // Rejected/old attempts never own somebody else's commands.
            if (feeding && reason != Reason.DEPARTURE_OBSERVED) {
                recoveryRequired = true;
                recoveryAfterCycle = ownerClock.cycle();
                generation++;
            }
            try {
                CleanupActions.attemptAll(
                        () -> { if (phaseTask != null) phaseTask.cancel(); },
                        ReferenceLauncherMechanism.this::requestIdle);
            } catch (RuntimeException failure) {
                reason = Reason.FAILED;
                if (feeding) {
                    recoveryRequired = true;
                    recoveryAfterCycle = ownerClock.cycle();
                    generation++;
                }
                throw failure;
            } finally {
                active = null;
                lastPhase = phase;
                lastReason = reason;
                publishStatus();
            }
        }

        /** Frozen per-attempt diagnostics remain attributable after another attempt starts. */
        @Override protected void debugState(DebugSink dbg, String prefix) {
            dbg.addData(prefix + ".phase", phase).addData(prefix + ".reason", reason)
                    .addData(prefix + ".requestId", requestId)
                    .addData(prefix + ".releaseRealizedCycle", releaseRealizedCycle)
                    .addData(prefix + ".armedOccupiedCycle", armedOccupiedCycle)
                    .addData(prefix + ".departureObserved", departureObserved);
        }
    }

    /** Validate all authored policy before any actuator lookup. */
    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "ReferenceLauncherMechanism.Config is required");
        Config c = new Config();
        c.flywheels = ReferenceFlywheelMechanism.copyAndValidate(s.flywheels);
        ReferenceInventoryStatusService.Config i = Objects.requireNonNull(s.inventory, "inventory");
        c.inventory = ReferenceInventoryStatusService.Config.defaults();
        c.inventory.firstPositionSensorName = requireName(i.firstPositionSensorName,
                "firstPositionSensorName");
        c.inventory.secondPositionSensorName = requireName(i.secondPositionSensorName,
                "secondPositionSensorName");
        c.inventory.thirdPositionSensorName = requireName(i.thirdPositionSensorName,
                "thirdPositionSensorName");
        c.inventory.occupiedDebounceSec = nonnegative(i.occupiedDebounceSec, "occupiedDebounceSec");
        c.inventory.vacatedDebounceSec = nonnegative(i.vacatedDebounceSec, "vacatedDebounceSec");
        c.transferName = requireName(s.transferName, "transferName");
        c.transferDirection = Objects.requireNonNull(s.transferDirection, "transferDirection");
        c.releaseServoName = requireName(s.releaseServoName, "releaseServoName");
        c.releaseServoDirection = Objects.requireNonNull(s.releaseServoDirection, "releaseServoDirection");
        c.feedVelocityTicksPerSec = positive(s.feedVelocityTicksPerSec, "feedVelocityTicksPerSec");
        if (c.feedVelocityTicksPerSec > c.flywheels.maximumVelocityTicksPerSec
                || c.feedVelocityTicksPerSec <= c.flywheels.velocityToleranceTicksPerSec) {
            throw new IllegalArgumentException("feedVelocityTicksPerSec must exceed wheel tolerance"
                    + " and not exceed maximumVelocityTicksPerSec");
        }
        c.spinUpTimeoutSec = positive(s.spinUpTimeoutSec, "spinUpTimeoutSec");
        c.readySettlingSec = positive(s.readySettlingSec, "readySettlingSec");
        c.evidenceMaxAgeSec = positive(s.evidenceMaxAgeSec, "evidenceMaxAgeSec");
        c.departureTimeoutSec = positive(s.departureTimeoutSec, "departureTimeoutSec");
        c.transferPower = s.transferPower;
        if (!Double.isFinite(c.transferPower) || c.transferPower == 0 || Math.abs(c.transferPower) > 1) {
            throw new IllegalArgumentException("transferPower must be finite, nonzero, and in [-1, 1]");
        }
        c.transferDurationSec = positive(s.transferDurationSec, "transferDurationSec");
        c.releaseDurationSec = positive(s.releaseDurationSec, "releaseDurationSec");
        c.releaseRetractedNativePosition = unit(s.releaseRetractedNativePosition,
                "releaseRetractedNativePosition");
        c.releaseExtendedNativePosition = unit(s.releaseExtendedNativePosition,
                "releaseExtendedNativePosition");
        if (c.releaseRetractedNativePosition == c.releaseExtendedNativePosition) {
            throw new IllegalArgumentException("releaseRetractedNativePosition and"
                    + " releaseExtendedNativePosition must be different");
        }
        if (c.readySettlingSec >= c.spinUpTimeoutSec) {
            throw new IllegalArgumentException("readySettlingSec must be less than spinUpTimeoutSec");
        }
        if (c.departureTimeoutSec <= c.releaseDurationSec + c.transferDurationSec) {
            throw new IllegalArgumentException("departureTimeoutSec must exceed releaseDurationSec"
                    + " + transferDurationSec to allow confirmation after the timed phases");
        }
        return c;
    }

    private static String requireName(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must be a nonblank FTC hardware name");
        }
        return value.trim();
    }

    private static double positive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double nonnegative(double value, String field) {
        if (!Double.isFinite(value) || value < 0) {
            throw new IllegalArgumentException(field + " must be finite and >= 0, got " + value);
        }
        return value;
    }

    private static double unit(double value, String field) {
        if (!Double.isFinite(value) || value < 0 || value > 1) {
            throw new IllegalArgumentException(field + " must be finite and in [0, 1], got " + value);
        }
        return value;
    }
}
