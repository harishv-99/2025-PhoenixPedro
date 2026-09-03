package edu.ftcsushi.robots.examples.reference.capability.launcher;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantTargetResolver;
import edu.ftcsushi.fw.actuation.PlantTargets;
import edu.ftcsushi.fw.actuation.ScalarTasks;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.OutputTaskRunner;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheelMechanism;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheels;

/** Owns release/feed policy and delegates paired velocity to one focused flywheel owner. */
public final class ReferenceLauncherMechanism
        implements ReferenceLauncher, RobotProgram.Output {

    /** Data-only wiring, bounds, and behavior values for the reference mechanism. */
    public static final class Config {
        public ReferenceFlywheelMechanism.Config flywheels;
        public String transferName;
        public Direction transferDirection;
        public String releaseServoName;
        public Direction releaseServoDirection;
        public String objectSensorName;
        public double launchVelocityTicksPerSec;
        public double spinUpTimeoutSec;
        public double transferPower;
        public double transferDurationSec;

        /**
         * Native FTC Servo endpoint mapped from the normalized release Plant's retracted target
         * {@code 0.0}, in inclusive range {@code [0, 1]}. This fact must be backed off from unsafe
         * travel and reviewed on the assembled mechanism.
         */
        public double releaseRetractedNativePosition;

        /**
         * Native FTC Servo endpoint mapped from the normalized release Plant's extended target
         * {@code 1.0}, in inclusive range {@code [0, 1]}. This fact must be backed off from unsafe
         * travel and reviewed on the assembled mechanism.
         */
        public double releaseExtendedNativePosition;
        public double releaseDurationSec;

        private Config() {
        }

        /** Returns compiling example values, not reviewed physical facts. */
        public static Config defaults() {
            Config c = new Config();
            c.flywheels = ReferenceFlywheelMechanism.Config.defaults();
            c.transferName = "transfer";
            c.transferDirection = Direction.FORWARD;
            c.releaseServoName = "release";
            c.releaseServoDirection = Direction.FORWARD;
            c.objectSensorName = "objectPresent";
            c.launchVelocityTicksPerSec = 3000.0;
            c.spinUpTimeoutSec = 2.0;
            c.transferPower = 0.25;
            c.transferDurationSec = 0.20;
            c.releaseRetractedNativePosition = 0.25;
            c.releaseExtendedNativePosition = 0.60;
            c.releaseDurationSec = 0.15;
            return c;
        }
    }

    private static final double IDLE_FLYWHEEL_VELOCITY_TICKS_PER_SEC = 0.0;
    private static final double IDLE_TRANSFER_POWER = 0.0;
    private static final double RELEASE_RETRACTED_TARGET = 0.0;
    private static final double RELEASE_EXTENDED_TARGET = 1.0;

    private final ReferenceFlywheelMechanism flywheels;
    private final Plant transfer;
    private final Plant release;
    private final BooleanSource objectPresent;
    private final OutputTaskRunner transferOverrides =
            Tasks.outputQueue(IDLE_TRANSFER_POWER);
    private final double launchVelocityTicksPerSec;
    private final double spinUpTimeoutSec;
    private final double transferPower;
    private final double transferDurationSec;
    private final double releaseDurationSec;

    private long launchGeneration;
    private Status lastStatus;

    /**
     * Constructs the complete mechanism after validating its copied configuration.
     *
     * <p>The focused {@link ReferenceFlywheelMechanism} owns the grouped Plant and independent
     * wheel evidence. This launcher composes that capability with release, transfer, and object
     * sensing; it does not create a second flywheel target or writer.</p>
     */
    public ReferenceLauncherMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        BooleanSource builtObjectPresent = FtcSensors.digitalLow(map, c.objectSensorName)
                .debouncedOnOff(0.02, 0.02);

        PlantTargetResolver transferTarget = PlantTargets.overlay(
                        ScalarSource.of(() -> IDLE_TRANSFER_POWER))
                .add("temporaryTransfer", transferOverrides.activeSource(), transferOverrides)
                .build();

        ReferenceFlywheelMechanism builtFlywheels = null;
        Plant builtTransfer = null;
        Plant builtRelease = null;
        Status builtInitialStatus;
        try {
            builtFlywheels = new ReferenceFlywheelMechanism(map, c.flywheels);
            builtTransfer = FtcActuators.plant(map)
                    .crServo(c.transferName, c.transferDirection)
                    .power()
                    .targetFromResolver(transferTarget)
                    .build();
            builtRelease = FtcActuators.plant(map)
                    .servo(c.releaseServoName, c.releaseServoDirection)
                    .position()
                    .nonPeriodic()
                    .bounded(RELEASE_RETRACTED_TARGET, RELEASE_EXTENDED_TARGET)
                    .rangeMapsToNative(
                            c.releaseRetractedNativePosition,
                            c.releaseExtendedNativePosition)
                    .targetFromNewCommand(RELEASE_RETRACTED_TARGET)
                    .build();
            builtInitialStatus = new Status(
                    builtFlywheels.status(),
                    false,
                    false);
        } catch (RuntimeException failure) {
            ReferenceFlywheelMechanism f = builtFlywheels;
            Plant t = builtTransfer;
            Plant r = builtRelease;
            throw CleanupActions.attemptAllAfterFailure(
                    failure,
                    () -> stopIfBuilt(r),
                    () -> stopIfBuilt(t),
                    () -> stopIfBuilt(f));
        }

        flywheels = builtFlywheels;
        transfer = builtTransfer;
        release = builtRelease;
        objectPresent = builtObjectPresent;
        launchVelocityTicksPerSec = c.launchVelocityTicksPerSec;
        spinUpTimeoutSec = c.spinUpTimeoutSec;
        transferPower = c.transferPower;
        transferDurationSec = c.transferDurationSec;
        releaseDurationSec = c.releaseDurationSec;
        lastStatus = builtInitialStatus;
    }

    /** {@inheritDoc} */
    @Override
    public ReferenceFlywheels flywheels() {
        return flywheels;
    }

    /** {@inheritDoc} */
    @Override
    public void abortLaunches() {
        launchGeneration++;
        requestActiveMatchIdle();
    }

    /** {@inheritDoc} */
    @Override
    public Task launchOne() {
        return new LaunchTask(launchGeneration);
    }

    /** {@inheritDoc} */
    @Override
    public Status status() {
        return lastStatus;
    }

    /**
     * Advances the transfer queue, delegated flywheel owner, and two local Plants in output order,
     * then publishes one complete launcher Status after every evidence capture succeeds.
     */
    @Override
    public void update(LoopClock clock) {
        transferOverrides.update(clock);
        flywheels.update(clock);
        transfer.update(clock);
        release.update(clock);

        boolean capturedObjectPresent = objectPresent.getAsBoolean(clock);
        boolean transferPulseActive = transferOverrides.hasActiveTask();

        lastStatus = new Status(
                flywheels.status(),
                capturedObjectPresent,
                transferPulseActive);
    }

    /** Terminally stops the complete owned actuator graph and invalidates outstanding launches. */
    @Override
    public void stop() {
        launchGeneration++;
        Status priorStatus = lastStatus;
        CleanupActions.attemptAll(
                transferOverrides::cancelAndClear,
                release::stop,
                transfer::stop,
                flywheels::stop);
        lastStatus = new Status(
                flywheels.status(),
                priorStatus.objectPresent(),
                false);
    }

    /** Enqueue the one temporary transfer override used only by a started launch Task. */
    private void enqueueTransferPulse() {
        transferOverrides.enqueue(
                Tasks.outputForSeconds(
                        "referenceTransfer",
                        transferPower,
                        transferDurationSec));
    }

    /** Clear every temporary request without terminally stopping the owned Plants. */
    private void requestActiveMatchIdle() {
        CleanupActions.attemptAll(
                transferOverrides::cancelAndClear,
                () -> release.commandTarget().set(RELEASE_RETRACTED_TARGET),
                () -> flywheels.setVelocityTicksPerSec(
                        IDLE_FLYWHEEL_VELOCITY_TICKS_PER_SEC));
    }

    /**
     * One example-local lifecycle wrapper around factory-composed launch phases.
     *
     * <p>The wrapper adds only the policy generic factories cannot express together: retained
     * timeout outcome, terminal cleanup, and invalidation of active or queued tasks created before
     * {@link #abortLaunches()}.</p>
     */
    private final class LaunchTask implements Task {
        private final long generationAtCreation;
        private final Task spinUp;
        private final Task launchFlow;

        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private boolean cleanupAttempted;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private LaunchTask(long generationAtCreation) {
            this.generationAtCreation = generationAtCreation;

            ReferenceFlywheels.Status[] statusBeforeRequest = {null};
            spinUp = Tasks.sequence(
                    Tasks.runOnce(() -> {
                        statusBeforeRequest[0] = flywheels.status();
                        flywheels.setVelocityTicksPerSec(launchVelocityTicksPerSec);
                    }),
                    Tasks.waitUntil(BooleanSource.of(() -> {
                        ReferenceFlywheels.Status status = flywheels.status();
                        return status != statusBeforeRequest[0]
                                && Double.compare(
                                        status.requestedVelocityTicksPerSec(),
                                        launchVelocityTicksPerSec) == 0
                                && status.ready();
                    }), spinUpTimeoutSec));

            Task feed = Tasks.sequence(
                    ScalarTasks.set(release.commandTarget(), RELEASE_EXTENDED_TARGET)
                            .forSeconds(releaseDurationSec)
                            .leaveThere()
                            .build(),
                    Tasks.runOnce(() -> release.commandTarget().set(
                            RELEASE_RETRACTED_TARGET)),
                    Tasks.runOnce(ReferenceLauncherMechanism.this::enqueueTransferPulse),
                    Tasks.waitForSeconds(transferDurationSec));

            launchFlow = Tasks.branchOnOutcome(spinUp, feed, Tasks.noop());
        }

        /** {@inheritDoc} */
        @Override
        public void start(LoopClock clock) {
            markStartAttempt();
            Objects.requireNonNull(clock, "Reference launch start clock is required");
            started = true;

            if (generationAtCreation != launchGeneration) {
                complete = true;
                outcome = TaskOutcome.CANCELLED;
                return;
            }

            launchFlow.start(clock);
        }

        /** {@inheritDoc} */
        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException(
                        "Reference launch Task cannot be updated before start(clock). Start the "
                                + "fresh Task returned by ReferenceLauncher.launchOne().");
            }
            if (complete) {
                return;
            }
            Objects.requireNonNull(clock, "Reference launch update clock is required");

            if (generationAtCreation != launchGeneration) {
                finishInvalidated();
                return;
            }

            launchFlow.update(clock);

            if (generationAtCreation != launchGeneration) {
                finishInvalidated();
                return;
            }

            if (spinUp.isComplete() && spinUp.getOutcome() == TaskOutcome.TIMEOUT) {
                finishAndCleanup(TaskOutcome.TIMEOUT, true);
                return;
            }

            if (launchFlow.isComplete()) {
                TaskOutcome flowOutcome = launchFlow.getOutcome();
                if (flowOutcome != TaskOutcome.SUCCESS
                        && flowOutcome != TaskOutcome.TIMEOUT
                        && flowOutcome != TaskOutcome.CANCELLED) {
                    throw new IllegalStateException(
                            "Reference launch flow completed with unsupported outcome "
                                    + flowOutcome + ". Create a fresh launch Task.");
                }
                finishAndCleanup(flowOutcome, false);
            }
        }

        /** {@inheritDoc} */
        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            if (generationAtCreation != launchGeneration) {
                finishInvalidated();
                return;
            }
            finishAndCleanup(TaskOutcome.CANCELLED, true);
        }

        /** {@inheritDoc} */
        @Override
        public boolean isComplete() {
            return complete;
        }

        /** {@inheritDoc} */
        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }

        /** {@inheritDoc} */
        @Override
        public String getDebugName() {
            return complete
                    ? "ReferenceLauncher.launchOne(DONE:" + outcome + ")"
                    : "ReferenceLauncher.launchOne";
        }

        /** Publish terminal state first, then cancel child work and clean every owned request. */
        private void finishAndCleanup(TaskOutcome terminalOutcome, boolean cancelFlow) {
            complete = true;
            outcome = terminalOutcome;

            RuntimeException primaryFailure = null;
            if (cancelFlow) {
                try {
                    launchFlow.cancel();
                } catch (RuntimeException failure) {
                    primaryFailure = failure;
                }
            }

            if (primaryFailure == null) {
                cleanupOnce();
            } else {
                RuntimeException retainedFailure = CleanupActions.attemptAllAfterFailure(
                        primaryFailure,
                        this::cleanupOnce);
                throw retainedFailure;
            }
        }

        /** Run task-terminal request cleanup at most once. */
        private void cleanupOnce() {
            if (cleanupAttempted) {
                return;
            }
            cleanupAttempted = true;
            requestActiveMatchIdle();
        }

        /**
         * Cancel only the private child graph after abort already cleaned shared requests.
         *
         * <p>The timed release child deliberately leaves its target unchanged on cancellation, and
         * no remaining child cancellation can rewrite flywheel intent. Skipping shared cleanup here
         * prevents an old Task's later update or direct cancellation from overwriting requests made
         * after {@link #abortLaunches()}; that abort already established the owned cleanup state.</p>
         */
        private void finishInvalidated() {
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            launchFlow.cancel();
        }

        /** Consume the one allowed start before any child or capability side effect. */
        private void markStartAttempt() {
            if (startAttempted) {
                throw new IllegalStateException(
                        "Reference launch Task is single-use and start(...) was called more than "
                                + "once. Create a fresh Task with ReferenceLauncher.launchOne().");
            }
            startAttempted = true;
        }
    }

    /** Copy and validate every authored fact before the constructor performs hardware lookup. */
    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "ReferenceLauncherMechanism.Config is required");
        Config c = new Config();
        c.flywheels = ReferenceFlywheelMechanism.copyAndValidate(s.flywheels);
        c.transferName = requireName(s.transferName, "transferName");
        c.transferDirection = Objects.requireNonNull(s.transferDirection, "transferDirection");
        c.releaseServoName = requireName(s.releaseServoName, "releaseServoName");
        c.releaseServoDirection = Objects.requireNonNull(s.releaseServoDirection,
                "releaseServoDirection");
        c.objectSensorName = requireName(s.objectSensorName, "objectSensorName");
        c.launchVelocityTicksPerSec = positive(
                s.launchVelocityTicksPerSec,
                "launchVelocityTicksPerSec");
        if (c.launchVelocityTicksPerSec > c.flywheels.maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "launchVelocityTicksPerSec must be <= maximumVelocityTicksPerSec");
        }
        if (c.launchVelocityTicksPerSec <= c.flywheels.velocityToleranceTicksPerSec) {
            throw new IllegalArgumentException(
                    "launchVelocityTicksPerSec must be > velocityToleranceTicksPerSec");
        }
        c.spinUpTimeoutSec = positive(s.spinUpTimeoutSec, "spinUpTimeoutSec");
        c.transferPower = normalizedNonzero(s.transferPower, "transferPower");
        c.transferDurationSec = positive(s.transferDurationSec, "transferDurationSec");
        c.releaseRetractedNativePosition = unit(s.releaseRetractedNativePosition,
                "releaseRetractedNativePosition");
        c.releaseExtendedNativePosition = unit(s.releaseExtendedNativePosition,
                "releaseExtendedNativePosition");
        c.releaseDurationSec = positive(s.releaseDurationSec, "releaseDurationSec");
        if (Double.compare(
                c.releaseRetractedNativePosition,
                c.releaseExtendedNativePosition) == 0) {
            throw new IllegalArgumentException(
                    "releaseRetractedNativePosition and releaseExtendedNativePosition "
                            + "must be different");
        }
        return c;
    }

    private static String requireName(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must be a non-blank FTC hardware name");
        }
        return value;
    }

    private static double positive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double unit(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(field + " must be finite and in [0, 1], got " + value);
        }
        return value;
    }

    private static double normalizedNonzero(double value, String field) {
        if (!Double.isFinite(value) || value == 0.0 || value < -1.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    field + " must be finite, nonzero, and in [-1, 1], got " + value);
        }
        return value;
    }

    private static void stopIfBuilt(Plant plant) {
        if (plant != null) plant.stop();
    }

    private static void stopIfBuilt(ReferenceFlywheelMechanism mechanism) {
        if (mechanism != null) mechanism.stop();
    }
}
