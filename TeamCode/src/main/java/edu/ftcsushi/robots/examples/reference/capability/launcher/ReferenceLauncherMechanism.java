package edu.ftcsushi.robots.examples.reference.capability.launcher;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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

/** Owns the reference launcher's Plants, sensor evidence, feed queue, and safe task factories. */
public final class ReferenceLauncherMechanism
        implements ReferenceLauncher, RobotProgram.Output {

    /** Data-only wiring, bounds, and behavior values for the reference mechanism. */
    public static final class Config {
        public String leftFlywheelName;
        public Direction leftFlywheelDirection;
        public String rightFlywheelName;
        public Direction rightFlywheelDirection;
        public String transferName;
        public Direction transferDirection;
        public String releaseServoName;
        public Direction releaseServoDirection;
        public String objectSensorName;
        public double maximumVelocityTicksPerSec;
        public double velocityToleranceTicksPerSec;
        public double launchVelocityTicksPerSec;
        public double spinUpTimeoutSec;
        public double transferPower;
        public double transferDurationSec;
        public double releaseRetractedPosition;
        public double releaseExtendedPosition;
        public double releaseDurationSec;

        private Config() {
        }

        /** Returns compiling example values, not reviewed physical facts. */
        public static Config defaults() {
            Config c = new Config();
            c.leftFlywheelName = "flywheelLeft";
            c.leftFlywheelDirection = Direction.FORWARD;
            c.rightFlywheelName = "flywheelRight";
            c.rightFlywheelDirection = Direction.REVERSE;
            c.transferName = "transfer";
            c.transferDirection = Direction.FORWARD;
            c.releaseServoName = "release";
            c.releaseServoDirection = Direction.FORWARD;
            c.objectSensorName = "objectPresent";
            c.maximumVelocityTicksPerSec = 5000.0;
            c.velocityToleranceTicksPerSec = 100.0;
            c.launchVelocityTicksPerSec = 3000.0;
            c.spinUpTimeoutSec = 2.0;
            c.transferPower = 0.25;
            c.transferDurationSec = 0.20;
            c.releaseRetractedPosition = 0.25;
            c.releaseExtendedPosition = 0.60;
            c.releaseDurationSec = 0.15;
            return c;
        }
    }

    private static final double IDLE_FLYWHEEL_VELOCITY_TICKS_PER_SEC = 0.0;
    private static final double IDLE_TRANSFER_POWER = 0.0;

    private final Plant flywheel;
    private final Plant transfer;
    private final Plant release;
    private final ScalarSource leftMeasuredVelocityTicksPerSec;
    private final ScalarSource rightMeasuredVelocityTicksPerSec;
    private final BooleanSource objectPresent;
    private final OutputTaskRunner transferOverrides =
            Tasks.outputQueue(IDLE_TRANSFER_POWER);
    private final double maximumVelocityTicksPerSec;
    private final double velocityToleranceTicksPerSec;
    private final double launchVelocityTicksPerSec;
    private final double spinUpTimeoutSec;
    private final double transferPower;
    private final double transferDurationSec;
    private final double releaseRetractedPosition;
    private final double releaseExtendedPosition;
    private final double releaseDurationSec;

    private long launchGeneration;
    private Status lastStatus = new Status(
            IDLE_FLYWHEEL_VELOCITY_TICKS_PER_SEC,
            Double.NaN,
            Double.NaN,
            false,
            false,
            false,
            false);

    /**
     * Constructs the complete mechanism after validating its copied configuration.
     *
     * <p>The two flywheel names are checked for distinct trimmed FTC keys before the first hardware
     * lookup. The paired velocity Plant remains the one grouped actuator command/stop owner. Two
     * separate memoized SDK velocity sources add no Sushi direction transform; their readings
     * are observed only after the paired Plant has configured the motor directions. They provide
     * truthful per-wheel evidence without creating another command path.</p>
     */
    public ReferenceLauncherMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        DcMotorEx leftFlywheel = map.get(DcMotorEx.class, c.leftFlywheelName);
        DcMotorEx rightFlywheel = map.get(DcMotorEx.class, c.rightFlywheelName);
        ScalarSource builtLeftMeasuredVelocityTicksPerSec =
                FtcSensors.motorVelocityTicksPerSec(leftFlywheel);
        ScalarSource builtRightMeasuredVelocityTicksPerSec =
                FtcSensors.motorVelocityTicksPerSec(rightFlywheel);
        BooleanSource builtObjectPresent = FtcSensors.digitalLow(map, c.objectSensorName)
                .debouncedOnOff(0.02, 0.02);

        PlantTargetResolver transferTarget = PlantTargets.overlay(
                        ScalarSource.of(() -> IDLE_TRANSFER_POWER))
                .add("temporaryTransfer", transferOverrides.activeSource(), transferOverrides)
                .build();

        Plant builtFlywheel = null;
        Plant builtTransfer = null;
        Plant builtRelease = null;
        try {
            builtFlywheel = FtcActuators.plant(map)
                    .motor(c.leftFlywheelName, c.leftFlywheelDirection)
                    .andMotor(c.rightFlywheelName, c.rightFlywheelDirection)
                    .velocity()
                    .deviceManaged()
                    .bounded(IDLE_FLYWHEEL_VELOCITY_TICKS_PER_SEC,
                            c.maximumVelocityTicksPerSec)
                    .nativeUnits()
                    .velocityTolerance(c.velocityToleranceTicksPerSec)
                    .targetFromNewCommand(IDLE_FLYWHEEL_VELOCITY_TICKS_PER_SEC)
                    .build();
            builtTransfer = FtcActuators.plant(map)
                    .crServo(c.transferName, c.transferDirection)
                    .power()
                    .targetFromResolver(transferTarget)
                    .build();
            builtRelease = FtcActuators.plant(map)
                    .servo(c.releaseServoName, c.releaseServoDirection)
                    .position()
                    .nonPeriodic()
                    .bounded(0.0, 1.0)
                    .nativeUnits()
                    .targetFromNewCommand(c.releaseRetractedPosition)
                    .build();
        } catch (RuntimeException failure) {
            Plant f = builtFlywheel;
            Plant t = builtTransfer;
            Plant r = builtRelease;
            throw CleanupActions.attemptAllAfterFailure(
                    failure,
                    () -> stopIfBuilt(r),
                    () -> stopIfBuilt(t),
                    () -> stopIfBuilt(f));
        }

        flywheel = builtFlywheel;
        transfer = builtTransfer;
        release = builtRelease;
        leftMeasuredVelocityTicksPerSec = builtLeftMeasuredVelocityTicksPerSec;
        rightMeasuredVelocityTicksPerSec = builtRightMeasuredVelocityTicksPerSec;
        objectPresent = builtObjectPresent;
        maximumVelocityTicksPerSec = c.maximumVelocityTicksPerSec;
        velocityToleranceTicksPerSec = c.velocityToleranceTicksPerSec;
        launchVelocityTicksPerSec = c.launchVelocityTicksPerSec;
        spinUpTimeoutSec = c.spinUpTimeoutSec;
        transferPower = c.transferPower;
        transferDurationSec = c.transferDurationSec;
        releaseRetractedPosition = c.releaseRetractedPosition;
        releaseExtendedPosition = c.releaseExtendedPosition;
        releaseDurationSec = c.releaseDurationSec;
    }

    /** {@inheritDoc} */
    @Override
    public void setTargetVelocityTicksPerSec(double velocityTicksPerSec) {
        if (!Double.isFinite(velocityTicksPerSec)
                || velocityTicksPerSec < IDLE_FLYWHEEL_VELOCITY_TICKS_PER_SEC
                || velocityTicksPerSec > maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "velocityTicksPerSec must be finite and in [0, "
                            + maximumVelocityTicksPerSec + "], got " + velocityTicksPerSec);
        }
        flywheel.commandTarget().set(velocityTicksPerSec);
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
     * Advances the transfer queue and all three owned Plants once in program output order, then
     * publishes one per-wheel evidence snapshot for this loop cycle.
     */
    @Override
    public void update(LoopClock clock) {
        transferOverrides.update(clock);
        flywheel.update(clock);
        transfer.update(clock);
        release.update(clock);

        double targetVelocityTicksPerSec = flywheel.commandTarget().get();
        double leftVelocityTicksPerSec =
                leftMeasuredVelocityTicksPerSec.getAsDouble(clock);
        double rightVelocityTicksPerSec =
                rightMeasuredVelocityTicksPerSec.getAsDouble(clock);
        boolean leftAtTarget = withinTolerance(
                leftVelocityTicksPerSec,
                targetVelocityTicksPerSec,
                velocityToleranceTicksPerSec);
        boolean rightAtTarget = withinTolerance(
                rightVelocityTicksPerSec,
                targetVelocityTicksPerSec,
                velocityToleranceTicksPerSec);

        lastStatus = new Status(
                targetVelocityTicksPerSec,
                leftVelocityTicksPerSec,
                rightVelocityTicksPerSec,
                leftAtTarget,
                rightAtTarget,
                objectPresent.getAsBoolean(clock),
                transferOverrides.activeSource().getAsBoolean(clock));
    }

    /** Terminally stops the complete owned actuator graph and invalidates outstanding launches. */
    @Override
    public void stop() {
        launchGeneration++;
        CleanupActions.attemptAll(
                transferOverrides::cancelAndClear,
                release::stop,
                transfer::stop,
                flywheel::stop);
    }

    /** Enqueue the one temporary transfer override used only by a started launch Task. */
    private void enqueueTransferPulse() {
        transferOverrides.enqueue(
                Tasks.outputForSeconds(
                        "referenceTransfer",
                        transferPower,
                        transferDurationSec));
    }

    /** Return whether the latest cached snapshot proves readiness for this launch target. */
    private boolean launchTargetIsReady() {
        Status status = lastStatus;
        return Double.compare(
                status.targetVelocityTicksPerSec,
                launchVelocityTicksPerSec) == 0
                && status.ready;
    }

    /** Clear every temporary request without terminally stopping the owned Plants. */
    private void requestActiveMatchIdle() {
        CleanupActions.attemptAll(
                transferOverrides::cancelAndClear,
                () -> release.commandTarget().set(releaseRetractedPosition),
                () -> flywheel.commandTarget().set(
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

            spinUp = Tasks.sequence(
                    Tasks.runOnce(() -> setTargetVelocityTicksPerSec(
                            launchVelocityTicksPerSec)),
                    Tasks.waitUntil(
                            BooleanSource.of(
                                    ReferenceLauncherMechanism.this::launchTargetIsReady),
                            spinUpTimeoutSec));

            Task feed = Tasks.sequence(
                    ScalarTasks.set(release.commandTarget(), releaseExtendedPosition)
                            .forSeconds(releaseDurationSec)
                            .leaveThere()
                            .build(),
                    Tasks.runOnce(() -> release.commandTarget().set(
                            releaseRetractedPosition)),
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
        c.leftFlywheelName = requireName(s.leftFlywheelName, "leftFlywheelName");
        c.leftFlywheelDirection = Objects.requireNonNull(s.leftFlywheelDirection,
                "leftFlywheelDirection");
        c.rightFlywheelName = requireName(s.rightFlywheelName, "rightFlywheelName");
        c.rightFlywheelDirection = Objects.requireNonNull(s.rightFlywheelDirection,
                "rightFlywheelDirection");
        requireDistinctFlywheelNames(c.leftFlywheelName, c.rightFlywheelName);
        c.transferName = requireName(s.transferName, "transferName");
        c.transferDirection = Objects.requireNonNull(s.transferDirection, "transferDirection");
        c.releaseServoName = requireName(s.releaseServoName, "releaseServoName");
        c.releaseServoDirection = Objects.requireNonNull(s.releaseServoDirection,
                "releaseServoDirection");
        c.objectSensorName = requireName(s.objectSensorName, "objectSensorName");
        c.maximumVelocityTicksPerSec = positive(
                s.maximumVelocityTicksPerSec,
                "maximumVelocityTicksPerSec");
        c.velocityToleranceTicksPerSec = positive(
                s.velocityToleranceTicksPerSec,
                "velocityToleranceTicksPerSec");
        c.launchVelocityTicksPerSec = positive(
                s.launchVelocityTicksPerSec,
                "launchVelocityTicksPerSec");
        if (c.launchVelocityTicksPerSec > c.maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "launchVelocityTicksPerSec must be <= maximumVelocityTicksPerSec");
        }
        if (c.launchVelocityTicksPerSec <= c.velocityToleranceTicksPerSec) {
            throw new IllegalArgumentException(
                    "launchVelocityTicksPerSec must be > velocityToleranceTicksPerSec");
        }
        c.spinUpTimeoutSec = positive(s.spinUpTimeoutSec, "spinUpTimeoutSec");
        c.transferPower = normalizedNonzero(s.transferPower, "transferPower");
        c.transferDurationSec = positive(s.transferDurationSec, "transferDurationSec");
        c.releaseRetractedPosition = unit(s.releaseRetractedPosition,
                "releaseRetractedPosition");
        c.releaseExtendedPosition = unit(s.releaseExtendedPosition,
                "releaseExtendedPosition");
        c.releaseDurationSec = positive(s.releaseDurationSec, "releaseDurationSec");
        if (Double.compare(c.releaseRetractedPosition, c.releaseExtendedPosition) == 0) {
            throw new IllegalArgumentException("release positions must be different");
        }
        return c;
    }

    /** Require the two group members to resolve from different trimmed, case-sensitive keys. */
    private static void requireDistinctFlywheelNames(String leftName, String rightName) {
        String leftKey = leftName.trim();
        String rightKey = rightName.trim();
        if (leftKey.equals(rightKey)) {
            throw new IllegalArgumentException(
                    "ReferenceLauncherMechanism.Config.leftFlywheelName and "
                            + "ReferenceLauncherMechanism.Config.rightFlywheelName must identify "
                            + "different FTC hardware devices after trimming; got effective key \""
                            + leftKey + "\".");
        }
    }

    /** Return whether one finite wheel measurement is inside the inclusive error tolerance. */
    private static boolean withinTolerance(double measured,
                                           double target,
                                           double tolerance) {
        return Double.isFinite(measured)
                && Double.isFinite(target)
                && Math.abs(measured - target) <= tolerance;
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
        if (plant != null) {
            plant.stop();
        }
    }
}
