package edu.ftcsushi.robots.examples.reference.capability.flywheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.actuation.ScalarTasks;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/** Owns one grouped paired-velocity Plant and publishes independent member readiness. */
public final class ReferenceFlywheelMechanism
        implements ReferenceFlywheels, RobotProgram.Output {

    /** Data-only wiring, velocity range, and per-wheel readiness tolerance. */
    public static final class Config {
        /** FTC configuration name for the left flywheel motor. */
        public String leftMotorName;
        /** Logical direction for positive left-wheel velocity. */
        public Direction leftMotorDirection;
        /** FTC configuration name for the right flywheel motor. */
        public String rightMotorName;
        /** Logical direction for positive right-wheel velocity. */
        public Direction rightMotorDirection;
        /** Inclusive maximum velocity command, in native encoder ticks per second. */
        public double maximumVelocityTicksPerSec;
        /** Inclusive readiness tolerance for each wheel, in ticks per second. */
        public double velocityToleranceTicksPerSec;

        private Config() {
        }

        /** Returns compiling software values, not reviewed motor identity, direction, or tuning. */
        public static Config defaults() {
            Config c = new Config();
            c.leftMotorName = "flywheelLeft";
            c.leftMotorDirection = Direction.FORWARD;
            c.rightMotorName = "flywheelRight";
            c.rightMotorDirection = Direction.REVERSE;
            c.maximumVelocityTicksPerSec = 5000.0;
            c.velocityToleranceTicksPerSec = 100.0;
            return c;
        }
    }

    private static final double IDLE_VELOCITY_TICKS_PER_SEC = 0.0;

    private final Plant flywheels;
    private final ScalarSource leftMeasuredVelocityTicksPerSec;
    private final ScalarSource rightMeasuredVelocityTicksPerSec;
    private final double maximumVelocityTicksPerSec;
    private final double velocityToleranceTicksPerSec;

    private Status lastStatus;

    /**
     * Constructs and privately owns the complete paired flywheel realization.
     *
     * <p>Configuration, including distinct effective names, is validated before the first
     * hardware lookup. The two direct SDK sources only observe member velocity after the grouped
     * Plant update; they never write either motor.</p>
     *
     * @param hardwareMap FTC registry containing both configured motors
     * @param config data-only paired flywheel configuration
     */
    public ReferenceFlywheelMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        DcMotorEx leftMotor = map.get(DcMotorEx.class, c.leftMotorName);
        DcMotorEx rightMotor = map.get(DcMotorEx.class, c.rightMotorName);
        ScalarSource builtLeftVelocity = FtcSensors.motorVelocityTicksPerSec(leftMotor);
        ScalarSource builtRightVelocity = FtcSensors.motorVelocityTicksPerSec(rightMotor);

        Plant builtFlywheels = createPlant(map, c);
        flywheels = builtFlywheels;
        leftMeasuredVelocityTicksPerSec = builtLeftVelocity;
        rightMeasuredVelocityTicksPerSec = builtRightVelocity;
        maximumVelocityTicksPerSec = c.maximumVelocityTicksPerSec;
        velocityToleranceTicksPerSec = c.velocityToleranceTicksPerSec;
        lastStatus = new Status(
                builtFlywheels.snapshot(),
                Double.NaN,
                Double.NaN,
                c.velocityToleranceTicksPerSec);
    }

    /**
     * Creates a fresh grouped Plant for one exclusive tuning workflow.
     *
     * <p>The returned Plant uses the production recipe, has not been updated, and is owned and
     * stopped only by its caller. This is an advanced exclusive-host seam, not a second ordinary
     * mechanism construction path.</p>
     */
    public static Plant createPlantForTuning(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        return createPlant(map, copyAndValidate(config));
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocityTicksPerSec(double velocityTicksPerSec) {
        requireVelocityInRange(velocityTicksPerSec);
        flywheels.commandTarget().set(velocityTicksPerSec);
    }

    /** {@inheritDoc} */
    @Override
    public Task setVelocityTask(double velocityTicksPerSec, double timeoutSec) {
        requireVelocityInRange(velocityTicksPerSec);
        requirePositive(timeoutSec, "timeoutSec");
        return new PairedReadyTask(velocityTicksPerSec, timeoutSec);
    }

    /** {@inheritDoc} */
    @Override
    public Status status() {
        return lastStatus;
    }

    /**
     * Advances the grouped Plant once, then atomically publishes its capture and both later member
     * samples. A failed sample leaves the preceding complete Status visible.
     */
    @Override
    public void update(LoopClock clock) {
        flywheels.update(clock);
        PlantSnapshot snapshot = flywheels.snapshot();
        double leftVelocity = leftMeasuredVelocityTicksPerSec.getAsDouble(clock);
        double rightVelocity = rightMeasuredVelocityTicksPerSec.getAsDouble(clock);
        lastStatus = new Status(
                snapshot,
                leftVelocity,
                rightVelocity,
                velocityToleranceTicksPerSec);
    }

    /** Terminally stops the one grouped Plant while retaining the last member measurements. */
    @Override
    public void stop() {
        Status prior = lastStatus;
        flywheels.stop();
        lastStatus = new Status(
                flywheels.snapshot(),
                prior.leftMeasuredVelocityTicksPerSec(),
                prior.rightMeasuredVelocityTicksPerSec(),
                velocityToleranceTicksPerSec);
    }

    /** Builds the canonical grouped realization shared by match and exclusive tuning owners. */
    private static Plant createPlant(HardwareMap map, Config c) {
        return FtcActuators.plant(map)
                .motor(c.leftMotorName, c.leftMotorDirection)
                .andMotor(c.rightMotorName, c.rightMotorDirection)
                .velocity()
                .deviceManaged()
                .bounded(IDLE_VELOCITY_TICKS_PER_SEC, c.maximumVelocityTicksPerSec)
                .nativeUnits()
                .velocityTolerance(c.velocityToleranceTicksPerSec)
                .targetFromNewCommand(IDLE_VELOCITY_TICKS_PER_SEC)
                .build();
    }

    private void requireVelocityInRange(double velocityTicksPerSec) {
        if (!Double.isFinite(velocityTicksPerSec)
                || velocityTicksPerSec < IDLE_VELOCITY_TICKS_PER_SEC
                || velocityTicksPerSec > maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "velocityTicksPerSec must be finite and in [0, "
                            + maximumVelocityTicksPerSec + "], got " + velocityTicksPerSec);
        }
    }

    /**
     * Adds paired member evidence to the factory-provided command-correlated feedback move.
     *
     * <p>The generic move may observe a misleading grouped mean when one wheel is high and the
     * other low. This wrapper therefore waits for both independent samples from a post-start
     * publication before reporting success.</p>
     */
    private final class PairedReadyTask implements Task {
        private final double requestedVelocityTicksPerSec;
        private final double timeoutSec;
        private final Task groupedMove;

        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private double startedAtSec;
        private Status statusAtStart;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private PairedReadyTask(double requestedVelocityTicksPerSec, double timeoutSec) {
            this.requestedVelocityTicksPerSec = requestedVelocityTicksPerSec;
            this.timeoutSec = timeoutSec;
            groupedMove = ScalarTasks.set(
                            flywheels.commandTarget(),
                            requestedVelocityTicksPerSec)
                    .untilReachedBy(flywheels)
                    .leaveRequestOnCancel()
                    .build();
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException(
                        "Reference flywheel velocity Task is single-use. Call "
                                + "setVelocityTask(...) again for another run.");
            }
            startAttempted = true;
            LoopClock requiredClock = Objects.requireNonNull(
                    clock,
                    "Reference flywheel Task start clock is required");
            started = true;
            startedAtSec = requiredClock.nowSec();
            statusAtStart = lastStatus;
            groupedMove.start(requiredClock);
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException(
                        "Reference flywheel velocity Task cannot be updated before start(clock)");
            }
            if (complete) {
                return;
            }
            LoopClock requiredClock = Objects.requireNonNull(
                    clock,
                    "Reference flywheel Task update clock is required");

            groupedMove.update(requiredClock);
            Status current = lastStatus;
            boolean groupedReached = groupedMove.isComplete()
                    && groupedMove.getOutcome() == TaskOutcome.SUCCESS;
            boolean independentReady = current != statusAtStart
                    && Double.compare(
                            current.requestedVelocityTicksPerSec(),
                            requestedVelocityTicksPerSec) == 0
                    && current.ready();

            // Exact-boundary readiness wins over timeout, like ScalarTasks feedback moves.
            if (groupedReached && independentReady) {
                finish(TaskOutcome.SUCCESS, false);
            } else if (Math.max(0.0, requiredClock.nowSec() - startedAtSec) >= timeoutSec) {
                finish(TaskOutcome.TIMEOUT, true);
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            CleanupActions.attemptAll(
                    groupedMove::cancel,
                    () -> flywheels.commandTarget().set(IDLE_VELOCITY_TICKS_PER_SEC));
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return complete
                    ? "ReferenceFlywheels.setVelocityTask(DONE:" + outcome + ")"
                    : "ReferenceFlywheels.setVelocityTask";
        }

        private void finish(TaskOutcome terminalOutcome, boolean cancelGroupedMove) {
            complete = true;
            outcome = terminalOutcome;
            if (cancelGroupedMove) {
                // The child deliberately leaves the persistent request unchanged on timeout.
                groupedMove.cancel();
            }
        }
    }

    /** Returns a complete copied configuration, also reused by the delegating launcher. */
    public static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(
                source,
                "ReferenceFlywheelMechanism.Config is required");
        Config c = new Config();
        c.leftMotorName = requireName(s.leftMotorName, "leftMotorName");
        c.leftMotorDirection = Objects.requireNonNull(
                s.leftMotorDirection,
                "leftMotorDirection");
        c.rightMotorName = requireName(s.rightMotorName, "rightMotorName");
        c.rightMotorDirection = Objects.requireNonNull(
                s.rightMotorDirection,
                "rightMotorDirection");
        if (c.leftMotorName.equals(c.rightMotorName)) {
            throw new IllegalArgumentException(
                    "leftMotorName and rightMotorName must identify different FTC devices after "
                            + "trimming; got effective key \"" + c.leftMotorName + "\"");
        }
        c.maximumVelocityTicksPerSec = requirePositive(
                s.maximumVelocityTicksPerSec,
                "maximumVelocityTicksPerSec");
        c.velocityToleranceTicksPerSec = requireNonnegative(
                s.velocityToleranceTicksPerSec,
                "velocityToleranceTicksPerSec");
        return c;
    }

    private static String requireName(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must be a non-blank FTC hardware name");
        }
        return value.trim();
    }

    private static double requirePositive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double requireNonnegative(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(field + " must be finite and >= 0, got " + value);
        }
        return value;
    }
}
