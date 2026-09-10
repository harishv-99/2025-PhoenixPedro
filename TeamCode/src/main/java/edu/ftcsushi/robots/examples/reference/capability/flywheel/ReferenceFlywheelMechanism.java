package edu.ftcsushi.robots.examples.reference.capability.flywheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.AbstractTask;
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
    private long requestId;
    private LoopClock ownerClock;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean updating;
    private boolean stopped;
    private RuntimeException updateFailure;

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
                c.velocityToleranceTicksPerSec,
                LoopTimestamp.unavailable(), -1, requestId);
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
        if (stopped) {
            throw new IllegalStateException("Reference flywheels are stopped; construct a fresh owner.");
        }
        if (requestId == Long.MAX_VALUE) {
            throw new IllegalStateException("Reference flywheel request identity exhausted; "
                    + "construct a fresh owner.");
        }
        flywheels.commandTarget().set(velocityTicksPerSec);
        requestId++;
    }

    /** {@inheritDoc} */
    @Override
    public long requestId() {
        return requestId;
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
     * samples. Repeated successful same-cycle calls leave the exact publication unchanged. An
     * effectful failure or reentrant update is retained and rethrown without retrying hardware;
     * the preceding complete Status remains historical. The timestamp names software sampling,
     * not native encoder acquisition. A request changed during sampling cannot acquire that sample.
     */
    @Override
    public void update(LoopClock clock) {
        checkUpdateFailure();
        if (stopped) {
            return;
        }
        if (updating) {
            updateFailure = new IllegalStateException(
                    "Reference flywheel update must not be called reentrantly.");
            throw updateFailure;
        }
        Objects.requireNonNull(clock, "clock is required");
        LoopTimestamp sampledAt = clock.nowTimestamp();
        if (ownerClock != null && ownerClock != clock) {
            throw new IllegalArgumentException("Reference flywheels require their one owner LoopClock.");
        }
        ownerClock = clock;
        if (lastUpdateCycle == clock.cycle()) {
            return;
        }
        long sampledCycle = clock.cycle();
        lastUpdateCycle = sampledCycle;
        long sampledRequestId = requestId;
        updating = true;
        try {
            flywheels.update(clock);
            checkUpdateFailure();
            if (stopped) {
                return;
            }
            PlantSnapshot snapshot = flywheels.snapshot();
            double leftVelocity = leftMeasuredVelocityTicksPerSec.getAsDouble(clock);
            checkUpdateFailure();
            if (stopped) {
                return;
            }
            double rightVelocity = rightMeasuredVelocityTicksPerSec.getAsDouble(clock);
            checkUpdateFailure();
            if (stopped || sampledRequestId != requestId) {
                return;
            }
            if (sampledCycle != clock.cycle()) {
                throw new IllegalStateException("Do not advance the LoopClock during flywheel update.");
            }
            lastStatus = new Status(snapshot, leftVelocity, rightVelocity,
                    velocityToleranceTicksPerSec, sampledAt, sampledCycle, sampledRequestId);
        } catch (RuntimeException failure) {
            if (updateFailure == null) {
                updateFailure = failure;
            } else if (updateFailure != failure) {
                updateFailure.addSuppressed(failure);
            }
            throw updateFailure;
        } finally {
            updating = false;
        }
    }

    /**
     * Terminally stops the grouped Plant and withdraws sample availability even when stopping
     * fails. Member measurements are retained only as historical diagnostics, never ready evidence.
     */
    @Override
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        Status prior = lastStatus;
        // Withdraw readiness before any vendor stop callback can inspect the owner.
        lastStatus = new Status(prior.plantSnapshot(), prior.leftMeasuredVelocityTicksPerSec(),
                prior.rightMeasuredVelocityTicksPerSec(), velocityToleranceTicksPerSec,
                LoopTimestamp.unavailable(), -1, requestId);
        try {
            flywheels.stop();
        } finally {
            lastStatus = new Status(flywheels.snapshot(),
                    prior.leftMeasuredVelocityTicksPerSec(),
                    prior.rightMeasuredVelocityTicksPerSec(), velocityToleranceTicksPerSec,
                    LoopTimestamp.unavailable(), -1, requestId);
        }
    }

    /** Retains the first uncertain effect instead of retrying it on another output heartbeat. */
    private void checkUpdateFailure() {
        if (updateFailure != null) {
            throw updateFailure;
        }
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
     * Waits on the capability's complete grouped and independent-member evidence.
     *
     * <p>The generic move may observe a misleading grouped mean when one wheel is high and the
     * other low. This owner-specific wait therefore checks both members and the exact request
     * occurrence. AbstractTask supplies the shared lifecycle; no second raw-target Task bypasses
     * the mechanism's request owner.</p>
     */
    private final class PairedReadyTask extends AbstractTask {
        private final double requestedVelocityTicksPerSec;
        private final double timeoutSec;
        private LoopTimestamp startedAt;
        private long ownedRequestId;

        private PairedReadyTask(double requestedVelocityTicksPerSec, double timeoutSec) {
            super("ReferenceFlywheels.setVelocityTask");
            this.requestedVelocityTicksPerSec = requestedVelocityTicksPerSec;
            this.timeoutSec = timeoutSec;
        }

        @Override
        protected void onStart(LoopClock clock) {
            startedAt = clock.nowTimestamp();
            setVelocityTicksPerSec(requestedVelocityTicksPerSec);
            ownedRequestId = requestId;
        }

        @Override
        protected void onUpdate(LoopClock clock) {
            checkUpdateFailure();
            double elapsedSec = startedAt.ageSec(clock);
            if (!Double.isFinite(elapsedSec)) {
                // A new clock epoch cannot extend this old bounded request or reuse its evidence.
                cancel();
                return;
            }
            Status current = lastStatus;
            boolean independentReady = requestId == ownedRequestId
                    && current.requestId() == ownedRequestId
                    && Double.isFinite(current.sampledAt().ageSec(clock))
                    && Double.compare(
                            current.requestedVelocityTicksPerSec(),
                            requestedVelocityTicksPerSec) == 0
                    && current.plantSnapshot().atCommandTarget()
                    && current.ready();

            // Exact-boundary readiness wins over timeout, like ScalarTasks feedback moves.
            if (independentReady) {
                complete(TaskOutcome.SUCCESS);
            } else if (elapsedSec >= timeoutSec) {
                complete(TaskOutcome.TIMEOUT);
            }
        }

        @Override
        protected void onCancel() {
            if (!stopped) {
                setVelocityTicksPerSec(IDLE_VELOCITY_TICKS_PER_SEC);
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
