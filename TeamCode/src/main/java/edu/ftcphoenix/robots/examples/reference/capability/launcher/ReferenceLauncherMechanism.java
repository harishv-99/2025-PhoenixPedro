package edu.ftcphoenix.robots.examples.reference.capability.launcher;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetResolver;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.actuation.ScalarTasks;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcSensors;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.task.OutputTaskRunner;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;

/** Owns the reference launcher's Plants, sensor conditioning, feed queue, and safe task factories. */
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
        public double maximumVelocity;
        public double velocityTolerance;
        public double launchVelocity;
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
            c.maximumVelocity = 5000.0;
            c.velocityTolerance = 100.0;
            c.launchVelocity = 3000.0;
            c.spinUpTimeoutSec = 2.0;
            c.transferPower = 0.25;
            c.transferDurationSec = 0.20;
            c.releaseRetractedPosition = 0.25;
            c.releaseExtendedPosition = 0.60;
            c.releaseDurationSec = 0.15;
            return c;
        }
    }

    private static final double IDLE = 0.0;

    private final Plant flywheel;
    private final Plant transfer;
    private final Plant release;
    private final BooleanSource objectPresent;
    private final OutputTaskRunner transferOverrides = Tasks.outputQueue(IDLE);
    private final double maximumVelocity;
    private final double launchVelocity;
    private final double spinUpTimeoutSec;
    private final double transferPower;
    private final double transferDurationSec;
    private final double releaseRetractedPosition;
    private final double releaseExtendedPosition;
    private final double releaseDurationSec;
    private Status lastStatus = new Status(IDLE, Double.NaN, false, false, false);

    /** Constructs the complete mechanism after validating its copied configuration. */
    public ReferenceLauncherMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);
        BooleanSource builtObjectPresent = FtcSensors.digitalLow(map, c.objectSensorName)
                .debouncedOnOff(0.02, 0.02);

        PlantTargetResolver transferTarget = PlantTargets.overlay(
                        ScalarSource.of(() -> IDLE))
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
                    .bounded(IDLE, c.maximumVelocity)
                    .nativeUnits()
                    .velocityTolerance(c.velocityTolerance)
                    .targetFromNewCommand(IDLE)
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
        objectPresent = builtObjectPresent;
        maximumVelocity = c.maximumVelocity;
        launchVelocity = c.launchVelocity;
        spinUpTimeoutSec = c.spinUpTimeoutSec;
        transferPower = c.transferPower;
        transferDurationSec = c.transferDurationSec;
        releaseRetractedPosition = c.releaseRetractedPosition;
        releaseExtendedPosition = c.releaseExtendedPosition;
        releaseDurationSec = c.releaseDurationSec;
    }

    @Override
    public void setTargetVelocity(double velocity) {
        if (!Double.isFinite(velocity) || velocity < IDLE || velocity > maximumVelocity) {
            throw new IllegalArgumentException(
                    "velocity must be finite and in [0, " + maximumVelocity + "], got " + velocity);
        }
        flywheel.commandTarget().set(velocity);
    }

    @Override
    public void idle() {
        flywheel.commandTarget().set(IDLE);
        release.commandTarget().set(releaseRetractedPosition);
        transferOverrides.cancelAndClear();
    }

    @Override
    public void requestTransferPulse() {
        transferOverrides.enqueue(
                Tasks.outputForSeconds("referenceTransfer", transferPower, transferDurationSec));
    }

    @Override
    public Task launchOne() {
        Task waitForVelocity = Tasks.sequence(
                Tasks.runOnce(() -> setTargetVelocity(launchVelocity)),
                Tasks.waitUntil(BooleanSource.of(() -> flywheel.atTarget(launchVelocity)),
                        spinUpTimeoutSec));
        Task feed = Tasks.sequence(
                ScalarTasks.set(release.commandTarget(), releaseExtendedPosition)
                        .forSeconds(releaseDurationSec)
                        .then(releaseRetractedPosition)
                        .build(),
                Tasks.runOnce(this::requestTransferPulse),
                Tasks.waitForSeconds(transferDurationSec));
        return Tasks.branchOnOutcome(waitForVelocity, feed, Tasks.runOnce(this::idle));
    }

    @Override
    public Status status() {
        return lastStatus;
    }

    @Override
    public void update(LoopClock clock) {
        transferOverrides.update(clock);
        flywheel.update(clock);
        transfer.update(clock);
        release.update(clock);
        lastStatus = new Status(
                flywheel.commandTarget().get(),
                flywheel.getMeasurement(),
                flywheel.atTarget(),
                objectPresent.getAsBoolean(clock),
                transferOverrides.activeSource().getAsBoolean(clock));
    }

    @Override
    public void stop() {
        CleanupActions.attemptAll(
                transferOverrides::cancelAndClear,
                release::stop,
                transfer::stop,
                flywheel::stop);
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "ReferenceLauncherMechanism.Config is required");
        Config c = new Config();
        c.leftFlywheelName = requireName(s.leftFlywheelName, "leftFlywheelName");
        c.leftFlywheelDirection = Objects.requireNonNull(s.leftFlywheelDirection,
                "leftFlywheelDirection");
        c.rightFlywheelName = requireName(s.rightFlywheelName, "rightFlywheelName");
        c.rightFlywheelDirection = Objects.requireNonNull(s.rightFlywheelDirection,
                "rightFlywheelDirection");
        c.transferName = requireName(s.transferName, "transferName");
        c.transferDirection = Objects.requireNonNull(s.transferDirection, "transferDirection");
        c.releaseServoName = requireName(s.releaseServoName, "releaseServoName");
        c.releaseServoDirection = Objects.requireNonNull(s.releaseServoDirection,
                "releaseServoDirection");
        c.objectSensorName = requireName(s.objectSensorName, "objectSensorName");
        c.maximumVelocity = positive(s.maximumVelocity, "maximumVelocity");
        c.velocityTolerance = positive(s.velocityTolerance, "velocityTolerance");
        c.launchVelocity = positive(s.launchVelocity, "launchVelocity");
        if (c.launchVelocity > c.maximumVelocity) {
            throw new IllegalArgumentException("launchVelocity must be <= maximumVelocity");
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
