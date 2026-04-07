package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.control.DebounceBoolean;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.OutputTaskRunner;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Phoenix shooter + ball-path subsystem.
 *
 * <p>This class remains the single writer to the intake, transfer, and flywheel plants. Outside
 * code requests intent and reads {@link ShooterStatus}; it does not format FTC telemetry or command
 * the hardware directly.</p>
 */
public final class Shooter {

    private final PhoenixProfile.ShooterConfig cfg;

    private final Plant plantIntakeMotor;
    private final Plant plantIntakeTransfer;
    private final Plant plantShooterTransfer;
    private final Plant plantFlywheel;
    private final DcMotorEx flywheelMotor;

    private double manualIntakeMotorPower = 0.0;
    private double manualIntakeTransferPower = 0.0;
    private double manualShooterTransferPower = 0.0;

    private boolean flywheelEnabled = false;
    private double selectedVelocityNative;
    private String flywheelPidfWarning;

    private final OutputTaskRunner feedQueue = Tasks.outputQueue();
    private final DebounceBoolean readyLatch;
    private final BooleanSource flywheelReadySource;

    private double flywheelTargetNative = 0.0;
    private double flywheelMeasuredNative = 0.0;
    private double flywheelMeasuredAbs = 0.0;
    private double flywheelMeasuredAccel = 0.0;
    private double flywheelMeasuredAccelAbs = 0.0;
    private double prevFlywheelMeasuredAbs = 0.0;

    /**
     * Creates the Phoenix shooter subsystem and claims ownership of the shooter-path actuators.
     *
     * @param hardwareMap FTC hardware map used to resolve the configured devices
     * @param config shooter configuration snapshot; copied on construction so callers can keep
     *               editing profile objects without mutating the live subsystem
     */
    public Shooter(HardwareMap hardwareMap, PhoenixProfile.ShooterConfig config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.selectedVelocityNative = cfg.velocityMin;
        this.readyLatch = DebounceBoolean.onAfterOffImmediately(cfg.readyStableSec);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, cfg.nameMotorShooterWheel);
        if (cfg.applyFlywheelVelocityPIDF) {
            try {
                flywheelMotor.setVelocityPIDFCoefficients(
                        cfg.flywheelVelKp,
                        cfg.flywheelVelKi,
                        cfg.flywheelVelKd,
                        cfg.flywheelVelKf
                );
            } catch (RuntimeException e) {
                flywheelPidfWarning = "flywheel PIDF not applied: " + e.getMessage();
            }
        }

        plantIntakeMotor = Actuators.plant(hardwareMap)
                .motor(cfg.nameMotorIntake, cfg.directionMotorIntake)
                .power()
                .build();

        plantIntakeTransfer = Actuators.plant(hardwareMap)
                .crServo(cfg.nameCrServoIntakeTransfer, cfg.directionCrServoIntakeTransfer)
                .power()
                .build();

        plantShooterTransfer = Actuators.plant(hardwareMap)
                .crServo(cfg.nameCrServoShooterTransferRight, cfg.directionCrServoShooterTransferRight)
                .andCrServo(cfg.nameCrServoShooterTransferLeft, cfg.directionCrServoShooterTransferLeft)
                .scale(cfg.shooterTransferLeftScale)
                .bias(cfg.shooterTransferLeftBias)
                .power()
                .build();

        plantFlywheel = Actuators.plant(hardwareMap)
                .motor(cfg.nameMotorShooterWheel, cfg.directionMotorShooterWheel)
                .velocity(cfg.velocityToleranceNative)
                .build();

        flywheelReadySource = new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            /**
             * Evaluates flywheel readiness at most once per loop cycle.
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;

                if (!flywheelEnabled) {
                    last = false;
                    readyLatch.reset(false);
                    return false;
                }

                double target = Math.abs(flywheelTargetNative);
                double measured = flywheelMeasuredAbs;
                boolean enabled = target > 1e-6;

                double leadSec = Math.max(0.0, cfg.readyPredictLeadSec);
                double predicted = measured + flywheelMeasuredAccel * leadSec;
                if (predicted < 0.0) {
                    predicted = 0.0;
                }

                double errAtContact = predicted - target;
                boolean withinBand = enabled
                        && errAtContact >= -cfg.velocityToleranceBelowNative
                        && errAtContact <= cfg.velocityToleranceAboveNative;

                last = readyLatch.update(clock, withinBand);
                return last;
            }

            /**
             * Clears cached readiness state when the flywheel is disabled.
             */
            @Override
            public void reset() {
                lastCycle = Long.MIN_VALUE;
                last = false;
                readyLatch.reset(false);
            }
        };
    }

    /**
     * Returns the subsystem-owned feed queue used by the supervisor to schedule shot pulses.
     *
     * <p>This remains public for the current stage so the existing supervisor can coordinate with the
     * queue, but callers should treat it as a narrow integration seam rather than a general-purpose
     * escape hatch.</p>
     *
     * @return subsystem-owned feed task runner
     */
    public OutputTaskRunner feedQueue() {
        return feedQueue;
    }

    /**
     * Cancels the active feed task and clears any queued follow-up feed tasks.
     */
    public void clearFeedQueue() {
        feedQueue.cancelAndClear();
    }

    /**
     * Enables or disables the flywheel target.
     *
     * @param enabled {@code true} to command the selected velocity, {@code false} to spin down
     */
    public void setFlywheelEnabled(boolean enabled) {
        flywheelEnabled = enabled;
        if (!enabled) {
            flywheelReadySource.reset();
        }
    }

    /**
     * Returns whether the flywheel is currently enabled.
     *
     * @return {@code true} when the subsystem is commanding a nonzero flywheel target
     */
    public boolean flywheelEnabled() {
        return flywheelEnabled;
    }

    /**
     * Sets the operator-selected flywheel velocity target.
     *
     * @param velocityNative requested target in motor native velocity units; clamped to the
     *                       configured minimum and maximum range
     */
    public void setSelectedVelocity(double velocityNative) {
        selectedVelocityNative = clampVelocity(velocityNative);
    }

    /**
     * Returns the operator-selected flywheel velocity target.
     *
     * @return selected flywheel target in motor native velocity units
     */
    public double selectedVelocity() {
        return selectedVelocityNative;
    }

    /**
     * Increases the selected flywheel velocity by one configured increment.
     */
    public void increaseSelectedVelocity() {
        setSelectedVelocity(selectedVelocityNative + cfg.velocityIncrement);
    }

    /**
     * Decreases the selected flywheel velocity by one configured increment.
     */
    public void decreaseSelectedVelocity() {
        setSelectedVelocity(selectedVelocityNative - cfg.velocityIncrement);
    }

    /**
     * Looks up a flywheel velocity recommendation for a measured camera range.
     *
     * @param rangeInches measured target range in inches
     * @return interpolated and clamped flywheel velocity in motor native units
     */
    public double velocityForRangeInches(double rangeInches) {
        return clampVelocity(cfg.velocityTable.interpolate(rangeInches));
    }

    /**
     * Returns the memoized flywheel-ready gate for the current loop.
     *
     * @return boolean source that predicts whether the flywheel will be within its ready band when a
     *         ring reaches the shooter
     */
    public BooleanSource flywheelReady() {
        return flywheelReadySource;
    }

    /**
     * Sets the manual intake-motor power used when no queued feed task is overriding the path.
     *
     * @param power desired open-loop power in the range [-1, 1]
     */
    public void setManualIntakeMotorPower(double power) {
        manualIntakeMotorPower = clampPower(power);
    }

    /**
     * Sets the manual intake-transfer power used when no queued feed task is overriding the path.
     *
     * @param power desired open-loop power in the range [-1, 1]
     */
    public void setManualIntakeTransferPower(double power) {
        manualIntakeTransferPower = clampPower(power);
    }

    /**
     * Sets the manual shooter-transfer power used when no queued feed task is overriding the path.
     *
     * @param power desired open-loop power in the range [-1, 1]
     */
    public void setManualShooterTransferPower(double power) {
        manualShooterTransferPower = clampPower(power);
    }

    /**
     * Updates flywheel control, queued feed tasks, and all actuator outputs for the loop.
     *
     * @param clock shared loop clock for the current OpMode cycle
     */
    public void update(LoopClock clock) {
        flywheelTargetNative = flywheelEnabled ? selectedVelocityNative : 0.0;
        plantFlywheel.setTarget(flywheelTargetNative);
        plantFlywheel.update(clock.dtSec());

        flywheelMeasuredNative = flywheelMotor.getVelocity();
        flywheelMeasuredAbs = Math.abs(flywheelMeasuredNative);

        double dt = clock.dtSec();
        if (dt > 1e-6 && Double.isFinite(dt)) {
            flywheelMeasuredAccel = (flywheelMeasuredAbs - prevFlywheelMeasuredAbs) / dt;
            flywheelMeasuredAccelAbs = Math.abs(flywheelMeasuredAccel);
        } else {
            flywheelMeasuredAccel = 0.0;
            flywheelMeasuredAccelAbs = 0.0;
        }
        prevFlywheelMeasuredAbs = flywheelMeasuredAbs;

        feedQueue.update(clock);

        boolean feedOverrideActive = feedQueue.hasActiveTask();
        double feedOverride = feedQueue.getAsDouble(clock);

        double intakeMotorTarget = feedOverrideActive
                ? feedOverride * cfg.feedScaleIntakeMotor
                : manualIntakeMotorPower;

        double intakeTransferTarget = feedOverrideActive
                ? feedOverride * cfg.feedScaleIntakeTransfer
                : manualIntakeTransferPower;

        double shooterTransferTarget = feedOverrideActive
                ? feedOverride * cfg.feedScaleShooterTransfer
                : manualShooterTransferPower;

        plantIntakeMotor.setTarget(intakeMotorTarget);
        plantIntakeTransfer.setTarget(intakeTransferTarget);
        plantShooterTransfer.setTarget(shooterTransferTarget);

        plantIntakeMotor.update(clock.dtSec());
        plantIntakeTransfer.update(clock.dtSec());
        plantShooterTransfer.update(clock.dtSec());
    }

    /**
     * Returns a compact status snapshot for telemetry and higher-level coordination.
     *
     * @param clock shared loop clock for the current OpMode cycle
     * @return immutable shooter status snapshot describing the current commanded and measured state
     */
    public ShooterStatus status(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");

        double err = flywheelMeasuredNative - flywheelTargetNative;
        double targetAbs = Math.abs(flywheelTargetNative);
        double leadSec = Math.max(0.0, cfg.readyPredictLeadSec);
        double predictedAbs = flywheelMeasuredAbs + flywheelMeasuredAccel * leadSec;
        if (predictedAbs < 0.0) {
            predictedAbs = 0.0;
        }

        return new ShooterStatus(
                flywheelEnabled,
                cfg.applyFlywheelVelocityPIDF,
                flywheelPidfWarning,
                selectedVelocityNative,
                flywheelTargetNative,
                flywheelMeasuredNative,
                err,
                Math.abs(err),
                cfg.velocityToleranceNative,
                cfg.velocityToleranceBelowNative,
                cfg.velocityToleranceAboveNative,
                flywheelMeasuredAccel,
                flywheelMeasuredAccelAbs,
                leadSec,
                predictedAbs,
                predictedAbs - targetAbs,
                plantFlywheel.atSetpoint(),
                flywheelReady().getAsBoolean(clock),
                feedQueue.backlogCount(),
                feedQueue.queuedCount(),
                feedQueue.hasActiveTask(),
                feedQueue.getAsDouble(clock)
        );
    }

    /**
     * Stops all outputs and clears transient loop state.
     */
    public void stop() {
        flywheelEnabled = false;
        manualIntakeMotorPower = 0.0;
        manualIntakeTransferPower = 0.0;
        manualShooterTransferPower = 0.0;
        feedQueue.cancelAndClear();
        plantFlywheel.stop();
        plantIntakeMotor.stop();
        plantIntakeTransfer.stop();
        plantShooterTransfer.stop();
    }

    private static double clampPower(double power) {
        if (power > 1.0) return 1.0;
        if (power < -1.0) return -1.0;
        return power;
    }

    private double clampVelocity(double velocityNative) {
        if (velocityNative > cfg.velocityMax) return cfg.velocityMax;
        if (velocityNative < cfg.velocityMin) return cfg.velocityMin;
        return velocityNative;
    }
}
