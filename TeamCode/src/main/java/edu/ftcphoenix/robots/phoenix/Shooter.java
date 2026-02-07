package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.control.DebounceBoolean;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.OutputTaskRunner;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Phoenix shooter + ball path subsystem.
 *
 * <p>Hardware for this robot:
 * <ul>
 *   <li>Intake wheels: 1 motor</li>
 *   <li>Intake transfer: 1 CR servo</li>
 *   <li>Shooter transfer: 2 CR servos (left/right)</li>
 *   <li>Shooter wheel: 1 motor (velocity control)</li>
 * </ul>
 *
 * <p>Design pattern:
 * <ul>
 *   <li><b>Single writer:</b> this class is the only place that commands the hardware plants.</li>
 *   <li><b>Selected velocity lives here:</b> driver D-pad and auto-aim both update the same
 *       {@link #selectedVelocityNative}.</li>
 *   <li><b>Flywheel enable is latched externally:</b> the supervisor toggles
 *       {@link #flywheelEnabled} (P2 right bumper).</li>
 *   <li><b>Automation is queued:</b> temporary feed overrides (shoot pulses) live in an
 *       {@link OutputTaskRunner} and override the manual feed path while active.</li>
 * </ul>
 */
public final class Shooter {

    // ---------------------------------------------------------------------
    // Calibration
    // ---------------------------------------------------------------------

    /**
     * Distance → flywheel velocity mapping.
     *
     * <p>Values are in inches → native velocity units. Tune for your robot.</p>
     */
    private static final InterpolatingTable1D SHOOTER_VELOCITY_TABLE = InterpolatingTable1D.ofSortedPairs(
            28.06, 1505.6,
            36.52, 1427.4,
            42.3, 1424.35,
            50.3, 1450,
            56.5, 1484.7,
            61.7, 1509.95,
            62.9, 1542.93,
            65.8, 1535.7,
            74.2, 1575,
            79.5, 1600,
            83.4, 1625,
            93.6, 1700,
            96.6, 1700,
            103.2, 1775,
            104.7, 1800,
            109.2, 1800,
            112, 1875,
            130, 1875
    );

    // ---------------------------------------------------------------------
    // Hardware plants
    // ---------------------------------------------------------------------

    private final Plant plantIntakeMotor;
    private final Plant plantIntakeTransfer;
    private final Plant plantShooterTransfer;
    private final Plant plantFlywheel;

    // We read measured velocity directly for the "ready" gate.
    private final DcMotorEx flywheelMotor;

    // ---------------------------------------------------------------------
    // Manual state (base outputs when no feed macro is active)
    // ---------------------------------------------------------------------

    private double manualIntakeMotorPower = 0.0;
    private double manualIntakeTransferPower = 0.0;
    private double manualShooterTransferPower = 0.0;

    // ---------------------------------------------------------------------
    // Flywheel state (selected velocity + enable)
    // ---------------------------------------------------------------------

    private boolean flywheelEnabled = false;
    private double selectedVelocityNative = RobotConfig.Shooter.velocityMin;

    /**
     * Feed path override queue.
     *
     * <p>While a feed macro is running, this output overrides the manual feed powers.</p>
     */
    private final OutputTaskRunner feedQueue = Tasks.outputQueue();

    /**
     * Debounced "ready" latch for the flywheel.
     */
    private final DebounceBoolean readyLatch =
            DebounceBoolean.onAfterOffImmediately(RobotConfig.Shooter.readyStableSec);

    /**
     * Flywheel-ready source (idempotent by cycle).
     *
     * <p>Important: this source updates {@link #readyLatch}. If we created a new BooleanSource
     * each time {@link #flywheelReady()} was called, multiple callers in the same loop could
     * accidentally advance the debounce timer more than once per cycle. This single, cached source
     * avoids that class of bug.</p>
     */
    private final BooleanSource flywheelReadySource;

    // Cached for telemetry/debug (computed in update())
    private double flywheelTargetNative = 0.0;
    private double flywheelMeasuredNative = 0.0;

    // Absolute value + simple accel estimate (computed in update())
    private double flywheelMeasuredAbs = 0.0;
    private double flywheelMeasuredAccelAbs = 0.0;
    private double prevFlywheelMeasuredAbs = 0.0;

    private final Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Keep a direct handle to the flywheel motor so we can read measured velocity.
        flywheelMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.Shooter.nameMotorShooterWheel);

        // Intake wheels (motor).
        plantIntakeMotor = Actuators.plant(hardwareMap)
                .motor(RobotConfig.Shooter.nameMotorIntake, RobotConfig.Shooter.directionMotorIntake)
                .power()
                .build();

        // Intake transfer (CR servo).
        plantIntakeTransfer = Actuators.plant(hardwareMap)
                .crServo(RobotConfig.Shooter.nameCrServoIntakeTransfer, RobotConfig.Shooter.directionCrServoIntakeTransfer)
                .power()
                .build();

        // Shooter transfer (two CR servos). Apply scaling/bias to the *last added* servo (left).
        plantShooterTransfer = Actuators.plant(hardwareMap)
                .crServo(RobotConfig.Shooter.nameCrServoShooterTransferRight, RobotConfig.Shooter.directionCrServoShooterTransferRight)
                .andCrServo(RobotConfig.Shooter.nameCrServoShooterTransferLeft, RobotConfig.Shooter.directionCrServoShooterTransferLeft)
                .scale(RobotConfig.Shooter.shooterTransferLeftScale)
                .bias(RobotConfig.Shooter.shooterTransferLeftBias)
                .power()
                .build();

        // Flywheel (single motor, velocity control).
        plantFlywheel = Actuators.plant(hardwareMap)
                .motor(RobotConfig.Shooter.nameMotorShooterWheel, RobotConfig.Shooter.directionMotorShooterWheel)
                .velocity(RobotConfig.Shooter.velocityToleranceNative)
                .build();

        // Ready gate source (safe to sample multiple times per loop).
        flywheelReadySource = new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;

                // If the flywheel is disabled, we're never "ready".
                if (!flywheelEnabled) {
                    last = false;
                    readyLatch.reset(false);
                    return false;
                }

                double target = Math.abs(flywheelTargetNative);
                double measured = flywheelMeasuredAbs;

                boolean enabled = target > 1e-6;

                // Asymmetric ready band:
                //  - allow being a bit low (underspeed) while recovering
                //  - be stricter on overspeed to avoid shooting while the wheel is still "springing back"
                double err = measured - target;
                boolean withinBand = enabled
                        && err >= -RobotConfig.Shooter.velocityToleranceBelowNative
                        && err <= RobotConfig.Shooter.velocityToleranceAboveNative;

                // Optional stability gate: don't call it ready while the speed is changing quickly.
                boolean accelOk = RobotConfig.Shooter.velocityMaxAccelNativePerSec2 <= 0.0
                        || flywheelMeasuredAccelAbs <= RobotConfig.Shooter.velocityMaxAccelNativePerSec2;

                boolean at = withinBand && accelOk;

                last = readyLatch.update(clock, at);
                return last;
            }

            @Override
            public void reset() {
                lastCycle = Long.MIN_VALUE;
                last = false;
                readyLatch.reset(false);
            }
        };
    }

    // ---------------------------------------------------------------------
    // Supervisor-facing API
    // ---------------------------------------------------------------------

    public OutputTaskRunner feedQueue() {
        return feedQueue;
    }

    public void clearFeedQueue() {
        feedQueue.clear();
    }

    public void setFlywheelEnabled(boolean enabled) {
        flywheelEnabled = enabled;
        if (!enabled) {
            // Prevent the ready latch from staying true after we turn the wheel off.
            flywheelReadySource.reset();
        }
    }

    public boolean flywheelEnabled() {
        return flywheelEnabled;
    }

    public void setSelectedVelocity(double velocityNative) {
        selectedVelocityNative = clampVelocity(velocityNative);
    }

    public double selectedVelocity() {
        return selectedVelocityNative;
    }

    public void increaseSelectedVelocity() {
        setSelectedVelocity(selectedVelocityNative + RobotConfig.Shooter.velocityIncrement);
    }

    public void decreaseSelectedVelocity() {
        setSelectedVelocity(selectedVelocityNative - RobotConfig.Shooter.velocityIncrement);
    }

    public double velocityForRangeInches(double rangeInches) {
        return clampVelocity(SHOOTER_VELOCITY_TABLE.interpolate(rangeInches));
    }

    /**
     * Debounced flywheel-ready signal.
     *
     * <p>We treat the flywheel as "ready" when its measured speed is inside an
     * <b>asymmetric band</b> around the target:</p>
     * <ul>
     *   <li>underspeed allowed: {@link RobotConfig.Shooter#velocityToleranceBelowNative}</li>
     *   <li>overspeed allowed: {@link RobotConfig.Shooter#velocityToleranceAboveNative}</li>
     * </ul>
     *
     * <p>Additionally, we can require the wheel to be "settled" by limiting
     * {@code |dV/dt|} using {@link RobotConfig.Shooter#velocityMaxAccelNativePerSec2}.</p>
     *
     * <p>This gate is intentionally a little forgiving (tolerance + debounce) so we don't
     * "wait for perfection" before feeding a ball.</p>
     */
    public BooleanSource flywheelReady() {
        return flywheelReadySource;
    }

    // ---------------------------------------------------------------------
    // Manual feed (base) API
    // ---------------------------------------------------------------------

    public void setManualIntakeMotorPower(double power) {
        manualIntakeMotorPower = clampPower(power);
    }

    public void setManualIntakeTransferPower(double power) {
        manualIntakeTransferPower = clampPower(power);
    }

    public void setManualShooterTransferPower(double power) {
        manualShooterTransferPower = clampPower(power);
    }

    // ---------------------------------------------------------------------
    // Loop update + safety
    // ---------------------------------------------------------------------

    public void update(LoopClock clock) {
        // 1) Compute + apply flywheel target BEFORE updating the feed queue.
        //    Feed tasks gate on flywheelReady(), which uses the latest target + measured value.
        flywheelTargetNative = flywheelEnabled ? selectedVelocityNative : 0.0;

        plantFlywheel.setTarget(flywheelTargetNative);
        plantFlywheel.update(clock.dtSec());

        // Cache measured velocity for readiness + telemetry.
        flywheelMeasuredNative = flywheelMotor.getVelocity();

        // Track abs + a simple |dV/dt| estimate so our ready gate can avoid feeding during
        // the "spring back" overshoot after a shot.
        flywheelMeasuredAbs = Math.abs(flywheelMeasuredNative);
        double dt = clock.dtSec();
        if (dt > 1e-6 && Double.isFinite(dt)) {
            flywheelMeasuredAccelAbs = Math.abs((flywheelMeasuredAbs - prevFlywheelMeasuredAbs) / dt);
        } else {
            flywheelMeasuredAccelAbs = 0.0;
        }
        prevFlywheelMeasuredAbs = flywheelMeasuredAbs;

        // 2) Update the feed macro queue.
        feedQueue.update(clock);

        boolean feedOverrideActive = feedQueue.hasActiveTask();
        double feedOverride = feedQueue.getAsDouble(clock);

        double intakeMotorTarget = feedOverrideActive
                ? feedOverride * RobotConfig.Shooter.feedScaleIntakeMotor
                : manualIntakeMotorPower;

        double intakeTransferTarget = feedOverrideActive
                ? feedOverride * RobotConfig.Shooter.feedScaleIntakeTransfer
                : manualIntakeTransferPower;

        double shooterTransferTarget = feedOverrideActive
                ? feedOverride * RobotConfig.Shooter.feedScaleShooterTransfer
                : manualShooterTransferPower;

        plantIntakeMotor.setTarget(intakeMotorTarget);
        plantIntakeTransfer.setTarget(intakeTransferTarget);
        plantShooterTransfer.setTarget(shooterTransferTarget);

        plantIntakeMotor.update(clock.dtSec());
        plantIntakeTransfer.update(clock.dtSec());
        plantShooterTransfer.update(clock.dtSec());
    }

    public void stop() {
        flywheelEnabled = false;

        manualIntakeMotorPower = 0.0;
        manualIntakeTransferPower = 0.0;
        manualShooterTransferPower = 0.0;

        feedQueue.clear();

        plantFlywheel.stop();
        plantIntakeMotor.stop();
        plantIntakeTransfer.stop();
        plantShooterTransfer.stop();
    }

    public void telemetryDump(LoopClock clock, String prefix) {
        if (telemetry == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "shooter" : prefix;

        telemetry.addData(p + ".flywheelEnabled", flywheelEnabled);
        telemetry.addData(p + ".selectedVel", selectedVelocityNative);
        telemetry.addData(p + ".flywheelTarget", flywheelTargetNative);
        telemetry.addData(p + ".flywheelMeasured", flywheelMeasuredNative);
        double err = flywheelMeasuredNative - flywheelTargetNative;
        telemetry.addData(p + ".flywheelErr", err);
        telemetry.addData(p + ".flywheelErrAbs", Math.abs(err));
        telemetry.addData(p + ".flywheelTol", RobotConfig.Shooter.velocityToleranceNative);
        telemetry.addData(p + ".flywheelTolBelow", RobotConfig.Shooter.velocityToleranceBelowNative);
        telemetry.addData(p + ".flywheelTolAbove", RobotConfig.Shooter.velocityToleranceAboveNative);
        telemetry.addData(p + ".flywheelAccelAbs", flywheelMeasuredAccelAbs);
        telemetry.addData(p + ".flywheelAccelMax", RobotConfig.Shooter.velocityMaxAccelNativePerSec2);
        telemetry.addData(p + ".flywheelAtSetpoint", plantFlywheel.atSetpoint());
        telemetry.addData(p + ".ready", flywheelReady().getAsBoolean(clock));

        telemetry.addData(p + ".feedBacklog", feedQueue.backlogCount());
        telemetry.addData(p + ".feedQueued", feedQueue.queuedCount());
        telemetry.addData(p + ".feedActive", feedQueue.hasActiveTask());
        telemetry.addData(p + ".feedOut", feedQueue.getAsDouble(clock));
    }

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------

    private static double clampPower(double p) {
        if (p > 1.0) return 1.0;
        if (p < -1.0) return -1.0;
        return p;
    }

    private static double clampVelocity(double v) {
        if (v > RobotConfig.Shooter.velocityMax) return RobotConfig.Shooter.velocityMax;
        if (v < RobotConfig.Shooter.velocityMin) return RobotConfig.Shooter.velocityMin;
        return v;
    }
}
