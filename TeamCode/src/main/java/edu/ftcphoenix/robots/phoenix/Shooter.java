package edu.ftcphoenix.robots.phoenix;

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
 * <p>This subsystem demonstrates the recommended pattern:
 * <ul>
 *   <li><b>Single writer:</b> this class is the only place that calls {@link Plant#setTarget(double)}.</li>
 *   <li><b>Manual intent is state:</b> variables such as "manualShooterEnabled".</li>
 *   <li><b>Automation is queued:</b> temporary overrides (shoot-all feed pulses) live in an
 *       {@link OutputTaskRunner} and override the manual feed path while active.</li>
 * </ul>
 *
 * <p>The corresponding {@link ShooterSupervisor} owns the policy and macro logic.
 */
public final class Shooter {

    // ---------------------------------------------------------------------
    // Calibration
    // ---------------------------------------------------------------------

    /**
     * Distance → flywheel velocity mapping for Phoenix.
     *
     * <p>Values are in inches → native velocity units. These are placeholders; tune for your robot.</p>
     */
    private static final InterpolatingTable1D SHOOTER_VELOCITY_TABLE = InterpolatingTable1D.ofSortedPairs(
            24, 1600,
            36, 1700,
            48, 1750,
            60, 1800,
            72, 1875
    );

    // ---------------------------------------------------------------------
    // Hardware plants
    // ---------------------------------------------------------------------

    private final Plant plantIntakeTransfer;
    private final Plant plantTransferMotor;
    private final Plant plantShooterTransfer;
    private final Plant plantFlywheel;

    // ---------------------------------------------------------------------
    // Manual state (backup controls)
    // ---------------------------------------------------------------------

    private double manualIntakeTransferPower = 0.0;
    private double manualTransferMotorPower = 0.0;
    private double manualShooterTransferPower = 0.0;

    private boolean manualShooterEnabled = false;
    private double manualShooterVelocity = RobotConfig.Shooter.velocityMin;

    // ---------------------------------------------------------------------
    // Macro state (set by ShooterSupervisor)
    // ---------------------------------------------------------------------

    private boolean macroShooterEnabled = false;
    private double macroShooterVelocity = RobotConfig.Shooter.velocityMin;

    /**
     * Feed path override queue.
     *
     * <p>When a feed macro is running, this output overrides the manual feed powers.</p>
     */
    private final OutputTaskRunner feedQueue = Tasks.outputQueue(0.0);

    /**
     * Debounced "ready" latch for the flywheel.
     */
    private final DebounceBoolean readyLatch = DebounceBoolean.onAfterOffImmediately(RobotConfig.Shooter.readyStableSec);

    // Cached for telemetry/debug (computed in update())
    private double flywheelTargetNative = 0.0;

    private final Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // --- Feed path ---
        plantIntakeTransfer = Actuators.plant(hardwareMap)
                .crServo(RobotConfig.Shooter.nameCrServoIntakeTransfer, RobotConfig.Shooter.directionCrServoIntakeTransfer)
                .power()
                .build();

        plantTransferMotor = Actuators.plant(hardwareMap)
                .motor(RobotConfig.Shooter.nameMotorTransfer, RobotConfig.Shooter.directionMotorTransfer)
                .power()
                .build();

        plantShooterTransfer = Actuators.plant(hardwareMap)
                .crServo(RobotConfig.Shooter.nameCrServoShooterTransfer, RobotConfig.Shooter.directionCrServoShooterTransfer)
                .power()
                .build();

        // --- Flywheel ---
        plantFlywheel = Actuators.plant(hardwareMap)
                .motor(RobotConfig.Shooter.nameMotorShooterLeft, RobotConfig.Shooter.directionMotorShooterLeft)
                .andMotor(RobotConfig.Shooter.nameMotorShooterRight, RobotConfig.Shooter.directionMotorShooterRight)
                .velocity(RobotConfig.Shooter.velocityToleranceNative)
                .build();
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

    public void setMacroShooterEnabled(boolean enabled) {
        macroShooterEnabled = enabled;
        if (!enabled) {
            // Reset macro velocity to something sane so it doesn't hold stale values.
            macroShooterVelocity = manualShooterVelocity;
        }
    }

    public boolean macroShooterEnabled() {
        return macroShooterEnabled;
    }

    public void setMacroShooterVelocity(double velocityNative) {
        macroShooterVelocity = clampVelocity(velocityNative);
    }

    public double velocityForRangeInches(double rangeInches) {
        return clampVelocity(SHOOTER_VELOCITY_TABLE.interpolate(rangeInches));
    }

    /**
     * Debounced flywheel-ready signal.
     *
     * <p>This is designed to be used as a gate in {@link edu.ftcphoenix.fw.task.GatedOutputUntilTask}.
     * The returned source is safe to sample multiple times per loop.</p>
     */
    public BooleanSource flywheelReady() {
        return new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                boolean enabled = Math.abs(flywheelTargetNative) > 1e-6;
                boolean at = enabled && plantFlywheel.atSetpoint();
                return readyLatch.update(clock, at);
            }
        };
    }

    // ---------------------------------------------------------------------
    // Manual (backup) API
    // ---------------------------------------------------------------------

    public void setManualIntakeTransferPower(double power) {
        manualIntakeTransferPower = clampPower(power);
    }

    public void setManualTransferMotorPower(double power) {
        manualTransferMotorPower = clampPower(power);
    }

    public void setManualShooterTransferPower(double power) {
        manualShooterTransferPower = clampPower(power);
    }

    public void setManualShooterEnabled(boolean enabled) {
        manualShooterEnabled = enabled;
    }

    public boolean manualShooterEnabled() {
        return manualShooterEnabled;
    }

    public void increaseManualVelocity() {
        setManualShooterVelocity(manualShooterVelocity + RobotConfig.Shooter.velocityIncrement);
    }

    public void decreaseManualVelocity() {
        setManualShooterVelocity(manualShooterVelocity - RobotConfig.Shooter.velocityIncrement);
    }

    public void setManualShooterVelocity(double velocityNative) {
        manualShooterVelocity = clampVelocity(velocityNative);
    }

    public double manualShooterVelocity() {
        return manualShooterVelocity;
    }

    // ---------------------------------------------------------------------
    // Loop update + safety
    // ---------------------------------------------------------------------

    public void update(LoopClock clock) {
        // 1) Compute + apply flywheel target BEFORE updating the feed queue.
        //    Feed tasks use plantFlywheel.atSetpoint(), which is relative to the latest setTarget.
        flywheelTargetNative = macroShooterEnabled
                ? macroShooterVelocity
                : (manualShooterEnabled ? manualShooterVelocity : 0.0);

        plantFlywheel.setTarget(flywheelTargetNative);
        plantFlywheel.update(clock.dtSec());

        // 2) Update the feed macro queue.
        feedQueue.update(clock);

        boolean feedOverrideActive = feedQueue.hasActiveTask();
        double feedOverride = feedQueue.getAsDouble(clock);

        double intakeTarget = feedOverrideActive ? feedOverride : manualIntakeTransferPower;
        double transferTarget = feedOverrideActive ? feedOverride : manualTransferMotorPower;
        double shooterTransferTarget = feedOverrideActive ? feedOverride : manualShooterTransferPower;

        plantIntakeTransfer.setTarget(intakeTarget);
        plantTransferMotor.setTarget(transferTarget);
        plantShooterTransfer.setTarget(shooterTransferTarget);

        plantIntakeTransfer.update(clock.dtSec());
        plantTransferMotor.update(clock.dtSec());
        plantShooterTransfer.update(clock.dtSec());
    }

    public void stop() {
        macroShooterEnabled = false;
        manualShooterEnabled = false;

        manualIntakeTransferPower = 0.0;
        manualTransferMotorPower = 0.0;
        manualShooterTransferPower = 0.0;

        feedQueue.clear();

        plantFlywheel.stop();
        plantIntakeTransfer.stop();
        plantTransferMotor.stop();
        plantShooterTransfer.stop();
    }

    public void telemetryDump(LoopClock clock, String prefix) {
        if (telemetry == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "shooter" : prefix;

        telemetry.addData(p + ".flywheelTarget", flywheelTargetNative);
        telemetry.addData(p + ".flywheelAtSetpoint", plantFlywheel.atSetpoint());
        telemetry.addData(p + ".ready", flywheelReady().getAsBoolean(clock));
        telemetry.addData(p + ".macroEnabled", macroShooterEnabled);
        telemetry.addData(p + ".manualEnabled", manualShooterEnabled);
        telemetry.addData(p + ".feedBacklog", feedQueue.backlogCount());
        telemetry.addData(p + ".feedQueued", feedQueue.queuedCount());
        telemetry.addData(p + ".feedActive", feedQueue.hasActiveTask());
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
