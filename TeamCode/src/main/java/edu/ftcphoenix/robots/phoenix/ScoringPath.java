package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.control.DebounceBoolean;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.task.OutputTask;
import edu.ftcphoenix.fw.task.OutputTaskRunner;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Phoenix scoring path: intake, transfer, flywheel, and scoring policy in one owner.
 *
 * <p>This class is the single writer to the scoring-path plants. It also owns the small amount of
 * scoring policy that used to live in a separate policy object: intake/eject/shoot priority, queued
 * shot requests, flywheel enable state, and shot-feed pulses. Keeping those pieces together makes
 * the student-facing object graph smaller without allowing multiple writers to fight over hardware.</p>
 */
public final class ScoringPath implements PhoenixCapabilities.Scoring {

    private enum FeedMode {
        IDLE("IDLE"),
        INTAKE("INTAKE"),
        EJECT("EJECT"),
        SHOOT("SHOOT");

        final String debugName;

        FeedMode(String debugName) {
            this.debugName = debugName;
        }
    }

    /**
     * Immutable scoring-path snapshot for telemetry, drive assists, and Auto coordination.
     */
    public static final class Status {
        public final boolean intakeEnabled;
        public final boolean ejectRequested;
        public final boolean shootingRequested;
        public final boolean flywheelRequested;
        public final boolean shootActive;
        public final int feedBacklog;
        public final String mode;

        public final boolean flywheelEnabled;
        public final boolean pidfEnabled;
        public final String pidfWarning;
        public final double selectedVelocityNative;
        public final double flywheelTargetNative;
        public final double flywheelMeasuredNative;
        public final double flywheelErrorNative;
        public final double flywheelErrorAbsNative;
        public final double flywheelToleranceNative;
        public final double flywheelToleranceBelowNative;
        public final double flywheelToleranceAboveNative;
        public final double flywheelAccelNativePerSec;
        public final double flywheelAccelAbsNativePerSec;
        public final double readyLeadSec;
        public final double predictedFlywheelAbsNative;
        public final double predictedFlywheelErrorNative;
        public final boolean flywheelAtSetpoint;
        public final boolean ready;
        public final int feedQueued;
        public final boolean feedActive;
        public final double feedOutput;

        /**
         * Creates a complete immutable scoring-path status snapshot.
         */
        public Status(boolean intakeEnabled,
                      boolean ejectRequested,
                      boolean shootingRequested,
                      boolean flywheelRequested,
                      boolean shootActive,
                      int feedBacklog,
                      String mode,
                      boolean flywheelEnabled,
                      boolean pidfEnabled,
                      String pidfWarning,
                      double selectedVelocityNative,
                      double flywheelTargetNative,
                      double flywheelMeasuredNative,
                      double flywheelErrorNative,
                      double flywheelErrorAbsNative,
                      double flywheelToleranceNative,
                      double flywheelToleranceBelowNative,
                      double flywheelToleranceAboveNative,
                      double flywheelAccelNativePerSec,
                      double flywheelAccelAbsNativePerSec,
                      double readyLeadSec,
                      double predictedFlywheelAbsNative,
                      double predictedFlywheelErrorNative,
                      boolean flywheelAtSetpoint,
                      boolean ready,
                      int feedQueued,
                      boolean feedActive,
                      double feedOutput) {
            this.intakeEnabled = intakeEnabled;
            this.ejectRequested = ejectRequested;
            this.shootingRequested = shootingRequested;
            this.flywheelRequested = flywheelRequested;
            this.shootActive = shootActive;
            this.feedBacklog = feedBacklog;
            this.mode = mode != null ? mode : FeedMode.IDLE.debugName;
            this.flywheelEnabled = flywheelEnabled;
            this.pidfEnabled = pidfEnabled;
            this.pidfWarning = pidfWarning;
            this.selectedVelocityNative = selectedVelocityNative;
            this.flywheelTargetNative = flywheelTargetNative;
            this.flywheelMeasuredNative = flywheelMeasuredNative;
            this.flywheelErrorNative = flywheelErrorNative;
            this.flywheelErrorAbsNative = flywheelErrorAbsNative;
            this.flywheelToleranceNative = flywheelToleranceNative;
            this.flywheelToleranceBelowNative = flywheelToleranceBelowNative;
            this.flywheelToleranceAboveNative = flywheelToleranceAboveNative;
            this.flywheelAccelNativePerSec = flywheelAccelNativePerSec;
            this.flywheelAccelAbsNativePerSec = flywheelAccelAbsNativePerSec;
            this.readyLeadSec = readyLeadSec;
            this.predictedFlywheelAbsNative = predictedFlywheelAbsNative;
            this.predictedFlywheelErrorNative = predictedFlywheelErrorNative;
            this.flywheelAtSetpoint = flywheelAtSetpoint;
            this.ready = ready;
            this.feedQueued = feedQueued;
            this.feedActive = feedActive;
            this.feedOutput = feedOutput;
        }
    }

    private final PhoenixProfile.ScoringPathConfig cfg;
    private final LoopClock clock;
    private final ScoringTargeting targeting;
    private final BooleanSource aimOkToShoot;
    private final BooleanSource shootOverride;

    private final Plant plantIntakeMotor;
    private final Plant plantIntakeTransfer;
    private final Plant plantShooterTransfer;
    private final Plant plantFlywheel;

    private final OutputTaskRunner feedQueue = Tasks.outputQueue();
    private final DebounceBoolean readyLatch;
    private final BooleanSource flywheelReadySource;
    private final String flywheelPidfWarning;

    private boolean intakeEnabled = false;
    private boolean ejectRequested = false;
    private boolean shootingRequested = false;
    private boolean flywheelRequested = false;
    private boolean flywheelEnabled = false;
    private int queuedShotRequests = 0;
    private FeedMode lastFeedMode = FeedMode.IDLE;

    private double manualIntakeMotorPower = 0.0;
    private double manualIntakeTransferPower = 0.0;
    private double manualShooterTransferPower = 0.0;
    private double selectedVelocityNative;

    private double flywheelTargetNative = 0.0;
    private double flywheelMeasuredNative = 0.0;
    private double flywheelMeasuredAbs = 0.0;
    private double flywheelMeasuredAccel = 0.0;
    private double flywheelMeasuredAccelAbs = 0.0;
    private double prevFlywheelMeasuredAbs = 0.0;

    private Status lastStatus;

    /**
     * Creates the Phoenix scoring path and claims ownership of all scoring-path actuators.
     *
     * @param hardwareMap FTC hardware map used to resolve the configured devices
     * @param config      scoring-path configuration snapshot
     * @param targeting   shared targeting service used for aim gates and shot-velocity suggestions
     * @param clock       shared loop clock used when capturing the current targeting suggestion
     */
    public ScoringPath(HardwareMap hardwareMap,
                       PhoenixProfile.ScoringPathConfig config,
                       ScoringTargeting targeting,
                       LoopClock clock) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.targeting = Objects.requireNonNull(targeting, "targeting");
        this.clock = Objects.requireNonNull(clock, "clock");
        this.aimOkToShoot = targeting.aimOkToShootSource().memoized();
        this.shootOverride = targeting.aimOverrideSource().memoized();
        this.selectedVelocityNative = cfg.velocityMin;
        this.readyLatch = DebounceBoolean.onAfterOffImmediately(cfg.readyStableSec);
        this.flywheelPidfWarning = null;

        plantIntakeMotor = FtcActuators.plant(hardwareMap)
                .motor(cfg.nameMotorIntake, cfg.directionMotorIntake)
                .power()
                .build();

        plantIntakeTransfer = FtcActuators.plant(hardwareMap)
                .crServo(cfg.nameCrServoIntakeTransfer, cfg.directionCrServoIntakeTransfer)
                .power()
                .build();

        plantShooterTransfer = FtcActuators.plant(hardwareMap)
                .crServo(cfg.nameCrServoShooterTransferRight, cfg.directionCrServoShooterTransferRight)
                .andCrServo(cfg.nameCrServoShooterTransferLeft, cfg.directionCrServoShooterTransferLeft)
                .scale(cfg.shooterTransferLeftScale)
                .bias(cfg.shooterTransferLeftBias)
                .power()
                .build();

        if (cfg.applyFlywheelVelocityPIDF) {
            plantFlywheel = FtcActuators.plant(hardwareMap)
                    .motor(cfg.nameMotorShooterWheel, cfg.directionMotorShooterWheel)
                    .velocity()
                    .deviceManaged()
                    .velocityPidf(
                            cfg.flywheelVelKp,
                            cfg.flywheelVelKi,
                            cfg.flywheelVelKd,
                            cfg.flywheelVelKf)
                    .doneDeviceManaged()
                    .bounded(0.0, cfg.velocityMax)
                    .nativeUnits()
                    .velocityTolerance(cfg.velocityToleranceNative)
                    .build();
        } else {
            plantFlywheel = FtcActuators.plant(hardwareMap)
                    .motor(cfg.nameMotorShooterWheel, cfg.directionMotorShooterWheel)
                    .velocity()
                    .deviceManagedWithDefaults()
                    .bounded(0.0, cfg.velocityMax)
                    .nativeUnits()
                    .velocityTolerance(cfg.velocityToleranceNative)
                    .build();
        }

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

                if (!flywheelEnabled) {
                    last = false;
                    readyLatch.reset(false);
                    return false;
                }

                double target = Math.abs(flywheelTargetNative);
                boolean enabled = target > 1e-6;
                double leadSec = Math.max(0.0, cfg.readyPredictLeadSec);
                double predicted = flywheelMeasuredAbs + flywheelMeasuredAccel * leadSec;
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

            @Override
            public void reset() {
                lastCycle = Long.MIN_VALUE;
                last = false;
                readyLatch.reset(false);
            }
        };

        lastStatus = buildStatus(null);
    }

    @Override
    public void setIntakeEnabled(boolean enabled) {
        intakeEnabled = enabled;
        if (enabled) {
            clearPendingShots();
        }
    }

    @Override
    public void setFlywheelEnabled(boolean enabled) {
        flywheelRequested = enabled;
        if (!enabled) {
            clearPendingShots();
            setFlywheelEnabledInternal(false);
        }
    }

    @Override
    public void setShootingEnabled(boolean enabled) {
        shootingRequested = enabled;
        if (!enabled) {
            clearPendingShots();
        }
    }

    @Override
    public void setEjectEnabled(boolean enabled) {
        ejectRequested = enabled;
        if (enabled) {
            clearPendingShots();
        }
    }

    @Override
    public void requestSingleShot() {
        requestShots(1);
    }

    @Override
    public void requestShots(int shotCount) {
        if (shotCount <= 0) {
            return;
        }
        queuedShotRequests += shotCount;
    }

    @Override
    public void cancelTransientActions() {
        clearPendingShots();
    }

    @Override
    public void setSelectedVelocityNative(double velocityNative) {
        selectedVelocityNative = clampVelocity(velocityNative);
    }

    @Override
    public void adjustSelectedVelocityNative(double deltaNative) {
        setSelectedVelocityNative(selectedVelocityNative + deltaNative);
    }

    @Override
    public void captureSuggestedShotVelocity() {
        setSelectedVelocityNative(targeting.suggestedVelocityNative(clock, selectedVelocityNative));
    }

    @Override
    public boolean hasPendingShots() {
        return queuedShotRequests > 0 || feedQueue.backlogCount() > 0;
    }

    @Override
    public Status status() {
        return lastStatus;
    }

    /**
     * Returns the memoized flywheel-ready gate for the current loop.
     */
    public BooleanSource flywheelReady() {
        return flywheelReadySource;
    }

    /**
     * Advances scoring policy, feed tasks, flywheel control, and all scoring-path actuator outputs.
     */
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");

        setFlywheelEnabledInternal(flywheelRequested && !ejectRequested);
        applyFeedPolicy(clock);
        updateFlywheel(clock);
        updateFeedAndTransfer(clock);
        lastStatus = buildStatus(clock);
    }

    /**
     * Writes a compact scoring-path dump into the provided debug sink.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "scoring" : prefix;
        Status s = status();
        dbg.addData(p + ".mode", s.mode)
                .addData(p + ".intakeEnabled", s.intakeEnabled)
                .addData(p + ".ejectRequested", s.ejectRequested)
                .addData(p + ".shootingRequested", s.shootingRequested)
                .addData(p + ".flywheelRequested", s.flywheelRequested)
                .addData(p + ".shootActive", s.shootActive)
                .addData(p + ".feedBacklog", s.feedBacklog)
                .addData(p + ".selectedVelocityNative", s.selectedVelocityNative)
                .addData(p + ".flywheelTargetNative", s.flywheelTargetNative)
                .addData(p + ".flywheelMeasuredNative", s.flywheelMeasuredNative)
                .addData(p + ".ready", s.ready);
    }

    /**
     * Stops all scoring-path outputs and clears transient loop state.
     */
    public void stop() {
        intakeEnabled = false;
        ejectRequested = false;
        shootingRequested = false;
        flywheelRequested = false;
        setFlywheelEnabledInternal(false);
        manualIntakeMotorPower = 0.0;
        manualIntakeTransferPower = 0.0;
        manualShooterTransferPower = 0.0;
        clearPendingShots();
        plantFlywheel.stop();
        plantIntakeMotor.stop();
        plantIntakeTransfer.stop();
        plantShooterTransfer.stop();
        lastFeedMode = FeedMode.IDLE;
        lastStatus = buildStatus(null);
    }

    private void applyFeedPolicy(LoopClock clock) {
        int backlog = feedQueue.backlogCount();
        FeedMode mode = selectFeedMode(backlog);
        lastFeedMode = mode;

        switch (mode) {
            case EJECT:
                applyEject();
                break;

            case SHOOT:
                int desiredBacklog = shootingRequested ? Math.max(1, backlog) : backlog + queuedShotRequests;
                feedQueue.ensureBacklog(clock, desiredBacklog, this::shootOneTask);
                queuedShotRequests = 0;
                applyIdleFeeds();
                break;

            case INTAKE:
                applyIntake();
                break;

            case IDLE:
            default:
                queuedShotRequests = 0;
                applyIdleFeeds();
                break;
        }
    }

    private OutputTask shootOneTask() {
        BooleanSource flywheelArmed = BooleanSource.of(this::flywheelEnabled);
        BooleanSource flywheelOkToFeed = flywheelReady().or(shootOverride.and(flywheelArmed));
        BooleanSource startWhen = flywheelOkToFeed.and(aimOkToShoot);
        BooleanSource doneWhen = BooleanSource.constant(true);

        return Tasks.gatedOutputUntil(
                "shootOne",
                startWhen,
                doneWhen,
                ScalarSource.constant(cfg.shootFeedPower),
                0.0,
                cfg.shootFeedPulseSec,
                cfg.shootFeedPulseSec + 1.0,
                cfg.shootFeedCooldownSec
        );
    }

    private FeedMode selectFeedMode(int backlog) {
        if (ejectRequested) {
            return FeedMode.EJECT;
        }
        if (shootingRequested || backlog > 0 || queuedShotRequests > 0) {
            return FeedMode.SHOOT;
        }
        if (intakeEnabled) {
            return FeedMode.INTAKE;
        }
        return FeedMode.IDLE;
    }

    private void updateFlywheel(LoopClock clock) {
        flywheelTargetNative = flywheelEnabled ? selectedVelocityNative : 0.0;
        plantFlywheel.setTarget(flywheelTargetNative);
        plantFlywheel.update(clock);

        flywheelMeasuredNative = plantFlywheel.getMeasurement();
        if (!Double.isFinite(flywheelMeasuredNative)) {
            flywheelMeasuredNative = 0.0;
        }
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
    }

    private void updateFeedAndTransfer(LoopClock clock) {
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

        plantIntakeMotor.update(clock);
        plantIntakeTransfer.update(clock);
        plantShooterTransfer.update(clock);
    }

    private Status buildStatus(LoopClock clockOrNull) {
        double err = flywheelMeasuredNative - flywheelTargetNative;
        double targetAbs = Math.abs(flywheelTargetNative);
        double leadSec = Math.max(0.0, cfg.readyPredictLeadSec);
        double predictedAbs = flywheelMeasuredAbs + flywheelMeasuredAccel * leadSec;
        if (predictedAbs < 0.0) {
            predictedAbs = 0.0;
        }

        int feedBacklog = feedQueue.backlogCount() + queuedShotRequests;
        boolean shootActive = shootingRequested || hasPendingShots();
        boolean ready = clockOrNull != null && flywheelReady().getAsBoolean(clockOrNull);

        return new Status(
                intakeEnabled,
                ejectRequested,
                shootingRequested,
                flywheelRequested,
                shootActive,
                feedBacklog,
                lastFeedMode.debugName,
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
                ready,
                feedQueue.queuedCount(),
                feedQueue.hasActiveTask(),
                clockOrNull != null ? feedQueue.getAsDouble(clockOrNull) : 0.0
        );
    }

    private void setFlywheelEnabledInternal(boolean enabled) {
        flywheelEnabled = enabled;
        if (!enabled) {
            flywheelReadySource.reset();
        }
    }

    private boolean flywheelEnabled() {
        return flywheelEnabled;
    }

    private void clearPendingShots() {
        queuedShotRequests = 0;
        feedQueue.cancelAndClear();
    }

    private void applyIntake() {
        manualIntakeMotorPower = clampPower(cfg.intakeMotorPower);
        manualIntakeTransferPower = clampPower(cfg.intakeTransferPower);
        manualShooterTransferPower = clampPower(-cfg.intakeShooterTransferHoldBackPower);
    }

    private void applyEject() {
        manualIntakeMotorPower = clampPower(-cfg.ejectMotorPower);
        manualIntakeTransferPower = clampPower(-cfg.ejectTransferPower);
        manualShooterTransferPower = clampPower(-cfg.ejectShooterTransferPower);
    }

    private void applyIdleFeeds() {
        manualIntakeMotorPower = 0.0;
        manualIntakeTransferPower = 0.0;
        manualShooterTransferPower = 0.0;
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
