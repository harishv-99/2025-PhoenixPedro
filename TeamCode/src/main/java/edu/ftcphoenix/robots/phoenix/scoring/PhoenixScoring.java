package edu.ftcphoenix.robots.phoenix.scoring;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetResolver;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.supervisor.RequestCounter;
import edu.ftcphoenix.fw.task.OutputTask;
import edu.ftcphoenix.fw.task.OutputTaskFactory;
import edu.ftcphoenix.fw.task.OutputTaskRunner;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Phoenix's one scoring mechanism owner.
 *
 * <p>The class owns scoring requests, feed policy, the shot queue, all four private Plants, final
 * target resolution, flywheel readiness evidence, update order, status, and terminal cleanup.
 * Request, policy, and realization remain visible as private method/field sections rather than
 * separate peer owners.</p>
 *
 * <p>TeleOp and Auto use only {@link PhoenixCapabilities.Scoring}. The sole advanced exception is
 * {@link #createFlywheelPlantForTuning(HardwareMap, PhoenixProfile.ScoringConfig)}, which creates a
 * fresh caller-owned Plant for the exclusive flywheel tuner from the same private recipe used by
 * production.</p>
 */
public final class PhoenixScoring implements PhoenixCapabilities.Scoring, RobotProgram.Output {

    private static final double ENABLED_TARGET_EPSILON = 1e-6;

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

    private final PhoenixProfile.ScoringConfig cfg;
    private final PhoenixTargeting targeting;
    private final BooleanSource aimOkToShoot;
    private final BooleanSource shootOverride;

    // Caller requests: held intent and pending one-shot work.
    private boolean intakeRequested;
    private boolean ejectRequested;
    private boolean shootingRequested;
    private boolean flywheelRequested;
    private double selectedVelocityNative;
    private final RequestCounter shotRequests = new RequestCounter();
    private boolean transientCancelRequested;

    // Scoring policy and queue state.
    private final OutputTaskRunner feedQueue = Tasks.outputQueue();
    private final BooleanSource feedPulseActive = feedQueue.activeSource();
    private final ScalarSource feedPulseOutput = feedQueue;
    private FeedMode lastFeedMode = FeedMode.IDLE;
    private boolean flywheelEnabled;
    private boolean previousIntakeRequested;
    private boolean previousEjectRequested;
    private boolean previousShootingRequested;
    private boolean previousFlywheelRequested;
    private double baseIntakeMotorPower;
    private double baseIntakeTransferPower;
    private double baseShooterTransferPower;
    private double lastFeedOutput;

    // Final realization and retained flywheel evidence.
    private final Plant intakeMotor;
    private final Plant intakeTransfer;
    private final Plant shooterTransfer;
    private final Plant flywheel;
    private final BooleanSource flywheelReadySource;
    private boolean finiteFlywheelMeasurementAvailable;
    private boolean finiteFlywheelAccelerationAvailable;
    private double flywheelTargetNative;
    private double flywheelMeasuredNative;
    private double flywheelMeasuredAbsNative;
    private double flywheelAccelNativePerSec;
    private double flywheelAccelAbsNativePerSec;
    private double previousFlywheelMeasuredAbsNative;

    private PhoenixCapabilities.ScoringStatus lastStatus;
    private boolean stopped;

    /**
     * Creates Phoenix's complete scoring owner and claims every configured scoring actuator.
     *
     * @param hardwareMap FTC hardware map used to resolve scoring devices
     * @param config checked-in scoring configuration; defensively copied
     * @param targeting shared targeting service used only for aim gates and velocity suggestions
     */
    public PhoenixScoring(HardwareMap hardwareMap,
                          PhoenixProfile.ScoringConfig config,
                          PhoenixTargeting targeting) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.targeting = Objects.requireNonNull(targeting, "targeting");
        this.aimOkToShoot = targeting.aimOkToShootSource();
        this.shootOverride = targeting.aimOverrideSource();
        this.selectedVelocityNative = cfg.velocityMin;

        PlantTargetResolver intakeMotorTarget = PlantTargets
                .overlay(ScalarSource.of(() -> baseIntakeMotorPower))
                .add("feedPulse", feedPulseActive,
                        feedPulseOutput.scaled(cfg.feedScaleIntakeMotor))
                .build();
        PlantTargetResolver intakeTransferTarget = PlantTargets
                .overlay(ScalarSource.of(() -> baseIntakeTransferPower))
                .add("feedPulse", feedPulseActive,
                        feedPulseOutput.scaled(cfg.feedScaleIntakeTransfer))
                .build();
        PlantTargetResolver shooterTransferTarget = PlantTargets
                .overlay(ScalarSource.of(() -> baseShooterTransferPower))
                .add("feedPulse", feedPulseActive,
                        feedPulseOutput.scaled(cfg.feedScaleShooterTransfer))
                .build();

        Plant builtIntakeMotor = null;
        Plant builtIntakeTransfer = null;
        Plant builtShooterTransfer = null;
        Plant builtFlywheel = null;
        BooleanSource builtFlywheelReadySource;
        try {
            builtIntakeMotor = FtcActuators.plant(hardwareMap)
                    .motor(cfg.nameMotorIntake, cfg.directionMotorIntake)
                    .power()
                    .targetFromResolver(intakeMotorTarget)
                    .build();

            builtIntakeTransfer = FtcActuators.plant(hardwareMap)
                    .crServo(cfg.nameCrServoIntakeTransfer, cfg.directionCrServoIntakeTransfer)
                    .power()
                    .targetFromResolver(intakeTransferTarget)
                    .build();

            builtShooterTransfer = FtcActuators.plant(hardwareMap)
                    .crServo(
                            cfg.nameCrServoShooterTransferRight,
                            cfg.directionCrServoShooterTransferRight)
                    .andCrServo(
                            cfg.nameCrServoShooterTransferLeft,
                            cfg.directionCrServoShooterTransferLeft)
                    .scale(cfg.shooterTransferLeftScale)
                    .power()
                    .targetFromResolver(shooterTransferTarget)
                    .build();

            builtFlywheel = buildFlywheelPlant(hardwareMap, cfg);
            builtFlywheelReadySource = BooleanSource.of(this::withinFlywheelReadyBand)
                    .debouncedOn(cfg.readyStableSec);
        } catch (RuntimeException primaryFailure) {
            final Plant rollbackFlywheel = builtFlywheel;
            final Plant rollbackIntakeMotor = builtIntakeMotor;
            final Plant rollbackIntakeTransfer = builtIntakeTransfer;
            final Plant rollbackShooterTransfer = builtShooterTransfer;
            throw CleanupActions.attemptAllAfterFailure(
                    primaryFailure,
                    () -> stopIfConstructed(rollbackFlywheel),
                    () -> stopIfConstructed(rollbackIntakeMotor),
                    () -> stopIfConstructed(rollbackIntakeTransfer),
                    () -> stopIfConstructed(rollbackShooterTransfer)
            );
        }

        this.intakeMotor = builtIntakeMotor;
        this.intakeTransfer = builtIntakeTransfer;
        this.shooterTransfer = builtShooterTransfer;
        this.flywheel = builtFlywheel;
        this.flywheelReadySource = builtFlywheelReadySource;
        try {
            this.lastStatus = buildStatus(null);
        } catch (RuntimeException primaryFailure) {
            throw CleanupActions.attemptAllAfterFailure(primaryFailure, this::stopOwnedPlants);
        }
    }

    /**
     * Creates a fresh caller-owned flywheel Plant for Phoenix's exclusive tuning OpMode.
     *
     * <p>This is an explicitly advanced assembly seam, not a second ordinary mechanism recipe.
     * Production calls the same private recipe from this owner. The returned Plant is not shared
     * with production, must have exactly one tuning-session heartbeat and lifecycle owner, and must
     * never be exposed through {@link PhoenixCapabilities}.</p>
     *
     * @param hardwareMap FTC hardware map used to resolve the configured flywheel motor
     * @param config scoring configuration; defensively copied before construction
     * @return fresh velocity Plant owned by the calling exclusive tuning session
     */
    public static Plant createFlywheelPlantForTuning(
            HardwareMap hardwareMap,
            PhoenixProfile.ScoringConfig config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        PhoenixProfile.ScoringConfig copiedConfig =
                Objects.requireNonNull(config, "config").copy();
        return buildFlywheelPlant(hardwareMap, copiedConfig);
    }

    @Override
    public void setIntakeEnabled(boolean enabled) {
        intakeRequested = enabled;
    }

    @Override
    public void setFlywheelEnabled(boolean enabled) {
        flywheelRequested = enabled;
    }

    @Override
    public void setShootingEnabled(boolean enabled) {
        shootingRequested = enabled;
    }

    @Override
    public void setEjectEnabled(boolean enabled) {
        ejectRequested = enabled;
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
        shotRequests.request(shotCount);
    }

    @Override
    public void cancelTransientActions() {
        transientCancelRequested = true;
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
        PhoenixCapabilities.TargetingStatus targetingStatus = targeting.status();
        if (targetingStatus.hasSuggestedVelocity) {
            setSelectedVelocityNative(targetingStatus.suggestedVelocityNative);
        }
    }

    @Override
    public boolean hasPendingShots() {
        return shotRequests.count() > 0 || feedQueue.backlogCount() > 0;
    }

    @Override
    public PhoenixCapabilities.ScoringStatus status() {
        return lastStatus;
    }

    /** Returns the memoized flywheel-ready gate for the current loop. */
    public BooleanSource flywheelReady() {
        return flywheelReadySource;
    }

    /**
     * Resolves scoring intent and advances every owned Plant exactly once in the required order.
     */
    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (stopped) {
            throw new IllegalStateException("PhoenixScoring cannot update after stop");
        }

        updatePolicyBeforeFlywheel(clock);
        updateFlywheel(clock);
        updatePolicyAfterFlywheel(clock);
        updateFeedPlants(clock);
        lastStatus = buildStatus(clock);
    }

    /** Writes a compact scoring dump without advancing behavior or hardware. */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "scoring" : prefix;
        PhoenixCapabilities.ScoringStatus s = status();
        dbg.addData(p + ".class", "PhoenixScoring")
                .addData(p + ".mode", s.mode)
                .addData(p + ".intakeEnabled", s.intakeEnabled)
                .addData(p + ".ejectRequested", s.ejectRequested)
                .addData(p + ".shootingRequested", s.shootingRequested)
                .addData(p + ".flywheelRequested", s.flywheelRequested)
                .addData(p + ".shootActive", s.shootActive)
                .addData(p + ".feedBacklog", s.feedBacklog)
                .addData(p + ".selectedVelocityNative", s.selectedVelocityNative)
                .addData(p + ".flywheelTargetNative", s.flywheelTargetNative)
                .addData(p + ".flywheelMeasuredNative", s.flywheelMeasuredNative)
                .addData(p + ".ready", s.ready)
                .addData(p + ".requests.requestedShots", shotRequests.count())
                .addData(p + ".requests.transientCancelRequested", transientCancelRequested)
                .addData(p + ".policy.baseIntakeMotorPower", baseIntakeMotorPower)
                .addData(p + ".policy.baseIntakeTransferPower", baseIntakeTransferPower)
                .addData(p + ".policy.baseShooterTransferPower", baseShooterTransferPower)
                .addData(p + ".policy.feedPulseActive", feedQueue.hasActiveTask())
                .addData(p + ".policy.feedPulseOutput", lastFeedOutput)
                .addData(p + ".flywheel.finiteMeasurementAvailable",
                        finiteFlywheelMeasurementAvailable)
                .addData(p + ".flywheel.finiteAccelerationAvailable",
                        finiteFlywheelAccelerationAvailable)
                .addData(p + ".flywheel.measuredAccelNativePerSec",
                        flywheelAccelNativePerSec);
        feedQueue.debugDump(dbg, p + ".policy.feedQueue");
    }

    /** Permanently stops scoring, clears transient work, and stops every owned Plant. */
    @Override
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        CleanupActions.attemptAll(
                this::stopMotionRequests,
                this::stopPolicy,
                this::stopOwnedPlants,
                () -> lastStatus = buildStatus(null)
        );
    }

    // Policy: turn caller requests into flywheel and feed behavior.

    private void updatePolicyBeforeFlywheel(LoopClock clock) {
        handleRequestTransitions();
        setFlywheelActuallyEnabled(flywheelRequested && !ejectRequested);
        applyFeedPolicy(clock);
    }

    private void updatePolicyAfterFlywheel(LoopClock clock) {
        feedQueue.update(clock);
        lastFeedOutput = feedQueue.getAsDouble(clock);
    }

    private void handleRequestTransitions() {
        boolean clearTransients = !previousIntakeRequested && intakeRequested;
        if (!previousEjectRequested && ejectRequested) {
            clearTransients = true;
        }
        if (previousShootingRequested && !shootingRequested) {
            clearTransients = true;
        }
        if (previousFlywheelRequested && !flywheelRequested) {
            clearTransients = true;
        }
        if (transientCancelRequested) {
            clearTransients = true;
        }
        if (clearTransients) {
            clearTransientShotWork();
        }

        previousIntakeRequested = intakeRequested;
        previousEjectRequested = ejectRequested;
        previousShootingRequested = shootingRequested;
        previousFlywheelRequested = flywheelRequested;
    }

    private void setFlywheelActuallyEnabled(boolean enabled) {
        if (flywheelEnabled == enabled) {
            return;
        }
        flywheelEnabled = enabled;
        if (!enabled) {
            flywheelReadySource.reset();
        }
    }

    private void applyFeedPolicy(LoopClock clock) {
        int backlog = feedQueue.backlogCount();
        FeedMode mode = selectFeedMode(backlog);
        lastFeedMode = mode;

        switch (mode) {
            case EJECT:
                applyEjectFeedBase();
                break;

            case SHOOT:
                int desiredBacklog;
                if (shootingRequested) {
                    shotRequests.clear();
                    desiredBacklog = Math.max(1, backlog);
                } else {
                    desiredBacklog = backlog + shotRequests.consumeAll();
                }
                feedQueue.ensureBacklog(clock, desiredBacklog, this::shootOneTask);
                applyIdleFeedBase();
                break;

            case INTAKE:
                applyIntakeFeedBase();
                break;

            case IDLE:
            default:
                shotRequests.clear();
                applyIdleFeedBase();
                break;
        }
    }

    private OutputTask shootOneTask() {
        BooleanSource flywheelArmed = BooleanSource.of(() -> flywheelEnabled);
        BooleanSource flywheelOkToFeed = flywheelReadySource
                .or(shootOverride.and(flywheelArmed));
        BooleanSource startWhen = flywheelOkToFeed.and(aimOkToShoot);

        OutputTaskFactory factory = Tasks.outputPulse("shootOne")
                .startWhen(startWhen)
                .runOutput(cfg.shootFeedPower)
                .forSeconds(cfg.shootFeedPulseSec)
                .cooldownSec(cfg.shootFeedCooldownSec)
                .build();
        return factory.create();
    }

    private FeedMode selectFeedMode(int backlog) {
        if (ejectRequested) {
            return FeedMode.EJECT;
        }
        if (shootingRequested || backlog > 0 || shotRequests.count() > 0) {
            return FeedMode.SHOOT;
        }
        if (intakeRequested) {
            return FeedMode.INTAKE;
        }
        return FeedMode.IDLE;
    }

    private void clearTransientShotWork() {
        shotRequests.clear();
        transientCancelRequested = false;
        feedQueue.cancelAndClear();
        lastFeedOutput = 0.0;
    }

    private int feedBacklogCount() {
        return feedQueue.backlogCount() + shotRequests.count();
    }

    private void applyIntakeFeedBase() {
        baseIntakeMotorPower = clampPower(cfg.intakeMotorPower);
        baseIntakeTransferPower = clampPower(cfg.intakeTransferPower);
        baseShooterTransferPower = clampPower(-cfg.intakeShooterTransferHoldBackPower);
    }

    private void applyEjectFeedBase() {
        baseIntakeMotorPower = clampPower(-cfg.ejectMotorPower);
        baseIntakeTransferPower = clampPower(-cfg.ejectTransferPower);
        baseShooterTransferPower = clampPower(-cfg.ejectShooterTransferPower);
    }

    private void applyIdleFeedBase() {
        baseIntakeMotorPower = 0.0;
        baseIntakeTransferPower = 0.0;
        baseShooterTransferPower = 0.0;
    }

    private void stopMotionRequests() {
        intakeRequested = false;
        ejectRequested = false;
        shootingRequested = false;
        flywheelRequested = false;
        shotRequests.clear();
        transientCancelRequested = false;
    }

    private void stopPolicy() {
        flywheelEnabled = false;
        CleanupActions.attemptAll(this::clearTransientShotWork, this::resetPolicyState);
    }

    private void resetPolicyState() {
        applyIdleFeedBase();
        lastFeedMode = FeedMode.IDLE;
        previousIntakeRequested = false;
        previousEjectRequested = false;
        previousShootingRequested = false;
        previousFlywheelRequested = false;
    }

    // Realization: one final command path and one update for each private Plant.

    private void updateFlywheel(LoopClock clock) {
        flywheel.commandTarget().set(flywheelEnabled ? selectedVelocityNative : 0.0);
        flywheel.update(clock);
        flywheelTargetNative = flywheel.getRequestedTarget();

        double rawMeasurement = flywheel.getMeasurement();
        finiteFlywheelMeasurementAvailable = Double.isFinite(rawMeasurement);
        flywheelMeasuredNative = finiteFlywheelMeasurementAvailable ? rawMeasurement : 0.0;
        flywheelMeasuredAbsNative = Math.abs(flywheelMeasuredNative);

        double dtSec = clock.dtSec();
        if (finiteFlywheelMeasurementAvailable && dtSec > 1e-6 && Double.isFinite(dtSec)) {
            double derivedAcceleration =
                    (flywheelMeasuredAbsNative - previousFlywheelMeasuredAbsNative) / dtSec;
            finiteFlywheelAccelerationAvailable = Double.isFinite(derivedAcceleration);
            flywheelAccelNativePerSec = finiteFlywheelAccelerationAvailable
                    ? derivedAcceleration : 0.0;
            flywheelAccelAbsNativePerSec = finiteFlywheelAccelerationAvailable
                    ? Math.abs(derivedAcceleration) : 0.0;
        } else {
            finiteFlywheelAccelerationAvailable = false;
            flywheelAccelNativePerSec = 0.0;
            flywheelAccelAbsNativePerSec = 0.0;
        }
        previousFlywheelMeasuredAbsNative = flywheelMeasuredAbsNative;
    }

    private void updateFeedPlants(LoopClock clock) {
        intakeMotor.update(clock);
        intakeTransfer.update(clock);
        shooterTransfer.update(clock);
    }

    private boolean withinFlywheelReadyBand() {
        if (!finiteFlywheelMeasurementAvailable) {
            return false;
        }
        double targetAbs = Math.abs(flywheelTargetNative);
        boolean enabled = targetAbs > ENABLED_TARGET_EPSILON;
        double predictedAbs = predictedFlywheelAbsNative();
        double predictedError = predictedAbs - targetAbs;
        return enabled
                && predictedError >= -cfg.velocityToleranceBelowNative
                && predictedError <= cfg.velocityToleranceAboveNative;
    }

    private double predictedFlywheelAbsNative() {
        double leadSec = Math.max(0.0, cfg.readyPredictLeadSec);
        double predictedAbs = flywheelMeasuredAbsNative + flywheelAccelNativePerSec * leadSec;
        return Math.max(0.0, predictedAbs);
    }

    private void stopOwnedPlants() {
        finiteFlywheelMeasurementAvailable = false;
        finiteFlywheelAccelerationAvailable = false;
        flywheelTargetNative = 0.0;
        flywheelMeasuredNative = 0.0;
        flywheelMeasuredAbsNative = 0.0;
        flywheelAccelNativePerSec = 0.0;
        flywheelAccelAbsNativePerSec = 0.0;
        previousFlywheelMeasuredAbsNative = 0.0;
        CleanupActions.attemptAll(
                flywheelReadySource::reset,
                flywheel::stop,
                intakeMotor::stop,
                intakeTransfer::stop,
                shooterTransfer::stop
        );
    }

    private static Plant buildFlywheelPlant(HardwareMap hardwareMap,
                                            PhoenixProfile.ScoringConfig cfg) {
        if (cfg.applyFlywheelVelocityPIDF) {
            return FtcActuators.plant(hardwareMap)
                    .motor(cfg.nameMotorShooterWheel, cfg.directionMotorShooterWheel)
                    .velocity()
                    .deviceManagedWithOverrides()
                    .velocityPidf(
                            cfg.flywheelVelKp,
                            cfg.flywheelVelKi,
                            cfg.flywheelVelKd,
                            cfg.flywheelVelKf)
                    .bounded(0.0, cfg.velocityMax)
                    .nativeUnits()
                    .velocityTolerance(cfg.velocityToleranceNative)
                    .targetFromNewCommand(0.0)
                    .build();
        }

        return FtcActuators.plant(hardwareMap)
                .motor(cfg.nameMotorShooterWheel, cfg.directionMotorShooterWheel)
                .velocity()
                .deviceManaged()
                .bounded(0.0, cfg.velocityMax)
                .nativeUnits()
                .velocityTolerance(cfg.velocityToleranceNative)
                .targetFromNewCommand(0.0)
                .build();
    }

    private PhoenixCapabilities.ScoringStatus buildStatus(LoopClock clockOrNull) {
        double error = flywheelMeasuredNative - flywheelTargetNative;
        double targetAbs = Math.abs(flywheelTargetNative);
        double leadSec = Math.max(0.0, cfg.readyPredictLeadSec);
        double predictedAbs = predictedFlywheelAbsNative();
        int feedBacklog = feedBacklogCount();
        boolean shootActive = shootingRequested || hasPendingShots();
        boolean ready = clockOrNull != null && flywheelReadySource.getAsBoolean(clockOrNull);

        return new PhoenixCapabilities.ScoringStatus(
                intakeRequested,
                ejectRequested,
                shootingRequested,
                flywheelRequested,
                shootActive,
                feedBacklog,
                lastFeedMode.debugName,
                flywheelEnabled,
                cfg.applyFlywheelVelocityPIDF,
                selectedVelocityNative,
                flywheelTargetNative,
                flywheelMeasuredNative,
                error,
                Math.abs(error),
                cfg.velocityToleranceNative,
                cfg.velocityToleranceBelowNative,
                cfg.velocityToleranceAboveNative,
                flywheelAccelNativePerSec,
                flywheelAccelAbsNativePerSec,
                leadSec,
                predictedAbs,
                predictedAbs - targetAbs,
                flywheel.atTarget(),
                ready,
                feedQueue.queuedCount(),
                feedQueue.hasActiveTask(),
                clockOrNull != null ? lastFeedOutput : 0.0
        );
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

    private static void stopIfConstructed(Plant plant) {
        if (plant != null) {
            plant.stop();
        }
    }
}
