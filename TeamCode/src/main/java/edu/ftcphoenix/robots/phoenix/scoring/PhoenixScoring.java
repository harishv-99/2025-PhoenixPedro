package edu.ftcphoenix.robots.phoenix.scoring;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetResolver;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.Direction;
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

/**
 * Phoenix's one scoring mechanism owner.
 *
 * <p>The class owns scoring requests, feed policy, the shot queue, all four private Plants, final
 * target resolution, flywheel readiness evidence, update order, status, and terminal cleanup.
 * Request, policy, and realization remain visible as private method/field sections rather than
 * separate peer owners.</p>
 *
 * <p>TeleOp and Auto use only {@link PhoenixCapabilities.Scoring}. The sole advanced exception is
 * {@link #createFlywheelPlantForTuning(HardwareMap, Config)}, which creates a
 * fresh caller-owned Plant for the exclusive flywheel tuner from the same private recipe used by
 * production.</p>
 */
public final class PhoenixScoring implements PhoenixCapabilities.Scoring, RobotProgram.Output {

    /** Mutable data-only scoring hardware, readiness, feed, and controller configuration. */
    public static final class Config {
        public String nameMotorIntake;
        public Direction directionMotorIntake;
        public String nameCrServoIntakeTransfer;
        public Direction directionCrServoIntakeTransfer;
        public String nameCrServoShooterTransferLeft;
        public Direction directionCrServoShooterTransferLeft;
        public String nameCrServoShooterTransferRight;
        public Direction directionCrServoShooterTransferRight;
        public double shooterTransferLeftScale;
        public String nameMotorShooterWheel;
        public Direction directionMotorShooterWheel;
        public double velocityMin;
        public double velocityMax;
        public double velocityToleranceNative;
        public boolean applyFlywheelVelocityPIDF;
        public double flywheelVelKp;
        public double flywheelVelKi;
        public double flywheelVelKd;
        public double flywheelVelKf;
        public double velocityToleranceBelowNative;
        public double velocityToleranceAboveNative;
        public double readyPredictLeadSec;
        public double readyStableSec;
        public double intakeMotorPower;
        public double intakeTransferPower;
        public double intakeShooterTransferHoldBackPower;
        public double ejectMotorPower;
        public double ejectTransferPower;
        public double ejectShooterTransferPower;
        public double shootFeedPower;
        public double shootFeedPulseSec;
        public double shootFeedCooldownSec;
        public double feedScaleIntakeMotor;
        public double feedScaleIntakeTransfer;
        public double feedScaleShooterTransfer;

        private Config() {
            // Use defaults() to start from the complete Phoenix software baseline.
        }

        /** Returns a fresh software baseline; this does not establish physical calibration. */
        public static Config defaults() {
            Config config = new Config();
            applyHardwareAndFlywheelDefaults(config);
            applyReadinessAndFeedDefaults(config);
            return config;
        }

        private static Config rawCopyOf(Config source) {
            Config copy = new Config();
            copy.nameMotorIntake = source.nameMotorIntake;
            copy.directionMotorIntake = source.directionMotorIntake;
            copy.nameCrServoIntakeTransfer = source.nameCrServoIntakeTransfer;
            copy.directionCrServoIntakeTransfer = source.directionCrServoIntakeTransfer;
            copy.nameCrServoShooterTransferLeft = source.nameCrServoShooterTransferLeft;
            copy.directionCrServoShooterTransferLeft = source.directionCrServoShooterTransferLeft;
            copy.nameCrServoShooterTransferRight = source.nameCrServoShooterTransferRight;
            copy.directionCrServoShooterTransferRight = source.directionCrServoShooterTransferRight;
            copy.shooterTransferLeftScale = source.shooterTransferLeftScale;
            copy.nameMotorShooterWheel = source.nameMotorShooterWheel;
            copy.directionMotorShooterWheel = source.directionMotorShooterWheel;
            copy.velocityMin = source.velocityMin;
            copy.velocityMax = source.velocityMax;
            copy.velocityToleranceNative = source.velocityToleranceNative;
            copy.applyFlywheelVelocityPIDF = source.applyFlywheelVelocityPIDF;
            copy.flywheelVelKp = source.flywheelVelKp;
            copy.flywheelVelKi = source.flywheelVelKi;
            copy.flywheelVelKd = source.flywheelVelKd;
            copy.flywheelVelKf = source.flywheelVelKf;
            copy.velocityToleranceBelowNative = source.velocityToleranceBelowNative;
            copy.velocityToleranceAboveNative = source.velocityToleranceAboveNative;
            copy.readyPredictLeadSec = source.readyPredictLeadSec;
            copy.readyStableSec = source.readyStableSec;
            copy.intakeMotorPower = source.intakeMotorPower;
            copy.intakeTransferPower = source.intakeTransferPower;
            copy.intakeShooterTransferHoldBackPower = source.intakeShooterTransferHoldBackPower;
            copy.ejectMotorPower = source.ejectMotorPower;
            copy.ejectTransferPower = source.ejectTransferPower;
            copy.ejectShooterTransferPower = source.ejectShooterTransferPower;
            copy.shootFeedPower = source.shootFeedPower;
            copy.shootFeedPulseSec = source.shootFeedPulseSec;
            copy.shootFeedCooldownSec = source.shootFeedCooldownSec;
            copy.feedScaleIntakeMotor = source.feedScaleIntakeMotor;
            copy.feedScaleIntakeTransfer = source.feedScaleIntakeTransfer;
            copy.feedScaleShooterTransfer = source.feedScaleShooterTransfer;
            return copy;
        }
    }

    private static final double ENABLED_TARGET_EPSILON = 1e-6;
    private static final double MAX_FTC_PIDF_COEFFICIENT = Integer.MAX_VALUE / 65536.0;

    private static void applyHardwareAndFlywheelDefaults(Config config) {
        config.nameMotorIntake = "intakeMotor";
        config.directionMotorIntake = Direction.FORWARD;
        config.nameCrServoIntakeTransfer = "intakeTransfer";
        config.directionCrServoIntakeTransfer = Direction.REVERSE;
        config.nameCrServoShooterTransferLeft = "shooterTransferLeft";
        config.directionCrServoShooterTransferLeft = Direction.REVERSE;
        config.nameCrServoShooterTransferRight = "shooterTransferRight";
        config.directionCrServoShooterTransferRight = Direction.FORWARD;
        config.shooterTransferLeftScale = 0.65;
        config.nameMotorShooterWheel = "shooterMotor";
        config.directionMotorShooterWheel = Direction.FORWARD;
        config.velocityMin = 700.0;
        config.velocityMax = 2000.0;
        config.velocityToleranceNative = 50.0;
        config.applyFlywheelVelocityPIDF = false;
        config.flywheelVelKp = 0.0;
        config.flywheelVelKi = 0.0;
        config.flywheelVelKd = 0.0;
        config.flywheelVelKf = 0.0;
    }

    private static void applyReadinessAndFeedDefaults(Config config) {
        config.velocityToleranceBelowNative = 50.0;
        config.velocityToleranceAboveNative = 50.0;
        config.readyPredictLeadSec = 0.10;
        config.readyStableSec = 0.03;
        config.intakeMotorPower = 1.0;
        config.intakeTransferPower = 1.0;
        config.intakeShooterTransferHoldBackPower = 0.5;
        config.ejectMotorPower = 1.0;
        config.ejectTransferPower = 1.0;
        config.ejectShooterTransferPower = 1.0;
        config.shootFeedPower = 1.0;
        config.shootFeedPulseSec = 0.22;
        config.shootFeedCooldownSec = 0.06;
        config.feedScaleIntakeMotor = 1.0;
        config.feedScaleIntakeTransfer = 1.0;
        config.feedScaleShooterTransfer = 1.0;
    }

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

    private final Config cfg;
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
                          Config config,
                          PhoenixTargeting targeting) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = captureProductionConfig(config);
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
            Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Config copiedConfig = captureFlywheelTunerConfig(config);
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
        double predictedAbs = flywheelMeasuredAbsNative
                + flywheelAccelNativePerSec * cfg.readyPredictLeadSec;
        return Double.isFinite(predictedAbs) ? Math.max(0.0, predictedAbs) : Double.NaN;
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

    /** Captures and validates the complete production slice before any Plant is constructed. */
    private static Config captureProductionConfig(Config config) {
        Config copy = copyAllConfig(Objects.requireNonNull(
                config,
                "PhoenixScoring.Config is required"
        ));

        requireHardwareName("nameMotorIntake", copy.nameMotorIntake);
        requireDirection("directionMotorIntake", copy.directionMotorIntake);
        requireHardwareName("nameCrServoIntakeTransfer", copy.nameCrServoIntakeTransfer);
        requireDirection("directionCrServoIntakeTransfer", copy.directionCrServoIntakeTransfer);
        requireHardwareName(
                "nameCrServoShooterTransferLeft",
                copy.nameCrServoShooterTransferLeft
        );
        requireDifferentHardwareNames(
                "nameCrServoIntakeTransfer",
                copy.nameCrServoIntakeTransfer,
                "nameCrServoShooterTransferLeft",
                copy.nameCrServoShooterTransferLeft
        );
        requireDirection(
                "directionCrServoShooterTransferLeft",
                copy.directionCrServoShooterTransferLeft
        );
        requireHardwareName(
                "nameCrServoShooterTransferRight",
                copy.nameCrServoShooterTransferRight
        );
        requireDifferentHardwareNames(
                "nameCrServoIntakeTransfer",
                copy.nameCrServoIntakeTransfer,
                "nameCrServoShooterTransferRight",
                copy.nameCrServoShooterTransferRight
        );
        requireDifferentHardwareNames(
                "nameCrServoShooterTransferLeft",
                copy.nameCrServoShooterTransferLeft,
                "nameCrServoShooterTransferRight",
                copy.nameCrServoShooterTransferRight
        );
        requireDirection(
                "directionCrServoShooterTransferRight",
                copy.directionCrServoShooterTransferRight
        );
        requireSignedScale("shooterTransferLeftScale", copy.shooterTransferLeftScale);
        requireHardwareName("nameMotorShooterWheel", copy.nameMotorShooterWheel);
        requireDifferentHardwareNames(
                "nameMotorIntake",
                copy.nameMotorIntake,
                "nameMotorShooterWheel",
                copy.nameMotorShooterWheel
        );
        requireDirection("directionMotorShooterWheel", copy.directionMotorShooterWheel);
        requireVelocityRange(copy.velocityMin, copy.velocityMax);
        requireFiniteNonNegative("velocityToleranceNative", copy.velocityToleranceNative);
        if (copy.applyFlywheelVelocityPIDF) {
            requireFtcPidfCoefficient("flywheelVelKp", copy.flywheelVelKp);
            requireFtcPidfCoefficient("flywheelVelKi", copy.flywheelVelKi);
            requireFtcPidfCoefficient("flywheelVelKd", copy.flywheelVelKd);
            requireFtcPidfCoefficient("flywheelVelKf", copy.flywheelVelKf);
        }
        requireFiniteNonNegative(
                "velocityToleranceBelowNative",
                copy.velocityToleranceBelowNative
        );
        requireFiniteNonNegative(
                "velocityToleranceAboveNative",
                copy.velocityToleranceAboveNative
        );
        requireFiniteNonNegative("readyPredictLeadSec", copy.readyPredictLeadSec);
        requireFiniteNonNegative("readyStableSec", copy.readyStableSec);
        requireMagnitudePower("intakeMotorPower", copy.intakeMotorPower);
        requireMagnitudePower("intakeTransferPower", copy.intakeTransferPower);
        requireMagnitudePower(
                "intakeShooterTransferHoldBackPower",
                copy.intakeShooterTransferHoldBackPower
        );
        requireMagnitudePower("ejectMotorPower", copy.ejectMotorPower);
        requireMagnitudePower("ejectTransferPower", copy.ejectTransferPower);
        requireMagnitudePower("ejectShooterTransferPower", copy.ejectShooterTransferPower);
        requireMagnitudePower("shootFeedPower", copy.shootFeedPower);
        requireFiniteNonNegative("shootFeedPulseSec", copy.shootFeedPulseSec);
        requireFiniteNonNegative("shootFeedCooldownSec", copy.shootFeedCooldownSec);
        requireSignedScale("feedScaleIntakeMotor", copy.feedScaleIntakeMotor);
        requireSignedScale("feedScaleIntakeTransfer", copy.feedScaleIntakeTransfer);
        requireSignedScale("feedScaleShooterTransfer", copy.feedScaleShooterTransfer);
        return copy;
    }

    /** Captures only the fields the exclusive flywheel tuner can activate. */
    private static Config captureFlywheelTunerConfig(Config config) {
        Config source = Objects.requireNonNull(config, "PhoenixScoring.Config is required");
        Config copy = Config.defaults();
        copy.nameMotorShooterWheel = source.nameMotorShooterWheel;
        copy.directionMotorShooterWheel = source.directionMotorShooterWheel;
        copy.velocityMin = source.velocityMin;
        copy.velocityMax = source.velocityMax;
        copy.velocityToleranceNative = source.velocityToleranceNative;
        copy.applyFlywheelVelocityPIDF = source.applyFlywheelVelocityPIDF;
        copy.flywheelVelKp = source.flywheelVelKp;
        copy.flywheelVelKi = source.flywheelVelKi;
        copy.flywheelVelKd = source.flywheelVelKd;
        copy.flywheelVelKf = source.flywheelVelKf;

        requireHardwareName("nameMotorShooterWheel", copy.nameMotorShooterWheel);
        requireDirection("directionMotorShooterWheel", copy.directionMotorShooterWheel);
        requireVelocityRange(copy.velocityMin, copy.velocityMax);
        requireFiniteNonNegative("velocityToleranceNative", copy.velocityToleranceNative);
        if (copy.applyFlywheelVelocityPIDF) {
            requireFtcPidfCoefficient("flywheelVelKp", copy.flywheelVelKp);
            requireFtcPidfCoefficient("flywheelVelKi", copy.flywheelVelKi);
            requireFtcPidfCoefficient("flywheelVelKd", copy.flywheelVelKd);
            requireFtcPidfCoefficient("flywheelVelKf", copy.flywheelVelKf);
        }
        return copy;
    }

    private static Config copyAllConfig(Config source) {
        return Config.rawCopyOf(source);
    }

    private static void requireHardwareName(String fieldName, String value) {
        if (value == null || value.trim().isEmpty()) {
            throw invalidConfig(fieldName, "a non-blank FTC hardware name", value);
        }
    }

    private static void requireDirection(String fieldName, Direction value) {
        if (value == null) {
            throw invalidConfig(fieldName, "non-null", null);
        }
    }

    private static void requireDifferentHardwareNames(String firstField,
                                                      String firstName,
                                                      String secondField,
                                                      String secondName) {
        if (firstName.trim().equals(secondName.trim())) {
            String effectiveKey = firstName.trim();
            throw new IllegalArgumentException(
                    "PhoenixScoring.Config." + firstField + " and PhoenixScoring.Config."
                            + secondField + " must identify different FTC hardware devices after "
                            + "trimming; got \"" + firstName + "\" and \"" + secondName
                            + "\" with effective key \"" + effectiveKey + "\"."
            );
        }
    }

    private static void requireVelocityRange(double minimum, double maximum) {
        requireFiniteNonNegative("velocityMin", minimum);
        requireFiniteNonNegative("velocityMax", maximum);
        if (minimum > maximum) {
            throw new IllegalArgumentException(
                    "PhoenixScoring.Config.velocityMin must be <= "
                            + "PhoenixScoring.Config.velocityMax, got " + minimum + " and "
                            + maximum + "."
            );
        }
    }

    private static void requireFiniteNonNegative(String fieldName, double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw invalidConfig(fieldName, "finite and >= 0", value);
        }
    }

    private static void requireMagnitudePower(String fieldName, double value) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw invalidConfig(fieldName, "finite and in [0.0, 1.0]", value);
        }
    }

    private static void requireSignedScale(String fieldName, double value) {
        if (!Double.isFinite(value) || value < -1.0 || value > 1.0) {
            throw invalidConfig(fieldName, "finite and in [-1.0, 1.0]", value);
        }
    }

    private static void requireFtcPidfCoefficient(String fieldName, double value) {
        if (!Double.isFinite(value)
                || value < -MAX_FTC_PIDF_COEFFICIENT
                || value > MAX_FTC_PIDF_COEFFICIENT) {
            throw invalidConfig(
                    fieldName,
                    "finite and in [-Integer.MAX_VALUE / 65536.0, "
                            + "+Integer.MAX_VALUE / 65536.0] while PIDF overrides are enabled",
                    value
            );
        }
    }

    private static IllegalArgumentException invalidConfig(String fieldName,
                                                          String requirement,
                                                          Object value) {
        String renderedValue = value instanceof String
                ? "\"" + value + "\""
                : String.valueOf(value);
        return new IllegalArgumentException(
                "PhoenixScoring.Config." + fieldName + " must be " + requirement + ", got "
                        + renderedValue + "."
        );
    }

    private static Plant buildFlywheelPlant(HardwareMap hardwareMap,
                                            Config cfg) {
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
