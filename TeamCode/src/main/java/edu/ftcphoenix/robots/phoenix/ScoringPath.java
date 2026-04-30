package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetSource;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.core.control.DebounceBoolean;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.supervisor.HeldValue;
import edu.ftcphoenix.fw.supervisor.RequestCounter;
import edu.ftcphoenix.fw.task.OutputTask;
import edu.ftcphoenix.fw.task.OutputTaskFactory;
import edu.ftcphoenix.fw.task.OutputTaskRunner;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Phoenix scoring path: intake, transfer, flywheel, and scoring policy in one owner.
 *
 * <p>This class is still the single owner of the scoring-path source graph and plants, but it now keeps its mutable
 * state in three internal roles that mirror Phoenix's recommended mechanism layering:</p>
 * <ul>
 *   <li><b>Inputs</b>: caller-owned held and pending values written through the shared capability surface.</li>
 *   <li><b>Execution</b>: robot-owned priority, queueing, gates, and transient behavior state.</li>
 *   <li><b>Realization</b>: plant ownership, final target sources, and flywheel readback.</li>
 * </ul>
 *
 * <p>Keeping those roles explicit makes it easier for TeleOp and Auto to share the same public API
 * without allowing plant writes or queue state to leak out through capability methods.</p>
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
        public final boolean flywheelAtTarget;
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
                      boolean flywheelAtTarget,
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
            this.flywheelAtTarget = flywheelAtTarget;
            this.ready = ready;
            this.feedQueued = feedQueued;
            this.feedActive = feedActive;
            this.feedOutput = feedOutput;
        }
    }

    /**
     * Layer-1 caller-owned input memory.
     *
     * <p>These values are written by capability methods and then interpreted later by execution
     * policy. Held values keep the last explicit selection, while request counters hold pending
     * one-shot work until execution consumes or clears it.</p>
     */
    private static final class Inputs {
        final HeldValue<Boolean> intakeRequested = new HeldValue<>(false);
        final HeldValue<Boolean> ejectRequested = new HeldValue<>(false);
        final HeldValue<Boolean> shootingRequested = new HeldValue<>(false);
        final HeldValue<Boolean> flywheelRequested = new HeldValue<>(false);
        final HeldValue<Double> selectedVelocityNative;
        final RequestCounter shotRequests = new RequestCounter();
        final RequestCounter transientCancelRequests = new RequestCounter(1);

        Inputs(double initialSelectedVelocityNative) {
            this.selectedVelocityNative = new HeldValue<>(initialSelectedVelocityNative);
        }

        void stopMotionRequests() {
            intakeRequested.set(false);
            ejectRequested.set(false);
            shootingRequested.set(false);
            flywheelRequested.set(false);
            shotRequests.clear();
            transientCancelRequests.clear();
        }

        void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "inputs" : prefix;
            dbg.addData(p + ".class", "ScoringInputs");
            intakeRequested.debugDump(dbg, p + ".intakeRequested");
            ejectRequested.debugDump(dbg, p + ".ejectRequested");
            shootingRequested.debugDump(dbg, p + ".shootingRequested");
            flywheelRequested.debugDump(dbg, p + ".flywheelRequested");
            selectedVelocityNative.debugDump(dbg, p + ".selectedVelocityNative");
            shotRequests.debugDump(dbg, p + ".shotRequests");
            transientCancelRequests.debugDump(dbg, p + ".transientCancelRequests");
        }
    }

    /**
     * Layer-2 robot-owned execution state.
     *
     * <p>This is where Phoenix decides what scoring behavior is active, which queued shot work is
     * admitted, and whether the feed queue is currently overriding the base feed powers.</p>
     */
    private final class Execution {
        private final OutputTaskRunner feedQueue = Tasks.outputQueue();

        private FeedMode lastFeedMode = FeedMode.IDLE;
        private boolean flywheelEnabled = false;

        private boolean prevIntakeRequested = false;
        private boolean prevEjectRequested = false;
        private boolean prevShootingRequested = false;
        private boolean prevFlywheelRequested = false;

        private double baseIntakeMotorPower = 0.0;
        private double baseIntakeTransferPower = 0.0;
        private double baseShooterTransferPower = 0.0;

        private final BooleanSource feedPulseActive = feedQueue.activeSource();
        private final ScalarSource feedPulseOutput = feedQueue;
        private double lastFeedOutput = 0.0;

        void updateBeforeFlywheel(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            handleInputTransitions();
            setFlywheelEnabled(inputs.flywheelRequested.get() && !inputs.ejectRequested.get());
            applyFeedPolicy(clock);
        }

        void updateAfterFlywheel(LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            feedQueue.update(clock);
            lastFeedOutput = feedQueue.getAsDouble(clock);
        }

        boolean isFlywheelEnabled() {
            return flywheelEnabled;
        }

        String modeName() {
            return lastFeedMode.debugName;
        }

        double baseIntakeMotorPower() {
            return baseIntakeMotorPower;
        }

        double baseIntakeTransferPower() {
            return baseIntakeTransferPower;
        }

        double baseShooterTransferPower() {
            return baseShooterTransferPower;
        }

        boolean isFeedPulseActive() {
            return feedQueue.hasActiveTask();
        }

        double feedPulseOutput() {
            return lastFeedOutput;
        }

        BooleanSource feedPulseActiveSource() {
            return feedPulseActive;
        }

        ScalarSource feedPulseOutputSource() {
            return feedPulseOutput;
        }

        int queuedFeedTasks() {
            return feedQueue.queuedCount();
        }

        int feedBacklogCount() {
            return feedQueue.backlogCount() + inputs.shotRequests.count();
        }

        boolean hasPendingShots() {
            return inputs.shotRequests.count() > 0 || feedQueue.backlogCount() > 0;
        }

        void stop() {
            clearTransientShotWork();
            setFlywheelEnabled(false);
            applyIdleFeedBase();
            lastFeedMode = FeedMode.IDLE;
            prevIntakeRequested = false;
            prevEjectRequested = false;
            prevShootingRequested = false;
            prevFlywheelRequested = false;
        }

        void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "execution" : prefix;
            dbg.addData(p + ".class", "ScoringExecution")
                    .addData(p + ".mode", lastFeedMode.debugName)
                    .addData(p + ".flywheelEnabled", flywheelEnabled)
                    .addData(p + ".baseIntakeMotorPower", baseIntakeMotorPower)
                    .addData(p + ".baseIntakeTransferPower", baseIntakeTransferPower)
                    .addData(p + ".baseShooterTransferPower", baseShooterTransferPower)
                    .addData(p + ".feedPulseActive", isFeedPulseActive())
                    .addData(p + ".feedPulseOutput", lastFeedOutput);
            feedQueue.debugDump(dbg, p + ".feedQueue");
        }

        private void handleInputTransitions() {
            boolean intakeCur = inputs.intakeRequested.get();
            boolean ejectCur = inputs.ejectRequested.get();
            boolean shootingCur = inputs.shootingRequested.get();
            boolean flywheelCur = inputs.flywheelRequested.get();

            boolean clearTransients = !prevIntakeRequested && intakeCur;
            if (!prevEjectRequested && ejectCur) {
                clearTransients = true;
            }
            if (prevShootingRequested && !shootingCur) {
                clearTransients = true;
            }
            if (prevFlywheelRequested && !flywheelCur) {
                clearTransients = true;
            }
            if (inputs.transientCancelRequests.consumeAll() > 0) {
                clearTransients = true;
            }
            if (clearTransients) {
                clearTransientShotWork();
            }

            prevIntakeRequested = intakeCur;
            prevEjectRequested = ejectCur;
            prevShootingRequested = shootingCur;
            prevFlywheelRequested = flywheelCur;
        }

        private void clearTransientShotWork() {
            inputs.shotRequests.clear();
            inputs.transientCancelRequests.clear();
            feedQueue.cancelAndClear();
            lastFeedOutput = 0.0;
        }

        private void setFlywheelEnabled(boolean enabled) {
            if (flywheelEnabled == enabled) {
                return;
            }
            flywheelEnabled = enabled;
            if (!enabled) {
                realization.resetFlywheelReady();
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
                    if (inputs.shootingRequested.get()) {
                        inputs.shotRequests.clear();
                        desiredBacklog = Math.max(1, backlog);
                    } else {
                        desiredBacklog = backlog + inputs.shotRequests.consumeAll();
                    }
                    feedQueue.ensureBacklog(clock, desiredBacklog, this::shootOneTask);
                    applyIdleFeedBase();
                    break;

                case INTAKE:
                    applyIntakeFeedBase();
                    break;

                case IDLE:
                default:
                    inputs.shotRequests.clear();
                    applyIdleFeedBase();
                    break;
            }
        }

        private OutputTask shootOneTask() {
            BooleanSource flywheelArmed = BooleanSource.of(this::isFlywheelEnabled);
            BooleanSource flywheelOkToFeed = realization.flywheelReadySource().or(shootOverride.and(flywheelArmed));
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
            if (inputs.ejectRequested.get()) {
                return FeedMode.EJECT;
            }
            if (inputs.shootingRequested.get() || backlog > 0 || inputs.shotRequests.count() > 0) {
                return FeedMode.SHOOT;
            }
            if (inputs.intakeRequested.get()) {
                return FeedMode.INTAKE;
            }
            return FeedMode.IDLE;
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
    }

    /**
     * Layer-3 plant realization and readback.
     *
     * <p>This is the only owner that touches the scoring-path plants. It turns execution outputs into
     * final plant targets and maintains flywheel measurement state used by readiness gates and status.</p>
     */
    private final class Realization {
        private final Plant plantIntakeMotor;
        private final Plant plantIntakeTransfer;
        private final Plant plantShooterTransfer;
        private final Plant plantFlywheel;

        private final PlantTargetSource intakeMotorTargetSource;
        private final PlantTargetSource intakeTransferTargetSource;
        private final PlantTargetSource shooterTransferTargetSource;
        private final PlantTargetSource flywheelTargetSource;

        private final DebounceBoolean readyLatch = DebounceBoolean.onAfterOffImmediately(cfg.readyStableSec);
        private final BooleanSource flywheelReadySource;

        private double flywheelTargetNative = 0.0;
        private double flywheelMeasuredNative = 0.0;
        private double flywheelMeasuredAbs = 0.0;
        private double flywheelMeasuredAccel = 0.0;
        private double flywheelMeasuredAccelAbs = 0.0;
        private double prevFlywheelMeasuredAbs = 0.0;

        Realization(HardwareMap hardwareMap) {
            flywheelTargetSource = PlantTargets.exact(ScalarSource.of(() ->
                    execution.isFlywheelEnabled() ? inputs.selectedVelocityNative.get() : 0.0));

            intakeMotorTargetSource = PlantTargets.overlay(ScalarSource.of(execution::baseIntakeMotorPower))
                    .add("feedPulse", execution.feedPulseActiveSource(),
                            execution.feedPulseOutputSource().scaled(cfg.feedScaleIntakeMotor))
                    .build();

            intakeTransferTargetSource = PlantTargets.overlay(ScalarSource.of(execution::baseIntakeTransferPower))
                    .add("feedPulse", execution.feedPulseActiveSource(),
                            execution.feedPulseOutputSource().scaled(cfg.feedScaleIntakeTransfer))
                    .build();

            shooterTransferTargetSource = PlantTargets.overlay(ScalarSource.of(execution::baseShooterTransferPower))
                    .add("feedPulse", execution.feedPulseActiveSource(),
                            execution.feedPulseOutputSource().scaled(cfg.feedScaleShooterTransfer))
                    .build();

            plantIntakeMotor = FtcActuators.plant(hardwareMap)
                    .motor(cfg.nameMotorIntake, cfg.directionMotorIntake)
                    .power()
                    .targetedBy(intakeMotorTargetSource)
                    .build();

            plantIntakeTransfer = FtcActuators.plant(hardwareMap)
                    .crServo(cfg.nameCrServoIntakeTransfer, cfg.directionCrServoIntakeTransfer)
                    .power()
                    .targetedBy(intakeTransferTargetSource)
                    .build();

            plantShooterTransfer = FtcActuators.plant(hardwareMap)
                    .crServo(cfg.nameCrServoShooterTransferRight, cfg.directionCrServoShooterTransferRight)
                    .andCrServo(cfg.nameCrServoShooterTransferLeft, cfg.directionCrServoShooterTransferLeft)
                    .scale(cfg.shooterTransferLeftScale)
                    .bias(cfg.shooterTransferLeftBias)
                    .power()
                    .targetedBy(shooterTransferTargetSource)
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
                        .targetedBy(flywheelTargetSource)
                        .build();
            } else {
                plantFlywheel = FtcActuators.plant(hardwareMap)
                        .motor(cfg.nameMotorShooterWheel, cfg.directionMotorShooterWheel)
                        .velocity()
                        .deviceManagedWithDefaults()
                        .bounded(0.0, cfg.velocityMax)
                        .nativeUnits()
                        .velocityTolerance(cfg.velocityToleranceNative)
                        .targetedBy(flywheelTargetSource)
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

                    if (!execution.isFlywheelEnabled()) {
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
        }

        BooleanSource flywheelReadySource() {
            return flywheelReadySource;
        }

        void resetFlywheelReady() {
            flywheelReadySource.reset();
        }

        void updateFlywheel(LoopClock clock) {
            plantFlywheel.update(clock);
            flywheelTargetNative = plantFlywheel.getRequestedTarget();

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

        void updateFeedAndTransfer(LoopClock clock) {
            plantIntakeMotor.update(clock);
            plantIntakeTransfer.update(clock);
            plantShooterTransfer.update(clock);
        }

        double flywheelTargetNative() {
            return flywheelTargetNative;
        }

        double flywheelMeasuredNative() {
            return flywheelMeasuredNative;
        }

        double flywheelMeasuredAbs() {
            return flywheelMeasuredAbs;
        }

        double flywheelMeasuredAccel() {
            return flywheelMeasuredAccel;
        }

        double flywheelMeasuredAccelAbs() {
            return flywheelMeasuredAccelAbs;
        }

        boolean flywheelAtTarget() {
            return plantFlywheel.atTarget();
        }

        void stop() {
            resetFlywheelReady();
            flywheelTargetNative = 0.0;
            flywheelMeasuredNative = 0.0;
            flywheelMeasuredAbs = 0.0;
            flywheelMeasuredAccel = 0.0;
            flywheelMeasuredAccelAbs = 0.0;
            prevFlywheelMeasuredAbs = 0.0;
            plantFlywheel.stop();
            plantIntakeMotor.stop();
            plantIntakeTransfer.stop();
            plantShooterTransfer.stop();
        }

        void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "realization" : prefix;
            dbg.addData(p + ".class", "ScoringRealization")
                    .addData(p + ".flywheelTargetNative", flywheelTargetNative)
                    .addData(p + ".flywheelMeasuredNative", flywheelMeasuredNative)
                    .addData(p + ".flywheelMeasuredAccel", flywheelMeasuredAccel)
                    .addData(p + ".flywheelMeasuredAccelAbs", flywheelMeasuredAccelAbs);
        }
    }

    private final PhoenixProfile.ScoringPathConfig cfg;
    private final LoopClock clock;
    private final ScoringTargeting targeting;
    private final BooleanSource aimOkToShoot;
    private final BooleanSource shootOverride;
    private final String flywheelPidfWarning;

    private final Inputs inputs;
    private final Execution execution;
    private final Realization realization;

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
        this.flywheelPidfWarning = null;

        this.inputs = new Inputs(cfg.velocityMin);
        this.execution = new Execution();
        this.realization = new Realization(hardwareMap);
        this.lastStatus = buildStatus(null);
    }

    @Override
    public void setIntakeEnabled(boolean enabled) {
        if (inputs.intakeRequested.get() == enabled) {
            return;
        }
        inputs.intakeRequested.set(enabled);
    }

    @Override
    public void setFlywheelEnabled(boolean enabled) {
        if (inputs.flywheelRequested.get() == enabled) {
            return;
        }
        inputs.flywheelRequested.set(enabled);
    }

    @Override
    public void setShootingEnabled(boolean enabled) {
        if (inputs.shootingRequested.get() == enabled) {
            return;
        }
        inputs.shootingRequested.set(enabled);
    }

    @Override
    public void setEjectEnabled(boolean enabled) {
        if (inputs.ejectRequested.get() == enabled) {
            return;
        }
        inputs.ejectRequested.set(enabled);
    }

    @Override
    public void requestSingleShot() {
        requestShots(1);
    }

    @Override
    public void requestShots(int shotCount) {
        inputs.shotRequests.request(shotCount);
    }

    @Override
    public void cancelTransientActions() {
        inputs.transientCancelRequests.request();
    }

    @Override
    public void setSelectedVelocityNative(double velocityNative) {
        inputs.selectedVelocityNative.set(clampVelocity(velocityNative));
    }

    @Override
    public void adjustSelectedVelocityNative(double deltaNative) {
        setSelectedVelocityNative(inputs.selectedVelocityNative.get() + deltaNative);
    }

    @Override
    public void captureSuggestedShotVelocity() {
        setSelectedVelocityNative(targeting.suggestedVelocityNative(clock, inputs.selectedVelocityNative.get()));
    }

    @Override
    public boolean hasPendingShots() {
        return execution.hasPendingShots();
    }

    @Override
    public Status status() {
        return lastStatus;
    }

    /**
     * Returns the memoized flywheel-ready gate for the current loop.
     */
    public BooleanSource flywheelReady() {
        return realization.flywheelReadySource();
    }

    /**
     * Advances scoring inputs, execution policy, flywheel readback, and final actuator writes.
     *
     * <p>The update is intentionally split into two execution/realization phases:
     * execution first resolves held and pending inputs into flywheel/feed behavior, realization then
     * updates the flywheel, execution advances the feed queue using that fresh flywheel readback,
     * and realization finally writes the feed plants.</p>
     */
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");

        execution.updateBeforeFlywheel(clock);
        realization.updateFlywheel(clock);
        execution.updateAfterFlywheel(clock);
        realization.updateFeedAndTransfer(clock);
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
        inputs.debugDump(dbg, p + ".inputs");
        execution.debugDump(dbg, p + ".execution");
        realization.debugDump(dbg, p + ".realization");
    }

    /**
     * Stops all scoring-path outputs and clears transient loop state.
     *
     * <p>This clears held motion requests and pending shot work, but it intentionally preserves the
     * selected flywheel velocity so the next activation can reuse the operator's last selection.</p>
     */
    public void stop() {
        inputs.stopMotionRequests();
        execution.stop();
        realization.stop();
        lastStatus = buildStatus(null);
    }

    private Status buildStatus(LoopClock clockOrNull) {
        double err = realization.flywheelMeasuredNative() - realization.flywheelTargetNative();
        double targetAbs = Math.abs(realization.flywheelTargetNative());
        double leadSec = Math.max(0.0, cfg.readyPredictLeadSec);
        double predictedAbs = realization.flywheelMeasuredAbs() + realization.flywheelMeasuredAccel() * leadSec;
        if (predictedAbs < 0.0) {
            predictedAbs = 0.0;
        }

        int feedBacklog = execution.feedBacklogCount();
        boolean shootActive = inputs.shootingRequested.get() || execution.hasPendingShots();
        boolean ready = clockOrNull != null && flywheelReady().getAsBoolean(clockOrNull);

        return new Status(
                inputs.intakeRequested.get(),
                inputs.ejectRequested.get(),
                inputs.shootingRequested.get(),
                inputs.flywheelRequested.get(),
                shootActive,
                feedBacklog,
                execution.modeName(),
                execution.isFlywheelEnabled(),
                cfg.applyFlywheelVelocityPIDF,
                flywheelPidfWarning,
                inputs.selectedVelocityNative.get(),
                realization.flywheelTargetNative(),
                realization.flywheelMeasuredNative(),
                err,
                Math.abs(err),
                cfg.velocityToleranceNative,
                cfg.velocityToleranceBelowNative,
                cfg.velocityToleranceAboveNative,
                realization.flywheelMeasuredAccel(),
                realization.flywheelMeasuredAccelAbs(),
                leadSec,
                predictedAbs,
                predictedAbs - targetAbs,
                realization.flywheelAtTarget(),
                ready,
                execution.queuedFeedTasks(),
                execution.isFeedPulseActive(),
                clockOrNull != null ? execution.feedPulseOutput() : 0.0
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
}
