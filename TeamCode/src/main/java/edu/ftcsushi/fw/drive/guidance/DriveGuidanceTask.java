package edu.ftcsushi.fw.drive.guidance;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.task.AbstractTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/**
 * Executes a {@link DriveGuidancePlan} as an autonomous {@link Task}.
 *
 * <p>This is the task counterpart to {@link DriveGuidancePlan#overlay()}. Both modes share the
 * same underlying evaluation/controller engine (see {@link DriveGuidanceCore}), so behavior stays
 * consistent between TeleOp assist and autonomous execution.</p>
 *
 * <p>Ordinary managed Auto usage:</p>
 * <pre>{@code
 * DriveGuidanceTask.Config cfg = new DriveGuidanceTask.Config();
 * cfg.positionTolInches = 1.0;
 * cfg.headingTolRad = Math.toRadians(4.0);
 *
 * Task autoAlign = plan.task(drivebase, cfg);
 * program.rootTask(autoAlign);
 * }</pre>
 *
 * <p>The managed program cancels its root Task during shutdown. A custom or private host,
 * framework tool, or test that explicitly owns a runner may interrupt the Task via
 * {@link #cancel()} or
 * {@link edu.ftcsushi.fw.task.TaskRunner#cancelAndClear()}.</p>
 *
 * <p>The Task calls {@link DriveCommandSink#update(LoopClock)} while active. A stateful external
 * drive adapter that also needs a heartbeat during mechanism/wait phases must have one stable
 * composition-root owner and make same-cycle calls idempotent.</p>
 *
 * <p>A {@code DriveGuidanceTask} instance is single-use. Create a fresh task with
 * {@link DriveGuidancePlan#task(DriveCommandSink, Config)}, a fresh macro builder, or a
 * {@code Supplier<Task>} each time guidance should run.</p>
 */
public final class DriveGuidanceTask extends AbstractTask {

    /**
     * Task-level settings (tolerances/timeouts) independent of controller tuning.
     *
     * <p>This is a mutable setup value. A {@link DriveGuidanceTask} validates and defensively
     * snapshots it when the Task is created, so later mutations affect only Tasks created after
     * those mutations. Pass {@code null} to a Task factory to use the defaults below.</p>
     */
    public static final class Config {

        /**
         * Position tolerance in inches (applies when translation is requested); must be finite
         * and greater than or equal to zero.
         */
        public double positionTolInches = 1.5;

        /**
         * Heading tolerance in radians (applies when omega is requested); must be finite and
         * greater than or equal to zero.
         */
        public double headingTolRad = Math.toRadians(6.0);

        /**
         * Overall timeout for the task; must be finite and greater than zero.
         */
        public double timeoutSec = 3.0;

        /**
         * How long we will tolerate having no usable guidance command before timing out.
         *
         * <p>The consecutive interval begins when the task starts, or on the first no-command
         * loop after usable guidance was last available. Time from before that boundary is not
         * charged to this timeout. This value must be finite and greater than zero.</p>
         */
        public double maxNoGuidanceSec = 0.35;

        /**
         * Optional override: which DOFs the task should actively drive.
         *
         * <p>If null, the task defaults to {@link DriveGuidancePlan#requestedMask()}. The reference
         * is captured with the other settings when the Task is created.</p>
         */
        public DriveOverlayMask requestedMask = null;
    }

    /** Validated construction-time settings retained privately by one Task instance. */
    private static final class ConfigSnapshot {
        final double positionTolInches;
        final double headingTolRad;
        final double timeoutSec;
        final double maxNoGuidanceSec;
        final DriveOverlayMask requestedMask;

        private ConfigSnapshot(double positionTolInches,
                               double headingTolRad,
                               double timeoutSec,
                               double maxNoGuidanceSec,
                               DriveOverlayMask requestedMask) {
            this.positionTolInches = positionTolInches;
            this.headingTolRad = headingTolRad;
            this.timeoutSec = timeoutSec;
            this.maxNoGuidanceSec = maxNoGuidanceSec;
            this.requestedMask = requestedMask;
        }

        static ConfigSnapshot from(Config config) {
            Config source = config != null ? config : new Config();

            // Capture each mutable answer once so validation and retained behavior use the same
            // construction-time values.
            double positionTolInches = source.positionTolInches;
            double headingTolRad = source.headingTolRad;
            double timeoutSec = source.timeoutSec;
            double maxNoGuidanceSec = source.maxNoGuidanceSec;
            DriveOverlayMask requestedMask = source.requestedMask;

            requireFiniteNonNegative(positionTolInches, "positionTolInches");
            requireFiniteNonNegative(headingTolRad, "headingTolRad");
            requireFinitePositive(timeoutSec, "timeoutSec");
            requireFinitePositive(maxNoGuidanceSec, "maxNoGuidanceSec");

            return new ConfigSnapshot(
                    positionTolInches,
                    headingTolRad,
                    timeoutSec,
                    maxNoGuidanceSec,
                    requestedMask
            );
        }

        private static void requireFiniteNonNegative(double value, String fieldName) {
            if (!Double.isFinite(value) || value < 0.0) {
                throw new IllegalArgumentException(
                        "DriveGuidanceTask.Config." + fieldName
                                + " must be finite and >= 0, got " + value + "."
                );
            }
        }

        private static void requireFinitePositive(double value, String fieldName) {
            if (!Double.isFinite(value) || value <= 0.0) {
                throw new IllegalArgumentException(
                        "DriveGuidanceTask.Config." + fieldName
                                + " must be finite and > 0, got " + value + "."
                );
            }
        }
    }

    private final DriveCommandSink drivebase;
    private final DriveGuidancePlan plan;
    private final ConfigSnapshot cfg;

    private final DriveGuidanceCore core;

    private boolean stopInProgress;
    private boolean terminalStopAlreadyAttempted;

    private double startTimeSec = 0.0;
    private double noGuidanceStartSec = Double.NaN;
    private double noGuidanceSec = 0.0;

    private double lastTranslationErrorIn = Double.NaN;
    private double lastOmegaErrorRad = Double.NaN;

    /**
     * Creates a named autonomous task that executes the supplied guidance plan.
     */
    DriveGuidanceTask(String debugName,
                      DriveCommandSink drivebase,
                      DriveGuidancePlan plan,
                      Config cfg) {
        super(debugName != null && !debugName.trim().isEmpty() ? debugName : "DriveGuidanceTask");
        this.drivebase = Objects.requireNonNull(drivebase, "drivebase");
        this.plan = Objects.requireNonNull(plan, "plan");
        this.cfg = ConfigSnapshot.from(cfg);
        this.core = new DriveGuidanceCore(plan);
    }

    /**
     * Creates an autonomous guidance task with a default debug name.
     */
    DriveGuidanceTask(DriveCommandSink drivebase,
                      DriveGuidancePlan plan,
                      Config cfg) {
        this("DriveGuidanceTask", drivebase, plan, cfg);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onStart(LoopClock clock) {
        startTimeSec = clock.nowSec();
        noGuidanceStartSec = clock.nowSec();
        noGuidanceSec = 0.0;
        lastTranslationErrorIn = Double.NaN;
        lastOmegaErrorRad = Double.NaN;

        core.onEnable();
        if (!isActive()) {
            return;
        }
        stopDuringActiveGuidance();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onUpdate(LoopClock clock) {
        drivebase.update(clock);
        if (!isActive()) {
            return;
        }

        // Hard timeout.
        double elapsed = clock.nowSec() - startTimeSec;
        if (elapsed > cfg.timeoutSec) {
            complete(TaskOutcome.TIMEOUT);
            return;
        }

        DriveOverlayMask requested = (cfg.requestedMask != null) ? cfg.requestedMask : plan.requestedMask();
        DriveGuidanceCore.Step step = core.step(clock, requested);
        if (!isActive()) {
            return;
        }

        // No usable command this loop.
        if (step.out.mask.isNone()) {
            if (!Double.isFinite(noGuidanceStartSec)) {
                noGuidanceStartSec = clock.nowSec();
            }
            noGuidanceSec = Math.max(0.0, clock.nowSec() - noGuidanceStartSec);
            stopDuringActiveGuidance();
            if (!isActive()) {
                return;
            }
            if (noGuidanceSec > cfg.maxNoGuidanceSec) {
                // Preserve cancellation from the just-completed stop before choosing timeout.
                terminalStopAlreadyAttempted = true;
                complete(TaskOutcome.TIMEOUT);
            }
            return;
        }

        noGuidanceStartSec = Double.NaN;
        noGuidanceSec = 0.0;
        drivebase.drive(step.out.signal);
        if (!isActive()) {
            return;
        }

        // Update error bookkeeping for debug.
        lastTranslationErrorIn = step.hasTranslationError
                ? Math.hypot(step.forwardErrorIn, step.leftErrorIn)
                : Double.NaN;
        lastOmegaErrorRad = step.hasOmegaError ? step.omegaErrorRad : Double.NaN;

        boolean wantTranslation = requested.overridesTranslation();
        boolean wantOmega = requested.overridesOmega();

        boolean translationOk = !wantTranslation
                || (step.hasTranslationError && lastTranslationErrorIn <= cfg.positionTolInches);
        boolean omegaOk = !wantOmega
                || (step.hasOmegaError && Math.abs(lastOmegaErrorRad) <= cfg.headingTolRad);

        if (translationOk && omegaOk) {
            complete(TaskOutcome.SUCCESS);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onCancel() {
        // Every ending uses the same owned stop policy in onFinish().
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onFinish() {
        if (!stopInProgress && !terminalStopAlreadyAttempted) {
            terminalStopAlreadyAttempted = true;
            drivebase.stop();
        }
    }

    /** Share an in-flight ordinary stop with any ending it triggers. */
    private void stopDuringActiveGuidance() {
        // START and unavailable-guidance loops also stop normally. If that very call causes
        // cancellation or fails, it is already the terminal stop attempt: do not recurse/retry.
        stopInProgress = true;
        try {
            drivebase.stop();
        } catch (RuntimeException failure) {
            terminalStopAlreadyAttempted = true;
            throw failure;
        } finally {
            if (!isActive()) {
                terminalStopAlreadyAttempted = true;
            }
            stopInProgress = false;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void debugState(DebugSink dbg, String p) {
        dbg.addData(p + ".mode", core.lastMode());
        dbg.addData(p + ".mask", core.lastStep().out.mask.toString());
        dbg.addData(p + ".axial", core.lastStep().out.signal.axial);
        dbg.addData(p + ".lateral", core.lastStep().out.signal.lateral);
        dbg.addData(p + ".omega", core.lastStep().out.signal.omega);

        if (!Double.isNaN(lastTranslationErrorIn)) {
            dbg.addData(p + ".translationErrorIn", lastTranslationErrorIn);
        }
        if (!Double.isNaN(lastOmegaErrorRad)) {
            dbg.addData(p + ".omegaErrorRad", lastOmegaErrorRad);
        }

        dbg.addData(p + ".noGuidanceSec", noGuidanceSec)
                .addData(p + ".maxNoGuidanceSec", cfg.maxNoGuidanceSec);

        // Helpful if you want to tune takeover.
        dbg.addData(p + ".aprilTagsInRangeForTranslation", core.aprilTagsInRangeForTranslation());
        dbg.addData(p + ".blendTTranslate", core.blendTTranslate());
        dbg.addData(p + ".blendTOmega", core.blendTOmega());

    }
}
