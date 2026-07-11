package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

/**
 * Executes a {@link DriveGuidancePlan} as an autonomous {@link Task}.
 *
 * <p>This is the task counterpart to {@link DriveGuidancePlan#overlay()}. Both modes share the
 * same underlying evaluation/controller engine (see {@link DriveGuidanceCore}), so behavior stays
 * consistent between TeleOp assist and autonomous execution.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * DriveGuidanceTask.Config cfg = new DriveGuidanceTask.Config();
 * cfg.positionTolInches = 1.0;
 * cfg.headingTolRad = Math.toRadians(4.0);
 *
 * Task autoAlign = plan.task(drivebase, cfg);
 * runner.enqueue(autoAlign);
 * }</pre>
 *
 * <p>The task can be interrupted cleanly via {@link #cancel()} or by calling
 * {@link edu.ftcphoenix.fw.task.TaskRunner#cancelAndClear()} on the owning runner.</p>
 *
 * <p>A {@code DriveGuidanceTask} instance is single-use. Create a fresh task with
 * {@link DriveGuidancePlan#task(DriveCommandSink, Config)}, a fresh macro builder, or a
 * {@code Supplier<Task>} each time guidance should run.</p>
 */
public final class DriveGuidanceTask implements Task {

    /**
     * Task-level settings (tolerances/timeouts) independent of controller tuning.
     */
    public static final class Config {

        /**
         * Position tolerance in inches (applies when translation is requested).
         */
        public double positionTolInches = 1.5;

        /**
         * Heading tolerance in radians (applies when omega is requested).
         */
        public double headingTolRad = Math.toRadians(6.0);

        /**
         * Overall timeout for the task.
         */
        public double timeoutSec = 3.0;

        /**
         * How long we will tolerate having no usable guidance command before timing out.
         *
         * <p>The consecutive interval begins when the task starts, or on the first no-command
         * loop after usable guidance was last available. Time from before that boundary is not
         * charged to this timeout.</p>
         */
        public double maxNoGuidanceSec = 0.35;

        /**
         * Optional override: which DOFs the task should actively drive.
         *
         * <p>If null, the task defaults to {@link DriveGuidancePlan#requestedMask()}.</p>
         */
        public DriveOverlayMask requestedMask = null;
    }

    private final String debugName;
    private final DriveCommandSink drivebase;
    private final DriveGuidancePlan plan;
    private final Config cfg;

    private final DriveGuidanceCore core;

    private boolean startAttempted = false;
    private boolean started = false;
    private boolean complete = false;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;

    private double startTimeSec = 0.0;
    private double noGuidanceStartSec = Double.NaN;
    private double noGuidanceSec = 0.0;

    private double lastTranslationErrorIn = Double.NaN;
    private double lastOmegaErrorRad = Double.NaN;

    /**
     * Creates a named autonomous task that executes the supplied guidance plan.
     */
    public DriveGuidanceTask(String debugName,
                             DriveCommandSink drivebase,
                             DriveGuidancePlan plan,
                             Config cfg) {
        this.debugName = (debugName != null && !debugName.isEmpty()) ? debugName : "DriveGuidanceTask";
        this.drivebase = Objects.requireNonNull(drivebase, "drivebase");
        this.plan = Objects.requireNonNull(plan, "plan");
        this.cfg = (cfg != null) ? cfg : new Config();
        this.core = new DriveGuidanceCore(plan);
    }

    /**
     * Creates an autonomous guidance task with a default debug name.
     */
    public DriveGuidanceTask(DriveCommandSink drivebase,
                             DriveGuidancePlan plan,
                             Config cfg) {
        this("DriveGuidanceTask", drivebase, plan, cfg);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getDebugName() {
        return debugName;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start(LoopClock clock) {
        if (startAttempted) {
            throw new IllegalStateException("DriveGuidanceTask '" + debugName
                    + "' is single-use and has already been started. Create a fresh task with "
                    + "DriveGuidancePlan.task(...), a fresh macro builder, or a Supplier<Task> "
                    + "for each run.");
        }
        startAttempted = true;
        started = true;
        complete = false;
        outcome = TaskOutcome.NOT_DONE;

        startTimeSec = (clock != null) ? clock.nowSec() : 0.0;
        noGuidanceStartSec = (clock != null) ? clock.nowSec() : Double.NaN;
        noGuidanceSec = 0.0;
        lastTranslationErrorIn = Double.NaN;
        lastOmegaErrorRad = Double.NaN;

        core.onEnable();
        if (complete) {
            return;
        }
        drivebase.stop();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw new IllegalStateException("DriveGuidanceTask '" + debugName + "' cannot be "
                    + "updated before start(clock). Start it first, normally by enqueueing it in "
                    + "a TaskRunner.");
        }
        if (complete) {
            return;
        }
        if (clock == null) {
            // Defensive: no clock means no safe control.
            drivebase.stop();
            return;
        }

        drivebase.update(clock);
        if (complete) {
            return;
        }

        // Hard timeout.
        double elapsed = clock.nowSec() - startTimeSec;
        if (cfg.timeoutSec > 0.0 && elapsed > cfg.timeoutSec) {
            complete = true;
            outcome = TaskOutcome.TIMEOUT;
            drivebase.stop();
            return;
        }

        DriveOverlayMask requested = (cfg.requestedMask != null) ? cfg.requestedMask : plan.requestedMask();
        DriveGuidanceCore.Step step = core.step(clock, requested);
        if (complete) {
            return;
        }

        // No usable command this loop.
        if (step.out.mask.isNone()) {
            drivebase.stop();
            if (complete) {
                return;
            }
            if (!Double.isFinite(noGuidanceStartSec)) {
                noGuidanceStartSec = clock.nowSec();
            }
            noGuidanceSec = Math.max(0.0, clock.nowSec() - noGuidanceStartSec);
            if (cfg.maxNoGuidanceSec > 0.0 && noGuidanceSec > cfg.maxNoGuidanceSec) {
                complete = true;
                outcome = TaskOutcome.TIMEOUT;
            }
            return;
        }

        noGuidanceStartSec = Double.NaN;
        noGuidanceSec = 0.0;
        drivebase.drive(step.out.signal);
        if (complete) {
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
            complete = true;
            outcome = TaskOutcome.SUCCESS;
            drivebase.stop();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void cancel() {
        if (!started || complete) {
            return;
        }
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        drivebase.stop();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isComplete() {
        return complete;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TaskOutcome getOutcome() {
        return outcome;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        Task.super.debugDump(dbg, prefix);
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "task" : prefix;

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
