package edu.ftcphoenix.robots.phoenix.autonomous;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Reusable autonomous task snippets expressed over {@link PhoenixCapabilities}.
 *
 * <p>These helpers are the Auto-side sibling of TeleOp button bindings: they describe high-level
 * Phoenix intents such as "aim and request one shot" without reaching into raw shooter, targeting,
 * or scoring-path internals.</p>
 */
public final class PhoenixAutoTasks {

    private PhoenixAutoTasks() {
        // Utility class.
    }

    /**
     * Build drive-guidance aim config from Phoenix's Auto profile section.
     */
    public static DriveGuidanceTask.Config aimConfig(PhoenixProfile.AutoConfig autoCfg) {
        PhoenixProfile.AutoConfig auto = cfgOrDefault(autoCfg);
        DriveGuidanceTask.Config cfg = new DriveGuidanceTask.Config();
        cfg.headingTolRad = Math.toRadians(auto.aimHeadingToleranceDeg);
        cfg.timeoutSec = auto.aimTimeoutSec;
        cfg.maxNoGuidanceSec = auto.aimMaxNoGuidanceSec;
        return cfg;
    }

    /**
     * Build one truthful scoring attempt: wait briefly for a selected target, capture shot
     * velocity, run Phoenix aim, request one shot, and wait for the shot queue to drain.
     *
     * <p>Each phase must succeed before the next phase begins. Target-selection, aiming, and shot-
     * drain timeouts remain {@link edu.ftcphoenix.fw.task.TaskOutcome#TIMEOUT}; cancellation and
     * unknown terminal outcomes likewise remain visible to the owning Auto routine. If an abnormal
     * ending occurs after the shot request, the Task best-effort asks scoring to cancel that
     * transient action.</p>
     */
    public static Task aimAndShootOne(PhoenixCapabilities capabilities,
                                      DriveCommandSink driveSink,
                                      PhoenixProfile.AutoConfig autoCfg) {
        Objects.requireNonNull(capabilities, "capabilities");
        Objects.requireNonNull(driveSink, "driveSink");
        final PhoenixProfile.AutoConfig auto = cfgOrDefault(autoCfg);
        final PhoenixCapabilities.Scoring scoring = capabilities.scoring();
        final PhoenixCapabilities.Targeting targeting = capabilities.targeting();

        Task waitForTargetTask = Tasks.waitUntil(new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return targeting.status(clock).selection.hasSelection;
            }
        }, auto.waitForTargetSec);
        Task aimTask = targeting.aimTask(driveSink, aimConfig(auto));
        Task waitForShotTask = Tasks.waitUntil(new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return !scoring.hasPendingShots();
            }
        }, auto.waitForShotCompleteSec);

        return new PhoenixScoringAttemptTask(
                scoring,
                waitForTargetTask,
                aimTask,
                waitForShotTask
        );
    }

    /**
     * Create a task that spins down the flywheel request.
     */
    public static Task disableFlywheel(final PhoenixCapabilities capabilities) {
        Objects.requireNonNull(capabilities, "capabilities");
        return Tasks.runOnce(new Runnable() {
            @Override
            public void run() {
                capabilities.scoring().setFlywheelEnabled(false);
            }
        });
    }

    private static PhoenixProfile.AutoConfig cfgOrDefault(PhoenixProfile.AutoConfig autoCfg) {
        return autoCfg == null ? new PhoenixProfile.AutoConfig() : autoCfg;
    }
}
