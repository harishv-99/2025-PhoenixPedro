package edu.ftcsushi.robots.examples.reference.control;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;

/**
 * A single-attempt TeleOp policy: extra feed presses never create a backlog of feed attempts.
 * The managed program owns scheduling; the launcher owns all feeding and recovery decisions.
 */
public final class ReferenceFeedingControls {
    private final GamepadDevice operator;
    private boolean bindAttempted;
    private Task lastFeed;
    private ReferenceLauncher.RecoveryResult lastRecoveryResult;

    /** Retains stable gamepad sources without registering callbacks or requesting motion. */
    public ReferenceFeedingControls(GamepadDevice operator) {
        this.operator = Objects.requireNonNull(operator, "operator is required");
    }

    /**
     * Registers A feed, B abort, and X no-motion acknowledgement once during configuration.
     *
     * <p>The original A input owns edge detection. Filtering that input with availability would
     * manufacture a new press when availability changed while A remained held. B invalidates
     * already-created attempts through the capability; it does not erase the program's queue.
     * Another explicit A press is needed after the retained attempt has ended.</p>
     */
    public void bind(CallbackBindings callbacks, TaskBindings tasks, ReferenceLauncher launcher) {
        Objects.requireNonNull(tasks, "tasks are required");
        Objects.requireNonNull(callbacks, "callbacks are required");
        Objects.requireNonNull(launcher, "launcher is required");
        if (bindAttempted) {
            throw new IllegalStateException("Bind ReferenceFeedingControls once; create a fresh "
                    + "controls owner for another program.");
        }
        bindAttempted = true;
        tasks.onRise(operator.a(), () -> createFeedUnlessPending(launcher));
        callbacks.onRise(operator.b(), launcher::abortFeedAttempts);
        callbacks.onRise(operator.x(),
                () -> lastRecoveryResult = launcher.acknowledgeRecovery());
    }

    /** Last acknowledgement decision, or null before X has requested one; this does not sample. */
    public ReferenceLauncher.RecoveryResult lastRecoveryResult() {
        return lastRecoveryResult;
    }

    /** Ignores a press while the previous feed is queued or active; it never restarts that Task. */
    private Task createFeedUnlessPending(ReferenceLauncher launcher) {
        if (lastFeed != null && !lastFeed.isComplete()) {
            return Tasks.noop();
        }
        lastFeed = launcher.feedOne();
        return lastFeed;
    }
}
