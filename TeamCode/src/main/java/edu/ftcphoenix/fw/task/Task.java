package edu.ftcphoenix.fw.task;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A cooperative, stateful unit of work that is driven by the main robot loop.
 *
 * <p>Typical lifecycle:</p>
 * <ol>
 *   <li>{@link #start(LoopClock)} is called once when the task is first scheduled.</li>
 *   <li>{@link #update(LoopClock)} is called every loop while {@link #isComplete()} returns
 *       {@code false}.</li>
 *   <li>Optional early-exit: {@link #cancel()} may be called when the owning code wants to stop the
 *       task before normal completion.</li>
 *   <li>Once {@link #isComplete()} returns {@code true}, the task is considered finished and will
 *       no longer receive updates.</li>
 * </ol>
 *
 * <p>Tasks are intended to be used with a runner such as {@link TaskRunner}, which manages calling
 * {@code start()}, {@code update()}, and checking {@code isComplete()} each iteration.</p>
 *
 * <h2>Single-use lifecycle</h2>
 * <p>A Task instance may enter {@link #start(LoopClock)} once. Framework Tasks throw an actionable
 * {@link IllegalStateException} if the same instance is started again, whether it is still active
 * or already complete. Build a fresh Task for repeated behavior using the relevant builder or
 * macro method, a {@code Supplier<Task>}, or an {@link OutputTaskFactory}.</p>
 *
 * <p>A runner may call {@code start(clock)} and the first {@code update(clock)} in the same loop
 * cycle. In that case {@link LoopClock#dtSec()} describes the loop interval before the task
 * started. A task-owned timer should capture {@link LoopClock#nowSec()} when its interval begins
 * and compare later {@code nowSec()} values instead of charging that pre-start delta.</p>
 *
 * <h2>Cancellation and direct-call lifecycle</h2>
 * <p>Framework Tasks treat cancellation before their first start as a side-effect-free no-op.
 * Cancellation while active is terminal, and cancellation after completion or repeated
 * cancellation is also a no-op. A direct {@link #update(LoopClock)} before start is a lifecycle
 * error; {@link Tasks#noop()} is the intentional always-success exception. Custom Tasks should
 * follow the same rules so runners and composites can clean them up predictably.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * Task task = Tasks.sequence(
 *         Tasks.waitUntil(driverReady),
 *         Tasks.runForSeconds(0.4, shooter::startFeed, shooter::stopFeed)
 * );
 *
 * runner.enqueue(task);
 * // later in the loop...
 * runner.update(clock);
 * }</pre>
 */
public interface Task {

    /**
     * Called once when the task is first started.
     *
     * <p>Implementations should perform any initialization here, including capturing the initial
     * time from {@link LoopClock} if needed. Framework implementations are single-use and reject a
     * second call rather than silently restarting or skipping behavior.</p>
     *
     * @param clock loop timing information for the current iteration
     */
    void start(LoopClock clock);

    /**
     * Called once per loop while the task is running.
     *
     * <p>Implementations should advance their internal state based on the information in
     * {@link LoopClock}, and may mark themselves complete by causing {@link #isComplete()} to
     * return {@code true}. The first update may share a cycle with {@link #start(LoopClock)}, so
     * task-owned elapsed intervals should be measured from a start-time anchor rather than assuming
     * the current {@link LoopClock#dtSec()} occurred while the task was active. Framework Tasks
     * other than the already-complete {@link Tasks#noop()} reject a direct update before their
     * first start with an actionable lifecycle error.</p>
     *
     * @param clock loop timing information for the current iteration
     * @throws IllegalStateException if a framework Task other than {@link Tasks#noop()} is updated
     *                               before its first start
     */
    void update(LoopClock clock);

    /**
     * Optional early-stop hook.
     *
     * <p>The default implementation is a no-op, which is appropriate for tasks that complete in
     * {@link #start(LoopClock)} and own no temporary external state. Any custom Task that can remain
     * active after start must override this method so active cancellation makes its own lifecycle
     * terminal, even when runner detachment would otherwise stop later updates.</p>
     *
     * <p>Tasks that command hardware, own child tasks, or need to report a cancellation outcome
     * should override this method. Cancellation before start must not acquire, release, or mutate
     * task-owned resources. Active cancellation must make the task terminal before cleanup that may
     * throw. Cancellation after completion and repeated cancellation must be no-ops.</p>
     */
    default void cancel() {
        // default no-op
    }

    /**
     * @return {@code true} once the task has finished and no longer needs to receive
     *         {@link #update(LoopClock)} calls.
     */
    boolean isComplete();

    /**
     * @return a short human-readable label for debugging. The default implementation returns the
     *         simple class name.
     */
    default String getDebugName() {
        return getClass().getSimpleName();
    }

    /**
     * Returns the outcome of this task, if it exposes one.
     *
     * <p>The default implementation returns {@link TaskOutcome#UNKNOWN}, which is appropriate for
     * simple tasks that do not distinguish between different terminal states.</p>
     *
     * <p>Tasks that care about outcomes (for example, that may finish with success vs timeout vs
     * cancellation) should override this method and follow this convention:</p>
     * <ul>
     *   <li>While the task is still running (before {@link #isComplete()} becomes {@code true}),
     *       return {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>Once the task has completed, return a terminal value such as
     *       {@link TaskOutcome#SUCCESS}, {@link TaskOutcome#TIMEOUT}, or
     *       {@link TaskOutcome#CANCELLED}.</li>
     * </ul>
     *
     * @return the current outcome for this task
     */
    TaskOutcome getOutcome();

    /**
     * Debug helper: emit a compact summary of this task.
     *
     * <p>This is intentionally lightweight and safe to call every loop. Tasks with meaningful
     * internal state should override this method to provide richer telemetry.</p>
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. {@code "auto.task"}
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "task" : prefix;

        dbg.addData(p + ".name", getDebugName())
                .addData(p + ".class", getClass().getSimpleName())
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());
    }
}
