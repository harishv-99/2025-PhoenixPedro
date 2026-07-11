package edu.ftcphoenix.fw.task;

/** Builds consistent, actionable errors for internal Task lifecycle guards. */
final class TaskLifecycle {

    private TaskLifecycle() {
    }

    /**
     * Create the standard error for a direct update that bypasses the required first start.
     */
    static IllegalStateException updateBeforeStart(String taskName) {
        String name = (taskName == null || taskName.isEmpty()) ? "Task" : taskName;
        return new IllegalStateException(
                name + ".update(clock) was called before start(clock). Start the task once before "
                        + "updating it, normally by enqueuing it in a TaskRunner.");
    }
}
