package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Package-local exact-success continuation shared by the two independent Auto lessons. */
final class BasicAutoSuccessGate {

    private BasicAutoSuccessGate() {
        // Static task-composition helper.
    }

    /** Retains every terminal outcome and constructs the continuation only after exact success. */
    static Task continueOnlyAfterSuccess(String debugName,
                                         Task prerequisite,
                                         Supplier<? extends Task> continuation) {
        Task requiredPrerequisite = Objects.requireNonNull(prerequisite, "prerequisite");
        Supplier<? extends Task> requiredContinuation = Objects.requireNonNull(
                continuation, "continuation");

        // This built-in composition starts iteration two only after iteration one reports SUCCESS;
        // TIMEOUT, CANCELLED, and UNKNOWN remain the exact terminal result.
        return Tasks.repeatWhileSuccessful(
                debugName,
                2,
                BooleanSource.constant(true),
                new Supplier<Task>() {
                    private int iteration;

                    @Override
                    public Task get() {
                        if (iteration++ == 0) {
                            return requiredPrerequisite;
                        }
                        return requiredContinuation.get();
                    }
                });
    }
}
