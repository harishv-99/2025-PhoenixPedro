package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.task.Task;

/**
 * Convenience factory methods for building tasks that execute {@link DriveGuidancePlan}s.
 *
 * <p>This is a thin utility wrapper over {@link DriveGuidanceTask}. It exists so robot code can
 * read cleanly at the call site while still depending only on the small {@link
 * edu.ftcphoenix.fw.drive.DriveCommandSink} seam.</p>
 */
public final class DriveGuidanceTasks {

    private DriveGuidanceTasks() {
        // utility
    }

    public static Task driveGuidance(DriveCommandSink drivebase,
                                     DriveGuidancePlan plan,
                                     DriveGuidanceTask.Config cfg) {
        return new DriveGuidanceTask(drivebase, plan, cfg);
    }

    public static Task driveGuidance(String debugName,
                                     DriveCommandSink drivebase,
                                     DriveGuidancePlan plan,
                                     DriveGuidanceTask.Config cfg) {
        return new DriveGuidanceTask(debugName, drivebase, plan, cfg);
    }
}
