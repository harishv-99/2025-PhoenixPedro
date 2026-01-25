package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.task.Task;

/**
 * Convenience factory methods for building tasks that execute {@link DriveGuidancePlan}s.
 */
public final class DriveGuidanceTasks {

    private DriveGuidanceTasks() {
        // utility
    }

    public static Task driveGuidance(MecanumDrivebase drivebase,
                                     DriveGuidancePlan plan,
                                     DriveGuidanceTask.Config cfg) {
        return new DriveGuidanceTask(drivebase, plan, cfg);
    }

    public static Task driveGuidance(String debugName,
                                     MecanumDrivebase drivebase,
                                     DriveGuidancePlan plan,
                                     DriveGuidanceTask.Config cfg) {
        return new DriveGuidanceTask(debugName, drivebase, plan, cfg);
    }
}
