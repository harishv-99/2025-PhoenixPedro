package edu.ftcphoenix.robots.examples.reference.tester;

import edu.ftcphoenix.fw.tools.tester.StandardTesters;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.robots.examples.reference.robot.ReferenceProfile;

/** Builds the reference robot's experiment menu beside the canonical framework tools. */
public final class ReferenceRobotTesters {
    private ReferenceRobotTesters() {
    }

    /** Returns a fresh tester tree; motion experiments are locked in checked-in configuration. */
    public static TesterSuite create() {
        ReferenceProfile profile = ReferenceProfile.current();
        ReferenceExperimentCriteria criteria = ReferenceExperimentCriteria.locked();
        TesterSuite suite = new TesterSuite()
                .setTitle("Reference experiments")
                .setHelp("Review the lab card before unlocking motion");
        suite.add("Launcher spin-up",
                "Computed controller evidence; operator records physical outcome",
                criteria.reviewedForMotion ? "READY" : "LOCKED",
                () -> new ReferenceLauncherExperiment(
                        profile.launcher, criteria));
        StandardTesters.register(suite);
        return suite;
    }
}
