package edu.ftcsushi.robots.examples.reference.tester;

import edu.ftcsushi.fw.tools.tester.StandardTesters;
import edu.ftcsushi.fw.tools.tester.TesterSuite;
import edu.ftcsushi.robots.examples.reference.robot.ReferenceProfile;

/** Builds the reference robot's experiment menu beside the canonical framework tools. */
public final class ReferenceRobotTesters {
    private ReferenceRobotTesters() {
    }

    /** Returns a fresh tester tree; motion experiments are locked in checked-in configuration. */
    public static TesterSuite create() {
        ReferenceProfile profile = ReferenceProfile.current();
        ReferenceFlywheelSpinUpCriteria criteria = ReferenceFlywheelSpinUpCriteria.current();
        TesterSuite suite = new TesterSuite()
                .setTitle("Reference experiments")
                .setHelp("Review the lab card before unlocking motion");
        suite.add("Flywheel spin-up",
                "Independent per-wheel velocity evidence",
                criteria.reviewedForMotion ? "READY" : "LOCKED",
                () -> new ReferenceFlywheelSpinUpExperiment(
                        profile.launcher, criteria));
        StandardTesters.register(suite);
        return suite;
    }
}
