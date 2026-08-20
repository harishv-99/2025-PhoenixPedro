package edu.ftcphoenix.robots.examples.reference.tester;

/** Team-authored physical gate for the checked-in launcher experiment. */
public final class ReferenceExperimentCriteria {
    public boolean reviewedForMotion;
    public double targetVelocity;
    public double maximumTrialSec;

    private ReferenceExperimentCriteria() {
    }

    /** Returns a locked card with software-valid placeholders, not physical success criteria. */
    public static ReferenceExperimentCriteria locked() {
        ReferenceExperimentCriteria criteria = new ReferenceExperimentCriteria();
        criteria.reviewedForMotion = false;
        criteria.targetVelocity = 3000.0;
        criteria.maximumTrialSec = 3.0;
        return criteria;
    }
}
