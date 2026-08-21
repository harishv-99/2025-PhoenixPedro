package edu.ftcphoenix.robots.examples.reference.tester;

/** Team-authored physical gate for the checked-in flywheel spin-up experiment. */
final class ReferenceFlywheelSpinUpCriteria {
    boolean reviewedForMotion;
    double targetVelocityTicksPerSec;

    /**
     * Cooperative elapsed-time boundary checked once per active tester loop.
     *
     * <p>The experiment requests and applies zero on the first loop observed at or after this
     * boundary; this is not a hard real-time cutoff. The reviewed lab card and STOP plan must allow
     * for the worst-case active-loop delay.</p>
     */
    double maximumPoweredRunSec;

    private ReferenceFlywheelSpinUpCriteria() {
    }

    /** Returns the checked-in locked card with software-valid placeholders, not safety facts. */
    static ReferenceFlywheelSpinUpCriteria current() {
        ReferenceFlywheelSpinUpCriteria criteria = new ReferenceFlywheelSpinUpCriteria();
        criteria.reviewedForMotion = false;
        criteria.targetVelocityTicksPerSec = 3000.0;
        criteria.maximumPoweredRunSec = 3.0;
        return criteria;
    }
}
