package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

/** Data-only lift answers and the explicit motion permission used by each lift lesson. */
public final class BasicLiftProfile {

    /**
     * Complete lift wiring, mechanism-inch coordinate, legal range, named heights, power limit,
     * reference search, and Task timeouts. The owned config documents each field's exact units and
     * polarity.
     */
    public BasicLiftMechanism.Config lift;

    /**
     * Whether supervised lift motion and homing are permitted after wiring, polarity, limits,
     * clearance, and physical STOP behavior have been reviewed. {@link #current()} returns false.
     */
    public boolean allowLiftMotion;

    private BasicLiftProfile() {
        // Start from current() so every required lift answer is populated together.
    }

    /** Returns a fresh complete software baseline, not reviewed physical robot facts. */
    public static BasicLiftProfile current() {
        BasicLiftProfile profile = new BasicLiftProfile();
        profile.lift = BasicLiftMechanism.Config.defaults();
        profile.allowLiftMotion = false;
        return profile;
    }

    /** Rejects unchecked lift motion before a host performs any hardware lookup. */
    static void requireMotionAllowed(BasicLiftProfile profile, String mode) {
        BasicLiftProfile p = Objects.requireNonNull(profile, "liftProfile");
        if (!p.allowLiftMotion) {
            throw new IllegalStateException(
                    "BasicLiftProfile.allowLiftMotion must be true before " + mode
                            + " may construct a motion-capable lift. Review the motor and active-low "
                            + "switch names, direction, limits, and small supervised motion first; "
                            + "then verify physical STOP.");
        }
    }
}
