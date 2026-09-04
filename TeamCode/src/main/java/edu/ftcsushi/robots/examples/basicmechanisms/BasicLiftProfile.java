package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.core.hal.Direction;

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
        profile.lift.motorName = "liftMotor";
        profile.lift.direction = Direction.FORWARD;
        profile.lift.bottomSwitchName = "liftBottom";
        profile.lift.maximumHeightIn = 18.0;
        profile.lift.ticksPerIn = 100.0;
        profile.lift.toleranceIn = 0.20;
        profile.lift.maximumPower = 0.30;
        profile.lift.stowedHeightIn = 0.0;
        profile.lift.lowHeightIn = 4.0;
        profile.lift.highHeightIn = 14.0;
        profile.lift.homingPower = -0.15;
        profile.lift.homingTimeoutSec = 3.0;
        profile.lift.moveTimeoutSec = 2.0;
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
