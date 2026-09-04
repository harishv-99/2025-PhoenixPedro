package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.core.hal.Direction;

/** Data-only claw answers and the explicit motion permission used by each claw lesson. */
public final class BasicClawProfile {

    /**
     * Complete standard-servo wiring, direction, initial semantic request, and CLOSED/OPEN native
     * Servo endpoints in {@code [0, 1]}. HALF is derived between those endpoints; none of these
     * command positions are feedback measurements.
     */
    public BasicClawMechanism.Config claw;

    /**
     * Whether supervised Servo motion is permitted after direction, endpoints, clearance, and
     * initial CLOSED motion have been reviewed. {@link #current()} always returns {@code false}.
     */
    public boolean allowClawMotion;

    private BasicClawProfile() {
        // Start from current() so every required claw answer is populated together.
    }

    /** Returns a fresh complete software baseline, not reviewed physical robot facts. */
    public static BasicClawProfile current() {
        BasicClawProfile profile = new BasicClawProfile();
        profile.claw = BasicClawMechanism.Config.defaults();
        profile.claw.servoName = "clawServo";
        profile.claw.direction = Direction.FORWARD;
        profile.claw.closedNativePosition = 0.25;
        profile.claw.openNativePosition = 0.70;
        profile.claw.initialState = BasicClaw.State.CLOSED;
        profile.allowClawMotion = false;
        return profile;
    }

    /** Rejects unchecked claw motion before a host performs any hardware lookup. */
    static void requireMotionAllowed(BasicClawProfile profile, String mode) {
        BasicClawProfile p = Objects.requireNonNull(profile, "clawProfile");
        if (!p.allowClawMotion) {
            throw new IllegalStateException(
                    "BasicClawProfile.allowClawMotion must be true before " + mode
                            + " may construct a motion-capable claw. Review the Servo name, "
                            + "direction, endpoints, initial CLOSED motion, and physical STOP "
                            + "behavior first.");
        }
    }
}
