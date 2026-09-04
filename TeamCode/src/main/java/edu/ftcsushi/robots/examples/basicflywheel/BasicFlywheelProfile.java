package edu.ftcsushi.robots.examples.basicflywheel;

import java.util.Objects;

import edu.ftcsushi.fw.core.hal.Direction;

/** Data-only flywheel answers, one isolated test candidate, and explicit motion permission. */
public final class BasicFlywheelProfile {

    /** Complete one-motor velocity configuration in native encoder ticks per second. */
    public BasicFlywheelMechanism.Config flywheel;

    /** Positive velocity used by the isolated A-button fixture, in encoder ticks per second. */
    public double candidateVelocityTicksPerSec;

    /** Whether supervised flywheel motion is permitted after physical review. */
    public boolean allowFlywheelMotion;

    private BasicFlywheelProfile() {
        // Start from current() so every required flywheel answer is populated together.
    }

    /** Returns a fresh complete software baseline, not reviewed physical robot facts. */
    public static BasicFlywheelProfile current() {
        BasicFlywheelProfile profile = new BasicFlywheelProfile();
        profile.flywheel = BasicFlywheelMechanism.Config.defaults();
        profile.flywheel.motorName = "flywheelMotor";
        profile.flywheel.direction = Direction.FORWARD;
        profile.flywheel.maximumVelocityTicksPerSec = 500.0;
        profile.flywheel.velocityToleranceTicksPerSec = 25.0;
        profile.flywheel.spinUpTimeoutSec = 2.0;
        profile.candidateVelocityTicksPerSec = 250.0;
        profile.allowFlywheelMotion = false;
        return profile;
    }

    /** Rejects unchecked motion before a host performs any hardware lookup. */
    static void requireMotionAllowed(BasicFlywheelProfile profile, String mode) {
        BasicFlywheelProfile p = Objects.requireNonNull(profile, "flywheelProfile");
        if (!p.allowFlywheelMotion) {
            throw new IllegalStateException(
                    "BasicFlywheelProfile.allowFlywheelMotion must be true before " + mode
                            + " may construct a motion-capable flywheel. Review motor identity, "
                            + "direction, restraint, encoder units, candidate range, tuning, and "
                            + "physical STOP behavior first.");
        }
    }

    /** Rejects a fixture candidate that stopped feedback could satisfy or that exceeds the range. */
    static void requireCandidateInConfiguredRange(BasicFlywheelMechanism.Config config,
                                                  double candidateVelocityTicksPerSec) {
        BasicFlywheelMechanism.Config c = Objects.requireNonNull(config, "config");
        if (!Double.isFinite(candidateVelocityTicksPerSec)
                || !Double.isFinite(c.velocityToleranceTicksPerSec)
                || !Double.isFinite(c.maximumVelocityTicksPerSec)
                || candidateVelocityTicksPerSec <= c.velocityToleranceTicksPerSec
                || candidateVelocityTicksPerSec > c.maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "candidateVelocityTicksPerSec must be finite, greater than "
                            + "flywheel.velocityToleranceTicksPerSec, and no greater than "
                            + "flywheel.maximumVelocityTicksPerSec; got candidate "
                            + candidateVelocityTicksPerSec + ", tolerance "
                            + c.velocityToleranceTicksPerSec + ", and maximum "
                            + c.maximumVelocityTicksPerSec);
        }
    }
}
