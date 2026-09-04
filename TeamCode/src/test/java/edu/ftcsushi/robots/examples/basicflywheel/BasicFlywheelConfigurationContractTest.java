package edu.ftcsushi.robots.examples.basicflywheel;

import org.junit.Test;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Maintainer boundary coverage for Basic Flywheel configuration and fixture candidates. */
public final class BasicFlywheelConfigurationContractTest {

    @Test
    public void fixtureCandidateMustBeObservableAndInsideConfiguredRange() {
        BasicFlywheelMechanism.Config config = BasicFlywheelMechanism.Config.defaults();

        IllegalArgumentException insideZeroBand = assertThrows(
                IllegalArgumentException.class,
                () -> BasicFlywheelProfile.requireCandidateInConfiguredRange(
                        config, config.velocityToleranceTicksPerSec));
        assertTrue(insideZeroBand.getMessage().contains("greater than"));

        IllegalArgumentException aboveMaximum = assertThrows(
                IllegalArgumentException.class,
                () -> BasicFlywheelProfile.requireCandidateInConfiguredRange(
                        config, config.maximumVelocityTicksPerSec + 1.0));
        assertTrue(aboveMaximum.getMessage().contains("maximumVelocityTicksPerSec"));
    }

    @Test
    public void activeProfileKeepsConfigurationCandidateAndPermissionTogether() {
        BasicFlywheelProfile profile = BasicFlywheelProfile.current();

        assertEquals("flywheelMotor", profile.flywheel.motorName);
        assertEquals(Direction.FORWARD, profile.flywheel.direction);
        assertEquals(500.0, profile.flywheel.maximumVelocityTicksPerSec, 0.0);
        assertEquals(25.0, profile.flywheel.velocityToleranceTicksPerSec, 0.0);
        assertEquals(2.0, profile.flywheel.spinUpTimeoutSec, 0.0);
        assertEquals(250.0, profile.candidateVelocityTicksPerSec, 0.0);
        assertFalse(profile.allowFlywheelMotion);

        IllegalStateException locked = assertThrows(
                IllegalStateException.class,
                () -> BasicFlywheelProfile.requireMotionAllowed(profile, "test"));
        assertTrue(locked.getMessage().contains("allowFlywheelMotion"));

        profile.allowFlywheelMotion = true;
        BasicFlywheelProfile.requireMotionAllowed(profile, "test");
        BasicFlywheelProfile.requireCandidateInConfiguredRange(
                profile.flywheel, profile.candidateVelocityTicksPerSec);
    }

    @Test
    public void toleranceMustBeStrictlyLessThanMaximumBeforeHardwareLookup() {
        BasicFlywheelMechanism.Config config = BasicFlywheelMechanism.Config.defaults();
        config.velocityToleranceTicksPerSec = config.maximumVelocityTicksPerSec;
        FtcTestHardware hardware = new FtcTestHardware();

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> new BasicFlywheelMechanism(hardware, config));

        assertTrue(failure.getMessage().contains("velocityToleranceTicksPerSec"));
        assertTrue(failure.getMessage().contains("strictly less than"));
        assertTrue(failure.getMessage().contains("maximumVelocityTicksPerSec"));
        assertEquals("invalid config must fail before FTC lookup", 0, hardware.lookupCalls());
    }
}
