package edu.ftcsushi.robots.examples.basicflywheel;

import org.junit.Test;

import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Maintainer boundary coverage for Basic Flywheel configuration and fixture candidates. */
public final class BasicFlywheelConfigurationContractTest {

    @Test
    public void fixtureCandidateMustBeObservableAndInsideConfiguredRange() {
        BasicFlywheelMechanism.Config config = BasicFlywheelMechanism.Config.defaults();

        IllegalArgumentException insideZeroBand = assertThrows(
                IllegalArgumentException.class,
                () -> BasicFlywheelTeleOp.requireCandidateInConfiguredRange(
                        config, config.velocityToleranceTicksPerSec));
        assertTrue(insideZeroBand.getMessage().contains("greater than"));

        IllegalArgumentException aboveMaximum = assertThrows(
                IllegalArgumentException.class,
                () -> BasicFlywheelTeleOp.requireCandidateInConfiguredRange(
                        config, config.maximumVelocityTicksPerSec + 1.0));
        assertTrue(aboveMaximum.getMessage().contains("maximumVelocityTicksPerSec"));
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
