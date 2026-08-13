package edu.ftcphoenix.fw.core.hal;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the narrow capability split between generic and power-limited position outputs. */
public final class PowerLimitedPositionOutputApiTest {

    @Test
    public void inheritedOneArgumentCommandDelegatesWithFullMagnitude() {
        RecordingOutput output = new RecordingOutput();

        output.setPosition(42.5);

        assertEquals(42.5, output.position, 0.0);
        assertEquals(1.0, output.maximumMagnitude, 0.0);
        assertEquals(1, output.commands);
    }

    @Test
    public void richerCapabilityAddsPairedCommandCacheAndRequiredStopOnly() throws Exception {
        assertTrue(PositionOutput.class.isAssignableFrom(PowerLimitedPositionOutput.class));
        assertEquals(4, PowerLimitedPositionOutput.class.getDeclaredMethods().length);
        assertNoDeclaredMethod(PositionOutput.class, "setPosition", double.class, double.class);
        assertNoDeclaredMethod(PositionOutput.class,
                "getCommandedMaximumOutputPowerMagnitude");

        Method inheritedCommand = PowerLimitedPositionOutput.class.getDeclaredMethod(
                "setPosition", double.class);
        Method pairedCommand = PowerLimitedPositionOutput.class.getDeclaredMethod(
                "setPosition", double.class, double.class);
        Method magnitudeCache = PowerLimitedPositionOutput.class.getDeclaredMethod(
                "getCommandedMaximumOutputPowerMagnitude");
        Method stop = PowerLimitedPositionOutput.class.getDeclaredMethod("stop");

        assertTrue(inheritedCommand.isDefault());
        assertEquals(void.class, inheritedCommand.getReturnType());
        assertTrue(Modifier.isAbstract(pairedCommand.getModifiers()));
        assertEquals(void.class, pairedCommand.getReturnType());
        assertTrue(Modifier.isAbstract(magnitudeCache.getModifiers()));
        assertEquals(double.class, magnitudeCache.getReturnType());
        assertTrue(Modifier.isAbstract(stop.getModifiers()));
        assertEquals(void.class, stop.getReturnType());
        assertFalse(stop.isDefault());
    }

    private static void assertNoDeclaredMethod(Class<?> owner,
                                               String name,
                                               Class<?>... parameterTypes) {
        try {
            owner.getDeclaredMethod(name, parameterTypes);
            fail(owner.getSimpleName() + " must not declare " + name);
        } catch (NoSuchMethodException expected) {
            // Expected: the minimal PositionOutput seam remains unchanged.
        }
    }

    private static final class RecordingOutput implements PowerLimitedPositionOutput {
        private double position = Double.NaN;
        private double maximumMagnitude = Double.NaN;
        private int commands;

        @Override
        public void setPosition(double position, double maximumOutputPowerMagnitude) {
            this.position = position;
            maximumMagnitude = maximumOutputPowerMagnitude;
            commands++;
        }

        @Override
        public double getCommandedPosition() {
            return position;
        }

        @Override
        public double getCommandedMaximumOutputPowerMagnitude() {
            return maximumMagnitude;
        }

        @Override
        public void stop() {
            // Explicit natural stop required by the richer capability.
        }
    }
}
