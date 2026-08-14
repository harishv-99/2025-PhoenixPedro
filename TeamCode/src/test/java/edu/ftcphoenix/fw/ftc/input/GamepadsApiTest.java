package edu.ftcphoenix.fw.ftc.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcphoenix.fw.core.source.BooleanSource;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down CLEAN-01's single gamepad factory and full button names. */
public final class GamepadsApiTest {

    @Test
    public void createIsTheOnlyRawFtcGamepadFactory() throws Exception {
        Method create = Gamepads.class.getDeclaredMethod(
                "create", Gamepad.class, Gamepad.class);
        assertTrue(Modifier.isPublic(create.getModifiers()));
        assertTrue(Modifier.isStatic(create.getModifiers()));
        assertEquals(Gamepads.class, create.getReturnType());

        Gamepads gamepads = Gamepads.create(new Gamepad(), new Gamepad());
        assertNotNull(gamepads.p1());
        assertNotNull(gamepads.p2());

        assertDeclaredMethodAbsent(Gamepads.class,
                "of", Gamepad.class, Gamepad.class);

        int rawFactoryCount = 0;
        for (Method method : Gamepads.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())
                    && Modifier.isStatic(method.getModifiers())
                    && method.getReturnType() == Gamepads.class) {
                rawFactoryCount++;
                assertEquals("create", method.getName());
                Class<?>[] parameters = method.getParameterTypes();
                assertEquals(2, parameters.length);
                assertEquals(Gamepad.class, parameters[0]);
                assertEquals(Gamepad.class, parameters[1]);
            }
        }
        assertEquals(1, rawFactoryCount);
    }

    @Test
    public void completedDeviceConstructorRemainsThePublicAdvancedSeam() throws Exception {
        Constructor<Gamepads> constructor = Gamepads.class.getConstructor(
                GamepadDevice.class, GamepadDevice.class);
        assertTrue(Modifier.isPublic(constructor.getModifiers()));
        assertEquals(1, Gamepads.class.getConstructors().length);

        GamepadDevice playerOne = new GamepadDevice(new Gamepad());
        GamepadDevice playerTwo = new GamepadDevice(new Gamepad());
        Gamepads gamepads = new Gamepads(playerOne, playerTwo);

        assertSame(playerOne, gamepads.p1());
        assertSame(playerTwo, gamepads.p2());
    }

    @Test
    public void bumperAndStickButtonSourcesUseOnlyFullNames() throws Exception {
        GamepadDevice device = new GamepadDevice(new Gamepad());
        assertNotNull(device.leftBumper());
        assertNotNull(device.rightBumper());
        assertNotNull(device.leftStickButton());
        assertNotNull(device.rightStickButton());

        assertPublicBooleanSourceMethod("leftBumper");
        assertPublicBooleanSourceMethod("rightBumper");
        assertPublicBooleanSourceMethod("leftStickButton");
        assertPublicBooleanSourceMethod("rightStickButton");

        assertDeclaredMethodAbsent(GamepadDevice.class, "lb");
        assertDeclaredMethodAbsent(GamepadDevice.class, "rb");
        assertDeclaredMethodAbsent(GamepadDevice.class, "ls");
        assertDeclaredMethodAbsent(GamepadDevice.class, "rs");
    }

    private static void assertPublicBooleanSourceMethod(String name) throws Exception {
        Method method = GamepadDevice.class.getDeclaredMethod(name);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertFalse(Modifier.isStatic(method.getModifiers()));
        assertEquals(BooleanSource.class, method.getReturnType());
    }

    private static void assertDeclaredMethodAbsent(Class<?> owner,
                                                   String name,
                                                   Class<?>... parameterTypes) {
        try {
            owner.getDeclaredMethod(name, parameterTypes);
            fail(owner.getSimpleName() + "." + name + " must be absent");
        } catch (NoSuchMethodException expected) {
            // Expected: CLEAN-01 removes the parallel alias.
        }
    }
}
