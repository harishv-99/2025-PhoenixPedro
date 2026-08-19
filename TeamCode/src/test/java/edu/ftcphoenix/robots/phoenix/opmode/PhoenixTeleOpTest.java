package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix TeleOp's managed-only host and exact-START drive boundary. */
public final class PhoenixTeleOpTest {

    @Test
    public void productionTeleOpUsesOnlyTheManagedConfigureCallback() throws Exception {
        assertEquals(FtcRobotOpMode.class, PhoenixTeleOp.class.getSuperclass());

        Method configure = PhoenixTeleOp.class.getDeclaredMethod(
                "configure",
                RobotProgram.class
        );
        assertTrue(Modifier.isProtected(configure.getModifiers()));
        assertFalse(Modifier.isStatic(configure.getModifiers()));

        assertNoDeclaredTeleOpMethod("init");
        assertNoDeclaredTeleOpMethod("init_loop");
        assertNoDeclaredTeleOpMethod("start");
        assertNoDeclaredTeleOpMethod("loop");
        assertNoDeclaredTeleOpMethod("stop");
    }

    @Test
    public void robotSurfaceUsesOneManagedDeclarationGrammarForTeleOpAndAuto() throws Exception {
        Constructor<?>[] publicConstructors = PhoenixRobot.class.getConstructors();
        assertEquals(1, publicConstructors.length);
        assertTrue(Arrays.equals(
                new Class<?>[]{HardwareMap.class},
                publicConstructors[0].getParameterTypes()
        ));

        Method declareTeleOp = PhoenixRobot.class.getMethod(
                "declareTeleOp",
                RobotProgram.class,
                PhoenixProfile.class,
                Gamepad.class,
                Gamepad.class,
                Source.class
        );
        assertEquals(Void.TYPE, declareTeleOp.getReturnType());
        assertNotNull(PhoenixRobot.class.getMethod(
                "teleOpPresenter",
                PhoenixMatchHandoff.RestoreResult.class
        ));
        Method declareAuto = PhoenixRobot.class.getMethod(
                "declareAuto",
                RobotProgram.class,
                PhoenixProfile.class,
                DriveCommandSink.class,
                MotionPredictor.class,
                Source.class,
                BooleanSource.class,
                BooleanSource.class,
                Runnable.class
        );
        assertEquals(PhoenixCapabilities.class, declareAuto.getReturnType());
        assertNotNull(PhoenixRobot.class.getMethod("autoPresenter", Task.class));

        int declaredPublicMethods = 0;
        for (Method method : PhoenixRobot.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                declaredPublicMethods++;
            }
        }
        assertEquals(4, declaredPublicMethods);
        assertNoPublicRobotMethod("capabilities");

        assertNoPublicRobotMethod("initAny");
        assertNoPublicRobotMethod("initTeleOp");
        assertNoPublicRobotMethod("startAny", double.class);
        assertNoPublicRobotMethod("startTeleOp");
        assertNoPublicRobotMethod("updateAny", double.class);
        assertNoPublicRobotMethod("updateTeleOp");
        assertNoPublicRobotMethod("initAuto", DriveCommandSink.class, MotionPredictor.class);
        assertNoPublicRobotMethod("installAutoRoutine", Task.class);
        assertNoPublicRobotMethod("startAuto", double.class);
        assertNoPublicRobotMethod("updateAuto", double.class);
        assertNoPublicRobotMethod("stopAuto");
        assertNoPublicRobotMethod("stop");
    }

    @Test
    public void managedDriveSourceWritesZeroAtStartBeforeSamplingDriverIntent()
            throws Exception {
        PhoenixRobot robot = new PhoenixRobot(new HardwareMap(null, null));
        int[] activeSamples = {0};
        DriveSource activeSource = clock -> {
            activeSamples[0]++;
            return new DriveSignal(0.7, -0.4, 0.25);
        };
        DriveSource managedSource = constructManagedDriveSource(robot, activeSource);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        DriveSignal exactStart = managedSource.get(clock);
        assertEquals(0.0, exactStart.axial, 0.0);
        assertEquals(0.0, exactStart.lateral, 0.0);
        assertEquals(0.0, exactStart.omega, 0.0);
        assertEquals(0, activeSamples[0]);

        setField(robot, "teleOpOrdinaryLoopReached", true);
        clock.update(0.02);
        DriveSignal active = managedSource.get(clock);
        assertEquals(0.7, active.axial, 0.0);
        assertEquals(-0.4, active.lateral, 0.0);
        assertEquals(0.25, active.omega, 0.0);
        assertEquals(1, activeSamples[0]);
    }

    private static DriveSource constructManagedDriveSource(
            PhoenixRobot robot,
            DriveSource activeSource
    ) throws Exception {
        for (Class<?> nested : PhoenixRobot.class.getDeclaredClasses()) {
            if (!"ManagedTeleOpDriveSource".equals(nested.getSimpleName())) {
                continue;
            }
            Constructor<?> constructor = nested.getDeclaredConstructor(
                    PhoenixRobot.class,
                    DriveSource.class
            );
            constructor.setAccessible(true);
            return (DriveSource) constructor.newInstance(robot, activeSource);
        }
        throw new AssertionError("PhoenixRobot.ManagedTeleOpDriveSource was not found");
    }

    private static void setField(Object target, String name, Object value) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static void assertNoDeclaredTeleOpMethod(String name, Class<?>... parameterTypes) {
        try {
            PhoenixTeleOp.class.getDeclaredMethod(name, parameterTypes);
            fail("Expected managed PhoenixTeleOp not to declare " + name);
        } catch (NoSuchMethodException expected) {
            // The final callbacks are inherited from FtcRobotOpMode.
        }
    }

    private static void assertNoPublicRobotMethod(String name, Class<?>... parameterTypes) {
        try {
            PhoenixRobot.class.getMethod(name, parameterTypes);
            fail("Expected legacy PhoenixRobot." + name + " to be removed");
        } catch (NoSuchMethodException expected) {
            // Managed RobotProgram declaration owns both mode lifecycles.
        }
    }

}
