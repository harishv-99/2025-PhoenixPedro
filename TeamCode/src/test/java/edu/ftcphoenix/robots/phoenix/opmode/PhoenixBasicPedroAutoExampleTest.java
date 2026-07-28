package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoRobot;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoRobotTest;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that the disabled host forwards the complete FTC lifecycle to one example root. */
public final class PhoenixBasicPedroAutoExampleTest {

    @Before
    public void clearHandoffBeforeTest() {
        PhoenixMatchHandoff.clear();
    }

    @After
    public void clearHandoffAfterTest() {
        PhoenixMatchHandoff.clear();
    }

    @Test
    public void disabledOpModeForwardsInitStartLoopAndStop() {
        List<String> events = new ArrayList<String>();
        BasicPedroAutoRobot robot = BasicPedroAutoRobotTest.newRecordingRobot(events);
        PhoenixBasicPedroAutoExample mode = new PhoenixBasicPedroAutoExample(() -> robot);
        AtomicInteger updateAttempts = new AtomicInteger();
        mode.telemetry = recordingTelemetry(updateAttempts);

        mode.init();
        assertEquals(1, updateAttempts.get());
        mode.init_loop();
        assertEquals(2, updateAttempts.get());
        mode.start();
        assertEquals(2, updateAttempts.get());
        mode.loop();
        assertEquals(3, updateAttempts.get());
        mode.stop();
        mode.stop();

        assertNotNull(PhoenixBasicPedroAutoExample.class.getAnnotation(Disabled.class));
        assertTrue(events.contains("startPose"));
        assertTrue(events.contains("task.cancel"));
        assertTrue(events.contains("plant.stop"));
        assertTrue(events.contains("drive.stop"));
        assertTrue(robot.isStopped());
    }

    @Test
    public void loopTelemetryFailureFailStopsTheOwnedRobot() {
        List<String> events = new ArrayList<String>();
        BasicPedroAutoRobot robot = BasicPedroAutoRobotTest.newRecordingRobot(events);
        PhoenixBasicPedroAutoExample mode = new PhoenixBasicPedroAutoExample(() -> robot);
        RuntimeException telemetryFailure = new RuntimeException("telemetry failed");
        AtomicInteger updateAttempts = new AtomicInteger();

        mode.init();
        mode.start();
        mode.telemetry = throwingOnUpdateTelemetry(telemetryFailure, updateAttempts);

        try {
            mode.loop();
            fail("expected telemetry failure");
        } catch (RuntimeException failure) {
            assertSame(telemetryFailure, failure);
        }

        assertEquals(1, updateAttempts.get());
        assertTrue(events.contains("task.cancel"));
        assertTrue(events.contains("plant.stop"));
        assertTrue(events.contains("drive.stop"));
        assertTrue(robot.isStopped());
    }

    @Test
    public void initInvalidatesPendingPhoenixMatchHandoff() {
        PhoenixMatchHandoff.publishFromAuto(
                new EmptyOpMode(),
                new PoseEstimate(
                        new Pose3d(1.0, 2.0, 0.0, 0.3, 0.0, 0.0),
                        true,
                        1.0,
                        LoopTimestamp.unavailable()
                )
        );
        BasicPedroAutoRobot robot =
                BasicPedroAutoRobotTest.newRecordingRobot(new ArrayList<String>());
        PhoenixBasicPedroAutoExample mode = new PhoenixBasicPedroAutoExample(() -> robot);

        mode.init();

        assertEquals(
                PhoenixMatchHandoff.RestoreResult.MISSING,
                PhoenixMatchHandoff.restoreForTeleOp(
                        new EmptyOpMode(),
                        uninitializedPhoenixRobot()
                )
        );
        mode.stop();
    }

    private static PhoenixRobot uninitializedPhoenixRobot() {
        return new PhoenixRobot(
                new HardwareMap(null, null),
                inertTelemetry(),
                new Gamepad(),
                new Gamepad(),
                PhoenixProfile.current()
        );
    }

    private static Telemetry throwingOnUpdateTelemetry(
            RuntimeException failure,
            AtomicInteger updateAttempts
    ) {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if ("update".equals(method.getName())) {
                        updateAttempts.incrementAndGet();
                        throw failure;
                    }
                    return defaultValue(method.getReturnType());
                }
        );
    }

    private static Telemetry recordingTelemetry(AtomicInteger updateAttempts) {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if ("update".equals(method.getName())) {
                        updateAttempts.incrementAndGet();
                    }
                    return defaultValue(method.getReturnType());
                }
        );
    }

    private static Telemetry inertTelemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> defaultValue(method.getReturnType())
        );
    }

    private static Object defaultValue(Class<?> returnType) {
        if (returnType == boolean.class) {
            return true;
        }
        if (returnType == byte.class) {
            return (byte) 0;
        }
        if (returnType == short.class) {
            return (short) 0;
        }
        if (returnType == int.class) {
            return 0;
        }
        if (returnType == long.class) {
            return 0L;
        }
        if (returnType == float.class) {
            return 0.0f;
        }
        if (returnType == double.class) {
            return 0.0;
        }
        if (returnType == char.class) {
            return '\0';
        }
        return null;
    }

    private static final class EmptyOpMode extends OpMode {
        @Override
        public void init() {
        }

        @Override
        public void loop() {
        }
    }
}
