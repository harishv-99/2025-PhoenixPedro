package edu.ftcsushi.robots.examples.pedro.opmode;

import edu.ftcsushi.robots.examples.pedro.robot.BasicPedroAutoRobot;
import edu.ftcsushi.robots.examples.pedro.robot.BasicPedroAutoRobotTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.route.RouteStatus;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixMatchHandoff;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that the disabled host declares the example through the managed FTC lifecycle. */
public final class BasicPedroAutoExampleTest {

    @Before
    public void clearHandoffBeforeTest() {
        PhoenixMatchHandoff.clear();
    }

    @After
    public void clearHandoffAfterTest() {
        PhoenixMatchHandoff.clear();
    }

    @Test
    public void disabledOpModeUsesManagedInitStartLoopAndStop() {
        List<String> events = new ArrayList<String>();
        BasicPedroAutoRobot[] retainedRobot = new BasicPedroAutoRobot[1];
        BasicPedroAutoExample mode = new BasicPedroAutoExample(program -> {
            retainedRobot[0] = BasicPedroAutoRobotTest.newRecordingRobot(program, events);
            return retainedRobot[0];
        });
        AtomicInteger updateAttempts = new AtomicInteger();
        List<String> telemetryKeys = new ArrayList<String>();
        mode.telemetry = recordingTelemetry(updateAttempts, telemetryKeys);

        mode.init();
        assertEquals(1, updateAttempts.get());
        assertTrue(events.isEmpty());
        mode.init_loop();
        assertEquals(2, updateAttempts.get());
        assertTrue(events.isEmpty());
        mode.start();
        assertEquals(2, updateAttempts.get());
        mode.loop();
        assertEquals(3, updateAttempts.get());
        mode.stop();
        mode.stop();

        assertNotNull(BasicPedroAutoExample.class.getAnnotation(Disabled.class));
        assertTrue(FtcRobotOpMode.class.isAssignableFrom(BasicPedroAutoExample.class));
        assertTrue(events.contains("startPose"));
        assertTrue(events.contains("task.cancel"));
        assertTrue(events.contains("plant.stop"));
        assertTrue(events.contains("drive.stop"));
        assertEquals(TaskOutcome.CANCELLED, retainedRobot[0].rootOutcome());
        assertEquals(RouteStatus.NOT_STARTED,
                BasicPedroAutoRobotTest.latestRouteStatusOf(retainedRobot[0]));
        assertTrue(telemetryKeys.contains("example.expectedPhysicalStartPedro"));
        assertTrue(telemetryKeys.contains("example.routeStatus"));
        assertTrue(telemetryKeys.contains("example.rootComplete"));
        assertTrue(telemetryKeys.contains("example.rootOutcome"));
    }

    @Test
    public void activeTelemetryFailureFailStopsTheDeclaredProgram() {
        List<String> events = new ArrayList<String>();
        BasicPedroAutoExample mode = new BasicPedroAutoExample(
                program -> BasicPedroAutoRobotTest.newRecordingRobot(program, events)
        );
        RuntimeException telemetryFailure = new RuntimeException("telemetry failed");
        AtomicInteger updateAttempts = new AtomicInteger();
        mode.telemetry = throwingOnUpdateAttemptTelemetry(
                telemetryFailure,
                updateAttempts,
                2
        );

        mode.init();
        mode.start();

        try {
            mode.loop();
            fail("expected telemetry failure");
        } catch (RuntimeException failure) {
            assertSame(telemetryFailure, failure);
        }

        assertEquals(2, updateAttempts.get());
        assertTrue(events.contains("task.cancel"));
        assertTrue(events.contains("plant.stop"));
        assertTrue(events.contains("drive.stop"));
        mode.stop();
    }

    @Test
    public void configureClearsPendingPhoenixMatchHandoffSoAnotherAutoCanPublish() {
        PoseEstimate snapshot = new PoseEstimate(
                new Pose3d(1.0, 2.0, 0.0, 0.3, 0.0, 0.0),
                true,
                1.0,
                LoopTimestamp.unavailable()
        );
        PhoenixMatchHandoff.publishFromAuto(
                new EmptyOpMode(),
                snapshot,
                PhoenixAlliance.RED
        );
        BasicPedroAutoExample mode = new BasicPedroAutoExample(
                program -> BasicPedroAutoRobotTest.newRecordingRobot(
                        program,
                        new ArrayList<String>()
                )
        );
        mode.telemetry = inertTelemetry();

        mode.init();

        PhoenixMatchHandoff.publishFromAuto(
                new EmptyOpMode(),
                snapshot,
                PhoenixAlliance.BLUE
        );
        mode.stop();
    }

    private static Telemetry throwingOnUpdateAttemptTelemetry(
            RuntimeException failure,
            AtomicInteger updateAttempts,
            int throwingAttempt
    ) {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if ("update".equals(method.getName())
                            && updateAttempts.incrementAndGet() == throwingAttempt) {
                        throw failure;
                    }
                    return defaultValue(method.getReturnType());
                }
        );
    }

    private static Telemetry recordingTelemetry(
            AtomicInteger updateAttempts,
            List<String> telemetryKeys
    ) {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if ("update".equals(method.getName())) {
                        updateAttempts.incrementAndGet();
                    } else if ("addData".equals(method.getName())
                            && args != null
                            && args.length > 0
                            && args[0] instanceof String) {
                        telemetryKeys.add((String) args[0]);
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
