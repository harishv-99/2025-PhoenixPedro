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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
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

        mode.init();
        mode.init_loop();
        mode.start();
        mode.loop();
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

        mode.init();
        mode.start();
        mode.telemetry = throwingTelemetry(telemetryFailure);

        try {
            mode.loop();
            fail("expected telemetry failure");
        } catch (RuntimeException failure) {
            assertSame(telemetryFailure, failure);
        }

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
                        0.0,
                        0.0
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

    private static Telemetry throwingTelemetry(RuntimeException failure) {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    throw failure;
                }
        );
    }

    private static Telemetry inertTelemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    Class<?> returnType = method.getReturnType();
                    if (returnType == boolean.class) {
                        return true;
                    }
                    if (returnType == int.class) {
                        return 0;
                    }
                    return null;
                }
        );
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
