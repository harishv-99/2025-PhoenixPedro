package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoRobot;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoRobotTest;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that the disabled host forwards the complete FTC lifecycle to one example root. */
public final class PhoenixBasicPedroAutoExampleTest {

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

    private static Telemetry throwingTelemetry(RuntimeException failure) {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    throw failure;
                }
        );
    }
}
