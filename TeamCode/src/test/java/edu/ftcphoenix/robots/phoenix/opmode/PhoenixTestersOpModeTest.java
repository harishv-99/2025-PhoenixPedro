package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/** Verifies that entering Phoenix diagnostics invalidates match-only process state. */
public final class PhoenixTestersOpModeTest {

    @Before
    public void clearHandoffBeforeTest() {
        PhoenixMatchHandoff.clear();
    }

    @After
    public void clearHandoffAfterTest() {
        PhoenixMatchHandoff.clear();
    }

    @Test
    public void testerFactoryRunsDuringInitAndClearsPendingHandoff() {
        PhoenixMatchHandoff.publishFromAuto(
                new EmptyOpMode(),
                new PoseEstimate(
                        new Pose3d(9.0, -3.0, 0.0, 0.4, 0.0, 0.0),
                        true,
                        1.0,
                        0.0,
                        0.0
                )
        );

        PhoenixTestersOpMode mode = new PhoenixTestersOpMode();
        assertNotNull(mode.createTester());

        assertEquals(
                PhoenixMatchHandoff.RestoreResult.MISSING,
                PhoenixMatchHandoff.restoreForTeleOp(
                        new EmptyOpMode(),
                        uninitializedRobot()
                )
        );
    }

    private static PhoenixRobot uninitializedRobot() {
        return new PhoenixRobot(
                new HardwareMap(null, null),
                inertTelemetry(),
                new Gamepad(),
                new Gamepad(),
                PhoenixProfile.current()
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
