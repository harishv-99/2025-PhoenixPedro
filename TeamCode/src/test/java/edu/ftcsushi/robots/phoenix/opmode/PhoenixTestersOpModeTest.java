package edu.ftcsushi.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcsushi.robots.phoenix.PhoenixRobot;

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
                        LoopTimestamp.unavailable()
                ),
                PhoenixAlliance.RED
        );

        PhoenixTestersOpMode mode = new PhoenixTestersOpMode();
        assertNotNull(mode.createTester());

        assertEquals(
                PhoenixMatchHandoff.RestoreResult.MISSING,
                PhoenixMatchHandoff.restoreForTeleOp(
                        new EmptyOpMode(),
                        uninitializedRobot(),
                        alliance -> { }
                )
        );
    }

    private static PhoenixRobot uninitializedRobot() {
        return new PhoenixRobot(new HardwareMap(null, null));
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
