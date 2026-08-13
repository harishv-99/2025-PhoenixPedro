package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.junit.Test;

import edu.ftcphoenix.fw.integrations.panels.FtcPanelsTeleOpTesterOpMode;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies Phoenix's thin, directly registered Panels tuning entry. */
public final class PhoenixPanelsTuningOpModeTest {

    @Test
    public void opModeIsOnePanelsHostInTheParallelOpModePackage() {
        TeleOp annotation = PhoenixPanelsTuningOpMode.class.getAnnotation(TeleOp.class);

        assertEquals("Phoenix: Tuning (Panels)", annotation.name());
        assertEquals("Phoenix", annotation.group());
        assertTrue(FtcPanelsTeleOpTesterOpMode.class
                .isAssignableFrom(PhoenixPanelsTuningOpMode.class));
    }

    @Test
    public void factoryReturnsTheVelocityTunerDirectlyWithoutAOneItemSuite() {
        TeleOpTester tester = new PhoenixPanelsTuningOpMode().createTester();

        assertEquals("Phoenix Flywheel Velocity PIDF", tester.name());
        assertFalse(tester instanceof TesterSuite);
    }
}
