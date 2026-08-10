package edu.ftcphoenix.robots.phoenix.tester;

import org.junit.Test;

import java.lang.reflect.Field;
import java.util.List;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.ftc.ui.MenuItem;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Menu-order regression for the beginner-first Phoenix calibration walkthrough. */
public final class PhoenixCalibrationWalkthroughShapeTest {

    @Test
    public void initialSelectionRemainsOnPhysicalActuatorBringUp() throws Exception {
        TesterSuite suite = PhoenixCalibrationWalkthrough.createSuite();
        SelectionMenu<Supplier<TeleOpTester>> menu = menuOf(suite);
        List<MenuItem<Supplier<TeleOpTester>>> items = menu.itemsSnapshot();

        assertEquals(0, menu.selectedIndex());
        assertTrue(items.get(0).label.endsWith("HW: Actuator Bring-up"));
        assertTrue(items.get(1).label.endsWith("HW: Configured Drivetrain Verification"));
    }

    @SuppressWarnings("unchecked")
    private static SelectionMenu<Supplier<TeleOpTester>> menuOf(TesterSuite suite)
            throws Exception {
        Field field = TesterSuite.class.getDeclaredField("menu");
        field.setAccessible(true);
        return (SelectionMenu<Supplier<TeleOpTester>>) field.get(suite);
    }
}
