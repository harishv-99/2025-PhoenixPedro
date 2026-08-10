package edu.ftcphoenix.fw.tools.tester;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.ftc.ui.MenuItem;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Guards the one ordinary actuator path and the intentionally advanced diagnostic submenu. */
public final class StandardTestersShapeTest {

    private static final List<String> HOME_LABELS = Arrays.asList(
            "HW: Actuator Bring-up",
            "Framework: Calibration & Localization",
            "Advanced: Hardware Diagnostics"
    );

    @Test
    public void standaloneAndEmbeddedHomesUseTheSameThreeEntriesInTheSameOrder()
            throws Exception {
        TesterSuite standalone = StandardTesters.createSuite();
        assertEquals(HOME_LABELS, labelsOf(standalone));

        TesterSuite embedded = new TesterSuite();
        StandardTesters.register(embedded);
        assertEquals(HOME_LABELS, labelsOf(embedded));
    }

    @Test
    public void ordinaryEntryIsBringUpAndAdvancedMenuRetainsOnlyDistinctDiagnostics()
            throws Exception {
        List<MenuItem<Supplier<TeleOpTester>>> homeItems = itemsOf(StandardTesters.createSuite());

        TeleOpTester ordinary = homeItems.get(0).value.get();
        assertTrue(ordinary instanceof ActuatorBringUpTester);

        TeleOpTester advanced = homeItems.get(2).value.get();
        assertTrue(advanced instanceof TesterSuite);
        assertEquals(Arrays.asList(
                        "Advanced: Motor Power & Encoder Evidence",
                        "Advanced: Motor Position",
                        "Advanced: Motor Velocity",
                        "Advanced: Normalized Color Sensor"
                ),
                labelsOf((TesterSuite) advanced));
    }

    @Test
    public void publicApiExposesBringUpFactoryButNotRemovedParallelFactories() {
        boolean hasBringUp = false;
        boolean hasWalkthrough = false;
        boolean hasHardwareSuite = false;

        for (Method method : StandardTesters.class.getDeclaredMethods()) {
            if (method.getName().equals("createActuatorBringUp")) {
                hasBringUp = true;
                assertTrue(Modifier.isPublic(method.getModifiers()));
                assertTrue(Modifier.isStatic(method.getModifiers()));
                assertEquals(TeleOpTester.class, method.getReturnType());
            } else if (method.getName().equals("createWalkthrough")) {
                hasWalkthrough = true;
            } else if (method.getName().equals("createHardwareSuite")) {
                hasHardwareSuite = true;
            }
        }

        assertTrue(hasBringUp);
        assertFalse(hasWalkthrough);
        assertFalse(hasHardwareSuite);
    }

    private static List<String> labelsOf(TesterSuite suite) throws Exception {
        List<String> labels = new ArrayList<String>();
        for (MenuItem<Supplier<TeleOpTester>> item : itemsOf(suite)) {
            labels.add(item.label);
        }
        return labels;
    }

    @SuppressWarnings("unchecked")
    private static List<MenuItem<Supplier<TeleOpTester>>> itemsOf(TesterSuite suite)
            throws Exception {
        Field field = TesterSuite.class.getDeclaredField("menu");
        field.setAccessible(true);
        SelectionMenu<Supplier<TeleOpTester>> menu =
                (SelectionMenu<Supplier<TeleOpTester>>) field.get(suite);
        return menu.itemsSnapshot();
    }
}
