package edu.ftcphoenix.fw.tools.tester.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.ftc.FtcTeleOpTesterOpMode;
import edu.ftcphoenix.fw.ftc.ui.MenuItem;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;
import edu.ftcphoenix.fw.tools.tester.StandardTesters;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Guards the two explicit, parallel ready-to-run framework tester console entries. */
public final class FrameworkTesterOpModesShapeTest {

    private static final List<Class<? extends FtcTeleOpTesterOpMode>> READY_ENTRIES =
            Arrays.<Class<? extends FtcTeleOpTesterOpMode>>asList(
                    FrameworkDriverStationTestersOpMode.class,
                    FrameworkPanelsTestersOpMode.class
            );

    @Test
    public void readyEntriesExposeExactlyTheTwoExplicitConsoleNames() throws Exception {
        List<String> displayNames = new ArrayList<String>();

        for (Class<? extends FtcTeleOpTesterOpMode> entry : READY_ENTRIES) {
            int modifiers = entry.getModifiers();
            assertTrue(Modifier.isPublic(modifiers));
            assertTrue(Modifier.isFinal(modifiers));
            assertFalse(Modifier.isAbstract(modifiers));
            assertTrue(Modifier.isPublic(entry.getDeclaredConstructor().getModifiers()));
            assertNull(entry.getAnnotation(Disabled.class));

            TeleOp registration = entry.getAnnotation(TeleOp.class);
            assertNotNull(registration);
            assertEquals("Framework Testers", registration.group());
            displayNames.add(registration.name());
        }

        assertEquals(Arrays.asList(
                "FW: Testers (Driver Station)",
                "FW: Testers (Panels)"
        ), displayNames);
        assertEquals(READY_ENTRIES.get(0).getSuperclass(), READY_ENTRIES.get(1).getSuperclass());

        // The old ambiguous registration may be removed or retained as an internal base, but it
        // must not remain a third ready-to-run OpMode.
        assertLegacyEntryIsNotRegistered();
    }

    @Test
    public void bothReadyEntriesBuildFreshCanonicalStandardTesterSuites() throws Exception {
        TeleOpTester driverStation = createTester(READY_ENTRIES.get(0));
        TeleOpTester panels = createTester(READY_ENTRIES.get(1));

        assertEquals(TesterSuite.class, driverStation.getClass());
        assertEquals(TesterSuite.class, panels.getClass());
        assertNotSame(driverStation, panels);

        assertCanonicalSuite((TesterSuite) driverStation);
        assertCanonicalSuite((TesterSuite) panels);
    }

    private static TeleOpTester createTester(Class<? extends FtcTeleOpTesterOpMode> entry)
            throws Exception {
        Object instance = entry.getDeclaredConstructor().newInstance();
        Method factory = findCreateTester(entry);
        factory.setAccessible(true);
        return (TeleOpTester) factory.invoke(instance);
    }

    private static Method findCreateTester(Class<?> type) throws NoSuchMethodException {
        Class<?> current = type;
        while (current != null) {
            try {
                return current.getDeclaredMethod("createTester");
            } catch (NoSuchMethodException ignored) {
                current = current.getSuperclass();
            }
        }
        throw new NoSuchMethodException(type.getName() + ".createTester()");
    }

    private static void assertCanonicalSuite(TesterSuite actual) throws Exception {
        TesterSuite expected = StandardTesters.createSuite();
        assertNotSame(expected, actual);
        assertEquals(expected.name(), actual.name());

        SelectionMenu<Supplier<TeleOpTester>> expectedMenu = menuOf(expected);
        SelectionMenu<Supplier<TeleOpTester>> actualMenu = menuOf(actual);
        assertEquals(expectedMenu.title(), actualMenu.title());

        List<MenuItem<Supplier<TeleOpTester>>> expectedItems = expectedMenu.itemsSnapshot();
        List<MenuItem<Supplier<TeleOpTester>>> actualItems = actualMenu.itemsSnapshot();
        assertEquals(expectedItems.size(), actualItems.size());

        for (int i = 0; i < expectedItems.size(); i++) {
            MenuItem<Supplier<TeleOpTester>> expectedItem = expectedItems.get(i);
            MenuItem<Supplier<TeleOpTester>> actualItem = actualItems.get(i);
            assertEquals(expectedItem.id, actualItem.id);
            assertEquals(expectedItem.label, actualItem.label);
            assertEquals(expectedItem.help, actualItem.help);
            assertEquals(expectedItem.tag, actualItem.tag);
            assertEquals(expectedItem.enabled, actualItem.enabled);
            assertEquals(expectedItem.disabledReason, actualItem.disabledReason);

            TeleOpTester expectedChild = expectedItem.value.get();
            TeleOpTester actualChild = actualItem.value.get();
            assertNotNull(expectedChild);
            assertNotNull(actualChild);
            assertSame(expectedChild.getClass(), actualChild.getClass());
        }
    }

    @SuppressWarnings("unchecked")
    private static SelectionMenu<Supplier<TeleOpTester>> menuOf(TesterSuite suite)
            throws Exception {
        Field field = TesterSuite.class.getDeclaredField("menu");
        field.setAccessible(true);
        return (SelectionMenu<Supplier<TeleOpTester>>) field.get(suite);
    }

    private static void assertLegacyEntryIsNotRegistered() throws Exception {
        try {
            Class<?> legacy = Class.forName(
                    "edu.ftcphoenix.fw.tools.tester.opmode.FrameworkTestersOpMode"
            );
            assertNull(legacy.getAnnotation(TeleOp.class));
        } catch (ClassNotFoundException expected) {
            // Removing the obsolete class is also a valid cleanup.
        }
    }
}
