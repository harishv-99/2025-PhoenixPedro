package edu.ftcsushi.fw.input.binding;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the registration-only callback vocabulary and removal of its old alias. */
public final class CallbackBindingsApiTest {

    @Test
    public void rootAndContextExposeTheSameNarrowCallbackSurface() {
        assertTrue(CallbackBindings.class.isAssignableFrom(Bindings.class));
        assertTrue(CallbackBindings.class.isAssignableFrom(Bindings.ControlContext.class));
    }

    @Test
    public void callbackSurfaceHasOnlyTheApprovedRegistrationMethods() throws Exception {
        Set<String> methodNames = new HashSet<String>();
        Method[] methods = CallbackBindings.class.getDeclaredMethods();
        assertEquals(9, methods.length);
        for (Method method : methods) {
            assertTrue(Modifier.isPublic(method.getModifiers()));
            assertTrue(Modifier.isAbstract(method.getModifiers()));
            assertEquals(Void.TYPE, method.getReturnType());
            methodNames.add(method.getName());
        }
        assertEquals(
                new HashSet<String>(Arrays.asList(
                        "onRise",
                        "onFall",
                        "mirrorOnChange",
                        "whileHigh",
                        "whileLow",
                        "toggleOnRise",
                        "nudgeOnRise",
                        "copyEachCycle")),
                methodNames);

        assertMethod("onRise", BooleanSource.class, Runnable.class);
        assertMethod("onFall", BooleanSource.class, Runnable.class);
        assertMethod("mirrorOnChange", BooleanSource.class, Consumer.class);
        assertMethod("whileHigh", BooleanSource.class, Runnable.class);
        assertMethod("whileLow", BooleanSource.class, Runnable.class);
        assertMethod("toggleOnRise", BooleanSource.class, Runnable.class, Runnable.class);
        assertMethod("toggleOnRise", BooleanSource.class, Consumer.class);
        assertMethod(
                "nudgeOnRise",
                BooleanSource.class,
                BooleanSource.class,
                double.class,
                DoubleConsumer.class);
        assertMethod("copyEachCycle", ScalarSource.class, DoubleConsumer.class);
    }

    @Test
    public void oldBindingRegistrarTypeIsAbsent() {
        try {
            Class.forName(
                    "edu.ftcsushi.fw.input.binding.BindingRegistrar",
                    false,
                    CallbackBindingsApiTest.class.getClassLoader());
            fail("BindingRegistrar must not remain as a compatibility alias");
        } catch (ClassNotFoundException expected) {
            // Expected: CallbackBindings is the only registration-only public noun.
        }
    }

    private static void assertMethod(String name, Class<?>... parameterTypes) throws Exception {
        Method method = CallbackBindings.class.getDeclaredMethod(name, parameterTypes);
        assertEquals(Void.TYPE, method.getReturnType());
    }
}
