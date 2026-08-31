package edu.ftcsushi.fw.core.control;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down the retained PID value API and advanced custom-regulator decorators. */
public final class ScalarRegulatorsApiTest {

    @Test
    public void pidUsesOnlyItsFactoryAndOldGetterSpellingsAreAbsent() throws Exception {
        Constructor<?>[] publicConstructors = Pid.class.getConstructors();
        assertEquals(0, publicConstructors.length);

        Constructor<?>[] declaredConstructors = Pid.class.getDeclaredConstructors();
        assertEquals(1, declaredConstructors.length);
        assertTrue(Modifier.isPrivate(declaredConstructors[0].getModifiers()));
        assertArrayEquals(new Class<?>[]{double.class, double.class, double.class},
                declaredConstructors[0].getParameterTypes());

        Method factory = Pid.class.getDeclaredMethod(
                "withGains", double.class, double.class, double.class);
        assertTrue(Modifier.isPublic(factory.getModifiers()));
        assertTrue(Modifier.isStatic(factory.getModifiers()));
        assertEquals(Pid.class, factory.getReturnType());

        assertPublicInstanceMethod(Pid.class, "setGains", Pid.class,
                double.class, double.class, double.class);
        assertPublicInstanceMethod(Pid.class, "setIntegralLimits", Pid.class,
                double.class, double.class);
        assertPublicInstanceMethod(Pid.class, "setOutputLimits", Pid.class,
                double.class, double.class);
        assertPublicInstanceMethod(Pid.class, "getKP", double.class);
        assertPublicInstanceMethod(Pid.class, "getKI", double.class);
        assertPublicInstanceMethod(Pid.class, "getKD", double.class);
        assertPublicInstanceMethod(Pid.class, "getIntegral", double.class);

        assertDeclaredMethodAbsent(Pid.class, "getkP");
        assertDeclaredMethodAbsent(Pid.class, "getkI");
        assertDeclaredMethodAbsent(Pid.class, "getkD");

        assertOnlyPublicDeclaredMethods(Pid.class,
                "static withGains(double,double,double):Pid",
                "instance setGains(double,double,double):Pid",
                "instance setIntegralLimits(double,double):Pid",
                "instance setOutputLimits(double,double):Pid",
                "instance update(double,double):double",
                "instance reset():void",
                "instance getKP():double",
                "instance getKI():double",
                "instance getKD():double",
                "instance getIntegral():double",
                "instance debugDump(DebugSink,String):void");
    }

    @Test
    public void scalarRegulatorsRetainsOnlyDistinctAdvancedCustomDecorators() {
        assertOnlyPublicDeclaredMethods(ScalarRegulators.class,
                "static voltageCompensated(ScalarRegulator,ScalarSource,double,double,double):ScalarRegulator",
                "static outputLimited(ScalarRegulator,double,double):ScalarRegulator");

        assertDeclaredMethodAbsent(ScalarRegulators.class, "pid", PidController.class);
        assertDeclaredMethodAbsent(ScalarRegulators.class, "pidf",
                double.class, double.class, double.class, double.class);
        assertDeclaredMethodAbsent(ScalarRegulators.class, "setpointFeedforward");

        for (Method method : ScalarControllers.class.getDeclaredMethods()) {
            assertFalse("ScalarControllers must not add a duplicate Plant regulator layer",
                    method.getName().equals("pidf") && Modifier.isPublic(method.getModifiers()));
        }
    }

    private static void assertPublicInstanceMethod(Class<?> owner,
                                                   String name,
                                                   Class<?> returnType,
                                                   Class<?>... parameterTypes)
            throws Exception {
        Method method = owner.getDeclaredMethod(name, parameterTypes);
        assertNotNull(method);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertFalse(Modifier.isStatic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }

    private static void assertDeclaredMethodAbsent(Class<?> owner,
                                                   String name,
                                                   Class<?>... parameterTypes) {
        try {
            owner.getDeclaredMethod(name, parameterTypes);
            fail(owner.getSimpleName() + "." + name + " must be absent");
        } catch (NoSuchMethodException expected) {
            // Expected: TUNE-03 removes the obsolete peer standard-control construction path.
        }
    }

    private static void assertOnlyPublicDeclaredMethods(Class<?> owner,
                                                        String... expectedSignatures) {
        Set<String> actual = new HashSet<>();
        for (Method method : owner.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                actual.add(publicSignature(method));
            }
        }
        assertEquals(new HashSet<>(Arrays.asList(expectedSignatures)), actual);
    }

    private static String publicSignature(Method method) {
        StringBuilder signature = new StringBuilder();
        signature.append(Modifier.isStatic(method.getModifiers()) ? "static " : "instance ")
                .append(method.getName())
                .append('(');
        Class<?>[] parameters = method.getParameterTypes();
        for (int i = 0; i < parameters.length; i++) {
            if (i > 0) {
                signature.append(',');
            }
            signature.append(parameters[i].getSimpleName());
        }
        return signature.append("):")
                .append(method.getReturnType().getSimpleName())
                .toString();
    }
}
