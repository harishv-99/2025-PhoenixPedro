package edu.ftcphoenix.fw.core.control;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down API-03's one-layer PID and standard-PIDF construction surface. */
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
    public void pidfRegulatorIsPublicFinalRetainedCapabilityWithoutPublicFactory()
            throws Exception {
        int modifiers = PidfRegulator.class.getModifiers();
        assertTrue(Modifier.isPublic(modifiers));
        assertTrue(Modifier.isFinal(modifiers));
        assertTrue(ScalarRegulator.class.isAssignableFrom(PidfRegulator.class));

        assertEquals(0, PidfRegulator.class.getConstructors().length);
        Constructor<?>[] declaredConstructors =
                PidfRegulator.class.getDeclaredConstructors();
        assertEquals(1, declaredConstructors.length);
        int constructorModifiers = declaredConstructors[0].getModifiers();
        assertFalse(Modifier.isPublic(constructorModifiers));
        assertFalse(Modifier.isProtected(constructorModifiers));
        assertFalse(Modifier.isPrivate(constructorModifiers));
        assertArrayEquals(
                new Class<?>[]{double.class, double.class, double.class, double.class},
                declaredConstructors[0].getParameterTypes());

        for (Method method : PidfRegulator.class.getDeclaredMethods()) {
            boolean publicStaticFactory = Modifier.isPublic(method.getModifiers())
                    && Modifier.isStatic(method.getModifiers())
                    && method.getReturnType() == PidfRegulator.class;
            assertFalse("PidfRegulator must not expose class-local factory "
                    + method, publicStaticFactory);
        }

        assertPublicInstanceMethod(PidfRegulator.class, "setGains",
                PidfRegulator.class,
                double.class, double.class, double.class, double.class);
        assertPublicInstanceMethod(PidfRegulator.class, "setIntegralLimits",
                PidfRegulator.class, double.class, double.class);
        assertPublicInstanceMethod(PidfRegulator.class, "setPidOutputLimits",
                PidfRegulator.class, double.class, double.class);
        assertPublicInstanceMethod(PidfRegulator.class, "getKP", double.class);
        assertPublicInstanceMethod(PidfRegulator.class, "getKI", double.class);
        assertPublicInstanceMethod(PidfRegulator.class, "getKD", double.class);
        assertPublicInstanceMethod(PidfRegulator.class, "getKF", double.class);

        assertDeclaredMethodAbsent(PidfRegulator.class, "withGains",
                double.class, double.class, double.class, double.class);
        assertDeclaredMethodAbsent(PidfRegulator.class, "of",
                double.class, double.class, double.class, double.class);

        assertOnlyPublicDeclaredMethods(PidfRegulator.class,
                "instance setGains(double,double,double,double):PidfRegulator",
                "instance setIntegralLimits(double,double):PidfRegulator",
                "instance setPidOutputLimits(double,double):PidfRegulator",
                "instance getKP():double",
                "instance getKI():double",
                "instance getKD():double",
                "instance getKF():double",
                "instance update(double,double,LoopClock):double",
                "instance reset():void",
                "instance debugDump(DebugSink,String):void");
    }

    @Test
    public void scalarRegulatorsHasExactlyOnePublicPidfFactoryWithApprovedSignature()
            throws Exception {
        List<Method> publicPidfMethods = new ArrayList<>();
        for (Method method : ScalarRegulators.class.getDeclaredMethods()) {
            if (method.getName().equals("pidf")
                    && Modifier.isPublic(method.getModifiers())) {
                publicPidfMethods.add(method);
            }
        }

        assertEquals(1, publicPidfMethods.size());
        Method pidf = publicPidfMethods.get(0);
        assertTrue(Modifier.isStatic(pidf.getModifiers()));
        assertEquals(PidfRegulator.class, pidf.getReturnType());
        assertArrayEquals(
                new Class<?>[]{double.class, double.class, double.class, double.class},
                pidf.getParameterTypes());

        assertDeclaredMethodAbsent(ScalarRegulators.class, "pidf",
                PidController.class, double.class);
        assertDeclaredMethodAbsent(ScalarRegulators.class, "pidf",
                PidController.class, DoubleUnaryOperator.class);
    }

    @Test
    public void customControllerAndAdvancedFeedforwardRemainExplicitDistinctCapabilities()
            throws Exception {
        Method pid = ScalarRegulators.class.getDeclaredMethod(
                "pid", PidController.class);
        assertTrue(Modifier.isPublic(pid.getModifiers()));
        assertTrue(Modifier.isStatic(pid.getModifiers()));
        assertEquals(ScalarRegulator.class, pid.getReturnType());

        Method feedforward = ScalarRegulators.class.getDeclaredMethod(
                "setpointFeedforward",
                ScalarRegulator.class,
                DoubleUnaryOperator.class);
        assertTrue(Modifier.isPublic(feedforward.getModifiers()));
        assertTrue(Modifier.isStatic(feedforward.getModifiers()));
        assertEquals(ScalarRegulator.class, feedforward.getReturnType());

        for (Method method : ScalarControllers.class.getDeclaredMethods()) {
            assertFalse("ScalarControllers must not add a duplicate pidf construction layer",
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
            // Expected: API-03 deliberately removes this public construction/getter path.
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
