package edu.ftcphoenix.fw.core.lifecycle;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public final class CleanupActionsTest {

    @Test
    public void publicSurfaceIsStaticOnlyAndContainsOnlyApprovedMethods() throws Exception {
        assertTrue(Modifier.isPublic(CleanupActions.class.getModifiers()));
        assertTrue(Modifier.isFinal(CleanupActions.class.getModifiers()));
        assertEquals(0, CleanupActions.class.getConstructors().length);

        Constructor<?>[] declaredConstructors = CleanupActions.class.getDeclaredConstructors();
        assertEquals(1, declaredConstructors.length);
        assertTrue(Modifier.isPrivate(declaredConstructors[0].getModifiers()));

        Set<String> publicMethods = new HashSet<>();
        for (Method method : CleanupActions.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                assertTrue(Modifier.isStatic(method.getModifiers()));
                assertTrue(method.isVarArgs());
                publicMethods.add(publicSignature(method));
            }
        }
        assertEquals(new HashSet<>(Arrays.asList(
                "attemptAll(Runnable[]):void",
                "attemptAllAfterFailure(RuntimeException,Runnable[]):RuntimeException")),
                publicMethods);
    }

    @Test
    public void attemptAllRunsActionsInCallerOrder() {
        List<Integer> order = new ArrayList<>();

        CleanupActions.attemptAll(
                () -> order.add(1),
                () -> order.add(2),
                () -> order.add(3));

        assertEquals(Arrays.asList(1, 2, 3), order);
    }

    @Test
    public void attemptAllAttemptsRemainingActionsAndRetainsFailureOrder() {
        List<Integer> order = new ArrayList<>();
        RuntimeException first = new IllegalStateException("first");
        RuntimeException second = new IllegalArgumentException("second");
        RuntimeException third = new UnsupportedOperationException("third");

        RuntimeException thrown = expectRuntimeException(() -> CleanupActions.attemptAll(
                () -> {
                    order.add(1);
                    throw first;
                },
                () -> order.add(2),
                () -> {
                    order.add(3);
                    throw second;
                },
                () -> {
                    order.add(4);
                    throw third;
                }));

        assertSame(first, thrown);
        assertEquals(Arrays.asList(1, 2, 3, 4), order);
        assertArrayEquals(new Throwable[]{second, third}, thrown.getSuppressed());
    }

    @Test
    public void attemptAllAfterFailureReturnsAndEnrichesSamePrimary() {
        RuntimeException primary = new IllegalStateException("operation failed");
        RuntimeException firstCleanup = new IllegalArgumentException("first cleanup");
        RuntimeException secondCleanup = new UnsupportedOperationException("second cleanup");

        RuntimeException returned = CleanupActions.attemptAllAfterFailure(
                primary,
                () -> {
                    throw firstCleanup;
                },
                () -> {
                    throw secondCleanup;
                });

        assertSame(primary, returned);
        assertArrayEquals(
                new Throwable[]{firstCleanup, secondCleanup},
                primary.getSuppressed());
    }

    @Test
    public void alreadyOwnedPrimaryIsNotSuppressedOntoItself() {
        List<Integer> order = new ArrayList<>();
        RuntimeException primary = new IllegalStateException("primary");
        RuntimeException later = new IllegalArgumentException("later");

        RuntimeException returned = CleanupActions.attemptAllAfterFailure(
                primary,
                () -> {
                    order.add(1);
                    throw primary;
                },
                () -> {
                    order.add(2);
                    throw later;
                },
                () -> order.add(3));

        assertSame(primary, returned);
        assertEquals(Arrays.asList(1, 2, 3), order);
        assertArrayEquals(new Throwable[]{later}, primary.getSuppressed());
    }

    @Test
    public void repeatedLaterFailureObjectIsNotGloballyDeduplicated() {
        RuntimeException primary = new IllegalStateException("primary");
        RuntimeException repeated = new IllegalArgumentException("repeated");

        CleanupActions.attemptAllAfterFailure(
                primary,
                () -> {
                    throw repeated;
                },
                () -> {
                    throw repeated;
                });

        assertArrayEquals(new Throwable[]{repeated, repeated}, primary.getSuppressed());
    }

    @Test
    public void nullPrimaryIsRejectedBeforeCleanup() {
        List<Integer> order = new ArrayList<>();

        RuntimeException thrown = expectRuntimeException(
                () -> CleanupActions.attemptAllAfterFailure(
                        null,
                        () -> order.add(1)));

        assertTrue(thrown instanceof NullPointerException);
        assertEquals("primaryFailure must not be null", thrown.getMessage());
        assertTrue(order.isEmpty());
    }

    @Test
    public void nullActionArrayIsRejectedWithoutPrimary() {
        RuntimeException thrown = expectRuntimeException(
                () -> CleanupActions.attemptAll((Runnable[]) null));

        assertTrue(thrown instanceof NullPointerException);
        assertEquals("cleanupActions must not be null", thrown.getMessage());
    }

    @Test
    public void nullActionArrayIsAttachedToExistingPrimary() {
        RuntimeException primary = new IllegalStateException("primary");

        RuntimeException returned = CleanupActions.attemptAllAfterFailure(
                primary,
                (Runnable[]) null);

        assertSame(primary, returned);
        assertEquals(1, primary.getSuppressed().length);
        assertTrue(primary.getSuppressed()[0] instanceof NullPointerException);
        assertEquals(
                "cleanupActions must not be null",
                primary.getSuppressed()[0].getMessage());
    }

    @Test
    public void individualNullActionIsIndexedAndDoesNotStopLaterActions() {
        List<Integer> order = new ArrayList<>();

        RuntimeException thrown = expectRuntimeException(() -> CleanupActions.attemptAll(
                () -> order.add(1),
                null,
                () -> order.add(3)));

        assertTrue(thrown instanceof NullPointerException);
        assertEquals("cleanupActions[1] must not be null", thrown.getMessage());
        assertEquals(Arrays.asList(1, 3), order);
    }

    @Test
    public void individualNullActionIsSuppressedInItsActionPosition() {
        RuntimeException primary = new IllegalStateException("primary");
        RuntimeException later = new IllegalArgumentException("later");

        CleanupActions.attemptAllAfterFailure(
                primary,
                null,
                () -> {
                    throw later;
                });

        Throwable[] suppressed = primary.getSuppressed();
        assertEquals(2, suppressed.length);
        assertTrue(suppressed[0] instanceof NullPointerException);
        assertEquals("cleanupActions[0] must not be null", suppressed[0].getMessage());
        assertSame(later, suppressed[1]);
    }

    @Test
    public void noActionsSucceedAndPreserveExistingPrimary() {
        CleanupActions.attemptAll();

        RuntimeException primary = new IllegalStateException("primary");
        assertSame(primary, CleanupActions.attemptAllAfterFailure(primary));
        assertEquals(0, primary.getSuppressed().length);
    }

    @Test
    public void errorIsNotCaughtAndStopsRemainingActions() {
        List<Integer> order = new ArrayList<>();
        AssertionError error = new AssertionError("fatal");

        try {
            CleanupActions.attemptAll(
                    () -> order.add(1),
                    () -> {
                        order.add(2);
                        throw error;
                    },
                    () -> order.add(3));
            fail("Expected Error to propagate");
        } catch (AssertionError thrown) {
            assertSame(error, thrown);
        }

        assertEquals(Arrays.asList(1, 2), order);
    }

    @Test
    public void errorAfterRuntimeFailureStillPropagatesImmediatelyAndUnchanged() {
        List<Integer> order = new ArrayList<>();
        RuntimeException earlierFailure = new IllegalStateException("earlier cleanup");
        AssertionError error = new AssertionError("fatal");

        try {
            CleanupActions.attemptAll(
                    () -> {
                        order.add(1);
                        throw earlierFailure;
                    },
                    () -> {
                        order.add(2);
                        throw error;
                    },
                    () -> order.add(3));
            fail("Expected Error to propagate");
        } catch (AssertionError thrown) {
            assertSame(error, thrown);
        }

        assertEquals(Arrays.asList(1, 2), order);
        assertEquals(0, error.getSuppressed().length);
    }

    private static RuntimeException expectRuntimeException(Runnable operation) {
        try {
            operation.run();
            fail("Expected RuntimeException");
            throw new AssertionError("unreachable");
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static String publicSignature(Method method) {
        StringBuilder signature = new StringBuilder(method.getName()).append('(');
        Class<?>[] parameterTypes = method.getParameterTypes();
        for (int index = 0; index < parameterTypes.length; index++) {
            if (index > 0) {
                signature.append(',');
            }
            signature.append(parameterTypes[index].getSimpleName());
        }
        return signature.append("):")
                .append(method.getReturnType().getSimpleName())
                .toString();
    }
}
